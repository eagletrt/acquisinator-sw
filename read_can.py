#!/usr/bin/env python3
"""Ascolta il CAN dell'acquisinator e stampa i frame decodificati.

Uso:
  python read_can.py                   frame di misura e di stato (DIAG, BRIDGE, EXC), streaming dei
                                       campioni grezzi dell'ADC (ACQUISINATOR_ADC_STREAM) con un
                                       riassunto al secondo, e registrazione in CSV
  python read_can.py --plot [--ms 50]  in piu' un oscilloscopio live
  python read_can.py --csv prova.csv   nome del CSV (default adc_AAAAMMGG_HHMMSS.csv)
  python read_can.py --no-stream       solo i frame di misura e di stato, niente streaming ne' CSV

CSV, una riga per campione: n e t_s (indice e tempo del campione, i campioni persi contano),
seq e phase del frame, il valore dell'ADC (codice e V) e il valore del DAC che lo ha prodotto
(codice e V), ricostruito dalla tabella del firmware con l'eccitazione di ACQUISINATOR_EXC, e
adc1 = CH1 dell'ADC dello stesso tick (solo col firmware compilato con -DACQ_STREAM_CH1). I
campioni arrivati prima del primo EXC (entro 1 s) vengono scritti appena arriva. Con un generatore
esterno al posto del DAC sono solo il riferimento interno, non l'eccitazione vera.
A 8 kHz sono ~400 kB/s.

Lo streaming resta acceso finche' lo script rinnova il comando (ACQUISINATOR_SET_STREAM ogni
secondo): se lo script si chiude o si pianta il firmware lo spegne da solo dopo 3 s, cosi' sul bus
della macchina non resta un terzo di traffico. I frame dello streaming (migliaia al secondo) e gli
error frame non vengono mai stampati uno per uno, solo contati: il terminale non si intasa.

Nel riassunto: "frame persi" sono buchi nel contatore seq (persi sul bus o dal transceiver/PC),
"salti" sono discontinuita' dell'indice di fase, cioe' campioni mancanti per qualunque motivo
(anche coda piena nel firmware). L'oscilloscopio parte da un inizio di periodo dell'eccitazione
(fase 0): col DAC la forma d'onda sta ferma, con un generatore esterno scorre lentamente.
"""

import argparse
import csv
import math
import struct
import threading
import time
from collections import deque

import can
import cantools

from gs_usb_bus import open_bus

BITRATE = 1_000_000
ADC_VREF_V = 3.29  # LTC1865L_VREF
ADC_FULL_SCALE = 65536
KEEPALIVE_S = 1.0  # rinnovo dello streaming, il firmware lo tiene acceso 3 s
SUMMARY_S = 1.0
SCOPE_MAX_MS = 200.0
SCOPE_MAX_TICK_HZ = 16000  # EXC_MAX_TICK_HZ del firmware
HISTORY_MAX_S = 2.0  # campioni con indice per le analisi (banco.py)
PLOT_FPS = 20
STREAM_PER_FRAME = 3
# Come nel firmware (main.c): tabella del DAC = exc_plan_api_make_dac_table
VDDA_V = 3.3
DAC_MAX = 4095
Q15_ONE = 32767
CSV_COLUMNS = ("n", "t_s", "seq", "phase", "adc_code", "adc_v", "dac_code", "dac_v", "exc_id", "adc1_code", "adc1_v")
FLAG_NAMES = ((0, "SATURA BASSO"), (1, "SATURA ALTO"), (2, "ERRORE SPI"), (3, "non calibrato"), (4, "NESSUNA MISURA"), (5, "TICK PERSO"))

db = cantools.database.load_file("acquisinator.dbc")
DIAG = db.get_message_by_name("ACQUISINATOR_DIAG")
BRIDGE = db.get_message_by_name("ACQUISINATOR_BRIDGE")
EXC = db.get_message_by_name("ACQUISINATOR_EXC")
STREAM = db.get_message_by_name("ACQUISINATOR_ADC_STREAM")
SET_STREAM = db.get_message_by_name("ACQUISINATOR_SET_STREAM")
STREAM_LAYOUT = struct.Struct("<HHHBB")  # come il DBC; piu' leggero di cantools a migliaia di frame/s
TAG_PHASE, TAG_CH1, TAG_DUAL = 0x3F, 0x40, 0x80  # byte phase: indice di fase, campione di CH1, due canali


def volts(code: float) -> float:
    return code * ADC_VREF_V / ADC_FULL_SCALE


def dac_table(samples: int, dc_mv: int, amp_mv: int) -> list[int]:
    """Codici del DAC per un periodo, con gli stessi arrotondamenti del firmware."""
    table = []
    for k in range(samples):
        sin_q15 = round(Q15_ONE * math.sin(2.0 * math.pi * k / samples))
        volts_k = dc_mv / 1000.0 + amp_mv / 1000.0 * sin_q15 / Q15_ONE
        table.append(min(max(round(volts_k / VDDA_V * DAC_MAX), 0), DAC_MAX))
    return table


class Stream:
    """Conteggi dello streaming tra due riassunti e coda dei campioni per l'oscilloscopio.

    I campioni sono raccolti per tick: CH0 e, con il firmware ACQ_STREAM_CH1, CH1 dello stesso tick,
    che possono arrivare in due frame diversi. Un tick e' completo quando arriva il campione del tick
    dopo; se ne manca uno dei due canali resta vuoto.
    """

    def __init__(self, writer=None) -> None:
        self.lock = threading.Lock()
        self.writer = writer  # csv.writer, oppure None
        self.csv_base = 0  # indice del primo tick del CSV
        self.dac: list[int] | None = None
        self.exc_id: int | None = None
        self.n = 0  # indice del prossimo tick, compresi quelli persi
        # (CH0, indice di fase, CH1 o None): abbastanza per la finestra piu' lunga piu' un periodo di trigger
        self.scope: deque[tuple[int, int, int | None]] = deque(maxlen=2 * int(SCOPE_MAX_MS / 1000.0 * SCOPE_MAX_TICK_HZ))
        # (indice, CH0, indice di fase, CH1 o None) dei tick con CH0, per spettro e lock-in (banco.py)
        self.history: deque[tuple[int, int, int, int | None]] = deque(maxlen=int(HISTORY_MAX_S * SCOPE_MAX_TICK_HZ))
        self.samples_per_period: int | None = None
        self.tick_hz: float | None = None
        self.last_seq: int | None = None
        self.expected: tuple[int, int] | None = None  # (fase, canale) del prossimo campione
        self.tick: list | None = None  # [indice, fase, seq, CH0, CH1] del tick in costruzione
        self.dual = False  # l'ultimo frame era a due canali
        # frame arrivati prima del primo ACQUISINATOR_EXC: senza periodo non si sa la fase dei
        # campioni dopo il primo ne' il DAC, si scrivono appena arriva (entro 1 s)
        self.pending: list[tuple[int, int, list[int]]] = []
        self.clear_counts()

    def clear_counts(self) -> None:
        self.samples = 0
        self.samples_ch1 = 0
        self.lost_frames = 0
        self.jumps = 0
        self.total = 0
        self.low: int | None = None
        self.high: int | None = None

    def set_writer(self, writer) -> None:
        """Nuovo CSV (o None): n e t_s ripartono dal tick in corso."""
        with self.lock:
            self.writer = writer
            self.csv_base = self.n if self.tick is None else self.tick[0]

    def set_excitation(self, exc: dict) -> None:
        samples = int(exc["samples"])
        with self.lock:
            if samples != self.samples_per_period:
                self.finish_tick()
                self.expected = None
            self.samples_per_period = samples
            self.tick_hz = exc["freq"] * samples
            self.dac = dac_table(samples, int(exc["dc"]), int(exc["amp"]))
            self.exc_id = int(exc["id"])
            for seq, tag, values in self.pending:
                self.place(seq, tag, values, to_scope=False)
            self.pending = []

    def add(self, data: bytes) -> None:
        *values, tag, seq = STREAM_LAYOUT.unpack(data)
        with self.lock:
            if self.last_seq is not None:
                self.lost_frames += (seq - self.last_seq - 1) & 0xFF
            self.last_seq = seq
            if self.samples_per_period:
                self.place(seq, tag, values)
            else:
                self.pending.append((seq, tag, values))

    def place(self, seq: int, tag: int, values: list[int], to_scope: bool = True) -> None:
        """Campioni di un frame, con il periodo noto, nei tick a cui appartengono."""
        period = self.samples_per_period
        phase, channel = tag & TAG_PHASE, 1 if tag & TAG_CH1 else 0
        self.dual = bool(tag & TAG_DUAL)
        for value in values:
            phase %= period
            if self.expected is not None and (phase, channel) != self.expected:
                self.jumps += 1
            tick = self.tick
            if tick is None or channel == 0 or phase != tick[1]:
                if tick is None:
                    index = self.n
                else:
                    # meno di un periodo perso: non si vede di piu'
                    index = tick[0] + ((phase - tick[1]) % period or period)
                    self.flush(tick, to_scope)
                self.tick = tick = [index, phase, seq, None, None]
            tick[3 + channel] = value
            if self.dual and channel == 0:
                channel = 1
            else:
                phase, channel = phase + 1, 0
            self.expected = (phase % period, channel)

    def finish_tick(self) -> None:
        if self.tick is not None:
            self.flush(self.tick, True)
            self.n = self.tick[0] + 1
            self.tick = None

    def flush(self, tick: list, to_scope: bool) -> None:
        index, phase, seq, ch0, ch1 = tick
        self.n = index + 1
        if ch0 is not None:
            if to_scope:
                self.scope.append((ch0, phase, ch1))
                self.history.append((index, ch0, phase, ch1))
            self.samples += 1
            self.total += ch0
            self.low = ch0 if self.low is None else min(self.low, ch0)
            self.high = ch0 if self.high is None else max(self.high, ch0)
        if ch1 is not None:
            self.samples_ch1 += 1
        if self.writer is not None:
            self.write_row(index - self.csv_base, seq, phase, ch0, ch1)

    def write_row(self, n: int, seq: int, phase: int, ch0: int | None, ch1: int | None) -> None:
        dac = self.dac[phase] if self.dac is not None and phase < len(self.dac) else None
        self.writer.writerow((
            n, f"{n / self.tick_hz:.6f}", seq, phase,
            "" if ch0 is None else ch0, "" if ch0 is None else f"{volts(ch0):.5f}",
            "" if dac is None else dac, "" if dac is None else f"{dac * VDDA_V / DAC_MAX:.4f}", self.exc_id,
            "" if ch1 is None else ch1, "" if ch1 is None else f"{volts(ch1):.5f}",
        ))

    def summary(self, seconds: float) -> str:
        with self.lock:
            if not self.samples:
                text = "stream: nessun campione (firmware senza streaming? scheda accesa?)"
            else:
                expected = f" (attesi {self.tick_hz:.0f})" if self.tick_hz else ""
                ch1 = f"  CH1 {self.samples_ch1 / seconds:.0f}/s" if self.dual else ""
                mean = self.total / self.samples
                text = (
                    f"stream: {self.samples / seconds:6.0f} campioni/s{expected}{ch1}  frame persi {self.lost_frames}"
                    f"  salti {self.jumps}  min {self.low:5d} ({volts(self.low):.4f} V)  max {self.high:5d}"
                    f" ({volts(self.high):.4f} V)  media {mean:7.1f} ({volts(mean):.4f} V)"
                )
            self.clear_counts()
        return text


def send_stream(bus: can.BusABC, enable: bool) -> None:
    message = can.Message(arbitration_id=SET_STREAM.frame_id, data=SET_STREAM.encode({"enable": int(enable)}), is_extended_id=False)
    try:
        bus.send(message, timeout=0.1)
    except can.CanError as error:
        print(f"comando streaming non inviato: {error}")


def print_frame(message: can.Message, stream: Stream) -> None:
    frame_id = message.arbitration_id
    if frame_id == DIAG.frame_id:
        d = DIAG.decode(message.data)
        spi = "OK" if int(d["adc_poll"]) == 0 else "ERRORE"
        print(f"DIAG    v_out = {d['lockin_out'] * 1000:+9.3f} mV   adc_raw = {d['adc_raw']:5.0f} ({volts(d['adc_raw']):.4f} V)"
              f"   spi = {spi}   exc #{int(d['exc_id'])}")
    elif frame_id == BRIDGE.frame_id:
        d = BRIDGE.decode(message.data)
        flags = int(d["flags"])
        notes = ", ".join(name for bit, name in FLAG_NAMES if flags & (1 << bit))
        print(f"BRIDGE  uscita = {d['bridge_out']:+9.5f} mV/V   fase = {d['phase']:+7.2f} deg   #{int(d['seq']):3d}   {notes}")
    elif frame_id == EXC.frame_id:
        d = EXC.decode(message.data)
        stream.set_excitation(d)
        print(f"EXC     #{int(d['id'])}: {d['freq']:.1f} Hz, {int(d['samples'])} campioni/periodo, "
              f"finestra {int(d['periods'])} periodi, {int(d['amp'])} mV su {int(d['dc'])} mV")
    else:
        print(f"RAW     id=0x{frame_id:X} dlc={message.dlc} data={message.data.hex()}")


def listen(bus: can.BusABC, stream: Stream, streaming: bool, stop: threading.Event, csv_file=None) -> None:
    last_keepalive = 0.0
    last_summary = last_frame = time.monotonic()
    errors = 0
    while not stop.is_set():
        now = time.monotonic()
        if streaming and now - last_keepalive >= KEEPALIVE_S:
            send_stream(bus, True)
            last_keepalive = now
        if now - last_summary >= SUMMARY_S:
            if streaming:
                print(stream.summary(now - last_summary))
            if csv_file is not None:
                csv_file.flush()
            if errors:
                print(f"error frame: {errors} nell'ultimo secondo (bus senza ACK, cablaggio o bitrate)")
            if now - last_frame >= SUMMARY_S:
                print(f"[nessun frame da {now - last_frame:.0f} s]")
            errors = 0
            last_summary = now

        message = bus.recv(timeout=0.1)
        if message is None:
            continue
        if message.is_error_frame:
            errors += 1
            continue
        last_frame = now
        if message.arbitration_id == STREAM.frame_id:
            stream.add(message.data)
        else:
            print_frame(message, stream)


def plot_scope(stream: Stream, initial_ms: float) -> None:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.widgets import Slider

    fig, ax = plt.subplots(figsize=(9.0, 5.5))
    fig.subplots_adjust(bottom=0.20)
    (line,) = ax.plot([], [], linewidth=1.0)
    ax.set_xlabel("tempo dall'inizio del periodo [ms]")
    ax.set_ylabel("ADC [code]")
    ax.grid(True)
    ax.secondary_yaxis("right", functions=(volts, lambda v: v * ADC_FULL_SCALE / ADC_VREF_V)).set_ylabel("[V]")
    view = { "ms": initial_ms }

    def update(_) -> None:
        with stream.lock:
            data = list(stream.scope)
            tick_hz = stream.tick_hz
        if not data or not tick_hz:
            ax.set_title("in attesa di campioni e di ACQUISINATOR_EXC...")
            return
        count = min(len(data), max(2, int(view["ms"] / 1000.0 * tick_hz)))
        # trigger: l'ultimo inizio di periodo che ha `count` campioni dopo di se'
        start = len(data) - count
        for k in range(start, -1, -1):
            if data[k][1] == 0:
                start = k
                break
        shown = [value for value, *_ in data[start : start + count]]
        line.set_data([1000.0 * k / tick_hz for k in range(len(shown))], shown)
        ax.set_xlim(0.0, view["ms"])
        low, high = min(shown), max(shown)
        pad = max((high - low) * 0.05, 1.0)
        ax.set_ylim(low - pad, high + pad)
        ax.set_title(f"min {low}  max {high}  p-p {high - low} code = {volts(high - low) * 1000.0:.1f} mV")

    slider = Slider(fig.add_axes((0.20, 0.05, 0.60, 0.03)), "finestra [ms]", 2.0, SCOPE_MAX_MS, valinit=initial_ms)
    slider.on_changed(lambda value: view.update(ms=value))
    # tenere i riferimenti: senza, slider e animazione vengono raccolti dal GC
    anim = FuncAnimation(fig, update, interval=1000 // PLOT_FPS, cache_frame_data=False)  # noqa: F841
    plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--no-stream", action="store_true", help="niente streaming dell'ADC ne' CSV")
    parser.add_argument("--plot", action="store_true", help="oscilloscopio live dello streaming")
    parser.add_argument("--csv", default=time.strftime("adc_%Y%m%d_%H%M%S.csv"), help="file dei campioni")
    parser.add_argument("--ms", type=float, default=50.0, help=f"finestra iniziale dell'oscilloscopio [ms], max {SCOPE_MAX_MS:.0f}")
    args = parser.parse_args()
    streaming = not args.no_stream
    if args.plot and not streaming:
        parser.error("--plot richiede lo streaming")

    bus = open_bus(bitrate=BITRATE)
    csv_file = open(args.csv, "w", newline="") if streaming else None
    writer = None
    if csv_file is not None:
        writer = csv.writer(csv_file)
        writer.writerow(CSV_COLUMNS)
    stream = Stream(writer)
    stop = threading.Event()
    print("In ascolto" + (f" con lo streaming dell'ADC, campioni in {args.csv}" if streaming else "") + "...")
    try:
        if args.plot:
            reader = threading.Thread(target=listen, args=(bus, stream, streaming, stop, csv_file), daemon=True)
            reader.start()
            plot_scope(stream, min(args.ms, SCOPE_MAX_MS))  # ritorna alla chiusura della finestra
            stop.set()
            reader.join(timeout=1.0)
        else:
            listen(bus, stream, streaming, stop, csv_file)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        if streaming:
            send_stream(bus, False)
        bus.shutdown()
        if csv_file is not None:
            with stream.lock:  # il thread di lettura potrebbe essere ancora a meta' riga
                stream.writer = None
                csv_file.close()
            print(f"Campioni salvati in {args.csv}")


if __name__ == "__main__":
    main()
