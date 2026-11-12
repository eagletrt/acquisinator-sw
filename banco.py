#!/usr/bin/env python3
"""Banco di prova dell'acquisinator: grafici live, cambio dell'eccitazione e registrazione.

Uso:
  python banco.py [--dir registrazioni]

Finestra:
  oscilloscopio  campioni grezzi dell'ADC (streaming), con trigger sull'inizio del periodo
  spettro        dell'ultimo secondo, con le armoniche dell'eccitazione segnate
  andamento      ampiezza a f, 2f e DC dell'ultimo secondo, nel tempo: si vede subito cosa fa
                 il rumore quando lo accendi e lo spegni
  comandi        frequenza [Hz], DC [mV], ampiezza [mV] e "Applica" (0 = invariato, come
                 set_exc.py; per un'ampiezza quasi nulla usa 1); "Registra" / "Stop"

Col firmware compilato con -DACQ_STREAM_CH1 arriva anche CH1 (l'eccitazione vera, es. dal
generatore esterno): arancione nell'oscilloscopio e nello spettro, e una seconda riga di numeri con
la sua ampiezza, la frequenza vera, il rapporto CH0/CH1 e lo sfasamento CH0-CH1. Lo sfasamento
comprende il ritardo fisso tra le due conversioni (CH1 e' convertito subito dopo CH0): per
calibrarlo, lo stesso segnale su entrambi gli ingressi, oppure il ponte sbilanciato (shunt) per
calibrare tutta la catena: "calibra" salva come ritardo della frequenza in uso la fase della
variazione del rapporto (due passi: a riposo, poi deformato o con lo shunt; conta la fase della
differenza), senza i 180° del segno (in --cal, ricaricato quando torni a quella frequenza).
USCITA = parte in fase del rapporto, corretta del ritardo, in mV/V con segno:
  1000 * |CH0/CH1| * cos(sfasamento - ritardo) * partitore / guadagno
con partitore = V su CH1 / V di eccitazione (1 senza partitore) e guadagno dell'INA (100).

Le analisi sono fatte qui dai campioni dello streaming, su un numero intero di periodi, con i
campioni persi ricostruiti per interpolazione: ampiezze in code dell'ADC, fase rispetto al DAC,
rumore = scarto dalla forma d'onda media del periodo. I numeri sono gli stessi delle tabelle
fatte a mano sui CSV.

Registrazione: nella cartella --dir, con nome dalla nota e dall'eccitazione in uso, due file:
  <nome>.csv         un campione per riga, come read_can.py (n riparte da 0)
  <nome>_lockin.csv  un riga per ogni ACQUISINATOR_BRIDGE della scheda, con l'ultimo DIAG, piu'
                     l'analisi dello streaming del momento
Se cambi eccitazione mentre registri la colonna exc_id lo dice.

Ricorda: al riavvio la scheda torna all'eccitazione di default, l'id in alto lo mostra.
"""

import argparse
import csv
import json
import math
import os
import queue
import threading
import time
from collections import deque

import can
import numpy as np

from gs_usb_bus import open_bus
from read_can import (
    BITRATE, BRIDGE, CSV_COLUMNS, DIAG, EXC, FLAG_NAMES, KEEPALIVE_S, STREAM, Stream, send_stream, volts,
)

import cantools

SET_EXC = cantools.database.load_file("acquisinator.dbc").get_message_by_name("ACQUISINATOR_SET_EXC")

ANALYSIS_S = 1.0
TREND_S = 300.0
TREND_EVERY_S = 0.25
SCOPE_MS = 40.0
FPS = 10
TEXT_EVERY_S = 0.5  # i numeri: ridisegnare il testo e' la parte che costa
CONFIRM_TIMEOUT_S = 3.0
HARMONICS = 4
LOCKIN_COLUMNS = ("t_s", "exc_id", "bridge_out_mv_per_v", "phase_deg", "flags", "lockin_out_v", "adc_raw",
                  "a1_code", "phase1_deg", "a2_code", "a3_code", "dc_code", "rms_code",
                  "ch1_a1_code", "ch1_dc_code", "ch1_freq_hz", "ratio_ch0_ch1", "dphase_ch0_ch1_deg",
                  "delay_deg", "out_mv_per_v")


class Calibration:
    """Ritardo CH0-CH1 per frequenza, guadagno dell'INA e partitore di CH1, salvati in un file JSON."""

    def __init__(self, path: str) -> None:
        self.path = path
        self.gain = 100.0  # AMP_GAIN del firmware
        self.divider = 1.0  # V su CH1 / V di eccitazione
        self.delays: dict[str, float] = {}  # "125.0" -> gradi
        try:
            with open(path) as f:
                data = json.load(f)
            self.gain = float(data.get("gain", self.gain))
            self.divider = float(data.get("divider", self.divider))
            self.delays = {k: float(v) for k, v in data.get("delays", {}).items()}
        except (OSError, ValueError):
            pass

    def save(self) -> None:
        with open(self.path, "w") as f:
            json.dump({"gain": self.gain, "divider": self.divider, "delays": self.delays}, f, indent=2)

    @staticmethod
    def key(freq: float) -> str:
        return f"{round(freq * 2.0) / 2.0:.1f}"  # a 0.5 Hz: la frequenza misurata oscilla di poco

    def delay(self, freq: float) -> float:
        return self.delays.get(self.key(freq), 0.0)

    @staticmethod
    def fold(dphase: float) -> float:
        """Ritardo da uno sfasamento misurato, senza i 180 gradi del segno: in (-90, 90]."""
        return -((-dphase + 90.0) % 180.0 - 90.0)

    def output(self, ch1: dict, freq: float) -> float:
        """Uscita del ponte in mV/V con segno: parte in fase del rapporto CH0/CH1, corretta."""
        in_phase = ch1["ratio"] * math.cos(math.radians(ch1["dphase"] - self.delay(freq)))
        return 1000.0 * in_phase * self.divider / self.gain


class Bench:
    """Stato condiviso tra il thread del CAN e la finestra."""

    def __init__(self, bus: can.BusABC, out_dir: str, cal: Calibration) -> None:
        self.bus = bus
        self.out_dir = out_dir
        self.cal = cal  # usata solo dal thread della finestra e sotto self.lock
        self.stream = Stream()
        self.commands: queue.Queue[can.Message] = queue.Queue()
        self.stop = threading.Event()
        self.lock = threading.Lock()  # tutto quello che segue, lo stream ha il suo
        self.exc: dict | None = None
        self.pending: tuple[float, int | None] | None = None  # (istante, id prima del comando)
        self.message = "in attesa della scheda..."
        self.diag: dict | None = None
        self.bridge: dict | None = None
        self.bridge_sum = 0.0  # bridge_out della scheda dall'ultimo punto dell'andamento
        self.bridge_count = 0
        self.errors = 0
        self.last_frame = 0.0
        self.analysis: dict | None = None
        self.rec_name: str | None = None
        self.rec_start = 0.0
        self.rec_files: list = []
        self.lockin_writer = None

    # --- thread del CAN ---------------------------------------------------------------------

    def run(self) -> None:
        last_keepalive = 0.0
        while not self.stop.is_set():
            now = time.monotonic()
            if now - last_keepalive >= KEEPALIVE_S:
                send_stream(self.bus, True)
                last_keepalive = now
            while not self.commands.empty():
                try:
                    self.bus.send(self.commands.get_nowait(), timeout=0.1)
                except can.CanError as error:
                    with self.lock:
                        self.message = f"comando non inviato: {error}"
            with self.lock:
                if self.pending is not None and now - self.pending[0] > CONFIRM_TIMEOUT_S:
                    self.message = "RIFIUTATA (frequenza non realizzabile o livelli fuori dal DAC)"
                    self.pending = None

            message = self.bus.recv(timeout=0.05)
            if message is None:
                continue
            if message.is_error_frame:
                with self.lock:
                    self.errors += 1
                continue
            self.last_frame = now
            frame_id = message.arbitration_id
            if frame_id == STREAM.frame_id:
                self.stream.add(message.data)
            elif frame_id == EXC.frame_id:
                self.on_exc(EXC.decode(message.data))
            elif frame_id == DIAG.frame_id:
                with self.lock:
                    self.diag = DIAG.decode(message.data)
            elif frame_id == BRIDGE.frame_id:
                self.on_bridge(BRIDGE.decode(message.data))

    def on_exc(self, exc: dict) -> None:
        self.stream.set_excitation(exc)
        with self.lock:
            if self.pending is not None and exc["id"] != self.pending[1]:
                self.message = f"applicata, id {int(exc['id'])}"
                self.pending = None
            self.exc = exc

    def on_bridge(self, bridge: dict) -> None:
        with self.lock:
            self.bridge = bridge
            if math.isfinite(bridge["bridge_out"]):
                self.bridge_sum += bridge["bridge_out"]
                self.bridge_count += 1
            if self.lockin_writer is None:
                return
            diag = self.diag or {}
            a = self.analysis or {}
            harm = a.get("harm", [math.nan] * (HARMONICS + 1))
            self.lockin_writer.writerow((
                f"{time.monotonic() - self.rec_start:.3f}", int(diag.get("exc_id", -1)),
                f"{bridge['bridge_out']:.6f}", f"{bridge['phase']:.2f}", int(bridge["flags"]),
                f"{diag.get('lockin_out', math.nan):.6f}", diag.get("adc_raw", ""),
                f"{harm[1]:.2f}", f"{a.get('phase1', math.nan):.1f}", f"{harm[2]:.2f}", f"{harm[3]:.2f}",
                f"{harm[0]:.1f}", f"{a.get('rms', math.nan):.2f}",
                *((f"{ch1['a1']:.2f}", f"{ch1['dc']:.1f}", f"{ch1['freq']:.4f}", f"{ch1['ratio']:.7f}",
                   f"{ch1['dphase']:.3f}", f"{self.cal.delay(ch1['freq']):.3f}", f"{self.cal.output(ch1, ch1['freq']):.6f}")
                  if (ch1 := a.get("ch1")) else ("",) * 7),
            ))

    # --- comandi dalla finestra ---------------------------------------------------------------

    def apply(self, freq: int, dc: int, amp: int) -> None:
        data = SET_EXC.encode({"freq": freq, "amp": amp, "dc": dc})
        self.commands.put(can.Message(arbitration_id=SET_EXC.frame_id, data=data, is_extended_id=False))
        with self.lock:
            self.pending = (time.monotonic(), None if self.exc is None else self.exc["id"])
            self.message = "inviata, attendo conferma..."

    def start_recording(self, note: str) -> str:
        with self.lock:
            exc = self.exc
        parts = ["banco"]
        if note:
            parts.append("".join(c if c.isalnum() or c in "-." else "_" for c in note))
        if exc is not None:
            parts.append(f"f{exc['freq']:g}_dc{int(exc['dc'])}_a{int(exc['amp'])}")
        parts.append(time.strftime("%Y%m%d_%H%M%S"))
        name = os.path.join(self.out_dir, "_".join(parts))
        os.makedirs(self.out_dir, exist_ok=True)
        samples = open(name + ".csv", "w", newline="")
        lockin = open(name + "_lockin.csv", "w", newline="")
        writer = csv.writer(samples)
        writer.writerow(CSV_COLUMNS)
        lockin_writer = csv.writer(lockin)
        lockin_writer.writerow(LOCKIN_COLUMNS)
        self.stream.set_writer(writer)
        with self.lock:
            self.rec_name, self.rec_start = name, time.monotonic()
            self.rec_files = [samples, lockin]
            self.lockin_writer = lockin_writer
        return name

    def stop_recording(self) -> str | None:
        self.stream.set_writer(None)
        with self.lock:
            name, files = self.rec_name, self.rec_files
            self.rec_name, self.rec_files, self.lockin_writer = None, [], None
        for f in files:
            f.close()
        return name

    def flush(self) -> None:
        with self.stream.lock, self.lock:
            for f in self.rec_files:
                f.flush()


def analyse(stream: Stream) -> dict | None:
    """Ultimo secondo dello streaming: armoniche dell'eccitazione, fase, rumore e spettro."""
    with stream.lock:
        period = stream.samples_per_period
        tick_hz = stream.tick_hz
        if not period or not tick_hz:
            return None
        count = int(ANALYSIS_S * tick_hz)
        history = list(stream.history)[-count:]
    if len(history) < 4 * period:
        return None
    n = np.array([h[0] for h in history])
    if len(n) < 4 * period or n[-1] - n[0] > 2 * len(n):  # pochi campioni o troppi persi
        return None
    v = np.array([h[1] for h in history], dtype=float)
    x = np.interp(np.arange(n[0], n[-1] + 1), n, v)
    periods = len(x) // period
    skipped = len(x) - periods * period
    x = x[skipped:]
    first_phase = (history[0][2] + skipped) % period
    spectrum = np.fft.rfft(x) * 2.0 / len(x)
    harm = [x.mean()] + [abs(spectrum[h * periods]) if h * periods < len(spectrum) else math.nan
                         for h in range(1, HARMONICS + 1)]
    # x ~ A sin(2 pi (fase + j) / periodo + phi): stesso riferimento della tabella del DAC
    theta = np.angle(spectrum[periods])
    phase1 = math.degrees(theta + math.pi / 2.0 - 2.0 * math.pi * first_phase / period) % 360.0
    shaped = x.reshape(periods, period)
    rms = float((shaped - shaped.mean(axis=0)).std())
    third = len(x) / 3.0  # 8000/3 Hz con 3 campioni per frame: artefatto dello streaming
    lo, hi = int(third) - 2, int(third) + 3
    tone3 = float(np.abs(spectrum[lo:hi]).max()) if hi < len(spectrum) else math.nan
    freqs = np.fft.rfftfreq(len(x), 1.0 / tick_hz)
    result = {"harm": harm, "phase1": phase1, "rms": rms, "tone3": tone3, "freqs": freqs,
              "amps": np.abs(spectrum), "f0": tick_hz / period, "tick_hz": tick_hz,
              "missing": 1.0 - len(n) / (n[-1] - n[0] + 1), "ch1": None}

    # CH1 (firmware con ACQ_STREAM_CH1): eccitazione vera, stessa finestra e stessi tick di CH0
    with1 = [h for h in history if h[3] is not None]
    if len(with1) >= len(history) // 2:
        n1 = np.array([h[0] for h in with1])
        x1 = np.interp(np.arange(n[0], n[-1] + 1), n1, np.array([h[3] for h in with1], dtype=float))[skipped:]
        spectrum1 = np.fft.rfft(x1) * 2.0 / len(x1)
        # Il generatore non e' agganciato al tick e puo' essere a una frequenza diversa da quella
        # della scheda: la sua frequenza vera e' il picco dello spettro di CH1 (finestra di Hann,
        # interpolazione parabolica) e CH0 e CH1 si demodulano li', con la stessa finestra di Hann.
        window = np.hanning(len(x1))
        peaks = np.abs(np.fft.rfft((x1 - x1.mean()) * window))
        k = int(np.argmax(peaks[1:-1])) + 1
        a, b, c = np.log(peaks[k - 1:k + 2] + 1e-12)
        freq = (k + 0.5 * (a - c) / (a - 2.0 * b + c)) * tick_hz / len(x1)
        local = np.exp(-2j * np.pi * freq * np.arange(len(x1)) / tick_hz) * window
        y0 = 2.0 * np.dot(x - x.mean(), local) / window.sum()
        y1 = 2.0 * np.dot(x1 - x1.mean(), local) / window.sum()
        ratio = y0 / y1
        result["ch1"] = {"a1": abs(y1), "dc": x1.mean(), "amps": np.abs(spectrum1), "a0": abs(y0),
                         "ratio": abs(ratio), "dphase": math.degrees(np.angle(ratio)), "freq": freq}
    return result


def fit_limits(ax, low: float, high: float, min_span: float, log: bool = False) -> bool:
    """Limiti y con isteresi: cambiano (e serve ridisegnare tutto) solo se i dati escono o occupano
    meno di un terzo dell'asse. Vero se sono cambiati."""
    if log:
        low, high = math.log10(low), math.log10(high)
    lo, hi = ax.get_ylim()
    if log:
        lo, hi = math.log10(lo), math.log10(hi)
    span = max(high - low, min_span)
    if lo <= low and high <= hi and span >= (hi - lo) / 3.0:
        return False
    pad = span * 0.25
    lo, hi = low - pad, low + span + pad
    ax.set_ylim(*((10.0 ** lo, 10.0 ** hi) if log else (lo, hi)))
    return True


def window(bench: Bench) -> None:
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button, Slider, TextBox

    # Due finestre: grafici e comandi. Scrivere in un campo ridisegna solo la finestra dei comandi;
    # nei grafici si ridisegnano solo curve e testi (blitting), assi ed etichette solo quando i
    # limiti cambiano. Ridisegnare tutto costa ~150 ms, quasi tutti per il testo.
    fig = plt.figure(figsize=(14.0, 7.5))
    fig.canvas.manager.set_window_title("Banco acquisinator - grafici")
    ctl = plt.figure(figsize=(14.0, 2.6))
    ctl.canvas.manager.set_window_title("Banco acquisinator - comandi")

    grid = fig.add_gridspec(2, 2, left=0.06, right=0.87, top=0.80, bottom=0.12, hspace=0.35, wspace=0.22)
    ax_scope = fig.add_subplot(grid[0, 0])
    ax_spec = fig.add_subplot(grid[0, 1])
    ax_trend = fig.add_subplot(grid[1, :])
    ax_dc = ax_trend.twinx()

    (scope_line,) = ax_scope.plot([], [], linewidth=0.9, label="CH0 (ponte)")
    ax_scope1 = ax_scope.twinx()
    (scope1_line,) = ax_scope1.plot([], [], linewidth=0.9, color="tab:orange", label="CH1 (asse destro)")
    ax_scope1.set_visible(False)
    ax_scope.set_xlabel("tempo dall'inizio del periodo [ms]")
    ax_scope.set_ylabel("ADC [code]")
    ax_scope.grid(True)
    (spec1_line,) = ax_spec.semilogy([], [], linewidth=0.8, color="tab:orange", alpha=0.7)
    (spec_line,) = ax_spec.semilogy([], [], linewidth=0.8)
    ax_spec.set_xlabel("frequenza [Hz]")
    ax_spec.set_ylabel("ampiezza [code]")
    ax_spec.grid(True, alpha=0.4)
    harm_lines = [ax_spec.axvline(0, color="tab:red", linestyle=":", linewidth=0.8) for _ in range(HARMONICS)]
    (trend_a1,) = ax_trend.plot([], [], label="ampiezza a f", color="tab:blue")
    (trend_a2,) = ax_trend.plot([], [], label="ampiezza a 2f", color="tab:red")
    (trend_rms,) = ax_trend.plot([], [], label="rumore rms", color="tab:gray", linewidth=0.8)
    (trend_dc,) = ax_dc.plot([], [], label="DC (asse destro)", color="tab:green", linewidth=0.9)
    ax_out = ax_trend.twinx()
    ax_out.spines["right"].set_position(("axes", 1.07))
    (trend_out,) = ax_out.plot([], [], label="uscita CH0/CH1 [mV/V] (asse esterno)", color="tab:purple", linewidth=1.2)
    (trend_board,) = ax_out.plot([], [], label="lock-in scheda [mV/V] (asse esterno)", color="black", linewidth=1.0,
                                 linestyle="--")
    ax_out.set_ylabel("uscita [mV/V]")
    ax_out.set_visible(False)
    ax_trend.set_xlabel("tempo [s]")
    ax_trend.set_ylabel("[code]")
    ax_dc.set_ylabel("DC [code]")
    ax_trend.grid(True)
    ax_trend.legend(handles=[trend_a1, trend_a2, trend_rms, trend_dc, trend_out, trend_board], loc="upper left",
                    fontsize=8)
    ax_trend.set_ylim(0.0, 10.0)
    title = fig.text(0.06, 0.955, "", fontsize=12, weight="bold")
    numbers = fig.text(0.06, 0.935, "", fontsize=10, family="monospace", va="top")
    status = fig.text(0.06, 0.01, "", fontsize=10, family="monospace")
    ax_scope.set_title(" ", fontsize=10)
    ax_spec.set_title(" ", fontsize=10)

    lines = [scope_line, scope1_line, spec_line, spec1_line, trend_a1, trend_a2, trend_rms, trend_dc, trend_out,
             trend_board]
    texts = [title, numbers, status, ax_scope.title, ax_spec.title]
    for artist in lines + texts:
        artist.set_animated(True)

    t0 = time.monotonic()
    # (t, f, 2f, DC, rumore, uscita CH0/CH1 e lock-in della scheda in mV/V o nan)
    trend: deque[tuple[float, float, float, float, float, float, float]] = deque(maxlen=int(TREND_S / TREND_EVERY_S))
    view = {"ms": SCOPE_MS, "last_trend": 0.0, "filled": False, "last_flush": 0.0, "cal_freq": None,
            "full": True, "background": None, "text_background": None, "last_text": 0.0, "f0": None,
            "trend_x": (0.0, 0.0)}

    def draw_artists(artists) -> None:
        for artist in artists:
            if artist.get_visible() and (artist.axes is None or artist.axes.get_visible()):
                fig.draw_artist(artist)

    def draw_texts() -> None:
        """Sfondo + testi nuovi, salvato: fino ai testi successivi si ridisegnano solo le curve."""
        fig.canvas.restore_region(view["background"])
        draw_artists(texts)
        view["text_background"] = fig.canvas.copy_from_bbox(fig.bbox)
        view["last_text"] = time.monotonic()

    def on_draw(_event) -> None:
        # ridisegno completo (anche ridimensionando la finestra): nuovo sfondo senza curve e testi
        view["background"] = fig.canvas.copy_from_bbox(fig.bbox)
        draw_texts()
        draw_artists(lines)

    fig.canvas.mpl_connect("draw_event", on_draw)
    fig.canvas.mpl_connect("close_event", lambda _event: plt.close("all"))
    ctl.canvas.mpl_connect("close_event", lambda _event: plt.close("all"))

    # comandi
    row1, row2, row3 = 0.62, 0.40, 0.08
    height = 0.17

    def box(x, label, initial, width=0.08):
        return TextBox(ctl.add_axes((x, row1, width, height)), label, initial=initial)

    ctl.text(0.06, 0.86, "Eccitazione (0 = invariato)", fontsize=10, weight="bold")
    freq_box = box(0.10, "freq [Hz] ", "0")
    dc_box = box(0.25, "DC [mV] ", "0")
    amp_box = box(0.40, "amp [mV] ", "0")
    apply_button = Button(ctl.add_axes((0.50, row1, 0.08, height)), "Applica")
    preset_axes = [ctl.add_axes((0.10 + 0.085 * k, row2, 0.08, height)) for k in range(5)]
    presets = [("default", 125, 550, 300), ("DC 1650", 0, 1650, 300), ("amp 1", 0, 0, 1),
               ("amp 300", 0, 0, 300), ("125 Hz", 125, 0, 0)]
    preset_buttons = [Button(ax, label) for ax, (label, *_rest) in zip(preset_axes, presets)]

    ctl.text(0.66, 0.86, "Registrazione", fontsize=10, weight="bold")
    note_box = TextBox(ctl.add_axes((0.70, row1, 0.12, height)), "nota ", initial="")
    rec_button = Button(ctl.add_axes((0.84, row1, 0.10, height)), "REGISTRA", color="#f4c7c3", hovercolor="#ee9a92")
    reset_button = Button(ctl.add_axes((0.84, row2, 0.10, height)), "azzera grafico")
    slider = Slider(ctl.add_axes((0.70, row2 + 0.04, 0.10, 0.10)), "scope [ms] ", 2.0, 200.0, valinit=SCOPE_MS)

    def on_slider(value: float) -> None:
        view["ms"] = value
        view["full"] = True

    slider.on_changed(on_slider)

    cal = bench.cal
    ctl.text(0.06, 0.28, "Calibrazione CH1 (ritardo per frequenza; salvata in " + os.path.basename(cal.path) + ")",
             fontsize=10, weight="bold")
    delay_box = TextBox(ctl.add_axes((0.10, row3, 0.07, height)), "ritardo [°] ", initial="0")
    calibrate_button = Button(ctl.add_axes((0.18, row3, 0.11, height)), "calibra 1: a riposo")
    rest: dict = {}  # rapporto CH0/CH1 a riposo, tra i due passi della calibrazione
    gain_box = TextBox(ctl.add_axes((0.38, row3, 0.06, height)), "guadagno INA ", initial=f"{cal.gain:g}")
    divider_box = TextBox(ctl.add_axes((0.55, row3, 0.06, height)), "partitore V_CH1/V_ecc ",
                          initial=f"{cal.divider:g}")

    def parse(text: str) -> int | None:
        try:
            return max(0, int(float(text.strip() or 0)))
        except ValueError:
            return None

    def on_apply(_=None) -> None:
        values = [parse(b.text) for b in (freq_box, dc_box, amp_box)]
        if None in values:
            with bench.lock:
                bench.message = "valori non validi"
            return
        bench.apply(*values)

    def on_preset(index: int) -> None:
        _label, freq, dc, amp = presets[index]
        for b, value in ((freq_box, freq), (dc_box, dc), (amp_box, amp)):
            b.set_val(str(value))
        on_apply()

    def on_rec(_) -> None:
        if bench.rec_name is None:
            name = bench.start_recording(note_box.text.strip())
            rec_button.label.set_text("STOP")
            with bench.lock:
                bench.message = f"registro in {name}.csv"
        else:
            name = bench.stop_recording()
            rec_button.label.set_text("REGISTRA")
            with bench.lock:
                bench.message = f"salvato {name}.csv e {name}_lockin.csv"

    def on_reset(_) -> None:
        trend.clear()
        view["full"] = True

    def number(text: str) -> float | None:
        try:
            value = float(text.strip().replace(",", "."))
        except ValueError:
            return None
        return value if math.isfinite(value) else None

    def on_delay(text: str) -> None:
        value, freq = number(text), view["cal_freq"]
        if value is None or freq is None:
            return
        with bench.lock:
            cal.delays[cal.key(freq)] = value
            cal.save()

    def on_calibrate(_) -> None:
        """Due passi: rapporto a riposo, poi deformato (o con lo shunt). Il ritardo e' la fase della
        differenza: lo sbilanciamento a riposo del ponte ha una fase sua e non deve entrarci."""
        with bench.lock:
            ch1 = bench.analysis["ch1"] if bench.analysis else None
        if ch1 is None:
            with bench.lock:
                bench.message = "calibra fase: nessun CH1 (firmware senza ACQ_STREAM_CH1?)"
            return
        ratio = ch1["ratio"] * np.exp(1j * math.radians(ch1["dphase"]))
        if "ratio" not in rest:
            rest["ratio"] = ratio
            calibrate_button.label.set_text("calibra 2: deformato")
            with bench.lock:
                bench.message = "calibra fase: riposo salvato; ora deforma (o metti lo shunt), tieni fermo e ripremi"
            return
        change = ratio - rest.pop("ratio")
        calibrate_button.label.set_text("calibra 1: a riposo")
        if abs(change) < 0.05 * abs(ratio):
            with bench.lock:
                bench.message = ("calibra fase: variazione troppo piccola (meno del 5%), non salvata: "
                                 "deforma di piu' e rifai dai due passi")
            return
        measured = math.degrees(np.angle(change))
        delay_box.set_val(f"{Calibration.fold(measured):.2f}")  # chiama on_delay
        with bench.lock:
            bench.message = (f"ritardo {Calibration.fold(measured):.2f}° salvato per {view['cal_freq']:.1f} Hz "
                             f"(variazione {abs(change) / abs(ratio) * 100:.0f}%, fase {measured:+.1f}°: "
                             f"il resto e' il segno)")

    def on_gain(text: str) -> None:
        value = number(text)
        if value:
            with bench.lock:
                cal.gain = value
                cal.save()

    def on_divider(text: str) -> None:
        value = number(text)
        if value:
            with bench.lock:
                cal.divider = value
                cal.save()

    delay_box.on_submit(on_delay)
    calibrate_button.on_clicked(on_calibrate)
    gain_box.on_submit(on_gain)
    divider_box.on_submit(on_divider)
    apply_button.on_clicked(on_apply)
    for k, button in enumerate(preset_buttons):
        button.on_clicked(lambda _, k=k: on_preset(k))
    rec_button.on_clicked(on_rec)
    reset_button.on_clicked(on_reset)

    def update() -> None:
        now = time.monotonic()
        stream = bench.stream
        analysis = analyse(stream)
        with bench.lock:
            bench.analysis = analysis
            exc, bridge, message, errors = bench.exc, bench.bridge, bench.message, bench.errors
            bench.errors = 0
            rec_name, rec_start = bench.rec_name, bench.rec_start
        if now - view["last_flush"] > 1.0:
            bench.flush()
            view["last_flush"] = now
        full = view["full"]

        # titolo ed eccitazione
        if exc is None:
            title.set_text("Nessun ACQUISINATOR_EXC: scheda accesa?")
        else:
            default = "  (DEFAULT, scheda riavviata?)" if int(exc["id"]) == 0 else ""
            title.set_text(f"Eccitazione id {int(exc['id'])}: {exc['freq']:.1f} Hz, DC {int(exc['dc'])} mV, "
                           f"amp {int(exc['amp'])} mV, {int(exc['samples'])} campioni/periodo{default}")
            if not view["filled"]:
                for b, value in ((freq_box, exc["freq"]), (dc_box, exc["dc"]), (amp_box, exc["amp"])):
                    b.set_val(f"{value:g}")
                view["filled"] = True

        # oscilloscopio
        with stream.lock:
            data = list(stream.scope)
            tick_hz = stream.tick_hz
        if data and tick_hz:
            count = min(len(data), max(2, int(view["ms"] / 1000.0 * tick_hz)))
            start = len(data) - count
            for k in range(start, -1, -1):
                if data[k][1] == 0:
                    start = k
                    break
            shown = [value for value, *_ in data[start:start + count]]
            times = [1000.0 * k / tick_hz for k in range(len(shown))]
            scope_line.set_data(times, shown)
            shown1 = [(t, ch1) for t, (_, _, ch1) in zip(times, data[start:start + count]) if ch1 is not None]
            if ax_scope1.get_visible() != bool(shown1):
                ax_scope1.set_visible(bool(shown1))
                full = True
            if shown1:
                scope1_line.set_data([t for t, _ in shown1], [v for _, v in shown1])
                values1 = [v for _, v in shown1]
                full |= fit_limits(ax_scope1, min(values1), max(values1), 10.0)
            if ax_scope.get_xlim() != (0.0, view["ms"]):
                ax_scope.set_xlim(0.0, view["ms"])
                ax_scope1.set_xlim(0.0, view["ms"])
                full = True
            low, high = min(shown), max(shown)
            full |= fit_limits(ax_scope, low, high, 10.0)
            ax_scope.set_title(f"ADC  min {low}  max {high}  p-p {high - low} code ({volts(high - low) * 1000:.1f} mV)",
                               fontsize=10)

        # spettro, numeri e andamento
        out = math.nan
        if analysis is not None:
            freqs, amps = analysis["freqs"][1:], analysis["amps"][1:]
            spec_line.set_data(freqs, np.maximum(amps, 1e-3))
            if ax_spec.get_xlim() != (0.0, analysis["tick_hz"] / 2.0):
                ax_spec.set_xlim(0.0, analysis["tick_hz"] / 2.0)
                full = True
            ch1 = analysis["ch1"]
            top = float(amps.max())
            if spec1_line.get_visible() != (ch1 is not None):
                spec1_line.set_visible(ch1 is not None)
            if ch1 is not None:
                spec1_line.set_data(freqs, np.maximum(ch1["amps"][1:], 1e-3))
                top = max(top, float(ch1["amps"][1:].max()))
            full |= fit_limits(ax_spec, max(float(np.median(amps)) / 10.0, 1e-3), max(top * 2.0, 1.0), 1.0, log=True)
            if view["f0"] != analysis["f0"]:
                view["f0"] = analysis["f0"]
                for h, line in enumerate(harm_lines, start=1):
                    line.set_xdata([h * analysis["f0"]] * 2)
                full = True
            ax_spec.set_title(f"spettro ultimo {ANALYSIS_S:g} s (rosso: f, 2f, 3f, 4f"
                              + ("; arancione: CH1)" if ch1 is not None else ")"), fontsize=10)
            harm = analysis["harm"]
            freq = analysis["f0"] if ch1 is None else ch1["freq"]  # la calibrazione segue l'eccitazione vera
            if view["cal_freq"] is None or cal.key(freq) != cal.key(view["cal_freq"]):
                view["cal_freq"] = freq  # frequenza nuova: il suo ritardo, se gia' calibrato
                delay_box.set_val(f"{cal.delay(freq):g}")
            with bench.lock:
                out = math.nan if ch1 is None else cal.output(ch1, freq)
                delay = cal.delay(freq)
            numbers.set_text(
                f"f {harm[1]:7.1f}   2f {harm[2]:6.1f}   3f {harm[3]:6.1f}   4f {harm[4]:6.1f}   "
                f"DC {harm[0]:8.1f}   fase {analysis['phase1']:6.1f}°   rumore rms {analysis['rms']:6.1f}   "
                f"8k/3 {analysis['tone3']:5.2f}   persi {100 * analysis['missing']:.1f}%   [code]"
                + ("" if ch1 is None else
                   f"\nCH1  f {ch1['a1']:8.1f}   DC {ch1['dc']:8.1f}   generatore {ch1['freq']:8.3f} Hz   "
                   f"CH0 a quella f {ch1['a0']:7.1f}   CH0/CH1 {ch1['ratio']:.6f}   sfasamento CH0-CH1 {ch1['dphase']:+7.2f}°"
                   + (f"   !! SCHEDA A {analysis['f0']:.1f} Hz: impostala come il generatore"
                      if abs(ch1["freq"] - analysis["f0"]) > 0.3 else "") + "\n"
                   f"USCITA {out:+.5f} mV/V   (ritardo {delay:+.2f}°, residuo di fase "
                   f"{(ch1['dphase'] - delay + 90.0) % 180.0 - 90.0:+.2f}°, guadagno {cal.gain:g}, partitore {cal.divider:g})"))
            if now - view["last_trend"] >= TREND_EVERY_S:
                with bench.lock:
                    board = bench.bridge_sum / bench.bridge_count if bench.bridge_count else math.nan
                    bench.bridge_sum, bench.bridge_count = 0.0, 0
                trend.append((now - t0, harm[1], harm[2], harm[0], analysis["rms"], out, board))
                view["last_trend"] = now
        if trend:
            t = [p[0] for p in trend]
            trend_a1.set_data(t, [p[1] for p in trend])
            trend_a2.set_data(t, [p[2] for p in trend])
            trend_rms.set_data(t, [p[4] for p in trend])
            trend_dc.set_data(t, [p[3] for p in trend])
            # asse x a passi di 30 s: si ridisegna tutto solo quando si allarga o scorre
            x_low, x_high = view["trend_x"]
            if t[-1] > x_high or t[0] > x_low + 30.0 or t[0] < x_low:
                view["trend_x"] = (t[0], max(t[-1], t[0] + 10.0) + 30.0)
                ax_trend.set_xlim(*view["trend_x"])
                full = True
            top = max(max(p[1] for p in trend), max(p[2] for p in trend), max(p[4] for p in trend))
            if top > ax_trend.get_ylim()[1] or top < ax_trend.get_ylim()[1] / 3.0:
                ax_trend.set_ylim(0.0, top * 1.3 + 1.0)
                full = True
            dcs = [p[3] for p in trend]
            full |= fit_limits(ax_dc, min(dcs), max(dcs), 20.0)
            outs = [p[k] for p in trend for k in (5, 6) if not math.isnan(p[k])]
            if ax_out.get_visible() != bool(outs):
                ax_out.set_visible(bool(outs))
                full = True
            if outs:
                trend_out.set_data(t, [p[5] for p in trend])
                trend_board.set_data(t, [p[6] for p in trend])
                full |= fit_limits(ax_out, min(outs), max(outs), 1e-3)

        # stato
        notes = []
        if bridge is not None:
            flags = int(bridge["flags"])
            names = ", ".join(name for bit, name in FLAG_NAMES if flags & (1 << bit)) or "nessun flag"
            notes.append(f"scheda: {bridge['bridge_out']:+.5f} mV/V, fase {bridge['phase']:+.1f}°, {names}")
        if errors:
            notes.append(f"error frame {errors}")
        if now - bench.last_frame > 1.0:
            notes.append("NESSUN FRAME")
        if rec_name is not None:
            notes.append(f"● REC {now - rec_start:5.0f} s")
        status.set_text(f"{message}\n" + "   |   ".join(notes))

        if full or view["background"] is None:
            view["full"] = False
            fig.canvas.draw_idle()  # on_draw rifa lo sfondo e ci disegna sopra curve e testi
        else:
            if now - view["last_text"] >= TEXT_EVERY_S:
                draw_texts()
            else:
                fig.canvas.restore_region(view["text_background"])
            draw_artists(lines)
            fig.canvas.blit(fig.bbox)

    timer = fig.canvas.new_timer(interval=1000 // FPS)
    timer.add_callback(update)
    timer.start()
    plt.show()
    timer.stop()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--dir", default="registrazioni", help="cartella delle registrazioni")
    parser.add_argument("--cal", default="banco_cal.json", help="file della calibrazione di CH1")
    args = parser.parse_args()

    bus = open_bus(bitrate=BITRATE)
    bench = Bench(bus, args.dir, Calibration(args.cal))
    reader = threading.Thread(target=bench.run, daemon=True)
    reader.start()
    try:
        window(bench)  # ritorna alla chiusura della finestra
    except KeyboardInterrupt:
        pass
    finally:
        bench.stop.set()
        reader.join(timeout=1.0)
        name = bench.stop_recording()
        if name is not None:
            print(f"Salvato {name}.csv e {name}_lockin.csv")
        send_stream(bus, False)
        bus.shutdown()


if __name__ == "__main__":
    main()
