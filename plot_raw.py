#!/usr/bin/env python3
"""Plotta ACQUISINATOR_RAW da SocketCAN (can0), con salvataggio CSV.

Uso: python plot_raw.py [--channel can0] [--window S] [--max-record S] [--volts] [path/al/acquisinator.dbc]

L'interfaccia SocketCAN va creata e portata up prima di lanciare lo script, ad esempio:
  sudo ip link set can0 up type can bitrate 1000000

Il firmware manda un frame 0x12 (ACQUISINATOR_RAW) per ogni campione dell'ADC, a 8 kHz:
qui i campioni vengono raccolti in un thread e il grafico mostra gli ultimi --window
secondi (0 = tutto). Il grafico viene ridisegnato al massimo PLOT_FPS volte al secondo e,
quando i punti sono troppi, ogni gruppo di campioni viene sostituito con la sua media:
cosi' la traccia resta leggera e scorre fluida invece di avanzare a scatti.

Il bottone "Salva CSV" scrive tutti i campioni tenuti in memoria (al massimo --max-record
secondi, 0 = illimitato) in un file CSV nella cartella corrente, con il nome
acquisinator_raw_AAAAMMGG_HHMMSS.csv e le colonne time_s, unix_time, adc_raw, voltage_v.
La registrazione non si ferma: si puo' salvare piu' volte.
"""

import argparse
import csv
import sys
import threading
import time
from array import array
from bisect import bisect_left
from collections import deque
from datetime import datetime
from pathlib import Path

import can
import cantools
import numpy as np

RAW_MESSAGE_NAME = "ACQUISINATOR_RAW"
DEFAULT_CHANNEL = "can0"
PLOT_FPS = 25
MAX_PLOT_POINTS = 1500  # punti disegnati per volta: oltre, si media ogni gruppo di campioni
STATS_SECONDS = 1.0  # finestra per media, rumore e frequenza di campionamento
STATS_REFRESH_S = 1.0  # il testo delle statistiche cambia al massimo una volta al secondo
ADC_VREF_V = 3.29  # LTC1865L_VREF
ADC_FULL_SCALE = 65536

DEFAULT_DBC = Path(__file__).resolve().parent / "acquisinator.dbc"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("dbc", nargs="?", type=Path, default=DEFAULT_DBC, help="path all'acquisinator.dbc")
    parser.add_argument(
        "--channel",
        default=DEFAULT_CHANNEL,
        help=f"interfaccia SocketCAN (default: {DEFAULT_CHANNEL})",
    )
    parser.add_argument(
        "--window",
        type=float,
        default=10.0,
        metavar="S",
        help="secondi mostrati nel grafico, 0 = tutta la registrazione (default: 10)",
    )
    parser.add_argument(
        "--max-record",
        type=float,
        default=300.0,
        metavar="S",
        help="secondi di campioni tenuti in memoria (e salvabili), 0 = illimitato (default: 300)",
    )
    parser.add_argument(
        "--volts",
        action="store_true",
        help="asse y in volt invece che in codici ADC (il CSV contiene sempre entrambi)",
    )
    return parser.parse_args()


class Record:
    """Campioni grezzi in memoria; array compatti per reggere 8 kHz senza gonfiare la RAM."""

    TRIM_CHECK = 4096  # lo scorrimento del buffer si controlla ogni tot campioni

    def __init__(self, max_record: float) -> None:
        self.t = array("d")  # timestamp assoluti (s)
        self.raw = array("H")  # codici ADC
        self.max_record = max_record
        self.t0: float | None = None
        self.appends = 0

    def __len__(self) -> int:
        return len(self.t)

    def append(self, timestamp: float, raw: int) -> None:
        if self.t0 is None:
            self.t0 = timestamp
        self.t.append(timestamp)
        self.raw.append(raw)
        self.appends += 1
        if self.max_record > 0 and self.appends % self.TRIM_CHECK == 0:
            # si taglia a blocchi, non a ogni campione: del self.t[:1] a 8 kHz
            # sposterebbe tutto il buffer 8000 volte al secondo
            cutoff = self.t[-1] - self.max_record
            if self.t[0] < cutoff:
                drop = bisect_left(self.t, cutoff)
                del self.t[:drop]
                del self.raw[:drop]

    def voltage(self, raw: int) -> float:
        return raw * ADC_VREF_V / ADC_FULL_SCALE


def read_frames(bus: can.BusABC, raw_message, stop: threading.Event, samples: deque) -> None:
    while not stop.is_set():
        frame = bus.recv(timeout=0.1)
        if frame is None or frame.is_error_frame:
            continue
        if frame.arbitration_id != raw_message.frame_id:
            continue  # ACQUISINATOR_DIAG / BRIDGE non interessano qui
        data = raw_message.decode(frame.data)
        samples.append((frame.timestamp, int(data["adc_raw"])))


def noise_stats(values: np.ndarray) -> tuple[float, float, float]:
    """Media, deviazione standard e picco-picco."""
    mean = float(values.mean())
    return mean, float(values.std()), float(values.max() - values.min())


def decimate(xs: np.ndarray, ys: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Riduce i punti facendo la media di ogni gruppo di campioni.

    La media (invece di tenere un campione ogni N) evita l'aliasing e soprattutto
    produce una traccia molto piu' corta da disegnare: con il rumore dell'ADC
    l'inviluppo min/max faceva oscillare la linea da un estremo all'altro a ogni
    punto e matplotlib rallentava fino a scatti di centinaia di ms.
    """
    n = len(xs)
    if n <= MAX_PLOT_POINTS:
        return xs, ys
    step = max(2, n // MAX_PLOT_POINTS)
    starts = np.arange(0, n, step)
    counts = np.diff(np.append(starts, n))
    out_x = (xs[starts] + xs[np.minimum(starts + counts - 1, n - 1)]) / 2.0
    out_y = np.add.reduceat(ys, starts) / counts
    return out_x, out_y


def plot_live(record: Record, samples: deque, window: float, volts: bool) -> None:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.transforms import Bbox
    from matplotlib.widgets import Button

    unit = "V" if volts else "code"
    scale = ADC_VREF_V / ADC_FULL_SCALE if volts else 1.0

    fig, ax = plt.subplots(figsize=(9.0, 5.5))
    # spazio in alto per le statistiche, fuori dagli assi: cosi' il blitting della
    # linea non le cancella a ogni frame
    fig.subplots_adjust(bottom=0.18, top=0.83)
    ax.set_xlabel("tempo [s]")
    ax.set_ylabel(f"ACQUISINATOR_RAW.adc_raw [{unit}]")
    ax.ticklabel_format(useOffset=False)
    ax.grid(True)
    if window > 0:
        # asse x fisso 0..window: i campioni scorrono verso sinistra, cosi' tick ed
        # etichette non cambiano a ogni frame e si puo' usare il blitting
        ax.set_xlim(0.0, window)

    (line,) = ax.plot([], [])
    stats_text = fig.text(
        0.01, 0.845, "in attesa di campioni...", va="bottom", ha="left", family="monospace", fontsize=9
    )
    fig.suptitle("ACQUISINATOR_RAW (0x12)")

    def shown_range() -> tuple[int, int]:
        if not len(record):
            return 0, 0
        first = 0 if window <= 0 else bisect_left(record.t, record.t[-1] - window)
        return first, len(record)

    last_stats = 0.0

    def redraw() -> tuple[bool, bool]:
        """Aggiorna linea e statistiche; ritorna (limiti y cambiati, testo cambiato)."""
        nonlocal last_stats
        first, last = shown_range()
        if not last:
            line.set_data([], [])
            if stats_text.get_text() != "in attesa di campioni...":
                stats_text.set_text("in attesa di campioni...")
                return False, True
            return False, False

        count = last - first
        # viste senza copia sugli array compatti del Record
        times = np.frombuffer(record.t, dtype=np.float64, count=count, offset=first * record.t.itemsize)
        raws = np.frombuffer(record.raw, dtype=np.uint16, count=count, offset=first * record.raw.itemsize)
        if window > 0:
            # relativo all'ultimo campione: il piu' recente resta sul bordo destro
            xs = times - record.t[-1] + window
        else:
            xs = times - record.t0
        ys = raws * scale
        xs, ys = decimate(xs, ys)
        line.set_data(xs, ys)
        if window <= 0:
            ax.set_xlim(0.0, max(1.0, float(xs[-1])))

        low, high = float(ys.min()), float(ys.max())
        span = high - low
        # margine abbondante: la scala non deve saltare per ogni nuovo picco di rumore
        margin = span * 0.25 if span > 0 else (0.001 if volts else 1.0)
        y_low, y_high = low - margin, high + margin
        current_low, current_high = ax.get_ylim()
        # isteresi: si cambia scala solo se i dati escono davvero dalla vista o se
        # la vista e' diventata molto piu' grande del necessario; ogni cambio costa
        # un ridisegno completo di tick ed etichette
        changed = low < current_low or high > current_high or span < 0.5 * (current_high - current_low)
        if changed:
            ax.set_ylim(y_low, y_high)

        now = time.monotonic()
        text_changed = False
        if now - last_stats >= STATS_REFRESH_S:
            # rigenerare il testo ogni frame costa molto (layout dei glifi) e non
            # serve: le statistiche si aggiornano al massimo una volta al secondo
            last_stats = now
            recent_first = max(first, bisect_left(record.t, record.t[-1] - STATS_SECONDS))
            recent = np.frombuffer(
                record.raw, dtype=np.uint16, count=last - recent_first, offset=recent_first * record.raw.itemsize
            )
            mean, std, peak_peak = noise_stats(recent)
            rate = len(recent) / STATS_SECONDS
            stats_text.set_text(
                f"{len(record):8d} campioni in memoria, {rate:7.0f} Sa/s   "
                f"media {mean * scale:.4f} {unit}, rumore {std * scale:.4f} {unit} rms,"
                f" {peak_peak * scale:.4f} {unit} p-p"
            )
            text_changed = True
        return changed, text_changed

    def save_csv(_event) -> None:
        if not len(record):
            print("Nessun campione da salvare")
            stats_text.set_text("nessun campione da salvare")
            fig.canvas.draw_idle()
            return
        path = Path.cwd() / f"acquisinator_raw_{datetime.now():%Y%m%d_%H%M%S}.csv"
        with path.open("w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(("time_s", "unix_time", "adc_raw", "voltage_v"))
            for timestamp, raw in zip(record.t, record.raw):
                writer.writerow((f"{timestamp - record.t0:.6f}", f"{timestamp:.6f}", raw, f"{record.voltage(raw):.6f}"))
        print(f"Salvati {len(record)} campioni in {path}")
        stats_text.set_text(f"salvati {len(record)} campioni in {path}")
        fig.canvas.draw_idle()

    # tenere il riferimento al bottone: senza, smette di rispondere ai click
    button = Button(fig.add_axes((0.40, 0.03, 0.20, 0.07)), "Salva CSV")
    button.on_clicked(save_csv)

    def advance() -> tuple[bool, bool, bool]:
        """Svuota la coda; ritorna (novita', limiti y cambiati, testo cambiato)."""
        if not samples:
            return False, False, False
        while samples:
            record.append(*samples.popleft())
        ylim_changed, text_changed = redraw()
        return True, ylim_changed, text_changed

    def update(_):
        advance()  # FuncAnimation ridisegna da solo

    redraw()

    # Con l'asse x fisso si ridisegna solo la linea sopra uno sfondo gia'
    # renderizzato: molto piu' fluido che ridisegnare tick ed etichette a ogni
    # frame. Le statistiche stanno fuori dagli assi e vengono ridisegnate solo
    # quando cambiano. Se il backend non supporta timer/blit si usa FuncAnimation.
    bg = None
    stats_bg = None

    def stats_bbox() -> Bbox:
        width, height = fig.canvas.get_width_height()
        return Bbox.from_extents(0.0, height * 0.835, width, height * 0.915)

    def recapture() -> None:
        nonlocal bg, stats_bg
        line.set_visible(False)
        stats_text.set_visible(False)
        try:
            fig.canvas.draw()
            bg = fig.canvas.copy_from_bbox(ax.bbox)
            stats_bg = fig.canvas.copy_from_bbox(stats_bbox())
        finally:
            line.set_visible(True)
            stats_text.set_visible(True)

    def blit_stats() -> None:
        if stats_bg is None:
            return
        region = stats_bbox()
        fig.canvas.restore_region(stats_bg)
        fig.draw_artist(stats_text)
        fig.canvas.blit(region)

    def blit_frame() -> None:
        fig.canvas.restore_region(bg)
        ax.draw_artist(line)
        fig.canvas.blit(ax.bbox)

    def on_resize(_event) -> None:
        nonlocal bg, stats_bg
        bg = None
        stats_bg = None

    def on_timer() -> None:
        drew, limits_changed, text_changed = advance()
        if not drew:
            return
        if bg is None or stats_bg is None or limits_changed:
            recapture()
            text_changed = True
        if text_changed:
            blit_stats()
        blit_frame()

    timer = None
    if window > 0:
        try:
            recapture()
            blit_stats()
            blit_frame()
            timer = fig.canvas.new_timer(interval=1000 // PLOT_FPS)
            timer.add_callback(on_timer)
            timer.start()
            fig.canvas.mpl_connect("resize_event", on_resize)
            fig.canvas.mpl_connect("close_event", lambda _event: timer.stop())
        except (NotImplementedError, AttributeError, RuntimeError):
            timer = None

    if timer is None:
        # tenere il riferimento: senza, l'animazione viene raccolta dal GC e si ferma
        anim = FuncAnimation(fig, update, interval=1000 // PLOT_FPS, cache_frame_data=False)  # noqa: F841

    plt.show()
    if timer is not None:
        timer.stop()


def main() -> None:
    args = parse_args()
    if not args.dbc.is_file():
        sys.exit(f"{args.dbc} non trovato: passalo come argomento")
    db = cantools.database.load_file(args.dbc)
    raw_message = db.get_message_by_name(RAW_MESSAGE_NAME)
    print(f"DBC: {args.dbc}")
    print(f"In ascolto di {raw_message.name} (0x{raw_message.frame_id:X})...")

    try:
        bus = can.Bus(interface="socketcan", channel=args.channel)
    except (can.CanError, OSError) as error:
        sys.exit(f"impossibile aprire {args.channel}: {error} (interfaccia su? vedi docstring)")

    record = Record(args.max_record)
    samples: deque = deque()
    stop = threading.Event()
    reader = threading.Thread(target=read_frames, args=(bus, raw_message, stop, samples), daemon=True)
    try:
        reader.start()
        plot_live(record, samples, args.window, args.volts)  # ritorna alla chiusura della finestra
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        reader.join(timeout=1.0)
        bus.shutdown()


if __name__ == "__main__":
    main()
