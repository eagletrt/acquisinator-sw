#!/usr/bin/env python3
"""Legge ACQUISINATOR_BRIDGE / ACQUISINATOR_DIAG dal CAN tramite transceiver gs_usb.

Uso: python plot_can.py [--signal {deformation,bridge_out,lockin_out,phase,adc_raw}] [--no-zero] [--no-plot] [--window S] [path/al/acquisinator.dbc]

Di default apre un grafico live della deformazione in microstrain, ricavata dalla resistenza
misurata dal lock-in con la stessa formula dei vecchi acquisinator. All'avvio la lettura viene
azzerata sulla media dei primi campioni (puntone scarico!); con --no-zero lo zero e' a 0 mV/V.
Il CAN viene
letto in un thread separato e il grafico viene ridisegnato solo quando arrivano campioni
nuovi, al massimo PLOT_FPS volte al secondo: cosi' non accumula ritardo.

Nel grafico la grandezza mostrata si cambia con il selettore a destra (--signal sceglie
solo quella iniziale); la storia di tutte le grandezze viene tenuta in parallelo.
"Set high" / "Set low" fissano come estremo alto / basso il valore attuale (media dell'ultimo
secondo): portare il segnale al massimo prima della saturazione e premere "Set high", poi al
minimo e premere "Set low". Gli estremi vengono presi nello stesso istante per deformazione,
resistenza e millivolt, quindi valgono per tutte e tre (la fase non ha un range). Con entrambi
impostati i valori vengono centrati sul punto medio del range (curva, media e asse, non solo
la vista) e il rumore viene mostrato anche in % del range. "Reset" toglie il range e torna ai
valori assoluti con autoscala. Vale solo nell'app: il firmware non viene toccato.

Gli slider sotto il grafico lo dimensionano: "finestra" i secondi mostrati (in fondo alla corsa
tutta la storia tenuta, almeno WINDOW_MAX_S), "zoom y" da 1/16 a 16 volte l'autoscala sui punti
visibili, "sposta y" sposta la vista in su o in giu' (in mezze altezze). Reset rimette zoom e
spostamento.
"""

import argparse
import sys
import threading
from collections import deque
from pathlib import Path

import can
import cantools

from gs_usb_bus import open_bus

BRIDGE_MESSAGE_NAME = "ACQUISINATOR_BRIDGE"
DIAG_MESSAGE_NAME = "ACQUISINATOR_DIAG"
BITRATE = 1_000_000
PLOT_FPS = 20
ADC_VREF_V = 3.29  # LTC1865L_VREF
ADC_FULL_SCALE = 65536

# Come nel firmware dei vecchi acquisinator (acquisinator_config.h)
STRAIN_GAUGE_K = 2.01  # gauge factor
# Medie a tempo, non a numero di campioni: le misure al secondo dipendono dalla frequenza
# di eccitazione impostata a runtime (set_exc.py)
ZERO_SECONDS = 1.0  # media per lo zero
SET_SECONDS = 1.0  # media di "Set high" / "Set low"
NOISE_SECONDS = 5.0  # finestra su cui si calcola il rumore
RANGE_MARGIN = 0.05  # margine sopra e sotto il range impostato, in frazione del range
WINDOW_MAX_S = 300.0  # corsa dello slider della finestra (e storia minima tenuta)
# La fase non varia in modo monotono con il carico: niente estremi alto / basso
RANGED_SIGNALS = ("deformation", "bridge_out", "lockin_out")

DEFAULT_DBC = Path(__file__).resolve().parent / "acquisinator.dbc"

# segnale -> (messaggio, etichetta asse y)
SIGNALS = {
    "deformation": (BRIDGE_MESSAGE_NAME, "deformazione [µε]"),
    "bridge_out": (BRIDGE_MESSAGE_NAME, "uscita del ponte [mV/V]"),
    "phase": (BRIDGE_MESSAGE_NAME, "fase [deg]"),
    "lockin_out": (DIAG_MESSAGE_NAME, "componente in fase [mV]"),
    "adc_raw": (DIAG_MESSAGE_NAME, "valore ADC, media della finestra [code]"),
}

FLAG_NAMES = ((0, "SATURA BASSO"), (1, "SATURA ALTO"), (2, "ERRORE SPI"), (3, "non calibrato"), (4, "NESSUNA MISURA"), (5, "TICK PERSO"))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("dbc", nargs="?", type=Path, default=DEFAULT_DBC, help="path all'acquisinator.dbc")
    parser.add_argument(
        "--signal",
        choices=SIGNALS,
        default="deformation",
        help="grandezza mostrata all'avvio, poi si cambia dal grafico (default: deformation)",
    )
    parser.add_argument(
        "--no-zero",
        action="store_true",
        help="non azzerare all'avvio: deformazione riferita a 0 mV/V (ponte bilanciato)",
    )
    parser.add_argument("--no-plot", action="store_true", help="stampa soltanto, senza grafico")
    parser.add_argument(
        "--window",
        type=float,
        default=30.0,
        metavar="S",
        help="secondi mostrati nel grafico, 0 = tutta la storia (default: 30)",
    )
    return parser.parse_args()


def deformation(bridge_out: float, out_zero: float) -> float:
    """Deformazione in microstrain da un quarto di ponte, stessa convenzione di segno del vecchio
    firmware (positiva quando la resistenza scende): dR/R = 4 * uscita del ponte."""
    return 4.0 * (bridge_out - out_zero) / 1000.0 / STRAIN_GAUGE_K * 1e6


def read_frames(bus: can.BusABC, bridge, diag, zero: bool, stop: threading.Event, samples: deque | None) -> None:
    out_zero = None if zero else 0.0
    zero_buffer: list[float] = []
    zero_start = None
    while not stop.is_set():
        frame = bus.recv(timeout=0.1)
        if frame is None or frame.is_error_frame:
            continue
        if frame.arbitration_id == bridge.frame_id:
            data = bridge.decode(frame.data)
            flags = int(data["flags"])
            notes = ", ".join(name for bit, name in FLAG_NAMES if flags & (1 << bit))
            if data["bridge_out"] != data["bridge_out"]:  # NaN: frame di stato senza misura
                print(f"{frame.timestamp:.3f}  {notes}")
                continue
            if out_zero is None:
                zero_buffer.append(data["bridge_out"])
                zero_start = frame.timestamp if zero_start is None else zero_start
                if frame.timestamp - zero_start < ZERO_SECONDS:
                    continue
                out_zero = sum(zero_buffer) / len(zero_buffer)
                print(f"Zero: {out_zero:.5f} mV/V (media di {len(zero_buffer)} campioni in {ZERO_SECONDS:.0f} s)")
            data["deformation"] = deformation(data["bridge_out"], out_zero)
            print(
                f"{frame.timestamp:.3f}  out={data['bridge_out']:+9.5f} mV/V  def={data['deformation']:+9.1f} µε"
                f"  fase={data['phase']:+7.2f} deg  {notes}"
            )
        elif frame.arbitration_id == diag.frame_id:
            data = diag.decode(frame.data)
            mean_v = data["adc_raw"] * ADC_VREF_V / ADC_FULL_SCALE
            data["lockin_out"] *= 1000.0  # il firmware lo manda in V, qui si lavora in mV
            print(f"{frame.timestamp:.3f}  v_out={data['lockin_out']:+9.3f} mV  adc_raw={data['adc_raw']:5.0f}  media={mean_v:.4f} V")
        else:
            continue
        if samples is not None:
            message_name = bridge.name if frame.arbitration_id == bridge.frame_id else diag.name
            for signal, (source, _) in SIGNALS.items():
                if source == message_name:
                    samples.append((frame.timestamp, signal, data[signal]))


def noise_stats(values: list[float]) -> tuple[float, float, float]:
    """Media, deviazione standard e picco-picco di una serie di campioni."""
    mean = sum(values) / len(values)
    std = (sum((v - mean) ** 2 for v in values) / len(values)) ** 0.5
    return mean, std, max(values) - min(values)


def plot_live(samples: deque, initial_signal: str, window: float) -> None:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.widgets import Button, RadioButtons, Slider

    fig, ax = plt.subplots(figsize=(9.0, 6.5))
    fig.subplots_adjust(bottom=0.32, top=0.82, right=0.76)
    ax.set_xlabel("tempo [s]")
    ax.ticklabel_format(useOffset=False)
    ax.grid(True)

    t0 = None
    current = initial_signal
    # Storia tenuta: almeno quanto lo slider della finestra puo' mostrare (0 = tutta)
    history = 0.0 if window <= 0 else max(window, WINDOW_MAX_S)
    # Dimensionamento dagli slider: finestra in s (0 = tutta la storia), zoom e spostamento verticali
    view = { "window": window, "zoom": 1.0, "shift": 0.0 }
    # Storia e range di ogni grandezza (valori assoluti): cambiando selezione non si perde niente
    series: dict[str, tuple[list[float], list[float]]] = { name: ([], []) for name in SIGNALS }
    limits: dict[str, dict[str, float | None]] = { name: { "high": None, "low": None } for name in SIGNALS }

    (line,) = ax.plot([], [])
    limit_lines = {
        "high": ax.axhline(0.0, color="tab:red", linestyle="--", visible=False),
        "low": ax.axhline(0.0, color="tab:green", linestyle="--", visible=False),
    }
    stats_text = ax.text(0.0, 1.02, "", transform=ax.transAxes, va="bottom", family="monospace", fontsize=9)

    def unit_of(signal: str) -> str:
        label = SIGNALS[signal][1]
        return label[label.index("[") + 1 : label.index("]")]

    def redraw() -> None:
        xs, ys = series[current]
        unit = unit_of(current)
        high, low = limits[current]["high"], limits[current]["low"]
        ranged = high is not None and low is not None and high != low
        # Con il range impostato tutto viene riferito al suo punto medio
        center = (high + low) / 2.0 if ranged else 0.0

        fig.suptitle(f"{SIGNALS[current][0]}.{current}")
        ax.set_ylabel(SIGNALS[current][1] + (" (centrato)" if ranged else ""))
        for which, value in (("high", high), ("low", low)):
            limit_lines[which].set_visible(value is not None)
            if value is not None:
                limit_lines[which].set_ydata([value - center, value - center])

        shown = [y - center for y in ys]
        line.set_data(xs, shown)
        shown_window = view["window"]
        visible = shown
        if xs:
            if shown_window > 0:
                x_low = max(0.0, xs[-1] - shown_window)
                ax.set_xlim(x_low, max(shown_window, xs[-1]))
                visible = [y for x, y in zip(xs, shown) if x >= x_low]
            else:
                ax.set_xlim(xs[0], max(xs[-1], xs[0] + 1.0))

        # Asse y: range impostato, oppure autoscala sui soli punti visibili; poi zoom e spostamento
        mid, half = 0.0, 1.0
        if ranged:
            half = abs(high - low) * (0.5 + RANGE_MARGIN)
        elif visible:
            y_low, y_high = min(visible), max(visible)
            mid = (y_low + y_high) / 2.0
            half = (y_high - y_low) / 2.0 * (1.0 + RANGE_MARGIN) or max(abs(mid) * 0.01, 1e-6)
        mid += view["shift"] * half
        half /= view["zoom"]
        ax.set_ylim(mid - half, mid + half)

        text = "in attesa di campioni..."
        if xs:
            recent = [y for x, y in zip(xs, shown) if x >= xs[-1] - NOISE_SECONDS]
            mean, std, peak_peak = noise_stats(recent)
            text = f"ultimi {NOISE_SECONDS:.0f} s: media {mean:.3f} {unit}\nrumore {std:.3f} {unit} rms, {peak_peak:.3f} {unit} p-p"
            if ranged:
                span = abs(high - low)
                text += (
                    f"\nrange {span:.3f} {unit} (centro {center:.3f} {unit}):"
                    f" rumore {100.0 * std / span:.3f} % rms, {100.0 * peak_peak / span:.3f} % p-p"
                )
        stats_text.set_text(text)
        fig.canvas.draw_idle()

    def update(_):
        nonlocal t0
        if not samples:
            return  # niente di nuovo: nessun ridisegno
        while samples:
            t, signal, value = samples.popleft()
            if t0 is None:
                t0 = t
            xs, ys = series[signal]
            xs.append(t - t0)
            ys.append(value)

        if history > 0:
            for xs, ys in series.values():
                while xs and xs[0] < xs[-1] - history:
                    del xs[0], ys[0]
        redraw()

    def select_signal(signal: str) -> None:
        nonlocal current
        current = signal
        redraw()

    # Le grandezze sono la stessa misura in unita' diverse: l'estremo viene preso
    # nello stesso istante per tutte, cosi' vale qualunque sia quella mostrata.
    def set_limit(which: str) -> None:
        for signal in RANGED_SIGNALS:
            xs, ys = series[signal]
            if not ys:
                continue
            recent = [y for x, y in zip(xs, ys) if x >= xs[-1] - SET_SECONDS]
            limits[signal][which] = sum(recent) / len(recent)
            print(f"Set {which} ({signal}): {limits[signal][which]:.3f} {unit_of(signal)}")
        redraw()

    def reset_limits(_event) -> None:
        for signal in limits:
            limits[signal] = { "high": None, "low": None }
        for slider in (zoom_slider, shift_slider):
            slider.reset()
        print("Range azzerato: valori assoluti, autoscala")
        redraw()

    def set_window(value: float) -> None:
        # in fondo alla corsa: tutta la storia tenuta
        view["window"] = 0.0 if value >= window_slider.valmax else value
        window_slider.valtext.set_text("tutto" if view["window"] <= 0 else f"{value:.0f} s")
        redraw()

    def set_zoom(value: float) -> None:
        view["zoom"] = 2.0**value
        zoom_slider.valtext.set_text(f"x{view['zoom']:.2g}")
        redraw()

    def set_shift(value: float) -> None:
        view["shift"] = value
        redraw()

    # tenere i riferimenti ai widget: senza, smettono di rispondere ai click
    widgets = []
    for column, (text, callback) in enumerate(
        (
            ("Set high", lambda _event: set_limit("high")),
            ("Set low", lambda _event: set_limit("low")),
            ("Reset", reset_limits),
        )
    ):
        button = Button(fig.add_axes((0.10 + 0.17 * column, 0.03, 0.15, 0.07)), text)
        button.on_clicked(callback)
        widgets.append(button)

    window_max = history if history > 0 else WINDOW_MAX_S
    window_slider = Slider(
        fig.add_axes((0.20, 0.21, 0.50, 0.03)), "finestra", 1.0, window_max, valinit=window if window > 0 else window_max
    )
    # zoom in potenze di 2: da 1/16 a 16 volte l'autoscala
    zoom_slider = Slider(fig.add_axes((0.20, 0.165, 0.50, 0.03)), "zoom y", -4.0, 4.0, valinit=0.0)
    # spostamento in mezze altezze dell'autoscala, verso l'alto se positivo
    shift_slider = Slider(fig.add_axes((0.20, 0.12, 0.50, 0.03)), "sposta y", -2.0, 2.0, valinit=0.0)
    window_slider.on_changed(set_window)
    zoom_slider.on_changed(set_zoom)
    shift_slider.on_changed(set_shift)
    widgets.extend((window_slider, zoom_slider, shift_slider))
    set_window(window_slider.val)
    zoom_slider.valtext.set_text("x1")

    radio_ax = fig.add_axes((0.79, 0.45, 0.20, 0.30))
    radio_ax.set_title("grandezza", fontsize=9)
    names = list(SIGNALS)
    radio = RadioButtons(radio_ax, names, active=names.index(initial_signal))
    radio.on_clicked(select_signal)
    widgets.append(radio)

    redraw()
    # tenere il riferimento: senza, l'animazione viene raccolta dal GC e si ferma
    anim = FuncAnimation(fig, update, interval=1000 // PLOT_FPS, cache_frame_data=False)  # noqa: F841
    plt.show()


def main() -> None:
    args = parse_args()
    if not args.dbc.is_file():
        sys.exit(f"{args.dbc} non trovato: passalo come argomento")
    db = cantools.database.load_file(args.dbc)
    bridge = db.get_message_by_name(BRIDGE_MESSAGE_NAME)
    diag = db.get_message_by_name(DIAG_MESSAGE_NAME)
    print(f"DBC: {args.dbc}")
    print(f"In ascolto di {bridge.name} (0x{bridge.frame_id:X}) e {diag.name} (0x{diag.frame_id:X})...")

    try:
        bus = open_bus(BITRATE)
    except can.CanInitializationError as error:
        sys.exit(str(error))

    stop = threading.Event()
    reader = None
    try:
        if args.no_plot:
            read_frames(bus, bridge, diag, not args.no_zero, stop, None)
        else:
            samples: deque = deque()
            reader = threading.Thread(target=read_frames, args=(bus, bridge, diag, not args.no_zero, stop, samples), daemon=True)
            reader.start()
            plot_live(samples, args.signal, args.window)  # ritorna alla chiusura della finestra
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        if reader is not None:
            reader.join(timeout=1.0)
        bus.shutdown()


if __name__ == "__main__":
    main()
