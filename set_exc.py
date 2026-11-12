#!/usr/bin/env python3
"""Cambia l'eccitazione del ponte (ACQUISINATOR_SET_EXC, 0x7A2) e, con --sweep, misura il rumore
a ogni frequenza.

Uso:
  python set_exc.py --freq 175 [--amp 300] [--dc 550]       una sola impostazione (mV)
  python set_exc.py --sweep 25:1000:10 --dwell 5 [--csv sweep.csv] [--amp ...] [--dc ...]
  python set_exc.py --sweep 125,175,333,777 --dwell 10

Il firmware calcola da solo campioni per periodo, periodo del timer e finestra; le frequenze
non realizzabili (es. vicino a un'armonica di rete) vengono rifiutate e nello sweep saltate.
Ogni impostazione applicata ha un id nuovo, riportato su ACQUISINATOR_EXC (0x7A3) e in
ACQUISINATOR_DIAG.exc_id: nello sweep contano solo le misure prese con l'impostazione giusta.

Per ogni frequenza dello sweep: media e deviazione standard dell'uscita del ponte in mV/V e la
deviazione standard in microstrain (quarto di ponte, gauge factor 2.01), numero di misure e flag
visti. La deviazione standard e' il rumore: a ruote ferme e motore acceso si vede a quali
frequenze l'inverter viene rifiutato. Un'interferenza che cade esattamente sulla frequenza di
eccitazione da' invece un offset costante, non rumore: va guardata anche la media.
"""

import argparse
import csv
import math
import sys
import time

import can
import cantools

from gs_usb_bus import open_bus

BITRATE = 1_000_000
CONFIRM_TIMEOUT_S = 3.0
SETTLE_S = 0.5  # dopo il cambio, oltre alla prima finestra gia' scartata dal firmware
STRAIN_GAUGE_K = 2.01

db = cantools.database.load_file("acquisinator.dbc")
SET_EXC = db.get_message_by_name("ACQUISINATOR_SET_EXC")
EXC = db.get_message_by_name("ACQUISINATOR_EXC")
DIAG = db.get_message_by_name("ACQUISINATOR_DIAG")
BRIDGE = db.get_message_by_name("ACQUISINATOR_BRIDGE")


def parse_sweep(text: str) -> list[int]:
    if ":" in text:
        start, stop, step = (int(v) for v in text.split(":"))
        return list(range(start, stop + 1, step))
    return [int(v) for v in text.split(",")]


def wait_exc(bus: can.BusABC, timeout: float) -> dict | None:
    """Prossimo ACQUISINATOR_EXC, o None."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        frame = bus.recv(timeout=0.1)
        if frame is not None and frame.arbitration_id == EXC.frame_id:
            return EXC.decode(frame.data)
    return None


def apply(bus: can.BusABC, freq: int, amp: int, dc: int) -> tuple[bool, dict | None]:
    """Invia il comando: (applicato, impostazione in uso)."""
    current = wait_exc(bus, 1.5)  # id di partenza (lo stato arriva ogni secondo)
    bus.send(can.Message(arbitration_id=SET_EXC.frame_id, data=SET_EXC.encode({"freq": freq, "amp": amp, "dc": dc}),
                         is_extended_id=False))
    deadline = time.monotonic() + CONFIRM_TIMEOUT_S
    while time.monotonic() < deadline:
        exc = wait_exc(bus, deadline - time.monotonic())
        if exc is None:
            break
        if current is None or exc["id"] != current["id"]:
            return True, exc
        # stesso id: rifiutato (risposta immediata) oppure stato periodico arrivato prima;
        # un comando accettato si applica entro un periodo, al massimo 1 s (a 1 Hz)
        if wait_exc_changed(bus, exc["id"], 1.1) is None:
            return False, exc
    return False, current


def wait_exc_changed(bus: can.BusABC, old_id: int, timeout: float) -> dict | None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        exc = wait_exc(bus, deadline - time.monotonic())
        if exc is not None and exc["id"] != old_id:
            return exc
    return None


def describe(exc: dict) -> str:
    window_ms = 1000.0 * exc["periods"] / exc["freq"]
    return (f"{exc['freq']:.1f} Hz, {exc['samples']} campioni/periodo (tick {exc['freq'] * exc['samples'] / 1000:.1f} kHz), "
            f"finestra {exc['periods']} periodi = {window_ms:.1f} ms, {exc['amp']} mV su {exc['dc']} mV, id {exc['id']}")


def collect(bus: can.BusABC, exc_id: int, seconds: float) -> dict:
    """Misure prese con l'impostazione `exc_id` per `seconds` secondi."""
    values: list[float] = []
    flags_seen = 0
    last_diag_id = None
    deadline = time.monotonic() + SETTLE_S
    while time.monotonic() < deadline:
        bus.recv(timeout=0.05)
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        frame = bus.recv(timeout=0.1)
        if frame is None:
            continue
        if frame.arbitration_id == DIAG.frame_id:
            last_diag_id = int(DIAG.decode(frame.data)["exc_id"])
        elif frame.arbitration_id == BRIDGE.frame_id and last_diag_id == exc_id:
            data = BRIDGE.decode(frame.data)
            flags_seen |= int(data["flags"])
            if not math.isnan(data["bridge_out"]):
                values.append(data["bridge_out"])
            last_diag_id = None  # una BRIDGE per ogni DIAG
    n = len(values)
    mean = sum(values) / n if n else math.nan
    std = math.sqrt(sum((v - mean) ** 2 for v in values) / (n - 1)) if n > 1 else math.nan
    return {"n": n, "mean_mv_per_v": mean, "std_uv_per_v": std * 1000.0,
            "std_ue": 4.0 * std / 1000.0 / STRAIN_GAUGE_K * 1e6, "flags": flags_seen}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument("--freq", type=int, help="frequenza [Hz], 0 = invariata (per cambiare solo i livelli)")
    target.add_argument("--sweep", type=parse_sweep, help="start:stop:step oppure f1,f2,... [Hz]")
    parser.add_argument("--amp", type=int, default=0, help="ampiezza di picco [mV], 0 = invariata")
    parser.add_argument("--dc", type=int, default=0, help="livello DC [mV], 0 = invariato")
    parser.add_argument("--dwell", type=float, default=5.0, help="secondi di misura per frequenza nello sweep")
    parser.add_argument("--csv", default="sweep.csv", help="file dei risultati dello sweep")
    args = parser.parse_args()

    bus = open_bus(bitrate=BITRATE)
    try:
        if args.sweep is None:
            ok, exc = apply(bus, args.freq, args.amp, args.dc)
            if exc is None:
                sys.exit("Nessun ACQUISINATOR_EXC ricevuto: la scheda e' accesa e con il firmware nuovo?")
            print(("Applicata: " if ok else "Rifiutata, resta: ") + describe(exc))
            return

        with open(args.csv, "w", newline="") as out:
            writer = csv.writer(out)
            writer.writerow(["freq_req_hz", "freq_hz", "samples", "periods", "window_ms", "amp_mv", "dc_mv",
                             "n", "mean_mv_per_v", "std_uv_per_v", "std_ue", "flags"])
            for freq in args.sweep:
                ok, exc = apply(bus, freq, args.amp, args.dc)
                if not ok or exc is None:
                    print(f"{freq:5d} Hz: rifiutata, saltata")
                    continue
                stats = collect(bus, exc["id"], args.dwell)
                window_ms = 1000.0 * exc["periods"] / exc["freq"]
                print(f"{freq:5d} Hz: rumore {stats['std_ue']:8.2f} µε ({stats['std_uv_per_v']:7.3f} µV/V), "
                      f"{stats['n']:4d} misure, finestra {window_ms:5.1f} ms, flag 0x{stats['flags']:02X}")
                writer.writerow([freq, f"{exc['freq']:.1f}", exc["samples"], exc["periods"], f"{window_ms:.2f}",
                                 exc["amp"], exc["dc"], stats["n"], f"{stats['mean_mv_per_v']:.6f}",
                                 f"{stats['std_uv_per_v']:.4f}", f"{stats['std_ue']:.3f}", stats["flags"]])
                out.flush()
        print(f"Risultati in {args.csv}")
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
