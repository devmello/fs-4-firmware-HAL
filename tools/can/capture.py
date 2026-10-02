#!/usr/bin/env python3
"""Log the VCU's two CAN buses to one CSV (plus a JSON sidecar).

    capture.py --powertrain pcan:PCAN_USBBUS1 --data pcan:PCAN_USBBUS2 \
        --firmware mbed --setup car -o captures/run.csv

Type a label and press Enter to add a MARK row. Ctrl-C stops and prints counts.
"""

import argparse
import csv
import datetime
import json
import os
import sys
import threading
import time
from collections import Counter
from pathlib import Path

import can

HEADER = ["time_s", "bus", "id", "ext", "dlc", "data"]
DEFAULT_BITRATES = {"P": 500000, "D": 1000000}
BUS_FLAGS = {"P": "powertrain", "D": "data"}
# On stop, D is read a little longer than P so the D copies of the last P
# frames (5 ms later, vcu/main.cpp:223) are still in the log.
STOP_TAIL_S = {"P": 0.0, "D": 0.1}


def parse_spec(spec):
    """'pcan:PCAN_USBBUS1' -> ('pcan', 'PCAN_USBBUS1'). Numeric channels become ints."""
    interface, sep, channel = spec.partition(":")
    if not sep or not interface or not channel:
        raise ValueError(f"bad bus spec {spec!r}, expected interface:channel")
    if channel.isdigit():
        channel = int(channel)
    return interface, channel


def bus_kwargs(spec, bitrate, listen_only):
    """Keyword arguments for can.Bus(). Returns (kwargs, warning or None)."""
    interface, channel = parse_spec(spec)
    kwargs = {"interface": interface, "channel": channel}
    warning = None
    if interface != "virtual":
        kwargs["bitrate"] = bitrate
    if listen_only:
        if interface == "pcan":
            kwargs["state"] = can.BusState.PASSIVE
        elif interface == "kvaser":
            kwargs["driver_mode"] = False  # python-can: False = silent
        elif interface == "slcan":
            kwargs["listen_only"] = True
        elif interface in ("gs_usb", "virtual"):
            pass  # gs_usb is switched after opening; virtual has no ACK
        elif interface == "socketcan":
            warning = (f"{spec}: set listen-only with `ip link ... listen-only on`;"
                       " without it the adapter will ACK")
        else:
            warning = f"{spec}: listen-only not supported here, the adapter will ACK"
    return kwargs, warning


def open_bus(spec, bitrate, listen_only=False, warnings=None):
    """warnings: list that each printed warning is added to."""
    kwargs, warning = bus_kwargs(spec, bitrate, listen_only)
    bus = can.Bus(**kwargs)
    if listen_only and kwargs["interface"] == "gs_usb":
        # python-can starts gs_usb in normal mode; restart it in listen-only.
        try:
            from gs_usb.constants import GS_CAN_MODE_LISTEN_ONLY

            bus.gs_usb.stop()
            bus.gs_usb.start(GS_CAN_MODE_LISTEN_ONLY)
        except Exception as e:  # noqa: BLE001 - any failure means we still ACK
            warning = f"{spec}: could not set listen-only ({e}), the adapter will ACK"
    if warning:
        print(f"warning: {warning}", file=sys.stderr)
        if warnings is not None:
            warnings.append(warning)
    return bus


def open_buses(args, listen_only, warnings=None):
    """Open the buses named by --powertrain/--data. Returns {'P': bus, 'D': bus}."""
    buses = {}
    try:
        for name, flag in BUS_FLAGS.items():
            spec = getattr(args, flag)
            if spec:
                buses[name] = open_bus(spec, getattr(args, flag + "_bitrate"), listen_only,
                                       warnings)
    except Exception:
        for bus in buses.values():
            bus.shutdown()
        raise
    if not buses:
        raise SystemExit("give at least one of --powertrain / --data")
    return buses


# --- CSV log ---------------------------------------------------------------

def open_log(path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    f = open(path, "w", newline="", encoding="utf-8", buffering=1)
    writer = csv.writer(f, lineterminator="\n")
    writer.writerow(HEADER)
    return {
        "path": path,
        "file": f,
        "writer": writer,
        "lock": threading.Lock(),
        "t0_ns": time.monotonic_ns(),
        "counts": Counter(),
        "marks": 0,
    }


def log_time(log, now_ns=None):
    if now_ns is None:
        now_ns = time.monotonic_ns()
    return (now_ns - log["t0_ns"]) / 1e9


def write_frame(log, bus_name, msg):
    data = bytes(msg.data[: msg.dlc]).hex().upper()
    ext = 1 if msg.is_extended_id else 0
    # Timestamp taken under the lock so rows stay in time order across threads.
    with log["lock"]:
        t = log_time(log)
        log["writer"].writerow([f"{t:.6f}", bus_name, msg.arbitration_id, ext, msg.dlc, data])
        log["counts"][(bus_name, msg.arbitration_id)] += 1


def write_mark(log, label):
    label = " ".join(str(label).split())  # no newlines or runs of spaces
    quoted = '"' + label.replace('"', '""') + '"'
    with log["lock"]:
        t = log_time(log)
        log["file"].write(f"{t:.6f},MARK,,,,{quoted}\n")
        log["marks"] += 1
    return t


def close_log(log):
    with log["lock"]:
        log["file"].close()


def sidecar_path(csv_path):
    return Path(csv_path).with_suffix(".json")


def write_sidecar(csv_path, args, extra_notes=""):
    adapter = args.adapter
    if not adapter:
        adapter = " ".join(f"{name}={getattr(args, flag)}"
                           for name, flag in BUS_FLAGS.items() if getattr(args, flag))
    notes = args.notes
    if extra_notes:
        notes = f"{notes} ({extra_notes})" if notes else extra_notes
    meta = {
        "firmware": args.firmware,
        "commit": args.commit,
        "date": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "adapter": adapter,
        "setup": args.setup,
        "notes": notes,
    }
    path = sidecar_path(csv_path)
    path.write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")
    return path


# --- threads ---------------------------------------------------------------

def read_loop(bus, bus_name, log, stop, skipped):
    tail_ns = round(STOP_TAIL_S.get(bus_name, 0) * 1e9)
    end_ns = None
    while True:
        try:
            msg = bus.recv(timeout=0.1)
        except can.CanError as e:
            print(f"warning: {bus_name}: receive error: {e}", file=sys.stderr)
            time.sleep(0.1)
            msg = None
        if stop.is_set():
            if end_ns is None:
                end_ns = time.monotonic_ns() + tail_ns
            if time.monotonic_ns() >= end_ns:
                break
        if msg is None:
            continue
        if not msg.is_rx:
            # TX echo (gs_usb does this); stimulus.py logs its own sends.
            continue
        if msg.is_error_frame or msg.is_remote_frame:
            skipped[bus_name] += 1
            continue
        write_frame(log, bus_name, msg)


def start_readers(buses, log):
    """One reader thread per bus. Returns (stop_event, threads, skipped_counter)."""
    stop = threading.Event()
    skipped = Counter()
    threads = []
    for name, bus in buses.items():
        t = threading.Thread(target=read_loop, args=(bus, name, log, stop, skipped),
                             name=f"reader-{name}", daemon=True)
        t.start()
        threads.append(t)
    return stop, threads, skipped


def stop_readers(stop, threads):
    stop.set()
    for t in threads:
        t.join(timeout=2)


def in_foreground(stream):
    """False when stream is a terminal owned by another process group. Reading
    it from a background job (&) would stop this whole process (SIGTTIN)."""
    try:
        return os.tcgetpgrp(stream.fileno()) == os.getpgrp()
    except (OSError, ValueError):
        return True  # not a terminal


def start_stdin_reader(on_line, stream=None):
    """Call on_line(text) for each line typed on stdin (daemon thread).
    Returns None, and reads nothing, when stdin is closed or not ours."""
    stream = stream or sys.stdin
    if stream is None or not in_foreground(stream):
        return None

    def loop():
        for line in stream:
            on_line(line.strip())

    t = threading.Thread(target=loop, name="stdin", daemon=True)
    t.start()
    return t


def print_counts(log, duration_s, skipped=None, out=None):
    out = out or sys.stdout
    print(f"\n{log['path']}: {duration_s:.1f} s, {log['marks']} marks", file=out)
    by_bus = {}
    for (bus, frame_id), n in log["counts"].items():
        by_bus.setdefault(bus, []).append((frame_id, n))
    for bus in sorted(by_bus):
        rows = sorted(by_bus[bus])
        total = sum(n for _, n in rows)
        print(f"  {bus}: {total} frames", file=out)
        for frame_id, n in rows:
            rate = n / duration_s if duration_s > 0 else 0.0
            print(f"    {frame_id:>5} (0x{frame_id:03X})  {n:>8}  {rate:7.1f} Hz", file=out)
    if not by_bus:
        print("  no frames received", file=out)
    for bus, n in sorted((skipped or {}).items()):
        print(f"  {bus}: {n} error/remote frames not logged", file=out)


# --- CLI -------------------------------------------------------------------

def add_bus_args(parser):
    parser.add_argument("--powertrain", metavar="SPEC",
                        help="powertrain bus (CAN1), e.g. pcan:PCAN_USBBUS1, virtual:p")
    parser.add_argument("--data", metavar="SPEC",
                        help="data bus (CAN2), e.g. pcan:PCAN_USBBUS2, virtual:d")
    parser.add_argument("--powertrain-bitrate", type=int, default=DEFAULT_BITRATES["P"])
    parser.add_argument("--data-bitrate", type=int, default=DEFAULT_BITRATES["D"])


def add_meta_args(parser):
    parser.add_argument("--firmware", choices=["mbed", "hal"], required=True)
    parser.add_argument("--commit", default="", help="firmware commit on the VCU")
    parser.add_argument("--setup", choices=["bench", "car"], default="bench")
    parser.add_argument("--adapter", default="",
                        help="adapter description for the sidecar (default: the bus specs)")
    parser.add_argument("--notes", default="")


def default_out(args):
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    return Path("captures") / f"{args.firmware}-{args.setup}-{stamp}.csv"


def main(argv=None, stdin=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    add_bus_args(p)
    add_meta_args(p)
    p.add_argument("-o", "--out", help="CSV path (default captures/<firmware>-<setup>-<time>.csv)")
    p.add_argument("--ack", action="store_true",
                   help="let the adapter ACK (bench with no other node); default listen-only")
    p.add_argument("--duration", type=float, help="stop after this many seconds")
    args = p.parse_args(argv)

    out = Path(args.out) if args.out else default_out(args)
    warnings = []
    buses = open_buses(args, listen_only=not args.ack, warnings=warnings)
    log = open_log(out)
    write_sidecar(out, args)
    stop, threads, skipped = start_readers(buses, log)

    def on_line(text):
        if text:
            t = write_mark(log, text)
            print(f"[{t:9.3f}] MARK {text}")

    reader = start_stdin_reader(on_line, stdin)
    mode = "ACK on" if args.ack else "may ACK, see warning" if warnings else "listen-only"
    print(f"logging {', '.join(f'{n}={getattr(args, BUS_FLAGS[n])}' for n in buses)} "
          f"to {out} ({mode})")
    print("type a label + Enter to mark, Ctrl-C to stop" if reader
          else "stdin is closed or this is a background job, so no typed marks")

    t_start = time.monotonic()
    try:
        while args.duration is None or time.monotonic() - t_start < args.duration:
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        stop_readers(stop, threads)
        for bus in buses.values():
            bus.shutdown()
        duration = log_time(log)
        close_log(log)
        print_counts(log, duration, skipped)
    return 0


if __name__ == "__main__":
    sys.exit(main())
