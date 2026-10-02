#!/usr/bin/env python3
"""Play a scenario of periodic CAN frames to stand in for the BMS, motor
controller, wheel boards and steering wheel on a VCU bench.

    stimulus.py scenarios/baseline.yaml --powertrain pcan:PCAN_USBBUS1 \
        --data pcan:PCAN_USBBUS2 --log captures/run.csv --firmware mbed

With --log it also records both buses (like capture.py) on the same adapter,
logs the frames it sends, and writes a MARK row as each step starts.
Steps marked `wait: true` run until Enter is pressed; --no-wait uses their
duration_s instead. A typed label + Enter adds a MARK row.
"""

import argparse
import re
import sys
import threading
import time
from pathlib import Path

import can
import cantools
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import capture  # noqa: E402

DEFAULT_DBC = Path.home() / "Projects" / "fs-4-ref" / "CANbus.dbc"
END_TAIL_S = 0.5  # --log keeps recording this long after the "end" mark


def load_dbc(path):
    """Load the DBC. The fs-4 DBC lists the VDM frames (0xA0000..) without the
    extended-id flag bit, which cantools rejects, so set the bit here."""
    text = Path(path).read_text(encoding="utf-8", errors="replace")

    def fix(m):
        frame_id = int(m.group(2))
        if 0x7FF < frame_id < 0x80000000:
            frame_id |= 0x80000000
        return f"{m.group(1)}{frame_id} "

    text = re.sub(r"^(BO_ )(\d+) ", fix, text, flags=re.M)
    return cantools.database.load_string(text, database_format="dbc", strict=False)


def load_scenario(path):
    with open(path, encoding="utf-8") as f:
        scenario = yaml.safe_load(f)
    if not isinstance(scenario, dict) or "streams" not in scenario or "steps" not in scenario:
        raise ValueError(f"{path}: needs 'streams' and 'steps'")
    return scenario


# --- streams ---------------------------------------------------------------

def make_streams(scenario, db):
    """Turn the scenario's stream table into a list of dicts with encoders."""
    streams = []
    for name, spec in scenario["streams"].items():
        bus = spec["bus"]
        if bus not in ("P", "D"):
            raise ValueError(f"stream {name}: bus must be P or D")
        s = {
            "name": name,
            "bus": bus,
            "period_ns": round(float(spec["period_ms"]) * 1e6),
            "offset_ns": round(float(spec.get("offset_ms", 0)) * 1e6),
        }
        if s["period_ns"] <= 0:
            raise ValueError(f"stream {name}: period_ms must be > 0")
        if "id" in spec:
            s["id"] = int(spec["id"])
            s["ext"] = bool(spec.get("ext", s["id"] > 0x7FF))
            s["data"] = bytes.fromhex(str(spec.get("data", "")))
        else:
            if db is None:
                raise ValueError(f"stream {name}: needs a DBC or an explicit id")
            msg = db.get_message_by_name(spec.get("message", name))
            s["dbc"] = msg
            s["id"] = msg.frame_id
            s["ext"] = msg.is_extended_frame
            # Signals left out are sent as raw 0.
            s["values"] = {sig.name: sig.offset for sig in msg.signals}
            set_values(s, spec.get("signals", {}))
        streams.append(s)
    return streams


def set_values(stream, values):
    if "dbc" not in stream:
        unknown = set(values) - {"data"}
        if unknown:
            raise ValueError(f"stream {stream['name']}: raw stream only takes 'data'")
        if "data" in values:
            stream["data"] = bytes.fromhex(str(values["data"]))
        return
    unknown = set(values) - set(stream["values"])
    if unknown:
        raise ValueError(f"stream {stream['name']}: unknown signals {sorted(unknown)}")
    stream["values"].update(values)
    stream["data"] = stream["dbc"].encode(stream["values"], strict=False)


def apply_step(streams, step):
    by_name = {s["name"]: s for s in streams}
    for name, values in (step.get("set") or {}).items():
        if name not in by_name:
            raise ValueError(f"step {step.get('label')!r}: unknown stream {name!r}")
        set_values(by_name[name], values or {})


def check_scenario(scenario, db):
    """Encode every step once so mistakes show up before anything is sent."""
    streams = make_streams(scenario, db)
    for step in scenario["steps"]:
        if not step.get("label"):
            raise ValueError("every step needs a label")
        if float(step.get("duration_s", 0)) <= 0:
            raise ValueError(f"step {step['label']!r}: duration_s must be > 0")
        apply_step(streams, step)
    return streams


def frame(stream):
    return can.Message(arbitration_id=stream["id"], is_extended_id=stream["ext"],
                       data=stream["data"], dlc=len(stream["data"]))


# --- player ----------------------------------------------------------------

def sleep_until(t_ns):
    # macOS stretches sleeps by up to ~25 % (timer coalescing), so sleep half
    # the remaining time at a time and spin for the last half millisecond.
    while True:
        left = t_ns - time.monotonic_ns()
        if left <= 0:
            return
        time.sleep(left / 2e9 if left > 500_000 else 0)


def play(scenario, buses, db=None, log=None, log_tx=(), enter=None, stop=None,
         stats=None, out=print):
    """Play the steps. Counts go into stats {'sent': n, 'errors': n}, plus the
    monotonic start time t_start_ns that the frame schedule is based on.

    log_tx: names of the buses whose sent frames are written to log.
    enter: threading.Event set by the operator's Enter; when given, steps with
    wait: true end on it instead of their duration.
    """
    all_streams = make_streams(scenario, db)
    streams = [s for s in all_streams if s["bus"] in buses]
    for bus in sorted({s["bus"] for s in all_streams} - set(buses)):
        out(f"note: bus {bus} not given, its streams are skipped")
    if stats is None:
        stats = {"sent": 0, "errors": 0}

    def mark(label, t_ns):
        if log is not None:
            t = capture.write_mark(log, label)
        else:
            t = (t_ns - t_start) / 1e9
        return t

    t_start = time.monotonic_ns()
    stats["t_start_ns"] = t_start
    for s in streams:
        s["k"] = 0
        s["next_ns"] = t_start + s["offset_ns"]

    step_start = t_start
    for step in scenario["steps"]:
        label = step["label"]
        waiting = enter is not None and step.get("wait", False)
        apply_step(all_streams, step)
        sleep_until(step_start)
        t = mark(label, step_start)
        line = f"[{t:9.3f}] {label}"
        if step.get("operator"):
            line += f"\n            operator: {step['operator']}"
        if waiting:
            enter.clear()
            line += "\n            press Enter when done"
        out(line)

        end = None if waiting else step_start + round(float(step["duration_s"]) * 1e9)
        while True:
            if stop is not None and stop.is_set():
                mark("stimulus aborted", time.monotonic_ns())
                return stats
            due = min((s["next_ns"] for s in streams), default=None)
            if waiting:
                if enter.is_set():
                    end = time.monotonic_ns()
                    break
                if due is None or due > time.monotonic_ns() + 50_000_000:
                    enter.wait(0.05)
                    continue
            elif due is None or due >= end:
                sleep_until(end)
                break
            sleep_until(due)
            for s in streams:
                if s["next_ns"] <= due:
                    send(buses[s["bus"]], s["bus"], frame(s),
                         log if s["bus"] in log_tx else None, stats)
                    s["k"] += 1
                    s["next_ns"] = t_start + s["offset_ns"] + s["k"] * s["period_ns"]
        step_start = end

    sleep_until(step_start)
    mark("end", step_start)
    return stats


def send(bus, bus_name, msg, log, stats):
    try:
        bus.send(msg, timeout=0.05)
    except can.CanError as e:
        stats["errors"] += 1
        if stats["errors"] <= 5:
            print(f"warning: {bus_name}: send 0x{msg.arbitration_id:X} failed: {e}",
                  file=sys.stderr)
        return
    stats["sent"] += 1
    if log is not None:
        capture.write_frame(log, bus_name, msg)


# --- CLI -------------------------------------------------------------------

def main(argv=None, stdin=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("scenario", help="scenario YAML")
    capture.add_bus_args(p)
    p.add_argument("--dbc", default=None, help=f"DBC file (default {DEFAULT_DBC})")
    p.add_argument("--no-wait", action="store_true",
                   help="do not wait for Enter on operator steps; use duration_s")
    p.add_argument("--check", action="store_true",
                   help="only load and encode the scenario, send nothing")
    p.add_argument("--log", metavar="CSV", help="also record both buses to this CSV")
    p.add_argument("--firmware", choices=["mbed", "hal"], help="for the --log sidecar")
    p.add_argument("--commit", default="")
    p.add_argument("--setup", choices=["bench", "car"], default="bench")
    p.add_argument("--adapter", default="")
    p.add_argument("--notes", default="")
    args = p.parse_args(argv)

    scenario = load_scenario(args.scenario)
    needs_dbc = any("id" not in s for s in scenario["streams"].values())
    db = load_dbc(args.dbc or scenario.get("dbc") or DEFAULT_DBC) if needs_dbc else None
    check_scenario(scenario, db)
    if args.check:
        total = sum(float(s["duration_s"]) for s in scenario["steps"])
        print(f"{args.scenario}: {len(scenario['streams'])} streams, "
              f"{len(scenario['steps'])} steps, {total:.0f} s without waits")
        return 0
    if args.log and not args.firmware:
        p.error("--log needs --firmware")
    stream = stdin or sys.stdin
    if not args.no_wait and any(s.get("wait") for s in scenario["steps"]) and not (
            stream is not None and stream.isatty() and capture.in_foreground(stream)):
        p.error("operator steps wait for Enter, which needs stdin on a terminal; add --no-wait")

    buses = capture.open_buses(args, listen_only=False)
    log = None
    readers = None
    log_tx = ()
    if args.log:
        log = capture.open_log(args.log)
        capture.write_sidecar(args.log, args, f"stimulus {Path(args.scenario).name}")
        readers = capture.start_readers(buses, log)
        # udp_multicast hands sent frames back to our own reader, which logs them.
        log_tx = [n for n in buses if capture.parse_spec(
            getattr(args, capture.BUS_FLAGS[n]))[0] != "udp_multicast"]

    enter = None if args.no_wait else threading.Event()

    def on_line(text):
        if text and log is not None:
            t = capture.write_mark(log, text)
            print(f"[{t:9.3f}] MARK {text}")
        elif not text and enter is not None:
            enter.set()

    capture.start_stdin_reader(on_line, stream)
    stats = {"sent": 0, "errors": 0}
    t0 = time.monotonic()
    try:
        play(scenario, buses, db, log=log, log_tx=log_tx, enter=enter, stats=stats)
        if readers:
            # Keep the VCU's replies to the last frames (D copies come 5 ms
            # after P) and give the "end" segment some length.
            time.sleep(END_TAIL_S)
    except KeyboardInterrupt:
        if log is not None:
            capture.write_mark(log, "stimulus aborted")
        print("\nstopped")
    finally:
        if readers:
            capture.stop_readers(readers[0], readers[1])
        for bus in buses.values():
            bus.shutdown()
        if log is not None:
            duration = capture.log_time(log)
            capture.close_log(log)
            capture.print_counts(log, duration, readers[2])
    print(f"sent {stats['sent']} frames in {time.monotonic() - t0:.1f} s, "
          f"{stats['errors']} send errors")
    return 0


if __name__ == "__main__":
    sys.exit(main())
