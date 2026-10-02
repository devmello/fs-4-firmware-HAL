#!/usr/bin/env python3
"""Compare a CAN capture of the Mbed VCU firmware (A) with one of the HAL port (B).

    compare.py A.csv B.csv [--dbc PATH] [--config tol.toml] [--tol NAME=VALUE ...]
                           [--timing KEY=VALUE ...] [--json] [--full]

A is the reference (normally Mbed), B the build under test (normally HAL).
Exit status: 0 = B matches A within tolerance, 1 = differences, 2 = bad input.

Expected traffic, from fs-4 vcu/main.cpp (c12c834d):
  P: 390, 646 every 40 ms (send_sme_CAN_messages_powertrain)
  D: 390, 646 copied 5 ms after the P frames (sleep_for(5ms) in the same job),
     plus 390, 660, 646 every 80 ms (send_sme_CAN_messages_data); both jobs
     run on etc_queue, so 390/646 on D interleave two schedules.
  D: 402, 403 every 50 ms; 720, 976, 721, 977, 722 every 10 ms (IMU).
  D: 1154 and 1666 forwarded from P by the main loop.

Signal tolerances: flags, counters and enums must match (min/max). Analog
signals may differ by their tolerance (default one DBC scale step) or by
range_frac of A's range, whichever is larger. Means are only checked when a
mean tolerance is set.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import difflib
import functools
import json
import math
import os
import re
import sys
import tomllib
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path

import cantools

DEFAULT_DBC = Path.home() / "Projects/fs-4-ref/CANbus.dbc"


@dataclass(frozen=True)
class Sched:
    period_ms: float
    dual: bool = False  # two jobs send this id on this bus
    note: str = ""


# (bus, id) -> expected schedule; ids not listed use A's median interval.
SCHEDULE = {
    ("P", 390): Sched(40),
    ("P", 646): Sched(40),
    ("D", 390): Sched(40, True, "40 ms copy + 80 ms data job"),
    ("D", 646): Sched(40, True, "40 ms copy + 80 ms data job"),
    ("D", 660): Sched(80),
    ("D", 402): Sched(50),
    ("D", 403): Sched(50),
    ("D", 720): Sched(10),
    ("D", 721): Sched(10),
    ("D", 722): Sched(10),
    ("D", 976): Sched(10),
    ("D", 977): Sched(10),
}
COPIED_IDS = (390, 646)  # P frame copied to D by the same job
FORWARDED_IDS = (1154, 1666)  # forwarded P -> D by the main loop
ALIVE_ID = 390  # MBB_Alive: byte 5, low nibble
COPY_DELAY_S = 0.005  # sleep_for(5ms) between the P and D writes
COPY_WINDOW_S = 0.030
FORWARD_WINDOW_S = 0.050
HIST_BIN_MS = 5.0
DROP_FACTOR = 1.5  # one lost frame doubles the gap, so flag anything over 1.5x the period
LOST_FACTOR = 1.75  # a gap this long or longer is called dropped frames, shorter a late frame
MIN_INTERVALS = 50  # fewer and the median / p5 / p95 move with jitter alone

DEFAULT_TIMING = {
    "period_ms": 1.0,  # max difference of median interval
    "jitter_ms": 5.0,  # max difference of p5 / p95 interval
    "copy_delay_ms": 1.0,  # max difference of median P->D copy delay
    "forward_ms": 2.0,  # max increase of median forwarding latency
    "count_frac": 0.02,  # max relative frame count difference (after scaling by duration)
    "hist_frac": 0.10,  # max per-bin difference of interval histograms (dual schedules)
    "unmatched_frac": 0.01,  # max increase of the fraction of uncopied / unforwarded frames
    "skew_ms": 3.0,  # how far before its P frame a D copy may be stamped (bus timestamp skew)
    "min_segment_s": 0.5,  # MARK segments shorter than this are listed but not compared
}


class InputError(Exception):
    pass


# ---------------------------------------------------------------- input


@dataclass
class Frame:
    t: float
    bus: str
    id: int
    ext: int
    dlc: int
    data: bytes


@dataclass
class Log:
    path: Path
    frames: list[Frame]
    marks: list[tuple[float, str]]
    meta: dict
    warnings: list[str]
    duration: float
    d_end: float  # time of the last D frame
    index: dict  # (bus, id) -> (times, frames)


def index_frames(frames):
    index = {}
    for f in frames:
        times, fr = index.setdefault((f.bus, f.id), ([], []))
        times.append(f.t)
        fr.append(f)
    return index


def frames_in(log, key, t0, t1):
    times, frames = log.index.get(key, ([], []))
    return frames[bisect.bisect_left(times, t0):bisect.bisect_left(times, t1)]


def load_log(path) -> Log:
    path = Path(path)
    frames, marks, warnings = [], [], []
    last_t = None
    end_t = 0.0
    try:
        fh = open(path, newline="", encoding="utf-8")
    except OSError as e:
        raise InputError(f"{path}: {e}") from e
    with fh:
        reader = csv.DictReader(fh)
        need = {"time_s", "bus", "id", "ext", "dlc", "data"}
        if not reader.fieldnames or need - set(reader.fieldnames):
            raise InputError(f"{path}: header must contain {','.join(sorted(need))}")
        for line, row in enumerate(reader, start=2):
            try:
                t = float(row["time_s"])
            except (TypeError, ValueError):
                warnings.append(f"line {line}: bad time_s {row['time_s']!r}, skipped")
                continue
            if last_t is not None and t < last_t:
                warnings.append(f"line {line}: time goes backwards ({t:.6f} < {last_t:.6f})")
            last_t = t
            end_t = max(end_t, t)
            bus = (row["bus"] or "").strip()
            if bus == "MARK":
                marks.append((t, row["data"] or ""))
                continue
            if bus not in ("P", "D"):
                warnings.append(f"line {line}: unknown bus {bus!r}, skipped")
                continue
            try:
                fid = int(row["id"])
                ext = int(row["ext"] or 0)
                dlc = int(row["dlc"])
                data = bytes.fromhex(row["data"] or "")
            except (TypeError, ValueError):
                warnings.append(f"line {line}: unparsable frame, skipped")
                continue
            if len(data) != dlc:
                warnings.append(f"line {line}: dlc {dlc} but {len(data)} data bytes")
            frames.append(Frame(t, bus, fid, ext, dlc, data))
    meta = {}
    side = path.with_suffix(".json")
    if side.exists():
        try:
            meta = json.loads(side.read_text(encoding="utf-8"))
        except (OSError, ValueError) as e:
            warnings.append(f"{side.name}: unreadable ({e})")
    if not frames:
        warnings.append("no frames")
    d_end = max((f.t for f in frames if f.bus == "D"), default=0.0)
    return Log(path, frames, marks, meta, warnings, end_t, d_end, index_frames(frames))


def load_dbc(path):
    """Load the DBC. Ids above 0x7FF without the extended flag (the VDM_* messages in
    CANbus.dbc) make cantools refuse the file, so mark them extended first."""
    try:
        text = Path(path).read_text(encoding="utf-8", errors="replace")
    except OSError as e:
        raise InputError(f"DBC {path}: {e}") from e
    fixes = []
    for raw in re.findall(r"^BO_ (\d+) ", text, re.M):
        fid = int(raw)
        if 0x7FF < fid < 0x80000000:
            fixed = fid | 0x80000000
            text = re.sub(
                rf"(?m)^((?:BO_|CM_|BA_|VAL_|SIG_VALTYPE_|BO_TX_BU_)\b.*?)\b{fid}\b",
                lambda m, s=str(fixed): m.group(1) + s,
                text,
            )
            fixes.append(fid)
    try:
        db = cantools.database.load_string(text, "dbc", strict=False)
    except Exception as e:  # cantools raises several error types
        raise InputError(f"DBC {path}: {e}") from e
    if fixes:
        fixes = [f"DBC ids {', '.join(map(str, fixes))} are over 0x7FF without the extended flag;"
                 " loaded as extended"]
    return db, fixes


# ---------------------------------------------------------------- config


@dataclass
class Config:
    timing: dict
    default_analog: float | None = None  # None: one DBC scale step
    range_frac: float = 0.02  # analog min/max tolerance is at least this share of A's range
    signals: dict = field(default_factory=dict)  # name -> (tol, mean_tol or None)


def tolerance(cfg, sig, kind, a):
    """Return (min/max tolerance, mean tolerance or None if the mean is not checked).
    a is A's stats for the signal. Analog peaks move with the sampling phase, so their
    tolerance is at least range_frac of A's range. Means depend on how the operator
    moved things, so they are only checked when set."""
    tol, mean_tol = cfg.signals.get(sig.name, (None, None))
    if kind != "analog":
        return tol or 0.0, mean_tol
    if tol is None:
        tol = abs(sig.scale) if cfg.default_analog is None else cfg.default_analog
    if a:
        tol = max(tol, cfg.range_frac * (a["max"] - a["min"]))
    return tol, mean_tol


def to_float(name, value):
    if isinstance(value, bool):
        raise InputError(f"tolerance {name}: expected a number, got {value}")
    try:
        return float(value)
    except (TypeError, ValueError):
        raise InputError(f"tolerance {name}: expected a number, got {value!r}") from None


def parse_tol_value(name, value):
    """A number sets the min/max tolerance. { tol = X, mean = Y } also checks the mean;
    mean = false (or no mean) leaves it unchecked."""
    if isinstance(value, dict):
        unknown = set(value) - {"tol", "mean"}
        if unknown:
            raise InputError(f"tolerance {name}: unknown keys {sorted(unknown)}")
        tol = to_float(name, value.get("tol", 0.0))
        mean = value.get("mean", False)
        return tol, None if mean is False else to_float(name, mean)
    return to_float(name, value), None


def set_signal_tol(cfg, name, value):
    if name == "default_analog":
        cfg.default_analog = to_float(name, value)
    elif name == "range_frac":
        cfg.range_frac = to_float(name, value)
    else:
        cfg.signals[name] = parse_tol_value(name, value)


def load_config(path, tol_args, timing_args) -> Config:
    cfg = Config(dict(DEFAULT_TIMING))
    if path:
        try:
            data = tomllib.loads(Path(path).read_text(encoding="utf-8"))
        except (OSError, tomllib.TOMLDecodeError) as e:
            raise InputError(f"config {path}: {e}") from e
        for k, v in data.get("timing", {}).items():
            if k not in DEFAULT_TIMING:
                raise InputError(f"config {path}: unknown timing key {k}")
            cfg.timing[k] = float(v)
        for k, v in data.get("signals", {}).items():
            set_signal_tol(cfg, k, v)
    for arg in tol_args or []:
        name, _, val = arg.partition("=")
        if not val:
            raise InputError(f"--tol {arg}: expected NAME=VALUE")
        set_signal_tol(cfg, name, val)
    for arg in timing_args or []:
        name, _, val = arg.partition("=")
        if name not in DEFAULT_TIMING or not val:
            raise InputError(f"--timing {arg}: expected one of {', '.join(DEFAULT_TIMING)}=VALUE")
        cfg.timing[name] = float(val)
    return cfg


# ---------------------------------------------------------------- helpers


def percentile(sorted_vals, p):
    if not sorted_vals:
        return None
    k = (len(sorted_vals) - 1) * p / 100.0
    lo, hi = math.floor(k), math.ceil(k)
    return sorted_vals[lo] + (sorted_vals[hi] - sorted_vals[lo]) * (k - lo)


def spread(values):
    """median / p5 / p95 / max of a list (ms)."""
    if not values:
        return None
    s = sorted(values)
    return {"n": len(s), "median": percentile(s, 50), "p5": percentile(s, 5),
            "p95": percentile(s, 95), "max": s[-1]}


def signal_kind(sig):
    if sig.length == 1:
        return "flag"
    if re.search(r"alive|counter|cnt", sig.name, re.I):
        return "counter"
    if sig.scale == 1 and sig.offset == 0 and not sig.unit and not sig.is_float:
        return "enum"
    return "analog"


def fmt(v, nd=2):
    if v is None:
        return "-"
    if isinstance(v, float) and not v.is_integer():
        return f"{v:.{nd}f}"
    return str(int(v)) if isinstance(v, float) else str(v)


def fmt_sig(v):
    if v is None:
        return "-"
    return f"{v:.6g}"


EPS = 1e-9


# ---------------------------------------------------------------- analysis


def message_for(db, fid, ext):
    try:
        return db.get_message_by_frame_id(fid | (0x80000000 if ext else 0))
    except KeyError:
        return None


@functools.lru_cache(maxsize=1 << 16)
def decode(msg, data):
    try:
        return msg.decode(data, decode_choices=False, scaling=True,
                          allow_truncated=True, allow_excess=True)
    except Exception as e:  # report, don't crash
        return e


@dataclass
class Window:
    name: str
    a: tuple[float, float]
    b: tuple[float, float]


def half_period_s(times):
    """Half the median interval of a sorted list of times (inf with under 2 frames)."""
    if len(times) < 2:
        return math.inf
    return percentile(sorted(t1 - t0 for t0, t1 in zip(times, times[1:])), 50) / 2


def match_copies(src, dst_times, dst_frames, before_s, after_s, expect_s, end_t):
    """Pair each src frame with an unused dst frame with the same payload in
    [t - before, t + after], taking the one nearest t + expect. Frames whose search
    window runs past end_t (the last D frame in the log) are left out, since their
    copy may not have been logged. Returns (delays_ms, unmatched, frames checked)."""
    used = set()
    delays, unmatched, n = [], 0, 0
    for f in src:
        if f.t + after_s > end_t:
            continue
        n += 1
        best = None
        for i in range(bisect.bisect_left(dst_times, f.t - before_s),
                       bisect.bisect_right(dst_times, f.t + after_s)):
            if i in used or dst_frames[i].data != f.data:
                continue
            if best is None or (abs(dst_times[i] - f.t - expect_s)
                                < abs(dst_times[best] - f.t - expect_s)):
                best = i
        if best is None:
            unmatched += 1
        else:
            used.add(best)
            delays.append((dst_times[best] - f.t) * 1000.0)
    return delays, unmatched, n


def alive_check(frames, bus):
    vals = [f.data[5] & 0x0F for f in frames if len(f.data) > 5]
    res = {"n": len(vals), "repeats": 0, "gaps": 0, "missing": 0, "at": []}
    times = [f.t for f in frames if len(f.data) > 5]
    for i in range(1, len(vals)):
        d = (vals[i] - vals[i - 1]) % 16
        bad = False
        if d == 0:
            res["repeats"] += 1
            bad = bus == "P"  # on D the 80 ms job repeats the current value
        elif d > 1:
            res["gaps"] += 1
            res["missing"] += d - 1
            bad = True
        if bad and len(res["at"]) < 5:
            res["at"].append(round(times[i], 6))
    return res


def analyse_key(key, fa, fb, log_a, log_b, win, db, cfg):
    """Compare one (bus, id) within one window. fa/fb are the frames in the window."""
    bus, fid = key
    tm = cfg.timing
    r = {"bus": bus, "id": fid, "name": None, "count": [len(fa), len(fb)],
         "presence": "both" if fa and fb else ("A only" if fa else "B only"),
         "failures": [], "notes": []}
    fail = r["failures"].append
    note = r["notes"].append
    ext = (fa or fb)[0].ext
    msg = message_for(db, fid, ext)
    if msg is not None:
        r["name"] = msg.name
    sched = SCHEDULE.get(key)
    if sched and sched.note:
        note(sched.note)
    dur_a, dur_b = win.a[1] - win.a[0], win.b[1] - win.b[0]

    if not fa:
        fail(f"only in B ({len(fb)} frames)")
    if not fb:
        fail(f"missing in B (A has {len(fa)} frames)")

    # frame count, scaled by window length; each schedule can gain or lose a frame at each edge
    if fa and fb and dur_a > 0 and dur_b > 0:
        expect = len(fa) * dur_b / dur_a
        allowed = max(4.0 if sched and sched.dual else 2.0, tm["count_frac"] * expect)
        r["count_expected_b"] = round(expect, 1)
        if abs(len(fb) - expect) > allowed:
            fail(f"count {len(fb)}, expected {expect:.0f} +/- {allowed:.0f} from A's rate")

    # DLC and standard / extended id
    dlc_a, dlc_b = Counter(f.dlc for f in fa), Counter(f.dlc for f in fb)
    r["dlc"] = [dict(dlc_a), dict(dlc_b)]
    if fa and fb and set(dlc_a) != set(dlc_b):
        fail(f"DLC differs: A {sorted(dlc_a)} B {sorted(dlc_b)}")
    if msg is not None:
        for side, c in (("A", dlc_a), ("B", dlc_b)):
            for dlc, n in sorted(c.items()):
                if dlc != msg.length:
                    note(f"{side}: DLC {dlc} but DBC length {msg.length} ({n} frames)")
    ide_a, ide_b = sorted({f.ext for f in fa}), sorted({f.ext for f in fb})
    r["ext"] = [ide_a, ide_b]
    if fa and fb and ide_a != ide_b:
        fail(f"IDE differs: A {'/'.join('ext' if e else 'std' for e in ide_a)},"
             f" B {'/'.join('ext' if e else 'std' for e in ide_b)}")

    # intervals
    ia = [(fa[i].t - fa[i - 1].t) * 1000 for i in range(1, len(fa))]
    ib = [(fb[i].t - fb[i - 1].t) * 1000 for i in range(1, len(fb))]
    sa, sb = spread(ia), spread(ib)
    ref = sched.period_ms if sched else (sa["median"] if sa else None)
    period = {"expected_ms": sched.period_ms if sched else None, "ref_ms": ref,
              "dual": bool(sched and sched.dual), "a": sa, "b": sb}
    if ref:
        lim = DROP_FACTOR * ref
        period["drops"] = [sum(1 for x in ia if x > lim), sum(1 for x in ib if x > lim)]
        if period["drops"][1] > period["drops"][0]:
            worst = max(ib)
            why = "dropped frames" if worst >= LOST_FACTOR * ref else "late frames or timestamp jitter"
            fail(f"{period['drops'][1]} gap(s) over {DROP_FACTOR:g}x {ref:g} ms in B (A: {period['drops'][0]}),"
                 f" longest {worst:.1f} ms: {why}")
    if sa and sb and sa["n"] >= 3 and sb["n"] >= 3:
        if period["dual"]:
            ha, hb = histogram(ia, ref), histogram(ib, ref)
            period["hist"] = [ha, hb]
            for (label, pa), (_, pb) in zip(ha, hb):
                if abs(pa - pb) > tm["hist_frac"] + EPS:
                    fail(f"interval histogram bin {label} ms: A {pa:.0%} B {pb:.0%}")
        elif min(sa["n"], sb["n"]) < MIN_INTERVALS:
            note(f"under {MIN_INTERVALS} intervals: median / p5 / p95 not compared")
        else:
            d = sb["median"] - sa["median"]
            if abs(d) > tm["period_ms"] + EPS:
                fail(f"period median {sb['median']:.2f} ms, A {sa['median']:.2f} ms")
            for p in ("p5", "p95"):
                if abs(sb[p] - sa[p]) > tm["jitter_ms"] + EPS:
                    fail(f"interval {p} {sb[p]:.2f} ms, A {sa[p]:.2f} ms")
    r["period"] = period

    # P -> D copy delay (390/646) and forwarding latency (1154/1666)
    if bus == "P" and fid in COPIED_IDS + FORWARDED_IDS:
        copy = fid in COPIED_IDS
        out = {}
        for side, frames, log in (("a", fa, log_a), ("b", fb, log_b)):
            dt, dfr = log.index.get(("D", fid), ([], []))
            # stay within half a period so a frame can't take the next or previous copy
            half = half_period_s(log.index.get(key, ([], []))[0])
            wnd = min(COPY_WINDOW_S if copy else FORWARD_WINDOW_S, half)
            delays, unmatched, n = match_copies(frames, dt, dfr, min(tm["skew_ms"] / 1000, half),
                                                wnd, COPY_DELAY_S if copy else 0.0, log.d_end)
            out[side] = {"delay": spread(delays), "unmatched": unmatched, "n": n,
                         "window_ms": wnd * 1000}
        label = "P->D copy" if copy else "forwarding"
        r["copy" if copy else "forward"] = out
        a, b = out["a"], out["b"]
        if a["n"] and b["n"]:
            fa_un, fb_un = a["unmatched"] / a["n"], b["unmatched"] / b["n"]
            if b["unmatched"] > a["unmatched"] and fb_un - fa_un > tm["unmatched_frac"] + EPS:
                fail(f"{label}: {b['unmatched']}/{b['n']} P frames not seen on D within"
                     f" {b['window_ms']:.0f} ms (A {a['unmatched']}/{a['n']})")
            if a["delay"] and b["delay"]:
                ma, mb = a["delay"]["median"], b["delay"]["median"]
                if copy and abs(mb - ma) > tm["copy_delay_ms"] + EPS:
                    fail(f"P->D copy delay median {mb:.2f} ms, A {ma:.2f} ms")
                if not copy and mb - ma > tm["forward_ms"] + EPS:
                    fail(f"forwarding latency median {mb:.2f} ms, A {ma:.2f} ms")

    # MBB_Alive
    if fid == ALIVE_ID:
        al = [alive_check(fa, bus), alive_check(fb, bus)]
        r["alive"] = al
        if bus == "P":
            ba, bb = al[0]["gaps"] + al[0]["repeats"], al[1]["gaps"] + al[1]["repeats"]
            if bb > ba:
                fail(f"MBB_Alive: {al[1]['gaps']} gap(s) ({al[1]['missing']} values missing),"
                     f" {al[1]['repeats']} repeat(s) in B at t={al[1]['at']}"
                     f" (A {al[0]['gaps']} gaps, {al[0]['repeats']} repeats)")
        elif al[1]["gaps"] > al[0]["gaps"]:
            fail(f"MBB_Alive: {al[1]['gaps']} jump(s) in B at t={al[1]['at']} (A {al[0]['gaps']})")

    # decoded signals
    if msg is not None:
        periods = min(dur_a, dur_b) * 1000 / ref if ref else 0.0
        r["signals"] = compare_signals(msg, fa, fb, cfg, fail, note, periods)
    else:
        note("not in DBC; payload compared byte by byte")
        r["signals"] = compare_bytes(fa, fb, fail)
    return r


def histogram(intervals_ms, ref_ms):
    """Fraction of intervals per 5 ms bin, bins centred on multiples of 5 ms so that
    intervals near a whole period don't straddle a bin edge."""
    nb = int(math.ceil(2 * ref_ms / HIST_BIN_MS)) + 1
    counts = [0] * (nb + 1)
    for x in intervals_ms:
        counts[min(nb, max(0, int(math.floor(x / HIST_BIN_MS + 0.5))))] += 1
    n = len(intervals_ms) or 1
    h = HIST_BIN_MS / 2
    labels = [f"{max(0.0, i * HIST_BIN_MS - h):g}-{i * HIST_BIN_MS + h:g}" for i in range(nb)]
    labels.append(f">{nb * HIST_BIN_MS - h:g}")
    return [(lab, c / n) for lab, c in zip(labels, counts)]


def signal_stats(msg, frames):
    acc, errors = {}, 0
    for f in frames:
        vals = decode(msg, f.data)
        if isinstance(vals, Exception):
            errors += 1
            continue
        for k, v in vals.items():
            if not isinstance(v, (int, float)):
                continue
            s = acc.setdefault(k, [math.inf, -math.inf, 0.0, 0])
            s[0] = min(s[0], v)
            s[1] = max(s[1], v)
            s[2] += v
            s[3] += 1
    return {k: {"min": s[0], "max": s[1], "mean": s[2] / s[3], "n": s[3]}
            for k, s in acc.items()}, errors


def compare_signals(msg, fa, fb, cfg, fail, note, periods):
    """periods: length of the shorter window in periods of this id."""
    sa, ea = signal_stats(msg, fa)
    sb, eb = signal_stats(msg, fb)
    if ea or eb:
        note(f"decode errors: A {ea}, B {eb}")
    rows = []
    for sig in msg.signals:
        kind = signal_kind(sig)
        a, b = sa.get(sig.name), sb.get(sig.name)
        tol, mean_tol = tolerance(cfg, sig, kind, a)
        if kind == "counter" and periods < 2 ** sig.length + 1:
            # min/max of a counter that hasn't wrapped depend on where the window starts
            tol = None
            note(f"{sig.name}: window shorter than one counter cycle, min/max not compared")
        row = {"signal": sig.name, "kind": kind, "unit": sig.unit or "", "a": a, "b": b,
               "tol": tol, "mean_tol": mean_tol, "ok": True}
        rows.append(row)
        if not fa or not fb:
            continue
        if a is None and b is None:
            note(f"{sig.name}: not decoded in either log (frames too short)")
            continue
        if a is None or b is None:
            row["ok"] = False
            fail(f"{sig.name}: decoded only in {'A' if a else 'B'} (frame length)")
            continue
        diff = {k: b[k] - a[k] for k in ("min", "max", "mean")}
        row["diff"] = diff
        bad = [k for k in ("min", "max") if tol is not None and abs(diff[k]) > tol + EPS]
        if mean_tol is not None and abs(diff["mean"]) > mean_tol + EPS:
            bad.append("mean")
        if bad:
            row["ok"] = False
            parts = ", ".join(f"{k} {fmt_sig(b[k])} vs {fmt_sig(a[k])}" for k in bad)
            fail(f"{sig.name}: {parts} ({tol_text(tol, mean_tol)})")
    return rows


def tol_text(tol, mean_tol):
    t = "min/max not compared" if tol is None else f"tol {fmt_sig(tol)}"
    return t + ("" if mean_tol is None else f", mean tol {fmt_sig(mean_tol)}")


def compare_bytes(fa, fb, fail):
    rows = []
    n = max([len(f.data) for f in fa + fb] or [0])
    for i in range(n):
        va = [f.data[i] for f in fa if len(f.data) > i]
        vb = [f.data[i] for f in fb if len(f.data) > i]
        a = {"min": min(va), "max": max(va), "mean": sum(va) / len(va), "n": len(va)} if va else None
        b = {"min": min(vb), "max": max(vb), "mean": sum(vb) / len(vb), "n": len(vb)} if vb else None
        row = {"signal": f"byte{i}", "kind": "raw", "unit": "", "a": a, "b": b,
               "tol": 0.0, "mean_tol": None, "ok": True}
        if fa and fb and a and b and (a["min"] != b["min"] or a["max"] != b["max"]):
            row["ok"] = False
            fail(f"byte{i}: range {b['min']}..{b['max']} vs {a['min']}..{a['max']}")
        rows.append(row)
    return rows


def windows(log_a, log_b):
    """Overall window, then one per MARK label the logs share in the same order. A
    segment runs to the next MARK of either kind in that log. Returns (windows, warning)."""
    wins = [Window("overall", (0.0, math.inf), (0.0, math.inf))]
    la = [m[1] for m in log_a.marks]
    lb = [m[1] for m in log_b.marks]
    if not la or not lb:
        return wins, "MARK rows only in one log; segment comparison skipped" if la or lb else None
    ta = [m[0] for m in log_a.marks] + [log_a.duration + 1e-6]
    tb = [m[0] for m in log_b.marks] + [log_b.duration + 1e-6]
    if ta[0] > 0 and tb[0] > 0:
        wins.append(Window("before first MARK", (0.0, ta[0]), (0.0, tb[0])))
    pairs = [(m.a + k, m.b + k)
             for m in difflib.SequenceMatcher(None, la, lb, autojunk=False).get_matching_blocks()
             for k in range(m.size)]
    for i, j in pairs:
        wins.append(Window(f'MARK {i + 1} "{la[i]}"', (ta[i], ta[i + 1]), (tb[j], tb[j + 1])))
    only_a = [f'"{la[i]}"' for i in sorted(set(range(len(la))) - {p[0] for p in pairs})]
    only_b = [f'"{lb[j]}"' for j in sorted(set(range(len(lb))) - {p[1] for p in pairs})]
    if not only_a and not only_b:
        return wins, None
    return wins, ("MARK labels differ between logs; segments not compared:"
                  f" A {', '.join(only_a) or '-'}; B {', '.join(only_b) or '-'}")


def compare(log_a, log_b, db, cfg, dbc_fixes=()):
    wins, seg_note = windows(log_a, log_b)
    result = {"a": describe(log_a), "b": describe(log_b), "windows": [],
              "warnings": [f"A: {w}" for w in log_a.warnings[:20]]
              + [f"B: {w}" for w in log_b.warnings[:20]] + list(dbc_fixes),
              "failures": []}
    if seg_note:
        result["warnings"].append(seg_note)
    for side, log, expect in (("A", log_a, "mbed"), ("B", log_b, "hal")):
        fw = log.meta.get("firmware")
        if fw and fw != expect:
            result["warnings"].append(f"{side} sidecar says firmware={fw}, expected {expect}")
    keys = sorted(set(log_a.index) | set(log_b.index))
    min_seg = cfg.timing["min_segment_s"]
    for n, win in enumerate(wins):
        wa = (win.a[0], min(win.a[1], log_a.duration + 1e-6))
        wb = (win.b[0], min(win.b[1], log_b.duration + 1e-6))
        w = Window(win.name, wa, wb)
        ids = []
        entry = {"name": w.name, "a_span": list(wa), "b_span": list(wb), "ids": ids}
        result["windows"].append(entry)
        if n and min(wa[1] - wa[0], wb[1] - wb[0]) < min_seg:
            entry["skipped"] = f"shorter than {min_seg:g} s"
            continue
        for key in keys:
            fa = frames_in(log_a, key, *wa)
            fb = frames_in(log_b, key, *wb)
            if not fa and not fb:
                continue
            r = analyse_key(key, fa, fb, log_a, log_b, w, db, cfg)
            ids.append(r)
            for msg in r["failures"]:
                result["failures"].append(f"[{w.name}] {key[0]} {key[1]}"
                                          f"{' ' + r['name'] if r['name'] else ''}: {msg}")
    result["result"] = "fail" if result["failures"] else "pass"
    return result


def describe(log):
    return {"path": str(log.path), "meta": log.meta, "frames": len(log.frames),
            "duration_s": log.duration, "marks": [list(m) for m in log.marks]}


# ---------------------------------------------------------------- output


def text_report(res, full=False):
    out = []
    w = out.append
    w("CAN log comparison (A = reference, B = under test)")
    for side in ("a", "b"):
        d = res[side]
        m = d["meta"]
        meta = " ".join(f"{k}={m[k]}" for k in ("firmware", "commit", "setup", "adapter") if m.get(k))
        w(f"  {side.upper()}: {d['path']}  {meta}".rstrip())
        w(f"     {d['frames']} frames, {d['duration_s']:.3f} s, {len(d['marks'])} marks")
    n = len(res["failures"])
    w(f"Result: {'PASS' if not n else f'FAIL ({n} problem' + ('s' if n != 1 else '') + ')'}")
    for warn in res["warnings"]:
        w(f"  warning: {warn}")
    for i, win in enumerate(res["windows"]):
        w("")
        sa, sb = win["a_span"], win["b_span"]
        w(f"== {win['name']}  (A {sa[0]:.3f}-{sa[1]:.3f} s, B {sb[0]:.3f}-{sb[1]:.3f} s) ==")
        if win.get("skipped"):
            w(f"  not compared: {win['skipped']}")
            continue
        detailed = full or i == 0
        ok = 0
        for r in win["ids"]:
            if not detailed and not r["failures"]:
                ok += 1
                continue
            id_block(r, w, detailed)
        if ok:
            other = "other " if ok < len(win["ids"]) else ""
            w(f"  {ok} {other}id(s) within tolerance (use --full for detail)")
    if res["failures"]:
        w("")
        w("Problems:")
        for f in res["failures"]:
            w(f"  - {f}")
    return "\n".join(out) + "\n"


def id_block(r, w, detailed):
    name = f"  {r['name']}" if r["name"] else ""
    status = "FAIL" if r["failures"] else "ok"
    ca, cb = r["count"]
    w(f"{r['bus']} {r['id']}{name}  [{r['presence']}]  count A {ca} B {cb}  {status}")
    for n in r["notes"]:
        w(f"    note: {n}")
    p = r.get("period", {})
    if p.get("a") or p.get("b"):
        exp = f" (expected {fmt(p['expected_ms'])})" if p.get("expected_ms") else ""
        w(f"    interval ms{exp}:  median / p5 / p95 / max")
        for side, s in (("A", p.get("a")), ("B", p.get("b"))):
            if s:
                w(f"      {side}  {fmt(s['median'])} / {fmt(s['p5'])} / {fmt(s['p95'])}"
                  f" / {fmt(s['max'])}")
        if "drops" in p:
            w(f"      gaps > {DROP_FACTOR:g}x {fmt(p['ref_ms'])} ms: A {p['drops'][0]}  B {p['drops'][1]}")
        if p.get("hist") and detailed:
            w("      histogram (fraction of intervals)   A      B")
            for (lab, fa), (_, fb) in zip(*p["hist"]):
                if fa or fb:
                    w(f"        {lab:>10} ms                {fa:5.0%}  {fb:5.0%}")
    for k, label in (("copy", "P->D copy delay ms"), ("forward", "forwarding latency ms")):
        if k in r:
            w(f"    {label}:  median / p95 / max, unmatched")
            for side in ("a", "b"):
                s = r[k][side]
                d = s["delay"] or {}
                w(f"      {side.upper()}  {fmt(d.get('median'))} / {fmt(d.get('p95'))}"
                  f" / {fmt(d.get('max'))}, {s['unmatched']}/{s['n']}")
    if "alive" in r:
        a, b = r["alive"]
        w(f"    MBB_Alive: gaps A {a['gaps']} B {b['gaps']}, repeats A {a['repeats']} B {b['repeats']}"
          + ("  (repeats expected on D)" if r["bus"] == "D" else ""))
    rows = r.get("signals") or []
    if rows and (detailed or any(not x["ok"] for x in rows)):
        w(f"    {'signal':<36} {'A min/max/mean':<30} {'B min/max/mean':<30} {'B-A min/max/mean':<26} tol")
        for x in rows:
            if not detailed and x["ok"]:
                continue
            a, b, d = x["a"], x["b"], x.get("diff")
            sa = f"{fmt_sig(a['min'])}/{fmt_sig(a['max'])}/{fmt_sig(a['mean'])}" if a else "-"
            sb = f"{fmt_sig(b['min'])}/{fmt_sig(b['max'])}/{fmt_sig(b['mean'])}" if b else "-"
            sd = "/".join(f"{d[k]:+.6g}" for k in ("min", "max", "mean")) if d else "-"
            tol = "-" if x["tol"] is None else fmt_sig(x["tol"])
            if x["mean_tol"] is not None:
                tol += f", mean {fmt_sig(x['mean_tol'])}"
            flag = "" if x["ok"] else "  FAIL"
            w(f"    {x['signal']:<36} {sa:<30} {sb:<30} {sd:<26} {tol}{flag}")
    for f in r["failures"]:
        w(f"    ! {f}")


def to_json(res):
    def clean(o):
        if isinstance(o, float) and not math.isfinite(o):
            return None
        if isinstance(o, dict):
            return {str(k): clean(v) for k, v in o.items()}
        if isinstance(o, (list, tuple)):
            return [clean(v) for v in o]
        return o
    return json.dumps(clean(res), indent=2)


# ---------------------------------------------------------------- main


def main(argv=None):
    ap = argparse.ArgumentParser(description="Compare a Mbed VCU CAN log (A) with a HAL one (B).")
    ap.add_argument("a", help="reference log (Mbed)")
    ap.add_argument("b", help="log under test (HAL)")
    ap.add_argument("--dbc", default=os.environ.get("CAN_DBC", str(DEFAULT_DBC)))
    ap.add_argument("--config", help="TOML file with [timing] and [signals] tolerances")
    ap.add_argument("--tol", action="append", metavar="SIGNAL=VALUE",
                    help="signal min/max tolerance; default_analog=VALUE and range_frac=VALUE"
                         " set the analog defaults")
    ap.add_argument("--timing", action="append", metavar="KEY=VALUE",
                    help=f"timing tolerance, keys: {', '.join(DEFAULT_TIMING)}")
    ap.add_argument("--json", action="store_true", help="print JSON instead of text")
    ap.add_argument("--full", action="store_true", help="print every id in every segment")
    args = ap.parse_args(argv)
    try:
        cfg = load_config(args.config, args.tol, args.timing)
        db, fixes = load_dbc(args.dbc)
        log_a, log_b = load_log(args.a), load_log(args.b)
    except (InputError, ValueError) as e:
        print(f"compare.py: {e}", file=sys.stderr)
        return 2
    if not log_a.frames:
        print(f"compare.py: {log_a.path}: no frames in the reference log", file=sys.stderr)
        return 2
    res = compare(log_a, log_b, db, cfg, fixes)
    sys.stdout.write(to_json(res) + "\n" if args.json else text_report(res, args.full))
    return 1 if res["failures"] else 0


if __name__ == "__main__":
    sys.exit(main())
