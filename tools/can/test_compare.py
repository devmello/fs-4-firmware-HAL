"""Tests for compare.py using synthetic logs that follow the vcu/main.cpp schedule.

Run: .venv/bin/python -m pytest tools/can
Regenerate the sample logs in testdata/: .venv/bin/python tools/can/test_compare.py
"""

import json
import random
import struct
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import compare  # noqa: E402

TESTDATA = HERE / "testdata"
DBC = Path(compare.os.environ.get("CAN_DBC", str(compare.DEFAULT_DBC)))
pytestmark = pytest.mark.skipif(not DBC.exists(), reason=f"DBC not found at {DBC}")

MARKS = [(0.5, "idle"), (1.0, "pedal sweep start"), (2.0, "pedal sweep end")]


def pedal_at(t):
    """Accelerator position 0..1, ramps up and down between 1.0 and 2.0 s (nominal time)."""
    if 1.0 <= t < 1.5:
        return (t - 1.0) / 0.5
    if 1.5 <= t < 2.0:
        return (2.0 - t) / 0.5
    return 0.0


def make_log(duration=3.0, period_ms=40.0, copy_delay_ms=5.0, fwd_delay_ms=0.3,
             jitter_ms=0.0, seed=1, drop=(), skip_ids=(), marks=MARKS, mutate=None,
             fwd_jitter_ms=None, fwd_period_ms=10.0, pedal_lag_s=0.0):
    """Frames as (t, bus, id, data). drop: set of (bus, id, index) to leave out.
    mutate: f(bus, id, index, bytearray) to change payloads.
    fwd_jitter_ms: jitter on both the P frame and the D copy of 1154/1666 (default: a
    quarter of jitter_ms on the D copy only). pedal_lag_s: delay the pedal sweep by this
    much relative to the send times."""
    rng = random.Random(seed)
    frames = []
    counters = {}

    def j():
        return rng.uniform(-jitter_ms, jitter_ms) / 1000.0 if jitter_ms else 0.0

    def fj():
        return rng.uniform(-fwd_jitter_ms, fwd_jitter_ms) / 1000.0

    def pedal(t):
        return pedal_at(t - pedal_lag_s)

    def add(t, bus, fid, data):
        n = counters.get((bus, fid), 0)
        counters[(bus, fid)] = n + 1
        if fid in skip_ids or (bus, fid, n) in drop:
            return
        data = bytearray(data)
        if mutate:
            mutate(bus, fid, n, data)
        frames.append((max(0.0, t), bus, fid, bytes(data)))

    def throttle(t, alive):
        torque = int(pedal(t) * 20000)
        return struct.pack("<hhBB", torque, 6000, 1 | (1 << 3), alive) + bytes(2)

    currents = struct.pack("<HH", 50, 200) + bytes(4)

    # 40 ms powertrain job: P frames, D copy after sleep_for(5ms)
    alive_at = []  # (time, alive) for the 80 ms job
    k = 0
    while (t := 0.001 + k * period_ms / 1000.0) < duration:
        alive = (k + 1) % 16
        alive_at.append((t, alive))
        tj = t + j()
        add(tj, "P", 390, throttle(t, alive))
        add(tj + 0.0002, "P", 646, currents)
        td = tj + copy_delay_ms / 1000.0 + j()
        add(td, "D", 390, throttle(t, alive))
        add(td + 0.0002, "D", 646, currents)
        k += 1

    # 80 ms data job on the same thread: runs after the 40 ms job finishes
    k = 0
    while (t := 0.001 + k * 0.080 + copy_delay_ms / 1000.0 + 0.0005) < duration:
        alive = [a for ta, a in alive_at if ta <= t][-1]
        tj = t + j()
        add(tj, "D", 390, throttle(t - copy_delay_ms / 1000.0, alive))
        add(tj + 0.0002, "D", 660, bytes([int(pedal(t) * 10), 100, 0, 10, 0, 0, 0, 0]))
        add(tj + 0.0004, "D", 646, currents)
        k += 1

    # 50 ms ETC job
    k = 0
    while (t := 0.002 + k * 0.050) < duration:
        p = pedal(t)
        apps1 = int(500 + p * 3000)
        pedal_msg = struct.pack("<HHHBB", apps1, apps1 // 2, 400, int(p * 100), 0)
        status = struct.pack("<BBHHh", 0b00001011, 0, 120, 110, -150)
        tj = t + j()
        add(tj, "D", 402, pedal_msg)
        add(tj + 0.0002, "D", 403, status)
        k += 1

    # 10 ms IMU job (722 is 6 bytes, the DBC says 7)
    k = 0
    while (t := 0.003 + k * 0.010) < duration:
        tj = t + j()
        add(tj, "D", 720, struct.pack("<hhh", 12, -5, -981))
        add(tj + 0.0001, "D", 976, struct.pack("<hhh", 9000, 150, -40))
        add(tj + 0.0002, "D", 721, struct.pack("<ii", 369_999_000, -1_220_600_000))
        add(tj + 0.0003, "D", 977, struct.pack("<hhh", 3, 0, -2))
        add(tj + 0.0004, "D", 722, struct.pack("<hhh", 0, 0, 0))
        k += 1

    # SME traffic on P, forwarded to D by the main loop
    def forward(t, fid, msg):
        if fwd_jitter_ms is None:
            add(t, "P", fid, msg)
            add(t + fwd_delay_ms / 1000.0 + j() / 4, "D", fid, msg)
        else:
            add(t + fj(), "P", fid, msg)
            add(t + fwd_delay_ms / 1000.0 + fj(), "D", fid, msg)

    k = 0
    while (t := 0.004 + k * (fwd_period_ms / 1000.0)) < duration:
        rpm = int(pedal(t) * 3000)
        forward(t, 1154, struct.pack("<HhBBH", rpm, int(pedal(t) * 8000), 0, 0b00110000, 0))
        k += 1
    k = 0
    while (t := 0.006 + k * 0.100) < duration:
        forward(t, 1666, struct.pack("<BBHBBh", 65, 60, 3950, 0, 0, -12))
        k += 1

    rows = [(t, bus, fid, data.hex().upper(), None) for t, bus, fid, data in frames]
    rows += [(t, "MARK", None, None, label) for t, label in marks]
    rows.sort(key=lambda r: (r[0], r[1] == "MARK"))
    return rows


def write_log(path, rows, meta=None, ext_ids=()):
    lines = ["time_s,bus,id,ext,dlc,data"]
    for t, bus, fid, data, label in rows:
        if bus == "MARK":
            lines.append(f'{t:.6f},MARK,,,,"{label}"')
        else:
            lines.append(f"{t:.6f},{bus},{fid},{int(fid in ext_ids)},{len(data) // 2},{data}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    if meta:
        path.with_suffix(".json").write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")
    return path


def run(tmp_path, a_rows, b_rows, *extra):
    a = write_log(tmp_path / "a.csv", a_rows, {"firmware": "mbed"})
    b = write_log(tmp_path / "b.csv", b_rows, {"firmware": "hal"})
    rc = compare.main([str(a), str(b), "--dbc", str(DBC), "--json", *extra])
    return rc


def result(tmp_path, a_rows, b_rows, *extra, capsys):
    rc = run(tmp_path, a_rows, b_rows, *extra)
    res = json.loads(capsys.readouterr().out)
    return rc, res


def failures(res, contains, window="overall"):
    return [f for f in res["failures"] if contains in f and f.startswith(f"[{window}")]


# ---------------------------------------------------------------- pass cases


def test_identical_logs_pass(tmp_path, capsys):
    rows = make_log()
    rc, res = result(tmp_path, rows, rows, capsys=capsys)
    assert res["failures"] == []
    assert rc == 0 and res["result"] == "pass"
    names = [w["name"] for w in res["windows"]]
    assert names[0] == "overall" and 'MARK 2 "pedal sweep start"' in names


def test_small_jitter_passes(tmp_path, capsys):
    rc, res = result(tmp_path, make_log(jitter_ms=0.2, seed=1),
                     make_log(jitter_ms=0.2, seed=2), capsys=capsys)
    assert res["failures"] == [] and rc == 0


def test_sample_logs_pass(capsys):
    rc = compare.main([str(TESTDATA / "mbed_bench_sample.csv"), str(TESTDATA / "hal_bench_sample.csv"),
                       "--dbc", str(DBC), "--config", str(TESTDATA / "tolerances.toml")])
    out = capsys.readouterr().out
    assert rc == 0, out
    assert "Result: PASS" in out


def test_dual_schedule_histogram_reported(tmp_path, capsys):
    rows = make_log()
    _, res = result(tmp_path, rows, rows, capsys=capsys)
    d390 = next(r for r in res["windows"][0]["ids"] if r["bus"] == "D" and r["id"] == 390)
    assert d390["period"]["dual"] and d390["period"]["hist"]
    assert d390["alive"][0]["repeats"] > 0  # 80 ms job repeats the value on D


def test_dlc_mismatch_reported_not_fatal(tmp_path, capsys):
    rows = make_log()
    _, res = result(tmp_path, rows, rows, capsys=capsys)
    r722 = next(r for r in res["windows"][0]["ids"] if r["id"] == 722)
    assert any("DLC 6 but DBC length 7" in n for n in r722["notes"])
    assert {s["signal"] for s in r722["signals"] if s["a"]} == {
        "VCU_VN_BODY_VX", "VCU_VN_BODY_VY", "VCU_VN_BODY_VZ"}


def test_analog_tolerance_from_cli(tmp_path, capsys):
    def bump(bus, fid, n, data):
        if fid == 402 and n == 5:  # pedal released, so A's range is 0 before the first MARK
            v = int.from_bytes(data[0:2], "little") + 10
            data[0:2] = v.to_bytes(2, "little")
    a, b = make_log(), make_log(mutate=bump)
    rc, res = result(tmp_path, a, b, capsys=capsys)
    assert rc == 1
    assert failures(res, "VCU_APPS1_HE: max 510 vs 500 (tol 1)", "before first MARK"), res["failures"]
    assert not failures(res, "VCU_APPS1_HE")  # 10 mV is under 2% of the sweep range
    rc, res = result(tmp_path, a, b, "--tol", "VCU_APPS1_HE=20", capsys=capsys)
    assert rc == 0, res["failures"]


def test_tolerance_file(tmp_path, capsys):
    cfg = tmp_path / "tol.toml"
    cfg.write_text('[timing]\ncopy_delay_ms = 4.0\n[signals]\nVCU_APPS1_HE = { tol = 5, mean = 1 }\n')
    rc, res = result(tmp_path, make_log(), make_log(copy_delay_ms=8.0),
                     "--config", str(cfg), capsys=capsys)
    assert rc == 0, res["failures"]


def test_sampling_phase_passes_with_default_analog(tmp_path, capsys):
    # the sweep 3 ms earlier against the send times: peaks and means move a little
    rc, res = result(tmp_path, make_log(), make_log(pedal_lag_s=-0.003), capsys=capsys)
    assert res["failures"] == [] and rc == 0
    d402 = next(r for r in res["windows"][0]["ids"] if r["id"] == 402)
    apps = next(s for s in d402["signals"] if s["signal"] == "VCU_APPS1_HE")
    assert apps["diff"]["max"] != 0 and apps["mean_tol"] is None
    assert apps["tol"] == pytest.approx(0.02 * (apps["a"]["max"] - apps["a"]["min"]))


def test_analog_offset_fails(tmp_path, capsys):
    def offset(bus, fid, n, data):
        if fid == 402:
            v = int.from_bytes(data[0:2], "little") + 100
            data[0:2] = v.to_bytes(2, "little")
    rc, res = result(tmp_path, make_log(), make_log(mutate=offset), capsys=capsys)
    assert rc == 1
    assert failures(res, "VCU_APPS1_HE: min 600 vs 500, max 3588 vs 3488 (tol 59.76)"), res["failures"]


def test_mean_tolerance(tmp_path, capsys):
    def hold(bus, fid, n, data):
        if fid == 402 and 22 <= n < 27:  # held at 3000 mV on the way up: same peak, higher mean
            data[0:2] = (3000).to_bytes(2, "little")
    a, b = make_log(), make_log(mutate=hold)
    cfg = tmp_path / "tol.toml"
    cfg.write_text("[signals]\nVCU_APPS1_HE = { tol = 0, mean = 1 }\n")
    rc, res = result(tmp_path, a, b, "--config", str(cfg), capsys=capsys)
    assert rc == 1 and failures(res, "VCU_APPS1_HE: mean"), res["failures"]
    cfg.write_text("[signals]\nVCU_APPS1_HE = { tol = 0, mean = false }\n")
    rc, res = result(tmp_path, a, b, "--config", str(cfg), capsys=capsys)
    assert rc == 0, res["failures"]
    cfg.write_text('[signals]\nVCU_APPS1_HE = { tol = 0, mean = "none" }\n')
    assert run(tmp_path, a, b, "--config", str(cfg)) == 2
    assert "VCU_APPS1_HE: expected a number" in capsys.readouterr().err


def test_shipped_tolerances_cover_vcu_analog_outputs():
    db, _ = compare.load_dbc(DBC)
    cfg = compare.load_config(TESTDATA / "tolerances.toml", [], [])
    for fid in (390, 646, 402, 403, 660, 720, 721, 722, 976, 977):
        for sig in db.get_message_by_frame_id(fid).signals:
            if compare.signal_kind(sig) == "analog":
                assert sig.name in cfg.signals, sig.name


# ---------------------------------------------------------------- fail cases


def test_dropped_frame_fails(tmp_path, capsys):
    rc, res = result(tmp_path, make_log(), make_log(drop={("P", 390, 30)}), capsys=capsys)
    assert rc == 1
    msg = failures(res, "P 390")
    assert any("gap(s) over 1.5x 40 ms in B (A: 0), longest 80.0 ms: dropped frames" in f for f in msg), msg
    assert any("MBB_Alive: 1 gap(s) (1 values missing)" in f for f in msg), msg
    # the drop falls in the "pedal sweep start" segment (1.2 s)
    assert failures(res, "dropped frames", 'MARK 2 "pedal sweep start"')
    assert not failures(res, "dropped frames", 'MARK 1 "idle"')


def test_dropped_frame_with_jitter_fails(tmp_path, capsys):
    rc, res = result(tmp_path, make_log(jitter_ms=0.2, seed=1),
                     make_log(jitter_ms=0.2, seed=2, drop={("D", 720, 100)}), capsys=capsys)
    assert rc == 1
    assert failures(res, "D 720 VCU_VN_BODY_ACCEL: 1 gap(s) over 1.5x 10 ms in B"), res["failures"]


def test_shifted_period_fails(tmp_path, capsys):
    rc, res = result(tmp_path, make_log(), make_log(period_ms=42.0), capsys=capsys)
    assert rc == 1
    assert failures(res, "P 390 SME_RPDO_Throttle_Demand: period median 42.00 ms, A 40.00 ms"), \
        res["failures"]
    assert failures(res, "P 390 SME_RPDO_Throttle_Demand: count")
    # 12 intervals in the 0.5 s idle segment: too few for the median, values still reported
    assert not failures(res, "period median", 'MARK 1 "idle"')
    idle = next(w for w in res["windows"] if w["name"] == 'MARK 1 "idle"')
    p390 = next(r for r in idle["ids"] if r["bus"] == "P" and r["id"] == 390)
    assert p390["period"]["b"]["median"] == pytest.approx(42.0)
    assert any("median / p5 / p95 not compared" in n for n in p390["notes"])


def test_changed_payload_byte_fails(tmp_path, capsys):
    def poke(bus, fid, n, data):
        if fid == 403 and n == 10:
            data[1] |= 0x40  # VCU_BSPD_FAULT
    rc, res = result(tmp_path, make_log(), make_log(mutate=poke), capsys=capsys)
    assert rc == 1
    assert failures(res, "D 403 VCU_TPDO_STATUS: VCU_BSPD_FAULT: max 1 vs 0 (tol 0)"), res["failures"]


def test_missing_id_fails(tmp_path, capsys):
    rc, res = result(tmp_path, make_log(), make_log(skip_ids={660}), capsys=capsys)
    assert rc == 1
    assert failures(res, "D 660 VCU_TPDO_TRACTION_DATA: missing in B (A has")


def test_extra_id_fails(tmp_path, capsys):
    rc, res = result(tmp_path, make_log(skip_ids={660}), make_log(), capsys=capsys)
    assert rc == 1 and failures(res, "D 660 VCU_TPDO_TRACTION_DATA: only in B")


def test_wrong_copy_delay_fails(tmp_path, capsys):
    rc, res = result(tmp_path, make_log(), make_log(copy_delay_ms=8.0), capsys=capsys)
    assert rc == 1
    assert failures(res, "P 390 SME_RPDO_Throttle_Demand: P->D copy delay median 8.00 ms, A 5.00 ms")
    assert failures(res, "P 646 SME_RPDO_Max_Currents: P->D copy delay median 8.00 ms, A 5.00 ms")


def test_missing_copy_fails(tmp_path, capsys):
    drop = {("D", 390, n) for n in range(0, 40)}
    rc, res = result(tmp_path, make_log(), make_log(drop=drop), capsys=capsys)
    assert rc == 1 and failures(res, "P 390 SME_RPDO_Throttle_Demand: P->D copy:")


def test_slow_forwarding_fails(tmp_path, capsys):
    rc, res = result(tmp_path, make_log(), make_log(fwd_delay_ms=4.0), capsys=capsys)
    assert rc == 1
    assert failures(res, "P 1154 SME_TPDO_Torque_speed: forwarding latency median 4.00 ms, A 0.30 ms")


def test_not_forwarded_fails(tmp_path, capsys):
    rc, res = result(tmp_path, make_log(), make_log(drop={("D", 1666, n) for n in range(5)}),
                     capsys=capsys)
    assert rc == 1
    assert failures(res, "P 1666 SME_TPDO_Temperature: forwarding: 5/30 P frames not seen on D")


def test_alive_repeat_fails(tmp_path, capsys):
    def stick(bus, fid, n, data):
        if fid == 390 and bus == "P" and n == 20:
            data[5] = (data[5] - 1) % 16
    rc, res = result(tmp_path, make_log(), make_log(mutate=stick), capsys=capsys)
    assert rc == 1
    assert failures(res, "MBB_Alive: 1 gap(s) (1 values missing), 1 repeat(s)"), res["failures"]


def test_mark_labels_differ_compares_shared(tmp_path, capsys):
    # B has an extra operator mark and was stopped with Ctrl-C near the end
    other = make_log(marks=[(0.3, "note"), (0.5, "idle"), (1.0, "pedal sweep start"),
                            (2.0, "pedal sweep end"), (2.9, "stimulus aborted")])
    rc, res = result(tmp_path, make_log(), other, capsys=capsys)
    assert rc == 0, res["failures"]
    wins = {w["name"]: w for w in res["windows"]}
    assert list(wins) == ["overall", "before first MARK", 'MARK 1 "idle"',
                          'MARK 2 "pedal sweep start"', 'MARK 3 "pedal sweep end"']
    assert all(wins[n]["ids"] for n in list(wins)[2:])
    assert wins['MARK 3 "pedal sweep end"']["b_span"] == [2.0, 2.9]
    assert any('not compared: A -; B "note", "stimulus aborted"' in w for w in res["warnings"])


def test_mark_labels_differ_still_finds_faults(tmp_path, capsys):
    other = make_log(drop={("P", 390, 30)}, marks=MARKS + [(2.9, "stimulus aborted")])
    rc, res = result(tmp_path, make_log(), other, capsys=capsys)
    assert rc == 1 and failures(res, "dropped frames", 'MARK 2 "pedal sweep start"')


def test_short_segments_not_compared(tmp_path, capsys):
    # stimulus.py: first MARK 1 ms after logging starts, "end" 30 ms before it stops
    marks = [(0.001, "idle"), (1.0, "pedal sweep start"), (2.97, "end")]
    a = make_log(marks=marks)
    b = make_log(jitter_ms=0.2, seed=2, marks=[(0.0012, "idle"), (1.0, "pedal sweep start"),
                                               (2.975, "end")])
    rc, res = result(tmp_path, a, b, capsys=capsys)
    assert rc == 0, res["failures"]
    wins = {w["name"]: w for w in res["windows"]}
    assert wins["before first MARK"]["skipped"] == "shorter than 0.5 s"
    assert wins['MARK 3 "end"']["skipped"] == "shorter than 0.5 s"
    assert wins['MARK 3 "end"']["ids"] == [] and wins['MARK 1 "idle"']["ids"]


def test_counter_min_max_needs_a_full_cycle(tmp_path, capsys):
    # 0.55 s segments one period apart: MBB_Alive covers 1..14 in A and 2..15 in B
    a = make_log(marks=[(0.0005, "a"), (0.55, "b")])
    b = make_log(marks=[(0.04, "a"), (0.59, "b")])
    rc, res = result(tmp_path, a, b, capsys=capsys)
    assert rc == 0, res["failures"]

    def alive(window):
        p390 = next(r for r in window["ids"] if r["bus"] == "P" and r["id"] == 390)
        return next(s for s in p390["signals"] if s["signal"] == "SME_THROTL_MBB_Alive")
    seg = alive(next(w for w in res["windows"] if w["name"] == 'MARK 1 "a"'))
    assert (seg["a"]["min"], seg["b"]["min"]) == (1, 2) and seg["tol"] is None
    assert alive(res["windows"][0])["tol"] == 0  # the overall window still checks it


def test_count_allows_one_frame_per_schedule_per_edge(tmp_path, capsys):
    # A's segment starts and ends just after a 40 ms + 80 ms pair of D 390 frames,
    # B's just before, so B has 2 extra frames per schedule
    rows = make_log(duration=1.0)
    a = [r for r in rows if r[1] != "MARK"] + [(0.487, "MARK", None, None, "idle"),
                                               (0.966, "MARK", None, None, "x")]
    b = [r for r in rows if r[1] != "MARK"] + [(0.4855, "MARK", None, None, "idle"),
                                               (0.9885, "MARK", None, None, "x")]
    rc, res = result(tmp_path, sorted(a), sorted(b), "--timing", "min_segment_s=0.4", capsys=capsys)
    assert rc == 0, res["failures"]
    idle = next(w for w in res["windows"] if w["name"] == 'MARK 1 "idle"')
    d390 = next(r for r in idle["ids"] if r["bus"] == "D" and r["id"] == 390)
    assert d390["count"] == [16, 20] and d390["count_expected_b"] == pytest.approx(16.8)


def test_forwarding_with_skew_passes(tmp_path, capsys):
    # +-1 ms on both the P frame and its D copy: copies are often stamped first
    rc, res = result(tmp_path, make_log(jitter_ms=0.2, seed=1, fwd_jitter_ms=1.0),
                     make_log(jitter_ms=0.2, seed=2, fwd_jitter_ms=1.0), capsys=capsys)
    assert res["failures"] == [] and rc == 0
    fwd = next(r for r in res["windows"][0]["ids"] if r["id"] == 1154 and r["bus"] == "P")["forward"]
    assert fwd["b"]["unmatched"] == 0 and fwd["b"]["delay"]["p5"] < 0


def test_forward_stamped_before_source_passes(tmp_path, capsys):
    rows = make_log(fwd_period_ms=20.0)  # 1154 payload is constant while the pedal is released
    d1154 = [i for i, r in enumerate(rows) if r[1] == "D" and r[2] == 1154]
    early = list(rows)
    i = d1154[10]
    early[i] = (rows[i][0] - 0.0005,) + rows[i][1:]  # 0.2 ms before its P frame
    rc, res = result(tmp_path, rows, sorted(early, key=lambda r: r[0]), capsys=capsys)
    assert res["failures"] == [] and rc == 0
    fwd = next(r for r in res["windows"][0]["ids"] if r["id"] == 1154 and r["bus"] == "P")["forward"]
    assert fwd["b"]["unmatched"] == 0 and fwd["b"]["delay"]["median"] == pytest.approx(0.3)


def test_dropped_forward_is_not_latency(tmp_path, capsys):
    rc, res = result(tmp_path, make_log(fwd_period_ms=20.0),
                     make_log(fwd_period_ms=20.0, drop={("D", 1154, 10)}), capsys=capsys)
    assert rc == 1
    assert failures(res, "D 1154 SME_TPDO_Torque_speed: 1 gap(s) over 1.5x 20 ms in B (A: 0),"
                         " longest 40.0 ms: dropped frames"), res["failures"]
    assert not [f for f in res["failures"] if "latency" in f]  # in any window
    fwd = next(r for r in res["windows"][0]["ids"] if r["id"] == 1154 and r["bus"] == "P")["forward"]
    assert fwd["b"]["unmatched"] == 1 and fwd["b"]["delay"]["median"] == pytest.approx(0.3)


def test_capture_ending_before_copy_passes(tmp_path, capsys):
    rows = make_log()
    last_p390 = max(r[0] for r in rows if r[1] == "P" and r[2] == 390)
    cut = [r for r in rows if r[0] <= last_p390 + 0.002]  # stops before its 5 ms D copy
    rc, res = result(tmp_path, rows, cut, capsys=capsys)
    assert res["failures"] == [] and rc == 0


def test_late_frame_is_not_called_dropped(tmp_path, capsys):
    rows = make_log()
    late = list(rows)
    i = [k for k, r in enumerate(rows) if r[1] == "D" and r[2] == 720][100]
    late[i] = (rows[i][0] + 0.0057,) + rows[i][1:]
    rc, res = result(tmp_path, rows, sorted(late, key=lambda r: r[0]), capsys=capsys)
    assert rc == 1
    assert failures(res, "D 720 VCU_VN_BODY_ACCEL: 1 gap(s) over 1.5x 10 ms in B (A: 0),"
                         " longest 15.7 ms: late frames or timestamp jitter"), res["failures"]


def test_ext_flag_differs_fails(tmp_path, capsys):
    rows = make_log()
    a = write_log(tmp_path / "a.csv", rows)
    b = write_log(tmp_path / "b.csv", rows, ext_ids={390})
    rc = compare.main([str(a), str(b), "--dbc", str(DBC), "--json"])
    res = json.loads(capsys.readouterr().out)
    assert rc == 1
    assert failures(res, "P 390 SME_RPDO_Throttle_Demand: IDE differs: A std, B ext"), res["failures"]
    assert failures(res, "D 390 SME_RPDO_Throttle_Demand: IDE differs: A std, B ext")


def test_empty_b_log_fails(tmp_path, capsys):
    rows = make_log()
    rc, res = result(tmp_path, rows, [(0.5, "MARK", None, None, "idle")], capsys=capsys)
    assert rc == 1
    assert failures(res, "D 403 VCU_TPDO_STATUS: missing in B (A has 60 frames)")
    assert "B: no frames" in res["warnings"]


def test_empty_a_log_exit_2(tmp_path, capsys):
    assert run(tmp_path, [], make_log()) == 2
    assert f"{tmp_path / 'a.csv'}: no frames" in capsys.readouterr().err


def test_text_report(tmp_path, capsys):
    rc = run(tmp_path, make_log(), make_log(copy_delay_ms=8.0))
    assert rc == 1
    capsys.readouterr()
    compare.main([str(tmp_path / "a.csv"), str(tmp_path / "b.csv"), "--dbc", str(DBC)])
    out = capsys.readouterr().out
    assert "Result: FAIL" in out and "Problems:" in out
    assert "P->D copy delay ms" in out
    assert "B-A min/max/mean" in out


def test_text_report_shows_signal_difference(tmp_path, capsys):
    def offset(bus, fid, n, data):
        if fid == 402:
            v = int.from_bytes(data[0:2], "little") + 100
            data[0:2] = v.to_bytes(2, "little")
    write_log(tmp_path / "a.csv", make_log())
    write_log(tmp_path / "b.csv", make_log(mutate=offset))
    compare.main([str(tmp_path / "a.csv"), str(tmp_path / "b.csv"), "--dbc", str(DBC)])
    line = next(x for x in capsys.readouterr().out.splitlines() if "VCU_APPS1_HE" in x and "FAIL" in x)
    assert "+100/+100/+100" in line


def test_bad_input_exit_2(tmp_path, capsys):
    bad = tmp_path / "bad.csv"
    bad.write_text("foo,bar\n1,2\n")
    assert compare.main([str(bad), str(bad), "--dbc", str(DBC)]) == 2


def write_samples():
    TESTDATA.mkdir(exist_ok=True)
    write_log(TESTDATA / "mbed_bench_sample.csv", make_log(jitter_ms=0.15, seed=11),
              {"firmware": "mbed", "commit": "c12c834d", "date": "2026-10-01",
               "adapter": "synthetic", "setup": "bench", "notes": "generated by test_compare.py"})
    write_log(TESTDATA / "hal_bench_sample.csv", make_log(jitter_ms=0.15, seed=12, copy_delay_ms=5.3),
              {"firmware": "hal", "commit": "synthetic", "date": "2026-10-01",
               "adapter": "synthetic", "setup": "bench", "notes": "generated by test_compare.py"})


if __name__ == "__main__":
    write_samples()
