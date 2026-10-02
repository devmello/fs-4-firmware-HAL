"""Tests for capture.py and stimulus.py on python-can's virtual interface.

Run: .venv/bin/python -m pytest tools/can/test_capture_stimulus.py
"""

import csv
import io
import json
import re
import sys
import threading
import time
import uuid
from pathlib import Path

import can
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import capture  # noqa: E402
import stimulus  # noqa: E402

HERE = Path(__file__).resolve().parent
TOL_S = 0.002

# Raw frames only, so this does not need the DBC.
SCENARIO = {
    "streams": {
        "p_status": {"bus": "P", "id": 913, "period_ms": 100, "data": "01"},
        "d_ext": {"bus": "D", "id": 0x1ABCDE, "ext": 1, "period_ms": 25,
                  "offset_ms": 5, "data": "AA"},
    },
    "steps": [
        {"label": "first", "duration_s": 0.5},
        {"label": "second step", "duration_s": 0.4, "wait": True,
         "set": {"p_status": {"data": "02"}}},
        {"label": "third, \"quoted\"", "duration_s": 0.3,
         "set": {"p_status": {"data": "03"}, "d_ext": {"data": "BBCC"}}},
    ],
}


def channels():
    tag = uuid.uuid4().hex[:8]
    return f"virtual:p-{tag}", f"virtual:d-{tag}"


def read_rows(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def expected_frames(scenario, t_start):
    """(time_s, bus, id, data) for every frame the scenario should send."""
    bounds = []
    t = 0.0
    for step in scenario["steps"]:
        bounds.append((t, t + step["duration_s"], step))
        t += step["duration_s"]
    total = t
    out = []
    for name, spec in scenario["streams"].items():
        data = spec["data"]
        k = 0
        while True:
            ts = spec.get("offset_ms", 0) / 1000 + k * spec["period_ms"] / 1000
            if ts >= total - 1e-9:
                break
            # A frame due exactly at a step start goes out with the new step's data.
            data = spec["data"]
            for start, end, step in bounds:
                if start - 1e-9 <= ts:
                    data = (step.get("set") or {}).get(name, {}).get("data", data)
            out.append((t_start + ts, spec["bus"], spec["id"], data.upper()))
            k += 1
    return sorted(out)


def test_bus_kwargs_listen_only():
    kw, warn = capture.bus_kwargs("pcan:PCAN_USBBUS1", 500000, True)
    assert kw == {"interface": "pcan", "channel": "PCAN_USBBUS1", "bitrate": 500000,
                  "state": can.BusState.PASSIVE}
    assert warn is None
    kw, _ = capture.bus_kwargs("kvaser:0", 1000000, True)
    assert kw["channel"] == 0 and kw["driver_mode"] is False
    kw, _ = capture.bus_kwargs("slcan:/dev/cu.usbmodem1", 500000, True)
    assert kw["channel"] == "/dev/cu.usbmodem1" and kw["listen_only"] is True
    kw, _ = capture.bus_kwargs("gs_usb:0", 500000, True)
    assert kw == {"interface": "gs_usb", "channel": 0, "bitrate": 500000}
    kw, warn = capture.bus_kwargs("socketcan:can0", 500000, True)
    assert warn and "ip link" in warn and "will ACK" in warn
    kw, _ = capture.bus_kwargs("pcan:PCAN_USBBUS1", 500000, False)
    assert "state" not in kw
    kw, _ = capture.bus_kwargs("virtual:p", 500000, True)
    assert kw == {"interface": "virtual", "channel": "p"}
    with pytest.raises(ValueError):
        capture.parse_spec("pcan")


def test_stimulus_captured_rows_markers_and_timing(tmp_path):
    p_spec, d_spec = channels()
    cap_buses = {"P": capture.open_bus(p_spec, 500000, listen_only=True),
                 "D": capture.open_bus(d_spec, 1000000, listen_only=True)}
    stim_buses = {"P": capture.open_bus(p_spec, 500000),
                  "D": capture.open_bus(d_spec, 1000000)}
    out = tmp_path / "run.csv"
    log = capture.open_log(out)
    stop, threads, skipped = capture.start_readers(cap_buses, log)
    try:
        stats = stimulus.play(SCENARIO, stim_buses, log=log, out=lambda *_: None)
        time.sleep(0.1)
    finally:
        capture.stop_readers(stop, threads)
        for bus in list(cap_buses.values()) + list(stim_buses.values()):
            bus.shutdown()
        capture.close_log(log)

    t_start = capture.log_time(log, stats["t_start_ns"])
    rows = read_rows(out)
    marks = [r for r in rows if r["bus"] == "MARK"]
    frames = [r for r in rows if r["bus"] != "MARK"]

    # Markers: one per step plus "end", at the scheduled step starts.
    labels = [s["label"] for s in SCENARIO["steps"]] + ["end"]
    assert [m["data"] for m in marks] == labels
    starts = [0.0, 0.5, 0.9, 1.2]
    for m, start in zip(marks, starts):
        assert abs(float(m["time_s"]) - (t_start + start)) < TOL_S, m

    # Frames: same set, same order per stream, data per step, within 2 ms.
    want = expected_frames(SCENARIO, t_start)
    assert len(frames) == len(want) == stats["sent"] == 12 + 48
    assert skipped == {}
    for key in (("P", 913), ("D", 0x1ABCDE)):
        got = [r for r in frames if (r["bus"], int(r["id"])) == key]
        exp = [w for w in want if (w[1], w[2]) == key]
        assert len(got) == len(exp)
        for r, (t, bus, frame_id, data) in zip(got, exp):
            assert r["data"] == data
            assert int(r["dlc"]) == len(data) // 2
            assert r["ext"] == ("1" if frame_id > 0x7FF else "0")
            assert abs(float(r["time_s"]) - t) < TOL_S, (r, t)

    assert log["counts"][("P", 913)] == 12
    assert log["counts"][("D", 0x1ABCDE)] == 48

    # Row format: monotonic times with 6 decimals, quoted MARK labels.
    lines = out.read_text(encoding="utf-8").splitlines()
    assert lines[0] == "time_s,bus,id,ext,dlc,data"
    times = [float(line.split(",")[0]) for line in lines[1:]]
    assert times == sorted(times)
    for line in lines[1:]:
        assert re.match(r"^\d+\.\d{6},(P|D),\d+,[01],[0-8],[0-9A-F]*$", line) or \
            re.match(r'^\d+\.\d{6},MARK,,,,".*"$', line), line
    assert '"third, ""quoted"""' in out.read_text(encoding="utf-8")


@pytest.mark.parametrize("bus_name", ["P", "D"])
def test_stimulus_one_bus(bus_name):
    """Steps that set streams on the bus left out still play to the end."""
    spec = channels()[0]
    stim = capture.open_bus(spec, 500000)
    cap = capture.open_bus(spec, 500000)
    try:
        stats = stimulus.play(SCENARIO, {bus_name: stim}, out=lambda *_: None)
        got = []
        while (msg := cap.recv(timeout=0.1)) is not None:
            got.append(bytes(msg.data).hex().upper())
    finally:
        stim.shutdown()
        cap.shutdown()
    want = [w[3] for w in expected_frames(SCENARIO, 0.0) if w[1] == bus_name]
    assert got == want and stats["sent"] == len(want)


def test_stop_reads_d_longer_than_p(tmp_path):
    p_spec, d_spec = channels()
    cap = {"P": capture.open_bus(p_spec, 500000, listen_only=True),
           "D": capture.open_bus(d_spec, 1000000, listen_only=True)}
    tx = {"P": capture.open_bus(p_spec, 500000), "D": capture.open_bus(d_spec, 1000000)}
    log = capture.open_log(tmp_path / "stop.csv")
    stop, threads, _ = capture.start_readers(cap, log)
    try:
        stop.set()
        time.sleep(0.02)
        for name, bus in tx.items():
            bus.send(can.Message(arbitration_id=390, data=b"\x01", is_extended_id=False))
        capture.stop_readers(stop, threads)
    finally:
        for bus in list(cap.values()) + list(tx.values()):
            bus.shutdown()
        capture.close_log(log)
    assert log["counts"] == {("D", 390): 1}


def test_stdin_closed(monkeypatch):
    monkeypatch.setattr(sys, "stdin", None)
    assert capture.start_stdin_reader(print) is None


def test_stimulus_wait_steps_need_a_terminal(tmp_path):
    import yaml

    scen = tmp_path / "s.yaml"
    scen.write_text(yaml.safe_dump(SCENARIO), encoding="utf-8")
    with pytest.raises(SystemExit) as e:
        stimulus.main([str(scen), "--powertrain", channels()[0]], stdin=io.StringIO(""))
    assert e.value.code == 2


def test_cli_capture_and_stimulus_together(tmp_path):
    """capture.py and stimulus.py --log in one process (the virtual bus is
    per-process), each writing its own CSV and sidecar."""
    import yaml

    p_spec, d_spec = channels()
    scen = tmp_path / "s.yaml"
    scen.write_text(yaml.safe_dump(SCENARIO), encoding="utf-8")
    cap_csv = tmp_path / "cap.csv"
    stim_csv = tmp_path / "stim.csv"

    result = {}

    def run_capture():
        result["rc"] = capture.main(
            ["--powertrain", p_spec, "--data", d_spec, "--firmware", "mbed",
             "--commit", "abc123", "--notes", "test", "-o", str(cap_csv),
             "--duration", "2.0"],
            stdin=io.StringIO("operator mark\n"))

    t = threading.Thread(target=run_capture)
    t.start()
    deadline = time.monotonic() + 2
    while not cap_csv.exists() and time.monotonic() < deadline:
        time.sleep(0.01)
    time.sleep(0.1)

    rc = stimulus.main([str(scen), "--powertrain", p_spec, "--data", d_spec,
                        "--no-wait", "--log", str(stim_csv), "--firmware", "hal",
                        "--setup", "bench"],
                       stdin=io.StringIO(""))
    t.join(timeout=5)
    assert rc == 0 and result["rc"] == 0

    cap_rows = read_rows(cap_csv)
    stim_rows = read_rows(stim_csv)
    # capture.py saw every frame stimulus sent, with the same payloads in order.
    cap_frames = [(r["bus"], r["id"], r["data"]) for r in cap_rows if r["bus"] != "MARK"]
    stim_frames = [(r["bus"], r["id"], r["data"]) for r in stim_rows if r["bus"] != "MARK"]
    assert len(stim_frames) == 60
    assert sorted(cap_frames) == sorted(stim_frames)
    for key in (("P", "913"), ("D", str(0x1ABCDE))):
        assert [f for f in cap_frames if f[:2] == key] == [f for f in stim_frames if f[:2] == key]
    assert [r["data"] for r in cap_rows if r["bus"] == "MARK"] == ["operator mark"]
    assert [r["data"] for r in stim_rows if r["bus"] == "MARK"] == \
        [s["label"] for s in SCENARIO["steps"]] + ["end"]

    meta = json.loads(capture.sidecar_path(cap_csv).read_text())
    assert set(meta) == {"firmware", "commit", "date", "adapter", "setup", "notes"}
    assert meta["firmware"] == "mbed" and meta["commit"] == "abc123"
    assert meta["setup"] == "bench" and meta["notes"] == "test"
    assert p_spec in meta["adapter"] and d_spec in meta["adapter"]
    meta = json.loads(capture.sidecar_path(stim_csv).read_text())
    assert meta["firmware"] == "hal" and "s.yaml" in meta["notes"]


def test_capture_prints_counts(tmp_path, capsys):
    p_spec, _ = channels()
    sender = capture.open_bus(p_spec, 500000)
    out = tmp_path / "c.csv"

    def send_some():
        time.sleep(0.3)
        for i in range(5):
            sender.send(can.Message(arbitration_id=390, data=bytes([i] * 8), is_extended_id=False))
            time.sleep(0.01)

    t = threading.Thread(target=send_some)
    t.start()
    capture.main(["--powertrain", p_spec, "--firmware", "hal", "-o", str(out),
                  "--duration", "0.6"], stdin=io.StringIO(""))
    t.join()
    sender.shutdown()
    text = capsys.readouterr().out
    assert re.search(r"P: 5 frames", text)
    assert re.search(r"390 \(0x186\)\s+5", text)
    rows = read_rows(out)
    assert [r["data"] for r in rows] == [bytes([i] * 8).hex().upper() for i in range(5)]


dbc_missing = not stimulus.DEFAULT_DBC.exists()


@pytest.mark.skipif(dbc_missing, reason="fs-4-ref DBC not checked out")
def test_dbc_loads_with_vdm_frames():
    db = stimulus.load_dbc(stimulus.DEFAULT_DBC)
    assert db.get_message_by_name("VDM_GPS_LAT_LONG").is_extended_frame
    assert db.get_message_by_frame_id(913).name == "BATT_TPDO_STATUS"


@pytest.mark.skipif(dbc_missing, reason="fs-4-ref DBC not checked out")
def test_baseline_sends_what_the_vcu_reads():
    """Decode the baseline frames the way vcu/main.cpp does."""
    db = stimulus.load_dbc(stimulus.DEFAULT_DBC)
    scenario = stimulus.load_scenario(HERE / "scenarios" / "baseline.yaml")
    stimulus.check_scenario(scenario, db)
    streams = {s["id"]: s for s in stimulus.make_streams(scenario, db)}
    assert {s["bus"] for i, s in streams.items() if i in (913, 1154, 1216, 1666)} == {"P"}
    assert {s["bus"] for i, s in streams.items() if i in (421, 422, 423, 424, 432)} == {"D"}

    seen = {}
    for step in scenario["steps"]:
        stimulus.apply_step(list(streams.values()), step)
        d = {i: s["data"] for i, s in streams.items()}
        seen[step["label"]] = d
        assert len(d[1216]) == 5 and len(d[421]) == 7 and len(d[432]) == 1

    def precharged(d):
        return bool(d[913][0] & 0b01000000)

    def shutdown_closed(d):
        return bool(d[913][0] & 0b00000100)

    assert not precharged(seen["idle not precharged"])
    assert precharged(seen["precharged shutdown closed"])
    assert shutdown_closed(seen["precharged shutdown closed"])
    assert seen["tray temp 45 C"][1216][1] / 2.0 > 40.0
    assert seen["tray temp normal"][1216][1] / 2.0 <= 40.0
    assert not shutdown_closed(seen["shutdown open"]) and precharged(seen["shutdown open"])
    def rpm(d, i):
        return (d[i][0] + (d[i][1] << 8)) * 0.1

    wheels = seen["wheels 500 rpm"]
    for i in (421, 422, 423, 424):
        assert rpm(wheels, i) == pytest.approx(500)
    assert wheels[1154][0] | (wheels[1154][1] << 8) == 1818
    # Slip as etc/traction_control.cpp computes it.
    slip = seen["wheels rear slip 20 %"]
    front = (rpm(slip, 421) + rpm(slip, 422)) / 2
    rear = (rpm(slip, 423) + rpm(slip, 424)) / 2
    assert (rear - front) / rear == pytest.approx(0.20)
    modes = seen["modes all 1 with wheels 200 rpm"][432][0]
    assert (modes & 3, (modes >> 2) & 3, (modes >> 4) & 3) == (1, 1, 1)
    assert seen["modes regen 3"][432][0] >> 4 & 3 == 3
