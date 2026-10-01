#!/usr/bin/env python3
"""
Runs the VCU firmware on an emulated STM32F446 in Renode and checks what it
does: CAN frames on CAN1, the debug lines on UART4, when the ETC enables and
disables the motor, and how it left the peripheral registers.

    python3 test/renode/run_tests.py
    python3 test/renode/run_tests.py --elf build/release/vcu/vcu.elf
    python3 test/renode/run_tests.py boot apps_deviation

Needs Renode 1.17 (on PATH, or --renode) and the Arm toolchain (gdb and
addr2line are taken from next to the compiler in the ELF's CMake cache, or
PATH). Times are virtual time.

How the checks work
  Each scenario feeds the two APPS inputs as a timeline. A float32 copy of the
  ETC math (same operations, same order as etc_controller.cpp) and a model of
  its two 100 ms timers predict, from that timeline alone, when the motor
  should be enabled, which fault flags are up, the torque and every value in
  the debug line. etc.motor_enabled is watched in emulated RAM, so the real
  enable/disable times are measured, and every CAN frame and debug line is
  compared against the prediction. The 50 ms CAN and 500 ms debug schedules,
  the immediate zero-torque frame, the MBB_Alive counter, frame layout and the
  peripheral register setup are checked too.

What the emulator can't tell you (check these on the car)
  - Clocks: HSE/PLL/over-drive are ready instantly and nothing is clock gated.
    The register audit catches wrong settings, not a crystal that won't start.
  - CAN: frames go out instantly with no ACK, bit timing, error counters or
    bus-off, even if the controller were left in init mode. Mailboxes are
    never busy unless a scenario forces it.
  - ADC: inputs are raw codes, no analog front end, no noise.
  - UART: characters go out instantly, so the baud rate and the TX buffer
    filling up are not exercised.
  - Timing: the CPU runs a flat 180 instructions per us, so loop times are
    close but not exact.
"""

import argparse
import concurrent.futures
import os
import re
import shutil
import struct
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))

MS = 1000  # everything is in us

# One pass of the main loop (16 ADC conversions, plus a CAN send or a debug
# printf when due) is well under this
LOOP = 300
# An input change can take three passes to show: the pass already reading
# samples, the pass that sees the change, and one more because the 100 ms
# timers are only checked once per pass
REACT = 3 * LOOP

SAMPLES = 8
IMPLAUS_TIME = 100 * MS
CLEAR_TIME = 100 * MS
CAN_PERIOD = 50 * MS
DEBUG_PERIOD = 500 * MS

TIM5_CNT = 0x40000C24
CAN1_ESR = 0x40006418
CAN1_BTR = 0x4000641C

DEBUG_LINE = re.compile(
    r"APPS1 (\d+\.\d{3}) V (\d+\.\d{3}) \| APPS2 (\d+\.\d{3}) V (\d+\.\d{3}) \| "
    r"pedal (\d+\.\d{3}) \| torque (\d+) \| dev ([01]) oor ([01]) \| CAN tx err (\d+)( bus-off)?$")

# Renode warnings that are expected: the flash cache bits aren't modelled
ALLOWED_WARNINGS = [
    re.compile(r"Translation cache size"),
    re.compile(r"flash_controller: Unhandled write to offset 0x0\. Unhandled bits: \[(9|10)\]"),
]


# --- float32 model of etc_controller.cpp -------------------------------------

def f32(x):
    return struct.unpack("<f", struct.pack("<f", x))[0]


ADC_SCALE = f32(1.0 / 4095.0)  # (1.0f / ADC_FULL_SCALE)
VREF = f32(3.3)
APPS1_MIN, APPS1_MAX = f32(0.396), f32(1.086)
APPS2_MIN, APPS2_MAX = f32(0.439), f32(1.133)
DEADZONE = f32(0.03)
DEADZONE_SPAN = f32(1.0 - f32(2.0 * DEADZONE))
MAX_DEVIATION = f32(0.10)
MARGIN = f32(0.05)
MAX_TORQUE = 3276


class Etc:
    """What one update_state() computes from steady ADC codes"""

    def __init__(self, code1, code2):
        self.codes = (code1, code2)
        self.v1 = self._average(code1)
        self.v2 = self._average(code2)
        t1 = f32(f32(self.v1 - APPS1_MIN) / f32(APPS1_MAX - APPS1_MIN))
        t2 = f32(f32(self.v2 - APPS2_MIN) / f32(APPS2_MAX - APPS2_MIN))
        self.p1, self.p2 = self._position(t1), self._position(t2)
        self.pedal = f32(f32(self.p1 + self.p2) / 2.0)
        mapped = f32(self.pedal * f32(0.5 + f32(0.5 * self.pedal)))
        self.torque = int(f32(mapped * MAX_TORQUE))
        self.dev = abs(f32(t1 - t2)) > MAX_DEVIATION
        self.oor = (not f32(APPS1_MIN - MARGIN) <= self.v1 <= f32(APPS1_MAX + MARGIN)
                    or not f32(APPS2_MIN - MARGIN) <= self.v2 <= f32(APPS2_MAX + MARGIN))
        self.fault = self.dev or self.oor

    @staticmethod
    def _average(code):
        volts = f32(f32(code * ADC_SCALE) * VREF)
        total = 0.0
        for _ in range(SAMPLES):
            total = f32(total + volts)
        return f32(total / SAMPLES)

    @staticmethod
    def _position(travel):
        p = f32(f32(travel - DEADZONE) / DEADZONE_SPAN)
        return 0.0 if p < 0.0 else 1.0 if p > 1.0 else p

    def debug_fields(self, enabled):
        return [f"{self.v1:.3f}", f"{self.p1:.3f}", f"{self.v2:.3f}", f"{self.p2:.3f}",
                f"{self.pedal:.3f}", str(self.torque if enabled else 0)]


def code(volts):
    return max(0, min(4095, round(volts / 3.3 * 4095)))


def apps(travel1, travel2=None):
    """ADC codes for APPS1/APPS2 at a pedal travel (0..1)"""
    if travel2 is None:
        travel2 = travel1
    return (code(0.396 + travel1 * (1.086 - 0.396)), code(0.439 + travel2 * (1.133 - 0.439)))


class Prediction:
    """Continuous time model of the ETC timers: implausibility after a fault has
    lasted 100 ms (the timer keeps running through clean gaps shorter than the
    100 ms clear time), cleared after 100 ms of clean readings, motor disabled
    at boot until then."""

    def __init__(self, segments, first_update, end):
        # segments: [(start, Etc)] sorted, first one at 0
        self.changes = []  # (t, enabled, dev_flag, oor_flag)
        active, dev_seen, oor_seen = True, False, False
        fault_since = clean_since = None
        self._note(first_update, active, dev_seen, oor_seen)

        bounds = [s for s, _ in segments[1:]] + [end]
        for (start, etc), stop in zip(segments, bounds):
            t = max(start, first_update)
            while t < stop:
                if etc.fault:
                    clean_since = None
                    dev_seen = dev_seen or etc.dev
                    oor_seen = oor_seen or etc.oor
                    self._note(t, active, dev_seen, oor_seen)
                    if fault_since is None:
                        fault_since = t
                    trip = max(fault_since + IMPLAUS_TIME, t)
                    if not active and trip < stop:
                        active = True
                        self._note(trip, active, dev_seen, oor_seen)
                    t = stop
                else:
                    if clean_since is None:
                        clean_since = t
                    clear = clean_since + CLEAR_TIME
                    if clear < stop and (active or fault_since is not None):
                        active, dev_seen, oor_seen = False, False, False
                        fault_since = clean_since = None
                        self._note(clear, active, dev_seen, oor_seen)
                        t = clear
                    else:
                        t = stop

    def _note(self, t, active, dev_seen, oor_seen):
        state = (not active, active and dev_seen, active and oor_seen)
        if not self.changes or self.changes[-1][1:] != state:
            self.changes.append((t,) + state)

    def at(self, t):
        state = self.changes[0]
        for change in self.changes:
            if change[0] <= t:
                state = change
        return state[1:]

    def enable_changes(self):
        out = []
        for t, enabled, _, _ in self.changes:
            if not out or out[-1][1] != enabled:
                out.append((t, enabled))
        return out[1:] if out and not out[0][1] else out  # boot starts disabled


# --- running Renode ------------------------------------------------------------

class Frame:
    def __init__(self, t, can_id, data):
        self.t = t
        self.id = can_id
        self.data = data
        self.status = None  # the 0x193 sent with a 0x186

    @property
    def torque(self):
        return int.from_bytes(self.data[0:2], "little", signed=True)

    @property
    def max_speed(self):
        return int.from_bytes(self.data[2:4], "little", signed=True)

    @property
    def power_ready(self):
        return (self.data[4] >> 3) & 1

    @property
    def alive(self):
        return self.data[5] & 0x0F

    @property
    def deviation(self):
        return (self.data[0] >> 6) & 1

    @property
    def out_of_range(self):
        return (self.data[0] >> 4) & 1

    def __repr__(self):
        return f"{self.t / MS:.3f}ms {self.id:03X} {self.data.hex()}"


class Result:
    pass


class Scenario:
    """Steps:
       ('apps', (code1, code2))  change the ADC inputs (time is recorded)
       ('wait', seconds)         run the emulation
       ('mark', label)           record the time
       ('cnt', label)            record the time and read TIM5->CNT
       ('cmd', monitor command)  anything else"""

    def __init__(self, name, start, steps, checks=(), hooks=(), blocked=(), can_errors=None,
                 general=True, registers=False):
        self.name = name
        self.start = start
        self.steps = steps
        self.checks = list(checks)
        self.hooks = list(hooks)        # monitor commands run before the firmware starts
        self.blocked = list(blocked)    # (from, to) us where CAN sends are made to fail
        self.can_errors = can_errors    # t -> (TEC, bus_off) shown in the debug line
        self.general = general
        self.registers = registers
        self.end = int(round(sum(s[1] for s in steps if s[0] == "wait") * 1e6))

    def script(self, elf, out, enabled_address):
        lines = [
            f"include @{HERE}/STM32F4_ADC_Patched.cs",
            f"include @{HERE}/Recorders.cs",
            'mach create "vcu"',
            f"machine LoadPlatformDescription @{HERE}/vcu.repl",
            f"sysbus LoadELF @{elf}",
            f"can1 RecordFrames @{out}/can.txt",
            f"uart4 RecordLines @{out}/uart.txt",
            f"sysbus RecordByteChanges {enabled_address:#x} @{out}/enabled.txt",
            # Renode's bxCAN model only acks init mode with SLEEP clear. ST's
            # HAL sets INRQ before clearing SLEEP and that works on the chip
            # (the Mbed build used the same sequence on this board).
            "can1 WriteDoubleWord 0x0 0x00010000",
            # BTR only reads back in init mode, so grab it as HAL_CAN_Start runs
            f'cpu RecordWordAt `sysbus GetSymbolAddress "HAL_CAN_Start"` {CAN1_BTR:#x} @{out}/btr.txt',
        ]
        lines += self.hooks
        lines += apps_commands(self.start)
        n = 0
        for step in self.steps:
            kind, arg = step
            if kind == "apps":
                lines += apps_commands(arg)
                lines.append(f'machine WriteMarker @{out}/marks.txt "in{n}"')
                n += 1
            elif kind == "wait":
                lines.append(f'emulation RunFor "{arg:.6f}"')
            elif kind == "mark":
                lines.append(f'machine WriteMarker @{out}/marks.txt "{arg}"')
            elif kind == "cnt":
                lines.append(f'machine WriteMarker @{out}/marks.txt "{arg}"')
                lines += [f'echo "CNT {arg}"', f"sysbus ReadDoubleWord {TIM5_CNT:#x}"]
            elif kind == "cmd":
                lines.append(arg)
        lines.append("uart4 FlushLine")
        if self.registers:
            for name, address, _, _ in REGISTERS:
                lines += [f'echo "REG {name}"', f"sysbus ReadDoubleWord {address:#x}"]
        lines += ['echo "END_OF_SCRIPT"', "quit"]
        return "\n".join(lines) + "\n"


def apps_commands(codes):
    return [f"adc1 FeedSample {codes[0]} 11 -1", f"adc1 FeedSample {codes[1]} 12 -1"]


def echoed(stdout, prefix):
    """{label: value} for 'echo "<prefix> label"' followed by a ReadDoubleWord"""
    values = {}
    lines = stdout.splitlines()
    for i, line in enumerate(lines):
        if line.startswith(prefix + " "):
            for after in lines[i + 1:i + 40]:
                if re.fullmatch(r"0x[0-9A-Fa-f]{8}", after.strip()):
                    values[line[len(prefix) + 1:].strip()] = int(after.strip(), 16)
                    break
    return values


def read_lines(path):
    return open(path).read().splitlines() if os.path.exists(path) else []


def run_renode(tools, scenario, out):
    script = os.path.join(out, "run.resc")
    with open(script, "w") as f:
        f.write(scenario.script(tools.elf, out, tools.enabled_address))

    timeout = 120 + 60 * scenario.end / 1e6
    proc = subprocess.run([tools.renode, "--console", "--disable-gui", "--plain", script],
                          stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=timeout)
    r = Result()
    r.stdout = proc.stdout + proc.stderr
    r.returncode = proc.returncode

    r.can = []
    for line in read_lines(os.path.join(out, "can.txt")):
        t, can_id, data = line.split()
        r.can.append(Frame(int(t), int(can_id, 16), bytes.fromhex(data)))
    r.uart = []
    for line in read_lines(os.path.join(out, "uart.txt")):
        t, _, text = line.partition(" ")
        r.uart.append((int(t), text))
    r.enabled = [(int(t), int(v)) for t, v in (line.split() for line in read_lines(os.path.join(out, "enabled.txt")))]
    r.marks = {label: int(t) for t, label in (line.split(maxsplit=1) for line in read_lines(os.path.join(out, "marks.txt")))}
    r.btr = [int(v, 16) for _, v in (line.split() for line in read_lines(os.path.join(out, "btr.txt")))]
    r.cnt = echoed(r.stdout, "CNT")
    r.registers = echoed(r.stdout, "REG")

    # Pair every 0x186 with the 0x193 sent right after it
    r.throttle, r.status, r.unpaired = [], [], []
    i = 0
    while i < len(r.can):
        f = r.can[i]
        if f.id == 0x186 and i + 1 < len(r.can) and r.can[i + 1].id == 0x193 and r.can[i + 1].t - f.t <= 200:
            f.status = r.can[i + 1]
            r.throttle.append(f)
            r.status.append(r.can[i + 1])
            i += 2
        else:
            r.unpaired.append(f)
            i += 1

    hello = [t for t, line in r.uart if line == "Hello World!!"]
    r.hello = hello[0] if hello else None
    r.debug = [(t, line) for t, line in r.uart if line.startswith("APPS1")]
    return r


# --- checks ------------------------------------------------------------------

class Checker:
    def __init__(self):
        self.failures = []
        self.passed = 0

    def check(self, condition, message):
        if condition:
            self.passed += 1
        else:
            self.failures.append(message)
        return bool(condition)


def check_run(c, s, r):
    """Renode itself: ran to the end, no errors, no unexpected warnings"""
    c.check(r.returncode == 0, f"Renode exit code {r.returncode}")
    c.check("END_OF_SCRIPT" in r.stdout, "script didn't run to the end")
    for bad in ("Errors during compilation", "There was an error", "[ERROR]"):
        c.check(bad not in r.stdout, f"Renode reported '{bad}'")
    warnings = sorted({line.split("[WARNING]", 1)[1].strip() for line in r.stdout.splitlines()
                       if "[WARNING]" in line and not any(p.search(line) for p in ALLOWED_WARNINGS)})
    c.check(not warnings, f"unexpected Renode warnings: {warnings[:5]}")


def segments_of(s, r):
    segments = [(0, Etc(*s.start))]
    n = 0
    for kind, arg in s.steps:
        if kind == "apps":
            segments.append((r.marks[f"in{n}"], Etc(*arg)))
            n += 1
    return segments


def segment_at(segments, t):
    current = segments[0][1]
    for start, etc in segments:
        if start <= t:
            current = etc
    return current


def steady(segments, t):
    """Inputs at t, if they haven't changed in the last REACT us, else None"""
    for start, _ in segments[1:]:
        if t - REACT < start <= t:
            return None
    return segment_at(segments, t)


def enabled_at(r, t):
    state = 0
    for change_t, value in r.enabled:
        if change_t <= t:
            state = value
    return state


def check_general(c, s, r):
    segments = segments_of(s, r)
    if not c.check(r.hello is not None, "no 'Hello World!!'"):
        return
    c.check(r.uart[0][1] == "Hello World!!", f"first UART line: {r.uart[0]}")
    for t, line in r.uart:
        c.check(line == "Hello World!!" or DEBUG_LINE.match(line), f"unexpected UART output: {line!r}")
    c.check(not r.unpaired, f"CAN frames without a partner: {r.unpaired[:3]}")

    # Frame layout
    for f in r.throttle:
        c.check(len(f.data) == 8 and len(f.status.data) == 8, f"DLC is not 8: {f}")
        c.check(f.max_speed == 7500, f"MaxSpeed is not 7500: {f}")
        c.check(f.data[4] & 0xF7 == 0x01, f"0x186 byte 4 (Forward, PowerReady, unused) wrong: {f}")
        c.check(f.data[5] & 0xF0 == 0 and f.data[6:] == b"\0\0", f"0x186 unused bits set: {f}")
        c.check(0 <= f.torque <= MAX_TORQUE, f"torque out of range: {f}")
        c.check(f.power_ready or f.torque == 0, f"torque without PowerReady: {f}")
        st = f.status
        # READY_TO_DRIVE and MOTOR_ENABLED are always 1, as the onboarding
        # assignment asks (Mbed main.cpp). The DBC says MOTOR_ENABLED should
        # follow the implausibilities, this firmware doesn't do that.
        c.check(st.data[0] & 0b10101111 == 0b11 and st.data[1:] == bytes(7), f"0x193 layout wrong: {st}")
        c.check(not (st.deviation or st.out_of_range) or not f.power_ready,
                f"fault flag set while the motor is enabled: {f} {st}")

    # MBB_Alive goes up by one per frame actually sent
    c.check(r.throttle and r.throttle[0].alive == 1, "MBB_Alive doesn't start at 1")
    for a, b in zip(r.throttle, r.throttle[1:]):
        if not c.check(b.alive == (a.alive + 1) % 16, f"MBB_Alive jumped: {a} -> {b}"):
            break

    # Enable/disable times against the model
    prediction = Prediction(segments, r.hello, s.end)
    # The motor starts disabled. Only count real changes from there (the debug
    # build's constructor also writes the initial 0, the release build doesn't)
    measured, state = [], False
    for t, v in r.enabled:
        if bool(v) != state:
            measured.append((t, bool(v)))
            state = bool(v)
    expected = prediction.enable_changes()
    c.check(len(measured) == len(expected),
            f"motor enable changes {[(t / MS, v) for t, v in measured]}, "
            f"expected {[(round(t / MS, 3), v) for t, v in expected]}")
    for (t, v), (et, ev) in zip(measured, expected):
        c.check(v == ev and et <= t <= et + REACT,
                f"motor {'enabled' if v else 'disabled'} at {t / MS:.3f} ms, expected "
                f"{'enabled' if ev else 'disabled'} at {et / MS:.3f}-{(et + REACT) / MS:.3f} ms")

    # Every frame agrees with the measured state and the model
    checked_torque = 0
    for f in r.throttle:
        c.check(f.power_ready == enabled_at(r, f.t), f"PowerReady disagrees with motor_enabled: {f}")
        now, before = prediction.at(f.t), prediction.at(f.t - REACT)
        if now == before:
            c.check((f.status.deviation, f.status.out_of_range) == (now[1], now[2]),
                    f"fault flags {f.status}, expected dev {now[1]:d} oor {now[2]:d}")
        etc = steady(segments, f.t)
        if f.power_ready and etc:
            checked_torque += 1
            c.check(f.torque == etc.torque, f"torque {f.torque}, expected {etc.torque} for {etc.codes}: {f}")
    r.checked_torque = checked_torque

    # Zero torque goes out right away when the motor gets disabled, not at the
    # next 50 ms slot
    immediate = set()
    for t, v in measured:
        if v or any(a - LOOP <= t <= b + LOOP for a, b in s.blocked):
            continue
        sent = [f for f in r.throttle if t <= f.t <= t + LOOP and not f.power_ready]
        if c.check(sent, f"no zero torque frame right after the motor was disabled at {t / MS:.3f} ms"):
            immediate.add(id(sent[0]))

    # 50 ms schedule: one frame per slot after the first timestamp, no extras
    periodic = [f for f in r.throttle if id(f) not in immediate]
    k = 1
    for f in periodic:
        while r.hello + k * CAN_PERIOD + LOOP < f.t:
            slot = r.hello + k * CAN_PERIOD
            if not any(a - LOOP <= slot <= b + LOOP for a, b in s.blocked):
                c.check(False, f"no CAN frame in the {slot / MS:.3f} ms slot")
            k += 1
        slot = r.hello + k * CAN_PERIOD
        if c.check(slot <= f.t <= slot + LOOP, f"CAN frame outside its 50 ms slot ({slot / MS:.3f} ms): {f}"):
            k += 1
    while r.hello + k * CAN_PERIOD + LOOP <= s.end:
        c.check(False, f"CAN stopped, nothing in the {(r.hello + k * CAN_PERIOD) / MS:.3f} ms slot")
        k += 1

    # 500 ms debug lines, each value against the model
    slots = int((s.end - LOOP - r.hello) // DEBUG_PERIOD)
    c.check(len(r.debug) == slots, f"{len(r.debug)} debug lines, expected {slots}")
    for i, (t, line) in enumerate(r.debug):
        slot = r.hello + (i + 1) * DEBUG_PERIOD
        c.check(slot <= t <= slot + LOOP, f"debug line {i + 1} at {t / MS:.3f} ms, expected right after {slot / MS:.3f}")
        m = DEBUG_LINE.match(line)
        if not m:
            continue
        etc = steady(segments, t)
        if etc:
            c.check(list(m.groups()[:6]) == etc.debug_fields(enabled_at(r, t)),
                    f"debug line {line!r}, expected values {etc.debug_fields(enabled_at(r, t))}")
        now, before = prediction.at(t), prediction.at(t - REACT)
        if now == before:
            c.check((m.group(7), m.group(8)) == (str(int(now[1])), str(int(now[2]))),
                    f"debug line flags wrong: {line!r}, expected dev {now[1]:d} oor {now[2]:d}")
        tec, bus_off = s.can_errors(t) if s.can_errors else (0, False)
        c.check(int(m.group(9)) == tec and bool(m.group(10)) == bus_off,
                f"debug line CAN error part wrong: {line!r}, expected {tec}{' bus-off' if bus_off else ''}")
    return prediction


# Expected peripheral setup after boot: (name, address, mask, value). Values
# come from board.c / adc.c / can.c / console.c / timebase.c and RM0390.
REGISTERS = [
    ("RCC_CR", 0x40023800, 1 << 16 | 1 << 24, 1 << 16 | 1 << 24),                      # HSEON, PLLON
    ("RCC_PLLCFGR", 0x40023804, 0x7F437FFF, 2 << 28 | 8 << 24 | 1 << 22 | 180 << 6 | 12),  # R2 Q8 HSE P2 N180 M12
    ("RCC_CFGR", 0x40023808, 0xFCF3, 0b100 << 13 | 0b101 << 10 | 2),                  # APB2 /2, APB1 /4, PLL
    ("RCC_AHB1ENR", 0x40023830, 0x6, 0x6),                                            # GPIOB, GPIOC
    ("RCC_APB1ENR", 0x40023840, 1 << 3 | 1 << 19 | 1 << 25 | 1 << 28,
     1 << 3 | 1 << 19 | 1 << 25 | 1 << 28),                                            # TIM5, UART4, CAN1, PWR
    ("RCC_APB2ENR", 0x40023844, 1 << 8 | 1 << 14, 1 << 8 | 1 << 14),                  # ADC1, SYSCFG
    ("FLASH_ACR", 0x40023C00, 0x10F, 5),                                              # 5 WS, prefetch off
    ("PWR_CR", 0x40007000, 0x3C000, 0x3C000),                                         # VOS scale 1, over-drive
    ("GPIOB_MODER", 0x40020400, 0xF << 16, 0b1010 << 16),                             # PB8/PB9 alternate
    ("GPIOB_PUPDR", 0x4002040C, 0xF << 16, 0b0101 << 16),                             # pull-ups
    ("GPIOB_AFRH", 0x40020424, 0xFF, 0x99),                                           # AF9 CAN1
    ("GPIOC_MODER", 0x40020800, 0xF << 20 | 0xF << 2, 0b1010 << 20 | 0b1111 << 2),    # PC10/11 AF, PC1/2 analog
    ("GPIOC_PUPDR", 0x4002080C, 0xF << 20 | 0xF << 2, 0b0101 << 20),                  # UART pull-ups, ADC none
    ("GPIOC_AFRH", 0x40020824, 0xFF00, 0x8800),                                       # AF8 UART4
    ("CAN1_MCR", 0x40006400, 0xFF, 0x00),                                             # running, no ABOM/NART/TXFP
    ("CAN1_MSR", 0x40006404, 0x3, 0x0),                                               # not in init or sleep
    ("CAN1_FMR", 0x40006600, 0x3F01, 14 << 8),                                        # CAN2 filters from bank 14
    ("CAN1_FM1R", 0x40006604, 0x1, 0x0),                                              # bank 0 mask mode
    ("CAN1_FS1R", 0x4000660C, 0x1, 0x1),                                              # 32-bit
    ("CAN1_FFA1R", 0x40006614, 0x1, 0x0),                                             # FIFO0
    ("CAN1_FA1R", 0x4000661C, 0x1, 0x1),                                              # active
    ("CAN1_F0R2", 0x40006644, 0xFFFFFFFF, 0x0),                                       # mask 0 = accept all
    ("UART4_BRR", 0x40004C08, 0xFFFF, 0x187),                                         # 115200 at 45 MHz
    ("UART4_CR1", 0x40004C0C, 0xB40C, 0x200C),                                        # UE TE RE, 8N1, OVER16
    ("UART4_CR2", 0x40004C10, 0x3000, 0x0),                                           # 1 stop bit
    ("TIM5_CR1", 0x40000C00, 0x91, 0x1),                                              # counting up
    ("TIM5_PSC", 0x40000C28, 0xFFFF, 89),                                             # 90 MHz / 90
    ("TIM5_ARR", 0x40000C2C, 0xFFFFFFFF, 0xFFFFFFFF),
    ("ADC1_CR1", 0x40012004, 0x3000100, 0x0),                                         # 12-bit, no scan
    ("ADC1_CR2", 0x40012008, 0x903, 0x1),                                             # on, single, right aligned
    ("ADC1_SMPR1", 0x4001200C, 0x3F << 3, 0b011011 << 3),                             # ch11, ch12: 56 cycles
    ("ADC1_SQR1", 0x4001202C, 0xF << 20, 0x0),                                        # 1 conversion
    ("ADC_CCR", 0x40012304, 0x3001F, 1 << 16),                                        # ADCCLK = PCLK2 / 4
    ("DBGMCU_APB1_FZ", 0xE0042008, 1 << 3, 1 << 3),                                   # TIM5 stops in debug halt
    ("NVIC_ISER1", 0xE000E104, 1 << 20, 1 << 20),                                     # UART4 IRQ on
    ("NVIC_IPR13", 0xE000E434, 0xFF, 5 << 4),                                         # UART4 priority 5
    ("SCB_SHPR3", 0xE000ED20, 0xFF000000, 0xF0000000),                                # SysTick priority 15
    ("SYST_CSR", 0xE000E010, 0x7, 0x7),
    ("SYST_RVR", 0xE000E014, 0xFFFFFF, 179999),                                       # 1 kHz at 180 MHz
    ("SCB_CPACR", 0xE000ED88, 0xF << 20, 0xF << 20),                                  # FPU on
    ("SCB_AIRCR", 0xE000ED0C, 0x700, 0x300),                                          # 4 bits preemption
]


def check_registers(c, s, r):
    for name, _, mask, value in REGISTERS:
        got = r.registers.get(name)
        if c.check(got is not None, f"couldn't read {name}"):
            c.check(got & mask == value, f"{name} = {got:#010x}, expected {value:#x} under mask {mask:#x}")
    c.check(r.btr == [0x014B0004], f"CAN1 BTR {[hex(b) for b in r.btr]}, expected 0x14b0004 "
                                   "(500 kbit/s, 12+5 tq, SJW 2, same as Mbed)")


# --- scenarios -----------------------------------------------------------------

def limit_codes():
    """ADC codes right at the edges of the out of range and deviation checks"""
    rest, full = apps(0.0), apps(1.0)

    def last_good(start, step, make):
        c = start
        while not Etc(*make(c + step)).fault:
            c += step
        assert not Etc(*make(c)).fault and Etc(*make(c + step)).fault
        return c, c + step

    cases = [
        ("apps1_low", lambda c: (c, rest[1]), rest[0], -1),
        ("apps1_high", lambda c: (c, full[1]), full[0], 1),
        ("apps2_low", lambda c: (rest[0], c), rest[1], -1),
        ("apps2_high", lambda c: (full[0], c), full[1], 1),
    ]
    limits = []
    for name, make, start, step in cases:
        good, bad = last_good(start, step, make)
        assert Etc(*make(bad)).oor and not Etc(*make(bad)).dev
        limits.append((name, make(good), make(bad), make(start)))
    half = apps(0.5)
    good, bad = last_good(half[1], 1, lambda c: (half[0], c))
    assert Etc(half[0], bad).dev and not Etc(half[0], bad).oor
    return limits, ((half[0], good), (half[0], bad))


def scenarios():
    half = apps(0.5)
    result = []

    result.append(Scenario("boot", half, [("wait", 1.2)], registers=True,
                           checks=[check_registers]))

    # Pedal map across the travel, the deadzones, a bit past full travel, and
    # unequal sensors (less than 10% apart) to check the averaging
    points = [apps(t) for t in (0.0, 0.02, 0.05, 0.25, 0.5, 0.75, 0.95, 1.0, 1.05)]
    points += [apps(0.30, 0.38), apps(0.62, 0.55), apps(0.5, 0.59), apps(0.07, 0.0)]
    steps = [("wait", 0.15)]
    for p in points:
        assert not Etc(*p).fault, p
        steps += [("apps", p), ("wait", 0.2)]

    def torque_check(c, s, r):
        c.check(r.checked_torque >= 3 * len(points), f"only {r.checked_torque} frames had their torque checked")
        seen = {f.torque for f in r.throttle}
        for p in points:
            c.check(Etc(*p).torque in seen, f"never saw the torque for {p} ({Etc(*p).torque})")

    result.append(Scenario("torque_map", apps(0.0), steps, checks=[torque_check]))

    # Faults, injected off the 50 ms grid so a missing immediate send would show
    def fault_scenario(name, start, fault, clean):
        assert Etc(*fault).fault and not Etc(*start).fault and not Etc(*clean).fault
        return Scenario(name, start, [("wait", 0.525), ("apps", fault), ("wait", 0.5),
                                      ("apps", clean), ("wait", 0.475)])

    # Only one sensor moves in the out of range cases, and they stay under 10%
    # apart. Each update reads APPS1 8 times then APPS2 8 times, so if both
    # jumped at once one update could see new APPS1 with old APPS2, which is a
    # deviation and the ETC would latch that flag too.
    result.append(fault_scenario("apps_deviation", half, apps(0.5, 0.62), apps(0.3)))
    result.append(fault_scenario("apps1_out_of_range_high", apps(1.0), (code(1.14), apps(1.0)[1]), apps(1.0)))
    result.append(fault_scenario("apps2_out_of_range_low", apps(0.0), (apps(0.0)[0], code(0.38)), apps(0.0)))
    result.append(fault_scenario("apps1_unplugged", half, (0, half[1]), apps(0.3)))

    # Right at the limits: last code that must not trip, then first that must
    limits, deviation = limit_codes()
    steps = [("wait", 0.15)]
    for name, good, bad, back in limits:
        steps += [("apps", good), ("wait", 0.3), ("apps", bad), ("wait", 0.25), ("apps", back), ("wait", 0.25)]
    result.append(Scenario("out_of_range_limits", apps(0.0), steps))
    good, bad = deviation
    result.append(Scenario("deviation_limit", half,
                           [("wait", 0.15), ("apps", good), ("wait", 0.3), ("apps", bad), ("wait", 0.25),
                            ("apps", half), ("wait", 0.3)]))

    result.append(Scenario("short_glitch", half,
                           [("wait", 0.525), ("apps", apps(0.5, 0.7)), ("wait", 0.06), ("apps", half), ("wait", 0.6)]))

    # Fault 60 ms, clean 50 ms (less than the 100 ms clear time), fault again:
    # the fault timer keeps running, so it trips as soon as the second one
    # starts, 110 ms after the first
    result.append(Scenario("fault_spans_gap", half,
                           [("wait", 0.525), ("apps", apps(0.5, 0.7)), ("wait", 0.06), ("apps", half),
                            ("wait", 0.05), ("apps", apps(0.5, 0.7)), ("wait", 0.3), ("apps", half), ("wait", 0.3)]))

    # Powered up with APPS1 unplugged: never enabled until it's plugged in
    result.append(Scenario("boot_with_fault", (0, half[1]),
                           [("wait", 0.625), ("apps", half), ("wait", 0.4)]))

    # TIM5 jumped near the top right after timebase_init(), before the first
    # timestamp, so it wraps 0.4 s in. A fault starts 45 ms before the wrap.
    def wrap_check(c, s, r):
        c.check(r.cnt.get("before", 0) >= 0xFFF00000, f"TIM5 not near the top before the wrap: {r.cnt}")
        c.check(r.cnt.get("after", 0xFFFFFFFF) < 0x00100000, f"TIM5 didn't wrap: {r.cnt}")

    wrap_at = 0.4
    result.append(Scenario(
        "timer_wrap", half,
        [("wait", 0.355), ("cnt", "before"), ("apps", apps(0.5, 0.7)), ("wait", 0.3), ("cnt", "after"),
         ("apps", half), ("wait", 0.445)],
        hooks=['cpu AddHook `sysbus GetSymbolAddress "console_init"` '
               f'"machine.SystemBus.WriteDoubleWord({TIM5_CNT:#x}, {0x100000000 - int(wrap_at * 1e6):#x})"'],
        checks=[wrap_check]))

    # All TX mailboxes full for 120 ms: can_write fails, nothing goes out,
    # MBB_Alive must not move, sending picks up again after
    blocked = (300 * MS, 420 * MS)

    def blocked_check(c, s, r):
        c.check(not [f for f in r.throttle if blocked[0] + LOOP < f.t < blocked[1]], "sent CAN while mailboxes were full")
        c.check([f for f in r.throttle if f.t > blocked[1]], "CAN didn't come back")

    result.append(Scenario(
        "can_tx_blocked", half, [("wait", 0.7)],
        hooks=['cpu ReturnBetween `sysbus GetSymbolAddress "HAL_CAN_GetTxMailboxesFreeLevel"` '
               f'0 {blocked[0]} {blocked[1]}'],
        blocked=[blocked], checks=[blocked_check]))

    # Debug line's CAN error part: TEC set to 128 at 0.2 s, bus-off reported
    # from 0.7 to 1.2 s (the model can't go bus-off, so can_bus_off is forced)
    result.append(Scenario(
        "can_error_display", half,
        [("wait", 0.2), ("cmd", f"sysbus WriteDoubleWord {CAN1_ESR:#x} 0x00800000"), ("wait", 1.1)],
        hooks=['cpu ReturnBetween `sysbus GetSymbolAddress "can_bus_off"` 1 700000 1200000'],
        can_errors=lambda t: (128 if t > 200 * MS else 0, 700 * MS <= t < 1200 * MS)))

    # HAL_CAN_Init fails: Error_Handler prints where it was called from, halts
    def error_check(c, s, r):
        check_run(c, s, r)
        lines = [line for _, line in r.uart]
        c.check(len(lines) == 2 and lines[0] == "" and lines[1].startswith("Error_Handler from 0x"),
                f"expected just the Error_Handler line, got {lines}")
        c.check(not r.can, "sent CAN frames after Error_Handler")
        c.check(not [t for t, v in r.enabled if v], "ETC ran after Error_Handler")
        if lines and lines[-1].startswith("Error_Handler from 0x"):
            address = (int(lines[-1].split()[-1], 16) & ~1) - 1  # the call, not the return address
            where = subprocess.run([TOOLS.addr2line, "-f", "-e", TOOLS.elf, hex(address)],
                                   capture_output=True, text=True).stdout.split()
            c.check(where and where[0] == "can_init", f"Error_Handler called from {where}, expected can_init")

    result.append(Scenario(
        "error_handler", half, [("wait", 0.2)],
        hooks=['cpu ReturnBetween `sysbus GetSymbolAddress "HAL_CAN_Init"` 1 0 1000000000'],
        checks=[error_check], general=False))

    # 10 s: pedal sweeps up and down every 4 s (0.05 travel per 100 ms, so a
    # mixed update never looks like a deviation), with 300 ms faults off the
    # 50 ms grid
    steps = []
    t = 0
    faults = (2625, 5625, 8625)  # ms
    while t < 10000:
        phase = (t % 4000) / 2000
        travel = phase if phase <= 1 else 2 - phase
        in_fault = any(f <= t < f + 300 for f in faults)
        other = travel + 0.2 if travel < 0.8 else travel - 0.2
        steps.append(("apps", apps(travel, other) if in_fault else apps(travel)))
        nxt = min([t + 100] + [f for f in faults if f > t] + [f + 300 for f in faults if f + 300 > t])
        steps.append(("wait", (nxt - t) / 1000))
        t = nxt
    result.append(Scenario("soak_10s", apps(0.0), steps))
    return result


# --- main --------------------------------------------------------------------

class Tools:
    pass


TOOLS = Tools()


def find_tool(elf, name):
    """arm-none-eabi-<name> next to the compiler CMake used for this ELF"""
    build = os.path.dirname(os.path.dirname(elf))
    cache = os.path.join(build, "CMakeCache.txt")
    if os.path.exists(cache):
        for line in open(cache):
            if line.startswith("CMAKE_C_COMPILER:"):
                compiler = line.split("=", 1)[1].strip()
                candidate = compiler[:-len("gcc")] + name if compiler.endswith("gcc") else None
                if candidate and os.path.exists(candidate):
                    return candidate
    return shutil.which("arm-none-eabi-" + name)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--elf", default=os.path.join(ROOT, "build", "debug", "vcu", "vcu.elf"))
    parser.add_argument("--renode", default=shutil.which("renode") or "/Applications/Renode.app/Contents/MacOS/renode")
    parser.add_argument("--jobs", type=int, default=4)
    parser.add_argument("--keep", action="store_true", help="keep each run's files")
    parser.add_argument("only", nargs="*", help="scenario names to run")
    args = parser.parse_args()

    TOOLS.elf = os.path.abspath(args.elf)
    TOOLS.renode = args.renode
    if not os.path.exists(TOOLS.elf):
        sys.exit(f"{TOOLS.elf} not found, build it first")
    if not os.path.exists(TOOLS.renode):
        sys.exit(f"Renode not found at {TOOLS.renode}, pass --renode")
    gdb, TOOLS.addr2line = find_tool(TOOLS.elf, "gdb"), find_tool(TOOLS.elf, "addr2line")
    if not gdb or not TOOLS.addr2line:
        sys.exit("arm-none-eabi-gdb and arm-none-eabi-addr2line are needed")
    out = subprocess.run([gdb, "-batch", "-ex", "print/x &etc.motor_enabled", TOOLS.elf],
                         capture_output=True, text=True).stdout
    m = re.search(r"= (0x[0-9a-f]+)", out)
    if not m:
        sys.exit(f"couldn't find etc.motor_enabled in the ELF: {out}")
    TOOLS.enabled_address = int(m.group(1), 16)

    todo = [s for s in scenarios() if not args.only or s.name in args.only]
    if args.only and len(todo) != len(set(args.only)):
        sys.exit(f"unknown scenario in {args.only}")
    work = tempfile.mkdtemp(prefix="vcu-renode-")
    print(f"{TOOLS.elf}\n{len(todo)} scenarios, files in {work}\n")

    def run(s):
        out_dir = os.path.join(work, s.name)
        os.makedirs(out_dir)
        started = time.time()
        c = Checker()
        try:
            r = run_renode(TOOLS, s, out_dir)
            if s.general:
                check_run(c, s, r)
                check_general(c, s, r)
            for check in s.checks:
                check(c, s, r)
        except Exception as e:  # report it and carry on with the other scenarios
            c.failures.append(f"{type(e).__name__}: {e}")
        return s, c, time.time() - started

    failed = 0
    with concurrent.futures.ThreadPoolExecutor(args.jobs) as pool:
        for s, c, took in pool.map(run, todo):
            status = "FAIL" if c.failures else "ok"
            print(f"{status:4} {s.name:24} {c.passed:5} checks  {took:6.1f} s")
            for failure in c.failures[:10]:
                print(f"       {failure}")
            if len(c.failures) > 10:
                print(f"       ... {len(c.failures) - 10} more")
            failed += bool(c.failures)

    if not args.keep and not failed:
        shutil.rmtree(work)
    print(f"\n{len(todo) - failed}/{len(todo)} scenarios passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
