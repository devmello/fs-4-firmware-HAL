#!/usr/bin/env python3
"""
Runs the fs-4 VCU firmware on an emulated STM32F446 in Renode and checks it
against a model of what it should do: CAN frames on both buses, the debug
lines on UART4, the four output pins, the VN-200 traffic on UART5, the ETC's
state in RAM, and how it left the peripheral registers.

    python3 test/renode/run_tests.py
    python3 test/renode/run_tests.py --elf build/release/vcu/vcu.elf
    python3 test/renode/run_tests.py boot rtd_sequence
    python3 test/renode/run_tests.py --list

Needs Renode 1.17 (on PATH, or --renode) and the Arm toolchain (nm, gdb and
addr2line are taken from next to the compiler in the ELF's CMake cache, or
PATH). Times are virtual time. Takes about 2.5 minutes with the default jobs
on a 10-core Mac. --keep keeps each run's files, --recheck checks kept runs
again without Renode, --verbose shows what each scenario compared.
mutants.py runs the suite against deliberately broken builds.

The emulated board (vcu.repl)
  Renode's STM32F4 plus: ADC1 with every channel fed separately
  (STM32F4_ADC_Patched.cs), a DMA controller with circular mode and half
  transfer interrupts wired to UART5 RX on stream 0 channel 4
  (STM32F4_DMA_Patched.cs), and plain memory for the registers Renode doesn't
  model (ADC common, SYSCFG, DBGMCU). Recorders.cs records CAN frames (both
  buses in one file, so their order is kept), UART lines, the output pins, ETC
  bytes in RAM (polled every 100 us) and the RCC reset flags, plays a timeline
  of inputs (ADC codes, pin levels for the RTD button and BSPD inputs, frames
  received on either bus) and injects faults (mailboxes busy, watchdog not
  reloaded, a HAL call failing). FakeVN200.cs is a VN-200 on UART5: it answers
  the configuration commands like the sensor and then streams binary
  messages, 82 bytes at 100 Hz, paced at the baud rate.

How the checks work
  Each scenario is a timeline of inputs. From that timeline alone, model.py
  (a float32 copy of the ETC math, the filters, the pedal map, frame packing,
  traction control and IMU scaling) and the code below (the implausibility
  timers, brake + accel latch, ready to drive, buzzer, job schedule, VN-200
  driver states) predict, for any moment, what each output should be. The
  EWMA filters make values depend on loop timing, so predictions are sets
  (see model.py), and moments within a loop pass of a change are "can't say".
  Every recorded frame, debug line, pin change and RAM sample is compared
  with the prediction at its time. The job schedule (deadlines, order when
  jobs coincide, the 5 ms copies, MBB_Alive), frame layouts, forwarding, the
  VN-200 command sequence and its timeouts, and the register setup after boot
  are checked too, and each scenario checks that it exercised what it's for.
  The model assumes loop passes of 30-400 us (they measure 63-330 us); the
  debug lines' "loop max" is checked against that.

What the emulator can't tell you (check these on the car)
  - Clocks: HSE/PLL/over-drive are ready instantly and nothing is clock gated.
    The register audit catches wrong settings, not a crystal that won't start.
  - CAN: a frame goes out the moment a mailbox is filled: no arbitration, ACK,
    bit timing, error counters or bus-off (the debug line's error part is
    tested by writing ESR and forcing can_bus_off()). On the chip, three
    mailboxes with TXFP = 0 send the lowest id first, so frames queued
    together (the five IMU frames, coinciding jobs) can reach the bus in
    another order than here. Mailboxes never stay busy on their own; the
    TX queue is tested by making them read busy, and when that ends no TX
    interrupt fires (see can_tx_queue). Received frames are injected at
    back-to-back bus spacing, never faster; FIFO overruns don't happen.
  - UART: the console's characters go out instantly, so its baud rate and a
    full TX buffer aren't exercised. UART5 RX is paced at the baud rate, with
    no framing errors, noise or overruns. The DMA ring never overflows (the
    loop never stalls 125 ms), so the "lost" count is only seen at 0.
  - ADC: inputs are raw codes, no analog front end, noise or settling.
  - GPIO: output type (push-pull) isn't modelled and pins have no electrical
    behavior. The RTD button doesn't bounce (the board RC filters it); a
    bounce that spans two loop passes would turn RTD on and off again.
  - IWDG: the LSI is exactly 32 kHz here; on the chip it's 17-47 kHz, so the
    real timeout is anywhere in about 170-470 ms.
  - Timing: the CPU runs a flat 180 instructions per us, so loop times are
    close but not exact, and the EWMA filters see different time steps.
  - The VN-200 is a model written from the driver's view of ICD 1.3 (echo with
    checksum, $VNERR codes as two hex digits, binary layout and CRC). The real
    sensor's reply format and timing, its error codes (hex or decimal), what
    it does with VNWRG,76/77 "0,0,00", and whether it refuses register 75
    while VNINS is still on need the bench.
"""

import argparse
import bisect
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
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True  # no __pycache__ in the source tree

import model  # noqa: E402
from model import Interval, f32  # noqa: E402

MS = 1000  # times are in us

# One loop pass is ~63-110 us here, the one that prints the debug lines up
# to ~330 us. Checked against "loop max" in every debug line.
PASS_MAX = 400
# Input to visible effect: the event waits for the next pass, then the
# output is written partway into it
REACT = PASS_MAX + 200
# A job's frames go out in the pass that finds it due
JOB_LATE = PASS_MAX + 200
SAMPLE = 100           # RAM is polled this often
SETTLE = 100 * MS      # the 60 Hz filters after a step (38 time constants)
SETTLE_TORQUE = 150 * MS  # plus the 40 Hz torque filter after them
IMPLAUS = 101 * MS     # flag after "> 100" whole ms
BUZZER = 2000 * MS
COPY_DELAY = 5 * MS
VN_RESPONSE = 100 * MS
VN_BACKOFF = 1000 * MS
VN_DATA_TIMEOUT = 500 * MS

P, D = "P", "D"  # CAN_P = CAN1 500k, CAN_D = CAN2 1M

# ADC1 channels
REAR_BSE, FRONT_BSE, APPS1, APPS2, BPPS, STEERING = 0, 1, 11, 12, 13, 15
ETC_CHANNELS = (APPS1, APPS2, BPPS, FRONT_BSE, REAR_BSE)

# Frames the VCU sends
THROTTLE, CURRENTS, PEDALS, STATUS, TRACTION = 390, 646, 402, 403, 660
ACCEL, YPR, LATLON, GYRO, VEL = 720, 976, 721, 977, 722
IMU_IDS = (ACCEL, YPR, LATLON, GYRO, VEL)
DLC = {THROTTLE: 8, CURRENTS: 8, PEDALS: 8, STATUS: 8, TRACTION: 8,
       ACCEL: 6, YPR: 6, LATLON: 8, GYRO: 6, VEL: 6}

# Frames it handles
BATTERY, SPEED, TRAY, SME_TEMP, MODES = 913, 1154, 1216, 1666, 432
WHEELS = (421, 422, 423, 424)  # fl, fr, bl, br
FORWARDED = (SPEED, SME_TEMP)

# Jobs in the order main() runs them when several are due in one pass
JOBS = [  # name, period, frames (bus, id)
    ("data80", 80 * MS, [(D, THROTTLE), (D, TRACTION), (D, CURRENTS)]),
    ("etc50", 50 * MS, [(D, PEDALS), (D, STATUS)]),
    ("pt40", 40 * MS, [(P, THROTTLE), (P, CURRENTS)]),
    ("imu10", 10 * MS, [(D, i) for i in IMU_IDS]),
]
DEBUG_PERIOD = 500 * MS

# VN-200 commands with their checksums, worked out by hand from ICD 1.4.2
VN_COMMANDS = [
    "$VNASY,0*4F",
    "$VNWRG,06,0*6C",
    "$VNWRG,76,0,0,00*5B",
    "$VNWRG,77,0,0,00*5A",
    "$VNWRG,75,2,8,34,0600,0002,000A*0C",
    "$VNRRG,01*72",
    "$VNASY,1*4E",
]
VN_MODEL = "VN-200T-CR"  # FakeVN200's answer to $VNRRG,01
# Replies that aren't the echo of their command
VN_REPLIES = {"$VNRRG,01*72": f"$VNRRG,01,{VN_MODEL}*31"}

# Renode warnings that are expected: the flash cache bits aren't modelled
ALLOWED_WARNINGS = [
    re.compile(r"Translation cache size"),
    re.compile(r"flash_controller: Unhandled write to offset 0x0\. Unhandled bits: \[(9|10)\]"),
]
# and in scenarios with a reset: the watchdog's own message, and the UART
# model flushing bytes still on their way from the sensor when its baud rate
# register resets (they're thrown away right after)
RESET_WARNINGS = [
    re.compile(r"iwdg: Watchdog reset triggered"),
    re.compile(r"uart5: Unknown baud rate, couldn't trigger the idle line interrupt"),
]

DEBUG_LINE_1 = re.compile(
    r"APPS (-?\d+\.\d{3}) (-?\d+\.\d{3}) V \| BPPS (-?\d+\.\d{3}) V \| BSE (-?\d+\.\d{3}) (-?\d+\.\d{3}) V \| "
    r"torque (-?\d+) \| RTD ([01]) EN ([01]) \| implaus ([01])([01])([01])([01])([01])$")
DEBUG_LINE_2 = re.compile(
    r"CAN P err (\d+)( bus-off)? drop (\d+) \| CAN D err (\d+)( bus-off)? drop (\d+) \| "
    r"IMU (ok|setup) pkts (\d+) crc (\d+) lost (\d+) cfg (\d+) err ([0-9A-F]{2}) \| "
    r"loop max (\d+) us, (\d+) passes$")


# --- inputs -------------------------------------------------------------------

def apps_codes(travel1, travel2=None):
    """ADC codes for APPS1/APPS2 at a pedal travel, 0..1 between the
    calibration voltages"""
    if travel2 is None:
        travel2 = travel1
    return (model.code_for(0.396 + travel1 * (1.086 - 0.396)),
            model.code_for(0.439 + travel2 * (1.133 - 0.439)))


def bpps_code(position):
    """BPPS code for a brake pedal position (0 at 0.47 V, 1 at 0.982 V)"""
    return model.code_for(0.470 + position * (0.972 - 0.460))


def bse_code(psi):
    return model.code_for((psi / 2000.0 * 2640.0 + 330.0) / 1000.0)


REST = apps_codes(0.0)
DEFAULT_CODES = {
    APPS1: REST[0], APPS2: REST[1],
    BPPS: bpps_code(0.0),          # brake released
    FRONT_BSE: bse_code(20.0),     # some residual pressure, under the 30 psi brake light
    REAR_BSE: bse_code(20.0),
    STEERING: 804,                 # about straight ahead
}


class Scenario:
    """A timeline of inputs, what to record, and extra checks. Times passed to
    the helpers are in ms."""

    def __init__(self, name, seconds, about, imu="on", registers=False, boots=("power on",)):
        self.name = name
        self.about = about
        self.end = int(round(seconds * 1e6))
        self.imu = imu                  # "on", or "off": no sensor answering
        self.registers = registers
        self.boots = boots              # the reset cause each boot should print
        self.events = []                # (us, timeline text)
        self.adc = {ch: [(0, code)] for ch, code in DEFAULT_CODES.items()}
        self.pins = {("A", 2): [(0, 0)], ("A", 3): [(0, 0)], ("C", 13): [(0, 0)]}
        self.rx = []                    # (us, bus, id, ext, rtr, data)
        self.monitor = []               # monitor commands run before the start
        self.checks = []                # check(c, s, run, expect)
        self.mailbox_holds = []         # (bus, from, to)
        self.overflows = []             # holds that overflow the TX queue
        self.watchdog_blocks = []       # (from, to)
        self.fail_can_init = None       # (from, to): HAL_CAN_Init returns HAL_ERROR
        self.bus_off = None             # (from, to): can_bus_off() returns true
        self.tim5_start = None          # TIM5 count right after timebase_init()
        self.stuck_boots = set()        # boots (by number) that should never reach the loop
        self.allowed = []
        for ch, code in DEFAULT_CODES.items():
            self.events.append((0, f"adc {ch} {code}"))

    @staticmethod
    def us(ms):
        return int(round(ms * MS))

    def set_adc(self, ms, channel, code):
        t = self.us(ms)
        assert 0 <= code <= 4095
        self.adc[channel].append((t, code))
        self.adc[channel].sort(key=lambda x: x[0])  # stable: the later of two at one time wins, as in the timeline
        self.events.append((t, f"adc {channel} {code}"))

    def apps(self, ms, travel1, travel2=None):
        c1, c2 = apps_codes(travel1, travel2)
        self.set_adc(ms, APPS1, c1)
        self.set_adc(ms, APPS2, c2)

    def apps_raw(self, ms, code1, code2):
        self.set_adc(ms, APPS1, code1)
        self.set_adc(ms, APPS2, code2)

    def brake(self, ms, position):
        self.set_adc(ms, BPPS, bpps_code(position))

    def pressure(self, ms, front_psi, rear_psi=None):
        self.set_adc(ms, FRONT_BSE, bse_code(front_psi))
        self.set_adc(ms, REAR_BSE, bse_code(front_psi if rear_psi is None else rear_psi))

    def pin(self, ms, port, number, level):
        t = self.us(ms)
        self.pins[(port, number)].append((t, level))
        self.pins[(port, number)].sort(key=lambda x: x[0])
        self.events.append((t, f"pin {port} {number} {level}"))

    def button(self, ms, hold_ms=100):
        self.pin(ms, "C", 13, 1)
        self.pin(ms + hold_ms, "C", 13, 0)

    def frame(self, ms, bus, can_id, data=b"", ext=False, rtr=False):
        t = self.us(ms)
        data = bytes(data)
        self.rx.append((t, bus, can_id, ext, rtr, data))
        flags = ("x" if ext else "s") + ("r" if rtr else "d")
        self.events.append((t, f"can {1 if bus == P else 2} {can_id:X} {flags} {data.hex() or '-'}"))

    def imu_command(self, ms, command):
        self.events.append((self.us(ms), f"imu {command}"))

    def wheels(self, ms, fl, fr, bl, br):
        """The four wheel speed frames (rpm), back to back on CAN_D"""
        for n, (can_id, rpm) in enumerate(zip(WHEELS, (fl, fr, bl, br))):
            raw = int(round(rpm * 10))
            self.frame(ms + 0.13 * n, D, can_id, [raw & 0xFF, raw >> 8, 0, 0, 0, 0, 0, 0])

    def expected_drops(self, t, b):
        """(CAN_P, CAN_D) frames the TX queues should have dropped by t"""
        return (0, 0)

    def expected_can_errors(self, t):
        """((TEC, bus-off) of CAN_P, same of CAN_D) the debug line shows at t"""
        return ((0, False), (0, False))

    def hold_mailboxes(self, bus, from_ms, to_ms, overflow=False):
        """overflow: more frames come due than the TX queue holds; the
        scenario checks those itself"""
        self.mailbox_holds.append((bus, self.us(from_ms), self.us(to_ms)))
        if overflow:
            self.overflows.append((bus, self.us(from_ms), self.us(to_ms)))

    def block_watchdog(self, from_ms, to_ms):
        self.watchdog_blocks.append((self.us(from_ms), self.us(to_ms)))

    def code_at(self, channel, t):
        code = None
        for at, value in self.adc[channel]:
            if at <= t:
                code = value
        return code

    def pin_at(self, port, number, t):
        level = 0
        for at, value in self.pins[(port, number)]:
            if at <= t:
                level = value
        return level


# --- running Renode ---------------------------------------------------------------

class Tools:
    pass


TOOLS = Tools()

FUNCTIONS = ["can_init", "Vn200::start(unsigned long long)", "HAL_CAN_Init", "HAL_CAN_Start",
             "HAL_IWDG_Init", "HAL_RCC_OscConfig", "console_init", "can_bus_off"]
RAM_FIELDS = {  # label: expression for gdb
    "rtd": "etc.state.ready_to_drive",
    "en": "etc.state.motor_enabled",
    "dev": "etc.state.implaus_APPS_deviation",
    "apps": "etc.state.implaus_APPS_range",
    "bpps": "etc.state.implaus_BPPS_range",
    "bse": "etc.state.implaus_BSE_range",
    "ba": "etc.state.implaus_brake_and_accel",
}
OTHER_SYMBOLS = {
    "drive_mode": "etc.state.drive_mode",
    "traction_mode": "etc.state.traction_mode",
    "regen_mode": "etc.state.regen_mode",
    "rx_ring": "rx_ring",
    "tim5_wraps": "tim5_wraps",
}

GPIO = {"A": 0x40020000, "B": 0x40020400, "C": 0x40020800, "D": 0x40020C00}
CAN_BASE = {P: 0x40006400, D: 0x40006800}
TIM5_CNT = 0x40000C24


def script(s, out):
    elf = TOOLS.elf
    fn = TOOLS.functions
    lines = [
        f"include @{HERE}/STM32F4_ADC_Patched.cs",
        f"include @{HERE}/STM32F4_DMA_Patched.cs",
        f"include @{HERE}/FakeVN200.cs",
        f"include @{HERE}/Recorders.cs",
        'mach create "vcu"',
        f"machine LoadPlatformDescription @{HERE}/vcu.repl",
        # Runs at every reset (the watchdog's too) to put the firmware back
        "macro reset",
        '"""',
        f"    sysbus LoadELF @{elf}",
        '"""',
        "runMacro $reset",
        f'can1 RecordFrames "P" @{out}/can.txt',
        f'can2 RecordFrames "D" @{out}/can.txt',
        f"uart4 RecordLines @{out}/uart.txt",
        f'gpioPortC RecordPin 0 "PC0" @{out}/pins.txt',
        f'gpioPortA RecordPin 7 "PA7" @{out}/pins.txt',
        f'gpioPortB RecordPin 1 "PB1" @{out}/pins.txt',
        f'gpioPortC RecordPin 4 "PC4" @{out}/pins.txt',
        # Renode's bxCAN only enters init mode with SLEEP clear. ST's HAL sets
        # INRQ before clearing SLEEP, which works on the chip, so clear SLEEP
        # on both controllers as can_init() starts (at every boot).
        f"cpu WriteWordAt {fn['can_init']:#x} {CAN_BASE[P]:#x} 0x00010000",
        f"cpu WriteWordAt {fn['can_init']:#x} {CAN_BASE[D]:#x} 0x00010000",
        # Time the job deadlines start from (Vn200::start(now)), 64 bits in r2
        # (low) and r3 (high)
        f"cpu RecordRegisterAt {fn['Vn200::start(unsigned long long)']:#x} 2 @{out}/now0.txt",
        f"cpu RecordRegisterAt {fn['Vn200::start(unsigned long long)']:#x} 3 @{out}/now0_high.txt",
        f"machine EmulateResetFlags @{out}/resets.txt",
        f'machine SampleBytes "{" ".join(f"{k}={v:#x}" for k, v in TOOLS.ram.items())}" {SAMPLE} @{out}/ram.txt',
    ]
    # Pin setup when the watchdog starts and when the clocks start: the
    # outputs have to be low outputs by then
    for where in ("HAL_IWDG_Init", "HAL_RCC_OscConfig"):
        for port in ("A", "B", "C"):
            lines.append(f'cpu RecordWordAt {fn[where]:#x} {GPIO[port]:#x} "{where} {port}_MODER" @{out}/audit.txt')
            lines.append(f'cpu RecordWordAt {fn[where]:#x} {GPIO[port] + 0x14:#x} "{where} {port}_ODR" @{out}/audit.txt')
    # BTR only reads back in init mode, so grab both as HAL_CAN_Start runs
    for bus in (P, D):
        lines.append(f'cpu RecordWordAt {fn["HAL_CAN_Start"]:#x} {CAN_BASE[bus] + 0x1C:#x} "BTR_{bus}" @{out}/audit.txt')
    lines.append(f"uart5 AttachFakeVN200 0x40005000 @{out}/vn200.txt")
    for bus, a, b in s.mailbox_holds:
        lines.append(f"can{1 if bus == P else 2} HoldMailboxes {a} {b}")
    for a, b in s.watchdog_blocks:
        lines.append(f"machine BlockWatchdogRefresh {a} {b}")
    if s.fail_can_init:
        a, b = s.fail_can_init
        lines.append(f"cpu ReturnBetween {fn['HAL_CAN_Init']:#x} 1 {a} {b}")
    if s.bus_off:
        # Renode's bxCAN can't go bus-off; only called for the debug line
        a, b = s.bus_off
        lines.append(f"cpu ReturnBetween {fn['can_bus_off']:#x} 1 {a} {b}")
    if s.tim5_start is not None:
        lines.append(f"cpu WriteWordAt {fn['console_init']:#x} {TIM5_CNT:#x} {s.tim5_start:#x}")
    lines += s.monitor
    events = sorted(s.events, key=lambda e: e[0])
    if s.imu == "off":
        events.insert(0, (0, "imu off"))
    with open(os.path.join(out, "timeline.txt"), "w") as f:
        f.writelines(f"{t} {text}\n" for t, text in events)
    lines += [
        f"machine LoadTimeline @{out}/timeline.txt @{out}/events.txt",
        f'emulation RunFor "{s.end / 1e6:.6f}"',
        "uart4 FlushLine",
    ]
    if s.registers:
        for name, address, _, _ in REGISTERS:
            lines += [f'echo "REG {name}"', f"sysbus ReadDoubleWord {address:#x}"]
    for name, address in TOOLS.other.items():
        lines += [f'echo "RAM {name}"', f"sysbus ReadDoubleWord {address & ~3:#x}"]
    lines += ['echo "END_OF_SCRIPT"', "quit"]
    return "\n".join(lines) + "\n"


class Frame:
    __slots__ = ("t", "bus", "id", "ext", "rtr", "data", "index", "job", "k", "at")

    def __init__(self, t, bus, can_id, ext, rtr, data, index):
        self.t, self.bus, self.id, self.ext, self.rtr, self.data, self.index = t, bus, can_id, ext, rtr, data, index
        self.job = self.k = None
        self.at = t  # when its contents were taken (earlier than t if it waited in the TX queue)

    def i16(self, at):
        return struct.unpack_from("<h", self.data, at)[0]

    def u16(self, at):
        return struct.unpack_from("<H", self.data, at)[0]

    def __repr__(self):
        flags = ("x" if self.ext else "") + ("r" if self.rtr else "")
        return f"{self.t / MS:.3f}ms {self.bus} {self.id}{flags} {self.data.hex()}"


class Run:
    pass


def read_lines(path):
    return open(path).read().splitlines() if os.path.exists(path) else []


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


def run_renode(s, out):
    with open(os.path.join(out, "run.resc"), "w") as f:
        f.write(script(s, out))
    timeout = 120 + 30 * s.end / 1e6
    proc = subprocess.run([TOOLS.renode, "--console", "--disable-gui", "--plain", os.path.join(out, "run.resc")],
                          stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=timeout)
    with open(os.path.join(out, "renode.txt"), "w") as f:
        f.write(proc.stdout + proc.stderr)
    with open(os.path.join(out, "exit.txt"), "w") as f:
        f.write(str(proc.returncode))
    return parse_run(out)


def parse_run(out):
    """Everything a run recorded, from its directory"""
    r = Run()
    r.stdout = open(os.path.join(out, "renode.txt")).read()
    r.returncode = int(open(os.path.join(out, "exit.txt")).read())
    r.frames = []
    for line in read_lines(os.path.join(out, "can.txt")):
        t, bus, can_id, flags, data = line.split()
        r.frames.append(Frame(int(t), bus, int(can_id, 16), flags[0] == "x", flags[1] == "r",
                              b"" if data == "-" else bytes.fromhex(data), len(r.frames)))
    r.uart = []
    for line in read_lines(os.path.join(out, "uart.txt")):
        t, _, text = line.partition(" ")
        r.uart.append((int(t), text))
    r.pins = {"PC0": [], "PA7": [], "PB1": [], "PC4": []}
    for line in read_lines(os.path.join(out, "pins.txt")):
        t, name, level = line.split()
        r.pins[name].append((int(t), int(level)))
    r.ram = {name: [] for name in RAM_FIELDS}
    for line in read_lines(os.path.join(out, "ram.txt")):
        t, name, value = line.split()
        r.ram[name].append((int(t), int(value)))
    r.vn = []
    for line in read_lines(os.path.join(out, "vn200.txt")):
        t, _, rest = line.partition(" ")
        r.vn.append((int(t), rest))
    low = [line.split() for line in read_lines(os.path.join(out, "now0.txt"))]
    high = [line.split() for line in read_lines(os.path.join(out, "now0_high.txt"))]
    r.now0 = [(int(t), int(v) | int(h) << 32) for (t, v), (_, h) in zip(low, high)]
    r.resets = []
    for line in read_lines(os.path.join(out, "resets.txt")):
        t, _, flags = line.split()
        r.resets.append((int(t), int(flags, 16)))
    r.audit = []
    for line in read_lines(os.path.join(out, "audit.txt")):
        parts = line.split()
        r.audit.append((int(parts[0]), " ".join(parts[1:-1]), int(parts[-1], 16)))
    r.events = []
    for line in read_lines(os.path.join(out, "events.txt")):
        t, _, text = line.partition(" ")
        r.events.append((int(t), text))
    r.registers = echoed(r.stdout, "REG")
    r.ram_end = echoed(r.stdout, "RAM")
    return r


# --- what should happen -------------------------------------------------------
#
# Levels are 0, 1 or None (can't say: around a change, or the model can't
# decide which side of a threshold the firmware's float is on).

def t_and(*values):
    if any(v == 0 for v in values):
        return 0
    return 1 if all(v == 1 for v in values) else None


def t_not(v):
    return None if v is None else 1 - v


def above(interval, threshold):
    """value > threshold"""
    if interval.lo > threshold:
        return 1
    return 0 if interval.hi <= threshold else None


def below(interval, threshold):
    """value < threshold"""
    if interval.hi < threshold:
        return 1
    return 0 if interval.lo >= threshold else None


def outside(interval, lo, hi):
    """!(lo <= value <= hi)"""
    if interval.lo >= lo and interval.hi <= hi:
        return 0
    return 1 if interval.hi < lo or interval.lo > hi else None


class Signal:
    """A level over time, built from changes that happen somewhere inside a
    window. Inside any window the level is None."""

    def __init__(self, initial=0):
        self.initial = initial
        self.changes = []  # (from, to, value)
        self.times = self.values = None

    def change(self, t_from, t_to, value):
        self.changes.append((t_from, max(t_from, t_to), value))
        self.times = None

    def _build(self):
        points = []
        for n, (a, b, v) in enumerate(self.changes):
            points.append((a, 1, n, None))
            points.append((b, 0, n, v))
        points.sort(key=lambda p: (p[0], p[1], p[2]))
        times, values = [0], [self.initial]
        open_windows, value = 0, self.initial
        for t, kind, _, v in points:
            if kind == 1:
                open_windows += 1
            else:
                open_windows -= 1
                value = v
            level = value if open_windows == 0 else None
            if times[-1] == t:
                values[-1] = level
            elif values[-1] != level:
                times.append(t)
                values.append(level)
        self.times, self.values = times, values

    def at(self, t):
        if self.times is None:
            self._build()
        return self.values[bisect.bisect_right(self.times, t) - 1]

    def breakpoints(self):
        if self.times is None:
            self._build()
        return list(zip(self.times, self.values))

    @staticmethod
    def of_points(points, initial=0):
        """From (t, level) samples where each level holds until the next"""
        s = Signal(initial)
        s.times, s.values = [0], [initial]
        for t, v in points:
            if s.times[-1] == t:
                s.values[-1] = v
            elif s.values[-1] != v:
                s.times.append(t)
                s.values.append(v)
        return s


def combine(fn, signals):
    times = sorted({t for s in signals for t, _ in s.breakpoints()})
    return Signal.of_points([(t, fn(*[s.at(t) for s in signals])) for t in times])


class Filtered:
    """One FilteredAnalogIn during one boot: seeded at its first read, then
    following the input's steps"""

    def __init__(self, changes, seed_t):
        self.seed_t = seed_t
        code = None
        for t, c in changes:
            if t <= seed_t:
                code = c
        r = model.adc_fraction(code)
        self.segments = [(seed_t, r, Interval(r, r), code)]
        for t, c in changes:
            if t <= seed_t or c == self.segments[-1][3]:
                continue
            start = self.state(t)
            self.segments.append((t, model.adc_fraction(c), start, c))

    def _segment(self, t):
        i = bisect.bisect_right(self.segments, (t, 9.0)) - 1
        return self.segments[max(i, 0)]

    def state(self, t):
        t0, target, start, _ = self._segment(t)
        if t0 == self.seed_t and start.lo == start.hi == target:
            return model.filter_interval(target, start, 0, 0)
        return model.filter_interval(target, start, (t - t0 - PASS_MAX) * (1 - model.FILTER_TIME_LOST) * 1e-6,
                                     (t - t0 + PASS_MAX) * 1e-6)

    def volts(self, t):
        return self.state(t).map(model.filtered_volts)

    def last_change(self, t):
        return self._segment(t)[0]

    def change_times(self):
        return [seg[0] for seg in self.segments[1:]]


class EtcAt:
    """What update_state() works with at time t, as intervals"""

    def __init__(self, filters, t):
        self.t = t
        v = {ch: filters[ch].volts(t) for ch in ETC_CHANNELS}
        self.v1, self.v2, self.vb, self.vf, self.vr = (v[APPS1], v[APPS2], v[BPPS], v[FRONT_BSE], v[REAR_BSE])
        self.p1 = self.v1.map(model.apps1_position)
        self.p2 = self.v2.map(model.apps2_position)
        self.avg = Interval(model.average(self.p1.lo, self.p2.lo), model.average(self.p1.hi, self.p2.hi))
        self.bpps = self.vb.map(model.bpps_position)
        self.psi_f = self.vf.map(model.pressure)
        self.psi_r = self.vr.map(model.pressure)

    def deviation(self):
        diffs = [abs(f32(a - b)) for a in (self.p1.lo, self.p1.hi) for b in (self.p2.lo, self.p2.hi)]
        overlap = self.p1.lo <= self.p2.hi and self.p2.lo <= self.p1.hi
        lo = 0.0 if overlap else min(diffs)
        if lo > model.MAX_DEVIATION:
            return 1
        return 0 if max(diffs) <= model.MAX_DEVIATION else None

    def apps_range(self):
        a = outside(self.v1, model.APPS1_LOW, model.APPS1_HIGH)
        b = outside(self.v2, model.APPS2_LOW, model.APPS2_HIGH)
        return 1 if a == 1 or b == 1 else 0 if a == 0 and b == 0 else None

    def bpps_range(self):
        return outside(self.vb, model.BPPS_LOW, model.BPPS_HIGH)

    def bse_range(self):
        a = outside(self.vf, model.BSE_LOW, model.BSE_HIGH)
        b = outside(self.vr, model.BSE_LOW, model.BSE_HIGH)
        return 1 if a == 1 or b == 1 else 0 if a == 0 and b == 0 else None

    def brake_accel_trigger(self):
        return t_and(above(self.psi_f, 30.0), above(self.avg, 0.25))

    def brake_accel_clear(self):
        return below(self.avg, 0.05)

    def brakelight(self):
        return above(self.psi_f, 30.0)

    def _avg_floats(self):
        if self.avg.width() > 1e-4:
            return None
        return model.floats_between(self.avg.lo, self.avg.hi)

    def torques(self):
        """Possible unfiltered torques. While the pedal moves, the range
        between the ends (the map is monotonic)."""
        floats = self._avg_floats()
        if floats is None:
            return model.int_range(model.unfiltered_torque(self.avg.lo), model.unfiltered_torque(self.avg.hi))
        return {model.unfiltered_torque(a) for a in floats}

    def mapped_percents(self):
        floats = self._avg_floats()
        if floats is None:
            ends = [model.percent(model.accelerator_mapping(a)) for a in (self.avg.lo, self.avg.hi)]
            return model.int_range(*ends)
        return {model.percent(model.accelerator_mapping(a)) for a in floats}


def int_set(interval, fn):
    """{fn(x)} over an interval, for a monotonic fn"""
    a, b = fn(interval.lo), fn(interval.hi)
    return model.int_range(min(a, b), max(a, b))


def printed(interval, text):
    """printf("%.3f") of some value in the interval could give text"""
    if interval.width() < 1e-4:
        return text in ("%.3f" % interval.lo, "%.3f" % interval.hi)
    return interval.lo - 0.0005 <= float(text) <= interval.hi + 0.0005


class Boot:
    """One run of the firmware, from a reset to the next"""

    def __init__(self, start, end):
        self.start, self.end = start, end
        self.t0 = None       # virtual time main() read 'now' (job deadlines count from it)
        self.now0 = None     # and the time it read (timebase_micros())
        self.hello = None


class Expect:
    """The model's view of a scenario run"""

    def __init__(self, s, r):
        self.s, self.r = s, r
        starts = [0] + [t for t, _ in r.resets]
        self.boots = [Boot(a, b) for a, b in zip(starts, starts[1:] + [s.end])]
        for t, now0 in r.now0:
            for b in self.boots:
                if b.start <= t < b.end:
                    b.t0, b.now0 = t, now0
        for t, line in r.uart:
            for b in self.boots:
                if b.start <= t < b.end and line == "Hello World!!" and b.hello is None:
                    b.hello = t
        self.running = [b for b in self.boots if b.t0 is not None]
        self.vn = {}  # id(boot): VnBoot, filled in by check_vn200
        for b in self.running:
            self._boot_model(b)
        self._pins()

    def boot_at(self, t):
        for b in self.boots:
            if b.start <= t < b.end:
                return b
        return self.boots[-1]

    def _grid(self, b):
        """Times to evaluate the ETC conditions: dense right after input steps,
        every 50 ms otherwise"""
        times = set(range(b.t0, b.end, 50 * MS))
        for ch in ETC_CHANNELS:
            for tc in b.filters[ch].change_times():
                times.update(range(max(b.t0, tc - PASS_MAX), min(b.end, tc + 30 * MS), 200))
                times.update(range(tc + 30 * MS, min(b.end, tc + SETTLE + 2 * PASS_MAX), 2 * MS))
                times.add(min(b.end - 1, tc + SETTLE + 2 * PASS_MAX))
        return sorted(t for t in times if b.t0 <= t < b.end)

    def _boot_model(self, b):
        s = self.s
        # The first update_state() is right after 'now' is read
        b.filters = {ch: Filtered(s.adc[ch], b.t0) for ch in ETC_CHANNELS}
        b.grid = self._grid(b)
        conditions = {k: [] for k in ("dev", "apps", "bpps", "bse", "trigger", "clear", "light")}
        for t in b.grid:
            e = EtcAt(b.filters, t)
            conditions["dev"].append((t, e.deviation()))
            conditions["apps"].append((t, e.apps_range()))
            conditions["bpps"].append((t, e.bpps_range()))
            conditions["bse"].append((t, e.bse_range()))
            conditions["trigger"].append((t, e.brake_accel_trigger()))
            conditions["clear"].append((t, e.brake_accel_clear()))
            conditions["light"].append((t, e.brakelight()))
        b.cond = {k: self._held(v, b) for k, v in conditions.items()}
        b.flags = {k: self._timed_flag(b.cond[k], b) for k in ("dev", "apps", "bpps", "bse")}
        b.flags["ba"] = self._latch(b.cond["trigger"], b.cond["clear"], b)
        b.brakelight = self._delayed(b.cond["light"])
        self._rtd(b)
        b.enabled = combine(lambda rtd, *flags: t_and(rtd, *[t_not(f) for f in flags]),
                            [b.rtd] + [b.flags[k] for k in ("dev", "apps", "bpps", "bse", "ba")])

    @staticmethod
    def _held(samples, b):
        """Samples to (start, end, level) pieces: a level holds until the next
        sample, and between two different levels it's None"""
        pieces = []
        for (t, v), (t2, v2) in zip(samples, samples[1:] + [(b.end, samples[-1][1])]):
            level = v if v == v2 else None
            if pieces and pieces[-1][2] == level:
                pieces[-1] = (pieces[-1][0], t2, level)
            else:
                pieces.append((t, t2, level))
        return pieces

    @staticmethod
    def _level_at(pieces, t):
        for a, z, v in pieces:
            if a <= t < z:
                return v
        return pieces[-1][2]

    @staticmethod
    def _timed_flag(pieces, b):
        """update_implaus_timer(): set once the condition has held for 101 ms
        (counted from the pass that first saw it), cleared in the first pass
        without it. Possibly set from 101 ms into a stretch where the
        condition may hold; certainly set from 101 ms plus two passes into one
        where it does hold."""
        maybe, sure = [], []
        for runs, accept in ((maybe, (1, None)), (sure, (1,))):
            for a, z, v in pieces:
                if v in accept:
                    if runs and runs[-1][1] == a:
                        runs[-1][1] = z
                    else:
                        runs.append([a, z])
        windows = [(a + IMPLAUS, z + PASS_MAX, None) for a, z in maybe if a + IMPLAUS < z + PASS_MAX]
        windows += [(a + IMPLAUS + 2 * PASS_MAX, z, 1) for a, z in sure if a + IMPLAUS + 2 * PASS_MAX < z]
        times = sorted({b.t0} | {w[0] for w in windows} | {w[1] for w in windows})
        levels = []
        for t in times:
            inside = [v for a, z, v in windows if a <= t < z]
            levels.append((t, 1 if 1 in inside else None if inside else 0))
        return Signal.of_points(levels)

    @classmethod
    def _latch(cls, trigger, clear, b):
        """implaus_brake_and_accel, once per pass: cleared if latched and the
        pedal is under 5%, then set if the trigger holds. Tracks the set of
        states the firmware could be in."""
        times = sorted({a for a, _, _ in trigger} | {a for a, _, _ in clear})
        states = {0}
        sig = Signal(0)
        for t in times:
            g, c = cls._level_at(trigger, t), cls._level_at(clear, t)
            new = set()
            for st in states:
                for gv in ((0, 1) if g is None else (g,)):
                    for cv in ((0, 1) if c is None else (c,)):
                        new.add(1 if gv else 0 if st and cv else st)
            if new != states:
                sig.change(t, t + PASS_MAX, new.copy().pop() if len(new) == 1 else None)
            states = new
        return sig

    @staticmethod
    def _delayed(pieces):
        """A level the firmware writes every pass: a change shows within a pass"""
        sig = Signal(0)
        level, unsure_from = 0, None
        for a, z, v in pieces:
            if v is None:
                unsure_from = a if unsure_from is None else unsure_from
                continue
            if unsure_from is not None or v != level:
                sig.change(a if unsure_from is None else unsure_from, a + PASS_MAX, v)
            level, unsure_from = v, None
        if unsure_from is not None:
            sig.change(unsure_from, pieces[-1][1], None)
        return sig

    def _rtd(self, b):
        """Ready to drive, the light (PC0) and the buzzer (PA7)"""
        s = self.s
        events = []
        for t, bus, can_id, ext, rtr, data in s.rx:
            if bus == P and can_id in (BATTERY, TRAY) and b.t0 < t < b.end and not rtr:
                events.append((t, can_id, data))
        prev = 0
        for t, level in s.pins[("C", 13)]:
            if level == 1 and prev == 0 and b.t0 < t < b.end:
                events.append((t, "button", None))
            prev = level
        events.sort(key=lambda e: (e[0], 0 if e[1] != "button" else 1))
        b.rtd = Signal(0)
        b.buzzer = Signal(0)
        b.rtd_events = []  # (t, what) for coverage checks
        rtd, precharged, shutdown = 0, False, False
        buzzer_off = None  # (from, to) of the pending turn-off
        b.precharged = Signal(0)

        def turn_off(t):
            nonlocal rtd
            if rtd != 0:
                b.rtd.change(t, t + REACT, 0)
                b.rtd_events.append((t, "off"))
            rtd = 0

        def buzzer_until(t):
            nonlocal buzzer_off
            if buzzer_off and buzzer_off[0] <= t:
                b.buzzer.change(buzzer_off[0], buzzer_off[1], 0)
                b.rtd_events.append((buzzer_off[0], "buzzer off"))
                buzzer_off = None

        for t, what, data in events:
            buzzer_until(t)
            if what == BATTERY:
                new_pre, new_shut = bool(data[0] & 0x40), bool(data[0] & 0x04)
                if new_pre != precharged:
                    b.precharged.change(t, t + REACT, int(new_pre))
                precharged, shutdown = new_pre, new_shut
                if not (precharged and shutdown):
                    turn_off(t)
            elif what == TRAY:
                if data[1] / 2.0 > 40.0:
                    turn_off(t)
            else:
                e1, e2 = EtcAt(b.filters, t), EtcAt(b.filters, t + REACT)
                cond = above(e1.bpps, model.BPPS_BRAKE_ENGAGE)
                if cond != above(e2.bpps, model.BPPS_BRAKE_ENGAGE):
                    cond = None
                if rtd == 0 and precharged and shutdown:
                    if cond == 1:
                        rtd = 1
                        b.rtd.change(t, t + REACT, 1)
                        b.buzzer.change(t, t + REACT, 1)
                        buzzer_off = (t + BUZZER, t + REACT + BUZZER + PASS_MAX)
                        b.rtd_events.append((t, "on"))
                    elif cond is None:
                        rtd = None
                        b.rtd.change(t, t + REACT, None)
                        b.rtd_events.append((t, "unsure"))
                else:
                    turn_off(t)
        buzzer_until(b.end + BUZZER)
        b.light = b.rtd

    def _pins(self):
        """Output pins across all boots: low from reset until set up"""
        self.pins = {}
        for name, per_boot in (("PC0", "light"), ("PA7", "buzzer"), ("PC4", "brakelight")):
            points = []
            for b in self.boots:
                points.append((b.start, 0))
                if b.t0 is None:
                    continue
                for t, v in getattr(b, per_boot).breakpoints():
                    if b.t0 <= t < b.end:
                        points.append((t, v))
                    elif t < b.t0:
                        points.append((b.t0, v))
            # a reset drives every pin low at once
            self.pins[name] = Signal.of_points(sorted(points, key=lambda p: p[0]))
        self.pins["PB1"] = Signal(0)

    def signal(self, t, name):
        """ETC flag/state level at t (None before the loop runs)"""
        b = self.boot_at(t)
        if b.t0 is None or t < b.t0:
            return 0
        if name == "rtd":
            return b.rtd.at(t)
        if name == "en":
            return b.enabled.at(t)
        return b.flags[name].at(t)

    def etc(self, t):
        b = self.boot_at(t)
        return EtcAt(b.filters, t) if b.t0 is not None and t >= b.t0 else None

    def settled(self, t, channels, how_long=SETTLE):
        b = self.boot_at(t)
        if b.t0 is None or t < b.t0 + how_long:
            return False
        return all(t - b.filters[ch].last_change(t) >= how_long for ch in channels)


# --- checks --------------------------------------------------------------------

class Checker:
    def __init__(self):
        self.failures = []
        self.passed = 0
        self.counts = {}  # what got compared, for the coverage checks

    def check(self, condition, message):
        if condition:
            self.passed += 1
        else:
            self.failures.append(message)
        return bool(condition)

    def count(self, what, n=1):
        self.counts[what] = self.counts.get(what, 0) + n


def level_at(changes, t, initial=0):
    level = initial
    for at, value in changes:
        if at <= t:
            level = value
        else:
            break
    return level


def compare_levels(c, name, observed, predicted, start, end, slack):
    """observed: [(t, level)] changes (t up to slack after the real change).
    predicted: Signal. Every observed change has to fall where the prediction
    is unknown or switches that way, and wherever it's known the observed
    level has to match."""
    bad = []
    for t, v in observed:
        if not start <= t < end:
            continue
        old = level_at([(a, b) for a, b in observed if a < t], t)
        if old == v:
            continue
        before, after = predicted.at(t - slack - 1), predicted.at(t)
        window = {lvl for at, lvl in predicted.breakpoints() if t - slack <= at <= t}
        if None in window or before is None or after is None or (before == old and after == v):
            continue
        bad.append(f"{name} -> {v} at {t / MS:.3f} ms, expected {after}")
    points = predicted.breakpoints() + [(end, None)]
    for (a, v), (z, _) in zip(points, points[1:]):
        a, z = max(a, start), min(z, end)
        if v is None or z - a <= slack:
            continue
        got = level_at(observed, a + slack)
        changes = [t for t, _ in observed if a + slack < t < z]
        if got != v or changes:
            bad.append(f"{name} should be {v} from {a / MS:.3f} to {z / MS:.3f} ms, "
                       f"was {got}{' and changed at ' + ', '.join(f'{t / MS:.3f}' for t in changes[:3]) if changes else ''}")
    for message in bad[:6]:
        c.check(False, message)
    if not bad:
        c.check(True, "")
    return not bad


def check_run(c, s, r, ex):
    """Renode itself: ran to the end, no errors, no unexpected warnings"""
    c.check(r.returncode == 0, f"Renode exit code {r.returncode}")
    c.check("END_OF_SCRIPT" in r.stdout, "script didn't run to the end")
    for bad in ("Errors during compilation", "There was an error", "[ERROR]"):
        c.check(bad not in r.stdout, f"Renode reported '{bad}'")
    allowed = ALLOWED_WARNINGS + s.allowed
    warnings = sorted({line.split("[WARNING]", 1)[1].strip() for line in r.stdout.splitlines()
                       if "[WARNING]" in line and not any(p.search(line) for p in allowed)})
    c.check(not warnings, f"unexpected Renode warnings: {warnings[:5]}")
    errors = [text for _, text in r.events if text.startswith("error")]
    c.check(not errors, f"timeline events failed: {errors[:3]}")


def check_tim5_wraps(c, s, r, ex):
    """The timebase counted every TIM5 wrap of the last boot, once"""
    b = ex.boots[-1]
    if b.t0 is None:
        return
    want = (b.now0 + s.end - b.t0) >> 32
    got = r.ram_end.get("tim5_wraps")
    c.check(got == want, f"the timebase counted {got} TIM5 wraps, expected {want}")


def check_boots(c, s, r, ex):
    c.check(len(ex.boots) == len(s.boots), f"{len(ex.boots)} boots (resets at {[t / MS for t, _ in r.resets]} ms), "
                                           f"expected {len(s.boots)}")
    for n, b in enumerate(ex.boots):
        if n in s.stuck_boots:
            c.check(b.t0 is None, f"boot {n} reached the loop")
            continue
        if not c.check(b.hello is not None and b.t0 is not None, f"boot at {b.start / MS:.3f} ms never reached the loop"):
            continue
        lines = [line for t, line in r.uart if b.start <= t < b.end]
        c.check(lines[:1] == ["Hello World!!"], f"boot at {b.start / MS:.3f} ms: first line {lines[:1]}")
        cause = s.boots[n] if n < len(s.boots) else "?"
        c.check(lines[1:2] == [f"Reset cause: {cause}"], f"boot at {b.start / MS:.3f} ms: {lines[1:2]}, "
                                                          f"expected 'Reset cause: {cause}'")
        c.check(b.t0 - b.start < 2 * MS, f"boot at {b.start / MS:.3f} ms took {(b.t0 - b.start) / MS:.3f} ms to the loop")


def held_bus(s, bus, t, overflowing=False):
    """Inside a mailbox hold on that bus (one that overflows the queue, if
    asked), up to the drain at the first send after it"""
    return any(h_bus == bus and a <= t <= drain and (over or not overflowing)
               for h_bus, a, drain, over in getattr(s, "holds_until", []))


def drain_times(s, r):
    """Each hold lasts, as far as frames go, until the queue empties at the
    first send on that bus after it"""
    s.holds_until = []
    for bus, a, z in s.mailbox_holds:
        after = [f.t for f in r.frames if f.bus == bus and f.t >= z]
        s.holds_until.append((bus, a, after[0] + 200 if after else s.end, (bus, a, z) in s.overflows))


def check_schedule(c, s, r, ex):
    """Every job's frames at its deadline, in main()'s order when jobs
    coincide, the 5 ms copies, MBB_Alive, and nothing else on either bus"""
    drain_times(s, r)
    for b in ex.boots:
        frames = [f for f in r.frames if b.start <= f.t < b.end]
        if b.t0 is None:
            c.check(not frames, f"frames from a boot that never reached the loop: {frames[:3]}")
            continue
        streams = {}
        for f in frames:
            streams.setdefault((f.bus, f.id), []).append(f)
        instances = []  # (deadline, job order, name, k, (bus, id))
        for order, (name, period, ids) in enumerate(JOBS):
            k = 1
            while b.t0 + k * period < b.end:
                for key in ids:
                    instances.append((b.t0 + k * period, order, name, k, key))
                k += 1
        for deadline, order, name, k, key in instances:
            if held_bus(s, key[0], deadline):
                # Queued until the mailboxes free up, in order. Holds that
                # overflow the queue are checked by their scenario.
                if held_bus(s, key[0], deadline, overflowing=True):
                    continue
                late = [f for f in streams.get(key, []) if f.job is None and f.t >= deadline]
                if c.check(late, f"{name} #{k} (due {deadline / MS:.3f} ms, mailboxes held) never went out"):
                    late[0].job, late[0].k, late[0].at = name, k, deadline
                continue
            match = [f for f in streams.get(key, []) if deadline - 2 <= f.t <= deadline + JOB_LATE and f.job is None]
            # Due right before a reset or the end, it may not have gone out
            optional = deadline + JOB_LATE >= b.end
            if c.check(len(match) == 1 or (optional and not match),
                       f"{name} #{k} (due {deadline / MS:.3f} ms): {len(match)} {key[0]}:{key[1]} frames"):
                if match:
                    match[0].job, match[0].k = name, k
        # The copies of each 40 ms frame pair go out on CAN_D 5 ms after that
        # job ran (its frames can be later if CAN_P's mailboxes are held)
        for f in streams.get((P, THROTTLE), []):
            ran = b.t0 + f.k * 40 * MS if f.job == "pt40" else None
            if ran is None:
                continue
            pair = [g for g in streams.get((P, CURRENTS), []) if g.job == "pt40" and g.k == f.k]
            for key, original in (((D, THROTTLE), f), ((D, CURRENTS), pair[0] if pair else None)):
                if original is None:
                    continue
                due = ran + COPY_DELAY
                if held_bus(s, D, due):
                    if held_bus(s, D, due, overflowing=True):
                        continue
                    late = [g for g in streams.get(key, []) if g.job is None and g.t >= due]
                    if c.check(late, f"copy of {original} (mailboxes held) never went out"):
                        late[0].job, late[0].k, late[0].at = "copy", f.k, due
                        c.check(late[0].data == original.data, f"CAN_D copy {late[0]} differs from {original}")
                    continue
                match = [g for g in streams.get(key, []) if due <= g.t <= due + 2 * JOB_LATE and g.job is None]
                optional = due + 2 * JOB_LATE >= b.end
                if c.check(len(match) == 1 or (optional and not match),
                           f"copy of {original} on CAN_D: {len(match)} frames 5 ms later"):
                    if match:
                        match[0].job, match[0].k = "copy", f.k
                        c.check(match[0].data == original.data, f"CAN_D copy {match[0]} differs from {original}")
        # Forwarded frames are checked in check_forwarding
        for f in frames:
            if f.job is None and held_bus(s, f.bus, f.t, overflowing=True):
                f.job = "held"
            elif f.job is None and not (f.bus == D and f.id in FORWARDED):
                c.check(False, f"unexpected frame {f}")
        # Order inside a pass: data80, etc50, pt40, imu10
        by_deadline = {}
        for f in frames:
            if f.job in ("data80", "etc50", "pt40", "imu10") and f.at == f.t:
                period = dict((n, p) for n, p, _ in JOBS)[f.job]
                by_deadline.setdefault(b.t0 + f.k * period, []).append(f)
        names = [n for n, _, _ in JOBS]
        expected_order = {n: [key for key in ids] for n, _, ids in JOBS}
        for deadline, group in by_deadline.items():
            group.sort(key=lambda f: f.index)
            sequence = [(f.job, (f.bus, f.id)) for f in group]
            want = [(n, key) for n in names for key in expected_order[n] if (n, key) in sequence]
            if c.check(sequence == want, f"frames due at {deadline / MS:.3f} ms in the wrong order: "
                                         f"{[f'{j}:{k[0]}:{k[1]}' for j, k in sequence]}"):
                if len({f.job for f in group}) > 1:
                    c.count("coinciding jobs")
        # MBB_Alive: +1 per 40 ms job from 1 after boot; the 80 ms frames
        # carry the value of the 40 ms job before them
        for f in streams.get((P, THROTTLE), []):
            if f.job == "pt40":
                c.check(f.data[5] == f.k % 16, f"MBB_Alive {f.data[5]} in 40 ms frame #{f.k}, expected {f.k % 16}: {f}")
        for f in streams.get((D, THROTTLE), []):
            if f.job == "data80":
                c.check(f.data[5] == (2 * f.k - 1) % 16,
                        f"MBB_Alive {f.data[5]} in 80 ms frame #{f.k}, expected {(2 * f.k - 1) % 16}: {f}")
        c.count("scheduled frames", sum(1 for f in frames if f.job))


def quiet_since(changes, t, window):
    """No input change in (t - window, t]"""
    return not any(t - window < at <= t for at, _ in changes)


def check_frames(c, s, r, ex):
    """Contents of the 390/646/402/403 frames against the model"""
    for f in r.frames:
        if f.job in (None, "held"):
            continue
        c.check(len(f.data) == DLC[f.id] and not f.ext and not f.rtr, f"wrong DLC or format: {f}")
        if f.job == "copy":
            continue  # compared with its original in check_schedule
        if f.id == THROTTLE:
            check_throttle(c, ex, f)
        elif f.id == CURRENTS:
            c.check(f.data == bytes.fromhex("64003A0200000000"), f"646 should be 100 A / 570 A and zeros: {f}")
        elif f.id == PEDALS:
            check_pedals(c, ex, f)
        elif f.id == STATUS:
            check_status(c, s, ex, f)


def check_throttle(c, ex, f):
    t = f.at
    c.check(f.u16(2) == 7500 and f.data[6:8] == b"\0\0" and f.data[4] & 0xF7 == 0x01,
            f"390 MaxSpeed, Forward/Reverse or unused bytes wrong: {f}")
    en = ex.signal(t, "en")
    if en is not None and c.check((f.data[4] >> 3) & 1 == en, f"PowerReady {(f.data[4] >> 3) & 1}, expected {en}: {f}"):
        c.count("PowerReady bits")
    allowed = torque_options(ex, t)
    if allowed:
        if c.check(f.i16(0) in allowed, f"torque {f.i16(0)}, expected {describe(allowed)}: {f}"):
            c.count("torques" if len(allowed) <= 3 else "torque ranges")
            if len(allowed) <= 3:
                c.count(f"torque {max(allowed)}")


def describe(values):
    values = sorted(values)
    return str(values) if len(values) <= 4 else f"{values[0]}..{values[-1]}"


def torque_options(ex, t):
    """What motor_torque.read() can be at t. Once the pedal has settled,
    the set the 40 Hz filter can stick at; while it moves, anything between
    the unfiltered torques of the last 60 ms (the filter averages them, and
    what came before has decayed to e^-15)."""
    b = ex.boot_at(t)
    if b.t0 is None or t < b.t0 + 60 * MS:
        return None
    if ex.settled(t, (APPS1, APPS2), SETTLE_TORQUE):
        torques = ex.etc(t).torques()
        return model.torque_reading(min(torques), max(torques))
    times = {t - 60 * MS, t}
    for ch in (APPS1, APPS2):
        for tc in b.filters[ch].change_times():
            if t - 60 * MS < tc < t:
                times.update((tc, tc + PASS_MAX))
    lo, hi = None, None
    for x in times:
        torques = EtcAt(b.filters, x).torques()
        lo = min(torques) if lo is None else min(lo, min(torques))
        hi = max(torques) if hi is None else max(hi, max(torques))
    return model.torque_reading(lo, hi)


def check_pedals(c, ex, f):
    t = f.at
    e = ex.etc(t)
    checks = [(0, e.v1), (2, e.v2), (4, e.vb)]
    for at, interval in checks:
        want = int_set(interval, model.millivolts)
        if c.check(f.u16(at) in want, f"402 bytes {at}-{at + 1}: {f.u16(at)}, expected {describe(want)}: {f}"):
            c.count("402 voltages")
    want = e.mapped_percents()
    if c.check(f.data[6] in want, f"402 byte 6 (pedal map output %): {f.data[6]}, expected {describe(want)}: {f}"):
        c.count("402 pedal")
    want = int_set(e.bpps, model.percent)
    if c.check(f.data[7] in want, f"402 byte 7 (BPPS %): {f.data[7]}, expected {describe(want)}: {f}"):
        c.count("402 brake")


def check_status(c, s, ex, f):
    t = f.at
    b = ex.boot_at(t)
    names = ["rtd", "en", None, "precharged", "apps", "bpps", "dev", "bse"]
    for bit, name in enumerate(names):
        got = (f.data[0] >> bit) & 1
        if name is None:  # RTD button pin, read every pass
            want = s.pin_at("C", 13, t) if quiet_since(s.pins[("C", 13)], t, REACT) else None
        elif name == "precharged":
            want = b.precharged.at(t)
        else:
            want = ex.signal(t, name)
        if want is not None:
            c.check(got == want, f"403 byte 0 bit {bit} ({name or 'button'}) {got}, expected {want}: {f}")
    want = {0: ex.signal(t, "ba"), 1: 0, 2: b.brakelight.at(t), 3: 0, 4: 1, 5: 0}
    for port_pin, bit in ((2, 6), (3, 7)):
        changes = s.pins[("A", port_pin)]
        want[bit] = s.pin_at("A", port_pin, t) if quiet_since(changes, t, REACT) else None
    for bit, value in want.items():
        got = (f.data[1] >> bit) & 1
        if value is not None:
            c.check(got == value, f"403 byte 1 bit {bit} {got}, expected {value}: {f}")
    e = ex.etc(t)
    for at, interval in ((2, e.psi_f), (4, e.psi_r)):
        want = int_set(interval, lambda p: model.trunc_int(p, 16, False))
        if c.check(f.u16(at) in want, f"403 bytes {at}-{at + 1} (pressure): {f.u16(at)}, expected {describe(want)}: {f}"):
            c.count("403 pressures")
    if quiet_since(s.adc[STEERING], t, REACT):
        want = model.steering_x100(s.code_at(STEERING, t))
        if c.check(f.i16(6) == want, f"403 steering {f.i16(6)}, expected {want}: {f}"):
            c.count("403 steering")


def check_forwarding(c, s, r, ex):
    """1154 and 1666 from CAN_P go out on CAN_D unchanged, within a pass,
    in order; nothing else is forwarded"""
    forwarded = [f for f in r.frames if f.bus == D and f.id in FORWARDED and f.job is None]
    used = set()
    last_index = -1
    worst = 0
    for t, bus, can_id, ext, rtr, data in sorted(s.rx, key=lambda x: x[0]):
        b = ex.boot_at(t)
        if bus != P or can_id not in FORWARDED or b.t0 is None or t < b.t0 or held_bus(s, D, t):
            continue
        match = [f for f in forwarded if id(f) not in used and f.id == can_id and f.ext == ext and f.rtr == rtr
                 and len(f.data) == len(data) and (rtr or f.data == data) and t <= f.t <= t + REACT]
        if c.check(match, f"{can_id}{'x' if ext else ''}{'r' if rtr else ''} {data.hex()} received at {t / MS:.3f} ms "
                          f"not forwarded within {REACT} us"):
            f = match[0]
            used.add(id(f))
            c.check(f.index > last_index, f"forwarded out of order: {f}")
            last_index = f.index
            worst = max(worst, f.t - t)
            c.count("forwarded")
    for f in forwarded:
        if id(f) not in used and not held_bus(s, D, f.t):
            c.check(False, f"forwarded a frame that wasn't received on CAN_P: {f}")
    c.forward_latency = worst


def traction_updates(s, b):
    """(earliest, latest, Traction state after) for each traction update in a
    boot. An update runs in the pass after the fourth wheel frame arrives."""
    tc = model.Traction()
    # The loop timer started at construction, before TIM5 ran: time 0
    restart = (b.t0 - b.now0, b.t0 - b.now0)
    seen = {}
    updates = []
    for t, bus, can_id, ext, rtr, data in sorted(s.rx, key=lambda x: x[0]):
        if bus != D or can_id not in WHEELS or not b.t0 < t < b.end:
            continue
        seen[can_id] = model.wheel_rpm(data[0] | data[1] << 8)
        if len(seen) == 4:
            lo, hi = t, t + REACT
            tc.update(seen[421], seen[422], seen[423], seen[424], lo - restart[1], hi - restart[0])
            restart = (lo, hi)
            updates.append((lo, hi, {k: set(v) if isinstance(v, set) else v for k, v in tc.frame_options().items()}))
            seen = {}
    return updates


def check_traction(c, s, r, ex):
    for b in ex.running:
        updates = traction_updates(s, b)
        initial = {"slip": {0}, "output": {100}, "integral": {0}, "loop_ms": {0}, "raw": {0}, "smoothed": {0}}
        for f in r.frames:
            if f.job != "data80" or f.id != TRACTION or not b.start <= f.t < b.end:
                continue
            if any(lo <= f.at <= hi for lo, hi, _ in updates):
                continue
            done = [u for u in updates if u[1] < f.at]
            want = done[-1][2] if done else initial
            got = {"slip": f.data[0], "output": f.data[1], "integral": f.data[2], "loop_ms": f.data[3],
                   "raw": f.i16(4), "smoothed": f.i16(6)}
            wrong = {k: (v, sorted(want[k])) for k, v in got.items() if v not in want[k]}
            if c.check(not wrong, f"660 fields {wrong} (got, expected): {f}"):
                c.count("660 frames")
                if done:
                    c.count("660 after an update")


class VnBoot:
    """What the VN-200 driver did in one boot, worked out from the fake
    sensor's log and checked step by step against the driver's rules"""

    def __init__(self):
        self.running = []   # (from, to): it was parsing packets
        self.packets = []   # (done, n) good messages it decoded
        self.bad = []       # done times of corrupt messages it saw
        self.configs = []   # times a configuration started
        self.errors = []    # (done, code) $VNERR lines it got


def vn_log(r):
    """The fake sensor's log: commands it got, replies and messages it sent"""
    sent = [(t, text[3:]) for t, text in r.vn if text.startswith("rx ")]
    replies, packets = [], []
    for t, text in r.vn:
        if text.startswith("tx "):
            _, done, line = text.split(" ", 2)
            replies.append((int(done), line))
        elif text.startswith("pkt "):
            parts = text.split()
            packets.append((t, int(parts[2]), int(parts[4]), "corrupt" in parts))
    return sent, replies, packets


COMMAND_ERRORS = (2, 3, 4, 5, 6, 7, 8, 9, 12)  # ICD table 1.6, answers to a command


def check_vn200(c, s, r, ex):
    """Walks through the commands the driver sent, checking each against
    Vn200's rules: the next command once the echo is in, a retry after a
    command error or 100 ms without a reply, a 1 s back off after 3 tries,
    and a new configuration 500 ms after the last good message. Also works
    out which messages it decoded, for the IMU frames and the debug line."""
    sent, replies, packets = vn_log(r)
    ex.vn = {}
    for b in ex.running:
        vb = VnBoot()
        ex.vn[id(b)] = vb
        vb.errors = [(done, int(line[7:9], 16)) for done, line in replies
                     if line.startswith("$VNERR,") and b.start <= done < b.end]
        mine = [(t, line) for t, line in sent if b.t0 - 10 <= t < b.end]
        idx = tries = 0
        due = (b.t0 - 10, b.t0 + REACT)  # start(now) sends the first command at once
        for n, (t, line) in enumerate(mine):
            if not c.check(line == VN_COMMANDS[idx], f"VN-200 command {n + 1} after the boot at {b.start / MS:.0f} ms: "
                                                    f"{line!r}, expected {VN_COMMANDS[idx]!r}"):
                break
            if not c.check(due and due[0] <= t <= due[1], f"{line} at {t / MS:.3f} ms, expected "
                                                          f"{due and due[0] / MS:.3f}-{due and due[1] / MS:.3f} ms"):
                break
            c.count("VN-200 commands")
            if idx == 0 and tries == 0:
                vb.configs.append(t)
            tries += 1
            answers = [(done, reply) for done, reply in replies if done > t and (
                reply == VN_REPLIES.get(line, line) or (reply.startswith("$VNERR,") and int(reply[7:9], 16) in COMMAND_ERRORS))]
            answer = answers[0] if answers else None
            timeout = t + VN_RESPONSE
            if answer and answer[0] + PASS_MAX < timeout - PASS_MAX:
                done, reply = answer
                if reply != VN_REPLIES.get(line, line):
                    due = (done - 50, done + REACT)
                    if tries == 3:
                        due = (done + VN_BACKOFF - 50, done + VN_BACKOFF + REACT + PASS_MAX)
                        idx = tries = 0
                    continue
                idx, tries = idx + 1, 0
                due = (done - 50, done + REACT)
                if idx < len(VN_COMMANDS):
                    continue
                # Configured: it parses messages until none good comes for 500 ms
                idx = 0
                last = (done - 50, done + PASS_MAX)
                for q, k, d, bad in packets:
                    if q <= done or not b.start <= q < b.end:
                        continue
                    if d >= last[0] + VN_DATA_TIMEOUT:
                        c.check(d > last[1] + VN_DATA_TIMEOUT + PASS_MAX, f"message {k} at {d / MS:.3f} ms is too "
                                                                          "close to the 500 ms timeout to say")
                        break
                    if bad:
                        vb.bad.append(d)
                    else:
                        vb.packets.append((d, k))
                        last = (d - 50, d + PASS_MAX)
                due = (last[0] + VN_DATA_TIMEOUT, last[1] + VN_DATA_TIMEOUT + PASS_MAX)
                vb.running.append((done + PASS_MAX, min(b.end, due[0])))
            else:
                c.check(not answer or answer[0] > timeout + PASS_MAX,
                        f"reply to {line} at {answer and answer[0] / MS:.3f} ms is too close to its timeout to say")
                due = (timeout - PASS_MAX, timeout + REACT)
                if tries == 3:
                    due = (due[0] + VN_BACKOFF, due[1] + VN_BACKOFF + PASS_MAX)
                    idx = tries = 0
        if due and due[1] + 10 * MS < b.end and not any(due[0] <= t <= due[1] for t, _ in mine):
            c.check(False, f"VN-200: no command at {due[0] / MS:.3f}-{due[1] / MS:.3f} ms")


def imu_options(vb, t):
    """Message numbers the IMU frames at t can carry (None: nothing yet)"""
    sure = [k for d, k in vb.packets if d + REACT < t]
    maybe = [k for d, k in vb.packets if t - REACT <= d <= t]
    options = set(maybe)
    options.add(sure[-1] if sure else None)
    return options


def check_imu_frames(c, s, r, ex):
    for b in ex.running:
        vb = ex.vn.get(id(b))
        if vb is None:
            continue
        for f in r.frames:
            if f.job != "imu10" or not b.start <= f.t < b.end:
                continue
            options = imu_options(vb, f.at)
            wanted = {model.imu_frames(n)[f.id] for n in options}
            if c.check(f.data in wanted, f"IMU frame {f} is none of messages {sorted(options, key=str)}"):
                c.count("IMU frames")
                if None not in options:
                    c.count("IMU frames with data")


# Lines print_imu_event() prints, as the Mbed build's VectorNav wrapper did
IMU_LINE = re.compile(r"Connected to sensor!$|Sensor Model Number: |baud: |Binary output messages configured\.$|"
                      r"Received async error: |VN: ")
VN_ERROR_NAMES = {0x01: "HardFault", 0x02: "SerialBufferOverflow", 0x03: "InvalidChecksum", 0x04: "InvalidCommand",
                  0x05: "NotEnoughParameters", 0x06: "TooManyParameters", 0x07: "InvalidParameter",
                  0x08: "InvalidRegister", 0x09: "UnauthorizedAccess", 0x0A: "WatchdogReset",
                  0x0B: "OutputBufferOverflow", 0x0C: "InsufficientBaudRate", 0xFF: "ErrorBufferOverflow"}


def check_imu_console(c, s, r, ex):
    """The IMU's console lines: the model and "configured" for each completed
    setup, one line per $VNERR the driver got, with its name"""
    for b in ex.running:
        vb = ex.vn.get(id(b))
        if vb is None:
            continue
        lines = [line for t, line in r.uart if b.start <= t < b.end and IMU_LINE.match(line)]
        near_end = any(z > b.end - REACT for z, _ in vb.running) or any(d > b.end - REACT for d, _ in vb.errors)
        configured = lines.count("Binary output messages configured.")
        c.check(configured == len(vb.running) or (near_end and configured == len(vb.running) - 1),
                f"{configured} 'configured' lines in the boot at {b.start / MS:.0f} ms, expected {len(vb.running)}")
        for i, line in enumerate(lines):
            if line == "Binary output messages configured.":
                c.check(lines[max(0, i - 3):i] == ["Connected to sensor!", f"Sensor Model Number: {VN_MODEL}",
                                                   "baud: 115200"],
                        f"before {line!r}: {lines[max(0, i - 3):i]}, expected the model lines")
        errors = []
        for line in lines:
            m = re.fullmatch(r"Received async error: (\w+)|VN: Error \d+ \((\w+)\) in reply to a setup command", line)
            if m:
                errors.append(m.group(1) or m.group(2))
            else:
                c.check(re.fullmatch(r"Connected to sensor!|Sensor Model Number: .*|baud: 115200|"
                                     r"Binary output messages configured\.|"
                                     r"VN: Error (303 \(ResponseTimeout\)|\d+ \(\w+\)), setup tried again in 1 s|"
                                     r"VN: no data for 500 ms, setting the sensor up again", line),
                        f"IMU console line {line!r}")
        want = [VN_ERROR_NAMES.get(code, "Unknown") for _, code in vb.errors]
        c.check(errors == want or (near_end and errors == want[:len(errors)]),
                f"IMU error lines {errors}, expected {want}")
        c.count("IMU console lines", len(lines))


def check_debug_lines(c, s, r, ex):
    """Two lines every 500 ms, each value against the model"""
    for b in ex.running:
        lines = [(t, line) for t, line in r.uart if b.start <= t < b.end
                 and not line.startswith(("Hello", "Reset cause")) and not IMU_LINE.match(line)]
        k = 1
        expected = []
        while b.t0 + k * DEBUG_PERIOD + JOB_LATE < b.end:
            expected.append(b.t0 + k * DEBUG_PERIOD)
            k += 1
        c.check(len(lines) == 2 * len(expected), f"{len(lines)} debug lines in the boot at {b.start / MS:.0f} ms, "
                                                  f"expected {2 * len(expected)}")
        vb = ex.vn.get(id(b))
        for due, ((t1, line1), (t2, line2)) in zip(expected, zip(lines[0::2], lines[1::2])):
            c.check(due <= t1 <= due + JOB_LATE and t1 <= t2 <= t1 + JOB_LATE,
                    f"debug lines at {t1 / MS:.3f}/{t2 / MS:.3f} ms, due {due / MS:.3f}")
            m1, m2 = DEBUG_LINE_1.match(line1), DEBUG_LINE_2.match(line2)
            if not c.check(m1 and m2, f"debug lines don't parse: {line1!r} / {line2!r}"):
                continue
            c.count("debug lines")
            t = due  # print_debug() runs right after update_state() in that pass
            e = ex.etc(t)
            for group, interval in ((1, e.v1), (2, e.v2), (3, e.vb), (4, e.vf), (5, e.vr)):
                c.check(printed(interval, m1.group(group)),
                        f"debug voltage {m1.group(group)}, expected {interval.lo:.4f}-{interval.hi:.4f}: {line1!r}")
            allowed = torque_options(ex, t)
            if allowed:
                c.check(int(m1.group(6)) in allowed, f"debug torque {m1.group(6)}, expected {describe(allowed)}: {line1!r}")
            for group, name in ((7, "rtd"), (8, "en"), (9, "dev"), (10, "apps"), (11, "bpps"), (12, "bse"), (13, "ba")):
                want = ex.signal(t, name)
                if want is not None:
                    c.check(int(m1.group(group)) == want, f"debug {name} {m1.group(group)}, expected {want}: {line1!r}")
            drops = s.expected_drops(t, b)
            (tec_p, off_p), (tec_d, off_d) = s.expected_can_errors(t)
            got = (int(m2.group(1)), bool(m2.group(2)), int(m2.group(4)), bool(m2.group(5)))
            c.check(got == (tec_p, off_p, tec_d, off_d), f"CAN error counts / bus-off {got}, "
                                                         f"expected {(tec_p, off_p, tec_d, off_d)}: {line2!r}")
            c.check((int(m2.group(3)), int(m2.group(6))) == drops, f"CAN drop counts {m2.group(3)}/{m2.group(6)}, "
                                                                   f"expected {drops}: {line2!r}")
            c.check(int(m2.group(10)) == 0, f"IMU bytes lost: {line2!r}")
            loop_max, passes = int(m2.group(13)), int(m2.group(14))
            c.check(loop_max <= PASS_MAX, f"loop max {loop_max} us is over the {PASS_MAX} us the checks assume: {line2!r}")
            c.check(DEBUG_PERIOD / 150 <= passes <= DEBUG_PERIOD / 30, f"{passes} passes in 500 ms: {line2!r}")
            if vb is None:
                continue
            running = [any(a <= x < z for a, z in vb.running) for x in (t - REACT, t + REACT)]
            if running[0] == running[1]:
                c.check(m2.group(7) == ("ok" if running[0] else "setup"), f"IMU state {m2.group(7)}: {line2!r}")
            pkts = (sum(1 for d, _ in vb.packets if d + REACT < t), sum(1 for d, _ in vb.packets if d <= t))
            crcs = (sum(1 for d in vb.bad if d + REACT < t), sum(1 for d in vb.bad if d <= t))
            configs = (sum(1 for x in vb.configs if x + REACT < t), sum(1 for x in vb.configs if x <= t))
            errors = [code for d, code in vb.errors if d + REACT < t]
            maybe_errors = [code for d, code in vb.errors if d <= t]
            for group, (lo, hi), what in ((8, pkts, "packets"), (9, crcs, "CRC errors"), (11, configs, "configurations")):
                c.check(lo <= int(m2.group(group)) <= hi, f"IMU {what} {m2.group(group)}, expected {lo}-{hi}: {line2!r}")
            want = {f"{errors[-1] if errors else 0:02X}", f"{maybe_errors[-1] if maybe_errors else 0:02X}"}
            c.check(m2.group(12) in want, f"IMU last error {m2.group(12)}, expected {want}: {line2!r}")


def check_pins(c, s, r, ex):
    for name in ("PC0", "PA7", "PB1", "PC4"):
        if compare_levels(c, name, r.pins[name], ex.pins[name], 0, s.end, 2):
            c.count(f"{name} changes", len(r.pins[name]))


def check_ram(c, s, r, ex):
    """The ETC flags, RTD and motor_enabled, polled in RAM every 100 us"""
    for b in ex.running:
        for name in RAM_FIELDS:
            observed = [(t, v) for t, v in r.ram[name] if b.t0 <= t < b.end]
            initial = level_at([(t, v) for t, v in r.ram[name] if t < b.t0 + SAMPLE], b.t0 + SAMPLE)
            observed = [(b.t0, initial)] + observed
            predicted = {"rtd": b.rtd, "en": b.enabled}.get(name) or b.flags[name]
            compare_levels(c, f"{name} (RAM)", observed, predicted, b.t0 + SAMPLE, b.end, SAMPLE + 5)


def check_outputs_before_clocks(c, s, r, ex):
    """board_init() drives the four outputs low before the watchdog and the
    clocks start"""
    outputs = {"A": [7], "B": [1], "C": [0, 4]}
    for where in ("HAL_IWDG_Init", "HAL_RCC_OscConfig"):
        values = {label: v for t, label, v in r.audit if label.startswith(where) and t < ex.boots[0].end}
        for port, pins in outputs.items():
            moder, odr = values.get(f"{where} {port}_MODER"), values.get(f"{where} {port}_ODR")
            if not c.check(moder is not None and odr is not None, f"no pin snapshot at {where}"):
                continue
            for pin in pins:
                c.check((moder >> 2 * pin) & 3 == 1 and not (odr >> pin) & 1,
                        f"P{port}{pin} at {where}: mode {(moder >> 2 * pin) & 3}, level {(odr >> pin) & 1}, "
                        "expected a low output")


def bits(*numbers):
    value = 0
    for n in numbers:
        value |= 1 << n
    return value


# Expected setup after boot: (name, address, mask, value), from board.c,
# gpio.c, adc.c, can.c, console.c, imu_uart.c, timebase.c, watchdog.c and
# RM0390. Reset values of the untouched pins are Renode's (SWD on PA13-15,
# PB3-4). Output type (push-pull) isn't modelled, so it isn't checked.
REGISTERS = [
    ("RCC_CR", 0x40023800, bits(16, 18, 24), bits(16, 24)),                          # HSEON, no bypass, PLLON
    ("RCC_PLLCFGR", 0x40023804, 0x0F437FFF, 8 << 24 | 1 << 22 | 0 << 16 | 180 << 6 | 12),  # Q8 HSE P2 N180 M12
    ("RCC_CFGR", 0x40023808, 0xFCF3, 0b100 << 13 | 0b101 << 10 | 2),                # APB2 /2, APB1 /4, PLL
    ("RCC_AHB1ENR", 0x40023830, bits(0, 1, 2, 3, 4, 5, 6, 7, 21, 22), bits(0, 1, 2, 3, 21)),  # GPIOA-D, DMA1
    ("RCC_APB1ENR", 0x40023840, 0x3FFEC9FF, bits(3, 19, 20, 25, 26, 28)),  # TIM5 UART4 UART5 CAN1 CAN2 PWR
    ("RCC_APB2ENR", 0x40023844, 0x00C77F33, bits(8, 14)),                    # ADC1, SYSCFG
    ("FLASH_ACR", 0x40023C00, 0x10F, 5),                                    # 5 WS, prefetch off
    ("PWR_CR", 0x40007000, 0x3C000, 0x3C000),                               # VOS scale 1, over-drive
    ("GPIOA_MODER", 0x40020000, 0xFFFFFFFF, 0xA8000000 | 0b1111 | 0b01 << 14),  # PA0-1 analog, PA2-3 in, PA7 out
    ("GPIOA_OSPEEDR", 0x40020008, 0xFFFFFFFF, 0x0C000000),
    ("GPIOA_PUPDR", 0x4002000C, 0xFFFFFFFF, 0x64000000),
    ("GPIOA_AFRL", 0x40020020, 0xFFFFFFFF, 0),
    ("GPIOB_MODER", 0x40020400, 0xFFFFFFFF, 0x280 | 0b01 << 2 | 0b10 << 10 | 0b10 << 12 | 0b10 << 16 | 0b10 << 18),
    ("GPIOB_OSPEEDR", 0x40020408, 0xFFFFFFFF, 0xC0 | 0b11 << 10 | 0b11 << 12 | 0b11 << 16 | 0b11 << 18),
    ("GPIOB_PUPDR", 0x4002040C, 0xFFFFFFFF, 0x100 | 0b01 << 10 | 0b01 << 12 | 0b01 << 16 | 0b01 << 18),
    ("GPIOB_AFRL", 0x40020420, 0xFFFFFFFF, 9 << 20 | 9 << 24),               # PB5, PB6 AF9 (CAN2)
    ("GPIOB_AFRH", 0x40020424, 0xFFFFFFFF, 9 << 0 | 9 << 4),                 # PB8, PB9 AF9 (CAN1)
    ("GPIOC_MODER", 0x40020800, 0xFFFFFFFF,
     0b01 | 0b111111 << 2 | 0b01 << 8 | 0b11 << 10 | 0b10 << 20 | 0b10 << 22 | 0b10 << 24),
    ("GPIOC_OSPEEDR", 0x40020808, 0xFFFFFFFF, 0b11 << 20 | 0b11 << 22 | 0b10 << 24),
    ("GPIOC_PUPDR", 0x4002080C, 0xFFFFFFFF, 0b01 << 20 | 0b01 << 22 | 0b01 << 24),  # pull-ups on PC10-12 only
    ("GPIOC_AFRL", 0x40020820, 0xFFFFFFFF, 0),
    ("GPIOC_AFRH", 0x40020824, 0xFFFFFFFF, 8 << 8 | 8 << 12 | 8 << 16),     # PC10-11 UART4, PC12 UART5
    ("GPIOD_MODER", 0x40020C00, 0xFFFFFFFF, 0b10 << 4),
    ("GPIOD_OSPEEDR", 0x40020C08, 0xFFFFFFFF, 0b10 << 4),
    ("GPIOD_PUPDR", 0x40020C0C, 0xFFFFFFFF, 0b01 << 4),
    ("GPIOD_AFRL", 0x40020C20, 0xFFFFFFFF, 8 << 8),                         # PD2 UART5
    ("EXTI_IMR", 0x40013C00, 0x7FFFFF, bits(13)),
    ("EXTI_EMR", 0x40013C04, 0x7FFFFF, 0),
    ("EXTI_RTSR", 0x40013C08, 0x7FFFFF, bits(13)),
    ("EXTI_FTSR", 0x40013C0C, 0x7FFFFF, 0),
    ("SYSCFG_EXTICR1", 0x40013808, 0xFFFF, 0),
    ("SYSCFG_EXTICR4", 0x40013814, 0xFFFF, 2 << 4),                         # line 13 from port C
    ("CAN1_MCR", 0x40006400, 0xFFFFFFFF, 1 << 16),                          # running; DBF stays at reset
    ("CAN1_MSR", 0x40006404, 0x3, 0),
    ("CAN1_IER", 0x40006414, 0xFFFFFFFF, bits(0, 1, 3)),                    # TME, FMP0, FOV0
    ("CAN2_MCR", 0x40006800, 0xFFFFFFFF, 1 << 16),
    ("CAN2_MSR", 0x40006804, 0x3, 0),
    ("CAN2_IER", 0x40006814, 0xFFFFFFFF, bits(0, 1, 3)),
    ("CAN_FMR", 0x40006600, 0x3F01, 14 << 8),                               # CAN2 from bank 14, not in init
    ("CAN_FM1R", 0x40006604, 0x0FFFFFFF, 0),                                # mask mode
    ("CAN_FS1R", 0x4000660C, bits(0, 14), bits(0, 14)),                     # 32-bit
    ("CAN_FFA1R", 0x40006614, 0x0FFFFFFF, 0),                               # FIFO0
    ("CAN_FA1R", 0x4000661C, 0x0FFFFFFF, bits(0, 14)),                      # banks 0 (CAN1) and 14 (CAN2)
    ("CAN_F0R1", 0x40006640, 0xFFFFFFFF, 0),
    ("CAN_F0R2", 0x40006644, 0xFFFFFFFF, 0),                                # mask 0: everything
    ("CAN_F14R1", 0x400066B0, 0xFFFFFFFF, 0),
    ("CAN_F14R2", 0x400066B4, 0xFFFFFFFF, 0),
    ("UART4_BRR", 0x40004C08, 0xFFFF, 0x187),                               # 115200 at 45 MHz
    ("UART4_CR1", 0x40004C0C, 0xBF3F, bits(2, 3, 13)),                      # RE TE UE, 8N1, OVER16
    ("UART4_CR2", 0x40004C10, 0x3000, 0),
    ("UART5_BRR", 0x40005008, 0xFFFF, 0x187),
    ("UART5_CR1", 0x4000500C, 0xBF3F, bits(2, 3, 13)),
    ("UART5_CR2", 0x40005010, 0x3000, 0),
    ("UART5_CR3", 0x40005014, 0xC0, bits(6)),                               # RX by DMA
    ("DMA1_S0CR", 0x40026010, 0x0FFFFFFF,
     4 << 25 | 2 << 16 | bits(10, 8, 4, 3, 2, 1, 0)),  # channel 4, high, MINC, CIRC, TC HT TE DME, on
    ("DMA1_S0PAR", 0x40026018, 0xFFFFFFFF, 0x40005004),                     # UART5_DR
    ("DMA1_S0FCR", 0x40026024, 0x7, 0),                                     # direct mode
    ("NVIC_ISER0", 0xE000E100, 0xFFFFFFFF, bits(11, 19, 20)),               # DMA1_S0, CAN1 TX, RX0
    ("NVIC_ISER1", 0xE000E104, 0xFFFFFFFF,
     bits(40 - 32, 50 - 32, 52 - 32, 53 - 32, 63 - 32)),  # EXTI15_10, TIM5, UART4, UART5, CAN2 TX
    ("NVIC_ISER2", 0xE000E108, 0xFFFFFFFF, bits(64 - 64)),                  # CAN2 RX0
    ("NVIC_IPR2", 0xE000E408, 0xFF000000, 0x50 << 24),                      # IRQ 11: priority 5
    ("NVIC_IPR4", 0xE000E410, 0xFF000000, 0x50 << 24),                      # IRQ 19
    ("NVIC_IPR5", 0xE000E414, 0xFF, 0x50),                                  # IRQ 20
    ("NVIC_IPR10", 0xE000E428, 0xFF, 0x50),                                 # IRQ 40
    ("NVIC_IPR12", 0xE000E430, 0xFF0000, 0x50 << 16),                       # IRQ 50
    ("NVIC_IPR13", 0xE000E434, 0xFFFF, 0x5050),                             # IRQ 52, 53
    ("NVIC_IPR15", 0xE000E43C, 0xFF000000, 0x50 << 24),                     # IRQ 63
    ("NVIC_IPR16", 0xE000E440, 0xFF, 0x50),                                 # IRQ 64
    ("SCB_SHPR3", 0xE000ED20, 0xFF000000, 0xF0000000),                      # SysTick priority 15
    ("SCB_AIRCR", 0xE000ED0C, 0x700, 0x300),                                # 4 bits preemption
    ("SYST_CSR", 0xE000E010, 0x7, 0x7),
    ("SYST_RVR", 0xE000E014, 0xFFFFFF, 179999),                             # 1 kHz at 180 MHz
    ("SCB_CPACR", 0xE000ED88, 0xF << 20, 0xF << 20),                        # FPU on
    ("TIM5_CR1", 0x40000C00, 0x91, 0x1),                                    # counting up
    ("TIM5_PSC", 0x40000C28, 0xFFFF, 89),                                   # 90 MHz / 90
    ("TIM5_ARR", 0x40000C2C, 0xFFFFFFFF, 0xFFFFFFFF),
    ("TIM5_DIER", 0x40000C0C, 0x5F5F, bits(0)),                             # update interrupt only
    ("IWDG_PR", 0x40003004, 0x7, 4),                                        # /64
    ("IWDG_RLR", 0x40003008, 0xFFF, 124),                                   # 125 counts = 250 ms
    ("DBGMCU_APB1_FZ", 0xE0042008, 0xFFFFFFFF, bits(3, 12)),                # TIM5 and IWDG stop in debug
    ("ADC1_CR1", 0x40012004, 0x3000120, 0),                                 # 12-bit, no scan, no EOC IRQ
    ("ADC1_CR2", 0x40012008, 0xF03, bits(0, 10)),                           # on, single, right aligned, EOC each
    ("ADC1_SMPR1", 0x4001200C, 0x7FFFFFF, 3 << 3 | 3 << 6 | 3 << 9 | 3 << 15),  # ch 11, 12, 13, 15: 56 cycles
    ("ADC1_SMPR2", 0x40012010, 0x3FFFFFFF, 3 << 0 | 3 << 3),                # ch 0, 1: 56 cycles
    ("ADC1_SQR1", 0x4001202C, 0xF << 20, 0),                                # 1 conversion
    ("ADC_CCR", 0x40012304, 0x3001F, 1 << 16),                              # ADCCLK = PCLK2 / 4
    # read for check_registers' own checks
    ("DMA1_S0NDTR", 0x40026014, 0, 0),
    ("DMA1_S0M0AR", 0x4002601C, 0, 0),
]


def check_registers(c, s, r, ex):
    for name, _, mask, value in REGISTERS:
        got = r.registers.get(name)
        if c.check(got is not None, f"couldn't read {name}"):
            c.check(got & mask == value, f"{name} = {got:#010x}, expected {value:#x} under mask {mask:#x}")
    ring = TOOLS.other_addresses["rx_ring"]
    got = r.registers.get("DMA1_S0M0AR")
    c.check(got == ring, f"DMA1 stream 0 memory address {got and hex(got)}, expected rx_ring at {ring:#x}")
    ndtr = r.registers.get("DMA1_S0NDTR")
    c.check(ndtr is not None and 1 <= ndtr <= 1024, f"DMA1 stream 0 NDTR {ndtr}, expected 1-1024 of the 1 KB ring")
    btr = {bus: {v for t, label, v in r.audit if label == f"BTR_{bus}" and v} for bus in (P, D)}
    c.check(btr[P] == {0x014B0004}, f"CAN1 BTR {sorted(hex(v) for v in btr[P])}, expected 0x14b0004 "
                                     "(500 kbit/s, 1 + 12 + 5 tq, SJW 2, same as Mbed)")
    c.check(btr[D] == {0x01390002}, f"CAN2 BTR {sorted(hex(v) for v in btr[D])}, expected 0x1390002 "
                                     "(1 Mbit/s, 1 + 10 + 4 tq, SJW 2, same as Mbed)")


# --- scenarios -----------------------------------------------------------------

def volts_band(code):
    """Filtered volts an input of this code can settle at, from either side"""
    r = model.adc_fraction(code)
    w = model.STUCK_APPS * model.ulp(r)
    return Interval(model.filtered_volts(r - w), model.filtered_volts(r + w))


def limit(start, step, verdict):
    """Walks codes from start until verdict(code) (0, 1 or None) turns 1.
    Returns (last code that's surely 0, first that's surely 1)."""
    code = start
    assert verdict(code) == 0, code
    good = code
    while True:
        code += step
        v = verdict(code)
        if v == 0:
            good = code
        elif v == 1:
            return good, code


def range_limits():
    """(name, channel, good code, bad code, rest code) at each range check"""
    out = []
    for name, ch, lo, hi, rest_low, rest_high in (
            ("APPS1", APPS1, model.APPS1_LOW, model.APPS1_HIGH, REST[0], apps_codes(1.0)[0]),
            ("APPS2", APPS2, model.APPS2_LOW, model.APPS2_HIGH, REST[1], apps_codes(1.0)[1]),
            ("BPPS", BPPS, model.BPPS_LOW, model.BPPS_HIGH, bpps_code(0.0), bpps_code(0.95)),
            ("front BSE", FRONT_BSE, model.BSE_LOW, model.BSE_HIGH, bse_code(10.0), bse_code(700.0)),
            ("rear BSE", REAR_BSE, model.BSE_LOW, model.BSE_HIGH, bse_code(10.0), bse_code(700.0))):
        verdict = lambda code, lo=lo, hi=hi: outside(volts_band(code), lo, hi)
        good, bad = limit(rest_low, -1, verdict)
        out.append((name + " low", ch, good, bad, rest_low))
        good, bad = limit(rest_high, 1, verdict)
        out.append((name + " high", ch, good, bad, rest_high))
    return out


def deviation_limit(travel):
    """APPS2 codes either side of the 10% deviation check, APPS1 at travel"""
    c1 = apps_codes(travel)[0]
    p1 = volts_band(c1).map(model.apps1_position)

    def verdict(c2):
        p2 = volts_band(c2).map(model.apps2_position)
        diffs = [abs(f32(a - b)) for a in (p1.lo, p1.hi) for b in (p2.lo, p2.hi)]
        overlap = p1.lo <= p2.hi and p2.lo <= p1.hi
        if (0.0 if overlap else min(diffs)) > model.MAX_DEVIATION:
            return 1
        return 0 if max(diffs) <= model.MAX_DEVIATION else None
    good, bad = limit(apps_codes(travel)[1], 1, verdict)
    return c1, good, bad


def code_limit(start, step, interval_of, test):
    """Codes either side of a threshold: test(interval) is 0, 1 or None"""
    return limit(start, step, lambda code: test(interval_of(code)))


def pedal_limit(test, start_travel):
    """APPS code pairs either side of a test on the averaged position"""
    def avg_of(codes):
        p1 = volts_band(codes[0]).map(model.apps1_position)
        p2 = volts_band(codes[1]).map(model.apps2_position)
        return Interval(model.average(p1.lo, p2.lo), model.average(p1.hi, p2.hi))
    travel, good = start_travel, None
    while True:
        codes = apps_codes(travel)
        v = test(avg_of(codes))
        if v == 0:
            good = codes
        elif v == 1:
            assert good
            return good, codes
        travel += 0.0002


def ready_to_drive(s, ms):
    """Battery ready, brake pressed, RTD button, brake released: RTD on
    about ms + 150"""
    s.frame(ms, P, BATTERY, [0x44])  # PRECHARGE_DONE (bit 6) and SHUTDOWN_FINAL (bit 2)
    s.brake(ms + 20, 0.3)
    s.button(ms + 150)
    s.brake(ms + 400, 0.0)


def expect_counts(c, wanted):
    for what, n in wanted.items():
        c.check(c.counts.get(what, 0) >= n, f"only {c.counts.get(what, 0)} {what} checked, expected at least {n}")


def ram_rises(r, name):
    out, last = [], 0
    for t, v in r.ram[name]:
        if v and not last:
            out.append(t)
        last = v
    return out


def scenarios():
    result = []

    # Boot with everything at rest: the whole schedule on both buses, the
    # register audit, the VN-200 setup, torque 0 at rest, nothing enabled
    s = Scenario("boot", 1.3, "boot, schedule, register audit", registers=True)

    def boot_check(c, s, r, ex):
        check_registers(c, s, r, ex)
        check_outputs_before_clocks(c, s, r, ex)
        expect_counts(c, {"coinciding jobs": 6, "torque 0": 25, "IMU frames with data": 300, "VN-200 commands": 6,
                          "403 steering": 20, "402 pedal": 20, "660 frames": 10})
        c.check(not r.pins["PC0"] and not r.pins["PA7"] and not r.pins["PB1"] and not r.pins["PC4"],
                f"an output changed at rest: {r.pins}")
    s.checks.append(boot_check)
    result.append(s)

    # Pedal map: torque across the travel, the deadzones, past full travel,
    # and two sensors that disagree by less than 10%
    s = Scenario("pedal_map", 5.0, "torque and 402 across the pedal travel")
    ready_to_drive(s, 100)
    points = [(0.0, 0.0), (0.02, 0.02), (0.04, 0.04), (0.06, 0.06), (0.25, 0.25), (0.5, 0.5), (0.75, 0.75),
              (0.96, 0.96), (0.98, 0.98), (1.0, 1.0), (1.012, 1.012), (0.30, 0.38), (0.62, 0.55), (0.5, 0.59)]
    for n, (a, b) in enumerate(points):
        s.apps(700 + 300 * n, a, b)
        s.set_adc(850 + 300 * n, STEERING, 280 + 75 * n)  # steering from full left to past full right
    s.apps(700 + 300 * len(points), 0.0)

    def pedal_check(c, s, r, ex):
        seen = {k for k in c.counts if k.startswith("torque ")}
        c.check(len(seen) >= len(points) - 3, f"torques checked at only {len(seen)} pedal positions: {sorted(seen)}")
        expect_counts(c, {"torques": 3 * len(points), "PowerReady bits": 100, "402 pedal": 3 * len(points)})
        c.check(len(ram_rises(r, "en")) == 1 and r.ram["en"][-1][1] == 1,
                f"motor should be enabled once and stay enabled: {r.ram['en'][:6]}")
    s.checks.append(pedal_check)
    result.append(s)

    # Each implausibility trips 101 ms after it starts and clears with it.
    # Faults start off the job grid. Range faults are at a pedal position
    # where the other sensor's position matches, so only the range check
    # trips.
    s = Scenario("implausibility_timing", 6.4, "each implausibility: trips after 100 ms, clears, glitches")
    ready_to_drive(s, 100)
    half = apps_codes(0.5)
    full = apps_codes(1.0)
    episodes = [  # (what, flag, fault inputs, restore inputs)
        ("APPS deviation", "dev", [(APPS2, apps_codes(0.5, 0.66)[1])], [(APPS2, half[1])], half),
        ("APPS1 low", "apps", [(APPS1, model.code_for(0.30))], [(APPS1, REST[0])], REST),
        ("APPS1 high", "apps", [(APPS1, model.code_for(1.20))], [(APPS1, full[0])], full),
        ("APPS2 low", "apps", [(APPS2, model.code_for(0.30))], [(APPS2, REST[1])], REST),
        ("APPS2 high", "apps", [(APPS2, model.code_for(1.25))], [(APPS2, full[1])], full),
        ("BPPS low", "bpps", [(BPPS, model.code_for(0.40))], [(BPPS, bpps_code(0.0))], REST),
        ("BPPS high", "bpps", [(BPPS, model.code_for(1.10))], [(BPPS, bpps_code(0.0))], REST),
        ("front BSE low", "bse", [(FRONT_BSE, model.code_for(0.25))], [(FRONT_BSE, bse_code(20.0))], REST),
        ("rear BSE high", "bse", [(REAR_BSE, model.code_for(1.60))], [(REAR_BSE, bse_code(20.0))], REST),
    ]
    t = 700
    for what, flag, fault, restore, pedal in episodes:
        s.apps_raw(t, *pedal)
        for ch, code in fault:
            s.set_adc(t + 143, ch, code)
        for ch, code in restore:
            s.set_adc(t + 143 + 250, ch, code)
        t += 550
    s.apps(t, 0.0)
    # A 60 ms glitch doesn't trip; neither do two 60 ms ones 30 ms apart (the
    # timer restarts when the fault clears)
    s.set_adc(t + 107, APPS1, model.code_for(0.30))
    s.set_adc(t + 167, APPS1, REST[0])
    s.set_adc(t + 307, APPS1, model.code_for(0.30))
    s.set_adc(t + 367, APPS1, REST[0])
    s.set_adc(t + 397, APPS1, model.code_for(0.30))
    s.set_adc(t + 457, APPS1, REST[0])

    def implaus_check(c, s, r, ex):
        for name in ("dev", "apps", "bpps", "bse"):
            want = sum(1 for e in episodes if e[1] == name)
            got = ram_rises(r, name)
            c.check(len(got) == want,
                    f"{name} tripped {len(got)} times ({[round(x / MS, 1) for x in got]} ms), expected {want}")
        expect_counts(c, {"PowerReady bits": 100})
    s.checks.append(implaus_check)
    result.append(s)

    # The last code that passes and the first that fails, for each range
    # check and the deviation check
    s = Scenario("range_limits", 8.4, "first failing ADC code of every range and deviation check")
    limits = range_limits()
    t = 200
    for name, ch, good, bad, start in limits:
        pedal = apps_codes(1.0) if "high" in name and ch in (APPS1, APPS2) else REST
        s.apps_raw(t, *pedal)
        if ch not in (APPS1, APPS2):
            s.set_adc(t, ch, start)
        s.set_adc(t + 120, ch, good)
        s.set_adc(t + 320, ch, bad)
        s.set_adc(t + 520, ch, start if ch in (APPS1, APPS2) else DEFAULT_CODES[ch])
        t += 700
    c1, good, bad = deviation_limit(0.5)
    s.apps_raw(t, c1, apps_codes(0.5)[1])
    s.set_adc(t + 20, APPS2, good)
    s.set_adc(t + 220, APPS2, bad)
    s.apps(t + 420, 0.0)

    def limits_check(c, s, r, ex):
        trips = sum(len(ram_rises(r, name)) for name in ("apps", "bpps", "bse", "dev"))
        c.check(trips == len(limits) + 1, f"{trips} trips, expected {len(limits) + 1} (one per first failing code)")
    s.checks.append(limits_check)
    result.append(s)

    # Brake + accel: front pressure over 30 psi with the pedal over 25%
    # latches at once and holds until the pedal is under 5%, brake or not.
    # Also the brake light, which follows the same pressure.
    s = Scenario("brake_and_accel", 4.0, "brake + accel latch, its thresholds, the brake light")
    psi_ok, psi_over = code_limit(bse_code(20.0), 1, lambda c: volts_band(c).map(model.pressure),
                                  lambda i: above(i, 30.0))
    accel_ok, accel_over = pedal_limit(lambda i: above(i, 0.25), 0.24)
    clears, held = pedal_limit(lambda i: t_not(below(i, 0.05)), 0.05)
    ready_to_drive(s, 100)
    s.apps(700, 0.5)
    s.pressure(900, 50.0)            # latches, brake light on
    s.pressure(1100, 20.0)           # still latched
    s.apps(1300, 0.1)                # still latched (pedal 7%)
    s.apps_raw(1500, *held)          # just over 5%: still latched
    s.apps_raw(1700, *clears)        # just under 5%: clears
    s.apps(1900, 0.5)
    s.set_adc(2100, FRONT_BSE, psi_ok)    # 30 psi or less: nothing
    s.set_adc(2300, FRONT_BSE, psi_over)  # over 30: latches, light on
    s.pressure(2500, 20.0)
    s.apps(2550, 0.0)                # clears
    s.pressure(2700, 50.0)
    s.apps_raw(2750, *accel_ok)      # 25% or less: nothing
    s.apps_raw(2950, *accel_over)    # over 25%: latches
    s.apps(3150, 0.0)
    s.pressure(3150, 20.0)
    s.apps(3350, 0.5)                # driving again

    def brake_check(c, s, r, ex):
        got = ram_rises(r, "ba")
        c.check(len(got) == 3, f"brake + accel latched {len(got)} times ({[round(x / MS) for x in got]} ms), expected 3")
        rises = [t for t, v in r.pins["PC4"] if v]
        c.check(len(rises) == 3, f"brake light came on {len(rises)} times, expected 3")
    s.checks.append(brake_check)
    result.append(s)

    # Ready to drive: each condition missing in turn, the brake threshold,
    # then on and off by the button, 913 (precharge, shutdown) and the tray
    # temperature, with the light and the 2 s buzzer
    s = Scenario("rtd_sequence", 9.6, "RTD conditions, every way it turns off, light and buzzer timing")
    brake_ok, brake_over = code_limit(bpps_code(0.0), 1, lambda c: volts_band(c).map(model.bpps_position),
                                      lambda i: above(i, model.BPPS_BRAKE_ENGAGE))
    s.button(100)                           # nothing from the battery yet
    s.frame(250, P, BATTERY, [0x40])        # precharged, shutdown open
    s.brake(300, 0.3)
    s.button(450)
    s.frame(600, P, BATTERY, [0x04])        # shutdown closed, not precharged
    s.button(750)
    s.frame(900, P, BATTERY, [0x44])        # both
    s.brake(950, 0.0)
    s.button(1100)                          # brake not pressed
    s.set_adc(1200, BPPS, brake_ok)
    s.button(1350)                          # brake just under the 9%
    s.set_adc(1450, BPPS, brake_over)
    s.button(1600)                          # on; buzzer until 3.6 s
    s.brake(1750, 0.0)
    s.button(2250)                          # off; the buzzer keeps going
    s.brake(2400, 0.3)
    s.button(2550)                          # on again: buzzer until 4.55 s
    s.brake(2700, 0.0)
    s.frame(2900, D, BATTERY, [0x00])       # 913 on CAN_D: not for the VCU
    s.frame(4700, P, BATTERY, [0x04])       # precharge lost: off (buzzer went off at 4.55 s, RTD on)
    s.frame(4900, P, BATTERY, [0x44])
    s.brake(4950, 0.3)
    s.button(5100)                          # on: buzzer until 7.1 s
    s.brake(5250, 0.0)
    s.frame(5600, P, BATTERY, [0x40])       # shutdown opened: off
    s.frame(5800, P, BATTERY, [0x44])
    s.brake(5850, 0.3)
    s.button(6000)                          # on: buzzer until 8.0 s
    s.brake(6150, 0.0)
    s.frame(6400, P, TRAY, [0x00, 80, 0, 0, 0, 0, 0, 0])   # 40.0 C: nothing
    s.frame(6600, P, TRAY, [0x00, 81, 0, 0, 0, 0, 0, 0])   # 40.5 C: off
    s.brake(6800, 0.3)
    s.button(6950)                          # on: buzzer until 8.95 s
    s.brake(7100, 0.0)
    s.frame(7500, P, TRAY, [0x00, 0xFF, 0, 0, 0, 0, 0, 0])  # 127.5 C: off; buzzer off at 8.95 s

    def rtd_check(c, s, r, ex):
        b = ex.running[0]
        ons = [t for t, what in b.rtd_events if what == "on"]
        offs = [t for t, what in b.rtd_events if what == "off"]
        c.check(len(ons) == 5 and len(offs) == 5, f"model has RTD on at {ons}, off at {offs}, expected 5 each")
        light = [t for t, v in r.pins["PC0"] if v]
        buzzer = r.pins["PA7"]
        c.check(len(light) == 5, f"light came on {len(light)} times, expected 5")
        # On at 1.6 s, re-armed at 2.55 s, off 2 s later with RTD still on;
        # on at 5.1 s, re-armed twice, off 2 s after the last with RTD off
        c.check([v for _, v in buzzer] == [1, 0, 1, 0], f"buzzer changes {buzzer}, expected on and off twice")
    s.checks.append(rtd_check)
    result.append(s)

    # What the VCU does with received frames: forwarding (formats, DLCs),
    # frames on the wrong bus, unknown ids, the modes frame, and bursts on
    # both buses at back-to-back spacing
    s = Scenario("can_rx", 2.6, "RX dispatch, forwarding, bursts on both buses, BSPD inputs")
    s.frame(300, P, SPEED, bytes.fromhex("1027a00f00000000"))
    s.frame(320, P, SME_TEMP, bytes.fromhex("0102030405060708"))
    s.frame(340, P, SPEED, bytes.fromhex("e803"))                 # DLC 2
    s.frame(360, P, SPEED, bytes.fromhex("11223344"), ext=True)   # extended id 1154: forwarded too
    s.frame(380, P, SME_TEMP, b"", rtr=True)                      # remote frame
    s.frame(400, D, SPEED, bytes.fromhex("1027a00f00000000"))     # 1154 on CAN_D: not forwarded
    s.frame(420, D, SME_TEMP, bytes.fromhex("0102030405060708"))
    s.frame(440, P, 0x100, bytes.fromhex("ff"))                   # unknown ids
    s.frame(460, D, 0x7FF, bytes.fromhex("ffffffffffffffff"))
    s.frame(480, P, 0x1FFFFFFF, bytes.fromhex("00"), ext=True)
    s.frame(500, P, MODES, [0x3F])                                # modes on CAN_P: ignored
    s.frame(520, P, 421, bytes.fromhex("1027"))                   # wheel speeds on CAN_P: ignored
    s.frame(530, P, 422, bytes.fromhex("1027"))
    s.frame(540, P, 423, bytes.fromhex("1027"))
    s.frame(550, P, 424, bytes.fromhex("1027"))
    s.frame(600, D, MODES, [0x2D])                                # drive 1, traction 3, regen 2
    s.pin(700, "A", 2, 1)                                         # BSPD fault and shutdown inputs (403 byte 1)
    s.pin(900, "A", 3, 1)
    s.pin(1200, "A", 2, 0)
    s.pin(1400, "A", 3, 0)
    for n in range(40):                                           # 500k: ~230 us per 8-byte frame
        s.frame(1000 + 0.25 * n, P, (SPEED, SME_TEMP)[n % 2], bytes([n, n + 1, 2, 3, 4, 5, 6, n]))
    for n in range(40):                                           # 1M: ~115 us per frame
        s.frame(1500 + 0.13 * n, D, (0x123, 0x55, MODES)[n % 3], bytes([n] * 8))
    for n in range(30):                                           # both at once
        s.frame(2000 + 0.25 * n, P, (SPEED, SME_TEMP)[n % 2], bytes([0xA0 + n] * 8))
        s.frame(2000 + 0.13 * n, D, 0x321, bytes([n] * 3))

    def rx_check(c, s, r, ex):
        expect_counts(c, {"forwarded": 4 + 40 + 30})
        modes = r.ram_end.get("drive_mode"), r.ram_end.get("traction_mode"), r.ram_end.get("regen_mode")
        got = tuple(None if v is None else (v >> 8 * (TOOLS.other[name] & 3)) & 0xFF
                    for v, name in zip(modes, ("drive_mode", "traction_mode", "regen_mode")))
        last = [d for t, bus, i, e, rtr, d in s.rx if bus == D and i == MODES][-1][0]
        want = (last & 3, (last >> 2) & 3, (last >> 4) & 3)
        c.check(got == want, f"drive/traction/regen mode {got}, expected {want} from the last 432 on CAN_D")
        c.check(c.forward_latency <= REACT, f"forwarding took up to {c.forward_latency} us")
        bspd = {(f.data[1] >> 6) & 3 for f in r.frames if f.id == STATUS}
        c.check(bspd == {0, 1, 2, 3}, f"403 BSPD bits seen {sorted(bspd)}, expected every combination")
    s.checks.append(rx_check)
    result.append(s)

    # Wheel speeds to traction control (660): reset below 100 rpm at the
    # rear, slip, its clamps, the loop time and derivatives, partial sets
    s = Scenario("wheel_speeds", 2.4, "wheel speed frames to traction control and 660")
    t = 150.0
    for fl, fr, bl, br, until in ((40, 40, 50, 50, 400),           # rear under 100 rpm: reset
                                  (1000, 1000, 1100, 1100, 750),   # slip 0.0909
                                  (800, 800, 1000, 1000, 1000),    # slip 0.2, derivative
                                  (1200, 1200, 1000, 1000, 1200),  # front faster: slip clamps to 0
                                  (90, 90, 100, 100, 1350),        # rear exactly 100.0: reset
                                  (90, 90, 100.1, 100.1, 1550),    # just over: slip again, no derivative yet
                                  (0, 0, 2000, 2000, 1750)):       # front stopped: slip 1
        while t < until:
            s.wheels(t, fl, fr, bl, br)
            t += 10.5
    for n in range(10):                       # only three corners: no update
        s.frame(1760 + 10.5 * n, D, 421, [0x10, 0x27])
        s.frame(1760.2 + 10.5 * n, D, 422, [0x10, 0x27])
        s.frame(1760.4 + 10.5 * n, D, 423, [0x10, 0x27])
    t = 1900.0
    while t < 2350:                           # 30 ms apart: loop time 30 ms
        s.wheels(t, 500, 500, 520, 520)
        t += 30.0

    def wheel_check(c, s, r, ex):
        expect_counts(c, {"660 after an update": 20})
        slips = {f.data[0] for f in r.frames if f.id == TRACTION}
        c.check({0, 9, 20, 10, 100, 3} <= slips, f"660 slips seen {sorted(slips)}")
        loops = {f.data[3] for f in r.frames if f.id == TRACTION}
        c.check({10, 30} <= loops, f"660 loop times seen {sorted(loops)}")
    s.checks.append(wheel_check)
    result.append(s)

    # VN-200 end to end: values and scaling in all five frames, corrupt
    # messages, noise, an asynchronous error, a short gap, and a sensor reset
    # that stops the messages (reconfigured 500 ms later)
    s = Scenario("imu", 4.2, "VN-200 setup, frames, CRC errors, gaps, reset and reconfiguration")
    s.imu_command(1000, "corrupt 3")
    s.imu_command(1252, "inject 00112233445566778899aabbccddeeff")
    s.imu_command(1400, "error 0B")
    s.imu_command(1603, "mute")                # 250 ms without messages: no timeout
    s.imu_command(1853, "unmute")
    s.imu_command(2303, "reset")               # factory settings: binary output off

    def imu_check(c, s, r, ex):
        vb = ex.vn[id(ex.running[0])]
        c.check(len(vb.configs) == 2, f"{len(vb.configs)} configurations, expected 2 (boot, after the reset)")
        c.check(len(vb.bad) == 3, f"{len(vb.bad)} corrupt messages reached the driver, expected 3")
        if len(vb.configs) == 2:
            last = max(d for d, k in vb.packets if d < vb.configs[1])
            gap = vb.configs[1] - last
            c.check(VN_DATA_TIMEOUT <= gap <= VN_DATA_TIMEOUT + 2 * PASS_MAX,
                    f"reconfigured {gap / MS:.3f} ms after the last message, expected 500 ms")
        expect_counts(c, {"IMU frames with data": 1500})
    s.checks.append(imu_check)
    result.append(s)

    # No VN-200: retries every 100 ms, 1 s back off, nothing disturbed; then
    # it's plugged in and the next attempt configures it
    s = Scenario("imu_missing", 3.6, "VN-200 not answering, then plugged in", imu="off")
    s.imu_command(2000, "on")
    ready_to_drive(s, 200)
    s.apps(800, 0.4)

    def missing_check(c, s, r, ex):
        vb = ex.vn[id(ex.running[0])]
        sent = [t for t, line in vn_log(r)[0] if line == VN_COMMANDS[0]]
        c.check(len(sent) == 7, f"{len(sent)} VNASY,0 sent, expected 3 + 3 + 1: {[round(x / MS) for x in sent]}")
        c.check(len(vb.configs) == 3, f"{len(vb.configs)} configurations started, expected 3")
        c.check(vb.packets, "never configured after the sensor came on")
        expect_counts(c, {"torques": 30, "IMU frames": 1500})
    s.checks.append(missing_check)
    result.append(s)

    # TX queue: mailboxes busy for 25 ms on CAN_D and 60 ms on CAN_P (all
    # frames wait in the queue and go out in order), then 200 ms on CAN_D
    # (the 32-frame queue overflows; the oldest 32 go out, the rest count as
    # dropped)
    s = Scenario("can_tx_queue", 2.1, "TX queue: mailboxes held, frames kept in order, overflow counted; "
                                      "CAN errors on the debug line")
    s.hold_mailboxes(D, 303.3, 328.3)
    s.hold_mailboxes(P, 501.7, 561.7)
    s.hold_mailboxes(D, 803.3, 1003.3, overflow=True)
    # Error counters on the debug line: TEC set through ESR, bus-off forced
    s.events.append((1200 * MS, f"write {CAN_BASE[P] + 0x18:X} 00800000"))  # TEC 128
    s.events.append((1200 * MS, f"write {CAN_BASE[D] + 0x18:X} 00050000"))  # TEC 5
    s.bus_off = (1700 * MS, 2100 * MS)
    s.expected_can_errors = lambda t: ((128 if t > 1200 * MS else 0, 1700 * MS < t < 2100 * MS),
                                       (5 if t > 1200 * MS else 0, 1700 * MS < t < 2100 * MS))

    def can_d_attempts(b, a, z):
        """CAN_D sends main() makes in (a, z), in order"""
        out = []
        for order, (name, period, ids) in enumerate(JOBS):
            k = 1
            while b.t0 + k * period < z:
                due = b.t0 + k * period
                for n, (bus, can_id) in enumerate(ids):
                    if bus == D and due > a:
                        out.append((due, order, n, can_id))
                # The copies go 5 ms after the 40 ms job's pass, a little
                # after its deadline
                copy = due + COPY_DELAY + 50
                if name == "pt40" and a < copy < z:
                    out += [(copy, 9, 0, THROTTLE), (copy, 9, 1, CURRENTS)]
                k += 1
        return [x[3] for x in sorted(out)]

    def drops(t, b, s=s):
        """The debug line prints after the jobs due in its pass. When the hold
        ends the queue is still full and no TX interrupt fires here; the next
        can_send() moves the queue along before it checks for space, so
        nothing more is dropped after the hold."""
        _, a, z = s.overflows[0]
        attempted = len(can_d_attempts(b, a, min(t + 1, z)))
        return (0, max(0, attempted - 32))
    s.expected_drops = drops

    def queue_check(c, s, r, ex):
        b = ex.running[0]
        _, a, z = s.overflows[0]
        attempted = can_d_attempts(b, a, z)
        c.check(len(attempted) > 40, f"only {len(attempted)} frames due during the long hold")
        during = [f for f in r.frames if f.bus == D and a < f.t < z]
        c.check(not during, f"CAN_D sent during the hold: {during[:2]}")
        after = [f for f in r.frames if f.bus == D and z < f.t]
        drained = [f.id for f in after[:32]]
        c.check(drained == attempted[:32], f"after the hold CAN_D sent {drained}, expected the first 32 queued: "
                                           f"{attempted[:32]}")
        # Then both copy frames of that 40 ms job, right behind the drained
        # ones (the can_send() that ends the wait pumps the queue first)
        c.check(len(after) > 34 and after[32].id == THROTTLE and after[33].id == CURRENTS
                and after[33].t - after[31].t < 50, f"after the drained frames: {after[32:34]}")
        alive = [f.data[5] for f in r.frames if f.bus == P and f.id == THROTTLE]
        c.check(all((b2 - a2) % 16 == 1 for a2, b2 in zip(alive, alive[1:])), f"MBB_Alive skipped: {alive}")
    s.checks.append(queue_check)
    result.append(s)

    # TIM5 wraps 1 s after boot, in the middle of a buzzer, an implausibility,
    # pedal steps, wheel speeds and VN-200 messages. The timebase counts the
    # wrap (check_tim5_wraps), so the 64-bit time just carries on.
    s = Scenario("timer_wrap", 2.6, "TIM5 wrapping under everything, counted by the timebase")
    s.tim5_start = 2 ** 32 - 1000 * MS
    ready_to_drive(s, 150)                    # buzzer until ~2.3 s
    s.apps(500, 0.3)
    s.set_adc(953, APPS2, model.code_for(0.30))   # fault from 0.953 s, trips after the wrap
    s.set_adc(1203, APPS2, apps_codes(0.3)[1])
    s.apps(1300, 0.6)
    t = 600.0
    s.wheels(t, 40, 40, 40, 40)
    while t < 1600:
        t += 10.5
        s.wheels(t, 1000, 1000, 1100, 1100)
    s.events.append((900 * MS, f"read tim5 {TIM5_CNT:X}"))
    s.events.append((1100 * MS, f"read tim5 {TIM5_CNT:X}"))

    def wrap_check(c, s, r, ex):
        reads = [int(text.split()[-1], 16) for t, text in r.events if text.startswith("read tim5")]
        c.check(len(reads) == 2 and reads[0] >= 0xFFF00000 and reads[1] < 0x00200000, f"TIM5 before/after: {reads}")
        c.check(ram_rises(r, "apps"), "the implausibility across the wrap never tripped")
        expect_counts(c, {"660 after an update": 8, "torques": 30})
    s.checks.append(wrap_check)
    result.append(s)

    # 20 s of everything at once at the real rates
    s = Scenario("soak", 20.0, "20 s: pedal sweeps, brakes, RTD cycles, every frame type, VN-200")
    ready_to_drive(s, 150)
    for ms in range(500, 20000, 100):
        phase = (ms % 4000) / 2000
        travel = phase if phase <= 1 else 2 - phase
        s.apps(ms, min(travel, 1.0))
    for cycle in range(0, 20000, 4000):
        s.pressure(cycle + 1003, 60.0)        # brake + accel at ~50% pedal: latches until the pedal is back
        s.pressure(cycle + 1103, 20.0)
        if cycle + 4000 < 20000:
            s.pressure(cycle + 3953, 60.0)    # braking at rest: brake light only
            s.pressure(cycle + 4203, 20.0)
        s.button(cycle + 2903)                # RTD off
        s.brake(cycle + 3203, 0.3)
        s.button(cycle + 3353)                # and on again
        s.brake(cycle + 3503, 0.0)
    for ms in range(200, 20000, 100):
        s.frame(ms + 0.7, P, BATTERY, [0x44])
        s.frame(ms + 5.3, P, SME_TEMP, bytes([ms // 100 % 256, 1, 2, 3, 4, 5, 6, 7]))
    for ms in range(330, 20000, 300):
        s.set_adc(ms, STEERING, 280 + (ms * 7) % 1100)
    for ms in range(250, 20000, 500):
        s.frame(ms + 2.1, P, TRAY, [0, 50, 51, 52, 53, 54, 0, 0])
        s.frame(ms + 7.7, D, MODES, [(ms // 500) % 64])
    for ms in range(200, 20000, 10):
        rpm = (ms // 10) % 4000
        s.frame(ms + 3.3, P, SPEED, bytes([rpm & 0xFF, rpm >> 8, 0, 0, 0, 0, 0, 0]))
    t = 205.0
    while t < 19990:
        rear = 1000 + 400 * ((t // 1000) % 3)
        s.wheels(t, rear * 0.9, rear * 0.92, rear, rear)
        t += 10.5

    def soak_check(c, s, r, ex):
        expect_counts(c, {"torques": 90, "torque ranges": 500, "forwarded": 2100, "660 after an update": 200,
                          "IMU frames with data": 9500, "debug lines": 39, "PowerReady bits": 700})
        c.check(len(ram_rises(r, "ba")) == 5, f"brake + accel latched {len(ram_rises(r, 'ba'))} times, expected 5")
        light = len([1 for t, v in r.pins["PC0"] if v])
        c.check(light == 6, f"RTD light came on {light} times, expected 6")
    s.checks.append(soak_check)
    result.append(s)

    # Watchdog: the reload stops for a while; the chip resets about 250 ms
    # later and boots again with everything back to its reset state. Then a
    # reset from the reset pin, and the boot after it has to say so (the
    # watchdog's flag was cleared).
    s = Scenario("watchdog", 1.8, "watchdog reset when refreshes stop, the boot after it, then a pin reset",
                 boots=("power on", "watchdog", "reset pin"))
    s.allowed += RESET_WARNINGS
    ready_to_drive(s, 150)
    s.block_watchdog(600, 860)
    s.events.append((1300 * MS, "reset"))

    def watchdog_check(c, s, r, ex):
        # 124 ticks of 2 ms; the prescaler isn't reset by a reload, so 246-248 ms
        c.check(len(r.resets) == 2 and 600 * MS + 246 * MS <= r.resets[0][0] <= 600 * MS + 249 * MS,
                f"resets at {[x[0] / MS for x in r.resets]} ms, expected 246-248 ms after the refreshes "
                "stopped at 600 ms, then at 1300 ms")
        c.check(r.resets and r.resets[0][1] & (1 << 29), "IWDGRSTF not set after the reset")
        light = r.pins["PC0"]
        c.check([v for _, v in light] == [1, 0] and light[1][0] == r.resets[0][0],
                f"RTD light {light}, expected on, then off at the reset")
    s.checks.append(watchdog_check)
    result.append(s)

    # Error_Handler: HAL_CAN_Init fails on the first boot; the VCU says where
    # from, sends nothing, and the watchdog restarts it
    s = Scenario("error_handler", 0.9, "Error_Handler on a failed init, then the watchdog reset",
                 boots=("power on", "watchdog"))
    s.allowed += RESET_WARNINGS
    s.fail_can_init = (0, 50 * MS)
    s.stuck_boots = {0}

    def error_check(c, s, r, ex):
        b = ex.boots[0]
        lines = [line for t, line in r.uart if b.start <= t < b.end]
        c.check(len(lines) == 2 and lines[0] == "" and lines[1].startswith("Error_Handler from 0x"),
                f"first boot printed {lines}, expected just the Error_Handler line")
        if lines and lines[-1].startswith("Error_Handler from 0x"):
            address = (int(lines[-1].split()[-1], 16) & ~1) - 1  # the call, not the return address
            where = subprocess.run([TOOLS.addr2line, "-f", "-C", "-e", TOOLS.elf, hex(address)],
                                   capture_output=True, text=True).stdout.split()
            c.check(where and where[0] == "bus_init", f"Error_Handler called from {where}, expected bus_init")
        started = [t for t, label, v in r.audit if label.startswith("HAL_IWDG_Init") and t < b.end]
        if c.check(started and r.resets, "no watchdog start or reset"):
            c.check(started[0] + 246 * MS <= r.resets[0][0] <= started[0] + 249 * MS,
                    f"reset {(r.resets[0][0] - started[0]) / MS:.1f} ms after the watchdog started, expected 246-248")
    s.checks.append(error_check)
    result.append(s)
    return result


GENERAL = [check_run, check_boots, check_tim5_wraps, check_schedule, check_frames, check_forwarding,
           check_traction, check_vn200, check_imu_frames, check_debug_lines, check_pins, check_ram]


# --- main ------------------------------------------------------------------------

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


def symbols(elf):
    """Function and variable addresses from the ELF"""
    nm, gdb = find_tool(elf, "nm"), find_tool(elf, "gdb")
    if not nm or not gdb:
        sys.exit("arm-none-eabi-nm and arm-none-eabi-gdb are needed")
    out = subprocess.run([nm, "-C", elf], capture_output=True, text=True).stdout
    functions = {}
    for line in out.splitlines():
        parts = line.split(" ", 2)
        if len(parts) == 3 and parts[1] in "Tt" and parts[2] in FUNCTIONS:
            functions[parts[2]] = int(parts[0], 16)
    missing = [f for f in FUNCTIONS if f not in functions]
    if missing:
        sys.exit(f"not in the ELF: {missing}")
    expressions = list(RAM_FIELDS.values()) + list(OTHER_SYMBOLS.values())
    args = [gdb, "-batch"] + [a for e in expressions for a in ("-ex", f"print/x &{e}")] + [elf]
    out = subprocess.run(args, capture_output=True, text=True).stdout
    found = re.findall(r"= (?:\([^)]*\) )?(0x[0-9a-f]+)", out)
    if len(found) != len(expressions):
        sys.exit(f"couldn't find {expressions} in the ELF:\n{out}")
    addresses = dict(zip(expressions, (int(a, 16) for a in found)))
    ram = {label: addresses[e] for label, e in RAM_FIELDS.items()}
    other = {label: addresses[e] for label, e in OTHER_SYMBOLS.items()}
    return functions, ram, other


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--elf", default=os.path.join(ROOT, "build", "debug", "vcu", "vcu.elf"))
    parser.add_argument("--renode", default=shutil.which("renode") or "/Applications/Renode.app/Contents/MacOS/renode")
    parser.add_argument("--jobs", type=int, default=max(1, min(8, (os.cpu_count() or 4) // 2)))
    parser.add_argument("--keep", action="store_true", help="keep each run's files")
    parser.add_argument("--list", action="store_true", help="list the scenarios")
    parser.add_argument("--verbose", action="store_true", help="show what each scenario compared")
    parser.add_argument("--recheck", metavar="DIR", help="check the runs kept in DIR again, without Renode")
    parser.add_argument("only", nargs="*", help="scenario names to run")
    args = parser.parse_args()

    all_scenarios = scenarios()
    if args.list:
        for s in all_scenarios:
            print(f"{s.name:24} {s.end / 1e6:5.1f} s  {s.about}")
        return 0
    TOOLS.elf = os.path.abspath(args.elf)
    TOOLS.renode = args.renode
    if not os.path.exists(TOOLS.elf):
        sys.exit(f"{TOOLS.elf} not found, build it first")
    if not os.path.exists(TOOLS.renode):
        sys.exit(f"Renode not found at {TOOLS.renode}, pass --renode")
    TOOLS.addr2line = find_tool(TOOLS.elf, "addr2line")
    TOOLS.functions, TOOLS.ram, TOOLS.other_addresses = symbols(TOOLS.elf)
    TOOLS.other = {k: v for k, v in TOOLS.other_addresses.items() if k != "rx_ring"}

    todo = [s for s in all_scenarios if not args.only or s.name in args.only]
    if args.only and len(todo) != len(set(args.only)):
        sys.exit(f"unknown scenario in {args.only}, see --list")
    work = args.recheck or tempfile.mkdtemp(prefix="vcu-renode-")
    print(f"{TOOLS.elf}\n{len(todo)} scenarios, {sum(s.end for s in todo) / 1e6:.1f} s of virtual time, "
          f"files in {work}\n")

    def run(s):
        out_dir = os.path.join(work, s.name)
        started = time.time()
        c = Checker()
        try:
            if args.recheck:
                r = parse_run(out_dir)
            else:
                os.makedirs(out_dir)
                r = run_renode(s, out_dir)
            ex = Expect(s, r)
            for check in GENERAL + s.checks:
                check(c, s, r, ex)
        except Exception as e:  # report it and carry on with the other scenarios
            import traceback
            c.failures.append(f"{type(e).__name__}: {e}\n{traceback.format_exc()}")
        return s, c, time.time() - started

    started = time.time()
    failed = 0
    with concurrent.futures.ThreadPoolExecutor(args.jobs) as pool:
        # Longest first so the soak doesn't start last; reported in list order
        futures = {s.name: pool.submit(run, s) for s in sorted(todo, key=lambda s: -s.end)}
        for name in [s.name for s in todo]:
            s, c, took = futures[name].result()
            status = "FAIL" if c.failures else "ok"
            print(f"{status:4} {s.name:24} {c.passed:6} checks  {took:6.1f} s  {s.about}")
            for failure in c.failures[:12]:
                print(f"       {failure}")
            if len(c.failures) > 12:
                print(f"       ... {len(c.failures) - 12} more")
            if args.verbose:
                print("       compared: " + ", ".join(f"{k} {v}" for k, v in sorted(c.counts.items())))
            failed += bool(c.failures)

    if not args.keep and not failed and not args.recheck:
        shutil.rmtree(work)
    print(f"\n{len(todo) - failed}/{len(todo)} scenarios passed in {time.time() - started:.0f} s")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
