"""
float32 model of the VCU firmware's math, for run_tests.py.

Every expression follows the firmware's operations in the same order and
precision (etc_controller.cpp, filtered_analog_in.cpp, low_pass_filter.h,
traction_control.cpp, main.cpp), so outputs can be predicted bit for bit
where the inputs are known exactly.

The EWMA filters are the exception. Their result depends on the time between
loop passes, which varies, and after a step they get stuck within some ulps
of the input because of float rounding (up to ~1 / (1 - e) ulps, e being the
decay factor of the shortest pass). So a filtered value is modelled as an
interval: where it would be in exact arithmetic, widened by that band on the
side it came from. Outputs are then given as the set of values the interval
can produce, and a check passes if the firmware's value is in the set.
Scenarios pick inputs where the set is a single value when it matters.
"""

import math
import struct
from fractions import Fraction

US = 1e-6


def f32(x):
    return struct.unpack("<f", struct.pack("<f", x))[0]


def _bits(x):
    return struct.unpack("<I", struct.pack("<f", x))[0]


def _from_bits(b):
    return struct.unpack("<f", struct.pack("<I", b))[0]


def _round_f32(q):
    """Fraction q to the nearest float32 (ties to even), normal range only"""
    if q == 0:
        return 0.0
    sign, q = (-1.0, -q) if q < 0 else (1.0, q)
    e = q.numerator.bit_length() - q.denominator.bit_length()
    if Fraction(2) ** e > q:
        e -= 1
    m = q * Fraction(2) ** (23 - e)  # 2**23 <= m < 2**24
    n, rest = divmod(m.numerator, m.denominator)
    if 2 * rest > m.denominator or (2 * rest == m.denominator and n % 2):
        n += 1
    return sign * math.ldexp(n, e - 23)


def fma32(a, b, c):
    """std::fma on floats (vfma.f32): a * b + c, rounded once to float32.
    math.fma (Python 3.13+) rounds to double; rounding that to float32 again
    only goes wrong when it lands exactly halfway between two floats, and
    then the exact sum decides."""
    d = math.fma(a, b, c)
    if struct.unpack("<Q", struct.pack("<d", d))[0] & 0x1FFFFFFF == 0x10000000:
        return _round_f32(Fraction(a) * Fraction(b) + Fraction(c))
    return f32(d)


def ulp(x):
    """Spacing of floats at |x| (the larger one at a power of 2)"""
    x = abs(f32(x))
    if x == 0.0:
        return 2.0 ** -149
    m, e = math.frexp(x)  # x = m * 2**e, 0.5 <= m < 1
    return max(2.0 ** (e - 24), 2.0 ** -149)


def floats_between(lo, hi):
    """Every float32 in [lo, hi], both >= 0 or both <= 0"""
    lo, hi = f32(lo), f32(hi)
    if lo >= 0.0:
        return [_from_bits(b) for b in range(_bits(lo), _bits(hi) + 1)]
    if hi <= 0.0:
        return [-v for v in floats_between(-hi, -lo)]
    return floats_between(lo, -0.0) + floats_between(0.0, hi)


def trunc_int(x, bits, signed):
    """static_cast to an integer type on the Cortex-M4: vcvt saturates to 32
    bits (toward zero), then the low bits are kept"""
    if math.isnan(x):
        return 0
    if signed:
        v = max(-2 ** 31, min(2 ** 31 - 1, int(x)))
    else:
        v = max(0, min(2 ** 32 - 1, int(x)))
    v &= (1 << bits) - 1
    if signed and v >= 1 << (bits - 1):
        v -= 1 << bits
    return v


def int_range(lo, hi):
    return set(range(lo, hi + 1)) if lo <= hi else set(range(hi, lo + 1))


# --- ADC and filters --------------------------------------------------------

ADC_SCALE = f32(f32(1.0) / f32(4095.0))  # (1.0f / ADC_FULL_SCALE)
VREF = f32(3.3)                           # AdcInput::vref


def adc_fraction(code):
    """adc_read(): (float)raw * (1.0f / 4095.0f)"""
    return f32(f32(float(code)) * ADC_SCALE)


def adc_volts(code):
    """AdcInput::read_voltage(), unfiltered (steering)"""
    return f32(adc_fraction(code) * VREF)


def code_for(volts):
    return max(0, min(4095, int(round(volts / 3.3 * 4095))))


TAU_APPS = f32(1.0 / (2.0 * math.pi * 60))   # FilteredAnalogIn, 60 Hz
TAU_TORQUE = f32(1.0 / (2.0 * math.pi * 40))  # motor_torque LowPassFilter, 40 Hz

# Shortest time between two filter updates (one loop pass) the bands allow
# for. Passes measure 63-110 us in the emulator, more with CAN or printf.
DT_MIN_US = 30

# FilteredAnalogIn::read() takes elapsed_time(), reads the ADC, then resets the
# timer, so the time the read takes (about 4 us of a ~65 us pass here) is never
# counted and the filter runs slow. A 402 at 7.07 ms into a step showed
# 4.5-6.7 %. Wider than this makes the bands too wide to pin torques down.
FILTER_TIME_LOST = 0.07


def stuck_ulps(tau):
    """How far from its input a filter can stay stuck after a step, in ulps"""
    e = math.exp(-DT_MIN_US * US / tau)
    return int(math.ceil(1.1 / (1.0 - e))) + 2


STUCK_APPS = stuck_ulps(TAU_APPS)
STUCK_TORQUE = stuck_ulps(TAU_TORQUE)


class Interval:
    """Possible values of a filtered signal"""

    def __init__(self, lo, hi):
        self.lo, self.hi = (lo, hi) if lo <= hi else (hi, lo)

    def __repr__(self):
        return f"[{self.lo!r}, {self.hi!r}]"

    def map(self, fn):
        """For a monotonic fn"""
        a, b = fn(self.lo), fn(self.hi)
        return Interval(min(a, b), max(a, b))

    def width(self):
        return self.hi - self.lo


def filter_interval(target, start, elapsed_lo, elapsed_hi, tau=TAU_APPS):
    """EWMA state some time after its input stepped to target (float).
    start: Interval, the state at the step. elapsed_lo/hi: bounds on the time
    since the step in seconds (a loop pass or two of uncertainty).

    Exact arithmetic gives target + (start - target) * exp(-elapsed / tau).
    Each update rounds by up to about an ulp and later updates decay that
    error, so the float state stays within stuck_ulps() of the exact path.
    It doesn't overshoot the target by more than 2 ulps. A filter seeded
    with the target at its first read stays within 2 ulps of it."""
    u = max(ulp(target), ulp(start.lo), ulp(start.hi))
    if start.lo == start.hi == target:
        return Interval(target - 2 * u, target + 2 * u)
    k_slow = math.exp(-max(0.0, elapsed_lo) / tau)
    k_fast = math.exp(-max(0.0, elapsed_hi) / tau)
    ends = [target + (s - target) * k for s in (start.lo, start.hi) for k in (k_slow, k_fast)]
    w = stuck_ulps(tau) * u
    lo, hi = min(ends) - w, max(ends) + w
    if start.hi < target:
        hi = min(hi, target + 2 * u)
    if start.lo > target:
        lo = max(lo, target - 2 * u)
    return Interval(lo, hi)


# --- ETC (etc_controller.cpp) -----------------------------------------------

APPS1_MIN = f32(0.396)
APPS1_MAX = f32(1.086)
APPS2_MIN = f32(0.439)
APPS2_MAX = f32(1.133)
DEADZONE = f32(0.03)
DEADZONE_SPAN = f32(1.0 - f32(2.0 * DEADZONE))  # (1 - 2*PEDAL_DEADZONE_PERCENTAGE)
BPPS_MIN = f32(0.460)
BPPS_MAX = f32(0.972)
BSE_MIN = f32(0.340)
BSE_MAX = f32(1.386)
APPS_BUFFER = f32(0.015)
BPPS_BUFFER = f32(0.010)
BSE_BUFFER = f32(0.02)
BPPS_BRAKE_ENGAGE = f32(0.09)
MAX_DEVIATION = f32(0.10)
MAX_TORQUE = 21298  # (int16_t)(32767 * 0.65)

APPS1_LOW, APPS1_HIGH = f32(APPS1_MIN - APPS_BUFFER), f32(APPS1_MAX + APPS_BUFFER)
APPS2_LOW, APPS2_HIGH = f32(APPS2_MIN - APPS_BUFFER), f32(APPS2_MAX + APPS_BUFFER)
BPPS_LOW, BPPS_HIGH = f32(BPPS_MIN - BPPS_BUFFER), f32(BPPS_MAX + BPPS_BUFFER)
BSE_LOW, BSE_HIGH = f32(BSE_MIN - BSE_BUFFER), f32(BSE_MAX + BSE_BUFFER)

MAP_A, MAP_B, MAP_C = f32(-0.2), f32(0.9), f32(0.3)  # -0.2x^3 + 0.9x^2 + 0.3x


def clamp01(x):
    return 0.0 if x < 0.0 else 1.0 if x > 1.0 else x


def filtered_volts(fraction):
    """FilteredAnalogIn::read_voltage(): smoothed fraction * vref"""
    return f32(fraction * VREF)


def apps_position(volts, lo, hi):
    travel = f32(f32(volts - lo) / f32(hi - lo))
    return f32(f32(clamp01(travel) - DEADZONE) / DEADZONE_SPAN)


def apps1_position(v):
    return apps_position(v, APPS1_MIN, APPS1_MAX)


def apps2_position(v):
    return apps_position(v, APPS2_MIN, APPS2_MAX)


def bpps_position(v):
    return clamp01(f32(f32(f32(v - BPPS_MIN) - BPPS_BUFFER) / f32(BPPS_MAX - BPPS_MIN)))


def pressure(v):
    """psi, ((V * 1000 - 330) / (3300 - 660)) * 2000"""
    return f32(f32(f32(f32(v * 1000.0) - 330.0) / 2640.0) * 2000.0)


def average(p1, p2):
    return f32(f32(p1 + p2) / 2.0)


def accelerator_mapping(x):
    """The port's cubic (upstream interpolates a table of it)"""
    if x < 0.0:  # the port's clamp (not upstream)
        x = 0.0
    if x >= 1.0:
        return 1.0
    inner = fma32(f32(MAP_A * x), x, f32(MAP_B * x))
    return fma32(inner, x, f32(MAP_C * x))


def unfiltered_torque(avg):
    return trunc_int(f32(accelerator_mapping(avg) * float(MAX_TORQUE)), 16, True)


def in_range(v, lo, hi):
    return lo <= v <= hi


def millivolts(v):
    """static_cast<uint16_t>(V * 1000)"""
    return trunc_int(f32(v * 1000.0), 16, False)


def percent(x):
    """static_cast<uint8_t>(x * 100)"""
    return trunc_int(f32(x * 100.0), 8, False)


def torque_reading(lo, hi):
    """motor_torque.read() once settled, for unfiltered torques lo..hi: the
    40 Hz filter can stick a little below or above, and read() truncates
    toward zero"""
    span_lo = lo - STUCK_TORQUE * ulp(float(lo)) if lo != 0 else 0.0
    span_hi = hi + STUCK_TORQUE * ulp(float(hi)) if hi != 0 else 0.0
    return int_range(int(span_lo), int(span_hi))


# --- steering (main.cpp, unfiltered, read in the 50 ms job) ----------------

STEER_MIN = f32(0.227)
STEER_MAX = f32(1.069)
STEER_AVG = f32(f32(STEER_MAX + STEER_MIN) / 2.0)
STEER_RANGE = f32(STEER_MAX - STEER_AVG)
STEER_MAX_ANGLE = f32(75.82)


def steering_x100(code):
    v = adc_volts(code)
    angle = f32(f32(f32(v - STEER_AVG) / STEER_RANGE) * STEER_MAX_ANGLE)
    return trunc_int(f32(angle * 100.0), 16, True)


# --- IMU (FakeVN200.Values, main.cpp send_imu_CAN_messages) ---------------

RAD_TO_DEG = f32(57.2957795)


def imu_values(n):
    """Same doubles and casts as FakeVN200.Values() in C#"""
    accel = [f32(1.25 + 0.01 * (n % 100)), f32(-0.75 - 0.013 * (n % 50)), f32(-9.80665 + 0.0021 * (n % 30))]
    gyro = [f32(0.0175 * ((n % 40) - 20)), f32(-0.25 + 0.003 * (n % 60)), f32(0.5 - 0.0071 * (n % 25))]
    ypr = [f32(-179.5 + 3.61 * (n % 99)), f32(12.25 - 0.5 * (n % 13)), f32(-3.3 + 0.07 * (n % 90))]
    lla = [36.99999123 + 1e-6 * n, -122.06123456 - 1e-6 * n, 12.5 + 0.1 * (n % 10)]
    vel = [f32(27.75 - 0.25 * (n % 40)), f32(-0.5 + 0.02 * (n % 50)), f32(0.031 * ((n % 20) - 10))]
    return accel, gyro, ypr, lla, vel


def _le16(values):
    return b"".join(struct.pack("<h", v) for v in values)


def imu_frames(n):
    """{id: data} the IMU job sends with message n decoded (None: all zero,
    nothing received yet)"""
    if n is None:
        accel = gyro = ypr = vel = [0.0, 0.0, 0.0]
        lla = [0.0, 0.0, 0.0]
    else:
        accel, gyro, ypr, lla, vel = imu_values(n)
    x100 = lambda v: trunc_int(f32(v * 100.0), 16, True)
    return {
        0x2D0: _le16([x100(v) for v in accel]),
        0x3D0: _le16([x100(v) for v in ypr]),
        0x2D1: struct.pack("<ii", trunc_int(lla[0] * 1e7, 32, True), trunc_int(lla[1] * 1e7, 32, True)),
        0x3D1: _le16([trunc_int(f32(f32(v * RAD_TO_DEG) * 10.0), 16, True) for v in gyro]),
        0x2D2: _le16([x100(v) for v in vel]),
    }


def crc16(data):
    """CRC-16-CCITT, polynomial 0x1021, initial value 0 (bitwise)"""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc


# --- traction control (traction_control.cpp, KP = KI = KD = 0) ------------

def wheel_rpm(raw):
    """(data[0] + (data[1] << 8)) * 0.1f"""
    return f32(f32(float(raw)) * f32(0.1))


class Traction:
    """TractionController. The update runs a loop pass after the last of the
    four wheel frames arrives, so the loop time is an interval, and so are
    the derivatives computed from it."""

    def __init__(self):
        self.loop_time = (0.0, 0.0)
        self.reset()

    def reset(self):
        # reset() leaves loop_time alone, like the firmware
        self.slip = 0.0
        self.prev_slip = 0.0
        self.stale = True
        self.integral = 0.0
        self.raw = (0.0, 0.0)
        self.smoothed = (0.0, 0.0)
        self.filter_init = False
        self.output = 1.0

    def update(self, fl, fr, rl, rr, elapsed_lo_us, elapsed_hi_us):
        v_front = f32(f32(fr + fl) / 2.0)
        v_rear = f32(f32(rr + rl) / 2.0)
        if not v_rear > 100.0:
            self.reset()
            return
        slip = clamp01(f32(f32(v_rear - v_front) / v_rear))
        times = sorted({f32(f32(float(max(0, e))) / 1000000.0) for e in (elapsed_lo_us, elapsed_hi_us)})
        self.loop_time = (times[0], times[-1])
        if not self.stale and times[0] > 0.0:
            raws, smooths = [], []
            for lt in times:
                raw = f32(f32(slip - self.prev_slip) / lt)
                raws.append(raw)
                if not self.filter_init:
                    smooths.append(raw)
                    continue
                c = f32(lt / f32(f32(0.2) + lt))
                for prev in self.smoothed:
                    smooths.append(f32(f32(f32(1.0 - c) * prev) + f32(c * raw)))
            self.raw = (min(raws), max(raws))
            self.smoothed = (min(smooths), max(smooths))
            self.filter_init = True
        else:
            self.raw = (0.0, 0.0)
        self.slip = slip
        self.integral = 0.0  # clamped to [0, KI-limited max], which is 0 with KI = 0
        self.output = 1.0    # all gains are 0
        self.prev_slip = slip
        self.stale = False

    def frame_options(self):
        """Possible values of each 660 field"""
        def span(pair, bits, signed, scale):
            a = trunc_int(f32(pair[0] * scale), bits, signed)
            b = trunc_int(f32(pair[1] * scale), bits, signed)
            return int_range(a, b)
        return {
            "slip": {percent(self.slip)},
            "output": {percent(self.output)},
            "integral": {percent(self.integral)},
            "loop_ms": span(self.loop_time, 8, False, 1000.0),
            "raw": span(self.raw, 16, True, 1000.0),
            "smoothed": span(self.smoothed, 16, True, 1000.0),
        }
