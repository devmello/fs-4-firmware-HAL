#include "vn200.h"

#include <bit>
#include <cstring>

// Payload values are copied out with memcpy, so the CPU has to be
// little-endian like the payload (ICD A.1.2)
static_assert(std::endian::native == std::endian::little);
static_assert(sizeof(float) == 4 && sizeof(double) == 8);

namespace {

// Sent in this order, each as $<payload>*<checksum>\r\n (ICD 1.1.2). Each
// reply is the echo of the command (ICD 1.3) or $VNERR (ICD 1.5). Registers 76
// and 77 are turned off before 75 is written, in case the sensor counts what
// they have enabled when it checks 75 against the baud rate (ICD 3.2.7).
// Nothing is saved to flash: the sensor gets configured again after a reset.
constexpr const char *COMMANDS[] = {
    "VNASY,0",         // pause async output on this port (ICD 1.3.10)
    "VNWRG,06,0",      // ASCII async output off on this port (ICD 3.2.3)
    "VNWRG,76,0,0,00", // binary output 2 off, its default contents (ICD 3.2.8)
    "VNWRG,77,0,0,00", // binary output 3 off (ICD 3.2.9)
    // Binary output 1 (ICD 3.2.7, A.1.1): AsyncMode 2 = serial port 2 only,
    // RateDivisor 8 = 800 Hz / 8 = 100 Hz, then the group byte and the type
    // words of HEADER below, in hex
    "VNWRG,75,2,8,34,0600,0002,000A",
    "VNASY,1", // resume
};
constexpr size_t COMMAND_COUNT = sizeof(COMMANDS) / sizeof(COMMANDS[0]);

constexpr size_t longest_command() {
    size_t longest = 0;
    for (const char *command : COMMANDS) {
        size_t len = 0;
        while (command[len] != '\0') {
            len++;
        }
        longest = len > longest ? len : longest;
    }
    return longest;
}

// '$' + payload + '*' + 2 hex digits + "\r\n"
constexpr size_t TX_MAX = 1 + longest_command() + 5;
constexpr char HEX[] = "0123456789ABCDEF";

// Binary output message (ICD 2.1.3): sync byte, group byte, one type word per
// group (little-endian), payload, CRC (big-endian)
constexpr uint8_t SYNC = 0xFA;
constexpr uint8_t HEADER[] = {
    SYNC,
    0x34,       // groups IMU (bit 2), Attitude (4), INS (5), ICD table 2.1
    0x00, 0x06, // IMU: Accel (bit 9), AngularRate (10), ICD 2.4
    0x02, 0x00, // Attitude: Ypr (bit 1), ICD 2.6
    0x0A, 0x00, // INS: PosLla (bit 1), VelBody (3), ICD 2.7
};

// Payload offsets in the packet: groups in bit order, then types in bit
// order, no padding (ICD 2.1.3). Sizes from ICD table 2.2.
constexpr size_t ACCEL = 8;     // 3 floats, m/s^2
constexpr size_t GYRO = 20;     // 3 floats, rad/s
constexpr size_t YPR = 32;      // 3 floats, deg
constexpr size_t POS_LLA = 44;  // 3 doubles: lat, lon (deg), alt (m)
constexpr size_t VEL_BODY = 68; // 3 floats, m/s
static_assert(sizeof(HEADER) == ACCEL);
static_assert(VEL_BODY + 12 + 2 == Vn200::PACKET_SIZE);

// CRC-16-CCITT, polynomial 0x1021, initial value 0, MSB first (ICD 1.4.3).
// Over a packet after the sync byte, CRC included, it comes out 0.
uint16_t crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc = static_cast<uint16_t>(crc ^ (data[i] << 8));
        for (int bit = 0; bit < 8; bit++) {
            if ((crc & 0x8000U) != 0U) {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
            } else {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
    }
    return crc;
}

// 8-bit XOR of the bytes between '$' and '*' (ICD 1.4, 1.4.2)
uint8_t xor8(const char *s, size_t len) {
    uint8_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum ^= static_cast<uint8_t>(s[i]);
    }
    return sum;
}

float get_float(const uint8_t *p) {
    float value;
    memcpy(&value, p, sizeof(value));
    return value;
}

double get_double(const uint8_t *p) {
    double value;
    memcpy(&value, p, sizeof(value));
    return value;
}

int digit_value(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    return -1;
}

// 1 to 8 digits in base 10 or 16, nothing else
bool parse_uint(const char *s, size_t len, uint32_t base, uint32_t &out) {
    if (len == 0 || len > 8) {
        return false;
    }
    uint32_t value = 0;
    for (size_t i = 0; i < len; i++) {
        int digit = digit_value(s[i]);
        if (digit < 0 || static_cast<uint32_t>(digit) >= base) {
            return false;
        }
        value = value * base + static_cast<uint32_t>(digit);
    }
    out = value;
    return true;
}

// Field number index (0 = header) of a comma separated payload
bool get_field(const char *s, size_t len, size_t index, const char *&field, size_t &field_len) {
    field = s;
    field_len = 0;
    size_t start = 0;
    for (size_t i = 0; i <= len; i++) {
        if (i == len || s[i] == ',') {
            if (index == 0) {
                field = s + start;
                field_len = i - start;
                return true;
            }
            index--;
            start = i + 1;
        }
    }
    return false;
}

// Same header and same first field (register number or output state).
// Compared as numbers, so a reply with 6 matches a command with 06.
bool is_reply_to(const char *command, const char *reply, size_t reply_len) {
    size_t command_len = strlen(command);
    const char *a;
    const char *b;
    size_t a_len;
    size_t b_len;
    get_field(command, command_len, 0, a, a_len);
    get_field(reply, reply_len, 0, b, b_len);
    if (a_len != b_len || memcmp(a, b, a_len) != 0) {
        return false;
    }
    uint32_t x;
    uint32_t y;
    return get_field(command, command_len, 1, a, a_len) && get_field(reply, reply_len, 1, b, b_len) &&
           parse_uint(a, a_len, 10, x) && parse_uint(b, b_len, 10, y) && x == y;
}

// Error codes that answer a command (ICD table 1.6). The others (hard fault,
// watchdog reset, output and error buffer overflow) can come at any time.
bool is_command_error(uint32_t code) {
    return (code >= 0x02U && code <= 0x09U) || code == 0x0CU;
}

} // namespace

void Vn200::start(uint32_t now_us) {
    restart();
    poll(now_us);
}

void Vn200::restart() {
    counts.configs_started++;
    phase = Phase::SEND;
    step = 0;
    tries = 0;
    pkt_len = 0;
    in_line = false;
}

void Vn200::feed(const uint8_t *data, size_t len, uint32_t now_us) {
    size_t i = 0;
    while (i < len) {
        if (phase == Phase::RUNNING) {
            // n > 0: scan() always takes at least one byte off a full buffer
            size_t n = len - i;
            if (n > PACKET_SIZE - pkt_len) {
                n = PACKET_SIZE - pkt_len;
            }
            memcpy(pkt + pkt_len, data + i, n);
            pkt_len += n;
            i += n;
            scan(now_us);
        } else {
            // While configuring only replies matter. The last one switches to
            // RUNNING, and the rest of this chunk goes to scan().
            if (phase != Phase::IDLE) {
                line_byte(data[i], now_us);
            }
            i++;
        }
    }
}

void Vn200::poll(uint32_t now_us) {
    switch (phase) {
    case Phase::WAIT:
        if (now_us - sent_us >= RESPONSE_TIMEOUT_US) {
            counts.command_timeouts++;
            command_failed(now_us);
        }
        break;
    case Phase::BACKOFF:
        if (now_us - backoff_start_us >= BACKOFF_US) {
            restart();
        }
        break;
    case Phase::RUNNING:
        if (now_us - last_data_us >= DATA_TIMEOUT_US) {
            counts.data_timeouts++;
            restart();
        }
        break;
    default:
        break;
    }

    if (phase == Phase::SEND) {
        send(now_us);
    }
}

bool Vn200::update_state(VectornavState &state) {
    if (!fresh) {
        return false;
    }
    state = latest;
    fresh = false;
    return true;
}

void Vn200::send(uint32_t now_us) {
    const char *payload = COMMANDS[step];
    size_t len = strlen(payload);
    uint8_t sum = xor8(payload, len);

    uint8_t buf[TX_MAX];
    size_t n = 0;
    buf[n++] = '$';
    memcpy(buf + n, payload, len);
    n += len;
    buf[n++] = '*';
    buf[n++] = static_cast<uint8_t>(HEX[sum >> 4]);
    buf[n++] = static_cast<uint8_t>(HEX[sum & 0x0F]);
    buf[n++] = '\r';
    buf[n++] = '\n';

    tries++;
    sent_us = now_us;
    phase = Phase::WAIT;
    // A refused write is retried after the timeout, like a lost reply
    if (!write_fn(buf, n)) {
        counts.write_failures++;
    }
}

void Vn200::command_failed(uint32_t now_us) {
    if (tries >= COMMAND_TRIES) {
        phase = Phase::BACKOFF;
        backoff_start_us = now_us;
    } else {
        phase = Phase::SEND;
    }
}

void Vn200::command_done(uint32_t now_us) {
    step++;
    tries = 0;
    if (step < COMMAND_COUNT) {
        phase = Phase::SEND;
        return;
    }
    phase = Phase::RUNNING;
    counts.configs_done++;
    last_data_us = now_us;
}

// Takes complete packets off the front of pkt and drops bytes that can't
// start one. A bad header or CRC means the 0xFA wasn't a sync byte, so the
// search goes on from the byte after it: a real packet may start inside.
void Vn200::scan(uint32_t now_us) {
    size_t i = 0;
    while (i < pkt_len) {
        size_t avail = pkt_len - i;
        if (pkt[i] != SYNC) {
            skip(pkt[i++], now_us);
            continue;
        }
        size_t n = avail < sizeof(HEADER) ? avail : sizeof(HEADER);
        if (memcmp(pkt + i, HEADER, n) != 0) {
            counts.bad_headers++;
            skip(pkt[i++], now_us);
            continue;
        }
        if (avail < PACKET_SIZE) {
            break; // wait for the rest
        }
        if (crc16(pkt + i + 1, PACKET_SIZE - 1) != 0) {
            counts.crc_errors++;
            skip(pkt[i++], now_us);
            continue;
        }
        decode(pkt + i, now_us);
        i += PACKET_SIZE;
    }

    if (i > 0) {
        memmove(pkt, pkt + i, pkt_len - i);
        pkt_len -= i;
    }
}

// Skipped bytes still go to the line collector, which picks out $VNERR
void Vn200::skip(uint8_t byte, uint32_t now_us) {
    counts.bytes_skipped++;
    line_byte(byte, now_us);
}

void Vn200::decode(const uint8_t *p, uint32_t now_us) {
    for (size_t k = 0; k < 3; k++) {
        latest.accel[k] = get_float(p + ACCEL + 4 * k);
        latest.ang_rate[k] = get_float(p + GYRO + 4 * k);
        latest.vel[k] = get_float(p + VEL_BODY + 4 * k);
    }
    latest.ypr.yaw = get_float(p + YPR);
    latest.ypr.pitch = get_float(p + YPR + 4);
    latest.ypr.roll = get_float(p + YPR + 8);
    latest.pos.lat = get_double(p + POS_LLA);
    latest.pos.lon = get_double(p + POS_LLA + 8);
    latest.pos.alt = get_double(p + POS_LLA + 16);

    fresh = true;
    counts.packets++;
    last_packet_time = now_us;
    last_data_us = now_us;
}

// Collects "$...\r\n" lines. A line ends at the '\n', so after the last reply
// of the configuration nothing of it is left for scan(). Anything else that
// isn't printable ASCII drops the line, which gets rid of binary data and of
// lines too long to be a reply (like the default VNINS output).
void Vn200::line_byte(uint8_t byte, uint32_t now_us) {
    if (byte == '$') {
        in_line = true;
        line_len = 0;
        return;
    }
    if (!in_line || byte == '\r') {
        return;
    }
    if (byte == '\n') {
        in_line = false;
        handle_line(now_us);
        return;
    }
    if (byte < ' ' || byte > '~' || line_len == LINE_SIZE) {
        in_line = false;
        return;
    }
    line[line_len++] = static_cast<char>(byte);
}

void Vn200::handle_line(uint32_t now_us) {
    // <payload>*<checksum>: 2 hex digits for the XOR checksum, 4 for the CRC
    // if register 30 selects it (ICD 1.4, 3.2.5)
    const char *star = static_cast<const char *>(memchr(line, '*', line_len));
    if (star == nullptr) {
        return;
    }
    size_t len = static_cast<size_t>(star - line);
    size_t digits = line_len - len - 1;
    uint32_t sum;
    if (!parse_uint(star + 1, digits, 16, sum)) {
        return;
    }
    if (!(digits == 2 && sum == xor8(line, len)) &&
        !(digits == 4 && sum == crc16(reinterpret_cast<const uint8_t *>(line), len))) {
        return;
    }

    const char *field;
    size_t field_len;
    get_field(line, len, 0, field, field_len);
    if (field_len == 5 && memcmp(field, "VNERR", 5) == 0) {
        uint32_t code;
        if (!get_field(line, len, 1, field, field_len) || !parse_uint(field, field_len, 16, code) ||
            code > 0xFFU) {
            return;
        }
        counts.last_error = static_cast<uint8_t>(code);
        if (phase == Phase::WAIT && is_command_error(code)) {
            counts.command_errors++;
            command_failed(now_us);
        } else {
            counts.sensor_errors++;
        }
        return;
    }

    if (phase == Phase::WAIT && is_reply_to(COMMANDS[step], line, len)) {
        command_done(now_us);
    }
}
