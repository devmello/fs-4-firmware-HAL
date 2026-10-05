// Host tests for the VN-200 driver. Packets are built here from the layout in
// the VN-200 ICD (2.1.3, table 2.2, 2.4, 2.6, 2.7) with their own CRC code,
// and a fake sensor answers the configuration commands.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include "vn200.h"

static int failures = 0;

#define CHECK(cond)                                                      \
    do {                                                                 \
        if (!(cond)) {                                                   \
            std::printf("%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #cond); \
            failures++;                                                  \
        }                                                                \
    } while (0)

using Bytes = std::vector<uint8_t>;

constexpr uint32_t REPLY_TIMEOUT = Vn200::RESPONSE_TIMEOUT_US;
constexpr uint32_t DATA_TIMEOUT = Vn200::DATA_TIMEOUT_US;

// The commands, checksums included (worked out separately from the driver)
static const char *const EXPECTED[] = {
    "$VNASY,0*4F\r\n",
    "$VNWRG,06,0*6C\r\n",
    "$VNWRG,76,0,0,00*5B\r\n",
    "$VNWRG,77,0,0,00*5A\r\n",
    "$VNWRG,75,2,8,34,0600,0002,000A*0C\r\n",
    "$VNRRG,01*72\r\n",
    "$VNASY,1*4E\r\n",
};
constexpr size_t COMMAND_COUNT = sizeof(EXPECTED) / sizeof(EXPECTED[0]);

// Fake UART: one string per write() call
static std::vector<std::string> writes;
static bool write_ok = true;

static bool fake_write(const uint8_t *data, size_t len) {
    writes.emplace_back(reinterpret_cast<const char *>(data), len);
    return write_ok;
}

static void reset_fakes() {
    writes.clear();
    write_ok = true;
}

// CRC-16-CCITT (polynomial 0x1021, initial value 0), table driven so it
// shares nothing with the driver's version
static uint16_t crc_table[256];

static void make_crc_table() {
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t crc = i << 8;
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x8000U) != 0U ? (crc << 1) ^ 0x1021U : crc << 1;
        }
        crc_table[i] = static_cast<uint16_t>(crc);
    }
}

static uint16_t crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc = static_cast<uint16_t>((crc << 8) ^ crc_table[((crc >> 8) ^ data[i]) & 0xFFU]);
    }
    return crc;
}

static uint8_t xor_of(const std::string &s) {
    uint8_t sum = 0;
    for (char c : s) {
        sum ^= static_cast<uint8_t>(c);
    }
    return sum;
}

// An ASCII message as the sensor sends it: $payload*XX\r\n
static std::string vn_line(const std::string &payload) {
    char sum[3];
    std::snprintf(sum, sizeof(sum), "%02X", xor_of(payload));
    return "$" + payload + "*" + sum + "\r\n";
}

// Same, with the 16-bit CRC instead (register 30 set to CRC)
static std::string vn_line_crc(const std::string &payload) {
    char sum[5];
    std::snprintf(sum, sizeof(sum), "%04X",
                  crc16(reinterpret_cast<const uint8_t *>(payload.data()), payload.size()));
    return "$" + payload + "*" + sum + "\r\n";
}

constexpr const char *MODEL = "VN-200T-CR";

// The sensor's echo of a command is the command itself (ICD 1.3). A register
// read also carries the value.
static std::string echo_of(const std::string &command) {
    if (command == "$VNRRG,01*72\r\n") {
        return vn_line(std::string("VNRRG,01,") + MODEL);
    }
    return command;
}

struct Sample {
    float accel[3];
    float gyro[3];
    float ypr[3];
    double lla[3];
    float vel[3];
};

static void put_u16(Bytes &b, uint16_t v) {
    b.push_back(static_cast<uint8_t>(v));
    b.push_back(static_cast<uint8_t>(v >> 8));
}

static void put_f32(Bytes &b, float v) {
    uint32_t u;
    std::memcpy(&u, &v, sizeof(u));
    for (int i = 0; i < 4; i++) {
        b.push_back(static_cast<uint8_t>(u >> (8 * i)));
    }
}

static void put_f64(Bytes &b, double v) {
    uint64_t u;
    std::memcpy(&u, &v, sizeof(u));
    for (int i = 0; i < 8; i++) {
        b.push_back(static_cast<uint8_t>(u >> (8 * i)));
    }
}

static float f32_bits(uint32_t u) {
    float v;
    std::memcpy(&v, &u, sizeof(v));
    return v;
}

static double f64_bits(uint64_t u) {
    double v;
    std::memcpy(&v, &u, sizeof(v));
    return v;
}

// Groups in bit order, types in bit order: IMU Accel, AngularRate, Attitude
// Ypr, INS PosLla, VelBody
static Bytes payload_of(const Sample &s) {
    Bytes b;
    for (float v : s.accel) {
        put_f32(b, v);
    }
    for (float v : s.gyro) {
        put_f32(b, v);
    }
    for (float v : s.ypr) {
        put_f32(b, v);
    }
    for (double v : s.lla) {
        put_f64(b, v);
    }
    for (float v : s.vel) {
        put_f32(b, v);
    }
    return b;
}

// Sync byte, group byte, little-endian type words, payload, big-endian CRC of
// everything after the sync byte
static Bytes packet(uint8_t groups, const std::vector<uint16_t> &types, const Bytes &payload) {
    Bytes p{0xFA, groups};
    for (uint16_t t : types) {
        put_u16(p, t);
    }
    p.insert(p.end(), payload.begin(), payload.end());
    uint16_t crc = crc16(p.data() + 1, p.size() - 1);
    p.push_back(static_cast<uint8_t>(crc >> 8));
    p.push_back(static_cast<uint8_t>(crc));
    return p;
}

static Bytes good_packet(const Sample &s) {
    return packet(0x34, {0x0600, 0x0002, 0x000A}, payload_of(s));
}

static Sample sample(int n) {
    float k = static_cast<float>(n);
    return Sample{
        {0.125f + k, -9.80665f - k, 3.4e-5f},
        {-0.0175f, 1.5f + k, -2.25e-3f * k},
        {-179.5f + k, 12.25f, -0.003f - k},
        {36.99999912345678 + n * 1e-7, -122.06123456789012 - n * 1e-7, -12.5 + n},
        {27.75f - k, -0.5f, 1e-7f},
    };
}

static bool matches(const VectornavState &st, const Sample &s) {
    bool same = true;
    for (int k = 0; k < 3; k++) {
        same = same && st.accel[k] == s.accel[k] && st.ang_rate[k] == s.gyro[k] &&
               st.vel[k] == s.vel[k];
    }
    return same && st.ypr.yaw == s.ypr[0] && st.ypr.pitch == s.ypr[1] && st.ypr.roll == s.ypr[2] &&
           st.pos.lat == s.lla[0] && st.pos.lon == s.lla[1] && st.pos.alt == s.lla[2];
}

static Bytes concat(std::initializer_list<Bytes> parts) {
    Bytes out;
    for (const Bytes &p : parts) {
        out.insert(out.end(), p.begin(), p.end());
    }
    return out;
}

static Bytes bytes_of(const std::string &s) {
    return Bytes(s.begin(), s.end());
}

static size_t count_fa(const Bytes &b) {
    size_t n = 0;
    for (uint8_t x : b) {
        n += x == 0xFA ? 1 : 0;
    }
    return n;
}

static void feed(Vn200 &vn, const Bytes &b, uint64_t now) {
    vn.feed(b.data(), b.size(), now);
}

static void feed(Vn200 &vn, const std::string &s, uint64_t now) {
    vn.feed(reinterpret_cast<const uint8_t *>(s.data()), s.size(), now);
}

// A default VNINS line, longer than any reply
static const std::string VNINS = vn_line(
    "VNINS,333374.123456,2201,0000,+012.345,-001.250,+000.500,+37.12345678,"
    "-121.12345678,+00045.123,+000.000,+000.000,+000.000,25.0,1.0,0.12");

// Runs the whole configuration against a sensor that echoes every command
static void configure(Vn200 &vn, uint64_t &now) {
    size_t first = writes.size();
    vn.start(now);
    for (size_t i = 0; i < COMMAND_COUNT && writes.size() == first + i + 1; i++) {
        now += 2000;
        feed(vn, echo_of(writes.back()), now);
        vn.poll(now);
    }
    CHECK(writes.size() == first + COMMAND_COUNT);
    CHECK(vn.configured());
}

static void test_crc_reference() {
    // CRC-16/XMODEM check value, to trust the packet builder
    const char *check = "123456789";
    CHECK(crc16(reinterpret_cast<const uint8_t *>(check), 9) == 0x31C3);
}

static void test_command_sequence() {
    reset_fakes();
    Vn200 vn{fake_write};
    uint64_t now = 1000;

    vn.start(now);
    for (size_t i = 0; i < COMMAND_COUNT; i++) {
        CHECK(writes.size() == i + 1);
        if (writes.size() != i + 1) {
            return;
        }
        CHECK(writes[i] == EXPECTED[i]);
        CHECK(!vn.configured());

        // $ payload * two hex digits \r\n, with the XOR of the payload
        const std::string &w = writes[i];
        size_t star = w.find('*');
        CHECK(w[0] == '$' && star == w.size() - 5 && w.compare(w.size() - 2, 2, "\r\n") == 0);
        char sum[3];
        std::snprintf(sum, sizeof(sum), "%02X", xor_of(w.substr(1, star - 1)));
        CHECK(w.compare(star + 1, 2, sum) == 0);

        now += 3000;
        feed(vn, echo_of(w), now);
        vn.poll(now);
    }
    CHECK(writes.size() == COMMAND_COUNT);
    CHECK(vn.configured());
    CHECK(vn.stats().configs_started == 1);
    CHECK(vn.stats().configs_done == 1);
    CHECK(vn.stats().command_errors == 0);
    CHECK(vn.stats().command_timeouts == 0);
    CHECK(vn.stats().write_failures == 0);

    // The group byte and type words written to register 75 are the header the
    // parser expects, type words little-endian in the packet
    Bytes p = good_packet(sample(0));
    CHECK(std::string(EXPECTED[4]).find(",34,0600,0002,000A*") != std::string::npos);
    CHECK(p[1] == 0x34 && p[2] == 0x00 && p[3] == 0x06 && p[4] == 0x02 && p[5] == 0x00 &&
          p[6] == 0x0A && p[7] == 0x00);
    CHECK(p.size() == Vn200::PACKET_SIZE);

    // Nothing else goes out while running, and nothing is saved to flash
    vn.poll(now + DATA_TIMEOUT - 1);
    CHECK(writes.size() == COMMAND_COUNT);
    for (const std::string &w : writes) {
        CHECK(w.find("VNWNV") == std::string::npos);
    }
}

static void test_idle_before_start() {
    reset_fakes();
    Vn200 vn{fake_write};
    feed(vn, good_packet(sample(1)), 10);
    feed(vn, vn_line("VNASY,0"), 20);
    vn.poll(30);
    vn.poll(10'000'000);
    VectornavState st;
    CHECK(!vn.update_state(st));
    CHECK(!vn.configured());
    CHECK(writes.empty());
    CHECK(vn.stats().packets == 0);
    CHECK(vn.stats().configs_started == 0);
    CHECK(vn.stats().bytes_skipped == 0);
}

// Wrong and broken lines don't count as the reply, a split one does
static void test_replies_among_noise() {
    reset_fakes();
    Vn200 vn{fake_write};
    uint64_t now = 500;
    vn.start(now);
    CHECK(writes.size() == 1); // $VNASY,0

    std::string noise = VNINS + vn_line("VNYPR,+012.345,-001.250,+000.500") +
                        vn_line("VNASY,1") +    // other output state
                        vn_line("VNWRG,06,0") + // other command
                        "$VNASY,0*4E\r\n" +     // bad checksum
                        "$VNASY,0*4\r\n" +      // short checksum
                        "$VNASY,0\r\n" +        // no checksum
                        "$VNA";                 // cut off by the packet after it
    feed(vn, noise, now += 1000);
    feed(vn, good_packet(sample(2)), now += 1000);
    feed(vn, Bytes{0x00, 0xFA, 0x24, 0x0D, 0x0A, 0xFF, 0x7F}, now += 1000);
    vn.poll(now);
    CHECK(writes.size() == 1);
    CHECK(vn.stats().packets == 0);
    CHECK(vn.stats().command_errors == 0);
    CHECK(vn.stats().command_timeouts == 0);

    // Binary bytes in the middle of a line end it
    std::string cut = vn_line("VNASY,0");
    std::string broken = cut.substr(0, 4) + '\xFA' + cut.substr(4);
    feed(vn, broken, now += 1000);
    vn.poll(now);
    CHECK(writes.size() == 1);

    // The real reply one byte at a time
    std::string reply = vn_line("VNASY,0");
    for (size_t i = 0; i < reply.size(); i++) {
        vn.poll(now);
        CHECK(writes.size() == 1);
        feed(vn, reply.substr(i, 1), now += 10);
    }
    vn.poll(now);
    CHECK(writes.size() == 2);
    CHECK(writes[1] == EXPECTED[1]);

    // Register numbers compare as numbers: 6 answers 06
    feed(vn, vn_line("VNWRG,6,0"), now += 1000);
    vn.poll(now);
    CHECK(writes.size() == 3);
    CHECK(writes[2] == EXPECTED[2]);

    // A reply with the 16-bit CRC instead of the XOR checksum
    feed(vn, vn_line_crc("VNWRG,76,0,0,00"), now += 1000);
    vn.poll(now);
    CHECK(writes.size() == 4);

    // A second echo for 76 doesn't answer 77
    feed(vn, vn_line("VNWRG,76,0,0,00"), now += 1000);
    vn.poll(now);
    CHECK(writes.size() == 4);
    feed(vn, echo_of(writes[3]), now += 1000);
    vn.poll(now);
    CHECK(writes.size() == 5);
    CHECK(writes[4] == EXPECTED[4]);

    // The reply in the middle of a chunk of other traffic
    std::string chunk = VNINS + echo_of(writes[4]) + VNINS;
    feed(vn, chunk, now += 1000);
    vn.poll(now);
    CHECK(writes.size() == 6);
    CHECK(writes[5] == EXPECTED[5]);
    feed(vn, echo_of(writes[5]), now += 1000);
    vn.poll(now);
    CHECK(writes.size() == 7);
    CHECK(writes[6] == EXPECTED[6]);

    // The last reply and the first packet in one chunk
    Bytes last = concat({bytes_of(echo_of(writes[6])), good_packet(sample(3))});
    feed(vn, last, now += 1000);
    CHECK(vn.configured());
    CHECK(vn.stats().packets == 1);
    VectornavState st;
    CHECK(vn.update_state(st));
    CHECK(matches(st, sample(3)));
    CHECK(vn.stats().configs_done == 1);
    CHECK(vn.stats().command_errors == 0);
    CHECK(vn.stats().command_timeouts == 0);
}

static void test_vnerr_reply_retries() {
    reset_fakes();
    Vn200 vn{fake_write};
    uint64_t now = 100;
    vn.start(now);
    feed(vn, echo_of(writes.back()), now += 1000);
    vn.poll(now);
    CHECK(writes.size() == 2); // $VNWRG,06

    // An error that can come at any time doesn't answer the command
    feed(vn, vn_line("VNERR,0B"), now += 1000);
    vn.poll(now);
    CHECK(writes.size() == 2);
    CHECK(vn.stats().sensor_errors == 1);
    CHECK(vn.stats().command_errors == 0);
    CHECK(vn.stats().last_error == 0x0B);

    for (int i = 0; i < 3; i++) {
        feed(vn, echo_of(writes.back()), now += 1000);
        vn.poll(now);
    }
    CHECK(writes.size() == 5);
    CHECK(writes[4] == EXPECTED[4]); // register 75

    // Insufficient baud rate: the same command again
    feed(vn, vn_line("VNERR,0C"), now += 1000);
    CHECK(vn.stats().command_errors == 1);
    CHECK(vn.stats().last_error == 0x0C);
    vn.poll(now);
    CHECK(writes.size() == 6);
    CHECK(writes[5] == EXPECTED[4]);

    // Invalid checksum, with the error sent with the 16-bit CRC
    feed(vn, vn_line_crc("VNERR,03"), now += 1000);
    vn.poll(now);
    CHECK(vn.stats().command_errors == 2);
    CHECK(vn.stats().last_error == 0x03);
    CHECK(writes.size() == 7);
    CHECK(writes[6] == EXPECTED[4]);

    feed(vn, echo_of(writes.back()), now += 1000);
    vn.poll(now);
    CHECK(writes.size() == 8);
    CHECK(writes[7] == EXPECTED[5]);
    feed(vn, echo_of(writes.back()), now += 1000);
    vn.poll(now);
    CHECK(writes.size() == 9);
    CHECK(writes[8] == EXPECTED[6]);
    feed(vn, echo_of(writes.back()), now += 1000);
    vn.poll(now);
    CHECK(vn.configured());
    CHECK(vn.stats().configs_started == 1);
    CHECK(vn.stats().command_timeouts == 0);
}

static void test_reply_timeout_retries() {
    reset_fakes();
    Vn200 vn{fake_write};
    uint64_t t0 = 5000;
    vn.start(t0);
    CHECK(writes.size() == 1);

    vn.poll(t0 + REPLY_TIMEOUT - 1);
    CHECK(writes.size() == 1);
    CHECK(vn.stats().command_timeouts == 0);

    vn.poll(t0 + REPLY_TIMEOUT);
    CHECK(writes.size() == 2);
    CHECK(writes[1] == EXPECTED[0]);
    CHECK(vn.stats().command_timeouts == 1);

    // A late reply to the first try answers the second
    uint64_t now = t0 + REPLY_TIMEOUT + 3000;
    feed(vn, echo_of(writes[0]), now);
    vn.poll(now);
    CHECK(writes.size() == 3);
    CHECK(writes[2] == EXPECTED[1]);

    // The timeout starts again for each command
    vn.poll(now + REPLY_TIMEOUT - 1);
    CHECK(writes.size() == 3);
    vn.poll(now + REPLY_TIMEOUT);
    CHECK(writes.size() == 4);
    CHECK(writes[3] == EXPECTED[1]);
    CHECK(vn.stats().command_timeouts == 2);
}

static void test_write_failure_retries() {
    reset_fakes();
    Vn200 vn{fake_write};
    write_ok = false;
    uint64_t t0 = 0;
    vn.start(t0);
    CHECK(writes.size() == 1);
    CHECK(vn.stats().write_failures == 1);

    write_ok = true;
    vn.poll(t0 + REPLY_TIMEOUT - 1);
    CHECK(writes.size() == 1);
    vn.poll(t0 + REPLY_TIMEOUT);
    CHECK(writes.size() == 2);
    CHECK(writes[1] == EXPECTED[0]);
    CHECK(vn.stats().write_failures == 1);

    feed(vn, echo_of(writes[1]), t0 + REPLY_TIMEOUT + 1000);
    vn.poll(t0 + REPLY_TIMEOUT + 1000);
    CHECK(writes.size() == 3);
}

static void test_backoff_and_restart() {
    reset_fakes();
    Vn200 vn{fake_write};
    uint64_t t0 = 20'000;
    vn.start(t0);

    // No reply at all: COMMAND_TRIES sends, then quiet for BACKOFF_US
    uint64_t t = t0;
    for (uint32_t i = 1; i < Vn200::COMMAND_TRIES; i++) {
        t += REPLY_TIMEOUT;
        vn.poll(t);
        CHECK(writes.size() == i + 1);
    }
    t += REPLY_TIMEOUT;
    vn.poll(t);
    CHECK(writes.size() == Vn200::COMMAND_TRIES);
    CHECK(vn.stats().command_timeouts == Vn200::COMMAND_TRIES);

    vn.poll(t + Vn200::BACKOFF_US - 1);
    CHECK(writes.size() == Vn200::COMMAND_TRIES);
    CHECK(vn.stats().configs_started == 1);

    vn.poll(t + Vn200::BACKOFF_US);
    CHECK(writes.size() == Vn200::COMMAND_TRIES + 1);
    CHECK(writes.back() == EXPECTED[0]);
    CHECK(vn.stats().configs_started == 2);

    // A command that keeps getting refused starts the whole sequence over
    uint64_t now = t + Vn200::BACKOFF_US;
    for (size_t i = 0; i < 4; i++) {
        feed(vn, echo_of(writes.back()), now += 1000);
        vn.poll(now);
    }
    CHECK(writes.back() == EXPECTED[4]);
    size_t sent = writes.size();
    for (uint32_t i = 0; i < Vn200::COMMAND_TRIES; i++) {
        feed(vn, vn_line("VNERR,0C"), now += 1000);
        vn.poll(now);
    }
    CHECK(writes.size() == sent + Vn200::COMMAND_TRIES - 1);
    CHECK(vn.stats().command_errors == Vn200::COMMAND_TRIES);
    vn.poll(now + Vn200::BACKOFF_US - 1);
    CHECK(writes.size() == sent + Vn200::COMMAND_TRIES - 1);
    vn.poll(now + Vn200::BACKOFF_US);
    CHECK(writes.back() == EXPECTED[0]);
    CHECK(vn.stats().configs_started == 3);
    CHECK(vn.stats().configs_done == 0);
    CHECK(!vn.configured());
}

static void test_decode_exact() {
    reset_fakes();
    Vn200 vn{fake_write};
    uint64_t now = 1000;
    configure(vn, now);

    VectornavState st;
    CHECK(!vn.update_state(st));

    Sample s{
        {-0.0f, -9.80665f, 1.17549435e-38f},
        {-3.14159274f, 0.000123f, 65504.0f},
        {179.999f, -89.5f, -0.25f},
        {-33.86881965432101, 151.20929487654321, -123.456789012345},
        {-1e-6f, 341.5f, -27.0f},
    };
    feed(vn, good_packet(s), now += 777);
    CHECK(vn.update_state(st));
    CHECK(matches(st, s));
    CHECK(std::signbit(st.accel[0]));
    CHECK(!vn.update_state(st)); // nothing new since
    CHECK(vn.last_packet_us() == now);
    CHECK(vn.stats().packets == 1);
    CHECK(vn.stats().crc_errors == 0);
    CHECK(vn.stats().bad_headers == 0);
    CHECK(vn.stats().bytes_skipped == 0);
}

static void test_byte_at_a_time() {
    reset_fakes();
    Vn200 vn{fake_write};
    uint64_t now = 0;
    configure(vn, now);

    Bytes p = good_packet(sample(4));
    VectornavState st;
    for (size_t i = 0; i < p.size(); i++) {
        CHECK(!vn.update_state(st));
        vn.feed(&p[i], 1, ++now);
        vn.poll(now);
    }
    CHECK(vn.update_state(st));
    CHECK(matches(st, sample(4)));
    CHECK(vn.last_packet_us() == now);
    CHECK(vn.stats().packets == 1);
    CHECK(vn.stats().bytes_skipped == 0);
}

static void test_back_to_back() {
    reset_fakes();
    Vn200 vn{fake_write};
    uint64_t now = 0;
    configure(vn, now);

    feed(vn, concat({good_packet(sample(5)), good_packet(sample(6))}), now += 10);
    CHECK(vn.stats().packets == 2);
    VectornavState st;
    CHECK(vn.update_state(st));
    CHECK(matches(st, sample(6))); // the newest
    CHECK(!vn.update_state(st));
    CHECK(vn.stats().bytes_skipped == 0);
}

// Packets with garbage and ASCII lines between them, as one chunk and in
// chunks of every size
static void test_garbage_and_chunks() {
    Bytes stream;
    size_t junk = 0;
    size_t junk_fa = 0;
    for (int i = 0; i < 20; i++) {
        Bytes g;
        for (int k = 0; k < i; k++) {
            g.push_back(static_cast<uint8_t>(37 * k + 11 * i));
        }
        if (i % 3 == 0) {
            g.push_back(0xFA);
        }
        Bytes ascii = bytes_of(i % 2 == 0 ? VNINS : vn_line("VNYPR,+012.345,-001.250,+000.500"));
        junk += g.size() + ascii.size();
        junk_fa += count_fa(g) + count_fa(ascii);
        stream = concat({stream, g, ascii, good_packet(sample(i))});
    }

    for (size_t chunk : {stream.size(), size_t{1}, size_t{7}, size_t{81}, size_t{82}, size_t{83},
                         size_t{128}, size_t{1000}}) {
        reset_fakes();
        Vn200 vn{fake_write};
        uint64_t now = 0;
        configure(vn, now);

        for (size_t pos = 0; pos < stream.size(); pos += chunk) {
            size_t n = stream.size() - pos < chunk ? stream.size() - pos : chunk;
            vn.feed(stream.data() + pos, n, now += 100);
            vn.poll(now);
        }
        CHECK(vn.stats().packets == 20);
        CHECK(vn.stats().crc_errors == 0);
        CHECK(vn.stats().bytes_skipped == junk);
        CHECK(vn.stats().bad_headers == junk_fa);
        VectornavState st;
        CHECK(vn.update_state(st));
        CHECK(matches(st, sample(19)));
    }
}

static void test_fa_in_payload_and_garbage() {
    reset_fakes();
    Vn200 vn{fake_write};
    uint64_t now = 0;
    configure(vn, now);

    // 0xFA inside the payload, once followed by the right group byte, once by
    // a whole header (in a double)
    Sample s = sample(7);
    s.accel[1] = f32_bits(0xFA34FA00);
    s.lla[1] = f64_bits(0x000A0002060034FAULL);
    s.vel[2] = f32_bits(0xFAFAFAFA);
    Bytes p = good_packet(s);
    CHECK(count_fa(p) >= 8);
    feed(vn, p, now += 10);
    VectornavState st;
    CHECK(vn.update_state(st));
    CHECK(matches(st, s));
    CHECK(vn.stats().bad_headers == 0);
    CHECK(vn.stats().bytes_skipped == 0);

    // 0xFA in garbage, including a header cut short and one right before the
    // real sync byte
    Bytes garbage{0x00, 0xFA, 0x12, 0xFA, 0x34, 0x00, 0x99, 0xFA, 0x34, 0x00, 0x06, 0x02, 0xFA};
    feed(vn, concat({garbage, good_packet(sample(8))}), now += 10);
    CHECK(vn.update_state(st));
    CHECK(matches(st, sample(8)));
    CHECK(vn.stats().packets == 2);
    CHECK(vn.stats().bad_headers == 4);
    CHECK(vn.stats().bytes_skipped == garbage.size());
    CHECK(vn.stats().crc_errors == 0);
}

static void test_crc_error_resync() {
    reset_fakes();
    Vn200 vn{fake_write};
    uint64_t now = 0;
    configure(vn, now);

    // One flipped bit, then a good packet right behind it
    Bytes bad = good_packet(sample(9));
    bad[40] ^= 0x10;
    feed(vn, concat({bad, good_packet(sample(10))}), now += 10);
    CHECK(vn.stats().crc_errors == 1);
    CHECK(vn.stats().packets == 1);
    CHECK(vn.stats().bytes_skipped == bad.size());
    VectornavState st;
    CHECK(vn.update_state(st));
    CHECK(matches(st, sample(10)));

    // A broken packet with a whole header inside its payload: the false
    // packet starting there runs into the next one, which must survive
    Bytes payload = payload_of(sample(11));
    const uint8_t fake_header[] = {0xFA, 0x34, 0x00, 0x06, 0x02, 0x00, 0x0A, 0x00};
    std::memcpy(&payload[36], fake_header, sizeof(fake_header)); // PosLla lat
    Bytes tricky = packet(0x34, {0x0600, 0x0002, 0x000A}, payload);
    tricky[20] ^= 0x01;
    uint32_t crc_before = vn.stats().crc_errors;
    uint32_t skipped_before = vn.stats().bytes_skipped;
    feed(vn, concat({tricky, good_packet(sample(12))}), now += 10);
    CHECK(vn.stats().crc_errors == crc_before + 2);
    CHECK(vn.stats().packets == 2);
    CHECK(vn.stats().bytes_skipped == skipped_before + tricky.size());
    CHECK(vn.update_state(st));
    CHECK(matches(st, sample(12)));

    // A packet cut short, the next one complete
    Bytes cut = good_packet(sample(13));
    cut.resize(40);
    skipped_before = vn.stats().bytes_skipped;
    feed(vn, concat({cut, good_packet(sample(14))}), now += 10);
    CHECK(vn.stats().packets == 3);
    CHECK(vn.stats().bytes_skipped == skipped_before + cut.size());
    CHECK(vn.update_state(st));
    CHECK(matches(st, sample(14)));

    // The same, one byte at a time
    skipped_before = vn.stats().bytes_skipped;
    Bytes both = concat({bad, cut, good_packet(sample(15))});
    for (uint8_t b : both) {
        vn.feed(&b, 1, ++now);
    }
    CHECK(vn.stats().packets == 4);
    CHECK(vn.stats().bytes_skipped == skipped_before + bad.size() + cut.size());
    CHECK(vn.update_state(st));
    CHECK(matches(st, sample(15)));
}

static void test_wrong_headers_rejected() {
    reset_fakes();
    Vn200 vn{fake_write};
    uint64_t now = 0;
    configure(vn, now);

    Sample s = sample(16);
    Bytes full = payload_of(s);
    Bytes imu(full.begin(), full.begin() + 24);
    Bytes ypr(full.begin() + 24, full.begin() + 36);
    Bytes ins(full.begin() + 36, full.end());

    // The three messages the Mbed build set up, valid packets but not ours
    Bytes wrong = concat({packet(0x04, {0x0600}, imu), packet(0x20, {0x000A}, ins),
                          packet(0x10, {0x0002}, ypr)});
    // Our groups with other types: Accel only, Quaternion instead of Ypr,
    // PosLla only
    Bytes accel_only(full.begin(), full.begin() + 12);
    Bytes other_types = concat({packet(0x34, {0x0200, 0x0002, 0x000A},
                                       concat({accel_only, Bytes(full.begin() + 24, full.end())})),
                                packet(0x34, {0x0600, 0x0004, 0x000A},
                                       concat({imu, Bytes(16, 0x11), ins})),
                                packet(0x34, {0x0600, 0x0002, 0x0002},
                                       concat({imu, ypr, Bytes(full.begin() + 36, full.begin() + 60)}))});
    // Group byte with Common (bit 0) added, otherwise the same
    Bytes extra_group = packet(0x35, {0x0001, 0x0600, 0x0002, 0x000A}, concat({Bytes(8, 0x22), full}));

    Bytes all = concat({wrong, other_types, extra_group});
    feed(vn, all, now += 10);
    CHECK(vn.stats().packets == 0);
    CHECK(vn.stats().crc_errors == 0);
    CHECK(vn.stats().bad_headers == count_fa(all));
    CHECK(vn.stats().bytes_skipped == all.size());
    VectornavState st;
    CHECK(!vn.update_state(st));

    feed(vn, good_packet(s), now += 10);
    CHECK(vn.stats().packets == 1);
    CHECK(vn.update_state(st));
    CHECK(matches(st, s));
}

static void test_errors_while_running() {
    reset_fakes();
    Vn200 vn{fake_write};
    uint64_t now = 0;
    configure(vn, now);

    std::string err = vn_line("VNERR,0B");
    Bytes stream = concat({good_packet(sample(17)), bytes_of(err), bytes_of(VNINS),
                           good_packet(sample(18))});
    feed(vn, stream, now += 10);
    CHECK(vn.stats().packets == 2);
    CHECK(vn.stats().sensor_errors == 1);
    CHECK(vn.stats().last_error == 0x0B);
    CHECK(vn.stats().command_errors == 0);
    CHECK(vn.stats().bytes_skipped == err.size() + VNINS.size());
    CHECK(vn.configured());
}

// Events, in order, for the console
static std::vector<std::pair<Vn200::Event, uint8_t>> events;

static void record_event(Vn200::Event event, uint8_t code) {
    events.emplace_back(event, code);
}

static void test_events() {
    using E = Vn200::Event;
    using List = std::vector<std::pair<E, uint8_t>>;
    reset_fakes();
    events.clear();
    Vn200 vn{fake_write, record_event};
    CHECK(std::string(vn.model()).empty());

    // No reply at all: one BACKOFF with code 0, nothing per try
    uint64_t now = 1000;
    vn.start(now);
    for (uint32_t i = 0; i < Vn200::COMMAND_TRIES; i++) {
        vn.poll(now += REPLY_TIMEOUT);
    }
    CHECK(events == (List{{E::BACKOFF, 0}}));

    // A refused command, an async error during the setup, then the model and
    // the end of the setup
    events.clear();
    vn.poll(now += Vn200::BACKOFF_US);
    feed(vn, echo_of(writes.back()), now += 1000);
    vn.poll(now);
    feed(vn, vn_line("VNERR,0C"), now += 1000);
    vn.poll(now);
    feed(vn, vn_line("VNERR,0A"), now += 1000);
    vn.poll(now);
    while (!vn.configured() && writes.size() < 30) {
        feed(vn, echo_of(writes.back()), now += 1000);
        vn.poll(now);
    }
    CHECK(events == (List{{E::COMMAND_ERROR, 0x0C}, {E::SENSOR_ERROR, 0x0A}, {E::MODEL, 0}, {E::CONFIGURED, 0}}));
    CHECK(std::string(vn.model()) == MODEL);

    // While running: an async error, then the data timeout
    events.clear();
    feed(vn, concat({good_packet(sample(1)), bytes_of(vn_line("VNERR,0B"))}), now += 10);
    vn.poll(now);
    vn.poll(now + DATA_TIMEOUT);
    CHECK(events == (List{{E::SENSOR_ERROR, 0x0B}, {E::DATA_TIMEOUT, 0}}));

    // A refused command three times: BACKOFF carries its code
    events.clear();
    now += DATA_TIMEOUT;
    for (uint32_t i = 0; i < Vn200::COMMAND_TRIES; i++) {
        feed(vn, vn_line("VNERR,08"), now += 1000);
        vn.poll(now);
    }
    CHECK(events == (List{{E::COMMAND_ERROR, 0x08}, {E::COMMAND_ERROR, 0x08}, {E::COMMAND_ERROR, 0x08},
                          {E::BACKOFF, 0x08}}));

    // A model reply without the value leaves the model empty
    reset_fakes();
    Vn200 bare{fake_write};
    now = 0;
    bare.start(now);
    while (writes.size() < COMMAND_COUNT) {
        feed(bare, writes.back(), now += 1000);
        bare.poll(now);
    }
    CHECK(std::string(bare.model()).empty());

    CHECK(std::string(Vn200::error_name(0x0B)) == "OutputBufferOverflow");
    CHECK(std::string(Vn200::error_name(0xFF)) == "ErrorBufferOverflow");
    CHECK(std::string(Vn200::error_name(0x42)) == "Unknown");
}

static void test_packets_ignored_while_configuring() {
    reset_fakes();
    Vn200 vn{fake_write};
    uint64_t now = 0;
    vn.start(now);
    feed(vn, good_packet(sample(19)), now += 10);
    VectornavState st;
    CHECK(!vn.update_state(st));
    CHECK(vn.stats().packets == 0);
    CHECK(vn.stats().bytes_skipped == 0);
}

static void test_data_timeout_reconfigures() {
    reset_fakes();
    Vn200 vn{fake_write};
    uint64_t now = 1000;
    configure(vn, now);

    // No packet at all: counted from the end of the configuration
    uint64_t done = now;
    vn.poll(done + DATA_TIMEOUT - 1);
    CHECK(vn.configured());
    vn.poll(done + DATA_TIMEOUT);
    CHECK(!vn.configured());
    CHECK(vn.stats().data_timeouts == 1);
    CHECK(vn.stats().configs_started == 2);
    CHECK(writes.size() == COMMAND_COUNT + 1);
    CHECK(writes.back() == EXPECTED[0]);

    now = done + DATA_TIMEOUT;
    for (size_t i = 0; i < COMMAND_COUNT; i++) {
        feed(vn, echo_of(writes.back()), now += 1000);
        vn.poll(now);
    }
    CHECK(vn.configured());
    CHECK(vn.stats().configs_done == 2);

    // 100 Hz for a second
    for (int i = 0; i < 100; i++) {
        feed(vn, good_packet(sample(i)), now += 10'000);
        vn.poll(now);
        CHECK(vn.configured());
    }
    CHECK(vn.stats().packets == 100);
    VectornavState st;
    CHECK(vn.update_state(st));
    CHECK(matches(st, sample(99)));

    // A packet with a bad CRC doesn't count
    uint64_t last = now;
    Bytes bad = good_packet(sample(1));
    bad[50] ^= 0xFF;
    feed(vn, bad, last + 200'000);
    vn.poll(last + 200'000);
    vn.poll(last + DATA_TIMEOUT - 1);
    CHECK(vn.configured());
    vn.poll(last + DATA_TIMEOUT);
    CHECK(!vn.configured());
    CHECK(vn.stats().data_timeouts == 2);
    CHECK(writes.back() == EXPECTED[0]);

    // The last values stay, but aren't handed out again
    VectornavState kept = st;
    CHECK(!vn.update_state(st));
    CHECK(matches(st, sample(99)));
    CHECK(vn.last_packet_us() == last);

    now = last + DATA_TIMEOUT;
    for (size_t i = 0; i < COMMAND_COUNT; i++) {
        feed(vn, echo_of(writes.back()), now += 1000);
        vn.poll(now);
    }
    CHECK(vn.configured());
    CHECK(!vn.update_state(kept));
    feed(vn, good_packet(sample(42)), now += 10'000);
    CHECK(vn.update_state(st));
    CHECK(matches(st, sample(42)));
}

static void test_start_while_running() {
    reset_fakes();
    Vn200 vn{fake_write};
    uint64_t now = 0;
    configure(vn, now);

    // Half a packet, then start(): it's dropped, not glued to later bytes
    Bytes p = good_packet(sample(20));
    vn.feed(p.data(), 41, now += 10);
    vn.start(now);
    CHECK(!vn.configured());
    CHECK(vn.stats().configs_started == 2);
    CHECK(writes.back() == EXPECTED[0]);
    for (size_t i = 0; i < COMMAND_COUNT; i++) {
        feed(vn, echo_of(writes.back()), now += 1000);
        vn.poll(now);
    }
    CHECK(vn.configured());
    vn.feed(p.data() + 41, p.size() - 41, now += 10);
    feed(vn, good_packet(sample(21)), now += 10);
    CHECK(vn.stats().packets == 1);
    VectornavState st;
    CHECK(vn.update_state(st));
    CHECK(matches(st, sample(21)));
}

// Times past 2^32 us (~71.6 min), where a 32-bit microsecond count wraps
static void test_time_past_32_bits() {
    // Reply timeout across 2^32
    reset_fakes();
    Vn200 vn{fake_write};
    uint64_t t0 = 0xFFFFFFFFU - 30'000;
    vn.start(t0);
    vn.poll(t0 + 1); // the deadline is past 2^32, the time not yet
    vn.poll(0xFFFFFFFFU);
    CHECK(writes.size() == 1);
    vn.poll(t0 + REPLY_TIMEOUT - 1); // past 2^32
    CHECK(writes.size() == 1);
    CHECK(vn.stats().command_timeouts == 0);
    vn.poll(t0 + REPLY_TIMEOUT);
    CHECK(writes.size() == 2);
    CHECK(vn.stats().command_timeouts == 1);

    // Backoff across 2^32
    reset_fakes();
    Vn200 vb{fake_write};
    uint64_t t = 0xFFFFFFFFU - 3 * REPLY_TIMEOUT - 500'000;
    vb.start(t);
    for (uint32_t i = 0; i < Vn200::COMMAND_TRIES; i++) {
        t += REPLY_TIMEOUT;
        vb.poll(t);
    }
    CHECK(writes.size() == Vn200::COMMAND_TRIES);
    vb.poll(t + 1);
    vb.poll(0xFFFFFFFFU);
    CHECK(writes.size() == Vn200::COMMAND_TRIES);
    vb.poll(t + Vn200::BACKOFF_US - 1); // past 2^32
    CHECK(writes.size() == Vn200::COMMAND_TRIES);
    vb.poll(t + Vn200::BACKOFF_US);
    CHECK(writes.size() == Vn200::COMMAND_TRIES + 1);
    CHECK(vb.stats().configs_started == 2);

    // Packets on both sides of 2^32
    reset_fakes();
    Vn200 vd{fake_write};
    uint64_t now = 0xFFFFFFFFU - 300'000;
    configure(vd, now);
    for (int i = 0; i < 40; i++) {
        feed(vd, good_packet(sample(i)), now += 10'000);
        vd.poll(now);
        CHECK(vd.configured());
    }
    CHECK(now > 0xFFFFFFFFU);
    CHECK(vd.stats().packets == 40);
    CHECK(vd.stats().data_timeouts == 0);

    // Data timeout across 2^32
    reset_fakes();
    Vn200 ve{fake_write};
    now = 0xFFFFFFFFU - 100'000;
    configure(ve, now);
    uint64_t last = 0xFFFFFFF0U;
    feed(ve, good_packet(sample(31)), last);
    ve.poll(last);
    ve.poll(last + DATA_TIMEOUT - 1); // past 2^32
    CHECK(ve.configured());
    ve.poll(last + DATA_TIMEOUT);
    CHECK(!ve.configured());
    CHECK(ve.stats().data_timeouts == 1);
    CHECK(ve.last_packet_us() == last);
}

int main() {
    make_crc_table();

    test_crc_reference();
    test_command_sequence();
    test_idle_before_start();
    test_replies_among_noise();
    test_vnerr_reply_retries();
    test_reply_timeout_retries();
    test_write_failure_retries();
    test_backoff_and_restart();
    test_decode_exact();
    test_byte_at_a_time();
    test_back_to_back();
    test_garbage_and_chunks();
    test_fa_in_payload_and_garbage();
    test_crc_error_resync();
    test_wrong_headers_rejected();
    test_errors_while_running();
    test_events();
    test_packets_ignored_while_configuring();
    test_data_timeout_reconfigures();
    test_start_while_running();
    test_time_past_32_bits();

    if (failures != 0) {
        std::printf("%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("all tests passed\n");
    return 0;
}
