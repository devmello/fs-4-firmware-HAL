#ifndef VN200_H
#define VN200_H

// VectorNav VN-200 over a UART, without the VectorNav SDK. Sets up one binary
// output message with ASCII commands, then parses that message. Nothing
// blocks: main feeds the received bytes and calls poll() every loop pass.
//
// Section and table numbers refer to the VN-200 ICD for firmware v2.1.0.0
// (ICD20006-R1).
//
// Times are the free-running 32-bit microsecond counter (timebase_micros()).
// Pass the current time to every call, never an older one than before. All
// intervals are unsigned differences, so the wrap after ~71 minutes is fine.

#include <cstddef>
#include <cstdint>

#include "vectornav_state.h"

class Vn200 {
public:
    using WriteFn = bool (*)(const uint8_t *data, size_t len); // imu_uart_write

    static constexpr uint32_t RESPONSE_TIMEOUT_US = 100'000; // per command try
    static constexpr uint32_t COMMAND_TRIES = 3;             // then back off
    static constexpr uint32_t BACKOFF_US = 1'000'000;        // before starting over
    static constexpr uint32_t DATA_TIMEOUT_US = 500'000;     // no good packet: reconfigure

    // 8 header + 72 payload + 2 CRC, see vn200.cpp
    static constexpr size_t PACKET_SIZE = 82;

    struct Stats {
        uint32_t packets = 0;          // good binary packets
        uint32_t crc_errors = 0;       // our header, bad CRC
        uint32_t bad_headers = 0;      // 0xFA not followed by the header we configured
        uint32_t bytes_skipped = 0;    // received while running, not part of a good packet
        uint32_t configs_started = 0;
        uint32_t configs_done = 0;
        uint32_t command_errors = 0;   // $VNERR in reply to a command
        uint32_t command_timeouts = 0; // no reply within RESPONSE_TIMEOUT_US
        uint32_t write_failures = 0;   // write() refused a command
        uint32_t data_timeouts = 0;    // reconfigured after DATA_TIMEOUT_US without a packet
        uint32_t sensor_errors = 0;    // other $VNERR, e.g. 0B (output buffer overflow)
        uint8_t last_error = 0;        // code of the last $VNERR, ICD table 1.6
    };

    explicit Vn200(WriteFn write) : write_fn(write) {}

    // Starts the configuration from the first command, also when running
    void start(uint32_t now_us);

    // Bytes from the UART in the order received, any amount per call. Call
    // before poll() in the same pass, so a reply that has arrived isn't
    // taken for a timeout.
    void feed(const uint8_t *data, size_t len, uint32_t now_us);

    // Sends the next command, handles reply timeouts, retries, the backoff
    // and the data timeout
    void poll(uint32_t now_us);

    // Copies the newest measurement into state and returns true if one
    // arrived since the last call. Otherwise leaves state as it is.
    bool update_state(VectornavState &state);

    const Stats &stats() const { return counts; }

    // The sensor acknowledged every command, and there has been no data
    // timeout or start() since
    bool configured() const { return phase == Phase::RUNNING; }

    // now_us of the feed() call that completed the last good packet. Only
    // meaningful once stats().packets > 0. now - last_packet_us() wraps after
    // ~71 minutes; configured() goes false DATA_TIMEOUT_US after the last
    // packet, so check that too.
    uint32_t last_packet_us() const { return last_packet_time; }

private:
    enum class Phase : uint8_t {
        IDLE,    // before start()
        SEND,    // next command goes out on the next poll()
        WAIT,    // command sent, waiting for its reply
        BACKOFF, // a command failed COMMAND_TRIES times
        RUNNING, // configured, parsing packets
    };

    static constexpr size_t LINE_SIZE = 64;

    WriteFn write_fn;
    Phase phase = Phase::IDLE;
    uint8_t step = 0;  // index of the current command
    uint8_t tries = 0; // sends of the current command
    uint32_t sent_us = 0;
    uint32_t backoff_start_us = 0;
    uint32_t last_data_us = 0; // last good packet, or the end of the configuration
    uint32_t last_packet_time = 0;

    // Received bytes scan() hasn't used up. Between calls it's empty or starts
    // with a 0xFA that may begin a packet.
    uint8_t pkt[PACKET_SIZE] = {};
    size_t pkt_len = 0;

    // ASCII line after the '$'
    char line[LINE_SIZE] = {};
    size_t line_len = 0;
    bool in_line = false;

    VectornavState latest;
    bool fresh = false;
    Stats counts;

    void restart();
    void send(uint32_t now_us);
    void command_failed(uint32_t now_us);
    void command_done(uint32_t now_us);
    void scan(uint32_t now_us);
    void skip(uint8_t byte, uint32_t now_us);
    void decode(const uint8_t *p, uint32_t now_us);
    void line_byte(uint8_t byte, uint32_t now_us);
    void handle_line(uint32_t now_us);
};

#endif
