#ifndef VN200_H
#define VN200_H

// VectorNav VN-200 over a UART, without the VectorNav SDK. Sets up one binary
// output message with ASCII commands, then parses that message. Nothing
// blocks: main feeds the received bytes and calls poll() every loop pass.
//
// Section and table numbers refer to the VN-200 ICD for firmware v2.1.0.0
// (ICD20006-R1).
//
// Times are microseconds from timebase_micros(). Pass the current time to
// every call, never an older one than before.

#include <cstddef>
#include <cstdint>

#include "vectornav_state.h"

class Vn200 {
public:
    using WriteFn = bool (*)(const uint8_t *data, size_t len); // imu_uart_write

    // What happened, for the console. Called from feed() and poll().
    enum class Event : uint8_t {
        MODEL,         // the sensor answered the model read, see model()
        CONFIGURED,    // every command acknowledged
        COMMAND_ERROR, // $VNERR in reply to a command, code = its error code
        BACKOFF,       // a command failed COMMAND_TRIES times, code = the last
                       // try's $VNERR code, or 0 if it got no reply
        DATA_TIMEOUT,  // no good packet for DATA_TIMEOUT_US, configuring again
        SENSOR_ERROR,  // any other $VNERR, code = its error code
    };
    using EventFn = void (*)(Event event, uint8_t code);

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

    explicit Vn200(WriteFn write, EventFn event = nullptr) : write_fn(write), event_fn(event) {}

    // Starts the configuration from the first command, also when running
    void start(uint64_t now_us);

    // Bytes from the UART in the order received, any amount per call. Call
    // before poll() in the same pass, so a reply that has arrived isn't
    // taken for a timeout.
    void feed(const uint8_t *data, size_t len, uint64_t now_us);

    // Sends the next command, handles reply timeouts, retries, the backoff
    // and the data timeout
    void poll(uint64_t now_us);

    // Copies the newest measurement into state and returns true if one
    // arrived since the last call. Otherwise leaves state as it is.
    bool update_state(VectornavState &state);

    const Stats &stats() const { return counts; }

    // The sensor acknowledged every command, and there has been no data
    // timeout or start() since
    bool configured() const { return phase == Phase::RUNNING; }

    // now_us of the feed() call that completed the last good packet. Only
    // meaningful once stats().packets > 0.
    uint64_t last_packet_us() const { return last_packet_time; }

    // Model number from the last read of register 1, "" before
    const char *model() const { return model_name; }

    // Name of a $VNERR code (ICD table 1.6), as the VectorNav SDK's
    // errorCodeToString() gives it
    static const char *error_name(uint8_t code);

private:
    enum class Phase : uint8_t {
        IDLE,    // before start()
        SEND,    // next command goes out on the next poll()
        WAIT,    // command sent, waiting for its reply
        BACKOFF, // a command failed COMMAND_TRIES times
        RUNNING, // configured, parsing packets
    };

    static constexpr size_t LINE_SIZE = 64;
    static constexpr size_t MODEL_SIZE = 24;

    WriteFn write_fn;
    EventFn event_fn;
    Phase phase = Phase::IDLE;
    uint8_t step = 0;  // index of the current command
    uint8_t tries = 0; // sends of the current command
    uint8_t fail_code = 0; // $VNERR code of the last failed try, 0 = no reply
    uint64_t sent_us = 0;
    uint64_t backoff_start_us = 0;
    uint64_t last_data_us = 0; // last good packet, or the end of the configuration
    uint64_t last_packet_time = 0;

    // Received bytes scan() hasn't used up. Between calls it's empty or starts
    // with a 0xFA that may begin a packet.
    uint8_t pkt[PACKET_SIZE] = {};
    size_t pkt_len = 0;

    // ASCII line after the '$'
    char line[LINE_SIZE] = {};
    size_t line_len = 0;
    bool in_line = false;

    char model_name[MODEL_SIZE + 1] = {};

    VectornavState latest;
    bool fresh = false;
    Stats counts;

    void emit(Event event, uint8_t code = 0);
    void restart();
    void send(uint64_t now_us);
    void command_failed(uint64_t now_us);
    void command_done(uint64_t now_us);
    void scan(uint64_t now_us);
    void skip(uint8_t byte, uint64_t now_us);
    void decode(const uint8_t *p, uint64_t now_us);
    void line_byte(uint8_t byte, uint64_t now_us);
    void handle_line(uint64_t now_us);
};

#endif
