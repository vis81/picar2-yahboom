/*
 * timesync_test.cpp — C++ timesync accuracy benchmark
 *
 * Measures STM32↔Pi clock offset stability across configurable sync periods.
 * Captures T4 at CRC-pass inside the read loop (no buffering delay).
 *
 * Build: g++ -O2 -std=c++17 -o timesync_test timesync_test.cpp
 * Usage: ./timesync_test [port] [period_s ...]
 *   port     : serial device (default /dev/ttyYahboom0)
 *   period_s : space-separated list of sync periods in seconds
 *              (default: 0.5 1 2 5 10)
 */

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <numeric>
#include <string>
#include <vector>

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

// ---------------------------------------------------------------------------
// Protocol constants
// ---------------------------------------------------------------------------

static constexpr uint8_t FRAME_START      = 0xAA;
static constexpr uint8_t MSG_TIMESYNC     = 0x84;   // Pi → STM32
static constexpr uint8_t MSG_TIMESYNC_RESP = 0x05;  // STM32 → Pi

static constexpr int PROTO_MAX_PAYLOAD = 32;

// ---------------------------------------------------------------------------
// CRC-8 (poly 0x31, init 0x00, no reflect)
// ---------------------------------------------------------------------------

static uint8_t crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
    }
    return crc;
}

// ---------------------------------------------------------------------------
// Timestamp helpers
// ---------------------------------------------------------------------------

static int64_t now_us()
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000000LL +
           static_cast<int64_t>(ts.tv_nsec) / 1000LL;
}

// ---------------------------------------------------------------------------
// Serial helpers
// ---------------------------------------------------------------------------

static int open_serial(const char *port, speed_t baud)
{
    int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) { perror("open"); return -1; }

    struct termios tty{};
    if (tcgetattr(fd, &tty) < 0) { perror("tcgetattr"); close(fd); return -1; }

    cfmakeraw(&tty);
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) < 0) { perror("tcsetattr"); close(fd); return -1; }
    tcflush(fd, TCIOFLUSH);

    // Switch to blocking reads after configuration
    int flags = fcntl(fd, F_GETFL);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

    return fd;
}

// ---------------------------------------------------------------------------
// Frame encoder
// ---------------------------------------------------------------------------

static void put_le32(uint8_t *p, uint32_t v)
{
    p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF;
    p[2] = (v >> 16) & 0xFF; p[3] = (v >> 24) & 0xFF;
}

static void put_le64(uint8_t *p, int64_t v)
{
    put_le32(p,     static_cast<uint32_t>(static_cast<uint64_t>(v) & 0xFFFFFFFFu));
    put_le32(p + 4, static_cast<uint32_t>(static_cast<uint64_t>(v) >> 32));
}

// Returns total frame length written into buf.
static int encode_frame(uint8_t type, const uint8_t *payload, uint8_t len, uint8_t *buf)
{
    buf[0] = FRAME_START;
    buf[1] = type;
    buf[2] = len;
    memcpy(buf + 3, payload, len);
    buf[3 + len] = crc8(buf + 1, 2 + len);
    return 4 + len;
}

static void send_timesync(int fd, int64_t t1_us, int64_t t4_prev_us)
{
    uint8_t payload[16];
    put_le64(&payload[0], t1_us);
    put_le64(&payload[8], t4_prev_us);
    uint8_t frame[24];
    int n = encode_frame(MSG_TIMESYNC, payload, 16, frame);
    ssize_t _ = write(fd, frame, n); (void)_;
}

// ---------------------------------------------------------------------------
// Frame decoder — state machine, returns payload length on complete frame
// or -1. When complete, type_out/payload_out are filled; capture_t4 is set
// to the timestamp captured immediately at CRC-pass.
// ---------------------------------------------------------------------------

struct Decoder {
    enum class St { START, TYPE, LEN, PAYLOAD, CRC } st{St::START};
    uint8_t type{0}, len{0}, pos{0};
    uint8_t buf[PROTO_MAX_PAYLOAD + 4]{};
};

// Returns true when a complete valid frame is decoded. t4_out set at CRC-pass.
static bool feed_byte(Decoder &dec, uint8_t b, uint8_t &type_out,
                       uint8_t *payload_out, uint8_t &len_out, int64_t &t4_out)
{
    switch (dec.st) {
    case Decoder::St::START:
        if (b == FRAME_START) dec.st = Decoder::St::TYPE;
        break;
    case Decoder::St::TYPE:
        dec.type = b; dec.st = Decoder::St::LEN;
        break;
    case Decoder::St::LEN:
        dec.len = b; dec.pos = 0;
        dec.st = (b == 0) ? Decoder::St::CRC : Decoder::St::PAYLOAD;
        break;
    case Decoder::St::PAYLOAD:
        dec.buf[dec.pos++] = b;
        if (dec.pos == dec.len) dec.st = Decoder::St::CRC;
        break;
    case Decoder::St::CRC: {
        // Build [type, len, payload...] for CRC computation
        uint8_t hdr[2] = {dec.type, dec.len};
        uint8_t expected = crc8(hdr, 2);
        // CRC covers type + len + payload
        uint8_t crc_input[2 + PROTO_MAX_PAYLOAD];
        crc_input[0] = dec.type; crc_input[1] = dec.len;
        memcpy(crc_input + 2, dec.buf, dec.len);
        expected = crc8(crc_input, 2 + dec.len);

        dec.st = Decoder::St::START;
        if (b == expected) {
            t4_out = now_us();   // capture T4 immediately at CRC-pass
            type_out = dec.type;
            len_out  = dec.len;
            memcpy(payload_out, dec.buf, dec.len);
            return true;
        }
        break;
    }
    }
    return false;
}

// ---------------------------------------------------------------------------
// Read loop: wait for a specific message type with timeout_ms
// ---------------------------------------------------------------------------

static bool wait_for(int fd, Decoder &dec, uint8_t want_type,
                     uint8_t *payload_out, uint8_t &len_out,
                     int64_t &t4_out, int timeout_ms = 3000)
{
    struct timespec deadline;
    clock_gettime(CLOCK_MONOTONIC, &deadline);
    deadline.tv_sec  += timeout_ms / 1000;
    deadline.tv_nsec += static_cast<long>(timeout_ms % 1000) * 1000000L;
    if (deadline.tv_nsec >= 1000000000L) {
        deadline.tv_nsec -= 1000000000L;
        deadline.tv_sec++;
    }

    while (true) {
        // Check deadline
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        long remaining_ms = static_cast<long>(deadline.tv_sec - now.tv_sec) * 1000 +
                            (deadline.tv_nsec - now.tv_nsec) / 1000000L;
        if (remaining_ms <= 0) return false;

        fd_set rset;
        FD_ZERO(&rset);
        FD_SET(fd, &rset);
        struct timeval tv{ remaining_ms / 1000, (remaining_ms % 1000) * 1000L };
        int rc = select(fd + 1, &rset, nullptr, nullptr, &tv);
        if (rc <= 0) return false;

        uint8_t byte;
        ssize_t n = read(fd, &byte, 1);
        if (n != 1) continue;

        uint8_t type_out_tmp, len_tmp;
        uint8_t payload_tmp[PROTO_MAX_PAYLOAD];
        int64_t t4_tmp;
        if (feed_byte(dec, byte, type_out_tmp, payload_tmp, len_tmp, t4_tmp)) {
            if (type_out_tmp == want_type) {
                len_out  = len_tmp;
                t4_out   = t4_tmp;
                memcpy(payload_out, payload_tmp, len_tmp);
                return true;
            }
            // Ignore other frame types
        }
    }
}

// ---------------------------------------------------------------------------
// Decode helpers
// ---------------------------------------------------------------------------

static int64_t get_le64(const uint8_t *p)
{
    uint64_t lo = static_cast<uint32_t>(p[0]) |
                  (static_cast<uint32_t>(p[1]) << 8) |
                  (static_cast<uint32_t>(p[2]) << 16) |
                  (static_cast<uint32_t>(p[3]) << 24);
    uint64_t hi = static_cast<uint32_t>(p[4]) |
                  (static_cast<uint32_t>(p[5]) << 8) |
                  (static_cast<uint32_t>(p[6]) << 16) |
                  (static_cast<uint32_t>(p[7]) << 24);
    return static_cast<int64_t>(lo | (hi << 32));
}

// ---------------------------------------------------------------------------
// One timesync exchange: send MSG_TIMESYNC, wait for MSG_TIMESYNC_RESP.
// Returns raw offset or INT64_MIN on failure.
// t4_prev_us: Pi time of last received resp (0 on first).
// Updates t4_out with T4 captured at CRC-pass.
// ---------------------------------------------------------------------------

static int64_t do_exchange(int fd, Decoder &dec, int64_t t4_prev_us, int64_t &t4_out)
{
    int64_t t1 = now_us();
    send_timesync(fd, t1, t4_prev_us);

    uint8_t payload[PROTO_MAX_PAYLOAD];
    uint8_t len;
    int64_t t4;
    if (!wait_for(fd, dec, MSG_TIMESYNC_RESP, payload, len, t4, 3000)) {
        fprintf(stderr, "  [timeout waiting for MSG_TIMESYNC_RESP]\n");
        return INT64_MIN;
    }
    if (len < 8) {
        fprintf(stderr, "  [short MSG_TIMESYNC_RESP: len=%u]\n", len);
        return INT64_MIN;
    }

    int64_t t2 = get_le64(payload);
    t4_out = t4;
    int64_t raw = (t1 + t4) / 2 - t2;
    return raw;
}

// ---------------------------------------------------------------------------
// Benchmark one period: warmup then N samples
// ---------------------------------------------------------------------------

struct PeriodResult {
    double   period_s;
    int      samples;
    int64_t  avg_diff_us;
    int64_t  max_diff_us;
    int64_t  jitter_us;    // max - min of raw offsets
};

static PeriodResult benchmark(int fd, Decoder &dec, double period_s, int n_samples)
{
    printf("  period=%.1fs  ", period_s);
    fflush(stdout);

    int64_t t4_prev = 0;
    int64_t last_raw = INT64_MIN;
    std::vector<int64_t> raws;
    std::vector<int64_t> diffs;

    // 2-exchange warmup (let STM32 compute first valid offset)
    for (int i = 0; i < 2; i++) {
        int64_t t4_out;
        int64_t raw = do_exchange(fd, dec, t4_prev, t4_out);
        if (raw == INT64_MIN) { printf("WARMUP FAIL\n"); return {period_s, 0, 0, 0, 0}; }
        t4_prev = t4_out;
        putchar('w'); fflush(stdout);

        if (i < 1) {
            struct timespec ts{ static_cast<time_t>(period_s),
                                static_cast<long>((period_s - static_cast<time_t>(period_s)) * 1e9) };
            nanosleep(&ts, nullptr);
        }
    }

    // Measurement samples
    for (int i = 0; i < n_samples; i++) {
        struct timespec ts{ static_cast<time_t>(period_s),
                            static_cast<long>((period_s - static_cast<time_t>(period_s)) * 1e9) };
        nanosleep(&ts, nullptr);

        int64_t t4_out;
        int64_t raw = do_exchange(fd, dec, t4_prev, t4_out);
        if (raw == INT64_MIN) { putchar('?'); fflush(stdout); continue; }
        t4_prev = t4_out;

        raws.push_back(raw);
        if (last_raw != INT64_MIN) {
            int64_t diff = std::abs(raw - last_raw);
            diffs.push_back(diff);
        }
        last_raw = raw;
        putchar('.'); fflush(stdout);
    }
    putchar('\n');

    PeriodResult r{};
    r.period_s  = period_s;
    r.samples   = static_cast<int>(raws.size());

    if (!diffs.empty()) {
        r.avg_diff_us = std::accumulate(diffs.begin(), diffs.end(), 0LL) /
                        static_cast<int64_t>(diffs.size());
        r.max_diff_us = *std::max_element(diffs.begin(), diffs.end());
    }

    if (!raws.empty()) {
        int64_t mn = *std::min_element(raws.begin(), raws.end());
        int64_t mx = *std::max_element(raws.begin(), raws.end());
        r.jitter_us = mx - mn;
    }

    return r;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char *argv[])
{
    const char *port = "/dev/ttyYahboom0";
    std::vector<double> periods;

    int arg = 1;
    if (arg < argc && argv[arg][0] == '/') port = argv[arg++];
    while (arg < argc) {
        periods.push_back(std::stod(argv[arg++]));
    }
    if (periods.empty()) periods = {0.5, 1.0, 2.0, 5.0, 10.0};

    printf("Timesync benchmark: port=%s\n", port);
    printf("Periods: ");
    for (double p : periods) printf("%.1fs ", p);
    printf("\n");

    int fd = open_serial(port, B460800);
    if (fd < 0) return 1;

    // Flush any stale bytes
    tcflush(fd, TCIOFLUSH);
    usleep(100000);

    Decoder dec;
    static constexpr int N_SAMPLES = 20;

    std::vector<PeriodResult> results;
    for (double p : periods) {
        printf("\nBenchmarking %.1f s period (%d samples + 2 warmup):\n", p, N_SAMPLES);
        auto r = benchmark(fd, dec, p, N_SAMPLES);
        results.push_back(r);
    }

    close(fd);

    // Summary table
    printf("\n%-10s  %7s  %12s  %12s  %12s\n",
           "period(s)", "samples", "avg_diff(µs)", "max_diff(µs)", "jitter(µs)");
    printf("%-10s  %7s  %12s  %12s  %12s\n",
           "----------", "-------", "------------", "------------", "----------");
    for (const auto &r : results) {
        if (r.samples == 0) {
            printf("%-10.1f  %7s  %12s  %12s  %12s\n",
                   r.period_s, "FAIL", "—", "—", "—");
        } else {
            printf("%-10.1f  %7d  %12lld  %12lld  %12lld\n",
                   r.period_s, r.samples,
                   static_cast<long long>(r.avg_diff_us),
                   static_cast<long long>(r.max_diff_us),
                   static_cast<long long>(r.jitter_us));
        }
    }
    printf("\n");
    return 0;
}
