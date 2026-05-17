# Timesync Protocol: STM32 ↔ Pi Clock Synchronisation

**Status: implemented** (commits f20d527 → 2d88e99)

## Problem

All ROS2 messages (joint states, TF, odometry) were timestamped with the Pi's
receive time, not the STM32 measurement time. The QDEC poll at 20 Hz means
measurements are up to 50 ms stale by the time they are stamped.

## Goal

STM32 computes the Pi clock offset autonomously and embeds Pi-domain timestamps
directly in STREAM_JOINT frames. Pi sends `MSG_TIMESYNC` (with T4 piggybacked from
the previous round) and reads back frames already timestamped in Pi time. No
separate offset message, no Pi-side math.

---

## Protocol

### Exchange mechanics

```
Round N-1:  Pi sends [T1_prev, 0] ─────────────► STM32 captures T2_prev, responds [T2_prev]
            Pi records T4_prev at CRC-pass ◄───── response arrives

Round N:    Pi sends [T1, T4_prev] ────────────► STM32 computes:
                                                   offset = (T1_prev + T4_prev)/2 − T2_prev
                                                   fills median buffer, ts_offset_valid = true
                                                   future STREAM_JOINT carry pi_time_us
            Pi records T4 at CRC-pass ◄─────────  STM32 responds [T2]
```

Offset valid after the **second exchange** (≈ 2 s with the initial 8-cycle burst).

**T4 must be captured at CRC-pass** in the receiver thread/loop — not deferred to a
poll callback. Deferring to a 100 ms tkinter `after()` timer added ±1–2 ms of jitter.

### Message types

| ID     | Direction    | Name                | Payload                              |
|--------|--------------|---------------------|--------------------------------------|
| `0x84` | Pi → STM32   | `MSG_TIMESYNC`      | int64 T1 + int64 T4_prev LE (16 B)  |
| `0x05` | STM32 → Pi   | `MSG_TIMESYNC_RESP` | int64 T2 LE (8 B)                   |

**T1** = Pi `CLOCK_REALTIME` µs at the moment the MSG_TIMESYNC frame is written to serial.  
**T4_prev** = Pi `CLOCK_REALTIME` µs when the previous MSG_TIMESYNC_RESP CRC passed (0 on first).  
**T2** = STM32 `get_uptime_us()` captured at the first instruction of the MSG_TIMESYNC handler.

### STREAM_JOINT extension

Frame grew 14 → 22 bytes (< `PROTO_MAX_LEN = 32`):

```
[0:3]   int32 LE   pos_left     (deg × 100, i.e. 0.01 deg LSB)
[4:7]   int32 LE   pos_right    (deg)
[8]     uint8      steer        (0-100)
[9]     uint8      seq
[10:11] int16 LE   vel_left     (deg/s)
[12:13] int16 LE   vel_right    (deg/s)
[14:21] int64 LE   pi_time_us   (Pi CLOCK_REALTIME µs; 0 while offset not yet valid)
```

STM32 computes: `pi_time_us = get_uptime_us() + ts_offset_us`

---

## STM32 implementation (`yahboom/src/comms.c`)

### High-precision clock

```c
static int64_t get_uptime_us(void) {
    return (int64_t)(k_cycle_get_64() / 72u);
}
```

`CONFIG_TIMER_HAS_64BIT_CYCLE_COUNTER=y` — 72 MHz counter, ~1 µs resolution,
wraps in ~256 years. No race condition, no tick arithmetic.

### Offset formula (corrected from original plan)

```c
int64_t raw = (ts_t1_prev + t4_prev) / 2 - ts_t2_prev;
```

Applied as `pi_time = stm32_us + offset`. The sign was inverted in the original
plan (`T2 − midpoint`), which produced a negative offset ≈ −1.75 × 10⁹ µs and was
masked by zero-contamination of the median buffer (see Bugs section).

### Rolling median with zero-contamination fix

8-slot ring buffer, insertion sort, returns `tmp[4]` (upper median of 8).

On the **first** valid sample the entire buffer is pre-filled with that sample's
value before entering the ring. Without this, the 7 uninitialised zeros dominated
the sorted array and the median returned 0 — masking the sign bug above.

### State variables

```c
static uint32_t ts_count;
static bool     ts_offset_valid;
static int64_t  ts_offset_us;
static int64_t  ts_t1_prev;
static int64_t  ts_t2_prev;
static int64_t  ts_last_rx_ms;
static int64_t  ts_offsets[8];
static int      ts_hist_idx;
/* For diff tracking */
static bool     ts_last_raw_valid;
static int64_t  ts_last_raw;
static int64_t  ts_diffs[8];      /* rolling window of |raw[n] − raw[n-1]| */
static int      ts_diff_idx;
static int      ts_diff_count;
```

### Shell command `ts`

```
uart:~$ ts status
ts: count=42  valid=yes  offset=1748123456789 us  jitter=312 us  avg_diff=58 us  age=523 ms

uart:~$ ts status   (before second exchange)
ts: count=1  valid=no  offset=—

uart:~$ ts clear
```

- **jitter**: `max(ts_offsets) − min(ts_offsets)` across the 8-slot buffer — measurement noise.
- **avg_diff**: mean of `|raw[n] − raw[n-1]|` over the last 8 exchanges — round-trip stability.
- **age**: `k_uptime_get() − ts_last_rx_ms` — ms since last exchange.

---

## Pi implementation

### ROS2 hardware interface (`picar2_control`)

Send schedule in `write()`: unconditional for the first 8 cycles, then every 50th
cycle (≈ 1 s at 20 Hz control loop).

`dispatch_timesync_resp()` records T4 immediately when called from `process_byte()`
at CRC-pass — not deferred.

`dispatch_joint_frame()` reads `pi_time_us` when `len >= 22` and stores it as
`last_corrected_stamp_` for use in joint-state and odometry messages.

### GUI (`tools/picar2_gui.py`)

Timesync panel in the stats section:

- Row 1: count / valid (green/red) / jitter µs / age ms / period selector (0.5 / 1 / 2 / 5 / 10 s)
- Row 2: Δ last µs / Δ avg µs / Δ max µs

T4 captured at CRC-pass inside `Receiver._feed()` — stored as `rx_time_us` in the
decoded dict before the frame is queued to the main thread.

Send schedule: unconditional for first 8 poll cycles, then every `period_s / 0.1`
cycles (poll runs at 100 ms).

### Python integration test (`tests/integration/test_protocol.py`)

- `test_timesync()`: two warmup exchanges, then 8 measurement rounds.
  Verifies `pi_time_us != 0` and within 1 s of host time after the second exchange.
  Verifies drift < 500 µs between consecutive STREAM_JOINT frames.

### C++ benchmark (`tests/integration/timesync_test.cpp`)

Standalone binary; no Python overhead, no GIL.

- Opens serial port at 460800 baud with POSIX termios.
- Uses `clock_gettime(CLOCK_REALTIME)` for µs timestamps.
- Captures T4 at CRC-pass inside the byte-read loop.
- For each period: 2 warmup exchanges + 20 timed measurement exchanges.
- Prints summary table: avg_diff / max_diff / jitter (all µs).

Build and run:
```bash
make test-timesync                               # default port, all 5 periods
make test-timesync TIMESYNC_PORT=/dev/ttyUSB0   # override port
make build-timesync                              # compile only
./tests/integration/timesync_test /dev/ttyYahboom0 0.5 1
```

---

## Benchmark results (2026-05-16, Pi 5, systemd-timesyncd +15.8 ppm)

| Period (s) | avg_diff (µs) | max_diff (µs) | jitter (µs) |
|------------|---------------|---------------|-------------|
| **0.5**    | **58**        | **154**       | 846         |
| **1.0**    | **94**        | **169**       | 1761        |
| 2.0        | 1547          | 2044          | 29394       |
| 5.0        | 1342          | 2349          | 25509       |
| 10.0       | 598           | 1050          | 11380       |

**Recommended period: 0.5–1.0 s.**

At ≥ 2 s the avg_diff jumps to ~1.5 ms. Two causes:

1. **STM32 HSI drift** — HSI is ±1 % spec but typically 10–100 ppm in practice;
   at 5 s intervals that is 50–500 µs of accumulated error between syncs.

2. **systemd-timesyncd step corrections** — unlike chrony, timesyncd applies clock
   adjustments as discrete steps (here offset = −3.6 ms, poll = 8.5 min). A step
   landing during a long sync interval shows up as a sudden jump in raw offset.

At 0.5 s the accumulated drift between corrections is ≤ 8 µs, well below the 58 µs
UART latency noise floor.

---

## Bugs found during implementation

### 1. Offset sign inversion

**Symptom**: `test_timesync` reported `pi_time_us` delta of 1.78 × 10⁹ s from host time.  
**Cause**: formula was `T2 − (T1 + T4)/2` (negative ≈ −1.75 × 10⁹ µs).  
**Fix**: `(T1 + T4)/2 − T2`.

### 2. Zero-contamination of median buffer

**Symptom**: drift check passed despite wrong sign (offset effectively 0).  
**Cause**: with 1 valid sample at −1.75 × 10⁹ and 7 zeros, sorted median is 0.
STM32 emitted `pi_time_us ≈ uptime` (≈ seconds since boot) instead of Pi epoch time.  
**Fix**: pre-fill all 8 slots with the first valid sample.

Both bugs masked each other: the wrong-sign offset was zeroed by the contaminated
median, so pi_time_us was roughly equal to STM32 uptime, making drift between
consecutive frames appear stable.

### 3. T4 recorded in poll loop (Python GUI)

**Symptom**: avg_diff ≈ 2 ms even at 0.5 s sync period.  
**Cause**: `_on_timesync_resp()` called `time.time()` but ran in the tkinter 100 ms
`after()` callback; T4 was 0–100 ms stale and `after()` timer jitter of ±1–2 ms
appeared as offset diff.  
**Fix**: capture `rx_time_us = int(time.time() * 1e6)` in `Receiver._feed()` at
CRC-pass and pass it through the decoded dict.  
**Result**: avg_diff reduced from ~2 ms to ~0.6 ms.
