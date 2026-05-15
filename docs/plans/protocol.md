# Plan: STM32↔Pi Custom Binary Protocol

## Context

The PICAR-2 needs a custom binary protocol over USART1 between the Yahboom STM32F103 and
a Raspberry Pi running ros2_control (Jazzy). micro-ROS was ruled out (flash/RAM too
constrained on the F103). The protocol carries joint state, IMU, and battery data upstream
(STM32→Pi), and velocity commands, stream requests, and rate-configuration commands
downstream (Pi→STM32). CMD_VEL doubles as the keepalive; its absence triggers RC fallback.

## Key Findings from Codebase

- **USART1** (`DT_NODELABEL(usart1)`) is already enabled at 921600 baud in the board DTS
  (`boards/arm/yahboom_ros_stm32f103/yahboom_ros_stm32f103.dts:96-101`). No overlay change needed.
- **`motor_pos(id, *pos)`** (`motor.h:19`) returns int32_t encoder ticks — used directly as `enc_left/right`.
- **`servo_get(*pval)`** (`servo.h:12`) returns current 0–100 steering value — used for JOINT_STATE echo.
- **`battery_read(*value)`** (`battery.c:49`) returns mV directly. `VBAT_MIN_MV`/`VBAT_MAX_MV` are
  local to `battery.c` — need `battery_level()` exposed so `comms.c` doesn't duplicate constants.
- **`imu_sample()`** (`imu.h:13`) only calls `sensor_sample_fetch()`; `sensor_channel_get()` needs
  the private `imu` device ptr. Need a new `imu_get_data()` function that fetches, converts to
  fixed-point, and applies mag calibration.
- **`rc_disable()`** (`rc.h:11`) is the only RC control exposed. Need `rc_set_enable(int en)` for
  comms to re-enable RC on connection loss.
- **Draft plan IMU payload bug**: accel[3] + gyro[3] + magn[3] + temp = 6+6+6+2 = **20 bytes**
  (not 16 as stated). Corrected here; frame total = 24 bytes.

## Frame Format

```
[0xAA][TYPE:u8][LEN:u8][PAYLOAD: LEN bytes][CRC8:u8]    total = 3 + LEN + 1
```

- CRC8 Dallas/Maxim (poly 0x31), covers TYPE+LEN+PAYLOAD
- Little-endian integers

## Message Definitions

### STM32 → Pi

All streams respond to `REQ` and also support periodic output when `SET_RATE > 0`.
Default rate: **0 Hz** (silent at startup).

| Type | Name        | Payload fields                                                        | LEN |
|------|-------------|-----------------------------------------------------------------------|-----|
| 0x01 | JOINT_STATE | enc_left:i32, enc_right:i32, steering:u8, seq:u8                     | 10  |
| 0x02 | IMU         | accel[3]:i16(×0.001 m/s²), gyro[3]:i16(×0.001 rad/s), magn[3]:i16(×0.1µT), temp:i16(×0.01°C) | 20 |
| 0x03 | BATTERY     | voltage_mv:u16, charge_pct:u8, _pad:u8                               | 4   |

### Pi → STM32

| Type | Name     | Payload fields                               | LEN |
|------|----------|----------------------------------------------|-----|
| 0x80 | CMD_VEL  | left_mms:i16, right_mms:i16, steering:u8     | 5   |
| 0x81 | REQ      | stream_id:u8                                 | 1   |
| 0x82 | SET_RATE | stream_id:u8, rate_hz:u16                    | 3   |

- **CMD_VEL** is sent at 100 Hz continuously (zero velocity when stopped).
- **REQ** triggers one immediate response frame for the given `stream_id` (any stream).
- **SET_RATE** configures the periodic rate for any stream. `rate_hz` is u16 (Hz); 0 disables.
  **Default for all streams: 0 Hz** (silent at startup).
- Any valid frame from the Pi resets the 500 ms watchdog. 500 ms of silence → DISCONNECTED.

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│  comms.c (high-level)                                               │
│                                                                     │
│  tmr[1] (rate configurable via SET_RATE) ──► wrk[1] ──► send_js() ►│
│  tmr[2] (rate configurable via SET_RATE) ──► wrk[2] ──► send_imu()►│─► proto_encode()
│  tmr[3] (rate configurable via SET_RATE) ──► wrk[3] ──► send_bat()►│   ─► uart_poll_out()
│    (all timers stopped at boot; rates set by SET_RATE 0x82)        │
│                                                                     │
│  comms_rx(type, payload)    ← ANY valid frame resets watchdog       │
│     0x80 CMD_VEL ──► motor_speed() + servo_steer()                  │
│     0x81 REQ(id) ──► call send_stream(id) immediately               │
│     0x82 SET_RATE──► hz==0: k_timer_stop(tmr[id])                   │
│                  ──► hz>0: k_timer_start(tmr[id], K_MSEC(1000/hz)) │
│                                                                     │
│  k_work_delayable watchdog (500ms) ──► DISCONNECTED                 │
│                                        rc_set_enable(1)             │
└──────────────┬──────────────────────────────────────────────────────┘
               │ proto_decode_feed() on each byte
┌──────────────▼──────────────────────────────────────────┐
│  protocol.c (framing layer)                             │
│                                                         │
│  UART RX ISR ──► ring buffer ──► k_work ──► state machine
│                                             complete frame ──► callback
└─────────────────────────────────────────────────────────┘
```

## Connection State Machine

```
DISCONNECTED ──(any valid frame)──► CONNECTED   [rc_set_enable(0)]
CONNECTED    ──(500 ms silence)──► DISCONNECTED [rc_set_enable(1)]
```

Initial state: DISCONNECTED (RC active).
Any valid received frame (CMD_VEL, REQ, SET_RATE) resets the 500 ms watchdog and, if
currently DISCONNECTED, transitions to CONNECTED.

## Bandwidth Check (corrected)

At 921600 baud (≈92 KB/s):

At startup all streams are silent (0 Hz). The Pi must call SET_RATE or REQ to receive data.

At maximum configured rates (IMU 200 Hz, JOINT_STATE 100 Hz, BATTERY 10 Hz):

| Stream           | Bytes/frame | Hz  | KB/s |
|------------------|-------------|-----|------|
| JOINT_STATE (↑)  | 14          | 100 | 1.4  |
| IMU (↑)          | 24          | 200 | 4.8  |
| BATTERY (↑)      | 8           | 10  | ~0.1 |
| CMD_VEL (↓)      | 9           | 100 | 0.9  |
| REQ/SET_RATE (↓) | 8/9         | <1  | ~0   |
| **Total max**    |             |     | **7.2** |

~8% utilisation at max — well within 92 KB/s budget.

---

## Files to Create

### `yahboom/src/protocol.h`

```c
#define PROTO_START   0xAA
#define PROTO_MAX_LEN 32

typedef void (*proto_rx_cb_t)(uint8_t type, const uint8_t *payload, uint8_t len);

void proto_init(const struct device *uart, proto_rx_cb_t cb);
int  proto_encode(uint8_t type, const uint8_t *payload, uint8_t len, uint8_t *out);
/* returns total frame length */
```

### `yahboom/src/protocol.c`

- `crc8(buf, len)` — Dallas/Maxim poly 0x31
- `proto_encode()` — fills header + payload + CRC into caller-provided buffer
- `proto_init()` — opens USART1 via `uart_irq_callback_user_data_set()` + `uart_irq_rx_enable()`
- UART ISR: reads bytes with `uart_fifo_read()`, pushes to `ring_buf` (Zephyr `ring_buf.h`)
- `k_work` dispatched from ISR: drains ring buf byte-by-byte through receive state machine
  (states: WAIT_START → GOT_TYPE → GOT_LEN → PAYLOAD → CHECK_CRC → dispatch)
  → calls `proto_rx_cb_t` on valid complete frame; silently drops bad CRC

### `yahboom/src/comms.h`

```c
void comms_init(void);
```

### `yahboom/src/comms.c`

```c
/* One timer+work per stream, indexed by stream_id (0x01..0x03) */
static struct k_timer tmr[4];   /* [1]=JOINT_STATE [2]=IMU [3]=BATTERY */
static struct k_work  wrk[4];

K_WORK_DELAYABLE_DEFINE(watchdog_work, on_timeout);

static uint8_t js_seq;
static bool connected;
```

All three timers are stopped at boot (`comms_init()` does not start them).
`SET_RATE` restarts or stops the appropriate timer; `REQ` calls `send_*(id)` inline.

- `comms_init()`: calls `proto_init(DEVICE_DT_GET(DT_NODELABEL(usart1)), comms_rx)`,
  initialises `tmr[]` and `wrk[]` arrays, leaves all timers stopped
- `send_stream(id)`: dispatch table `{send_joint_state, send_imu, send_battery}` keyed by id
- `send_joint_state()`: calls `motor_pos(MOTOR_L/R)`, `servo_get()`, encodes 0x01
- `send_imu()`: calls `imu_get_data()`, encodes 0x02
- `send_battery()`: calls `battery_read()` + `battery_level()`, encodes 0x03
- `comms_rx(type, payload, len)`: first always reschedules watchdog + sets CONNECTED + `rc_set_enable(0)`, then:
  - 0x80 CMD_VEL → `motor_speed()` + `servo_steer()`
  - 0x81 REQ(id) → `send_stream(id)` immediately
  - 0x82 SET_RATE(id, hz) → hz==0: `k_timer_stop(&tmr[id])`; hz>0: `k_timer_start(&tmr[id], K_MSEC(1000/hz), K_MSEC(1000/hz))`
- `on_timeout()`: set disconnected, `rc_set_enable(1)`

---

## Files to Modify

### `yahboom/src/imu.h` — add data struct and getter

```c
struct imu_data {
    int16_t accel[3];   /* ×0.001 m/s²  */
    int16_t gyro[3];    /* ×0.001 rad/s */
    int16_t magn[3];    /* ×0.1 µT      */
    int16_t temp;       /* ×0.01 °C     */
};
int imu_get_data(struct imu_data *d);
```

### `yahboom/src/imu.c` — implement `imu_get_data()`

Add after `imu_sample()`:

```c
int imu_get_data(struct imu_data *d) {
    struct sensor_value a[3], g[3], m[3], t;
    int ret = sensor_sample_fetch(imu);
    if (ret) return ret;
    sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, a);
    sensor_channel_get(imu, SENSOR_CHAN_GYRO_XYZ,  g);
    sensor_channel_get(imu, SENSOR_CHAN_MAGN_XYZ,  m);
    sensor_channel_get(imu, SENSOR_CHAN_DIE_TEMP,  &t);
    apply_mag_cal(m);
    for (int i = 0; i < 3; i++) {
        d->accel[i] = (int16_t)(a[i].val1 * 1000 + a[i].val2 / 1000);
        d->gyro[i]  = (int16_t)(g[i].val1 * 1000 + g[i].val2 / 1000);
        d->magn[i]  = (int16_t)(m[i].val1 * 10   + m[i].val2 / 100000);
    }
    d->temp = (int16_t)(t.val1 * 100 + t.val2 / 10000);
    return 0;
}
```

Note: `apply_mag_cal()` is already `static` in `imu.c` — `imu_get_data()` goes in the same file, so it can call it directly.

### `yahboom/src/rc.h` — add `rc_set_enable()`

```c
void rc_set_enable(int en);
```

### `yahboom/src/rc.c` — implement `rc_set_enable()`

```c
void rc_set_enable(int en) { rc_enable = en; }
```

`rc_enable` is a word-aligned `int`; ARM single-word writes are atomic, no mutex needed.

### `yahboom/src/battery.h` — add `battery_level()`

```c
int battery_level(uint8_t *pct);
```

### `yahboom/src/battery.c` — implement `battery_level()`

```c
int battery_level(uint8_t *pct) {
    int32_t mv;
    int err = battery_read(&mv);
    if (err) return err;
    *pct = (uint8_t)CLAMP((mv - VBAT_MIN_MV) * 100 / (VBAT_MAX_MV - VBAT_MIN_MV), 0, 100);
    return 0;
}
```

### `yahboom/src/main.c`

Add `#include "comms.h"` near existing includes, and add `comms_init();` at the end of `main()` after `rc_init()`.

### `yahboom/prj.conf`

Add:
```
CONFIG_UART_INTERRUPT_DRIVEN=y
CONFIG_RING_BUFFER=y
```

(`CONFIG_SERIAL=y` is already implied by the existing UART shell and SBUS use.)

### `yahboom/CMakeLists.txt`

Append `src/protocol.c src/comms.c` to the `target_sources` list on line 17.

---

## Implementation Order

1. `rc.h/rc.c` — add `rc_set_enable()` (trivial, unblocks comms.c compile)
2. `battery.h/battery.c` — add `battery_level()`
3. `imu.h/imu.c` — add `struct imu_data` + `imu_get_data()`
4. `prj.conf` — add config knobs
5. `CMakeLists.txt` — add new source files
6. `protocol.h/protocol.c` — framing layer (no app deps, easiest to test in isolation)
7. `comms.h/comms.c` — wires everything together
8. `main.c` — add `comms_init()` call

---

## Verification

1. `make` in the yahboom directory — confirm zero new errors/warnings
2. Flash and open `picocom -b 921600 /dev/ttyYahboom1` — Zephyr shell still responds on USART3
3. Python test script on Pi (`/dev/ttyYahboom0`, 921600 baud):
   - Send CMD_VEL (zero velocity) at 100 Hz → STM32 enters CONNECTED; RC should no longer respond
   - Send REQ(0x01) → receive one JOINT_STATE frame; spin wheels by hand, send again, verify ticks changed
   - Verify no periodic frames arrive at startup (all streams at 0 Hz by default)
   - Send SET_RATE(0x02, 10) → verify IMU frames arrive at ~10 Hz
   - Send REQ(0x02) → verify one immediate IMU frame is returned (even while periodic is running)
   - Send SET_RATE(0x02, 0) → verify IMU frames stop
   - Send SET_RATE(0x01, 100) + SET_RATE(0x03, 1) → verify JOINT_STATE at 100 Hz and BATTERY at 1 Hz
   - Send CMD_VEL (non-zero) → verify motors and servo respond
   - Stop sending CMD_VEL → after 500 ms STM32 returns to DISCONNECTED; RC resumes control
   - Send malformed frame (bad CRC) → verify STM32 does not crash or stall
