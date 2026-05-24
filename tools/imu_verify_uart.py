#!/usr/bin/env python3
"""
IMU sensor verification — Layer 2 (UART level, no ROS).

Tests accel, gyro, and magnetometer at the raw wire level.
No ROS required. Run on the Pi while bringup is NOT active.

Layer 0 pre-check (manual, one time):
    picocom -b 921600 /dev/ttyYahboom1
    uart:~$ imu whoami   →  must print 0x71

Wire frame: [0xAA][type][len][payload...][crc8(type+len+payload)]
STREAM_IMU (0x02) payload — 10 × LE int16 (chip axes: X=right, Y=forward, Z=up):
  [0] accel_x ×0.001 m/s²   [1] accel_y   [2] accel_z
  [3] gyro_x  ×0.001 rad/s  [4] gyro_y    [5] gyro_z
  [6] mag_x   ×0.1 µT       [7] mag_y     [8] mag_z
  [9] temp    ×0.01 °C

Usage:
    python3 tools/imu_verify_uart.py                        # all tests
    python3 tools/imu_verify_uart.py --tests A              # static only
    python3 tools/imu_verify_uart.py --tests A B            # static + gyro sign
    python3 tools/imu_verify_uart.py --tests B C D          # motion tests only
    python3 tools/imu_verify_uart.py --tests E              # mag orientation only (manual)
    python3 tools/imu_verify_uart.py --port /dev/ttyYahboom0 --tests A B C D E
    python3 tools/imu_verify_uart.py --nomag                # skip mag tests D and E

Note on magnetometer: the PCB (YB-ERF01-V1.0) was designed for ICM20948 but ships
with MPU9250; chassis hard-iron (~200 µT, ~7× Earth's field) makes mag heading
unreliable. Use --nomag to skip tests D and E when the environment is unknown.
"""

import argparse
import math
import struct
import sys
import time

import serial

# ── Protocol constants ────────────────────────────────────────────────────────

PROTO_START   = 0xAA
PROTO_MAX_LEN = 32
MSG_CMD_VEL   = 0x80
MSG_SET_RATE  = 0x82
STREAM_IMU    = 0x02

ACCEL_SCALE = 0.001   # raw int16 → m/s²
GYRO_SCALE  = 0.001   # raw int16 → rad/s
MAG_SCALE   = 0.1     # raw int16 → µT
TEMP_SCALE  = 0.01    # raw int16 → °C

# ── Motor speeds (dps) and steering ──────────────────────────────────────────

ARC_FAST_DPS    = 500   # B: faster (outer) wheel for gyro-sign arc
ARC_SLOW_DPS    = 200   # B: slower (inner) wheel — differential creates yaw rate
ARC_STEER_US    = 400   # B: steer delta µs (positive = CW/right turn)
DRIVE_DPS       = 500   # C: forward drive for accel axis test
CIRCLE_FAST_DPS = 300   # D: faster wheel for mag circle arc
CIRCLE_SLOW_DPS = 150   # D: slower wheel for mag circle arc
CIRCLE_STEER_US = 300   # D: steer delta µs for mag circle (positive = right turn,
                         #    matches wheel differential; negate if arc curves wrong way)

# ── Protocol helpers ──────────────────────────────────────────────────────────

def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def make_frame(msg_type: int, payload: bytes = b'') -> bytes:
    hdr = bytes([PROTO_START, msg_type, len(payload)])
    return hdr + payload + bytes([crc8(bytes([msg_type, len(payload)]) + payload)])


def set_rate(ser: serial.Serial, stream_id: int, hz: int) -> None:
    ser.write(make_frame(MSG_SET_RATE, struct.pack('<BH', stream_id, hz)))
    ser.flush()


def cmd_vel(ser: serial.Serial, l_dps: int, r_dps: int, steer_us: int = 0) -> None:
    ser.write(make_frame(MSG_CMD_VEL, struct.pack('<hhh', l_dps, r_dps, steer_us)))
    ser.flush()


# ── Frame parser (mirrors firmware state machine) ─────────────────────────────

class FrameParser:
    """Incremental parser — mirrors the S_START/TYPE/LEN/PAYLOAD/CRC state machine
    in yahboom/src/protocol.c so resync behaviour is identical."""

    def __init__(self):
        self._state   = 'START'
        self._mtype   = 0
        self._mlen    = 0
        self._payload = bytearray()

    def feed(self, data: bytes):
        """Yield (msg_type, payload_bytes) for each valid frame found in data."""
        for b in data:
            if self._state == 'START':
                if b == PROTO_START:
                    self._state = 'TYPE'
            elif self._state == 'TYPE':
                self._mtype = b
                self._state = 'LEN'
            elif self._state == 'LEN':
                if b > PROTO_MAX_LEN:
                    self._state = 'START'
                    continue
                self._mlen    = b
                self._payload = bytearray()
                self._state   = 'PAYLOAD' if b > 0 else 'CRC'
            elif self._state == 'PAYLOAD':
                self._payload.append(b)
                if len(self._payload) == self._mlen:
                    self._state = 'CRC'
            elif self._state == 'CRC':
                expected = crc8(bytes([self._mtype, self._mlen]) + bytes(self._payload))
                if b == expected:
                    yield self._mtype, bytes(self._payload)
                self._state = 'START'


# ── IMU decoding ──────────────────────────────────────────────────────────────

def decode_imu(payload: bytes):
    """Return (accel_xyz, gyro_xyz, mag_xyz, temp) in SI / µT / °C."""
    v     = struct.unpack_from('<10h', payload)
    accel = [v[i] * ACCEL_SCALE for i in range(3)]
    gyro  = [v[i] * GYRO_SCALE  for i in range(3, 6)]
    mag   = [v[i] * MAG_SCALE   for i in range(6, 9)]
    temp  = v[9] * TEMP_SCALE
    return accel, gyro, mag, temp


# ── Collection helpers ────────────────────────────────────────────────────────

def collect_imu(ser: serial.Serial, parser: FrameParser,
                duration_s: float, motor_cmd=None) -> list:
    """
    Read STREAM_IMU frames for duration_s seconds.
    motor_cmd=(l_dps, r_dps, steer_us): resend every 200 ms to keep STM32
    watchdog alive (watchdog = 500 ms reference_timeout).
    Returns list of (timestamp, payload_bytes).
    """
    frames     = []
    t0         = time.monotonic()
    last_cmd_t = -1.0

    if motor_cmd is not None:
        cmd_vel(ser, *motor_cmd)
        last_cmd_t = t0

    while time.monotonic() - t0 < duration_s:
        now = time.monotonic()
        if motor_cmd is not None and now - last_cmd_t >= 0.2:
            cmd_vel(ser, *motor_cmd)
            last_cmd_t = now

        chunk = ser.read(64)
        if chunk:
            for mtype, payload in parser.feed(chunk):
                if mtype == STREAM_IMU and len(payload) >= 20:
                    frames.append((time.monotonic(), payload))

    return frames


# ── Calibration helpers ───────────────────────────────────────────────────────

def load_cal(path: str) -> dict:
    try:
        import yaml
        with open(path) as f:
            return yaml.safe_load(f) or {}
    except FileNotFoundError:
        sys.exit(f'Calibration file not found: {path}')
    except Exception as e:
        sys.exit(f'Failed to load calibration file: {e}')


def apply_cal(accel: list, gyro: list, mag: list, cal: dict):
    """Apply loaded calibration to a decoded IMU sample."""
    if 'gyro' in cal:
        b = cal['gyro']['bias']
        gyro = [gyro[i] - b[i] for i in range(3)]
    if 'accel' in cal:
        b = cal['accel']['bias']
        accel = [accel[i] - b[i] for i in range(3)]
    if 'mag' in cal:
        hi = cal['mag']['hard_iron']
        si = cal['mag']['soft_iron']
        mag = [(mag[i] - hi[i]) * si[i] for i in range(3)]
    return accel, gyro, mag


def decode_imu_cal(payload: bytes, cal: dict):
    """Decode IMU payload and apply calibration (if any)."""
    accel, gyro, mag, temp = decode_imu(payload)
    if cal:
        accel, gyro, mag = apply_cal(accel, gyro, mag, cal)
    return accel, gyro, mag, temp


# ── Statistics helpers ────────────────────────────────────────────────────────

def mean(xs): return sum(xs) / len(xs)

def std(xs):
    if len(xs) < 2:
        return 0.0
    m = mean(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


# ── Pass / fail output ────────────────────────────────────────────────────────

_G, _R, _Y, _RST = '\033[32m', '\033[31m', '\033[33m', '\033[0m'
_all_passed = True


def _tag(ok: bool) -> str:
    return f'{_G}PASS{_RST}' if ok else f'{_R}FAIL{_RST}'


def _record(ok: bool, name: str, got: str, expected: str, note: str = '') -> bool:
    global _all_passed
    suffix = f'  ← {note}' if (not ok and note) else ''
    print(f'  [{_tag(ok)}] {name}: {got}  (expected {expected}){suffix}')
    if not ok:
        _all_passed = False
    return ok


def check(name, val, lo, hi, fmt='.4f', unit='', note=''):
    return _record(lo <= val <= hi, name,
                   f'{val:{fmt}}{unit}', f'{lo} – {hi}{unit}', note)


def check_lt(name, val, limit, fmt='.4f', unit='', note=''):
    return _record(val < limit, name,
                   f'{val:{fmt}}{unit}', f'< {limit}{unit}', note)


def check_gt(name, val, limit, fmt='.4f', unit='', note=''):
    return _record(val > limit, name,
                   f'{val:{fmt}}{unit}', f'> {limit}{unit}', note)


def check_warn(name, val, lo, hi, fmt='.4f', unit='', note='needs calibration'):
    """Range check that emits WARN (not FAIL) when out of range.
    Use for values fixable by calibration rather than code changes."""
    ok = lo <= val <= hi
    tag = f'{_G}PASS{_RST}' if ok else f'{_Y}WARN{_RST}'
    suffix = '' if ok else f'  ← {note}'
    print(f'  [{tag}] {name}: {val:{fmt}}{unit}  (expected {lo} – {hi}{unit}){suffix}')
    return ok


def warn(msg: str):
    print(f'  [{_Y}WARN{_RST}] {msg}')


# ── Test A — static ───────────────────────────────────────────────────────────

def test_static(ser: serial.Serial, parser: FrameParser, hz: int, cal: dict) -> bool:
    print(f'\n── A. Static checks (10 s @ {hz} Hz) ───────────────────────────────')

    set_rate(ser, STREAM_IMU, hz)
    print(f'  Collecting ~{hz * 10} frames…', end='', flush=True)
    raw = collect_imu(ser, parser, duration_s=12.0)
    set_rate(ser, STREAM_IMU, 0)
    print(f' got {len(raw)}')

    if len(raw) < 50:
        global _all_passed
        _all_passed = False
        print(f'  [{_R}FAIL{_RST}] only {len(raw)} frames — check UART connection')
        return False

    data = [decode_imu_cal(p, cal) for _, p in raw]
    ts   = [t for t, _ in raw]

    ax = [d[0][0] for d in data];  ay = [d[0][1] for d in data];  az = [d[0][2] for d in data]
    gx = [d[1][0] for d in data];  gy = [d[1][1] for d in data];  gz = [d[1][2] for d in data]
    mx = [d[2][0] for d in data];  my = [d[2][1] for d in data];  mz = [d[2][2] for d in data]
    temps = [d[3] for d in data]
    amag  = [math.sqrt(ax[i]**2 + ay[i]**2 + az[i]**2) for i in range(len(data))]
    mmag  = [math.sqrt(mx[i]**2 + my[i]**2 + mz[i]**2) for i in range(len(data))]

    if len(ts) >= 2:
        rate = (len(ts) - 1) / (ts[-1] - ts[0])
        check('frame rate', rate, hz * 0.9, hz * 1.1, '.1f', ' Hz')

    check_warn('accel.z mean  (chip Z = up)',    mean(az),    9.50,  10.10, unit=' m/s²',
               note='calibrate: imu_mount_roll/pitch in URDF or ROS imu_tools')
    check_warn('accel.x mean  (chip X = right)', mean(ax),  -0.15,   0.15, unit=' m/s²',
               note='calibrate: imu_mount_roll/pitch in URDF')
    check_warn('accel.y mean  (chip Y = fwd)',   mean(ay),  -0.15,   0.15, unit=' m/s²',
               note='calibrate: imu_mount_roll/pitch in URDF')
    check_warn('accel magnitude',                mean(amag), 9.51,  10.11, unit=' m/s²',
               note='calibrate: accel scale via ROS imu_tools')

    check_warn('gyro.x mean',  mean(gx), -0.020,  0.020, unit=' rad/s',
               note='calibrate: `imu cal gyro` on STM32 shell')
    check_warn('gyro.y mean',  mean(gy), -0.020,  0.020, unit=' rad/s',
               note='calibrate: `imu cal gyro` on STM32 shell')
    check_warn('gyro.z mean',  mean(gz), -0.020,  0.020, unit=' rad/s',
               note='calibrate: `imu cal gyro` on STM32 shell')
    check_lt  ('gyro.x std',   std(gx),   0.010,          unit=' rad/s')
    check_lt  ('gyro.y std',   std(gy),   0.010,          unit=' rad/s')
    check_lt  ('gyro.z std',   std(gz),   0.010,          unit=' rad/s')

    if len(ts) >= 2:
        total_t      = ts[-1] - ts[0]
        drift_thresh = 2.0  if cal.get('gyro') else 15.0
        drift_note   = ('gyro bias calibration may need updating'
                        if cal.get('gyro') else
                        'calibrate: python3 tools/imu_calibrate.py --cal gyro')
        for label, gvals in [('gyro.x', gx), ('gyro.y', gy), ('gyro.z', gz)]:
            integral_rad = sum((gvals[i] + gvals[i + 1]) * 0.5 * (ts[i + 1] - ts[i])
                               for i in range(len(gvals) - 1))
            check_warn(f'{label} drift {total_t:.0f} s',
                       math.degrees(abs(integral_rad)), 0.0, drift_thresh,
                       '.2f', '°', note=drift_note)

    check_warn('mag magnitude',      mean(mmag), 20.0,   65.0,  unit=' µT',
               note='calibrate: hard-iron distortion — use ROS imu_tools magnetometer_calibration')
    _MAG_NOISE_NOTE = 'AC/fluorescent interference or motor coupling — move away from electrical sources'
    check_lt('mag.x std (noise)',    std(mx),     3.0,  unit=' µT', note=_MAG_NOISE_NOTE)
    check_lt('mag.y std (noise)',    std(my),     3.0,  unit=' µT', note=_MAG_NOISE_NOTE)
    check_lt('mag.z std (noise)',    std(mz),     3.0,  unit=' µT', note=_MAG_NOISE_NOTE)

    check   ('temperature',          mean(temps), 10.0,  50.0,  unit=' °C')
    return True


# ── Test B — gyro sign ────────────────────────────────────────────────────────

def test_gyro_sign(ser: serial.Serial, parser: FrameParser, hz: int, cal: dict) -> None:
    """
    Drive a forward arc CW then CCW. Both wheels roll forward to avoid slip.
    Yaw rate comes from the wheel speed differential + steering servo.
    Expected yaw rate ≈ (v_fast - v_slow) / track_width ≈ 0.3 rad/s at these speeds.
    """
    print('\n── B. Gyro sign (forward arc CW / CCW) ─────────────────────────────')
    DUR = 2.0

    set_rate(ser, STREAM_IMU, hz)

    print(f'  CW  arc (l=+{ARC_FAST_DPS} r=+{ARC_SLOW_DPS} steer=+{ARC_STEER_US}µs, {DUR} s)…',
          end='', flush=True)
    raw_cw = collect_imu(ser, parser, DUR,
                         motor_cmd=(ARC_FAST_DPS, ARC_SLOW_DPS, ARC_STEER_US))
    cmd_vel(ser, 0, 0, 0)
    time.sleep(0.5)

    print(f'\n  CCW arc (l=+{ARC_SLOW_DPS} r=+{ARC_FAST_DPS} steer=-{ARC_STEER_US}µs, {DUR} s)…',
          end='', flush=True)
    raw_ccw = collect_imu(ser, parser, DUR,
                          motor_cmd=(ARC_SLOW_DPS, ARC_FAST_DPS, -ARC_STEER_US))
    cmd_vel(ser, 0, 0, 0)
    set_rate(ser, STREAM_IMU, 0)
    print()

    if raw_cw:
        gz_cw = mean([decode_imu_cal(p, cal)[1][2] for _, p in raw_cw])
        check_lt('CW  arc: gyro.z (expect negative)', gz_cw, -0.05, unit=' rad/s')
    else:
        _record(False, 'CW arc: frames received', '0', '> 0')

    if raw_ccw:
        gz_ccw = mean([decode_imu_cal(p, cal)[1][2] for _, p in raw_ccw])
        check_gt('CCW arc: gyro.z (expect positive)', gz_ccw,  0.05, unit=' rad/s')
    else:
        _record(False, 'CCW arc: frames received', '0', '> 0')


# ── Test C — accel forward axis ───────────────────────────────────────────────

def test_accel_axis(ser: serial.Serial, parser: FrameParser, hz: int, cal: dict) -> None:
    print('\n── C. Accel forward axis (drive & brake) ───────────────────────────')

    set_rate(ser, STREAM_IMU, hz)

    # Collect baseline accel.y at rest (1 s)
    print('  Baseline (1 s at rest)…', end='', flush=True)
    raw_rest = collect_imu(ser, parser, 1.0)
    ay_rest = mean([decode_imu_cal(p, cal)[0][1] for _, p in raw_rest]) if raw_rest else 0.0
    print(f' accel.y = {ay_rest:.4f} m/s²')

    # Drive forward 0.5 s then stop abruptly
    print('  Driving forward 0.5 s then stopping…', end='', flush=True)
    collect_imu(ser, parser, 0.5, motor_cmd=(DRIVE_DPS, DRIVE_DPS, 0))
    cmd_vel(ser, 0, 0, 0)

    # Collect 0.5 s immediately after stop (captures braking transient)
    raw_brake = collect_imu(ser, parser, 0.5)
    set_rate(ser, STREAM_IMU, 0)

    if not raw_brake:
        warn('no braking frames captured')
        return

    ay_brake_vals = [decode_imu_cal(p, cal)[0][1] for _, p in raw_brake]
    ay_mean = mean(ay_brake_vals)
    ay_min  = min(ay_brake_vals)
    print(f'\n  accel.y post-stop:  mean={ay_mean:.4f}  min={ay_min:.4f} m/s²')

    # chip Y = forward; braking should pull accel.y negative relative to rest
    delta = ay_rest - ay_min   # positive = braking dip occurred
    _record(delta > 0.10, 'chip Y axis senses deceleration (forward axis confirmed)',
            f'Δ={delta:.4f} m/s²', '> 0.10 m/s²')


# ── Test D — mag circle ───────────────────────────────────────────────────────

def test_mag_circle(ser: serial.Serial, parser: FrameParser, hz: int, cal: dict) -> None:
    print('\n── D. Magnetometer rotation check (slow circle) ────────────────────')

    set_rate(ser, STREAM_IMU, hz)
    print('  Driving slow circle for 6 s…', end='', flush=True)
    raw = collect_imu(ser, parser, 6.0, motor_cmd=(CIRCLE_FAST_DPS, CIRCLE_SLOW_DPS, CIRCLE_STEER_US))
    cmd_vel(ser, 0, 0, 0)
    set_rate(ser, STREAM_IMU, 0)
    print(f' got {len(raw)} frames')

    if len(raw) < 20:
        warn(f'too few frames ({len(raw)}) — skip mag circle check')
        return

    mags = [decode_imu_cal(p, cal)[2] for _, p in raw]
    mx = [m[0] for m in mags];  my = [m[1] for m in mags];  mz = [m[2] for m in mags]
    mmag = [math.sqrt(mx[i]**2 + my[i]**2 + mz[i]**2) for i in range(len(mags))]

    # Estimate circle radius in XY plane by centroid method
    cx, cy = mean(mx), mean(my)
    radii  = [math.sqrt((mx[i] - cx)**2 + (my[i] - cy)**2) for i in range(len(mags))]

    check_warn('mag XY circle radius',             mean(radii), 5.0,  65.0, unit=' µT',
               note='calibrate: hard-iron via ROS imu_tools magnetometer_calibration')
    check_warn('mag XY circle residual (std)',     std(radii),  0.0,  10.0, unit=' µT',
               note='calibrate: soft-iron via ROS imu_tools magnetometer_calibration')
    check_warn('mag magnitude during rotation',    mean(mmag),  20.0, 65.0, unit=' µT',
               note='calibrate: hard-iron distortion — use ROS imu_tools magnetometer_calibration')
    check_warn('mag.z std (should stay ~constant)', std(mz),   0.0,   5.0, unit=' µT',
               note='calibrate: soft-iron distortion — use ROS imu_tools magnetometer_calibration')


# ── Test E — mag orientation ──────────────────────────────────────────────────

def test_mag_orientation(ser: serial.Serial, parser: FrameParser, hz: int, cal: dict) -> None:
    """
    Measure mag magnitude AND XY heading at 0 / 90 / 180 / 270° (robot rotated by hand).

    Magnitude check: distinguishes scale/unit errors (constant magnitude) from
    hard-iron distortion (varying magnitude).

    Direction check: verifies the mag XY heading rotates by ~90° with each
    declared 90° robot rotation. Catches axis swaps and sign errors that the
    magnitude test cannot detect.
    """
    print('\n── E. Mag magnitude vs orientation (manual, 4 positions) ───────────')
    if cal.get('mag'):
        print('  (mag calibration active — magnitudes and headings are corrected)')
    else:
        print('  (no mag calibration — run imu_calibrate.py --cal mag to calibrate)')
    print('  Chip axes: X=right, Y=forward (heading = atan2(my, mx) in XY plane)')
    print('  Rotate CW (clockwise when viewed from above) in 90° steps.')

    ORIENTATIONS = [0, 90, 180, 270]
    results = []   # [(declared_angle, mean_magnitude, heading_deg)]

    set_rate(ser, STREAM_IMU, hz)

    for angle in ORIENTATIONS:
        if angle == 0:
            input(f'\n  Face robot in its starting direction (0°), set flat and still, press Enter: ')
        else:
            input(f'  Rotate {angle}° CW from start, set flat and still, press Enter: ')

        frames = collect_imu(ser, parser, 1.5)
        if not frames:
            warn(f'No frames at {angle}° — skipping')
            continue

        mags    = [decode_imu_cal(p, cal)[2] for _, p in frames]
        mmag    = mean([math.sqrt(m[0]**2 + m[1]**2 + m[2]**2) for m in mags])
        heading = math.degrees(math.atan2(mean([m[1] for m in mags]),
                                          mean([m[0] for m in mags])))
        results.append((angle, mmag, heading))
        print(f'    {angle:3d}°:  magnitude={mmag:.1f} µT  heading={heading:+.1f}°')

    set_rate(ser, STREAM_IMU, 0)

    if len(results) < 2:
        warn('Not enough orientation samples to diagnose.')
        return

    def wrap180(d):
        while d > 180:  d -= 360
        while d <= -180: d += 360
        return d

    # ── Pre-compute direction diffs — needed for magnitude diagnosis ───────────
    diffs = [wrap180(results[i][2] - results[i - 1][2])
             for i in range(1, len(results))]
    mean_abs_delta = mean([abs(d) for d in diffs]) if diffs else 0.0
    # heading_tracks: True when the mag XY vector actually follows the robot's rotation.
    # When hard-iron offset >> Earth field, the heading barely moves (~5–15° per 90° step).
    # When only a scale error is present, the heading still tracks correctly (~90° per step).
    heading_tracks = mean_abs_delta > 20.0

    # ── Magnitude analysis ────────────────────────────────────────────────────
    mags_ = [r[1] for r in results]
    spread = max(mags_) - min(mags_)
    ratio  = max(mags_) / min(mags_) if min(mags_) > 0 else 999.0
    avg_mag = mean(mags_)

    print(f'\n  Magnitude:  range={min(mags_):.1f}–{max(mags_):.1f} µT'
          f'  spread={spread:.1f} µT  ratio={ratio:.2f}×')

    if avg_mag > 65 and not heading_tracks:
        # Hard-iron offset so large it buries the Earth field: magnitude appears
        # constant because |H_robot| >> |E_earth|, and heading doesn't track
        # because the dominant vector (hard-iron) is fixed in the robot frame.
        print(f'    {_Y}→ Hard-iron offset DOMINATES — Earth field is buried.{_RST}')
        print( '       Magnitude constant AND heading not tracking rotation:')
        print( '       motor magnets create a permanent field >> Earth field (~50 µT).')
        print( '       Run: python3 tools/imu_calibrate.py --cal mag')
        print( '       After calibration, re-run test E — heading should track ~90° per step.')
    elif avg_mag > 65 and heading_tracks:
        # Heading tracks correctly, but magnitude is inflated → scale or unit error.
        factor = avg_mag / 40.0
        print(f'    {_Y}→ Scale/unit error — heading tracks but magnitude is ~{factor:.1f}× too high.{_RST}')
        print( '       Expected Earth field: 25–65 µT.')
        print( '       Check AK8963 sensitivity (0.15 vs 0.6 µT/LSB,')
        print( '       or milligauss→µT confusion: 1 gauss = 100 µT).')
    elif ratio > 2.0:
        print(f'    {_Y}→ VARIES strongly — hard-iron distortion (likely motor magnets).{_RST}')
        print( '       Run: python3 tools/imu_calibrate.py --cal mag')
    elif ratio > 1.15:
        print(f'    {_Y}→ Moderate variation — mild hard-iron distortion.{_RST}')
        print( '       Run: python3 tools/imu_calibrate.py --cal mag')
    else:
        print(f'    Magnitude consistent and within range.')

    # ── Direction analysis ────────────────────────────────────────────────────
    # Direction FAILs caused by hard-iron dominance are calibration-fixable
    # (same root cause as magnitude WARN above). Emit WARN in that case, FAIL
    # only when magnitude is normal (suggesting a firmware axis-swap bug).
    hard_iron_dominant = avg_mag > 65 and not heading_tracks

    print('\n  Direction (Δheading per 90° rotation step):')
    for i, delta in enumerate(diffs):
        pa = results[i][0];  ca = results[i + 1][0]
        ok = 65.0 <= abs(delta) <= 115.0
        if not ok and hard_iron_dominant:
            # calibration-fixable: emit WARN, don't set _all_passed False
            check_warn(f'{pa:3d}° → {ca:3d}°: Δheading',
                       abs(delta), 65.0, 115.0, '.1f', '°',
                       note='hard-iron dominates — calibrate mag, then re-test')
        else:
            _record(ok, f'{pa:3d}° → {ca:3d}°: Δheading',
                    f'{delta:+.1f}°', '±90° (±25° tolerance)')

    if len(diffs) >= 2:
        n_pos = sum(1 for d in diffs if d > 0)
        n_neg = sum(1 for d in diffs if d < 0)
        if n_pos == 0 or n_neg == 0:
            direction = 'CCW in sensor frame' if n_neg == len(diffs) else 'CW in sensor frame'
            print(f'    Rotation direction: consistent ({direction})')
        elif not hard_iron_dominant:
            warn('Heading changes are not all in the same direction — '
                 'rotation was inconsistent, or an axis is swapped/negated')


# ── Test F — gyro pitch/roll axis sign ───────────────────────────────────────

def test_gyro_axes(ser: serial.Serial, parser: FrameParser, hz: int, cal: dict) -> None:
    """
    Verify gyro.x and gyro.y axis signs by manual tilt.

    Expected (chip axes X=right, Y=forward, Z=up, right-hand rule):
      Nose DOWN  → gyro.x < 0  (v = ω×P: ω_x<0, P=(0,r,0) → v points down)
      Lean RIGHT → gyro.y > 0  (v = ω×P: ω_y>0, P=(r,0,0) → v points down)

    These axes feed Madgwick for roll/pitch estimation. A sign flip causes
    the filter to diverge — not caught by any motor-driven test.
    """
    print('\n── F. Gyro pitch/roll axis sign (manual tilt) ──────────────────────')
    print('  Press Enter to start each 1.5 s window, then tilt and hold.')

    set_rate(ser, STREAM_IMU, hz)

    def _tilt_collect(prompt: str, duration: float) -> list:
        input(prompt)
        for n in ['3', '2', '1', 'GO']:
            print(f'  {n}…', end=' ', flush=True)
            time.sleep(1.0 if n != 'GO' else 0)
        print(f'collecting {duration:.0f} s', flush=True)
        return collect_imu(ser, parser, duration)

    # F1: tilt nose down → gyro.x negative
    print('\n  Tilt 1: slowly tilt nose DOWN during the collection window.')
    raw = _tilt_collect('  Get hands on robot, press Enter to start countdown: ', 2.0)
    if raw:
        gx_mean = mean([decode_imu_cal(p, cal)[1][0] for _, p in raw])
        print(f'    gyro.x mean = {gx_mean:+.4f} rad/s')
        if abs(gx_mean) < 0.02:
            warn('tilt was too small — tilt more decisively during the GO window')
        else:
            _record(gx_mean < 0, 'gyro.x sign: nose DOWN → negative',
                    f'{gx_mean:+.4f} rad/s', '< 0')
    else:
        _record(False, 'gyro.x tilt: frames received', '0', '> 0')

    # F2: lean right side down → gyro.y positive
    print('\n  Tilt 2: slowly lean RIGHT side DOWN during the collection window.')
    raw = _tilt_collect('  Get hands on robot, press Enter to start countdown: ', 2.0)
    if raw:
        gy_mean = mean([decode_imu_cal(p, cal)[1][1] for _, p in raw])
        print(f'    gyro.y mean = {gy_mean:+.4f} rad/s')
        if abs(gy_mean) < 0.02:
            warn('tilt was too small — tilt more decisively during the GO window')
        else:
            _record(gy_mean > 0, 'gyro.y sign: lean RIGHT → positive',
                    f'{gy_mean:+.4f} rad/s', '> 0')
    else:
        _record(False, 'gyro.y tilt: frames received', '0', '> 0')

    set_rate(ser, STREAM_IMU, 0)


# ── Main ──────────────────────────────────────────────────────────────────────

TESTS = {
    'A': 'static accel/gyro/mag checks (includes gyro drift)',
    'B': 'gyro sign (forward arc CW/CCW — avoids wheel slip)',
    'C': 'accel forward axis (drive & brake)',
    'D': 'magnetometer circle (slow arc)',
    'E': 'mag orientation check (manual 4-position rotation)',
    'F': 'gyro pitch/roll axis sign (manual tilt)',
}
MOTION_TESTS = {'B', 'C', 'D'}   # E and F are manual, no motors


def main():
    ap = argparse.ArgumentParser(
        description='IMU verification at UART level (Layer 2, no ROS)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='\n'.join(f'  {k}: {v}' for k, v in TESTS.items()))
    ap.add_argument('--port',  default='/dev/ttyYahboom0')
    ap.add_argument('--baud',  type=int, default=460800)
    ap.add_argument('--freq',     type=int, default=50,
                    help='IMU stream rate in Hz (default: 50)')
    ap.add_argument('--cal-file', metavar='FILE', default=None,
                    help='YAML calibration file from imu_calibrate.py (optional)')
    ap.add_argument('--tests', nargs='+', choices=sorted(TESTS), metavar='TEST',
                    default=sorted(TESTS),
                    help='tests to run (default: all); choices: A B C D E F')
    ap.add_argument('--nomag', action='store_true',
                    help='skip magnetometer tests D and E (chassis hard-iron makes them '
                         'unreliable without a stable calibration environment)')
    args = ap.parse_args()

    tests = set(t.upper() for t in args.tests)
    if args.nomag:
        tests -= {'D', 'E'}
    cal   = load_cal(args.cal_file) if args.cal_file else {}

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except serial.SerialException as e:
        sys.exit(f'Cannot open {args.port}: {e}')

    ser.reset_input_buffer()
    time.sleep(0.1)
    parser = FrameParser()

    print('═' * 62)
    print('  IMU Verification — Layer 2 (UART, no ROS)')
    print(f'  Port: {args.port}  Baud: {args.baud}  Freq: {args.freq} Hz')
    print(f'  Tests: {", ".join(sorted(tests))}')
    if cal:
        print(f'  Cal:   {args.cal_file}  (sections: {", ".join(cal.keys())})')
    else:
        print('  Cal:   none (run imu_calibrate.py first, then use --cal-file)')
    print('═' * 62)
    print()
    print('  Layer 0 pre-check (manual, one time):')
    print('    picocom -b 921600 /dev/ttyYahboom1')
    print('    uart:~$ imu whoami   →  must print 0x71')

    needs_static  = 'A' in tests
    needs_motion  = bool(tests & MOTION_TESTS)
    needs_manual  = bool(tests & {'E', 'F'})

    if needs_static or needs_motion or needs_manual:
        input('\n[1] Place robot on a flat level surface, press Enter when ready: ')

    if needs_static:
        test_static(ser, parser, args.freq, cal)

    if needs_motion:
        motion_list = ', '.join(f'{t} ({TESTS[t]})' for t in sorted(tests & MOTION_TESTS))
        ans = input(
            f'\n[2] Motion test(s): {motion_list}.\n'
            '    Robot will move — ensure 0.5 m clearance. Proceed? [y/N] ')
        if ans.strip().lower() == 'y':
            if 'B' in tests:
                test_gyro_sign(ser, parser, args.freq, cal)
            if 'C' in tests:
                test_accel_axis(ser, parser, args.freq, cal)
            if 'D' in tests:
                test_mag_circle(ser, parser, args.freq, cal)
        else:
            print('  Motion tests skipped.')

    if 'E' in tests:
        test_mag_orientation(ser, parser, args.freq, cal)
    if 'F' in tests:
        test_gyro_axes(ser, parser, args.freq, cal)

    # Safety: stop motors and stream
    cmd_vel(ser, 0, 0, 0)
    set_rate(ser, STREAM_IMU, 0)
    ser.close()

    print('\n' + '═' * 62)
    if _all_passed:
        print(f'  Overall: {_G}PASS{_RST} — all checks passed')
    else:
        print(f'  Overall: {_R}FAIL{_RST} — one or more checks failed')
        print('  Tip: stop at first layer that fails — the bug is there or below.')
    print('═' * 62)
    sys.exit(0 if _all_passed else 1)


if __name__ == '__main__':
    main()
