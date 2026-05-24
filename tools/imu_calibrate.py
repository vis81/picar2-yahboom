#!/usr/bin/env python3
"""
IMU calibration — UART level, no ROS.

Calibrates gyro bias, accel bias, and magnetometer hard/soft-iron distortion.
Saves results to a YAML file compatible with imu_verify_uart.py --cal-file.

If the output file already exists, only the sections for the selected
calibrations are updated; other sections are preserved.

Calibration flows:
  gyro   10 s at rest → per-axis bias (rad/s)
  accel  10 s at rest → per-axis bias (m/s²)
  mag    360° arc → hard-iron offset + per-axis soft-iron scale
           default: motor-driven (bakes in motor hard-iron field)
           --manual: rotate by hand, motors off (valid for static/test-E use)

Usage:
    python3 tools/imu_calibrate.py                          # all (motor-driven mag)
    python3 tools/imu_calibrate.py --cal mag --manual       # hand-rotation mag only
    python3 tools/imu_calibrate.py --cal gyro accel         # skip mag
    python3 tools/imu_calibrate.py --cal mag --output /tmp/cal.yaml
    python3 tools/imu_calibrate.py --port /dev/ttyYahboom0 --freq 50
"""

import argparse
import datetime
import math
import os
import struct
import sys
import time

import serial
import yaml

# ── Protocol constants ────────────────────────────────────────────────────────

PROTO_START   = 0xAA
PROTO_MAX_LEN = 32
MSG_CMD_VEL   = 0x80
MSG_SET_RATE  = 0x82
STREAM_IMU    = 0x02

ACCEL_SCALE = 0.001
GYRO_SCALE  = 0.001
MAG_SCALE   = 0.1
TEMP_SCALE  = 0.01

# ── Motor speeds for mag rotation ─────────────────────────────────────────────

MAG_CIRCLE_FAST_DPS = 300
MAG_CIRCLE_SLOW_DPS = 150
MAG_CIRCLE_STEER_US = 300   # positive = right turn; negate if arc curves wrong way
MAG_CIRCLE_DURATION = 15.0  # seconds — aim for >1 full rotation
MAG_MANUAL_DURATION = 30.0  # seconds for hand-rotation calibration (motors off)

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


# ── Frame parser ──────────────────────────────────────────────────────────────

class FrameParser:
    def __init__(self):
        self._state   = 'START'
        self._mtype   = 0
        self._mlen    = 0
        self._payload = bytearray()

    def feed(self, data: bytes):
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
    v     = struct.unpack_from('<10h', payload)
    accel = [v[i] * ACCEL_SCALE for i in range(3)]
    gyro  = [v[i] * GYRO_SCALE  for i in range(3, 6)]
    mag   = [v[i] * MAG_SCALE   for i in range(6, 9)]
    temp  = v[9] * TEMP_SCALE
    return accel, gyro, mag, temp


# ── Frame collection ──────────────────────────────────────────────────────────

def collect_imu(ser: serial.Serial, parser: FrameParser,
                duration_s: float, motor_cmd=None) -> list:
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
                    frames.append(payload)

    return frames


def mean(xs): return sum(xs) / len(xs)


# ── Calibration flows ─────────────────────────────────────────────────────────

def calibrate_gyro(ser: serial.Serial, parser: FrameParser, hz: int) -> dict:
    print('\n── Gyro bias calibration ────────────────────────────────────────────')
    input('  Keep robot completely still. Press Enter when ready: ')

    set_rate(ser, STREAM_IMU, hz)
    print(f'  Collecting {hz * 10} frames over 10 s…', end='', flush=True)
    frames = collect_imu(ser, parser, 10.0)
    set_rate(ser, STREAM_IMU, 0)
    print(f' got {len(frames)}')

    if len(frames) < 20:
        sys.exit('  Too few frames — check UART connection.')

    gx = mean([decode_imu(p)[1][0] for p in frames])
    gy = mean([decode_imu(p)[1][1] for p in frames])
    gz = mean([decode_imu(p)[1][2] for p in frames])

    print(f'  Gyro bias:  x={gx:+.5f}  y={gy:+.5f}  z={gz:+.5f}  rad/s')
    return {'bias': [round(gx, 6), round(gy, 6), round(gz, 6)]}


def calibrate_accel(ser: serial.Serial, parser: FrameParser, hz: int) -> dict:
    print('\n── Accel bias calibration ───────────────────────────────────────────')
    print('  Measures the static offset from (0, 0, +9.81) while flat.')
    print('  This absorbs both chip-level bias AND mounting tilt into one')
    print('  correction — it is a UART-level alternative to setting')
    print('  imu_mount_roll/pitch in the URDF. Do not apply both.')
    input('  Place robot on a flat level surface, completely still. Press Enter: ')

    set_rate(ser, STREAM_IMU, hz)
    print(f'  Collecting {hz * 10} frames over 10 s…', end='', flush=True)
    frames = collect_imu(ser, parser, 10.0)
    set_rate(ser, STREAM_IMU, 0)
    print(f' got {len(frames)}')

    if len(frames) < 20:
        sys.exit('  Too few frames — check UART connection.')

    ax = mean([decode_imu(p)[0][0] for p in frames])
    ay = mean([decode_imu(p)[0][1] for p in frames])
    az = mean([decode_imu(p)[0][2] for p in frames])

    # Deviation from expected gravity vector (0, 0, +9.81).
    # Large x/y values indicate mounting tilt, not just chip bias — both are
    # captured here and subtracted together by apply_cal in imu_verify_uart.py.
    bx, by, bz = ax, ay, az - 9.81

    print(f'  Accel offset: x={bx:+.5f}  y={by:+.5f}  z={bz:+.5f}  m/s²')
    print(f'  (raw means:   x={ax:.4f}  y={ay:.4f}  z={az:.4f})')
    if abs(bx) > 0.5 or abs(by) > 0.5:
        tilt_x = math.degrees(math.asin(max(-1.0, min(1.0, bx / 9.81))))
        tilt_y = math.degrees(math.asin(max(-1.0, min(1.0, by / 9.81))))
        print(f'  (mounting tilt estimate: roll≈{tilt_y:+.1f}°  pitch≈{tilt_x:+.1f}°)')
    return {'bias': [round(bx, 5), round(by, 5), round(bz, 5)]}


def _lstsq3(A, b):
    """Solve 3×3 linear system via Gaussian elimination with partial pivoting."""
    M = [list(A[i]) + [b[i]] for i in range(3)]
    for col in range(3):
        max_row = max(range(col, 3), key=lambda r: abs(M[r][col]))
        M[col], M[max_row] = M[max_row], M[col]
        if abs(M[col][col]) < 1e-12:
            return None
        for row in range(col + 1, 3):
            f = M[row][col] / M[col][col]
            for j in range(col, 4):
                M[row][j] -= f * M[col][j]
    x = [0.0] * 3
    for i in range(2, -1, -1):
        x[i] = M[i][3]
        for j in range(i + 1, 3):
            x[i] -= M[i][j] * x[j]
        x[i] /= M[i][i]
    return x


def _fit_circle_xy(xs, ys):
    """Kasa least-squares circle fit in XY plane. Returns (cx, cy, r) or None."""
    n = len(xs)
    if n < 3:
        return None
    u = [xs[i]**2 + ys[i]**2 for i in range(n)]
    sx2  = sum(xs[i]**2 for i in range(n))
    sxy  = sum(xs[i]*ys[i] for i in range(n))
    sy2  = sum(ys[i]**2 for i in range(n))
    sx   = sum(xs)
    sy   = sum(ys)
    sxu  = sum(xs[i]*u[i] for i in range(n))
    syu  = sum(ys[i]*u[i] for i in range(n))
    su   = sum(u)
    A    = [[sx2, sxy, sx], [sxy, sy2, sy], [sx, sy, n]]
    sol  = _lstsq3(A, [sxu, syu, su])
    if sol is None:
        return None
    cx, cy = sol[0] / 2, sol[1] / 2
    r = math.sqrt(abs(sol[2] + cx**2 + cy**2))
    return cx, cy, r


def calibrate_mag(ser: serial.Serial, parser: FrameParser, hz: int,
                  manual: bool = False) -> dict:
    if manual:
        print('\n── Magnetometer calibration (manual rotation, motors OFF) ───────────')
        print( '  Spin the robot in place about its vertical axis (Z) by hand.')
        print( '  At least one full 360° turn — more is better. Keep it flat.')
        print( '  Motors stay off — this calibration is valid for static/test-E use.')
        print(f'  Collection window: {MAG_MANUAL_DURATION:.0f} s.')
        input( '  Press Enter, then spin slowly and continuously: ')
        duration   = MAG_MANUAL_DURATION
        motor_cmd  = None
    else:
        print('\n── Magnetometer calibration (motor-driven arc) ───────────────────────')
        print(f'  Robot will drive a circle for {MAG_CIRCLE_DURATION:.0f} s.')
        print( '  Note: motor hard-iron is baked into this calibration.')
        print( '  Use --manual for a motors-off calibration (test E / static use).')
        print( '  Ensure 0.5 m clearance around robot.')
        input( '  Press Enter to start: ')
        duration   = MAG_CIRCLE_DURATION
        motor_cmd  = (MAG_CIRCLE_FAST_DPS, MAG_CIRCLE_SLOW_DPS, MAG_CIRCLE_STEER_US)

    set_rate(ser, STREAM_IMU, hz)
    print(f'  Collecting mag data ({duration:.0f} s)…', end='', flush=True)
    frames = collect_imu(ser, parser, duration, motor_cmd=motor_cmd)
    if motor_cmd:
        cmd_vel(ser, 0, 0, 0)
    set_rate(ser, STREAM_IMU, 0)
    print(f' got {len(frames)}')

    min_frames = 20 if manual else 30
    if len(frames) < min_frames:
        sys.exit(f'  Too few frames ({len(frames)}) — check UART connection'
                 + ('' if manual else ' and motor operation.'))

    mx = [decode_imu(p)[2][0] for p in frames]
    my = [decode_imu(p)[2][1] for p in frames]
    mz = [decode_imu(p)[2][2] for p in frames]

    # Hard-iron XY: Kasa least-squares circle fit (uses all points, not just extremes)
    fit = _fit_circle_xy(mx, my)
    if fit is not None:
        hx, hy, _fit_r = fit
        _xy_method = 'least-squares circle fit'
    else:
        hx = (max(mx) + min(mx)) / 2
        hy = (max(my) + min(my)) / 2
        _xy_method = 'bounding box (circle fit failed)'
    hz_val = (max(mz) + min(mz)) / 2

    # Soft-iron: per-axis scale so all half-ranges match their mean
    rx = (max(mx) - min(mx)) / 2
    ry = (max(my) - min(my)) / 2
    rz = (max(mz) - min(mz)) / 2

    # Average radius from the two horizontal axes (z coverage is limited
    # during flat rotation — keep its scale factor at 1.0)
    avg_r = (rx + ry) / 2
    sx = avg_r / rx if rx > 0 else 1.0
    sy = avg_r / ry if ry > 0 else 1.0
    sz = 1.0   # z not well-sampled in horizontal rotation

    # Report corrected magnitude using the calibration we just computed
    mag_corrected = [
        math.sqrt(((mx[i] - hx) * sx) ** 2 +
                  ((my[i] - hy) * sy) ** 2 +
                  ((mz[i] - hz_val) * sz) ** 2)
        for i in range(len(frames))
    ]

    print(f'  Hard-iron:  x={hx:+.2f}  y={hy:+.2f}  z={hz_val:+.2f}  µT  ({_xy_method})')
    print(f'  Soft-iron:  x={sx:.4f}  y={sy:.4f}  z={sz:.4f}')
    print(f'  Corrected magnitude: mean={mean(mag_corrected):.2f} µT'
          f'  (raw range: {min(mx):.1f}–{max(mx):.1f} / '
          f'{min(my):.1f}–{max(my):.1f} / {min(mz):.1f}–{max(mz):.1f})')

    return {
        'hard_iron': [round(hx, 3), round(hy, 3), round(hz_val, 3)],
        'soft_iron': [round(sx, 5), round(sy, 5), round(sz, 5)],
    }


# ── YAML load / save ──────────────────────────────────────────────────────────

def load_yaml(path: str) -> dict:
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        return yaml.safe_load(f) or {}


def save_yaml(path: str, existing: dict, updates: dict) -> None:
    merged = dict(existing)
    merged.update(updates)

    header = (
        f'# IMU calibration — generated by tools/imu_calibrate.py\n'
        f'# Last updated: {datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")}\n'
        f'#\n'
        f'# Apply with: python3 tools/imu_verify_uart.py --cal-file {path}\n'
        f'#\n'
        f'# gyro.bias      [x, y, z] rad/s  — subtract from raw gyro reading\n'
        f'# accel.bias     [x, y, z] m/s²   — static offset from (0,0,9.81): includes chip bias\n'
        f'#                                    AND mounting tilt. UART-level alternative to\n'
        f'#                                    imu_mount_roll/pitch in the URDF — do not use both.\n'
        f'# mag.hard_iron  [x, y, z] µT     — subtract from raw mag reading\n'
        f'# mag.soft_iron  [x, y, z]        — per-axis scale after hard-iron correction\n'
    )

    with open(path, 'w') as f:
        f.write(header)
        yaml.dump(merged, f, default_flow_style=None, sort_keys=True)

    print(f'\n  Saved → {os.path.abspath(path)}')


# ── Main ──────────────────────────────────────────────────────────────────────

CALS = {
    'gyro':  'gyro bias (10 s at rest)',
    'accel': 'accel bias (10 s at rest)',
    'mag':   'mag hard/soft-iron (motor-driven arc or --manual for hand rotation)',
}


def main():
    ap = argparse.ArgumentParser(
        description='IMU calibration at UART level (no ROS)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='\n'.join(f'  {k}: {v}' for k, v in CALS.items()))
    ap.add_argument('--port',   default='/dev/ttyYahboom0')
    ap.add_argument('--baud',   type=int, default=460800)
    ap.add_argument('--freq',   type=int, default=50,
                    help='IMU stream rate in Hz (default: 50)')
    ap.add_argument('--cal',    nargs='+', choices=sorted(CALS), metavar='CAL',
                    default=sorted(CALS),
                    help='calibrations to run (default: all); choices: gyro accel mag')
    ap.add_argument('--manual', action='store_true',
                    help='mag calibration: rotate robot by hand (motors off) instead of '
                         'motor-driven circle. Use this when calibrating for static/test use.')
    ap.add_argument('--output', default='imu_calibration.yaml',
                    help='output YAML file (default: ./imu_calibration.yaml)')
    args = ap.parse_args()

    cals = set(args.cal)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except serial.SerialException as e:
        sys.exit(f'Cannot open {args.port}: {e}')

    ser.reset_input_buffer()
    time.sleep(0.1)
    parser = FrameParser()

    print('═' * 62)
    print('  IMU Calibration — UART level (no ROS)')
    print(f'  Port: {args.port}  Baud: {args.baud}  Freq: {args.freq} Hz')
    print(f'  Cals: {", ".join(sorted(cals))}'
          + ('  (mag: manual rotation)' if args.manual and 'mag' in cals else ''))
    print(f'  Output: {args.output}')
    existing = load_yaml(args.output)
    if existing:
        print(f'  Existing sections: {", ".join(existing.keys())} (will be merged)')
    print('═' * 62)

    updates = {}

    if 'gyro' in cals:
        updates['gyro'] = calibrate_gyro(ser, parser, args.freq)

    if 'accel' in cals:
        updates['accel'] = calibrate_accel(ser, parser, args.freq)

    if 'mag' in cals:
        updates['mag'] = calibrate_mag(ser, parser, args.freq, manual=args.manual)

    # Safety: stop motors and stream
    cmd_vel(ser, 0, 0, 0)
    set_rate(ser, STREAM_IMU, 0)
    ser.close()

    save_yaml(args.output, existing, updates)

    print('\n  Done. To use this calibration in verification:')
    print(f'    python3 tools/imu_verify_uart.py --cal-file {args.output}')


if __name__ == '__main__':
    main()
