#!/usr/bin/env python3
"""Set or show steering servo center position over the binary serial protocol.

Usage:
    python3 tools/servo_center.py                        # show all centers
    python3 tools/servo_center.py 0 1470                 # set servo 0 to 1470 µs
    python3 tools/servo_center.py 0 1470 --port /dev/ttyYahboom0
    python3 tools/servo_center.py 1 1550 --baud 460800
"""

import argparse
import struct
import sys

import serial

# ── Protocol ─────────────────────────────────────────────────────────────────

PROTO_START      = 0xAA
MSG_REQ          = 0x81
MSG_SERVO_CENTER = 0x86
STREAM_JOINT     = 0x01


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def frame(msg_type: int, payload: bytes = b'') -> bytes:
    hdr = bytes([PROTO_START, msg_type, len(payload)])
    return hdr + payload + bytes([crc8(bytes([msg_type, len(payload)]) + payload)])


def encode_servo_center(servo_id: int, center_us: int) -> bytes:
    return frame(MSG_SERVO_CENTER, struct.pack('<BH', servo_id, center_us))


# ── Serial helpers ────────────────────────────────────────────────────────────

def recv_frame(ser: serial.Serial, timeout: float = 0.5):
    """Read one validated frame; return (type, payload) or None on timeout/error."""
    ser.timeout = timeout
    deadline = ser.timeout
    buf = b''
    while True:
        b = ser.read(1)
        if not b:
            return None
        buf += b
        # scan for start byte
        start = buf.rfind(bytes([PROTO_START]))
        if start < 0 or len(buf) - start < 3:
            continue
        frame_start = buf[start:]
        msg_type = frame_start[1]
        length   = frame_start[2]
        need = 4 + length
        if len(frame_start) < need:
            more = ser.read(need - len(frame_start))
            if len(more) < need - len(frame_start):
                return None
            frame_start += more
        payload = frame_start[3:3 + length]
        crc_got = frame_start[3 + length]
        crc_exp = crc8(bytes([msg_type, length]) + payload)
        if crc_got != crc_exp:
            buf = b''
            continue
        return msg_type, payload


def read_joint_frame(ser: serial.Serial):
    """Request a JOINT frame and decode steer position (µs delta from center)."""
    ser.write(frame(MSG_REQ, bytes([STREAM_JOINT])))
    result = recv_frame(ser, timeout=0.5)
    if result and result[0] == STREAM_JOINT and len(result[1]) >= 10:
        delta_us = struct.unpack_from('<h', result[1], 8)[0]
        return delta_us
    return None


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description='Set/show servo center via binary protocol')
    ap.add_argument('servo_id',   type=int, nargs='?', help='Servo ID (0, 1, 2)')
    ap.add_argument('center_us',  type=int, nargs='?', help='New center pulse width in µs')
    ap.add_argument('--port', default='/dev/ttyYahboom0')
    ap.add_argument('--baud', type=int, default=460800)
    args = ap.parse_args()

    if args.servo_id is not None and args.center_us is None:
        ap.error('center_us is required when servo_id is given')

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.5)
    except serial.SerialException as e:
        sys.exit(f'Cannot open {args.port}: {e}')

    if args.servo_id is None:
        # Show mode: request current steer offset (servo 0 only for now)
        delta = read_joint_frame(ser)
        if delta is not None:
            print(f'servo 0 current offset: {delta:+d} µs from center  (use servo_id center_us to set center)')
        else:
            print('No response — is bringup running?')
    else:
        ser.write(encode_servo_center(args.servo_id, args.center_us))
        ser.flush()
        print(f'servo {args.servo_id}: center set to {args.center_us} µs')
        # Verify by reading back steer offset (should be ~0 for servo 0)
        if args.servo_id == 0:
            delta = read_joint_frame(ser)
            if delta is not None:
                print(f'servo 0 offset after centering: {delta:+d} µs')

    ser.close()


if __name__ == '__main__':
    main()
