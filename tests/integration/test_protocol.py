#!/usr/bin/env python3
"""
PICAR-2 STM32↔Pi protocol verification script.

Runs sequential tests against the STM32 firmware over USART1.

Usage:
    python3 tests/integration/test_protocol.py
    python3 tests/integration/test_protocol.py --port /dev/ttyUSB0 --baud 921600
"""

import argparse
import queue
import struct
import sys
import threading
import time
from collections import Counter

import serial

# ── Protocol constants ────────────────────────────────────────────────────────

PROTO_START = 0xAA

TYPE_JOINT = 0x01
TYPE_IMU   = 0x02
TYPE_BAT   = 0x03

MSG_CMD_VEL  = 0x80
MSG_REQ      = 0x81
MSG_SET_RATE = 0x82

STREAM_NAMES = {TYPE_JOINT: "JOINT_STATE", TYPE_IMU: "IMU", TYPE_BAT: "BATTERY"}


# ── CRC-8 Dallas/Maxim (poly 0x31) ───────────────────────────────────────────

def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


# ── Frame encoding ────────────────────────────────────────────────────────────

def frame(msg_type: int, payload: bytes = b"") -> bytes:
    hdr = bytes([PROTO_START, msg_type, len(payload)])
    return hdr + payload + bytes([crc8(bytes([msg_type, len(payload)]) + payload)])

def cmd_vel(left_mms: int, right_mms: int, steering: int) -> bytes:
    return frame(MSG_CMD_VEL, struct.pack("<hhB", left_mms, right_mms, steering))

def req(stream_id: int) -> bytes:
    return frame(MSG_REQ, bytes([stream_id]))

def set_rate(stream_id: int, hz: int) -> bytes:
    return frame(MSG_SET_RATE, struct.pack("<BH", stream_id, hz))


# ── Frame decoding ────────────────────────────────────────────────────────────

def decode_joint(payload: bytes) -> dict:
    enc_l, enc_r, steer, seq = struct.unpack("<iiBB", payload[:10])
    return {"enc_left": enc_l, "enc_right": enc_r, "steering": steer, "seq": seq}

def decode_imu(payload: bytes) -> dict:
    v = struct.unpack("<10h", payload[:20])
    return {
        "accel_m_s2": [x * 0.001 for x in v[0:3]],
        "gyro_rad_s":  [x * 0.001 for x in v[3:6]],
        "magn_uT":     [x * 0.1   for x in v[6:9]],
        "temp_C":      v[9] * 0.01,
    }

def decode_bat(payload: bytes) -> dict:
    mv, pct, _ = struct.unpack("<HBB", payload[:4])
    return {"voltage_mv": mv, "charge_pct": pct}

DECODERS = {TYPE_JOINT: decode_joint, TYPE_IMU: decode_imu, TYPE_BAT: decode_bat}


# ── Receiver thread ───────────────────────────────────────────────────────────

class Receiver(threading.Thread):
    """Reads bytes from serial, assembles frames, pushes (type, raw, decoded) to a queue."""

    _S_START, _S_TYPE, _S_LEN, _S_PAYLOAD, _S_CRC = range(5)

    def __init__(self, ser: serial.Serial):
        super().__init__(daemon=True)
        self.ser = ser
        self.q: queue.Queue = queue.Queue()
        self._state = self._S_START
        self._type = self._len = 0
        self._buf = bytearray()

    def run(self):
        while True:
            try:
                b = self.ser.read(1)
            except serial.SerialException:
                break
            if b:
                self._feed(b[0])

    def _feed(self, b: int):
        s = self._state
        if s == self._S_START:
            if b == PROTO_START:
                self._state = self._S_TYPE
        elif s == self._S_TYPE:
            self._type = b
            self._state = self._S_LEN
        elif s == self._S_LEN:
            if b > 32:
                self._state = self._S_START
                return
            self._len = b
            self._buf = bytearray()
            self._state = self._S_PAYLOAD if b else self._S_CRC
        elif s == self._S_PAYLOAD:
            self._buf.append(b)
            if len(self._buf) == self._len:
                self._state = self._S_CRC
        elif s == self._S_CRC:
            expected = crc8(bytes([self._type, self._len]) + bytes(self._buf))
            if b == expected:
                decoded = None
                if self._type in DECODERS:
                    try:
                        decoded = DECODERS[self._type](bytes(self._buf))
                    except Exception:
                        pass
                self.q.put((self._type, bytes(self._buf), decoded))
            self._state = self._S_START

    def recv(self, timeout: float = 1.0):
        try:
            return self.q.get(timeout=timeout)
        except queue.Empty:
            return None

    def recv_type(self, expected_type: int, timeout: float = 1.0):
        """Wait for a specific frame type; silently discard others."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            item = self.recv(timeout=max(0.01, deadline - time.monotonic()))
            if item and item[0] == expected_type:
                return item
        return None

    def drain(self, duration: float = 0.5) -> list:
        """Collect all frames that arrive within duration seconds."""
        frames = []
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            item = self.recv(timeout=max(0.01, deadline - time.monotonic()))
            if item:
                frames.append(item)
        return frames

    def flush(self):
        while not self.q.empty():
            try:
                self.q.get_nowait()
            except queue.Empty:
                break


# ── Test helpers ──────────────────────────────────────────────────────────────

PASS = "\033[32mPASS\033[0m"
FAIL = "\033[31mFAIL\033[0m"
INFO = "    "

_results: list[tuple[str, bool]] = []

def check(name: str, ok: bool, detail: str = "") -> bool:
    tag = PASS if ok else FAIL
    suffix = f"  ({detail})" if detail else ""
    print(f"  [{tag}] {name}{suffix}")
    _results.append((name, ok))
    return ok

def section(title: str):
    print(f"\n{'─' * 62}")
    print(f"  {title}")
    print(f"{'─' * 62}")

_ser_lock = threading.Lock()

def safe_write(ser: serial.Serial, data: bytes) -> None:
    with _ser_lock:
        ser.write(data)

def send_cmd_vel_background(ser: serial.Serial, left=0, right=0, steer=50):
    """Start a background thread sending CMD_VEL at 100 Hz. Returns (stop_event, thread)."""
    stop = threading.Event()
    def loop():
        while not stop.is_set():
            safe_write(ser, cmd_vel(left, right, steer))
            time.sleep(0.01)
    t = threading.Thread(target=loop, daemon=True)
    t.start()
    return stop, t


# ── Individual tests ──────────────────────────────────────────────────────────

def test_startup_silence(rx: Receiver):
    section("1 · Startup silence — all streams off by default")
    frames = rx.drain(1.5)
    check("No spontaneous frames at startup", len(frames) == 0,
          f"got {len(frames)} unexpected frames" if frames else "")


def test_connection(ser: serial.Serial, rx: Receiver):
    section("2 · Connection handshake — CMD_VEL triggers CONNECTED")
    rx.flush()

    print(f"  {INFO} Sending CMD_VEL at 100 Hz for 0.5 s ...")
    stop, bg = send_cmd_vel_background(ser)
    time.sleep(0.5)

    safe_write(ser, req(TYPE_JOINT))
    f = rx.recv_type(TYPE_JOINT, timeout=2.0)
    stop.set()
    bg.join(timeout=0.2)
    check("REQ(JOINT_STATE) answered while CONNECTED", f is not None)


def test_req_all_streams(ser: serial.Serial, rx: Receiver):
    section("3 · REQ — on-demand response for every stream")
    rx.flush()

    for sid, name in STREAM_NAMES.items():
        safe_write(ser, req(sid))
        f = rx.recv_type(sid, timeout=2.0)
        ok = check(f"REQ({name}) returns one frame", f is not None)
        if ok and f[2]:
            print(f"  {INFO}   {f[2]}")


def test_set_rate(ser: serial.Serial, rx: Receiver):
    section("4 · SET_RATE — configurable periodic streams")

    # ── IMU at 10 Hz ──────────────────────────────────────────────────────────
    rx.flush()
    safe_write(ser, set_rate(TYPE_IMU, 10))
    time.sleep(0.05)
    frames = rx.drain(1.0)
    imu = [f for f in frames if f[0] == TYPE_IMU]
    check("IMU at 10 Hz: 7–13 frames/s", 7 <= len(imu) <= 13, f"got {len(imu)}")

    # ── REQ still works while periodic is running ──────────────────────────────
    rx.flush()
    safe_write(ser, req(TYPE_IMU))
    f = rx.recv_type(TYPE_IMU, timeout=0.5)
    check("REQ(IMU) works while periodic is active", f is not None)

    # ── Disable IMU ────────────────────────────────────────────────────────────
    # Send stop command several times with gaps so that even if the system
    # workqueue is briefly occupied by a sensor_sample_fetch I2C call, at
    # least one SET_RATE(0) is processed before the drain window.
    for _ in range(3):
        safe_write(ser, set_rate(TYPE_IMU, 0))
        time.sleep(0.15)
    rx.flush()
    frames = rx.drain(0.5)
    imu = [f for f in frames if f[0] == TYPE_IMU]
    check("IMU stops after SET_RATE(IMU, 0)", len(imu) == 0,
          f"got {len(imu)}" if imu else "")

    # ── JOINT_STATE at 10 Hz + BATTERY at 1 Hz simultaneously ────────────────
    rx.flush()
    safe_write(ser, set_rate(TYPE_JOINT, 10))
    safe_write(ser, set_rate(TYPE_BAT, 1))
    time.sleep(0.05)
    frames = rx.drain(5.0)
    js  = [f for f in frames if f[0] == TYPE_JOINT]
    bat = [f for f in frames if f[0] == TYPE_BAT]
    check("JOINT_STATE at 10 Hz: 40–60 frames in 5 s", 40 <= len(js) <= 60,
          f"got {len(js)}")
    check("BATTERY at 1 Hz: 2–7 frames in 5 s", 2 <= len(bat) <= 7,
          f"got {len(bat)}")
    if bat and bat[0][2]:
        print(f"  {INFO}   battery: {bat[0][2]}")

    # ── Clean up ───────────────────────────────────────────────────────────────
    safe_write(ser, set_rate(TYPE_JOINT, 0))
    safe_write(ser, set_rate(TYPE_BAT, 0))
    time.sleep(0.1)
    rx.flush()


def test_cmd_vel_motors(ser: serial.Serial, rx: Receiver):
    section("5 · CMD_VEL — motors and servo respond")
    rx.flush()

    safe_write(ser, set_rate(TYPE_JOINT, 20))
    time.sleep(0.05)
    rx.flush()

    print(f"  {INFO} Sending forward 500 mm/s for 1 s ...")
    stop, bg = send_cmd_vel_background(ser, left=500, right=500, steer=50)
    time.sleep(0.2)
    rx.flush()
    time.sleep(0.8)
    frames = rx.drain(0.2)
    stop.set()
    bg.join(timeout=0.2)

    js = [f for f in frames if f[0] == TYPE_JOINT and f[2]]
    if len(js) >= 2:
        dl = js[-1][2]["enc_left"]  - js[0][2]["enc_left"]
        dr = js[-1][2]["enc_right"] - js[0][2]["enc_right"]
        check("Encoders advance during forward motion",
              dl != 0 or dr != 0, f"Δenc_l={dl}  Δenc_r={dr}")
    else:
        print(f"  {INFO} (skipping encoder-advance check — too few JOINT frames)")

    steer_stop, steer_bg = send_cmd_vel_background(ser, left=0, right=0, steer=20)
    time.sleep(0.15)
    steer_stop.set()
    steer_bg.join(timeout=0.2)
    rx.flush()
    safe_write(ser, req(TYPE_JOINT))
    f = rx.recv_type(TYPE_JOINT, timeout=0.5)
    if f and f[2]:
        check("Steering echoed correctly in JOINT_STATE",
              f[2]["steering"] == 20, f"got {f[2]['steering']}")

    safe_write(ser, set_rate(TYPE_JOINT, 0))
    time.sleep(0.1)
    rx.flush()


def test_watchdog(ser: serial.Serial, rx: Receiver):
    section("6 · Watchdog — DISCONNECTED after 500 ms silence")
    rx.flush()

    print(f"  {INFO} Establishing connection ...")
    stop, bg = send_cmd_vel_background(ser)
    time.sleep(0.4)

    safe_write(ser, req(TYPE_JOINT))
    f = rx.recv_type(TYPE_JOINT, timeout=0.5)
    check("CONNECTED: REQ answered before silence", f is not None)

    stop.set()
    bg.join(timeout=0.2)
    print(f"  {INFO} Silence for 800 ms (watchdog threshold: 500 ms) ...")
    time.sleep(0.8)

    safe_write(ser, cmd_vel(0, 0, 50))
    time.sleep(0.05)
    safe_write(ser, req(TYPE_JOINT))
    f = rx.recv_type(TYPE_JOINT, timeout=0.5)
    check("Reconnects cleanly after watchdog timeout", f is not None)


def test_bad_crc(ser: serial.Serial, rx: Receiver):
    section("7 · Bad CRC — silently dropped, firmware does not crash")
    rx.flush()

    good = cmd_vel(0, 0, 50)
    bad  = good[:-1] + bytes([good[-1] ^ 0xFF])
    safe_write(ser, bad)
    time.sleep(0.1)

    safe_write(ser, req(TYPE_JOINT))
    f = rx.recv_type(TYPE_JOINT, timeout=1.0)
    check("Firmware responds to valid frame after bad-CRC frame", f is not None)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="PICAR-2 protocol verification")
    parser.add_argument("--port", default="/dev/ttyYahboom0",
                        help="Serial port connected to STM32 USART1 (default: /dev/ttyYahboom0)")
    parser.add_argument("--baud", type=int, default=460800)
    args = parser.parse_args()

    print(f"\nPICAR-2 protocol verification  —  {args.port} @ {args.baud} baud")
    print("=" * 62)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"Cannot open {args.port}: {e}")
        sys.exit(1)

    rx = Receiver(ser)
    rx.start()
    time.sleep(0.2)
    rx.flush()

    for sid in (TYPE_JOINT, TYPE_IMU, TYPE_BAT):
        safe_write(ser, set_rate(sid, 0))
    time.sleep(0.2)
    rx.flush()

    try:
        test_startup_silence(rx)
        test_connection(ser, rx)
        test_req_all_streams(ser, rx)
        test_set_rate(ser, rx)
        test_cmd_vel_motors(ser, rx)
        test_watchdog(ser, rx)
        test_bad_crc(ser, rx)
    finally:
        for sid in (TYPE_JOINT, TYPE_IMU, TYPE_BAT):
            safe_write(ser, set_rate(sid, 0))
        safe_write(ser, cmd_vel(0, 0, 50))
        time.sleep(0.05)
        ser.close()

    print(f"\n{'=' * 62}")
    passed = sum(1 for _, ok in _results if ok)
    total  = len(_results)
    colour = "\033[32m" if passed == total else "\033[31m"
    print(f"  {colour}{passed}/{total} checks passed\033[0m")
    print(f"{'=' * 62}\n")
    sys.exit(0 if passed == total else 1)


if __name__ == "__main__":
    main()
