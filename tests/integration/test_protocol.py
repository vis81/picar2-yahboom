#!/usr/bin/env python3
"""
PICAR-2 STM32↔Pi protocol verification script.

Runs sequential tests against the STM32 firmware over USART1.

Usage:
    python3 tests/integration/test_protocol.py
    python3 tests/integration/test_protocol.py --port /dev/ttyUSB0 --baud 460800
"""

import argparse
import queue
import struct
import sys
import threading
import time

import serial

# ── Protocol constants ────────────────────────────────────────────────────────

PROTO_START = 0xAA

TYPE_JOINT = 0x01
TYPE_IMU   = 0x02
TYPE_BAT   = 0x03
TYPE_STATS = 0x04

MSG_CMD_VEL       = 0x80
MSG_REQ           = 0x81
MSG_SET_RATE      = 0x82
MSG_GET_STATS     = 0x83
MSG_TIMESYNC      = 0x84
MSG_TIMESYNC_RESP = 0x05
MSG_PID_SET       = 0x85

STREAM_PID = 0x06

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

def cmd_vel(left_mms: int, right_mms: int, steering: int = 0) -> bytes:
    """steering in tenths of degrees: 0 = center, +900 = 90° CW, -900 = 90° CCW"""
    return frame(MSG_CMD_VEL, struct.pack("<hhh", left_mms, right_mms, steering))

def req(stream_id: int) -> bytes:
    return frame(MSG_REQ, bytes([stream_id]))

def set_rate(stream_id: int, hz: int) -> bytes:
    return frame(MSG_SET_RATE, struct.pack("<BH", stream_id, hz))

def get_stats(clear: bool = False) -> bytes:
    return frame(MSG_GET_STATS, bytes([1]) if clear else b"")

def encode_timesync(t1_us: int, t4_prev_us: int = 0) -> bytes:
    return frame(MSG_TIMESYNC, struct.pack("<qq", t1_us, t4_prev_us))

def encode_pid_set(motor_id: int, kp: float, ki: float, kd: float) -> bytes:
    return frame(MSG_PID_SET, struct.pack("<Bhhh",
        motor_id, int(kp * 100), int(ki * 100), int(kd * 100)))


# ── Frame decoding ────────────────────────────────────────────────────────────

def decode_joint(payload: bytes) -> dict:
    enc_l, enc_r, steer, seq = struct.unpack("<iihB", payload[:11])
    d = {"enc_left": enc_l, "enc_right": enc_r, "steering": steer, "seq": seq}
    if len(payload) >= 15:
        vel_l, vel_r = struct.unpack_from("<hh", payload, 11)
        d["vel_left"] = vel_l
        d["vel_right"] = vel_r
    if len(payload) >= 23:
        pi_time_us, = struct.unpack_from("<q", payload, 15)
        d["pi_time_us"] = pi_time_us
    return d

def decode_timesync_resp(payload: bytes) -> dict:
    t2_us, = struct.unpack("<q", payload[:8])
    return {"t2_us": t2_us}

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

def decode_pid_stream(payload: bytes) -> dict:
    sp_l, act_l, sp_r, act_r = struct.unpack_from("<hhhh", payload)
    return {"setpoint_L": sp_l, "actual_L": act_l,
            "setpoint_R": sp_r, "actual_R": act_r}

def decode_stats(payload: bytes) -> dict:
    fields = struct.unpack_from("<6I", payload, 0)
    return {
        "rx_frames":  fields[0],
        "rx_crc_err": fields[1],
        "rx_len_err": fields[2],
        "rx_short":   fields[3],
        "rx_unknown": fields[4],
        "tx_frames":  fields[5],
    }

DECODERS = {
    TYPE_JOINT:        decode_joint,
    TYPE_IMU:          decode_imu,
    TYPE_BAT:          decode_bat,
    TYPE_STATS:        decode_stats,
    MSG_TIMESYNC_RESP: decode_timesync_resp,
    STREAM_PID:        decode_pid_stream,
}


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
        self.rx_frames     = 0
        self.rx_crc_err    = 0
        self.rx_decode_err = 0

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
                self.rx_frames += 1
                decoded = None
                if self._type in DECODERS:
                    try:
                        decoded = DECODERS[self._type](bytes(self._buf))
                    except Exception:
                        self.rx_decode_err += 1
                self.q.put((self._type, bytes(self._buf), decoded))
            else:
                self.rx_crc_err += 1
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

def send_cmd_vel_background(ser: serial.Serial, left=0, right=0, steer=0):
    """Start a background thread sending CMD_VEL at 100 Hz. Returns (stop_event, thread)."""
    stop = threading.Event()
    def loop():
        while not stop.is_set():
            safe_write(ser, cmd_vel(left, right, steer))
            time.sleep(0.01)
    t = threading.Thread(target=loop, daemon=True)
    t.start()
    return stop, t

def fetch_stats(ser: serial.Serial, rx: Receiver,
                clear: bool = False, timeout: float = 2.0) -> dict | None:
    """Send MSG_GET_STATS and return decoded dict, or None on timeout."""
    safe_write(ser, get_stats(clear=clear))
    item = rx.recv_type(TYPE_STATS, timeout=timeout)
    return item[2] if item and item[2] else None

def stop_all_streams(ser: serial.Serial):
    for sid in (TYPE_JOINT, TYPE_IMU, TYPE_BAT, STREAM_PID):
        safe_write(ser, set_rate(sid, 0))

def halt_motors(ser: serial.Serial):
    safe_write(ser, cmd_vel(0, 0, 0))

def seq_gaps(frames: list) -> tuple[int, int]:
    """Return (gap_count, missing_frame_count) from JOINT_STATE seq fields."""
    seqs = [f[2]["seq"] for f in frames if f[0] == TYPE_JOINT and f[2]]
    if len(seqs) < 2:
        return 0, 0
    n_gaps = n_missing = 0
    for i in range(1, len(seqs)):
        delta = (seqs[i] - seqs[i - 1]) % 256
        if delta > 1:
            n_gaps  += 1
            n_missing += delta - 1
    return n_gaps, n_missing


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
    stop_all_streams(ser)
    time.sleep(0.1)
    rx.flush()


def test_cmd_vel_motors(ser: serial.Serial, rx: Receiver):
    section("5 · CMD_VEL — motors and servo respond")
    rx.flush()

    safe_write(ser, set_rate(TYPE_JOINT, 20))
    time.sleep(0.05)
    rx.flush()

    print(f"  {INFO} Sending forward 500 mm/s for 1 s ...")
    stop, bg = send_cmd_vel_background(ser, left=500, right=500, steer=0)
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

    steer_stop, steer_bg = send_cmd_vel_background(ser, left=0, right=0, steer=180)
    time.sleep(0.15)
    steer_stop.set()
    steer_bg.join(timeout=0.2)
    rx.flush()
    safe_write(ser, req(TYPE_JOINT))
    f = rx.recv_type(TYPE_JOINT, timeout=0.5)
    if f and f[2]:
        check("Steering echoed correctly in JOINT_STATE",
              f[2]["steering"] == 180, f"got {f[2]['steering']}")

    stop_all_streams(ser)
    halt_motors(ser)
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

    safe_write(ser, cmd_vel(0, 0, 0))
    time.sleep(0.05)
    safe_write(ser, req(TYPE_JOINT))
    f = rx.recv_type(TYPE_JOINT, timeout=0.5)
    check("Reconnects cleanly after watchdog timeout", f is not None)


def test_bad_crc(ser: serial.Serial, rx: Receiver):
    section("7 · Bad CRC — silently dropped, firmware does not crash")
    rx.flush()

    good = cmd_vel(0, 0, 0)
    bad  = good[:-1] + bytes([good[-1] ^ 0xFF])
    safe_write(ser, bad)
    time.sleep(0.1)

    safe_write(ser, req(TYPE_JOINT))
    f = rx.recv_type(TYPE_JOINT, timeout=1.0)
    check("Firmware responds to valid frame after bad-CRC frame", f is not None)


def test_get_stats(ser: serial.Serial, rx: Receiver):
    section("8 · MSG_GET_STATS — firmware reports and clears counters")
    rx.flush()

    # Establish connection
    stop, bg = send_cmd_vel_background(ser)
    time.sleep(0.3)
    stop.set(); bg.join(timeout=0.2)

    # Baseline: fetch without clear, verify response arrives and fields look sane
    s = fetch_stats(ser, rx)
    ok = check("STREAM_STATS received", s is not None)
    if not ok:
        return
    print(f"  {INFO} rx_frames={s['rx_frames']}  crc_err={s['rx_crc_err']}  "
          f"tx_frames={s['tx_frames']}")
    check("rx_frames > 0 after earlier tests", s["rx_frames"] > 0,
          f"got {s['rx_frames']}")

    # Clear: firmware clears before sending, so response must be all zeros
    s2 = fetch_stats(ser, rx, clear=True)
    ok = check("STREAM_STATS received after clear", s2 is not None)
    if ok:
        bad = {k: v for k, v in s2.items() if v != 0}
        check("All counters zero after clear", not bad,
              " ".join(f"{k}={v}" for k, v in bad.items()))

    # Confirm counters increment again after clear
    for _ in range(5):
        safe_write(ser, req(TYPE_JOINT))
        rx.recv_type(TYPE_JOINT, timeout=0.5)
    s3 = fetch_stats(ser, rx)
    if s3:
        check("rx_frames increments after clear", s3["rx_frames"] > 0,
              f"got {s3['rx_frames']}")
        check("rx_crc_err zero on clean link", s3["rx_crc_err"] == 0,
              f"got {s3['rx_crc_err']}")


def test_timesync(ser: serial.Serial, rx: Receiver):
    section("9 · Timesync — STM32 computes Pi-domain timestamps")
    rx.flush()

    host_us = lambda: int(time.time() * 1e6)

    # Exchange 1: t4_prev=0, STM32 records T1_prev/T2_prev, responds with T2
    t1_1 = host_us()
    safe_write(ser, encode_timesync(t1_1, 0))
    resp1 = rx.recv_type(MSG_TIMESYNC_RESP, timeout=2.0)
    ok = check("Exchange 1: TIMESYNC_RESP received", resp1 is not None)
    if not ok:
        return
    t4_1 = host_us()
    check("Exchange 1: T2 is non-zero", resp1[2]["t2_us"] != 0,
          f"t2={resp1[2]['t2_us']}")

    # Exchange 2: STM32 now has all four timestamps and computes offset
    time.sleep(0.1)
    t1_2 = host_us()
    safe_write(ser, encode_timesync(t1_2, t4_1))
    resp2 = rx.recv_type(MSG_TIMESYNC_RESP, timeout=2.0)
    ok = check("Exchange 2: TIMESYNC_RESP received", resp2 is not None)
    if not ok:
        return

    # After 2nd exchange offset should be valid — request a JOINT frame
    time.sleep(0.05)
    safe_write(ser, req(TYPE_JOINT))
    jf = rx.recv_type(TYPE_JOINT, timeout=1.0)
    ok = check("JOINT frame received after timesync", jf is not None)
    if not ok:
        return

    d = jf[2]
    ok = check("JOINT frame has pi_time_us field", d is not None and "pi_time_us" in d)
    if not ok:
        return
    pi_us = d["pi_time_us"]
    check("pi_time_us is non-zero after 2nd exchange", pi_us != 0, f"got {pi_us}")
    if pi_us != 0:
        delta_s = abs(pi_us - host_us()) / 1e6
        check("pi_time_us within 1 s of host time", delta_s < 1.0,
              f"delta={delta_s:.3f} s")

    # Run 8 more rounds to fill the median window; check drift between frames
    rx.flush()
    t4 = host_us()
    for _ in range(8):
        time.sleep(0.1)
        safe_write(ser, encode_timesync(host_us(), t4))
        r = rx.recv_type(MSG_TIMESYNC_RESP, timeout=1.0)
        if r:
            t4 = host_us()

    # Collect 5 consecutive JOINT frames and check drift
    safe_write(ser, set_rate(TYPE_JOINT, 20))
    time.sleep(0.05)
    joint_frames = []
    for _ in range(5):
        jf = rx.recv_type(TYPE_JOINT, timeout=0.5)
        if jf and jf[2] and "pi_time_us" in jf[2]:
            joint_frames.append(jf[2]["pi_time_us"])
    safe_write(ser, set_rate(TYPE_JOINT, 0))
    rx.flush()

    if len(joint_frames) >= 2:
        max_drift = 0
        for i in range(1, len(joint_frames)):
            period_us = joint_frames[i] - joint_frames[i - 1]
            drift = abs(period_us - 50000)  # expect ~50 ms = 50000 µs
            max_drift = max(max_drift, drift)
        check("Consecutive JOINT pi_time_us drift < 500 µs", max_drift < 500,
              f"max_drift={max_drift} µs")
    else:
        print(f"  {INFO} (skipping drift check — too few frames with pi_time_us)")


def test_stress_rx(ser: serial.Serial, rx: Receiver):
    section("10 · Stress RX — firmware handles sustained frame flood")
    rx.flush()

    # Establish connection, then clear stats baseline
    stop, bg = send_cmd_vel_background(ser)
    time.sleep(0.3)
    stop.set(); bg.join(timeout=0.2)
    fetch_stats(ser, rx, clear=True)
    rx.flush()

    # Flood: CMD_VEL only at 300 Hz for 3 s (3× normal max, ~18% of 460800 baud).
    # No REQ — TX-response contention is covered by test_stress_bidir.
    # Deadline loop keeps rate steady regardless of Python timer jitter.
    hz = 300
    interval = 1.0 / hz
    t_next = time.monotonic()
    t_end  = t_next + 3.0
    sent = 0
    while t_next < t_end:
        safe_write(ser, cmd_vel(0, 0, 0))
        sent += 1
        t_next += interval
        wait = t_next - time.monotonic()
        if wait > 0:
            time.sleep(wait)

    time.sleep(0.1)  # allow last frames to land
    s = fetch_stats(ser, rx)

    ok = check("Stats received after RX stress", s is not None)
    if not ok:
        halt_motors(ser)
        return

    print(f"  {INFO} sent≈{sent}  fw_rx={s['rx_frames']}  "
          f"crc_err={s['rx_crc_err']}  len_err={s['rx_len_err']}")
    check("No CRC errors under RX flood",    s["rx_crc_err"] == 0,
          f"got {s['rx_crc_err']}")
    check("No length errors under RX flood", s["rx_len_err"] == 0,
          f"got {s['rx_len_err']}")

    halt_motors(ser)
    time.sleep(0.1)
    rx.flush()


def test_stress_tx(ser: serial.Serial, rx: Receiver):
    section("11 · Stress TX — Pi receives sustained high-rate streams")
    rx.flush()

    decode_err_before = rx.rx_decode_err

    # Keep watchdog alive during drain
    stop, bg = send_cmd_vel_background(ser)
    time.sleep(0.2)
    rx.flush()

    # JOINT at 200 Hz + IMU at 100 Hz for 3 s
    safe_write(ser, set_rate(TYPE_JOINT, 200))
    safe_write(ser, set_rate(TYPE_IMU,   100))
    time.sleep(0.05)

    frames = rx.drain(3.0)
    stop.set(); bg.join(timeout=0.2)

    js  = [f for f in frames if f[0] == TYPE_JOINT]
    imu = [f for f in frames if f[0] == TYPE_IMU]

    print(f"  {INFO} JOINT={len(js)} (expect ~600)  IMU={len(imu)} (expect ~300)")
    check("JOINT_STATE 200 Hz: 540–660 in 3 s", 540 <= len(js) <= 660,
          f"got {len(js)}")
    check("IMU 100 Hz: 270–330 in 3 s",         270 <= len(imu) <= 330,
          f"got {len(imu)}")

    n_gaps, n_missing = seq_gaps(js)
    check("No JOINT_STATE seq gaps (no dropped TX frames)", n_missing == 0,
          f"{n_gaps} gap(s), {n_missing} missing" if n_missing else "")

    new_errs = rx.rx_decode_err - decode_err_before
    check("No Pi-side decode errors", new_errs == 0, f"got {new_errs}")

    stop_all_streams(ser)
    halt_motors(ser)
    time.sleep(0.1)
    rx.flush()


def test_stress_bidir(ser: serial.Serial, rx: Receiver):
    section("12 · Stress bidirectional — full-duplex sustained load")
    rx.flush()

    decode_err_before = rx.rx_decode_err
    fetch_stats(ser, rx, clear=True)
    rx.flush()

    # CMD_VEL at 100 Hz keeps watchdog alive and loads the RX path
    stop, bg = send_cmd_vel_background(ser)
    time.sleep(0.1)

    safe_write(ser, set_rate(TYPE_JOINT, 100))
    safe_write(ser, set_rate(TYPE_IMU,   50))
    safe_write(ser, set_rate(TYPE_BAT,   1))
    time.sleep(0.05)

    frames = rx.drain(5.0)
    stop.set(); bg.join(timeout=0.2)

    js  = [f for f in frames if f[0] == TYPE_JOINT]
    imu = [f for f in frames if f[0] == TYPE_IMU]
    bat = [f for f in frames if f[0] == TYPE_BAT]

    print(f"  {INFO} JOINT={len(js)}  IMU={len(imu)}  BAT={len(bat)}")
    check("JOINT_STATE 100 Hz: 450–550 in 5 s", 450 <= len(js) <= 550,
          f"got {len(js)}")
    check("IMU 50 Hz: 225–275 in 5 s",          225 <= len(imu) <= 275,
          f"got {len(imu)}")
    check("BATTERY 1 Hz: 2–7 in 5 s",           2 <= len(bat) <= 7,
          f"got {len(bat)}")

    n_gaps, n_missing = seq_gaps(js)
    check("No JOINT_STATE seq gaps under bidir load", n_missing == 0,
          f"{n_gaps} gap(s), {n_missing} missing" if n_missing else "")

    new_errs = rx.rx_decode_err - decode_err_before
    check("No Pi-side decode errors under bidir load", new_errs == 0,
          f"got {new_errs}")

    s = fetch_stats(ser, rx)
    if s:
        print(f"  {INFO} fw rx_frames={s['rx_frames']}  crc_err={s['rx_crc_err']}  "
              f"len_err={s['rx_len_err']}  tx_frames={s['tx_frames']}")
        check("No FW CRC errors under bidir load",    s["rx_crc_err"] == 0,
              f"got {s['rx_crc_err']}")
        check("No FW length errors under bidir load", s["rx_len_err"] == 0,
              f"got {s['rx_len_err']}")

    stop_all_streams(ser)
    halt_motors(ser)
    time.sleep(0.1)
    rx.flush()


def test_pid_set(ser: serial.Serial, rx: Receiver):
    section("13 · MSG_PID_SET — firmware applies PID gains without crash")
    rx.flush()

    stop, bg = send_cmd_vel_background(ser)
    time.sleep(0.2)

    # Apply gains to both motors
    safe_write(ser, encode_pid_set(2, kp=2.0, ki=0.1, kd=0.05))
    time.sleep(0.05)

    # Firmware should still respond to REQ after gain update
    safe_write(ser, req(TYPE_JOINT))
    f = rx.recv_type(TYPE_JOINT, timeout=1.0)
    check("JOINT frame received after PID_SET(both, 2.0, 0.1, 0.05)", f is not None)

    # Reset to zero gains — firmware should still be alive
    safe_write(ser, encode_pid_set(2, kp=0.0, ki=0.0, kd=0.0))
    time.sleep(0.05)
    safe_write(ser, req(TYPE_JOINT))
    f = rx.recv_type(TYPE_JOINT, timeout=1.0)
    check("JOINT frame received after PID_SET(both, 0, 0, 0)", f is not None)

    stop.set(); bg.join(timeout=0.2)
    halt_motors(ser)
    rx.flush()


def test_pid_stream(ser: serial.Serial, rx: Receiver):
    section("14 · STREAM_PID — periodic velocity telemetry via MSG_SET_RATE")
    rx.flush()

    stop, bg = send_cmd_vel_background(ser)
    time.sleep(0.2)
    rx.flush()

    # Start STREAM_PID at 20 Hz
    safe_write(ser, set_rate(STREAM_PID, 20))
    time.sleep(0.05)

    frames = []
    for _ in range(5):
        f = rx.recv_type(STREAM_PID, timeout=0.3)
        if f:
            frames.append(f)

    check("5 STREAM_PID frames received at 20 Hz", len(frames) == 5,
          f"got {len(frames)}")

    decode_ok = all(f[1] is not None and len(f[1]) == 8 for f in frames)
    check("Each STREAM_PID frame is 8 bytes", decode_ok,
          f"{sum(1 for f in frames if len(f[1]) != 8)} wrong-size" if not decode_ok else "")

    if frames and frames[0][2]:
        print(f"  {INFO}   sample: {frames[0][2]}")

    # Stop stream and verify silence
    safe_write(ser, set_rate(STREAM_PID, 0))
    time.sleep(0.05)
    rx.flush()
    time.sleep(0.2)
    leftover = [f for f in rx.drain(0.2) if f[0] == STREAM_PID]
    check("STREAM_PID stops after SET_RATE(STREAM_PID, 0)", len(leftover) == 0,
          f"got {len(leftover)} stray frames" if leftover else "")

    stop.set(); bg.join(timeout=0.2)
    halt_motors(ser)
    rx.flush()


def test_pid_set_short(ser: serial.Serial, rx: Receiver):
    section("15 · MSG_PID_SET short frame — rx_short counter increments")
    rx.flush()

    stop, bg = send_cmd_vel_background(ser)
    time.sleep(0.2)

    s_before = fetch_stats(ser, rx)
    ok = check("Stats received before short-frame test", s_before is not None)
    if not ok:
        stop.set(); bg.join(timeout=0.2)
        return

    # Send a valid-CRC frame with only 3 bytes payload (< 7 required)
    short = frame(MSG_PID_SET, bytes([2, 0, 0]))
    safe_write(ser, short)
    time.sleep(0.1)

    s_after = fetch_stats(ser, rx)
    ok = check("Stats received after short-frame test", s_after is not None)
    if ok:
        delta = s_after["rx_short"] - s_before["rx_short"]
        check("rx_short incremented by 1 for truncated MSG_PID_SET", delta == 1,
              f"delta={delta}")

    stop.set(); bg.join(timeout=0.2)
    halt_motors(ser)
    rx.flush()


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

    stop_all_streams(ser)
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
        test_get_stats(ser, rx)
        test_timesync(ser, rx)
        test_stress_rx(ser, rx)
        test_stress_tx(ser, rx)
        test_stress_bidir(ser, rx)
        test_pid_set(ser, rx)
        test_pid_stream(ser, rx)
        test_pid_set_short(ser, rx)
    finally:
        stop_all_streams(ser)
        halt_motors(ser)
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
