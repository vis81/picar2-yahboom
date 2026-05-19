#!/usr/bin/env python3
"""
PICAR-2 motor PID auto-tuner.

Three-phase procedure:
  1. Kp sweep at each setpoint  → find critical gain Ku via oscillation detection
  2. Kp = 0.2–0.25 × Ku_min    → stable across full speed range
  3. Ki empirical sweep at median setpoint → stops when steady-state error < 2%

Usage:
    python3 tools/pid_tuner.py                         # full range, both motors
    python3 tools/pid_tuner.py --motor l --dry-run     # left motor, don't apply
    python3 tools/pid_tuner.py --setpoints 200,1000,3000 --detune 0.4
"""

import argparse
import datetime
import json
import queue
import struct
import sys
import threading
import time

import serial

# ── Protocol constants ──────────────────────────────────────────────────────

PROTO_START = 0xAA
TYPE_JOINT  = 0x01
MSG_CMD_VEL  = 0x80
MSG_SET_RATE = 0x82
MSG_PID_SET  = 0x85
STREAM_PID   = 0x06
COMMS_KO           = 100   # gain scale factor; matches DEF_KO in firmware

DEFAULT_KP_SWEEP  = [0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0, 128.0]
DEFAULT_SETPOINTS = [200, 500, 1500, 3000]  # deg/s — covers full motor range
DEFAULT_KI_SWEEP  = [0.2, 0.5, 1.0, 1.5, 2.0, 3.0, 5.0]


# ── CRC-8 Dallas/Maxim (poly 0x31) ──────────────────────────────────────────

def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


# ── Frame encoding ───────────────────────────────────────────────────────────

def frame(msg_type: int, payload: bytes = b"") -> bytes:
    hdr = bytes([PROTO_START, msg_type, len(payload)])
    return hdr + payload + bytes([crc8(bytes([msg_type, len(payload)]) + payload)])

def cmd_vel(left: int, right: int, steering: int = 50) -> bytes:
    return frame(MSG_CMD_VEL, struct.pack("<hhB", left, right, steering))

def set_rate(stream_id: int, hz: int) -> bytes:
    return frame(MSG_SET_RATE, struct.pack("<BH", stream_id, hz))

def encode_pid_set(motor_id: int, kp: float, ki: float, kd: float) -> bytes:
    return frame(MSG_PID_SET, struct.pack("<Bhhh",
        motor_id, int(kp * COMMS_KO), int(ki * COMMS_KO), int(kd * COMMS_KO)))

# ── Frame decoding ───────────────────────────────────────────────────────────

def decode_joint(payload: bytes) -> dict:
    enc_l, enc_r, steer, seq = struct.unpack("<iiBB", payload[:10])
    d = {"enc_left": enc_l, "enc_right": enc_r, "steering": steer, "seq": seq}
    if len(payload) >= 14:
        vel_l, vel_r = struct.unpack_from("<hh", payload, 10)
        d["vel_left"] = vel_l
        d["vel_right"] = vel_r
    return d


# ── Receiver thread ──────────────────────────────────────────────────────────

class Receiver(threading.Thread):
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
                if self._type == TYPE_JOINT:
                    try:
                        decoded = decode_joint(bytes(self._buf))
                    except Exception:
                        pass
                self.q.put((self._type, bytes(self._buf), decoded))
            self._state = self._S_START

    def flush(self):
        while not self.q.empty():
            try:
                self.q.get_nowait()
            except queue.Empty:
                break


# ── Watchdog keepalive ───────────────────────────────────────────────────────

class Keepalive:
    """Sends CMD_VEL at 10 Hz to keep the STM32 watchdog connection alive."""

    def __init__(self, ser: serial.Serial):
        self._ser = ser
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._vel_l = self._vel_r = 0
        self._t = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._t.start()

    def set_vel(self, left: int, right: int):
        with self._lock:
            self._vel_l, self._vel_r = left, right

    def stop(self):
        self._stop.set()
        self._t.join(timeout=0.5)

    def _run(self):
        while not self._stop.is_set():
            with self._lock:
                l, r = self._vel_l, self._vel_r
            self._ser.write(cmd_vel(l, r))
            time.sleep(0.1)


# ── Oscillation detection ────────────────────────────────────────────────────

def detect_oscillation(velocities: list, times: list, setpoint: float,
                       min_crossings: int = 4, window: float = 4.0,
                       min_amp_ratio: float = 0.05,
                       min_half_period: float = 0.08) -> tuple:
    """
    Detect oscillation via signed zero-crossings of (velocity - setpoint).

    Uses the full `window` seconds of the trace so slow oscillations
    (< 2 Hz) are not missed.  Returns (oscillating: bool, Tu_seconds: float).
    """
    if not times or abs(setpoint) < 1:
        return False, 0.0, 0

    t_end = times[-1]
    pairs = [(v, t) for v, t in zip(velocities, times) if t >= t_end - window]
    if len(pairs) < max(4, min_crossings * 2):
        return False, 0.0, 0

    v_dev = [v - setpoint for v, t in pairs]
    t_win = [t for _, t in pairs]

    # Require meaningful oscillation amplitude (not just sensor noise)
    amp = (max(v_dev) - min(v_dev)) / 2.0
    if amp < abs(setpoint) * min_amp_ratio:
        return False, 0.0, 0

    # Find zero crossings with linear interpolation; suppress noise re-crossings
    crossings = []
    for i in range(1, len(v_dev)):
        if v_dev[i - 1] * v_dev[i] < 0:
            frac = abs(v_dev[i - 1]) / (abs(v_dev[i - 1]) + abs(v_dev[i]))
            t_cross = t_win[i - 1] + frac * (t_win[i] - t_win[i - 1])
            if not crossings or t_cross - crossings[-1] >= min_half_period:
                crossings.append(t_cross)

    if len(crossings) < min_crossings:
        return False, 0.0, len(crossings)

    # Period = 2 × mean half-period; remove obvious outliers first
    half_periods = [crossings[i + 1] - crossings[i]
                    for i in range(len(crossings) - 1)]
    if half_periods:
        med = sorted(half_periods)[len(half_periods) // 2]
        half_periods = [p for p in half_periods if 0 < p <= 3.0 * med]
    if not half_periods:
        return False, 0.0, len(crossings)

    tu = 2.0 * sum(half_periods) / len(half_periods)
    return True, tu, len(crossings)


# ── Step response collection ─────────────────────────────────────────────────

def run_step(rx: Receiver, keepalive: Keepalive,
             motor_id: int, setpoint: int, duration: float = 4.0):
    """Run a step to setpoint deg/s, collect velocity samples. Returns (velocities, times)."""
    rx.flush()
    vel_l = setpoint if motor_id in (0, 2) else 0
    vel_r = setpoint if motor_id in (1, 2) else 0
    keepalive.set_vel(vel_l, vel_r)

    velocities, times = [], []
    t_start = time.monotonic()
    deadline = t_start + duration
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        try:
            item = rx.q.get(timeout=max(0.01, remaining))
            if item[0] == TYPE_JOINT and item[2]:
                d = item[2]
                if motor_id == 1:
                    vel = float(d.get("vel_right", 0))
                elif motor_id == 2:
                    vel = (float(d.get("vel_left", 0)) + float(d.get("vel_right", 0))) / 2.0
                else:
                    vel = float(d.get("vel_left", 0))
                velocities.append(vel)   # signed — needed for oscillation detection
                times.append(time.monotonic() - t_start)
        except queue.Empty:
            pass

    keepalive.set_vel(0, 0)
    # Wait until motor actually stops, not just a fixed delay.
    # Poll velocity until it drops below 50 deg/s or 3 s have elapsed.
    t_stop = time.monotonic()
    while time.monotonic() - t_stop < 3.0:
        try:
            item = rx.q.get(timeout=0.12)
            if item[0] == TYPE_JOINT and item[2]:
                d = item[2]
                vl = abs(float(d.get("vel_left",  0)))
                vr = abs(float(d.get("vel_right", 0)))
                if max(vl, vr) < 50:
                    break
        except queue.Empty:
            pass
    time.sleep(0.1)
    rx.flush()
    return velocities, times


# ── Step response metrics ────────────────────────────────────────────────────

def step_metrics(velocities: list, times: list, setpoint: float) -> dict:
    if not velocities:
        return {}
    target = abs(setpoint)
    abs_vels = [abs(v) for v in velocities]
    t10 = t90 = None
    for v, t in zip(abs_vels, times):
        if t10 is None and v >= 0.1 * target:
            t10 = t
        if t90 is None and v >= 0.9 * target:
            t90 = t
    rise_time = round(t90 - t10, 3) if (t10 is not None and t90 is not None) else None

    peak = max(abs_vels)
    overshoot_pct = round((peak - target) / target * 100, 1) if target else 0.0

    band = 0.05 * target
    settling = None
    velocities = abs_vels  # use abs for settling check
    for v, t in reversed(list(zip(velocities, times))):
        if abs(v - target) > band:
            settling = round(t, 3)
            break

    return {
        "rise_time_s":     rise_time,
        "overshoot_pct":   overshoot_pct,
        "settling_time_s": settling,
    }


# ── Plotting ─────────────────────────────────────────────────────────────────

def plot_step(times: list, velocities: list, setpoint: int, label: str):
    try:
        import matplotlib.pyplot as plt
        plt.figure(figsize=(8, 3))
        plt.plot(times, velocities, label="actual (signed)")
        plt.axhline(setpoint, color="r", linestyle="--", label="setpoint")
        plt.title(label)
        plt.xlabel("time (s)")
        plt.ylabel("velocity (deg/s)")
        plt.legend()
        plt.tight_layout()
        plt.show()
    except ImportError:
        print("    (matplotlib not available — skipping plot)")


# ── Ki empirical search ───────────────────────────────────────────────────────

def find_best_ki(ser: serial.Serial, rx: Receiver, keepalive: Keepalive,
                 motor_id: int, kp: float, setpoint: int,
                 ki_candidates: list, max_ss_err_pct: float = 2.0,
                 max_osc_range_pct: float = 15.0) -> float:
    """
    Step through ki_candidates at fixed kp.  Accept the largest Ki where:
      - steady-state error (last 10 samples vs setpoint) < max_ss_err_pct
      - oscillation range in last 1 s < max_osc_range_pct of setpoint
    Returns the chosen Ki.
    """
    best_ki = ki_candidates[0]
    for ki in ki_candidates:
        ser.write(encode_pid_set(motor_id, kp, ki, 0.0))
        time.sleep(0.05)
        velocities, times = run_step(rx, keepalive, motor_id, setpoint, duration=4.0)
        if not velocities:
            continue

        abs_vels = [abs(v) for v in velocities]
        ss_vel   = sum(abs_vels[-10:]) / 10
        ss_err   = abs(ss_vel - setpoint) / setpoint * 100

        tail = [v for v, t in zip(abs_vels, times) if t >= times[-1] - 1.0]
        osc_range = (max(tail) - min(tail)) / setpoint * 100 if len(tail) > 2 else 0.0

        status = "osc!" if osc_range > max_osc_range_pct else f"ss_err={ss_err:.1f}%"
        print(f"    Ki={ki:.2f}  {status}  osc_range={osc_range:.1f}%", end="")

        if osc_range > max_osc_range_pct:
            print(f"  → oscillating, keeping Ki={best_ki:.2f}")
            break

        best_ki = ki
        if ss_err < max_ss_err_pct:
            print(f"  ✓ accepted")
            break
        print()

    return best_ki


# ── Per-motor tuning ─────────────────────────────────────────────────────────

def tune_motor(ser: serial.Serial, rx: Receiver, keepalive: Keepalive,
               motor_id: int, motor_label: str,
               setpoints: list, kp_sweep: list, ki_sweep: list,
               dry_run: bool, do_plot: bool,
               method: str = "tyreus-luyben", detune: float = 1.0,
               with_kd: bool = False) -> dict:

    print(f"\n{'=' * 62}")
    print(f"  Motor: {motor_label}   setpoints: {setpoints} deg/s")
    print(f"{'=' * 62}")

    ser.write(set_rate(TYPE_JOINT, 50))
    time.sleep(0.1)
    rx.flush()

    # ── Phase 1: Kp sweep at each setpoint to find Ku ────────────────────────
    per_speed: dict = {}  # setpoint → {"Ku": float, "Tu": float}

    for sp in setpoints:
        print(f"\n  ── Kp sweep @ {sp} deg/s ──────────────────────────────")
        ku = tu = None

        for kp in kp_sweep:
            print(f"    Kp={kp:5.1f}  ...", end=" ", flush=True)
            ser.write(encode_pid_set(motor_id, kp, 0.0, 0.0))
            time.sleep(0.05)

            velocities, times = run_step(rx, keepalive, motor_id, sp, duration=4.0)
            osc, period, n_cross = detect_oscillation(velocities, times, sp)

            if osc:
                print(f"OSCILLATION  Tu={period:.3f} s  ({n_cross} crossings)")
                ku, tu = kp, period
                break
            else:
                final = abs(velocities[-1]) if velocities else 0
                peak  = max((abs(v) for v in velocities), default=0)
                print(f"stable  final={final:.0f}  peak={peak:.0f}  crossings={n_cross}")

        if ku is not None:
            per_speed[sp] = {"Ku": ku, "Tu": tu}
            print(f"    → Ku={ku}  Tu={tu:.3f} s")
        else:
            print(f"    → no oscillation up to Kp={kp_sweep[-1]}")

    if not per_speed:
        print(f"\n  ERROR: No Ku found. Check motor responds or raise --kp-sweep max.")
        ser.write(set_rate(TYPE_JOINT, 0))
        return {}

    # ── Phase 2: Kp from Ku (Tyreus-Luyben / ZN formula) ────────────────────
    ku_min = min(v["Ku"] for v in per_speed.values())
    if method == "tyreus-luyben":
        kp_pid = 0.454 * ku_min * detune
    else:  # zn
        kp_pid = 0.6   * ku_min * detune
    print(f"\n  Ku_min={ku_min}  →  Kp={kp_pid:.4f}  "
          f"({method}, ×{detune} detune)")

    # ── Phase 3: Ki empirical sweep at median setpoint ───────────────────────
    mid_sp = sorted(setpoints)[len(setpoints) // 2]
    print(f"\n  Ki sweep at {mid_sp} deg/s (Kp={kp_pid:.4f}):")
    ki_pid = find_best_ki(ser, rx, keepalive, motor_id,
                          kp_pid, mid_sp, ki_sweep)
    kd_pid = 0.0  # derivative not used for velocity PIDs
    print(f"\n  Final gains:  Kp={kp_pid:.4f}  Ki={ki_pid:.4f}  Kd={kd_pid:.4f}")

    # ── Phase 4: Apply and verify ─────────────────────────────────────────────
    verify: dict = {}
    if not dry_run:
        ser.write(encode_pid_set(motor_id, kp_pid, ki_pid, kd_pid))
        time.sleep(0.1)
        print("\n  Verification (4 s step each):")
        for sp in setpoints:
            velocities, times = run_step(rx, keepalive, motor_id, sp, duration=4.0)
            m = step_metrics(velocities, times, sp)
            verify[sp] = m
            ss_vel = sum(abs(v) for v in velocities[-10:]) / 10 if velocities else 0
            ss_err = abs(ss_vel - sp) / sp * 100 if sp else 0
            rise = m.get('rise_time_s')
            print(f"    {sp:5d} deg/s → "
                  f"rise={'?' if rise is None else rise:>5}s  "
                  f"peak={m.get('overshoot_pct', '?'):>6}%  "
                  f"ss_err={ss_err:.1f}%")
            if do_plot:
                plot_step(times, velocities, sp, f"Motor {motor_label}  {sp} deg/s")
    else:
        print("  [dry-run] Gains NOT applied.")

    ser.write(set_rate(TYPE_JOINT, 0))
    return {
        "motor":    motor_label,
        "method":   method,
        "detune":   detune,
        "ku_min":   ku_min,
        "per_speed": {str(k): v for k, v in per_speed.items()},
        "kp":       round(kp_pid, 4),
        "ki":       round(ki_pid, 4),
        "kd":       round(kd_pid, 4),
        "verify":   verify,
    }


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="PICAR-2 motor PID auto-tuner")
    parser.add_argument("--port",      default="/dev/ttyYahboom0")
    parser.add_argument("--baud",      type=int, default=460800)
    parser.add_argument("--motor",     choices=["l", "r", "both"], default="both",
                        help="Motor(s) to tune (default: both)")
    parser.add_argument("--setpoints", default=",".join(str(s) for s in DEFAULT_SETPOINTS),
                        help="Comma-separated target speeds in deg/s (default covers full range)")
    parser.add_argument("--kp-sweep",  default=",".join(str(k) for k in DEFAULT_KP_SWEEP),
                        help="Kp values to sweep when searching for Ku")
    parser.add_argument("--ki-sweep",  default=",".join(str(k) for k in DEFAULT_KI_SWEEP),
                        help="Ki candidates to test empirically (stops when ss_err < 2%%)")
    parser.add_argument("--method",    choices=["zn", "tyreus-luyben"], default="tyreus-luyben",
                        help="Kp formula: tyreus-luyben (default) or zn")
    parser.add_argument("--detune",    type=float, default=0.5,
                        help="Scale Kp by this factor for stability margin (default 0.5)")
    parser.add_argument("--dry-run",   action="store_true",
                        help="Compute gains but do not apply them")
    parser.add_argument("--plot",      action="store_true",
                        help="Plot step responses (requires matplotlib)")
    args = parser.parse_args()

    setpoints = [int(x)   for x in args.setpoints.split(",")]
    kp_sweep  = [float(x) for x in args.kp_sweep.split(",")]
    ki_sweep  = [float(x) for x in args.ki_sweep.split(",")]
    motors_to_tune = {
        "l":    [(0, "LEFT")],
        "r":    [(1, "RIGHT")],
        "both": [(0, "LEFT"), (1, "RIGHT")],
    }[args.motor]

    print(f"\nPICAR-2 Motor PID Auto-Tuner — {args.port} @ {args.baud} baud")
    if args.dry_run:
        print("  [dry-run — gains will NOT be applied]")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"Cannot open {args.port}: {e}")
        sys.exit(1)

    rx = Receiver(ser)
    rx.start()
    time.sleep(0.2)
    rx.flush()

    keepalive = Keepalive(ser)
    keepalive.start()
    time.sleep(0.5)  # establish watchdog connection

    results = {}
    try:
        for motor_id, motor_label in motors_to_tune:
            result = tune_motor(
                ser, rx, keepalive,
                motor_id, motor_label,
                setpoints, kp_sweep, ki_sweep,
                args.dry_run, args.plot,
                method=args.method, detune=args.detune,
            )
            if result:
                results[motor_label] = result
    finally:
        keepalive.set_vel(0, 0)
        time.sleep(0.1)
        keepalive.stop()
        ser.write(set_rate(TYPE_JOINT, 0))
        time.sleep(0.05)
        ser.close()

    if results:
        ts    = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        fname = f"pid_gains_{args.motor}_{ts}.json"
        with open(fname, "w") as f:
            json.dump(results, f, indent=2)
        print(f"\n  Results saved to {fname}")

    print()


if __name__ == "__main__":
    main()
