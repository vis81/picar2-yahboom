#!/usr/bin/env python3
"""PICAR-2 GUI Controller — manual drive and telemetry viewer.

Usage:
    python3 tools/picar2_gui.py
    python3 tools/picar2_gui.py --port /dev/ttyUSB0 --baud 460800

Controls:
    Left / Right sliders : wheel velocity in mm/s (-3000 … +3000)
    Steer slider         : servo position (0 … 100)
    CMD_VEL rate         : how often the current slider values are sent
    Stream rate dropdowns: ask the STM32 to push data at that Hz
    Double-click a motor slider to zero it.
"""

import argparse
import collections
import queue
import struct
import threading
import time
import tkinter as tk
from tkinter import ttk

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    raise SystemExit("pyserial not found — run: pip install pyserial")


# ── Protocol constants ────────────────────────────────────────────────────────

PROTO_START = 0xAA

TYPE_JOINT = 0x01
TYPE_IMU   = 0x02
TYPE_BAT   = 0x03

MSG_CMD_VEL       = 0x80
MSG_REQ           = 0x81
MSG_SET_RATE      = 0x82
MSG_GET_STATS     = 0x83
MSG_TIMESYNC      = 0x84
MSG_TIMESYNC_RESP = 0x05

TYPE_STATS = 0x04

STREAM_NAMES  = {TYPE_JOINT: "JOINT_STATE", TYPE_IMU: "IMU", TYPE_BAT: "BATTERY"}
STREAM_LABELS = {TYPE_JOINT: "JOINT",       TYPE_IMU: "IMU", TYPE_BAT: "BAT  "}

RATE_OPTIONS_STREAM = ["0", "1", "5", "10", "20", "50", "100", "200"]
RATE_OPTIONS_CMD    = ["0", "1", "5", "10", "20", "50", "100"]


# ── CRC-8 Dallas/Maxim (poly 0x31) ───────────────────────────────────────────

def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


# ── Frame encoding ────────────────────────────────────────────────────────────

def _frame(msg_type: int, payload: bytes = b"") -> bytes:
    hdr = bytes([PROTO_START, msg_type, len(payload)])
    return hdr + payload + bytes([crc8(bytes([msg_type, len(payload)]) + payload)])

def enc_cmd_vel(left_mms: int, right_mms: int, steering: int) -> bytes:
    return _frame(MSG_CMD_VEL, struct.pack("<hhB", left_mms, right_mms, steering))

def enc_req(stream_id: int) -> bytes:
    return _frame(MSG_REQ, bytes([stream_id]))

def enc_set_rate(stream_id: int, hz: int) -> bytes:
    return _frame(MSG_SET_RATE, struct.pack("<BH", stream_id, hz))

def enc_get_stats(clear: bool = False) -> bytes:
    return _frame(MSG_GET_STATS, bytes([1]) if clear else b"")

def enc_timesync(t1_us: int, t4_prev_us: int = 0) -> bytes:
    return _frame(MSG_TIMESYNC, struct.pack("<qq", t1_us, t4_prev_us))


# ── Frame decoding ────────────────────────────────────────────────────────────

def decode_joint(payload: bytes) -> dict:
    enc_l, enc_r, steer, seq = struct.unpack("<iiBB", payload[:10])
    d = {"enc_left": enc_l, "enc_right": enc_r, "steering": steer, "seq": seq}
    if len(payload) >= 14:
        vel_l, vel_r = struct.unpack_from("<hh", payload, 10)
        d["vel_left"] = vel_l
        d["vel_right"] = vel_r
    if len(payload) >= 22:
        pi_time_us, = struct.unpack_from("<q", payload, 14)
        d["pi_time_us"] = pi_time_us
    return d

def decode_imu(payload: bytes) -> dict:
    v = struct.unpack("<10h", payload[:20])
    return {
        "accel": [x * 0.001 for x in v[0:3]],
        "gyro":  [x * 0.001 for x in v[3:6]],
        "magn":  [x * 0.1   for x in v[6:9]],
        "temp":  v[9] * 0.01,
    }

def decode_bat(payload: bytes) -> dict:
    mv, pct, _ = struct.unpack("<HBB", payload[:4])
    return {"voltage_mv": mv, "charge_pct": pct}

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

def decode_timesync_resp(payload: bytes) -> dict:
    t2_us, = struct.unpack("<q", payload[:8])
    return {"t2_us": t2_us}

DECODERS = {TYPE_JOINT: decode_joint, TYPE_IMU: decode_imu,
            TYPE_BAT: decode_bat, TYPE_STATS: decode_stats,
            MSG_TIMESYNC_RESP: decode_timesync_resp}


# ── Receiver thread ───────────────────────────────────────────────────────────

class Receiver(threading.Thread):
    _S_START, _S_TYPE, _S_LEN, _S_PAYLOAD, _S_CRC = range(5)

    def __init__(self, ser: serial.Serial):
        super().__init__(daemon=True)
        self.ser = ser
        self.q: queue.Queue = queue.Queue()
        self._state = self._S_START
        self._type = self._len = 0
        self._buf = bytearray()
        self.rx_frames      = 0
        self.rx_crc_err     = 0
        self.rx_len_err     = 0
        self.rx_decode_err  = 0

    def clear_stats(self):
        self.rx_frames = self.rx_crc_err = self.rx_len_err = self.rx_decode_err = 0

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
                self.rx_len_err += 1
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
                if self._type == MSG_TIMESYNC_RESP and decoded is not None:
                    decoded['rx_time_us'] = int(time.time() * 1e6)
                self.q.put((self._type, bytes(self._buf), decoded))
            else:
                self.rx_crc_err += 1
            self._state = self._S_START


# ── Application ───────────────────────────────────────────────────────────────

class App(tk.Tk):
    def __init__(self, default_port: str, default_baud: int):
        super().__init__()
        self.title("PICAR-2 Controller")
        self.resizable(False, False)

        self._ser: serial.Serial | None = None
        self._rx:  Receiver | None = None
        self._tx_lock = threading.Lock()

        self._send_active = False
        self._sender_stop = threading.Event()
        self._sender_thread: threading.Thread | None = None

        self._rate_dq = {t: collections.deque(maxlen=50) for t in STREAM_NAMES}

        self._tx_cmd_vel  = 0
        self._tx_req      = 0
        self._tx_set_rate = 0
        self._rx_stream   = {t: 0 for t in STREAM_NAMES}
        self._last_fw_stats_req = 0.0

        # Timesync state
        self._sync_t1_us      = 0
        self._sync_last_t4_us = 0
        self._sync_count      = 0
        self._sync_offsets    = [0] * 8
        self._sync_hist_idx   = 0
        self._sync_valid      = False
        self._sync_offset_us  = 0
        self._sync_jitter_us  = 0
        self._sync_last_rx_ms = 0.0
        self._sync_poll_cycle = 0
        self._sync_last_raw    = None   # previous raw offset, for diff tracking
        self._sync_diffs       : list[int] = []
        self._sync_last_diff_us = 0
        self._sync_avg_diff_us  = 0
        self._sync_max_diff_us  = 0
        self._sync_period_var   = tk.StringVar(value="1")

        self._build_ui(default_port, default_baud)
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self._poll()

    # ── UI construction ───────────────────────────────────────────────────────

    def _build_ui(self, default_port: str, default_baud: int):
        self._build_connection_bar(default_port, default_baud)

        body = ttk.Frame(self)
        body.pack(fill="both", expand=True, padx=8, pady=(0, 4))
        self._build_control_panel(body)
        self._build_streams_panel(body)

        self._build_stats_panel()

    def _build_stats_panel(self):
        sf = ttk.LabelFrame(self, text="Comms stats")
        sf.pack(fill="x", padx=8, pady=(0, 8))

        def _lbl(parent, text, col, row=0, width=None):
            kw = {"width": width} if width else {}
            l = ttk.Label(parent, text=text, **kw)
            l.grid(row=row, column=col, sticky="w", padx=(6, 2), pady=3)
            return l

        row1 = ttk.Frame(sf)
        row1.pack(fill="x")
        row2 = ttk.Frame(sf)
        row2.pack(fill="x")

        _lbl(row1, "rx frames:",    0); self._stat_rx_frames     = _lbl(row1, "0",  1, width=8)
        _lbl(row1, "crc_err:",      2); self._stat_rx_crc_err    = _lbl(row1, "0",  3, width=6)
        _lbl(row1, "len_err:",      4); self._stat_rx_len_err    = _lbl(row1, "0",  5, width=6)
        _lbl(row1, "decode_err:",   6); self._stat_rx_decode_err = _lbl(row1, "0",  7, width=6)
        _lbl(row1, "rx joint:",     8); self._stat_tx_joint      = _lbl(row1, "0",  9, width=8)
        _lbl(row1, "rx imu:",      10); self._stat_tx_imu        = _lbl(row1, "0", 11, width=8)
        _lbl(row1, "rx battery:",  12); self._stat_tx_bat        = _lbl(row1, "0", 13, width=8)

        _lbl(row2, "tx cmd_vel:",  0); self._stat_tx_cmd_vel  = _lbl(row2, "0", 1, width=8)
        _lbl(row2, "tx req:",      2); self._stat_tx_req      = _lbl(row2, "0", 3, width=6)
        _lbl(row2, "tx set_rate:", 4); self._stat_tx_set_rate = _lbl(row2, "0", 5, width=6)

        ttk.Button(sf, text="Clear", command=self._clear_stats).pack(
            side="right", padx=8, pady=4)

        ttk.Separator(sf, orient="horizontal").pack(fill="x", padx=6, pady=(2, 0))

        fw = ttk.LabelFrame(sf, text="Firmware (STM32)")
        fw.pack(fill="x", padx=6, pady=(4, 4))

        fw1 = ttk.Frame(fw)
        fw1.pack(fill="x")

        def _fw(parent, text, col, row=0, width=None):
            kw = {"width": width} if width else {}
            l = ttk.Label(parent, text=text, **kw)
            l.grid(row=row, column=col, sticky="w", padx=(6, 2), pady=2)
            return l

        _fw(fw1, "rx frames:",  0); self._fw_rx_frames  = _fw(fw1, "—",  1, width=8)
        _fw(fw1, "crc_err:",    2); self._fw_rx_crc_err = _fw(fw1, "—",  3, width=6)
        _fw(fw1, "len_err:",    4); self._fw_rx_len_err = _fw(fw1, "—",  5, width=6)
        _fw(fw1, "short:",      6); self._fw_rx_short   = _fw(fw1, "—",  7, width=6)
        _fw(fw1, "unknown:",    8); self._fw_rx_unknown = _fw(fw1, "—",  9, width=6)
        _fw(fw1, "tx frames:", 10); self._fw_tx_frames  = _fw(fw1, "—", 11, width=8)

        ttk.Separator(sf, orient="horizontal").pack(fill="x", padx=6, pady=(4, 0))

        ts = ttk.LabelFrame(sf, text="Timesync")
        ts.pack(fill="x", padx=6, pady=(4, 4))
        ts1 = ttk.Frame(ts)
        ts1.pack(fill="x")

        def _ts(parent, text, col, row=0, width=None):
            kw = {"width": width} if width else {}
            l = ttk.Label(parent, text=text, **kw)
            l.grid(row=row, column=col, sticky="w", padx=(6, 2), pady=2)
            return l

        _ts(ts1, "count:",   0); self._ts_count  = _ts(ts1, "0",  1, width=6)
        _ts(ts1, "valid:",   2)
        self._ts_valid = tk.Label(ts1, text="no", fg="gray", width=4, anchor="w")
        self._ts_valid.grid(row=0, column=3, sticky="w", padx=(0, 6), pady=2)
        _ts(ts1, "jitter µs:", 4); self._ts_jitter = _ts(ts1, "—", 5, width=8)
        _ts(ts1, "age ms:",    6); self._ts_age    = _ts(ts1, "—", 7, width=8)
        _ts(ts1, "period:",    8)
        ttk.Combobox(ts1, textvariable=self._sync_period_var, width=4,
                     values=["0.5", "1", "2", "5", "10"]).grid(
            row=0, column=9, padx=2, pady=2)
        _ts(ts1, "s", 10)

        _ts(ts1, "Δ last µs:", 0, row=1); self._ts_last_diff = _ts(ts1, "—", 1, row=1, width=8)
        _ts(ts1, "Δ avg µs:",  2, row=1); self._ts_avg_diff  = _ts(ts1, "—", 3, row=1, width=8)
        _ts(ts1, "Δ max µs:",  4, row=1); self._ts_max_diff  = _ts(ts1, "—", 5, row=1, width=8)


    def _clear_stats(self):
        if self._rx:
            self._rx.clear_stats()
        self._tx_cmd_vel = self._tx_req = self._tx_set_rate = 0
        self._rx_stream = {t: 0 for t in STREAM_NAMES}
        for dq in self._rate_dq.values():
            dq.clear()
        if self._ser:
            self._write(enc_get_stats(clear=True))

    def _build_connection_bar(self, default_port: str, default_baud: int):
        bar = ttk.LabelFrame(self, text="Connection")
        bar.pack(fill="x", padx=8, pady=(8, 4))

        ttk.Label(bar, text="Port:").grid(row=0, column=0, padx=(6, 2), pady=4)
        self._port_var = tk.StringVar(value=default_port)
        port_cb = ttk.Combobox(bar, textvariable=self._port_var, width=22)
        port_cb["values"] = self._list_ports(default_port)
        port_cb.grid(row=0, column=1, padx=2, pady=4)

        ttk.Label(bar, text="Baud:").grid(row=0, column=2, padx=(8, 2))
        self._baud_var = tk.StringVar(value=str(default_baud))
        ttk.Combobox(bar, textvariable=self._baud_var, width=9,
                     values=["115200", "460800", "921600"]).grid(row=0, column=3, padx=2)

        self._conn_btn = ttk.Button(bar, text="Connect", command=self._toggle_connect)
        self._conn_btn.grid(row=0, column=4, padx=(8, 4))

        self._status_lbl = tk.Label(bar, text="● DISCONNECTED", fg="gray",
                                    font=("", 10, "bold"))
        self._status_lbl.grid(row=0, column=5, padx=(8, 8))

    def _build_control_panel(self, parent):
        ctrl = ttk.LabelFrame(parent, text="Control")
        ctrl.pack(side="left", fill="y", padx=(0, 4), pady=4)

        # Motor / steering sliders
        self._left_var  = tk.IntVar(value=0)
        self._right_var = tk.IntVar(value=0)
        self._steer_var = tk.IntVar(value=50)

        self._make_slider(ctrl, "Left (mm/s)",   self._left_var,  -3000, 3000, row=0)
        self._make_slider(ctrl, "Right (mm/s)",  self._right_var, -3000, 3000, row=1)
        self._make_slider(ctrl, "Steer (0–100)", self._steer_var,     0,  100, row=2)

        ttk.Separator(ctrl, orient="horizontal").grid(
            row=3, column=0, columnspan=3, sticky="ew", pady=(6, 4))

        # CMD_VEL rate + on/off
        rf = ttk.Frame(ctrl)
        rf.grid(row=4, column=0, columnspan=3, padx=6, pady=2, sticky="w")
        ttk.Label(rf, text="CMD_VEL rate:").pack(side="left")
        self._cmd_rate_var = tk.StringVar(value="10")
        ttk.Combobox(rf, textvariable=self._cmd_rate_var, width=5,
                     values=RATE_OPTIONS_CMD).pack(side="left", padx=4)
        ttk.Label(rf, text="Hz").pack(side="left")

        self._send_btn = ttk.Button(ctrl, text="Send CMD_VEL  [ OFF ]",
                                    command=self._toggle_send, width=24)
        self._send_btn.grid(row=5, column=0, columnspan=3, padx=6, pady=4, sticky="ew")

        ttk.Separator(ctrl, orient="horizontal").grid(
            row=6, column=0, columnspan=3, sticky="ew", pady=(4, 6))

        ttk.Label(ctrl, text="Set stream rates:", font=("", 9, "bold")).grid(
            row=7, column=0, columnspan=3, sticky="w", padx=6, pady=(0, 4))

        self._stream_rate_vars: dict[int, tk.StringVar] = {}
        defaults = {TYPE_JOINT: "100", TYPE_IMU: "200", TYPE_BAT: "1"}
        for i, (sid, lbl) in enumerate(STREAM_LABELS.items()):
            row = 8 + i
            ttk.Label(ctrl, text=f"{lbl}:").grid(
                row=row, column=0, sticky="e", padx=(6, 2), pady=2)
            v = tk.StringVar(value=defaults[sid])
            self._stream_rate_vars[sid] = v
            ttk.Combobox(ctrl, textvariable=v, width=5,
                         values=RATE_OPTIONS_STREAM).grid(row=row, column=1, padx=2)
            ttk.Button(ctrl, text="Set", width=4,
                       command=lambda s=sid: self._set_stream_rate(s)).grid(
                row=row, column=2, padx=(2, 6))

        ttk.Button(ctrl, text="Set All", command=self._set_all_rates).grid(
            row=11, column=0, columnspan=3, padx=6, pady=(6, 4), sticky="ew")

    def _make_slider(self, parent, label: str, var: tk.IntVar,
                     lo: int, hi: int, row: int):
        ttk.Label(parent, text=label).grid(
            row=row, column=0, sticky="e", padx=(6, 2), pady=3)
        sl = ttk.Scale(parent, from_=lo, to=hi, orient="horizontal",
                       variable=var, length=200,
                       command=lambda v: var.set(int(float(v))))
        sl.grid(row=row, column=1, padx=4, pady=3)
        val_lbl = ttk.Label(parent, text="0", width=6, anchor="e")
        val_lbl.grid(row=row, column=2, padx=(2, 6))
        var.trace_add("write",
                      lambda *_, v=var, l=val_lbl: l.config(text=str(v.get())))
        if lo < 0:
            sl.bind("<Double-Button-1>", lambda _e, v=var: v.set(0))

    def _build_streams_panel(self, parent):
        outer = ttk.LabelFrame(parent, text="Streams")
        outer.pack(side="left", fill="both", expand=True, pady=4)

        self._stream_widgets: dict[int, tuple[tk.Label, tk.Label]] = {}
        for sid, name in STREAM_NAMES.items():
            frm = ttk.LabelFrame(outer, text=name)
            frm.pack(fill="x", padx=6, pady=(4, 2))

            rate_lbl = tk.Label(frm, text="rate: —", fg="gray", anchor="w")
            rate_lbl.pack(fill="x", padx=6, pady=(2, 0))

            data_lbl = tk.Label(frm, text="—", font=("Courier", 9),
                                justify="left", anchor="w")
            data_lbl.pack(fill="x", padx=6, pady=(0, 4))

            self._stream_widgets[sid] = (rate_lbl, data_lbl)

    # ── Connection ────────────────────────────────────────────────────────────

    @staticmethod
    def _list_ports(default: str) -> list[str]:
        try:
            ports = [p.device for p in serial.tools.list_ports.comports()]
        except Exception:
            ports = []
        if default not in ports:
            ports.insert(0, default)
        return ports

    def _toggle_connect(self):
        if self._ser is None:
            self._connect()
        else:
            self._disconnect()

    def _connect(self):
        try:
            self._ser = serial.Serial(
                self._port_var.get(), int(self._baud_var.get()), timeout=0.1)
        except serial.SerialException as e:
            self._status_lbl.config(text=f"Error: {e}", fg="red")
            self._ser = None
            return
        self._rx = Receiver(self._ser)
        self._rx.start()
        for dq in self._rate_dq.values():
            dq.clear()
        self._tx_cmd_vel = self._tx_req = self._tx_set_rate = 0
        self._rx_stream  = {t: 0 for t in STREAM_NAMES}
        self._sync_t1_us = self._sync_last_t4_us = 0
        self._sync_count = self._sync_hist_idx = 0
        self._sync_offsets = [0] * 8
        self._sync_valid = False
        self._sync_offset_us = self._sync_jitter_us = 0
        self._sync_last_rx_ms = 0.0
        self._sync_poll_cycle = 0
        self._sync_last_raw = None
        self._sync_diffs = []
        self._sync_last_diff_us = self._sync_avg_diff_us = self._sync_max_diff_us = 0
        self._conn_btn.config(text="Disconnect")
        self._status_lbl.config(text="● CONNECTED", fg="green")

    def _disconnect(self):
        self._stop_send()
        ser = self._ser
        self._ser = None
        self._rx  = None
        if ser:
            try:
                with self._tx_lock:
                    for sid in STREAM_NAMES:
                        ser.write(enc_set_rate(sid, 0))
                    ser.write(enc_cmd_vel(0, 0, 50))
                    time.sleep(0.05)
                    ser.close()
            except Exception:
                pass
        self._conn_btn.config(text="Connect")
        self._status_lbl.config(text="● DISCONNECTED", fg="gray")

    # ── Sending ───────────────────────────────────────────────────────────────

    def _write(self, data: bytes):
        if self._ser:
            with self._tx_lock:
                try:
                    self._ser.write(data)
                except serial.SerialException:
                    pass

    def _toggle_send(self):
        if self._send_active:
            self._stop_send()
        else:
            self._start_send()

    def _start_send(self):
        if int(self._cmd_rate_var.get()) == 0:
            return
        self._send_active = True
        self._sender_stop.clear()
        self._sender_thread = threading.Thread(
            target=self._sender_loop, daemon=True, name="cmd_vel_sender")
        self._sender_thread.start()
        self._send_btn.config(text="Send CMD_VEL  [ ON  ]")

    def _stop_send(self):
        self._send_active = False
        self._sender_stop.set()
        self._send_btn.config(text="Send CMD_VEL  [ OFF ]")

    def _sender_loop(self):
        while not self._sender_stop.is_set():
            hz_str = self._cmd_rate_var.get()
            hz = int(hz_str) if hz_str else 10
            if hz == 0:
                self._sender_stop.wait(0.1)
                continue
            self._write(enc_cmd_vel(
                self._left_var.get(),
                self._right_var.get(),
                self._steer_var.get(),
            ))
            self._tx_cmd_vel += 1
            self._sender_stop.wait(1.0 / hz)

    def _set_stream_rate(self, sid: int):
        hz = int(self._stream_rate_vars[sid].get())
        self._write(enc_set_rate(sid, hz))
        self._tx_set_rate += 1
        if hz == 0:
            self._rate_dq[sid].clear()

    def _set_all_rates(self):
        for sid in STREAM_NAMES:
            self._set_stream_rate(sid)

    def _request_fw_stats(self):
        self._write(enc_get_stats())

    def _send_timesync(self):
        t1 = int(time.time() * 1e6)
        self._sync_t1_us = t1
        self._write(enc_timesync(t1, self._sync_last_t4_us))

    def _on_timesync_resp(self, d: dict):
        t4 = d["rx_time_us"]   # recorded in receiver thread at frame-complete time
        t2 = d["t2_us"]
        t1 = self._sync_t1_us
        self._sync_last_t4_us = t4
        self._sync_count += 1
        self._sync_last_rx_ms = time.monotonic() * 1000

        if t1 != 0:
            raw = (t1 + t4) // 2 - t2
            if not self._sync_valid:
                self._sync_offsets = [raw] * 8
                self._sync_hist_idx = 1
            else:
                self._sync_offsets[self._sync_hist_idx % 8] = raw
                self._sync_hist_idx += 1
            self._sync_offset_us = sorted(self._sync_offsets)[4]
            self._sync_jitter_us = max(self._sync_offsets) - min(self._sync_offsets)
            self._sync_valid = True

            if self._sync_last_raw is not None:
                diff = abs(raw - self._sync_last_raw)
                self._sync_last_diff_us = diff
                self._sync_diffs.append(diff)
                if len(self._sync_diffs) > 8:
                    self._sync_diffs.pop(0)
                self._sync_avg_diff_us = sum(self._sync_diffs) // len(self._sync_diffs)
                self._sync_max_diff_us = max(self._sync_diffs)
            self._sync_last_raw = raw

    def _refresh_sync(self):
        self._ts_count.config(text=str(self._sync_count))
        if self._sync_valid:
            age_ms = int(time.monotonic() * 1000 - self._sync_last_rx_ms)
            self._ts_valid.config(text="yes", fg="green")
            self._ts_jitter.config(text=str(self._sync_jitter_us))
            self._ts_age.config(text=str(age_ms))
        else:
            self._ts_valid.config(text="no", fg="gray")
            self._ts_jitter.config(text="—")
            self._ts_age.config(text="—")
        if self._sync_diffs:
            self._ts_last_diff.config(text=str(self._sync_last_diff_us))
            self._ts_avg_diff.config(text=str(self._sync_avg_diff_us))
            self._ts_max_diff.config(text=str(self._sync_max_diff_us))
        else:
            self._ts_last_diff.config(text="—")
            self._ts_avg_diff.config(text="—")
            self._ts_max_diff.config(text="—")

    # ── Poll / display update (GUI thread, every 100 ms) ─────────────────────

    def _poll(self):
        if self._rx:
            try:
                while True:
                    msg_type, _, decoded = self._rx.q.get_nowait()
                    if msg_type in STREAM_NAMES:
                        self._rate_dq[msg_type].append(time.monotonic())
                        self._rx_stream[msg_type] += 1
                        if decoded is not None:
                            self._refresh_stream(msg_type, decoded)
                    elif msg_type == TYPE_STATS and decoded is not None:
                        self._refresh_fw_stats(decoded)
                    elif msg_type == MSG_TIMESYNC_RESP and decoded is not None:
                        self._on_timesync_resp(decoded)
            except queue.Empty:
                pass
            self._refresh_stats()
            self._refresh_sync()
            now = time.monotonic()
            if now - self._last_fw_stats_req >= 1.0:
                self._request_fw_stats()
                self._last_fw_stats_req = now
            try:
                period_s = float(self._sync_period_var.get())
            except ValueError:
                period_s = 1.0
            if period_s > 0:
                interval = max(1, round(period_s / 0.1))
                if self._sync_poll_cycle < 8 or self._sync_poll_cycle % interval == 0:
                    self._send_timesync()
            self._sync_poll_cycle += 1
        self.after(100, self._poll)

    def _refresh_fw_stats(self, d: dict):
        self._fw_rx_frames.config( text=str(d["rx_frames"]))
        self._fw_rx_crc_err.config(text=str(d["rx_crc_err"]))
        self._fw_rx_len_err.config(text=str(d["rx_len_err"]))
        self._fw_rx_short.config(  text=str(d["rx_short"]))
        self._fw_rx_unknown.config(text=str(d["rx_unknown"]))
        self._fw_tx_frames.config( text=str(d["tx_frames"]))

    def _refresh_stats(self):
        rx = self._rx
        self._stat_rx_frames.config(    text=str(rx.rx_frames     if rx else 0))
        self._stat_rx_crc_err.config(   text=str(rx.rx_crc_err    if rx else 0))
        self._stat_rx_len_err.config(   text=str(rx.rx_len_err    if rx else 0))
        self._stat_rx_decode_err.config(text=str(rx.rx_decode_err if rx else 0))
        self._stat_tx_joint.config(  text=str(self._rx_stream[TYPE_JOINT]))
        self._stat_tx_imu.config(    text=str(self._rx_stream[TYPE_IMU]))
        self._stat_tx_bat.config(    text=str(self._rx_stream[TYPE_BAT]))
        self._stat_tx_cmd_vel.config( text=str(self._tx_cmd_vel))
        self._stat_tx_req.config(     text=str(self._tx_req))
        self._stat_tx_set_rate.config(text=str(self._tx_set_rate))

    def _refresh_stream(self, sid: int, data: dict):
        rate_lbl, data_lbl = self._stream_widgets[sid]

        dq = self._rate_dq[sid]
        if len(dq) >= 2:
            hz = (len(dq) - 1) / (dq[-1] - dq[0])
            rate_lbl.config(text=f"rate: {hz:5.1f} Hz", fg="black")
        else:
            rate_lbl.config(text="rate: —", fg="gray")

        if sid == TYPE_JOINT:
            txt = (
                f"enc_l={data['enc_left']:+10d}   enc_r={data['enc_right']:+10d}\n"
                f"steer={data['steering']:3d}   seq={data['seq']}"
            )
            if "vel_left" in data:
                txt += (
                    f"\nvel_l={data['vel_left']:+6d} °/s"
                    f"   vel_r={data['vel_right']:+6d} °/s"
                )
            if "pi_time_us" in data and data["pi_time_us"] != 0:
                txt += f"\npi_t={data['pi_time_us']/1e6:.3f} s"
        elif sid == TYPE_IMU:
            a, g, m = data["accel"], data["gyro"], data["magn"]
            txt = (
                f"accel: {a[0]:+7.3f}  {a[1]:+7.3f}  {a[2]:+7.3f}  m/s²\n"
                f"gyro:  {g[0]:+7.3f}  {g[1]:+7.3f}  {g[2]:+7.3f}  rad/s\n"
                f"magn:  {m[0]:+7.1f}  {m[1]:+7.1f}  {m[2]:+7.1f}  µT\n"
                f"temp:  {data['temp']:.2f} °C"
            )
        elif sid == TYPE_BAT:
            txt = f"{data['voltage_mv'] / 1000:.2f} V   ({data['charge_pct']}%)"
        else:
            return

        data_lbl.config(text=txt)

    # ── Shutdown ──────────────────────────────────────────────────────────────

    def _on_close(self):
        self._disconnect()
        self.destroy()


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="PICAR-2 GUI controller")
    parser.add_argument("--port", default="/dev/ttyYahboom0")
    parser.add_argument("--baud", type=int, default=460800)
    args = parser.parse_args()

    App(args.port, args.baud).mainloop()


if __name__ == "__main__":
    main()
