#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OWC TX ONLY GUI v5 RTC

Parse UART logs:
role board_role=TX_ONLY
alive_tx tx_frames=86252 tx_frame_id=86252 bit_rate=100000 tx_enabled=1 carrier_test=0
tx_frame tx_frame_id=86262 len=4 payload=55 A5 3C C3 checksum=FD frame=AA AA AA D5 04 55 A5 3C C3 FD
adc_mv rx_out_a=1809 vmon_bu=4931 vmon_bu_3v=3027 vmon_main_sys=9414 vmon_main=9392 vmon_5v_sys=5016 vmon_3v3_sys=3300
dac_mv threshold=1650
cmd_ok ...
cmd_err ...
tx_status ...
"""

import re
import time
import queue
import threading
from dataclasses import dataclass, field
from collections import deque
from typing import Dict, List, Optional, Tuple

import tkinter as tk
from tkinter import ttk, messagebox

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    raise SystemExit("Missing pyserial. Install: pip install pyserial") from exc

try:
    import matplotlib
    matplotlib.use("TkAgg")
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
except ImportError as exc:
    raise SystemExit("Missing matplotlib. Install: pip install matplotlib") from exc


POWER_KEYS = ["rx_out_a", "vmon_bu", "vmon_bu_3v", "vmon_main_sys", "vmon_main", "vmon_5v_sys", "vmon_3v3_sys"]
POWER_EXPECTED_MV = {
    "rx_out_a": (1800, 0, 3300),
    "vmon_bu": (5000, 4500, 5500),
    "vmon_bu_3v": (3000, 2800, 3300),
    "vmon_main_sys": (9400, 8500, 10000),
    "vmon_main": (9400, 8500, 10000),
    "vmon_5v_sys": (5000, 4750, 5250),
    "vmon_3v3_sys": (3300, 3135, 3465),
}

PREAMBLE = [0xAA, 0xAA, 0xAA]
SYNC = 0xD5
DEFAULT_PAYLOAD = [0x55, 0xA5, 0x3C, 0xC3]
DAC_EXPECTED_MV = 1650
CARRIER_HZ = 1_000_000


def checksum(payload: List[int]) -> int:
    return (len(payload) + sum(payload)) & 0xFF


def build_frame(payload: List[int]) -> List[int]:
    return PREAMBLE + [SYNC, len(payload)] + payload + [checksum(payload)]


def hex_bytes(values: List[int]) -> str:
    return " ".join(f"{v:02X}" for v in values)


def bits_from_bytes(values: List[int]) -> List[int]:
    bits = []
    for byte in values:
        for pos in range(7, -1, -1):
            bits.append((byte >> pos) & 1)
    return bits


def split_frame(frame: List[int]) -> Dict[str, List[int]]:
    if len(frame) < 6:
        return {"Preamble": frame[:3], "Sync": frame[3:4], "Length": frame[4:5], "Payload": [], "Checksum": frame[-1:] if frame else []}
    payload_len = frame[4]
    ps = 5
    pe = min(ps + payload_len, max(len(frame) - 1, ps))
    return {
        "Preamble": frame[:3],
        "Sync": frame[3:4],
        "Length": frame[4:5],
        "Payload": frame[ps:pe],
        "Checksum": frame[pe:pe + 1] if pe < len(frame) else [],
    }


def parse_kv_int(line: str) -> Dict[str, int]:
    out: Dict[str, int] = {}
    for k, v in re.findall(r"([A-Za-z0-9_]+)=(-?\d+)", line):
        try:
            out[k] = int(v)
        except ValueError:
            pass
    return out


def parse_hex_field(line: str, field: str) -> List[int]:
    # Match "field=AA BB CC" until next key=value or end-of-line.
    m = re.search(rf"(?:^|\s){field}=([0-9A-Fa-fxX ]+?)(?:\s+[A-Za-z0-9_]+=|$)", line)
    if not m:
        return []
    vals = []
    for tok in m.group(1).strip().split():
        tok = tok.replace("0x", "").replace("0X", "")
        if re.fullmatch(r"[0-9A-Fa-f]{1,2}", tok):
            vals.append(int(tok, 16))
    return vals


def normalize_line(line: str) -> str:
    line = line.strip()
    tokens = ["role ", "alive_tx ", "tx_frame ", "adc_mv ", "dac_mv ", "rtc ", "rtc_event ", "rtc_bkp ", "cmd_ok ", "cmd_err ", "tx_status "]
    positions = [(line.find(tok), tok) for tok in tokens if line.find(tok) >= 0]
    if positions:
        pos, _ = min(positions, key=lambda x: x[0])
        return line[pos:]
    return line


def mv_status(key: str, mv: int) -> str:
    _exp, lo, hi = POWER_EXPECTED_MV[key]
    return "OK" if lo <= mv <= hi else "WARN"


class SerialReader(threading.Thread):
    def __init__(self, port: str, baud: int, out_queue: "queue.Queue[str]"):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.out_queue = out_queue
        self.stop_event = threading.Event()
        self.write_queue: "queue.Queue[str]" = queue.Queue()
        self.ser: Optional[serial.Serial] = None

    def write_line(self, line: str):
        if not line.endswith("\n"):
            line += "\n"
        self.write_queue.put(line)

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.15)
            self.out_queue.put(f"__STATUS__ Connected to {self.port} @ {self.baud}")
        except Exception as exc:
            self.out_queue.put(f"__ERROR__ Cannot open {self.port}: {exc}")
            return

        while not self.stop_event.is_set():
            try:
                while not self.write_queue.empty():
                    tx = self.write_queue.get_nowait()
                    if self.ser and self.ser.is_open:
                        self.ser.write(tx.encode("ascii", errors="ignore"))
                        self.out_queue.put(f"__STATUS__ Sent: {tx.strip()}")
                raw = self.ser.readline()
                if raw:
                    line = raw.decode("utf-8", errors="replace").strip()
                    if line:
                        self.out_queue.put(line)
            except Exception as exc:
                self.out_queue.put(f"__ERROR__ Serial error: {exc}")
                break

        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        self.out_queue.put("__STATUS__ Disconnected")

    def stop(self):
        self.stop_event.set()


@dataclass
class TxState:
    role: str = "UNKNOWN"
    tx_frames: int = 0
    tx_frame_id: int = 0
    bit_rate: int = 100_000
    tx_enabled: int = 0
    carrier_test: int = 0
    payload: List[int] = field(default_factory=lambda: list(DEFAULT_PAYLOAD))
    frame: List[int] = field(default_factory=lambda: build_frame(DEFAULT_PAYLOAD))
    power_mv: Dict[str, int] = field(default_factory=lambda: {k: 0 for k in POWER_KEYS})
    dac_mv: int = DAC_EXPECTED_MV
    rtc_time: str = "---- -- -- --:--:--"
    rtc_valid: int = 0
    rtc_source: str = "-"
    rtc_backup: str = "-"
    rtc_event: str = "none"
    last_cmd_response: str = "none"
    raw_lines: deque = field(default_factory=lambda: deque(maxlen=2000))
    start_time: float = field(default_factory=time.time)
    last_alive_time: Optional[float] = None
    last_tx_frames: int = 0
    tx_fps: float = 0.0
    tx_bit_rate_est: float = 0.0
    payload_goodput: float = 0.0
    hist_t: deque = field(default_factory=lambda: deque(maxlen=300))
    hist_power: Dict[str, deque] = field(default_factory=lambda: {k: deque(maxlen=300) for k in POWER_KEYS})
    hist_rtc_t: deque = field(default_factory=lambda: deque(maxlen=300))
    hist_rtc_seconds: deque = field(default_factory=lambda: deque(maxlen=300))


class OwcTxGui(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("OWC TX-ONLY Monitor - STM32F407 - v5 RTC")
        self.geometry("1400x850")
        self.minsize(1150, 720)

        self.state = TxState()
        self.q: "queue.Queue[str]" = queue.Queue()
        self.reader: Optional[SerialReader] = None
        self.max_bits = tk.IntVar(value=80)
        self.selected_field = "Full Frame"
        self.payload_entry = tk.StringVar(value=hex_bytes(DEFAULT_PAYLOAD))

        self._build_ui()
        self._refresh_ports()
        self.after(50, self._process_queue)
        self.after(250, self._refresh_ui)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        style = ttk.Style(self)
        style.configure("Header.TLabel", font=("Segoe UI", 11, "bold"))
        style.configure("Big.TLabel", font=("Consolas", 15, "bold"))
        style.configure("Ok.TLabel", foreground="#0a7a22", font=("Segoe UI", 10, "bold"))
        style.configure("Warn.TLabel", foreground="#b00020", font=("Segoe UI", 10, "bold"))

        top = ttk.Frame(self, padding=8)
        top.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(top, text="COM:").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(top, width=14, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=(4, 8))
        ttk.Button(top, text="Refresh", command=self._refresh_ports).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Label(top, text="Baud:").pack(side=tk.LEFT)
        self.baud_var = tk.StringVar(value="115200")
        ttk.Entry(top, textvariable=self.baud_var, width=10).pack(side=tk.LEFT, padx=(4, 8))
        self.connect_btn = ttk.Button(top, text="Connect", command=self._toggle_connect)
        self.connect_btn.pack(side=tk.LEFT, padx=(0, 12))
        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(top, textvariable=self.status_var).pack(side=tk.LEFT)

        self.nb = ttk.Notebook(self)
        self.nb.pack(fill=tk.BOTH, expand=True)
        self.tx_tab = ttk.Frame(self.nb)
        self.power_tab = ttk.Frame(self.nb)
        self.rtc_tab = ttk.Frame(self.nb)
        self.raw_tab = ttk.Frame(self.nb)
        self.nb.add(self.tx_tab, text="TX DEMO")
        self.nb.add(self.power_tab, text="POWER")
        self.nb.add(self.rtc_tab, text="RTC")
        self.nb.add(self.raw_tab, text="RAW LOG")

        self._build_tx_tab()
        self._build_power_tab()
        self._build_rtc_tab()
        self._build_raw_tab()

    def _build_tx_tab(self):
        counters = ttk.Frame(self.tx_tab, padding=(10, 8))
        counters.pack(side=tk.TOP, fill=tk.X)

        self.role_var = tk.StringVar(value="role: UNKNOWN")
        ttk.Label(counters, textvariable=self.role_var, style="Header.TLabel").pack(side=tk.LEFT, padx=(0, 12))

        self.card_vars: Dict[str, tk.StringVar] = {}
        cards = [
            ("tx_frames", "0"),
            ("tx_frame_id", "0"),
            ("bit_rate", "100000 bps"),
            ("tx_enabled", "0"),
            ("carrier_test", "0"),
            ("TX fps", "0.0 fps"),
            ("TX bit est.", "0 bps"),
            ("goodput", "0 bps"),
        ]
        for title, default in cards:
            var = tk.StringVar(value=default)
            self.card_vars[title] = var
            box = ttk.Frame(counters, padding=5, relief="ridge")
            box.pack(side=tk.LEFT, padx=4)
            ttk.Label(box, text=title).pack()
            ttk.Label(box, textvariable=var, style="Big.TLabel").pack()

        rtc_strip = ttk.Frame(self.tx_tab, padding=(10, 0))
        rtc_strip.pack(side=tk.TOP, fill=tk.X)
        self.rtc_summary_var = tk.StringVar(value="RTC: waiting")
        ttk.Label(rtc_strip, textvariable=self.rtc_summary_var, style="Header.TLabel").pack(side=tk.LEFT, padx=(0, 16))
        ttk.Button(rtc_strip, text="RTC Get", command=self._cmd_rtc_get).pack(side=tk.LEFT, padx=3)
        ttk.Button(rtc_strip, text="RTC Set From PC", command=self._cmd_rtc_set_from_pc).pack(side=tk.LEFT, padx=3)
        ttk.Button(rtc_strip, text="RTC Backup", command=self._cmd_rtc_bkp).pack(side=tk.LEFT, padx=3)

        frame_box = ttk.LabelFrame(self.tx_tab, text="TX Frame đang phát", padding=8)
        frame_box.pack(side=tk.TOP, fill=tk.X, padx=10, pady=(8, 6))
        self.compact_frame = ttk.Frame(frame_box)
        self.compact_frame.pack(fill=tk.X)
        self.frame_detail_var = tk.StringVar(value="Waiting for tx_frame log...")
        ttk.Label(frame_box, textvariable=self.frame_detail_var, wraplength=1280).pack(anchor=tk.W, pady=(8, 0))
        self.bits_var = tk.StringVar(value="")
        ttk.Label(frame_box, textvariable=self.bits_var, font=("Consolas", 9), wraplength=1280).pack(anchor=tk.W, pady=(4, 0))

        body = ttk.Frame(self.tx_tab, padding=(10, 0))
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        left = ttk.Frame(body)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8))

        fields_box = ttk.LabelFrame(left, text="TX Frame Fields", padding=8)
        fields_box.pack(fill=tk.BOTH, expand=True)
        cols = ("field", "value", "note")
        self.field_tree = ttk.Treeview(fields_box, columns=cols, show="headings", height=8)
        for col, width in [("field", 110), ("value", 280), ("note", 150)]:
            self.field_tree.heading(col, text=col.upper())
            self.field_tree.column(col, width=width, anchor=tk.W)
        self.field_tree.pack(fill=tk.BOTH, expand=True)
        self.field_tree.bind("<<TreeviewSelect>>", self._on_field_selected)
        ttk.Label(
            fields_box,
            text="Click vào từng field để waveform bên phải chỉ hiển thị phần Preamble / Sync / Length / Payload / Checksum tương ứng.",
            wraplength=520,
        ).pack(anchor=tk.W, pady=(6, 0))

        control = ttk.LabelFrame(left, text="TX Control", padding=8)
        control.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(control, text="Payload HEX:").grid(row=0, column=0, sticky=tk.W, pady=3)
        ttk.Entry(control, textvariable=self.payload_entry, width=30, font=("Consolas", 10)).grid(row=0, column=1, columnspan=3, sticky=tk.W, pady=3)
        ttk.Button(control, text="Send Payload", command=self._cmd_payload).grid(row=0, column=4, sticky=tk.W, padx=(5, 0), pady=3)

        buttons = [
            ("Start TX", self._cmd_start),
            ("Stop TX", self._cmd_stop),
            ("Carrier ON", self._cmd_carrier_on),
            ("Carrier OFF", self._cmd_carrier_off),
            ("Single Frame", self._cmd_single),
            ("Status", self._cmd_status),
        ]
        for idx, (label, func) in enumerate(buttons):
            ttk.Button(control, text=label, command=func).grid(row=1 + idx // 3, column=idx % 3, sticky=tk.EW, padx=3, pady=3)

        self.cmd_var = tk.StringVar(value="cmd response: none")
        ttk.Label(control, textvariable=self.cmd_var, font=("Consolas", 9), wraplength=520).grid(row=3, column=0, columnspan=5, sticky=tk.W, pady=(6, 0))

        right = ttk.Frame(body)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        ctrl = ttk.Frame(right)
        ctrl.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(ctrl, text="OOK preview: bit 1 = 10 carrier cycles, bit 0 = OFF").pack(side=tk.LEFT)
        ttk.Label(ctrl, text="View:").pack(side=tk.LEFT, padx=(20, 4))
        self.selected_field_var = tk.StringVar(value="Full Frame")
        ttk.Label(ctrl, textvariable=self.selected_field_var, style="Header.TLabel").pack(side=tk.LEFT, padx=(0, 12))
        ttk.Button(ctrl, text="Show Full Frame", command=self._select_full_frame).pack(side=tk.LEFT, padx=(0, 12))
        ttk.Label(ctrl, text="Max bits:").pack(side=tk.LEFT, padx=(20, 4))
        ttk.Spinbox(ctrl, from_=8, to=160, increment=8, textvariable=self.max_bits, width=6).pack(side=tk.LEFT)

        self.fig = Figure(figsize=(8.5, 4.5), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _build_power_tab(self):
        main = ttk.Frame(self.power_tab, padding=10)
        main.pack(fill=tk.BOTH, expand=True)

        left = ttk.Frame(main)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8))
        self.power_tree = ttk.Treeview(left, columns=("measured", "expected", "range", "status"), show="tree headings", height=10)
        self.power_tree.heading("#0", text="Signal")
        self.power_tree.column("#0", width=150)
        for col, width in [("measured", 95), ("expected", 95), ("range", 125), ("status", 70)]:
            self.power_tree.heading(col, text=col.title())
            self.power_tree.column(col, width=width, anchor=tk.CENTER)
        self.power_tree.tag_configure("ok", background="#eaffea")
        self.power_tree.tag_configure("warn", background="#ffd9d9")
        self.power_tree.pack(fill=tk.BOTH, expand=True)
        for key in POWER_KEYS:
            exp, lo, hi = POWER_EXPECTED_MV[key]
            self.power_tree.insert("", "end", iid=key, text=key, values=("0.000 V", f"{exp/1000:.3f} V", f"{lo/1000:.2f}-{hi/1000:.2f} V", "WARN"), tags=("warn",))

        self.dac_var = tk.StringVar(value="DAC threshold: 1.650 V")
        ttk.Label(left, textvariable=self.dac_var, style="Header.TLabel").pack(anchor=tk.W, pady=(10, 0))

        right = ttk.Frame(main)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.pfig = Figure(figsize=(8, 5), dpi=100)
        self.pax = self.pfig.add_subplot(111)
        self.pcanvas = FigureCanvasTkAgg(self.pfig, master=right)
        self.pcanvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _build_rtc_tab(self):
        main = ttk.Frame(self.rtc_tab, padding=10)
        main.pack(fill=tk.BOTH, expand=True)

        top = ttk.LabelFrame(main, text="RTC LSE/VBAT Status", padding=10)
        top.pack(side=tk.TOP, fill=tk.X)

        self.rtc_time_var = tk.StringVar(value="time: --")
        self.rtc_valid_var = tk.StringVar(value="valid: 0")
        self.rtc_source_var = tk.StringVar(value="source: -")
        self.rtc_backup_var = tk.StringVar(value="backup: -")
        self.rtc_event_var = tk.StringVar(value="event: none")

        for i, (label, var) in enumerate([
            ("RTC time", self.rtc_time_var),
            ("Valid", self.rtc_valid_var),
            ("Source", self.rtc_source_var),
            ("Backup", self.rtc_backup_var),
            ("Last event", self.rtc_event_var),
        ]):
            ttk.Label(top, text=f"{label}:", style="Header.TLabel").grid(row=i, column=0, sticky=tk.W, pady=4, padx=(0, 8))
            ttk.Label(top, textvariable=var, font=("Consolas", 12, "bold")).grid(row=i, column=1, sticky=tk.W, pady=4)

        cmd = ttk.LabelFrame(main, text="RTC Commands", padding=10)
        cmd.pack(side=tk.TOP, fill=tk.X, pady=(10, 8))
        ttk.Button(cmd, text="rtc_get", command=self._cmd_rtc_get).pack(side=tk.LEFT, padx=3)
        ttk.Button(cmd, text="rtc_bkp", command=self._cmd_rtc_bkp).pack(side=tk.LEFT, padx=3)
        ttk.Button(cmd, text="rtc_set from PC time", command=self._cmd_rtc_set_from_pc).pack(side=tk.LEFT, padx=3)
        ttk.Button(cmd, text="rtc_reset default", command=self._cmd_rtc_reset).pack(side=tk.LEFT, padx=3)

        ttk.Label(cmd, text="Manual rtc_set:").pack(side=tk.LEFT, padx=(20, 4))
        self.rtc_manual_var = tk.StringVar(value="2026-05-28 14:30:05")
        ttk.Entry(cmd, textvariable=self.rtc_manual_var, width=22, font=("Consolas", 10)).pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(cmd, text="Send", command=self._cmd_rtc_set_manual).pack(side=tk.LEFT, padx=3)

        note = (
            "RTC log format: rtc time=YYYY-MM-DD HH:MM:SS valid=1 source=LSE backup=ok. "
            "Nếu backup=ok và reset board mà thời gian không về default, VBAT/LSE đang hoạt động."
        )
        ttk.Label(main, text=note, wraplength=1200).pack(side=tk.TOP, anchor=tk.W, pady=(4, 8))

        self.rtc_fig = Figure(figsize=(8, 4), dpi=100)
        self.rtc_ax = self.rtc_fig.add_subplot(111)
        self.rtc_canvas = FigureCanvasTkAgg(self.rtc_fig, master=main)
        self.rtc_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)


    def _build_raw_tab(self):
        frame = ttk.Frame(self.raw_tab, padding=8)
        frame.pack(fill=tk.BOTH, expand=True)
        self.raw_text = tk.Text(frame, height=20, wrap=tk.NONE, font=("Consolas", 10))
        self.raw_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        y = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=self.raw_text.yview)
        y.pack(side=tk.RIGHT, fill=tk.Y)
        self.raw_text.configure(yscrollcommand=y.set)

    def _refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_combo.get():
            self.port_combo.current(0)

    def _toggle_connect(self):
        if self.reader is None:
            port = self.port_combo.get().strip()
            if not port:
                messagebox.showwarning("Missing COM", "Chọn COM port trước.")
                return
            try:
                baud = int(self.baud_var.get())
            except ValueError:
                messagebox.showwarning("Invalid baud", "Baud rate không hợp lệ.")
                return
            self.reader = SerialReader(port, baud, self.q)
            self.reader.start()
            self.connect_btn.configure(text="Disconnect")
            self.status_var.set("Connecting...")
        else:
            self.reader.stop()
            self.reader = None
            self.connect_btn.configure(text="Connect")
            self.status_var.set("Disconnecting...")

    def _process_queue(self):
        while True:
            try:
                line = self.q.get_nowait()
            except queue.Empty:
                break

            if line.startswith("__ERROR__") or line.startswith("__STATUS__"):
                msg = line.replace("__ERROR__ ", "").replace("__STATUS__ ", "")
                self.status_var.set(msg)
                self._append_raw(line)
                if "Disconnected" in line:
                    self.reader = None
                    self.connect_btn.configure(text="Connect")
                continue

            self._handle_line(line)

        self.after(50, self._process_queue)

    def _handle_line(self, line: str):
        self.state.raw_lines.append(line)
        self._append_raw(line)

        line = normalize_line(line)

        if line.startswith("role"):
            m = re.search(r"board_role=([A-Za-z0-9_]+)", line)
            if m:
                self.state.role = m.group(1)

        elif line.startswith("alive_tx"):
            kv = parse_kv_int(line)
            self.state.tx_frames = kv.get("tx_frames", self.state.tx_frames)
            self.state.tx_frame_id = kv.get("tx_frame_id", self.state.tx_frame_id)
            self.state.bit_rate = kv.get("bit_rate", self.state.bit_rate)
            self.state.tx_enabled = kv.get("tx_enabled", self.state.tx_enabled)
            self.state.carrier_test = kv.get("carrier_test", self.state.carrier_test)
            self._update_rates()

        elif line.startswith("tx_frame"):
            kv = parse_kv_int(line)
            self.state.tx_frame_id = kv.get("tx_frame_id", self.state.tx_frame_id)
            payload = parse_hex_field(line, "payload")
            frame = parse_hex_field(line, "frame")
            if payload:
                self.state.payload = payload
                self.payload_entry.set(hex_bytes(payload))
            if frame:
                self.state.frame = frame
            elif payload:
                self.state.frame = build_frame(payload)

        elif line.startswith("adc_mv"):
            kv = parse_kv_int(line)
            for key in POWER_KEYS:
                if key in kv:
                    self.state.power_mv[key] = kv[key]
            t = time.time() - self.state.start_time
            self.state.hist_t.append(t)
            for key in POWER_KEYS:
                self.state.hist_power[key].append(self.state.power_mv[key] / 1000.0)

        elif line.startswith("dac"):
            kv = parse_kv_int(line)
            for key in ("threshold", "threshold_mv", "dac_threshold_mv", "dac_mv"):
                if key in kv:
                    self.state.dac_mv = kv[key]
                    break

        elif line.startswith("rtc "):
            self._parse_rtc_line(line)

        elif line.startswith("rtc_event"):
            self.state.rtc_event = line

        elif line.startswith("rtc_bkp"):
            self.state.rtc_event = line
            m_backup = re.search(r"backup=([^\s]+)", line)
            if m_backup:
                self.state.rtc_backup = m_backup.group(1)

        elif line.startswith("cmd_ok") or line.startswith("cmd_err"):
            self.state.last_cmd_response = line

        elif line.startswith("tx_status"):
            kv = parse_kv_int(line)
            self.state.tx_enabled = kv.get("tx_enabled", self.state.tx_enabled)
            self.state.carrier_test = kv.get("carrier_test", self.state.carrier_test)
            self.state.tx_frames = kv.get("tx_frames", self.state.tx_frames)
            self.state.tx_frame_id = kv.get("tx_frame_id", self.state.tx_frame_id)
            self.state.bit_rate = kv.get("bit_rate", self.state.bit_rate)
            payload = parse_hex_field(line, "payload")
            frame = parse_hex_field(line, "frame")
            if payload:
                self.state.payload = payload
                self.payload_entry.set(hex_bytes(payload))
            if frame:
                self.state.frame = frame
            elif payload:
                self.state.frame = build_frame(payload)
            self.state.last_cmd_response = line

    def _update_rates(self):
        now = time.time()
        if self.state.last_alive_time is not None:
            dt = max(now - self.state.last_alive_time, 1e-6)
            dframes = max(self.state.tx_frames - self.state.last_tx_frames, 0)
            self.state.tx_fps = dframes / dt
            self.state.tx_bit_rate_est = self.state.tx_fps * max(len(self.state.frame) * 8, 1)
            self.state.payload_goodput = self.state.tx_fps * max(len(self.state.payload) * 8, 1)
        self.state.last_alive_time = now
        self.state.last_tx_frames = self.state.tx_frames

    def _parse_rtc_line(self, line: str):
        # Expected:
        # rtc time=2026-01-01 00:00:22 valid=1 source=LSE backup=ok
        m_time = re.search(r"time=(\d{4}-\d{2}-\d{2}\s+\d{2}:\d{2}:\d{2})", line)
        m_valid = re.search(r"valid=(\d+)", line)
        m_source = re.search(r"source=([^\s]+)", line)
        m_backup = re.search(r"backup=([^\s]+)", line)

        if m_time:
            self.state.rtc_time = m_time.group(1)
            sec = self._rtc_time_to_seconds(self.state.rtc_time)
            if sec is not None:
                self.state.hist_rtc_t.append(time.time() - self.state.start_time)
                self.state.hist_rtc_seconds.append(sec)
        if m_valid:
            self.state.rtc_valid = int(m_valid.group(1))
        if m_source:
            self.state.rtc_source = m_source.group(1)
        if m_backup:
            self.state.rtc_backup = m_backup.group(1)

    def _rtc_time_to_seconds(self, time_str: str):
        try:
            _date, tod = time_str.split()
            hh, mm, ss = [int(x) for x in tod.split(":")]
            return hh * 3600 + mm * 60 + ss
        except Exception:
            return None

    def _cmd_rtc_get(self):
        self._send_cmd("rtc_get")

    def _cmd_rtc_bkp(self):
        self._send_cmd("rtc_bkp")

    def _cmd_rtc_reset(self):
        if messagebox.askyesno("RTC reset", "Reset RTC về 2026-01-01 00:00:00?"):
            self._send_cmd("rtc_reset")

    def _cmd_rtc_set_manual(self):
        value = self.rtc_manual_var.get().strip()
        if not re.fullmatch(r"\d{4}-\d{2}-\d{2}\s+\d{2}:\d{2}:\d{2}", value):
            messagebox.showwarning("Format sai", "Dùng format: YYYY-MM-DD HH:MM:SS")
            return
        self._send_cmd("rtc_set " + value)

    def _cmd_rtc_set_from_pc(self):
        now = time.localtime()
        value = time.strftime("%Y-%m-%d %H:%M:%S", now)
        if hasattr(self, "rtc_manual_var"):
            self.rtc_manual_var.set(value)
        self._send_cmd("rtc_set " + value)


    def _send_cmd(self, cmd: str):
        if self.reader is None:
            messagebox.showinfo("Not connected", f"Chưa kết nối COM. Lệnh chưa gửi: {cmd}")
            return
        self.reader.write_line(cmd)
        self.state.last_cmd_response = f"sent: {cmd}"

    def _cmd_payload(self):
        raw = self.payload_entry.get().replace(",", " ").replace(";", " ").strip()
        if not raw:
            messagebox.showwarning("Payload rỗng", "Nhập payload HEX, ví dụ: 55 A5 3C C3")
            return
        toks = raw.split()
        clean = []
        for tok in toks:
            tok = tok.replace("0x", "").replace("0X", "")
            if not re.fullmatch(r"[0-9A-Fa-f]{1,2}", tok):
                messagebox.showwarning("Payload sai", f"Byte HEX không hợp lệ: {tok}")
                return
            clean.append(tok.upper().zfill(2))
        self._send_cmd("tx_payload " + " ".join(clean))

    def _cmd_start(self):
        self._send_cmd("tx_start")

    def _cmd_stop(self):
        self._send_cmd("tx_stop")

    def _cmd_carrier_on(self):
        self._send_cmd("tx_carrier_on")

    def _cmd_carrier_off(self):
        self._send_cmd("tx_carrier_off")

    def _cmd_single(self):
        self._send_cmd("tx_single")

    def _cmd_status(self):
        self._send_cmd("tx_status")

    def _select_full_frame(self):
        self.selected_field = "Full Frame"
        if hasattr(self, "selected_field_var"):
            self.selected_field_var.set("Full Frame")
        if hasattr(self, "field_tree"):
            for item in self.field_tree.selection():
                self.field_tree.selection_remove(item)
        self._redraw_wave()

    def _on_field_selected(self, _event=None):
        selected = self.field_tree.selection()
        if not selected:
            return
        item = selected[0]
        values = self.field_tree.item(item, "values")
        if not values:
            return
        field_name = str(values[0])
        if field_name in ("Preamble", "Sync", "Length", "Payload", "Checksum"):
            self.selected_field = field_name
            if hasattr(self, "selected_field_var"):
                self.selected_field_var.set(field_name)
            self._redraw_wave()

    def _selected_frame_bytes(self):
        fields = split_frame(self.state.frame)
        if getattr(self, "selected_field", "Full Frame") == "Full Frame":
            return list(self.state.frame), "Full Frame"
        return list(fields.get(self.selected_field, [])), self.selected_field


    def _refresh_ui(self):
        self.role_var.set(f"role: {self.state.role}")
        self.card_vars["tx_frames"].set(str(self.state.tx_frames))
        self.card_vars["tx_frame_id"].set(str(self.state.tx_frame_id))
        self.card_vars["bit_rate"].set(f"{self.state.bit_rate} bps")
        self.card_vars["tx_enabled"].set(str(self.state.tx_enabled))
        self.card_vars["carrier_test"].set(str(self.state.carrier_test))
        self.card_vars["TX fps"].set(f"{self.state.tx_fps:.1f} fps")
        self.card_vars["TX bit est."].set(f"{self.state.tx_bit_rate_est:.0f} bps")
        self.card_vars["goodput"].set(f"{self.state.payload_goodput:.0f} bps")
        self.cmd_var.set(f"cmd response: {self.state.last_cmd_response}")

        self._refresh_frame_widgets()
        self._refresh_power()
        self._refresh_rtc()
        self._redraw_wave()
        self._redraw_power_chart()
        if hasattr(self, "rtc_canvas"):
            self._redraw_rtc_chart()

        self.after(250, self._refresh_ui)

    def _refresh_frame_widgets(self):
        for child in self.compact_frame.winfo_children():
            child.destroy()
        ttk.Label(self.compact_frame, text="TX:", width=5, style="Header.TLabel").pack(side=tk.LEFT)

        fields = split_frame(self.state.frame)
        colors = {
            "Preamble": "#d7ecff",
            "Sync": "#ffe5c2",
            "Length": "#eadcff",
            "Payload": "#dff6df",
            "Checksum": "#ffd8d8",
        }
        for name in ["Preamble", "Sync", "Length", "Payload", "Checksum"]:
            vals = fields.get(name, [])
            txt = hex_bytes(vals) if vals else "--"
            tk.Label(self.compact_frame, text=f"[{txt}]", bg=colors[name], fg="#111", font=("Consolas", 12, "bold"), padx=8, pady=4, relief="groove").pack(side=tk.LEFT, padx=(0, 4))
            tk.Label(self.compact_frame, text=name, fg="#444", font=("Segoe UI", 8)).pack(side=tk.LEFT, padx=(0, 8))

        self.frame_detail_var.set(
            f"TX frame_id={self.state.tx_frame_id} | "
            f"tx_enabled={self.state.tx_enabled} | carrier_test={self.state.carrier_test} | "
            f"payload={hex_bytes(fields.get('Payload', []))} | checksum={hex_bytes(fields.get('Checksum', []))} | "
            f"full={hex_bytes(self.state.frame)} | view={self.selected_field}"
        )

        bits = bits_from_bytes(self.state.frame)
        bit_text = "".join(str(b) for b in bits)
        preview = " ".join(bit_text[i:i + 8] for i in range(0, min(len(bit_text), 96), 8))
        if len(bit_text) > 96:
            preview += " ..."
        self.bits_var.set(f"Bits: {preview}")

        for item in self.field_tree.get_children():
            self.field_tree.delete(item)
        rows = [
            ("Preamble", hex_bytes(fields.get("Preamble", [])), "Frame start"),
            ("Sync", hex_bytes(fields.get("Sync", [])), "Expected D5"),
            ("Length", hex_bytes(fields.get("Length", [])), f"{len(fields.get('Payload', []))} byte payload"),
            ("Payload", hex_bytes(fields.get("Payload", [])), "Data"),
            ("Checksum", hex_bytes(fields.get("Checksum", [])), "LEN + payload"),
        ]
        for idx, row in enumerate(rows):
            iid = str(row[0])
            self.field_tree.insert("", "end", iid=iid, values=row)
        # Keep previous selection after refresh, so the waveform view does not jump back.
        if self.selected_field in ("Preamble", "Sync", "Length", "Payload", "Checksum"):
            try:
                self.field_tree.selection_set(self.selected_field)
                self.field_tree.focus(self.selected_field)
            except tk.TclError:
                pass

    def _refresh_power(self):
        self.dac_var.set(f"DAC threshold: {self.state.dac_mv/1000.0:.3f} V")
        for key in POWER_KEYS:
            mv = self.state.power_mv.get(key, 0)
            exp, lo, hi = POWER_EXPECTED_MV[key]
            st = mv_status(key, mv)
            tag = "ok" if st == "OK" else "warn"
            self.power_tree.item(key, values=(f"{mv/1000:.3f} V", f"{exp/1000:.3f} V", f"{lo/1000:.2f}-{hi/1000:.2f} V", st), tags=(tag,))

    def _refresh_rtc(self):
        summary = (
            f"RTC: {self.state.rtc_time} | valid={self.state.rtc_valid} | "
            f"source={self.state.rtc_source} | backup={self.state.rtc_backup}"
        )
        if hasattr(self, "rtc_summary_var"):
            self.rtc_summary_var.set(summary)
        if hasattr(self, "rtc_time_var"):
            self.rtc_time_var.set(self.state.rtc_time)
            self.rtc_valid_var.set(str(self.state.rtc_valid))
            self.rtc_source_var.set(self.state.rtc_source)
            self.rtc_backup_var.set(self.state.rtc_backup)
            self.rtc_event_var.set(self.state.rtc_event)

    def _redraw_rtc_chart(self):
        self.rtc_ax.clear()
        self.rtc_ax.set_title("RTC seconds-of-day progress")
        self.rtc_ax.set_xlabel("GUI time (s)")
        self.rtc_ax.set_ylabel("RTC seconds of day")
        self.rtc_ax.grid(True)

        t = list(self.state.hist_rtc_t)
        s = list(self.state.hist_rtc_seconds)
        n = min(len(t), len(s))
        if n > 1:
            self.rtc_ax.plot(t[-n:], s[-n:], label="RTC seconds")
            self.rtc_ax.legend(loc="upper left")
        self.rtc_canvas.draw_idle()


    def _redraw_wave(self):
        self.ax.clear()
        self.ax.set_title("TX OOK waveform preview from current tx_frame")
        self.ax.set_xlabel("Time (µs), bit window = 10 µs @ 100 kbps")
        self.ax.set_ylabel("Carrier gate")
        self.ax.grid(True)

        try:
            max_bits = int(self.max_bits.get())
        except Exception:
            max_bits = 80
        selected_bytes, selected_name = self._selected_frame_bytes()
        bits = bits_from_bytes(selected_bytes)[:max_bits]
        self.ax.set_title(f"TX OOK waveform - {selected_name}: {hex_bytes(selected_bytes) if selected_bytes else '--'}")
        bit_us = 1_000_000.0 / max(self.state.bit_rate, 1)
        carrier_period_us = 1_000_000.0 / CARRIER_HZ
        samples_per_carrier = 6
        tv: List[float] = []
        yv: List[float] = []

        # Add byte boundary markers every 8 bits for easier demo explanation.
        for byte_idx in range((len(bits) + 7) // 8 + 1):
            x = byte_idx * 8 * bit_us
            self.ax.axvline(x, linewidth=0.8, linestyle="-", alpha=0.45)

        for i, bit in enumerate(bits):
            t0 = i * bit_us
            self.ax.axvline(t0, linewidth=0.4, linestyle="--", alpha=0.35)
            if bit == 1:
                total_samples = int((bit_us / carrier_period_us) * samples_per_carrier)
                for n in range(total_samples + 1):
                    t = t0 + n * (carrier_period_us / samples_per_carrier)
                    phase = (t - t0) % carrier_period_us
                    y = 1.0 if phase < carrier_period_us / 2.0 else 0.0
                    tv.append(t)
                    yv.append(y)
            else:
                tv.extend([t0, t0 + bit_us])
                yv.extend([0.0, 0.0])
            self.ax.text(t0 + bit_us / 2.0, 1.08, str(bit), ha="center", va="bottom", fontsize=7)

        if len(tv) > 1:
            self.ax.step(tv, yv, where="post")
        self.ax.set_ylim(-0.15, 1.3)
        self.ax.set_xlim(0, max(len(bits) * bit_us, bit_us))
        self.canvas.draw_idle()

    def _redraw_power_chart(self):
        self.pax.clear()
        self.pax.set_title("TX Board Power Monitor")
        self.pax.set_xlabel("Time (s)")
        self.pax.set_ylabel("Voltage (V)")
        self.pax.grid(True)
        t = list(self.state.hist_t)
        for key in ("vmon_5v_sys", "vmon_3v3_sys", "vmon_main_sys", "vmon_main", "vmon_bu", "vmon_bu_3v"):
            y = list(self.state.hist_power[key])
            n = min(len(t), len(y))
            if n > 1:
                self.pax.plot(t[-n:], y[-n:], label=key)
        if len(t) > 1:
            self.pax.legend(loc="upper right")
        self.pcanvas.draw_idle()

    def _append_raw(self, line: str):
        self.raw_text.insert(tk.END, line + "\n")
        if int(self.raw_text.index("end-1c").split(".")[0]) > 1500:
            self.raw_text.delete("1.0", "300.0")
        self.raw_text.see(tk.END)

    def _on_close(self):
        if self.reader:
            self.reader.stop()
        self.destroy()


if __name__ == "__main__":
    OwcTxGui().mainloop()
