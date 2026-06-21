#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OWC RX-ONLY GUI Demo - STM32F407

Parse UART logs for RX_ONLY role:
role board_role=RX_ONLY
alive_rx rx_frames=0 frame_errors=0 checksum_errors=0 rx_sync_state=0 last_edge_delta=0 last_raw_bit=0
expected_tx_frame len=4 payload=55 A5 3C C3 checksum=FD frame=AA AA AA D5 04 55 A5 3C C3 FD
last_rx none
link_stats payload_bits=0 payload_bit_errors=0 payload_ber_ppm=0 payload_mismatch_frames=0 good_payload_frames=0
link_quality rx_total_observed=0 per_ppm=0
err_summary queued=0 printed=0 suppressed=0
adc_mv rx_out_a=1809 vmon_bu=4928 vmon_bu_3v=3025 vmon_main_sys=9276 vmon_main=9372 vmon_5v_sys=5012 vmon_3v3_sys=3295
dac_mv threshold=1650
rtc time=2026-06-09 18:49:37 valid=1 source=LSE backup=ok
"""

import re
import time
import math
import queue
import calendar
import threading
from dataclasses import dataclass, field
from collections import deque
from datetime import datetime
from typing import Dict, List, Optional

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

RX_STATE_NAMES = {
    0: "WAIT_ACTIVITY",
    1: "DETECT_PREAMBLE",
    2: "LOCK_BIT_TIMING",
    3: "READ_SYNC",
    4: "READ_LEN",
    5: "READ_PAYLOAD",
    6: "READ_CHECKSUM",
}

PREAMBLE = [0xAA, 0xAA, 0xAA]
SYNC = 0xD5
DEFAULT_PAYLOAD = [0x55, 0xA5, 0x3C, 0xC3]
DAC_EXPECTED_MV = 1650
EDGE_THRESHOLD = 6


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
    tokens = [
        "role ", "alive_rx ", "expected_tx_frame ", "last_rx ", "link_stats ",
        "link_quality ", "err_summary ", "err_frame ", "err_bits ", "adc_mv ",
        "dac_mv ", "rtc ", "rtc_event ", "rtc_bkp ", "cmd_ok ", "cmd_err ",
    ]
    positions = [(line.find(tok), tok) for tok in tokens if line.find(tok) >= 0]
    if positions:
        pos, _tok = min(positions, key=lambda x: x[0])
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
class RxState:
    role: str = "UNKNOWN"
    rx_frames: int = 0
    frame_errors: int = 0
    checksum_errors: int = 0
    rx_sync_state: int = 0
    last_edge_delta: int = 0
    last_raw_bit: int = 0
    expected_payload: List[int] = field(default_factory=lambda: list(DEFAULT_PAYLOAD))
    expected_frame: List[int] = field(default_factory=lambda: build_frame(DEFAULT_PAYLOAD))
    expected_checksum: int = checksum(DEFAULT_PAYLOAD)
    last_rx_none: bool = True
    last_rx_payload: List[int] = field(default_factory=list)
    last_rx_frame: List[int] = field(default_factory=list)
    last_rx_checksum: Optional[int] = None
    payload_bits: int = 0
    payload_bit_errors: int = 0
    payload_ber_ppm: int = 0
    payload_mismatch_frames: int = 0
    good_payload_frames: int = 0
    rx_total_observed: int = 0
    per_ppm: int = 0
    err_queued: int = 0
    err_printed: int = 0
    err_suppressed: int = 0
    power_mv: Dict[str, int] = field(default_factory=lambda: {k: 0 for k in POWER_KEYS})
    dac_mv: int = DAC_EXPECTED_MV
    rtc_time: str = "---- -- -- --:--:--"
    rtc_valid: int = 0
    rtc_source: str = "-"
    rtc_backup: str = "-"
    rtc_event: str = "none"
    raw_lines: deque = field(default_factory=lambda: deque(maxlen=3000))
    error_events: deque = field(default_factory=lambda: deque(maxlen=200))
    edge_history: deque = field(default_factory=lambda: deque(maxlen=320))
    start_time: float = field(default_factory=time.time)
    last_alive_time: Optional[float] = None
    last_rx_frames: int = 0
    rx_fps: float = 0.0
    hist_t: deque = field(default_factory=lambda: deque(maxlen=300))
    hist_power: Dict[str, deque] = field(default_factory=lambda: {k: deque(maxlen=300) for k in POWER_KEYS})
    hist_metric_t: deque = field(default_factory=lambda: deque(maxlen=300))
    hist_payload_ber_ppm: deque = field(default_factory=lambda: deque(maxlen=300))
    hist_per_ppm: deque = field(default_factory=lambda: deque(maxlen=300))


class OwcRxGui(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("OWC RX-ONLY Monitor - STM32F407 - Demo GUI")
        self.geometry("1500x900")
        self.minsize(1180, 760)
        self.state = RxState()
        self.q: "queue.Queue[str]" = queue.Queue()
        self.reader: Optional[SerialReader] = None
        self.max_bits = tk.IntVar(value=80)
        self._build_ui()
        self._refresh_ports()
        self.after(50, self._process_queue)
        self.after(250, self._refresh_ui)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        style = ttk.Style(self)
        style.configure("Header.TLabel", font=("Segoe UI", 11, "bold"))
        style.configure("Big.TLabel", font=("Consolas", 15, "bold"))
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
        ttk.Button(top, text="Inject sample log", command=self._inject_sample_log).pack(side=tk.LEFT, padx=(0, 12))
        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(top, textvariable=self.status_var).pack(side=tk.LEFT)
        self.nb = ttk.Notebook(self)
        self.nb.pack(fill=tk.BOTH, expand=True)
        self.rx_tab = ttk.Frame(self.nb)
        self.metrics_tab = ttk.Frame(self.nb)
        self.power_tab = ttk.Frame(self.nb)
        self.rtc_tab = ttk.Frame(self.nb)
        self.raw_tab = ttk.Frame(self.nb)
        self.nb.add(self.rx_tab, text="RX DEMO")
        self.nb.add(self.metrics_tab, text="LINK METRICS")
        self.nb.add(self.power_tab, text="POWER")
        self.nb.add(self.rtc_tab, text="RTC")
        self.nb.add(self.raw_tab, text="RAW LOG")
        self._build_rx_tab()
        self._build_metrics_tab()
        self._build_power_tab()
        self._build_rtc_tab()
        self._build_raw_tab()

    def _build_rx_tab(self):
        counters = ttk.Frame(self.rx_tab, padding=(10, 8))
        counters.pack(side=tk.TOP, fill=tk.X)
        self.role_var = tk.StringVar(value="role: UNKNOWN")
        ttk.Label(counters, textvariable=self.role_var, style="Header.TLabel").pack(side=tk.LEFT, padx=(0, 12))
        self.card_vars: Dict[str, tk.StringVar] = {}
        cards = [("rx_frames", "0"), ("frame_errors", "0"), ("checksum_errors", "0"), ("rx_sync_state", "0"), ("last_edge_delta", "0"), ("last_raw_bit", "0"), ("RX frame rate", "0.0 fps")]
        for title, default in cards:
            var = tk.StringVar(value=default)
            self.card_vars[title] = var
            box = ttk.Frame(counters, padding=5, relief="ridge")
            box.pack(side=tk.LEFT, padx=4)
            ttk.Label(box, text=title).pack()
            ttk.Label(box, textvariable=var, style="Big.TLabel").pack()
        state_bar = ttk.Frame(self.rx_tab, padding=(10, 0))
        state_bar.pack(side=tk.TOP, fill=tk.X)
        self.rx_state_var = tk.StringVar(value="RX state: WAIT_ACTIVITY")
        self.last_rx_var = tk.StringVar(value="Last RX: none")
        self.rtc_summary_var = tk.StringVar(value="RTC: waiting")
        ttk.Label(state_bar, textvariable=self.rx_state_var, style="Header.TLabel").pack(side=tk.LEFT, padx=(0, 25))
        ttk.Label(state_bar, textvariable=self.last_rx_var, style="Header.TLabel").pack(side=tk.LEFT, padx=(0, 25))
        ttk.Label(state_bar, textvariable=self.rtc_summary_var).pack(side=tk.LEFT)
        frame_box = ttk.LabelFrame(self.rx_tab, text="Expected TX Frame và Last RX Frame", padding=8)
        frame_box.pack(side=tk.TOP, fill=tk.X, padx=10, pady=(8, 6))
        self.expected_compact = ttk.Frame(frame_box)
        self.expected_compact.pack(side=tk.TOP, fill=tk.X, pady=(0, 4))
        self.rx_compact = ttk.Frame(frame_box)
        self.rx_compact.pack(side=tk.TOP, fill=tk.X, pady=(0, 4))
        self.compare_tree = ttk.Treeview(frame_box, columns=("expected", "received", "status", "note"), show="tree headings", height=5)
        self.compare_tree.heading("#0", text="Field")
        self.compare_tree.column("#0", width=110)
        for col, width in [("expected", 260), ("received", 260), ("status", 80), ("note", 420)]:
            self.compare_tree.heading(col, text=col.title())
            self.compare_tree.column(col, width=width, anchor=tk.W)
        self.compare_tree.tag_configure("ok", background="#eaffea")
        self.compare_tree.tag_configure("warn", background="#ffd9d9")
        self.compare_tree.tag_configure("na", background="#eeeeee")
        self.compare_tree.pack(side=tk.TOP, fill=tk.X, pady=(8, 0))
        body = ttk.Frame(self.rx_tab, padding=(10, 0))
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        left = ttk.LabelFrame(body, text="RX FSM / Debug", padding=8)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8))
        self.fsm_tree = ttk.Treeview(left, columns=("id", "description"), show="tree headings", height=8)
        self.fsm_tree.heading("id", text="State ID")
        self.fsm_tree.heading("description", text="State name")
        self.fsm_tree.column("id", width=70, anchor=tk.CENTER)
        self.fsm_tree.column("description", width=170)
        self.fsm_tree.tag_configure("active", background="#d7ecff")
        self.fsm_tree.pack(fill=tk.X)
        for sid, name in RX_STATE_NAMES.items():
            self.fsm_tree.insert("", "end", iid=str(sid), values=(sid, name))
        note = "RX_ONLY: chỉ chạy FSM nhận dữ liệu. GUI tập trung vào frame thu, lỗi frame, checksum, BER/PER, ADC/DAC và RTC."
        ttk.Label(left, text=note, wraplength=300).pack(anchor=tk.W, pady=(8, 0))
        right = ttk.LabelFrame(body, text="RX OOK Reconstruction / Expected Compare", padding=8)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        ctrl = ttk.Frame(right)
        ctrl.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(ctrl, text=f"RX reconstructed: edge_delta ≥ {EDGE_THRESHOLD} → bit 1, nhỏ hơn → bit 0").pack(side=tk.LEFT)
        ttk.Label(ctrl, text="Max bits:").pack(side=tk.LEFT, padx=(20, 4))
        ttk.Spinbox(ctrl, from_=16, to=160, increment=8, textvariable=self.max_bits, width=6).pack(side=tk.LEFT)
        self.fig = Figure(figsize=(9, 4.8), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _build_metrics_tab(self):
        top = ttk.Frame(self.metrics_tab, padding=10)
        top.pack(side=tk.TOP, fill=tk.X)
        self.metric_vars: Dict[str, tk.StringVar] = {}
        metrics = [("payload_bits", "0"), ("payload_bit_errors", "0"), ("payload_ber_ppm", "0 ppm"), ("payload_mismatch_frames", "0"), ("good_payload_frames", "0"), ("rx_total_observed", "0"), ("per_ppm", "0 ppm"), ("err_summary", "queued=0 printed=0 suppressed=0")]
        for title, default in metrics:
            var = tk.StringVar(value=default)
            self.metric_vars[title] = var
            box = ttk.Frame(top, padding=5, relief="ridge")
            box.pack(side=tk.LEFT, padx=4, pady=2)
            ttk.Label(box, text=title).pack()
            ttk.Label(box, textvariable=var, style="Big.TLabel").pack()
        middle = ttk.Frame(self.metrics_tab, padding=(10, 0))
        middle.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        chart_box = ttk.LabelFrame(middle, text="BER / PER History", padding=8)
        chart_box.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 8))
        self.mfig = Figure(figsize=(7, 4), dpi=100)
        self.max_metric = self.mfig.add_subplot(111)
        self.mcanvas = FigureCanvasTkAgg(self.mfig, master=chart_box)
        self.mcanvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        err_box = ttk.LabelFrame(middle, text="Captured error frames only", padding=8)
        err_box.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.err_tree = ttk.Treeview(err_box, columns=("kind", "frame_id", "summary"), show="headings", height=12)
        for col, width in [("kind", 100), ("frame_id", 110), ("summary", 520)]:
            self.err_tree.heading(col, text=col.title())
            self.err_tree.column(col, width=width, anchor=tk.W)
        self.err_tree.pack(fill=tk.BOTH, expand=True)
        self.err_detail = tk.Text(err_box, height=5, wrap=tk.WORD, font=("Consolas", 9))
        self.err_detail.pack(fill=tk.X, pady=(6, 0))
        self.err_tree.bind("<<TreeviewSelect>>", self._on_error_selected)

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
        for i, (label, var) in enumerate([("RTC time", self.rtc_time_var), ("Valid", self.rtc_valid_var), ("Source", self.rtc_source_var), ("Backup", self.rtc_backup_var), ("Last event", self.rtc_event_var)]):
            ttk.Label(top, text=f"{label}:", style="Header.TLabel").grid(row=i, column=0, sticky=tk.W, pady=4, padx=(0, 8))
            ttk.Label(top, textvariable=var, font=("Consolas", 12, "bold")).grid(row=i, column=1, sticky=tk.W, pady=4)
        cmd = ttk.LabelFrame(main, text="RTC Commands", padding=10)
        cmd.pack(side=tk.TOP, fill=tk.X, pady=(10, 8))
        ttk.Button(cmd, text="rtc_get", command=lambda: self._send_cmd("rtc_get")).pack(side=tk.LEFT, padx=3)
        ttk.Button(cmd, text="rtc_bkp", command=lambda: self._send_cmd("rtc_bkp")).pack(side=tk.LEFT, padx=3)
        ttk.Button(cmd, text="rtc_set from PC time", command=self._cmd_rtc_set_from_pc).pack(side=tk.LEFT, padx=3)
        content = ttk.Frame(main)
        content.pack(fill=tk.BOTH, expand=True)
        clock_box = ttk.LabelFrame(content, text="RTC Analog Clock", padding=8)
        clock_box.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 8))
        self.clock_canvas = tk.Canvas(clock_box, width=420, height=420, bg="white", highlightthickness=1, highlightbackground="#cccccc")
        self.clock_canvas.pack(fill=tk.BOTH, expand=True)
        cal_box = ttk.LabelFrame(content, text="Calendar", padding=8)
        cal_box.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.calendar_text = tk.Text(cal_box, width=42, height=18, font=("Consolas", 14), wrap=tk.NONE)
        self.calendar_text.pack(fill=tk.BOTH, expand=True)
        self.calendar_text.tag_configure("today", background="#d7ecff", foreground="#000000")

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

    def _send_cmd(self, cmd: str):
        if self.reader is None:
            messagebox.showinfo("Not connected", f"Chưa kết nối COM. Lệnh chưa gửi: {cmd}")
            return
        self.reader.write_line(cmd)

    def _cmd_rtc_set_from_pc(self):
        value = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        self._send_cmd("rtc_set " + value)

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
        elif line.startswith("alive_rx"):
            kv = parse_kv_int(line)
            self.state.rx_frames = kv.get("rx_frames", self.state.rx_frames)
            self.state.frame_errors = kv.get("frame_errors", self.state.frame_errors)
            self.state.checksum_errors = kv.get("checksum_errors", self.state.checksum_errors)
            self.state.rx_sync_state = kv.get("rx_sync_state", self.state.rx_sync_state)
            self.state.last_edge_delta = kv.get("last_edge_delta", self.state.last_edge_delta)
            self.state.last_raw_bit = kv.get("last_raw_bit", self.state.last_raw_bit)
            self.state.edge_history.append(self.state.last_edge_delta)
            self._update_rx_rate()
        elif line.startswith("expected_tx_frame"):
            kv = parse_kv_int(line)
            payload = parse_hex_field(line, "payload")
            frame = parse_hex_field(line, "frame")
            if payload:
                self.state.expected_payload = payload
            if frame:
                self.state.expected_frame = frame
            elif payload:
                self.state.expected_frame = build_frame(payload)
            if "checksum" in kv:
                self.state.expected_checksum = kv["checksum"]
            elif payload:
                self.state.expected_checksum = checksum(payload)
        elif line.startswith("last_rx"):
            if "none" in line:
                self.state.last_rx_none = True
                self.state.last_rx_payload = []
                self.state.last_rx_frame = []
                self.state.last_rx_checksum = None
            else:
                kv = parse_kv_int(line)
                payload = parse_hex_field(line, "payload")
                frame = parse_hex_field(line, "frame")
                self.state.last_rx_none = False
                if payload:
                    self.state.last_rx_payload = payload
                    self.state.last_rx_checksum = checksum(payload)
                if "checksum" in kv:
                    self.state.last_rx_checksum = kv["checksum"]
                if frame:
                    self.state.last_rx_frame = frame
                elif payload:
                    self.state.last_rx_frame = build_frame(payload)
        elif line.startswith("link_stats"):
            kv = parse_kv_int(line)
            self.state.payload_bits = kv.get("payload_bits", self.state.payload_bits)
            self.state.payload_bit_errors = kv.get("payload_bit_errors", self.state.payload_bit_errors)
            self.state.payload_ber_ppm = kv.get("payload_ber_ppm", self.state.payload_ber_ppm)
            self.state.payload_mismatch_frames = kv.get("payload_mismatch_frames", self.state.payload_mismatch_frames)
            self.state.good_payload_frames = kv.get("good_payload_frames", self.state.good_payload_frames)
            self._append_metric_history()
        elif line.startswith("link_quality"):
            kv = parse_kv_int(line)
            self.state.rx_total_observed = kv.get("rx_total_observed", self.state.rx_total_observed)
            self.state.per_ppm = kv.get("per_ppm", self.state.per_ppm)
            self._append_metric_history()
        elif line.startswith("err_summary"):
            kv = parse_kv_int(line)
            self.state.err_queued = kv.get("queued", self.state.err_queued)
            self.state.err_printed = kv.get("printed", self.state.err_printed)
            self.state.err_suppressed = kv.get("suppressed", self.state.err_suppressed)
        elif line.startswith("err_frame") or line.startswith("err_bits"):
            self._capture_error(line)
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

    def _update_rx_rate(self):
        now = time.time()
        if self.state.last_alive_time is not None:
            dt = max(now - self.state.last_alive_time, 1e-6)
            dframes = max(self.state.rx_frames - self.state.last_rx_frames, 0)
            self.state.rx_fps = dframes / dt
        self.state.last_alive_time = now
        self.state.last_rx_frames = self.state.rx_frames

    def _append_metric_history(self):
        t = time.time() - self.state.start_time
        if self.state.hist_metric_t and (t - self.state.hist_metric_t[-1]) < 0.20:
            return
        self.state.hist_metric_t.append(t)
        self.state.hist_payload_ber_ppm.append(self.state.payload_ber_ppm)
        self.state.hist_per_ppm.append(self.state.per_ppm)

    def _capture_error(self, line: str):
        kind = "err_bits" if line.startswith("err_bits") else "err_frame"
        kv = parse_kv_int(line)
        fid = kv.get("rx_frame_id", kv.get("frame_id", -1))
        self.state.error_events.appendleft({"kind": kind, "frame_id": fid, "summary": line, "raw": line})

    def _parse_rtc_line(self, line: str):
        m_time = re.search(r"time=(\d{4}-\d{2}-\d{2}\s+\d{2}:\d{2}:\d{2})", line)
        m_valid = re.search(r"valid=(\d+)", line)
        m_source = re.search(r"source=([^\s]+)", line)
        m_backup = re.search(r"backup=([^\s]+)", line)
        if m_time:
            self.state.rtc_time = m_time.group(1)
        if m_valid:
            self.state.rtc_valid = int(m_valid.group(1))
        if m_source:
            self.state.rtc_source = m_source.group(1)
        if m_backup:
            self.state.rtc_backup = m_backup.group(1)

    def _refresh_ui(self):
        st_name = RX_STATE_NAMES.get(self.state.rx_sync_state, f"UNKNOWN_{self.state.rx_sync_state}")
        self.role_var.set(f"role: {self.state.role}")
        for key in ["rx_frames", "frame_errors", "checksum_errors", "rx_sync_state", "last_edge_delta", "last_raw_bit"]:
            self.card_vars[key].set(str(getattr(self.state, key)))
        self.card_vars["RX frame rate"].set(f"{self.state.rx_fps:.1f} fps")
        self.rx_state_var.set(f"RX state: {st_name}")
        if self.state.last_rx_none:
            self.last_rx_var.set("Last RX: none")
        else:
            self.last_rx_var.set(f"Last RX: len={len(self.state.last_rx_payload)} payload={hex_bytes(self.state.last_rx_payload)}")
        self.rtc_summary_var.set(f"RTC: {self.state.rtc_time} | valid={self.state.rtc_valid} | source={self.state.rtc_source} | backup={self.state.rtc_backup}")
        self._refresh_frames()
        self._refresh_fsm()
        self._refresh_metrics()
        self._refresh_errors()
        self._refresh_power()
        self._refresh_rtc()
        self._redraw_rx_wave()
        self._redraw_metric_chart()
        self._redraw_power_chart()
        self._redraw_clock_and_calendar()
        self.after(250, self._refresh_ui)

    def _make_compact_frame(self, parent: ttk.Frame, label: str, frame: List[int], none: bool = False):
        for child in parent.winfo_children():
            child.destroy()
        ttk.Label(parent, text=label, width=12, style="Header.TLabel").pack(side=tk.LEFT)
        if none:
            ttk.Label(parent, text="none", font=("Consolas", 12, "bold")).pack(side=tk.LEFT)
            return
        fields = split_frame(frame)
        colors = {"Preamble": "#d7ecff", "Sync": "#ffe5c2", "Length": "#eadcff", "Payload": "#dff6df", "Checksum": "#ffd8d8"}
        for name in ["Preamble", "Sync", "Length", "Payload", "Checksum"]:
            vals = fields.get(name, [])
            txt = hex_bytes(vals) if vals else "--"
            tk.Label(parent, text=f"[{txt}]", bg=colors[name], fg="#111", font=("Consolas", 11, "bold"), padx=8, pady=4, relief="groove").pack(side=tk.LEFT, padx=(0, 4))
            tk.Label(parent, text=name, fg="#444", font=("Segoe UI", 8)).pack(side=tk.LEFT, padx=(0, 8))

    def _refresh_frames(self):
        self._make_compact_frame(self.expected_compact, "Expected:", self.state.expected_frame)
        self._make_compact_frame(self.rx_compact, "Last RX:", self.state.last_rx_frame, none=self.state.last_rx_none)
        for item in self.compare_tree.get_children():
            self.compare_tree.delete(item)
        exp = split_frame(self.state.expected_frame)
        rx = split_frame(self.state.last_rx_frame) if not self.state.last_rx_none else {}
        notes = {"Preamble": "0xAA giúp RX nhận hoạt động và đồng bộ ban đầu.", "Sync": "Expected D5; sai sync thường là mất đồng bộ bit/byte.", "Length": "Số byte payload.", "Payload": "Dữ liệu chính cần nhận đúng.", "Checksum": "Kiểm tra LEN + payload."}
        for field_name in ["Preamble", "Sync", "Length", "Payload", "Checksum"]:
            e = exp.get(field_name, [])
            r = rx.get(field_name, []) if not self.state.last_rx_none else []
            if self.state.last_rx_none:
                status, tag, recv = "N/A", "na", "--"
            else:
                status = "OK" if e == r else "MISMATCH"
                tag = "ok" if status == "OK" else "warn"
                recv = hex_bytes(r)
            self.compare_tree.insert("", "end", text=field_name, values=(hex_bytes(e), recv, status, notes[field_name]), tags=(tag,))

    def _refresh_fsm(self):
        for sid in RX_STATE_NAMES:
            self.fsm_tree.item(str(sid), tags=("active",) if sid == self.state.rx_sync_state else ())

    def _refresh_metrics(self):
        self.metric_vars["payload_bits"].set(str(self.state.payload_bits))
        self.metric_vars["payload_bit_errors"].set(str(self.state.payload_bit_errors))
        self.metric_vars["payload_ber_ppm"].set(f"{self.state.payload_ber_ppm} ppm")
        self.metric_vars["payload_mismatch_frames"].set(str(self.state.payload_mismatch_frames))
        self.metric_vars["good_payload_frames"].set(str(self.state.good_payload_frames))
        self.metric_vars["rx_total_observed"].set(str(self.state.rx_total_observed))
        self.metric_vars["per_ppm"].set(f"{self.state.per_ppm} ppm")
        self.metric_vars["err_summary"].set(f"queued={self.state.err_queued} printed={self.state.err_printed} suppressed={self.state.err_suppressed}")

    def _refresh_errors(self):
        for item in self.err_tree.get_children():
            self.err_tree.delete(item)
        for idx, ev in enumerate(self.state.error_events):
            self.err_tree.insert("", "end", iid=f"e{idx}", values=(ev["kind"], ev["frame_id"], ev["summary"][:220]))

    def _refresh_power(self):
        self.dac_var.set(f"DAC threshold: {self.state.dac_mv/1000.0:.3f} V")
        for key in POWER_KEYS:
            mv = self.state.power_mv.get(key, 0)
            exp, lo, hi = POWER_EXPECTED_MV[key]
            st = mv_status(key, mv)
            tag = "ok" if st == "OK" else "warn"
            self.power_tree.item(key, values=(f"{mv/1000:.3f} V", f"{exp/1000:.3f} V", f"{lo/1000:.2f}-{hi/1000:.2f} V", st), tags=(tag,))

    def _refresh_rtc(self):
        self.rtc_time_var.set(self.state.rtc_time)
        self.rtc_valid_var.set(str(self.state.rtc_valid))
        self.rtc_source_var.set(self.state.rtc_source)
        self.rtc_backup_var.set(self.state.rtc_backup)
        self.rtc_event_var.set(self.state.rtc_event)

    def _on_error_selected(self, _event=None):
        sel = self.err_tree.selection()
        if not sel:
            return
        idx = int(sel[0][1:]) if sel[0].startswith("e") else -1
        self.err_detail.delete("1.0", tk.END)
        if 0 <= idx < len(self.state.error_events):
            self.err_detail.insert(tk.END, self.state.error_events[idx]["raw"])

    def _redraw_rx_wave(self):
        self.ax.clear()
        self.ax.set_title("Expected TX bits vs RX reconstructed bits")
        self.ax.set_xlabel("Bit index")
        self.ax.set_ylabel("Logic level")
        self.ax.grid(True)
        try:
            max_bits = int(self.max_bits.get())
        except Exception:
            max_bits = 80
        exp_bits = bits_from_bytes(self.state.expected_frame)[:max_bits]
        edge_vals = list(self.state.edge_history)[-max_bits:]
        rx_bits = [1 if e >= EDGE_THRESHOLD else 0 for e in edge_vals]
        if exp_bits:
            x = list(range(len(exp_bits)))
            y = [b + 1.2 for b in exp_bits]
            self.ax.step(x, y, where="post", label="Expected TX bit pattern")
        if rx_bits:
            x2 = list(range(len(rx_bits)))
            y2 = [b for b in rx_bits]
            self.ax.step(x2, y2, where="post", label=f"RX reconstructed from edges ≥ {EDGE_THRESHOLD}")
            for i, edge in enumerate(edge_vals):
                if i < max_bits:
                    self.ax.text(i, -0.25, str(edge), ha="center", va="top", fontsize=7)
        if not rx_bits:
            self.ax.text(0.5, 0.45, "Waiting for alive_rx edge_delta samples...", transform=self.ax.transAxes, ha="center", va="center")
        self.ax.set_ylim(-0.45, 2.45)
        self.ax.set_xlim(0, max(max(len(exp_bits), len(rx_bits), 16) - 1, 16))
        self.ax.legend(loc="upper right")
        self.canvas.draw_idle()

    def _redraw_metric_chart(self):
        self.max_metric.clear()
        self.max_metric.set_title("Payload BER ppm / PER ppm")
        self.max_metric.set_xlabel("GUI time (s)")
        self.max_metric.set_ylabel("ppm")
        self.max_metric.grid(True)
        t = list(self.state.hist_metric_t)
        ber = list(self.state.hist_payload_ber_ppm)
        per = list(self.state.hist_per_ppm)
        n1, n2 = min(len(t), len(ber)), min(len(t), len(per))
        if n1 > 1:
            self.max_metric.plot(t[-n1:], ber[-n1:], label="payload_ber_ppm")
        if n2 > 1:
            self.max_metric.plot(t[-n2:], per[-n2:], label="per_ppm")
        if n1 > 1 or n2 > 1:
            self.max_metric.legend(loc="upper right")
        self.mcanvas.draw_idle()

    def _redraw_power_chart(self):
        self.pax.clear()
        self.pax.set_title("RX Board Power Monitor")
        self.pax.set_xlabel("Time (s)")
        self.pax.set_ylabel("Voltage (V)")
        self.pax.grid(True)
        t = list(self.state.hist_t)
        for key in ("vmon_5v_sys", "vmon_3v3_sys", "vmon_main_sys", "vmon_main", "vmon_bu", "vmon_bu_3v", "rx_out_a"):
            y = list(self.state.hist_power[key])
            n = min(len(t), len(y))
            if n > 1:
                self.pax.plot(t[-n:], y[-n:], label=key)
        if len(t) > 1:
            self.pax.legend(loc="upper right")
        self.pcanvas.draw_idle()

    def _parse_datetime(self) -> Optional[datetime]:
        try:
            return datetime.strptime(self.state.rtc_time, "%Y-%m-%d %H:%M:%S")
        except Exception:
            return None

    def _redraw_clock_and_calendar(self):
        dt = self._parse_datetime()
        self.clock_canvas.delete("all")
        w, h = max(self.clock_canvas.winfo_width(), 300), max(self.clock_canvas.winfo_height(), 300)
        cx, cy = w / 2, h / 2
        r = min(w, h) * 0.38
        self.clock_canvas.create_oval(cx - r, cy - r, cx + r, cy + r, width=3, outline="#0b2d5c")
        for i in range(60):
            ang = math.radians(i * 6 - 90)
            inner = r * (0.86 if i % 5 == 0 else 0.92)
            outer = r * 0.98
            self.clock_canvas.create_line(cx + inner * math.cos(ang), cy + inner * math.sin(ang), cx + outer * math.cos(ang), cy + outer * math.sin(ang), width=2 if i % 5 == 0 else 1, fill="#0b2d5c")
        for num in range(1, 13):
            ang = math.radians(num * 30 - 90)
            self.clock_canvas.create_text(cx + r * 0.72 * math.cos(ang), cy + r * 0.72 * math.sin(ang), text=str(num), font=("Segoe UI", 12, "bold"), fill="#0b2d5c")
        if dt:
            sec = dt.second
            minute = dt.minute + sec / 60.0
            hour = (dt.hour % 12) + minute / 60.0
            hands = [(hour * 30 - 90, r * 0.45, 5, "#0b2d5c"), (minute * 6 - 90, r * 0.66, 4, "#0b2d5c"), (sec * 6 - 90, r * 0.78, 2, "#b00020")]
            for deg, length, width, color in hands:
                ang = math.radians(deg)
                self.clock_canvas.create_line(cx, cy, cx + length * math.cos(ang), cy + length * math.sin(ang), width=width, fill=color)
            self.clock_canvas.create_oval(cx - 5, cy - 5, cx + 5, cy + 5, fill="#0b2d5c")
            self.clock_canvas.create_text(cx, cy + r + 30, text=dt.strftime("%Y-%m-%d  %H:%M:%S"), font=("Consolas", 16, "bold"), fill="#0b2d5c")
        else:
            self.clock_canvas.create_text(cx, cy, text="Waiting for RTC log", font=("Segoe UI", 16, "bold"), fill="#777777")
        self._update_calendar(dt)

    def _update_calendar(self, dt: Optional[datetime]):
        self.calendar_text.configure(state=tk.NORMAL)
        self.calendar_text.delete("1.0", tk.END)
        if not dt:
            self.calendar_text.insert(tk.END, "Waiting for RTC log...\n")
            self.calendar_text.configure(state=tk.DISABLED)
            return
        month_text = calendar.TextCalendar(firstweekday=0).formatmonth(dt.year, dt.month, w=4, l=1)
        self.calendar_text.insert(tk.END, month_text)
        day_s = f"{dt.day:2d}"
        idx = self.calendar_text.search(day_s, "1.0", tk.END)
        while idx:
            end = f"{idx}+{len(day_s)}c"
            if int(idx.split(".")[0]) >= 3:
                self.calendar_text.tag_add("today", idx, end)
                break
            idx = self.calendar_text.search(day_s, end, tk.END)
        self.calendar_text.configure(state=tk.DISABLED)

    def _append_raw(self, line: str):
        self.raw_text.insert(tk.END, line + "\n")
        if int(self.raw_text.index("end-1c").split(".")[0]) > 1800:
            self.raw_text.delete("1.0", "300.0")
        self.raw_text.see(tk.END)

    def _inject_sample_log(self):
        sample = """
expected_tx_frame len=4 payload=55 A5 3C C3 checksum=FD frame=AA AA AA D5 04 55 A5 3C C3 FD
last_rx none
link_stats payload_bits=0 payload_bit_errors=0 payload_ber_ppm=0 payload_mismatch_frames=0 good_payload_frames=0
link_quality rx_total_observed=0 per_ppm=0
err_summary queued=0 printed=0 suppressed=0
adc_mv rx_out_a=1801 vmon_bu=4912 vmon_bu_3v=3027 vmon_main_sys=9270 vmon_main=9353 vmon_5v_sys=5000 vmon_3v3_sys=3300
dac_mv threshold=1650
rtc time=2026-06-09 18:49:36 valid=1 source=LSE backup=ok
role board_role=RX_ONLY
alive_rx rx_frames=0 frame_errors=0 checksum_errors=0 rx_sync_state=0 last_edge_delta=0 last_raw_bit=0
expected_tx_frame len=4 payload=55 A5 3C C3 checksum=FD frame=AA AA AA D5 04 55 A5 3C C3 FD
last_rx none
link_stats payload_bits=0 payload_bit_errors=0 payload_ber_ppm=0 payload_mismatch_frames=0 good_payload_frames=0
link_quality rx_total_observed=0 per_ppm=0
err_summary queued=0 printed=0 suppressed=0
adc_mv rx_out_a=1809 vmon_bu=4928 vmon_bu_3v=3025 vmon_main_sys=9276 vmon_main=9372 vmon_5v_sys=5012 vmon_3v3_sys=3295
dac_mv threshold=1650
rtc time=2026-06-09 18:49:37 valid=1 source=LSE backup=ok
"""
        for line in sample.strip().splitlines():
            self._handle_line(line.strip())
        self.status_var.set("Injected sample RX log")

    def _on_close(self):
        if self.reader:
            self.reader.stop()
        self.destroy()


if __name__ == "__main__":
    OwcRxGui().mainloop()
