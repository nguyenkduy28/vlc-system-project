#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OWC UART Monitor v8 Demo
- Compact TX frame view: [Preamble] [Sync] [Length] [Payload] [Checksum]
- TX vs RX comparison table
- TX/RX OOK waveform comparison
- Error frame capture tab

Requires:
    pip install pyserial matplotlib
Run:
    python owc_uart_gui_v8_demo.py
"""

import re
import time
import queue
import threading
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import tkinter as tk
from tkinter import ttk, messagebox

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    raise SystemExit("Thiếu pyserial. Cài bằng: pip install pyserial") from exc

try:
    import matplotlib
    matplotlib.use("TkAgg")
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
except ImportError as exc:
    raise SystemExit("Thiếu matplotlib. Cài bằng: pip install matplotlib") from exc


# =====================
# Protocol configuration
# =====================
TX_PREAMBLE = [0xAA, 0xAA, 0xAA]
TX_SYNC = 0xD5
DEFAULT_PAYLOAD = [0x55, 0xA5, 0x3C, 0xC3]
RX_EDGE_THRESHOLD = 6
OOK_BIT_RATE_HZ = 100_000
OOK_CARRIER_HZ = 1_000_000
OOK_BIT_US = 1_000_000.0 / OOK_BIT_RATE_HZ
OOK_CARRIER_PERIOD_US = 1_000_000.0 / OOK_CARRIER_HZ

POWER_KEYS = [
    "rx_out_a", "vmon_bu", "vmon_bu_3v", "vmon_main_sys",
    "vmon_main", "vmon_5v_sys", "vmon_3v3_sys",
]
POWER_EXPECTED_MV: Dict[str, Tuple[int, int, int]] = {
    "rx_out_a":      (1800,    0, 3300),
    "vmon_bu":       (5000, 4500, 5500),
    "vmon_bu_3v":    (3000, 2800, 3300),
    "vmon_main_sys": (9400, 8500, 10000),
    "vmon_main":     (9400, 8500, 10000),
    "vmon_5v_sys":   (5000, 4750, 5250),
    "vmon_3v3_sys":  (3300, 3135, 3465),
}
DAC_EXPECTED_THRESHOLD_MV = 1650

ALIVE_KEYS = [
    "tx_frames", "rx_frames", "frame_errors", "checksum_errors",
    "rx_sync_state", "last_edge_delta", "last_raw_bit",
]
STATE_NAMES = {
    0: "WAIT_ACTIVITY", 1: "DETECT_PREAMBLE", 2: "LOCK_BIT_TIMING",
    3: "READ_SYNC", 4: "READ_LEN", 5: "READ_PAYLOAD", 6: "READ_CHECKSUM",
}

FIELD_NAMES = ["Preamble", "Sync", "Length", "Payload", "Checksum"]


# =====================
# Helpers
# =====================
def checksum(payload: List[int]) -> int:
    return (len(payload) + sum(payload)) & 0xFF


def build_frame(payload: List[int]) -> List[int]:
    return TX_PREAMBLE + [TX_SYNC, len(payload)] + list(payload) + [checksum(payload)]


def split_frame(frame: List[int]) -> Dict[str, List[int]]:
    if len(frame) < 6:
        return {
            "Preamble": frame[:3], "Sync": frame[3:4], "Length": frame[4:5],
            "Payload": [], "Checksum": frame[-1:] if frame else [],
        }
    length = frame[4]
    payload_start = 5
    payload_end = min(payload_start + length, max(len(frame) - 1, payload_start))
    return {
        "Preamble": frame[:3],
        "Sync": frame[3:4],
        "Length": frame[4:5],
        "Payload": frame[payload_start:payload_end],
        "Checksum": frame[payload_end:payload_end + 1] if payload_end < len(frame) else [],
    }


def hex_bytes(values: List[int]) -> str:
    return " ".join(f"{b:02X}" for b in values)


def bit_string(values: List[int]) -> str:
    return "".join(f"{b:08b}" for b in values)


def bits_from_bytes(values: List[int]) -> List[int]:
    bits: List[int] = []
    for b in values:
        for pos in range(7, -1, -1):
            bits.append((b >> pos) & 1)
    return bits


def parse_hex_bytes(s: str) -> List[int]:
    s = s.replace(",", " ").replace(";", " ").replace("-", " ")
    out: List[int] = []
    for tok in s.split():
        tok = tok.strip().replace("0x", "").replace("0X", "")
        if not re.fullmatch(r"[0-9A-Fa-f]{1,2}", tok):
            raise ValueError(f"Invalid hex byte: {tok}")
        out.append(int(tok, 16))
    if not out:
        raise ValueError("Payload rỗng")
    if len(out) > 255:
        raise ValueError("Payload tối đa 255 byte")
    return out


def parse_kv_int(line: str) -> Dict[str, int]:
    out: Dict[str, int] = {}
    for k, v in re.findall(r"([A-Za-z0-9_]+)=(-?\d+)", line):
        try:
            out[k] = int(v)
        except ValueError:
            pass
    return out


def parse_hex_field(line: str, field: str) -> List[int]:
    # Captures payload=55 A5 3C C3 until next key= or line end.
    m = re.search(rf"\b{re.escape(field)}=([0-9A-Fa-fxX ]+?)(?:\s+[A-Za-z0-9_]+=|$)", line)
    if not m:
        return []
    out: List[int] = []
    for tok in m.group(1).strip().split():
        tok = tok.replace("0x", "").replace("0X", "")
        if re.fullmatch(r"[0-9A-Fa-f]{1,2}", tok):
            out.append(int(tok, 16))
    return out


def parse_frame_field(line: str) -> List[int]:
    frame = parse_hex_field(line, "frame")
    if frame:
        return frame
    payload = parse_hex_field(line, "payload")
    if payload:
        return build_frame(payload)
    return []


def first_mismatch_bit(tx_bits: List[int], rx_bits: List[int]) -> Optional[int]:
    n = min(len(tx_bits), len(rx_bits))
    for i in range(n):
        if tx_bits[i] != rx_bits[i]:
            return i
    if len(tx_bits) != len(rx_bits):
        return n
    return None


def compare_frames(tx_frame: List[int], rx_frame: List[int]) -> Tuple[str, List[Tuple[str, str, str, str]], List[int]]:
    tx_fields = split_frame(tx_frame)
    rx_fields = split_frame(rx_frame)
    rows: List[Tuple[str, str, str, str]] = []
    mismatch_byte_indexes: List[int] = []
    byte_base = 0
    status_all = "OK"
    for name in FIELD_NAMES:
        tx = tx_fields.get(name, [])
        rx = rx_fields.get(name, [])
        ok = tx == rx
        if not ok:
            status_all = "MISMATCH"
            for i in range(max(len(tx), len(rx))):
                tv = tx[i] if i < len(tx) else None
                rv = rx[i] if i < len(rx) else None
                if tv != rv:
                    mismatch_byte_indexes.append(byte_base + i)
        rows.append((name, hex_bytes(tx), hex_bytes(rx), "OK" if ok else "ERROR"))
        byte_base += len(tx)
    return status_all, rows, mismatch_byte_indexes


def mv_status(key: str, mv: int) -> str:
    _, low, high = POWER_EXPECTED_MV[key]
    return "OK" if low <= mv <= high else "WARN"


@dataclass
class AppData:
    alive: Dict[str, int] = field(default_factory=lambda: {k: 0 for k in ALIVE_KEYS})
    power_mv: Dict[str, int] = field(default_factory=lambda: {k: 0 for k in POWER_KEYS})
    dac_threshold_mv: int = DAC_EXPECTED_THRESHOLD_MV
    tx_payload: List[int] = field(default_factory=lambda: list(DEFAULT_PAYLOAD))
    tx_frame: List[int] = field(default_factory=lambda: build_frame(DEFAULT_PAYLOAD))
    rx_frame: List[int] = field(default_factory=list)
    last_rx_payload: List[int] = field(default_factory=list)
    last_rx_text: str = "none"
    link_metrics: Dict[str, int] = field(default_factory=dict)
    link_quality: Dict[str, int] = field(default_factory=dict)
    raw_lines: deque = field(default_factory=lambda: deque(maxlen=1500))
    start_time: float = field(default_factory=time.time)
    hist_t: deque = field(default_factory=lambda: deque(maxlen=300))
    hist_power: Dict[str, deque] = field(default_factory=lambda: {k: deque(maxlen=300) for k in POWER_KEYS})
    hist_dac_t: deque = field(default_factory=lambda: deque(maxlen=300))
    hist_dac: deque = field(default_factory=lambda: deque(maxlen=300))
    hist_link_ber_ppm: deque = field(default_factory=lambda: deque(maxlen=300))
    hist_link_per_ppm: deque = field(default_factory=lambda: deque(maxlen=300))
    error_events: deque = field(default_factory=lambda: deque(maxlen=500))
    prev_frame_errors: int = 0
    prev_checksum_errors: int = 0
    prev_alive_time: Optional[float] = None
    prev_tx_frames: int = 0
    prev_rx_frames: int = 0


class SerialReader(threading.Thread):
    def __init__(self, port: str, baud: int, out_q: "queue.Queue[str]"):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.out_q = out_q
        self.stop_event = threading.Event()
        self.write_q: "queue.Queue[str]" = queue.Queue()
        self.ser: Optional[serial.Serial] = None

    def write_line(self, line: str):
        if not line.endswith("\n"):
            line += "\n"
        self.write_q.put(line)

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
            self.out_q.put(f"__STATUS__ Connected to {self.port} @ {self.baud}")
        except Exception as exc:
            self.out_q.put(f"__ERROR__ Cannot open {self.port}: {exc}")
            return
        while not self.stop_event.is_set():
            try:
                while not self.write_q.empty():
                    line = self.write_q.get_nowait()
                    if self.ser and self.ser.is_open:
                        self.ser.write(line.encode("ascii", errors="ignore"))
                        self.out_q.put(f"__STATUS__ Sent: {line.strip()}")
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                if line:
                    self.out_q.put(line)
            except Exception as exc:
                self.out_q.put(f"__ERROR__ Serial read error: {exc}")
                break
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        self.out_q.put("__STATUS__ Disconnected")

    def stop(self):
        self.stop_event.set()


class OwcGui(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("OWC UART Monitor - STM32F407 - v8 Demo Compare")
        self.geometry("1500x920")
        self.minsize(1220, 760)
        self.data = AppData()
        self.serial_q: "queue.Queue[str]" = queue.Queue()
        self.reader: Optional[SerialReader] = None
        self.wave_mode = tk.StringVar(value="Full frame")
        self.max_wave_bits = tk.IntVar(value=40)
        self.selected_power_keys = {k: tk.BooleanVar(value=(k != "rx_out_a")) for k in POWER_KEYS}

        self._setup_style()
        self._build_ui()
        self._refresh_ports()
        self.after(60, self._process_serial_queue)
        self.after(300, self._refresh_ui)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _setup_style(self):
        style = ttk.Style(self)
        style.configure("Big.TLabel", font=("Consolas", 16, "bold"))
        style.configure("Header.TLabel", font=("Segoe UI", 11, "bold"))
        style.configure("Ok.TLabel", foreground="#087d2c", font=("Segoe UI", 10, "bold"))
        style.configure("Warn.TLabel", foreground="#b00020", font=("Segoe UI", 10, "bold"))

    def _build_ui(self):
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

        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill=tk.BOTH, expand=True)
        self.power_tab = ttk.Frame(self.notebook)
        self.dac_tab = ttk.Frame(self.notebook)
        self.txrx_tab = ttk.Frame(self.notebook)
        self.link_tab = ttk.Frame(self.notebook)
        self.err_tab = ttk.Frame(self.notebook)
        self.raw_tab = ttk.Frame(self.notebook)
        for tab, name in [
            (self.power_tab, "POWER"), (self.dac_tab, "DAC"), (self.txrx_tab, "TX / RX DEMO"),
            (self.link_tab, "LINK METRICS"), (self.err_tab, "ERROR FRAMES"), (self.raw_tab, "RAW LOG")
        ]:
            self.notebook.add(tab, text=name)
        self._build_power_tab()
        self._build_dac_tab()
        self._build_txrx_tab()
        self._build_link_tab()
        self._build_error_tab()
        self._build_raw_tab()

    # ---------- POWER ----------
    def _build_power_tab(self):
        left = ttk.Frame(self.power_tab, padding=10)
        left.pack(side=tk.LEFT, fill=tk.Y)
        ttk.Label(left, text="Voltage Monitor", style="Header.TLabel").pack(anchor=tk.W, pady=(0, 8))
        cols = ("measured", "expected", "range", "status")
        self.power_tree = ttk.Treeview(left, columns=cols, show="tree headings", height=9)
        self.power_tree.heading("#0", text="Signal")
        self.power_tree.heading("measured", text="Measured")
        self.power_tree.heading("expected", text="Expected")
        self.power_tree.heading("range", text="Range")
        self.power_tree.heading("status", text="Status")
        self.power_tree.column("#0", width=140)
        for c in cols:
            self.power_tree.column(c, width=95, anchor=tk.CENTER)
        self.power_tree.tag_configure("ok", foreground="#087d2c")
        self.power_tree.tag_configure("warn", foreground="#b00020")
        for k in POWER_KEYS:
            exp, low, high = POWER_EXPECTED_MV[k]
            self.power_tree.insert("", "end", iid=k, text=k, values=("0.000 V", f"{exp/1000:.3f} V", f"{low/1000:.2f}-{high/1000:.2f} V", "WARN"), tags=("warn",))
        self.power_tree.pack(fill=tk.X)
        ttk.Separator(left).pack(fill=tk.X, pady=10)
        ttk.Label(left, text="Chart signals", style="Header.TLabel").pack(anchor=tk.W)
        for k, var in self.selected_power_keys.items():
            ttk.Checkbutton(left, text=k, variable=var).pack(anchor=tk.W)
        ttk.Button(left, text="Clear chart", command=self._clear_charts).pack(anchor=tk.W, pady=(12, 0))

        right = ttk.Frame(self.power_tab, padding=8)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.power_fig = Figure(figsize=(8, 5), dpi=100)
        self.power_ax = self.power_fig.add_subplot(111)
        self.power_canvas = FigureCanvasTkAgg(self.power_fig, master=right)
        self.power_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # ---------- DAC ----------
    def _build_dac_tab(self):
        top = ttk.Frame(self.dac_tab, padding=14)
        top.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(top, text="DAC Threshold Monitor", style="Header.TLabel").grid(row=0, column=0, columnspan=3, sticky=tk.W, pady=(0, 10))
        self.dac_mv_var = tk.StringVar(value="1.650 V")
        ttk.Label(top, text="Current threshold:").grid(row=1, column=0, sticky=tk.W, pady=4)
        ttk.Label(top, textvariable=self.dac_mv_var, style="Big.TLabel").grid(row=1, column=1, sticky=tk.W, pady=4)
        ttk.Label(top, text="Expected:").grid(row=2, column=0, sticky=tk.W, pady=4)
        ttk.Label(top, text=f"{DAC_EXPECTED_THRESHOLD_MV/1000:.3f} V").grid(row=2, column=1, sticky=tk.W, pady=4)
        bottom = ttk.Frame(self.dac_tab, padding=8)
        bottom.pack(fill=tk.BOTH, expand=True)
        self.dac_fig = Figure(figsize=(8, 4), dpi=100)
        self.dac_ax = self.dac_fig.add_subplot(111)
        self.dac_canvas = FigureCanvasTkAgg(self.dac_fig, master=bottom)
        self.dac_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # ---------- TX/RX ----------
    def _build_txrx_tab(self):
        # Counters strip
        counters = ttk.Frame(self.txrx_tab, padding=(10, 8))
        counters.pack(side=tk.TOP, fill=tk.X)
        self.alive_vars: Dict[str, tk.StringVar] = {}
        for k in ALIVE_KEYS:
            box = ttk.Frame(counters, padding=5, relief="ridge")
            box.pack(side=tk.LEFT, padx=5)
            ttk.Label(box, text=k).pack()
            var = tk.StringVar(value="0")
            self.alive_vars[k] = var
            ttk.Label(box, textvariable=var, style="Big.TLabel").pack()

        summary = ttk.Frame(self.txrx_tab, padding=(10, 0))
        summary.pack(side=tk.TOP, fill=tk.X)
        self.rx_state_var = tk.StringVar(value="RX state: WAIT_ACTIVITY")
        self.last_rx_var = tk.StringVar(value="Last RX: none")
        self.compare_status_var = tk.StringVar(value="Status: waiting for RX frame")
        ttk.Label(summary, textvariable=self.rx_state_var, style="Header.TLabel").pack(side=tk.LEFT, padx=(0, 20))
        ttk.Label(summary, textvariable=self.last_rx_var, style="Header.TLabel").pack(side=tk.LEFT, padx=(0, 20))
        self.compare_status_label = ttk.Label(summary, textvariable=self.compare_status_var, style="Warn.TLabel")
        self.compare_status_label.pack(side=tk.LEFT)

        # Compact frame view
        frame_box = ttk.LabelFrame(self.txrx_tab, text="Compact Frame View", padding=8)
        frame_box.pack(side=tk.TOP, fill=tk.X, padx=10, pady=(8, 6))
        entry_row = ttk.Frame(frame_box)
        entry_row.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(entry_row, text="Payload HEX:").pack(side=tk.LEFT)
        self.payload_var = tk.StringVar(value=hex_bytes(DEFAULT_PAYLOAD))
        ttk.Entry(entry_row, textvariable=self.payload_var, width=34, font=("Consolas", 10)).pack(side=tk.LEFT, padx=(5, 10))
        ttk.Button(entry_row, text="Update TX", command=self._update_payload_from_entry).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(entry_row, text="Send to MCU", command=self._send_payload_to_mcu).pack(side=tk.LEFT, padx=(0, 12))
        self.uart_cmd_var = tk.StringVar(value="UART command: tx_payload 55 A5 3C C3")
        ttk.Label(entry_row, textvariable=self.uart_cmd_var, font=("Consolas", 9)).pack(side=tk.LEFT)

        self.tx_compact = ttk.Frame(frame_box)
        self.tx_compact.pack(fill=tk.X, pady=2)
        self.rx_compact = ttk.Frame(frame_box)
        self.rx_compact.pack(fill=tk.X, pady=2)
        self.compact_labels: Dict[str, List[tk.Label]] = {"TX": [], "RX": []}
        self._build_compact_frame_widgets(self.tx_compact, "TX", self.data.tx_frame)
        self._build_compact_frame_widgets(self.rx_compact, "RX", [])

        # Compare table + waveform side by side
        body = ttk.Frame(self.txrx_tab, padding=(10, 0))
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        left = ttk.Frame(body)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8))
        comp = ttk.LabelFrame(left, text="TX vs RX Field Compare", padding=8)
        comp.pack(fill=tk.BOTH, expand=True)
        cols = ("field", "tx", "rx", "status")
        self.compare_tree = ttk.Treeview(comp, columns=cols, show="headings", height=8)
        for c, w in [("field", 90), ("tx", 210), ("rx", 210), ("status", 80)]:
            self.compare_tree.heading(c, text=c.upper())
            self.compare_tree.column(c, width=w, anchor=tk.W)
        self.compare_tree.tag_configure("ok", background="#eaffea")
        self.compare_tree.tag_configure("err", background="#ffd9d9")
        self.compare_tree.pack(fill=tk.BOTH, expand=True)

        self.mismatch_detail_var = tk.StringVar(value="No mismatch detail yet.")
        ttk.Label(comp, textvariable=self.mismatch_detail_var, wraplength=600).pack(anchor=tk.W, pady=(8, 0))

        right = ttk.Frame(body)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        wave_top = ttk.Frame(right)
        wave_top.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(wave_top, text="Waveform view:").pack(side=tk.LEFT)
        ttk.Combobox(wave_top, textvariable=self.wave_mode, values=["Full frame", "Payload only"], width=14, state="readonly").pack(side=tk.LEFT, padx=(4, 12))
        ttk.Label(wave_top, text="Max bits:").pack(side=tk.LEFT)
        ttk.Spinbox(wave_top, from_=8, to=96, increment=8, textvariable=self.max_wave_bits, width=6).pack(side=tk.LEFT, padx=(4, 0))

        self.wave_fig = Figure(figsize=(8.5, 4.5), dpi=100)
        self.tx_wave_ax = self.wave_fig.add_subplot(211)
        self.rx_wave_ax = self.wave_fig.add_subplot(212, sharex=self.tx_wave_ax)
        self.wave_canvas = FigureCanvasTkAgg(self.wave_fig, master=right)
        self.wave_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        metrics = ttk.LabelFrame(self.txrx_tab, text="System Evaluation Metrics", padding=8)
        metrics.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=(4, 8))
        self.metric_vars: Dict[str, tk.StringVar] = {}
        metric_items = [
            ("TX fps", "tx_frame_rate"), ("RX fps", "rx_frame_rate"),
            ("Goodput", "goodput_bps"), ("Frame success", "frame_success_rate"),
            ("PER", "frame_error_rate"), ("FW BER", "fw_payload_ber"),
            ("FW PER", "fw_per"), ("Edge margin", "edge_margin"),
        ]
        for i, (label, key) in enumerate(metric_items):
            ttk.Label(metrics, text=f"{label}:").grid(row=i//4, column=(i%4)*2, sticky=tk.W, padx=(0, 4), pady=2)
            var = tk.StringVar(value="0")
            self.metric_vars[key] = var
            ttk.Label(metrics, textvariable=var, font=("Consolas", 10, "bold")).grid(row=i//4, column=(i%4)*2+1, sticky=tk.W, padx=(0, 20), pady=2)

    def _build_compact_frame_widgets(self, parent: ttk.Frame, prefix: str, frame: List[int]):
        for child in parent.winfo_children():
            child.destroy()
        ttk.Label(parent, text=f"{prefix}:", width=4, style="Header.TLabel").pack(side=tk.LEFT)
        fields = split_frame(frame) if frame else {name: [] for name in FIELD_NAMES}
        colors = {
            "Preamble": "#d7ecff", "Sync": "#ffe5c2", "Length": "#eadcff",
            "Payload": "#dff6df", "Checksum": "#ffd8d8",
        }
        labels: List[tk.Label] = []
        for name in FIELD_NAMES:
            txt = hex_bytes(fields.get(name, [])) if fields.get(name) else "--"
            widget = tk.Label(parent, text=f"[{txt}]", bg=colors[name], fg="#111111", font=("Consolas", 11, "bold"), padx=6, pady=3, relief="groove")
            widget.pack(side=tk.LEFT, padx=(0, 4))
            labels.append(widget)
            sub = tk.Label(parent, text=name, fg="#444444", font=("Segoe UI", 8))
            sub.pack(side=tk.LEFT, padx=(0, 8))
        self.compact_labels[prefix] = labels

    # ---------- LINK TAB ----------
    def _build_link_tab(self):
        top = ttk.Frame(self.link_tab, padding=10)
        top.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(top, text="Firmware Link Statistics", style="Header.TLabel").pack(anchor=tk.W, pady=(0, 8))

        # Important: do not mix pack() and grid() in the same parent.
        # Header uses pack() in `top`; metric labels use grid() in this child frame.
        grid_frame = ttk.Frame(top)
        grid_frame.pack(anchor=tk.W, fill=tk.X)

        self.fw_metric_vars: Dict[str, tk.StringVar] = {}
        items = [
            ("Payload bits", "payload_bits"), ("Payload errors", "payload_bit_errors"),
            ("BER ppm", "payload_ber_ppm"), ("Mismatch frames", "payload_mismatch_frames"),
            ("Good frames", "good_payload_frames"), ("RX observed", "rx_total_observed"),
            ("PER ppm", "per_ppm"),
        ]
        for i, (label, key) in enumerate(items):
            ttk.Label(grid_frame, text=f"{label}:").grid(row=i//4, column=(i%4)*2, sticky=tk.W, padx=(0, 4), pady=3)
            var = tk.StringVar(value="0")
            self.fw_metric_vars[key] = var
            ttk.Label(grid_frame, textvariable=var, font=("Consolas", 12, "bold")).grid(row=i//4, column=(i%4)*2+1, sticky=tk.W, padx=(0, 20), pady=3)
        bottom = ttk.Frame(self.link_tab, padding=8)
        bottom.pack(fill=tk.BOTH, expand=True)
        self.link_fig = Figure(figsize=(8, 4), dpi=100)
        self.link_ax = self.link_fig.add_subplot(111)
        self.link_canvas = FigureCanvasTkAgg(self.link_fig, master=bottom)
        self.link_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # ---------- ERROR TAB ----------
    def _build_error_tab(self):
        frame = ttk.Frame(self.err_tab, padding=10)
        frame.pack(fill=tk.BOTH, expand=True)
        top = ttk.Frame(frame)
        top.pack(fill=tk.X)
        ttk.Label(top, text="Error Frame Capture - chỉ lưu frame lỗi", style="Header.TLabel").pack(side=tk.LEFT)
        ttk.Button(top, text="Clear", command=self._clear_errors).pack(side=tk.RIGHT)
        ttk.Label(frame, text="GUI sẽ parse err_frame nếu firmware log chi tiết. Nếu firmware chỉ có counter, GUI chỉ ghi nhận counter tăng.", wraplength=1300).pack(anchor=tk.W, pady=(6, 8))
        cols = ("time", "type", "tx_frame", "rx_frame", "state", "reason", "payload", "checksum", "detail")
        self.error_tree = ttk.Treeview(frame, columns=cols, show="headings", height=16)
        widths = {"time": 80, "type": 120, "tx_frame": 90, "rx_frame": 90, "state": 120, "reason": 150, "payload": 260, "checksum": 160, "detail": 420}
        for c in cols:
            self.error_tree.heading(c, text=c.upper())
            self.error_tree.column(c, width=widths[c], anchor=tk.W)
        self.error_tree.pack(fill=tk.BOTH, expand=True)
        self.error_detail_var = tk.StringVar(value="Click một lỗi để xem raw line.")
        ttk.Label(frame, textvariable=self.error_detail_var, font=("Consolas", 10), wraplength=1400).pack(anchor=tk.W, pady=(8, 0))
        self.error_tree.bind("<<TreeviewSelect>>", self._on_error_select)

    # ---------- RAW ----------
    def _build_raw_tab(self):
        frame = ttk.Frame(self.raw_tab, padding=8)
        frame.pack(fill=tk.BOTH, expand=True)
        self.raw_text = tk.Text(frame, height=20, wrap=tk.NONE, font=("Consolas", 10))
        self.raw_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        yscroll = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=self.raw_text.yview)
        yscroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.raw_text.configure(yscrollcommand=yscroll.set)

    # ---------- SERIAL ----------
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
            self.reader = SerialReader(port, baud, self.serial_q)
            self.reader.start()
            self.connect_btn.configure(text="Disconnect")
            self.status_var.set("Connecting...")
        else:
            self.reader.stop()
            self.reader = None
            self.connect_btn.configure(text="Connect")
            self.status_var.set("Disconnecting...")

    def _process_serial_queue(self):
        while True:
            try:
                line = self.serial_q.get_nowait()
            except queue.Empty:
                break
            if line.startswith("__ERROR__"):
                self.status_var.set(line.replace("__ERROR__ ", ""))
                self._append_raw(line)
            elif line.startswith("__STATUS__"):
                self.status_var.set(line.replace("__STATUS__ ", ""))
                self._append_raw(line)
                if "Disconnected" in line:
                    self.reader = None
                    self.connect_btn.configure(text="Connect")
            else:
                self._handle_line(line)
        self.after(60, self._process_serial_queue)

    # ---------- PARSING ----------
    def _handle_line(self, line: str):
        self.data.raw_lines.append(line)
        self._append_raw(line)
        if line.startswith("adc_mv"):
            kv = parse_kv_int(line)
            for k in POWER_KEYS:
                if k in kv:
                    self.data.power_mv[k] = kv[k]
            t = time.time() - self.data.start_time
            self.data.hist_t.append(t)
            for k in POWER_KEYS:
                self.data.hist_power[k].append(self.data.power_mv[k] / 1000.0)
        elif line.startswith("alive"):
            kv = parse_kv_int(line)
            for k in ALIVE_KEYS:
                if k in kv:
                    self.data.alive[k] = kv[k]
            self._detect_counter_errors(line)
            self._update_rates()
        elif line.startswith("last_rx"):
            self._parse_last_rx(line)
        elif line.startswith("tx_frame"):
            frame = parse_frame_field(line)
            payload = parse_hex_field(line, "payload")
            if payload:
                self.data.tx_payload = payload
            if frame:
                self.data.tx_frame = frame
        elif line.startswith("rx_frame"):
            frame = parse_frame_field(line)
            if frame:
                self.data.rx_frame = frame
        elif line.startswith("err_frame"):
            self._add_error_event(self._parse_err_frame(line))
        elif line.startswith("link_stats"):
            kv = parse_kv_int(line)
            self.data.link_metrics.update(kv)
            self.data.hist_link_ber_ppm.append(kv.get("payload_ber_ppm", self.data.link_metrics.get("payload_ber_ppm", 0)))
        elif line.startswith("link_quality"):
            kv = parse_kv_int(line)
            self.data.link_quality.update(kv)
            self.data.hist_link_per_ppm.append(kv.get("per_ppm", self.data.link_quality.get("per_ppm", 0)))
        elif line.startswith("dac"):
            kv = parse_kv_int(line)
            for key in ("threshold", "threshold_mv", "dac_threshold_mv", "dac_mv"):
                if key in kv:
                    self.data.dac_threshold_mv = kv[key]
                    break
            t = time.time() - self.data.start_time
            self.data.hist_dac_t.append(t)
            self.data.hist_dac.append(self.data.dac_threshold_mv / 1000.0)

    def _parse_last_rx(self, line: str):
        if "none" in line:
            self.data.last_rx_text = "none"
            return
        m = re.search(r"len=(\d+)", line)
        length = int(m.group(1)) if m else 0
        payload = parse_hex_field(line, "payload")
        self.data.last_rx_payload = payload
        self.data.last_rx_text = f"len={length} payload={hex_bytes(payload)}"
        self.data.rx_frame = build_frame(payload) if payload else []

    def _parse_err_frame(self, line: str) -> Dict[str, str]:
        ev: Dict[str, str] = {"raw": line, "time": f"{time.time() - self.data.start_time:.1f}s"}
        for k, v in re.findall(r"([A-Za-z0-9_]+)=([^\s]+)", line):
            ev[k] = v
        for field in ["payload", "rx_payload", "tx_payload", "frame"]:
            vals = parse_hex_field(line, field)
            if vals:
                ev[field] = hex_bytes(vals)
        return ev

    def _detect_counter_errors(self, raw_line: str):
        fe = int(self.data.alive.get("frame_errors", 0))
        ce = int(self.data.alive.get("checksum_errors", 0))
        if fe > self.data.prev_frame_errors:
            for _ in range(fe - self.data.prev_frame_errors):
                self._add_error_event({
                    "time": f"{time.time() - self.data.start_time:.1f}s", "type": "frame_error_counter",
                    "reason": "counter_increment_only", "raw": "frame_errors increased. Exact bad frame requires firmware err_frame log.",
                })
        if ce > self.data.prev_checksum_errors:
            for _ in range(ce - self.data.prev_checksum_errors):
                self._add_error_event({
                    "time": f"{time.time() - self.data.start_time:.1f}s", "type": "checksum_error_counter",
                    "reason": "counter_increment_only", "raw": "checksum_errors increased. Exact bad frame requires firmware err_frame log.",
                })
        self.data.prev_frame_errors = fe
        self.data.prev_checksum_errors = ce

    def _update_rates(self):
        now = time.time()
        tx = int(self.data.alive.get("tx_frames", 0))
        rx = int(self.data.alive.get("rx_frames", 0))
        total_errors = int(self.data.alive.get("frame_errors", 0)) + int(self.data.alive.get("checksum_errors", 0))
        attempts = rx + total_errors
        if self.data.prev_alive_time is not None:
            dt = max(now - self.data.prev_alive_time, 1e-6)
            dtx = max(tx - self.data.prev_tx_frames, 0)
            drx = max(rx - self.data.prev_rx_frames, 0)
            tx_fps = dtx / dt
            rx_fps = drx / dt
            self._set_metric("tx_frame_rate", tx_fps)
            self._set_metric("rx_frame_rate", rx_fps)
            self._set_metric("goodput_bps", rx_fps * max((len(self.data.tx_payload) * 8), 1))
        if attempts > 0:
            self._set_metric("frame_success_rate", 100.0 * rx / attempts)
            self._set_metric("frame_error_rate", 100.0 * total_errors / attempts)
        edge = int(self.data.alive.get("last_edge_delta", 0))
        self._set_metric("edge_margin", edge - RX_EDGE_THRESHOLD)
        if "payload_ber_ppm" in self.data.link_metrics:
            self._set_metric("fw_payload_ber", self.data.link_metrics["payload_ber_ppm"] / 1_000_000.0)
        if "per_ppm" in self.data.link_quality:
            self._set_metric("fw_per", self.data.link_quality["per_ppm"] / 1_000_000.0)
        self.data.prev_alive_time = now
        self.data.prev_tx_frames = tx
        self.data.prev_rx_frames = rx

    def _set_metric(self, key: str, value: float):
        if not hasattr(self, "runtime_metrics"):
            self.runtime_metrics = {}
        self.runtime_metrics[key] = value

    def _add_error_event(self, ev: Dict[str, str]):
        ev.setdefault("time", f"{time.time() - self.data.start_time:.1f}s")
        ev.setdefault("type", ev.get("type", "unknown"))
        ev.setdefault("tx_frame", str(self.data.alive.get("tx_frames", "")))
        ev.setdefault("rx_frame", str(self.data.alive.get("rx_frames", "")))
        ev.setdefault("state", STATE_NAMES.get(self.data.alive.get("rx_sync_state", 0), str(self.data.alive.get("rx_sync_state", ""))))
        ev.setdefault("reason", "")
        payload = ev.get("rx_payload") or ev.get("payload") or ""
        chks = ""
        if "rx_checksum" in ev or "checksum_rx" in ev:
            chks = f"rx={ev.get('rx_checksum', ev.get('checksum_rx', '?'))} calc={ev.get('tx_checksum', ev.get('checksum_calc', '?'))}"
        ev["payload_display"] = payload
        ev["checksum_display"] = chks
        self.data.error_events.append(ev)
        if hasattr(self, "error_tree"):
            iid = f"err{len(self.data.error_events)-1}"
            detail = ev.get("raw", "")[:500]
            self.error_tree.insert("", "end", iid=iid, values=(ev.get("time", ""), ev.get("type", ""), ev.get("tx_frame", ev.get("tx_frame_id", "")), ev.get("rx_frame", ev.get("rx_frame_id", "")), ev.get("state", ""), ev.get("reason", ""), ev.get("payload_display", ""), ev.get("checksum_display", ""), detail))
            self.error_tree.see(iid)

    def _append_raw(self, line: str):
        self.raw_text.insert(tk.END, line + "\n")
        line_count = int(self.raw_text.index("end-1c").split(".")[0])
        if line_count > 1500:
            self.raw_text.delete("1.0", "250.0")
        self.raw_text.see(tk.END)

    # ---------- UI ACTIONS ----------
    def _update_payload_from_entry(self):
        try:
            payload = parse_hex_bytes(self.payload_var.get())
        except ValueError as exc:
            messagebox.showwarning("Invalid payload", str(exc))
            return
        self.data.tx_payload = payload
        self.data.tx_frame = build_frame(payload)
        self.uart_cmd_var.set("UART command: tx_payload " + hex_bytes(payload))
        self._refresh_compare_ui()

    def _send_payload_to_mcu(self):
        self._update_payload_from_entry()
        cmd = "tx_payload " + hex_bytes(self.data.tx_payload)
        if self.reader is None:
            messagebox.showinfo("Not connected", "GUI đã cập nhật TX preview, nhưng chưa kết nối COM nên chưa gửi xuống MCU.")
            return
        self.reader.write_line(cmd)

    def _clear_errors(self):
        self.data.error_events.clear()
        for iid in self.error_tree.get_children():
            self.error_tree.delete(iid)
        self.error_detail_var.set("Đã xóa danh sách lỗi.")

    def _on_error_select(self, _event=None):
        sel = self.error_tree.selection()
        if not sel:
            return
        try:
            idx = int(sel[0].replace("err", ""))
            ev = list(self.data.error_events)[idx]
            self.error_detail_var.set(ev.get("raw", str(ev)))
        except Exception:
            self.error_detail_var.set(str(self.error_tree.item(sel[0], "values")))

    # ---------- REFRESH ----------
    def _refresh_ui(self):
        for k, var in self.alive_vars.items():
            var.set(str(self.data.alive.get(k, 0)))
        state = self.data.alive.get("rx_sync_state", 0)
        self.rx_state_var.set(f"RX state: {STATE_NAMES.get(state, f'STATE_{state}')} ({state})")
        self.last_rx_var.set(f"Last RX: {self.data.last_rx_text}")
        self.dac_mv_var.set(f"{self.data.dac_threshold_mv/1000.0:.3f} V")

        for k in POWER_KEYS:
            mv = self.data.power_mv.get(k, 0)
            exp, low, high = POWER_EXPECTED_MV[k]
            st = mv_status(k, mv)
            tag = "ok" if st == "OK" else "warn"
            self.power_tree.item(k, values=(f"{mv/1000:.3f} V", f"{exp/1000:.3f} V", f"{low/1000:.2f}-{high/1000:.2f} V", st), tags=(tag,))

        for key, var in self.metric_vars.items():
            val = getattr(self, "runtime_metrics", {}).get(key, 0.0)
            if key in ("fw_payload_ber", "fw_per"):
                var.set(f"{val:.3e}")
            elif key in ("frame_success_rate", "frame_error_rate"):
                var.set(f"{val:.3f}%")
            elif key == "goodput_bps":
                var.set(f"{val:.1f} bps")
            else:
                var.set(f"{val:.1f}")

        for key, var in self.fw_metric_vars.items():
            if key in self.data.link_metrics:
                var.set(str(self.data.link_metrics[key]))
            elif key in self.data.link_quality:
                var.set(str(self.data.link_quality[key]))

        self._refresh_compare_ui()
        self._redraw_power_chart()
        self._redraw_dac_chart()
        self._redraw_link_chart()
        self.after(300, self._refresh_ui)

    def _refresh_compare_ui(self):
        self._build_compact_frame_widgets(self.tx_compact, "TX", self.data.tx_frame)
        self._build_compact_frame_widgets(self.rx_compact, "RX", self.data.rx_frame)
        status, rows, mismatch_bytes = compare_frames(self.data.tx_frame, self.data.rx_frame) if self.data.rx_frame else ("NO_RX", [], [])
        for iid in self.compare_tree.get_children():
            self.compare_tree.delete(iid)
        if rows:
            for i, row in enumerate(rows):
                tag = "ok" if row[3] == "OK" else "err"
                self.compare_tree.insert("", "end", iid=f"cmp{i}", values=row, tags=(tag,))
        if status == "OK":
            self.compare_status_var.set("Status: TX == RX  |  Frame matched")
            self.compare_status_label.configure(style="Ok.TLabel")
            self.mismatch_detail_var.set("TX/RX map nhau. Không phát hiện mismatch ở frame cuối.")
        elif status == "MISMATCH":
            self.compare_status_var.set("Status: TX != RX  |  MISMATCH")
            self.compare_status_label.configure(style="Warn.TLabel")
            tx_bits = bits_from_bytes(self.data.tx_frame)
            rx_bits = bits_from_bytes(self.data.rx_frame)
            bit_idx = first_mismatch_bit(tx_bits, rx_bits)
            byte_idx = bit_idx // 8 if bit_idx is not None else None
            self.mismatch_detail_var.set(f"Mismatch bytes: {mismatch_bytes}. First mismatch bit={bit_idx}, byte={byte_idx}.")
        else:
            self.compare_status_var.set("Status: waiting for RX frame")
            self.compare_status_label.configure(style="Warn.TLabel")
            self.mismatch_detail_var.set("Chưa có RX frame để so sánh.")
        self._redraw_tx_rx_waveform()

    def _select_wave_bytes(self) -> Tuple[List[int], List[int], int]:
        if self.wave_mode.get() == "Payload only":
            tx = split_frame(self.data.tx_frame).get("Payload", [])
            rx = split_frame(self.data.rx_frame).get("Payload", []) if self.data.rx_frame else []
            bit_offset = 5 * 8
        else:
            tx = self.data.tx_frame
            rx = self.data.rx_frame
            bit_offset = 0
        return tx, rx, bit_offset

    def _draw_ook_axis(self, ax, bits: List[int], title: str, mismatch_positions: List[int]):
        ax.clear()
        ax.set_title(title)
        ax.set_ylabel("Carrier")
        ax.grid(True)
        max_bits = int(self.max_wave_bits.get())
        bits = bits[:max_bits]
        t_values: List[float] = []
        y_values: List[float] = []
        samples_per_carrier = 6
        for i, bit in enumerate(bits):
            t0 = i * OOK_BIT_US
            ax.axvline(t0, linewidth=0.4, linestyle="--", alpha=0.35)
            if i in mismatch_positions:
                ax.axvspan(t0, t0 + OOK_BIT_US, alpha=0.25)
            if bit == 1:
                total_samples = int((OOK_BIT_US / OOK_CARRIER_PERIOD_US) * samples_per_carrier)
                for n in range(total_samples + 1):
                    t = t0 + n * (OOK_CARRIER_PERIOD_US / samples_per_carrier)
                    phase = (t - t0) % OOK_CARRIER_PERIOD_US
                    y = 1.0 if phase < OOK_CARRIER_PERIOD_US / 2 else 0.0
                    t_values.append(t)
                    y_values.append(y)
            else:
                t_values.extend([t0, t0 + OOK_BIT_US])
                y_values.extend([0.0, 0.0])
            ax.text(t0 + OOK_BIT_US/2, 1.08, str(bit), ha="center", va="bottom", fontsize=7)
        if bits:
            ax.axvline(len(bits) * OOK_BIT_US, linewidth=0.4, linestyle="--", alpha=0.35)
        if len(t_values) > 1:
            ax.step(t_values, y_values, where="post")
        ax.set_ylim(-0.15, 1.3)
        ax.set_xlim(0, max(len(bits) * OOK_BIT_US, OOK_BIT_US))

    def _redraw_tx_rx_waveform(self):
        tx_bytes, rx_bytes, _ = self._select_wave_bytes()
        tx_bits = bits_from_bytes(tx_bytes)
        rx_bits = bits_from_bytes(rx_bytes) if rx_bytes else []
        mismatch_positions: List[int] = []
        for i in range(min(len(tx_bits), len(rx_bits), int(self.max_wave_bits.get()))):
            if tx_bits[i] != rx_bits[i]:
                mismatch_positions.append(i)
        if len(tx_bits) != len(rx_bits) and rx_bits:
            mismatch_positions.append(min(len(tx_bits), len(rx_bits)))
        self._draw_ook_axis(self.tx_wave_ax, tx_bits, "TX expected OOK", mismatch_positions)
        self._draw_ook_axis(self.rx_wave_ax, rx_bits if rx_bits else [], "RX decoded/reconstructed OOK", mismatch_positions)
        self.rx_wave_ax.set_xlabel("Time (µs), bit window = 10 µs")
        self.wave_fig.tight_layout()
        self.wave_canvas.draw_idle()

    def _redraw_power_chart(self):
        self.power_ax.clear()
        self.power_ax.set_title("Power Monitor")
        self.power_ax.set_xlabel("Time (s)")
        self.power_ax.set_ylabel("Voltage (V)")
        self.power_ax.grid(True)
        t = list(self.data.hist_t)
        for k, var in self.selected_power_keys.items():
            if not var.get():
                continue
            y = list(self.data.hist_power[k])
            n = min(len(t), len(y))
            if n > 1:
                self.power_ax.plot(t[-n:], y[-n:], label=k)
        if any(v.get() for v in self.selected_power_keys.values()):
            self.power_ax.legend(loc="upper right")
        self.power_canvas.draw_idle()

    def _redraw_dac_chart(self):
        self.dac_ax.clear()
        self.dac_ax.set_title("DAC Threshold")
        self.dac_ax.set_xlabel("Time (s)")
        self.dac_ax.set_ylabel("Voltage (V)")
        self.dac_ax.grid(True)
        if not self.data.hist_dac:
            self.data.hist_dac_t.append(time.time() - self.data.start_time)
            self.data.hist_dac.append(self.data.dac_threshold_mv / 1000.0)
        t = list(self.data.hist_dac_t)
        y = list(self.data.hist_dac)
        n = min(len(t), len(y))
        if n > 1:
            self.dac_ax.plot(t[-n:], y[-n:], label="threshold")
        self.dac_ax.axhline(DAC_EXPECTED_THRESHOLD_MV/1000.0, linestyle="--", linewidth=1, label="expected")
        self.dac_ax.set_ylim(0.0, 3.4)
        self.dac_ax.legend(loc="upper right")
        self.dac_canvas.draw_idle()

    def _redraw_link_chart(self):
        self.link_ax.clear()
        self.link_ax.set_title("Firmware BER/PER Metrics")
        self.link_ax.set_xlabel("Sample")
        self.link_ax.set_ylabel("ppm")
        self.link_ax.grid(True)
        ber = list(self.data.hist_link_ber_ppm)
        per = list(self.data.hist_link_per_ppm)
        if len(ber) > 1:
            self.link_ax.plot(range(len(ber)), ber, label="payload_ber_ppm")
        if len(per) > 1:
            self.link_ax.plot(range(len(per)), per, label="per_ppm")
        if len(ber) > 1 or len(per) > 1:
            self.link_ax.legend(loc="upper right")
        self.link_canvas.draw_idle()

    def _clear_charts(self):
        self.data.hist_t.clear()
        for d in self.data.hist_power.values():
            d.clear()
        self.data.hist_dac_t.clear()
        self.data.hist_dac.clear()
        self.data.hist_link_ber_ppm.clear()
        self.data.hist_link_per_ppm.clear()

    def _on_close(self):
        if self.reader:
            self.reader.stop()
        self.destroy()


def main():
    app = OwcGui()
    app.mainloop()


if __name__ == "__main__":
    main()
