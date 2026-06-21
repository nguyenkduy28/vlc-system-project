#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OWC UART GUI v2
GUI đọc log UART từ STM32 OWC project và hiển thị:
- Tab POWER: voltage sense, giá trị expected/min/max, trạng thái OK/WARN, chart realtime
- Tab DAC: threshold DAC hiện tại/expected và chart threshold nếu firmware log ra
- Tab TX/RX: counter TX/RX, lỗi, FSM state, edge delta, raw bit, TX frame expected, RX payload
- Tab RAW LOG: log UART thô

Yêu cầu:
    pip install pyserial matplotlib

Chạy:
    python owc_uart_gui_v2.py

Log hỗ trợ ví dụ:
    alive tx_frames=86248 rx_frames=0 frame_errors=0 checksum_errors=0 rx_sync_state=0 last_edge_delta=0 last_raw_bit=0
    last_rx none
    adc_mv rx_out_a=1821 vmon_bu=4972 vmon_bu_3v=3019 vmon_main_sys=9363 vmon_main=9408 vmon_5v_sys=5020 vmon_3v3_sys=3304

Nếu firmware sau này có log DAC, GUI cũng parse được các dạng:
    dac_mv threshold=1650
    dac threshold_mv=1650
    dac_mv dac_threshold_mv=1650
"""

import re
import time
import queue
import threading
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, Optional, List, Tuple

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


# ==========================
# User project expectations
# ==========================

POWER_KEYS = [
    "rx_out_a",
    "vmon_bu",
    "vmon_bu_3v",
    "vmon_main_sys",
    "vmon_main",
    "vmon_5v_sys",
    "vmon_3v3_sys",
]

# Expected/min/max in mV.
# Bạn có thể chỉnh trực tiếp trong file này theo thực tế board.
POWER_EXPECTED_MV: Dict[str, Tuple[int, int, int]] = {
    # key              expected, min,  max
    "rx_out_a":        (1800,      0, 3300),   # analog RX monitor, chỉ tham khảo
    "vmon_bu":         (5000,   4500, 5500),   # hiện log của bạn khoảng 4.97 V
    "vmon_bu_3v":      (3000,   2800, 3300),
    "vmon_main_sys":   (9400,   8500,10000),
    "vmon_main":       (9400,   8500,10000),
    "vmon_5v_sys":     (5000,   4750, 5250),
    "vmon_3v3_sys":    (3300,   3135, 3465),
}

DAC_EXPECTED_THRESHOLD_MV = 1650
DAC_MIN_MV = 0
DAC_MAX_MV = 3300

# TX frame expected based on current firmware protocol.
TX_PREAMBLE = [0xAA, 0xAA, 0xAA]
TX_SYNC = 0xD5
TX_PAYLOAD = [0x55, 0xA5, 0x3C, 0xC3]


def compute_checksum(payload: List[int]) -> int:
    return (len(payload) + sum(payload)) & 0xFF


TX_CHECKSUM = compute_checksum(TX_PAYLOAD)
TX_FRAME = TX_PREAMBLE + [TX_SYNC, len(TX_PAYLOAD)] + TX_PAYLOAD + [TX_CHECKSUM]
TX_FRAME_HEX = " ".join(f"{b:02X}" for b in TX_FRAME)
TX_PAYLOAD_HEX = " ".join(f"{b:02X}" for b in TX_PAYLOAD)
TX_FRAME_BITS = len(TX_FRAME) * 8
TX_PAYLOAD_BITS = len(TX_PAYLOAD) * 8
RX_EDGE_THRESHOLD = 6

METRIC_KEYS = [
    "tx_frame_rate",
    "rx_frame_rate",
    "tx_bit_rate",
    "goodput_bps",
    "frame_success_rate",
    "frame_error_rate",
    "last_payload_bit_errors",
    "last_payload_ber",
    "sampled_payload_ber",
    "edge_margin",
]

ALIVE_KEYS = [
    "tx_frames",
    "rx_frames",
    "frame_errors",
    "checksum_errors",
    "rx_sync_state",
    "last_edge_delta",
    "last_raw_bit",
]

STATE_NAMES = {
    0: "WAIT_ACTIVITY",
    1: "DETECT_PREAMBLE",
    2: "LOCK_BIT_TIMING",
    3: "READ_SYNC",
    4: "READ_LEN",
    5: "READ_PAYLOAD",
    6: "READ_CHECKSUM",
}


def parse_key_values(line: str) -> Dict[str, int]:
    """Parse key=value pairs where value is integer decimal."""
    out: Dict[str, int] = {}
    for key, val in re.findall(r"([A-Za-z0-9_]+)=(-?\d+)", line):
        try:
            out[key] = int(val)
        except ValueError:
            pass
    return out


def parse_last_rx(line: str) -> Optional[Dict[str, object]]:
    """
    Parse:
        last_rx none
        last_rx len=4 payload=55 A5 3C C3
        last_rx len=4 payload=0x55 0xA5 ...
    """
    line = line.strip()
    if not line.startswith("last_rx"):
        return None
    if "none" in line:
        return {"has_rx": False, "len": 0, "payload": ""}

    m_len = re.search(r"len=(\d+)", line)
    m_payload = re.search(r"payload=([0-9A-Fa-fxX ]+)", line)
    length = int(m_len.group(1)) if m_len else 0
    payload = ""
    if m_payload:
        tokens = m_payload.group(1).strip().split()
        norm: List[str] = []
        for t in tokens:
            t = t.replace("0x", "").replace("0X", "")
            if re.fullmatch(r"[0-9A-Fa-f]{1,2}", t):
                norm.append(t.upper().zfill(2))
        payload = " ".join(norm)
    return {"has_rx": True, "len": length, "payload": payload}


def payload_bit_errors(payload_hex: str, expected: List[int]) -> Optional[Tuple[int, int]]:
    """Return bit errors and total bits for a received payload string."""
    if not payload_hex:
        return None
    try:
        rx = [int(tok, 16) for tok in payload_hex.split()]
    except ValueError:
        return None
    if len(rx) != len(expected):
        # Count length mismatch as full payload error for a clear warning.
        return (max(len(rx), len(expected)) * 8, max(len(rx), len(expected)) * 8)
    errors = 0
    for a, b in zip(rx, expected):
        errors += (a ^ b).bit_count()
    return errors, len(expected) * 8


def mv_status(key: str, mv: int) -> str:
    expected, low, high = POWER_EXPECTED_MV[key]
    if low <= mv <= high:
        return "OK"
    return "WARN"


def status_style(status: str) -> str:
    return "Ok.TLabel" if status == "OK" else "Warn.TLabel"


@dataclass
class AppData:
    power_mv: Dict[str, int] = field(default_factory=lambda: {k: 0 for k in POWER_KEYS})
    alive: Dict[str, int] = field(default_factory=lambda: {k: 0 for k in ALIVE_KEYS})
    dac_threshold_mv: int = DAC_EXPECTED_THRESHOLD_MV
    last_rx_text: str = "none"
    raw_lines: deque = field(default_factory=lambda: deque(maxlen=1000))
    start_time: float = field(default_factory=time.time)

    hist_t: deque = field(default_factory=lambda: deque(maxlen=300))
    hist_power: Dict[str, deque] = field(default_factory=lambda: {k: deque(maxlen=300) for k in POWER_KEYS})
    hist_edge: deque = field(default_factory=lambda: deque(maxlen=300))
    hist_dac_t: deque = field(default_factory=lambda: deque(maxlen=300))
    hist_dac: deque = field(default_factory=lambda: deque(maxlen=300))

    metrics: Dict[str, float] = field(default_factory=lambda: {k: 0.0 for k in METRIC_KEYS})
    prev_alive_time: Optional[float] = None
    prev_tx_frames: int = 0
    prev_rx_frames: int = 0
    prev_total_errors: int = 0
    sampled_payload_error_bits: int = 0
    sampled_payload_total_bits: int = 0
    last_processed_rx_count: int = 0


class SerialReader(threading.Thread):
    def __init__(self, port: str, baud: int, out_queue: "queue.Queue[str]"):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.out_queue = out_queue
        self.stop_event = threading.Event()
        self.ser: Optional[serial.Serial] = None

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
            self.out_queue.put(f"__STATUS__ Connected to {self.port} @ {self.baud}")
        except Exception as exc:
            self.out_queue.put(f"__ERROR__ Cannot open {self.port}: {exc}")
            return

        while not self.stop_event.is_set():
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                if line:
                    self.out_queue.put(line)
            except Exception as exc:
                self.out_queue.put(f"__ERROR__ Serial read error: {exc}")
                break

        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        self.out_queue.put("__STATUS__ Disconnected")

    def stop(self):
        self.stop_event.set()


class OwcGui(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("OWC UART Monitor - STM32F407")
        self.geometry("1360x860")
        self.minsize(1180, 760)

        self.data = AppData()
        self.serial_q: "queue.Queue[str]" = queue.Queue()
        self.reader: Optional[SerialReader] = None

        self.selected_power_keys = {
            "vmon_5v_sys": tk.BooleanVar(value=True),
            "vmon_3v3_sys": tk.BooleanVar(value=True),
            "vmon_main_sys": tk.BooleanVar(value=True),
            "vmon_main": tk.BooleanVar(value=True),
            "vmon_bu": tk.BooleanVar(value=True),
            "vmon_bu_3v": tk.BooleanVar(value=True),
            "rx_out_a": tk.BooleanVar(value=False),
        }

        self._build_ui()
        self._refresh_ports()
        self.after(50, self._process_serial_queue)
        self.after(300, self._refresh_ui)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------------- UI ----------------
    def _build_ui(self):
        style = ttk.Style(self)
        style.configure("Ok.TLabel", foreground="#0a7a22")
        style.configure("Warn.TLabel", foreground="#b00020")
        style.configure("Header.TLabel", font=("Segoe UI", 11, "bold"))
        style.configure("Big.TLabel", font=("Consolas", 16, "bold"))

        top = ttk.Frame(self, padding=8)
        top.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(top, text="COM:").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(top, width=18, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=(4, 8))

        ttk.Button(top, text="Refresh", command=self._refresh_ports).pack(side=tk.LEFT, padx=(0, 8))

        ttk.Label(top, text="Baud:").pack(side=tk.LEFT)
        self.baud_var = tk.StringVar(value="115200")
        ttk.Entry(top, textvariable=self.baud_var, width=10).pack(side=tk.LEFT, padx=(4, 8))

        self.connect_btn = ttk.Button(top, text="Connect", command=self._toggle_connect)
        self.connect_btn.pack(side=tk.LEFT, padx=(0, 8))

        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(top, textvariable=self.status_var).pack(side=tk.LEFT, padx=(10, 0))

        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill=tk.BOTH, expand=True)

        self.power_tab = ttk.Frame(self.notebook)
        self.dac_tab = ttk.Frame(self.notebook)
        self.txrx_tab = ttk.Frame(self.notebook)
        self.raw_tab = ttk.Frame(self.notebook)

        self.notebook.add(self.power_tab, text="POWER")
        self.notebook.add(self.dac_tab, text="DAC")
        self.notebook.add(self.txrx_tab, text="TX / RX")
        self.notebook.add(self.raw_tab, text="RAW LOG")

        self._build_power_tab()
        self._build_dac_tab()
        self._build_txrx_tab()
        self._build_raw_tab()

    def _build_power_tab(self):
        left = ttk.Frame(self.power_tab, padding=10)
        left.pack(side=tk.LEFT, fill=tk.Y)

        ttk.Label(left, text="Voltage Monitor", style="Header.TLabel").pack(anchor=tk.W, pady=(0, 8))

        # Treeview for measured + expected values
        columns = ("measured", "expected", "range", "status")
        self.power_tree = ttk.Treeview(left, columns=columns, show="headings", height=9)
        self.power_tree.heading("measured", text="Measured")
        self.power_tree.heading("expected", text="Expected")
        self.power_tree.heading("range", text="Range")
        self.power_tree.heading("status", text="Status")
        self.power_tree.column("measured", width=90, anchor=tk.E)
        self.power_tree.column("expected", width=90, anchor=tk.E)
        self.power_tree.column("range", width=125, anchor=tk.CENTER)
        self.power_tree.column("status", width=70, anchor=tk.CENTER)
        self.power_tree.pack(fill=tk.X)

        # Insert with text as signal name via iid display workaround
        self.power_items: Dict[str, str] = {}
        for key in POWER_KEYS:
            expected, low, high = POWER_EXPECTED_MV[key]
            iid = key
            self.power_tree.insert(
                "",
                "end",
                iid=iid,
                text=key,
                values=(
                    "0.000 V",
                    f"{expected/1000.0:.3f} V",
                    f"{low/1000.0:.2f}–{high/1000.0:.2f} V",
                    "WARN",
                ),
            )
            self.power_items[key] = iid

        # Add separate labels for signal names because headings-only hides tree column.
        ttk.Label(left, text="Signals / expected values are defined in POWER_EXPECTED_MV inside the Python file.").pack(
            anchor=tk.W, pady=(8, 8)
        )

        ttk.Separator(left).pack(fill=tk.X, pady=8)

        ttk.Label(left, text="Chart signals", style="Header.TLabel").pack(anchor=tk.W, pady=(0, 5))
        for key, var in self.selected_power_keys.items():
            ttk.Checkbutton(left, text=key, variable=var, command=self._redraw_power_chart).pack(anchor=tk.W)

        ttk.Button(left, text="Clear chart", command=self._clear_chart).pack(anchor=tk.W, pady=(12, 0))

        # Compact current values display
        ttk.Separator(left).pack(fill=tk.X, pady=8)
        self.power_vars: Dict[str, tk.StringVar] = {}
        self.power_status_labels: Dict[str, ttk.Label] = {}
        for key in POWER_KEYS:
            row = ttk.Frame(left)
            row.pack(fill=tk.X, pady=2)
            ttk.Label(row, text=key, width=18).pack(side=tk.LEFT)
            var = tk.StringVar(value="0.000 V")
            self.power_vars[key] = var
            ttk.Label(row, textvariable=var, width=10, anchor=tk.E).pack(side=tk.LEFT)
            st_lbl = ttk.Label(row, text="WARN", width=7, style="Warn.TLabel")
            st_lbl.pack(side=tk.LEFT, padx=(4, 0))
            self.power_status_labels[key] = st_lbl

        right = ttk.Frame(self.power_tab, padding=8)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.power_fig = Figure(figsize=(8, 5), dpi=100)
        self.power_ax = self.power_fig.add_subplot(111)
        self.power_ax.set_title("Power Monitor")
        self.power_ax.set_xlabel("Time (s)")
        self.power_ax.set_ylabel("Voltage (V)")
        self.power_ax.grid(True)

        self.power_canvas = FigureCanvasTkAgg(self.power_fig, master=right)
        self.power_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _build_dac_tab(self):
        top = ttk.Frame(self.dac_tab, padding=14)
        top.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(top, text="DAC Threshold Monitor", style="Header.TLabel").grid(row=0, column=0, columnspan=4, sticky=tk.W, pady=(0, 10))

        self.dac_measured_var = tk.StringVar(value=f"{DAC_EXPECTED_THRESHOLD_MV/1000.0:.3f} V")
        self.dac_expected_var = tk.StringVar(value=f"{DAC_EXPECTED_THRESHOLD_MV/1000.0:.3f} V")
        self.dac_status_var = tk.StringVar(value="EXPECTED")

        ttk.Label(top, text="Current / Logged threshold:").grid(row=1, column=0, sticky=tk.W, padx=(0, 8), pady=4)
        ttk.Label(top, textvariable=self.dac_measured_var, style="Big.TLabel").grid(row=1, column=1, sticky=tk.W, pady=4)

        ttk.Label(top, text="Expected threshold:").grid(row=2, column=0, sticky=tk.W, padx=(0, 8), pady=4)
        ttk.Label(top, textvariable=self.dac_expected_var, style="Big.TLabel").grid(row=2, column=1, sticky=tk.W, pady=4)

        ttk.Label(top, text="Range:").grid(row=3, column=0, sticky=tk.W, padx=(0, 8), pady=4)
        ttk.Label(top, text=f"{DAC_MIN_MV/1000.0:.3f}–{DAC_MAX_MV/1000.0:.3f} V").grid(row=3, column=1, sticky=tk.W, pady=4)

        ttk.Label(top, text="Status:").grid(row=4, column=0, sticky=tk.W, padx=(0, 8), pady=4)
        self.dac_status_label = ttk.Label(top, textvariable=self.dac_status_var, style="Ok.TLabel")
        self.dac_status_label.grid(row=4, column=1, sticky=tk.W, pady=4)

        ttk.Label(
            top,
            text=(
                "Ghi chú: nếu firmware chưa log DAC, GUI sẽ hiển thị threshold expected mặc định.\n"
                "Để GUI cập nhật runtime, firmware có thể log: dac_mv threshold=1650"
            ),
        ).grid(row=5, column=0, columnspan=4, sticky=tk.W, pady=(12, 0))

        bottom = ttk.Frame(self.dac_tab, padding=8)
        bottom.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.dac_fig = Figure(figsize=(8, 4), dpi=100)
        self.dac_ax = self.dac_fig.add_subplot(111)
        self.dac_ax.set_title("DAC Threshold")
        self.dac_ax.set_xlabel("Time (s)")
        self.dac_ax.set_ylabel("Threshold (V)")
        self.dac_ax.grid(True)

        self.dac_canvas = FigureCanvasTkAgg(self.dac_fig, master=bottom)
        self.dac_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _build_txrx_tab(self):
        top = ttk.Frame(self.txrx_tab, padding=10)
        top.pack(side=tk.TOP, fill=tk.X)

        self.alive_vars: Dict[str, tk.StringVar] = {}
        for key in ALIVE_KEYS:
            box = ttk.Frame(top)
            box.pack(side=tk.LEFT, padx=8, pady=4)
            ttk.Label(box, text=key).pack()
            var = tk.StringVar(value="0")
            self.alive_vars[key] = var
            ttk.Label(box, textvariable=var, style="Big.TLabel").pack()

        mid = ttk.Frame(self.txrx_tab, padding=10)
        mid.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(mid, text="RX state:", style="Header.TLabel").pack(side=tk.LEFT)
        self.rx_state_name_var = tk.StringVar(value="WAIT_ACTIVITY")
        ttk.Label(mid, textvariable=self.rx_state_name_var, width=22).pack(side=tk.LEFT, padx=(4, 20))

        ttk.Label(mid, text="Last RX:", style="Header.TLabel").pack(side=tk.LEFT)
        self.last_rx_var = tk.StringVar(value="none")
        ttk.Label(mid, textvariable=self.last_rx_var, font=("Consolas", 11)).pack(side=tk.LEFT, padx=(4, 0))

        # TX frame panel
        tx_frame = ttk.LabelFrame(self.txrx_tab, text="TX Frame Expected", padding=10)
        tx_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=(0, 8))

        self.tx_frame_vars: Dict[str, tk.StringVar] = {
            "preamble": tk.StringVar(value=" ".join(f"{b:02X}" for b in TX_PREAMBLE)),
            "sync": tk.StringVar(value=f"{TX_SYNC:02X}"),
            "length": tk.StringVar(value=f"{len(TX_PAYLOAD):02X} ({len(TX_PAYLOAD)} bytes)"),
            "payload": tk.StringVar(value=TX_PAYLOAD_HEX),
            "checksum": tk.StringVar(value=f"{TX_CHECKSUM:02X}"),
            "full": tk.StringVar(value=TX_FRAME_HEX),
        }

        labels = [
            ("Preamble", "preamble"),
            ("Sync", "sync"),
            ("Length", "length"),
            ("Payload", "payload"),
            ("Checksum", "checksum"),
            ("Full frame", "full"),
        ]
        for r, (name, key) in enumerate(labels):
            ttk.Label(tx_frame, text=f"{name}:", width=12).grid(row=r, column=0, sticky=tk.W, pady=2)
            ttk.Label(tx_frame, textvariable=self.tx_frame_vars[key], font=("Consolas", 10)).grid(row=r, column=1, sticky=tk.W, pady=2)

        ttk.Label(
            tx_frame,
            text="Nếu firmware sau này log TX frame runtime, có thể mở rộng parser dòng tx_frame ...",
        ).grid(row=6, column=0, columnspan=2, sticky=tk.W, pady=(8, 0))

        metrics_frame = ttk.LabelFrame(self.txrx_tab, text="System Evaluation Metrics", padding=10)
        metrics_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=(0, 8))

        self.metric_vars: Dict[str, tk.StringVar] = {}
        metric_layout = [
            ("TX frame rate", "tx_frame_rate", "fps"),
            ("RX frame rate", "rx_frame_rate", "fps"),
            ("TX bit rate approx", "tx_bit_rate", "bps"),
            ("Goodput payload", "goodput_bps", "bps"),
            ("Frame success", "frame_success_rate", "%"),
            ("Frame error/PER", "frame_error_rate", "%"),
            ("Last payload bit errors", "last_payload_bit_errors", "bits"),
            ("Last payload BER", "last_payload_ber", ""),
            ("Sampled payload BER", "sampled_payload_ber", ""),
            ("Edge margin", "edge_margin", "edges"),
        ]

        for i, (label, key, unit) in enumerate(metric_layout):
            r = i // 5
            c = (i % 5) * 3
            ttk.Label(metrics_frame, text=f"{label}:").grid(row=r, column=c, sticky=tk.W, padx=(0, 4), pady=3)
            var = tk.StringVar(value="0")
            self.metric_vars[key] = var
            ttk.Label(metrics_frame, textvariable=var, font=("Consolas", 11, "bold")).grid(row=r, column=c+1, sticky=tk.W, pady=3)
            ttk.Label(metrics_frame, text=unit).grid(row=r, column=c+2, sticky=tk.W, padx=(2, 12), pady=3)

        ttk.Label(
            metrics_frame,
            text=(
                "Ghi chú: PER/Frame error rate tính từ counter rx_frames/frame_errors/checksum_errors. "
                "BER thật cần firmware log bit_errors hoặc so sánh payload từng frame; GUI hiện tính BER mẫu từ last_rx hợp lệ."
            ),
        ).grid(row=2, column=0, columnspan=15, sticky=tk.W, pady=(8, 0))

        bottom = ttk.Frame(self.txrx_tab, padding=8)
        bottom.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.edge_fig = Figure(figsize=(8, 4), dpi=100)
        self.edge_ax = self.edge_fig.add_subplot(111)
        self.edge_ax.set_title("RX Edge Delta per Bit Window")
        self.edge_ax.set_xlabel("Sample")
        self.edge_ax.set_ylabel("Edges / bit")
        self.edge_ax.grid(True)

        self.edge_canvas = FigureCanvasTkAgg(self.edge_fig, master=bottom)
        self.edge_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _build_raw_tab(self):
        frame = ttk.Frame(self.raw_tab, padding=8)
        frame.pack(fill=tk.BOTH, expand=True)

        self.raw_text = tk.Text(frame, height=20, wrap=tk.NONE, font=("Consolas", 10))
        self.raw_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        yscroll = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=self.raw_text.yview)
        yscroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.raw_text.configure(yscrollcommand=yscroll.set)

    # ---------------- Serial ----------------
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
                continue
            if line.startswith("__STATUS__"):
                self.status_var.set(line.replace("__STATUS__ ", ""))
                self._append_raw(line)
                if "Disconnected" in line:
                    self.reader = None
                    self.connect_btn.configure(text="Connect")
                continue

            self._handle_line(line)

        self.after(50, self._process_serial_queue)

    # ---------------- Parsing ----------------
    def _handle_line(self, line: str):
        self.data.raw_lines.append(line)
        self._append_raw(line)

        if line.startswith("adc_mv"):
            kv = parse_key_values(line)
            for k in POWER_KEYS:
                if k in kv:
                    self.data.power_mv[k] = kv[k]

            t = time.time() - self.data.start_time
            self.data.hist_t.append(t)
            for k in POWER_KEYS:
                self.data.hist_power[k].append(self.data.power_mv[k] / 1000.0)

        elif line.startswith("alive"):
            kv = parse_key_values(line)
            for k in ALIVE_KEYS:
                if k in kv:
                    self.data.alive[k] = kv[k]

            self.data.hist_edge.append(self.data.alive.get("last_edge_delta", 0))
            self._update_counter_metrics()

        elif line.startswith("last_rx"):
            rx = parse_last_rx(line)
            if rx is not None:
                if not rx["has_rx"]:
                    self.data.last_rx_text = "none"
                else:
                    self.data.last_rx_text = f"len={rx['len']} payload={rx['payload']}"
                    self._update_payload_ber(str(rx["payload"]))

        elif line.startswith("dac"):
            kv = parse_key_values(line)
            # Support several names.
            for key in ("threshold", "threshold_mv", "dac_threshold_mv", "dac_mv"):
                if key in kv:
                    self.data.dac_threshold_mv = kv[key]
                    break
            t = time.time() - self.data.start_time
            self.data.hist_dac_t.append(t)
            self.data.hist_dac.append(self.data.dac_threshold_mv / 1000.0)

    def _update_counter_metrics(self):
        now = time.time()
        tx = int(self.data.alive.get("tx_frames", 0))
        rx = int(self.data.alive.get("rx_frames", 0))
        frame_errors = int(self.data.alive.get("frame_errors", 0))
        checksum_errors = int(self.data.alive.get("checksum_errors", 0))
        total_errors = frame_errors + checksum_errors
        attempts = rx + total_errors

        if self.data.prev_alive_time is not None:
            dt = max(now - self.data.prev_alive_time, 1e-6)
            dtx = max(tx - self.data.prev_tx_frames, 0)
            drx = max(rx - self.data.prev_rx_frames, 0)

            tx_fps = dtx / dt
            rx_fps = drx / dt
            self.data.metrics["tx_frame_rate"] = tx_fps
            self.data.metrics["rx_frame_rate"] = rx_fps
            self.data.metrics["tx_bit_rate"] = tx_fps * TX_FRAME_BITS
            self.data.metrics["goodput_bps"] = rx_fps * TX_PAYLOAD_BITS

        if attempts > 0:
            self.data.metrics["frame_success_rate"] = 100.0 * rx / attempts
            self.data.metrics["frame_error_rate"] = 100.0 * total_errors / attempts

        self.data.metrics["edge_margin"] = float(int(self.data.alive.get("last_edge_delta", 0)) - RX_EDGE_THRESHOLD)

        self.data.prev_alive_time = now
        self.data.prev_tx_frames = tx
        self.data.prev_rx_frames = rx
        self.data.prev_total_errors = total_errors

    def _update_payload_ber(self, payload_hex: str):
        rx_count = int(self.data.alive.get("rx_frames", 0))
        # Avoid counting the same last_rx line repeatedly when the firmware repeats "last_rx".
        if rx_count <= self.data.last_processed_rx_count:
            result = payload_bit_errors(payload_hex, TX_PAYLOAD)
            if result is not None:
                errors, total = result
                self.data.metrics["last_payload_bit_errors"] = float(errors)
                self.data.metrics["last_payload_ber"] = float(errors) / float(total) if total else 0.0
            return

        result = payload_bit_errors(payload_hex, TX_PAYLOAD)
        if result is None:
            return

        errors, total = result
        self.data.metrics["last_payload_bit_errors"] = float(errors)
        self.data.metrics["last_payload_ber"] = float(errors) / float(total) if total else 0.0
        self.data.sampled_payload_error_bits += errors
        self.data.sampled_payload_total_bits += total
        if self.data.sampled_payload_total_bits > 0:
            self.data.metrics["sampled_payload_ber"] = (
                self.data.sampled_payload_error_bits / self.data.sampled_payload_total_bits
            )
        self.data.last_processed_rx_count = rx_count

    def _append_raw(self, line: str):
        self.raw_text.insert(tk.END, line + "\n")
        line_count = int(self.raw_text.index("end-1c").split(".")[0])
        if line_count > 1200:
            self.raw_text.delete("1.0", "200.0")
        self.raw_text.see(tk.END)

    # ---------------- Refresh UI ----------------
    def _refresh_ui(self):
        for k, var in self.power_vars.items():
            mv = self.data.power_mv.get(k, 0)
            var.set(f"{mv / 1000.0:.3f} V")
            st = mv_status(k, mv)
            self.power_status_labels[k].configure(text=st, style=status_style(st))
            expected, low, high = POWER_EXPECTED_MV[k]
            self.power_tree.item(
                k,
                values=(
                    f"{mv/1000.0:.3f} V",
                    f"{expected/1000.0:.3f} V",
                    f"{low/1000.0:.2f}–{high/1000.0:.2f} V",
                    st,
                ),
            )

        for k, var in self.alive_vars.items():
            val = self.data.alive.get(k, 0)
            var.set(str(val))

        state = self.data.alive.get("rx_sync_state", 0)
        self.rx_state_name_var.set(STATE_NAMES.get(state, f"STATE_{state}"))
        self.last_rx_var.set(self.data.last_rx_text)

        self.dac_measured_var.set(f"{self.data.dac_threshold_mv/1000.0:.3f} V")
        self.dac_expected_var.set(f"{DAC_EXPECTED_THRESHOLD_MV/1000.0:.3f} V")
        if DAC_MIN_MV <= self.data.dac_threshold_mv <= DAC_MAX_MV:
            self.dac_status_var.set("OK")
            self.dac_status_label.configure(style="Ok.TLabel")
        else:
            self.dac_status_var.set("WARN")
            self.dac_status_label.configure(style="Warn.TLabel")

        # If firmware has not logged DAC yet, keep a flat expected trace.
        if len(self.data.hist_dac) == 0:
            t = time.time() - self.data.start_time
            self.data.hist_dac_t.append(t)
            self.data.hist_dac.append(self.data.dac_threshold_mv / 1000.0)

        if hasattr(self, "metric_vars"):
            for key, var in self.metric_vars.items():
                value = self.data.metrics.get(key, 0.0)
                if key in ("frame_success_rate", "frame_error_rate"):
                    var.set(f"{value:.3f}")
                elif key in ("last_payload_ber", "sampled_payload_ber"):
                    var.set(f"{value:.3e}")
                elif key in ("last_payload_bit_errors", "edge_margin"):
                    var.set(f"{value:.0f}")
                else:
                    var.set(f"{value:.1f}")

        self._redraw_power_chart()
        self._redraw_dac_chart()
        self._redraw_edge_chart()

        self.after(300, self._refresh_ui)

    def _redraw_power_chart(self):
        self.power_ax.clear()
        self.power_ax.set_title("Power Monitor")
        self.power_ax.set_xlabel("Time (s)")
        self.power_ax.set_ylabel("Voltage (V)")
        self.power_ax.grid(True)

        t = list(self.data.hist_t)
        for key, enabled in self.selected_power_keys.items():
            if enabled.get():
                y = list(self.data.hist_power[key])
                n = min(len(t), len(y))
                if n > 1:
                    self.power_ax.plot(t[-n:], y[-n:], label=key)

        # Draw expected rails as faint dashed lines for selected stable rails.
        for key, enabled in self.selected_power_keys.items():
            if enabled.get() and key in POWER_EXPECTED_MV:
                expected_mv, _, _ = POWER_EXPECTED_MV[key]
                if key != "rx_out_a":
                    self.power_ax.axhline(expected_mv / 1000.0, linestyle="--", linewidth=0.8)

        if any(v.get() for v in self.selected_power_keys.values()):
            self.power_ax.legend(loc="upper right")
        self.power_canvas.draw_idle()

    def _redraw_dac_chart(self):
        self.dac_ax.clear()
        self.dac_ax.set_title("DAC Threshold")
        self.dac_ax.set_xlabel("Time (s)")
        self.dac_ax.set_ylabel("Threshold (V)")
        self.dac_ax.grid(True)

        t = list(self.data.hist_dac_t)
        y = list(self.data.hist_dac)
        n = min(len(t), len(y))
        if n > 1:
            self.dac_ax.plot(t[-n:], y[-n:], label="dac_threshold")
        self.dac_ax.axhline(DAC_EXPECTED_THRESHOLD_MV / 1000.0, linestyle="--", linewidth=1, label="expected")
        self.dac_ax.set_ylim(0.0, 3.4)
        self.dac_ax.legend(loc="upper right")
        self.dac_canvas.draw_idle()

    def _redraw_edge_chart(self):
        self.edge_ax.clear()
        self.edge_ax.set_title("RX Edge Delta per Bit Window")
        self.edge_ax.set_xlabel("Sample")
        self.edge_ax.set_ylabel("Edges / bit")
        self.edge_ax.grid(True)

        y = list(self.data.hist_edge)
        if len(y) > 1:
            x = list(range(len(y)))
            self.edge_ax.plot(x, y, label="last_edge_delta")
            self.edge_ax.axhline(6, linestyle="--", linewidth=1, label="threshold=6")
            self.edge_ax.legend(loc="upper right")

        self.edge_canvas.draw_idle()

    def _clear_chart(self):
        self.data.hist_t.clear()
        self.data.hist_edge.clear()
        self.data.hist_dac_t.clear()
        self.data.hist_dac.clear()
        for d in self.data.hist_power.values():
            d.clear()

    def _on_close(self):
        if self.reader is not None:
            self.reader.stop()
        self.destroy()


def main():
    app = OwcGui()
    app.mainloop()


if __name__ == "__main__":
    main()
