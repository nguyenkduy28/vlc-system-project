#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OWC UART GUI v6
GUI đọc log UART từ STM32 OWC project và hiển thị:
- Tab POWER: voltage sense, giá trị expected/min/max, trạng thái OK/WARN, chart realtime
- Tab DAC: threshold DAC hiện tại/expected và chart threshold nếu firmware log ra
- Tab TX/RX: counter TX/RX, lỗi, FSM state, edge delta, raw bit, TX frame expected, RX payload
- Tab RAW LOG: log UART thô

Yêu cầu:
    pip install pyserial matplotlib

Chạy:
    python owc_uart_gui_v6.py

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


def parse_hex_bytes(text_value: str) -> List[int]:
    """Parse a user-entered hex byte string such as '55 A5 3C C3' or '0x55,0xA5'."""
    cleaned = text_value.replace(",", " ").replace(";", " ").replace("-", " ")
    tokens = [tok.strip() for tok in cleaned.split() if tok.strip()]
    values: List[int] = []
    for tok in tokens:
        tok = tok.replace("0x", "").replace("0X", "")
        if not re.fullmatch(r"[0-9A-Fa-f]{1,2}", tok):
            raise ValueError(f"Invalid hex byte: {tok}")
        values.append(int(tok, 16))
    if not values:
        raise ValueError("Payload rỗng")
    if len(values) > 255:
        raise ValueError("Payload tối đa 255 byte")
    return values


def build_frame_from_payload(payload: List[int]) -> List[int]:
    return TX_PREAMBLE + [TX_SYNC, len(payload)] + payload + [compute_checksum(payload)]


TX_CHECKSUM = compute_checksum(TX_PAYLOAD)
TX_FRAME = TX_PREAMBLE + [TX_SYNC, len(TX_PAYLOAD)] + TX_PAYLOAD + [TX_CHECKSUM]
TX_FRAME_HEX = " ".join(f"{b:02X}" for b in TX_FRAME)
TX_PAYLOAD_HEX = " ".join(f"{b:02X}" for b in TX_PAYLOAD)
TX_FRAME_BITS = len(TX_FRAME) * 8
TX_PAYLOAD_BITS = len(TX_PAYLOAD) * 8
RX_EDGE_THRESHOLD = 6

OOK_BIT_RATE_HZ = 100_000
OOK_CARRIER_HZ = 1_000_000
OOK_BIT_US = 1_000_000.0 / OOK_BIT_RATE_HZ
OOK_CARRIER_PERIOD_US = 1_000_000.0 / OOK_CARRIER_HZ

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
    # Firmware-side metrics parsed from link_stats/link_quality.
    "fw_payload_bits",
    "fw_payload_bit_errors",
    "fw_payload_ber_ppm",
    "fw_payload_mismatch_frames",
    "fw_good_payload_frames",
    "fw_rx_total_observed",
    "fw_per_ppm",
    "fw_payload_ber",
    "fw_per",
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

METRIC_CAPTIONS = {
    "tx_frames": {
        "title": "TX frames",
        "desc": "Số frame mà firmware phía phát đã gửi. Frame hiện tại gồm AA AA AA | D5 | LEN | PAYLOAD | CHECKSUM.",
        "formula": "tx_frames = số lần TX hoàn tất phát 1 frame đầy đủ.",
        "hint": "Nếu tx_frames tăng đều thì TIM2 ISR và TX đang chạy."
    },
    "rx_frames": {
        "title": "RX frames",
        "desc": "Số frame RX đã nhận hợp lệ, qua được preamble, sync, length, payload và checksum.",
        "formula": "rx_frames = số frame hợp lệ được FSM xác nhận result.has_frame = 1.",
        "hint": "rx_frames nên gần bằng tx_frames trong loopback tốt."
    },
    "frame_errors": {
        "title": "Frame errors",
        "desc": "Số lỗi cấu trúc frame trong FSM RX, ví dụ sai sync byte, length không hợp lệ hoặc mất đồng bộ frame.",
        "formula": "frame_errors tăng khi FSM phát hiện lỗi cấu trúc frame và reset về WAIT_ACTIVITY.",
        "hint": "Nếu tăng nhanh: kiểm tra preamble/sync, threshold edge, timing bit hoặc nhiễu ở PB6."
    },
    "checksum_errors": {
        "title": "Checksum errors",
        "desc": "Số frame có cấu trúc đọc được nhưng checksum tính lại không khớp checksum nhận được.",
        "formula": "checksum = length + sum(payload) modulo 256. checksum_errors tăng khi checksum_rx != checksum_calc.",
        "hint": "Nếu có checksum error lẻ tẻ: link có thể bị sai bit thoáng qua, threshold/timing sát ngưỡng hoặc nhiễu."
    },
    "rx_sync_state": {
        "title": "RX sync state",
        "desc": "Trạng thái hiện tại của FSM nhận dữ liệu.",
        "formula": "0 WAIT_ACTIVITY, 1 DETECT_PREAMBLE, 2 LOCK_BIT_TIMING, 3 READ_SYNC, 4 READ_LEN, 5 READ_PAYLOAD, 6 READ_CHECKSUM.",
        "hint": "Khi hệ thống đang chạy liên tục, state có thể nằm ở READ_PAYLOAD/READ_CHECKSUM tại thời điểm log."
    },
    "last_edge_delta": {
        "title": "Last edge delta",
        "desc": "Số cạnh carrier TIM4 đếm được trong cửa sổ bit gần nhất.",
        "formula": "delta_edges = TIM4_CNT_now - TIM4_CNT_previous. Nếu delta_edges >= 6 thì quyết định bit 1.",
        "hint": "Ở 100 kbps và carrier 1 MHz, bit 1 thường khoảng 8–10 cạnh, bit 0 khoảng 0–2 cạnh."
    },
    "last_raw_bit": {
        "title": "Last raw bit",
        "desc": "Bit RX sau khi so sánh last_edge_delta với threshold.",
        "formula": "last_raw_bit = 1 nếu last_edge_delta >= 6, ngược lại = 0.",
        "hint": "Thông số này là bit đưa vào FSM rx_sync."
    },
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
    hist_rx_bits: deque = field(default_factory=lambda: deque(maxlen=128))
    hist_rx_edge_for_bits: deque = field(default_factory=lambda: deque(maxlen=128))
    hist_dac_t: deque = field(default_factory=lambda: deque(maxlen=300))
    hist_dac: deque = field(default_factory=lambda: deque(maxlen=300))
    hist_metric_t: deque = field(default_factory=lambda: deque(maxlen=300))
    hist_fw_ber_ppm: deque = field(default_factory=lambda: deque(maxlen=300))
    hist_fw_per_ppm: deque = field(default_factory=lambda: deque(maxlen=300))
    hist_good_payload_frames: deque = field(default_factory=lambda: deque(maxlen=300))
    hist_payload_mismatch_frames: deque = field(default_factory=lambda: deque(maxlen=300))

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
        self.write_queue: "queue.Queue[str]" = queue.Queue()

    def write_line(self, line: str):
        if not line.endswith("\n"):
            line += "\n"
        self.write_queue.put(line)

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
            self.out_queue.put(f"__STATUS__ Connected to {self.port} @ {self.baud}")
        except Exception as exc:
            self.out_queue.put(f"__ERROR__ Cannot open {self.port}: {exc}")
            return

        while not self.stop_event.is_set():
            try:
                while not self.write_queue.empty():
                    tx_line = self.write_queue.get_nowait()
                    if self.ser and self.ser.is_open:
                        self.ser.write(tx_line.encode("ascii", errors="ignore"))
                        self.out_queue.put(f"__STATUS__ Sent: {tx_line.strip()}")

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
        self.title("OWC UART Monitor - STM32F407 - v7 Error Captions")
        self.geometry("1420x900")
        self.minsize(1180, 760)

        self.data = AppData()
        self.serial_q: "queue.Queue[str]" = queue.Queue()
        self.reader: Optional[SerialReader] = None
        self.selected_metric_key: Optional[str] = None

        self.selected_power_keys = {
            "vmon_5v_sys": tk.BooleanVar(value=True),
            "vmon_3v3_sys": tk.BooleanVar(value=True),
            "vmon_main_sys": tk.BooleanVar(value=True),
            "vmon_main": tk.BooleanVar(value=True),
            "vmon_bu": tk.BooleanVar(value=True),
            "vmon_bu_3v": tk.BooleanVar(value=True),
            "rx_out_a": tk.BooleanVar(value=False),
        }

        self.tx_payload_bytes: List[int] = list(TX_PAYLOAD)
        self.tx_frame_bytes: List[int] = build_frame_from_payload(self.tx_payload_bytes)

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
        self.link_tab = ttk.Frame(self.notebook)
        self.raw_tab = ttk.Frame(self.notebook)

        self.notebook.add(self.power_tab, text="POWER")
        self.notebook.add(self.dac_tab, text="DAC")
        self.notebook.add(self.txrx_tab, text="TX / RX")
        self.notebook.add(self.link_tab, text="LINK METRICS")
        self.notebook.add(self.raw_tab, text="RAW LOG")

        self._build_power_tab()
        self._build_dac_tab()
        self._build_txrx_tab()
        self._build_link_tab()
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
        self.alive_cards: Dict[str, ttk.Frame] = {}
        for key in ALIVE_KEYS:
            box = ttk.Frame(top, padding=4, relief="ridge")
            box.pack(side=tk.LEFT, padx=6, pady=4)
            self.alive_cards[key] = box

            label = ttk.Label(box, text=key, cursor="hand2")
            label.pack()
            var = tk.StringVar(value="0")
            self.alive_vars[key] = var
            value_label = ttk.Label(box, textvariable=var, style="Big.TLabel", cursor="hand2")
            value_label.pack()

            box.bind("<Button-1>", lambda e, k=key: self._show_metric_caption(k))
            label.bind("<Button-1>", lambda e, k=key: self._show_metric_caption(k))
            value_label.bind("<Button-1>", lambda e, k=key: self._show_metric_caption(k))

        mid = ttk.Frame(self.txrx_tab, padding=10)
        mid.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(mid, text="RX state:", style="Header.TLabel").pack(side=tk.LEFT)
        self.rx_state_name_var = tk.StringVar(value="WAIT_ACTIVITY")
        ttk.Label(mid, textvariable=self.rx_state_name_var, width=22).pack(side=tk.LEFT, padx=(4, 20))

        ttk.Label(mid, text="Last RX:", style="Header.TLabel").pack(side=tk.LEFT)
        self.last_rx_var = tk.StringVar(value="none")
        ttk.Label(mid, textvariable=self.last_rx_var, font=("Consolas", 11)).pack(side=tk.LEFT, padx=(4, 0))

        caption_frame = ttk.LabelFrame(self.txrx_tab, text="Metric Caption / Ý nghĩa thông số", padding=8)
        caption_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=(0, 8))

        self.metric_caption_title_var = tk.StringVar(value="Click vào frame_errors, checksum_errors hoặc thông số phía trên để xem giải thích")
        self.metric_caption_desc_var = tk.StringVar(value="Ví dụ: frame_errors là lỗi cấu trúc frame; checksum_errors là frame đọc được nhưng checksum sai.")
        self.metric_caption_formula_var = tk.StringVar(value="")
        self.metric_caption_hint_var = tk.StringVar(value="")

        ttk.Label(caption_frame, textvariable=self.metric_caption_title_var, style="Header.TLabel").pack(anchor=tk.W)
        ttk.Label(caption_frame, textvariable=self.metric_caption_desc_var, wraplength=1300).pack(anchor=tk.W, pady=(4, 0))
        ttk.Label(caption_frame, textvariable=self.metric_caption_formula_var, font=("Consolas", 10), wraplength=1300).pack(anchor=tk.W, pady=(4, 0))
        ttk.Label(caption_frame, textvariable=self.metric_caption_hint_var, wraplength=1300).pack(anchor=tk.W, pady=(4, 0))

        # TX frame control + expected frame panel
        tx_frame = ttk.LabelFrame(self.txrx_tab, text="TX Frame Input / Expected", padding=10)
        tx_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=(0, 8))

        ttk.Label(tx_frame, text="Payload HEX:", width=12).grid(row=0, column=0, sticky=tk.W, pady=2)
        self.tx_payload_entry_var = tk.StringVar(value=TX_PAYLOAD_HEX)
        ttk.Entry(tx_frame, textvariable=self.tx_payload_entry_var, width=42, font=("Consolas", 10)).grid(row=0, column=1, sticky=tk.W, pady=2)
        ttk.Button(tx_frame, text="Update Preview", command=self._update_tx_frame_from_entry).grid(row=0, column=2, sticky=tk.W, padx=(8, 0), pady=2)
        ttk.Button(tx_frame, text="Send to MCU", command=self._send_tx_payload_to_mcu).grid(row=0, column=3, sticky=tk.W, padx=(8, 0), pady=2)

        self.tx_command_var = tk.StringVar(value="UART command: tx_payload 55 A5 3C C3")
        ttk.Label(tx_frame, textvariable=self.tx_command_var, font=("Consolas", 9)).grid(row=1, column=1, columnspan=3, sticky=tk.W, pady=(0, 5))

        self.tx_frame_vars: Dict[str, tk.StringVar] = {
            "preamble": tk.StringVar(value=" ".join(f"{b:02X}" for b in TX_PREAMBLE)),
            "sync": tk.StringVar(value=f"{TX_SYNC:02X}"),
            "length": tk.StringVar(value=f"{len(TX_PAYLOAD):02X} ({len(TX_PAYLOAD)} bytes)"),
            "payload": tk.StringVar(value=TX_PAYLOAD_HEX),
            "checksum": tk.StringVar(value=f"{TX_CHECKSUM:02X}"),
            "full": tk.StringVar(value=TX_FRAME_HEX),
            "bits": tk.StringVar(value=""),
        }

        labels = [
            ("Preamble", "preamble"),
            ("Sync", "sync"),
            ("Length", "length"),
            ("Payload", "payload"),
            ("Checksum", "checksum"),
            ("Full frame", "full"),
            ("Bits preview", "bits"),
        ]
        for r, (name, key) in enumerate(labels, start=2):
            ttk.Label(tx_frame, text=f"{name}:", width=12).grid(row=r, column=0, sticky=tk.W, pady=2)
            ttk.Label(tx_frame, textvariable=self.tx_frame_vars[key], font=("Consolas", 10)).grid(row=r, column=1, columnspan=3, sticky=tk.W, pady=2)

        ttk.Label(
            tx_frame,
            text=("Send to MCU chỉ hoạt động khi firmware có parser UART nhận lệnh: tx_payload <hex bytes>. "
                  "Nếu chưa có parser, nút này chỉ gửi chuỗi lệnh ra UART."),
        ).grid(row=9, column=0, columnspan=4, sticky=tk.W, pady=(8, 0))

        self._refresh_tx_frame_labels()

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
            ("FW payload BER", "fw_payload_ber", ""),
            ("FW PER", "fw_per", ""),
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

        self.ook_fig = Figure(figsize=(8, 4), dpi=100)
        self.ook_ax = self.ook_fig.add_subplot(111)
        self.ook_ax.set_title("OOK Modulated Waveform Preview")
        self.ook_ax.set_xlabel("Time (µs)")
        self.ook_ax.set_ylabel("Carrier gate")
        self.ook_ax.grid(True)

        self.ook_canvas = FigureCanvasTkAgg(self.ook_fig, master=bottom)
        self.ook_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        # Compatibility aliases for RX reconstructed OOK chart.
        self.rx_ook_ax = self.ook_ax
        self.rx_ook_canvas = self.ook_canvas

    def _show_metric_caption(self, key: str):
        info = METRIC_CAPTIONS.get(key)
        if not info:
            return

        self.selected_metric_key = key
        value = self.data.alive.get(key, 0)

        title = f"{info['title']}  |  current value = {value}"
        self.metric_caption_title_var.set(title)
        self.metric_caption_desc_var.set(info["desc"])
        self.metric_caption_formula_var.set("Công thức/nguồn: " + info["formula"])
        self.metric_caption_hint_var.set("Gợi ý debug: " + info["hint"])

    def _refresh_tx_frame_labels(self):
        payload = self.tx_payload_bytes
        frame = build_frame_from_payload(payload)
        self.tx_frame_bytes = frame
        checksum = compute_checksum(payload)
        bits = "".join(f"{b:08b}" for b in frame)
        bits_preview = " ".join(bits[i:i+8] for i in range(0, min(len(bits), 96), 8))
        if len(bits) > 96:
            bits_preview += " ..."

        self.tx_frame_vars["preamble"].set(" ".join(f"{b:02X}" for b in TX_PREAMBLE))
        self.tx_frame_vars["sync"].set(f"{TX_SYNC:02X}")
        self.tx_frame_vars["length"].set(f"{len(payload):02X} ({len(payload)} bytes)")
        self.tx_frame_vars["payload"].set(" ".join(f"{b:02X}" for b in payload))
        self.tx_frame_vars["checksum"].set(f"{checksum:02X}")
        self.tx_frame_vars["full"].set(" ".join(f"{b:02X}" for b in frame))
        self.tx_frame_vars["bits"].set(bits_preview)
        self.tx_command_var.set("UART command: tx_payload " + " ".join(f"{b:02X}" for b in payload))

    def _update_tx_frame_from_entry(self):
        try:
            payload = parse_hex_bytes(self.tx_payload_entry_var.get())
        except ValueError as exc:
            messagebox.showwarning("Invalid payload", str(exc))
            return
        self.tx_payload_bytes = payload
        self._refresh_tx_frame_labels()
        self._redraw_rx_ook_from_edges()

    def _send_tx_payload_to_mcu(self):
        try:
            payload = parse_hex_bytes(self.tx_payload_entry_var.get())
        except ValueError as exc:
            messagebox.showwarning("Invalid payload", str(exc))
            return
        self.tx_payload_bytes = payload
        self._refresh_tx_frame_labels()
        cmd = "tx_payload " + " ".join(f"{b:02X}" for b in payload)
        if self.reader is None:
            messagebox.showinfo("Not connected", "GUI đã cập nhật preview, nhưng chưa kết nối COM nên chưa gửi lệnh xuống MCU.")
            return
        self.reader.write_line(cmd)

    def _build_link_tab(self):
        top = ttk.Frame(self.link_tab, padding=10)
        top.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(top, text="Firmware Link Statistics", style="Header.TLabel").pack(anchor=tk.W, pady=(0, 8))

        self.fw_metric_vars: Dict[str, tk.StringVar] = {}
        fw_layout = [
            ("Payload bits checked", "fw_payload_bits", "bits"),
            ("Payload bit errors", "fw_payload_bit_errors", "bits"),
            ("Payload BER", "fw_payload_ber", ""),
            ("Payload BER ppm", "fw_payload_ber_ppm", "ppm"),
            ("Good payload frames", "fw_good_payload_frames", "frames"),
            ("Payload mismatch frames", "fw_payload_mismatch_frames", "frames"),
            ("RX total observed", "fw_rx_total_observed", "frames"),
            ("PER", "fw_per", ""),
            ("PER ppm", "fw_per_ppm", "ppm"),
        ]

        grid = ttk.Frame(top)
        grid.pack(fill=tk.X)

        for i, (label, key, unit) in enumerate(fw_layout):
            r = i // 3
            c = (i % 3) * 3
            ttk.Label(grid, text=f"{label}:", width=24).grid(row=r, column=c, sticky=tk.W, padx=(0, 4), pady=4)
            var = tk.StringVar(value="0")
            self.fw_metric_vars[key] = var
            ttk.Label(grid, textvariable=var, font=("Consolas", 12, "bold"), width=16).grid(row=r, column=c + 1, sticky=tk.W, pady=4)
            ttk.Label(grid, text=unit, width=8).grid(row=r, column=c + 2, sticky=tk.W, padx=(2, 16), pady=4)

        ttk.Label(
            top,
            text=(
                "Firmware log mới được parse trực tiếp từ: link_stats payload_bits=... payload_bit_errors=... "
                "payload_ber_ppm=... và link_quality rx_total_observed=... per_ppm=...\\n"
                "BER này là payload BER trên các frame đã decode hợp lệ; frame lỗi checksum/sync được phản ánh bằng PER."
            ),
        ).pack(anchor=tk.W, pady=(8, 0))

        bottom = ttk.Frame(self.link_tab, padding=8)
        bottom.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.link_fig = Figure(figsize=(8, 4), dpi=100)
        self.link_ax = self.link_fig.add_subplot(111)
        self.link_ax.set_title("Firmware BER/PER Metrics")
        self.link_ax.set_xlabel("Sample")
        self.link_ax.set_ylabel("ppm")
        self.link_ax.grid(True)

        self.link_canvas = FigureCanvasTkAgg(self.link_fig, master=bottom)
        self.link_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

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

            edge_delta = self.data.alive.get("last_edge_delta", 0)
            raw_bit_from_edge = 1 if edge_delta >= RX_EDGE_THRESHOLD else 0
            self.data.hist_edge.append(edge_delta)
            self.data.hist_rx_bits.append(raw_bit_from_edge)
            self.data.hist_rx_edge_for_bits.append(edge_delta)
            self._update_counter_metrics()

        elif line.startswith("last_rx"):
            rx = parse_last_rx(line)
            if rx is not None:
                if not rx["has_rx"]:
                    self.data.last_rx_text = "none"
                else:
                    self.data.last_rx_text = f"len={rx['len']} payload={rx['payload']}"
                    self._update_payload_ber(str(rx["payload"]))

        elif line.startswith("link_stats"):
            kv = parse_key_values(line)
            if "payload_bits" in kv:
                self.data.metrics["fw_payload_bits"] = float(kv["payload_bits"])
            if "payload_bit_errors" in kv:
                self.data.metrics["fw_payload_bit_errors"] = float(kv["payload_bit_errors"])
            if "payload_ber_ppm" in kv:
                self.data.metrics["fw_payload_ber_ppm"] = float(kv["payload_ber_ppm"])
                self.data.metrics["fw_payload_ber"] = float(kv["payload_ber_ppm"]) / 1_000_000.0
            if "payload_mismatch_frames" in kv:
                self.data.metrics["fw_payload_mismatch_frames"] = float(kv["payload_mismatch_frames"])
            if "good_payload_frames" in kv:
                self.data.metrics["fw_good_payload_frames"] = float(kv["good_payload_frames"])

            t = time.time() - self.data.start_time
            self.data.hist_metric_t.append(t)
            self.data.hist_fw_ber_ppm.append(self.data.metrics.get("fw_payload_ber_ppm", 0.0))
            self.data.hist_good_payload_frames.append(self.data.metrics.get("fw_good_payload_frames", 0.0))
            self.data.hist_payload_mismatch_frames.append(self.data.metrics.get("fw_payload_mismatch_frames", 0.0))

        elif line.startswith("link_quality"):
            kv = parse_key_values(line)
            if "rx_total_observed" in kv:
                self.data.metrics["fw_rx_total_observed"] = float(kv["rx_total_observed"])
            if "per_ppm" in kv:
                self.data.metrics["fw_per_ppm"] = float(kv["per_ppm"])
                self.data.metrics["fw_per"] = float(kv["per_ppm"]) / 1_000_000.0

            # Keep PER history aligned with link_stats history.
            if len(self.data.hist_fw_per_ppm) < len(self.data.hist_metric_t):
                self.data.hist_fw_per_ppm.append(self.data.metrics.get("fw_per_ppm", 0.0))
            else:
                t = time.time() - self.data.start_time
                self.data.hist_metric_t.append(t)
                self.data.hist_fw_ber_ppm.append(self.data.metrics.get("fw_payload_ber_ppm", 0.0))
                self.data.hist_fw_per_ppm.append(self.data.metrics.get("fw_per_ppm", 0.0))
                self.data.hist_good_payload_frames.append(self.data.metrics.get("fw_good_payload_frames", 0.0))
                self.data.hist_payload_mismatch_frames.append(self.data.metrics.get("fw_payload_mismatch_frames", 0.0))

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

        if self.selected_metric_key:
            # Refresh caption current value without changing the selected explanation.
            self._show_metric_caption(self.selected_metric_key)

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
                elif key in ("last_payload_ber", "sampled_payload_ber", "fw_payload_ber", "fw_per"):
                    var.set(f"{value:.3e}")
                elif key in ("last_payload_bit_errors", "edge_margin"):
                    var.set(f"{value:.0f}")
                else:
                    var.set(f"{value:.1f}")

        if hasattr(self, "fw_metric_vars"):
            for key, var in self.fw_metric_vars.items():
                value = self.data.metrics.get(key, 0.0)
                if key in ("fw_payload_ber", "fw_per"):
                    var.set(f"{value:.3e}")
                elif key in ("fw_payload_ber_ppm", "fw_per_ppm"):
                    var.set(f"{value:.0f}")
                else:
                    var.set(f"{value:.0f}")

        self._redraw_power_chart()
        self._redraw_dac_chart()
        self._redraw_rx_ook_from_edges()
        if hasattr(self, "link_canvas"):
            self._redraw_link_chart()

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

    def _redraw_ook_waveform(self):
        self.ook_ax.clear()
        self.ook_ax.set_title(
            f"OOK Preview: bit 1 = 10 carrier cycles @ 1 MHz, bit 0 = OFF | bit time = {OOK_BIT_US:.0f} µs"
        )
        self.ook_ax.set_xlabel("Time (µs)")
        self.ook_ax.set_ylabel("Carrier gate")
        self.ook_ax.grid(True)

        frame = self.tx_frame_bytes if hasattr(self, "tx_frame_bytes") else TX_FRAME
        bits = []
        for byte in frame:
            bits.extend([(byte >> bit) & 0x01 for bit in range(7, -1, -1)])

        max_bits = 32
        bits = bits[:max_bits]
        t_values = []
        y_values = []
        samples_per_carrier = 8
        for bit_index, bit in enumerate(bits):
            t0 = bit_index * OOK_BIT_US
            self.ook_ax.axvline(t0, linewidth=0.6, linestyle="--")
            if bit == 1:
                total_samples = int((OOK_BIT_US / OOK_CARRIER_PERIOD_US) * samples_per_carrier)
                for n in range(total_samples + 1):
                    t = t0 + n * (OOK_CARRIER_PERIOD_US / samples_per_carrier)
                    phase = (t - t0) % OOK_CARRIER_PERIOD_US
                    y = 1.0 if phase < (OOK_CARRIER_PERIOD_US / 2.0) else 0.0
                    t_values.append(t)
                    y_values.append(y)
            else:
                t_values.extend([t0, t0 + OOK_BIT_US])
                y_values.extend([0.0, 0.0])
            self.ook_ax.text(t0 + OOK_BIT_US / 2.0, 1.12, str(bit), ha="center", va="bottom", fontsize=8)

        self.ook_ax.axvline(len(bits) * OOK_BIT_US, linewidth=0.6, linestyle="--")
        if len(t_values) > 1:
            self.ook_ax.step(t_values, y_values, where="post", label="OOK TX waveform")
        self.ook_ax.set_ylim(-0.2, 1.35)
        self.ook_ax.set_xlim(0, max(len(bits) * OOK_BIT_US, OOK_BIT_US))
        self.ook_ax.legend(loc="upper right")
        self.ook_canvas.draw_idle()

    def _redraw_rx_ook_from_edges(self):
        """
        Reconstruct RX OOK waveform from firmware UART log:
        last_edge_delta >= RX_EDGE_THRESHOLD -> bit 1 -> draw a 10-cycle carrier burst
        last_edge_delta <  RX_EDGE_THRESHOLD -> bit 0 -> draw an OFF bit window.

        This is an illustrative reconstruction from logged bit-window decisions, not a
        true oscilloscope capture of the PA8/PB6 waveform.
        """
        ax = getattr(self, "rx_ook_ax", None)
        canvas = getattr(self, "rx_ook_canvas", None)

        # Compatibility with v5 layout, where the chart was named ook_ax/ook_canvas.
        if ax is None:
            ax = getattr(self, "ook_ax", None)
        if canvas is None:
            canvas = getattr(self, "ook_canvas", None)

        # If the chart widget has not been constructed yet, skip this refresh safely.
        if ax is None or canvas is None:
            return

        ax.clear()
        ax.set_title(
            f"RX OOK from Edge Count: edge >= {RX_EDGE_THRESHOLD} -> bit 1, edge < {RX_EDGE_THRESHOLD} -> bit 0"
        )
        ax.set_xlabel("Bit window / time (µs, illustrative)")
        ax.set_ylabel("Carrier gate")
        ax.grid(True)

        bits = list(self.data.hist_rx_bits)
        edges = list(self.data.hist_rx_edge_for_bits)

        # Show the most recent bit decisions so the display behaves like a realtime strip chart.
        max_bits = 48
        if len(bits) > max_bits:
            bits = bits[-max_bits:]
            edges = edges[-max_bits:]

        t_values = []
        y_values = []
        samples_per_carrier = 8

        for bit_index, bit in enumerate(bits):
            t0 = bit_index * OOK_BIT_US
            ax.axvline(t0, linewidth=0.5, linestyle="--", alpha=0.6)

            if bit == 1:
                # At 100 kbps and 1 MHz carrier, one bit window contains about 10 carrier cycles.
                total_samples = int((OOK_BIT_US / OOK_CARRIER_PERIOD_US) * samples_per_carrier)
                for n in range(total_samples + 1):
                    t = t0 + n * (OOK_CARRIER_PERIOD_US / samples_per_carrier)
                    phase = (t - t0) % OOK_CARRIER_PERIOD_US
                    y = 1.0 if phase < (OOK_CARRIER_PERIOD_US / 2.0) else 0.0
                    t_values.append(t)
                    y_values.append(y)
            else:
                t_values.extend([t0, t0 + OOK_BIT_US])
                y_values.extend([0.0, 0.0])

            edge_txt = str(edges[bit_index]) if bit_index < len(edges) else "-"
            ax.text(
                t0 + OOK_BIT_US / 2.0,
                1.12,
                f"{bit}\n{edge_txt}",
                ha="center",
                va="bottom",
                fontsize=7,
            )

        if bits:
            ax.axvline(len(bits) * OOK_BIT_US, linewidth=0.5, linestyle="--", alpha=0.6)

        if len(t_values) > 1:
            ax.step(t_values, y_values, where="post", label="RX OOK reconstructed")

        ax.set_ylim(-0.2, 1.35)
        ax.set_xlim(0, max(len(bits) * OOK_BIT_US, OOK_BIT_US))
        ax.legend(loc="upper right")
        canvas.draw_idle()

    def _redraw_link_chart(self):
        self.link_ax.clear()
        self.link_ax.set_title("Firmware BER/PER Metrics")
        self.link_ax.set_xlabel("Sample")
        self.link_ax.set_ylabel("ppm")
        self.link_ax.grid(True)

        ber = list(self.data.hist_fw_ber_ppm)
        per = list(self.data.hist_fw_per_ppm)
        n = max(len(ber), len(per))
        if n > 1:
            x = list(range(n))
            if len(ber) > 1:
                xb = list(range(len(ber)))
                self.link_ax.plot(xb, ber, label="payload_ber_ppm")
            if len(per) > 1:
                xp = list(range(len(per)))
                self.link_ax.plot(xp, per, label="per_ppm")
            self.link_ax.legend(loc="upper right")

        self.link_canvas.draw_idle()

    def _clear_chart(self):
        self.data.hist_t.clear()
        self.data.hist_edge.clear()
        self.data.hist_rx_bits.clear()
        self.data.hist_rx_edge_for_bits.clear()
        self.data.hist_dac_t.clear()
        self.data.hist_dac.clear()
        self.data.hist_metric_t.clear()
        self.data.hist_fw_ber_ppm.clear()
        self.data.hist_fw_per_ppm.clear()
        self.data.hist_good_payload_frames.clear()
        self.data.hist_payload_mismatch_frames.clear()
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
