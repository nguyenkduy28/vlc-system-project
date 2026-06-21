#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OWC TX/RX LOOPBACK GUI v2 - STM32F407

Designed for APP_BOARD_ROLE_TX_RX_LOOPBACK on ONE MCU.
Main tabs:
  1) TX CONTROL  : TX frame, OOK preview, TX commands
  2) RX / LINK   : RX frame, compare TX/RX, FSM, BER/PER/channel quality
Other tabs kept: POWER, RTC, RAW LOG

Requires:
    pip install pyserial matplotlib
Run:
    python owc_loopback_gui_v2_tx_rx_tabs.py
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


# ==========================
# Protocol / display config
# ==========================
PREAMBLE = [0xAA, 0xAA, 0xAA]
SYNC = 0xD5
DEFAULT_PAYLOAD = [0x55, 0xA5, 0x3C, 0xC3]
DAC_EXPECTED_MV = 1650
EDGE_THRESHOLD = 6

OOK_BIT_RATE_HZ = 100_000
OOK_CARRIER_HZ = 1_000_000
OOK_BIT_US = 1_000_000.0 / OOK_BIT_RATE_HZ
OOK_CARRIER_PERIOD_US = 1_000_000.0 / OOK_CARRIER_HZ

FIELD_NAMES = ["Preamble", "Sync", "Length", "Payload", "Checksum"]

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

RX_STATE_NAMES = {
    0: "WAIT_ACTIVITY",
    1: "DETECT_PREAMBLE",
    2: "LOCK_BIT_TIMING",
    3: "READ_SYNC",
    4: "READ_LEN",
    5: "READ_PAYLOAD",
    6: "READ_CHECKSUM",
}


# =========
# Helpers
# =========
def checksum(payload: List[int]) -> int:
    return (len(payload) + sum(payload)) & 0xFF


def build_frame(payload: List[int]) -> List[int]:
    return PREAMBLE + [SYNC, len(payload)] + list(payload) + [checksum(payload)]


def hex_bytes(values: List[int]) -> str:
    return " ".join(f"{v:02X}" for v in values)


def bits_from_bytes(values: List[int]) -> List[int]:
    out: List[int] = []
    for b in values:
        for pos in range(7, -1, -1):
            out.append((b >> pos) & 1)
    return out


def split_frame(frame: List[int]) -> Dict[str, List[int]]:
    if len(frame) < 6:
        return {
            "Preamble": frame[:3],
            "Sync": frame[3:4],
            "Length": frame[4:5],
            "Payload": [],
            "Checksum": frame[-1:] if frame else [],
        }
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


def parse_payload_entry(text: str) -> List[int]:
    toks = text.replace(",", " ").replace(";", " ").split()
    out: List[int] = []
    for tok in toks:
        tok = tok.replace("0x", "").replace("0X", "")
        if not re.fullmatch(r"[0-9A-Fa-f]{1,2}", tok):
            raise ValueError(f"Byte HEX không hợp lệ: {tok}")
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
    # Capture "payload=55 A5 3C C3" until next key=value or end of line.
    m = re.search(rf"(?:^|\s){re.escape(field)}=([0-9A-Fa-fxX ]+?)(?:\s+[A-Za-z0-9_]+=|$)", line)
    if not m:
        return []
    vals: List[int] = []
    for tok in m.group(1).strip().split():
        tok = tok.replace("0x", "").replace("0X", "")
        if re.fullmatch(r"[0-9A-Fa-f]{1,2}", tok):
            vals.append(int(tok, 16))
    return vals


def parse_frame_field(line: str) -> List[int]:
    frame = parse_hex_field(line, "frame")
    if frame:
        return frame
    payload = parse_hex_field(line, "payload")
    if payload:
        return build_frame(payload)
    return []


def normalize_line(line: str) -> str:
    line = line.strip()
    tokens = [
        "role ", "alive ", "alive_tx ", "alive_rx ",
        "tx_frame ", "tx_status ", "expected_tx_frame ", "rx_frame ", "last_rx ",
        "link_stats ", "link_quality ", "err_summary ",
        "err_frame ", "err_bits ",
        "adc_mv ", "dac_mv ", "rtc ", "rtc_event ", "rtc_bkp ",
        "cmd_ok ", "cmd_err ",
    ]
    positions = [(line.find(tok), tok) for tok in tokens if line.find(tok) >= 0]
    if not positions:
        return line
    pos, _ = min(positions, key=lambda x: x[0])
    return line[pos:]


def mv_status(key: str, mv: int) -> str:
    _exp, lo, hi = POWER_EXPECTED_MV[key]
    return "OK" if lo <= mv <= hi else "WARN"


def parse_int_list_csv(s: str) -> List[int]:
    out: List[int] = []
    for p in re.split(r"[,; ]+", s.strip()):
        if p and re.fullmatch(r"\d+", p):
            out.append(int(p))
    return out


# ==============
# Serial thread
# ==============
class SerialReader(threading.Thread):
    def __init__(self, port: str, baud: int, out_queue: "queue.Queue[str]"):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.out_queue = out_queue
        self.stop_event = threading.Event()
        self.write_queue: "queue.Queue[str]" = queue.Queue()
        self.ser: Optional[serial.Serial] = None
        self.rx_buffer = bytearray()

    def write_line(self, line: str):
        if not line.endswith("\n"):
            line += "\n"
        self.write_queue.put(line)

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
            try:
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
            except Exception:
                pass
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

                if not self.ser or not self.ser.is_open:
                    break
                chunk = self.ser.read(self.ser.in_waiting or 1)
                if not chunk:
                    continue

                self.rx_buffer.extend(chunk)
                if len(self.rx_buffer) > 4096:
                    line = self.rx_buffer.decode("utf-8", errors="replace").strip()
                    self.rx_buffer.clear()
                    if line:
                        self.out_queue.put(line)

                while b"\n" in self.rx_buffer:
                    line_raw, _, rest = self.rx_buffer.partition(b"\n")
                    self.rx_buffer = bytearray(rest)
                    line = line_raw.decode("utf-8", errors="replace").strip()
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


# ==========
# App state
# ==========
@dataclass
class LoopState:
    role: str = "UNKNOWN"
    tx_frames: int = 0
    tx_frame_id: int = 0
    bit_rate: int = 100_000
    tx_enabled: int = 1
    carrier_test: int = 0
    rx_frames: int = 0
    frame_errors: int = 0
    checksum_errors: int = 0
    rx_sync_state: int = 0
    last_edge_delta: int = 0
    last_raw_bit: int = 0

    tx_payload: List[int] = field(default_factory=lambda: list(DEFAULT_PAYLOAD))
    tx_frame: List[int] = field(default_factory=lambda: build_frame(DEFAULT_PAYLOAD))
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

    last_error_text: str = "none"
    last_err_bits_tx: str = ""
    last_err_bits_rx: str = ""
    last_err_positions: List[int] = field(default_factory=list)
    error_events: deque = field(default_factory=lambda: deque(maxlen=300))

    edge_history: deque = field(default_factory=lambda: deque(maxlen=320))
    raw_lines: deque = field(default_factory=lambda: deque(maxlen=3000))

    start_time: float = field(default_factory=time.time)
    last_alive_time: Optional[float] = None
    last_tx_frames: int = 0
    last_rx_frames: int = 0
    tx_fps: float = 0.0
    rx_fps: float = 0.0
    goodput_bps: float = 0.0

    hist_t: deque = field(default_factory=lambda: deque(maxlen=300))
    hist_power: Dict[str, deque] = field(default_factory=lambda: {k: deque(maxlen=300) for k in POWER_KEYS})
    hist_metric_t: deque = field(default_factory=lambda: deque(maxlen=300))
    hist_ber_ppm: deque = field(default_factory=lambda: deque(maxlen=300))
    hist_per_ppm: deque = field(default_factory=lambda: deque(maxlen=300))
    hist_tx_fps: deque = field(default_factory=lambda: deque(maxlen=300))
    hist_rx_fps: deque = field(default_factory=lambda: deque(maxlen=300))


# ========
# GUI app
# ========
class OwcLoopbackGui(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("OWC TX/RX LOOPBACK Monitor - STM32F407 - v2 TX/RX Tabs")
        self.geometry("1560x920")
        self.minsize(1220, 760)

        self.state = LoopState()
        self.q: "queue.Queue[str]" = queue.Queue()
        self.reader: Optional[SerialReader] = None
        self.max_tx_bits = tk.IntVar(value=80)
        self.max_rx_bits = tk.IntVar(value=80)
        self.rx_wave_mode = tk.StringVar(value="TX vs RX frame")
        self.payload_entry = tk.StringVar(value=hex_bytes(DEFAULT_PAYLOAD))
        self.selected_power_keys = {k: tk.BooleanVar(value=(k != "rx_out_a")) for k in POWER_KEYS}

        self._setup_style()
        self._build_ui()
        self._refresh_ports()
        self.after(50, self._process_queue)
        self.after(250, self._refresh_ui)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _setup_style(self):
        style = ttk.Style(self)
        style.configure("Header.TLabel", font=("Segoe UI", 11, "bold"))
        style.configure("Big.TLabel", font=("Consolas", 15, "bold"))
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
        ttk.Button(top, text="Inject sample", command=self._inject_sample_log).pack(side=tk.LEFT, padx=(0, 12))

        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(top, textvariable=self.status_var).pack(side=tk.LEFT)

        self.nb = ttk.Notebook(self)
        self.nb.pack(fill=tk.BOTH, expand=True)

        self.tx_tab = ttk.Frame(self.nb)
        self.rx_tab = ttk.Frame(self.nb)
        self.power_tab = ttk.Frame(self.nb)
        self.rtc_tab = ttk.Frame(self.nb)
        self.raw_tab = ttk.Frame(self.nb)

        self.nb.add(self.tx_tab, text="TX CONTROL")
        self.nb.add(self.rx_tab, text="RX / LINK STATE")
        self.nb.add(self.power_tab, text="POWER")
        self.nb.add(self.rtc_tab, text="RTC")
        self.nb.add(self.raw_tab, text="RAW LOG")

        self._build_tx_tab()
        self._build_rx_link_tab()
        self._build_power_tab()
        self._build_rtc_tab()
        self._build_raw_tab()

    # =================
    # TX TAB
    # =================
    def _build_tx_tab(self):
        counters = ttk.Frame(self.tx_tab, padding=(10, 8))
        counters.pack(side=tk.TOP, fill=tk.X)

        self.role_var = tk.StringVar(value="role: UNKNOWN")
        ttk.Label(counters, textvariable=self.role_var, style="Header.TLabel").pack(side=tk.LEFT, padx=(0, 12))

        self.tx_card_vars: Dict[str, tk.StringVar] = {}
        cards = [
            ("tx_frames", "0"),
            ("tx_frame_id", "0"),
            ("bit_rate", "100000 bps"),
            ("TX fps", "0.0"),
            ("tx_enabled", "1"),
            ("carrier_test", "0"),
            ("goodput", "0 bps"),
        ]
        for title, default in cards:
            var = tk.StringVar(value=default)
            self.tx_card_vars[title] = var
            box = ttk.Frame(counters, padding=5, relief="ridge")
            box.pack(side=tk.LEFT, padx=4)
            ttk.Label(box, text=title).pack()
            ttk.Label(box, textvariable=var, style="Big.TLabel").pack()

        frame_box = ttk.LabelFrame(self.tx_tab, text="TX Frame đang phát", padding=8)
        frame_box.pack(side=tk.TOP, fill=tk.X, padx=10, pady=(8, 6))
        self.tx_compact = ttk.Frame(frame_box)
        self.tx_compact.pack(fill=tk.X)
        self.tx_detail_var = tk.StringVar(value="TX frame: waiting")
        ttk.Label(frame_box, textvariable=self.tx_detail_var, wraplength=1400).pack(anchor=tk.W, pady=(8, 0))
        self.tx_bits_var = tk.StringVar(value="")
        ttk.Label(frame_box, textvariable=self.tx_bits_var, font=("Consolas", 9), wraplength=1400).pack(anchor=tk.W, pady=(4, 0))

        body = ttk.Frame(self.tx_tab, padding=(10, 0))
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        left = ttk.Frame(body)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8))

        fields_box = ttk.LabelFrame(left, text="TX Frame Fields", padding=8)
        fields_box.pack(fill=tk.BOTH, expand=True)
        self.tx_field_tree = ttk.Treeview(fields_box, columns=("value", "note"), show="tree headings", height=7)
        self.tx_field_tree.heading("#0", text="Field")
        self.tx_field_tree.heading("value", text="Value")
        self.tx_field_tree.heading("note", text="Note")
        self.tx_field_tree.column("#0", width=110)
        self.tx_field_tree.column("value", width=270)
        self.tx_field_tree.column("note", width=220)
        self.tx_field_tree.pack(fill=tk.BOTH, expand=True)

        control = ttk.LabelFrame(left, text="TX Control", padding=8)
        control.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(control, text="Payload HEX:").grid(row=0, column=0, sticky=tk.W, pady=3)
        ttk.Entry(control, textvariable=self.payload_entry, width=30, font=("Consolas", 10)).grid(row=0, column=1, columnspan=3, sticky=tk.W, pady=3)
        ttk.Button(control, text="Send Payload", command=self._cmd_payload).grid(row=0, column=4, sticky=tk.W, padx=(5, 0), pady=3)

        buttons = [
            ("Start TX", lambda: self._send_cmd("tx_start")),
            ("Stop TX", lambda: self._send_cmd("tx_stop")),
            ("Carrier ON", lambda: self._send_cmd("tx_carrier_on")),
            ("Carrier OFF", lambda: self._send_cmd("tx_carrier_off")),
            ("Single Frame", lambda: self._send_cmd("tx_single")),
            ("Status", lambda: self._send_cmd("tx_status")),
        ]
        for idx, (label, func) in enumerate(buttons):
            ttk.Button(control, text=label, command=func).grid(row=1 + idx // 3, column=idx % 3, sticky=tk.EW, padx=3, pady=3)

        self.cmd_var = tk.StringVar(value="cmd response: none")
        ttk.Label(control, textvariable=self.cmd_var, font=("Consolas", 9), wraplength=540).grid(row=3, column=0, columnspan=5, sticky=tk.W, pady=(6, 0))

        right = ttk.LabelFrame(body, text="TX OOK waveform", padding=8)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        ctrl = ttk.Frame(right)
        ctrl.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(ctrl, text="bit 1 = 10 carrier cycles, bit 0 = OFF").pack(side=tk.LEFT)
        ttk.Label(ctrl, text="Max bits:").pack(side=tk.LEFT, padx=(20, 4))
        ttk.Spinbox(ctrl, from_=8, to=160, increment=8, textvariable=self.max_tx_bits, width=6).pack(side=tk.LEFT)

        self.tx_fig = Figure(figsize=(8.5, 4.5), dpi=100)
        self.tx_ax = self.tx_fig.add_subplot(111)
        self.tx_canvas = FigureCanvasTkAgg(self.tx_fig, master=right)
        self.tx_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # =================
    # RX / LINK TAB
    # =================
    def _build_rx_link_tab(self):
        counters = ttk.Frame(self.rx_tab, padding=(10, 8))
        counters.pack(side=tk.TOP, fill=tk.X)

        self.rx_card_vars: Dict[str, tk.StringVar] = {}
        cards = [
            ("rx_frames", "0"),
            ("frame_errors", "0"),
            ("checksum_errors", "0"),
            ("rx_sync_state", "0"),
            ("last_edge_delta", "0"),
            ("last_raw_bit", "0"),
            ("RX fps", "0.0"),
        ]
        for title, default in cards:
            var = tk.StringVar(value=default)
            self.rx_card_vars[title] = var
            box = ttk.Frame(counters, padding=5, relief="ridge")
            box.pack(side=tk.LEFT, padx=4)
            ttk.Label(box, text=title).pack()
            ttk.Label(box, textvariable=var, style="Big.TLabel").pack()

        strip = ttk.Frame(self.rx_tab, padding=(10, 0))
        strip.pack(side=tk.TOP, fill=tk.X)
        self.rx_state_var = tk.StringVar(value="RX state: WAIT_ACTIVITY")
        self.loop_status_var = tk.StringVar(value="Loopback status: waiting")
        self.rtc_summary_var = tk.StringVar(value="RTC: waiting")
        ttk.Label(strip, textvariable=self.rx_state_var, style="Header.TLabel").pack(side=tk.LEFT, padx=(0, 24))
        self.loop_status_label = ttk.Label(strip, textvariable=self.loop_status_var, style="Warn.TLabel")
        self.loop_status_label.pack(side=tk.LEFT, padx=(0, 24))
        ttk.Label(strip, textvariable=self.rtc_summary_var).pack(side=tk.LEFT)

        frame_box = ttk.LabelFrame(self.rx_tab, text="RX Frame thu và so sánh với TX", padding=8)
        frame_box.pack(side=tk.TOP, fill=tk.X, padx=10, pady=(8, 6))
        self.rx_tx_compact = ttk.Frame(frame_box)
        self.rx_tx_compact.pack(side=tk.TOP, fill=tk.X, pady=(0, 4))
        self.rx_compact = ttk.Frame(frame_box)
        self.rx_compact.pack(side=tk.TOP, fill=tk.X, pady=(0, 4))
        self.rx_detail_var = tk.StringVar(value="RX frame: none")
        ttk.Label(frame_box, textvariable=self.rx_detail_var, wraplength=1400).pack(anchor=tk.W, pady=(6, 0))

        # Compare table
        self.compare_tree = ttk.Treeview(frame_box, columns=("tx", "rx", "status", "note"), show="tree headings", height=5)
        self.compare_tree.heading("#0", text="Field")
        self.compare_tree.column("#0", width=110)
        for col, width in [("tx", 260), ("rx", 260), ("status", 90), ("note", 470)]:
            self.compare_tree.heading(col, text=col.upper())
            self.compare_tree.column(col, width=width, anchor=tk.W)
        self.compare_tree.tag_configure("ok", background="#eaffea")
        self.compare_tree.tag_configure("err", background="#ffd9d9")
        self.compare_tree.tag_configure("na", background="#eeeeee")
        self.compare_tree.pack(side=tk.TOP, fill=tk.X, pady=(8, 0))

        lower = ttk.Frame(self.rx_tab, padding=(10, 0))
        lower.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        left = ttk.Frame(lower)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8))

        fsm_box = ttk.LabelFrame(left, text="RX FSM / Debug", padding=8)
        fsm_box.pack(fill=tk.X)
        self.fsm_tree = ttk.Treeview(fsm_box, columns=("id", "state"), show="headings", height=7)
        self.fsm_tree.heading("id", text="ID")
        self.fsm_tree.heading("state", text="State")
        self.fsm_tree.column("id", width=50, anchor=tk.CENTER)
        self.fsm_tree.column("state", width=180)
        self.fsm_tree.tag_configure("active", background="#d7ecff")
        self.fsm_tree.pack(fill=tk.X)
        for sid, name in RX_STATE_NAMES.items():
            self.fsm_tree.insert("", "end", iid=str(sid), values=(sid, name))
        self.debug_var = tk.StringVar(value="last_edge_delta=0 | last_raw_bit=0")
        ttk.Label(fsm_box, textvariable=self.debug_var, wraplength=280, font=("Consolas", 10)).pack(anchor=tk.W, pady=(10, 0))

        link_box = ttk.LabelFrame(left, text="Link quality / tỷ lệ lỗi", padding=8)
        link_box.pack(fill=tk.X, pady=(8, 0))
        self.metric_vars: Dict[str, tk.StringVar] = {}
        metrics = [
            ("payload_bits", "0"),
            ("payload_bit_errors", "0"),
            ("payload_ber_ppm", "0 ppm"),
            ("payload_mismatch_frames", "0"),
            ("good_payload_frames", "0"),
            ("rx_total_observed", "0"),
            ("per_ppm", "0 ppm"),
            ("frame_success", "0 %"),
            ("goodput_bps", "0 bps"),
            ("err_summary", "q=0 p=0 s=0"),
        ]
        for i, (label, default) in enumerate(metrics):
            ttk.Label(link_box, text=f"{label}:").grid(row=i, column=0, sticky=tk.W, pady=2)
            var = tk.StringVar(value=default)
            self.metric_vars[label] = var
            ttk.Label(link_box, textvariable=var, font=("Consolas", 10, "bold")).grid(row=i, column=1, sticky=tk.W, padx=(8, 0), pady=2)

        right = ttk.Frame(lower)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        wave_box = ttk.LabelFrame(right, text="RX waveform / channel behavior", padding=8)
        wave_box.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        ctrl = ttk.Frame(wave_box)
        ctrl.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(ctrl, text="View:").pack(side=tk.LEFT)
        ttk.Combobox(
            ctrl,
            textvariable=self.rx_wave_mode,
            values=["TX vs RX frame", "RX reconstructed edges", "Last error bits"],
            width=24,
            state="readonly",
        ).pack(side=tk.LEFT, padx=(4, 14))
        ttk.Label(ctrl, text="Max bits:").pack(side=tk.LEFT)
        ttk.Spinbox(ctrl, from_=16, to=160, increment=8, textvariable=self.max_rx_bits, width=6).pack(side=tk.LEFT, padx=(4, 0))

        self.rx_fig = Figure(figsize=(8.8, 4.2), dpi=100)
        self.rx_ax = self.rx_fig.add_subplot(111)
        self.rx_canvas = FigureCanvasTkAgg(self.rx_fig, master=wave_box)
        self.rx_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        chart_box = ttk.LabelFrame(right, text="BER/PER history", padding=8)
        chart_box.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True, pady=(8, 0))
        self.metric_fig = Figure(figsize=(8.8, 2.8), dpi=100)
        self.metric_ax = self.metric_fig.add_subplot(111)
        self.metric_canvas = FigureCanvasTkAgg(self.metric_fig, master=chart_box)
        self.metric_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        err_box = ttk.LabelFrame(self.rx_tab, text="Error detail", padding=8)
        err_box.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=(4, 8))
        self.err_tree = ttk.Treeview(err_box, columns=("kind", "frame_id", "summary"), show="headings", height=3)
        for col, width in [("kind", 120), ("frame_id", 100), ("summary", 980)]:
            self.err_tree.heading(col, text=col.title())
            self.err_tree.column(col, width=width, anchor=tk.W)
        self.err_tree.pack(fill=tk.X)
        self.err_detail = tk.Text(err_box, height=3, wrap=tk.WORD, font=("Consolas", 9))
        self.err_detail.pack(fill=tk.X, pady=(6, 0))
        self.err_tree.bind("<<TreeviewSelect>>", self._on_error_selected)

    # =================
    # POWER / RTC / RAW
    # =================
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
        ttk.Separator(left).pack(fill=tk.X, pady=10)
        ttk.Label(left, text="Chart signals", style="Header.TLabel").pack(anchor=tk.W)
        for key, var in self.selected_power_keys.items():
            ttk.Checkbutton(left, text=key, variable=var).pack(anchor=tk.W)

        right = ttk.Frame(main)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.power_fig = Figure(figsize=(8, 5), dpi=100)
        self.power_ax = self.power_fig.add_subplot(111)
        self.power_canvas = FigureCanvasTkAgg(self.power_fig, master=right)
        self.power_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

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

    # =================
    # Serial handlers
    # =================
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
        self.cmd_var.set(f"cmd response: sent: {cmd}")

    def _cmd_payload(self):
        try:
            payload = parse_payload_entry(self.payload_entry.get())
        except ValueError as exc:
            messagebox.showwarning("Payload sai", str(exc))
            return
        self.state.tx_payload = payload
        self.state.tx_frame = build_frame(payload)
        self._send_cmd("tx_payload " + hex_bytes(payload))

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

    # =================
    # Log parser
    # =================
    def _handle_line(self, line: str):
        self.state.raw_lines.append(line)
        self._append_raw(line)
        line = normalize_line(line)

        if line.startswith("role"):
            m = re.search(r"board_role=([A-Za-z0-9_]+)", line)
            if m:
                self.state.role = m.group(1)

        elif line.startswith("alive"):
            kv = parse_kv_int(line)
            self.state.tx_frames = kv.get("tx_frames", self.state.tx_frames)
            self.state.rx_frames = kv.get("rx_frames", self.state.rx_frames)
            self.state.frame_errors = kv.get("frame_errors", self.state.frame_errors)
            self.state.checksum_errors = kv.get("checksum_errors", self.state.checksum_errors)
            self.state.rx_sync_state = kv.get("rx_sync_state", self.state.rx_sync_state)
            self.state.last_edge_delta = kv.get("last_edge_delta", self.state.last_edge_delta)
            self.state.last_raw_bit = kv.get("last_raw_bit", self.state.last_raw_bit)
            self.state.bit_rate = kv.get("bit_rate", self.state.bit_rate)
            self.state.edge_history.append(self.state.last_edge_delta)
            self._update_rates()

        elif line.startswith("tx_frame") or line.startswith("tx_status"):
            kv = parse_kv_int(line)
            self.state.tx_frame_id = kv.get("tx_frame_id", self.state.tx_frame_id)
            self.state.bit_rate = kv.get("bit_rate", self.state.bit_rate)
            self.state.tx_enabled = kv.get("tx_enabled", self.state.tx_enabled)
            self.state.carrier_test = kv.get("carrier_test", self.state.carrier_test)
            payload = parse_hex_field(line, "payload")
            frame = parse_frame_field(line)
            if payload:
                self.state.tx_payload = payload
                self.payload_entry.set(hex_bytes(payload))
            if frame:
                self.state.tx_frame = frame

        elif line.startswith("last_rx"):
            if "none" in line:
                self.state.last_rx_none = True
                self.state.last_rx_payload = []
                self.state.last_rx_frame = []
                self.state.last_rx_checksum = None
            else:
                kv = parse_kv_int(line)
                payload = parse_hex_field(line, "payload")
                frame = parse_frame_field(line)
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

        elif line.startswith("rx_frame"):
            frame = parse_frame_field(line)
            payload = parse_hex_field(line, "payload")
            self.state.last_rx_none = False
            if payload:
                self.state.last_rx_payload = payload
                self.state.last_rx_checksum = checksum(payload)
            if frame:
                self.state.last_rx_frame = frame

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

        elif line.startswith("err_bits"):
            self._parse_err_bits(line)

        elif line.startswith("err_frame"):
            self._capture_error("err_frame", line)

        elif line.startswith("cmd_ok") or line.startswith("cmd_err"):
            self.cmd_var.set(f"cmd response: {line}")

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

    def _update_rates(self):
        now = time.time()
        if self.state.last_alive_time is not None:
            dt = max(now - self.state.last_alive_time, 1e-6)
            dtx = max(self.state.tx_frames - self.state.last_tx_frames, 0)
            drx = max(self.state.rx_frames - self.state.last_rx_frames, 0)
            self.state.tx_fps = dtx / dt
            self.state.rx_fps = drx / dt
            self.state.goodput_bps = self.state.rx_fps * max(len(self.state.tx_payload) * 8, 1)
        self.state.last_alive_time = now
        self.state.last_tx_frames = self.state.tx_frames
        self.state.last_rx_frames = self.state.rx_frames
        self._append_metric_history()

    def _append_metric_history(self):
        t = time.time() - self.state.start_time
        if self.state.hist_metric_t and (t - self.state.hist_metric_t[-1]) < 0.20:
            return
        self.state.hist_metric_t.append(t)
        self.state.hist_ber_ppm.append(self.state.payload_ber_ppm)
        self.state.hist_per_ppm.append(self.state.per_ppm)
        self.state.hist_tx_fps.append(self.state.tx_fps)
        self.state.hist_rx_fps.append(self.state.rx_fps)

    def _parse_err_bits(self, line: str):
        m_tx = re.search(r"tx_bits=([01]+)", line)
        m_rx = re.search(r"rx_bits=([01]+)", line)
        m_pos = re.search(r"mismatch_positions=([0-9,; ]+)", line)
        kv = parse_kv_int(line)
        if m_tx:
            self.state.last_err_bits_tx = m_tx.group(1)
        if m_rx:
            self.state.last_err_bits_rx = m_rx.group(1)
        if m_pos:
            self.state.last_err_positions = parse_int_list_csv(m_pos.group(1))
        self._capture_error("err_bits", line, frame_id=kv.get("frame_id", kv.get("rx_frame_id", -1)))

    def _capture_error(self, kind: str, line: str, frame_id: Optional[int] = None):
        if frame_id is None:
            kv = parse_kv_int(line)
            frame_id = kv.get("rx_frame_id", kv.get("frame_id", -1))
        self.state.last_error_text = line
        self.state.error_events.appendleft({"kind": kind, "frame_id": frame_id, "summary": line, "raw": line})

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

    # =================
    # UI refresh
    # =================
    def _refresh_ui(self):
        st_name = RX_STATE_NAMES.get(self.state.rx_sync_state, f"UNKNOWN_{self.state.rx_sync_state}")
        self.role_var.set(f"role: {self.state.role}")

        self.tx_card_vars["tx_frames"].set(str(self.state.tx_frames))
        self.tx_card_vars["tx_frame_id"].set(str(self.state.tx_frame_id))
        self.tx_card_vars["bit_rate"].set(f"{self.state.bit_rate} bps")
        self.tx_card_vars["TX fps"].set(f"{self.state.tx_fps:.1f}")
        self.tx_card_vars["tx_enabled"].set(str(self.state.tx_enabled))
        self.tx_card_vars["carrier_test"].set(str(self.state.carrier_test))
        self.tx_card_vars["goodput"].set(f"{self.state.goodput_bps:.1f} bps")

        self.rx_card_vars["rx_frames"].set(str(self.state.rx_frames))
        self.rx_card_vars["frame_errors"].set(str(self.state.frame_errors))
        self.rx_card_vars["checksum_errors"].set(str(self.state.checksum_errors))
        self.rx_card_vars["rx_sync_state"].set(str(self.state.rx_sync_state))
        self.rx_card_vars["last_edge_delta"].set(str(self.state.last_edge_delta))
        self.rx_card_vars["last_raw_bit"].set(str(self.state.last_raw_bit))
        self.rx_card_vars["RX fps"].set(f"{self.state.rx_fps:.1f}")

        self.rx_state_var.set(f"RX state: {st_name} ({self.state.rx_sync_state})")
        self.debug_var.set(f"last_edge_delta={self.state.last_edge_delta} | last_raw_bit={self.state.last_raw_bit}")
        if self.state.rx_frames > 0:
            self.loop_status_var.set("Loopback status: RX has valid frames")
            self.loop_status_label.configure(style="Ok.TLabel")
        elif self.state.last_edge_delta > 0:
            self.loop_status_var.set("Loopback status: RX sees edges but no valid frame")
            self.loop_status_label.configure(style="Warn.TLabel")
        else:
            self.loop_status_var.set("Loopback status: TX active, RX input idle/no edges")
            self.loop_status_label.configure(style="Warn.TLabel")

        self.rtc_summary_var.set(
            f"RTC: {self.state.rtc_time} | valid={self.state.rtc_valid} | "
            f"source={self.state.rtc_source} | backup={self.state.rtc_backup}"
        )

        self._refresh_tx_frame()
        self._refresh_rx_frame_and_compare()
        self._refresh_fsm()
        self._refresh_metrics()
        self._refresh_errors()
        self._refresh_power()
        self._refresh_rtc()
        self._redraw_tx_wave()
        self._redraw_rx_wave()
        self._redraw_metric_chart()
        self._redraw_power_chart()
        self._redraw_clock_and_calendar()
        self.after(250, self._refresh_ui)

    def _make_compact_frame(self, parent: ttk.Frame, label: str, frame: List[int], none: bool = False):
        for child in parent.winfo_children():
            child.destroy()
        ttk.Label(parent, text=label, width=8, style="Header.TLabel").pack(side=tk.LEFT)
        if none:
            ttk.Label(parent, text="none", font=("Consolas", 12, "bold")).pack(side=tk.LEFT)
            return
        fields = split_frame(frame)
        colors = {
            "Preamble": "#d7ecff",
            "Sync": "#ffe5c2",
            "Length": "#eadcff",
            "Payload": "#dff6df",
            "Checksum": "#ffd8d8",
        }
        for name in FIELD_NAMES:
            vals = fields.get(name, [])
            txt = hex_bytes(vals) if vals else "--"
            tk.Label(parent, text=f"[{txt}]", bg=colors[name], fg="#111", font=("Consolas", 11, "bold"), padx=8, pady=4, relief="groove").pack(side=tk.LEFT, padx=(0, 4))
            tk.Label(parent, text=name, fg="#444", font=("Segoe UI", 8)).pack(side=tk.LEFT, padx=(0, 8))

    def _refresh_tx_frame(self):
        self._make_compact_frame(self.tx_compact, "TX:", self.state.tx_frame)
        fields = split_frame(self.state.tx_frame)
        self.tx_detail_var.set(
            f"TX frame_id={self.state.tx_frame_id} | len={len(self.state.tx_payload)} | "
            f"payload={hex_bytes(self.state.tx_payload)} | checksum={hex_bytes(fields.get('Checksum', []))} | full={hex_bytes(self.state.tx_frame)}"
        )
        bits = bits_from_bytes(self.state.tx_frame)
        bit_text = "".join(str(b) for b in bits)
        preview = " ".join(bit_text[i:i + 8] for i in range(0, min(len(bit_text), 96), 8))
        if len(bit_text) > 96:
            preview += " ..."
        self.tx_bits_var.set(f"Bits: {preview}")

        for item in self.tx_field_tree.get_children():
            self.tx_field_tree.delete(item)
        rows = [
            ("Preamble", hex_bytes(fields.get("Preamble", [])), "Frame start"),
            ("Sync", hex_bytes(fields.get("Sync", [])), "Expected D5"),
            ("Length", hex_bytes(fields.get("Length", [])), f"{len(fields.get('Payload', []))} byte payload"),
            ("Payload", hex_bytes(fields.get("Payload", [])), "Data"),
            ("Checksum", hex_bytes(fields.get("Checksum", [])), "LEN + payload"),
        ]
        for row in rows:
            self.tx_field_tree.insert("", "end", text=row[0], values=(row[1], row[2]))

    def _refresh_rx_frame_and_compare(self):
        self._make_compact_frame(self.rx_tx_compact, "TX:", self.state.tx_frame)
        self._make_compact_frame(self.rx_compact, "RX:", self.state.last_rx_frame, none=self.state.last_rx_none)
        if self.state.last_rx_none:
            self.rx_detail_var.set("RX frame: none")
        else:
            self.rx_detail_var.set(
                f"RX len={len(self.state.last_rx_payload)} | payload={hex_bytes(self.state.last_rx_payload)} | full={hex_bytes(self.state.last_rx_frame)}"
            )

        for item in self.compare_tree.get_children():
            self.compare_tree.delete(item)
        tx_fields = split_frame(self.state.tx_frame)
        rx_fields = split_frame(self.state.last_rx_frame) if not self.state.last_rx_none else {}
        notes = {
            "Preamble": "Chuỗi AA AA AA để RX bắt hoạt động.",
            "Sync": "Byte đồng bộ, kỳ vọng D5.",
            "Length": "Độ dài payload.",
            "Payload": "Dữ liệu chính.",
            "Checksum": "LEN + payload, lấy 8 bit thấp.",
        }
        for name in FIELD_NAMES:
            tx = tx_fields.get(name, [])
            rx = rx_fields.get(name, []) if not self.state.last_rx_none else []
            if self.state.last_rx_none:
                status, tag, rx_txt = "N/A", "na", "--"
            else:
                status = "OK" if tx == rx else "MISMATCH"
                tag = "ok" if status == "OK" else "err"
                rx_txt = hex_bytes(rx)
            self.compare_tree.insert("", "end", text=name, values=(hex_bytes(tx), rx_txt, status, notes[name]), tags=(tag,))

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
        self.metric_vars["err_summary"].set(f"q={self.state.err_queued} p={self.state.err_printed} s={self.state.err_suppressed}")
        self.metric_vars["goodput_bps"].set(f"{self.state.goodput_bps:.1f} bps")
        attempts = self.state.rx_frames + self.state.frame_errors + self.state.checksum_errors
        if attempts > 0:
            ok_rate = self.state.rx_frames * 100.0 / attempts
            self.metric_vars["frame_success"].set(f"{ok_rate:.3f} %")
        else:
            self.metric_vars["frame_success"].set("0 %")

    def _refresh_errors(self):
        current = set(self.err_tree.get_children())
        wanted = set()
        for idx, ev in enumerate(self.state.error_events):
            iid = f"e{idx}"
            wanted.add(iid)
            values = (ev["kind"], ev["frame_id"], ev["summary"][:260])
            if iid in current:
                self.err_tree.item(iid, values=values)
            else:
                self.err_tree.insert("", "end", iid=iid, values=values)
        for iid in current - wanted:
            self.err_tree.delete(iid)

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

    # =================
    # Draw charts
    # =================
    def _draw_ook_bits(self, ax, bits: List[int], y_offset: float, label: str, max_bits: int, mark_positions: Optional[List[int]] = None):
        bits = bits[:max_bits]
        mark_positions = mark_positions or []
        t_vals: List[float] = []
        y_vals: List[float] = []
        samples_per_carrier = 6
        for i, bit in enumerate(bits):
            t0 = i * OOK_BIT_US
            ax.axvline(t0, linewidth=0.35, linestyle="--", alpha=0.25)
            if i in mark_positions:
                ax.axvspan(t0, t0 + OOK_BIT_US, alpha=0.25)
            if bit == 1:
                total_samples = int((OOK_BIT_US / OOK_CARRIER_PERIOD_US) * samples_per_carrier)
                for n in range(total_samples + 1):
                    t = t0 + n * (OOK_CARRIER_PERIOD_US / samples_per_carrier)
                    phase = (t - t0) % OOK_CARRIER_PERIOD_US
                    y = y_offset + (1.0 if phase < OOK_CARRIER_PERIOD_US / 2 else 0.0)
                    t_vals.append(t)
                    y_vals.append(y)
            else:
                t_vals.extend([t0, t0 + OOK_BIT_US])
                y_vals.extend([y_offset, y_offset])
            label_bit = str(bit) + ("*" if i in mark_positions else "")
            ax.text(t0 + OOK_BIT_US / 2, y_offset + 1.08, label_bit, ha="center", va="bottom", fontsize=7)
        if t_vals:
            ax.step(t_vals, y_vals, where="post", label=label)

    def _redraw_tx_wave(self):
        self.tx_ax.clear()
        self.tx_ax.grid(True)
        self.tx_ax.set_title(f"TX OOK waveform - frame: {hex_bytes(self.state.tx_frame)}")
        self.tx_ax.set_xlabel("Time (µs), bit window = 10 µs @ 100 kbps")
        self.tx_ax.set_ylabel("Carrier gate")
        try:
            max_bits = int(self.max_tx_bits.get())
        except Exception:
            max_bits = 80
        self._draw_ook_bits(self.tx_ax, bits_from_bytes(self.state.tx_frame), 0.0, "TX frame", max_bits)
        self.tx_ax.set_xlim(0, max(max_bits * OOK_BIT_US, OOK_BIT_US))
        self.tx_ax.set_ylim(-0.2, 1.35)
        self.tx_ax.legend(loc="upper right")
        self.tx_canvas.draw_idle()

    def _redraw_rx_wave(self):
        self.rx_ax.clear()
        self.rx_ax.grid(True)
        self.rx_ax.set_xlabel("Time (µs), bit window = 10 µs @ 100 kbps")
        self.rx_ax.set_ylabel("OOK gate")
        try:
            max_bits = int(self.max_rx_bits.get())
        except Exception:
            max_bits = 80
        mode = self.rx_wave_mode.get()
        tx_bits = bits_from_bytes(self.state.tx_frame)
        rx_bits = bits_from_bytes(self.state.last_rx_frame) if not self.state.last_rx_none else []
        edge_vals = list(self.state.edge_history)[-max_bits:]
        edge_bits = [1 if e >= EDGE_THRESHOLD else 0 for e in edge_vals]

        if mode == "TX vs RX frame":
            self.rx_ax.set_title("TX frame OOK vs last RX frame OOK")
            self._draw_ook_bits(self.rx_ax, tx_bits, 1.4, "TX frame", max_bits)
            if rx_bits:
                self._draw_ook_bits(self.rx_ax, rx_bits, 0.0, "RX frame", max_bits)
            else:
                self.rx_ax.text(0.5, 0.28, "RX frame = none", transform=self.rx_ax.transAxes, ha="center")
            self.rx_ax.set_ylim(-0.2, 2.7)
        elif mode == "RX reconstructed edges":
            self.rx_ax.set_title(f"RX reconstructed from edge count: edge ≥ {EDGE_THRESHOLD} → bit 1")
            if edge_bits:
                self._draw_ook_bits(self.rx_ax, edge_bits, 0.0, "RX edge reconstructed", max_bits)
                for i, edge in enumerate(edge_vals):
                    self.rx_ax.text(i * OOK_BIT_US + OOK_BIT_US / 2, -0.15, str(edge), ha="center", va="top", fontsize=7)
            else:
                self.rx_ax.text(0.5, 0.5, "Waiting for alive edge samples...", transform=self.rx_ax.transAxes, ha="center")
            self.rx_ax.set_ylim(-0.35, 1.35)
        else:
            self.rx_ax.set_title("Last error tx_bits / rx_bits")
            tx_err = [1 if ch == "1" else 0 for ch in self.state.last_err_bits_tx if ch in "01"]
            rx_err = [1 if ch == "1" else 0 for ch in self.state.last_err_bits_rx if ch in "01"]
            if tx_err or rx_err:
                if tx_err:
                    self._draw_ook_bits(self.rx_ax, tx_err, 1.4, "err tx_bits", max_bits, self.state.last_err_positions)
                if rx_err:
                    self._draw_ook_bits(self.rx_ax, rx_err, 0.0, "err rx_bits", max_bits, self.state.last_err_positions)
                self.rx_ax.set_ylim(-0.2, 2.7)
            else:
                self.rx_ax.text(0.5, 0.5, "No err_bits log yet", transform=self.rx_ax.transAxes, ha="center")
                self.rx_ax.set_ylim(-0.2, 1.35)
        self.rx_ax.set_xlim(0, max(max_bits * OOK_BIT_US, OOK_BIT_US))
        self.rx_ax.legend(loc="upper right")
        self.rx_canvas.draw_idle()

    def _redraw_metric_chart(self):
        self.metric_ax.clear()
        self.metric_ax.set_title("BER/PER ppm and TX/RX frame rate")
        self.metric_ax.set_xlabel("GUI time (s)")
        self.metric_ax.grid(True)
        t = list(self.state.hist_metric_t)
        series = [
            ("payload_ber_ppm", list(self.state.hist_ber_ppm)),
            ("per_ppm", list(self.state.hist_per_ppm)),
            ("TX fps", list(self.state.hist_tx_fps)),
            ("RX fps", list(self.state.hist_rx_fps)),
        ]
        has = False
        for label, y in series:
            n = min(len(t), len(y))
            if n > 1:
                self.metric_ax.plot(t[-n:], y[-n:], label=label)
                has = True
        if has:
            self.metric_ax.legend(loc="upper right")
        self.metric_canvas.draw_idle()

    def _redraw_power_chart(self):
        self.power_ax.clear()
        self.power_ax.set_title("Power Monitor")
        self.power_ax.set_xlabel("Time (s)")
        self.power_ax.set_ylabel("Voltage (V)")
        self.power_ax.grid(True)
        t = list(self.state.hist_t)
        for key, var in self.selected_power_keys.items():
            if not var.get():
                continue
            y = list(self.state.hist_power[key])
            n = min(len(t), len(y))
            if n > 1:
                self.power_ax.plot(t[-n:], y[-n:], label=key)
        if len(t) > 1:
            self.power_ax.legend(loc="upper right")
        self.power_canvas.draw_idle()

    # =================
    # RTC clock/calendar
    # =================
    def _parse_datetime(self) -> Optional[datetime]:
        try:
            return datetime.strptime(self.state.rtc_time, "%Y-%m-%d %H:%M:%S")
        except Exception:
            return None

    def _redraw_clock_and_calendar(self):
        dt = self._parse_datetime()
        self.clock_canvas.delete("all")
        w = max(self.clock_canvas.winfo_width(), 300)
        h = max(self.clock_canvas.winfo_height(), 300)
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

    # =================
    # Raw/log/test
    # =================
    def _append_raw(self, line: str):
        self.raw_text.insert(tk.END, line + "\n")
        if int(self.raw_text.index("end-1c").split(".")[0]) > 1800:
            self.raw_text.delete("1.0", "300.0")
        self.raw_text.see(tk.END)

    def _inject_sample_log(self):
        sample = """
link_stats payload_bits=0 payload_bit_errors=0 payload_ber_ppm=0 payload_mismatch_frames=0 good_payload_frames=0
link_quality rx_total_observed=0 per_ppm=0
err_summary queued=0 printed=0 suppressed=0
adc_mv rx_out_a=1808 vmon_bu=4960 vmon_bu_3v=3025 vmon_main_sys=9113 vmon_main=9270 vmon_5v_sys=5020 vmon_3v3_sys=3300
dac_mv threshold=1650
rtc time=2026-06-10 00:17:09 valid=1 source=LSE backup=ok
role board_role=TX_RX_LOOPBACK
alive tx_frames=63752 rx_frames=0 frame_errors=0 checksum_errors=0 rx_sync_state=0 last_edge_delta=0 last_raw_bit=0
tx_frame tx_frame_id=63765 len=4 payload=55 A5 3C C3 checksum=FD frame=AA AA AA D5 04 55 A5 3C C3 FD
last_rx none
link_stats payload_bits=0 payload_bit_errors=0 payload_ber_ppm=0 payload_mismatch_frames=0 good_payload_frames=0
link_quality rx_total_observed=0 per_ppm=0
err_summary queued=0 printed=0 suppressed=0
adc_mv rx_out_a=1810 vmon_bu=4953 vmon_bu_3v=3025 vmon_main_sys=9075 vmon_main=9238 vmon_5v_sys=5018 vmon_3v3_sys=3297
dac_mv threshold=1650
rtc time=2026-06-10 00:17:10 valid=1 source=LSE backup=ok
"""
        for line in sample.strip().splitlines():
            self._handle_line(line.strip())
        self.status_var.set("Injected TX_RX_LOOPBACK sample log")

    def _on_close(self):
        if self.reader:
            self.reader.stop()
        self.destroy()


if __name__ == "__main__":
    OwcLoopbackGui().mainloop()
