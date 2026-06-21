#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OWC TX-ONLY UART GUI v2 Control

Parse UART logs:
  role board_role=TX_ONLY
  alive_tx tx_frames=88751 tx_frame_id=88751 bit_rate=100000
  tx_frame tx_frame_id=88758 len=4 payload=55 A5 3C C3 checksum=FD frame=AA AA AA D5 04 55 A5 3C C3 FD
  adc_mv rx_out_a=1805 vmon_bu=4940 vmon_bu_3v=3018 vmon_main_sys=9366 vmon_main=9350 vmon_5v_sys=5022 vmon_3v3_sys=3298
  dac_mv threshold=1650

Requires:
  pip install pyserial matplotlib
Run:
  python owc_tx_only_gui_v2_control.py
"""

import re
import time
import queue
import threading
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional

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

POWER_KEYS = [
    "rx_out_a", "vmon_bu", "vmon_bu_3v", "vmon_main_sys",
    "vmon_main", "vmon_5v_sys", "vmon_3v3_sys",
]

POWER_EXPECTED_MV = {
    "rx_out_a":      (1800,    0,  3300),
    "vmon_bu":       (5000, 4500,  5500),
    "vmon_bu_3v":    (3000, 2800,  3300),
    "vmon_main_sys": (9400, 8500, 10000),
    "vmon_main":     (9400, 8500, 10000),
    "vmon_5v_sys":   (5000, 4750,  5250),
    "vmon_3v3_sys":  (3300, 3135,  3465),
}

TX_PREAMBLE = [0xAA, 0xAA, 0xAA]
TX_SYNC = 0xD5
DEFAULT_PAYLOAD = [0x55, 0xA5, 0x3C, 0xC3]
DAC_EXPECTED_THRESHOLD_MV = 1650
BIT_RATE_DEFAULT = 100_000
OOK_BIT_US = 10.0
OOK_CARRIER_PERIOD_US = 1.0


def checksum(payload: List[int]) -> int:
    return (len(payload) + sum(payload)) & 0xFF


def build_frame(payload: List[int]) -> List[int]:
    return TX_PREAMBLE + [TX_SYNC, len(payload)] + payload + [checksum(payload)]


def hex_bytes(vals: List[int]) -> str:
    return " ".join(f"{v:02X}" for v in vals)


def bits_from_bytes(vals: List[int]) -> List[int]:
    out = []
    for byte in vals:
        for pos in range(7, -1, -1):
            out.append((byte >> pos) & 1)
    return out


def bit_string(vals: List[int]) -> str:
    return "".join(str(b) for b in bits_from_bytes(vals))


def parse_kv(line: str) -> Dict[str, int]:
    out: Dict[str, int] = {}
    for key, val in re.findall(r"([A-Za-z0-9_]+)=(-?\d+)", line):
        try:
            out[key] = int(val)
        except ValueError:
            pass
    return out


def parse_hex_field(line: str, field: str) -> List[int]:
    match = re.search(rf"{field}=([0-9A-Fa-fxX ]+?)(?:\s+[A-Za-z0-9_]+=|$)", line)
    if not match:
        return []
    vals = []
    for tok in match.group(1).strip().split():
        tok = tok.replace("0x", "").replace("0X", "")
        if re.fullmatch(r"[0-9A-Fa-f]{1,2}", tok):
            vals.append(int(tok, 16))
    return vals


def split_frame(frame: List[int]) -> Dict[str, List[int]]:
    if len(frame) < 6:
        return {"Preamble": frame[:3], "Sync": frame[3:4], "Length": frame[4:5], "Payload": [], "Checksum": frame[-1:] if frame else []}
    length = frame[4]
    ps = 5
    pe = min(ps + length, max(len(frame) - 1, ps))
    return {
        "Preamble": frame[:3],
        "Sync": frame[3:4],
        "Length": frame[4:5],
        "Payload": frame[ps:pe],
        "Checksum": frame[pe:pe + 1] if pe < len(frame) else [],
    }


def mv_status(key: str, mv: int) -> str:
    _expected, low, high = POWER_EXPECTED_MV[key]
    return "OK" if low <= mv <= high else "WARN"


class SerialReader(threading.Thread):
    def __init__(self, port: str, baud: int, out_q: "queue.Queue[str]"):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.out_q = out_q
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
            self.out_q.put(f"__STATUS__ Connected to {self.port} @ {self.baud}")
        except Exception as exc:
            self.out_q.put(f"__ERROR__ Cannot open {self.port}: {exc}")
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


@dataclass
class TxData:
    board_role: str = "UNKNOWN"
    tx_frames: int = 0
    tx_frame_id: int = 0
    bit_rate: int = BIT_RATE_DEFAULT
    tx_payload: List[int] = field(default_factory=lambda: list(DEFAULT_PAYLOAD))
    tx_frame: List[int] = field(default_factory=lambda: build_frame(DEFAULT_PAYLOAD))
    power_mv: Dict[str, int] = field(default_factory=lambda: {k: 0 for k in POWER_KEYS})
    dac_threshold_mv: int = DAC_EXPECTED_THRESHOLD_MV
    start_time: float = field(default_factory=time.time)
    raw_lines: deque = field(default_factory=lambda: deque(maxlen=2000))
    prev_alive_time: Optional[float] = None
    prev_tx_frames: int = 0
    tx_fps: float = 0.0
    tx_bit_rate_est_bps: float = 0.0
    tx_goodput_bps: float = 0.0
    hist_t: deque = field(default_factory=lambda: deque(maxlen=300))
    hist_power: Dict[str, deque] = field(default_factory=lambda: {k: deque(maxlen=300) for k in POWER_KEYS})
    last_cmd_response: str = "none"


class TxOnlyGui(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("OWC TX-ONLY Monitor - STM32F407 - v2 Control")
        self.geometry("1360x820")
        self.minsize(1100, 720)
        self.data = TxData()
        self.serial_q: "queue.Queue[str]" = queue.Queue()
        self.reader: Optional[SerialReader] = None
        self.max_wave_bits = tk.IntVar(value=80)
        self._build_ui()
        self._refresh_ports()
        self.after(50, self._process_serial_queue)
        self.after(300, self._refresh_ui)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        style = ttk.Style(self)
        style.configure("Header.TLabel", font=("Segoe UI", 11, "bold"))
        style.configure("Big.TLabel", font=("Consolas", 16, "bold"))
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
        self.tx_tab = ttk.Frame(self.notebook)
        self.power_tab = ttk.Frame(self.notebook)
        self.raw_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.tx_tab, text="TX DEMO")
        self.notebook.add(self.power_tab, text="POWER")
        self.notebook.add(self.raw_tab, text="RAW LOG")
        self._build_tx_tab()
        self._build_power_tab()
        self._build_raw_tab()

    def _build_tx_tab(self):
        counters = ttk.Frame(self.tx_tab, padding=(10, 8))
        counters.pack(side=tk.TOP, fill=tk.X)
        self.role_var = tk.StringVar(value="role: UNKNOWN")
        ttk.Label(counters, textvariable=self.role_var, style="Header.TLabel").pack(side=tk.LEFT, padx=(0, 12))
        self.tx_frames_var = tk.StringVar(value="0")
        self.tx_frame_id_var = tk.StringVar(value="0")
        self.bit_rate_var = tk.StringVar(value="100000 bps")
        self.tx_fps_var = tk.StringVar(value="0.0 fps")
        self.tx_rate_var = tk.StringVar(value="0 bps")
        self.goodput_var = tk.StringVar(value="0 bps")
        for title, var in [
            ("tx_frames", self.tx_frames_var),
            ("tx_frame_id", self.tx_frame_id_var),
            ("configured bit rate", self.bit_rate_var),
            ("TX frame rate", self.tx_fps_var),
            ("TX bit rate est.", self.tx_rate_var),
            ("payload goodput", self.goodput_var),
        ]:
            box = ttk.Frame(counters, padding=5, relief="ridge")
            box.pack(side=tk.LEFT, padx=4)
            ttk.Label(box, text=title).pack()
            ttk.Label(box, textvariable=var, style="Big.TLabel").pack()

        frame_box = ttk.LabelFrame(self.tx_tab, text="TX Frame đang phát", padding=8)
        frame_box.pack(side=tk.TOP, fill=tk.X, padx=10, pady=(8, 6))
        self.tx_compact = ttk.Frame(frame_box)
        self.tx_compact.pack(fill=tk.X, pady=2)
        self.tx_detail_var = tk.StringVar(value="Waiting for tx_frame log...")
        ttk.Label(frame_box, textvariable=self.tx_detail_var, wraplength=1280).pack(anchor=tk.W, pady=(8, 0))
        self.bits_preview_var = tk.StringVar(value="")
        ttk.Label(frame_box, textvariable=self.bits_preview_var, font=("Consolas", 9), wraplength=1280).pack(anchor=tk.W, pady=(4, 0))

        body = ttk.Frame(self.tx_tab, padding=(10, 0))
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        left = ttk.Frame(body)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8))
        fields_box = ttk.LabelFrame(left, text="TX Frame Fields", padding=8)
        fields_box.pack(fill=tk.BOTH, expand=True)
        cols = ("field", "value", "note")
        self.tx_field_tree = ttk.Treeview(fields_box, columns=cols, show="headings", height=8)
        for col, width in [("field", 110), ("value", 280), ("note", 150)]:
            self.tx_field_tree.heading(col, text=col.upper())
            self.tx_field_tree.column(col, width=width, anchor=tk.W)
        self.tx_field_tree.pack(fill=tk.BOTH, expand=True)
        self.tx_note_var = tk.StringVar(value="TX_ONLY: firmware chỉ chạy tác vụ phát, không chạy RX FSM.")
        ttk.Label(fields_box, textvariable=self.tx_note_var, wraplength=550).pack(anchor=tk.W, pady=(8, 0))

        right = ttk.Frame(body)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        wave_ctrl = ttk.Frame(right)
        wave_ctrl.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(wave_ctrl, text="OOK preview: bit 1 = 10 carrier cycles, bit 0 = OFF").pack(side=tk.LEFT, padx=(0, 16))
        ttk.Label(wave_ctrl, text="Max bits:").pack(side=tk.LEFT)
        ttk.Spinbox(wave_ctrl, from_=8, to=160, increment=8, textvariable=self.max_wave_bits, width=6).pack(side=tk.LEFT, padx=(4, 0))
        self.wave_fig = Figure(figsize=(8.5, 4.5), dpi=100)
        self.wave_ax = self.wave_fig.add_subplot(111)
        self.wave_canvas = FigureCanvasTkAgg(self.wave_fig, master=right)
        self.wave_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _build_power_tab(self):
        main = ttk.Frame(self.power_tab, padding=10)
        main.pack(fill=tk.BOTH, expand=True)
        left = ttk.Frame(main)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8))
        ttk.Label(left, text="Voltage Monitor", style="Header.TLabel").pack(anchor=tk.W, pady=(0, 8))
        cols = ("measured", "expected", "range", "status")
        self.power_tree = ttk.Treeview(left, columns=cols, show="tree headings", height=10)
        self.power_tree.heading("#0", text="Signal")
        self.power_tree.column("#0", width=150)
        for col, width in [("measured", 95), ("expected", 95), ("range", 125), ("status", 70)]:
            self.power_tree.heading(col, text=col.title())
            self.power_tree.column(col, width=width, anchor=tk.CENTER)
        self.power_tree.tag_configure("ok", background="#eaffea")
        self.power_tree.tag_configure("warn", background="#ffd9d9")
        self.power_tree.pack(fill=tk.BOTH, expand=True)
        for key in POWER_KEYS:
            exp, low, high = POWER_EXPECTED_MV[key]
            self.power_tree.insert("", "end", iid=key, text=key, values=("0.000 V", f"{exp/1000:.3f} V", f"{low/1000:.2f}-{high/1000:.2f} V", "WARN"), tags=("warn",))
        self.dac_var = tk.StringVar(value="DAC threshold: 1.650 V")
        ttk.Label(left, textvariable=self.dac_var, style="Header.TLabel").pack(anchor=tk.W, pady=(10, 0))
        right = ttk.Frame(main)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.power_fig = Figure(figsize=(8, 5), dpi=100)
        self.power_ax = self.power_fig.add_subplot(111)
        self.power_canvas = FigureCanvasTkAgg(self.power_fig, master=right)
        self.power_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _build_raw_tab(self):
        frame = ttk.Frame(self.raw_tab, padding=8)
        frame.pack(fill=tk.BOTH, expand=True)
        self.raw_text = tk.Text(frame, height=20, wrap=tk.NONE, font=("Consolas", 10))
        self.raw_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        yscroll = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=self.raw_text.yview)
        yscroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.raw_text.configure(yscrollcommand=yscroll.set)

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
            if line.startswith("__ERROR__") or line.startswith("__STATUS__"):
                self.status_var.set(line.replace("__ERROR__ ", "").replace("__STATUS__ ", ""))
                self._append_raw(line)
                if "Disconnected" in line:
                    self.reader = None
                    self.connect_btn.configure(text="Connect")
                continue
            self._handle_line(line)
        self.after(50, self._process_serial_queue)

    def _handle_line(self, line: str):
        self.data.raw_lines.append(line)
        self._append_raw(line)
        if line.startswith("role"):
            match = re.search(r"board_role=([A-Za-z0-9_]+)", line)
            if match:
                self.data.board_role = match.group(1)
        elif line.startswith("alive_tx"):
            kv = parse_kv(line)
            self.data.tx_frames = kv.get("tx_frames", self.data.tx_frames)
            self.data.tx_frame_id = kv.get("tx_frame_id", self.data.tx_frame_id)
            self.data.bit_rate = kv.get("bit_rate", self.data.bit_rate)
            self._update_tx_rates()
        elif line.startswith("tx_frame"):
            kv = parse_kv(line)
            payload = parse_hex_field(line, "payload")
            full_frame = parse_hex_field(line, "frame")
            self.data.tx_frame_id = kv.get("tx_frame_id", self.data.tx_frame_id)
            if payload:
                self.data.tx_payload = payload
            if full_frame:
                self.data.tx_frame = full_frame
            elif payload:
                self.data.tx_frame = build_frame(payload)
        elif line.startswith("adc_mv"):
            kv = parse_kv(line)
            for key in POWER_KEYS:
                if key in kv:
                    self.data.power_mv[key] = kv[key]
            t = time.time() - self.data.start_time
            self.data.hist_t.append(t)
            for key in POWER_KEYS:
                self.data.hist_power[key].append(self.data.power_mv[key] / 1000.0)
        elif line.startswith("dac"):
            kv = parse_kv(line)
            for key in ("threshold", "threshold_mv", "dac_threshold_mv", "dac_mv"):
                if key in kv:
                    self.data.dac_threshold_mv = kv[key]
                    break

    def _update_tx_rates(self):
        now = time.time()
        if self.data.prev_alive_time is not None:
            dt = max(now - self.data.prev_alive_time, 1e-6)
            dtx = max(self.data.tx_frames - self.data.prev_tx_frames, 0)
            self.data.tx_fps = dtx / dt
            frame_bits = max(len(self.data.tx_frame) * 8, 1)
            payload_bits = max(len(self.data.tx_payload) * 8, 1)
            self.data.tx_bit_rate_est_bps = self.data.tx_fps * frame_bits
            self.data.tx_goodput_bps = self.data.tx_fps * payload_bits
        self.data.prev_alive_time = now
        self.data.prev_tx_frames = self.data.tx_frames

    def _refresh_ui(self):
        self.role_var.set(f"role: {self.data.board_role}")
        self.tx_frames_var.set(str(self.data.tx_frames))
        self.tx_frame_id_var.set(str(self.data.tx_frame_id))
        self.bit_rate_var.set(f"{self.data.bit_rate} bps")
        self.tx_enabled_var.set(str(self.data.tx_enabled))
        self.carrier_test_var.set(str(self.data.carrier_test))
        self.tx_fps_var.set(f"{self.data.tx_fps:.1f} fps")
        if hasattr(self, "cmd_response_var"):
            self.cmd_response_var.set(f"cmd response: {self.data.last_cmd_response}")
        self.tx_rate_var.set(f"{self.data.tx_bit_rate_est_bps:.0f} bps")
        self.goodput_var.set(f"{self.data.tx_goodput_bps:.0f} bps")
        self._refresh_tx_frame()
        self._refresh_power()
        self._redraw_wave()
        self._redraw_power_chart()
        self.after(300, self._refresh_ui)

    def _refresh_tx_frame(self):
        for child in self.tx_compact.winfo_children():
            child.destroy()
        ttk.Label(self.tx_compact, text="TX:", width=5, style="Header.TLabel").pack(side=tk.LEFT)
        fields = split_frame(self.data.tx_frame)
        colors = {"Preamble": "#d7ecff", "Sync": "#ffe5c2", "Length": "#eadcff", "Payload": "#dff6df", "Checksum": "#ffd8d8"}
        for name in ["Preamble", "Sync", "Length", "Payload", "Checksum"]:
            vals = fields.get(name, [])
            txt = hex_bytes(vals) if vals else "--"
            tk.Label(self.tx_compact, text=f"[{txt}]", bg=colors[name], fg="#111111", font=("Consolas", 12, "bold"), padx=8, pady=4, relief="groove").pack(side=tk.LEFT, padx=(0, 4))
            tk.Label(self.tx_compact, text=name, fg="#444444", font=("Segoe UI", 8)).pack(side=tk.LEFT, padx=(0, 8))
        self.tx_detail_var.set(
            f"TX frame_id={self.data.tx_frame_id} | payload={hex_bytes(fields.get('Payload', []))} | "
            f"checksum={hex_bytes(fields.get('Checksum', []))} | full={hex_bytes(self.data.tx_frame)}"
        )
        bits = bit_string(self.data.tx_frame)
        preview = " ".join(bits[i:i+8] for i in range(0, min(len(bits), 96), 8))
        if len(bits) > 96:
            preview += " ..."
        self.bits_preview_var.set(f"Bits: {preview}")
        for item in self.tx_field_tree.get_children():
            self.tx_field_tree.delete(item)
        rows = [
            ("Preamble", hex_bytes(fields.get("Preamble", [])), "Frame start"),
            ("Sync", hex_bytes(fields.get("Sync", [])), "Expected D5"),
            ("Length", hex_bytes(fields.get("Length", [])), f"{len(fields.get('Payload', []))} byte payload"),
            ("Payload", hex_bytes(fields.get("Payload", [])), "Data"),
            ("Checksum", hex_bytes(fields.get("Checksum", [])), "LEN + payload"),
        ]
        for idx, row in enumerate(rows):
            self.tx_field_tree.insert("", "end", iid=f"field{idx}", values=row)
        self.tx_note_var.set(
            "TX_ONLY: board chỉ chạy tác vụ phát. Frame AA AA AA | D5 | LEN | PAYLOAD | CHECKSUM. "
            "Payload 4 byte => frame 10 byte = 80 bit; 100 kbps lý thuyết ≈ 1250 frame/s."
        )

    def _refresh_power(self):
        self.dac_var.set(f"DAC threshold: {self.data.dac_threshold_mv/1000.0:.3f} V")
        for key in POWER_KEYS:
            mv = self.data.power_mv.get(key, 0)
            exp, low, high = POWER_EXPECTED_MV[key]
            status = mv_status(key, mv)
            tag = "ok" if status == "OK" else "warn"
            self.power_tree.item(key, values=(f"{mv/1000:.3f} V", f"{exp/1000:.3f} V", f"{low/1000:.2f}-{high/1000:.2f} V", status), tags=(tag,))

    def _redraw_wave(self):
        self.wave_ax.clear()
        self.wave_ax.set_title("TX OOK waveform preview from current tx_frame")
        self.wave_ax.set_xlabel("Time (µs), bit window = 10 µs @ 100 kbps")
        self.wave_ax.set_ylabel("Carrier gate")
        self.wave_ax.grid(True)
        bits = bits_from_bytes(self.data.tx_frame)
        try:
            max_bits = int(self.max_wave_bits.get())
        except Exception:
            max_bits = 80
        bits = bits[:max_bits]
        t_values, y_values = [], []
        samples_per_carrier = 6
        for i, bit in enumerate(bits):
            t0 = i * OOK_BIT_US
            self.wave_ax.axvline(t0, linewidth=0.4, linestyle="--", alpha=0.35)
            if bit == 1:
                total_samples = int((OOK_BIT_US / OOK_CARRIER_PERIOD_US) * samples_per_carrier)
                for n in range(total_samples + 1):
                    t = t0 + n * (OOK_CARRIER_PERIOD_US / samples_per_carrier)
                    phase = (t - t0) % OOK_CARRIER_PERIOD_US
                    y = 1.0 if phase < OOK_CARRIER_PERIOD_US / 2.0 else 0.0
                    t_values.append(t)
                    y_values.append(y)
            else:
                t_values.extend([t0, t0 + OOK_BIT_US])
                y_values.extend([0.0, 0.0])
            self.wave_ax.text(t0 + OOK_BIT_US / 2, 1.08, str(bit), ha="center", va="bottom", fontsize=7)
        if len(t_values) > 1:
            self.wave_ax.step(t_values, y_values, where="post")
        self.wave_ax.set_ylim(-0.15, 1.3)
        self.wave_ax.set_xlim(0, max(len(bits) * OOK_BIT_US, OOK_BIT_US))
        self.wave_canvas.draw_idle()

    def _redraw_power_chart(self):
        self.power_ax.clear()
        self.power_ax.set_title("TX Board Power Monitor")
        self.power_ax.set_xlabel("Time (s)")
        self.power_ax.set_ylabel("Voltage (V)")
        self.power_ax.grid(True)
        t = list(self.data.hist_t)
        for key in ("vmon_5v_sys", "vmon_3v3_sys", "vmon_main_sys", "vmon_main", "vmon_bu", "vmon_bu_3v"):
            y = list(self.data.hist_power[key])
            n = min(len(t), len(y))
            if n > 1:
                self.power_ax.plot(t[-n:], y[-n:], label=key)
        if len(t) > 1:
            self.power_ax.legend(loc="upper right")
        self.power_canvas.draw_idle()

    def _append_raw(self, line: str):
        self.raw_text.insert(tk.END, line + "\n")
        if int(self.raw_text.index("end-1c").split(".")[0]) > 1500:
            self.raw_text.delete("1.0", "300.0")
        self.raw_text.see(tk.END)

    def _on_close(self):
        if self.reader:
            self.reader.stop()
        self.destroy()


def main():
    app = TxOnlyGui()
    app.mainloop()


if __name__ == "__main__":
    main()
