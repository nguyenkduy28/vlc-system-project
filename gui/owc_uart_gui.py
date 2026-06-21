#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OWC UART GUI
GUI đọc log UART từ STM32 OWC project và hiển thị:
- Tab POWER: voltage sense + chart realtime
- Tab TX/RX: counter TX/RX, lỗi frame/checksum, edge delta, raw bit, payload RX
- Tab RAW LOG: log UART thô

Yêu cầu:
    pip install pyserial matplotlib

Chạy:
    python owc_uart_gui.py

Log hỗ trợ ví dụ:
    alive tx_frames=86248 rx_frames=0 frame_errors=0 checksum_errors=0 rx_sync_state=0 last_edge_delta=0 last_raw_bit=0
    last_rx none
    adc_mv rx_out_a=1821 vmon_bu=4972 vmon_bu_3v=3019 vmon_main_sys=9363 vmon_main=9408 vmon_5v_sys=5020 vmon_3v3_sys=3304
"""

import re
import time
import queue
import threading
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, Optional, List

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
    "rx_out_a",
    "vmon_bu",
    "vmon_bu_3v",
    "vmon_main_sys",
    "vmon_main",
    "vmon_5v_sys",
    "vmon_3v3_sys",
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


@dataclass
class AppData:
    power_mv: Dict[str, int] = field(default_factory=lambda: {k: 0 for k in POWER_KEYS})
    alive: Dict[str, int] = field(default_factory=lambda: {k: 0 for k in ALIVE_KEYS})
    last_rx_text: str = "none"
    raw_lines: deque = field(default_factory=lambda: deque(maxlen=1000))
    start_time: float = field(default_factory=time.time)

    # Chart history
    hist_t: deque = field(default_factory=lambda: deque(maxlen=300))
    hist_power: Dict[str, deque] = field(default_factory=lambda: {k: deque(maxlen=300) for k in POWER_KEYS})
    hist_edge: deque = field(default_factory=lambda: deque(maxlen=300))


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
        self.geometry("1180x760")
        self.minsize(1050, 680)

        self.data = AppData()
        self.serial_q: "queue.Queue[str]" = queue.Queue()
        self.reader: Optional[SerialReader] = None

        self.selected_power_keys = {
            "vmon_5v_sys": tk.BooleanVar(value=True),
            "vmon_3v3_sys": tk.BooleanVar(value=True),
            "vmon_main_sys": tk.BooleanVar(value=True),
            "vmon_main": tk.BooleanVar(value=True),
            "vmon_bu": tk.BooleanVar(value=False),
            "vmon_bu_3v": tk.BooleanVar(value=False),
            "rx_out_a": tk.BooleanVar(value=False),
        }

        self._build_ui()
        self._refresh_ports()
        self.after(50, self._process_serial_queue)
        self.after(300, self._refresh_ui)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------------- UI ----------------
    def _build_ui(self):
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
        self.txrx_tab = ttk.Frame(self.notebook)
        self.raw_tab = ttk.Frame(self.notebook)

        self.notebook.add(self.power_tab, text="POWER")
        self.notebook.add(self.txrx_tab, text="TX / RX")
        self.notebook.add(self.raw_tab, text="RAW LOG")

        self._build_power_tab()
        self._build_txrx_tab()
        self._build_raw_tab()

    def _build_power_tab(self):
        left = ttk.Frame(self.power_tab, padding=10)
        left.pack(side=tk.LEFT, fill=tk.Y)

        ttk.Label(left, text="Voltage Monitor", font=("Segoe UI", 12, "bold")).pack(anchor=tk.W, pady=(0, 8))

        self.power_vars: Dict[str, tk.StringVar] = {}
        for key in POWER_KEYS:
            row = ttk.Frame(left)
            row.pack(fill=tk.X, pady=3)
            ttk.Label(row, text=key, width=18).pack(side=tk.LEFT)
            var = tk.StringVar(value="0.000 V")
            self.power_vars[key] = var
            ttk.Label(row, textvariable=var, width=12, anchor=tk.E).pack(side=tk.LEFT)

        ttk.Separator(left).pack(fill=tk.X, pady=10)

        ttk.Label(left, text="Chart signals", font=("Segoe UI", 10, "bold")).pack(anchor=tk.W, pady=(0, 5))
        for key, var in self.selected_power_keys.items():
            ttk.Checkbutton(left, text=key, variable=var, command=self._redraw_power_chart).pack(anchor=tk.W)

        ttk.Button(left, text="Clear chart", command=self._clear_chart).pack(anchor=tk.W, pady=(12, 0))

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
            ttk.Label(box, textvariable=var, font=("Consolas", 14, "bold")).pack()

        mid = ttk.Frame(self.txrx_tab, padding=10)
        mid.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(mid, text="RX state:", font=("Segoe UI", 10, "bold")).pack(side=tk.LEFT)
        self.rx_state_name_var = tk.StringVar(value="WAIT_ACTIVITY")
        ttk.Label(mid, textvariable=self.rx_state_name_var, width=22).pack(side=tk.LEFT, padx=(4, 20))

        ttk.Label(mid, text="Last RX:", font=("Segoe UI", 10, "bold")).pack(side=tk.LEFT)
        self.last_rx_var = tk.StringVar(value="none")
        ttk.Label(mid, textvariable=self.last_rx_var, font=("Consolas", 11)).pack(side=tk.LEFT, padx=(4, 0))

        bottom = ttk.Frame(self.txrx_tab, padding=8)
        bottom.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.edge_fig = Figure(figsize=(8, 4), dpi=100)
        self.edge_ax = self.edge_fig.add_subplot(111)
        self.edge_ax.set_title("RX Edge Delta per Bit Window")
        self.edge_ax.set_xlabel("Time (s)")
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

            t = time.time() - self.data.start_time
            # keep edge history aligned even if adc lines are less frequent
            if not self.data.hist_t or (t - self.data.hist_t[-1] > 0.05):
                self.data.hist_t.append(t)
                for k in POWER_KEYS:
                    self.data.hist_power[k].append(self.data.power_mv[k] / 1000.0)
            self.data.hist_edge.append(self.data.alive.get("last_edge_delta", 0))

        elif line.startswith("last_rx"):
            rx = parse_last_rx(line)
            if rx is not None:
                if not rx["has_rx"]:
                    self.data.last_rx_text = "none"
                else:
                    self.data.last_rx_text = f"len={rx['len']} payload={rx['payload']}"

    def _append_raw(self, line: str):
        self.raw_text.insert(tk.END, line + "\n")
        # avoid unlimited Tk Text growth
        line_count = int(self.raw_text.index("end-1c").split(".")[0])
        if line_count > 1200:
            self.raw_text.delete("1.0", "200.0")
        self.raw_text.see(tk.END)

    # ---------------- Refresh UI ----------------
    def _refresh_ui(self):
        for k, var in self.power_vars.items():
            var.set(f"{self.data.power_mv.get(k, 0) / 1000.0:.3f} V")

        for k, var in self.alive_vars.items():
            val = self.data.alive.get(k, 0)
            var.set(str(val))

        state = self.data.alive.get("rx_sync_state", 0)
        self.rx_state_name_var.set(STATE_NAMES.get(state, f"STATE_{state}"))
        self.last_rx_var.set(self.data.last_rx_text)

        self._redraw_power_chart()
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

        if any(v.get() for v in self.selected_power_keys.values()):
            self.power_ax.legend(loc="upper right")
        self.power_canvas.draw_idle()

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
