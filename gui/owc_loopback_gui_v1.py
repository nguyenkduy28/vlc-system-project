#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OWC TX/RX LOOPBACK GUI - STM32F407
For APP_BOARD_ROLE_TX_RX_LOOPBACK on one MCU.
Requires: pip install pyserial matplotlib
Run: python owc_loopback_gui_v1.py
"""
import re, time, math, calendar, queue, threading
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

PREAMBLE=[0xAA,0xAA,0xAA]; SYNC=0xD5; DEFAULT_PAYLOAD=[0x55,0xA5,0x3C,0xC3]
FIELD_NAMES=["Preamble","Sync","Length","Payload","Checksum"]
EDGE_THRESHOLD=6; DAC_EXPECTED_MV=1650; OOK_BIT_US=10.0; OOK_CARRIER_US=1.0
POWER_KEYS=["rx_out_a","vmon_bu","vmon_bu_3v","vmon_main_sys","vmon_main","vmon_5v_sys","vmon_3v3_sys"]
POWER_EXPECTED_MV={
    "rx_out_a":(1800,0,3300),"vmon_bu":(5000,4500,5500),"vmon_bu_3v":(3000,2800,3300),
    "vmon_main_sys":(9400,8500,10000),"vmon_main":(9400,8500,10000),"vmon_5v_sys":(5000,4750,5250),"vmon_3v3_sys":(3300,3135,3465)
}
RX_STATE_NAMES={0:"WAIT_ACTIVITY",1:"DETECT_PREAMBLE",2:"LOCK_BIT_TIMING",3:"READ_SYNC",4:"READ_LEN",5:"READ_PAYLOAD",6:"READ_CHECKSUM"}

def checksum(payload:List[int])->int: return (len(payload)+sum(payload))&0xFF
def build_frame(payload:List[int])->List[int]: return PREAMBLE+[SYNC,len(payload)]+list(payload)+[checksum(payload)]
def hex_bytes(v:List[int])->str: return " ".join(f"{b:02X}" for b in v)
def bits_from_bytes(v:List[int])->List[int]:
    out=[]
    for b in v:
        out += [(b>>p)&1 for p in range(7,-1,-1)]
    return out

def split_frame(frame:List[int])->Dict[str,List[int]]:
    if len(frame)<6:
        return {"Preamble":frame[:3],"Sync":frame[3:4],"Length":frame[4:5],"Payload":[],"Checksum":frame[-1:] if frame else []}
    ln=frame[4]; ps=5; pe=min(ps+ln, max(len(frame)-1, ps))
    return {"Preamble":frame[:3],"Sync":frame[3:4],"Length":frame[4:5],"Payload":frame[ps:pe],"Checksum":frame[pe:pe+1] if pe<len(frame) else []}

def parse_kv_int(line:str)->Dict[str,int]:
    out={}
    for k,v in re.findall(r"([A-Za-z0-9_]+)=(-?\d+)", line):
        try: out[k]=int(v)
        except ValueError: pass
    return out

def parse_hex_field(line:str, field:str)->List[int]:
    m=re.search(rf"(?:^|\s){re.escape(field)}=([0-9A-Fa-fxX ]+?)(?:\s+[A-Za-z0-9_]+=|$)", line)
    if not m: return []
    vals=[]
    for tok in m.group(1).strip().split():
        tok=tok.replace("0x","").replace("0X","")
        if re.fullmatch(r"[0-9A-Fa-f]{1,2}", tok): vals.append(int(tok,16))
    return vals

def parse_frame_field(line:str)->List[int]:
    frame=parse_hex_field(line,"frame")
    if frame: return frame
    payload=parse_hex_field(line,"payload")
    return build_frame(payload) if payload else []

def normalize_line(line:str)->str:
    line=line.strip()
    toks=["role ","alive ","tx_frame ","tx_status ","last_rx ","rx_frame ","link_stats ","link_quality ","err_summary ","err_frame ","err_bits ","adc_mv ","dac_mv ","rtc ","rtc_event ","rtc_bkp ","cmd_ok ","cmd_err "]
    pos=[(line.find(t),t) for t in toks if line.find(t)>=0]
    return line[min(pos)[0]:] if pos else line

def mv_status(key:str,mv:int)->str:
    _,lo,hi=POWER_EXPECTED_MV[key]
    return "OK" if lo<=mv<=hi else "WARN"

def parse_positions(s:str)->List[int]:
    return [int(x) for x in re.split(r"[,; ]+",s.strip()) if x.isdigit()]

class SerialReader(threading.Thread):
    def __init__(self, port:str, baud:int, out_q:"queue.Queue[str]"):
        super().__init__(daemon=True); self.port=port; self.baud=baud; self.out_q=out_q; self.stop_event=threading.Event(); self.write_q=queue.Queue(); self.ser=None; self.rx=bytearray()
    def write_line(self,line:str): self.write_q.put(line if line.endswith("\n") else line+"\n")
    def run(self):
        try:
            self.ser=serial.Serial(self.port,self.baud,timeout=0.05)
            try: self.ser.reset_input_buffer(); self.ser.reset_output_buffer()
            except Exception: pass
            self.out_q.put(f"__STATUS__ Connected to {self.port} @ {self.baud}")
        except Exception as e:
            self.out_q.put(f"__ERROR__ Cannot open {self.port}: {e}"); return
        while not self.stop_event.is_set():
            try:
                while not self.write_q.empty():
                    s=self.write_q.get_nowait(); self.ser.write(s.encode("ascii",errors="ignore")); self.out_q.put(f"__STATUS__ Sent: {s.strip()}")
                chunk=self.ser.read(self.ser.in_waiting or 1)
                if not chunk: continue
                self.rx.extend(chunk)
                if len(self.rx)>4096:
                    line=self.rx.decode("utf-8",errors="replace").strip(); self.rx.clear()
                    if line: self.out_q.put(line)
                while b"\n" in self.rx:
                    line_raw,_,rest=self.rx.partition(b"\n"); self.rx=bytearray(rest)
                    line=line_raw.decode("utf-8",errors="replace").strip()
                    if line: self.out_q.put(line)
            except Exception as e:
                self.out_q.put(f"__ERROR__ Serial error: {e}"); break
        try:
            if self.ser and self.ser.is_open: self.ser.close()
        except Exception: pass
        self.out_q.put("__STATUS__ Disconnected")
    def stop(self): self.stop_event.set()

@dataclass
class State:
    role:str="UNKNOWN"
    tx_frames:int=0; tx_frame_id:int=0; bit_rate:int=100000
    rx_frames:int=0; frame_errors:int=0; checksum_errors:int=0; rx_sync_state:int=0; last_edge_delta:int=0; last_raw_bit:int=0
    tx_payload:List[int]=field(default_factory=lambda:list(DEFAULT_PAYLOAD)); tx_frame:List[int]=field(default_factory=lambda:build_frame(DEFAULT_PAYLOAD))
    last_rx_none:bool=True; last_rx_payload:List[int]=field(default_factory=list); last_rx_frame:List[int]=field(default_factory=list)
    payload_bits:int=0; payload_bit_errors:int=0; payload_ber_ppm:int=0; payload_mismatch_frames:int=0; good_payload_frames:int=0; rx_total_observed:int=0; per_ppm:int=0
    err_queued:int=0; err_printed:int=0; err_suppressed:int=0
    power_mv:Dict[str,int]=field(default_factory=lambda:{k:0 for k in POWER_KEYS}); dac_mv:int=DAC_EXPECTED_MV
    rtc_time:str="---- -- -- --:--:--"; rtc_valid:int=0; rtc_source:str="-"; rtc_backup:str="-"; rtc_event:str="none"
    last_err_bits_tx:str=""; last_err_bits_rx:str=""; last_err_positions:List[int]=field(default_factory=list); last_error_text:str="none"
    edge_history:deque=field(default_factory=lambda:deque(maxlen=320)); error_events:deque=field(default_factory=lambda:deque(maxlen=300)); raw_lines:deque=field(default_factory=lambda:deque(maxlen=3000))
    start_time:float=field(default_factory=time.time); last_alive_time:Optional[float]=None; last_tx_frames:int=0; last_rx_frames:int=0; tx_fps:float=0.0; rx_fps:float=0.0; goodput_bps:float=0.0
    hist_t:deque=field(default_factory=lambda:deque(maxlen=300)); hist_power:Dict[str,deque]=field(default_factory=lambda:{k:deque(maxlen=300) for k in POWER_KEYS})
    hist_m_t:deque=field(default_factory=lambda:deque(maxlen=300)); hist_ber:deque=field(default_factory=lambda:deque(maxlen=300)); hist_per:deque=field(default_factory=lambda:deque(maxlen=300)); hist_tx:deque=field(default_factory=lambda:deque(maxlen=300)); hist_rx:deque=field(default_factory=lambda:deque(maxlen=300))

class LoopGui(tk.Tk):
    def __init__(self):
        super().__init__(); self.title("OWC TX/RX LOOPBACK Monitor - STM32F407 - v1"); self.geometry("1560x920"); self.minsize(1220,760)
        self.state=State(); self.q=queue.Queue(); self.reader=None; self.max_bits=tk.IntVar(value=80); self.wave_mode=tk.StringVar(value="TX vs RX frame")
        self.selected_power={k:tk.BooleanVar(value=(k!="rx_out_a")) for k in POWER_KEYS}
        self._style(); self._build_ui(); self._refresh_ports(); self.after(50,self._process_q); self.after(250,self._refresh_ui); self.protocol("WM_DELETE_WINDOW", self._on_close)
    def _style(self):
        st=ttk.Style(self); st.configure("Header.TLabel",font=("Segoe UI",11,"bold")); st.configure("Big.TLabel",font=("Consolas",15,"bold")); st.configure("Ok.TLabel",foreground="#087d2c",font=("Segoe UI",10,"bold")); st.configure("Warn.TLabel",foreground="#b00020",font=("Segoe UI",10,"bold"))
    def _build_ui(self):
        top=ttk.Frame(self,padding=8); top.pack(side=tk.TOP,fill=tk.X)
        ttk.Label(top,text="COM:").pack(side=tk.LEFT); self.port_combo=ttk.Combobox(top,width=14,state="readonly"); self.port_combo.pack(side=tk.LEFT,padx=(4,8)); ttk.Button(top,text="Refresh",command=self._refresh_ports).pack(side=tk.LEFT,padx=(0,8))
        ttk.Label(top,text="Baud:").pack(side=tk.LEFT); self.baud_var=tk.StringVar(value="115200"); ttk.Entry(top,textvariable=self.baud_var,width=10).pack(side=tk.LEFT,padx=(4,8))
        self.connect_btn=ttk.Button(top,text="Connect",command=self._toggle_connect); self.connect_btn.pack(side=tk.LEFT,padx=(0,12)); ttk.Button(top,text="Inject loopback sample",command=self._inject_sample).pack(side=tk.LEFT,padx=(0,12))
        self.status_var=tk.StringVar(value="Disconnected"); ttk.Label(top,textvariable=self.status_var).pack(side=tk.LEFT)
        self.nb=ttk.Notebook(self); self.nb.pack(fill=tk.BOTH,expand=True)
        self.overview_tab=ttk.Frame(self.nb); self.frame_tab=ttk.Frame(self.nb); self.metrics_tab=ttk.Frame(self.nb); self.power_tab=ttk.Frame(self.nb); self.rtc_tab=ttk.Frame(self.nb); self.raw_tab=ttk.Frame(self.nb)
        for tab,name in [(self.overview_tab,"LOOPBACK OVERVIEW"),(self.frame_tab,"TX/RX FRAME"),(self.metrics_tab,"LINK METRICS"),(self.power_tab,"POWER"),(self.rtc_tab,"RTC"),(self.raw_tab,"RAW LOG")]: self.nb.add(tab,text=name)
        self._build_overview(); self._build_frame(); self._build_metrics(); self._build_power(); self._build_rtc(); self._build_raw()
    def _build_overview(self):
        row=ttk.Frame(self.overview_tab,padding=(10,8)); row.pack(side=tk.TOP,fill=tk.X); self.role_var=tk.StringVar(value="role: UNKNOWN"); ttk.Label(row,textvariable=self.role_var,style="Header.TLabel").pack(side=tk.LEFT,padx=(0,12))
        self.card={};
        for title in ["tx_frames","rx_frames","frame_errors","checksum_errors","rx_sync_state","last_edge_delta","last_raw_bit","TX fps","RX fps"]:
            var=tk.StringVar(value="0"); self.card[title]=var; box=ttk.Frame(row,padding=5,relief="ridge"); box.pack(side=tk.LEFT,padx=4); ttk.Label(box,text=title).pack(); ttk.Label(box,textvariable=var,style="Big.TLabel").pack()
        strip=ttk.Frame(self.overview_tab,padding=(10,0)); strip.pack(side=tk.TOP,fill=tk.X); self.rx_state_var=tk.StringVar(value="RX state: WAIT_ACTIVITY"); self.loop_status_var=tk.StringVar(value="Loopback status: waiting"); self.rtc_summary_var=tk.StringVar(value="RTC: waiting")
        ttk.Label(strip,textvariable=self.rx_state_var,style="Header.TLabel").pack(side=tk.LEFT,padx=(0,24)); self.loop_label=ttk.Label(strip,textvariable=self.loop_status_var,style="Warn.TLabel"); self.loop_label.pack(side=tk.LEFT,padx=(0,24)); ttk.Label(strip,textvariable=self.rtc_summary_var).pack(side=tk.LEFT)
        box=ttk.LabelFrame(self.overview_tab,text="Compact Frame View",padding=8); box.pack(side=tk.TOP,fill=tk.X,padx=10,pady=(8,6)); self.tx_compact=ttk.Frame(box); self.tx_compact.pack(fill=tk.X,pady=2); self.rx_compact=ttk.Frame(box); self.rx_compact.pack(fill=tk.X,pady=2)
        body=ttk.Frame(self.overview_tab,padding=(10,0)); body.pack(fill=tk.BOTH,expand=True); left=ttk.LabelFrame(body,text="RX FSM / Debug",padding=8); left.pack(side=tk.LEFT,fill=tk.Y,padx=(0,8))
        self.fsm=ttk.Treeview(left,columns=("id","state"),show="headings",height=8); self.fsm.heading("id",text="ID"); self.fsm.heading("state",text="State"); self.fsm.column("id",width=50,anchor=tk.CENTER); self.fsm.column("state",width=180); self.fsm.tag_configure("active",background="#d7ecff"); self.fsm.pack(fill=tk.X)
        for sid,name in RX_STATE_NAMES.items(): self.fsm.insert("","end",iid=str(sid),values=(sid,name))
        self.debug_var=tk.StringVar(value="last_edge_delta=0 | last_raw_bit=0"); ttk.Label(left,textvariable=self.debug_var,font=("Consolas",10),wraplength=280).pack(anchor=tk.W,pady=(10,0))
        ttk.Label(left,text="Mode TX_RX_LOOPBACK chạy TX và RX trên cùng MCU. Log alive dùng keyword 'alive'.",wraplength=280).pack(anchor=tk.W,pady=(10,0))
        right=ttk.LabelFrame(body,text="TX/RX waveform compare",padding=8); right.pack(side=tk.LEFT,fill=tk.BOTH,expand=True); ctrl=ttk.Frame(right); ctrl.pack(fill=tk.X)
        ttk.Label(ctrl,text="Wave view:").pack(side=tk.LEFT); ttk.Combobox(ctrl,textvariable=self.wave_mode,values=["TX vs RX frame","TX only OOK","RX reconstructed edges","Last error bits"],width=24,state="readonly").pack(side=tk.LEFT,padx=(4,14)); ttk.Label(ctrl,text="Max bits:").pack(side=tk.LEFT); ttk.Spinbox(ctrl,from_=16,to=160,increment=8,textvariable=self.max_bits,width=6).pack(side=tk.LEFT,padx=(4,0))
        self.wave_fig=Figure(figsize=(9,4.7),dpi=100); self.wave_ax=self.wave_fig.add_subplot(111); self.wave_canvas=FigureCanvasTkAgg(self.wave_fig,master=right); self.wave_canvas.get_tk_widget().pack(fill=tk.BOTH,expand=True)
    def _build_frame(self):
        top=ttk.Frame(self.frame_tab,padding=10); top.pack(side=tk.TOP,fill=tk.X); self.tx_detail=tk.StringVar(value="TX frame: waiting"); self.rx_detail=tk.StringVar(value="RX frame: none"); ttk.Label(top,textvariable=self.tx_detail,style="Header.TLabel",wraplength=1400).pack(anchor=tk.W,pady=3); ttk.Label(top,textvariable=self.rx_detail,style="Header.TLabel",wraplength=1400).pack(anchor=tk.W,pady=3)
        box=ttk.LabelFrame(self.frame_tab,text="Field-by-field compare",padding=8); box.pack(fill=tk.BOTH,expand=True,padx=10,pady=(4,8)); self.compare=ttk.Treeview(box,columns=("tx","rx","status","note"),show="tree headings",height=7); self.compare.heading("#0",text="Field"); self.compare.column("#0",width=110)
        for col,w in [("tx",280),("rx",280),("status",100),("note",480)]: self.compare.heading(col,text=col.upper()); self.compare.column(col,width=w,anchor=tk.W)
        self.compare.tag_configure("ok",background="#eaffea"); self.compare.tag_configure("err",background="#ffd9d9"); self.compare.tag_configure("na",background="#eeeeee"); self.compare.pack(fill=tk.BOTH,expand=True)
        err=ttk.LabelFrame(self.frame_tab,text="Error detail",padding=8); err.pack(side=tk.BOTTOM,fill=tk.X,padx=10,pady=(0,8)); self.err_tree=ttk.Treeview(err,columns=("kind","frame_id","summary"),show="headings",height=5)
        for col,w in [("kind",120),("frame_id",100),("summary",980)]: self.err_tree.heading(col,text=col.title()); self.err_tree.column(col,width=w,anchor=tk.W)
        self.err_tree.pack(fill=tk.X); self.err_detail=tk.Text(err,height=4,wrap=tk.WORD,font=("Consolas",9)); self.err_detail.pack(fill=tk.X,pady=(6,0)); self.err_tree.bind("<<TreeviewSelect>>",self._on_error_selected)
    def _build_metrics(self):
        top=ttk.Frame(self.metrics_tab,padding=10); top.pack(side=tk.TOP,fill=tk.X); self.metric={}
        for title,default in [("payload_bits","0"),("payload_bit_errors","0"),("payload_ber_ppm","0 ppm"),("payload_mismatch_frames","0"),("good_payload_frames","0"),("rx_total_observed","0"),("per_ppm","0 ppm"),("err_summary","queued=0 printed=0 suppressed=0"),("goodput_bps","0 bps"),("frame_success","0 %")]:
            var=tk.StringVar(value=default); self.metric[title]=var; box=ttk.Frame(top,padding=5,relief="ridge"); box.pack(side=tk.LEFT,padx=4,pady=2); ttk.Label(box,text=title).pack(); ttk.Label(box,textvariable=var,style="Big.TLabel").pack()
        chart=ttk.LabelFrame(self.metrics_tab,text="BER/PER and TX/RX FPS history",padding=8); chart.pack(fill=tk.BOTH,expand=True,padx=10,pady=(4,8)); self.metric_fig=Figure(figsize=(9,4.8),dpi=100); self.metric_ax=self.metric_fig.add_subplot(111); self.metric_canvas=FigureCanvasTkAgg(self.metric_fig,master=chart); self.metric_canvas.get_tk_widget().pack(fill=tk.BOTH,expand=True)
    def _build_power(self):
        main=ttk.Frame(self.power_tab,padding=10); main.pack(fill=tk.BOTH,expand=True); left=ttk.Frame(main); left.pack(side=tk.LEFT,fill=tk.Y,padx=(0,8)); self.power_tree=ttk.Treeview(left,columns=("measured","expected","range","status"),show="tree headings",height=10); self.power_tree.heading("#0",text="Signal"); self.power_tree.column("#0",width=150)
        for col,w in [("measured",95),("expected",95),("range",125),("status",70)]: self.power_tree.heading(col,text=col.title()); self.power_tree.column(col,width=w,anchor=tk.CENTER)
        self.power_tree.tag_configure("ok",background="#eaffea"); self.power_tree.tag_configure("warn",background="#ffd9d9"); self.power_tree.pack(fill=tk.BOTH,expand=True)
        for k in POWER_KEYS:
            exp,lo,hi=POWER_EXPECTED_MV[k]; self.power_tree.insert("","end",iid=k,text=k,values=("0.000 V",f"{exp/1000:.3f} V",f"{lo/1000:.2f}-{hi/1000:.2f} V","WARN"),tags=("warn",))
        self.dac_var=tk.StringVar(value="DAC threshold: 1.650 V"); ttk.Label(left,textvariable=self.dac_var,style="Header.TLabel").pack(anchor=tk.W,pady=(10,0)); ttk.Separator(left).pack(fill=tk.X,pady=10); ttk.Label(left,text="Chart signals",style="Header.TLabel").pack(anchor=tk.W)
        for k,v in self.selected_power.items(): ttk.Checkbutton(left,text=k,variable=v).pack(anchor=tk.W)
        right=ttk.Frame(main); right.pack(side=tk.LEFT,fill=tk.BOTH,expand=True); self.power_fig=Figure(figsize=(8,5),dpi=100); self.power_ax=self.power_fig.add_subplot(111); self.power_canvas=FigureCanvasTkAgg(self.power_fig,master=right); self.power_canvas.get_tk_widget().pack(fill=tk.BOTH,expand=True)
    def _build_rtc(self):
        main=ttk.Frame(self.rtc_tab,padding=10); main.pack(fill=tk.BOTH,expand=True); top=ttk.LabelFrame(main,text="RTC LSE/VBAT Status",padding=10); top.pack(side=tk.TOP,fill=tk.X); self.rtc_vars={}
        for i,k in enumerate(["RTC time","Valid","Source","Backup","Last event"]):
            var=tk.StringVar(value="-"); self.rtc_vars[k]=var; ttk.Label(top,text=f"{k}:",style="Header.TLabel").grid(row=i,column=0,sticky=tk.W,pady=4,padx=(0,8)); ttk.Label(top,textvariable=var,font=("Consolas",12,"bold")).grid(row=i,column=1,sticky=tk.W,pady=4)
        cmd=ttk.LabelFrame(main,text="RTC Commands",padding=10); cmd.pack(side=tk.TOP,fill=tk.X,pady=(10,8)); ttk.Button(cmd,text="rtc_get",command=lambda:self._send_cmd("rtc_get")).pack(side=tk.LEFT,padx=3); ttk.Button(cmd,text="rtc_bkp",command=lambda:self._send_cmd("rtc_bkp")).pack(side=tk.LEFT,padx=3); ttk.Button(cmd,text="rtc_set from PC time",command=self._cmd_rtc_pc).pack(side=tk.LEFT,padx=3)
        content=ttk.Frame(main); content.pack(fill=tk.BOTH,expand=True); clock=ttk.LabelFrame(content,text="RTC Analog Clock",padding=8); clock.pack(side=tk.LEFT,fill=tk.BOTH,expand=True,padx=(0,8)); self.clock_canvas=tk.Canvas(clock,width=420,height=420,bg="white",highlightthickness=1,highlightbackground="#cccccc"); self.clock_canvas.pack(fill=tk.BOTH,expand=True)
        cal=ttk.LabelFrame(content,text="Calendar",padding=8); cal.pack(side=tk.LEFT,fill=tk.BOTH,expand=True); self.calendar_text=tk.Text(cal,width=42,height=18,font=("Consolas",14),wrap=tk.NONE); self.calendar_text.pack(fill=tk.BOTH,expand=True); self.calendar_text.tag_configure("today",background="#d7ecff",foreground="#000000")
    def _build_raw(self):
        f=ttk.Frame(self.raw_tab,padding=8); f.pack(fill=tk.BOTH,expand=True); self.raw_text=tk.Text(f,height=20,wrap=tk.NONE,font=("Consolas",10)); self.raw_text.pack(side=tk.LEFT,fill=tk.BOTH,expand=True); y=ttk.Scrollbar(f,orient=tk.VERTICAL,command=self.raw_text.yview); y.pack(side=tk.RIGHT,fill=tk.Y); self.raw_text.configure(yscrollcommand=y.set)
    def _refresh_ports(self):
        ports=[p.device for p in list_ports.comports()]; self.port_combo["values"]=ports
        if ports and not self.port_combo.get(): self.port_combo.current(0)
    def _toggle_connect(self):
        if self.reader is None:
            port=self.port_combo.get().strip()
            if not port: messagebox.showwarning("Missing COM","Chọn COM port trước."); return
            try: baud=int(self.baud_var.get())
            except ValueError: messagebox.showwarning("Invalid baud","Baud rate không hợp lệ."); return
            self.reader=SerialReader(port,baud,self.q); self.reader.start(); self.connect_btn.configure(text="Disconnect"); self.status_var.set("Connecting...")
        else:
            self.reader.stop(); self.reader=None; self.connect_btn.configure(text="Connect"); self.status_var.set("Disconnecting...")
    def _send_cmd(self,cmd):
        if self.reader is None: messagebox.showinfo("Not connected",f"Chưa kết nối COM. Lệnh chưa gửi: {cmd}"); return
        self.reader.write_line(cmd)
    def _cmd_rtc_pc(self): self._send_cmd("rtc_set "+time.strftime("%Y-%m-%d %H:%M:%S",time.localtime()))
    def _process_q(self):
        while True:
            try: line=self.q.get_nowait()
            except queue.Empty: break
            if line.startswith("__ERROR__") or line.startswith("__STATUS__"):
                self.status_var.set(line.replace("__ERROR__ ","").replace("__STATUS__ ","")); self._append_raw(line)
                if "Disconnected" in line: self.reader=None; self.connect_btn.configure(text="Connect")
            else: self._handle_line(line)
        self.after(50,self._process_q)
    def _handle_line(self,line):
        self.state.raw_lines.append(line); self._append_raw(line); line=normalize_line(line)
        if line.startswith("role"):
            m=re.search(r"board_role=([^\s]+)",line); self.state.role=m.group(1) if m else self.state.role
        elif line.startswith("alive"):
            kv=parse_kv_int(line)
            for k in ["tx_frames","rx_frames","frame_errors","checksum_errors","rx_sync_state","last_edge_delta","last_raw_bit","bit_rate"]:
                if k in kv: setattr(self.state,k,kv[k])
            self.state.edge_history.append(self.state.last_edge_delta); self._update_rates()
        elif line.startswith("tx_frame") or line.startswith("tx_status"):
            kv=parse_kv_int(line); self.state.tx_frame_id=kv.get("tx_frame_id",self.state.tx_frame_id); self.state.bit_rate=kv.get("bit_rate",self.state.bit_rate); payload=parse_hex_field(line,"payload"); frame=parse_frame_field(line)
            if payload: self.state.tx_payload=payload
            if frame: self.state.tx_frame=frame
        elif line.startswith("last_rx") or line.startswith("rx_frame"):
            if line.startswith("last_rx") and "none" in line:
                self.state.last_rx_none=True; self.state.last_rx_payload=[]; self.state.last_rx_frame=[]
            else:
                self.state.last_rx_none=False; payload=parse_hex_field(line,"payload"); frame=parse_frame_field(line)
                if payload: self.state.last_rx_payload=payload
                if frame: self.state.last_rx_frame=frame
                elif payload: self.state.last_rx_frame=build_frame(payload)
        elif line.startswith("link_stats"):
            kv=parse_kv_int(line)
            for k in ["payload_bits","payload_bit_errors","payload_ber_ppm","payload_mismatch_frames","good_payload_frames"]:
                if k in kv: setattr(self.state,k,kv[k])
            self._append_metric_hist()
        elif line.startswith("link_quality"):
            kv=parse_kv_int(line); self.state.rx_total_observed=kv.get("rx_total_observed",self.state.rx_total_observed); self.state.per_ppm=kv.get("per_ppm",self.state.per_ppm); self._append_metric_hist()
        elif line.startswith("err_summary"):
            kv=parse_kv_int(line); self.state.err_queued=kv.get("queued",self.state.err_queued); self.state.err_printed=kv.get("printed",self.state.err_printed); self.state.err_suppressed=kv.get("suppressed",self.state.err_suppressed)
        elif line.startswith("err_bits"): self._parse_err_bits(line)
        elif line.startswith("err_frame"): self._capture_error("err_frame",line)
        elif line.startswith("adc_mv"):
            kv=parse_kv_int(line)
            for k in POWER_KEYS:
                if k in kv: self.state.power_mv[k]=kv[k]
            t=time.time()-self.state.start_time; self.state.hist_t.append(t)
            for k in POWER_KEYS: self.state.hist_power[k].append(self.state.power_mv[k]/1000.0)
        elif line.startswith("dac"):
            kv=parse_kv_int(line)
            for k in ("threshold","threshold_mv","dac_threshold_mv","dac_mv"):
                if k in kv: self.state.dac_mv=kv[k]; break
        elif line.startswith("rtc "): self._parse_rtc(line)
        elif line.startswith("rtc_event"): self.state.rtc_event=line
        elif line.startswith("rtc_bkp"):
            self.state.rtc_event=line; m=re.search(r"backup=([^\s]+)",line); self.state.rtc_backup=m.group(1) if m else self.state.rtc_backup
    def _update_rates(self):
        now=time.time()
        if self.state.last_alive_time is not None:
            dt=max(now-self.state.last_alive_time,1e-6); self.state.tx_fps=max(self.state.tx_frames-self.state.last_tx_frames,0)/dt; self.state.rx_fps=max(self.state.rx_frames-self.state.last_rx_frames,0)/dt; self.state.goodput_bps=self.state.rx_fps*max(len(self.state.tx_payload)*8,1)
        self.state.last_alive_time=now; self.state.last_tx_frames=self.state.tx_frames; self.state.last_rx_frames=self.state.rx_frames; self._append_metric_hist()
    def _append_metric_hist(self):
        t=time.time()-self.state.start_time
        if self.state.hist_m_t and t-self.state.hist_m_t[-1]<0.20: return
        self.state.hist_m_t.append(t); self.state.hist_ber.append(self.state.payload_ber_ppm); self.state.hist_per.append(self.state.per_ppm); self.state.hist_tx.append(self.state.tx_fps); self.state.hist_rx.append(self.state.rx_fps)
    def _parse_err_bits(self,line):
        mt=re.search(r"tx_bits=([01]+)",line); mr=re.search(r"rx_bits=([01]+)",line); mp=re.search(r"mismatch_positions=([0-9,; ]+)",line); kv=parse_kv_int(line)
        if mt: self.state.last_err_bits_tx=mt.group(1)
        if mr: self.state.last_err_bits_rx=mr.group(1)
        if mp: self.state.last_err_positions=parse_positions(mp.group(1))
        self._capture_error("err_bits",line,kv.get("frame_id",kv.get("rx_frame_id",-1)))
    def _capture_error(self,kind,line,frame_id=None):
        if frame_id is None:
            kv=parse_kv_int(line); frame_id=kv.get("rx_frame_id",kv.get("frame_id",-1))
        self.state.last_error_text=line; self.state.error_events.appendleft({"kind":kind,"frame_id":frame_id,"summary":line,"raw":line})
    def _parse_rtc(self,line):
        mt=re.search(r"time=(\d{4}-\d{2}-\d{2}\s+\d{2}:\d{2}:\d{2})",line); mv=re.search(r"valid=(\d+)",line); ms=re.search(r"source=([^\s]+)",line); mb=re.search(r"backup=([^\s]+)",line)
        if mt: self.state.rtc_time=mt.group(1)
        if mv: self.state.rtc_valid=int(mv.group(1))
        if ms: self.state.rtc_source=ms.group(1)
        if mb: self.state.rtc_backup=mb.group(1)
    def _refresh_ui(self):
        st=RX_STATE_NAMES.get(self.state.rx_sync_state,f"UNKNOWN_{self.state.rx_sync_state}"); self.role_var.set(f"role: {self.state.role}")
        for k in ["tx_frames","rx_frames","frame_errors","checksum_errors","rx_sync_state","last_edge_delta","last_raw_bit"]: self.card[k].set(str(getattr(self.state,k)))
        self.card["TX fps"].set(f"{self.state.tx_fps:.1f}"); self.card["RX fps"].set(f"{self.state.rx_fps:.1f}"); self.rx_state_var.set(f"RX state: {st} ({self.state.rx_sync_state})"); self.debug_var.set(f"last_edge_delta={self.state.last_edge_delta} | last_raw_bit={self.state.last_raw_bit}")
        if self.state.rx_frames>0: self.loop_status_var.set("Loopback status: RX has valid frames"); self.loop_label.configure(style="Ok.TLabel")
        elif self.state.last_edge_delta>0: self.loop_status_var.set("Loopback status: RX sees edges but no valid frame"); self.loop_label.configure(style="Warn.TLabel")
        else: self.loop_status_var.set("Loopback status: TX active, RX input idle/no edges"); self.loop_label.configure(style="Warn.TLabel")
        self.rtc_summary_var.set(f"RTC: {self.state.rtc_time} | valid={self.state.rtc_valid} | source={self.state.rtc_source} | backup={self.state.rtc_backup}")
        self._refresh_frames(); self._refresh_compare(); self._refresh_fsm(); self._refresh_metrics(); self._refresh_errors(); self._refresh_power(); self._refresh_rtc(); self._redraw_wave(); self._redraw_metrics(); self._redraw_power(); self._redraw_clock(); self.after(250,self._refresh_ui)
    def _compact(self,parent,label,frame,none=False):
        for c in parent.winfo_children(): c.destroy()
        ttk.Label(parent,text=label,width=8,style="Header.TLabel").pack(side=tk.LEFT)
        if none: ttk.Label(parent,text="none",font=("Consolas",12,"bold")).pack(side=tk.LEFT); return
        colors={"Preamble":"#d7ecff","Sync":"#ffe5c2","Length":"#eadcff","Payload":"#dff6df","Checksum":"#ffd8d8"}; f=split_frame(frame)
        for name in FIELD_NAMES:
            vals=f.get(name,[]); tk.Label(parent,text=f"[{hex_bytes(vals) if vals else '--'}]",bg=colors[name],fg="#111",font=("Consolas",11,"bold"),padx=8,pady=4,relief="groove").pack(side=tk.LEFT,padx=(0,4)); tk.Label(parent,text=name,fg="#444",font=("Segoe UI",8)).pack(side=tk.LEFT,padx=(0,8))
    def _refresh_frames(self):
        self._compact(self.tx_compact,"TX:",self.state.tx_frame); self._compact(self.rx_compact,"RX:",self.state.last_rx_frame,self.state.last_rx_none)
        self.tx_detail.set(f"TX frame_id={self.state.tx_frame_id} | len={len(self.state.tx_payload)} | payload={hex_bytes(self.state.tx_payload)} | full={hex_bytes(self.state.tx_frame)}")
        self.rx_detail.set("RX frame: none" if self.state.last_rx_none else f"RX len={len(self.state.last_rx_payload)} | payload={hex_bytes(self.state.last_rx_payload)} | full={hex_bytes(self.state.last_rx_frame)}")
    def _refresh_compare(self):
        for i in self.compare.get_children(): self.compare.delete(i)
        tf=split_frame(self.state.tx_frame); rf=split_frame(self.state.last_rx_frame) if not self.state.last_rx_none else {}; notes={"Preamble":"AA AA AA để RX bắt hoạt động","Sync":"Kỳ vọng D5","Length":"Độ dài payload","Payload":"Dữ liệu chính","Checksum":"LEN+payload, 8 bit thấp"}
        for name in FIELD_NAMES:
            tx=tf.get(name,[]); rx=rf.get(name,[]) if not self.state.last_rx_none else []
            if self.state.last_rx_none: status,tag,rx_txt="N/A","na","--"
            else: status="OK" if tx==rx else "MISMATCH"; tag="ok" if status=="OK" else "err"; rx_txt=hex_bytes(rx)
            self.compare.insert("","end",text=name,values=(hex_bytes(tx),rx_txt,status,notes[name]),tags=(tag,))
    def _refresh_fsm(self):
        for sid in RX_STATE_NAMES: self.fsm.item(str(sid),tags=("active",) if sid==self.state.rx_sync_state else ())
    def _refresh_metrics(self):
        for k in ["payload_bits","payload_bit_errors","payload_ber_ppm","payload_mismatch_frames","good_payload_frames","rx_total_observed","per_ppm"]: self.metric[k].set(f"{getattr(self.state,k)}" + (" ppm" if k.endswith("ppm") else ""))
        self.metric["err_summary"].set(f"queued={self.state.err_queued} printed={self.state.err_printed} suppressed={self.state.err_suppressed}"); self.metric["goodput_bps"].set(f"{self.state.goodput_bps:.1f} bps")
        attempts=self.state.rx_frames+self.state.frame_errors+self.state.checksum_errors; self.metric["frame_success"].set(f"{(self.state.rx_frames*100.0/attempts):.3f} %" if attempts else "0 %")
    def _refresh_errors(self):
        for i in self.err_tree.get_children(): self.err_tree.delete(i)
        for idx,ev in enumerate(self.state.error_events): self.err_tree.insert("","end",iid=f"e{idx}",values=(ev["kind"],ev["frame_id"],ev["summary"][:260]))
    def _refresh_power(self):
        self.dac_var.set(f"DAC threshold: {self.state.dac_mv/1000:.3f} V")
        for k in POWER_KEYS:
            mv=self.state.power_mv.get(k,0); exp,lo,hi=POWER_EXPECTED_MV[k]; st=mv_status(k,mv); self.power_tree.item(k,values=(f"{mv/1000:.3f} V",f"{exp/1000:.3f} V",f"{lo/1000:.2f}-{hi/1000:.2f} V",st),tags=("ok" if st=="OK" else "warn",))
    def _refresh_rtc(self):
        self.rtc_vars["RTC time"].set(self.state.rtc_time); self.rtc_vars["Valid"].set(str(self.state.rtc_valid)); self.rtc_vars["Source"].set(self.state.rtc_source); self.rtc_vars["Backup"].set(self.state.rtc_backup); self.rtc_vars["Last event"].set(self.state.rtc_event)
    def _on_error_selected(self,_=None):
        sel=self.err_tree.selection(); self.err_detail.delete("1.0",tk.END)
        if sel:
            idx=int(sel[0][1:]) if sel[0].startswith("e") else -1
            if 0<=idx<len(self.state.error_events): self.err_detail.insert(tk.END,self.state.error_events[idx]["raw"])
    def _draw_ook(self,bits,yoff,label,max_bits,marks=None):
        marks=marks or []; bits=bits[:max_bits]; tv=[]; yv=[]
        for i,b in enumerate(bits):
            t0=i*OOK_BIT_US; self.wave_ax.axvline(t0,linewidth=0.35,linestyle="--",alpha=0.25)
            if i in marks: self.wave_ax.axvspan(t0,t0+OOK_BIT_US,alpha=0.25)
            if b:
                for n in range(int(OOK_BIT_US/OOK_CARRIER_US*6)+1):
                    t=t0+n*(OOK_CARRIER_US/6); y=yoff+(1.0 if ((t-t0)%OOK_CARRIER_US)<OOK_CARRIER_US/2 else 0.0); tv.append(t); yv.append(y)
            else: tv += [t0,t0+OOK_BIT_US]; yv += [yoff,yoff]
            self.wave_ax.text(t0+OOK_BIT_US/2,yoff+1.08,str(b)+("*" if i in marks else ""),ha="center",va="bottom",fontsize=7)
        if tv: self.wave_ax.step(tv,yv,where="post",label=label)
    def _redraw_wave(self):
        self.wave_ax.clear(); self.wave_ax.grid(True); self.wave_ax.set_xlabel("Time (µs), bit window = 10 µs"); self.wave_ax.set_ylabel("OOK gate")
        try: mb=int(self.max_bits.get())
        except Exception: mb=80
        mode=self.wave_mode.get(); tx_bits=bits_from_bytes(self.state.tx_frame); rx_bits=bits_from_bytes(self.state.last_rx_frame) if not self.state.last_rx_none else []; edge_bits=[1 if e>=EDGE_THRESHOLD else 0 for e in list(self.state.edge_history)[-mb:]]
        if mode=="TX vs RX frame":
            self.wave_ax.set_title("TX frame OOK vs last RX frame OOK"); self._draw_ook(tx_bits,1.4,"TX frame",mb)
            if rx_bits: self._draw_ook(rx_bits,0.0,"RX frame",mb)
            else: self.wave_ax.text(0.5,0.28,"RX frame = none",transform=self.wave_ax.transAxes,ha="center")
            self.wave_ax.set_ylim(-0.2,2.7)
        elif mode=="TX only OOK": self.wave_ax.set_title("TX frame OOK waveform"); self._draw_ook(tx_bits,0.0,"TX frame",mb); self.wave_ax.set_ylim(-0.2,1.35)
        elif mode=="RX reconstructed edges":
            self.wave_ax.set_title(f"RX reconstructed from last_edge_delta history: edge ≥ {EDGE_THRESHOLD} → bit 1")
            if edge_bits:
                self._draw_ook(edge_bits,0.0,"RX edge reconstructed",mb)
                for i,e in enumerate(list(self.state.edge_history)[-mb:]): self.wave_ax.text(i*OOK_BIT_US+OOK_BIT_US/2,-0.15,str(e),ha="center",va="top",fontsize=7)
            else: self.wave_ax.text(0.5,0.5,"Waiting for alive edge samples...",transform=self.wave_ax.transAxes,ha="center")
            self.wave_ax.set_ylim(-0.35,1.35)
        else:
            self.wave_ax.set_title("Last error tx_bits/rx_bits"); tb=[1 if c=="1" else 0 for c in self.state.last_err_bits_tx if c in "01"]; rb=[1 if c=="1" else 0 for c in self.state.last_err_bits_rx if c in "01"]
            if tb: self._draw_ook(tb,1.4,"err tx_bits",mb,self.state.last_err_positions)
            if rb: self._draw_ook(rb,0.0,"err rx_bits",mb,self.state.last_err_positions)
            if not tb and not rb: self.wave_ax.text(0.5,0.5,"No err_bits log yet",transform=self.wave_ax.transAxes,ha="center")
            self.wave_ax.set_ylim(-0.2,2.7)
        self.wave_ax.set_xlim(0,max(mb*OOK_BIT_US,OOK_BIT_US)); self.wave_ax.legend(loc="upper right"); self.wave_canvas.draw_idle()
    def _redraw_metrics(self):
        self.metric_ax.clear(); self.metric_ax.set_title("BER/PER ppm and TX/RX frame rate"); self.metric_ax.set_xlabel("GUI time (s)"); self.metric_ax.grid(True); t=list(self.state.hist_m_t)
        for y,label in [(list(self.state.hist_ber),"payload_ber_ppm"),(list(self.state.hist_per),"per_ppm"),(list(self.state.hist_tx),"TX fps"),(list(self.state.hist_rx),"RX fps")]:
            n=min(len(t),len(y))
            if n>1: self.metric_ax.plot(t[-n:],y[-n:],label=label)
        if len(t)>1: self.metric_ax.legend(loc="upper right")
        self.metric_canvas.draw_idle()
    def _redraw_power(self):
        self.power_ax.clear(); self.power_ax.set_title("Power Monitor"); self.power_ax.set_xlabel("Time (s)"); self.power_ax.set_ylabel("Voltage (V)"); self.power_ax.grid(True); t=list(self.state.hist_t)
        for k,v in self.selected_power.items():
            if not v.get(): continue
            y=list(self.state.hist_power[k]); n=min(len(t),len(y))
            if n>1: self.power_ax.plot(t[-n:],y[-n:],label=k)
        if len(t)>1: self.power_ax.legend(loc="upper right")
        self.power_canvas.draw_idle()
    def _parse_dt(self):
        try: return datetime.strptime(self.state.rtc_time,"%Y-%m-%d %H:%M:%S")
        except Exception: return None
    def _redraw_clock(self):
        dt=self._parse_dt(); c=self.clock_canvas; c.delete("all"); w=max(c.winfo_width(),300); h=max(c.winfo_height(),300); cx=w/2; cy=h/2; r=min(w,h)*0.38; c.create_oval(cx-r,cy-r,cx+r,cy+r,width=3,outline="#0b2d5c")
        for i in range(60):
            a=math.radians(i*6-90); inn=r*(0.86 if i%5==0 else 0.92); out=r*0.98; c.create_line(cx+inn*math.cos(a),cy+inn*math.sin(a),cx+out*math.cos(a),cy+out*math.sin(a),width=2 if i%5==0 else 1,fill="#0b2d5c")
        for n in range(1,13):
            a=math.radians(n*30-90); c.create_text(cx+r*0.72*math.cos(a),cy+r*0.72*math.sin(a),text=str(n),font=("Segoe UI",12,"bold"),fill="#0b2d5c")
        if dt:
            sec=dt.second; minute=dt.minute+sec/60; hour=(dt.hour%12)+minute/60
            for deg,l,wid,col in [(hour*30-90,r*0.45,5,"#0b2d5c"),(minute*6-90,r*0.66,4,"#0b2d5c"),(sec*6-90,r*0.78,2,"#b00020")]:
                a=math.radians(deg); c.create_line(cx,cy,cx+l*math.cos(a),cy+l*math.sin(a),width=wid,fill=col)
            c.create_oval(cx-5,cy-5,cx+5,cy+5,fill="#0b2d5c"); c.create_text(cx,cy+r+30,text=dt.strftime("%Y-%m-%d  %H:%M:%S"),font=("Consolas",16,"bold"),fill="#0b2d5c")
        else: c.create_text(cx,cy,text="Waiting for RTC log",font=("Segoe UI",16,"bold"),fill="#777")
        self._update_calendar(dt)
    def _update_calendar(self,dt):
        self.calendar_text.configure(state=tk.NORMAL); self.calendar_text.delete("1.0",tk.END)
        if not dt: self.calendar_text.insert(tk.END,"Waiting for RTC log...\n"); self.calendar_text.configure(state=tk.DISABLED); return
        self.calendar_text.insert(tk.END,calendar.TextCalendar(firstweekday=0).formatmonth(dt.year,dt.month,w=4,l=1)); day=f"{dt.day:2d}"; idx=self.calendar_text.search(day,"1.0",tk.END)
        while idx:
            end=f"{idx}+{len(day)}c"
            if int(idx.split(".")[0])>=3: self.calendar_text.tag_add("today",idx,end); break
            idx=self.calendar_text.search(day,end,tk.END)
        self.calendar_text.configure(state=tk.DISABLED)
    def _append_raw(self,line):
        self.raw_text.insert(tk.END,line+"\n")
        if int(self.raw_text.index("end-1c").split(".")[0])>1800: self.raw_text.delete("1.0","300.0")
        self.raw_text.see(tk.END)
    def _inject_sample(self):
        sample='''link_stats payload_bits=0 payload_bit_errors=0 payload_ber_ppm=0 payload_mismatch_frames=0 good_payload_frames=0
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
adc_mv rx_out_a=1810 vmon_bu=4953 vmon_bu_3v=3025 vmon_main_sys=9075 vmon_main=9238 vmon_5v_sys=5018 vmon_3v3_sys=3297'''
        for line in sample.splitlines(): self._handle_line(line.strip())
        self.status_var.set("Injected TX_RX_LOOPBACK sample log")
    def _on_close(self):
        if self.reader: self.reader.stop()
        self.destroy()

if __name__=="__main__":
    LoopGui().mainloop()
