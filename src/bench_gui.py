#!/usr/bin/env python3
"""
TVC flight computer — bench annunciator panel.

A desktop GUI for hardware bring-up. Reads the $HLTH frames emitted by
health_emit_frame() and drives the serial command menus in bench_test.cpp
and main_control_loop.cpp.

    python3 tools/bench_gui.py              # pick a port in the UI
    python3 tools/bench_gui.py -p COM3
    python3 tools/bench_gui.py --demo       # synthesised data, no hardware

Requires only pyserial (already a PlatformIO dependency) and tkinter, which
ships with CPython on Windows and macOS. On Debian/Ubuntu:
    sudo apt install python3-tk

Layout follows annunciator convention rather than dashboard convention:
green is normal, amber is caution, red is warning, grey is inop. That is
the same colour language as the panel hardware this vehicle is modelled on,
so a lit amber lamp means the same thing here as it would in a cockpit.
"""

import argparse
import queue
import random
import sys
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
from tkinter import ttk, filedialog

# ----------------------------------------------------------------------
# Frame format — must mirror include/health.h
#   $HLTH,<t_ms>,<fsm>,<arm_counts>,NAME=st:flags:consec:fails:age:dHz,...*XX
# ----------------------------------------------------------------------

STATE_NAMES = {0: "INOP", 1: "OK", 2: "CAUTION", 3: "FAIL", 4: "NO DATA"}

# Short enough to sit in the annunciator's right-hand column without
# colliding with the age readout. Ordered most-severe first.
FLAG_BITS = [
    (0x10, "INIT FAIL"),
    (0x08, "NO SAMPLES"),
    (0x02, "RANGE"),
    (0x04, "UNSTABLE"),
    (0x01, "STALE"),
    (0x20, "RECOV"),
]
MAX_FLAGS_SHOWN = 2

# Why a channel is dark, when the firmware reports it as not fitted.
INOP_REASON = {
    "MAG":   "not fitted",
    "FLASH": "GD25Q128 miswired",
    "PYRO1": "no continuity telemetry",
    "PYRO2": "no continuity telemetry",
    "BATT":  "no firmware use yet",
}

ARM_THRESHOLD_COUNTS = 1614   # ARM_SENSE_THRESHOLD in flight_sm.h

# ----------------------------------------------------------------------
# Panel palette — cool instrument slate, not near-black, so the lit lamps
# are the brightest thing on screen and read as illuminated rather than
# merely coloured.
# ----------------------------------------------------------------------

BEZEL      = "#171c23"
PANEL      = "#232b35"
WELL       = "#1b222b"
RULE       = "#39434f"
TEXT       = "#d8dee6"
TEXT_DIM   = "#7d8998"

LAMP_GREEN = "#4cae7a"   # normal
LAMP_AMBER = "#d9a441"   # caution
LAMP_RED   = "#d2544a"   # warning
LAMP_WHITE = "#d8dee6"   # advisory
LAMP_GREY  = "#4a5563"   # inop

STATE_LAMP = {0: LAMP_GREY, 1: LAMP_GREEN, 2: LAMP_AMBER, 3: LAMP_RED, 4: LAMP_WHITE}


def mix(a, b, t):
    """Blend two #rrggbb colours; t=0 gives a, t=1 gives b."""
    av = [int(a[i:i + 2], 16) for i in (1, 3, 5)]
    bv = [int(b[i:i + 2], 16) for i in (1, 3, 5)]
    return "#" + "".join(f"{int(x + (y - x) * t):02x}" for x, y in zip(av, bv))


def pick_font(candidates, default):
    have = set(tkfont.families())
    for name in candidates:
        if name in have:
            return name
    return default


# ----------------------------------------------------------------------
# Parsing
# ----------------------------------------------------------------------


class Channel:
    __slots__ = ("name", "state", "flags", "consec", "fails", "age_ms", "dhz")

    def __init__(self, name, state, flags, consec, fails, age_ms, dhz):
        self.name = name
        self.state = state
        self.flags = flags
        self.consec = consec
        self.fails = fails
        self.age_ms = age_ms
        self.dhz = dhz

    @property
    def rate_hz(self):
        return self.dhz / 10.0

    def flag_text(self):
        words = [w for bit, w in FLAG_BITS if self.flags & bit]
        if len(words) > MAX_FLAGS_SHOWN:
            extra = len(words) - MAX_FLAGS_SHOWN
            words = words[:MAX_FLAGS_SHOWN] + [f"+{extra}"]
        return " ".join(words)


class Frame:
    def __init__(self):
        self.t_ms = 0
        self.fsm = "?"
        self.arm_counts = 0
        self.channels = []


def parse_frame(line):
    """Return a Frame, or None if the line is not a valid health frame."""
    line = line.strip()
    if not line.startswith("$") or "*" not in line:
        return None

    body, _, cks_txt = line[1:].rpartition("*")
    ck = 0
    for ch in body:
        ck ^= ord(ch)
    try:
        if ck != int(cks_txt[:2], 16):
            return None
    except ValueError:
        return None

    f = body.split(",")
    if len(f) < 4 or f[0] != "HLTH":
        return None

    fr = Frame()
    try:
        fr.t_ms = int(f[1])
        fr.fsm = f[2]
        fr.arm_counts = int(f[3])
    except ValueError:
        return None

    for tok in f[4:]:
        name, _, rest = tok.partition("=")
        parts = rest.split(":")
        if len(parts) != 6:
            continue
        try:
            fr.channels.append(Channel(name, *(int(p) for p in parts)))
        except ValueError:
            continue

    return fr if fr.channels else None


# ----------------------------------------------------------------------
# Serial I/O — runs off the UI thread, hands lines back through a queue
# ----------------------------------------------------------------------


class SerialLink:
    def __init__(self, out_queue):
        self.q = out_queue
        self.ser = None
        self._stop = threading.Event()
        self._thread = None

    @property
    def connected(self):
        return self.ser is not None

    @staticmethod
    def list_ports():
        try:
            from serial.tools import list_ports
        except ImportError:
            return []
        found = []
        for p in list_ports.comports():
            label = p.device
            if p.description and p.description != "n/a":
                label += f"  ({p.description})"
            found.append((p.device, label))
        return found

    @staticmethod
    def guess_port():
        try:
            from serial.tools import list_ports
        except ImportError:
            return None
        for p in list_ports.comports():
            if p.vid == 0x16C0:                      # PJRC
                return p.device
            blurb = f"{p.description} {p.manufacturer or ''}".lower()
            if "teensy" in blurb:
                return p.device
        ports = list(list_ports.comports())
        return ports[0].device if ports else None

    def connect(self, port, baud):
        try:
            import serial
        except ImportError:
            self.q.put(("error", "pyserial is not installed. Run: pip install pyserial"))
            return False
        try:
            self.ser = serial.Serial(port, baud, timeout=0.2)
        except Exception as exc:
            self.q.put(("error", f"Could not open {port}: {exc}"))
            self.ser = None
            return False

        self._stop.clear()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        self.q.put(("info", f"Connected to {port} at {baud} baud."))
        return True

    def disconnect(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.q.put(("info", "Disconnected."))

    def send(self, text):
        if not self.ser:
            self.q.put(("error", "Not connected — nothing was sent."))
            return
        try:
            self.ser.write(text.encode("ascii"))
            self.q.put(("sent", text))
        except Exception as exc:
            self.q.put(("error", f"Write failed: {exc}"))

    def _read_loop(self):
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(512)
            except Exception as exc:
                self.q.put(("error", f"Serial read failed: {exc}"))
                self.q.put(("dropped", ""))
                return
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, _, buf = buf.partition(b"\n")
                self.q.put(("line", raw.decode("utf-8", "replace").rstrip("\r")))


class DemoLink:
    """Synthesises frames so the panel can be driven without hardware."""

    NAMES = ["IMU", "BARO", "MAG", "FLASH", "SD", "ARM", "PYRO1", "PYRO2", "BATT"]
    FITTED = {"IMU", "BARO", "SD", "ARM"}

    def __init__(self, out_queue):
        self.q = out_queue
        self._stop = threading.Event()
        self.ser = True
        threading.Thread(target=self._run, daemon=True).start()
        self.q.put(("info", "Demo mode — data is synthesised, no hardware attached."))

    connected = True

    @staticmethod
    def list_ports():
        return [("demo", "demo  (synthesised)")]

    def connect(self, *a):
        return True

    def disconnect(self):
        self._stop.set()

    def send(self, text):
        self.q.put(("sent", text))
        self.q.put(("line", f"  [demo] received command '{text}'"))

    def _run(self):
        t0 = time.time()
        while not self._stop.is_set():
            t = int((time.time() - t0) * 1000)
            parts = [f"HLTH,{t},IDLE,3340"]
            for n in self.NAMES:
                if n not in self.FITTED:
                    parts.append(f"{n}=0:0:0:0:0:0")
                    continue
                st, flags, age = 1, 0, 0
                dhz = {"IMU": 1250, "BARO": 742, "SD": 50, "ARM": 1250}[n]
                if n == "BARO":
                    if 8 < (t / 1000) % 24 < 13:      # transient caution
                        st, flags, age = 2, 0x01, 640
                elif n == "SD":
                    if (t / 1000) % 24 > 18:          # sustained failure
                        st, flags, age = 3, 0x21, 12000
                parts.append(f"{n}={st}:{flags}:0:{random.randint(0, 3)}:{age}:{dhz}")
            body = ",".join(parts)
            ck = 0
            for c in body:
                ck ^= ord(c)
            self.q.put(("line", f"${body}*{ck:02X}"))
            time.sleep(0.5)


# ----------------------------------------------------------------------
# Annunciator row
# ----------------------------------------------------------------------


class Annunciator(tk.Canvas):
    """One channel: a lamp, a name, a status word, and its live counters."""

    W, H = 452, 34

    def __init__(self, parent, name, fonts):
        super().__init__(parent, width=self.W, height=self.H, bg=WELL,
                         highlightthickness=0, bd=0)
        self.name = name
        self.fonts = fonts
        self.forced = None          # lamp test override
        self.ch = None
        self.draw()

    def set(self, ch):
        self.ch = ch
        self.draw()

    def force(self, state):
        self.forced = state
        self.draw()

    def draw(self):
        self.delete("all")
        ch = self.ch
        state = self.forced if self.forced is not None else (ch.state if ch else 4)
        lamp = STATE_LAMP.get(state, LAMP_WHITE)
        lit = state != 0

        # Lamp with a soft bloom — three concentric ovals blended toward the
        # well colour. Tk has no alpha, so the falloff is drawn explicitly.
        cx, cy = 20, self.H // 2
        if lit:
            for r, t in ((11, 0.85), (8, 0.55), (6, 0.0)):
                self.create_oval(cx - r, cy - r, cx + r, cy + r,
                                 fill=mix(lamp, WELL, t), outline="")
        else:
            self.create_oval(cx - 6, cy - 6, cx + 6, cy + 6,
                             fill=mix(LAMP_GREY, WELL, 0.45),
                             outline=mix(LAMP_GREY, WELL, 0.1))

        name_col = TEXT if lit else TEXT_DIM
        self.create_text(40, cy, text=self.name, anchor="w",
                         fill=name_col, font=self.fonts["label"])

        self.create_text(112, cy, text=STATE_NAMES.get(state, "?"), anchor="w",
                         fill=lamp if lit else TEXT_DIM, font=self.fonts["label"])

        if ch is None or state == 0:
            detail = INOP_REASON.get(self.name, "") if ch else "awaiting data"
            self.create_text(200, cy, text=detail, anchor="w",
                             fill=TEXT_DIM, font=self.fonts["small"])
            return

        rate = f"{ch.rate_hz:6.1f} Hz" if ch.dhz else "     — "
        self.create_text(194, cy, text=rate, anchor="w",
                         fill=TEXT, font=self.fonts["mono"])
        self.create_text(268, cy, text=f"{ch.age_ms:>6} ms", anchor="w",
                         fill=TEXT_DIM, font=self.fonts["mono"])

        flags = ch.flag_text()
        if flags:
            self.create_text(self.W - 10, cy, text=flags, anchor="e",
                             fill=lamp, font=self.fonts["small"])
        elif ch.fails:
            self.create_text(self.W - 10, cy, text=f"{ch.fails} invalid", anchor="e",
                             fill=TEXT_DIM, font=self.fonts["small"])


# ----------------------------------------------------------------------
# Main window
# ----------------------------------------------------------------------


class BenchPanel:
    ORDER = ["IMU", "BARO", "MAG", "FLASH", "SD", "ARM", "PYRO1", "PYRO2", "BATT"]

    BENCH_CMDS = [
        ("1  Baro", "1"), ("2  IMU", "2"), ("3  Mag", "3"), ("4  Flash", "4"),
        ("5  SD", "5"), ("6  Logger", "6"), ("7  Run all", "7"),
        ("S  Servos", "S"), ("L  LEDs", "L"), ("B  Buzzer", "B"),
        ("M  Monitor", "M"), ("R  Menu", "R"),
    ]

    FLIGHT_CMDS = [
        ("H  Status page", "H"), ("G  Calibrate gyro", "G"),
        ("D  Disarm", "D"), ("X  Abort", "X"), ("R  Dump log", "R"),
    ]

    def __init__(self, root, args):
        self.root = root
        self.q = queue.Queue()
        self.link = DemoLink(self.q) if args.demo else SerialLink(self.q)
        self.demo = args.demo
        self.frame = None
        self.last_rx = 0.0
        self.frames = 0
        self.rejected = 0
        self.lamp_test_until = 0.0

        root.title("TVC Flight Computer — Bench Panel")
        root.configure(bg=BEZEL)
        root.geometry("1150x750")
        root.minsize(940, 700)

        mono_name = pick_font(
            ["JetBrains Mono", "Cascadia Mono", "SF Mono", "Menlo", "Consolas",
             "DejaVu Sans Mono", "Liberation Mono"], "Courier")
        ui_name = pick_font(
            ["Inter", "Segoe UI", "SF Pro Text", "Helvetica Neue", "Cantarell",
             "DejaVu Sans"], "Helvetica")

        self.fonts = {
            "title":  tkfont.Font(family=ui_name, size=13, weight="bold"),
            "label":  tkfont.Font(family=mono_name, size=10, weight="bold"),
            "mono":   tkfont.Font(family=mono_name, size=10),
            "small":  tkfont.Font(family=ui_name, size=9),
            "eyebrow": tkfont.Font(family=ui_name, size=8, weight="bold"),
            "master": tkfont.Font(family=mono_name, size=22, weight="bold"),
            "reading": tkfont.Font(family=mono_name, size=16, weight="bold"),
            "console": tkfont.Font(family=mono_name, size=9),
        }

        self._build_header()
        self._build_body()
        self._build_commands()

        if not args.demo:
            want = args.port or SerialLink.guess_port()
            if want:
                self.port_var.set(want)
                self.root.after(300, self.toggle_connection)

        self.root.after(50, self._pump)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # -- eyebrow label used above each panel region ---------------------
    def _eyebrow(self, parent, text):
        tk.Label(parent, text=text.upper(), bg=BEZEL, fg=TEXT_DIM,
                 font=self.fonts["eyebrow"]).pack(anchor="w", pady=(0, 4))

    def _build_header(self):
        bar = tk.Frame(self.root, bg=BEZEL)
        bar.pack(fill="x", padx=14, pady=(12, 6))

        tk.Label(bar, text="TVC FLIGHT COMPUTER", bg=BEZEL, fg=TEXT,
                 font=self.fonts["title"]).pack(side="left")
        tk.Label(bar, text="bench panel", bg=BEZEL, fg=TEXT_DIM,
                 font=self.fonts["small"]).pack(side="left", padx=(8, 0))

        right = tk.Frame(bar, bg=BEZEL)
        right.pack(side="right")

        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(right, textvariable=self.port_var, width=26,
                                     state="readonly" if not self.demo else "disabled")
        self.port_box.pack(side="left", padx=(0, 6))
        self._refresh_ports()

        ttk.Button(right, text="Refresh", width=8,
                   command=self._refresh_ports).pack(side="left", padx=(0, 6))
        self.conn_btn = ttk.Button(right, text="Connect", width=11,
                                   command=self.toggle_connection)
        self.conn_btn.pack(side="left")
        if self.demo:
            self.conn_btn.state(["disabled"])

    def _build_body(self):
        body = tk.Frame(self.root, bg=BEZEL)
        body.pack(fill="both", expand=True, padx=14, pady=(0, 6))

        # ---------------- left: annunciator stack ----------------
        left = tk.Frame(body, bg=BEZEL)
        left.pack(side="left", fill="y")

        # Master annunciator — the one lamp that matters before you commit.
        self._eyebrow(left, "flight-critical")
        self.master = tk.Canvas(left, width=452, height=76, bg=WELL,
                                highlightthickness=1, highlightbackground=RULE)
        self.master.pack()

        tk.Frame(left, bg=BEZEL, height=10).pack()
        self._eyebrow(left, "channels")

        stack = tk.Frame(left, bg=RULE)
        stack.pack()
        self.rows = {}
        for i, name in enumerate(self.ORDER):
            row = Annunciator(stack, name, self.fonts)
            row.pack(pady=(0 if i == 0 else 1, 0))
            self.rows[name] = row

        tk.Frame(left, bg=BEZEL, height=10).pack()
        self._eyebrow(left, "arm sense")
        self.arm = tk.Canvas(left, width=452, height=58, bg=WELL,
                             highlightthickness=1, highlightbackground=RULE)
        self.arm.pack()

        # ---------------- right: serial console ----------------
        right = tk.Frame(body, bg=BEZEL)
        right.pack(side="left", fill="both", expand=True, padx=(14, 0))

        head = tk.Frame(right, bg=BEZEL)
        head.pack(fill="x")
        tk.Label(head, text="SERIAL", bg=BEZEL, fg=TEXT_DIM,
                 font=self.fonts["eyebrow"]).pack(side="left", pady=(0, 4))
        self.autoscroll = tk.BooleanVar(value=True)
        ttk.Checkbutton(head, text="Follow", variable=self.autoscroll).pack(side="right")
        ttk.Button(head, text="Save…", width=7,
                   command=self._save_log).pack(side="right", padx=4)
        ttk.Button(head, text="Clear", width=7,
                   command=self._clear_console).pack(side="right")

        wrap = tk.Frame(right, bg=RULE, padx=1, pady=1)
        wrap.pack(fill="both", expand=True)
        self.console = tk.Text(wrap, bg=WELL, fg=TEXT, insertbackground=TEXT,
                               font=self.fonts["console"], wrap="none",
                               relief="flat", padx=10, pady=8, height=10)
        sb = ttk.Scrollbar(wrap, command=self.console.yview)
        self.console.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.console.pack(side="left", fill="both", expand=True)

        self.console.tag_configure("pass", foreground=LAMP_GREEN)
        self.console.tag_configure("fail", foreground=LAMP_RED)
        self.console.tag_configure("warn", foreground=LAMP_AMBER)
        self.console.tag_configure("meta", foreground=TEXT_DIM)
        self.console.tag_configure("sent", foreground="#6ba3d6")

        entry = tk.Frame(right, bg=BEZEL)
        entry.pack(fill="x", pady=(6, 0))
        self.entry_var = tk.StringVar()
        e = ttk.Entry(entry, textvariable=self.entry_var)
        e.pack(side="left", fill="x", expand=True)
        e.bind("<Return>", self._send_entry)
        ttk.Button(entry, text="Send", width=7,
                   command=self._send_entry).pack(side="left", padx=(6, 0))

    def _build_commands(self):
        foot = tk.Frame(self.root, bg=BEZEL)
        foot.pack(fill="x", padx=14, pady=(0, 12))

        bench = tk.LabelFrame(foot, text=" pio run -e bench ", bg=BEZEL, fg=TEXT_DIM,
                              font=self.fonts["small"], bd=1,
                              relief="solid", labelanchor="nw")
        bench.pack(side="left")
        grid = tk.Frame(bench, bg=BEZEL)
        grid.pack(padx=8, pady=8)
        for i, (label, cmd) in enumerate(self.BENCH_CMDS):
            ttk.Button(grid, text=label, width=11,
                       command=lambda c=cmd: self.link.send(c)
                       ).grid(row=i // 4, column=i % 4, padx=2, pady=2)

        flight = tk.LabelFrame(foot, text=" pio run -e flight ", bg=BEZEL, fg=TEXT_DIM,
                               font=self.fonts["small"], bd=1,
                               relief="solid", labelanchor="nw")
        flight.pack(side="left", padx=(12, 0))
        fgrid = tk.Frame(flight, bg=BEZEL)
        fgrid.pack(padx=8, pady=8)
        for i, (label, cmd) in enumerate(self.FLIGHT_CMDS):
            ttk.Button(fgrid, text=label, width=14,
                       command=lambda c=cmd: self.link.send(c)
                       ).grid(row=i // 2, column=i % 2, padx=2, pady=2)

        util = tk.Frame(foot, bg=BEZEL)
        util.pack(side="left", padx=(12, 0))
        ttk.Button(util, text="Lamp test", width=12,
                   command=self._lamp_test).pack(pady=2)
        self.link_lbl = tk.Label(util, text="no link", bg=BEZEL, fg=TEXT_DIM,
                                 font=self.fonts["small"])
        self.link_lbl.pack(pady=(6, 0))

    # ------------------------------------------------------------------
    # Actions
    # ------------------------------------------------------------------

    def _refresh_ports(self):
        ports = self.link.list_ports()
        self.port_box["values"] = [lbl for _, lbl in ports]
        self._port_map = {lbl: dev for dev, lbl in ports}
        if ports and not self.port_var.get():
            self.port_var.set(ports[0][1])

    def _selected_port(self):
        label = self.port_var.get()
        return getattr(self, "_port_map", {}).get(label, label)

    def toggle_connection(self):
        if self.link.connected:
            self.link.disconnect()
            self.conn_btn.configure(text="Connect")
        else:
            port = self._selected_port()
            if not port:
                self._log("No serial port selected. Plug in the Teensy, then "
                          "press Refresh.", "warn")
                return
            if self.link.connect(port, 115200):
                self.conn_btn.configure(text="Disconnect")

    def _lamp_test(self):
        """Illuminate every lamp so a dead indicator can't hide a live fault."""
        self.lamp_test_until = time.time() + 1.6
        for i, name in enumerate(self.ORDER):
            self.rows[name].force([1, 2, 3, 4][i % 4])
        self._draw_master(forced=True)
        self._draw_arm()   # keep the live reading visible during the test

    def _send_entry(self, _evt=None):
        text = self.entry_var.get()
        if text:
            self.link.send(text)
            self.entry_var.set("")

    def _clear_console(self):
        self.console.delete("1.0", "end")

    def _save_log(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".txt",
            initialfile=time.strftime("bench_%Y%m%d_%H%M%S.txt"),
            filetypes=[("Text", "*.txt"), ("All files", "*.*")])
        if not path:
            return
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(self.console.get("1.0", "end"))
        self._log(f"Saved console to {path}", "meta")

    def _log(self, text, tag=None):
        self.console.insert("end", text + "\n", tag or ())
        if int(self.console.index("end-1c").split(".")[0]) > 4000:
            self.console.delete("1.0", "500.0")
        if self.autoscroll.get():
            self.console.see("end")

    # ------------------------------------------------------------------
    # Event pump
    # ------------------------------------------------------------------

    def _pump(self):
        try:
            while True:
                kind, payload = self.q.get_nowait()
                if kind == "line":
                    self._on_line(payload)
                elif kind == "sent":
                    self._log(f"> {payload}", "sent")
                elif kind == "info":
                    self._log(payload, "meta")
                elif kind == "error":
                    self._log(payload, "fail")
                elif kind == "dropped":
                    self.conn_btn.configure(text="Connect")
                    self.link.ser = None
        except queue.Empty:
            pass

        if self.lamp_test_until and time.time() > self.lamp_test_until:
            self.lamp_test_until = 0.0
            for row in self.rows.values():
                row.force(None)

        self._redraw()
        self.root.after(60, self._pump)

    def _on_line(self, line):
        if line.startswith("$"):
            fr = parse_frame(line)
            if fr:
                self.frames += 1
                self.frame = fr
                self.last_rx = time.time()
                self._apply(fr)
            else:
                self.rejected += 1
            return

        tag = None
        if "[PASS]" in line:
            tag = "pass"
        elif "[FAIL]" in line or "[FAULT]" in line:
            tag = "fail"
        elif "[WARN]" in line:
            tag = "warn"
        self._log(line, tag)

    def _apply(self, fr):
        seen = set()
        for ch in fr.channels:
            if ch.name in self.rows:
                self.rows[ch.name].set(ch)
                seen.add(ch.name)
        for name in self.ORDER:
            if name not in seen:
                self.rows[name].set(None)

    # ------------------------------------------------------------------
    # Drawing
    # ------------------------------------------------------------------

    def _redraw(self):
        if self.lamp_test_until:
            return
        self._draw_master()
        self._draw_arm()

        if not self.link.connected:
            self.link_lbl.configure(text="no link", fg=TEXT_DIM)
        else:
            age = time.time() - self.last_rx if self.last_rx else 999
            if age > 3:
                self.link_lbl.configure(text="no frames", fg=LAMP_AMBER)
            else:
                self.link_lbl.configure(
                    text=f"{self.frames} frames", fg=TEXT_DIM)

    def _draw_master(self, forced=False):
        c = self.master
        c.delete("all")
        w, h = 452, 76

        fr = self.frame
        if forced:
            verdict, colour, reason = "GO", LAMP_GREEN, "lamp test"
        elif fr is None:
            verdict, colour, reason = "NO DATA", LAMP_WHITE, "waiting for the flight computer"
        else:
            bad = []
            for ch in fr.channels:
                if ch.name == "IMU" and ch.state != 1:
                    bad.append(f"IMU {STATE_NAMES.get(ch.state, '?').lower()}")
                if ch.name == "BARO" and ch.state in (3, 4):
                    bad.append(f"baro {STATE_NAMES.get(ch.state, '?').lower()}")
            if bad:
                verdict, colour, reason = "NO-GO", LAMP_RED, "; ".join(bad)
            else:
                verdict, colour, reason = "GO", LAMP_GREEN, "IMU and baro nominal"

        c.create_rectangle(0, 0, w, h, fill=mix(colour, WELL, 0.90), outline="")
        c.create_rectangle(0, 0, 5, h, fill=colour, outline="")
        c.create_text(24, 30, text=verdict, anchor="w", fill=colour,
                      font=self.fonts["master"])
        c.create_text(26, 56, text=reason, anchor="w", fill=TEXT_DIM,
                      font=self.fonts["small"])

        if fr:
            up = fr.t_ms
            clock = f"T+{up // 60000:02d}:{(up // 1000) % 60:02d}.{up % 1000:03d}"
            c.create_text(w - 18, 26, text=fr.fsm, anchor="e", fill=TEXT,
                          font=self.fonts["label"])
            c.create_text(w - 18, 48, text=clock, anchor="e", fill=TEXT_DIM,
                          font=self.fonts["mono"])

    def _draw_arm(self):
        c = self.arm
        c.delete("all")
        w, h = 452, 58
        counts = self.frame.arm_counts if self.frame else 0
        volts = counts * 3.3 / 4095.0
        armed = counts >= ARM_THRESHOLD_COUNTS
        colour = LAMP_AMBER if armed else LAMP_GREY

        c.create_text(16, 20, text=f"{volts:.2f} V", anchor="w",
                      fill=TEXT if self.frame else TEXT_DIM,
                      font=self.fonts["reading"])
        c.create_text(104, 22, text=f"{counts} counts", anchor="w",
                      fill=TEXT_DIM, font=self.fonts["mono"])
        c.create_text(16, 43, text="ARMED" if armed else "SAFE", anchor="w",
                      fill=colour, font=self.fonts["label"])

        # Threshold bar: where the reading sits against ARM_SENSE_THRESHOLD.
        x0, x1, y = 200, w - 18, 30
        c.create_rectangle(x0, y - 5, x1, y + 5, fill=mix(RULE, WELL, 0.4), outline="")
        frac = min(max(counts / 4095.0, 0.0), 1.0)
        if frac > 0:
            c.create_rectangle(x0, y - 5, x0 + (x1 - x0) * frac, y + 5,
                               fill=colour, outline="")
        tx = x0 + (x1 - x0) * (ARM_THRESHOLD_COUNTS / 4095.0)
        c.create_line(tx, y - 10, tx, y + 10, fill=TEXT, width=2)
        c.create_text(tx, y + 20, text="arm threshold", fill=TEXT_DIM,
                      font=self.fonts["small"])

    def _on_close(self):
        try:
            self.link.disconnect()
        except Exception:
            pass
        self.root.destroy()


def main():
    ap = argparse.ArgumentParser(description="TVC bench annunciator panel")
    ap.add_argument("-p", "--port", help="serial port to open on start")
    ap.add_argument("--demo", action="store_true",
                    help="synthesise data so the panel can be driven with no hardware")
    args = ap.parse_args()

    root = tk.Tk()
    try:
        style = ttk.Style(root)
        style.theme_use("clam")
        style.configure("TButton", background=PANEL, foreground=TEXT,
                        borderwidth=0, focusthickness=0, padding=5)
        style.map("TButton",
                  background=[("active", mix(PANEL, TEXT, 0.18)),
                              ("disabled", WELL)],
                  foreground=[("disabled", TEXT_DIM)])
        style.configure("TCheckbutton", background=BEZEL, foreground=TEXT_DIM,
                        indicatorbackground=WELL, indicatorforeground=TEXT,
                        focuscolor=BEZEL)
        style.map("TCheckbutton",
                  background=[("active", BEZEL)],
                  indicatorbackground=[("selected", LAMP_GREEN),
                                       ("!selected", WELL)])
        style.configure("TEntry", fieldbackground=WELL, foreground=TEXT,
                        insertcolor=TEXT, borderwidth=0)
        style.configure("TCombobox", fieldbackground=WELL, background=PANEL,
                        foreground=TEXT, arrowcolor=TEXT, borderwidth=0,
                        selectbackground=WELL, selectforeground=TEXT)
        style.map("TCombobox",
                  fieldbackground=[("readonly", WELL), ("disabled", WELL)],
                  foreground=[("readonly", TEXT), ("disabled", TEXT_DIM)],
                  selectbackground=[("readonly", WELL)],
                  selectforeground=[("readonly", TEXT)])
        style.configure("TScrollbar", background=PANEL, troughcolor=WELL,
                        borderwidth=0, arrowcolor=TEXT_DIM)
    except tk.TclError:
        pass

    BenchPanel(root, args)
    root.mainloop()


if __name__ == "__main__":
    main()