#!/usr/bin/env python3
"""
flight_live.py - live ground-station display for the TVC flight computer.

Shows the same things as flight_plot.m - an estimated 3D flight path, the
PID inputs with the P, I and D terms, altitude, vertical speed,
acceleration, pressure and temperature - updated as data arrives.

  python tools/flight_live.py --port COM7                 live $TLM frames
  python tools/flight_live.py --port COM7 --baud 57600    e.g. through a radio modem
  python tools/flight_live.py --replay FLIGHT_004.CSV     play a log back in real time
  python tools/flight_live.py --sim                       simulated flight, no hardware
  python tools/flight_live.py --list-ports
  python tools/flight_live.py --write-sim SIM.CSV         save a simulated flight log
                                                          (to try flight_plot.m)

Wire format: the $TLM frames in include/telemetry.h - one checksummed
ASCII line per frame. Today they come over USB; a radio link that
delivers the same lines works as-is, so only --port and --baud change.
Frames with a bad checksum are counted and dropped. Lines starting with
"[" (the flight computer's console messages, e.g. "[FSM] >> LIFTOFF
CONFIRMED!") are shown in the status bar. Rows in the SD-log CSV format
are accepted too, after their header line.

Everything received from --port is recorded to telemetry_logs/ in the
SD-log column layout, so flight_plot.m plots a telemetry recording the
same way as an SD-card log.

Keys (with the plot window focused):
  w      whole flight / rolling window
  x      clear the display (e.g. before a new flight)
  space  pause / resume the display (data keeps recording)
Drag the 3D plot to rotate it.

The 3D path is an estimate: the board has no GPS. See flight_plot.m.
"""

import argparse
import csv
import datetime
import math
import os
import queue
import re
import sys
import threading
import time
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

try:
    import serial
    import serial.tools.list_ports
except ImportError:          # only needed for --port / --list-ports
    serial = None

# ----------------------------------------------------------------------
# Firmware definitions
# ----------------------------------------------------------------------

STATE_NAMES = ("IDLE", "ARMED", "POWERED", "COAST", "APOGEE",
               "DESCENT", "MAIN", "LANDED", "ABORT")
STATE_COLORS = ("#999999", "#666666", "#d9541a", "#0072bd", "#edb120",
                "#7e2f8e", "#77ac30", "#404040", "#e60000")
IDLE, ARMED, POWERED, COAST, APOGEE, DESCENT, MAIN, LANDED, ABORT = range(9)
EVENT_STATES = {POWERED: "Liftoff", COAST: "Burnout", APOGEE: "Apogee",
                MAIN: "Main chute", ABORT: "ABORT"}

# $TLM field order - must mirror include/telemetry.h
TLM_FIELDS = (
    "t_ms", "state", "flags",
    "q0", "q1", "q2", "q3",
    "tip_a", "tip_b", "spin",
    "gx", "gy", "gz",
    "ax", "ay", "az",
    "mx", "my", "mz",
    "alt_m", "vel_ms", "baro_alt_m", "press_hpa", "temp_c",
    "pid_p", "pid_y", "servo_p_us", "servo_y_us",
    "pack_v", "loop_us",
)
F_IMU_VALID = 0x01
F_BARO_OK   = 0x02
F_TVC_LIVE  = 0x04

# SD-log CSV columns (CSV_HEADER in src/logger.cpp)
LOG_COLUMNS = (
    "timestamp_ms", "roll", "pitch", "yaw", "q0", "q1", "q2", "q3",
    "gx", "gy", "gz", "ax", "ay", "az", "mx", "my", "mz",
    "temperature_c", "pressure_hpa", "altitude_m", "velocity_ms",
    "servo_pitch_us", "servo_yaw_us", "pid_pitch_out", "pid_yaw_out",
    "flight_state", "imu_valid", "baro_valid", "mag_valid",
)
# Recordings add these after the log columns
EXTRA_COLUMNS = ("flags", "baro_alt_m", "pack_v", "loop_us", "host_time_s")

# Internal sample keys, and where each comes from in a $TLM frame and in a log row.
# The log's "pitch" column is tip_a and "roll" is tip_b (main_control_loop.cpp).
SAMPLE_KEYS = ("t_ms", "state", "flags", "q0", "q1", "q2", "q3",
               "tip_a", "tip_b", "spin", "gx", "gy", "gz", "ax", "ay", "az",
               "mx", "my", "mz", "alt", "vel", "baro_alt", "press", "temp",
               "pid_p", "pid_y", "servo_p", "servo_y", "pack_v", "loop_us")
FROM_TLM = {"alt_m": "alt", "vel_ms": "vel", "baro_alt_m": "baro_alt",
            "press_hpa": "press", "temp_c": "temp",
            "servo_p_us": "servo_p", "servo_y_us": "servo_y"}
FROM_LOG = {"timestamp_ms": "t_ms", "roll": "tip_b", "pitch": "tip_a", "yaw": "spin",
            "temperature_c": "temp", "pressure_hpa": "press",
            "altitude_m": "alt", "velocity_ms": "vel",
            "servo_pitch_us": "servo_p", "servo_yaw_us": "servo_y",
            "pid_pitch_out": "pid_p", "pid_yaw_out": "pid_y",
            "flight_state": "state", "baro_alt_m": "baro_alt"}

LINK_STALE_S = 1.0      # no frame for this long = link lost
START_RETRY_S = 2.0     # resend the start command after this long without frames


def read_firmware_config():
    """PID gains and limits from include/pid.h and servo_driver.h."""
    cfg = dict(kp=0.8, ki=0.215, kd=0.10, dalpha=0.5, ilim=8.5, lim_p=7.0, lim_y=10.0,
               src="built-in defaults")
    inc = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, "include")

    def grab(text, name, fallback):
        m = re.search(r"#define\s+" + name + r"\s+([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", text)
        return float(m.group(1)) if m else fallback

    try:
        with open(os.path.join(inc, "pid.h"), encoding="utf-8", errors="replace") as fh:
            t = fh.read()
        cfg.update(kp=grab(t, "PID_DEFAULT_KP", cfg["kp"]), ki=grab(t, "PID_DEFAULT_KI", cfg["ki"]),
                   kd=grab(t, "PID_DEFAULT_KD", cfg["kd"]), dalpha=grab(t, "PID_D_FILTER_ALPHA", cfg["dalpha"]),
                   ilim=grab(t, "PID_INTEGRAL_LIMIT", cfg["ilim"]), src="include/pid.h")
    except OSError:
        pass
    try:
        with open(os.path.join(inc, "servo_driver.h"), encoding="utf-8", errors="replace") as fh:
            t = fh.read()
        cfg.update(lim_p=grab(t, "SERVO_PITCH_MAX_ANGLE_DEG", cfg["lim_p"]),
                   lim_y=grab(t, "SERVO_YAW_MAX_ANGLE_DEG", cfg["lim_y"]))
    except OSError:
        pass
    return cfg


# ----------------------------------------------------------------------
# Parsing
# ----------------------------------------------------------------------

def _xor(text):
    ck = 0
    for ch in text:
        ck ^= ord(ch)
    return ck


def _num(text):
    try:
        return float(text)
    except ValueError:
        return math.nan


def _state_index(text):
    text = text.strip()
    if text in STATE_NAMES:
        return float(STATE_NAMES.index(text))
    return _num(text)


def empty_sample():
    return dict.fromkeys(SAMPLE_KEYS, math.nan)


def parse_tlm(line):
    """A $TLM line -> sample dict, or None if it isn't a valid frame."""
    line = line.strip()
    if not line.startswith("$TLM,"):
        return None
    body, star, ck = line[1:].rpartition("*")
    if not star:
        return None
    try:
        if _xor(body) != int(ck[:2], 16):
            return None
    except ValueError:
        return None
    f = body.split(",")[1:]
    if len(f) == len(TLM_FIELDS) - 3:              # firmware from before the mag fields
        at = TLM_FIELDS.index("mx")
        f = f[:at] + ["nan"] * 3 + f[at:]
    if len(f) != len(TLM_FIELDS):
        return None
    s = empty_sample()
    for name, text in zip(TLM_FIELDS, f):
        s[FROM_TLM.get(name, name)] = _state_index(text) if name == "state" else _num(text)
    return s if math.isfinite(s["t_ms"]) else None


def parse_log_row(header, values):
    """One SD-log / recording CSV row -> sample dict, or None."""
    if len(values) < len(header):
        return None
    s = empty_sample()
    for name, text in zip(header, values):
        key = FROM_LOG.get(name, name)
        if key in s:
            s[key] = _state_index(text) if key == "state" else _num(text)
    return s if math.isfinite(s["t_ms"]) else None


# ----------------------------------------------------------------------
# Sources - each runs in its own thread and puts samples on a queue
# ----------------------------------------------------------------------

class LinkStats:
    def __init__(self):
        self.lock = threading.Lock()
        self.ok = 0
        self.bad = 0
        self.last_rx = None
        self.arrivals = deque(maxlen=400)
        self.link = "starting"
        self.error = ""
        self.fc_message = ""
        self.finished = False

    def frame(self, good):
        now = time.monotonic()
        with self.lock:
            if good:
                self.ok += 1
                self.last_rx = now
                self.arrivals.append(now)
            else:
                self.bad += 1

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            recent = [t for t in self.arrivals if now - t < 2.0]
            return dict(ok=self.ok, bad=self.bad, link=self.link, error=self.error,
                        fc=self.fc_message, finished=self.finished, rate=len(recent) / 2.0,
                        age=None if self.last_rx is None else now - self.last_rx)

    def set(self, **kw):
        with self.lock:
            for k, v in kw.items():
                setattr(self, k, v)


class Source(threading.Thread):
    def __init__(self, out, stats):
        super().__init__(daemon=True)
        self.out = out
        self.stats = stats
        self.stopping = threading.Event()

    def stop(self):
        self.stopping.set()


class SerialSource(Source):
    """$TLM frames (or SD-log CSV rows) from a serial port; reconnects on its own."""

    def __init__(self, out, stats, port, baud, start_cmd):
        super().__init__(out, stats)
        self.port, self.baud, self.start_cmd = port, baud, start_cmd
        self.csv_header = None

    def run(self):
        while not self.stopping.is_set():
            try:
                with serial.Serial(self.port, self.baud, timeout=0.2) as ser:
                    self.stats.set(link=f"{self.port} @ {self.baud}", error="")
                    self._read(ser)
            except (serial.SerialException, OSError) as e:
                self.stats.set(link=f"{self.port} (reconnecting)", error=str(e).split("\n")[0][:80])
                self.stopping.wait(1.0)

    def _read(self, ser):
        buf = b""
        last_frame = last_start = 0.0
        while not self.stopping.is_set():
            now = time.monotonic()
            # The firmware's stream is off until it gets a 'T', and goes off
            # again if the flight computer resets - so ask again when quiet.
            if self.start_cmd and now - last_frame > START_RETRY_S and now - last_start > START_RETRY_S:
                ser.write(self.start_cmd.encode("ascii"))
                last_start = now
            chunk = ser.read(ser.in_waiting or 1)
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                if self._line(raw.decode("ascii", "replace").strip()):
                    last_frame = time.monotonic()
            if len(buf) > 4096:           # no newline in sight: noise
                buf = b""

    def _line(self, line):
        if not line:
            return False
        if line.startswith("$TLM"):
            s = parse_tlm(line)
            self.stats.frame(s is not None)
            if s:
                self.out.put(s)
            return s is not None
        if line.startswith("timestamp_ms"):
            self.csv_header = [c.strip() for c in line.split(",")]
            return False
        if self.csv_header and line[0].isdigit():
            s = parse_log_row(self.csv_header, line.split(","))
            self.stats.frame(s is not None)
            if s:
                self.out.put(s)
            return s is not None
        if line.startswith("["):
            self.stats.set(fc_message=line[:110])
        return False


def load_log(path):
    """All rows of an SD-log CSV (comma or tab separated) as sample dicts."""
    with open(path, newline="", encoding="utf-8", errors="replace") as fh:
        text = fh.read()
    lines = text.splitlines()
    try:
        start = next(i for i, ln in enumerate(lines) if ln.lstrip().startswith("timestamp_ms"))
    except StopIteration:
        raise SystemExit(f"{path}: no 'timestamp_ms' header line - is this a flight log?")
    delim = "\t" if "\t" in lines[start] else ","
    rows = csv.reader(lines[start:], delimiter=delim)
    header = [h.strip() for h in next(rows)]
    out = [s for s in (parse_log_row(header, r) for r in rows) if s]
    if not out:
        raise SystemExit(f"{path}: no data rows")
    return out


class PlaybackSource(Source):
    """Feeds a list of samples at their recorded pace (times speed)."""

    def __init__(self, out, stats, samples, speed, label):
        super().__init__(out, stats)
        self.samples, self.speed, self.label = samples, speed, label

    def run(self):
        self.stats.set(link=self.label)
        t0_data = self.samples[0]["t_ms"]
        t0_wall = time.monotonic()
        for s in self.samples:
            due = t0_wall + (s["t_ms"] - t0_data) / 1000.0 / self.speed
            while not self.stopping.is_set():
                wait = due - time.monotonic()
                if wait <= 0:
                    break
                self.stopping.wait(min(wait, 0.05))
            if self.stopping.is_set():
                return
            self.stats.frame(True)
            self.out.put(s)
        self.stats.set(finished=True, link=self.label + " - finished")


# ----------------------------------------------------------------------
# Estimation: flight path and PID terms
# ----------------------------------------------------------------------

class PathEstimator:
    """Horizontal position from tilt x vertical speed - see flight_plot.m."""

    MAX_TILT_COS = math.cos(math.radians(75))

    def __init__(self):
        self.north = 0.0
        self.east = 0.0

    def step(self, s, dt):
        """Returns (north, east, tilt_deg, up_accel_g)."""
        q = [s["q0"], s["q1"], s["q2"], s["q3"]]
        norm = math.sqrt(sum(v * v for v in q)) if all(map(math.isfinite, q)) else 0.0
        q0, q1, q2, q3 = (v / norm for v in q) if norm > 0.5 else (1.0, 0.0, 0.0, 0.0)
        # Filter frame: X/Y horizontal, Z along the airframe toward the
        # nozzle; R(q) turns it into NED. Nose direction = -R[:, 2].
        n_n = -2 * (q1 * q3 + q0 * q2)
        n_e = -2 * (q2 * q3 - q0 * q1)
        n_u = 1 - 2 * (q1 * q1 + q2 * q2)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, n_u))))
        # Upward specific force in the earth frame [g] (1 g at rest)
        fx, fy, fz = s["az"], -s["ay"], s["ax"]
        a_up = -(2 * (q1 * q3 - q0 * q2) * fx + 2 * (q2 * q3 + q0 * q1) * fy + n_u * fz)
        if s["state"] in (POWERED, COAST) and n_u > self.MAX_TILT_COS:
            v_up = s["vel"] if math.isfinite(s["vel"]) and s["vel"] > 0 else 0.0
            self.north += v_up * n_n / n_u * dt
            self.east += v_up * n_e / n_u * dt
        return self.north, self.east, tilt, a_up


class PidReplay:
    """Re-runs pid_update() from src/pid.cpp to split the output into P, I and D."""

    def __init__(self, kp, ki, kd, lim, ilim, dalpha=1.0):
        self.kp, self.ki, self.kd, self.lim, self.ilim = kp, ki, kd, lim, ilim
        self.dalpha = dalpha
        self.d_filt = 0.0
        self.active = False
        self.integral = self.prev_err = 0.0
        self.first = True

    def step(self, measured, active, dt):
        if not active or not math.isfinite(measured):
            self.active = False
            return math.nan, math.nan, math.nan
        if not self.active:                  # rising edge: pid_reset()
            self.integral = self.prev_err = self.d_filt = 0.0
            self.first = True
        self.active = True
        e = -measured                        # setpoint is 0
        p = self.kp * e
        self.integral = max(-self.ilim, min(self.ilim, self.integral + self.ki * e * dt))
        if not self.first:
            self.d_filt = self.dalpha * (e - self.prev_err) / dt + (1 - self.dalpha) * self.d_filt
        d = self.kd * self.d_filt
        self.first = False
        self.prev_err = e
        return p, self.integral, d


# ----------------------------------------------------------------------
# Recording - SD-log column layout, so flight_plot.m can read it
# ----------------------------------------------------------------------

class Recorder:
    def __init__(self, folder):
        os.makedirs(folder, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(folder, f"TLM_{stamp}.csv")
        self.fh = open(self.path, "w", newline="", encoding="ascii")
        self.w = csv.writer(self.fh)
        self.w.writerow(LOG_COLUMNS + EXTRA_COLUMNS)
        self.last_flush = time.monotonic()

    def write(self, s, t_ms):
        flags = int(s["flags"]) if math.isfinite(s["flags"]) else None

        def f(v, fmt="{:.4f}"):
            return fmt.format(v) if math.isfinite(v) else "nan"

        def bit(mask):
            return "" if flags is None else int(bool(flags & mask))

        self.w.writerow([
            int(t_ms), f(s["tip_b"]), f(s["tip_a"]), f(s["spin"]),
            f(s["q0"]), f(s["q1"]), f(s["q2"]), f(s["q3"]),
            f(s["gx"], "{:.3f}"), f(s["gy"], "{:.3f}"), f(s["gz"], "{:.3f}"),
            f(s["ax"]), f(s["ay"]), f(s["az"]), f(s["mx"]), f(s["my"]), f(s["mz"]),
            f(s["temp"], "{:.2f}"), f(s["press"], "{:.2f}"), f(s["alt"], "{:.2f}"), f(s["vel"], "{:.2f}"),
            f(s["servo_p"], "{:.1f}"), f(s["servo_y"], "{:.1f}"), f(s["pid_p"]), f(s["pid_y"]),
            int(s["state"]) if math.isfinite(s["state"]) else "",
            bit(F_IMU_VALID), bit(F_BARO_OK), int(math.isfinite(s["mx"])),
            "" if flags is None else flags, f(s["baro_alt"], "{:.2f}"), f(s["pack_v"], "{:.2f}"),
            f(s["loop_us"], "{:.0f}"), f"{time.time():.3f}",
        ])
        if time.monotonic() - self.last_flush > 1.0:
            self.fh.flush()
            self.last_flush = time.monotonic()

    def close(self):
        self.fh.close()


# ----------------------------------------------------------------------
# Display
# ----------------------------------------------------------------------

class Table:
    """Append-only numpy table, one column per name; grows by doubling."""

    def __init__(self, names, cap=8192):
        self.col = {n: i for i, n in enumerate(names)}
        self.a = np.full((cap, len(names)), np.nan)
        self.n = 0

    def append(self, row):
        if self.n == len(self.a):
            self.a = np.vstack([self.a, np.full_like(self.a, np.nan)])
        self.a[self.n] = row
        self.n += 1

    def __getitem__(self, name):
        return self.a[:self.n, self.col[name]]

    def clear(self):
        self.n = 0


COLS = ("t", "state", "alt", "vel", "ax", "ay", "az", "amag", "a_up", "press", "temp",
        "tip_a", "tip_b", "tilt", "pid_p", "pid_y", "p_p", "i_p", "d_p", "p_y", "i_y", "d_y",
        "servo_p", "servo_y", "north", "east", "tvc")


class LiveDisplay:
    def __init__(self, samples, stats, cfg, window_s, recorder=None, on_close=None):
        self.q, self.stats, self.cfg = samples, stats, cfg
        self.window = window_s
        self.recorder = recorder
        self.on_close = on_close
        self.full_view = False
        self.paused = False
        self._reset_state()
        self._build()

    def _reset_state(self):
        self.tab = Table(COLS)
        self.path = PathEstimator()
        c = self.cfg
        self.pid_pitch = PidReplay(c["kp"], c["ki"], c["kd"], c["lim_p"], min(c["ilim"], c["lim_p"]), c["dalpha"])
        self.pid_yaw = PidReplay(c["kp"], c["ki"], c["kd"], c["lim_y"], min(c["ilim"], c["lim_y"]), c["dalpha"])
        self.t_first = self.last_t = self.last_raw_ms = None
        self.t_offset_ms = 0.0
        self.dt_nominal = 0.024
        self.resets = 0
        self.events = []          # (t, name, north, east, alt)
        self.events_drawn = 0
        self.event_artists = []
        self.seen_states = set()
        self.last = None          # latest sample
        self.t_launch = None
        self.max_alt = -math.inf
        self.max_vel = -math.inf
        self.dirty = True

    # ---- data in ------------------------------------------------------

    def ingest(self, s):
        raw = s["t_ms"]
        if self.last_raw_ms is not None and raw < self.last_raw_ms - 1000:
            # Flight computer restarted (millis() back near 0): keep time running forward.
            self.t_offset_ms += self.last_raw_ms - raw + self.dt_nominal * 1000
            self.resets += 1
        self.last_raw_ms = raw
        t_ms = raw + self.t_offset_ms
        if self.t_first is None:
            self.t_first = t_ms
        t = (t_ms - self.t_first) / 1000.0
        dt = t - self.last_t if self.last_t is not None else 0.0
        if 0 < dt < 1:
            self.dt_nominal = 0.9 * self.dt_nominal + 0.1 * dt
        else:
            dt = self.dt_nominal
        self.last_t = t

        state = int(s["state"]) if math.isfinite(s["state"]) else -1
        if math.isfinite(s["flags"]):
            tvc = bool(int(s["flags"]) & F_TVC_LIVE)
        else:   # SD logs have no flags: outputs are exactly 0 while TVC is off
            def nonzero(v):
                return math.isfinite(v) and v != 0.0
            tvc = state == POWERED or nonzero(s["pid_p"]) or nonzero(s["pid_y"])
        north, east, tilt, a_up = self.path.step(s, dt)
        pp, ip, dp = self.pid_pitch.step(s["tip_a"], tvc, dt)
        py, iy, dy = self.pid_yaw.step(s["tip_b"], tvc, dt)
        amag = math.sqrt(s["ax"] ** 2 + s["ay"] ** 2 + s["az"] ** 2)

        self.tab.append((t, state, s["alt"], s["vel"], s["ax"], s["ay"], s["az"], amag, a_up,
                         s["press"], s["temp"], s["tip_a"], s["tip_b"], tilt,
                         s["pid_p"], s["pid_y"], pp, ip, dp, py, iy, dy,
                         s["servo_p"], s["servo_y"], north, east, float(tvc)))

        if state in EVENT_STATES and state not in self.seen_states:
            self.events.append((t, EVENT_STATES[state], north, east, s["alt"]))
            if state == POWERED:
                self.t_launch = t
        self.seen_states.add(state)
        if math.isfinite(s["alt"]):
            self.max_alt = max(self.max_alt, s["alt"])
        if math.isfinite(s["vel"]):
            self.max_vel = max(self.max_vel, s["vel"])
        self.last = dict(s, t=t, tilt=tilt, tvc=tvc)
        if self.recorder:
            self.recorder.write(s, t_ms)
        self.dirty = True

    # ---- figure -------------------------------------------------------

    def _build(self):
        for key in ("keymap.save", "keymap.quit"):      # avoid 's'/'q' surprises mid-flight
            plt.rcParams[key] = []
        self.fig = plt.figure(figsize=(17, 9.5))
        try:
            self.fig.canvas.manager.set_window_title("TVC flight computer - live")
        except AttributeError:
            pass
        gs = self.fig.add_gridspec(4, 3, width_ratios=(1.3, 1, 1), left=0.03, right=0.985,
                                   top=0.875, bottom=0.06, hspace=0.6, wspace=0.27)
        f = self.fig
        self.ax3d = f.add_subplot(gs[:, 0], projection="3d")
        a_alt = f.add_subplot(gs[0, 1])
        a_vel = f.add_subplot(gs[1, 1], sharex=a_alt)
        a_acc = f.add_subplot(gs[2, 1], sharex=a_alt)
        a_prs = f.add_subplot(gs[3, 1], sharex=a_alt)
        a_tmp = f.add_subplot(gs[0, 2], sharex=a_alt)
        a_pp = f.add_subplot(gs[1, 2], sharex=a_alt)
        a_py = f.add_subplot(gs[2, 2], sharex=a_alt)
        a_srv = f.add_subplot(gs[3, 2], sharex=a_alt)
        self.time_axes = (a_alt, a_vel, a_acc, a_prs, a_tmp, a_pp, a_py, a_srv)

        c = self.cfg
        gains = f"Kp {c['kp']:g}  Ki {c['ki']:g}  Kd {c['kd']:g}"
        for ax, title, unit in ((a_alt, "Altitude", "m"), (a_vel, "Vertical speed", "m/s"),
                                (a_acc, "Acceleration", "g"), (a_prs, "Pressure", "hPa"),
                                (a_tmp, "Temperature", "°C"),
                                (a_pp, f"Pitch PID (input tip_a)   {gains}", "deg"),
                                (a_py, f"Yaw PID (input tip_b)   {gains}", "deg"),
                                (a_srv, "Servo commands", "µs")):
            ax.set_title(title, fontsize=9, loc="left")
            ax.set_ylabel(unit, fontsize=8)
            ax.tick_params(labelsize=7)
            ax.grid(True, alpha=0.3)
        for ax in (a_prs, a_srv):
            ax.set_xlabel("time since first frame [s]", fontsize=8)

        def line(ax, key, label=None, **kw):
            kw.setdefault("lw", 1.1)
            return key, ax.plot([], [], label=label, **kw)[0]

        self.lines = [
            line(a_alt, "alt", color="#0072bd", lw=1.5),
            line(a_vel, "vel", color="#0072bd", lw=1.5),
            line(a_acc, "ax", "ax (+ nozzle)"), line(a_acc, "ay", "ay"), line(a_acc, "az", "az"),
            line(a_acc, "amag", "|a|", color="k", lw=1.4),
            line(a_acc, "a_up", "up (earth)", color="#d9541a", ls="--"),
            line(a_pp, "tip_a", "input: tilt", color="k", lw=1.5),
            line(a_pp, "p_p", "P", color="#0072bd"), line(a_pp, "i_p", "I", color="#ed8c1a", lw=1.3),
            line(a_pp, "d_p", "D", color="#7e2f8e", lw=0.8, alpha=0.6),
            line(a_pp, "pid_p", "output", color="#e60000", lw=1.6),
            line(a_py, "tip_b", "input: tilt", color="k", lw=1.5),
            line(a_py, "p_y", "P", color="#0072bd"), line(a_py, "i_y", "I", color="#ed8c1a", lw=1.3),
            line(a_py, "d_y", "D", color="#7e2f8e", lw=0.8, alpha=0.6),
            line(a_py, "pid_y", "output", color="#e60000", lw=1.6),
            line(a_srv, "servo_p", "pitch"), line(a_srv, "servo_y", "yaw"),
        ]
        # Baro values are NaN on rows without a fresh sample (SD logs): plot only valid points
        self.sparse_lines = [line(a_prs, "press", color="#0072bd", lw=1.5),
                             line(a_tmp, "temp", color="#d9541a", lw=1.5)]
        a_vel.axhline(0, color="0.5", lw=0.8, ls=":")
        for ax, lim in ((a_pp, c["lim_p"]), (a_py, c["lim_y"])):
            for y in (-lim, lim):
                ax.axhline(y, color="0.6", lw=0.8, ls=":")
        for ax in (a_acc, a_pp, a_py, a_srv):
            ax.legend(fontsize=6.5, loc="upper left", ncol=5, frameon=False)

        a3 = self.ax3d
        a3.set_xlabel("East [m]", fontsize=8)
        a3.set_ylabel("North [m]", fontsize=8)
        a3.set_zlabel("Altitude AGL [m]", fontsize=8)
        a3.tick_params(labelsize=7)
        a3.set_title("Estimated flight path (altitude measured; horizontal from tilt, no GPS)",
                     fontsize=9)
        a3.view_init(elev=22, azim=-50)
        a3.set_box_aspect((1, 1, 1.5), zoom=1.15)
        self.track, = a3.plot([], [], [], ":", color="0.7", lw=1)
        self.state_lines = {}
        for s, (name, color) in enumerate(zip(STATE_NAMES, STATE_COLORS)):
            self.state_lines[s], = a3.plot([], [], [], "-", color=color, lw=2.2, label=name)
        self.head, = a3.plot([], [], [], "o", color="k", ms=7)
        a3.plot([0], [0], [0], "^", color="k", ms=8)
        self.legend_states = None

        self.status_link = f.text(0.01, 0.965, "", fontsize=10, family="monospace", weight="bold")
        self.status_flight = f.text(0.01, 0.935, "", fontsize=10, family="monospace")
        self.status_misc = f.text(0.01, 0.905, "", fontsize=8.5, family="monospace", color="0.35")

        f.canvas.mpl_connect("key_press_event", self._key)
        f.canvas.mpl_connect("close_event", lambda _e: self.on_close and self.on_close())

    def _key(self, ev):
        if ev.key == "w":
            self.full_view = not self.full_view
        elif ev.key == "x":
            for a in self.event_artists:
                a.remove()
            self._reset_state()
            for _, ln in self.lines + self.sparse_lines:
                ln.set_data([], [])
            for ln in list(self.state_lines.values()) + [self.track, self.head]:
                ln.set_data_3d([], [], [])
        elif ev.key == " ":
            self.paused = not self.paused
        self.dirty = True

    # ---- drawing ------------------------------------------------------

    def update(self, _frame):
        n = 0
        try:
            while n < 20000:
                self.ingest(self.q.get_nowait())
                n += 1
        except queue.Empty:
            pass
        if not self.paused and self.dirty and self.tab.n:
            self._draw_series()
            self._draw_path()
            self._draw_events()
            self.dirty = False
        self._draw_status()
        return ()

    def _draw_series(self):
        t_all = self.tab["t"]
        t_end = t_all[-1]
        lo = 0 if self.full_view else int(np.searchsorted(t_all, t_end - self.window))
        stride = max(1, (self.tab.n - lo) // 3000)
        sl = slice(lo, self.tab.n, stride)
        t = t_all[sl]
        for key, ln in self.lines:
            ln.set_data(t, self.tab[key][sl])
        for key, ln in self.sparse_lines:
            y = self.tab[key][lo:]
            m = np.isfinite(y)
            ln.set_data(t_all[lo:][m][::stride], y[m][::stride])
        left = t[0]
        right = max(t_end, left + (self.window if not self.full_view else 1.0))
        self.time_axes[0].set_xlim(left, right)
        for ax in self.time_axes:
            ax.relim(visible_only=True)
            ax.autoscale_view(scalex=False)

    def _draw_path(self):
        n = self.tab.n
        stride = max(1, n // 6000)
        e, nn, u = self.tab["east"], self.tab["north"], self.tab["alt"]
        st = self.tab["state"]
        for s, ln in self.state_lines.items():
            m = st == s
            if not m.any():
                continue
            m = m | np.r_[False, m[:-1]]           # include the next point so runs join up
            ln.set_data_3d(np.where(m, e, np.nan)[::stride], np.where(m, nn, np.nan)[::stride],
                           np.where(m, u, np.nan)[::stride])
        self.track.set_data_3d(e[::stride], nn[::stride], np.zeros_like(e[::stride]))
        self.head.set_data_3d([e[-1]], [nn[-1]], [u[-1]])

        top = np.nanmax(u) if np.isfinite(u).any() else 1.0
        bottom = np.nanmin(u) if np.isfinite(u).any() else 0.0
        hs = max(np.nanmax(np.abs(e)), np.nanmax(np.abs(nn)), 0.15 * top, 5.0)
        self.ax3d.set_xlim(-hs, hs)
        self.ax3d.set_ylim(-hs, hs)
        self.ax3d.set_zlim(min(0.0, bottom), max(1.05 * top, 5.0))

        seen = tuple(sorted(s for s in self.seen_states if 0 <= s < len(STATE_NAMES)))
        if seen != self.legend_states:
            self.ax3d.legend(handles=[self.state_lines[s] for s in seen], fontsize=7,
                             loc="upper left")
            self.legend_states = seen

    def _draw_events(self):
        for t, name, north, east, alt in self.events[self.events_drawn:]:
            for i, ax in enumerate(self.time_axes):
                self.event_artists.append(ax.axvline(t, color="0.4", lw=0.9, ls=":"))
                if i in (0, 4):
                    self.event_artists.append(ax.text(t, 0.97, name + " ", transform=ax.get_xaxis_transform(),
                                                      fontsize=7, va="top", ha="right", rotation=90,
                                                      color="0.3"))
            if math.isfinite(alt):
                self.event_artists.append(self.ax3d.scatter([east], [north], [alt], color="#ffe033",
                                                            edgecolors="k", s=35, depthshade=False))
                self.event_artists.append(self.ax3d.text(east, north, alt, "  " + name, fontsize=7))
        self.events_drawn = len(self.events)

    def _draw_status(self):
        st = self.stats.snapshot()
        if st["finished"]:
            link_ok, age_txt = True, "done"
        elif st["age"] is None:
            link_ok, age_txt = False, "waiting for data"
        else:
            link_ok = st["age"] < LINK_STALE_S
            age_txt = f"last frame {st['age']:.1f} s ago" + ("" if link_ok else "  - NO SIGNAL")
        line = (f"{st['link']}   {st['rate']:5.1f} frames/s   ok {st['ok']}   "
                f"bad {st['bad']}   {age_txt}")
        if st["error"]:
            line += f"   [{st['error']}]"
        self.status_link.set_text(line)
        self.status_link.set_color("#1a7f37" if link_ok else "#d1242f")

        s = self.last
        if s:
            state = int(s["state"]) if math.isfinite(s["state"]) else -1
            name = STATE_NAMES[state] if 0 <= state < len(STATE_NAMES) else "?"
            tplus = f"T+{s['t'] - self.t_launch:6.2f} s" if self.t_launch is not None else "T  -- pad"
            pack = f"   pack {s['pack_v']:.2f} V" if math.isfinite(s["pack_v"]) else ""
            self.status_flight.set_text(
                f"{name:<8} {tplus}   alt {s['alt']:7.1f} m (max {self.max_alt:6.1f})   "
                f"vel {s['vel']:6.1f} m/s (max {self.max_vel:5.1f})   tilt {s['tilt']:5.1f}°   "
                f"TVC {'ON ' if s['tvc'] else 'off'}{pack}")
            self.status_flight.set_color("#e60000" if state == ABORT else "black")

        misc = []
        if st["fc"]:
            misc.append(f"FC: {st['fc']}")
        if self.resets:
            misc.append(f"flight computer restarted x{self.resets}")
        if self.recorder:
            misc.append(f"recording {self.recorder.path}")
        misc.append("keys: w whole/rolling  x clear  space pause" + ("  [PAUSED]" if self.paused else ""))
        self.status_misc.set_text("   |   ".join(misc))


# ----------------------------------------------------------------------
# Simulated flight - for trying the display without hardware
# ----------------------------------------------------------------------

def simulate_flight(rate_hz, cfg, seed=7):
    """A plausible single-deploy flight as sample dicts (made-up physics)."""
    rng = np.random.default_rng(seed)
    g = 9.80665
    dt = 1.0 / rate_hz
    pad_s, burn_s = 3.0, 3.45
    m_wet, m_prop = 0.80, 0.06
    k_body = 0.5 * 1.225 * 0.55 * math.pi * 0.0375 ** 2
    k_chute = 0.5 * 1.225 * 1.5 * 0.45
    ground_asl = 150.0
    heading = math.radians(40)                       # where the rocket weathercocks to
    us_per_deg = (72.9, 21.5)                        # SERVO_*_US_PER_DEG, roughly

    def thrust(tb):
        if tb < 0 or tb > burn_s:
            return 0.0
        if tb < 0.25:
            return 25.0
        return 13.5 if tb < burn_s - 0.25 else 13.5 * (burn_s - tb) / 0.25

    pid_p = PidReplay(cfg["kp"], cfg["ki"], cfg["kd"], cfg["lim_p"], min(cfg["ilim"], cfg["lim_p"]), cfg["dalpha"])
    pid_y = PidReplay(cfg["kp"], cfg["ki"], cfg["kd"], cfg["lim_y"], min(cfg["ilim"], cfg["lim_y"]), cfg["dalpha"])
    out = []
    h = v = 0.0
    state, t_apogee, t_land = IDLE, None, None
    t = 0.0
    while True:
        tb = t - pad_s                               # time since ignition
        m = m_wet - m_prop * min(max(tb, 0) / burn_s, 1.0)
        # --- state machine ---
        if state in (IDLE, ARMED):
            state = ARMED if t > 1.0 else IDLE
            if tb >= 0.15:
                state = POWERED
        elif state == POWERED and tb > burn_s:
            state = COAST
        elif state == COAST and v < 0:
            state, t_apogee = APOGEE, t
        elif state == APOGEE and t - t_apogee > 0.1:
            state = DESCENT
        elif state == DESCENT and t - t_apogee > 1.0:
            state = MAIN
        elif state == MAIN and h <= 0.05:
            state, t_land = LANDED, t
        if t_land is not None and t - t_land > 1.0:
            break

        # --- tilt: weathercock + a disturbance the PID damps; swinging under chute ---
        if tb < 0:
            lean = 0.4
        elif state in (IDLE, ARMED, POWERED, COAST, APOGEE):
            lean = 2.5 + 4.0 * math.exp(-tb / 1.2) * math.sin(2 * math.pi * 1.1 * tb) ** 2 + 3.0 * max(tb - burn_s, 0)
        else:
            lean = 12.0 + 8.0 * math.sin(2 * math.pi * 0.4 * t)
        lean = min(lean, 70.0)
        spin = math.radians(15.0 * max(tb, 0))       # slow roll, deg/s
        th = math.radians(lean)

        # --- vertical motion ---
        T = thrust(tb)
        if state == MAIN:
            drag = k_chute * v * abs(v)
        else:
            drag = k_body * v * abs(v)
        if state in (IDLE, ARMED) or (tb < 0.15):
            acc = 0.0
        else:
            acc = (T * math.cos(th) - drag) / m - g
        if t_land is not None:
            acc, v, h = 0.0, 0.0, 0.0
        v += acc * dt
        h = max(h + v * dt, 0.0) if t_land is None else 0.0

        # --- sensors ---
        axial = (T - (drag if state != MAIN else 0.0)) / (m * g)   # nose-forward specific force
        if state in (IDLE, ARMED, LANDED):
            axial = math.cos(th)
        elif state == MAIN:
            axial = max(0.0, (k_chute * v * v) / (m * g))
        ax = -axial + rng.normal(0, 0.02)
        ay = rng.normal(0, 0.02) + (0.0 if state in (IDLE, ARMED) else 0.05 * math.sin(7 * t))
        az = rng.normal(0, 0.02)

        # Body-frame tip vector, and the quaternion the firmware would hold
        phi = heading - spin
        tip_a = lean * math.cos(phi) + rng.normal(0, 0.05)
        tip_b = lean * math.sin(phi) + rng.normal(0, 0.05)
        c, sn = math.cos(th / 2), math.sin(th / 2)
        sw = (c, math.sin(heading) * sn, -math.cos(heading) * sn, 0.0)       # swing toward heading
        ct, st = math.cos(spin / 2), math.sin(spin / 2)                     # twist about the airframe
        q = (sw[0] * ct, sw[1] * ct + sw[2] * st, sw[2] * ct - sw[1] * st, sw[0] * st)

        tvc = state == POWERED
        pp, ip, dp = pid_p.step(tip_a, tvc, dt)
        py, iy, dy = pid_y.step(tip_b, tvc, dt)
        out_p = max(-cfg["lim_p"], min(cfg["lim_p"], pp + ip + dp)) if tvc else 0.0
        out_y = max(-cfg["lim_y"], min(cfg["lim_y"], py + iy + dy)) if tvc else 0.0

        alt_noisy = h + rng.normal(0, 0.15)
        press = 1013.25 * (1 - 2.25577e-5 * (ground_asl + alt_noisy)) ** 5.25588
        flags = F_IMU_VALID | F_BARO_OK | (F_TVC_LIVE if tvc else 0) | 0x08
        if state in (MAIN, LANDED):
            flags |= 0x80
        s = empty_sample()
        s.update(t_ms=round(t * 1000), state=float(state), flags=float(flags),
                 q0=q[0], q1=q[1], q2=q[2], q3=q[3], tip_a=tip_a, tip_b=tip_b,
                 spin=(math.degrees(spin) + 180) % 360 - 180,
                 gx=rng.normal(0, 0.3), gy=rng.normal(0, 0.3), gz=15.0 * (tb > 0) + rng.normal(0, 0.3),
                 ax=ax, ay=ay, az=az, mx=0.21, my=0.05, mz=-0.42,
                 alt=h, vel=v, baro_alt=alt_noisy, press=press, temp=24.0 - 0.0065 * h + 0.002 * t,
                 pid_p=out_p, pid_y=out_y,
                 servo_p=1500 + out_p * us_per_deg[0], servo_y=1500 + out_y * us_per_deg[1],
                 pack_v=11.4 - 0.15 * (tb > 0), loop_us=3400 + rng.normal(0, 150))
        out.append(s)
        t += dt
    return out


def write_sim_log(path, cfg):
    """A simulated flight as an SD-style log (125 Hz, baro fresh on ~60% of rows)."""
    samples = simulate_flight(125.0, cfg)

    def f(v):
        return f"{v:.4f}"

    with open(path, "w", newline="", encoding="ascii") as fh:
        w = csv.writer(fh)
        w.writerow(LOG_COLUMNS)
        for i, s in enumerate(samples):
            fresh = (i * 75) // 125 != ((i - 1) * 75) // 125     # LPS22HB at 75 Hz
            w.writerow([s["t_ms"] + 120000, f(s["tip_b"]), f(s["tip_a"]), f(s["spin"]),
                        f(s["q0"]), f(s["q1"]), f(s["q2"]), f(s["q3"]),
                        f(s["gx"]), f(s["gy"]), f(s["gz"]), f(s["ax"]), f(s["ay"]), f(s["az"]),
                        f(s["mx"]), f(s["my"]), f(s["mz"]),
                        f(s["temp"]) if fresh else "nan", f(s["press"]) if fresh else "nan",
                        f(s["alt"]), f(s["vel"]), f(s["servo_p"]), f(s["servo_y"]),
                        f(s["pid_p"]), f(s["pid_y"]), int(s["state"]), 1, int(fresh), 1])
    print(f"Wrote {len(samples)} simulated records to {path}")


# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog="\n\n".join(__doc__.split("\n\n")[1:]))
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="serial port, e.g. COM7 or /dev/ttyUSB0")
    src.add_argument("--replay", metavar="CSV", help="play back an SD log or a recording")
    src.add_argument("--sim", action="store_true", help="simulated flight")
    src.add_argument("--list-ports", action="store_true", help="list serial ports and exit")
    src.add_argument("--write-sim", metavar="CSV", help="write a simulated flight log and exit")
    ap.add_argument("--baud", type=int, default=115200,
                    help="serial baud rate (ignored by USB; set it for a radio modem)")
    ap.add_argument("--start-cmd", default="T",
                    help="sent to start the $TLM stream, and again whenever it goes quiet "
                         "(default 'T'; '' sends nothing)")
    ap.add_argument("--speed", type=float, default=1.0, help="replay / sim speed (default 1)")
    ap.add_argument("--window", type=float, default=30.0, help="rolling window [s] (default 30)")
    ap.add_argument("--record-dir", default="telemetry_logs", help="where --port recordings go")
    ap.add_argument("--no-record", action="store_true", help="don't record --port data")
    args = ap.parse_args()

    cfg = read_firmware_config()

    if args.list_ports or args.port:
        if serial is None:
            sys.exit("pyserial is not installed:  pip install pyserial")
    if args.list_ports:
        ports = serial.tools.list_ports.comports()
        for p in ports:
            print(f"{p.device:<12} {p.description}")
        if not ports:
            print("No serial ports found.")
        return
    if args.write_sim:
        write_sim_log(args.write_sim, cfg)
        return

    samples = queue.Queue()
    stats = LinkStats()
    recorder = None
    if args.port:
        source = SerialSource(samples, stats, args.port, args.baud, args.start_cmd)
        if not args.no_record:
            recorder = Recorder(args.record_dir)
            print(f"Recording to {recorder.path}")
    elif args.replay:
        source = PlaybackSource(samples, stats, load_log(args.replay), args.speed,
                                f"replay {os.path.basename(args.replay)} x{args.speed:g}")
    else:
        source = PlaybackSource(samples, stats, simulate_flight(42.0, cfg), args.speed,
                                f"simulated flight x{args.speed:g}")

    display = LiveDisplay(samples, stats, cfg, args.window, recorder, on_close=source.stop)
    source.start()
    anim = FuncAnimation(display.fig, display.update, interval=100, cache_frame_data=False)
    try:
        plt.show()
    finally:
        source.stop()
        if recorder:
            recorder.close()
            print(f"Saved {recorder.path}")
    del anim


if __name__ == "__main__":
    main()
