"""
Telemetry tab for bench_gui.py — live attitude and flight data.

Reads the $TLM frames emitted by telemetry_emit() (include/telemetry.h)
over USB and shows:

  * a 3D view of the vehicle, drawn from the filter quaternion
  * a tilt scope of tip A / tip B, the two angles the PID drives to zero
  * numeric readouts, flight state and flag lamps
  * strip charts of attitude, body rates, magnetic field, TVC command,
    altitude, velocity
  * CSV recording of every frame received

Colour carries meaning in two ways. Lamps keep bench_gui's annunciator
convention (green normal, amber caution, red warning, grey off). Data
is coloured by body axis everywhere it appears — 3D arrows, charts,
readouts, scope — so blue is always body Y and orchid is always body Z.
Body X (the long axis) and plain scalars are drawn in the panel's text
colour. On screen, axes carry the IMU names printed on the board:
IMU X across the airframe, IMU Y along it (+ toward the nose), IMU Z
out of the board face.

Frames
------
  * Body frame (firmware): +X nose-to-nozzle, +Y = IMU +X, +Z = IMU +Z.
    lsm6dsox_to_body() in include/lsm6dsox.h turns IMU axes into these.
  * The quaternion is in the filter frame, NED, after the remap in
    mahrs_integration.cpp: filter x = body Z, filter y = -body Y,
    filter z = body X. Identity = vertical on the pad, nose up.
  * Magnetometer (mx/my/mz): calibrated, in the magnetometer's own axes
    (which run the same way as the IMU's), NaN without a fresh sample.
  * Heading: filter "north" is magnetic north — the magnetometer holds the
    heading (rotation about the vertical) — falling back to the gyro when
    there's no fresh mag sample. The 3D view homes its camera to the side
    the board's +Z faces when telemetry starts.
"""

import collections
import csv
import math
import random
import time
import tkinter as tk
import tkinter.font as tkfont
from tkinter import ttk, filedialog

# ----------------------------------------------------------------------
# Frame format — must mirror include/telemetry.h
# ----------------------------------------------------------------------

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
Tlm = collections.namedtuple("Tlm", TLM_FIELDS)

F_IMU_VALID  = 0x01
F_BARO_OK    = 0x02
F_TVC_LIVE   = 0x04
F_ARM_SWITCH = 0x08
F_PAD_REST   = 0x10
F_LAUNCH_DET = 0x20
F_GROUND_CAP = 0x40
F_MAIN_FIRED = 0x80

MAG_FIELDS = ("mx", "my", "mz")
MAG_OK_GAUSS = (0.10, 1.00)   # Earth's field, with margin — same band as bench test 3

LOOP_BUDGET_US = 8000      # LOOP_INTERVAL_US in main_control_loop.cpp
PID_LIMIT_DEG  = 5.0       # PID_OUT_MAX in pid.h
STALE_S        = 1.0       # no frame for this long = telemetry lost
CHART_WINDOW_S = 10.0
TRAIL_S        = 2.0


def _checksum(body):
    ck = 0
    for ch in body:
        ck ^= ord(ch)
    return ck


def parse_tlm(line):
    """Return a Tlm, or None if the line isn't a valid $TLM frame."""
    line = line.strip()
    if not line.startswith("$TLM,") or "*" not in line:
        return None
    body, _, ck_txt = line[1:].rpartition("*")
    try:
        if _checksum(body) != int(ck_txt[:2], 16):
            return None
    except ValueError:
        return None

    f = body.split(",")
    if len(f) == len(TLM_FIELDS) + 1 - len(MAG_FIELDS):
        # Firmware from before the magnetometer fields: no mag data.
        at = TLM_FIELDS.index(MAG_FIELDS[0]) + 1
        f = f[:at] + ["nan"] * len(MAG_FIELDS) + f[at:]
    if len(f) != len(TLM_FIELDS) + 1:
        return None
    try:
        vals = ([int(f[1]), f[2], int(f[3])]
                + [float(x) for x in f[4:-1]]
                + [int(f[-1])])
    except ValueError:
        return None
    return Tlm(*vals)


def format_tlm(fr):
    """Build a $TLM line the way telemetry_emit() does. Used by the demo."""
    body = ("TLM,{t_ms},{state},{flags},"
            "{q0:.5f},{q1:.5f},{q2:.5f},{q3:.5f},"
            "{tip_a:.2f},{tip_b:.2f},{spin:.2f},"
            "{gx:.2f},{gy:.2f},{gz:.2f},"
            "{ax:.3f},{ay:.3f},{az:.3f},"
            "{mx:.4f},{my:.4f},{mz:.4f},"
            "{alt_m:.2f},{vel_ms:.2f},{baro_alt_m:.2f},{press_hpa:.2f},{temp_c:.1f},"
            "{pid_p:.3f},{pid_y:.3f},{servo_p_us:.0f},{servo_y_us:.0f},"
            "{pack_v:.2f},{loop_us}").format(**fr._asdict())
    return f"${body}*{_checksum(body):02X}"


# ----------------------------------------------------------------------
# Small maths helpers
# ----------------------------------------------------------------------


def mix(a, b, t):
    """Blend two #rrggbb colours; t=0 gives a, t=1 gives b."""
    av = [int(a[i:i + 2], 16) for i in (1, 3, 5)]
    bv = [int(b[i:i + 2], 16) for i in (1, 3, 5)]
    return "#" + "".join(f"{int(x + (y - x) * t):02x}" for x, y in zip(av, bv))


def finite(v):
    return v == v and v not in (math.inf, -math.inf)


def quat_matrix(q0, q1, q2, q3):
    """Rotation matrix taking filter-sensor-frame vectors into NED."""
    n = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3) or 1.0
    q0, q1, q2, q3 = q0 / n, q1 / n, q2 / n, q3 / n
    return (
        (1 - 2 * (q2 * q2 + q3 * q3), 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)),
        (2 * (q1 * q2 + q0 * q3), 1 - 2 * (q1 * q1 + q3 * q3), 2 * (q2 * q3 - q0 * q1)),
        (2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1 * q1 + q2 * q2)),
    )


def tilt_deg(fr):
    """Angle between the vehicle's long axis and vertical, from the quaternion."""
    n2 = fr.q0 ** 2 + fr.q1 ** 2 + fr.q2 ** 2 + fr.q3 ** 2 or 1.0
    c = 1 - 2 * (fr.q1 ** 2 + fr.q2 ** 2) / n2
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def euler_to_quat(roll, pitch, yaw):
    """ZYX Euler (deg) to quaternion — inverse of madgwick_get_euler()."""
    cr, sr = math.cos(math.radians(roll) / 2), math.sin(math.radians(roll) / 2)
    cp, sp = math.cos(math.radians(pitch) / 2), math.sin(math.radians(pitch) / 2)
    cy, sy = math.cos(math.radians(yaw) / 2), math.sin(math.radians(yaw) / 2)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


def nice_step(span, target=4):
    raw = span / target
    mag = 10 ** math.floor(math.log10(raw)) if raw > 0 else 1
    for m in (1, 2, 2.5, 5, 10):
        if raw <= m * mag:
            return m * mag
    return 10 * mag


# ----------------------------------------------------------------------
# 3D attitude view
# ----------------------------------------------------------------------


class AttitudeView(tk.Canvas):
    """Wireframe vehicle drawn from the filter quaternion. Drag to orbit."""

    # Home camera on the board's +Z side, as you'd stand facing it: IMU +Z
    # points at the camera and IMU +X is on the right, so leans on screen
    # go the same way as the real vehicle. HOME_AZ is for +Z pointing
    # north; _home() adds the vehicle's actual heading, since that now
    # comes from the magnetometer rather than the power-up direction.
    HOME_AZ, HOME_EL = 200.0, 16.0
    CAM_DIST = 3.4
    GROUND = -0.62

    # Airframe profile in filter-sensor coordinates (z = body X, + toward
    # the nozzle). Proportions only — the real vehicle is a 3" airframe.
    RADIUS = 0.075
    TUBE = (-0.40, 0.46)
    NOSE = ((-0.40, 0.075), (-0.49, 0.068), (-0.57, 0.050),
            (-0.63, 0.028), (-0.67, 0.0))
    NOZZLE = ((0.46, 0.036), (0.54, 0.052))
    RING_Z = (-0.40, 0.03, 0.46)
    N_MERID, N_SEG = 10, 22

    def __init__(self, parent, theme, fonts):
        super().__init__(parent, bg=theme["WELL"], highlightthickness=1,
                         highlightbackground=theme["RULE"], width=440, height=300)
        self.t = theme
        self.fonts = fonts
        self.az, self.el = self.HOME_AZ, self.HOME_EL
        self.frame = None
        self.live = False
        self._drag = None
        self._homed = False            # camera placed on the +Z side for this stream
        self.w, self.h = 440, 300

        self.bind("<Configure>", self._on_resize)
        self.bind("<ButtonPress-1>", self._on_press)
        self.bind("<B1-Motion>", self._on_drag)
        self.bind("<ButtonRelease-1>", lambda e: setattr(self, "_drag", None))
        self.bind("<Double-Button-1>", self._reset_view)

    # -- interaction ----------------------------------------------------
    def _on_resize(self, e):
        self.w, self.h = e.width, e.height
        self.draw()

    def _on_press(self, e):
        self._drag = (e.x, e.y, self.az, self.el)

    def _on_drag(self, e):
        if not self._drag:
            return
        x0, y0, az0, el0 = self._drag
        self.az = (az0 - (e.x - x0) * 0.5) % 360
        self.el = max(-5.0, min(85.0, el0 + (e.y - y0) * 0.4))
        self.draw()

    def _reset_view(self, _e=None):
        self.az, self.el = self.HOME_AZ, self.HOME_EL
        self._homed = False
        self.draw()

    def set(self, frame, live):
        # Re-home on the first frame of a stream, and after a reboot.
        if frame is not None and (self.frame is None or frame.t_ms < self.frame.t_ms):
            self._homed = False
        self.frame = frame
        self.live = live

    def _home(self, R):
        """Put the camera on the side IMU +Z faces now. Only when homing —
        following the heading continuously would hide the roll."""
        zv = self._to_view(R, (1, 0, 0))            # body +Z (sensor +x), view coords
        if math.hypot(zv[0], zv[1]) > 0.3:           # skip if +Z is near vertical
            heading = math.degrees(math.atan2(zv[0], zv[1]))   # from north toward east
            # The camera sits at compass bearing 180 - az (see _camera), so
            # turning it with the heading means subtracting.
            self.az = (self.HOME_AZ - heading) % 360
        self.el = self.HOME_EL
        self._homed = True

    # -- camera ---------------------------------------------------------
    def _camera(self):
        az, el = math.radians(self.az), math.radians(self.el)
        c = (math.sin(az) * math.cos(el), -math.cos(az) * math.cos(el), math.sin(el))
        r = (math.cos(az), math.sin(az), 0.0)
        u = (c[1] * r[2] - c[2] * r[1], c[2] * r[0] - c[0] * r[2], c[0] * r[1] - c[1] * r[0])
        return c, r, u

    def _project(self, p, cam):
        c, r, u = cam
        d = p[0] * c[0] + p[1] * c[1] + p[2] * c[2]
        f = self.CAM_DIST / (self.CAM_DIST - d)
        k = min(self.w, self.h) * 0.56
        return (self.w / 2 + (p[0] * r[0] + p[1] * r[1] + p[2] * r[2]) * k * f,
                self.h * 0.45 - (p[0] * u[0] + p[1] * u[1] + p[2] * u[2]) * k * f)

    @staticmethod
    def _to_view(R, p):
        """Sensor-frame point -> view coords (east, north, up)."""
        n = R[0][0] * p[0] + R[0][1] * p[1] + R[0][2] * p[2]
        e = R[1][0] * p[0] + R[1][1] * p[1] + R[1][2] * p[2]
        d = R[2][0] * p[0] + R[2][1] * p[1] + R[2][2] * p[2]
        return (e, n, -d)

    # -- drawing --------------------------------------------------------
    def draw(self):
        self.delete("all")
        t = self.t
        fr = self.frame
        R = quat_matrix(fr.q0, fr.q1, fr.q2, fr.q3) if fr else quat_matrix(1, 0, 0, 0)
        if fr is not None and not self._homed:
            self._home(R)
        cam = self._camera()
        c = cam[0]
        fade = 0.0 if self.live else 0.65

        def col(base, extra=0.0):
            return mix(base, t["WELL"], min(0.92, fade + extra * (1 - fade)))

        # Ground grid and pad
        g = self.GROUND
        for i in range(-4, 5):
            v = i * 0.22
            shade = 0.55 if i == 0 else 0.72
            self.create_line(*self._project((v, -0.88, g), cam),
                             *self._project((v, 0.88, g), cam), fill=col(t["RULE"], shade - 0.55))
            self.create_line(*self._project((-0.88, v, g), cam),
                             *self._project((0.88, v, g), cam), fill=col(t["RULE"], shade - 0.55))

        # Vertical reference and the vehicle's shadow on the pad
        self.create_line(*self._project((0, 0, g), cam), *self._project((0, 0, 0.80), cam),
                         fill=col(t["TEXT_DIM"], 0.35), dash=(3, 4))
        nose = self._to_view(R, (0, 0, self.NOSE[-1][0]))
        tail = self._to_view(R, (0, 0, self.NOZZLE[-1][0]))
        self.create_line(*self._project((nose[0], nose[1], g), cam),
                         *self._project((tail[0], tail[1], g), cam),
                         fill=col(t["RULE"], 0.1), width=6, capstyle="round")

        # Airframe — split into back (dim) and front segments, back first
        back, front = [], []

        def seg(p_a, p_b, normal):
            nv = self._to_view(R, normal)
            facing = nv[0] * c[0] + nv[1] * c[1] + nv[2] * c[2]
            a = self._project(self._to_view(R, p_a), cam)
            b = self._project(self._to_view(R, p_b), cam)
            (front if facing > 0 else back).append((a, b))

        rad = self.RADIUS
        profile = [(self.TUBE[1], rad), (self.TUBE[0], rad)] + list(self.NOSE[1:])
        for i in range(self.N_MERID):
            th = 2 * math.pi * i / self.N_MERID
            ct, st = math.cos(th), math.sin(th)
            for (za, ra), (zb, rb) in zip(profile, profile[1:]):
                seg((ra * ct, ra * st, za), (rb * ct, rb * st, zb), (ct, st, -0.3 * (za < self.TUBE[0] + 0.001)))

        rings = [(z, rad) for z in self.RING_Z] + [self.NOSE[2]] + list(self.NOZZLE)
        for z, rr in rings:
            for j in range(self.N_SEG):
                a0 = 2 * math.pi * j / self.N_SEG
                a1 = 2 * math.pi * (j + 1) / self.N_SEG
                am = (a0 + a1) / 2
                seg((rr * math.cos(a0), rr * math.sin(a0), z),
                    (rr * math.cos(a1), rr * math.sin(a1), z),
                    (math.cos(am), math.sin(am), 0))
        for i in range(4):
            th = math.pi / 4 + i * math.pi / 2
            (za, ra), (zb, rb) = self.NOZZLE
            seg((ra * math.cos(th), ra * math.sin(th), za),
                (rb * math.cos(th), rb * math.sin(th), zb), (math.cos(th), math.sin(th), 0))

        back_col, front_col = col(t["TEXT"], 0.72), col(t["TEXT"])
        for a, b in back:
            self.create_line(*a, *b, fill=back_col)

        # Body-axis arrows, mid-body, labelled with the IMU axis names:
        # body +Y (sensor -y) is IMU +X, body +Z (sensor +x) is IMU +Z.
        for label, d, base in (("+X", (0, -1, 0), t["AXIS_Y"]), ("+Z", (1, 0, 0), t["AXIS_Z"])):
            dv = self._to_view(R, d)
            facing = dv[0] * c[0] + dv[1] * c[1] + dv[2] * c[2]
            colour = col(base, 0.0 if facing > -0.15 else 0.5)
            z0 = 0.03
            p0 = self._project(self._to_view(R, (d[0] * rad, d[1] * rad, z0)), cam)
            p1 = self._project(self._to_view(R, (d[0] * (rad + 0.2), d[1] * (rad + 0.2), z0)), cam)
            p2 = self._project(self._to_view(R, (d[0] * (rad + 0.27), d[1] * (rad + 0.27), z0)), cam)
            self.create_line(*p0, *p1, fill=colour, width=2, arrow="last", arrowshape=(8, 9, 3))
            self.create_text(*p2, text=label, fill=colour, font=self.fonts["label"])

        for a, b in front:
            self.create_line(*a, *b, fill=front_col, width=1.4)

        # IMU +Y runs along the airframe, out through the nose.
        self.create_text(*self._project(self._to_view(R, (0, 0, self.NOSE[-1][0] - 0.09)), cam),
                         text="+Y", fill=col(t["TEXT"]), font=self.fonts["label"])

        # Overlay: tilt readout, hints
        if fr is not None:
            self.create_text(16, 14, text=f"{tilt_deg(fr):5.1f}°", anchor="nw",
                             fill=t["TEXT"] if self.live else t["TEXT_DIM"],
                             font=self.fonts["master"])
            self.create_text(18, 50, text="tilt from vertical", anchor="nw",
                             fill=t["TEXT_DIM"], font=self.fonts["small"])
        self.create_text(12, self.h - 26, anchor="w", fill=t["TEXT_DIM"], font=self.fonts["small"],
                         text="Heading from the magnetometer (magnetic north)")
        self.create_text(12, self.h - 11, anchor="w", fill=t["TEXT_DIM"], font=self.fonts["small"],
                         text="Drag to orbit, double-click to face the +Z side again")

        if not self.live:
            msg = "Waiting for telemetry" if fr is None else "Telemetry lost"
            self.create_text(self.w / 2, self.h * 0.42, text=msg,
                             fill=t["TEXT"], font=self.fonts["reading"])
            self.create_text(self.w / 2, self.h * 0.42 + 26, fill=t["TEXT_DIM"],
                             font=self.fonts["small"], justify="center",
                             text="Connect to the flight firmware (pio run -e flight).\n"
                                  "The stream starts automatically while this tab is open.")


# ----------------------------------------------------------------------
# Tilt scope
# ----------------------------------------------------------------------


class TiltScope(tk.Canvas):
    """Which way the nose leans, with a short trail. Centre = what the PID
    is chasing.

    Seen from above, oriented like the 3D view's home camera: IMU +X to
    the right, IMU +Z toward you (down). A nose lean toward +X is a
    negative tip B, toward +Z a positive tip A.
    """

    SIZE = 204
    RANGE_DEG = 15.0

    def __init__(self, parent, theme, fonts):
        super().__init__(parent, width=self.SIZE, height=self.SIZE, bg=theme["WELL"],
                         highlightthickness=1, highlightbackground=theme["RULE"])
        self.t = theme
        self.fonts = fonts
        self._static()

    def _xy(self, tip_a, tip_b):
        c = self.SIZE / 2
        k = (self.SIZE / 2 - 14) / self.RANGE_DEG
        return c - tip_b * k, c + tip_a * k

    def _static(self):
        t, c = self.t, self.SIZE / 2
        for deg in (5, 10, 15):
            r = (self.SIZE / 2 - 14) * deg / self.RANGE_DEG
            self.create_oval(c - r, c - r, c + r, c + r, outline=mix(t["RULE"], t["WELL"], 0.2))
            self.create_text(c + r * 0.72 + 3, c + r * 0.72 + 3, text=f"{deg}°", anchor="nw",
                             fill=t["TEXT_DIM"], font=self.fonts["small"])
        # Coloured like the 3D arrows: IMU X is body Y, IMU Z is body Z.
        self.create_line(8, c, self.SIZE - 8, c, fill=mix(t["AXIS_Y"], t["WELL"], 0.6))
        self.create_line(c, 8, c, self.SIZE - 8, fill=mix(t["AXIS_Z"], t["WELL"], 0.6))
        self.create_text(self.SIZE - 6, c - 4, text="+X", anchor="se",
                         fill=t["AXIS_Y"], font=self.fonts["small"])
        self.create_text(c + 5, self.SIZE - 5, text="+Z", anchor="sw",
                         fill=t["AXIS_Z"], font=self.fonts["small"])

    def draw(self, recent, live):
        self.delete("dyn")
        if not recent:
            return
        t = self.t
        lim = self.RANGE_DEG

        def clamp(v):
            return max(-lim, min(lim, v))

        newest = recent[-1].t_ms
        trail = [fr for fr in recent if newest - fr.t_ms <= TRAIL_S * 1000]
        pts = []
        for fr in trail:
            pts.extend(self._xy(clamp(fr.tip_a), clamp(fr.tip_b)))
        # Older half of the trail fainter than the newer half
        half = (len(pts) // 4) * 2
        if half >= 2 and len(pts[:half + 2]) >= 4:
            self.create_line(*pts[:half + 2], fill=mix(t["TEXT"], t["WELL"], 0.75), tags="dyn")
        if len(pts[half:]) >= 4:
            self.create_line(*pts[half:], fill=mix(t["TEXT"], t["WELL"], 0.4), width=1.5, tags="dyn")

        fr = recent[-1]
        out = abs(fr.tip_a) > lim or abs(fr.tip_b) > lim
        x, y = self._xy(clamp(fr.tip_a), clamp(fr.tip_b))
        colour = t["LAMP_AMBER"] if out else t["TEXT"]
        if not live:
            colour = t["TEXT_DIM"]
        self.create_oval(x - 5, y - 5, x + 5, y + 5, fill=colour, outline="", tags="dyn")


# ----------------------------------------------------------------------
# Strip chart
# ----------------------------------------------------------------------


class StripChart(tk.Canvas):
    PAD_L, PAD_R, PAD_T, PAD_B = 46, 10, 24, 15

    def __init__(self, parent, theme, fonts, title, unit, series,
                 min_span, symmetric=True, fixed=None, limits=None, fmt="{:+.1f}"):
        super().__init__(parent, bg=theme["WELL"], highlightthickness=1,
                         highlightbackground=theme["RULE"], height=80, width=300)
        self.t = theme
        self.fonts = fonts
        self.title = title
        self.unit = unit
        self.series = series            # [(field, label, colour)]
        self.fmt = fmt                  # legend value format
        self.min_span = min_span
        self.symmetric = symmetric
        self.fixed = fixed
        self.limits = limits
        self.range = None
        self.w, self.h = 300, 80
        self.lines = [self.create_line(0, 0, 0, 0, fill=c, width=1.5, state="hidden")
                      for _, _, c in series]
        self.bind("<Configure>", self._on_resize)

    def _on_resize(self, e):
        self.w, self.h = e.width, e.height
        self.range = None
        self._static()

    def _static(self):
        self.delete("static")
        t = self.t
        x0, x1 = self.PAD_L, self.w - self.PAD_R
        y1 = self.h - self.PAD_B
        title = self.create_text(8, 5, text=self.title.upper(), anchor="nw", fill=t["TEXT_DIM"],
                                 font=self.fonts["eyebrow"], tags="static")
        self.create_text(self.bbox(title)[2] + 6, 4, text=self.unit, anchor="nw",
                         fill=mix(t["TEXT_DIM"], t["WELL"], 0.3), font=self.fonts["small"],
                         tags="static")
        for s in range(0, int(CHART_WINDOW_S) + 1, 2):
            x = x1 - (x1 - x0) * s / CHART_WINDOW_S
            self.create_line(x, self.PAD_T, x, y1, fill=mix(t["RULE"], t["WELL"], 0.55), tags="static")
            if s % 4 == 0:
                self.create_text(x, y1 + 2, text="now" if s == 0 else f"-{s}s", anchor="n",
                                 fill=t["TEXT_DIM"], font=self.fonts["small"], tags="static")
        self.tag_lower("static")

    def _pick_range(self, vals):
        if self.fixed:
            return self.fixed
        if not vals:
            lo, hi = (-self.min_span, self.min_span) if self.symmetric else (0, self.min_span)
        elif self.symmetric:
            m = max(self.min_span, max(abs(v) for v in vals) * 1.15)
            lo, hi = -m, m
        else:
            vlo, vhi = min(vals), max(vals)
            span = max(vhi - vlo, self.min_span) * 1.2
            mid = (vlo + vhi) / 2
            lo, hi = mid - span / 2, mid + span / 2
        step = nice_step(hi - lo)
        lo, hi = math.floor(lo / step) * step, math.ceil(hi / step) * step

        # Hysteresis — only rescale when the data leaves the current range
        # or shrinks well inside it, so the axis doesn't twitch every frame.
        if self.range and vals:
            plo, phi = self.range
            if plo <= min(vals) and max(vals) <= phi and (phi - plo) <= 2.5 * (hi - lo):
                return self.range
        return lo, hi

    def refresh(self, recent, live):
        self.delete("dyn")
        t = self.t
        x0, x1 = self.PAD_L, self.w - self.PAD_R
        y0, y1 = self.PAD_T, self.h - self.PAD_B
        if x1 - x0 < 20 or y1 - y0 < 10:
            return

        now = recent[-1].t_ms if recent else 0
        cols = [[getattr(fr, f) for fr in recent] for f, _, _ in self.series]
        vals = [v for col in cols for v in col if finite(v)]
        self.range = lo, hi = self._pick_range(vals)
        step = nice_step(hi - lo)

        def ymap(v):
            return y1 - (v - lo) / (hi - lo) * (y1 - y0)

        # Grid and y labels
        fmt = "{:.0f}" if step >= 1 else ("{:.1f}" if step >= 0.1 else "{:.2f}")
        k = 0
        v = math.ceil(lo / step - 1e-6) * step      # ticks on whole multiples of step
        while v <= hi + step * 0.01 and k < 12:
            y = ymap(v)
            zero = abs(v) < step * 0.01
            self.create_line(x0, y, x1, y, tags="dyn",
                             fill=mix(t["RULE"], t["WELL"], 0.1 if zero else 0.55))
            self.create_text(x0 - 6, y, text=fmt.format(v), anchor="e",
                             fill=t["TEXT_DIM"], font=self.fonts["small"], tags="dyn")
            v += step
            k += 1

        if self.limits:
            for lv in self.limits:
                y = ymap(lv)
                self.create_line(x0, y, x1, y, dash=(4, 3), tags="dyn",
                                 fill=mix(t["LAMP_AMBER"], t["WELL"], 0.45))

        # Series
        span_ms = CHART_WINDOW_S * 1000
        for line, col in zip(self.lines, cols):
            pts = []
            for fr, v in zip(recent, col):
                if finite(v):
                    pts.append(x1 - (now - fr.t_ms) / span_ms * (x1 - x0))
                    pts.append(max(y0, min(y1, ymap(v))))
            if len(pts) >= 4:
                self.coords(line, *pts)
                self.itemconfigure(line, state="normal")
            else:
                self.itemconfigure(line, state="hidden")
        # First series on top: it's the primary one (e.g. the estimate over raw baro)
        self.tag_raise("dyn")
        for line in reversed(self.lines):
            self.tag_raise(line)

        # Legend with latest values, right-aligned
        x = self.w - 10
        for (field, label, colour), col in reversed(list(zip(self.series, cols))):
            val = col[-1] if col else float("nan")
            txt = f"{label} {self.fmt.format(val)}" if finite(val) else f"{label}  —"
            item = self.create_text(x, 4, text=txt, anchor="ne", tags="dyn",
                                    fill=colour if live else t["TEXT_DIM"], font=self.fonts["mono"])
            bx0, _, _, _ = self.bbox(item)
            x = bx0 - 12


# ----------------------------------------------------------------------
# CSV recorder
# ----------------------------------------------------------------------


class CsvRecorder:
    def __init__(self):
        self.fh = None
        self.writer = None
        self.path = None
        self.rows = 0

    @property
    def active(self):
        return self.fh is not None

    def start(self, path):
        self.fh = open(path, "w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.fh)
        self.writer.writerow(("host_time_s",) + TLM_FIELDS)
        self.path = path
        self.rows = 0

    def write(self, fr, host_t):
        if not self.fh:
            return
        self.writer.writerow((f"{host_t:.4f}",) + tuple(fr))
        self.rows += 1
        if self.rows % 50 == 0:
            self.fh.flush()

    def stop(self):
        if self.fh:
            self.fh.close()
        self.fh = self.writer = None


# ----------------------------------------------------------------------
# The tab
# ----------------------------------------------------------------------

STATE_LAMP_KEY = {
    "IDLE": "LAMP_WHITE", "ARMED": "LAMP_AMBER", "POWERED": "LAMP_GREEN",
    "COAST": "LAMP_GREEN", "APOGEE": "LAMP_GREEN", "DESCENT": "LAMP_GREEN",
    "MAIN": "LAMP_GREEN", "LANDED": "LAMP_WHITE", "ABORT": "LAMP_RED",
}

# (label, flag bit, lamp colour key when lit)
FLAG_LAMPS = [
    ("IMU", F_IMU_VALID, "LAMP_GREEN"), ("BARO", F_BARO_OK, "LAMP_GREEN"),
    ("TVC", F_TVC_LIVE, "LAMP_WHITE"), ("ARM", F_ARM_SWITCH, "LAMP_AMBER"),
    ("PAD", F_PAD_REST, "LAMP_GREEN"), ("LNCH", F_LAUNCH_DET, "LAMP_WHITE"),
    ("GND", F_GROUND_CAP, "LAMP_WHITE"), ("MAIN", F_MAIN_FIRED, "LAMP_AMBER"),
]
# A dark IMU or BARO lamp is a fault, not just "off".
FLAG_DARK = {F_IMU_VALID: "LAMP_RED", F_BARO_OK: "LAMP_AMBER"}


class TelemetryPanel(tk.Frame):
    def __init__(self, parent, fonts, theme):
        super().__init__(parent, bg=theme["BEZEL"])
        self.t = theme
        self.fonts = fonts
        self.frames = collections.deque(maxlen=4000)
        self.rx_times = collections.deque(maxlen=400)
        self.last_rx = 0.0
        self.rejected = 0
        self.recorder = CsvRecorder()
        self._tick_n = 0
        self._readout_cache = {}

        mono = fonts["mono"].actual("family")
        self.f_value = tkfont.Font(family=mono, size=11, weight="bold")

        self._build()
        self.after(40, self._tick)

    # -- public API used by bench_gui -----------------------------------
    def push(self, fr):
        now = time.time()
        if self.frames and fr.t_ms + 1000 < self.frames[-1].t_ms:
            self.frames.clear()                 # device rebooted — time went backwards
        self.frames.append(fr)
        self.rx_times.append(now)
        self.last_rx = now
        self.recorder.write(fr, now)

    def note_rejected(self):
        self.rejected += 1

    def event(self, text, tag=None):
        if not text.strip(" =\t"):
            return                      # skip the ===== banners around FSM transitions
        self.ticker.configure(state="normal")
        self.ticker.insert("end", text + "\n", tag or ())
        lines = int(self.ticker.index("end-1c").split(".")[0])
        if lines > 60:
            self.ticker.delete("1.0", f"{lines - 60}.0")
        self.ticker.see("end")
        self.ticker.configure(state="disabled")

    def receiving(self, within_s=STALE_S):
        return self.last_rx and time.time() - self.last_rx < within_s

    def shutdown(self):
        self.recorder.stop()

    # -- layout ---------------------------------------------------------
    def _eyebrow(self, parent, text):
        tk.Label(parent, text=text.upper(), bg=self.t["BEZEL"], fg=self.t["TEXT_DIM"],
                 font=self.fonts["eyebrow"]).pack(anchor="w", pady=(0, 4))

    def _build(self):
        t = self.t
        self.columnconfigure(2, weight=1)
        self.rowconfigure(0, weight=1)

        # Left: 3D view over scope + link box
        left = tk.Frame(self, bg=t["BEZEL"])
        left.grid(row=0, column=0, sticky="ns", padx=(0, 14), pady=(10, 0))
        under = tk.Frame(left, bg=t["BEZEL"])
        under.pack(side="bottom", fill="x", pady=(10, 0))
        self._eyebrow(left, "attitude")
        self.view = AttitudeView(left, t, self.fonts)
        self.view.pack(fill="both", expand=True)
        scope_col = tk.Frame(under, bg=t["BEZEL"])
        scope_col.pack(side="left")
        self._eyebrow(scope_col, "tilt scope")
        self.scope = TiltScope(scope_col, t, self.fonts)
        self.scope.pack()

        box = tk.Frame(under, bg=t["BEZEL"])
        box.pack(side="left", fill="both", expand=True, padx=(14, 0))
        self._eyebrow(box, "link")
        self.link_lbl = tk.Label(box, text="no frames", bg=t["BEZEL"], fg=t["TEXT_DIM"],
                                 font=self.fonts["mono"], anchor="w", justify="left")
        self.link_lbl.pack(anchor="w")
        tk.Frame(box, bg=t["BEZEL"], height=14).pack()
        self._eyebrow(box, "recording")
        self.rec_btn = ttk.Button(box, text="Record CSV", width=16, command=self._toggle_record)
        self.rec_btn.pack(anchor="w")
        self.rec_lbl = tk.Label(box, text="Every frame received is saved\nwith the host clock time.",
                                bg=t["BEZEL"], fg=t["TEXT_DIM"], font=self.fonts["small"],
                                anchor="w", justify="left", wraplength=210)
        self.rec_lbl.pack(anchor="w", pady=(6, 0))

        # Middle: state + readouts
        mid = tk.Frame(self, bg=t["BEZEL"])
        mid.grid(row=0, column=1, sticky="ns", padx=(0, 14), pady=(10, 0))
        self._eyebrow(mid, "flight state")
        self.state_cv = tk.Canvas(mid, width=250, height=92, bg=t["WELL"],
                                  highlightthickness=1, highlightbackground=t["RULE"])
        self.state_cv.pack()
        tk.Frame(mid, bg=t["BEZEL"], height=6).pack()
        self._build_readouts(mid)

        # Right: charts + event ticker
        right = tk.Frame(self, bg=t["BEZEL"])
        right.grid(row=0, column=2, sticky="nsew", pady=(10, 0))
        AY, AZ, TX = t["AXIS_Y"], t["AXIS_Z"], t["TEXT"]
        self.charts = [
            StripChart(right, t, self.fonts, "attitude", "deg",
                       [("tip_a", "A", AY), ("tip_b", "B", AZ)], min_span=5),
            StripChart(right, t, self.fonts, "body rates", "deg/s",
                       [("gx", "spin", TX), ("gy", "X", AY), ("gz", "Z", AZ)], min_span=20),
            # Magnetometer's own axes, coloured like the IMU axes of the same
            # name — assumes the two chips are oriented alike on the board.
            StripChart(right, t, self.fonts, "magnetic field", "G",
                       [("mx", "X", AY), ("my", "Y", TX), ("mz", "Z", AZ)], min_span=0.2,
                       fmt="{:+.3f}"),
            StripChart(right, t, self.fonts, "tvc command", "deg",
                       [("pid_p", "pitch", AY), ("pid_y", "yaw", AZ)], min_span=6,
                       fixed=(-6, 6), limits=(-PID_LIMIT_DEG, PID_LIMIT_DEG)),
            StripChart(right, t, self.fonts, "altitude", "m",
                       [("alt_m", "est", TX), ("baro_alt_m", "baro", t["TEXT_DIM"])],
                       min_span=2, symmetric=False),
            StripChart(right, t, self.fonts, "vertical velocity", "m/s",
                       [("vel_ms", "est", TX)], min_span=2),
        ]
        for ch in self.charts:
            ch.pack(fill="both", expand=True, pady=(0, 6))

        self._eyebrow(right, "serial events")
        wrap = tk.Frame(right, bg=t["RULE"], padx=1, pady=1)
        wrap.pack(fill="x")
        self.ticker = tk.Text(wrap, height=4, bg=t["WELL"], fg=t["TEXT"], relief="flat",
                              font=self.fonts["console"], wrap="none", padx=8, pady=4,
                              state="disabled", cursor="arrow")
        self.ticker.pack(fill="x")
        for tag, key in (("pass", "LAMP_GREEN"), ("fail", "LAMP_RED"), ("warn", "LAMP_AMBER")):
            self.ticker.tag_configure(tag, foreground=t[key])
        self.ticker.tag_configure("meta", foreground=t["TEXT_DIM"])

    def _build_readouts(self, parent):
        t = self.t
        AY, AZ = t["AXIS_Y"], t["AXIS_Z"]
        groups = [
            ("attitude", [("tip_a", "Tip A", AY), ("tip_b", "Tip B", AZ), ("spin", "Spin", None)]),
            ("motion", [("alt", "Altitude", None), ("vel", "Velocity", None),
                        ("acc", "Accel", None), ("rate", "Rate", None),
                        ("mag", "Mag field", None)]),
            ("control", [("pid_p", "Pitch cmd", AY), ("pid_y", "Yaw cmd", AZ),
                         ("srv_p", "Pitch servo", AY), ("srv_y", "Yaw servo", AZ)]),
            ("system", [("press", "Pressure", None), ("temp", "Temp", None),
                        ("pack", "Pack", None), ("loop", "Loop time", None)]),
        ]
        self.readouts = {}
        for gi, (title, rows) in enumerate(groups):
            if gi:
                tk.Frame(parent, bg=t["BEZEL"], height=4).pack()
            self._eyebrow(parent, title)
            grid = tk.Frame(parent, bg=t["WELL"], highlightthickness=1, highlightbackground=t["RULE"])
            grid.pack(fill="x")
            grid.columnconfigure(1, weight=1)
            for r, (key, label, colour) in enumerate(rows):
                tk.Label(grid, text=label, bg=t["WELL"], fg=colour or t["TEXT_DIM"],
                         font=self.fonts["small"], anchor="w", width=11
                         ).grid(row=r, column=0, sticky="w", padx=(10, 0), pady=0)
                val = tk.Label(grid, text="—", bg=t["WELL"], fg=t["TEXT_DIM"],
                               font=self.f_value, anchor="e", width=8)
                val.grid(row=r, column=1, sticky="e")
                unit = tk.Label(grid, text="", bg=t["WELL"], fg=t["TEXT_DIM"],
                                font=self.fonts["small"], anchor="w", width=5)
                unit.grid(row=r, column=2, sticky="w", padx=(4, 8))
                self.readouts[key] = (val, unit)

    # -- actions --------------------------------------------------------
    def _toggle_record(self):
        if self.recorder.active:
            path, rows = self.recorder.path, self.recorder.rows
            self.recorder.stop()
            self.rec_btn.configure(text="Record CSV")
            self.rec_lbl.configure(text=f"Saved {rows:,} frames to\n{path}", fg=self.t["TEXT_DIM"])
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".csv", initialfile=time.strftime("tlm_%Y%m%d_%H%M%S.csv"),
            filetypes=[("CSV", "*.csv"), ("All files", "*.*")])
        if not path:
            return
        try:
            self.recorder.start(path)
        except OSError as exc:
            self.rec_lbl.configure(text=f"Could not open file: {exc}", fg=self.t["LAMP_RED"])
            return
        self.rec_btn.configure(text="Stop recording")

    # -- refresh loop ---------------------------------------------------
    def _tick(self):
        try:
            self._refresh()
        finally:
            self.after(40, self._tick)

    def _refresh(self):
        if not self.winfo_viewable():
            return
        self._tick_n += 1
        live = bool(self.receiving())
        latest = self.frames[-1] if self.frames else None

        # Last CHART_WINDOW_S of frames, oldest first
        recent = []
        if latest:
            cutoff = latest.t_ms - CHART_WINDOW_S * 1000
            for fr in reversed(self.frames):
                if fr.t_ms < cutoff:
                    break
                recent.append(fr)
            recent.reverse()

        self.view.set(latest, live)
        self.view.draw()
        self.scope.draw(recent, live)

        if self._tick_n % 2:          # charts and text at half rate
            return
        for ch in self.charts:
            ch.refresh(recent, live)
        self._draw_state(latest, live)
        self._update_readouts(latest, live)
        self._update_link(live)

    def _update_link(self, live):
        now = time.time()
        rate = sum(1 for ts in self.rx_times if now - ts <= 1.0)
        if not self.last_rx:
            txt = "no frames yet"
        elif not live:
            txt = f"lost {now - self.last_rx:4.1f} s ago"
        else:
            txt = f"{rate:3d} frames/s"
        self.link_lbl.configure(text=f"{txt}\n{self.rejected} rejected",
                                fg=self.t["TEXT"] if live else self.t["TEXT_DIM"])
        if self.recorder.active:
            self.rec_lbl.configure(text=f"Recording {self.recorder.rows:,} frames\n{self.recorder.path}",
                                   fg=self.t["LAMP_WHITE"])

    def _draw_state(self, fr, live):
        c, t = self.state_cv, self.t
        c.delete("all")
        w = 250
        if fr is None or not live:
            name, colour = ("NO DATA" if fr is None else "LOST"), t["LAMP_GREY"]
        else:
            name, colour = fr.state, t[STATE_LAMP_KEY.get(fr.state, "LAMP_WHITE")]
        c.create_rectangle(0, 0, 5, 92, fill=colour, outline="")
        c.create_text(18, 20, text=name, anchor="w", fill=colour if live else t["TEXT_DIM"],
                      font=self.fonts["master"])
        if fr is not None:
            up = fr.t_ms
            c.create_text(w - 12, 20, anchor="e", fill=t["TEXT_DIM"], font=self.fonts["mono"],
                          text=f"T+{up // 60000:02d}:{(up // 1000) % 60:02d}.{(up % 1000) // 100}")

        flags = fr.flags if fr is not None else 0
        for i, (label, bit, key) in enumerate(FLAG_LAMPS):
            col, row = i % 4, i // 4
            x, y = 18 + col * 58, 52 + row * 24
            on = bool(flags & bit) and live
            if on:
                lamp = t[key]
            elif live and bit in FLAG_DARK:
                lamp = t[FLAG_DARK[bit]]
            else:
                lamp = mix(t["LAMP_GREY"], t["WELL"], 0.3)
            lit = on or (live and bit in FLAG_DARK)
            if lit:
                c.create_oval(x - 6, y - 6, x + 6, y + 6, fill=mix(lamp, t["WELL"], 0.7), outline="")
            c.create_oval(x - 4, y - 4, x + 4, y + 4, fill=lamp, outline="")
            c.create_text(x + 9, y, text=label, anchor="w", font=self.fonts["small"],
                          fill=t["TEXT"] if lit else t["TEXT_DIM"])

    def _set(self, key, text, unit="", colour=None):
        val, unit_lbl = self.readouts[key]
        colour = colour or self.t["TEXT"]
        if self._readout_cache.get(key) != (text, unit, colour):
            val.configure(text=text, fg=colour)
            unit_lbl.configure(text=unit)
            self._readout_cache[key] = (text, unit, colour)

    def _update_readouts(self, fr, live):
        t = self.t
        if fr is None:
            return
        dim = None if live else t["TEXT_DIM"]

        def num(v, fmt):
            return fmt.format(v) if finite(v) else "—"

        self._set("tip_a", num(fr.tip_a, "{:+.1f}"), "deg", dim)
        self._set("tip_b", num(fr.tip_b, "{:+.1f}"), "deg", dim)
        self._set("spin", num(fr.spin, "{:+.0f}"), "deg", dim)
        self._set("alt", num(fr.alt_m, "{:.1f}"), "m", dim)
        self._set("vel", num(fr.vel_ms, "{:+.1f}"), "m/s", dim)
        acc = math.sqrt(fr.ax ** 2 + fr.ay ** 2 + fr.az ** 2)
        rate = math.sqrt(fr.gx ** 2 + fr.gy ** 2 + fr.gz ** 2)
        self._set("acc", num(acc, "{:.2f}"), "g", dim)
        self._set("rate", num(rate, "{:.0f}"), "deg/s", dim)
        field = math.sqrt(fr.mx ** 2 + fr.my ** 2 + fr.mz ** 2)
        lo, hi = MAG_OK_GAUSS
        self._set("mag", num(field, "{:.3f}"), "G",
                  dim or (None if not finite(field) or lo <= field <= hi else t["LAMP_AMBER"]))

        tvc_on = bool(fr.flags & F_TVC_LIVE)
        sat = t["LAMP_AMBER"]
        self._set("pid_p", num(fr.pid_p, "{:+.2f}"), "deg",
                  dim or (sat if abs(fr.pid_p) >= PID_LIMIT_DEG - 1e-3 else
                          (None if tvc_on else t["TEXT_DIM"])))
        self._set("pid_y", num(fr.pid_y, "{:+.2f}"), "deg",
                  dim or (sat if abs(fr.pid_y) >= PID_LIMIT_DEG - 1e-3 else
                          (None if tvc_on else t["TEXT_DIM"])))
        self._set("srv_p", num(fr.servo_p_us, "{:.0f}"), "µs", dim)
        self._set("srv_y", num(fr.servo_y_us, "{:.0f}"), "µs", dim)

        baro_dim = dim or (None if fr.flags & F_BARO_OK else t["LAMP_AMBER"])
        self._set("press", num(fr.press_hpa, "{:.2f}"), "hPa", baro_dim)
        self._set("temp", num(fr.temp_c, "{:.1f}"), "°C", baro_dim)
        if fr.flags & F_ARM_SWITCH:
            self._set("pack", num(fr.pack_v, "{:.2f}"), "V", dim)
        else:
            self._set("pack", "safe", "", t["TEXT_DIM"])

        loop_col = (t["LAMP_RED"] if fr.loop_us >= LOOP_BUDGET_US else
                    t["LAMP_AMBER"] if fr.loop_us >= 0.75 * LOOP_BUDGET_US else None)
        self._set("loop", f"{fr.loop_us:d}", "µs", dim or loop_col)


# ----------------------------------------------------------------------
# Demo — a synthesised bench wiggle and short flight, for --demo mode
# ----------------------------------------------------------------------


class DemoFlight:
    """Generates plausible $TLM lines on a 32 s loop. Not a simulation."""

    CYCLE = 32.0
    GROUND_HPA = 1009.6

    def __init__(self):
        self.prev = None
        self.spin = 0.0
        self.state = None
        self.boot = random.randint(4000, 9000)
        self.t_land = next(tc / 100 for tc in range(1640, 3200)
                           if self._kinematics(tc / 100)[2] == "LANDED")

    @staticmethod
    def _smooth(x):
        x = max(0.0, min(1.0, x))
        return x * x * (3 - 2 * x)

    def _kinematics(self, tc):
        """Altitude [m], velocity [m/s], state for a time within the cycle."""
        if tc < 10.0:
            return 0.0, 0.0, ("IDLE" if tc < 5.0 else "ARMED")
        if tc < 13.4:
            tau = tc - 10.0
            return 4.5 * tau * tau, 9.0 * tau, ("ARMED" if tc < 10.5 else "POWERED")
        h_bo, v_bo = 4.5 * 3.4 ** 2, 9.0 * 3.4
        t_ap = v_bo / 10.3
        if tc < 13.4 + t_ap:
            tau = tc - 13.4
            return h_bo + v_bo * tau - 5.15 * tau * tau, v_bo - 10.3 * tau, "COAST"
        h_ap = h_bo + v_bo * t_ap - 5.15 * t_ap * t_ap
        tau = tc - 13.4 - t_ap
        v = -9.0 * (1 - math.exp(-tau / 0.6))
        h = h_ap - 9.0 * (tau - 0.6 * (1 - math.exp(-tau / 0.6)))
        if h <= 0:
            return 0.0, 0.0, "LANDED"
        state = "APOGEE" if tau < 0.3 else "DESCENT" if tau < 0.8 else "MAIN"
        return h, v, state

    def _attitude(self, tc, state):
        sm = self._smooth
        if tc < 5.0:                                   # being handled on the bench
            env = math.sin(math.pi * tc / 5.0)
            return 10 * env * math.sin(1.4 * tc), 7 * env * math.sin(0.9 * tc + 0.5)
        if tc < 10.0:                                  # on the rail
            k = sm(tc - 5.0) * (1 - sm((tc - 9.0) / 1.0))
            return k * 0.3 * math.sin(0.9 * tc), k * 0.2 * math.sin(1.3 * tc + 1)
        if tc < 13.4:                                  # TVC catching a kick off the rail
            tau = tc - 10.0
            return (5.0 * math.exp(-1.3 * tau) * math.sin(7 * tau) + 0.5 * math.sin(2.1 * tau),
                    -3.5 * math.exp(-1.0 * tau) * math.sin(6 * tau) + 0.4 * math.sin(1.7 * tau))
        a0 = 0.5 * math.sin(2.1 * 3.4)
        b0 = 0.4 * math.sin(1.7 * 3.4)
        if state == "COAST":                           # unstable airframe, TVC off
            tau = tc - 13.4
            return a0 + 3.0 * tau, b0 + 1.5 * tau
        if state == "LANDED":                          # tip over onto the ground
            a_l, b_l = self._descent_attitude(self.t_land)
            k = sm((tc - self.t_land) / 0.8)
            return a_l + (3.0 - a_l) * k, b_l + (84.0 - b_l) * k
        return self._descent_attitude(tc)

    def _descent_attitude(self, tc):
        a0 = 0.5 * math.sin(2.1 * 3.4)
        b0 = 0.4 * math.sin(1.7 * 3.4)
        tau = max(0.0, tc - 16.37)
        a1, b1 = a0 + 8.9, b0 + 4.5
        k = self._smooth(tau / 2.0)
        return (a1 + (5 - a1) * k + 6 * k * math.sin(1.1 * tau + 0.5),
                b1 + (70 - b1) * k + 12 * k * math.sin(1.4 * tau))

    def transition(self):
        """(old, new) if the last line() changed state, else None."""
        return self._transition

    def line(self, t_s):
        tc = t_s % self.CYCLE
        alt, vel, state = self._kinematics(tc)
        self._transition = (self.state, state) if self.state and state != self.state else None
        self.state = state
        tip_a, tip_b = self._attitude(tc, state)

        prev = self.prev
        if prev is None or tc < prev[0]:
            self.spin = 0.0
            prev = (tc, tip_a, tip_b)
        dt = max(1e-3, tc - prev[0])
        rate_a = (tip_a - prev[1]) / dt
        rate_b = (tip_b - prev[2]) / dt
        spin_rate = (15.0 * min(1.0, (tc - 10.0) / 3.4) if 10.0 <= tc < 16.4 else
                     15.0 * math.exp(-(tc - 16.4) / 2) if 16.4 <= tc < 27 else 0.0)
        self.spin = (self.spin + spin_rate * dt + 180) % 360 - 180
        self.prev = (tc, tip_a, tip_b)

        q = euler_to_quat(tip_b, -tip_a, self.spin)
        R = quat_matrix(*q)
        noise = random.gauss

        # Specific force in body axes [g]
        if state == "POWERED" or (state == "ARMED" and tc >= 10.0):
            ax, ay, az = -1.92, 0.0, 0.0
        elif state in ("COAST", "APOGEE"):
            ax, ay, az = 0.05, 0.0, 0.0
        else:
            # At rest or in steady descent the IMU feels -gravity: up in NED.
            up_s = (-R[2][0], -R[2][1], -R[2][2])      # R^T (0, 0, -1)
            ax, ay, az = up_s[2], -up_s[1], up_s[0]    # sensor -> body axes

        # Earth's field (north and down, NED [G]) seen from the sensor,
        # then body axes, then IMU axes — the magnetometer is assumed to
        # be oriented like the IMU.
        B = (0.20, -0.02, 0.45)
        s = [sum(R[j][i] * B[j] for j in range(3)) for i in range(3)]   # R^T B
        bx, by, bz = s[2], -s[1], s[0]
        mx, my, mz = by, -bx, bz

        tvc = 10.0 <= tc < 13.4
        if tvc:
            pid_p = max(-5.0, min(5.0, -(0.34 * tip_a + 0.05 * rate_a)))
            pid_y = max(-5.0, min(5.0, -(0.34 * tip_b + 0.05 * rate_b)))
        else:
            pid_p = pid_y = 0.0

        flags = F_IMU_VALID
        if not (2.5 < tc < 3.3):
            flags |= F_BARO_OK
        if tvc:
            flags |= F_TVC_LIVE
        if tc >= 5.0:
            flags |= F_ARM_SWITCH
        if 7.0 <= tc < 10.5:
            flags |= F_PAD_REST
        if 10.0 <= tc < 10.5:
            flags |= F_LAUNCH_DET
        if 5.0 <= tc < 6.4:
            flags |= F_GROUND_CAP
        if tc >= 17.2:
            flags |= F_MAIN_FIRED

        baro_alt = alt + noise(0, 0.25)
        fr = Tlm(
            t_ms=self.boot + int(t_s * 1000), state=state, flags=flags,
            q0=q[0], q1=q[1], q2=q[2], q3=q[3],
            tip_a=tip_a, tip_b=tip_b, spin=self.spin,
            gx=spin_rate + noise(0, 0.3), gy=rate_a + noise(0, 0.3), gz=rate_b + noise(0, 0.3),
            ax=ax + noise(0, 0.01), ay=ay + noise(0, 0.01), az=az + noise(0, 0.01),
            mx=mx + noise(0, 0.002), my=my + noise(0, 0.002), mz=mz + noise(0, 0.002),
            alt_m=alt + noise(0, 0.05), vel_ms=vel + noise(0, 0.08), baro_alt_m=baro_alt,
            press_hpa=self.GROUND_HPA * (1 - baro_alt / 44330.0) ** 5.255,
            temp_c=24.6 - alt * 0.0065,
            pid_p=pid_p, pid_y=pid_y,
            servo_p_us=1200 + pid_p * 5.556, servo_y_us=1175 + pid_y * 5.556,
            pack_v=(7.86 + noise(0, 0.01)) if flags & F_ARM_SWITCH else 0.0,
            loop_us=int(noise(520 if tvc else 380, 25)),
        )
        return format_tlm(fr)
