#!/usr/bin/env python3
"""
Live sensor status dashboard for the TVC flight computer.

Reads the $HLTH frames emitted by health_emit_frame() over USB serial and
renders them as a continuously updating terminal dashboard.

    python3 tools/health_monitor.py                  # auto-detect the Teensy
    python3 tools/health_monitor.py -p /dev/ttyACM0
    python3 tools/health_monitor.py --replay capture.txt

Only dependency is pyserial (already required by PlatformIO's monitor).
Replay mode needs nothing at all.

Frame format produced by the firmware:

    $HLTH,<t_ms>,<state>,NAME=st:flags:consec:fails:age_ms,...*XX

where XX is an XOR checksum over every character between $ and *.
"""

import argparse
import os
import sys
import time
from dataclasses import dataclass, field

# ----------------------------------------------------------------------
# Must mirror include/health.h
# ----------------------------------------------------------------------

STATE_NAMES = {0: "N/FIT", 1: "OK", 2: "DEGRADED", 3: "FAILED", 4: "UNKNOWN"}

FLAG_BITS = [
    (0x10, "I", "init-fail"),
    (0x08, "N", "never-good"),
    (0x01, "S", "stale"),
    (0x02, "R", "out-of-range"),
    (0x04, "U", "pad-unstable"),
    (0x20, "C", "recovering"),
]

# Channels the firmware reports as not fitted on the current board revision.
# Kept here only so the dashboard can explain *why* they are dark.
KNOWN_NOTES = {
    "MAG":   "not fitted this flight",
    "FLASH": "GD25Q128 miswired - unusable",
    "PYRO1": "H5 - no continuity telemetry",
    "PYRO2": "H5 - no continuity telemetry",
    "BATT":  "no firmware use this revision",
    "BARO":  "invalid read = no new sample (P_DA)",
}

# ANSI
RESET, BOLD, DIM = "\033[0m", "\033[1m", "\033[2m"
GREEN, YELLOW, RED, GREY, CYAN = (
    "\033[32m", "\033[33m", "\033[31m", "\033[90m", "\033[36m",
)
STATE_COLOR = {0: GREY, 1: GREEN, 2: YELLOW, 3: RED, 4: RED}

USE_COLOR = sys.stdout.isatty() and os.environ.get("NO_COLOR") is None


def c(text, color):
    return f"{color}{text}{RESET}" if USE_COLOR else text


# ----------------------------------------------------------------------
# Parsing
# ----------------------------------------------------------------------


@dataclass
class Channel:
    name: str
    state: int = 4
    flags: int = 0
    consec: int = 0
    fails: int = 0
    age_ms: int = 0


@dataclass
class Frame:
    t_ms: int = 0
    flight_state: str = "?"
    channels: "list[Channel]" = field(default_factory=list)


def checksum(body: str) -> int:
    ck = 0
    for ch in body:
        ck ^= ord(ch)
    return ck


def parse_frame(line: str):
    """Return a Frame, or None if the line is not a valid health frame."""
    line = line.strip()
    if not line.startswith("$") or "*" not in line:
        return None

    body, _, cks_txt = line[1:].rpartition("*")
    try:
        if checksum(body) != int(cks_txt[:2], 16):
            return None
    except ValueError:
        return None

    fields = body.split(",")
    if len(fields) < 3 or fields[0] != "HLTH":
        return None

    frame = Frame()
    try:
        frame.t_ms = int(fields[1])
    except ValueError:
        return None
    frame.flight_state = fields[2]

    for tok in fields[3:]:
        if "=" not in tok:
            continue
        name, _, rest = tok.partition("=")
        parts = rest.split(":")
        if len(parts) != 5:
            continue
        try:
            frame.channels.append(
                Channel(name, *(int(p) for p in parts))
            )
        except ValueError:
            continue

    return frame if frame.channels else None


# ----------------------------------------------------------------------
# Rendering
# ----------------------------------------------------------------------


def flags_str(flags: int) -> str:
    return "".join(letter if flags & bit else "-" for bit, letter, _ in FLAG_BITS)


def flags_words(flags: int) -> str:
    words = [word for bit, _, word in FLAG_BITS if flags & bit]
    return ", ".join(words)


def render(frame: Frame, link_age_s: float, frames: int, bad: int) -> str:
    up = frame.t_ms
    clock = f"T+{up // 60000:02d}:{(up // 1000) % 60:02d}.{up % 1000:03d}"

    critical_bad = []
    for ch in frame.channels:
        if ch.name == "IMU" and ch.state != 1:
            critical_bad.append("IMU " + STATE_NAMES.get(ch.state, "?"))
        if ch.name == "BARO" and ch.state in (3, 4):
            critical_bad.append("BARO " + STATE_NAMES.get(ch.state, "?"))

    go = not critical_bad
    verdict = c(" GO ", GREEN + BOLD) if go else c(" NO-GO ", RED + BOLD)
    reason = "all critical sensors nominal" if go else "; ".join(critical_bad)

    out = []
    out.append(c("╔" + "═" * 66 + "╗", CYAN))
    out.append(
        c("║", CYAN)
        + f" {BOLD if USE_COLOR else ''}SENSOR STATUS{RESET if USE_COLOR else ''}"
          f"   {clock}   FSM: {frame.flight_state:<8}".ljust(75 if USE_COLOR else 66)
        + c("║", CYAN)
    )
    out.append(c("╠" + "═" * 66 + "╣", CYAN))
    out.append(f"  FLIGHT-CRITICAL: {verdict}  {DIM if USE_COLOR else ''}{reason}{RESET if USE_COLOR else ''}")
    out.append("")
    out.append(f"  {'CHANNEL':<8}{'STATUS':<10}{'AGE':>8}  {'CONSEC':>7}{'FAILS':>8}  {'FLAGS':<7} NOTE")
    out.append("  " + "─" * 64)

    for ch in frame.channels:
        color = STATE_COLOR.get(ch.state, RED)
        status = c(f"{STATE_NAMES.get(ch.state, '?'):<10}", color)
        name = c(f"{ch.name:<8}", GREY if ch.state == 0 else BOLD)

        age = "-" if ch.state == 0 else f"{ch.age_ms} ms"
        consec = "-" if ch.state == 0 else str(ch.consec)
        fails = "-" if ch.state == 0 else str(ch.fails)

        note = flags_words(ch.flags) or KNOWN_NOTES.get(ch.name, "")
        note = c(note, GREY) if note else ""

        out.append(
            f"  {name}{status}{age:>8}  {consec:>7}{fails:>8}  "
            f"{flags_str(ch.flags):<7} {note}"
        )

    out.append("  " + "─" * 64)

    link = "live" if link_age_s < 2.0 else c(f"STALE {link_age_s:.1f}s", RED)
    out.append(
        f"  link: {link}   frames: {frames}   rejected: {bad}   "
        + c("Ctrl-C to exit", GREY)
    )
    out.append(c("╚" + "═" * 66 + "╝", CYAN))
    out.append(
        c("  flags: I=init-fail N=never-good S=stale R=out-of-range U=pad-unstable C=recovering", GREY)
    )
    return "\n".join(out)


# ----------------------------------------------------------------------
# Sources
# ----------------------------------------------------------------------


def autodetect_port():
    try:
        from serial.tools import list_ports
    except ImportError:
        return None
    for p in list_ports.comports():
        desc = f"{p.description} {p.manufacturer or ''}".lower()
        if "teensy" in desc or "usb serial" in desc or (p.vid == 0x16C0):
            return p.device
    ports = list(list_ports.comports())
    return ports[0].device if ports else None


def serial_lines(port, baud):
    try:
        import serial
    except ImportError:
        sys.exit("pyserial not installed.  pip install pyserial")

    with serial.Serial(port, baud, timeout=0.2) as ser:
        buf = b""
        while True:
            chunk = ser.read(256)
            if chunk:
                buf += chunk
                while b"\n" in buf:
                    raw, _, buf = buf.partition(b"\n")
                    yield raw.decode("ascii", "replace")
            else:
                yield None   # tick, so the display can age the link


def replay_lines(path, speed):
    with open(path, "r", encoding="ascii", errors="replace") as fh:
        for line in fh:
            yield line
            time.sleep(0.5 / max(speed, 0.01))


# ----------------------------------------------------------------------


def main():
    ap = argparse.ArgumentParser(description="Live TVC sensor status dashboard")
    ap.add_argument("-p", "--port", help="serial port (auto-detected if omitted)")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("--replay", help="replay $HLTH frames from a captured text file")
    ap.add_argument("--speed", type=float, default=1.0, help="replay speed multiplier")
    ap.add_argument("--raw", action="store_true", help="print frames instead of drawing")
    args = ap.parse_args()

    if args.replay:
        source = replay_lines(args.replay, args.speed)
        src_name = args.replay
    else:
        port = args.port or autodetect_port()
        if not port:
            sys.exit("No serial port found. Pass one with -p.")
        source = serial_lines(port, args.baud)
        src_name = f"{port} @ {args.baud}"

    frames = bad = 0
    last_frame = None
    last_rx = time.time()

    if not args.raw:
        sys.stdout.write("\033[2J\033[?25l")   # clear, hide cursor

    try:
        for line in source:
            if line is None:
                pass
            elif line.strip().startswith("$"):
                parsed = parse_frame(line)
                if parsed:
                    frames += 1
                    last_frame = parsed
                    last_rx = time.time()
                else:
                    bad += 1
            elif args.raw and line.strip():
                print(line.rstrip())

            if args.raw:
                if last_frame and line and line.strip().startswith("$"):
                    print(line.rstrip())
                continue

            if last_frame:
                sys.stdout.write("\033[H")   # home, overwrite in place
                sys.stdout.write(render(last_frame, time.time() - last_rx, frames, bad))
                sys.stdout.write("\n" + c(f"  source: {src_name}", GREY) + "\033[K\n")
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        if not args.raw:
            sys.stdout.write("\033[?25h\n")   # restore cursor


if __name__ == "__main__":
    main()