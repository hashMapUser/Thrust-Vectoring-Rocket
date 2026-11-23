#!/usr/bin/env python3
"""
Keyboard front end for the nozzle travel-limit finder.

The finder itself runs on the Teensy (src/servo_limit_finder.cpp) and works
from any serial monitor: type the keys from its help, plus Enter if the
monitor sends whole lines. This script just makes that nicer: single
keypresses with no Enter, arrow keys, auto-detects the Teensy, prints the
results when you quit and leaves the servos limp.

    pio run -e servo_cal -t upload
    python src/find_servo_limits.py
    python src/find_servo_limits.py -p COM5

Keys: a/d or Left/Right jog, w/s or Up/Down step size, x switch axis,
[ ] mark the low/high stop, type 1350 + Enter to go to 1350 us,
c centre, l limp, r results, h help, q quit.

Close any serial monitor first, since only one program can hold the port.
Requires pyserial (already a PlatformIO dependency).
"""

import argparse
import os
import sys
import threading
import time

READY = "SERVOCAL READY"
ARROWS = {"LEFT": "a", "RIGHT": "d", "UP": "w", "DOWN": "s"}


def guess_port():
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    for p in ports:
        if p.vid == 0x16C0 or "teensy" in f"{p.description} {p.manufacturer or ''}".lower():
            return p.device
    return ports[0].device if ports else None


class Keys:
    """Single keypresses without Enter. Arrow keys come back as LEFT/RIGHT/UP/DOWN."""
    ARROWS_NT = {"K": "LEFT", "M": "RIGHT", "H": "UP", "P": "DOWN"}
    ARROWS_VT = {"[D": "LEFT", "[C": "RIGHT", "[A": "UP", "[B": "DOWN"}

    def __enter__(self):
        if os.name != "nt":
            import termios
            import tty
            self.fd = sys.stdin.fileno()
            self.saved = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        return self

    def __exit__(self, *exc):
        if os.name != "nt":
            import termios
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)

    def get(self):
        if os.name == "nt":
            import msvcrt
            ch = msvcrt.getwch()
            if ch in ("\x00", "\xe0"):
                return self.ARROWS_NT.get(msvcrt.getwch(), "")
            return ch
        ch = sys.stdin.read(1)
        if ch == "\x1b":
            return self.ARROWS_VT.get(sys.stdin.read(2), "")
        return ch


def main():
    ap = argparse.ArgumentParser(description="Keyboard front end for the nozzle travel-limit finder.")
    ap.add_argument("-p", "--port", help="serial port (default: auto-detect the Teensy)")
    args = ap.parse_args()

    try:
        import serial
    except ImportError:
        sys.exit("pyserial is not installed. Run: pip install pyserial")

    port = args.port or guess_port()
    if not port:
        sys.exit("No serial ports found. Is the Teensy plugged in?")
    try:
        ser = serial.Serial(port, 115200, timeout=0.1)
    except Exception as exc:
        sys.exit(f"Couldn't open {port}: {exc}\n"
                 f"(Close any serial monitor that has it open: only one program can hold the port.)")

    # Print everything the Teensy says, once it has identified itself.
    ready = threading.Event()

    def reader():
        buf = b""
        while True:
            try:
                buf += ser.read(256)
            except Exception:
                return
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", "replace").rstrip("\r")
                if line.startswith(READY):
                    ready.set()
                if ready.is_set():
                    print(line, flush=True)

    threading.Thread(target=reader, daemon=True).start()

    if not ready.wait(1.5):
        # Missed the connect banner. '?' is the only thing sent before the
        # Teensy identifies itself: the flight, finned and bench builds all
        # ignore it, so keys never reach the wrong firmware.
        ser.write(b"?")
        if not ready.wait(1.5):
            ser.close()
            sys.exit(f"No '{READY}' from {port}. Flash the limit-finder firmware first:\n"
                     f"    pio run -e servo_cal -t upload")

    typed = 0   # digits of a pulse width being typed (the Teensy doesn't echo them)
    try:
        with Keys() as keys:
            while True:
                k = keys.get()
                if k in ("q", "Q", "\x03"):
                    break
                k = ARROWS.get(k, k)
                if len(k) != 1 or not k.isascii():
                    continue
                if k == "\r":
                    k = "\n"
                if k.isdigit():
                    typed += 1
                    print(k, end="", flush=True)
                elif k in ("\b", "\x7f"):
                    if typed:
                        typed -= 1
                        print("\b \b", end="", flush=True)
                elif typed:
                    print(flush=True)    # finish the typed number's line
                    typed = 0
                ser.write(k.encode("ascii"))
    except KeyboardInterrupt:
        pass
    finally:
        print()
        try:
            ser.write(b"r")    # results + #defines
            time.sleep(0.5)
            ser.write(b"l")    # limp (closing the port does this too)
            time.sleep(0.1)
        except Exception:
            pass
        ser.close()


if __name__ == "__main__":
    main()
