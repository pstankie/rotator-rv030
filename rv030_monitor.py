#!/usr/bin/env python3
"""rv030_monitor.py — ncurses real-time status monitor for the RV030 rotctld server.

Connects to the rotctld TCP server (default localhost:4533), polls every second,
and displays:
  - AZ/EL position in degrees and raw encoder turns
  - Axis state (IDLE / CLOSED_LOOP_CONTROL / HOMING / …)
  - AS5048A SPI encoder description (AZ) and incremental encoder (EL)
  - ODrive axis error codes
  - Endstop GPIO levels and triggered state

Usage:
    python rv030_monitor.py [host] [port]
    python rv030_monitor.py              # defaults: localhost 4533

Keys:
    q   quit
    r   force immediate refresh

Note: the 'T' (status) command is sent to the server once per poll cycle.
The server's T handler used to print [STATUS] to its console on every call —
that print has been removed from the server to avoid flooding its output.
"""
from __future__ import annotations

import curses
import re
import socket
import sys
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime

DEFAULT_HOST = "localhost"
DEFAULT_PORT = 4533
POLL_INTERVAL = 1.0
SOCKET_TIMEOUT = 3.0

_AXIS_STATES = {
    0:  "UNDEFINED",
    1:  "IDLE",
    2:  "STARTUP_SEQUENCE",
    3:  "FULL_CALIBRATION_SEQUENCE",
    4:  "MOTOR_CALIBRATION",
    6:  "ENCODER_INDEX_SEARCH",
    7:  "ENCODER_OFFSET_CALIBRATION",
    8:  "CLOSED_LOOP_CONTROL",
    9:  "LOCKIN_SPIN",
    10: "ENCODER_DIR_FIND",
    11: "HOMING",
    12: "ENCODER_HALL_POLARITY_CAL",
    13: "ENCODER_HALL_PHASE_CAL",
}


def _state_name(code: int) -> str:
    return _AXIS_STATES.get(code, "STATE_{0}".format(code))


@dataclass
class AxisInfo:
    label: str
    state_code: int = -1
    errors: str = "—"
    endstop_str: str = "—"
    pos_deg: float = 0.0
    raw_turns: float = 0.0
    encoder_desc: str = ""
    deg_per_turn: float = 1.0


@dataclass
class MonitorState:
    az: AxisInfo = field(default_factory=lambda: AxisInfo("AZ"))
    el: AxisInfo = field(default_factory=lambda: AxisInfo("EL"))
    connected: bool = False
    conn_error: str = ""
    last_update: str = ""
    model: str = ""
    psu: str = ""
    brake_resistor: str = ""


# ── TCP helpers ───────────────────────────────────────────────────────────────

def _recv_until_rprt(sock: socket.socket) -> list:
    """Read lines from sock until a line starting with 'RPRT' is received."""
    buf = b""
    lines = []
    while True:
        chunk = sock.recv(4096)
        if not chunk:
            raise ConnectionError("Server closed connection")
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            text = line.decode(errors="ignore").strip()
            if text:
                lines.append(text)
            if text.startswith("RPRT"):
                return lines
    return lines


def _send(sock: socket.socket, cmd: str) -> list:
    sock.sendall((cmd + "\n").encode())
    return _recv_until_rprt(sock)


# ── Response parsers ──────────────────────────────────────────────────────────

def _parse_dump(lines: list, st: MonitorState) -> None:
    """Parse 'd' (dump_state) response."""
    for line in lines:
        if line.startswith("Model:"):
            st.model = line.split(":", 1)[1].strip()
        elif line.startswith("PSU:"):
            st.psu = line.split(":", 1)[1].strip()
        elif line.startswith("Brake resistor:"):
            st.brake_resistor = line.split(":", 1)[1].strip()
        elif line.startswith("Position:"):
            m = re.search(r"AZ=([\d.\-]+)\s+EL=([\d.\-]+)", line)
            if m:
                st.az.pos_deg = float(m.group(1))
                st.el.pos_deg = float(m.group(2))
        elif line.startswith("Raw turns:"):
            m = re.search(r"AZ=([\d.\-]+)\s+EL=([\d.\-]+)", line)
            if m:
                st.az.raw_turns = float(m.group(1))
                st.el.raw_turns = float(m.group(2))
        elif line.startswith("AZ deg/turn:"):
            try:
                st.az.deg_per_turn = float(line.split(":")[1].strip())
            except ValueError:
                pass
        elif line.startswith("EL deg/turn:"):
            try:
                st.el.deg_per_turn = float(line.split(":")[1].strip())
            except ValueError:
                pass
        elif line.startswith("Encoder AZ:"):
            st.az.encoder_desc = line.split(":", 1)[1].strip()
        elif line.startswith("Encoder EL:"):
            st.el.encoder_desc = line.split(":", 1)[1].strip()


def _parse_status(lines: list, st: MonitorState) -> None:
    """Parse 'T' (status) response.

    Expected line format:
      AZ axis1: state=8 errors=none endstop=GPIO7 enabled=True raw=0 triggered=False
    """
    for line in lines:
        m_label = re.match(r"(AZ|EL)\s+axis\d+:", line)
        if not m_label:
            continue
        ax = st.az if m_label.group(1) == "AZ" else st.el

        m = re.search(r"state=(-?\d+)", line)
        if m:
            ax.state_code = int(m.group(1))

        # errors field: "none" or "{'axis': '0x1', ...}"
        m = re.search(r"errors=(none|\{[^}]*\})", line)
        if m:
            ax.errors = m.group(1)

        # endstop: everything after "endstop="
        m = re.search(r"endstop=(.*)", line)
        if m:
            ax.endstop_str = m.group(1).strip()


# ── Background poll thread ────────────────────────────────────────────────────

def _poll_loop(
    host: str,
    port: int,
    shared: MonitorState,
    lock: threading.Lock,
    stop: threading.Event,
    wake: threading.Event,
) -> None:
    sock = None
    while not stop.is_set():
        try:
            if sock is None:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(SOCKET_TIMEOUT)
                s.connect((host, port))
                sock = s
                with lock:
                    shared.connected = True
                    shared.conn_error = ""

            fresh = MonitorState()
            fresh.connected = True
            _parse_dump(_send(sock, "d"), fresh)
            _parse_status(_send(sock, "T"), fresh)
            fresh.last_update = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

            with lock:
                shared.az            = fresh.az
                shared.el            = fresh.el
                shared.connected     = True
                shared.conn_error    = ""
                shared.last_update   = fresh.last_update
                shared.model         = fresh.model
                shared.psu           = fresh.psu
                shared.brake_resistor = fresh.brake_resistor

        except Exception as exc:
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass
                sock = None
            with lock:
                shared.connected  = False
                shared.conn_error = str(exc)

        # Sleep for POLL_INTERVAL, but wake early if 'r' was pressed
        wake.wait(timeout=POLL_INTERVAL)
        wake.clear()

    if sock:
        try:
            sock.close()
        except Exception:
            pass


# ── Curses rendering ──────────────────────────────────────────────────────────

# Color pair indices
_CP_HEADER = 1   # bold cyan   — section titles
_CP_OK     = 2   # green       — connected, no errors, CL control
_CP_ERR    = 3   # bold red    — errors, disconnected
_CP_WARN   = 4   # yellow      — triggered endstop, non-CL state
_CP_LABEL  = 5   # bold white  — field labels
_CP_VALUE  = 6   # white       — normal values
_CP_DIM    = 7   # dim white   — secondary info


def _init_colors() -> None:
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(_CP_HEADER, curses.COLOR_CYAN,    -1)
    curses.init_pair(_CP_OK,     curses.COLOR_GREEN,   -1)
    curses.init_pair(_CP_ERR,    curses.COLOR_RED,     -1)
    curses.init_pair(_CP_WARN,   curses.COLOR_YELLOW,  -1)
    curses.init_pair(_CP_LABEL,  curses.COLOR_WHITE,   -1)
    curses.init_pair(_CP_VALUE,  curses.COLOR_WHITE,   -1)
    curses.init_pair(_CP_DIM,    curses.COLOR_WHITE,   -1)


def _put(win, row: int, col: int, text: str, attr: int = curses.A_NORMAL) -> None:
    h, w = win.getmaxyx()
    if row < 0 or row >= h - 1 or col < 0 or col >= w:
        return
    text = text[:w - col]
    try:
        win.addstr(row, col, text, attr)
    except curses.error:
        pass


def _draw(win, st: MonitorState, host: str, port: int) -> None:
    win.erase()
    h, w = win.getmaxyx()

    H  = curses.color_pair(_CP_HEADER) | curses.A_BOLD
    OK = curses.color_pair(_CP_OK)
    ER = curses.color_pair(_CP_ERR)    | curses.A_BOLD
    WA = curses.color_pair(_CP_WARN)
    LB = curses.color_pair(_CP_LABEL)  | curses.A_BOLD
    VA = curses.color_pair(_CP_VALUE)
    DM = curses.color_pair(_CP_DIM)    | curses.A_DIM

    row = 0
    sep = "─" * min(w - 1, 72)

    # ── Title bar ─────────────────────────────────────────────
    conn_label = "● CONNECTED" if st.connected else "✗ DISCONNECTED"
    conn_attr  = OK if st.connected else ER
    title = " RV030 Rotor Monitor  {0}:{1}  ".format(host, port)
    _put(win, row, 0, title, H)
    _put(win, row, len(title), conn_label, conn_attr)
    row += 1
    _put(win, row, 0, sep, DM)
    row += 1

    # ── Per-axis blocks ───────────────────────────────────────
    for ax in (st.az, st.el):
        sname      = _state_name(ax.state_code)
        is_cl      = ax.state_code == 8
        is_homing  = ax.state_code == 11
        state_attr = OK if is_cl else (WA if is_homing or ax.state_code == 1 else ER)
        has_errors = ax.errors not in ("none", "—", "")
        triggered  = "triggered=True" in ax.endstop_str

        _put(win, row, 0, " {0}  axis{1}".format(
            ax.label,
            1 if ax.label == "AZ" else 0), H)
        row += 1

        _put(win, row, 2, "State    ", LB)
        _put(win, row, 11, sname, state_attr)
        row += 1

        pos_str   = "{0:>10.4f}°".format(ax.pos_deg)
        turns_str = "raw {0:.6f} turns".format(ax.raw_turns)
        dpt_str   = "{0:.1f}°/turn".format(ax.deg_per_turn)
        _put(win, row, 2,  "Position ", LB)
        _put(win, row, 11, pos_str, VA)
        _put(win, row, 23, turns_str, DM)
        _put(win, row, 45, dpt_str, DM)
        row += 1

        if ax.encoder_desc:
            _put(win, row, 2,  "Encoder  ", LB)
            _put(win, row, 11, ax.encoder_desc, DM)
            row += 1

        _put(win, row, 2, "Errors   ", LB)
        _put(win, row, 11, ax.errors, ER if has_errors else OK)
        row += 1

        _put(win, row, 2, "Endstop  ", LB)
        _put(win, row, 11, ax.endstop_str, WA if triggered else VA)
        row += 1

        row += 1  # blank between axes

    # ── System info ───────────────────────────────────────────
    _put(win, row, 0, sep, DM)
    row += 1
    if st.model:
        _put(win, row, 1, st.model, DM)
        row += 1
    if st.psu:
        info = "PSU: {0}".format(st.psu)
        if st.brake_resistor:
            info += "    Brake: {0}".format(st.brake_resistor)
        _put(win, row, 1, info, DM)
        row += 1

    # ── Status / error bar ────────────────────────────────────
    if not st.connected and st.conn_error:
        _put(win, row, 1, "Error: {0}".format(st.conn_error), ER)
        row += 1

    # ── Footer ────────────────────────────────────────────────
    _put(win, row, 0, sep, DM)
    row += 1
    footer = " Updated: {0}    [r] refresh  [q] quit".format(
        st.last_update if st.last_update else "—")
    _put(win, row, 0, footer, DM)

    win.refresh()


# ── Main ──────────────────────────────────────────────────────────────────────

def _run(stdscr, host: str, port: int) -> None:
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(200)
    _init_colors()

    shared = MonitorState()
    lock   = threading.Lock()
    stop   = threading.Event()
    wake   = threading.Event()

    thread = threading.Thread(
        target=_poll_loop,
        args=(host, port, shared, lock, stop, wake),
        daemon=True,
    )
    thread.start()

    try:
        while True:
            key = stdscr.getch()
            if key in (ord("q"), ord("Q"), 27):   # q / Esc
                break
            if key in (ord("r"), ord("R"), curses.KEY_F5):
                wake.set()

            with lock:
                snapshot = MonitorState(
                    az=shared.az,
                    el=shared.el,
                    connected=shared.connected,
                    conn_error=shared.conn_error,
                    last_update=shared.last_update,
                    model=shared.model,
                    psu=shared.psu,
                    brake_resistor=shared.brake_resistor,
                )
            _draw(stdscr, snapshot, host, port)
    finally:
        stop.set()
        wake.set()
        thread.join(timeout=2)


def main() -> None:
    host = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_HOST
    port = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_PORT
    curses.wrapper(_run, host, port)


if __name__ == "__main__":
    main()
