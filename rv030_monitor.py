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

import ast
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

# ── ODrive error code tables (firmware 0.5.6) ─────────────────────────────────

_AXIS_ERRORS = {
    0x00001: ("INVALID_STATE",               "check calibration order"),
    0x00040: ("MOTOR_FAILED",                "see motor errors below"),
    0x00080: ("SENSORLESS_ESTIMATOR_FAILED", ""),
    0x00100: ("ENCODER_FAILED",              "see encoder errors below"),
    0x00200: ("CONTROLLER_FAILED",           "see controller errors below"),
    0x00800: ("WATCHDOG_TIMER_EXPIRED",      "watchdog not fed in time"),
    0x01000: ("MIN_ENDSTOP_PRESSED",         "min endstop triggered"),
    0x02000: ("MAX_ENDSTOP_PRESSED",         "max endstop triggered"),
    0x04000: ("ESTOP_REQUESTED",             "CAN e-stop message received"),
    0x20000: ("HOMING_WITHOUT_ENDSTOP",      "min endstop not enabled"),
    0x40000: ("OVER_TEMP",                   "see motor errors for details"),
    0x80000: ("UNKNOWN_POSITION",            "no valid position estimate"),
}

_MOTOR_ERRORS = {
    0x000000001: ("PHASE_RESISTANCE_OUT_OF_RANGE", "check motor lead connections"),
    0x000000002: ("PHASE_INDUCTANCE_OUT_OF_RANGE",  "check motor lead connections"),
    0x000000008: ("DRV_FAULT",                      "gate driver fault; check PSU ripple"),
    0x000000010: ("CONTROL_DEADLINE_MISSED",        ""),
    0x000000080: ("MODULATION_MAGNITUDE",           "bus voltage too low for requested current"),
    0x000000400: ("CURRENT_SENSE_SATURATION",       "increase requested_current_range"),
    0x000001000: ("CURRENT_LIMIT_VIOLATION",        "increase current_lim_margin"),
    0x000010000: ("MODULATION_IS_NAN",              ""),
    0x000020000: ("MOTOR_THERMISTOR_OVER_TEMP",     "motor too hot"),
    0x000040000: ("FET_THERMISTOR_OVER_TEMP",       "ODrive FETs too hot"),
    0x000080000: ("TIMER_UPDATE_MISSED",            ""),
    0x000100000: ("CURRENT_MEASUREMENT_UNAVAILABLE",""),
    0x000200000: ("CONTROLLER_FAILED",              "FOC controller failed"),
    0x000400000: ("I_BUS_OUT_OF_RANGE",             "DC current outside configured limits"),
    0x000800000: ("BRAKE_RESISTOR_DISARMED",        "send R to re-arm brake resistor"),
    0x001000000: ("SYSTEM_LEVEL",                   "check ODrive system-level errors"),
    0x002000000: ("BAD_TIMING",                     "main loop out of sync with motor loop"),
    0x004000000: ("UNKNOWN_PHASE_ESTIMATE",         "calibrate the encoder first"),
    0x008000000: ("UNKNOWN_PHASE_VEL",              ""),
    0x010000000: ("UNKNOWN_TORQUE",                 ""),
    0x020000000: ("UNKNOWN_CURRENT_COMMAND",        "check controller configuration"),
    0x040000000: ("UNKNOWN_CURRENT_MEASUREMENT",    ""),
    0x080000000: ("UNKNOWN_VBUS_VOLTAGE",           ""),
    0x100000000: ("UNKNOWN_VOLTAGE_COMMAND",        ""),
    0x200000000: ("UNKNOWN_GAINS",                  "run motor calibration"),
    0x400000000: ("CONTROLLER_INITIALIZING",        ""),
    0x800000000: ("UNBALANCED_PHASES",              ""),
}

_ENCODER_ERRORS = {
    0x001: ("UNSTABLE_GAIN",            ""),
    0x002: ("CPR_POLEPAIRS_MISMATCH",   "check CPR and pole_pairs settings"),
    0x004: ("NO_RESPONSE",              "check encoder wiring/power"),
    0x008: ("UNSUPPORTED_ENCODER_MODE", ""),
    0x010: ("ILLEGAL_HALL_STATE",       "add 22 nF caps on A/B/Z to GND"),
    0x020: ("INDEX_NOT_FOUND_YET",      "encoder has no index pin"),
    0x040: ("ABS_SPI_TIMEOUT",          "SPI timeout — check J3 wiring and series resistors"),
    0x080: ("ABS_SPI_COM_FAIL",         "SPI comm fail — check J3 pins 8/9/14"),
    0x100: ("ABS_SPI_NOT_READY",        "encoder not ready after power-up"),
    0x200: ("HALL_NOT_CALIBRATED_YET",  "run Hall polarity calibration"),
}

_CONTROLLER_ERRORS = {
    0x01: ("OVERSPEED",              "increase vel_limit in config"),
    0x02: ("INVALID_INPUT_MODE",     "check input_mode setting"),
    0x04: ("UNSTABLE_GAIN",          "reduce current_control_bandwidth"),
    0x08: ("INVALID_MIRROR_AXIS",    ""),
    0x10: ("INVALID_LOAD_ENCODER",   ""),
    0x20: ("INVALID_ESTIMATE",       ""),
    0x40: ("INVALID_CIRCULAR_RANGE", ""),
    0x80: ("SPINOUT_DETECTED",       "encoder slipping or wrong offset calibration"),
}

_SYSTEM_ERRORS = {
    0x01: ("CONTROL_ITERATION_MISSED", ""),
    0x02: ("DC_BUS_UNDER_VOLTAGE",      "check power supply voltage"),
    0x04: ("DC_BUS_OVER_VOLTAGE",       "add/check brake resistor"),
    0x08: ("DC_BUS_OVER_REGEN_CURRENT", "check brake resistor rating"),
    0x10: ("DC_BUS_OVER_CURRENT",       "reduce motor current limits"),
    0x20: ("BRAKE_DEADTIME_VIOLATION",  ""),
    0x40: ("BRAKE_DUTY_CYCLE_NAN",      ""),
    0x80: ("INVALID_BRAKE_RESISTANCE",  "check brake_resistance config value"),
}

_ERROR_TABLES = {
    "axis":       _AXIS_ERRORS,
    "motor":      _MOTOR_ERRORS,
    "encoder":    _ENCODER_ERRORS,
    "controller": _CONTROLLER_ERRORS,
    "system":     _SYSTEM_ERRORS,
}


def _decode_errors(err_str: str) -> list:
    """Parse the errors string from the T command and return decoded flag list.

    Input examples:
      "none"
      "{'axis': '0x100', 'encoder': '0x40'}"

    Returns a list of (component, flag_name, hint) tuples, empty when no errors.
    """
    if not err_str or err_str in ("none", "—", ""):
        return []
    try:
        d = ast.literal_eval(err_str)
        if not isinstance(d, dict):
            return [("?", err_str, "")]
    except Exception:
        return [("?", err_str, "")]

    result = []
    for comp, val_str in d.items():
        try:
            val = int(val_str, 16) if isinstance(val_str, str) else int(val_str)
        except (ValueError, TypeError):
            result.append((comp, str(val_str), ""))
            continue
        if val == 0:
            continue
        table = _ERROR_TABLES.get(comp, {})
        matched = False
        for bit, (name, hint) in sorted(table.items()):
            if val & bit:
                result.append((comp, name, hint))
                matched = True
        if not matched:
            result.append((comp, "0x{0:x}".format(val), "unknown error code"))
    return result


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
        decoded = _decode_errors(ax.errors)
        if not decoded:
            _put(win, row, 11, "none", OK)
            row += 1
        else:
            for comp, name, hint in decoded:
                flag_text = "{0}: {1}".format(comp, name)
                _put(win, row, 11, flag_text, ER)
                if hint:
                    _put(win, row, 11 + len(flag_text) + 1, "— " + hint, DM)
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
