#!/usr/bin/env python3
from __future__ import annotations

import asyncio
import concurrent.futures
import re
import time
from dataclasses import dataclass

import odrive
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    AXIS_STATE_HOMING,
    AXIS_STATE_MOTOR_CALIBRATION,
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
    CONTROL_MODE_POSITION_CONTROL,
    CONTROL_MODE_VELOCITY_CONTROL,
    CONTROL_MODE_TORQUE_CONTROL,
    INPUT_MODE_PASSTHROUGH,
    ENCODER_MODE_SPI_ABS_AMS,
)


@dataclass
class RotorConfig:
    listen_host: str = "0.0.0.0"
    listen_port: int = 4533

    # AZ encoder is AS5048A on the OUTPUT shaft (after 60:1 worm).
    # 1 encoder turn = 360° of antenna rotation, so deg_per_turn = 360.
    # All AZ velocity/accel limits are in OUTPUT turns/s (÷60 vs motor-shaft values).
    az_deg_per_turn: float = 360.0
    el_deg_per_turn: float = 6.0

    az_min_deg: float = 0.0
    az_max_deg: float = 360.0
    el_min_deg: float = 0.0
    el_max_deg: float = 100.0

    az_wrap_report: bool = True

    # AZ in output turns/s: 0.033 ≈ 12°/s (same physical speed as the previous 2.0 motor turns/s)
    az_max_turns_per_s: float = 0.033
    el_max_turns_per_s: float = 2.0

    zero_on_startup: bool = True
    az_zero_offset_deg: float = 0.0
    el_zero_offset_deg: float = 0.0

    enable_custom_homing_command: bool = True

    enable_endstops: bool = True
    home_on_startup: bool = True

    el_home_gpio_num: int = 8
    az_home_gpio_num: int = 7
    az_home_active_high: bool = True
    el_home_active_high: bool = True
    az_home_offset_deg: float = 0.0
    el_home_offset_deg: float = 0.0
    # AZ homing speed in output turns/s: ≈5°/s for gentle endstop approach
    az_home_search_turns_s: float = 0.014
    el_home_search_turns_s: float = 0.4
    home_debounce_ms: float = 10.0

    model_name: str = "ODrive v3.6 AZ/EL Rotor"
    backend_name: str = "python-odrive-rotctld-fw056"

    psu_voltage_v: float = 48.0
    psu_max_current_a: float = 30.0
    brake_resistor_ohm: float = 2.0
    brake_resistor_power_w: float = 50.0

    apply_runtime_config_on_startup: bool = False
    auto_calibrate_on_startup: bool = True
    calibration_timeout_s: float = 60.0
    clear_errors_before_calibration: bool = True

    motor_current_lim_a: float = 12.0
    motor_current_lim_margin_a: float = 3.0
    requested_current_range_a: float = 20.0

    # AZ in output turns/s (÷60 vs motor-shaft encoder values)
    az_vel_limit_turns_s: float = 0.033
    az_accel_limit_turns_s2: float = 0.033
    az_decel_limit_turns_s2: float = 0.033

    el_vel_limit_turns_s: float = 2.0
    el_accel_limit_turns_s2: float = 2.0
    el_decel_limit_turns_s2: float = 2.0

    # Physical axis wiring: which ODrive axis index drives AZ and EL
    az_axis_idx: int = 1
    el_axis_idx: int = 0

    # AS5048A SPI CS pin (any free GPIO except 1/2 which are UART, and 7/8 used by endstops)
    # GPIO 4 = J3 pin 14 — free and matches the ODrive docs example
    az_spi_cs_gpio_pin: int = 4


CFG = RotorConfig()
_PUNCT_RE = re.compile(r"^[^\w\s\\\?\_#]")


def parse_erp_prefix(line: str):
    line = line.rstrip("\r\n")
    if not line:
        return None, line
    if _PUNCT_RE.match(line):
        return line[0], line[1:].lstrip()
    return None, line


def erp_sep(ch: str) -> str:
    return "\n" if ch == "+" else ch


def long_to_short(cmd: str) -> str:
    return {
        r"\get_pos": "p",
        r"\set_pos": "P",
        r"\move": "M",
        r"\stop": "S",
        r"\park": "K",
        r"\get_info": "i",
        r"\dump_state": "d",
        r"\recover": "R",
        r"\homing": "H",
        r"\status": "T",
        r"\escape": "E",
        r"\escape_pulse": "EP",
        r"\escape_torque": "ET",
    }.get(cmd, cmd)


def fmt_rprt(code: int, sep: str) -> str:
    return "RPRT {0}{1}".format(code, sep)


class ODriveRotor:
    def __init__(self, cfg: RotorConfig):
        self.cfg = cfg
        self.odrv = None
        self.az_offset_turns = 0.0
        self.el_offset_turns = 0.0
        self._last_reported_errors: dict = {}  # axis_idx -> last printed error dict (for dedup)

    def connect(self) -> None:
        print("Finding ODrive...")
        self.odrv = odrive.find_any()
        print("Connected to ODrive.")

    def configure_az_spi_encoder_and_save(self) -> None:
        """One-time setup: configure AS5048A SPI absolute encoder on the AZ axis and
        save to ODrive flash. The ODrive will reboot after save — re-run the script
        to start normally afterwards.

        Wiring — all signals are on the J3 connector (20-pin header on ODrive v3.6):

          AS5048A VDD  → 3.3V   J3 pin 1   (VCC)
          AS5048A GND  → GND    J3 pin 2   (GND)
          AS5048A CLK  → SCK    J3 pin 8   (SPI_SCK)
          AS5048A MISO → MISO   J3 pin 9   (SPI_MISO)
          AS5048A CSn  → GPIO4  J3 pin 14  (az_spi_cs_gpio_pin, free from UART/endstop GPIOs)
          AS5048A MOSI → leave unconnected, or tie to 3.3V to save a wire (AMS encoders only)

        Series resistors: 20–50 Ω on CLK (most noise-sensitive), 100 Ω on MISO/CSn,
        placed close to the ODrive side of the cable.
        """
        az = self._axis(self.cfg.az_axis_idx)
        az.encoder.config.abs_spi_cs_gpio_pin = self.cfg.az_spi_cs_gpio_pin
        az.encoder.config.mode = ENCODER_MODE_SPI_ABS_AMS
        az.encoder.config.cpr = 2 ** 14   # AS5048A: 14-bit = 16384 counts/rev
        az.encoder.config.use_index = False
        print("[INFO] AS5048A SPI encoder configured on AZ axis{0} (CPR=16384, CS=GPIO{1}).".format(
            self.cfg.az_axis_idx, self.cfg.az_spi_cs_gpio_pin))
        print("[INFO] Saving to flash — ODrive will reboot...")
        self.odrv.save_configuration()
        print("[INFO] Done. Re-run the script to start normally.")

    def clear_errors_if_supported(self) -> None:
        try:
            self.odrv.clear_errors()
            self._last_reported_errors.clear()
            print("Cleared ODrive errors.")
        except Exception:
            pass

    def _get_axis_errors(self, axis_idx: int) -> dict:
        """Return a dict of non-zero error codes for the given axis."""
        ax = self._axis(axis_idx)
        candidates = {
            "axis":       lambda: int(ax.error),
            "motor":      lambda: int(ax.motor.error),
            "encoder":    lambda: int(ax.encoder.error),
            "controller": lambda: int(ax.controller.error),
        }
        result = {}
        for name, getter in candidates.items():
            try:
                v = getter()
                if v != 0:
                    result[name] = hex(v)
            except Exception:
                pass
        return result

    def print_errors_if_any(self) -> bool:
        """Print errors for both axes to the server console. Deduplicates — only prints when errors change."""
        found = False
        for label, i in (("AZ", self.cfg.az_axis_idx), ("EL", self.cfg.el_axis_idx)):
            errs = self._get_axis_errors(i)
            if errs != self._last_reported_errors.get(i):
                self._last_reported_errors[i] = errs
                if errs:
                    print("[ERROR] {0} axis{1} error codes: {2}".format(label, i, errs))
                else:
                    print("[INFO] {0} axis{1} errors cleared.".format(label, i))
            if errs:
                found = True
        return found

    def recover_from_errors(self) -> None:
        """Print errors, clear them, re-arm brake resistor if needed. Leaves axes IDLE.
        Use 'M' (jog) immediately after to move away from any hard stop."""
        had_errors = self.print_errors_if_any()
        if had_errors:
            # Re-arm brake resistor before clearing if it was disarmed
            brake_disarmed = any(
                errs.get("motor") == hex(0x4000000)
                for _, i in (("AZ", self.cfg.az_axis_idx), ("EL", self.cfg.el_axis_idx))
                for errs in [self._get_axis_errors(i)]
            )
            if brake_disarmed:
                print("[INFO] Brake resistor was disarmed — re-arming...")
                try:
                    self.odrv.config.enable_brake_resistor = False
                    self.odrv.config.enable_brake_resistor = True
                    print("[INFO] Brake resistor re-armed.")
                except Exception as exc:
                    print("[WARN] Could not re-arm brake resistor: {0}".format(exc))

            self.clear_errors_if_supported()
            print("[INFO] Errors cleared. Axes left IDLE — use 'M' to jog away from hard stop.")
        else:
            print("[INFO] No errors found on axes.")


    def _safe_set_attr(self, obj, attr: str, value) -> None:
        try:
            setattr(obj, attr, value)
        except Exception:
            pass

    def configure_endstops(self) -> None:
        if not self.cfg.enable_endstops:
            return

        for axis_idx, gpio_num, active_high, offset_deg, homing_speed in (
            (
                self.cfg.az_axis_idx,
                self.cfg.az_home_gpio_num,
                self.cfg.az_home_active_high,
                self.cfg.az_home_offset_deg,
                self.cfg.az_home_search_turns_s,
            ),
            (
                self.cfg.el_axis_idx,
                self.cfg.el_home_gpio_num,
                self.cfg.el_home_active_high,
                self.cfg.el_home_offset_deg,
                self.cfg.el_home_search_turns_s,
            ),
        ):
            ax = self._axis(axis_idx)
            sw = ax.min_endstop
            self._safe_set_attr(sw.config, "gpio_num", int(gpio_num))
            self._safe_set_attr(sw.config, "enabled", True)
            self._safe_set_attr(sw.config, "is_active_high", bool(active_high))
            self._safe_set_attr(sw.config, "offset", float(offset_deg))
            self._safe_set_attr(sw.config, "debounce_ms", float(self.cfg.home_debounce_ms))
            self._safe_set_attr(sw.config, "pullup", False)
            self._safe_set_attr(ax.controller.config, "homing_speed", float(homing_speed))

            try:
                ax.max_endstop.config.enabled = False
            except Exception:
                pass

    def _wait_for_axis_idle(self, axis_idx: int, timeout_s: float | None = None) -> None:
        timeout = self.cfg.calibration_timeout_s if timeout_s is None else timeout_s
        ax = self._axis(axis_idx)
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                if int(ax.current_state) == AXIS_STATE_IDLE:
                    # Idle reached — but check it wasn't caused by an error
                    errs = self._get_axis_errors(axis_idx)
                    if errs:
                        print("[ERROR] Axis{0} went IDLE with errors: {1}".format(axis_idx, errs))
                        raise RuntimeError(
                            "Axis{0} stopped with errors during homing: {1}".format(axis_idx, errs)
                        )
                    return
            except RuntimeError:
                raise
            except Exception:
                pass
            # Periodically check for error flags without waiting for IDLE
            try:
                errs = self._get_axis_errors(axis_idx)
                if errs:
                    print("[ERROR] Axis{0} error during homing: {1}".format(axis_idx, errs))
                    raise RuntimeError(
                        "Axis{0} error during homing: {1}".format(axis_idx, errs)
                    )
            except RuntimeError:
                raise
            except Exception:
                pass
            time.sleep(0.05)
        print("[ERROR] Axis{0} did not return to IDLE within {1:.0f}s".format(axis_idx, timeout))
        raise TimeoutError("Axis{0} did not return to IDLE after {1:.0f}s".format(axis_idx, timeout))

    def home_axis(self, axis_idx: int) -> None:
        ax = self._axis(axis_idx)
        print("Homing axis{0}...".format(axis_idx))
        ax.requested_state = AXIS_STATE_HOMING
        self._wait_for_axis_idle(axis_idx, timeout_s=max(self.cfg.calibration_timeout_s, 45.0))

    def home_both_axes(self) -> None:
        if self.cfg.clear_errors_before_calibration:
            self.clear_errors_if_supported()
        self.configure_endstops()
        # Home both axes simultaneously to cut total homing time in half
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
            fut_el = pool.submit(self.home_axis, self.cfg.el_axis_idx)
            fut_az = pool.submit(self.home_axis, self.cfg.az_axis_idx)
            # Collect results; re-raise the first exception if any
            for fut in concurrent.futures.as_completed([fut_el, fut_az]):
                fut.result()
        self.set_zero_here()
        print("Applied software zero at home switches.")

    def apply_runtime_config(self) -> None:
        odrv0 = self.odrv
        az = self._axis(self.cfg.az_axis_idx)
        el = self._axis(self.cfg.el_axis_idx)

        try:
            az.controller.config.vel_limit = self.cfg.az_vel_limit_turns_s
        except Exception:
            pass
        try:
            az.controller.config.accel_limit = self.cfg.az_accel_limit_turns_s2
            az.controller.config.decel_limit = self.cfg.az_decel_limit_turns_s2
        except Exception:
            pass

        try:
            el.controller.config.vel_limit = self.cfg.el_vel_limit_turns_s
        except Exception:
            pass
        try:
            el.controller.config.accel_limit = self.cfg.el_accel_limit_turns_s2
            el.controller.config.decel_limit = self.cfg.el_decel_limit_turns_s2
        except Exception:
            pass

        for ax in (az, el):
            for name, value in (
                ("current_lim", self.cfg.motor_current_lim_a),
                ("current_lim_margin", self.cfg.motor_current_lim_margin_a),
                ("requested_current_range", self.cfg.requested_current_range_a),
            ):
                try:
                    setattr(ax.motor.config, name, value)
                except Exception:
                    pass

    def calibrate_axis(self, axis_idx: int) -> None:
        ax = self._axis(axis_idx)
        print("Calibrating axis{0}: motor calibration...".format(axis_idx))
        ax.requested_state = AXIS_STATE_MOTOR_CALIBRATION
        time.sleep(self.cfg.calibration_timeout_s)
        print("Calibrating axis{0}: encoder offset calibration...".format(axis_idx))
        ax.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        time.sleep(self.cfg.calibration_timeout_s)

    def calibrate_both_axes(self) -> None:
        if self.cfg.clear_errors_before_calibration:
            self.clear_errors_if_supported()
        self.calibrate_axis(self.cfg.az_axis_idx)
        self.calibrate_axis(self.cfg.el_axis_idx)

    def _axis(self, idx: int):
        return getattr(self.odrv, "axis{0}".format(idx))

    def _set_mode_pos(self, axis_idx: int) -> None:
        ax = self._axis(axis_idx)
        ax.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        try:
            ax.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        except Exception:
            pass
        ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def _set_mode_vel(self, axis_idx: int, initial_vel: float = 0.0) -> None:
        ax = self._axis(axis_idx)
        ax.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        try:
            ax.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        except Exception:
            pass
        # Set velocity BEFORE enabling closed-loop so the motor begins moving
        # immediately on enable rather than briefly holding the current position.
        try:
            ax.controller.input_vel = float(initial_vel)
        except Exception:
            try:
                ax.controller.vel_setpoint = float(initial_vel)
            except Exception:
                pass
        ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def _az_turns_to_deg(self, turns: float) -> float:
        return turns * self.cfg.az_deg_per_turn

    def _el_turns_to_deg(self, turns: float) -> float:
        return turns * self.cfg.el_deg_per_turn

    def _az_deg_to_turns(self, deg: float) -> float:
        return deg / self.cfg.az_deg_per_turn

    def _el_deg_to_turns(self, deg: float) -> float:
        return deg / self.cfg.el_deg_per_turn

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))

    def set_zero_here(self) -> None:
        az_turns_now = float(self._axis(self.cfg.az_axis_idx).encoder.pos_estimate)
        el_turns_now = float(self._axis(self.cfg.el_axis_idx).encoder.pos_estimate)
        self.az_offset_turns = az_turns_now - self._az_deg_to_turns(self.cfg.az_zero_offset_deg)
        self.el_offset_turns = el_turns_now - self._el_deg_to_turns(self.cfg.el_zero_offset_deg)

    def get_pos_deg(self):
        az_turns = float(self._axis(self.cfg.az_axis_idx).encoder.pos_estimate)
        el_turns = float(self._axis(self.cfg.el_axis_idx).encoder.pos_estimate)

        az_deg = self._az_turns_to_deg(az_turns - self.az_offset_turns)
        el_deg = self._el_turns_to_deg(el_turns - self.el_offset_turns)

        if self.cfg.az_wrap_report:
            az_deg = az_deg % 360.0

        return az_deg, el_deg

    def _set_brake_resistor(self, enabled: bool) -> None:
        try:
            self.odrv.config.enable_brake_resistor = enabled
            print("[INFO] Brake resistor {0}.".format("enabled" if enabled else "DISABLED"))
        except Exception as exc:
            print("[WARN] Could not set brake_resistor enabled={0}: {1}".format(enabled, exc))

    def stop(self) -> None:
        for i in (self.cfg.az_axis_idx, self.cfg.el_axis_idx):
            ax = self._axis(i)
            try:
                ax.controller.input_vel = 0.0
            except Exception:
                try:
                    ax.controller.vel_setpoint = 0.0
                except Exception:
                    pass
            ax.requested_state = AXIS_STATE_IDLE
        # Re-enable brake resistor in case it was disabled for an escape maneuver
        self._set_brake_resistor(True)

    def escape_move(self, direction: int, speed: int) -> None:
        """Emergency jog: disables brake resistor check so the axis can move off a hard stop.
        Send S (stop) afterwards — stop will re-enable the brake resistor."""
        print("[WARN] ESCAPE: disabling brake resistor check for hard-stop recovery.")
        self._set_brake_resistor(False)
        self.clear_errors_if_supported()

        speed_pct = 50 if speed == -1 else int(self._clamp(speed, 1, 100))
        az_vel = (speed_pct / 100.0) * self.cfg.az_max_turns_per_s
        el_vel = (speed_pct / 100.0) * self.cfg.el_max_turns_per_s

        if direction == 8:
            self._set_mode_vel(self.cfg.az_axis_idx, initial_vel=-az_vel)
            print("[INFO] AZ escape jog: {0:.3f} turns/s".format(-az_vel))
        elif direction == 16:
            self._set_mode_vel(self.cfg.az_axis_idx, initial_vel=+az_vel)
            print("[INFO] AZ escape jog: +{0:.3f} turns/s".format(az_vel))
        if direction == 2:
            self._set_mode_vel(self.cfg.el_axis_idx, initial_vel=+el_vel)
            print("[INFO] EL escape jog: +{0:.3f} turns/s".format(el_vel))
        elif direction == 4:
            self._set_mode_vel(self.cfg.el_axis_idx, initial_vel=-el_vel)
            print("[INFO] EL escape jog: {0:.3f} turns/s".format(-el_vel))

    def set_pos_deg(self, az_deg: float, el_deg: float) -> None:
        # Check for errors and refuse rather than blindly clearing — clearing and re-sending
        # the same position target will just re-trigger the same error.
        errs_az = self._get_axis_errors(self.cfg.az_axis_idx)
        errs_el = self._get_axis_errors(self.cfg.el_axis_idx)
        if errs_az or errs_el:
            if errs_az:
                print("[ERROR] AZ axis{0} errors before set_pos: {1}".format(self.cfg.az_axis_idx, errs_az))
            if errs_el:
                print("[ERROR] EL axis{0} errors before set_pos: {1}".format(self.cfg.el_axis_idx, errs_el))
            raise RuntimeError(
                "Axis errors present — send 'R' (recover) or jog away with 'M' before setting position"
            )

        az_deg = self._clamp(az_deg, self.cfg.az_min_deg, self.cfg.az_max_deg)
        el_deg = self._clamp(el_deg, self.cfg.el_min_deg, self.cfg.el_max_deg)

        az_turns = self.az_offset_turns + self._az_deg_to_turns(az_deg)
        el_turns = self.el_offset_turns + self._el_deg_to_turns(el_deg)

        self._set_mode_pos(self.cfg.az_axis_idx)
        self._set_mode_pos(self.cfg.el_axis_idx)

        try:
            self._axis(self.cfg.az_axis_idx).controller.input_pos = float(az_turns)
        except Exception:
            self._axis(self.cfg.az_axis_idx).controller.pos_setpoint = float(az_turns)

        try:
            self._axis(self.cfg.el_axis_idx).controller.input_pos = float(el_turns)
        except Exception:
            self._axis(self.cfg.el_axis_idx).controller.pos_setpoint = float(el_turns)

    def move(self, direction: int, speed: int) -> None:
        # Auto-recover from error state before moving
        errs_az = self._get_axis_errors(self.cfg.az_axis_idx)
        errs_el = self._get_axis_errors(self.cfg.el_axis_idx)
        if errs_az or errs_el:
            if errs_az:
                print("[ERROR] AZ axis{0} errors before move: {1}".format(self.cfg.az_axis_idx, errs_az))
            if errs_el:
                print("[ERROR] EL axis{0} errors before move: {1}".format(self.cfg.el_axis_idx, errs_el))
            self.clear_errors_if_supported()
            print("[INFO] Errors cleared — proceeding with move command.")

        speed_pct = 50 if speed == -1 else int(self._clamp(speed, 1, 100))
        az_vel = (speed_pct / 100.0) * self.cfg.az_max_turns_per_s
        el_vel = (speed_pct / 100.0) * self.cfg.el_max_turns_per_s

        if direction == 8:
            self._set_mode_vel(self.cfg.az_axis_idx, initial_vel=-az_vel)

        elif direction == 16:
            self._set_mode_vel(self.cfg.az_axis_idx, initial_vel=+az_vel)

        if direction == 2:
            self._set_mode_vel(self.cfg.el_axis_idx, initial_vel=+el_vel)

        elif direction == 4:
            self._set_mode_vel(self.cfg.el_axis_idx, initial_vel=-el_vel)

    def park(self) -> None:
        self.set_pos_deg(0.0, 0.0)

    def homing(self) -> None:
        self.home_both_axes()

    def _read_gpio(self, gpio_num: int) -> bool | None:
        """Read a single GPIO pin level. Returns None if unavailable."""
        # fw 0.5.x: odrv.get_gpio_states() returns a bitmask; bit (N-1) = GPIO N
        try:
            states = self.odrv.get_gpio_states()
            return bool((int(states) >> (int(gpio_num) - 1)) & 1)
        except Exception:
            pass
        # Older fallback: odrv.gpioN attribute
        try:
            return bool(getattr(self.odrv, "gpio{0}".format(int(gpio_num))))
        except Exception:
            return None

    def get_status(self) -> str:
        """Return a human-readable status string: axis states, errors, and endstop levels."""
        lines = []
        for label, i in (("AZ", self.cfg.az_axis_idx), ("EL", self.cfg.el_axis_idx)):
            ax = self._axis(i)
            try:
                state = int(ax.current_state)
            except Exception:
                state = -1
            errs = self._get_axis_errors(i)
            err_str = str(errs) if errs else "none"

            # Endstop GPIO level
            try:
                gpio_num = int(ax.min_endstop.config.gpio_num)
                active_high = bool(ax.min_endstop.config.is_active_high)
                enabled = bool(ax.min_endstop.config.enabled)
                raw = self._read_gpio(gpio_num)
                if raw is None:
                    endstop_str = "GPIO{0} enabled={1} raw=unavailable".format(gpio_num, enabled)
                else:
                    triggered = raw if active_high else not raw
                    endstop_str = "GPIO{0} enabled={1} raw={2} triggered={3}".format(
                        gpio_num, enabled, int(raw), triggered
                    )
            except Exception as exc:
                endstop_str = "unavailable ({0})".format(exc)

            lines.append("{0} axis{1}: state={2} errors={3} endstop={4}".format(
                label, i, state, err_str, endstop_str
            ))
        return "\n".join(lines)

    def escape_torque(self, axis_idx: int, torque_nm: float, duration_s: float) -> None:
        """Apply a raw torque command (Nm) for duration_s seconds, bypassing velocity/position loops.
        Does NOT require a working encoder — useful when encoder sync is lost after a stall.
        Disables brake resistor for the duration; stop() re-enables it."""
        label = "AZ" if axis_idx == self.cfg.az_axis_idx else "EL"
        print("[WARN] Torque escape: {0} axis{1} torque={2:.3f}Nm for {3:.0f}ms".format(
            label, axis_idx, torque_nm, duration_s * 1000))
        self._set_brake_resistor(False)
        self.clear_errors_if_supported()
        ax = self._axis(axis_idx)
        try:
            ax.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
            ax.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        except Exception as exc:
            print("[WARN] Could not set torque mode: {0}".format(exc))
        try:
            ax.controller.input_torque = float(torque_nm)
        except Exception:
            try:
                ax.controller.torque_setpoint = float(torque_nm)
            except Exception as exc:
                print("[WARN] Could not set torque setpoint: {0}".format(exc))
        ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL


        try:
            serial_hex = "{0:012X}".format(int(self.odrv.serial_number))
        except Exception:
            serial_hex = "UNKNOWN"
        try:
            hw = "v{0}.{1}".format(int(self.odrv.hw_version_major), int(self.odrv.hw_version_minor))
        except Exception:
            hw = "unknown"
        try:
            fw = "{0}.{1}.{2}".format(
                int(self.odrv.fw_version_major),
                int(self.odrv.fw_version_minor),
                int(self.odrv.fw_version_revision),
            )
        except Exception:
            fw = "unknown"
        return "{0} | ODrive HW {1} | FW {2} | SN {3}".format(self.cfg.model_name, hw, fw, serial_hex)

    def dump_state(self) -> str:
        az, el = self.get_pos_deg()
        lines = [
            "Backend: {0}".format(self.cfg.backend_name),
            "Model: {0}".format(self.cfg.model_name),
            "AZ range deg: {0} .. {1}".format(self.cfg.az_min_deg, self.cfg.az_max_deg),
            "EL range deg: {0} .. {1}".format(self.cfg.el_min_deg, self.cfg.el_max_deg),
            "AZ deg/turn: {0}".format(self.cfg.az_deg_per_turn),
            "EL deg/turn: {0}".format(self.cfg.el_deg_per_turn),
            "Position: AZ={0:.6f} EL={1:.6f}".format(az, el),
            "Raw turns: AZ={0:.6f} EL={1:.6f}".format(
                float(self._axis(self.cfg.az_axis_idx).encoder.pos_estimate),
                float(self._axis(self.cfg.el_axis_idx).encoder.pos_estimate),
            ),
            "PSU: {0:.1f} V / {1:.1f} A".format(self.cfg.psu_voltage_v, self.cfg.psu_max_current_a),
            "Brake resistor: {0:.2f} ohm / {1:.1f} W".format(
                self.cfg.brake_resistor_ohm, self.cfg.brake_resistor_power_w
            ),
            "Firmware note: startup calibration enabled in this script",
            "Firmware note: min_endstop homing configured for AZ/EL home switches",
            "Encoder AZ: AS5048A SPI absolute on output shaft (CPR=16384, axis{0})".format(self.cfg.az_axis_idx),
            "Encoder EL: incremental on motor shaft (axis{0})".format(self.cfg.el_axis_idx),
            "Axis mapping: AZ=axis{0} / EL=axis{1}".format(self.cfg.az_axis_idx, self.cfg.el_axis_idx),
            "Home config: AZ GPIO {0} / EL GPIO {1}".format(self.cfg.az_home_gpio_num, self.cfg.el_home_gpio_num),
        ]
        return "\n".join(lines)


class RotctldServer:
    def __init__(self, rotor: ODriveRotor, cfg: RotorConfig):
        self.rotor = rotor
        self.cfg = cfg

    def _reply_default(self, text: str) -> bytes:
        if not text.endswith("\n"):
            text += "\n"
        return text.encode()

    def _reply_erp(self, erp_prefix: str, records: list[str], rprt: int) -> bytes:
        sep = erp_sep(erp_prefix)
        return (sep.join(records) + sep + fmt_rprt(rprt, sep)).encode()

    async def handle(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        while True:
            raw = await reader.readline()
            if not raw:
                break

            line = raw.decode(errors="ignore")
            erp_prefix, cmdline = parse_erp_prefix(line)
            if not cmdline:
                continue

            parts = cmdline.strip().split()
            cmd = parts[0]
            args = parts[1:]

            if cmd.startswith("\\"):
                cmd = long_to_short(cmd)

            try:
                if cmd == "p":
                    az, el = self.rotor.get_pos_deg()
                    if erp_prefix is None:
                        writer.write(self._reply_default("{0:.6f}\n{1:.6f}".format(az, el)))
                    else:
                        writer.write(self._reply_erp(
                            erp_prefix,
                            ["get_pos:", "Azimuth: {0:.6f}".format(az), "Elevation: {0:.6f}".format(el)],
                            0,
                        ))

                elif cmd == "P":
                    if len(args) < 2:
                        raise RuntimeError("Need: P <az_deg> <el_deg>")
                    self.rotor.set_pos_deg(float(args[0]), float(args[1]))
                    if erp_prefix is None:
                        writer.write(self._reply_default("RPRT 0"))
                    else:
                        writer.write(self._reply_erp(erp_prefix, ["set_pos: {0} {1}".format(args[0], args[1])], 0))

                elif cmd == "M":
                    if len(args) < 2:
                        raise RuntimeError("Need: M <dir> <speed>")
                    direction = int(float(args[0]))
                    speed = int(float(args[1]))
                    if direction not in (2, 4, 8, 16):
                        raise RuntimeError("Direction must be 2/4/8/16")
                    self.rotor.move(direction, speed)
                    if erp_prefix is None:
                        writer.write(self._reply_default("RPRT 0"))
                    else:
                        writer.write(self._reply_erp(erp_prefix, ["move: {0} {1}".format(args[0], args[1])], 0))

                elif cmd == "S":
                    self.rotor.stop()
                    if erp_prefix is None:
                        writer.write(self._reply_default("RPRT 0"))
                    else:
                        writer.write(self._reply_erp(erp_prefix, ["stop:"], 0))

                elif cmd == "K":
                    self.rotor.park()
                    if erp_prefix is None:
                        writer.write(self._reply_default("RPRT 0"))
                    else:
                        writer.write(self._reply_erp(erp_prefix, ["park:"], 0))

                elif cmd == "i":
                    info = self.rotor.get_info()
                    if erp_prefix is None:
                        writer.write(self._reply_default(info))
                    else:
                        writer.write(self._reply_erp(erp_prefix, ["get_info:", info], 0))

                elif cmd == "d":
                    state = self.rotor.dump_state()
                    if erp_prefix is None:
                        writer.write((state + "\nRPRT 0\n").encode())
                    else:
                        writer.write(self._reply_erp(erp_prefix, ["dump_state:", state], 0))

                elif cmd.upper() == "H" and self.cfg.enable_custom_homing_command:
                    self.rotor.homing()
                    if erp_prefix is None:
                        writer.write(self._reply_default("RPRT 0"))
                    else:
                        writer.write(self._reply_erp(erp_prefix, ["homing:"], 0))

                elif cmd.upper() == "R":
                    self.rotor.recover_from_errors()
                    if erp_prefix is None:
                        writer.write(self._reply_default("RPRT 0"))
                    else:
                        writer.write(self._reply_erp(erp_prefix, ["recover:"], 0))

                elif cmd.upper() == "T":
                    status = self.rotor.get_status()
                    print("[STATUS]\n{0}".format(status))
                    if erp_prefix is None:
                        writer.write((status + "\nRPRT 0\n").encode())
                    else:
                        writer.write(self._reply_erp(erp_prefix, ["status:", status], 0))

                elif cmd.upper() == "E":
                    if len(args) < 2:
                        raise RuntimeError("Need: E <dir> <speed>  (same as M but bypasses brake resistor for hard-stop escape)")
                    direction = int(float(args[0]))
                    speed = int(float(args[1]))
                    if direction not in (2, 4, 8, 16):
                        raise RuntimeError("Direction must be 2/4/8/16")
                    self.rotor.escape_move(direction, speed)
                    if erp_prefix is None:
                        writer.write(self._reply_default("RPRT 0"))
                    else:
                        writer.write(self._reply_erp(erp_prefix, ["escape: {0} {1}".format(args[0], args[1])], 0))

                elif cmd.upper() == "EP":
                    # Escape pulse: escape jog for duration_ms then auto-stop.
                    # Syntax: EP <dir> <speed> [duration_ms=300]
                    # Repeat several times to walk off a hard stop.
                    if len(args) < 2:
                        raise RuntimeError("Need: EP <dir> <speed> [duration_ms]")
                    direction = int(float(args[0]))
                    speed = int(float(args[1]))
                    duration_ms = float(args[2]) if len(args) >= 3 else 300.0
                    if direction not in (2, 4, 8, 16):
                        raise RuntimeError("Direction must be 2/4/8/16")
                    print("[INFO] Escape pulse: dir={0} speed={1}% duration={2:.0f}ms".format(
                        direction, speed, duration_ms))
                    self.rotor.escape_move(direction, speed)
                    # Reply immediately so the client isn't blocked, then auto-stop
                    if erp_prefix is None:
                        writer.write(self._reply_default("RPRT 0"))
                    else:
                        writer.write(self._reply_erp(erp_prefix, [
                            "escape_pulse: {0} {1} {2:.0f}ms".format(args[0], args[1], duration_ms)
                        ], 0))
                    await writer.drain()
                    await asyncio.sleep(duration_ms / 1000.0)
                    self.rotor.stop()
                    print("[INFO] Escape pulse finished — motor stopped, brake resistor re-enabled.")
                    continue  # reply already sent

                else:
                    if erp_prefix is None:
                        writer.write(self._reply_default("RPRT -1"))
                    else:
                        writer.write(self._reply_erp(erp_prefix, ["error:", "Unknown command"], -1))

                await writer.drain()

            except Exception as exc:
                print("[ERROR] Command '{0}' from client failed: {1}".format(cmd, exc))
                if erp_prefix is None:
                    writer.write(self._reply_default("RPRT -1"))
                else:
                    writer.write(self._reply_erp(erp_prefix, ["error:", str(exc)], -1))
                await writer.drain()

        writer.close()
        await writer.wait_closed()


async def _error_poll_loop(rotor: ODriveRotor, interval_s: float = 2.0) -> None:
    """Background task: periodically print any ODrive axis errors to the server console."""
    while True:
        await asyncio.sleep(interval_s)
        try:
            rotor.print_errors_if_any()
        except Exception:
            pass


async def async_main(rotor: ODriveRotor):
    server = RotctldServer(rotor, CFG)
    srv = await asyncio.start_server(server.handle, CFG.listen_host, CFG.listen_port)
    sockets = srv.sockets or []
    addrs = ", ".join([str(s.getsockname()) for s in sockets])
    print("rotctld-style server listening on {0}".format(addrs))
    async with srv:
        asyncio.create_task(_error_poll_loop(rotor))
        await srv.serve_forever()


def main():
    rotor = ODriveRotor(CFG)
    rotor.connect()
    if CFG.apply_runtime_config_on_startup:
        rotor.apply_runtime_config()
        print("Applied runtime config.")
    if CFG.auto_calibrate_on_startup:
        rotor.calibrate_both_axes()
        print("Startup calibration finished.")
    if CFG.home_on_startup:
        try:
            rotor.home_both_axes()
            print("Startup homing finished.")
        except Exception as exc:
            print("[ERROR] Startup homing failed: {0}".format(exc))
            print("[INFO]  Server starting anyway — send 'H' (or \\homing) command to retry homing.")
    elif CFG.zero_on_startup:
        rotor.set_zero_here()
        print("Zeroed software position offsets at startup.")
    asyncio.run(async_main(rotor))


if __name__ == "__main__":
    main()
