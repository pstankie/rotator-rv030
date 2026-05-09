# Copilot Instructions

## What this repo is

A single-file Python daemon (`odrive_rotctld_fw056_60to1_autocal_homing.py`) that exposes a [Hamlib `rotctld`-compatible](https://hamlib.github.io/) TCP server (default port 4533) for controlling an AZ/EL antenna rotor driven by an ODrive v3.6 motor controller (firmware 0.5.6) through 60:1 worm gearboxes.

## Running

```bash
python odrive_rotctld_fw056_60to1_autocal_homing.py
```

There are no tests, no build steps, and no linter configuration. The only runtime dependency is the `odrive` Python package (installs via `pip install odrive`).

## Architecture

Three logical layers, all in one file:

1. **`RotorConfig` dataclass** — central configuration. All tuning lives here (gear ratios, axis limits, GPIO pins, current limits, homing speeds, etc.). The singleton `CFG = RotorConfig()` is used throughout. Edit fields directly to change hardware parameters.

2. **`ODriveRotor` class** — hardware abstraction. Translates degree-based commands into ODrive turn-based setpoints. Manages axis state machine transitions, calibration, homing, and error recovery. Key internals:
   - Position is tracked in *turns* internally; degrees are computed via `az_deg_per_turn` / `el_deg_per_turn` (currently 6.0°/turn for 60:1 worm drive).
   - Software zero offsets (`az_offset_turns`, `el_offset_turns`) are set at startup or after homing. `set_zero_here()` captures the current encoder position as the software zero.
   - AZ = `axis1`, EL = `axis0` (see `az_axis_idx` / `el_axis_idx` in `RotorConfig`).
   - Axes are accessed via `self._axis(idx)` → `odrv.axis0` / `odrv.axis1`.

3. **`RotctldServer` class + `async_main`** — asyncio TCP server. Parses one command per line, dispatches to `ODriveRotor`, and writes replies. A background task (`_error_poll_loop`) polls ODrive axis errors every 2 s and prints them to the server console.

## Protocol (rotctld subset)

Both short and long command forms are accepted. The `long_to_short` dict maps `\get_pos` → `p`, `\set_pos` → `P`, etc.

| Short | Long | Description |
|-------|------|-------------|
| `p` | `\get_pos` | Get AZ/EL position in degrees |
| `P <az> <el>` | `\set_pos` | Move to absolute position |
| `M <dir> <spd>` | `\move` | Jog; dir ∈ {2=EL+, 4=EL−, 8=AZ−, 16=AZ+} |
| `S` | `\stop` | Stop all axes, re-enable brake resistor |
| `K` | `\park` | Move to AZ=0 EL=0 |
| `H` | `\homing` | Run full homing sequence |
| `R` | `\recover` | Clear axis errors (required before `P` if errors are present) |
| `T` | `\status` | Print axis states, errors, endstop GPIO levels |
| `E <dir> <spd>` | `\escape` | Jog with brake resistor disabled (hard-stop escape) |
| `EP <dir> <spd> [ms]` | `\escape_pulse` | Timed escape jog, auto-stops after `ms` (default 300) |
| `ET <axis> <Nm> <s>` | `\escape_torque` | Raw torque command, bypasses position/velocity loops |
| `i` | `\get_info` | ODrive HW/FW/SN info string |
| `d` | `\dump_state` | Full state dump |

Lines may optionally be prefixed with an ERP character (`+`, `;`, etc.) — see `parse_erp_prefix` / `erp_sep`.

## Encoder configuration

| Axis | Encoder | Location | CPR | deg/turn |
|------|---------|----------|-----|----------|
| AZ (axis1) | AS5048A (SPI absolute) | Output shaft (after 60:1 worm) | 16384 | 360.0 |
| EL (axis0) | Incremental (existing) | Motor shaft | — | 6.0 |

Because the AS5048A is on the **output shaft**, all AZ velocity/acceleration limits in `RotorConfig` are in **output turns/s** (not motor turns/s). The EL axis is still motor-shaft so its limits remain in motor turns/s. Mixing these units is a common source of confusion.

**One-time ODrive setup** (run once after wiring, then the configuration persists in flash):
```python
rotor = ODriveRotor(CFG)
rotor.connect()
rotor.configure_az_spi_encoder_and_save()
# ODrive reboots — re-run main script normally after this
```

**SPI wiring — all on J3 connector (ODrive v3.6)**:
| AS5048A pin | J3 pin | Signal |
|-------------|--------|--------|
| VDD | 1 | VCC (3.3 V) |
| GND | 2 | GND |
| CLK | 8 | SCK |
| MISO / DO | 9 | MISO |
| CSn | 14 | GPIO 4 (`az_spi_cs_gpio_pin = 4`) |
| MOSI / DI | — | leave unconnected (or tie to 3.3 V) |

GPIO 4 is free — GPIOs 1/2 are UART, GPIO 7 = AZ endstop, GPIO 8 = EL endstop.  
Series resistors: 20–50 Ω on CLK, 100 Ω on MISO/CSn (place near ODrive end).

**After encoder offset calibration** runs once, set `encoder.config.pre_calibrated = True` and save to avoid re-running the 60 s calibration on every startup.

## Key conventions

- **Error handling pattern**: `set_pos` refuses to move if axis errors are active — clients must send `R` first. `move` auto-clears errors before jogging (gentler recovery). Escape commands (`E`/`EP`/`ET`) disable the brake resistor; `S` always re-enables it.
- **Calibration/homing sequence**: On startup — connect → (optional) `apply_runtime_config` → `calibrate_both_axes` (motor cal + encoder offset, sequentially per axis) → `home_both_axes` (both axes homed in parallel via `ThreadPoolExecutor`) → `set_zero_here`.
- **ODrive attribute fallbacks**: Many setpoint attributes changed names between firmware versions. The pattern used throughout is `try: ax.controller.input_pos = v except: ax.controller.pos_setpoint = v` — always keep both fallbacks when touching setpoint writes.
- **`_safe_set_attr`**: Used for endstop config writes where the attribute may not exist on older firmware. Use it (rather than direct setattr) when writing optional ODrive config attributes.
- **GPIO reading**: `odrv.get_gpio_states()` returns a bitmask (fw 0.5.x); bit `N−1` = GPIO N. Falls back to `odrv.gpioN` attribute for older firmware.
- **Formatting**: f-strings are intentionally avoided; `.format()` is used throughout for compatibility with older Python 3.x.
