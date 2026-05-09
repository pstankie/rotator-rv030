# AS5048A SPI Encoder — Wiring & Setup (ODrive v3.6, AZ axis)

## Hardware wiring

The AS5048A connects to ODrive v3.6 via the hardware SPI bus.  
AZ = **axis1** (M1 encoder connector).

### Pin mapping

| AS5048A pin | ODrive v3.6 | Notes |
|-------------|-------------|-------|
| VDD | 3.3V — M1 encoder connector pin 1 | Do **not** use 5V |
| GND | GND — M1 encoder connector pin 2 | |
| CLK | GPIO 9 — SPI_SCK (J3 GPIO header) | Verify against board silkscreen |
| MISO / DO | GPIO 11 — SPI_MISO (J3) | Verify against board silkscreen |
| CSn | M1 encoder connector A pin | Check ODrive v3.6 pinout — must **not** share GPIO 8 (used for EL endstop) |
| MOSI / DI | leave unconnected | Not needed for normal reads |

> **Series resistors**: add 100 Ω on CLK, MISO, and CSn close to the ODrive if the cable is longer than ~20 cm.

### Encoder placement

The AS5048A magnet sits on the **output shaft** (after the 60:1 worm gear), not the motor shaft.  
This gives direct absolute antenna position — one encoder revolution = 360° of azimuth rotation.

---

## One-time ODrive configuration

Run this **once** after wiring. The configuration is saved to ODrive flash and persists across reboots.

```python
from odrive_rotctld_fw056_60to1_autocal_homing import ODriveRotor, CFG

rotor = ODriveRotor(CFG)
rotor.connect()
rotor.configure_az_spi_encoder_and_save()
```

The ODrive reboots automatically after saving. Wait for it to come back up before continuing.

---

## First-run calibration

Start the script normally:

```bash
python odrive_rotctld_fw056_60to1_autocal_homing.py
```

With `auto_calibrate_on_startup = True` (default), the startup sequence will:
1. Run motor calibration on AZ (~60 s)
2. Run encoder offset calibration on AZ (~60 s) — the motor will move a small amount through the gearbox
3. Run motor + encoder calibration on EL
4. Home both axes simultaneously via the endstop switches

Once calibration completes and the rotor is working correctly, save the calibration results to flash so subsequent startups skip re-calibration:

```python
import odrive
odrv = odrive.find_any()
odrv.axis1.encoder.config.pre_calibrated = True
odrv.axis1.motor.config.pre_calibrated   = True
odrv.save_configuration()
```

After this you can set `auto_calibrate_on_startup = False` in `RotorConfig` to skip the 2-minute calibration on every boot.

---

## Verifying the encoder

After startup, connect to the rotctld server (`telnet localhost 4533`) and run:

```
d
```

The `dump_state` output will show:
```
Encoder AZ: AS5048A SPI absolute on output shaft (CPR=16384, axis1)
```

Check that AZ position changes smoothly and in the correct direction as you manually turn the rotor.  
If the direction is reversed, negate `az_zero_offset_deg` or swap the physical magnet orientation.

---

## Configuration values changed for output-shaft encoder

These `RotorConfig` fields differ from a motor-shaft encoder setup because all AZ speeds are now in **output turns/s** (1 turn = 360°) instead of motor turns/s:

| Field | Motor-shaft value | Output-shaft value | Physical speed |
|-------|------------------|--------------------|----------------|
| `az_deg_per_turn` | 6.0 | **360.0** | — |
| `az_max_turns_per_s` | 2.0 | **0.033** | 12°/s |
| `az_vel_limit_turns_s` | 2.0 | **0.033** | 12°/s |
| `az_accel_limit_turns_s2` | 2.0 | **0.033** | — |
| `az_decel_limit_turns_s2` | 2.0 | **0.033** | — |
| `az_home_search_turns_s` | 0.5 | **0.014** | ≈5°/s |

EL axis values are unchanged — EL still uses a motor-shaft incremental encoder.
