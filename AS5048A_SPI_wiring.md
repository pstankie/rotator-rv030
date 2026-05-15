# AS5048A SPI Encoder — Wiring & Setup (ODrive v3.6, AZ axis)

## Hardware wiring

All signals connect to the **J3 connector** (20-pin header) on ODrive v3.6.

### CS pin selection

Only **GPIO 1–6** are available on this build — GPIO 7 (AZ endstop) and GPIO 8 (EL endstop) are already taken. Of the remaining six, **GPIO 1 and 2 are UART** and must not be used for CS. That leaves GPIOs 3, 4, 5, 6. **GPIO 4 (J3 pin 14)** is used, matching the ODrive docs example.

### J3 connector pinout (full)

| J3 pin | Label | Used for |
|--------|-------|----------|
| 1 | VCC | AS5048A VDD |
| 2 | GND | AS5048A GND |
| 3 | CANH | (CAN bus — not used) |
| 4 | CANL | (CAN bus — not used) |
| 5 | GND | — |
| 6 | AVCC | — |
| 7 | AGND | — |
| 8 | SCK | AS5048A CLK |
| 9 | MISO | AS5048A MISO / DO |
| 10 | MOSI | leave unconnected, or tie to 3.3V (AMS encoders only) |
| 11 | GPIO 1 | (UART TX — avoid for CS) |
| 12 | GPIO 2 | (UART RX — avoid for CS) |
| 13 | GPIO 3 | free |
| **14** | **GPIO 4** | **AS5048A CSn ← use this** |
| 15 | GPIO 5 | free |
| 16 | GPIO 6 | free |
| 17 | GPIO 7 | AZ home endstop |
| 18 | GPIO 8 | EL home endstop |
| 19 | GND | — |
| 20 | GND | — |

### AS5048A → ODrive summary

| AS5048A pin | ODrive J3 pin | Signal |
|-------------|---------------|--------|
| VDD | pin 1 | VCC (3.3 V) |
| GND | pin 2 | GND |
| CLK | pin 8 | SCK |
| MISO / DO | pin 9 | MISO |
| CSn | pin 14 | GPIO 4 (`az_spi_cs_gpio_pin = 4`) |
| MOSI / DI | leave unconnected | (tie to 3.3V to save a wire) |

> **Series resistors** (place near the ODrive end of the cable):
> - CLK: 20–50 Ω (most susceptible to noise per ODrive docs)
> - MISO / CSn: 100 Ω
> Not needed for cables under ~10 cm.

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
