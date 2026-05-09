# rotator-rv030

Antenna rotor controller — ODrive v3.6 + AS5048A SPI encoder, exposed as a `rotctld`-compatible TCP server.

## System diagram

```plantuml
@startuml
title RV030 Antenna Rotor — System Connections
left to right direction
skinparam defaultTextAlignment center
skinparam component {
  BackgroundColor<<hw>>   #DDEEFF
  BackgroundColor<<sw>>   #E8F5E9
  BackgroundColor<<pwr>>  #FFF3CD
  BackgroundColor<<ext>>  #F3E5F5
  BorderColor             #555555
}

' ── External client ──────────────────────────────────────────
component "Hamlib Client\n(rotctl / gpredict / other)" <<ext>> as CLIENT

' ── Software (Odroid / host PC) ──────────────────────────────
package "Odroid / Host PC" #E8F5E9 {
  component "RotorConfig\n(dataclass)" <<sw>> as RC
  component "ODriveRotor\n(hardware abstraction)" <<sw>> as ODR
  component "RotctldServer\nTCP :4533" <<sw>> as RCS
}

' ── ODrive v3.6 ──────────────────────────────────────────────
package "ODrive v3.6" #DDEEFF {
  component "axis1  (AZ)" <<hw>> as AX1
  component "axis0  (EL)" <<hw>> as AX0

  package "GPIO / SPI" #C8E0FF {
    component "GPIO 7\nAZ endstop in" <<hw>> as G7
    component "GPIO 8\nEL endstop in" <<hw>> as G8
    component "GPIO 9\nSPI SCK" <<hw>> as G9
    component "GPIO 11\nSPI MISO" <<hw>> as G11
    component "M1 enc. A pin\nSPI CS  (verify GPIO)" <<hw>> as GCS
  }
}

' ── Power ────────────────────────────────────────────────────
package "Power" #FFF3CD {
  component "PSU\n48 V / 30 A" <<pwr>> as PSU
  component "Brake Resistor\n2 Ω / 50 W" <<pwr>> as BR
}

' ── AZ axis (right side, top) ────────────────────────────────
package "AZ Axis" #FDECEA {
  component "AZ BLDC Motor" <<hw>> as AZM
  component "60:1 Worm Gear" <<hw>> as AZG
  component "AS5048A\n14-bit SPI absolute\nencoder (output shaft)" <<hw>> as AS5
  component "AZ Home\nEndstop Switch" <<hw>> as AZSW
}

' ── EL axis (right side, bottom) ─────────────────────────────
package "EL Axis" #EAF4EA {
  component "EL BLDC Motor" <<hw>> as ELM
  component "60:1 Worm Gear" <<hw>> as ELG
  component "Incremental Encoder\n(motor shaft)" <<hw>> as ELE
  component "EL Home\nEndstop Switch" <<hw>> as ELSW
}

' ── Connections ───────────────────────────────────────────────

' Client ↔ software
CLIENT <--> RCS : TCP rotctld\n(port 4533)

' Software internals
RC --> ODR
RC --> RCS
ODR --> RCS

' Software ↔ ODrive
ODR <--> AX1 : USB  (odrive Python lib)
ODR <--> AX0 : USB  (odrive Python lib)

' Power
PSU --> AX1 : 48 V DC
PSU --> AX0 : 48 V DC
AX1 --> BR : regen braking
AX0 --> BR : regen braking

' AZ axis
AX1 --> AZM : phase wires (U/V/W)
AZM --> AZG : motor shaft
AZG --> AS5 : output shaft\n(magnet)
G9  --> AS5 : SCK
G11 --> AS5 : MISO
GCS --> AS5 : CSn\n(100 Ω series)
AZSW --> G7  : endstop signal

' EL axis
AX0 --> ELM  : phase wires (U/V/W)
ELM --> ELG  : motor shaft
ELM --> ELE  : motor shaft\n(encoder)
ELE --> AX0  : encoder port
ELSW --> G8  : endstop signal

@enduml
```

![System diagram](https://www.plantuml.com/plantuml/png/LPTRkCs47xNAGREGssWsSXrl76285XRbfsWJcDO-Ia2lz0IRHEHI8EaumJ50llK4-GIlKmFiIVfK3yRKT4MVTdk6aAWp7ppCv-6C_oWD95wcyIEPZgcCBjrEoxqkQQS4vW9BIHy_lG2mRFID05FS4v3pGHNJaoN6hG0oLPh3H6JcS9H3unlY2G9H7H9jh6-fa-w7xCLJoZN4E8VAakeK2GRmOtg3mTWGCA7bHHR7daY5lBq.png)

> Source above — edit with any PlantUML-capable tool (VS Code PlantUML extension, `plantuml.jar`, or the [online server](https://www.plantuml.com/plantuml)).

## Files

| File | Purpose |
|------|---------|
| `odrive_rotctld_fw056_60to1_autocal_homing.py` | Main daemon — rotctld server + ODrive control |
| `AS5048A_SPI_wiring.md` | AS5048A wiring and first-run calibration procedure |
| `.github/copilot-instructions.md` | Copilot context for this repo |
