# Drone Selection: Parrot Anafi + Arduino/ESP32 LoRa Node

## Selected Configuration

The selected UAV setup is:

- **Drone:** Parrot Anafi
- **Experimental telemetry node:** ESP32 development board with SX1276/SX1278 LoRa module
- **Firmware environment:** Arduino IDE or PlatformIO using RadioLib
- **Ground station:** ESP32 LoRa receiver connected to a laptop over USB
- **Dashboard:** Python local web app

## Why Parrot Anafi

The Parrot Anafi is a practical choice for a field telemetry demo because it is portable, easy to deploy, and suitable for line-of-sight range testing without building an airframe from scratch. It also keeps the experimental link monitor separate from the aircraft's normal flight-control system.

The project does not require direct integration with the Anafi autopilot. Instead, the Anafi carries a lightweight telemetry payload that generates and transmits test flight-state packets.

## Why A Separate Arduino/ESP32 Node

Using a standalone Arduino-style ESP32 node keeps the project simple and testable:

- The LoRa link can be developed and bench-tested without the drone.
- The Anafi remains on its normal control and safety systems.
- The telemetry payload can be simulated or replaced with real sensor data later.
- The same firmware pattern works for other UAVs if the platform changes.

## Integration Boundary

```text
Parrot Anafi flight system
  - normal pilot control
  - normal failsafe behavior
  - no required firmware modification

ESP32 LoRa telemetry payload
  - independent power
  - independent antenna
  - test telemetry packets
  - RSSI/SNR/latency evaluation
```

This separation is intentional. The telemetry monitor is a measurement tool, not a replacement for the Anafi control link.

## Payload Data Strategy

The current firmware emits representative flight-state values from the ESP32 node. In a later version, those values can be replaced with:

- GPS module data connected to the ESP32
- Barometer or IMU data connected to the ESP32
- Manually injected range/bearing test values
- A companion-computer feed, if the aircraft platform supports it

The dashboard already accepts `drone_lat`, `drone_lon`, `altitude_m`, `range_m`, `bearing_deg`, and `antenna_heading_deg`, so real position data can be added without redesigning the UI.

## Mounting Guidance

- Use the lightest practical ESP32 and LoRa module combination.
- Avoid placing the antenna near the Anafi propellers, gimbal, GPS area, battery latch, or cooling vents.
- Use a removable mount for bench inspection and transport.
- Secure wiring so vibration cannot unplug the radio module.
- Keep the telemetry payload independent from flight-critical electronics.

## Ground Station Pairing

The ground receiver should use the same LoRa settings as the drone node:

- Frequency: 915 MHz
- Spreading factor: SF9 default
- Bandwidth: 125 kHz default
- Coding rate: 4/7
- CRC: enabled

The Python dashboard is anchored by default at the UBC ECE MacLeod Building area and can be pointed with:

```powershell
python dashboard\telemetry_dashboard.py --simulate --antenna-heading 45
```

For hardware:

```powershell
python dashboard\telemetry_dashboard.py --serial COM7 --baud 115200 --antenna-heading 45
```

## Decision Summary

Parrot Anafi plus a separate Arduino/ESP32 LoRa payload gives the cleanest demo path: a real UAV platform, a safe integration boundary, inexpensive radio hardware, and a dashboard focused on link quality rather than aircraft control.
