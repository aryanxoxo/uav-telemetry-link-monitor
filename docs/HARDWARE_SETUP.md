# Hardware Setup

This build uses a Parrot Anafi as the UAV platform and a separate ESP32 + SX1276/SX1278 LoRa telemetry node as the experimental radio payload. The LoRa node is intentionally independent of the Anafi flight-control system, so the monitor can evaluate RF link health without modifying the drone autopilot or safety-critical command link.

## Selected Drone Platform

| Item | Selection |
| --- | --- |
| UAV | Parrot Anafi |
| Role | Flight platform carrying the telemetry test node |
| Payload approach | External ESP32 LoRa node mounted as a non-flight-critical payload |
| Ground station | Laptop + ESP32 LoRa receiver over USB serial |
| Dashboard | Python local web dashboard |

The Anafi is a good fit for this project because it is lightweight, field-portable, and provides a practical test aircraft for line-of-sight telemetry experiments. The LoRa telemetry payload should be mounted so it does not obstruct propellers, interfere with the camera gimbal, block cooling, or change the aircraft balance beyond safe limits.

## System Architecture

```text
Parrot Anafi
  |
  | carries
  v
ESP32 drone node + 915 MHz LoRa radio + antenna
  |
  | LoRa telemetry packets
  v
ESP32 ground station + 915 MHz LoRa radio + directional antenna
  |
  | USB serial JSON
  v
Python dashboard on laptop
```

## Required Hardware

| Item | Qty | Notes |
| --- | ---: | --- |
| Parrot Anafi | 1 | UAV platform. The ESP32 LoRa node is a separate payload, not a flight controller replacement. |
| ESP32 development board | 2 | One drone-side transmitter, one ground-side receiver. |
| SX1276/SX1278 LoRa module | 2 | Use 915 MHz-capable modules. |
| 915 MHz antennas | 2 | Use a compact drone antenna and a ground antenna suited to field testing. |
| Small regulated 3.3 V supply or battery module | 1 | Powers the drone-side ESP32/radio payload. |
| USB cable | 1+ | Used for flashing and for the ground station serial feed. |
| Laptop | 1 | Runs the Python dashboard and records CSV logs. |
| Mounting hardware | As needed | Use nonconductive standoffs, hook-and-loop strap, foam tape, or a small printed bracket. |

## ESP32 LoRa Pin Map

Default ESP32 wiring used by both firmware sketches:

| LoRa Signal | ESP32 Pin | Purpose |
| --- | --- | --- |
| NSS / CS | GPIO 5 | SPI chip select |
| SCK | GPIO 18 | SPI clock |
| MISO | GPIO 19 | SPI data from LoRa module |
| MOSI | GPIO 23 | SPI data to LoRa module |
| DIO0 | GPIO 26 | Radio interrupt |
| RESET | GPIO 14 | Radio reset |
| DIO1 | GPIO 33 | Secondary interrupt |
| 3V3 | 3.3 V | Regulated radio power |
| GND | GND | Common ground |

Do not power SX127x LoRa modules from 5 V unless your breakout explicitly includes regulation and level shifting. Most modules expect 3.3 V logic.

## Drone-Side Mounting On Parrot Anafi

1. Keep the LoRa payload independent from the Anafi command/control system.
2. Mount the ESP32 and LoRa module on the top or side of the airframe where it does not block propellers, vents, sensors, GPS reception, or the camera/gimbal.
3. Keep the antenna clear of carbon-fiber or metal structures where possible.
4. Strain-relieve the antenna connector. Small u.FL connectors are easy to damage in flight handling.
5. Use a small standalone battery or regulated auxiliary supply for the ESP32 payload.
6. Confirm center of gravity and mechanical security before takeoff.

## Ground Station Setup

1. Flash `firmware/ground_station/ground_station.ino` to the receiver ESP32.
2. Attach the ground LoRa antenna before powering the radio.
3. Connect the receiver ESP32 to the laptop over USB.
4. Aim the ground antenna along the expected flight line.
5. Run the dashboard:

```powershell
python dashboard\telemetry_dashboard.py --serial COM7 --baud 115200 --port 8765 --ground-lat 49.262369 --ground-lon -123.250118 --antenna-heading 45
```

Change `COM7` to the actual Windows serial port and adjust `--antenna-heading` to match the physical antenna pointing direction.

## Drone Node Setup

1. Flash `firmware/drone_node/drone_node.ino` to the drone-side ESP32.
2. Confirm the serial boot message reports 915 MHz, SF9, and 125 kHz bandwidth.
3. Power the ESP32/LoRa payload from its own safe supply.
4. Attach the antenna before enabling transmit.
5. Start with the Anafi on the ground and verify the dashboard receives packets before flight.

## Field Test Checklist

- Both antennas attached and mechanically secure.
- ESP32 payload does not interfere with Anafi propellers, camera, GPS, or cooling.
- Drone-side payload power is secure and isolated from the Anafi flight electronics.
- Ground station dashboard receives packets for at least two minutes before takeoff.
- Antenna heading in the dashboard matches the physical ground antenna direction.
- CSV logging is enabled and the log file is growing.
- The Anafi remains under its normal pilot command link and failsafe behavior.

## Safety Notes

This project monitors an experimental telemetry link. It should not be treated as the primary command, control, or failsafe link for the aircraft. Keep the Parrot Anafi under normal operating procedures, fly line-of-sight, and comply with local aviation and radio regulations.
