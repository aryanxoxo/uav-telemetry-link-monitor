# UAV Telemetry Link Monitor

Telemetry link monitoring tool for ESP32 LoRa UAV ground stations. The drone node transmits compact flight state over a 915 MHz LoRa link; the ground station echoes sequence metadata to measure round-trip latency and streams link metrics to a Python dashboard.

The selected demo aircraft is a **Parrot Anafi** carrying a separate Arduino-style **ESP32 + LoRa telemetry payload**. The payload is independent of the Anafi flight-control system and is used only for telemetry link measurement.

## What Is Included

- ESP32 drone transmitter firmware with configurable spreading factor and bandwidth.
- ESP32 ground station receiver firmware that logs RSSI, SNR, packet loss inputs, and latency echo data.
- Python real-time dashboard with antenna geometry map, RSSI, SNR, packet loss, and latency trends.
- Simulator mode for dashboard development without hardware.
- CSV flight log output for post-flight latency and link-margin analysis.
- Hardware setup and drone-selection notes for Parrot Anafi + ESP32/Arduino LoRa payload.

## Project Layout

```text
firmware/
  drone_node/drone_node.ino          ESP32 + LoRa flight telemetry transmitter
  ground_station/ground_station.ino  ESP32 + LoRa receiver and serial logger
dashboard/
  telemetry_dashboard.py             Python live dashboard and CSV logger
  requirements.txt                   Optional serial dependency
data/
  sample_flight_log.csv              Example dashboard-compatible log
docs/
  HARDWARE_SETUP.md                  Parrot Anafi, ESP32 LoRa wiring, field setup, safety checks
  DRONE_SELECTION.md                 Why Parrot Anafi + Arduino/ESP32 was selected
```

## Hardware And Drone Setup

- [Hardware setup](docs/HARDWARE_SETUP.md)
- [Drone selection: Parrot Anafi + Arduino/ESP32 LoRa node](docs/DRONE_SELECTION.md)
- [Word hardware setup guide](UAV_Telemetry_Link_Monitor_Hardware_Setup.docx)

## Hardware Assumptions

- ESP32 development boards.
- SX1276/SX1278 LoRa modules configured for 915 MHz.
- RadioLib Arduino library.
- Drone node and ground station share the same LoRa configuration.

Default pin map:

| Signal | ESP32 Pin |
| --- | --- |
| NSS / CS | 5 |
| SCK | 18 |
| MISO | 19 |
| MOSI | 23 |
| DIO0 | 26 |
| RESET | 14 |
| DIO1 | 33 |

Adjust the pin constants in both sketches if your board differs.

## Quick Start: Dashboard Simulator

From this folder:

```powershell
python dashboard\telemetry_dashboard.py --simulate --port 8765 --antenna-heading 45
```

Open:

```text
http://127.0.0.1:8765
```

The simulator generates realistic RSSI fades, SNR variation, burst packet loss, latency jitter, aircraft range/bearing, and antenna off-axis effects so the dashboard can be evaluated before hardware is attached.

## Quick Start: Hardware

1. Install RadioLib in the Arduino IDE or PlatformIO.
2. Flash `firmware/drone_node/drone_node.ino` to the UAV ESP32.
3. Flash `firmware/ground_station/ground_station.ino` to the receiver ESP32.
4. Connect the receiver to the laptop over USB.
5. Install optional serial support:

```powershell
pip install -r dashboard\requirements.txt
```

6. Run the dashboard:

```powershell
python dashboard\telemetry_dashboard.py --serial COM7 --baud 115200 --port 8765 --ground-lat 49.262369 --ground-lon -123.250118 --antenna-heading 45
```

Change `COM7` to the receiver port shown by Windows Device Manager. The default ground station coordinates are set to the UBC ECE MacLeod Building area at 2356 Main Mall.

## Serial Protocol

The ground station emits newline-delimited JSON:

```json
{"type":"rx","seq":42,"rssi":-91.5,"snr":8.25,"latency_ms":73.1,"sf":9,"bw_khz":125,"lost":0,"timestamp_ms":128331,"drone_lat":37.42821,"drone_lon":-122.07154,"altitude_m":84.2,"antenna_heading_deg":45}
```

The dashboard accepts these fields:

- `seq`: monotonic packet sequence number.
- `rssi`: LoRa packet RSSI in dBm.
- `snr`: LoRa packet SNR in dB.
- `latency_ms`: sequence echo round-trip estimate.
- `sf`: current spreading factor.
- `bw_khz`: current signal bandwidth.
- `lost`: optional packet loss count since previous packet.
- `timestamp_ms`: sender or receiver millisecond clock.
- `drone_lat` / `drone_lon`: optional aircraft GPS position used by the map.
- `ground_lat` / `ground_lon`: optional ground station GPS position. If omitted, CLI defaults are used.
- `altitude_m`: optional aircraft altitude shown in the geometry panel.
- `antenna_heading_deg`: optional ground antenna heading in degrees true. If omitted, the CLI value is used.
- `range_m` / `bearing_deg`: optional fallback when GPS is unavailable. If latitude/longitude is present, the dashboard computes range and bearing itself.

## Map And Antenna View

The dashboard now opens with a local range map instead of pure telemetry first. It shows the ground station at center, the antenna centerline and beam, the aircraft track, live range, bearing, altitude, and off-axis angle.

This view is intentionally local and tile-free so it works at a field site without internet access. It is most useful for answering: "Did the link degrade because the aircraft left the antenna beam, because it got farther away, or because the pass caused multipath?"

## Link Adaptation

The ground station computes a rolling link quality estimate from packet loss, RSSI, SNR, and latency. It recommends a spreading-factor change using conservative hysteresis:

- Step up SF when packet loss exceeds 8%, SNR is below 4 dB, or RSSI is below -112 dBm.
- Step down SF when packet loss is below 2%, SNR is above 9 dB, and RSSI is above -98 dBm for a sustained window.

The sketches include a control message path for applying the recommendation. In flight testing, keep adaptation conservative and log every SF change so post-flight analysis can separate RF effects from configuration changes.

## Notes On The Latency Estimator

The telemetry payload and latency measurement are intentionally separated. The drone keeps a small table of transmit timestamps indexed by sequence number, and the ground station echoes sequence metadata. This keeps forward error correction or telemetry payload changes from biasing latency measurements during burst loss events caused by multipath fading.

## Flight Test Outcome Target

The included settings target the described open-field outcome: reliable telemetry at 2.8 km using SF9, 125 kHz bandwidth, and moderate transmit intervals. Antenna orientation effects are visible in the dashboard through correlated RSSI/SNR margin dips and latency spikes.
