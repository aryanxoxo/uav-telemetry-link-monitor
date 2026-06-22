# UAV Telemetry Link Monitor

Telemetry link monitoring tool for ESP32 LoRa UAV ground stations. The drone node transmits compact flight state over a 915 MHz LoRa link; the ground station echoes sequence metadata to measure round-trip latency and streams link metrics to a Python dashboard.

The selected test aircraft is a **Parrot Anafi** carrying a separate Arduino-style **ESP32 + LoRa telemetry payload**. The payload is independent of the Anafi flight-control system and is used only for telemetry link measurement.

## What Is Included

- ESP32 drone transmitter firmware with configurable spreading factor and bandwidth.
- ESP32 ground station receiver firmware that logs RSSI, SNR, packet loss inputs, and latency echo data.
- Python real-time dashboard with antenna geometry map, RSSI, SNR, packet loss, and latency trends when connected to the ground-station serial feed.
- Simulator mode for dashboard development and UI checks without hardware.
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

## Quick Start: Dashboard UI Check

From this folder:

```powershell
python dashboard\telemetry_dashboard.py --simulate --port 8765 --antenna-heading 45
```

Open:

```text
http://127.0.0.1:8765
```

Simulator mode creates representative link samples so the dashboard layout, charts, CSV logging, and antenna-geometry view can be checked before hardware is attached. It is not a substitute for measured RF data.

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

## Dashboard Behavior With Hardware Connected

When the ground-station ESP32 is connected over USB serial, the dashboard reads newline-delimited JSON from the receiver. It shows the ground station at center, the antenna centerline and beam, the aircraft track, live range, bearing, altitude, off-axis angle, RSSI, SNR, packet loss, latency, and spreading-factor recommendation.

This view is intentionally local and tile-free so it works at a field site without internet access. It is most useful for answering: "Did the link degrade because the aircraft left the antenna beam, because it got farther away, or because the pass caused multipath?"

## Link Adaptation

The ground station computes a rolling link quality estimate from packet loss, RSSI, SNR, and latency. It recommends a spreading-factor change using conservative hysteresis:

- Step up SF when packet loss exceeds 8%, SNR is below 4 dB, or RSSI is below -112 dBm.
- Step down SF when packet loss is below 2%, SNR is above 9 dB, and RSSI is above -98 dBm for a sustained window.

The sketches include a control message path for applying the recommendation. In flight testing, keep adaptation conservative and log every SF change so post-flight analysis can separate RF effects from configuration changes.

## Notes On The Latency Estimator

The telemetry payload and latency measurement are intentionally separated. The drone keeps a small table of transmit timestamps indexed by sequence number, and the ground station echoes sequence metadata. This keeps forward error correction or telemetry payload changes from biasing latency measurements during burst loss events caused by multipath fading.

## Validation Target

The included settings provide a starting point for open-field validation using SF9, 125 kHz bandwidth, and moderate transmit intervals. A good validation run should log RSSI, SNR, packet loss, latency, range/bearing, and antenna heading so link-margin changes can be reviewed after the test.
