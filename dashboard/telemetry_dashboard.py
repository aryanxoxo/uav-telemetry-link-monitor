#!/usr/bin/env python3
"""Real-time UAV LoRa telemetry link monitor.

Run in simulator mode:
    python telemetry_dashboard.py --simulate --port 8765

Run against a serial-connected ground station:
    python telemetry_dashboard.py --serial COM7 --baud 115200 --port 8765
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import statistics
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse


WINDOW = 180


@dataclass
class LinkSample:
    received_at: str
    seq: int
    rssi: float
    snr: float
    latency_ms: float
    sf: int
    bw_khz: int
    lost: int
    packet_loss_pct: float
    link_quality: int
    ground_lat: float
    ground_lon: float
    drone_lat: float
    drone_lon: float
    altitude_m: float
    range_m: float
    bearing_deg: float
    antenna_heading_deg: float
    off_axis_deg: float


@dataclass
class LinkState:
    ground_lat: float = 49.262369
    ground_lon: float = -123.250118
    antenna_heading_deg: float = 45.0
    samples: deque[LinkSample] = field(default_factory=lambda: deque(maxlen=WINDOW))
    total_packets: int = 0
    total_lost: int = 0
    last_seq: int | None = None
    recommendation: str = "Hold SF"
    lock: threading.Lock = field(default_factory=threading.Lock)

    def ingest(self, raw: dict[str, Any]) -> LinkSample:
        seq = int(raw.get("seq", 0))
        reported_lost = int(raw.get("lost", 0))
        inferred_lost = 0
        if self.last_seq is not None and seq > self.last_seq + 1:
            inferred_lost = seq - self.last_seq - 1
        lost = max(reported_lost, inferred_lost)
        self.last_seq = max(seq, self.last_seq or seq)
        self.total_packets += 1
        self.total_lost += lost

        rssi = float(raw.get("rssi", -130.0))
        snr = float(raw.get("snr", -20.0))
        latency_ms = float(raw.get("latency_ms", 0.0))
        sf = int(raw.get("sf", 9))
        bw_khz = int(raw.get("bw_khz", 125))
        packet_loss_pct = 100.0 * self.total_lost / max(1, self.total_packets + self.total_lost)
        quality = score_link(rssi, snr, latency_ms, packet_loss_pct)

        ground_lat = float(raw.get("ground_lat", self.ground_lat))
        ground_lon = float(raw.get("ground_lon", self.ground_lon))
        antenna_heading = normalize_degrees(float(raw.get("antenna_heading_deg", self.antenna_heading_deg)))
        if "drone_lat" in raw and "drone_lon" in raw:
            drone_lat = float(raw["drone_lat"])
            drone_lon = float(raw["drone_lon"])
            range_m, bearing_deg = distance_and_bearing(ground_lat, ground_lon, drone_lat, drone_lon)
        else:
            range_m = float(raw.get("range_m", 0.0))
            bearing_deg = normalize_degrees(float(raw.get("bearing_deg", antenna_heading)))
            drone_lat, drone_lon = destination_point(ground_lat, ground_lon, bearing_deg, range_m)

        sample = LinkSample(
            received_at=datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z"),
            seq=seq,
            rssi=rssi,
            snr=snr,
            latency_ms=latency_ms,
            sf=sf,
            bw_khz=bw_khz,
            lost=lost,
            packet_loss_pct=round(packet_loss_pct, 2),
            link_quality=quality,
            ground_lat=round(ground_lat, 7),
            ground_lon=round(ground_lon, 7),
            drone_lat=round(drone_lat, 7),
            drone_lon=round(drone_lon, 7),
            altitude_m=round(float(raw.get("altitude_m", 0.0)), 1),
            range_m=round(range_m, 1),
            bearing_deg=round(bearing_deg, 1),
            antenna_heading_deg=round(antenna_heading, 1),
            off_axis_deg=round(angular_delta(bearing_deg, antenna_heading), 1),
        )
        with self.lock:
            self.samples.append(sample)
            self.recommendation = recommend_sf(list(self.samples), sf)
        return sample

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            samples = list(self.samples)
            recommendation = self.recommendation
        latest = samples[-1] if samples else None
        latencies = [s.latency_ms for s in samples[-30:]]
        return {
            "latest": vars(latest) if latest else None,
            "samples": [vars(s) for s in samples],
            "summary": {
                "packets": self.total_packets,
                "lost": self.total_lost,
                "latency_p50": percentile(latencies, 50),
                "latency_p95": percentile(latencies, 95),
                "recommendation": recommendation,
            },
        }


def score_link(rssi: float, snr: float, latency_ms: float, loss_pct: float) -> int:
    rssi_score = clamp((rssi + 125.0) / 40.0, 0.0, 1.0)
    snr_score = clamp((snr + 5.0) / 18.0, 0.0, 1.0)
    latency_score = clamp(1.0 - max(0.0, latency_ms - 50.0) / 250.0, 0.0, 1.0)
    loss_score = clamp(1.0 - loss_pct / 18.0, 0.0, 1.0)
    return round(100 * (0.32 * rssi_score + 0.28 * snr_score + 0.2 * latency_score + 0.2 * loss_score))


def recommend_sf(samples: list[LinkSample], current_sf: int) -> str:
    if len(samples) < 12:
        return "Hold SF"
    window = samples[-24:]
    loss = window[-1].packet_loss_pct
    rssi = statistics.fmean(s.rssi for s in window)
    snr = statistics.fmean(s.snr for s in window)
    latency = statistics.fmean(s.latency_ms for s in window)
    if (loss > 8.0 or snr < 4.0 or rssi < -112.0 or latency > 180.0) and current_sf < 12:
        return f"Increase to SF{current_sf + 1}"
    if loss < 2.0 and snr > 9.0 and rssi > -98.0 and latency < 95.0 and current_sf > 7:
        return f"Decrease to SF{current_sf - 1}"
    return "Hold SF"


def percentile(values: list[float], pct: int) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    idx = min(len(ordered) - 1, math.ceil((pct / 100) * len(ordered)) - 1)
    return round(ordered[idx], 1)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_degrees(value: float) -> float:
    return value % 360.0


def angular_delta(a: float, b: float) -> float:
    return abs((a - b + 180.0) % 360.0 - 180.0)


def destination_point(lat: float, lon: float, bearing_deg: float, distance_m: float) -> tuple[float, float]:
    radius_m = 6_371_000.0
    bearing = math.radians(bearing_deg)
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    angular = distance_m / radius_m
    lat2 = math.asin(math.sin(lat1) * math.cos(angular) + math.cos(lat1) * math.sin(angular) * math.cos(bearing))
    lon2 = lon1 + math.atan2(
        math.sin(bearing) * math.sin(angular) * math.cos(lat1),
        math.cos(angular) - math.sin(lat1) * math.sin(lat2),
    )
    return math.degrees(lat2), math.degrees(lon2)


def distance_and_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> tuple[float, float]:
    radius_m = 6_371_000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lon2 - lon1)
    a = math.sin(d_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2) ** 2
    distance = radius_m * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    y = math.sin(d_lambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(d_lambda)
    return distance, normalize_degrees(math.degrees(math.atan2(y, x)))


class CsvLogger:
    def __init__(self, path: Path) -> None:
        self.path = path
        self.path.parent.mkdir(parents=True, exist_ok=True)
        expected_header = ",".join(LinkSample.__annotations__.keys())
        if self.path.exists() and self.path.stat().st_size > 0:
            current_header = self.path.open("r", encoding="utf-8", newline="").readline().strip()
            if current_header != expected_header:
                rotated = self.path.with_name(f"{self.path.stem}_legacy_{int(time.time())}{self.path.suffix}")
                self.path.replace(rotated)
        self.file = self.path.open("a", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.file, fieldnames=list(LinkSample.__annotations__.keys()))
        if self.path.stat().st_size == 0:
            self.writer.writeheader()

    def write(self, sample: LinkSample) -> None:
        self.writer.writerow(vars(sample))
        self.file.flush()


def simulator(state: LinkState, logger: CsvLogger, interval: float) -> None:
    seq = 0
    sf = 9
    while True:
        seq += 1
        phase = seq / 18.0
        orbit = seq / 95.0
        range_m = 1300.0 + 850.0 * (0.5 + 0.5 * math.sin(seq / 54.0))
        bearing_deg = normalize_degrees(state.antenna_heading_deg - 50.0 + 95.0 * (0.5 + 0.5 * math.sin(orbit)))
        off_axis = angular_delta(bearing_deg, state.antenna_heading_deg)
        geometry_fade = -0.32 * max(0.0, off_axis - 18.0)
        multipath_fade = -18.0 if 70 < (seq % 120) < 82 else 0.0
        range_fade = -8.0 * clamp((range_m - 1200.0) / 1800.0, 0.0, 1.0)
        fade = multipath_fade + geometry_fade + range_fade
        burst_loss = 2 if 70 < (seq % 120) < 74 else 0
        if burst_loss:
            seq += burst_loss
        rssi = -88.0 + 7.0 * math.sin(phase) + fade + random.uniform(-2.5, 2.5)
        snr = 10.5 + 2.5 * math.sin(phase + 1.4) + fade / 6.5 + random.uniform(-0.9, 0.9)
        latency = 58.0 + max(0.0, -snr + 6.0) * 14.0 + random.uniform(-5.0, 10.0)
        drone_lat, drone_lon = destination_point(state.ground_lat, state.ground_lon, bearing_deg, range_m)
        sample = state.ingest(
            {
                "seq": seq,
                "rssi": round(rssi, 1),
                "snr": round(snr, 1),
                "latency_ms": round(latency, 1),
                "sf": sf,
                "bw_khz": 125,
                "lost": burst_loss,
                "ground_lat": state.ground_lat,
                "ground_lon": state.ground_lon,
                "drone_lat": drone_lat,
                "drone_lon": drone_lon,
                "altitude_m": 68.0 + 22.0 * math.sin(seq / 41.0),
                "antenna_heading_deg": state.antenna_heading_deg,
            }
        )
        logger.write(sample)
        rec = state.recommendation
        if rec.startswith("Increase"):
            sf = min(12, sf + 1)
        elif rec.startswith("Decrease"):
            sf = max(7, sf - 1)
        time.sleep(interval)


def serial_reader(state: LinkState, logger: CsvLogger, port: str, baud: int) -> None:
    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise SystemExit("pyserial is required for --serial. Run: pip install -r dashboard/requirements.txt") from exc

    with serial.Serial(port, baud, timeout=1) as ser:
        while True:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if not line:
                continue
            try:
                payload = json.loads(line)
            except json.JSONDecodeError:
                continue
            if payload.get("type") != "rx":
                continue
            logger.write(state.ingest(payload))


INDEX_HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>UAV Telemetry Link Monitor</title>
  <style>
    :root {
      color-scheme: light;
      --bg: #d9ddd7; --panel: #f1f2ed; --panel-2: #e2e5df;
      --text: #18211d; --muted: #5e675e; --grid: #8c948c;
      --good: #2d6f46; --warn: #9a6b12; --bad: #9b2f2f;
      --blue: #275c7a; --cyan: #2c766d; --orange: #9a5a18;
      --ink: #173b32; --maroon: #6d2631; --paper-line: #c9cec6;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: "Courier New", ui-monospace, SFMono-Regular, Consolas, monospace;
      background:
        linear-gradient(var(--paper-line) 1px, transparent 1px),
        linear-gradient(90deg, var(--paper-line) 1px, transparent 1px),
        var(--bg);
      background-size: 28px 28px;
      color: var(--text);
    }
    header {
      display: flex; justify-content: space-between; align-items: end; gap: 24px;
      padding: 22px 28px 16px;
      background: #c7ccc3;
      border-bottom: 4px double #565f57;
    }
    h1 {
      margin: 0;
      font-family: Georgia, "Times New Roman", serif;
      font-size: 27px;
      font-weight: 700;
      letter-spacing: 0;
      color: var(--ink);
    }
    .sub { color: var(--maroon); margin-top: 5px; font-size: 13px; font-weight: 700; }
    .status {
      display: flex; align-items: center; gap: 10px; color: var(--ink);
      font-size: 13px; white-space: nowrap; border: 1px solid #6f796f;
      padding: 8px 10px; background: #edf0e9;
    }
    .dot { width: 10px; height: 10px; border-radius: 50%; background: var(--warn); box-shadow: 0 0 0 2px #d2d7cd; }
    main { padding: 22px 28px 32px; }
    .ops { display: grid; grid-template-columns: minmax(420px, 1.18fr) minmax(320px, .82fr); gap: 14px; margin-bottom: 14px; }
    .map-panel, .geometry-panel, .metric, .chart, .table-wrap { background: var(--panel); border: 2px solid #6f796f; border-radius: 2px; box-shadow: 3px 3px 0 #aab1a8; }
    .map-panel { padding: 14px; min-height: 486px; }
    .panel-head { display: flex; justify-content: space-between; align-items: baseline; gap: 16px; margin-bottom: 10px; }
    .panel-head h2, .chart h2 { margin: 0 0 8px; font-size: 14px; font-weight: 700; color: var(--ink); text-transform: uppercase; }
    .panel-note { color: var(--muted); font-size: 12px; white-space: nowrap; }
    #map { width: 100%; height: 420px; display: block; background: #173b32; border: 2px solid #4d584f; border-radius: 1px; }
    .geometry-panel { padding: 14px; display: grid; grid-template-columns: 1fr 1fr; gap: 10px; align-content: start; }
    .bearing-card { background: var(--panel-2); border: 1px solid #899389; border-radius: 1px; padding: 13px; min-height: 82px; }
    .bearing-card .value { font-size: 24px; margin-top: 8px; }
    .wide { grid-column: 1 / -1; }
    .hint { color: var(--muted); font-size: 13px; line-height: 1.45; }
    .metrics { display: grid; grid-template-columns: repeat(6, minmax(120px, 1fr)); gap: 12px; margin-bottom: 18px; }
    .metric { padding: 15px; min-height: 92px; }
    .label { color: var(--maroon); font-size: 11px; text-transform: uppercase; letter-spacing: .08em; font-weight: 700; }
    .value { margin-top: 10px; font-size: 28px; font-weight: 700; letter-spacing: 0; color: var(--ink); }
    .unit { color: var(--muted); font-size: 14px; margin-left: 4px; }
    .charts { display: grid; grid-template-columns: 1fr 1fr; gap: 14px; }
    .chart { padding: 14px; min-height: 278px; }
    canvas { width: 100%; height: 220px; display: block; }
    .table-wrap { margin-top: 14px; overflow: hidden; }
    table { width: 100%; border-collapse: collapse; font-size: 13px; background: #f6f7f2; }
    th, td { padding: 9px 12px; border-bottom: 1px solid #b9c0b7; text-align: right; }
    th:first-child, td:first-child { text-align: left; }
    th { color: var(--maroon); font-weight: 700; background: #dce1d8; text-transform: uppercase; font-size: 11px; }
    tr:last-child td { border-bottom: 0; }
    @media (max-width: 1050px) { .ops, .charts { grid-template-columns: 1fr; } .metrics { grid-template-columns: repeat(3, 1fr); } }
    @media (max-width: 680px) {
      header { align-items: start; flex-direction: column; padding: 20px 16px 14px; }
      main { padding: 16px; } .map-panel { min-height: 360px; } #map { height: 300px; }
      .metrics, .geometry-panel { grid-template-columns: repeat(2, 1fr); }
      .value { font-size: 23px; }
      th:nth-child(5), td:nth-child(5), th:nth-child(6), td:nth-child(6), th:nth-child(7), td:nth-child(7) { display: none; }
    }
  </style>
</head>
<body>
  <header>
    <div>
      <h1>UBC ECE Field Link Monitor</h1>
      <div class="sub">Macleod roof station - LoRa 915 MHz - antenna geometry - telemetry log</div>
    </div>
    <div class="status"><span class="dot" id="dot"></span><span id="status">Waiting for packets</span></div>
  </header>
  <main>
    <section class="ops">
      <div class="map-panel">
        <div class="panel-head"><h2>Ground Station Antenna Geometry</h2><div class="panel-note">range board, north up</div></div>
        <canvas id="map"></canvas>
      </div>
      <div class="geometry-panel">
        <div class="bearing-card"><div class="label">Range</div><div class="value"><span id="range">--</span><span class="unit">m</span></div></div>
        <div class="bearing-card"><div class="label">Bearing</div><div class="value"><span id="bearing">--</span><span class="unit">deg</span></div></div>
        <div class="bearing-card"><div class="label">Antenna</div><div class="value"><span id="antenna">--</span><span class="unit">deg</span></div></div>
        <div class="bearing-card"><div class="label">Off Axis</div><div class="value"><span id="offaxis">--</span><span class="unit">deg</span></div></div>
        <div class="bearing-card"><div class="label">Altitude</div><div class="value"><span id="altitude">--</span><span class="unit">m</span></div></div>
        <div class="bearing-card"><div class="label">Coordinates</div><div class="value" style="font-size:14px; line-height:1.35"><span id="coords">--</span></div></div>
        <div class="wide hint">Lab note: keep the Yagi on the orange centerline. RSSI/SNR drops that line up with large off-axis angles usually indicate antenna orientation or polarization; drops inside the beam are more likely range, obstruction, or multipath.</div>
      </div>
    </section>
    <section class="metrics">
      <div class="metric"><div class="label">RSSI</div><div class="value"><span id="rssi">--</span><span class="unit">dBm</span></div></div>
      <div class="metric"><div class="label">SNR</div><div class="value"><span id="snr">--</span><span class="unit">dB</span></div></div>
      <div class="metric"><div class="label">Packet Loss</div><div class="value"><span id="loss">--</span><span class="unit">%</span></div></div>
      <div class="metric"><div class="label">Latency P95</div><div class="value"><span id="p95">--</span><span class="unit">ms</span></div></div>
      <div class="metric"><div class="label">Link Quality</div><div class="value"><span id="quality">--</span><span class="unit">/100</span></div></div>
      <div class="metric"><div class="label">SF Control</div><div class="value" style="font-size:20px"><span id="rec">Hold SF</span></div></div>
    </section>
    <section class="charts">
      <div class="chart"><h2>RSSI and SNR Trend</h2><canvas id="rf"></canvas></div>
      <div class="chart"><h2>Latency and Packet Loss Trend</h2><canvas id="latency"></canvas></div>
    </section>
    <section class="table-wrap">
      <table>
        <thead><tr><th>Time</th><th>Seq</th><th>Range</th><th>Off Axis</th><th>RSSI</th><th>SNR</th><th>Latency</th><th>SF</th><th>Lost</th><th>Quality</th></tr></thead>
        <tbody id="rows"></tbody>
      </table>
    </section>
  </main>
  <script>
    const colors = { grid: "#6e8175", text: "#eef4df", muted: "#c4d0bd", blue: "#b6c7ff", cyan: "#99e0c8", warn: "#ffd36a", bad: "#ff8a8a", good: "#89df8f", orange: "#ffb14e" };
    async function refresh() { const res = await fetch("/api/state", { cache: "no-store" }); render(await res.json()); }
    function render(state) {
      const latest = state.latest;
      if (!latest) return;
      setText("rssi", latest.rssi.toFixed(1)); setText("snr", latest.snr.toFixed(1)); setText("loss", latest.packet_loss_pct.toFixed(1));
      setText("p95", state.summary.latency_p95 ?? "--"); setText("quality", latest.link_quality); setText("rec", state.summary.recommendation);
      setText("range", latest.range_m.toFixed(0)); setText("bearing", latest.bearing_deg.toFixed(0)); setText("antenna", latest.antenna_heading_deg.toFixed(0));
      setText("offaxis", latest.off_axis_deg.toFixed(0)); setText("altitude", latest.altitude_m.toFixed(0)); setText("coords", `${latest.drone_lat.toFixed(5)}, ${latest.drone_lon.toFixed(5)}`);
      setText("status", `${state.summary.packets} packets - ${state.summary.lost} lost - SF${latest.sf}/${latest.bw_khz} kHz`);
      document.getElementById("dot").style.background = latest.link_quality > 75 ? colors.good : latest.link_quality > 48 ? colors.warn : colors.bad;
      drawMap("map", state.samples);
      drawChart("rf", state.samples, [{ key: "rssi", label: "RSSI", color: colors.blue, min: -125, max: -70 }, { key: "snr", label: "SNR", color: colors.cyan, min: -10, max: 16 }]);
      drawChart("latency", state.samples, [{ key: "latency_ms", label: "Latency", color: colors.warn, min: 30, max: 240 }, { key: "packet_loss_pct", label: "Loss", color: colors.bad, min: 0, max: 35 }]);
      renderRows(state.samples.slice(-12).reverse());
    }
    function setText(id, value) { document.getElementById(id).textContent = value; }
    function setupCanvas(id) {
      const canvas = document.getElementById(id), dpr = window.devicePixelRatio || 1, rect = canvas.getBoundingClientRect();
      canvas.width = Math.max(1, Math.floor(rect.width * dpr)); canvas.height = Math.max(1, Math.floor(rect.height * dpr));
      const ctx = canvas.getContext("2d"); ctx.scale(dpr, dpr); return { ctx, w: rect.width, h: rect.height };
    }
    function polarToXY(range, bearing, cx, cy, scale) { const rad = bearing * Math.PI / 180; return { x: cx + Math.sin(rad) * range * scale, y: cy - Math.cos(rad) * range * scale }; }
    function drawMap(id, samples) {
      const { ctx, w, h } = setupCanvas(id); ctx.clearRect(0, 0, w, h);
      const cx = w / 2, cy = h / 2, latest = samples[samples.length - 1]; if (!latest) return;
      const maxRange = Math.max(500, ...samples.map(s => s.range_m || 0)); const scale = Math.min(w, h) * 0.42 / Math.max(500, maxRange * 1.12);
      drawGrid(ctx, cx, cy, scale, maxRange); drawBeam(ctx, cx, cy, scale, latest.antenna_heading_deg, maxRange * 1.05);
      drawTrack(ctx, samples, cx, cy, scale); drawGround(ctx, cx, cy, latest.antenna_heading_deg); drawAircraft(ctx, latest, cx, cy, scale); drawLabels(ctx, latest, cx, cy, scale);
    }
    function drawGrid(ctx, cx, cy, scale, maxRange) {
      ctx.strokeStyle = colors.grid; ctx.lineWidth = 1; const step = maxRange > 1800 ? 500 : 250;
      for (let r = step; r <= maxRange * 1.15; r += step) { ctx.beginPath(); ctx.arc(cx, cy, r * scale, 0, Math.PI * 2); ctx.stroke(); ctx.fillStyle = colors.muted; ctx.font = "12px system-ui"; ctx.fillText(`${r} m`, cx + 8, cy - r * scale - 4); }
      ctx.beginPath(); ctx.moveTo(cx, 10); ctx.lineTo(cx, cy * 2 - 10); ctx.moveTo(10, cy); ctx.lineTo(cx * 2 - 10, cy); ctx.stroke();
      ctx.fillStyle = colors.text; ctx.font = "12px system-ui"; ctx.fillText("N", cx - 4, 18);
    }
    function drawBeam(ctx, cx, cy, scale, heading, range) {
      const half = 28, p1 = polarToXY(range, heading - half, cx, cy, scale), pc = polarToXY(range, heading, cx, cy, scale);
      ctx.fillStyle = "rgba(245,158,11,.15)"; ctx.strokeStyle = "rgba(245,158,11,.75)"; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(p1.x, p1.y); ctx.arc(cx, cy, range * scale, (heading - half - 90) * Math.PI / 180, (heading + half - 90) * Math.PI / 180); ctx.closePath(); ctx.fill(); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(pc.x, pc.y); ctx.stroke();
    }
    function drawTrack(ctx, samples, cx, cy, scale) {
      ctx.lineWidth = 2; ctx.strokeStyle = colors.blue; ctx.beginPath();
      samples.forEach((s, i) => { const p = polarToXY(s.range_m, s.bearing_deg, cx, cy, scale); if (i === 0) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y); }); ctx.stroke();
      samples.slice(-18).forEach(s => { const p = polarToXY(s.range_m, s.bearing_deg, cx, cy, scale); ctx.fillStyle = s.link_quality > 75 ? colors.good : s.link_quality > 48 ? colors.warn : colors.bad; ctx.beginPath(); ctx.arc(p.x, p.y, 2.8, 0, Math.PI * 2); ctx.fill(); });
    }
    function drawGround(ctx, cx, cy, heading) {
      ctx.fillStyle = colors.text; ctx.beginPath(); ctx.arc(cx, cy, 7, 0, Math.PI * 2); ctx.fill();
      const tip = polarToXY(34, heading, cx, cy, 1); ctx.strokeStyle = colors.orange; ctx.lineWidth = 3; ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(tip.x, tip.y); ctx.stroke();
      ctx.fillStyle = colors.muted; ctx.font = "12px system-ui"; ctx.fillText("GS", cx + 10, cy + 18);
    }
    function drawAircraft(ctx, s, cx, cy, scale) {
      const p = polarToXY(s.range_m, s.bearing_deg, cx, cy, scale); ctx.fillStyle = colors.cyan; ctx.strokeStyle = "#0f1720"; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(p.x, p.y - 10); ctx.lineTo(p.x + 11, p.y + 9); ctx.lineTo(p.x, p.y + 4); ctx.lineTo(p.x - 11, p.y + 9); ctx.closePath(); ctx.fill(); ctx.stroke();
    }
    function drawLabels(ctx, s, cx, cy, scale) {
      const p = polarToXY(s.range_m, s.bearing_deg, cx, cy, scale); ctx.strokeStyle = "rgba(237,242,245,.55)"; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(p.x, p.y); ctx.stroke();
      ctx.fillStyle = colors.text; ctx.font = "13px system-ui"; ctx.fillText(`${s.range_m.toFixed(0)} m / ${s.bearing_deg.toFixed(0)} deg`, p.x + 12, p.y - 12);
      ctx.fillStyle = colors.muted; ctx.fillText(`off-axis ${s.off_axis_deg.toFixed(0)} deg`, p.x + 12, p.y + 5);
    }
    function drawChart(id, samples, series) {
      const { ctx, w, h } = setupCanvas(id), pad = 30; ctx.clearRect(0, 0, w, h); ctx.strokeStyle = colors.grid; ctx.lineWidth = 1;
      for (let i = 0; i <= 4; i++) { const y = pad + i * (h - pad * 1.7) / 4; ctx.beginPath(); ctx.moveTo(pad, y); ctx.lineTo(w - 8, y); ctx.stroke(); }
      series.forEach((s, si) => { ctx.strokeStyle = s.color; ctx.lineWidth = 2; ctx.beginPath(); samples.forEach((sample, i) => { const x = pad + i * (w - pad - 12) / Math.max(1, samples.length - 1); const y = pad + (1 - ((sample[s.key] - s.min) / (s.max - s.min))) * (h - pad * 1.7); if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y); }); ctx.stroke(); ctx.fillStyle = s.color; ctx.fillText(s.label, pad + si * 88, 14); });
    }
    function renderRows(samples) {
      document.getElementById("rows").innerHTML = samples.map(s => {
        const t = new Date(s.received_at).toLocaleTimeString();
        return `<tr><td>${t}</td><td>${s.seq}</td><td>${s.range_m.toFixed(0)} m</td><td>${s.off_axis_deg.toFixed(0)} deg</td><td>${s.rssi.toFixed(1)}</td><td>${s.snr.toFixed(1)}</td><td>${s.latency_ms.toFixed(1)} ms</td><td>SF${s.sf}</td><td>${s.lost}</td><td>${s.link_quality}</td></tr>`;
      }).join("");
    }
    setInterval(refresh, 1000); refresh();
  </script>
</body>
</html>"""


class DashboardHandler(BaseHTTPRequestHandler):
    state: LinkState

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path == "/":
            self.respond(200, "text/html; charset=utf-8", INDEX_HTML.encode("utf-8"))
        elif path == "/api/state":
            body = json.dumps(self.state.snapshot()).encode("utf-8")
            self.respond(200, "application/json", body)
        else:
            self.respond(404, "text/plain", b"Not found")

    def respond(self, status: int, content_type: str, body: bytes) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt: str, *args: Any) -> None:
        return


def main() -> None:
    parser = argparse.ArgumentParser(description="UAV LoRa telemetry link dashboard")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--simulate", action="store_true", help="generate synthetic link samples")
    mode.add_argument("--serial", help="serial port for ground station JSON, for example COM7")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--interval", type=float, default=1.0, help="simulated packet interval in seconds")
    parser.add_argument("--log", type=Path, default=Path("flight_logs") / "link_log.csv")
    parser.add_argument("--ground-lat", type=float, default=49.262369, help="ground station latitude")
    parser.add_argument("--ground-lon", type=float, default=-123.250118, help="ground station longitude")
    parser.add_argument("--antenna-heading", type=float, default=45.0, help="ground antenna heading in degrees true")
    args = parser.parse_args()

    state = LinkState(
        ground_lat=args.ground_lat,
        ground_lon=args.ground_lon,
        antenna_heading_deg=normalize_degrees(args.antenna_heading),
    )
    logger = CsvLogger(args.log)
    DashboardHandler.state = state

    if args.simulate:
        worker = threading.Thread(target=simulator, args=(state, logger, args.interval), daemon=True)
    else:
        worker = threading.Thread(target=serial_reader, args=(state, logger, args.serial, args.baud), daemon=True)
    worker.start()

    server = ThreadingHTTPServer((args.host, args.port), DashboardHandler)
    print(f"Dashboard running at http://{args.host}:{args.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping dashboard")


if __name__ == "__main__":
    main()
