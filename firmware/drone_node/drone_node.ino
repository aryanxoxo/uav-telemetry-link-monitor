/*
  UAV Telemetry Drone Node

  ESP32 + SX1276/SX1278 LoRa transmitter at 915 MHz.
  Requires RadioLib: https://github.com/jgromes/RadioLib

  Packet roles:
    TEL: flight telemetry payload sent by the UAV.
    ACK: ground station echo carrying the received sequence number.
    CFG: ground station command to adjust SF/BW conservatively.
*/

#include <Arduino.h>
#include <RadioLib.h>

static constexpr int PIN_LORA_NSS = 5;
static constexpr int PIN_LORA_DIO0 = 26;
static constexpr int PIN_LORA_RST = 14;
static constexpr int PIN_LORA_DIO1 = 33;

static constexpr float LORA_FREQ_MHZ = 915.0;
static constexpr int DEFAULT_SF = 9;
static constexpr float DEFAULT_BW_KHZ = 125.0;
static constexpr int CODING_RATE = 7;      // 4/7 adds resilience for bursty low-altitude fades.
static constexpr int TX_POWER_DBM = 17;
static constexpr uint32_t TELEMETRY_INTERVAL_MS = 1000;
static constexpr size_t TX_HISTORY = 32;

SX1276 radio = new Module(PIN_LORA_NSS, PIN_LORA_DIO0, PIN_LORA_RST, PIN_LORA_DIO1);

struct TxStamp {
  uint16_t seq;
  uint32_t sentAt;
};

TxStamp txHistory[TX_HISTORY];
uint16_t seq = 0;
uint8_t spreadingFactor = DEFAULT_SF;
float bandwidthKhz = DEFAULT_BW_KHZ;
uint32_t lastTelemetryAt = 0;

void configureRadio(uint8_t sf, float bwKhz) {
  radio.setFrequency(LORA_FREQ_MHZ);
  radio.setSpreadingFactor(sf);
  radio.setBandwidth(bwKhz);
  radio.setCodingRate(CODING_RATE);
  radio.setOutputPower(TX_POWER_DBM);
  radio.setCRC(true);
  radio.setPreambleLength(8);
}

void rememberTx(uint16_t packetSeq, uint32_t sentAt) {
  txHistory[packetSeq % TX_HISTORY] = {packetSeq, sentAt};
}

uint32_t lookupTxTime(uint16_t packetSeq) {
  TxStamp stamp = txHistory[packetSeq % TX_HISTORY];
  return stamp.seq == packetSeq ? stamp.sentAt : 0;
}

String buildTelemetryPacket() {
  seq++;

  // Replace these simulated flight values with MAVLink, sensor, or flight-controller data.
  float altitudeM = 42.0 + 4.0 * sin(millis() / 9000.0);
  float groundSpeedMps = 13.2 + 1.6 * sin(millis() / 6000.0);
  float batteryV = 15.6 - min(2.2, millis() / 1000000.0);
  int fix = 3;

  String packet = "TEL,";
  packet += seq;
  packet += ",";
  packet += millis();
  packet += ",";
  packet += String(altitudeM, 1);
  packet += ",";
  packet += String(groundSpeedMps, 1);
  packet += ",";
  packet += String(batteryV, 2);
  packet += ",";
  packet += fix;
  return packet;
}

void handleAck(const String& packet) {
  // ACK,<seq>,<ground_rx_ms>
  int first = packet.indexOf(',');
  int second = packet.indexOf(',', first + 1);
  if (first < 0 || second < 0) return;

  uint16_t ackSeq = packet.substring(first + 1, second).toInt();
  uint32_t sentAt = lookupTxTime(ackSeq);
  if (sentAt == 0) return;

  uint32_t rtt = millis() - sentAt;
  Serial.print("{\"type\":\"ack\",\"seq\":");
  Serial.print(ackSeq);
  Serial.print(",\"rtt_ms\":");
  Serial.print(rtt);
  Serial.println("}");
}

void handleConfig(const String& packet) {
  // CFG,SF,<sf>,BW,<bw_khz>
  int sfMarker = packet.indexOf("SF,");
  int bwMarker = packet.indexOf("BW,");
  if (sfMarker < 0) return;

  uint8_t requestedSf = packet.substring(sfMarker + 3, sfMarker + 5).toInt();
  float requestedBw = bandwidthKhz;
  if (bwMarker >= 0) {
    requestedBw = packet.substring(bwMarker + 3).toFloat();
  }

  if (requestedSf < 7 || requestedSf > 12) return;
  spreadingFactor = requestedSf;
  bandwidthKhz = requestedBw;
  configureRadio(spreadingFactor, bandwidthKhz);

  Serial.print("{\"type\":\"cfg\",\"sf\":");
  Serial.print(spreadingFactor);
  Serial.print(",\"bw_khz\":");
  Serial.print(bandwidthKhz, 0);
  Serial.println("}");
}

void pollDownlink() {
  String packet;
  int state = radio.receive(packet, 20);
  if (state != RADIOLIB_ERR_NONE) return;

  if (packet.startsWith("ACK,")) {
    handleAck(packet);
  } else if (packet.startsWith("CFG,")) {
    handleConfig(packet);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  int state = radio.begin(LORA_FREQ_MHZ);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("LoRa init failed: ");
    Serial.println(state);
    while (true) delay(1000);
  }

  configureRadio(spreadingFactor, bandwidthKhz);
  Serial.println("{\"type\":\"boot\",\"role\":\"drone_node\",\"freq_mhz\":915,\"sf\":9,\"bw_khz\":125}");
}

void loop() {
  pollDownlink();

  if (millis() - lastTelemetryAt >= TELEMETRY_INTERVAL_MS) {
    String packet = buildTelemetryPacket();
    uint32_t sentAt = millis();
    int state = radio.transmit(packet);
    if (state == RADIOLIB_ERR_NONE) {
      rememberTx(seq, sentAt);
      Serial.print("{\"type\":\"tx\",\"seq\":");
      Serial.print(seq);
      Serial.print(",\"timestamp_ms\":");
      Serial.print(sentAt);
      Serial.println("}");
    }
    lastTelemetryAt = millis();
  }
}
