/*
  UAV Telemetry Ground Station

  ESP32 + SX1276/SX1278 LoRa receiver at 915 MHz.
  Emits newline-delimited JSON over USB serial for telemetry_dashboard.py.
*/

#include <Arduino.h>
#include <RadioLib.h>

static constexpr int PIN_LORA_NSS = 5;
static constexpr int PIN_LORA_DIO0 = 26;
static constexpr int PIN_LORA_RST = 14;
static constexpr int PIN_LORA_DIO1 = 33;

static constexpr float LORA_FREQ_MHZ = 915.0;
static constexpr uint8_t DEFAULT_SF = 9;
static constexpr float DEFAULT_BW_KHZ = 125.0;
static constexpr int CODING_RATE = 7;
static constexpr int TX_POWER_DBM = 17;
static constexpr uint32_t ADAPT_INTERVAL_MS = 12000;

SX1276 radio = new Module(PIN_LORA_NSS, PIN_LORA_DIO0, PIN_LORA_RST, PIN_LORA_DIO1);

uint16_t lastSeq = 0;
uint32_t packetCount = 0;
uint32_t lostCount = 0;
uint8_t spreadingFactor = DEFAULT_SF;
float bandwidthKhz = DEFAULT_BW_KHZ;
float rssiAvg = -120.0;
float snrAvg = -10.0;
uint32_t lastAdaptAt = 0;

void configureRadio(uint8_t sf, float bwKhz) {
  radio.setFrequency(LORA_FREQ_MHZ);
  radio.setSpreadingFactor(sf);
  radio.setBandwidth(bwKhz);
  radio.setCodingRate(CODING_RATE);
  radio.setOutputPower(TX_POWER_DBM);
  radio.setCRC(true);
  radio.setPreambleLength(8);
}

uint16_t parseSeq(const String& packet) {
  // TEL,<seq>,<tx_ms>,<alt_m>,<groundspeed_mps>,<battery_v>,<gps_fix>
  int first = packet.indexOf(',');
  int second = packet.indexOf(',', first + 1);
  if (first < 0 || second < 0) return 0;
  return packet.substring(first + 1, second).toInt();
}

uint32_t parseTxMillis(const String& packet) {
  int first = packet.indexOf(',');
  int second = packet.indexOf(',', first + 1);
  int third = packet.indexOf(',', second + 1);
  if (second < 0 || third < 0) return 0;
  return packet.substring(second + 1, third).toInt();
}

uint16_t estimateLost(uint16_t packetSeq) {
  if (lastSeq == 0 || packetSeq <= lastSeq + 1) return 0;
  return packetSeq - lastSeq - 1;
}

void echoAck(uint16_t packetSeq) {
  String ack = "ACK,";
  ack += packetSeq;
  ack += ",";
  ack += millis();
  radio.transmit(ack);
  radio.startReceive();
}

void maybeAdaptSf() {
  if (millis() - lastAdaptAt < ADAPT_INTERVAL_MS || packetCount < 18) return;

  float lossPct = 100.0f * lostCount / max<uint32_t>(1, packetCount + lostCount);
  uint8_t requestedSf = spreadingFactor;

  if ((lossPct > 8.0 || snrAvg < 4.0 || rssiAvg < -112.0) && spreadingFactor < 12) {
    requestedSf++;
  } else if (lossPct < 2.0 && snrAvg > 9.0 && rssiAvg > -98.0 && spreadingFactor > 7) {
    requestedSf--;
  }

  if (requestedSf != spreadingFactor) {
    String cfg = "CFG,SF,";
    cfg += requestedSf;
    cfg += ",BW,";
    cfg += String(bandwidthKhz, 0);
    radio.transmit(cfg);
    spreadingFactor = requestedSf;
    configureRadio(spreadingFactor, bandwidthKhz);
    radio.startReceive();

    Serial.print("{\"type\":\"adapt\",\"sf\":");
    Serial.print(spreadingFactor);
    Serial.print(",\"loss_pct\":");
    Serial.print(lossPct, 1);
    Serial.print(",\"rssi_avg\":");
    Serial.print(rssiAvg, 1);
    Serial.print(",\"snr_avg\":");
    Serial.print(snrAvg, 1);
    Serial.println("}");
  }

  lastAdaptAt = millis();
}

void emitRxJson(uint16_t packetSeq, uint16_t lost, float rssi, float snr, float latencyMs) {
  Serial.print("{\"type\":\"rx\",\"seq\":");
  Serial.print(packetSeq);
  Serial.print(",\"rssi\":");
  Serial.print(rssi, 1);
  Serial.print(",\"snr\":");
  Serial.print(snr, 1);
  Serial.print(",\"latency_ms\":");
  Serial.print(latencyMs, 1);
  Serial.print(",\"sf\":");
  Serial.print(spreadingFactor);
  Serial.print(",\"bw_khz\":");
  Serial.print(bandwidthKhz, 0);
  Serial.print(",\"lost\":");
  Serial.print(lost);
  Serial.print(",\"timestamp_ms\":");
  Serial.print(millis());
  Serial.println("}");
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
  radio.startReceive();
  Serial.println("{\"type\":\"boot\",\"role\":\"ground_station\",\"freq_mhz\":915,\"sf\":9,\"bw_khz\":125}");
}

void loop() {
  String packet;
  int state = radio.receive(packet, 50);
  if (state == RADIOLIB_ERR_NONE && packet.startsWith("TEL,")) {
    uint16_t packetSeq = parseSeq(packet);
    uint32_t txMs = parseTxMillis(packet);
    uint16_t lost = estimateLost(packetSeq);
    float rssi = radio.getRSSI();
    float snr = radio.getSNR();
    float latencyMs = txMs > 0 ? (float)(millis() - txMs) : 0.0;

    packetCount++;
    lostCount += lost;
    lastSeq = packetSeq;
    rssiAvg = 0.85 * rssiAvg + 0.15 * rssi;
    snrAvg = 0.85 * snrAvg + 0.15 * snr;

    emitRxJson(packetSeq, lost, rssi, snr, latencyMs);
    echoAck(packetSeq);
  }

  maybeAdaptSf();
}
