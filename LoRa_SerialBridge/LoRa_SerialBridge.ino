// =============================================================================
// LoRa → Serial bridge (Feather RP2040 RFM95 or compatible)
// Receives 28-byte binary telemetry packets from FC3 over LoRa (915 MHz) and
// forwards them raw on USB Serial. Connect this board's USB serial port to the
// ground station as FC2 (or FC3) COM port at 57600 baud for visualization.
// Packet format: same as FC1/FC2 (preamble "RC", time_ms, alt_cm, vel_cmps,
// lat_1e7, lon_1e7, state, roll_centi, pitch_centi, yaw_centi, checksum).
// =============================================================================

#include <SPI.h>
#include <RH_RF95.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN 25
#endif

// LoRa pins (Feather RP2040 RFM95)
#define RFM95_CS   16
#define RFM95_INT  21
#define RFM95_RST  17
#define RF95_FREQ  915.0

// 28-byte telemetry packet (must match FC3 / FC1 / FC2)
#pragma pack(push, 1)
struct TelemetryPacket {
  uint8_t  preamble[2];
  uint32_t time_ms;
  int32_t  alt_cm;
  int16_t  vel_cmps;
  int32_t  lat_1e7;
  int32_t  lon_1e7;
  uint8_t  state;
  int16_t  roll_centi;
  int16_t  pitch_centi;
  int16_t  yaw_centi;
  uint8_t  checksum;
};
#pragma pack(pop)

static constexpr size_t PACKET_SIZE = sizeof(TelemetryPacket);

RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Only write binary packets to Serial — no status prints in loop, so ground
// station sees a clean stream of 28-byte packets.
uint32_t packets_forwarded = 0;

void setup() {
  Serial.begin(57600);
  delay(2000);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println("LoRa Receiver (FC3 bridge) starting...");

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    Serial.println("LoRa init failed!");
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
  }

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("Frequency set failed!");
    while (1) delay(100);
  }

  Serial.println("LoRa ready. Listening for FC3 telemetry...");
  Serial.print("Packet size: ");
  Serial.println(PACKET_SIZE);
  Serial.println("Forwarding binary packets to USB Serial. Connect this port in ground station at 57600.");
}

void loop() {
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      // Only forward if it looks like our 28-byte telemetry packet
      if (len == PACKET_SIZE && buf[0] == 'R' && buf[1] == 'C') {
        Serial.write(buf, len);
        packets_forwarded++;
        digitalWrite(LED_BUILTIN, HIGH);
      }
    }
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
  delay(1);
}
