// =============================================================================
// NRF24L01 → Serial bridge (Raspberry Pi Pico)
// Receives telemetry packets from FC2 over NRF24L01 (SPI) and forwards them
// as raw binary on USB Serial. Connect this Pico's USB serial port to the
// ground station as "FC2" COM port.
// Same pipe address and payload size as RocketComputerFC2.
// Pins: SPI0 GP4=MISO, GP6=SCK, GP7=MOSI; GP14=CSN, GP17=CE
// =============================================================================

#include <SPI.h>
#include <RF24.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN 25
#endif

// NRF24L01 pins (match FC2: GP14=CSN, GP17=CE)
static constexpr uint8_t NRF_CE_PIN  = 17;
static constexpr uint8_t NRF_CSN_PIN = 14;

// Must match FC2's openWritingPipe() address
static const uint64_t NRF_PIPE_ADDR = 0xE7E7E7E7E7LL;

// Telemetry packet (28 bytes, same as FC1/FC2 with roll/pitch/yaw)
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

RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
TelemetryPacket pkt;
uint32_t packets_received = 0;
uint32_t last_status_ms = 0;

void setup() {
  Serial.begin(57600);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println("\n=== NRF24 Serial Bridge (FC2 ground station) ===");

  SPI.setRX(4);
  SPI.setTX(7);
  SPI.setSCK(6);
  if (!radio.begin()) {
    Serial.println("NRF24L01 FAILED. Check CE/CSN/SPI wiring.");
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
  }

  radio.setPayloadSize(sizeof(TelemetryPacket));
  radio.openReadingPipe(1, NRF_PIPE_ADDR);
  radio.startListening();
  radio.setPALevel(RF24_PA_LOW);

  Serial.print("Listening on pipe 0x");
  Serial.println((unsigned long long)NRF_PIPE_ADDR, HEX);
  Serial.print("Payload size: ");
  Serial.println(sizeof(TelemetryPacket));
  Serial.println("Forwarding packets to USB Serial. Connect this port as FC2 in ground station.");
}

void loop() {
  if (radio.available()) {
    radio.read(&pkt, sizeof(pkt));
    Serial.write(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt));
    packets_received++;
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
  // Every 5 s print packet count (when Serial Monitor open on bridge at 57600, shows if link works)
  uint32_t now = millis();
  if (now - last_status_ms >= 5000) {
    last_status_ms = now;
    Serial.print("[bridge] pkts received: ");
    Serial.println(packets_received);
  }
  delay(1);
}
