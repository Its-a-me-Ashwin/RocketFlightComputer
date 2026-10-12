// =============================================================================
// LoRa ↔ Serial bridge (Feather RP2040 RFM95 or compatible)
// Half-duplex: Receives 28-byte telemetry from the rocket FC over LoRa and forwards to
// USB Serial. When ground station sends START or ARM over Serial, forwards
// that command over LoRa to the rocket (pre-boost only on FC side).
// Connect this board's USB serial to the ground station at 57600.
// Packet format: preamble "RC", time_ms, alt_cm, ...
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

// 28-byte telemetry packet (must match FC)
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

// Command strings sent from ground station (groundStation.py sends "START\n" / "ARM\n")
#define CMD_START  "START"
#define CMD_ARM    "ARM"

RH_RF95 rf95(RFM95_CS, RFM95_INT);

uint32_t packets_forwarded = 0;

// Line buffer for commands from Serial (ground station)
static char cmd_line[32];
static uint8_t cmd_len = 0;

void send_command_over_lora(const char* cmd) {
  uint8_t len = (uint8_t)strlen(cmd);
  if (len == 0 || len > RH_RF95_MAX_MESSAGE_LEN) return;
  rf95.send((const uint8_t*)cmd, len);
  rf95.waitPacketSent();
  rf95.setModeRx();
}

// Half-duplex: FC may be TXing telemetry; repeat command a few times to hit an RX window
void send_command_over_lora_reliable(const char* cmd) {
  for (int i = 0; i < 3; i++) {
    send_command_over_lora(cmd);
    if (i < 2) delay(45);
  }
}

void poll_serial_commands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      cmd_line[cmd_len] = '\0';
      if (cmd_len > 0) {
        if (strcmp(cmd_line, CMD_START) == 0) {
          send_command_over_lora_reliable(CMD_START);
        } else if (strcmp(cmd_line, CMD_ARM) == 0) {
          send_command_over_lora_reliable(CMD_ARM);
        }
        cmd_len = 0;
      }
    } else {
      if (cmd_len < (sizeof(cmd_line) - 1)) {
        cmd_line[cmd_len++] = c;
      }
    }
  }
}

void setup() {
  Serial.begin(57600);
  delay(2000);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println("LoRa Bridge (FC) starting — half-duplex: RX telemetry, TX START/ARM...");

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

  rf95.setTxPower(10, false);
  rf95.setModeRx();

  Serial.println("LoRa ready. Telemetry to Serial; START/ARM repeated 3x over LoRa for reliability.");
  Serial.print("Packet size: ");
  Serial.println(PACKET_SIZE);
}

void loop() {
  // Forward ground station commands (START / ARM) over LoRa
  poll_serial_commands();

  // Receive telemetry from rocket and forward to ground station
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
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
