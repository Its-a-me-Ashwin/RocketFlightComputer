// =============================================================================
// LoRa ↔ Serial bridge (Feather RP2040 RFM95 or compatible)
// TDD two-way until the FC enters ARMED: receives 28-byte telemetry from the
// rocket FC over LoRa and forwards to USB Serial; forwards any serial line
// from the ground station (START/ARM/DISARM/SET/CFG?/SAVECFG/PING) over LoRa,
// repeated 3x to punch through FC telemetry bursts. FC text replies (ACK/NACK/
// CFG/PONG) are forwarded to the ground station as lines.
// From ARMED onward the FC stops listening — telemetry flows one way only.
// Connect this board's USB serial to the ground station at 57600.
// Packet format: preamble "RC", time_ms, alt_cm, ...
// Bridge LED (GPIO13, onboard): slow 1 Hz blink = waiting for rocket (no telemetry
//   >2 s); 40 ms flicker per packet from the rocket; 250 ms solid pulse when a
//   command is forwarded over LoRa; 200 ms blink = LoRa init failed.
// =============================================================================

#include <SPI.h>
#include <RH_RF95.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN 25
#endif

// LoRa pins (Feather RP2040 RFM95)
// IMPORTANT: SPI pins are set explicitly in setup() — a wrong IDE board variant
// (e.g. plain "Raspberry Pi Pico") maps MISO to GP16 = RFM95 CS and the radio
// never works. Select "Adafruit Feather RP2040" (earlephilhower core).
#define RFM95_CS   16
#define RFM95_INT  21
#define RFM95_RST  17
#define RFM95_SCK  18
#define RFM95_MOSI 19
#define RFM95_MISO 20
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

// Commands come from groundStation.py as newline-terminated lines ("START\n",
// "ARM\n", "SET main_deploy_alt_m 250\n", ...). All are forwarded over LoRa.
RH_RF95 rf95(RFM95_CS, RFM95_INT);

uint32_t packets_forwarded = 0;

// Line buffer for commands from Serial (ground station)
static char cmd_line[64];
static uint8_t cmd_len = 0;

// LED pattern state:
//   - no telemetry for >2 s        -> slow 1 Hz blink ("waiting for rocket")
//   - each packet from the rocket  -> 40 ms flicker
//   - command forwarded over LoRa  -> 250 ms solid pulse
static uint32_t last_telemetry_ms = 0;
static uint32_t led_pulse_until_ms = 0;

// TX-done wait with timeout: a dead/misconfigured radio must never hang the bridge
static bool wait_tx_done(uint32_t timeout_ms) {
  uint32_t t0 = millis();
  while (rf95.mode() == RHGenericDriver::RHModeTx) {
    if (millis() - t0 >= timeout_ms) return false;
    delay(1);
  }
  return true;
}

void send_command_over_lora(const char* cmd) {
  uint8_t len = (uint8_t)strlen(cmd);
  if (len == 0 || len > RH_RF95_MAX_MESSAGE_LEN) return;
  rf95.send((const uint8_t*)cmd, len);
  wait_tx_done(200);
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
        Serial.print("[BRIDGE] >LoRa: ");
        Serial.println(cmd_line);
        led_pulse_until_ms = millis() + 250;  // solid pulse: "command sent"
        send_command_over_lora_reliable(cmd_line);
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

  // Pin SPI explicitly: wrong board variant (e.g. plain Pico) maps MISO to GP16 = RFM95 CS
  SPI.setSCK(RFM95_SCK);
  SPI.setTX(RFM95_MOSI);
  SPI.setRX(RFM95_MISO);

  Serial.println("LoRa Bridge v2.1 (FC) starting — half-duplex: RX telemetry, TX commands...");

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

  Serial.println("LoRa ready. Telemetry to Serial; any serial line forwarded 3x over LoRa (TDD until FC is ARMED).");
  Serial.print("Packet size: ");
  Serial.println(PACKET_SIZE);
}

void loop() {
  // Forward ground station commands (any text line) over LoRa
  poll_serial_commands();
  uint32_t now = millis();

  // Receive packets from rocket: 28-byte "RC" telemetry → raw binary to Serial;
  // text replies (ACK/NACK/CFG/PONG) → line to Serial
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      bool is_telemetry = (len == PACKET_SIZE && buf[0] == 'R' && buf[1] == 'C');
      bool is_text = !is_telemetry;
      if (is_text) {
        for (uint8_t i = 0; i < len; i++) {
          if (buf[i] < 0x20 || buf[i] > 0x7E) { is_text = false; break; }
        }
      }
      if (is_telemetry) {
        Serial.write(buf, len);
        packets_forwarded++;
        last_telemetry_ms = now;
        led_pulse_until_ms = now + 40;  // short flicker per packet
      } else if (is_text && len > 0) {
        Serial.write(buf, len);
        Serial.write('\n');
        led_pulse_until_ms = now + 40;
      }
    }
  }

  // LED pattern: pulse (command/packet) > 1 Hz blink when no telemetry >2 s > off
  if (now < led_pulse_until_ms) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else if (now - last_telemetry_ms > 2000) {
    digitalWrite(LED_BUILTIN, (now % 1000) < 500);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }

  // Link stats to the ground station every 5 s (packets + last RSSI)
  static uint32_t last_stats_ms = 0;
  static int16_t last_rssi = 0;
  if (packets_forwarded > 0) last_rssi = rf95.lastRssi();
  if (now - last_stats_ms >= 5000) {
    last_stats_ms = now;
    Serial.print("[BRIDGE] tele pkts=");
    Serial.print(packets_forwarded);
    Serial.print(" last RSSI=");
    Serial.println(last_rssi);
  }
  delay(1);
}
