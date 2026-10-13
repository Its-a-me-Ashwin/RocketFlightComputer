// =============================================================================
// ROCKET FLIGHT COMPUTER
// Board: Adafruit Feather RP2040 RFM95 (inbuilt LoRa)
// IMU: Adafruit LSM6DSOX (accel + gyro only, no magnetometer)
// Baro: Adafruit BMP390 (BMP3XX library)
// Telemetry: RH_RF95 LoRa 915 MHz (28-byte binary packet)
// GPS: UART0 on GP0 (TX) / GP1 (RX), 9600 baud — lat/lon in telemetry when fix valid; if lost, send sentinel and continue
// ARM/DISARM: USB Serial + LoRa (TDD two-way until ARMED; one-way rocket→ground from ARMED)
// Settings: SET/CFG?/SAVECFG text commands over USB or LoRa (READY/INIT only), persisted to flash
// Telemetry: 1 Hz beacon from power-up (fields 0 pre-cal), 10 Hz once calibrated
// FC LED (GPIO13, onboard): READY/CALIBRATING = 1 Hz blink, INIT = 2-burst, ARMED = 4 Hz blink,
//       BOOST = solid, APOGEE/DROGUE = 3-burst, MAIN/FINISHED = 4-burst/heartbeat,
//       + 150 ms solid flash whenever a LoRa command is received
// Pins: I2C SDA=2, SCL=3; LoRa CS=16, INT=21, RST=17
//       Pyro: GP9=MAIN, GP10=DROGUE, GP12=AIRSTART
// =============================================================================

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_LSM6DSOX.h>
#include <RH_RF95.h>
#include <TinyGPSPlus.h>
#include <EEPROM.h>
#include <math.h>
#include <limits.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN 25
#endif

enum class FlightState : uint8_t;
enum class LedPattern  : uint8_t;
enum class CfgType : uint8_t;
enum class ReplyChannel : uint8_t;

const char* stateToString(FlightState s);
LedPattern pattern_for_state(bool calibrated, FlightState s, bool armed_flag);
void led_update(uint32_t now_ms, LedPattern pat);

struct TelemetryPacket;
struct FlightConfig;
struct CfgKey;
uint8_t telemetry_checksum(const TelemetryPacket &pkt);
void send_telemetry(uint32_t now_ms);
bool init_sensors();
void init_radio_only();
void handle_serial_command();
void handle_lora_command();
bool is_two_way_state();
void load_config();
bool save_config();
void dispatch_command(char* line, ReplyChannel ch);
void gps_setup_uart0();
void gps_poll();

static constexpr bool LED_ACTIVE_HIGH = true;
inline void led_write(bool on) {
  digitalWrite(LED_BUILTIN, (LED_ACTIVE_HIGH ? (on ? HIGH : LOW) : (on ? LOW : HIGH)));
}

// ---- I2C (Feather RP2040 RFM95) ----
static constexpr uint8_t I2C_SDA = 2;
static constexpr uint8_t I2C_SCL = 3;
static constexpr uint8_t BMP_ADDR = 0x77;
static constexpr uint8_t LSM6DSOX_ADDR = 0x6A;

// ---- LoRa RFM95 (Feather inbuilt) ----
// One radio = half-duplex only (cannot TX and RX simultaneously on the same channel).
// "Pseudo full duplex" = time-division: e.g. FC listens in short windows while ground sends
// queued commands in those windows — needs shared schedule (e.g. slot index in telemetry)
// and tolerates clock drift; not implemented here — bridge repeats rare commands instead.
static constexpr uint8_t RFM95_CS   = 16;
static constexpr uint8_t RFM95_INT  = 21;
static constexpr uint8_t RFM95_RST  = 17;
static constexpr float  RF95_FREQ   = 915.0f;

// ---- GPS (UART0: MCU GP0=TX, GP1=RX) ----
static constexpr uint8_t GPS_TX_PIN = 0;
static constexpr uint8_t GPS_RX_PIN = 1;
static constexpr uint32_t GPS_BAUD  = 9600;

// ---- Calibration / timing ----
static constexpr uint32_t CAL_TIME_MS = 10000;
static constexpr uint32_t PRINT_DT_MS = 20;
static constexpr float G0 = 9.80665f;
static constexpr float P0_PA_FALLBACK = 101325.0f;

// Pyro
static constexpr uint8_t MAIN_PIN     = 9;
static constexpr uint8_t DROGUE_PIN   = 10;
static constexpr uint8_t AIRSTART_PIN = 12;

// ---------------- FLIGHT CONFIG (runtime-settable; persisted to flash) ----------------
enum class CfgType : uint8_t { U32 = 0, F32 = 1, BOOL = 2 };

struct FlightConfig {
  float    launch_alt_gain_m;          // 15 m alt gain for launch detect
  float    launch_anet_thresh;         // 3 m/s^2 net accel for launch detect
  uint32_t launch_confirm_ms;          // 200
  uint32_t min_time_after_arm_ms;      // 200
  uint32_t armed_idle_timeout_ms;      // 300000 (5 min on pad -> back to INIT)
  uint32_t apogee_min_ms;              // 8000
  uint32_t apogee_confirm_ms;          // 1000
  float    apogee_vneg_thresh;         // -4 m/s
  uint32_t drogue_min_ms;              // 10000
  uint32_t drogue_max_ms;              // 17000 (apogee failsafe)
  float    main_deploy_alt_m;          // 200
  uint32_t main_min_ms;                // 18000
  uint32_t main_max_ms;                // 40000 (main failsafe)
  bool     airstart_enabled;           // false
  uint32_t airstart_ms_after_launch;   // 3000
  uint32_t ground_baro_stable_ms;      // 30000
  float    ground_baro_thresh_pa;      // 15
  float    ground_alt_near_m;          // 10
  uint32_t ground_alt_hold_ms;         // 5000
  uint32_t pyro_pulse_ms;              // 1500
};

struct CfgKey {
  const char* name;
  CfgType     type;
  uint16_t    offset;
  float       min_val;
  float       max_val;
};

#define CFG_KEY(field, type, lo, hi) { #field, CfgType::type, offsetof(FlightConfig, field), lo, hi }
static const CfgKey CFG_KEYS[] = {
  CFG_KEY(launch_alt_gain_m,          F32,  1.0f,     1000.0f),
  CFG_KEY(launch_anet_thresh,         F32,  0.5f,     50.0f),
  CFG_KEY(launch_confirm_ms,          U32,  50.0f,    5000.0f),
  CFG_KEY(min_time_after_arm_ms,      U32,  0.0f,     10000.0f),
  CFG_KEY(armed_idle_timeout_ms,      U32,  10000.0f, 3600000.0f),
  CFG_KEY(apogee_min_ms,              U32,  0.0f,     120000.0f),
  CFG_KEY(apogee_confirm_ms,          U32,  100.0f,   10000.0f),
  CFG_KEY(apogee_vneg_thresh,         F32,  -50.0f,   0.0f),
  CFG_KEY(drogue_min_ms,              U32,  0.0f,     120000.0f),
  CFG_KEY(drogue_max_ms,              U32,  1000.0f,  300000.0f),
  CFG_KEY(main_deploy_alt_m,          F32,  10.0f,    2000.0f),
  CFG_KEY(main_min_ms,                U32,  0.0f,     300000.0f),
  CFG_KEY(main_max_ms,                U32,  1000.0f,  600000.0f),
  CFG_KEY(airstart_enabled,           BOOL, 0.0f,     1.0f),
  CFG_KEY(airstart_ms_after_launch,   U32,  0.0f,     120000.0f),
  CFG_KEY(ground_baro_stable_ms,      U32,  1000.0f,  300000.0f),
  CFG_KEY(ground_baro_thresh_pa,      F32,  1.0f,     500.0f),
  CFG_KEY(ground_alt_near_m,          F32,  1.0f,     100.0f),
  CFG_KEY(ground_alt_hold_ms,         U32,  1000.0f,  120000.0f),
  CFG_KEY(pyro_pulse_ms,              U32,  100.0f,   10000.0f),
};
static constexpr size_t CFG_KEY_COUNT = sizeof(CFG_KEYS) / sizeof(CFG_KEYS[0]);

static FlightConfig default_config() {
  FlightConfig c{};
  c.launch_alt_gain_m = 15.0f;
  c.launch_anet_thresh = 3.0f;
  c.launch_confirm_ms = 200;
  c.min_time_after_arm_ms = 200;
  c.armed_idle_timeout_ms = 300000;
  c.apogee_min_ms = 8000;
  c.apogee_confirm_ms = 1000;
  c.apogee_vneg_thresh = -4.0f;
  c.drogue_min_ms = 10000;
  c.drogue_max_ms = 17000;
  c.main_deploy_alt_m = 200.0f;
  c.main_min_ms = 18000;
  c.main_max_ms = 40000;
  c.airstart_enabled = false;
  c.airstart_ms_after_launch = 3000;
  c.ground_baro_stable_ms = 30000;
  c.ground_baro_thresh_pa = 15.0f;
  c.ground_alt_near_m = 10.0f;
  c.ground_alt_hold_ms = 5000;
  c.pyro_pulse_ms = 1500;
  return c;
}

FlightConfig cfg = default_config();

static constexpr uint32_t CFG_MAGIC = 0x52434647;  // "RCFG"
static constexpr uint16_t CFG_VERSION = 1;
struct ConfigSlot {
  uint32_t     magic;
  uint16_t     version;
  uint16_t     crc;
  FlightConfig c;
};

static uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

void load_config() {
  cfg = default_config();
  EEPROM.begin(sizeof(ConfigSlot));
  if (EEPROM.length() < (uint16_t)sizeof(ConfigSlot)) { Serial.println("[CFG] No stored config (size) — defaults"); return; }
  ConfigSlot slot;
  EEPROM.get(0, slot);
  if (slot.magic != CFG_MAGIC || slot.version != CFG_VERSION) { Serial.println("[CFG] No stored config — defaults"); return; }
  uint16_t crc = crc16_ccitt(reinterpret_cast<const uint8_t*>(&slot.c), sizeof(FlightConfig));
  if (crc != slot.crc) { Serial.println("[CFG] Stored config CRC bad — defaults"); return; }
  cfg = slot.c;
  Serial.println("[CFG] Loaded config from flash");
}

bool save_config() {
  ConfigSlot slot;
  slot.magic = CFG_MAGIC;
  slot.version = CFG_VERSION;
  slot.c = cfg;
  slot.crc = crc16_ccitt(reinterpret_cast<const uint8_t*>(&slot.c), sizeof(FlightConfig));
  EEPROM.put(0, slot);
  return EEPROM.commit();
}

static const CfgKey* find_cfg_key(const char* name) {
  for (size_t i = 0; i < CFG_KEY_COUNT; i++)
    if (strcmp(CFG_KEYS[i].name, name) == 0) return &CFG_KEYS[i];
  return nullptr;
}

static float cfg_get_value(const CfgKey& k) {
  const uint8_t* base = reinterpret_cast<const uint8_t*>(&cfg);
  if (k.type == CfgType::U32)  return (float)*reinterpret_cast<const uint32_t*>(base + k.offset);
  if (k.type == CfgType::F32)  return *reinterpret_cast<const float*>(base + k.offset);
  return *reinterpret_cast<const bool*>(base + k.offset) ? 1.0f : 0.0f;
}

static void cfg_set_value(const CfgKey& k, float v) {
  uint8_t* base = reinterpret_cast<uint8_t*>(&cfg);
  if (k.type == CfgType::U32)      *reinterpret_cast<uint32_t*>(base + k.offset) = (uint32_t)v;
  else if (k.type == CfgType::F32) *reinterpret_cast<float*>(base + k.offset) = v;
  else                             *reinterpret_cast<bool*>(base + k.offset) = (v != 0.0f);
}

// Fixed estimator/attitude tuning (not settable from ground)
static constexpr float ALT_LPF_ALPHA = 0.05f;
static constexpr float VBARO_LPF_ALPHA = 0.08f;
static constexpr float VEL_DEADBAND_MPS = 0.3f;

// Attitude (complementary filter, no mag)
static constexpr float ATTITUDE_ALPHA = 0.98f;
static constexpr float ACCEL_TRUST_THRESH = 2.0f;

// ---------------- OBJECTS ----------------
Adafruit_BMP3XX bmp;
Adafruit_LSM6DSOX imu;
RH_RF95 rf95(RFM95_CS, RFM95_INT);
TinyGPSPlus gps;

// ---------------- BASELINES ----------------
bool calibrated = false;
uint32_t cal_start_ms = 0;
uint32_t t0_ms = 0;
double p0_pa_sum = 0.0;
uint32_t p0_count = 0;
double ax_sum = 0.0, ay_sum = 0.0, az_sum = 0.0;
uint32_t imu_count = 0;
float P0_pa = P0_PA_FALLBACK;
float down0_x = 0, down0_y = 0, down0_z = 1;
float alt0_m = 0.0f;
uint32_t drogue_fire_start_ms = 0;
uint32_t main_fire_start_ms   = 0;
uint32_t airstart_fire_start_ms = 0;

// ---------------- FLIGHT STATE (same as FC1/FC2) ----------------
enum class FlightState : uint8_t {
  READY = 0, CALIBRATING, INIT, ARMED, BOOST, APOGEE,
  DEPLOY_DROGUE, DROGUE_DESCENT, DEPLOY_MAIN, MAIN_DESCENT, FINISHED
};
FlightState flight_state = FlightState::READY;
bool start_calibration_requested = false;
bool armed = false;
bool launch_detected = false;
bool apogee_detected = false;
uint32_t state_entry_ms = 0;
uint32_t t_launch_ms = 0;
uint32_t launch_trigger_start_ms = 0;
uint32_t vneg_start_ms = 0;
bool drogue_marked = false;
bool main_marked   = false;
float P_at_main_descent_start = 0.0f;
uint32_t ground_stable_start_ms = 0;
uint32_t alt_near_ground_start_ms = 0;
uint32_t lora_cmd_flash_until_ms = 0;  // LED override: solid while a LoRa command was just received

// ---------------- ESTIMATOR ----------------
float alt_f = 0.0f, alt_prev = 0.0f, v_baro = 0.0f, v_est = 0.0f;
uint32_t last_estimate_ms = 0;

// ---------------- ATTITUDE (LSM6DSOX: complementary filter, no mag) ----------------
float roll_deg = 0.0f, pitch_deg = 0.0f, yaw_deg = 0.0f;
uint32_t last_attitude_ms = 0;

// ---------------- TELEMETRY PACKET (28 bytes, same as FC1/FC2) ----------------
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

uint8_t telemetry_checksum(const TelemetryPacket &pkt) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t*>(&pkt);
  uint8_t cs = 0;
  for (size_t i = 0; i < sizeof(TelemetryPacket) - 1; ++i) cs ^= bytes[i];
  return cs;
}

// ---------------- UTILS ----------------
bool normalize3(float &x, float &y, float &z) {
  float n = sqrtf(x*x + y*y + z*z);
  if (n < 1e-6f) return false;
  x /= n; y /= n; z /= n;
  return true;
}

float altitude_from_pressure_relative(float P_pa, float P0_pa) {
  if (P_pa <= 0 || P0_pa <= 0) return 0.0f;
  float ratio = P_pa / P0_pa;
  return 44330.0f * (1.0f - powf(ratio, 0.190294957f));
}

const char* stateToString(FlightState s) {
  switch (s) {
    case FlightState::READY:          return "READY";
    case FlightState::CALIBRATING:   return "CALIBRATING";
    case FlightState::INIT:          return "INIT";
    case FlightState::ARMED:         return "ARMED";
    case FlightState::BOOST:         return "BOOST";
    case FlightState::APOGEE:        return "APOGEE";
    case FlightState::DEPLOY_DROGUE: return "DEPLOY_DROGUE";
    case FlightState::DROGUE_DESCENT: return "DROGUE_DESCENT";
    case FlightState::DEPLOY_MAIN:   return "DEPLOY_MAIN";
    case FlightState::MAIN_DESCENT:  return "MAIN_DESCENT";
    case FlightState::FINISHED:      return "FINISHED";
    default:                         return "UNKNOWN";
  }
}

// ---------------- ESTIMATOR ----------------
void estimator_reset(uint32_t now_ms, float alt_m) {
  alt_f = alt_m; alt_prev = alt_m; v_baro = 0.0f; v_est = 0.0f; last_estimate_ms = now_ms;
}
void estimator_update(uint32_t now_ms, float alt_m) {
  if (last_estimate_ms == 0) { estimator_reset(now_ms, alt_m); return; }
  float dt = (now_ms - last_estimate_ms) / 1000.0f;
  if (dt <= 0.0f || dt > 1.0f) return;
  last_estimate_ms = now_ms;
  float alt_raw = alt_m;
  alt_f = ALT_LPF_ALPHA * alt_raw + (1.0f - ALT_LPF_ALPHA) * alt_f;
  float v_raw = (alt_f - alt_prev) / dt;
  alt_prev = alt_f;
  v_baro = VBARO_LPF_ALPHA * v_raw + (1.0f - VBARO_LPF_ALPHA) * v_baro;
  if (fabsf(v_baro) < VEL_DEADBAND_MPS) v_baro = 0.0f;
  v_est = v_baro;
}

// ---------------- RADIO INIT (called from setup so we can receive START/ARM over LoRa before sensors) ----------------
void init_radio_only() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  if (!rf95.init()) {
    Serial.println("LoRa init FAILED");
    return;
  }
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("LoRa setFrequency FAILED");
    return;
  }
  rf95.setTxPower(13, false);
  rf95.setModeRx();
  Serial.println("LoRa OK (RX for START/ARM pre-boost)");
}

bool is_two_way_state() {
  return flight_state == FlightState::READY ||
         flight_state == FlightState::CALIBRATING ||
         flight_state == FlightState::INIT;
}

// ---------------- COMMAND DISPATCHER (USB Serial + LoRa, two-way states only) ----------------
enum class ReplyChannel : uint8_t { USB = 0, LORA = 1 };

static void reply_text(ReplyChannel ch, const char* msg) {
  Serial.println(msg);
  if (ch == ReplyChannel::LORA) {
    rf95.send(reinterpret_cast<const uint8_t*>(msg), strlen(msg));
    rf95.waitPacketSent();
    if (is_two_way_state()) rf95.setModeRx(); else rf95.setModeIdle();
  }
}

static void reply_fmt(ReplyChannel ch, const char* fmt, ...) {
  char buf[96];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  reply_text(ch, buf);
}

static bool parse_cfg_value(const CfgKey& k, const char* s, float& out) {
  if (k.type == CfgType::BOOL) {
    if (strcmp(s, "1") == 0 || strcmp(s, "true") == 0 || strcmp(s, "on") == 0)  { out = 1.0f; return true; }
    if (strcmp(s, "0") == 0 || strcmp(s, "false") == 0 || strcmp(s, "off") == 0) { out = 0.0f; return true; }
    return false;
  }
  char* end = nullptr;
  float v = (k.type == CfgType::U32) ? (float)strtoul(s, &end, 10) : strtof(s, &end);
  if (end == s || *end != '\0') return false;
  out = v;
  return true;
}

static void dump_config(ReplyChannel ch) {
  char buf[96];
  for (size_t i = 0; i < CFG_KEY_COUNT; i++) {
    const CfgKey& k = CFG_KEYS[i];
    float v = cfg_get_value(k);
    if (k.type == CfgType::F32)      snprintf(buf, sizeof(buf), "CFG %s %.3f", k.name, v);
    else if (k.type == CfgType::U32) snprintf(buf, sizeof(buf), "CFG %s %lu", k.name, (unsigned long)v);
    else                             snprintf(buf, sizeof(buf), "CFG %s %d", k.name, (int)v);
    reply_text(ch, buf);
  }
  reply_text(ch, "CFG END");
}

void dispatch_command(char* line, ReplyChannel ch) {
  char* save = nullptr;
  char* cmd = strtok_r(line, " ", &save);

  if (cmd == nullptr || cmd[0] == '\0') return;

  if (strcmp(cmd, "START") == 0 || strcmp(cmd, "INIT") == 0) {
    if (flight_state == FlightState::READY) {
      start_calibration_requested = true;
      reply_text(ch, "ACK START");
    } else {
      reply_fmt(ch, "NACK START STATE %s", stateToString(flight_state));
    }
  } else if (strcmp(cmd, "ARM") == 0) {
    if (flight_state == FlightState::INIT) {
      armed = true;
      reply_text(ch, "ACK ARM");
    } else {
      reply_fmt(ch, "NACK ARM STATE %s", stateToString(flight_state));
    }
  } else if (strcmp(cmd, "DISARM") == 0) {
    if (flight_state == FlightState::ARMED) {
      armed = false;
      launch_trigger_start_ms = 0;
      flight_state = FlightState::INIT;
      state_entry_ms = millis();
      Serial.println("[STATE] ARMED -> INIT (DISARM)");
      reply_text(ch, "ACK DISARM");
    } else if (flight_state == FlightState::INIT) {
      armed = false;
      reply_text(ch, "ACK DISARM");
    } else if (is_two_way_state()) {
      armed = false;
      reply_text(ch, "ACK DISARM");
    } else {
      reply_text(ch, "NACK DISARM IN FLIGHT");
    }
  } else if (strcmp(cmd, "PING") == 0) {
    reply_text(ch, "PONG");
  } else if (strcmp(cmd, "CFG?") == 0) {
    dump_config(ch);
  } else if (strcmp(cmd, "SAVECFG") == 0) {
    if (save_config()) reply_text(ch, "ACK SAVECFG");
    else               reply_text(ch, "NACK SAVECFG FAILED");
  } else if (strcmp(cmd, "SET") == 0) {
    char* key = strtok_r(nullptr, " ", &save);
    char* val = strtok_r(nullptr, " ", &save);
    if (key == nullptr || val == nullptr || strtok_r(nullptr, " ", &save) != nullptr) {
      reply_text(ch, "NACK SET USAGE (SET <key> <value>)");
      return;
    }
    const CfgKey* k = find_cfg_key(key);
    if (k == nullptr) { reply_fmt(ch, "NACK SET %s UNKNOWN", key); return; }
    if (!is_two_way_state() || flight_state == FlightState::CALIBRATING) {
      reply_fmt(ch, "NACK SET %s LOCKED", key);
      return;
    }
    float v;
    if (!parse_cfg_value(*k, val, v)) { reply_fmt(ch, "NACK SET %s BADVAL", key); return; }
    if (v < k->min_val || v > k->max_val) { reply_fmt(ch, "NACK SET %s RANGE", key); return; }
    cfg_set_value(*k, v);
    if (k->type == CfgType::F32)      reply_fmt(ch, "ACK SET %s %.3f", key, v);
    else if (k->type == CfgType::U32) reply_fmt(ch, "ACK SET %s %lu", key, (unsigned long)v);
    else                              reply_fmt(ch, "ACK SET %s %d", key, (int)v);
  } else {
    reply_fmt(ch, "NACK UNKNOWN %s", cmd);
  }
}

// Line-buffered USB serial commands
void handle_serial_command() {
  static char cmd_buf[64];
  static uint8_t cmd_len = 0;
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      cmd_buf[cmd_len] = '\0';
      if (cmd_len > 0) dispatch_command(cmd_buf, ReplyChannel::USB);
      cmd_len = 0;
    } else {
      if (cmd_len < sizeof(cmd_buf) - 1) { cmd_buf[cmd_len] = c; cmd_len++; }
    }
  }
}

// Process commands received over LoRa (two-way states only: READY/CALIBRATING/INIT)
void handle_lora_command() {
  if (!is_two_way_state()) return;
  if (!rf95.available()) return;
  uint8_t buf[80];
  uint8_t len = sizeof(buf);
  if (!rf95.recv(buf, &len)) return;
  if (len < sizeof(buf)) buf[len] = '\0'; else buf[sizeof(buf) - 1] = '\0';
  lora_cmd_flash_until_ms = millis() + 150;  // visible "heard the ground" blink
  Serial.print("[LORA RX] ");
  Serial.println(reinterpret_cast<const char*>(buf));
  dispatch_command(reinterpret_cast<char*>(buf), ReplyChannel::LORA);
}
// ---------------- SENSOR INIT (called on START from READY; LoRa already inited in setup) ----------------
bool init_sensors() {
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin();
  Wire.setClock(400000);

  Serial.print("BMP390 init... ");
  if (!bmp.begin_I2C(BMP_ADDR, &Wire)) {
    Serial.println("FAILED (check wiring / 0x76)");
    return false;
  }
  Serial.println("OK");
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  Serial.print("LSM6DSOX init... ");
  if (!imu.begin_I2C(LSM6DSOX_ADDR, &Wire)) {
    Serial.println("FAILED (check wiring)");
    return false;
  }
  Serial.println("OK");
  return true;
}

// ---------------- GPS (UART0 GP0/GP1; non-blocking — loss of GPS must not stop flight logic) ----------------
void gps_setup_uart0() {
  Serial1.setTX(GPS_TX_PIN);
  Serial1.setRX(GPS_RX_PIN);
  Serial1.begin(GPS_BAUD);
}

void gps_poll() {
  // Non-blocking: only read bytes already available; if GPS disconnects, we simply get no fix
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }
}

// ---------------- STATE MACHINE (same as FC1/FC2) ----------------
void update_state_machine(uint32_t now_ms, bool baro_ok, float a_net_mag, float P_pa) {
  (void)apogee_detected;
  uint32_t t_since_launch = launch_detected ? (now_ms - t_launch_ms) : 0;
  switch (flight_state) {
    case FlightState::READY:
    case FlightState::CALIBRATING:
      break;
    case FlightState::INIT:
      // Arm only on explicit ARM command (USB or LoRa) — no auto-arm
      if (armed) {
        flight_state = FlightState::ARMED;
        state_entry_ms = now_ms;
        Serial.println("[STATE] INIT -> ARMED (ARM command)");
      }
      break;
    case FlightState::ARMED: {
      uint32_t t_in_state = now_ms - state_entry_ms;
      // Pad idle: no launch for idle timeout -> disarm and INIT (re-arm with ARM over LoRa/USB)
      if (t_in_state >= cfg.armed_idle_timeout_ms) {
        armed = false;
        launch_trigger_start_ms = 0;
        flight_state = FlightState::INIT;
        state_entry_ms = now_ms;
        Serial.println("[STATE] ARMED -> INIT (idle timeout — send ARM to re-arm)");
        break;
      }
      if (t_in_state < cfg.min_time_after_arm_ms) { launch_trigger_start_ms = 0; break; }
      float alt_gain = alt_f;
      bool alt_trigger = (baro_ok && (alt_gain >= cfg.launch_alt_gain_m));
      bool imu_trigger = (a_net_mag >= cfg.launch_anet_thresh);
      bool launch_cond = (alt_trigger || imu_trigger);
      if (launch_cond) {
        if (launch_trigger_start_ms == 0) launch_trigger_start_ms = now_ms;
      } else launch_trigger_start_ms = 0;
      if (launch_trigger_start_ms > 0 && (now_ms - launch_trigger_start_ms) >= cfg.launch_confirm_ms) {
        launch_detected = true;
        t_launch_ms = now_ms;
        flight_state = FlightState::BOOST;
        state_entry_ms = now_ms;
        launch_trigger_start_ms = 0;
        Serial.println("[STATE] ARMED -> BOOST (launch detected)");
      }
      break;
    }
    case FlightState::BOOST: {
      if (!launch_detected) break;
      t_since_launch = now_ms - t_launch_ms;
      if (cfg.airstart_enabled && airstart_fire_start_ms == 0 && t_since_launch >= cfg.airstart_ms_after_launch) {
        airstart_fire_start_ms = now_ms;
        digitalWrite(AIRSTART_PIN, HIGH);
        Serial.print("[EVENT] AIRSTART at t_ms="); Serial.println(t_since_launch);
      }
      if (t_since_launch >= cfg.apogee_min_ms) {
        bool vneg = (v_est <= cfg.apogee_vneg_thresh);
        if (vneg) { if (vneg_start_ms == 0) vneg_start_ms = now_ms; }
        else vneg_start_ms = 0;
        bool apogee_confirmed = (vneg_start_ms > 0 && (now_ms - vneg_start_ms) >= cfg.apogee_confirm_ms);
        bool apogee_failsafe = (t_since_launch >= cfg.drogue_max_ms);
        if (apogee_confirmed || apogee_failsafe) {
          apogee_detected = true;
          flight_state = FlightState::APOGEE;
          state_entry_ms = now_ms;
          vneg_start_ms = 0;
          Serial.println("[STATE] BOOST -> APOGEE");
        }
      }
      break;
    }
    case FlightState::APOGEE:
      flight_state = FlightState::DEPLOY_DROGUE;
      state_entry_ms = now_ms;
      Serial.println("[STATE] APOGEE -> DEPLOY_DROGUE");
      break;
    case FlightState::DEPLOY_DROGUE: {
      t_since_launch = now_ms - t_launch_ms;
      if (!drogue_marked && (t_since_launch >= cfg.drogue_max_ms || t_since_launch >= cfg.drogue_min_ms)) {
        drogue_marked = true;
        drogue_fire_start_ms = now_ms;
        digitalWrite(DROGUE_PIN, HIGH);
        Serial.print("[EVENT] DROGUE at t_ms="); Serial.println(t_since_launch);
      }
      if (drogue_marked) {
        flight_state = FlightState::DROGUE_DESCENT;
        state_entry_ms = now_ms;
        Serial.println("[STATE] DEPLOY_DROGUE -> DROGUE_DESCENT");
      }
      break;
    }
    case FlightState::DROGUE_DESCENT: {
      t_since_launch = now_ms - t_launch_ms;
      bool main_time_ok = (t_since_launch >= cfg.main_min_ms);
      bool main_low_alt = (alt_f <= cfg.main_deploy_alt_m);
      bool main_cond = (main_time_ok && main_low_alt);
      bool main_failsafe = (t_since_launch >= cfg.main_max_ms);
      if (!main_marked && (main_cond || main_failsafe)) {
        main_marked = true;
        main_fire_start_ms = now_ms;
        digitalWrite(MAIN_PIN, HIGH);
        flight_state = FlightState::DEPLOY_MAIN;
        state_entry_ms = now_ms;
        Serial.print("[EVENT] MAIN at t_ms="); Serial.print(t_since_launch); Serial.print(" alt_f="); Serial.println(alt_f);
        Serial.println("[STATE] DROGUE_DESCENT -> DEPLOY_MAIN");
      }
      break;
    }
    case FlightState::DEPLOY_MAIN:
      flight_state = FlightState::MAIN_DESCENT;
      state_entry_ms = now_ms;
      P_at_main_descent_start = (baro_ok && isfinite(P_pa) && P_pa > 1000.0f) ? P_pa : 0.0f;
      ground_stable_start_ms = 0;
      alt_near_ground_start_ms = 0;
      Serial.println("[STATE] DEPLOY_MAIN -> MAIN_DESCENT");
      break;
    case FlightState::MAIN_DESCENT: {
      if (P_at_main_descent_start <= 0.0f && baro_ok && isfinite(P_pa) && P_pa > 1000.0f)
        P_at_main_descent_start = P_pa;
      if (P_at_main_descent_start > 0.0f && baro_ok && isfinite(P_pa)) {
        float dP = fabsf(P_pa - P_at_main_descent_start);
        if (dP <= cfg.ground_baro_thresh_pa) {
          if (ground_stable_start_ms == 0) ground_stable_start_ms = now_ms;
          else if ((now_ms - ground_stable_start_ms) >= cfg.ground_baro_stable_ms) {
            flight_state = FlightState::FINISHED;
            state_entry_ms = now_ms;
            Serial.println("[STATE] MAIN_DESCENT -> FINISHED (baro stable)");
            break;
          }
        } else ground_stable_start_ms = 0;
      }
      if (isfinite(alt_f) && alt_f >= -cfg.ground_alt_near_m && alt_f <= cfg.ground_alt_near_m) {
        if (alt_near_ground_start_ms == 0) alt_near_ground_start_ms = now_ms;
        else if ((now_ms - alt_near_ground_start_ms) >= cfg.ground_alt_hold_ms) {
          flight_state = FlightState::FINISHED;
          state_entry_ms = now_ms;
          Serial.println("[STATE] MAIN_DESCENT -> FINISHED (alt near pad)");
        }
      } else alt_near_ground_start_ms = 0;
      break;
    }
    case FlightState::FINISHED:
    default:
      break;
  }
  if (drogue_fire_start_ms > 0 && (now_ms - drogue_fire_start_ms) >= cfg.pyro_pulse_ms) {
    digitalWrite(DROGUE_PIN, LOW); drogue_fire_start_ms = 0; Serial.println("[PYRO] DROGUE done");
  }
  if (main_fire_start_ms > 0 && (now_ms - main_fire_start_ms) >= cfg.pyro_pulse_ms) {
    digitalWrite(MAIN_PIN, LOW); main_fire_start_ms = 0; Serial.println("[PYRO] MAIN done");
  }
  if (airstart_fire_start_ms > 0 && (now_ms - airstart_fire_start_ms) >= cfg.pyro_pulse_ms) {
    digitalWrite(AIRSTART_PIN, LOW); airstart_fire_start_ms = 0; Serial.println("[PYRO] AIRSTART done");
  }
}

// ---------------- LED ----------------
enum class LedPattern : uint8_t { OFF = 0, SOLID, BLINK_1HZ, BLINK_4HZ, BURST_2, BURST_3, BURST_4, HEARTBEAT };
LedPattern pattern_for_state(bool calibrated, FlightState s, bool armed_flag) {
  (void)calibrated;
  (void)armed_flag;
  if (s == FlightState::READY)       return LedPattern::BLINK_1HZ;
  if (s == FlightState::CALIBRATING) return LedPattern::BLINK_1HZ;
  if (s == FlightState::INIT)        return LedPattern::BURST_2;
  if (s == FlightState::ARMED)       return LedPattern::BLINK_4HZ;
  switch (s) {
    case FlightState::BOOST:          return LedPattern::SOLID;
    case FlightState::APOGEE:        return LedPattern::BURST_3;
    case FlightState::DEPLOY_DROGUE: return LedPattern::BURST_3;
    case FlightState::DROGUE_DESCENT: return LedPattern::BURST_3;
    case FlightState::DEPLOY_MAIN:   return LedPattern::BURST_4;
    case FlightState::MAIN_DESCENT:  return LedPattern::BURST_4;
    case FlightState::FINISHED:      return LedPattern::HEARTBEAT;
    default:                         return LedPattern::OFF;
  }
}
void led_update(uint32_t now_ms, LedPattern pat) {
  // Solid override: brief flash whenever a LoRa command arrives (any state pattern)
  if (now_ms < lora_cmd_flash_until_ms) { led_write(true); return; }
  static constexpr uint16_t BLIP_ON_MS = 80, BLIP_OFF_MS = 120, BURST_GAP_MS = 1200;
  static uint32_t last_ms = 0;
  static bool led_on = false;
  static uint8_t burst_remaining = 0;
  static uint32_t next_toggle_ms = 0;
  static LedPattern last_pat = LedPattern::OFF;
  if (pat != last_pat) { led_on = false; led_write(false); burst_remaining = 0; next_toggle_ms = now_ms; last_pat = pat; }
  switch (pat) {
    case LedPattern::OFF: led_write(false); return;
    case LedPattern::SOLID: led_write(true); return;
    case LedPattern::BLINK_1HZ: if (now_ms - last_ms >= 500) { last_ms = now_ms; led_on = !led_on; led_write(led_on); } return;
    case LedPattern::BLINK_4HZ: if (now_ms - last_ms >= 125) { last_ms = now_ms; led_on = !led_on; led_write(led_on); } return;
    case LedPattern::HEARTBEAT: led_write((now_ms % 1000) < 60); return;
    case LedPattern::BURST_2: case LedPattern::BURST_3: case LedPattern::BURST_4: {
      uint8_t bursts = (pat == LedPattern::BURST_2) ? 2 : (pat == LedPattern::BURST_3) ? 3 : 4;
      if (burst_remaining == 0 && now_ms >= next_toggle_ms) { burst_remaining = bursts * 2; led_on = false; next_toggle_ms = now_ms; }
      if (burst_remaining > 0 && now_ms >= next_toggle_ms) {
        led_on = !led_on; led_write(led_on); burst_remaining--;
        if (burst_remaining == 0) { led_write(false); next_toggle_ms = now_ms + BURST_GAP_MS; led_on = false; }
        else next_toggle_ms = now_ms + (led_on ? BLIP_ON_MS : BLIP_OFF_MS);
      }
      return;
    }
    default: led_write(false); return;
  }
}

// ---------------- SEND TELE (LoRa, 28-byte binary packet; GPS lat/lon when valid, else sentinel) ----------------
void send_telemetry(uint32_t now_ms) {
  TelemetryPacket pkt{};
  pkt.preamble[0] = 'R'; pkt.preamble[1] = 'C';
  pkt.time_ms = now_ms;
  float alt_m = alt_f, vel_mps = v_est;
  if (!isfinite(alt_m)) alt_m = 0.0f;
  if (!isfinite(vel_mps)) vel_mps = 0.0f;
  pkt.alt_cm   = (int32_t)lroundf(alt_m * 100.0f);
  pkt.vel_cmps = (int16_t)lroundf(vel_mps * 100.0f);
  // GPS: send lat/lon when fix valid; if GPS lost or no fix, send sentinel (ground station shows "--")
  if (gps.location.isValid()) {
    pkt.lat_1e7 = (int32_t)llround(gps.location.lat() * 1e7);
    pkt.lon_1e7 = (int32_t)llround(gps.location.lng() * 1e7);
  } else {
    pkt.lat_1e7 = INT32_MIN;
    pkt.lon_1e7 = INT32_MIN;
  }
  pkt.state = static_cast<uint8_t>(flight_state);
  int16_t rc = (int16_t)lroundf(roll_deg * 100.0f);
  int16_t pc = (int16_t)lroundf(pitch_deg * 100.0f);
  int16_t yc = (int16_t)lroundf(yaw_deg * 100.0f);
  if (rc > 18000) rc = 18000; else if (rc < -18000) rc = -18000;
  if (pc > 18000) pc = 18000; else if (pc < -18000) pc = -18000;
  if (yc > 18000) yc = 18000; else if (yc < -18000) yc = -18000;
  pkt.roll_centi = rc;
  pkt.pitch_centi = pc;
  pkt.yaw_centi = yc;
  pkt.checksum = telemetry_checksum(pkt);
  rf95.send(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
  rf95.waitPacketSent();
  // Two-way (TDD) until ARMED: listen for commands between telemetry bursts.
  // From ARMED onward: one-way only (rocket → ground), no RX.
  if (is_two_way_state()) rf95.setModeRx(); else rf95.setModeIdle();
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 1500) delay(10);

  Serial.println("\n=== Rocket FC: BMP390 + LSM6DSOX + LoRa (915 MHz) + GPS (UART0 GP0/GP1) ===");
  Serial.println("State READY. Send START via USB Serial or LoRa to begin init and calibration.");
  Serial.println("ARM only via explicit ARM command (no auto-arm). ARMED idle timeout -> INIT (send ARM to re-arm). From ARMED: one-way LoRa.");
  load_config();

  flight_state = FlightState::READY;
  calibrated = false;
  state_entry_ms = millis();

  init_radio_only();
  gps_setup_uart0();

  pinMode(MAIN_PIN, OUTPUT);
  pinMode(DROGUE_PIN, OUTPUT);
  pinMode(AIRSTART_PIN, OUTPUT);
  digitalWrite(MAIN_PIN, LOW);
  digitalWrite(DROGUE_PIN, LOW);
  digitalWrite(AIRSTART_PIN, LOW);
  pinMode(LED_BUILTIN, OUTPUT);
  led_write(false);
}

// ---------------- LOOP ----------------
void loop() {
  uint32_t now = millis();

  // GPS poll every loop (non-blocking); if GPS is lost, we just get no fix — flight logic unchanged
  gps_poll();

  LedPattern pat = pattern_for_state(calibrated, flight_state, armed);
  led_update(now, pat);

  // Telemetry: 10 Hz in flight, 1 Hz pre-calibration beacon so the ground
  // can see the link and FC state from power-up (fields are just 0 pre-cal)
  static uint32_t last_tel_ms = 0;
  uint32_t tele_period_ms = calibrated ? 100u : 1000u;
  if (now - last_tel_ms >= tele_period_ms) {
    last_tel_ms = now;
    send_telemetry(now);
  }

  handle_serial_command();
  handle_lora_command();

  // READY: wait for START
  if (flight_state == FlightState::READY) {
    if (start_calibration_requested) {
      start_calibration_requested = false;
      if (init_sensors()) {
        flight_state = FlightState::CALIBRATING;
        cal_start_ms = now;
        p0_pa_sum = 0.0; p0_count = 0;
        ax_sum = ay_sum = az_sum = 0.0;
        imu_count = 0;
        Serial.println("Calibrating 10s... keep the rocket still.");
      }
    }
    delay(50);
    return;
  }

  sensors_event_t accel, gyro, temp;
  bool imu_ok = imu.getEvent(&accel, &gyro, &temp);
  bool bmp_ok = bmp.performReading();
  float P_pa = bmp_ok ? bmp.pressure : NAN;

  if (flight_state == FlightState::CALIBRATING) {
    if (bmp_ok && isfinite(P_pa) && P_pa > 1000.0f) { p0_pa_sum += (double)P_pa; p0_count++; }
    if (imu_ok) {
      ax_sum += accel.acceleration.x;
      ay_sum += accel.acceleration.y;
      az_sum += accel.acceleration.z;
      imu_count++;
    }
    static uint32_t last_prog = 0;
    if (now - last_prog > 1000) {
      last_prog = now;
      Serial.print("... "); Serial.print((now - cal_start_ms) / 1000); Serial.println("s");
    }
    if (now - cal_start_ms >= CAL_TIME_MS) {
      P0_pa = (p0_count > 0) ? (float)(p0_pa_sum / (double)p0_count) : P0_PA_FALLBACK;
      float ax0 = (imu_count > 0) ? (float)(ax_sum / (double)imu_count) : 0.0f;
      float ay0 = (imu_count > 0) ? (float)(ay_sum / (double)imu_count) : 0.0f;
      float az0 = (imu_count > 0) ? (float)(az_sum / (double)imu_count) : G0;
      down0_x = ax0; down0_y = ay0; down0_z = az0;
      normalize3(down0_x, down0_y, down0_z);
      calibrated = true;
      t0_ms = now;
      float alt_baseline = 0.0f;
      if (bmp_ok && isfinite(P_pa) && P_pa > 1000.0f)
        alt_baseline = altitude_from_pressure_relative(P_pa, P0_pa);
      alt0_m = alt_baseline;
      estimator_reset(now, 0.0f);
      roll_deg  = atan2f(ay0, az0) * 180.0f / (float)M_PI;
      pitch_deg = atan2f(-ax0, sqrtf(ay0*ay0 + az0*az0)) * 180.0f / (float)M_PI;
      yaw_deg   = 0.0f;
      last_attitude_ms = now;
      flight_state = FlightState::INIT;
      state_entry_ms = now;
      Serial.println("\n=== Baseline locked (t=0) ===");
      Serial.print("P0_pa="); Serial.println(P0_pa, 2);
      Serial.println("(LSM6DSOX: no mag — yaw from gyro integration. GPS on UART0.)");
      Serial.println("============================\n");
      Serial.println("Calibration done. Send ARM to arm (no auto-arm).");
    }
    delay(10);
    return;
  }

  if (!calibrated) { delay(10); return; }

  static uint32_t last_print = 0;
  if (now - last_print < PRINT_DT_MS) return;
  last_print = now;

  uint32_t t_ms = now - t0_ms;
  float alt_m = NAN;
  if (bmp_ok && isfinite(P_pa))
    alt_m = altitude_from_pressure_relative(P_pa, P0_pa) - alt0_m;

  float a_net_mag = 0.0f;
  if (imu_ok) {
    float ax = accel.acceleration.x, ay = accel.acceleration.y, az = accel.acceleration.z;
    float a_mag = sqrtf(ax*ax + ay*ay + az*az);
    a_net_mag = a_mag - G0;
    if (a_net_mag < 0.0f) a_net_mag = 0.0f;
  }

  if (isfinite(alt_m)) estimator_update(now, alt_m);

  // Attitude: complementary filter (gyro + accel, no mag)
  if (imu_ok) {
    float ax = accel.acceleration.x, ay = accel.acceleration.y, az = accel.acceleration.z;
    float a_mag = sqrtf(ax*ax + ay*ay + az*az);
    if (last_attitude_ms > 0) {
      float dt_att = (now - last_attitude_ms) / 1000.0f;
      if (dt_att > 0.0f && dt_att < 1.0f) {
        float gx = gyro.gyro.x, gy = gyro.gyro.y, gz = gyro.gyro.z;
        roll_deg  += (float)(gx * (180.0 / M_PI) * dt_att);
        pitch_deg += (float)(gy * (180.0 / M_PI) * dt_att);
        yaw_deg   += (float)(gz * (180.0 / M_PI) * dt_att);
        if (fabsf(a_mag - G0) < ACCEL_TRUST_THRESH) {
          float roll_acc  = atan2f(ay, az) * 180.0f / (float)M_PI;
          float pitch_acc = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / (float)M_PI;
          roll_deg  = ATTITUDE_ALPHA * roll_deg  + (1.0f - ATTITUDE_ALPHA) * roll_acc;
          pitch_deg = ATTITUDE_ALPHA * pitch_deg + (1.0f - ATTITUDE_ALPHA) * pitch_acc;
        }
      }
    }
    last_attitude_ms = now;
  }

  update_state_machine(now, bmp_ok && isfinite(alt_m), a_net_mag, P_pa);

  Serial.print("t_ms="); Serial.print(t_ms);
  Serial.print(" | state="); Serial.print(stateToString(flight_state));
  Serial.print(" | armed="); Serial.print(armed ? 1 : 0);
  if (bmp_ok && isfinite(alt_m)) {
    Serial.print(" | P_pa="); Serial.print(P_pa, 1);
    Serial.print(" | alt_m="); Serial.print(alt_m, 2);
    Serial.print(" | alt_f="); Serial.print(alt_f, 2);
    Serial.print(" | v_est="); Serial.print(v_est, 2);
  } else Serial.print(" | bmp=ERR");
  if (imu_ok) {
    Serial.print(" | a_excess="); Serial.print(a_net_mag, 2);
    Serial.print(" | r/p/y="); Serial.print(roll_deg, 1); Serial.print(","); Serial.print(pitch_deg, 1); Serial.print(","); Serial.print(yaw_deg, 1);
  } else Serial.print(" | imu=ERR");
  if (gps.location.isValid()) {
    Serial.print(" | lat="); Serial.print(gps.location.lat(), 6);
    Serial.print(" lon="); Serial.print(gps.location.lng(), 6);
  } else {
    Serial.print(" | GPS=no fix");
  }
  Serial.println();
}
