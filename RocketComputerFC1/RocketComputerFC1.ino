// =============================================================================
// ROCKET FLIGHT COMPUTER — FC1
// IMU: Adafruit ICM20948 (accel + gyro + mag)
// Baro: BMP388
// Telemetry: Serial2 (UART) — connect to ground station or USB-serial adapter
// =============================================================================

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_ICM20948.h>
#include <math.h>
#include <limits.h>

// GPS parsing
#include <TinyGPSPlus.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN 25   // RP2040 Pico usually uses GP25 for onboard LED
#endif

enum class FlightState : uint8_t;
enum class LedPattern  : uint8_t;

const char* stateToString(FlightState s);
LedPattern pattern_for_state(bool calibrated, FlightState s, bool armed_flag);
void led_update(uint32_t now_ms, LedPattern pat);

struct TelemetryPacket;

uint8_t telemetry_checksum(const TelemetryPacket &pkt);
void send_telemetry(uint32_t now_ms);

static constexpr bool LED_ACTIVE_HIGH = true; // set false if your board LED is inverted

inline void led_write(bool on) {
  digitalWrite(LED_BUILTIN, (LED_ACTIVE_HIGH ? (on ? HIGH : LOW) : (on ? LOW : HIGH)));
}

// ---- I2C pins (I2C0 on GP20/GP21) ----
static constexpr uint8_t I2C_SDA = 20;
static constexpr uint8_t I2C_SCL = 21;

// ---- Addresses (adjust if needed) ----
static constexpr uint8_t BMP388_ADDR = 0x77;  // sometimes 0x76
static constexpr uint8_t ICM_ADDR    = 0x69;  // sometimes 0x68

// ---- Calibration window ----
static constexpr uint32_t CAL_TIME_MS = 10000;   // 10 seconds idle baseline
static constexpr uint32_t PRINT_DT_MS = 20;      // 50 Hz prints

// ---- GPS (UART0 on GP0/GP1) ----
static constexpr uint8_t GPS_TX_PIN = 0; // Pico TX0 -> GPS RX (optional)
static constexpr uint8_t GPS_RX_PIN = 1; // Pico RX0 <- GPS TX (required)
static constexpr uint32_t GPS_BAUD  = 9600;

// How often to print GPS summary (separate from 50Hz sensor print)
static constexpr uint32_t GPS_PRINT_MS = 200; // 5 Hz GPS print

// ---- Physical constants ----
static constexpr float G0 = 9.80665f;            // m/s^2
static constexpr float P0_PA_FALLBACK = 101325.0f; // only used if baseline fails (shouldn't)


// Pyro Channels (GPIO pins)
static constexpr uint8_t DROGUE_PIN   = 16;
static constexpr uint8_t MAIN_PIN     = 17;
static constexpr uint8_t AIRSTART_PIN = 18;

// Pyro pulse duration for solenoids (ms)
static constexpr uint32_t PYRO_PULSE_MS = 1500;

// ---------------- FLIGHT CONFIG CONSTANTS ----------------

// Launch detection
static constexpr float    LAUNCH_ALT_GAIN_M       = 15.0f;   // baro gain threshold
static constexpr float    LAUNCH_ANET_THRESH      = 3.0f;    // m/s^2 net accel magnitude above 1g
static constexpr uint32_t LAUNCH_CONFIRM_MS       = 200;     // condition must persist this long
static constexpr uint32_t MIN_TIME_AFTER_ARM_MS   = 200;     // ignore triggers immediately after arming

// Apogee detection
static constexpr uint32_t APOGEE_MIN_MS           = 10000;    // earliest apogee possible after launch
static constexpr uint32_t APOGEE_CONFIRM_MS       = 1000;    // must see descent / v<0 for this long
static constexpr float    APOGEE_VNEG_THRESH      = -4.0f;   // m/s threshold for "descending"

// Drogue deploy time window (for future pyro logic)
static constexpr uint32_t DROGUE_MIN_MS           = 12000;    // don't deploy before this
static constexpr uint32_t DROGUE_MAX_MS           = 16500;   // force deploy after this

// Main deploy conditions + window (for future pyro logic)
static constexpr float    MAIN_DEPLOY_ALT_M       = 150.0f;  // AGL (relative) threshold
static constexpr uint32_t MAIN_MIN_MS             = 20000;    // don't deploy before this
static constexpr uint32_t MAIN_MAX_MS             = 90000;  // force deploy after this

// Airstart configuration
static constexpr bool     AIRSTART_ENABLED        = false;  // Enable airstart functionality
static constexpr uint32_t AIRSTART_MS_AFTER_LAUNCH = 3000;  // Time after launch to trigger airstart (ms)

// Ground detection (MAIN_DESCENT -> FINISHED)
static constexpr uint32_t GROUND_BARO_STABLE_MS   = 30000;  // baro unchanged for 30 s
static constexpr float    GROUND_BARO_THRESH_PA   = 15.0f;  // max change (Pa) to consider "stable"
static constexpr float    GROUND_ALT_NEAR_M       = 10.0f;  // alt within ±10 m of pad
static constexpr uint32_t GROUND_ALT_HOLD_MS      = 5000;   // alt near pad for 5 s

// Filtering / estimator
// For baro inside rocket body (static air), we need heavy filtering
static constexpr float ALT_LPF_ALPHA              = 0.05f;   // 0..1 (lower = more smoothing, 0.05 = 95% old, 5% new)
static constexpr float VBARO_LPF_ALPHA            = 0.08f;   // derivative smoothing (heavier filtering for velocity)
static constexpr float VEL_DEADBAND_MPS           = 0.3f;    // velocity deadband - zero out velocities below this (m/s)

// ---------------- OBJECTS ----------------
Adafruit_BMP3XX bmp;
Adafruit_ICM20948 icm;
TinyGPSPlus gps;

// ---------------- BASELINES ----------------
bool calibrated = false;
uint32_t cal_start_ms = 0;
uint32_t t0_ms = 0;

double p0_pa_sum = 0.0;
uint32_t p0_count = 0;

double ax_sum = 0.0, ay_sum = 0.0, az_sum = 0.0;
double mx_sum = 0.0, my_sum = 0.0, mz_sum = 0.0;
uint32_t imu_count = 0;

float P0_pa = P0_PA_FALLBACK;   // baseline pressure
float down0_x = 0, down0_y = 0, down0_z = 1; // baseline "down" unit vector
float mag0_x = 1, mag0_y = 0, mag0_z = 0;    // baseline mag (unit-ish)

// Altitude baseline (relative altitude is alt - alt0)
float alt0_m = 0.0f;

// Track when we started firing pyros
uint32_t drogue_fire_start_ms = 0;
uint32_t main_fire_start_ms   = 0;
uint32_t airstart_fire_start_ms = 0;

// ---------------- FLIGHT STATE MACHINE ----------------
enum class FlightState : uint8_t {
  READY = 0,           // powered on, waiting for START from ground station
  CALIBRATING,         // baro/IMU init done, 10 s baseline
  INIT,                // calibrated, waiting for ARM
  ARMED,
  BOOST,
  APOGEE,
  DEPLOY_DROGUE,
  DROGUE_DESCENT,      // after drogue deployed, before main
  DEPLOY_MAIN,         // ephemeral: main pyro firing
  MAIN_DESCENT,        // under main chute; detect ground -> FINISHED
  FINISHED
};

FlightState flight_state = FlightState::READY;
bool start_calibration_requested = false;  // set when START/INIT received in READY
bool armed = false;               // set by ARM/DISARM commands
bool launch_detected = false;
bool apogee_detected = false;

uint32_t state_entry_ms = 0;
uint32_t t_launch_ms = 0;

// helpers for sustained conditions
uint32_t launch_trigger_start_ms = 0;
uint32_t vneg_start_ms = 0;

// drogue/main deploy flags
bool drogue_marked = false;
bool main_marked   = false;

// Ground detection (MAIN_DESCENT)
float P_at_main_descent_start = 0.0f;
uint32_t ground_stable_start_ms = 0;
uint32_t alt_near_ground_start_ms = 0;

// ---------------- ESTIMATOR ----------------
float alt_f   = 0.0f;
float alt_prev = 0.0f;
float v_baro  = 0.0f;
float v_est   = 0.0f;
uint32_t last_estimate_ms = 0;

// ---------------- ATTITUDE (simple complementary: gyro integrate + accel correct) ----------------
static constexpr float ATTITUDE_ALPHA = 0.98f;   // gyro weight (0.98 = trust gyro, 0.02 accel)
static constexpr float ACCEL_TRUST_THRESH = 2.0f; // only use accel when |a_mag - g| < this (m/s^2)
float roll_deg  = 0.0f;
float pitch_deg = 0.0f;
float yaw_deg   = 0.0f;
uint32_t last_attitude_ms = 0;

// ---------------- TELEMETRY ----------------
// Pack the struct to prevent compiler from adding padding bytes
// Matches Python unpack: 2s I i h i i B h h h B (preamble, time_ms, alt_cm, vel_cmps, lat_1e7, lon_1e7, state, roll_centi, pitch_centi, yaw_centi, checksum)
#pragma pack(push, 1)
struct TelemetryPacket {
  uint8_t  preamble[2];   // 'R', 'C'
  uint32_t time_ms;       // millis() since t0_ms or since boot
  int32_t  alt_cm;        // alt_f in centimeters (m * 100)
  int16_t  vel_cmps;      // v_est in cm/s (m/s * 100)
  int32_t  lat_1e7;       // latitude degrees * 1e7
  int32_t  lon_1e7;       // longitude degrees * 1e7
  uint8_t  state;         // FlightState as uint8_t
  int16_t  roll_centi;   // roll  degrees * 100
  int16_t  pitch_centi;  // pitch degrees * 100
  int16_t  yaw_centi;    // yaw   degrees * 100
  uint8_t  checksum;      // XOR of all previous bytes
};
#pragma pack(pop)

uint8_t telemetry_checksum(const TelemetryPacket &pkt) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t*>(&pkt);
  uint8_t cs = 0;
  // Exclude the checksum byte itself (last field)
  for (size_t i = 0; i < sizeof(TelemetryPacket) - 1; ++i) {
    cs ^= bytes[i];
  }
  return cs;
}

// FC1: Telemetry over Serial2 (UART)
static constexpr uint8_t TEL_TX_PIN = 4;
static constexpr uint8_t TEL_RX_PIN = 5;
static constexpr uint32_t TEL_BAUD  = 57600;

// ---------------- UTILS ----------------

// Utility: normalize a 3D vector, return false if too small
bool normalize3(float &x, float &y, float &z) {
  float n = sqrtf(x*x + y*y + z*z);
  if (n < 1e-6f) return false;
  x /= n; y /= n; z /= n;
  return true;
}

// Compute relative altitude from pressure ratio (hypsometric/barometric formula)
float altitude_from_pressure_relative(float P_pa, float P0_pa) {
  if (P_pa <= 0 || P0_pa <= 0) return 0.0f;
  float ratio = P_pa / P0_pa;
  return 44330.0f * (1.0f - powf(ratio, 0.190294957f)); // 1/5.255 ≈ 0.190295
}

// Tilt-compensated heading using accel-derived "down" and magnetometer
float heading_tilt_comp_deg(float down_x, float down_y, float down_z,
                            float mag_x, float mag_y, float mag_z) {
  if (!normalize3(down_x, down_y, down_z)) return NAN;

  float md = mag_x * down_x + mag_y * down_y + mag_z * down_z;
  float hx = mag_x - md * down_x;
  float hy = mag_y - md * down_y;
  float hz = mag_z - md * down_z;

  float hnorm = sqrtf(hx*hx + hy*hy + hz*hz);
  if (hnorm < 1e-3f) return NAN;

  float hdg = atan2f(hy, hx) * 180.0f / (float)M_PI;
  if (hdg < 0) hdg += 360.0f;
  return hdg;
}

const char* stateToString(FlightState s) {
  switch (s) {
    case FlightState::READY:          return "READY";
    case FlightState::CALIBRATING:     return "CALIBRATING";
    case FlightState::INIT:           return "INIT";
    case FlightState::ARMED:          return "ARMED";
    case FlightState::BOOST:          return "BOOST";
    case FlightState::APOGEE:         return "APOGEE";
    case FlightState::DEPLOY_DROGUE:  return "DEPLOY_DROGUE";
    case FlightState::DROGUE_DESCENT: return "DROGUE_DESCENT";
    case FlightState::DEPLOY_MAIN:    return "DEPLOY_MAIN";
    case FlightState::MAIN_DESCENT:   return "MAIN_DESCENT";
    case FlightState::FINISHED:       return "FINISHED";
    default:                          return "UNKNOWN";
  }
}

// ---------------- GPS HELPERS ----------------

void gps_setup_uart0() {
  Serial1.setTX(GPS_TX_PIN);
  Serial1.setRX(GPS_RX_PIN);
  Serial1.begin(GPS_BAUD);
}

void gps_poll() {
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }
}

void gps_print_summary(uint32_t now_ms) {
  static uint32_t last_gps_print = 0;
  if (now_ms - last_gps_print < GPS_PRINT_MS) return;
  last_gps_print = now_ms;

  Serial.print(" | GPS: ");

  if (gps.location.isValid()) {
    Serial.print("lat="); Serial.print(gps.location.lat(), 6);
    Serial.print(" lon="); Serial.print(gps.location.lng(), 6);
  } else {
    Serial.print("lat=NA lon=NA");
  }

  if (gps.altitude.isValid()) {
    Serial.print(" alt_m="); Serial.print(gps.altitude.meters(), 1);
  } else {
    Serial.print(" alt_m=NA");
  }

  if (gps.speed.isValid()) {
    Serial.print(" spd_mps="); Serial.print(gps.speed.mps(), 2);
  } else {
    Serial.print(" spd_mps=NA");
  }

  if (gps.satellites.isValid()) {
    Serial.print(" sats="); Serial.print(gps.satellites.value());
  } else {
    Serial.print(" sats=NA");
  }

  if (gps.hdop.isValid()) {
    Serial.print(" hdop="); Serial.print(gps.hdop.hdop(), 1);
  } else {
    Serial.print(" hdop=NA");
  }

  Serial.println();
}

// ---------------- ESTIMATOR FUNCTIONS ----------------

void estimator_reset(uint32_t now_ms, float alt_m) {
  alt_f = alt_m;
  alt_prev = alt_m;
  v_baro = 0.0f;
  v_est = 0.0f;
  last_estimate_ms = now_ms;
}

void estimator_update(uint32_t now_ms, float alt_m) {
  if (last_estimate_ms == 0) {
    estimator_reset(now_ms, alt_m);
    return;
  }

  float dt = (now_ms - last_estimate_ms) / 1000.0f;
  if (dt <= 0.0f || dt > 1.0f) return;  // Skip if dt is invalid or too large (sensor glitch)
  last_estimate_ms = now_ms;

  // Low-pass filter altitude (heavy filtering for baro inside rocket body)
  float alt_raw = alt_m;
  alt_f = ALT_LPF_ALPHA * alt_raw + (1.0f - ALT_LPF_ALPHA) * alt_f;

  // Derivative for vertical speed
  float v_raw = (alt_f - alt_prev) / dt;
  alt_prev = alt_f;

  // Heavy low-pass filtering for velocity (derivatives amplify noise)
  v_baro = VBARO_LPF_ALPHA * v_raw + (1.0f - VBARO_LPF_ALPHA) * v_baro;

  // Deadband: zero out small velocities (stationary detection)
  if (fabsf(v_baro) < VEL_DEADBAND_MPS) {
    v_baro = 0.0f;
  }

  // For now v_est is filtered baro velocity.
  // (IMU is used for launch detection; fusion can be added later.)
  v_est = v_baro;
}



// ---------------- SENSOR INIT (called when leaving READY on START) ----------------
bool init_sensors() {
  Serial.print("BMP388 init... ");
  if (!bmp.begin_I2C(BMP388_ADDR, &Wire)) {
    Serial.println("FAILED (try 0x76 or check wiring)");
    return false;
  }
  Serial.println("OK");
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  Serial.print("ICM20948 init... ");
  if (!icm.begin_I2C(ICM_ADDR, &Wire)) {
    Serial.println("FAILED (try 0x68 or check wiring)");
    return false;
  }
  Serial.println("OK");
  return true;
}

// ---------------- COMMAND HANDLING (ARM/DISARM + START) ----------------

// Helper function to process commands from a stream
void process_command_stream(Stream &stream, bool is_serial1, const char* stream_name) {
  static char cmd_buf_serial[16];
  static char cmd_buf_serial2[16];
  static uint8_t cmd_len_serial = 0;
  static uint8_t cmd_len_serial2 = 0;
  
  char* cmd_buf = is_serial1 ? cmd_buf_serial : cmd_buf_serial2;
  uint8_t* cmd_len = is_serial1 ? &cmd_len_serial : &cmd_len_serial2;

  while (stream.available() > 0) {
    char c = stream.read();
    if (c == '\r') continue;
    if (c == '\n') {
      cmd_buf[*cmd_len] = '\0';

      if (strcmp(cmd_buf, "START") == 0 || strcmp(cmd_buf, "INIT") == 0) {
        if (flight_state == FlightState::READY) {
          Serial.print("[CMD] ");
          Serial.print(cmd_buf);
          Serial.println(" received - begin sensor init and calibration.");
          Serial2.print("[CMD] ");
          Serial2.print(cmd_buf);
          Serial2.println(" received");
          start_calibration_requested = true;
        }
      } else if (strcmp(cmd_buf, "ARM") == 0) {
        armed = true;
        Serial.print("[CMD] ARM received via ");
        Serial.println(stream_name);
        Serial2.print("[CMD] ARM received via ");
        Serial2.println(stream_name);
      } else if (strcmp(cmd_buf, "DISARM") == 0) {
        armed = false;
        Serial.print("[CMD] DISARM received via ");
        Serial.println(stream_name);
        Serial2.print("[CMD] DISARM received via ");
        Serial2.println(stream_name);
      } else if (*cmd_len > 0) {
        Serial.print("[CMD] Unknown via ");
        Serial.print(stream_name);
        Serial.print(": ");
        Serial.println(cmd_buf);
      }

      *cmd_len = 0;
    } else {
      if (*cmd_len < 15) {  // sizeof(cmd_buf_serial) - 1 = 15
        cmd_buf[*cmd_len] = c;
        (*cmd_len)++;
      }
    }
  }
}

void handle_arm_command() {
  // Process commands from USB Serial
  process_command_stream(Serial, true, "USB Serial");
  
  // Process commands from Telemetry Serial2
  process_command_stream(Serial2, false, "Telemetry Serial2");
}

// ---------------- STATE MACHINE UPDATE (see definition below) ----------------

enum class LedPattern : uint8_t {
  OFF = 0,
  SOLID,
  BLINK_1HZ,
  BLINK_4HZ,
  BURST_2,   // double blink every ~2s
  BURST_3,   // triple blink every ~2s
  BURST_4,   // 4 blinks every ~2s
  HEARTBEAT  // short blip every 1s
};

LedPattern pattern_for_state(bool calibrated, FlightState s, bool armed_flag) {
  if (s == FlightState::READY)      return LedPattern::BLINK_1HZ;   // waiting for START
  if (s == FlightState::CALIBRATING) return LedPattern::BLINK_1HZ;  // calibrating
  if (s == FlightState::INIT)       return LedPattern::BURST_2;
  if (s == FlightState::ARMED)      return LedPattern::BLINK_4HZ;
  switch (s) {
    case FlightState::BOOST:          return LedPattern::SOLID;
    case FlightState::APOGEE:         return LedPattern::BURST_3;
    case FlightState::DEPLOY_DROGUE:  return LedPattern::BURST_3;
    case FlightState::DROGUE_DESCENT: return LedPattern::BURST_3;
    case FlightState::DEPLOY_MAIN:    return LedPattern::BURST_4;
    case FlightState::MAIN_DESCENT:   return LedPattern::BURST_4;
    case FlightState::FINISHED:       return LedPattern::HEARTBEAT;
    default:                          return LedPattern::OFF;
  }
}

void led_update(uint32_t now_ms, LedPattern pat) {
  static constexpr uint16_t BLIP_ON_MS = 80, BLIP_OFF_MS = 120, BURST_GAP_MS = 1200;
  static uint32_t last_ms = 0;
  static bool led_on = false;
  static uint8_t burst_remaining = 0;
  static uint32_t next_toggle_ms = 0;
  static LedPattern last_pat = LedPattern::OFF;

  if (pat != last_pat) {
    led_on = false;
    led_write(false);
    burst_remaining = 0;
    next_toggle_ms = now_ms;
    last_pat = pat;
  }

  switch (pat) {
    case LedPattern::OFF:
      led_write(false);
      return;
    case LedPattern::SOLID:
      led_write(true);
      return;
    case LedPattern::BLINK_1HZ:
      if (now_ms - last_ms >= 500) { last_ms = now_ms; led_on = !led_on; led_write(led_on); }
      return;
    case LedPattern::BLINK_4HZ:
      if (now_ms - last_ms >= 125) { last_ms = now_ms; led_on = !led_on; led_write(led_on); }
      return;
    case LedPattern::HEARTBEAT:
      led_write((now_ms % 1000) < 60);
      return;
    case LedPattern::BURST_2:
    case LedPattern::BURST_3:
    case LedPattern::BURST_4: {
        uint8_t bursts = (pat == LedPattern::BURST_2) ? 2 : (pat == LedPattern::BURST_3) ? 3 : 4;
        if (burst_remaining == 0 && now_ms >= next_toggle_ms) {
          burst_remaining = bursts * 2;
          led_on = false;
          next_toggle_ms = now_ms;
        }
        if (burst_remaining > 0 && now_ms >= next_toggle_ms) {
          led_on = !led_on;
          led_write(led_on);
          burst_remaining--;
          if (burst_remaining == 0) {
            led_write(false);
            next_toggle_ms = now_ms + BURST_GAP_MS;
            led_on = false;
          } else {
            next_toggle_ms = now_ms + (led_on ? BLIP_ON_MS : BLIP_OFF_MS);
          }
        }
        return;
      }
    default:
      led_write(false);
      return;
  }
}

// ---------------- SEND TELE (FC1: Serial2) ----------------
void send_telemetry(uint32_t now_ms) {
  TelemetryPacket pkt{};
  pkt.preamble[0] = 'R';
  pkt.preamble[1] = 'C';
  pkt.time_ms = now_ms;
  float alt_m = alt_f, vel_mps = v_est;
  if (!isfinite(alt_m)) alt_m = 0.0f;
  if (!isfinite(vel_mps)) vel_mps = 0.0f;
  pkt.alt_cm   = (int32_t)lroundf(alt_m * 100.0f);
  pkt.vel_cmps = (int16_t)lroundf(vel_mps * 100.0f);
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
  Serial2.write(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt));
}

// Forward declaration for update_state_machine (defined in .ino as single translation unit)
void update_state_machine(uint32_t now_ms, bool baro_ok, float a_net_mag, float P_pa) {
  uint32_t t_since_launch = launch_detected ? (now_ms - t_launch_ms) : 0;
  switch (flight_state) {
    case FlightState::READY:
    case FlightState::CALIBRATING:
      break;
    case FlightState::INIT:
      if (armed) {
        flight_state = FlightState::ARMED;
        state_entry_ms = now_ms;
        Serial.println("[STATE] INIT -> ARMED");
      }
      break;
    case FlightState::ARMED: {
      uint32_t t_in_state = now_ms - state_entry_ms;
      if (t_in_state < MIN_TIME_AFTER_ARM_MS) { launch_trigger_start_ms = 0; break; }
      float alt_gain = alt_f;  // alt_f is already relative to pad (0 at calibration)
      bool alt_trigger = (baro_ok && (alt_gain >= LAUNCH_ALT_GAIN_M));
      bool imu_trigger = (a_net_mag >= LAUNCH_ANET_THRESH);
      bool launch_cond = (alt_trigger || imu_trigger);
      if (launch_cond) {
        if (launch_trigger_start_ms == 0) launch_trigger_start_ms = now_ms;
      } else {
        launch_trigger_start_ms = 0;
      }
      if (launch_trigger_start_ms > 0 && (now_ms - launch_trigger_start_ms) >= LAUNCH_CONFIRM_MS) {
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
      if (AIRSTART_ENABLED && airstart_fire_start_ms == 0 && t_since_launch >= AIRSTART_MS_AFTER_LAUNCH) {
        airstart_fire_start_ms = now_ms;
        digitalWrite(AIRSTART_PIN, HIGH);
        Serial.print("[EVENT] AIRSTART triggered at t_ms=");
        Serial.println(t_since_launch);
      }
      if (t_since_launch >= APOGEE_MIN_MS) {
        bool vneg = (v_est <= APOGEE_VNEG_THRESH);
        if (vneg) {
          if (vneg_start_ms == 0) vneg_start_ms = now_ms;
        } else vneg_start_ms = 0;
        bool apogee_confirmed = (vneg_start_ms > 0 && (now_ms - vneg_start_ms) >= APOGEE_CONFIRM_MS);
        bool apogee_failsafe = (t_since_launch >= DROGUE_MAX_MS);
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
      if (!drogue_marked) {
        if (t_since_launch >= DROGUE_MAX_MS || (t_since_launch >= DROGUE_MIN_MS)) {
          drogue_marked = true;
          drogue_fire_start_ms = now_ms;
          digitalWrite(DROGUE_PIN, HIGH);
          Serial.print("[EVENT] DROGUE deploy at t_ms=");
          Serial.println(t_since_launch);
        }
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
      bool main_time_ok = (t_since_launch >= MAIN_MIN_MS);
      bool main_low_alt = (alt_f <= MAIN_DEPLOY_ALT_M);
      bool main_cond = (main_time_ok && main_low_alt);
      bool main_failsafe = (t_since_launch >= MAIN_MAX_MS);
      if (!main_marked && (main_cond || main_failsafe)) {
        main_marked = true;
        main_fire_start_ms = now_ms;
        digitalWrite(MAIN_PIN, HIGH);
        flight_state = FlightState::DEPLOY_MAIN;
        state_entry_ms = now_ms;
        Serial.print("[EVENT] MAIN deploy at t_ms=");
        Serial.print(t_since_launch);
        Serial.print(" alt_f=");
        Serial.println(alt_f);
        Serial.println("[STATE] DROGUE_DESCENT -> DEPLOY_MAIN");
      }
      break;
    }
    case FlightState::DEPLOY_MAIN:
      // Ephemeral: main is firing; go to MAIN_DESCENT immediately
      flight_state = FlightState::MAIN_DESCENT;
      state_entry_ms = now_ms;
      P_at_main_descent_start = (baro_ok && isfinite(P_pa) && P_pa > 1000.0f) ? P_pa : 0.0f;
      ground_stable_start_ms = 0;
      alt_near_ground_start_ms = 0;
      Serial.println("[STATE] DEPLOY_MAIN -> MAIN_DESCENT");
      break;
    case FlightState::MAIN_DESCENT: {
      // Set baseline pressure on first entry if not yet set
      if (P_at_main_descent_start <= 0.0f && baro_ok && isfinite(P_pa) && P_pa > 1000.0f)
        P_at_main_descent_start = P_pa;
      // Ground: baro unchanged 30 s OR altitude near pad for 5 s
      if (P_at_main_descent_start > 0.0f && baro_ok && isfinite(P_pa)) {
        float dP = fabsf(P_pa - P_at_main_descent_start);
        if (dP <= GROUND_BARO_THRESH_PA) {
          if (ground_stable_start_ms == 0) ground_stable_start_ms = now_ms;
          else if ((now_ms - ground_stable_start_ms) >= GROUND_BARO_STABLE_MS) {
            flight_state = FlightState::FINISHED;
            state_entry_ms = now_ms;
            Serial.println("[STATE] MAIN_DESCENT -> FINISHED (baro stable 30s)");
            break;
          }
        } else {
          ground_stable_start_ms = 0;
        }
      }
      if (isfinite(alt_f) && alt_f >= -GROUND_ALT_NEAR_M && alt_f <= GROUND_ALT_NEAR_M) {
        if (alt_near_ground_start_ms == 0) alt_near_ground_start_ms = now_ms;
        else if ((now_ms - alt_near_ground_start_ms) >= GROUND_ALT_HOLD_MS) {
          flight_state = FlightState::FINISHED;
          state_entry_ms = now_ms;
          Serial.println("[STATE] MAIN_DESCENT -> FINISHED (alt near pad 5s)");
        }
      } else {
        alt_near_ground_start_ms = 0;
      }
      break;
    }
    case FlightState::FINISHED:
    default:
      break;
  }
  if (drogue_fire_start_ms > 0 && (now_ms - drogue_fire_start_ms) >= PYRO_PULSE_MS) {
    digitalWrite(DROGUE_PIN, LOW);
    drogue_fire_start_ms = 0;
    Serial.println("[PYRO] DROGUE pulse complete");
  }
  if (main_fire_start_ms > 0 && (now_ms - main_fire_start_ms) >= PYRO_PULSE_MS) {
    digitalWrite(MAIN_PIN, LOW);
    main_fire_start_ms = 0;
    Serial.println("[PYRO] MAIN pulse complete");
  }
  if (airstart_fire_start_ms > 0 && (now_ms - airstart_fire_start_ms) >= PYRO_PULSE_MS) {
    digitalWrite(AIRSTART_PIN, LOW);
    airstart_fire_start_ms = 0;
    Serial.println("[PYRO] AIRSTART pulse complete");
  }
}

// ---------------- SETUP & LOOP ----------------

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 1500) delay(10);

  Serial.println("\n=== FC1: BMP388 + ICM20948 + Serial2 telemetry ===");
  Serial.println("State READY. Send 'START' from ground station to begin init and calibration.");

  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin();
  Wire.setClock(400000);

  Serial.print("GPS init (UART0 GP0/GP1 @ ");
  Serial.print(GPS_BAUD);
  Serial.println(")... ");
  gps_setup_uart0();
  Serial.println("GPS UART started");

  flight_state = FlightState::READY;
  calibrated = false;
  state_entry_ms = millis();

  pinMode(DROGUE_PIN, OUTPUT);
  pinMode(MAIN_PIN, OUTPUT);
  pinMode(AIRSTART_PIN, OUTPUT);
  digitalWrite(DROGUE_PIN, LOW);
  digitalWrite(MAIN_PIN, LOW);
  digitalWrite(AIRSTART_PIN, LOW);
  pinMode(LED_BUILTIN, OUTPUT);
  led_write(false);

  Serial2.setTX(TEL_TX_PIN);
  Serial2.setRX(TEL_RX_PIN);
  Serial2.begin(TEL_BAUD);
  Serial.println("Telemetry UART (Serial2) started.");
}

void loop() {
  uint32_t now = millis();
  LedPattern pat = pattern_for_state(calibrated, flight_state, armed);
  led_update(now, pat);

  // Telemetry at 10 Hz (100 ms). 28 bytes × 10 = 280 B/s @ 57600 baud ≈ 5% of link.
  static uint32_t last_tel_ms = 0;
  if (now - last_tel_ms >= 1000) {
    last_tel_ms = now;
    send_telemetry(now);
  }

  handle_arm_command();
  gps_poll();

  // READY: wait for START from ground station; no baro/IMU until then
  if (flight_state == FlightState::READY) {
    if (start_calibration_requested) {
      start_calibration_requested = false;
      if (init_sensors()) {
        flight_state = FlightState::CALIBRATING;
        cal_start_ms = now;
        p0_pa_sum = 0.0; p0_count = 0;
        ax_sum = ay_sum = az_sum = mx_sum = my_sum = mz_sum = 0.0;
        imu_count = 0;
        Serial.println("Calibrating for 10 seconds... keep the rocket still.");
      }
    }
    delay(50);
    return;
  }

  // CALIBRATING: need baro and IMU
  bool bmp_ok = bmp.performReading();
  float P_pa = bmp_ok ? bmp.pressure : NAN;
  sensors_event_t accel, gyro, mag, temp;
  bool icm_ok = icm.getEvent(&accel, &gyro, &temp, &mag);

  if (flight_state == FlightState::CALIBRATING) {
    if (bmp_ok && isfinite(P_pa) && P_pa > 1000.0f) {
      p0_pa_sum += (double)P_pa;
      p0_count++;
    }
    if (icm_ok) {
      ax_sum += accel.acceleration.x;
      ay_sum += accel.acceleration.y;
      az_sum += accel.acceleration.z;
      mx_sum += mag.magnetic.x;
      my_sum += mag.magnetic.y;
      mz_sum += mag.magnetic.z;
      imu_count++;
    }
    static uint32_t last_prog = 0;
    if (now - last_prog > 1000) {
      last_prog = now;
      Serial.print("... ");
      Serial.print((now - cal_start_ms) / 1000);
      Serial.println("s");
    }
    if (now - cal_start_ms >= CAL_TIME_MS) {
      P0_pa = (p0_count > 0) ? (float)(p0_pa_sum / (double)p0_count) : P0_PA_FALLBACK;
      float ax0 = (imu_count > 0) ? (float)(ax_sum / (double)imu_count) : 0.0f;
      float ay0 = (imu_count > 0) ? (float)(ay_sum / (double)imu_count) : 0.0f;
      float az0 = (imu_count > 0) ? (float)(az_sum / (double)imu_count) : G0;
      down0_x = ax0; down0_y = ay0; down0_z = az0;
      normalize3(down0_x, down0_y, down0_z);
      mag0_x = (imu_count > 0) ? (float)(mx_sum / (double)imu_count) : 1.0f;
      mag0_y = (imu_count > 0) ? (float)(my_sum / (double)imu_count) : 0.0f;
      mag0_z = (imu_count > 0) ? (float)(mz_sum / (double)imu_count) : 0.0f;
      normalize3(mag0_x, mag0_y, mag0_z);
      calibrated = true;
      t0_ms = now;
      float alt_baseline = 0.0f;
      if (bmp_ok && isfinite(P_pa) && P_pa > 1000.0f)
        alt_baseline = altitude_from_pressure_relative(P_pa, P0_pa);
      alt0_m = alt_baseline;
      estimator_reset(now, 0.0f);
      // Initial attitude from accel (gravity vector) at calibration
      roll_deg  = atan2f(ay0, az0) * 180.0f / (float)M_PI;
      pitch_deg = atan2f(-ax0, sqrtf(ay0*ay0 + az0*az0)) * 180.0f / (float)M_PI;
      yaw_deg   = 0.0f;
      last_attitude_ms = now;
      flight_state = FlightState::INIT;
      state_entry_ms = now;
      Serial.println("\n=== Baseline locked (t=0) ===");
      Serial.print("P0_pa="); Serial.println(P0_pa, 2);
      Serial.println("============================\n");
      Serial.println("Calibration done. Waiting for ARM command...");
    }
    delay(10);
    return;
  }

  // From here: INIT and beyond (calibrated)
  if (!calibrated) {
    delay(10);
    return;
  }

  static uint32_t last_print = 0;
  if (now - last_print < PRINT_DT_MS) {
    gps_print_summary(now);
    return;
  }
  last_print = now;

  uint32_t t_ms = now - t0_ms;
  float alt_m = NAN;
  if (bmp_ok && isfinite(P_pa))
    alt_m = altitude_from_pressure_relative(P_pa, P0_pa) - alt0_m;  // relative to pad

  float a_net_mag = 0.0f;
  if (icm_ok) {
    float ax = accel.acceleration.x, ay = accel.acceleration.y, az = accel.acceleration.z;
    float a_mag = sqrtf(ax*ax + ay*ay + az*az);
    a_net_mag = a_mag - G0;
    if (a_net_mag < 0.0f) a_net_mag = 0.0f;
  }

  if (isfinite(alt_m)) estimator_update(now, alt_m);

  // Attitude: complementary filter (gyro integrate + accel correct when near 1g)
  float down_x = NAN, down_y = NAN, down_z = NAN;
  float heading_deg = NAN;
  if (icm_ok) {
    float ax = accel.acceleration.x, ay = accel.acceleration.y, az = accel.acceleration.z;
    float a_mag = sqrtf(ax*ax + ay*ay + az*az);
    down_x = ax; down_y = ay; down_z = az;
    normalize3(down_x, down_y, down_z);
    float mx = mag.magnetic.x, my = mag.magnetic.y, mz = mag.magnetic.z;
    if (fabsf(mx) + fabsf(my) + fabsf(mz) > 1e-3f)
      heading_deg = heading_tilt_comp_deg(down_x, down_y, down_z, mx, my, mz);

    if (last_attitude_ms > 0) {
      float dt_att = (now - last_attitude_ms) / 1000.0f;
      if (dt_att > 0.0f && dt_att < 1.0f) {
        float gx = gyro.gyro.x, gy = gyro.gyro.y, gz = gyro.gyro.z; // rad/s
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
  if (icm_ok) {
    Serial.print(" | a_excess="); Serial.print(a_net_mag, 2);
    Serial.print(" | r/p/y="); Serial.print(roll_deg, 1); Serial.print(","); Serial.print(pitch_deg, 1); Serial.print(","); Serial.print(yaw_deg, 1);
    Serial.print(" | down=");
    Serial.print(down_x, 4); Serial.print(",");
    Serial.print(down_y, 4); Serial.print(",");
    Serial.print(down_z, 4);
    Serial.print(" | head_deg=");
    if (isnan(heading_deg)) Serial.print("NaN");
    else Serial.print(heading_deg, 1);
    Serial.print(" | m_uT=");
    Serial.print(mag.magnetic.x, 1); Serial.print(",");
    Serial.print(mag.magnetic.y, 1); Serial.print(",");
    Serial.print(mag.magnetic.z, 1);
  } else Serial.print(" | icm=ERR");
  Serial.println();
  gps_print_summary(now);
}
