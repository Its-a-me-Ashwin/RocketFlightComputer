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
static constexpr uint8_t DROGUE_PIN   = 18;
static constexpr uint8_t MAIN_PIN     = 17;
static constexpr uint8_t AIRSTART_PIN = 16;

// Pyro pulse duration for solenoids (ms)
static constexpr uint32_t PYRO_PULSE_MS = 1500;

// ---------------- FLIGHT CONFIG CONSTANTS ----------------

// Launch detection
static constexpr float    LAUNCH_ALT_GAIN_M       = 15.0f;   // baro gain threshold
static constexpr float    LAUNCH_ANET_THRESH      = 3.0f;    // m/s^2 net accel magnitude above 1g
static constexpr uint32_t LAUNCH_CONFIRM_MS       = 200;     // condition must persist this long
static constexpr uint32_t MIN_TIME_AFTER_ARM_MS   = 200;     // ignore triggers immediately after arming

// Apogee detection
static constexpr uint32_t APOGEE_MIN_MS           = 8000;    // earliest apogee possible after launch
static constexpr uint32_t APOGEE_CONFIRM_MS       = 1000;    // must see descent / v<0 for this long
static constexpr float    APOGEE_VNEG_THRESH      = -4.0f;   // m/s threshold for "descending"

// Drogue deploy time window (for future pyro logic)
static constexpr uint32_t DROGUE_MIN_MS           = 9000;    // don't deploy before this
static constexpr uint32_t DROGUE_MAX_MS           = 13500;   // force deploy after this

// Main deploy conditions + window (for future pyro logic)
static constexpr float    MAIN_DEPLOY_ALT_M       = 150.0f;  // AGL (relative) threshold
static constexpr uint32_t MAIN_MIN_MS             = 13500;    // don't deploy before this
static constexpr uint32_t MAIN_MAX_MS             = 90000;  // force deploy after this

// Airstart configuration
static constexpr bool     AIRSTART_ENABLED        = true;  // Enable airstart functionality
static constexpr uint32_t AIRSTART_MS_AFTER_LAUNCH = 3000;  // Time after launch to trigger airstart (ms)

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
  INIT = 0,
  ARMED,
  BOOST,
  APOGEE,
  DEPLOY_DROGUE,
  DEPLOY_MAIN,
  FINISHED
};

FlightState flight_state = FlightState::INIT;
bool armed = false;               // set by ARM/DISARM commands
bool launch_detected = false;
bool apogee_detected = false;

uint32_t state_entry_ms = 0;
uint32_t t_launch_ms = 0;

// helpers for sustained conditions
uint32_t launch_trigger_start_ms = 0;
uint32_t vneg_start_ms = 0;

// drogue/main "virtual" deploy flags (no hardware yet)
bool drogue_marked = false;
bool main_marked   = false;

// ---------------- ESTIMATOR ----------------
float alt_f   = 0.0f;
float alt_prev = 0.0f;
float v_baro  = 0.0f;
float v_est   = 0.0f;
uint32_t last_estimate_ms = 0;


// ---------------- TELEMETRY ----------------
// Pack the struct to prevent compiler from adding padding bytes
// This ensures the struct size matches Python's unpack format exactly
#pragma pack(push, 1)
struct TelemetryPacket {
  uint8_t  preamble[2];   // 'R', 'C'
  uint32_t time_ms;       // millis() since t0_ms or since boot
  int32_t  alt_cm;        // alt_f in centimeters (m * 100)
  int16_t  vel_cmps;      // v_est in cm/s (m/s * 100)
  int32_t  lat_1e7;       // latitude degrees * 1e7
  int32_t  lon_1e7;       // longitude degrees * 1e7
  uint8_t  state;         // FlightState as uint8_t
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
    case FlightState::INIT:          return "INIT";
    case FlightState::ARMED:         return "ARMED";
    case FlightState::BOOST:         return "BOOST";
    case FlightState::APOGEE:        return "APOGEE";
    case FlightState::DEPLOY_DROGUE: return "DEPLOY_DROGUE";
    case FlightState::DEPLOY_MAIN:   return "DEPLOY_MAIN";
    case FlightState::FINISHED:      return "FINISHED";
    default:                         return "UNKNOWN";
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



// ---------------- COMMAND HANDLING (ARM/DISARM) ----------------

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

      if (strcmp(cmd_buf, "ARM") == 0) {
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

// ---------------- STATE MACHINE UPDATE ----------------

void update_state_machine(uint32_t now_ms, bool baro_ok, float a_net_mag) {
  // Time since launch (if launch has been detected)
  uint32_t t_since_launch = launch_detected ? (now_ms - t_launch_ms) : 0;

  switch (flight_state) {
    case FlightState::INIT: {
      // After calibration, INIT waits for ARM command
      if (armed) {
        flight_state   = FlightState::ARMED;
        state_entry_ms = now_ms;
        Serial.println("[STATE] INIT -> ARMED");
      }
      break;
    }

    case FlightState::ARMED: {
      // Don't react immediately after arming
      uint32_t t_in_state = now_ms - state_entry_ms;
      if (t_in_state < MIN_TIME_AFTER_ARM_MS) {
        launch_trigger_start_ms = 0;
        break;
      }

      // Launch detection using altitude gain & IMU magnitude
      float alt_gain   = alt_f - alt0_m;
      bool alt_trigger = (baro_ok && (alt_gain >= LAUNCH_ALT_GAIN_M));
      bool imu_trigger = (a_net_mag >= LAUNCH_ANET_THRESH);

      // If baro is dead, IMU can still trigger launch; if both are dead, we do nothing
      bool launch_cond = (alt_trigger || imu_trigger);

      if (launch_cond) {
        if (launch_trigger_start_ms == 0) {
          launch_trigger_start_ms = now_ms;
        }
      } else {
        launch_trigger_start_ms = 0;
      }

      if (launch_trigger_start_ms > 0 &&
          (now_ms - launch_trigger_start_ms) >= LAUNCH_CONFIRM_MS) {
        launch_detected   = true;
        t_launch_ms       = now_ms;
        flight_state      = FlightState::BOOST;
        state_entry_ms    = now_ms;
        launch_trigger_start_ms = 0;
        Serial.println("[STATE] ARMED -> BOOST (launch detected)");
      }

      break;
    }

    case FlightState::BOOST: {
      if (!launch_detected) break;

      t_since_launch = now_ms - t_launch_ms;

      // Airstart logic (if enabled)
      if (AIRSTART_ENABLED && airstart_fire_start_ms == 0) {
        if (t_since_launch >= AIRSTART_MS_AFTER_LAUNCH) {
          airstart_fire_start_ms = now_ms;
          digitalWrite(AIRSTART_PIN, HIGH);
          Serial.print("[EVENT] AIRSTART triggered at t_ms=");
          Serial.println(t_since_launch);
        }
      }

      // Apogee detection only after minimum time
      if (t_since_launch >= APOGEE_MIN_MS) {
        bool vneg = (v_est <= APOGEE_VNEG_THRESH);

        if (vneg) {
          if (vneg_start_ms == 0) {
            vneg_start_ms = now_ms;
          }
        } else {
          vneg_start_ms = 0;
        }

        bool apogee_confirmed =
          (vneg_start_ms > 0 &&
           (now_ms - vneg_start_ms) >= APOGEE_CONFIRM_MS);

        // Failsafe: if drogue max time exceeded and no apogee detected,
        // treat as apogee anyway.
        bool apogee_failsafe = (t_since_launch >= DROGUE_MAX_MS);

        if (apogee_confirmed || apogee_failsafe) {
          apogee_detected = true;
          flight_state    = FlightState::APOGEE;
          state_entry_ms  = now_ms;
          vneg_start_ms   = 0;
          Serial.println("[STATE] BOOST -> APOGEE");
        }
      }

      break;
    }

    case FlightState::APOGEE: {
      // Immediately proceed to drogue-deploy state
      flight_state   = FlightState::DEPLOY_DROGUE;
      state_entry_ms = now_ms;
      Serial.println("[STATE] APOGEE -> DEPLOY_DROGUE");
      break;
    }

    case FlightState::DEPLOY_DROGUE: {
      t_since_launch = now_ms - t_launch_ms;

      if (!drogue_marked) {
        if (t_since_launch >= DROGUE_MAX_MS) {
          // Forced drogue deploy (failsafe)
          drogue_marked        = true;
          drogue_fire_start_ms = now_ms;
          digitalWrite(DROGUE_PIN, HIGH);
          Serial.print("[EVENT] DROGUE deploy FORCED at t_ms=");
          Serial.println(t_since_launch);
        } else if (t_since_launch >= DROGUE_MIN_MS) {
          // Normal drogue deploy window
          drogue_marked        = true;
          drogue_fire_start_ms = now_ms;
          digitalWrite(DROGUE_PIN, HIGH);
          Serial.print("[EVENT] DROGUE deploy marked at t_ms=");
          Serial.println(t_since_launch);
        }
      }

      // Transition to main-deploy state once drogue is marked (and fired)
      if (drogue_marked) {
        flight_state   = FlightState::DEPLOY_MAIN;
        state_entry_ms = now_ms;
        Serial.println("[STATE] DEPLOY_DROGUE -> DEPLOY_MAIN");
      }

      break;
    }

    case FlightState::DEPLOY_MAIN: {
      t_since_launch = now_ms - t_launch_ms;

      // Main deploy logic based on altitude & time
      bool main_time_ok  = (t_since_launch >= MAIN_MIN_MS);
      bool main_low_alt  = (alt_f <= MAIN_DEPLOY_ALT_M);

      bool main_cond     = (main_time_ok && main_low_alt);
      bool main_failsafe = (t_since_launch >= MAIN_MAX_MS);

      if (!main_marked && (main_cond || main_failsafe)) {
        main_marked        = true;
        main_fire_start_ms = now_ms;
        digitalWrite(MAIN_PIN, HIGH);

        Serial.print("[EVENT] MAIN deploy marked at t_ms=");
        Serial.print(t_since_launch);
        Serial.print(" alt_f=");
        Serial.println(alt_f);
      }

      if (main_marked) {
        flight_state   = FlightState::FINISHED;
        state_entry_ms = now_ms;
        Serial.println("[STATE] DEPLOY_MAIN -> FINISHED");
      }

      break;
    }

    case FlightState::FINISHED: {
      // Do nothing special; continue logging for recovery analysis
      break;
    }

    default:
      break;
  }

  // ---------------- PYRO PULSE TIMEOUTS ----------------
  // Turn off solenoids after the configured pulse duration
  if (drogue_fire_start_ms > 0 &&
      (now_ms - drogue_fire_start_ms) >= PYRO_PULSE_MS) {
    digitalWrite(DROGUE_PIN, LOW);
    drogue_fire_start_ms = 0;
    Serial.println("[PYRO] DROGUE pulse complete, output LOW");
  }

  if (main_fire_start_ms > 0 &&
      (now_ms - main_fire_start_ms) >= PYRO_PULSE_MS) {
    digitalWrite(MAIN_PIN, LOW);
    main_fire_start_ms = 0;
    Serial.println("[PYRO] MAIN pulse complete, output LOW");
  }

  if (airstart_fire_start_ms > 0 &&
      (now_ms - airstart_fire_start_ms) >= PYRO_PULSE_MS) {
    digitalWrite(AIRSTART_PIN, LOW);
    airstart_fire_start_ms = 0;
    Serial.println("[PYRO] AIRSTART pulse complete, output LOW");
  }
}


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

LedPattern current_led_pattern = LedPattern::OFF;

LedPattern pattern_for_state(bool calibrated, FlightState s, bool armed_flag) {
  if (!calibrated) return LedPattern::BLINK_1HZ; // calibrating

  // calibrated:
  if (s == FlightState::INIT)  return LedPattern::BURST_2;   // waiting for ARM
  if (s == FlightState::ARMED) return LedPattern::BLINK_4HZ; // armed

  switch (s) {
    case FlightState::BOOST:         return LedPattern::SOLID;
    case FlightState::APOGEE:        return LedPattern::BURST_3;
    case FlightState::DEPLOY_DROGUE: return LedPattern::BURST_3;
    case FlightState::DEPLOY_MAIN:   return LedPattern::BURST_4;
    case FlightState::FINISHED:      return LedPattern::HEARTBEAT;
    default:                         return LedPattern::OFF;
  }
}

// Non-blocking LED update function
void led_update(uint32_t now_ms, LedPattern pat) {
  // Timing constants (ms)
  static constexpr uint16_t BLIP_ON_MS      = 80;
  static constexpr uint16_t BLIP_OFF_MS     = 120;
  static constexpr uint16_t BURST_GAP_MS    = 1200; // gap after burst

  static uint32_t last_ms = 0;
  static bool led_on = false;

  // burst bookkeeping
  static uint8_t burst_remaining = 0;
  static uint32_t next_toggle_ms = 0;
  static LedPattern last_pat = LedPattern::OFF;

  if (pat != last_pat) {
    // Reset pattern state when switching patterns
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

    case LedPattern::BLINK_1HZ: {
      // toggle every 500 ms
      if (now_ms - last_ms >= 500) {
        last_ms = now_ms;
        led_on = !led_on;
        led_write(led_on);
      }
      return;
    }

    case LedPattern::BLINK_4HZ: {
      // toggle every 125 ms -> 4 Hz blink-ish
      if (now_ms - last_ms >= 125) {
        last_ms = now_ms;
        led_on = !led_on;
        led_write(led_on);
      }
      return;
    }

    case LedPattern::HEARTBEAT: {
      // quick blip ON for 60ms every 1000ms
      uint32_t phase = now_ms % 1000;
      led_write(phase < 60);
      return;
    }

    case LedPattern::BURST_2:
    case LedPattern::BURST_3:
    case LedPattern::BURST_4: {
      uint8_t bursts = (pat == LedPattern::BURST_2) ? 2 :
                       (pat == LedPattern::BURST_3) ? 3 : 4;

      // If we're idle, start a new burst cycle
      if (burst_remaining == 0 && now_ms >= next_toggle_ms) {
        burst_remaining = bursts * 2; // on/off counts
        led_on = false;
        next_toggle_ms = now_ms;
      }

      if (burst_remaining > 0 && now_ms >= next_toggle_ms) {
        led_on = !led_on;
        led_write(led_on);

        burst_remaining--;

        if (burst_remaining == 0) {
          // end of burst: ensure LED off then wait gap
          led_write(false);
          next_toggle_ms = now_ms + BURST_GAP_MS;
          led_on = false;
        } else {
          // within burst: ON and OFF spacing
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


// ---------------- SEND TELE ----------------
void send_telemetry(uint32_t now_ms) {
  TelemetryPacket pkt{};

  // Header
  pkt.preamble[0] = 'R';
  pkt.preamble[1] = 'C';

  pkt.time_ms = now_ms;  // or (now_ms - t0_ms) if you prefer flight-relative

  // Altitude and velocity
  // Clamp to reasonable ranges to avoid overflow
  float alt_m = alt_f;
  float vel_mps = v_est;

  if (!isfinite(alt_m)) alt_m = 0.0f;
  if (!isfinite(vel_mps)) vel_mps = 0.0f;

  // Scale to int
  pkt.alt_cm   = (int32_t)lroundf(alt_m * 100.0f);
  pkt.vel_cmps = (int16_t)lroundf(vel_mps * 100.0f);

  // GPS
  if (gps.location.isValid()) {
    double lat = gps.location.lat();
    double lon = gps.location.lng();
    pkt.lat_1e7 = (int32_t)llround(lat * 1e7);
    pkt.lon_1e7 = (int32_t)llround(lon * 1e7);
  } else {
    // Use a sentinel for "invalid"
    pkt.lat_1e7 = INT32_MIN;
    pkt.lon_1e7 = INT32_MIN;
  }

  // Flight state
  pkt.state = static_cast<uint8_t>(flight_state);

  // Checksum
  pkt.checksum = telemetry_checksum(pkt);

  // Debug: Print packet size (only once)
  static bool size_printed = false;
  if (!size_printed) {
    Serial.print("[TELE] Packet size: ");
    Serial.print(sizeof(TelemetryPacket));
    Serial.println(" bytes");
    size_printed = true;
  }

  // Send it
  Serial2.write(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt));
}


// ---------------- SETUP & LOOP ----------------

void setup() {
  Serial.begin(115200);

  // Don't hang forever if no Serial Monitor (important for VSYS/headless)
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 1500) {
    delay(10);
  }

  Serial.println("\nRocket baseline: BMP388 altitude + accel down + mag heading + GPS + flight state machine");
  Serial.println("Type 'ARM' + Enter on USB Serial after calibration to arm.");

  // I2C
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin();
  Wire.setClock(400000);

  // BMP388 init
  Serial.print("BMP388 init... ");
  if (!bmp.begin_I2C(BMP388_ADDR, &Wire)) {
    Serial.println("FAILED (try 0x76 or check wiring)");
    while (1) delay(100);
  }
  Serial.println("OK");

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // ICM20948 init
  Serial.print("ICM20948 init... ");
  if (!icm.begin_I2C(ICM_ADDR, &Wire)) {
    Serial.println("FAILED (try 0x68 or check wiring)");
    while (1) delay(100);
  }
  Serial.println("OK");

  // GPS init (UART0 on GP0/GP1)
  Serial.print("GPS init (UART0 GP0/GP1 @ ");
  Serial.print(GPS_BAUD);
  Serial.println(")... ");
  gps_setup_uart0();
  Serial.println("GPS UART started (waiting for NMEA)");

  // Start calibration window
  cal_start_ms = millis();
  calibrated = false;

  flight_state = FlightState::INIT;
  state_entry_ms = cal_start_ms;

  Serial.println("Calibrating for 10 seconds... keep the rocket still.");

  pinMode(DROGUE_PIN, OUTPUT);
  pinMode(MAIN_PIN, OUTPUT);
  pinMode(AIRSTART_PIN, OUTPUT);

  digitalWrite(DROGUE_PIN, LOW);
  digitalWrite(MAIN_PIN, LOW);
  digitalWrite(AIRSTART_PIN, LOW);

  pinMode(LED_BUILTIN, OUTPUT);
  led_write(false);

  // TELE SETUP
  Serial2.setTX(TEL_TX_PIN);
  Serial2.setRX(TEL_RX_PIN);
  Serial2.begin(TEL_BAUD);
  Serial.println("Telemetry UART (Serial2) started.");
}

void loop() {
  uint32_t now = millis();

  // Led indicator
  LedPattern pat = pattern_for_state(calibrated, flight_state, armed);
  led_update(now, pat);

  // SEND TELE  
  static uint32_t last_tel_ms = 0;
  if (now - last_tel_ms >= 1000) {   // 1 second
    last_tel_ms = now;
    send_telemetry(now);
  }

  // Always handle commands & poll GPS
  handle_arm_command();
  gps_poll();

  // Read BMP
  bool bmp_ok = bmp.performReading();
  float P_pa = bmp_ok ? bmp.pressure : NAN; // Pa

  // Read IMU events
  sensors_event_t accel, gyro, mag, temp;
  bool icm_ok = icm.getEvent(&accel, &gyro, &temp, &mag);

  // --- Calibration phase: average for CAL_TIME_MS ---
  if (!calibrated) {
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
      // Lock pressure baseline
      P0_pa = (p0_count > 0) ? (float)(p0_pa_sum / (double)p0_count) : P0_PA_FALLBACK;

      // Baseline accel -> "down"
      float ax0 = (imu_count > 0) ? (float)(ax_sum / (double)imu_count) : 0.0f;
      float ay0 = (imu_count > 0) ? (float)(ay_sum / (double)imu_count) : 0.0f;
      float az0 = (imu_count > 0) ? (float)(az_sum / (double)imu_count) : G0;

      down0_x = ax0; down0_y = ay0; down0_z = az0;
      normalize3(down0_x, down0_y, down0_z);

      // Baseline mag
      mag0_x = (imu_count > 0) ? (float)(mx_sum / (double)imu_count) : 1.0f;
      mag0_y = (imu_count > 0) ? (float)(my_sum / (double)imu_count) : 0.0f;
      mag0_z = (imu_count > 0) ? (float)(mz_sum / (double)imu_count) : 0.0f;
      normalize3(mag0_x, mag0_y, mag0_z);

      calibrated = true;
      t0_ms = now;

      // Calculate the actual altitude at calibration baseline
      // This accounts for any small pressure differences between calibration average and current reading
      float alt_baseline = 0.0f;
      if (bmp_ok && isfinite(P_pa) && P_pa > 1000.0f) {
        alt_baseline = altitude_from_pressure_relative(P_pa, P0_pa);
      }
      
      // Set baseline altitude to the calculated value (should be ~0, but accounts for sensor drift)
      alt0_m = alt_baseline;
      estimator_reset(now, alt_baseline);

      Serial.println("\n=== Baseline locked (t=0) ===");
      Serial.print("P0_pa="); Serial.println(P0_pa, 2);
      Serial.print("down0="); Serial.print(down0_x, 4); Serial.print(",");
      Serial.print(down0_y, 4); Serial.print(",");
      Serial.println(down0_z, 4);
      Serial.print("mag0="); Serial.print(mag0_x, 4); Serial.print(",");
      Serial.print(mag0_y, 4); Serial.print(",");
      Serial.println(mag0_z, 4);
      Serial.println("============================\n");
      Serial.println("Calibration done. Waiting for ARM command...");
    }

    delay(10);
    return;
  }

  // --- After calibration: compute relative altitude + heading + state machine ---

  static uint32_t last_print = 0;
  if (now - last_print < PRINT_DT_MS) {
    // Still occasionally print GPS while we wait
    gps_print_summary(now);
    return;
  }
  last_print = now;

  uint32_t t_ms = now - t0_ms;

  // Barometric altitude
  float alt_m = NAN;
  if (bmp_ok && isfinite(P_pa)) {
    alt_m = altitude_from_pressure_relative(P_pa, P0_pa); // relative to baseline
  }

  // Compute a_net magnitude (IMU) for launch detection
  float a_net_mag = 0.0f;
  if (icm_ok) {
    float ax = accel.acceleration.x;
    float ay = accel.acceleration.y;
    float az = accel.acceleration.z;
    float a_mag = sqrtf(ax*ax + ay*ay + az*az);
    a_net_mag = a_mag - G0;  // approx "excess" acceleration
    if (a_net_mag < 0.0f) a_net_mag = 0.0f; // keep non-negative for magnitude-based threshold
  }

  // Update estimator if we have a valid altitude
  if (isfinite(alt_m)) {
    estimator_update(now, alt_m);
  }

  // Compute orientation & heading (as before)
  float down_x = NAN, down_y = NAN, down_z = NAN;
  float heading_deg = NAN;

  if (icm_ok) {
    down_x = accel.acceleration.x;
    down_y = accel.acceleration.y;
    down_z = accel.acceleration.z;
    normalize3(down_x, down_y, down_z);

    float mx = mag.magnetic.x;
    float my = mag.magnetic.y;
    float mz = mag.magnetic.z;

    if (fabsf(mx) + fabsf(my) + fabsf(mz) > 1e-3f) {
      heading_deg = heading_tilt_comp_deg(down_x, down_y, down_z, mx, my, mz);
    }
  }

  // Update state machine
  update_state_machine(now, bmp_ok && isfinite(alt_m), a_net_mag);

  // Print main sensor/state line (50 Hz)
  Serial.print("t_ms="); Serial.print(t_ms);
  Serial.print(" | state="); Serial.print(stateToString(flight_state));
  Serial.print(" | armed="); Serial.print(armed ? 1 : 0);

  if (bmp_ok && isfinite(alt_m)) {
    Serial.print(" | P_pa="); Serial.print(P_pa, 1);
    Serial.print(" | alt_m="); Serial.print(alt_m, 2);
    Serial.print(" | alt_f="); Serial.print(alt_f, 2);
    Serial.print(" | v_est="); Serial.print(v_est, 2);
  } else {
    Serial.print(" | bmp=ERR");
  }

  if (icm_ok) {
    Serial.print(" | a_excess="); Serial.print(a_net_mag, 2);
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
  } else {
    Serial.print(" | icm=ERR");
  }

  Serial.println();

  // Print GPS summary at ~5Hz
  gps_print_summary(now);
}
