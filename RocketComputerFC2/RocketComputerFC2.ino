// =============================================================================
// ROCKET FLIGHT COMPUTER — FC2
// IMU: MPU6050 (accel + gyro only, no magnetometer — yaw from gyro integration)
// Baro: BMP388
// Telemetry: NRF24L01 over SPI (same 28-byte binary packet as FC1)
// ARM/DISARM: USB Serial only (telemetry link is one-way radio)
// Pins: NRF24 SPI0 GP4=RX(MISO), GP6=SCK, GP7=TX(MOSI), GP14=CSN, GP17=CE
//       Pyro: GP10=MAIN, GP11=DROGUE, GP12=AIRSTART
// =============================================================================

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_MPU6050.h>
#include <SPI.h>
#include <RF24.h>
#include <math.h>
#include <limits.h>

#include <TinyGPSPlus.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN 25
#endif

enum class FlightState : uint8_t;
enum class LedPattern  : uint8_t;

const char* stateToString(FlightState s);
LedPattern pattern_for_state(bool calibrated, FlightState s, bool armed_flag);
void led_update(uint32_t now_ms, LedPattern pat);

struct TelemetryPacket;
uint8_t telemetry_checksum(const TelemetryPacket &pkt);
void send_telemetry(uint32_t now_ms);
bool init_sensors();

static constexpr bool LED_ACTIVE_HIGH = true;
inline void led_write(bool on) {
  digitalWrite(LED_BUILTIN, (LED_ACTIVE_HIGH ? (on ? HIGH : LOW) : (on ? LOW : HIGH)));
}

// ---- I2C ----
static constexpr uint8_t I2C_SDA = 20;
static constexpr uint8_t I2C_SCL = 21;
static constexpr uint8_t BMP388_ADDR = 0x77;
static constexpr uint8_t MPU6050_ADDR = 0x69;

// ---- NRF24L01 (SPI0: GP4=MISO, GP6=SCK, GP7=MOSI; GP14=CSN, GP17=CE) ----
static constexpr uint8_t NRF_CE_PIN  = 17;
static constexpr uint8_t NRF_CSN_PIN = 14;
static const uint64_t NRF_PIPE_ADDR = 0xE7E7E7E7E7LL;

// ---- Calibration / timing ----
static constexpr uint32_t CAL_TIME_MS = 10000;
static constexpr uint32_t PRINT_DT_MS = 20;
static constexpr uint8_t GPS_TX_PIN = 0;
static constexpr uint8_t GPS_RX_PIN = 1;
static constexpr uint32_t GPS_BAUD  = 9600;
static constexpr uint32_t GPS_PRINT_MS = 200;
static constexpr float G0 = 9.80665f;
static constexpr float P0_PA_FALLBACK = 101325.0f;

// Pyro: GP10=MAIN, GP11=DROGUE, GP12=AIRSTART
static constexpr uint8_t MAIN_PIN     = 10;
static constexpr uint8_t DROGUE_PIN   = 11;
static constexpr uint8_t AIRSTART_PIN = 12;
static constexpr uint32_t PYRO_PULSE_MS = 1500;

// Flight config (same as FC1)
static constexpr float    LAUNCH_ALT_GAIN_M       = 15.0f;
static constexpr float    LAUNCH_ANET_THRESH      = 3.0f;
static constexpr uint32_t LAUNCH_CONFIRM_MS       = 200;
static constexpr uint32_t MIN_TIME_AFTER_ARM_MS   = 200;
static constexpr uint32_t APOGEE_MIN_MS           = 8000;
static constexpr uint32_t APOGEE_CONFIRM_MS       = 1000;
static constexpr float    APOGEE_VNEG_THRESH      = -4.0f;
static constexpr uint32_t DROGUE_MIN_MS           = 9000;
static constexpr uint32_t DROGUE_MAX_MS           = 13500;
static constexpr float    MAIN_DEPLOY_ALT_M       = 150.0f;
static constexpr uint32_t MAIN_MIN_MS             = 13500;
static constexpr uint32_t MAIN_MAX_MS             = 90000;
static constexpr bool     AIRSTART_ENABLED        = true;
static constexpr uint32_t AIRSTART_MS_AFTER_LAUNCH = 3000;
static constexpr uint32_t GROUND_BARO_STABLE_MS   = 30000;
static constexpr float    GROUND_BARO_THRESH_PA   = 15.0f;
static constexpr float    GROUND_ALT_NEAR_M       = 10.0f;
static constexpr uint32_t GROUND_ALT_HOLD_MS      = 5000;
static constexpr float ALT_LPF_ALPHA              = 0.05f;
static constexpr float VBARO_LPF_ALPHA            = 0.08f;
static constexpr float VEL_DEADBAND_MPS           = 0.3f;

// Attitude (complementary filter: gyro integrate + accel correct when near 1g)
static constexpr float ATTITUDE_ALPHA = 0.98f;
static constexpr float ACCEL_TRUST_THRESH = 2.0f;

// ---------------- OBJECTS ----------------
Adafruit_BMP3XX bmp;
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);

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

// ---------------- FLIGHT STATE (same as FC1) ----------------
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

// ---------------- ESTIMATOR ----------------
float alt_f = 0.0f, alt_prev = 0.0f, v_baro = 0.0f, v_est = 0.0f;
uint32_t last_estimate_ms = 0;

// ---------------- ATTITUDE (MPU6050: complementary filter, no mag) ----------------
float roll_deg = 0.0f, pitch_deg = 0.0f, yaw_deg = 0.0f;
uint32_t last_attitude_ms = 0;

// ---------------- TELEMETRY PACKET (28 bytes, same as FC1) ----------------
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
  for (size_t i = 0; i < sizeof(TelemetryPacket) - 1; ++i) cs ^= bytes[i];  // exclude checksum byte
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
    case FlightState::CALIBRATING:    return "CALIBRATING";
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

// ---------------- GPS ----------------
void gps_setup_uart0() {
  Serial1.setTX(GPS_TX_PIN);
  Serial1.setRX(GPS_RX_PIN);
  Serial1.begin(GPS_BAUD);
}
void gps_poll() {
  while (Serial1.available() > 0) gps.encode(Serial1.read());
}
void gps_print_summary(uint32_t now_ms) {
  static uint32_t last_gps_print = 0;
  if (now_ms - last_gps_print < GPS_PRINT_MS) return;
  last_gps_print = now_ms;
  Serial.print(" | GPS: ");
  if (gps.location.isValid()) {
    Serial.print("lat="); Serial.print(gps.location.lat(), 6);
    Serial.print(" lon="); Serial.print(gps.location.lng(), 6);
  } else Serial.print("lat=NA lon=NA");
  if (gps.altitude.isValid()) {
    Serial.print(" alt_m=");
    Serial.print(gps.altitude.meters(), 1);
  }
  else Serial.print(" alt_m=NA");
  if (gps.satellites.isValid()) {
     Serial.print(" sats=");
    Serial.print(gps.satellites.value());
  }
  else Serial.print(" sats=NA");
  Serial.println();
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

// ---------------- SENSOR INIT (called on START from READY) ----------------
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

  Serial.print("MPU6050 init... ");
  if (!mpu.begin(MPU6050_ADDR, &Wire)) {
    Serial.println("FAILED (try 0x68 or check wiring)");
    return false;
  }
  Serial.println("OK");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.print("NRF24L01 init (SPI0 GP4/6/7, CSN=GP14, CE=GP17)... ");
  SPI.setRX(4);
  SPI.setTX(7);
  SPI.setSCK(6);
  if (!radio.begin()) {
    Serial.println("FAILED (check CE/CSN/SPI wiring)");
    return false;
  }
  radio.setPayloadSize(sizeof(TelemetryPacket));
  radio.openWritingPipe(NRF_PIPE_ADDR);
  radio.setPALevel(RF24_PA_LOW);
  radio.stopListening();
  Serial.println("OK");
  return true;
}

// ---------------- COMMANDS (USB Serial only; no telemetry back-channel) ----------------
void handle_arm_command() {
  static char cmd_buf[16];
  static uint8_t cmd_len = 0;
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      cmd_buf[cmd_len] = '\0';
      if (strcmp(cmd_buf, "START") == 0 || strcmp(cmd_buf, "INIT") == 0) {
        if (flight_state == FlightState::READY) {
          Serial.print("[CMD] ");
          Serial.print(cmd_buf);
          Serial.println(" received — begin sensor init and calibration.");
          start_calibration_requested = true;
        }
      } else if (strcmp(cmd_buf, "ARM") == 0) {
        armed = true;
        Serial.println("[CMD] ARM received (USB Serial)");
      } else if (strcmp(cmd_buf, "DISARM") == 0) {
        armed = false;
        Serial.println("[CMD] DISARM received (USB Serial)");
      } else if (cmd_len > 0) {
        Serial.print("[CMD] Unknown: ");
        Serial.println(cmd_buf);
      }
      cmd_len = 0;
    } else {
      if (cmd_len < 15) { cmd_buf[cmd_len] = c; cmd_len++; }
    }
  }
}

// ---------------- STATE MACHINE (same as FC1: DROGUE_DESCENT, MAIN_DESCENT, ground detection) ----------------
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
      float alt_gain = alt_f;
      bool alt_trigger = (baro_ok && (alt_gain >= LAUNCH_ALT_GAIN_M));
      bool imu_trigger = (a_net_mag >= LAUNCH_ANET_THRESH);
      bool launch_cond = (alt_trigger || imu_trigger);
      if (launch_cond) {
        if (launch_trigger_start_ms == 0) launch_trigger_start_ms = now_ms;
      } else launch_trigger_start_ms = 0;
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
        Serial.print("[EVENT] AIRSTART at t_ms="); Serial.println(t_since_launch);
      }
      if (t_since_launch >= APOGEE_MIN_MS) {
        bool vneg = (v_est <= APOGEE_VNEG_THRESH);
        if (vneg) { if (vneg_start_ms == 0) vneg_start_ms = now_ms; }
        else vneg_start_ms = 0;
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
      if (!drogue_marked && (t_since_launch >= DROGUE_MAX_MS || t_since_launch >= DROGUE_MIN_MS)) {
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
        if (dP <= GROUND_BARO_THRESH_PA) {
          if (ground_stable_start_ms == 0) ground_stable_start_ms = now_ms;
          else if ((now_ms - ground_stable_start_ms) >= GROUND_BARO_STABLE_MS) {
            flight_state = FlightState::FINISHED;
            state_entry_ms = now_ms;
            Serial.println("[STATE] MAIN_DESCENT -> FINISHED (baro stable 30s)");
            break;
          }
        } else ground_stable_start_ms = 0;
      }
      if (isfinite(alt_f) && alt_f >= -GROUND_ALT_NEAR_M && alt_f <= GROUND_ALT_NEAR_M) {
        if (alt_near_ground_start_ms == 0) alt_near_ground_start_ms = now_ms;
        else if ((now_ms - alt_near_ground_start_ms) >= GROUND_ALT_HOLD_MS) {
          flight_state = FlightState::FINISHED;
          state_entry_ms = now_ms;
          Serial.println("[STATE] MAIN_DESCENT -> FINISHED (alt near pad 5s)");
        }
      } else alt_near_ground_start_ms = 0;
      break;
    }
    case FlightState::FINISHED:
    default:
      break;
  }
  if (drogue_fire_start_ms > 0 && (now_ms - drogue_fire_start_ms) >= PYRO_PULSE_MS) {
    digitalWrite(DROGUE_PIN, LOW); drogue_fire_start_ms = 0; Serial.println("[PYRO] DROGUE done");
  }
  if (main_fire_start_ms > 0 && (now_ms - main_fire_start_ms) >= PYRO_PULSE_MS) {
    digitalWrite(MAIN_PIN, LOW); main_fire_start_ms = 0; Serial.println("[PYRO] MAIN done");
  }
  if (airstart_fire_start_ms > 0 && (now_ms - airstart_fire_start_ms) >= PYRO_PULSE_MS) {
    digitalWrite(AIRSTART_PIN, LOW); airstart_fire_start_ms = 0; Serial.println("[PYRO] AIRSTART done");
  }
}

// ---------------- LED ----------------
enum class LedPattern : uint8_t { OFF = 0, SOLID, BLINK_1HZ, BLINK_4HZ, BURST_2, BURST_3, BURST_4, HEARTBEAT };
LedPattern pattern_for_state(bool calibrated, FlightState s, bool armed_flag) {
  if (s == FlightState::READY)       return LedPattern::BLINK_1HZ;
  if (s == FlightState::CALIBRATING) return LedPattern::BLINK_1HZ;
  if (s == FlightState::INIT)        return LedPattern::BURST_2;
  if (s == FlightState::ARMED)       return LedPattern::BLINK_4HZ;
  switch (s) {
    case FlightState::BOOST:          return LedPattern::SOLID;
    case FlightState::APOGEE:         return LedPattern::BURST_3;
    case FlightState::DEPLOY_DROGUE:  return LedPattern::BURST_3;
    case FlightState::DROGUE_DESCENT: return LedPattern::BURST_3;
    case FlightState::DEPLOY_MAIN:    return LedPattern::BURST_4;
    case FlightState::MAIN_DESCENT:   return LedPattern::BURST_4;
    case FlightState::FINISHED:       return LedPattern::HEARTBEAT;
    default:                         return LedPattern::OFF;
  }
}
void led_update(uint32_t now_ms, LedPattern pat) {
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

// ---------------- SEND TELE (FC2: NRF24L01, 28-byte packet like FC1) ----------------
void send_telemetry(uint32_t now_ms) {
  TelemetryPacket pkt{};
  pkt.preamble[0] = 'R'; pkt.preamble[1] = 'C';
  pkt.time_ms = now_ms;
  float alt_m = alt_f, vel_mps = v_est;
  if (!isfinite(alt_m)) alt_m = 0.0f;
  if (!isfinite(vel_mps)) vel_mps = 0.0f;
  pkt.alt_cm   = (int32_t)lroundf(alt_m * 100.0f);
  pkt.vel_cmps = (int16_t)lroundf(vel_mps * 100.0f);
  if (gps.location.isValid()) {
    pkt.lat_1e7 = (int32_t)llround(gps.location.lat() * 1e7);
    pkt.lon_1e7 = (int32_t)llround(gps.location.lng() * 1e7);
  } else { pkt.lat_1e7 = INT32_MIN; pkt.lon_1e7 = INT32_MIN; }
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
  radio.stopListening();
  radio.write(reinterpret_cast<const void*>(&pkt), sizeof(pkt));
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 1500) delay(10);

  Serial.println("\n=== FC2: BMP388 + MPU6050 + NRF24L01 telemetry ===");
  Serial.println("State READY. Send 'START' via USB Serial to begin init and calibration.");
  Serial.println("ARM/DISARM via USB Serial. Telemetry is one-way radio.");

  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin();
  Wire.setClock(400000);

  gps_setup_uart0();
  Serial.println("GPS UART started.");

  flight_state = FlightState::READY;
  calibrated = false;
  state_entry_ms = millis();

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
  LedPattern pat = pattern_for_state(calibrated, flight_state, armed);
  led_update(now, pat);

  // Telemetry at 10 Hz when radio is initialized (after calibration)
  static uint32_t last_tel_ms = 0;
  if (calibrated && (now - last_tel_ms >= 100)) {
    last_tel_ms = now;
    send_telemetry(now);
  }

  handle_arm_command();
  gps_poll();

  // READY: wait for START; no baro/IMU until then
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

  bool bmp_ok = bmp.performReading();
  float P_pa = bmp_ok ? bmp.pressure : NAN;
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  bool mpu_ok = true;

  if (flight_state == FlightState::CALIBRATING) {
    if (bmp_ok && isfinite(P_pa) && P_pa > 1000.0f) { p0_pa_sum += (double)P_pa; p0_count++; }
    if (mpu_ok) {
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
      Serial.println("(MPU6050: no mag — yaw from gyro integration)");
      Serial.println("============================\n");
      Serial.println("Calibration done. Send ARM via USB Serial.");
    }
    delay(10);
    return;
  }

  if (!calibrated) { delay(10); return; }

  static uint32_t last_print = 0;
  if (now - last_print < PRINT_DT_MS) { gps_print_summary(now); return; }
  last_print = now;

  uint32_t t_ms = now - t0_ms;
  float alt_m = NAN;
  if (bmp_ok && isfinite(P_pa))
    alt_m = altitude_from_pressure_relative(P_pa, P0_pa) - alt0_m;  // relative to pad

  float a_net_mag = 0.0f;
  if (mpu_ok) {
    float ax = accel.acceleration.x, ay = accel.acceleration.y, az = accel.acceleration.z;
    float a_mag = sqrtf(ax*ax + ay*ay + az*az);
    a_net_mag = a_mag - G0;
    if (a_net_mag < 0.0f) a_net_mag = 0.0f;
  }

  if (isfinite(alt_m)) estimator_update(now, alt_m);

  // Attitude: complementary filter (gyro integrate + accel correct when near 1g); no mag
  float down_x = NAN, down_y = NAN, down_z = NAN;
  if (mpu_ok) {
    float ax = accel.acceleration.x, ay = accel.acceleration.y, az = accel.acceleration.z;
    float a_mag = sqrtf(ax*ax + ay*ay + az*az);
    down_x = ax; down_y = ay; down_z = az;
    normalize3(down_x, down_y, down_z);
    if (last_attitude_ms > 0) {
      float dt_att = (now - last_attitude_ms) / 1000.0f;
      if (dt_att > 0.0f && dt_att < 1.0f) {
        float gx = gyro.gyro.x, gy = gyro.gyro.y, gz = gyro.gyro.z;  // rad/s
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
  if (mpu_ok) {
    Serial.print(" | a_excess="); Serial.print(a_net_mag, 2);
    Serial.print(" | r/p/y="); Serial.print(roll_deg, 1); Serial.print(","); Serial.print(pitch_deg, 1); Serial.print(","); Serial.print(yaw_deg, 1);
    Serial.print(" | down=");
    Serial.print(down_x, 4); Serial.print(",");
    Serial.print(down_y, 4); Serial.print(",");
    Serial.print(down_z, 4);
  } else Serial.print(" | mpu=ERR");
  Serial.println();
  gps_print_summary(now);
}
