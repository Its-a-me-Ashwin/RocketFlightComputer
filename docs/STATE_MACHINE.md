# Flight computer state machine (FC1)

## States

| State             | Description |
|-------------------|-------------|
| **READY**         | Powered on. Baro/IMU not inited. Waits for **START** from ground station. |
| **CALIBRATING**   | Baro/IMU inited, 10 s baseline. Then → INIT. |
| **INIT**          | Calibration done. Waiting for ARM command. |
| **ARMED**         | Armed, on pad. Waiting for launch detection. |
| **BOOST**         | Launch detected. Airstart may fire. Waiting for apogee. |
| **APOGEE**        | Apogee detected. Immediate → DEPLOY_DROGUE. |
| **DEPLOY_DROGUE** | Drogue deploy window. Fires drogue; then → DROGUE_DESCENT. |
| **DROGUE_DESCENT**| After drogue deployed. Waiting for main deploy conditions. |
| **DEPLOY_MAIN**   | Ephemeral: main pyro firing. Immediate → MAIN_DESCENT. |
| **MAIN_DESCENT**  | Under main chute. Ground detection → FINISHED. |
| **FINISHED**      | On ground. Flight over, still logging. |

---

## State transitions

```
    Power on
       │
       ▼
   ┌────────┐     "START" (ground station)     ┌──────────────┐
   │ READY  │  ─────────────────────────────►  │ CALIBRATING  │
   └────────┘                                   └──────┬───────┘
                                                       │ 10 s
                                                       ▼
   "ARM" command                              ┌──────────────┐
   ┌────────┐  ─────────────────────────────► │    INIT      │
   └────────┘                                  └──────┬───────┘
                                                      │
                                                      ▼
                                               ┌──────────────┐
                                               │    ARMED     │
                                               └──────┬───────┘
                                                      │ Launch
                                                      ▼
                                               ┌──────────────┐
                                               │    BOOST     │
                                               └──────┬───────┘
                                                      │ Apogee
                                                      ▼
                                               ┌──────────────┐
                                               │   APOGEE     │  (instant)
                                               └──────┬───────┘
                                                      ▼
                                               ┌──────────────────┐
                                               │ DEPLOY_DROGUE    │
                                               └────────┬─────────┘
                                                        │ Drogue fired
                                                        ▼
                                               ┌──────────────────┐
                                               │ DROGUE_DESCENT   │
                                               └────────┬─────────┘
                                                        │ Main conditions
                                                        ▼
                                               ┌──────────────────┐
                                               │  DEPLOY_MAIN     │  (ephemeral)
                                               └────────┬─────────┘
                                                        │
                                                        ▼
                                               ┌──────────────────┐
                                               │  MAIN_DESCENT    │
                                               └────────┬─────────┘
                                                        │ Ground
                                                        ▼
                                               ┌──────────────────┐
                                               │    FINISHED      │
                                               └──────────────────┘
```

---

## Transition conditions (summary)

| From           | To             | Condition |
|----------------|----------------|-----------|
| READY          | CALIBRATING    | Ground station sends **START** or **INIT**. Baro/IMU init, then 10 s baseline. |
| CALIBRATING    | INIT           | 10 s elapsed; baselines locked. |
| INIT           | ARMED          | User sends **ARM** (USB or telemetry serial). |
| ARMED          | BOOST          | Launch: alt_gain ≥ 15 m **or** a_net ≥ 3 m/s² for 200 ms. |
| BOOST          | APOGEE         | Apogee: v_est ≤ -4 m/s for 1 s **or** t_since_launch ≥ 13.5 s. |
| APOGEE         | DEPLOY_DROGUE  | Immediate. |
| DEPLOY_DROGUE  | DROGUE_DESCENT | Drogue fired (window 9–13.5 s or forced at 13.5 s). |
| DROGUE_DESCENT | DEPLOY_MAIN    | Main conditions: alt_f ≤ 150 m and t ≥ 13.5 s **or** t ≥ 90 s. Main pyro fires. |
| DEPLOY_MAIN    | MAIN_DESCENT   | Immediate (ephemeral). |
| MAIN_DESCENT   | FINISHED       | **Ground**: baro stable (ΔP ≤ 15 Pa) for 30 s **or** alt_f within ±10 m of pad for 5 s. |

---

## Constants (from code)

- **LAUNCH_ALT_GAIN_M** = 15 m  
- **LAUNCH_ANET_THRESH** = 3 m/s²  
- **LAUNCH_CONFIRM_MS** = 200 ms  
- **MIN_TIME_AFTER_ARM_MS** = 200 ms  
- **APOGEE_MIN_MS** = 8000 ms, **APOGEE_CONFIRM_MS** = 1000 ms, **APOGEE_VNEG_THRESH** = -4 m/s  
- **DROGUE_MIN_MS** = 9000 ms, **DROGUE_MAX_MS** = 13500 ms  
- **MAIN_DEPLOY_ALT_M** = 150 m, **MAIN_MIN_MS** = 13500 ms, **MAIN_MAX_MS** = 90000 ms  
- **AIRSTART_MS_AFTER_LAUNCH** = 3000 ms (if enabled)  
- **GROUND_BARO_STABLE_MS** = 30000 ms, **GROUND_BARO_THRESH_PA** = 15 Pa  
- **GROUND_ALT_NEAR_M** = 10 m, **GROUND_ALT_HOLD_MS** = 5000 ms  

---

## Ground station

- Send **START** (or **INIT**) on the FC telemetry serial port to take the FC from READY into calibration.  
- Send **ARM** when in INIT to arm for launch.
