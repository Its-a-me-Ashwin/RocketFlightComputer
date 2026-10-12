# RocketFlightComputer

DIY RP2040-based flight computer for rockets, with a LoRa telemetry ground station.

## Current version (`main` branch)

Three components:

| File | Role |
|---|---|
| `RocketComputer/RocketComputer.ino` | Flight controller — Adafruit Feather RP2040 RFM95 (inbuilt LoRa 915 MHz), BMP390 baro, LSM6DSOX IMU, GPS on UART0, 3 pyro channels (MAIN=GP9, DROGUE=GP10, AIRSTART=GP12) |
| `LoRa_SerialBridge/LoRa_SerialBridge.ino` | Ground-side LoRa↔Serial bridge (same Feather board), connects to the PC at 57600 baud |
| `groundStation.py` | Python GUI (PySide6) — telemetry HUD, commands, FC settings |

Older versions (FC1/FC2, NRF24 bridge, V1) live on the `legacy` branch.

### Flight state machine

`READY → CALIBRATING → INIT → ARMED → BOOST → APOGEE → DEPLOY_DROGUE → DROGUE_DESCENT → DEPLOY_MAIN → MAIN_DESCENT → FINISHED`

- **START** (USB or LoRa) begins sensor init + 10 s calibration. **No auto-arm**: the FC
  enters ARMED only on an explicit **ARM** command from INIT.
- Sitting in ARMED for the idle timeout (default 5 min, configurable) returns to INIT;
  send **ARM** again to re-arm. **DISARM** from ARMED also returns to INIT (safe pad disarm).
- Launch detect: 15 m alt gain or 3 m/s² net accel, confirmed 200 ms. Apogee: velocity
  ≤ −4 m/s for 1 s after 8 s, failsafe at 17 s. Drogue at ≥ 10 s. Main at ≥ 18 s and
  ≤ 200 m AGL, failsafe 40 s. All thresholds/times are settable (see below).

### Communication: TDD until ARMED, one-way after

One radio per side = half-duplex, so the link is **time-division duplexed**: in
READY/CALIBRATING/INIT the FC listens for commands between its 10 Hz telemetry bursts,
and every command is answered with an ACK/NACK text reply. From **ARMED** onward the FC
stops listening — telemetry flows one way (rocket → ground) for the whole flight. The
ground bridge repeats each command 3× to punch through telemetry bursts.

Text protocol (USB serial @ 115200 or LoRa via bridge):

```
START            begin init + calibration (READY only)   -> ACK START / NACK START STATE <s>
ARM              INIT -> ARMED                           -> ACK ARM / NACK ARM STATE <s>
DISARM           ARMED|INIT -> disarm                    -> ACK DISARM / NACK DISARM IN FLIGHT
PING             link check                              -> PONG
CFG?             dump all settings                       -> CFG <key> <value> ... CFG END
SET <key> <val>  change a setting (READY/INIT only)      -> ACK SET <key> <val> / NACK SET <key> RANGE|LOCKED|...
SAVECFG          persist settings to flash               -> ACK SAVECFG
```

Settings survive power cycles (stored in RP2040 flash with magic+version+CRC).
Configurable keys: launch thresholds/confirm, apogee window, drogue min/max, main
deploy altitude/min/max, airstart, ground detection, pyro pulse, armed idle timeout.
The GUI settings panel reads them on connect, applies changes, and saves to flash.

### Ground station

```bash
pip install -r requirements.txt   # or: pip install PySide6 pyserial
python groundStation.py
```

Connect the LoRa bridge's COM port (57600) to the **FC (LoRa → Serial)** panel. The window shows:

- **FC panel** — telemetry, START/ARM/DISARM buttons (ACK-confirmed), and the settings editor
- **Live plots** — altitude and velocity vs time (rolling 120 s window)
- **Live map** — OpenStreetMap tiles around the launch pad (±~1 km, zoom 16) with the
  rocket's track and current position. The PC's rough location is looked up via geo-IP at
  startup (with the first GPS fix as fallback) and the 3×3 tile square is downloaded on a
  background thread into a RAM cache, so the map never blocks telemetry. Map data
  © OpenStreetMap contributors.

## How to use
1. Simulate your rocket and get expected apogee/deploy times.
2. Set the min/max limits in the GUI settings panel (or rely on defaults) and **Save to flash**.
3. Turn on the FC, then the ground station — settings are read back automatically.
4. On the pad: **START** → wait for calibration → check settings → **ARM**.
5. Fly! Telemetry is one-way during the flight.

## Tests
* Tested on a J270 rocket motor for deployment of the main chute.
* Airst tested on a I205 and a H195 rocket motor setup.
