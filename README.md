# RocketFlightComputer


## This a simple DIY RP2040 based flight computer for use in rockets. 
### Components
* RP2040 RPi Pico
* BMP358 Baro
* ICM20948 IMU
* 915MHz Radio for Telemetry
* 2s 750mAh battery
* NEO6MV2 GPS Module

## Ground station
The ground station is a Python GUI (PySide6) that shows telemetry from FC1 and FC2 side by side.

```bash
pip install -r requirements.txt   # or: pip install PySide6 pyserial
python groundStation.py
```

Select COM ports for FC1 (direct serial) and FC2 (Pico NRF24 bridge), then Connect. ARM sends the arm command over the selected telemetry link.

## How to use.
* Simulate your rocket and get the expected times for apogee etc.
* Use the simulated graphs to set the min and max limits for the flight computer and upload the code.
* Turn on the FC and then the ground station.
* Once the rocket is on the pad, arm the computer via the ground station.
* Fly!

## Tests
* Tested on a J270 rocket motor for deployment of the main chute.
* Airst tested on a I205 and a H195 rocket motor setup.
