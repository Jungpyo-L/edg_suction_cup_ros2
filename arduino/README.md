# Arduino firmware

## `suction_cup_pressure/` — chamber pressure board

Adafruit Feather ESP32 V2 or ESP32-S3 + PCA9548A/TCA9548A mux (0x70) + one MPRLS per chamber (0x18).
Read by `src/ESP32_Pressure.py` (`ros2 run suction_cup ESP32_Pressure.py --ch <n>`).

Set at the top of the sketch:

| Setting | 3-ch cup (ESP32-S3) | 4-ch cup (ESP32 V2) | Notes |
|---|---|---|---|
| `N_SENSORS` | 3 | 4 | values per line |
| `MUX_PORT` | `{0, 1, 2}` | `{0, 1, 2, 3}` | mux port of chamber 1, 2, ... |
| `SAMPLE_HZ` | 144.0 | 144.0 or 0 | 144 = simulator sensor model (use for the AIS robot demo); 0 = as fast as possible, rate not constant |
| `SERIAL_BAUD` | ignored (native USB) | 115200 | V2 goes through a UART bridge; raise it for 6 chambers and pass the same `baudrate` to ESP32_Pressure.py |

Serial protocol (unchanged for ESP32_Pressure.py): `s` stream, `i` idle. Output: N pressures in Pa per line,
`nan` for a chamber whose read failed (the others keep streaming). New: `?` prints a status block (`#` lines:
sensor ok / not responding, read-error counts, achieved rate, and a scan of which mux ports have an MPRLS).
Missing sensors are probed again on every `s` and `?`, so a reseated cable comes back without a reset.

Differences from the earlier sketches (ROS1 `PCA_MPR_state_ch*.ino`, and the 4-ch free-run sketch of 2026-10):
no external library (the old ones needed the unavailable `Adafruit_MPRLS_JP`); exactly N values (the 3-ch sketch
printed sensor 3 twice); no halt in `setup()` when a sensor does not answer; one failing chamber no longer stops the
whole stream; fixed-rate option.

### Status LEDs

Both Feathers: red LED on GPIO 13 (next to the USB port, easy to see) and the NeoPixel (S3: GPIO 33, V2: GPIO 0).

| Board state | Red LED | NeoPixel |
|---|---|---|
| idle, all sensors ok | slow blink (1 Hz) | blue |
| streaming, all sensors ok | solid on | green |
| chamber *k* failing (not responding, or a read error in the last second) | *k* short blinks, pause, repeat | orange |
| mux not found at 0x70 | fast blink (5 Hz) | red |
| command (`s`, `i`, `?`) received | - | short white flash |

With several failing chambers the lowest number blinks; `?` lists all of them.

### Upload

Arduino IDE (Windows): install the "esp32 by Espressif Systems" boards package, open
`suction_cup_pressure/suction_cup_pressure.ino`, choose the board (Adafruit Feather ESP32 V2 or Adafruit Feather
ESP32-S3 No PSRAM) and port, upload. Then open the Serial Monitor (115200), send `?`, then `s`.

arduino-cli on edg-3090 (stop ESP32_Pressure.py first, it holds the port):

    arduino-cli compile --fqbn esp32:esp32:adafruit_feather_esp32s3_nopsram arduino/suction_cup_pressure
    arduino-cli upload  -p /dev/ttyPressure --fqbn esp32:esp32:adafruit_feather_esp32s3_nopsram arduino/suction_cup_pressure
    (Feather ESP32 V2: esp32:esp32:adafruit_feather_esp32_v2)

Compiled 2026-10-26 for 3 ch / ESP32-S3 / 144 Hz, 4 ch / V2 / free run, 6 ch / V2 / 144 Hz.

### Status (2026-10-26)

The 3-ch board (ESP32-S3) runs the earlier version of this sketch at 143.8 Hz. Chamber 2 (mux port 1) does not
answer on any mux port: check the sensor cable / sensor.
