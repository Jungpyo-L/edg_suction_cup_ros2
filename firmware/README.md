# Firmware

`ESP32S3_MPRLS_pressure/`: chamber pressure board (Adafruit Feather ESP32-S3 No PSRAM + PCA9548A mux + one MPRLS per
chamber), read by `src/ESP32_Pressure.py`. Set `N_SENSORS` and `MUX_PORT` at the top for the cup.

- Output: exactly N absolute pressures (Pa) per line at a fixed 144 Hz (the simulator's sensor model), `nan` for a
  failed read. Commands: `s` stream, `i` idle, `?` status, read-error counts, achieved rate and a mux port scan.
- Replaces the old `PCA_MPR_state_ch*.ino` sketches (ROS1 repo), which needed the unavailable `Adafruit_MPRLS_JP`
  library, printed sensor 3 twice on the 3-chamber board, hung when a sensor did not answer, and ran at ~50 Hz.

Build and upload with arduino-cli (installed in ~/.local/bin, core esp32:esp32). Stop `ESP32_Pressure.py` first:

    arduino-cli compile --fqbn esp32:esp32:adafruit_feather_esp32s3_nopsram firmware/ESP32S3_MPRLS_pressure
    arduino-cli upload -p /dev/ttyPressure --fqbn esp32:esp32:adafruit_feather_esp32s3_nopsram firmware/ESP32S3_MPRLS_pressure

2026-10-26: uploaded (3 sensors, ports 0/1/2). 143.8 Hz. Chamber 2 (port 1) did not answer on any mux port.
