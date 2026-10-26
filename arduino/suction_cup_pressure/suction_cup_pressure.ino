// Suction-cup chamber pressure: Adafruit Feather ESP32 V2 or ESP32-S3 + PCA9548A/TCA9548A I2C mux + one Honeywell
// MPRLS per chamber. Read by src/ESP32_Pressure.py over USB serial.
//
// Commands:  's' start streaming   'i' idle   '?' status report (lines start with '#', ESP32_Pressure.py skips them)
// Streaming: one line per sample, N_SENSORS absolute pressures in Pa separated by spaces, chamber 1 first.
//            A chamber whose read fails prints "nan"; the other chambers keep streaming.
//
// Acquisition: trigger every sensor, then read each once its conversion is done, so a sample takes about one
// conversion time (~5 ms) whatever the number of sensors. SAMPLE_HZ > 0 sends lines at that fixed rate (144 Hz = the
// simulator's sensor model; use it for the AIS robot demo). SAMPLE_HZ 0 starts the next sample right away
// (fastest, rate set by the sensors, not constant).
//
// Robustness: a sensor that does not answer at boot is reported and printed as "nan" instead of halting the board
// (the earlier sketches stopped in setup, which looks like "no data at all" on the ROS side because the one error
// line is printed before the PC opens the port). Missing sensors are probed again on every 's' and '?', so a reseated
// cable comes back without a reset.
//
// MPRLS (0-25 psi, I2C 0x18): write 0xAA 0x00 0x00 to start a conversion; read 4 bytes = status + 24-bit count.
// Status: bit6 powered, bit5 busy, bit2 memory error, bit0 math saturation.
// psi = (count - 10 % of 2^24) * 25 / (80 % of 2^24); Pa = psi * 6894.757.
// Both Feather variants switch the STEMMA QT / I2C power on in initVariant(), so no power pin handling is needed here.
//
// Status LEDs (both Feathers: red LED on GPIO 13 next to the USB port, NeoPixel inside; see LED STATUS below):
//   red LED                                   NeoPixel
//   slow blink (1 Hz)   idle, sensors ok      blue
//   solid on            streaming, ok         green
//   k blinks + pause    chamber k failing     orange   (not responding, or a read error within the last second;
//                                                       the lowest failing chamber is shown, '?' lists all)
//   fast blink (5 Hz)   mux not found         red
//   -                   command received      short white flash

#include <Wire.h>

// ===== configuration =================================================================================================
// 3-chamber cup (Feather ESP32-S3):  N_SENSORS 3, MUX_PORT {0, 1, 2}
// 4-chamber cup (Feather ESP32 V2):  N_SENSORS 4, MUX_PORT {0, 1, 2, 3}
// 6-chamber cup:                     N_SENSORS 6, MUX_PORT {0, 1, 2, 3, 4, 5}; with the V2 raise SERIAL_BAUD (below)
#define N_SENSORS 3
const uint8_t MUX_PORT[N_SENSORS] = {0, 1, 2};     // mux port of chamber 1, 2, ... (output order)

#define SAMPLE_HZ 144.0          // fixed output rate; 0 = as fast as the sensors convert
#define START_STREAMING false    // true: stream from power-up without waiting for 's'

// Feather ESP32 V2: USB goes through a UART bridge, so the baud rate is the real limit
// (a line is ~10 bytes per chamber; 115200 baud carries ~11 500 bytes/s, i.e. ~190 lines/s with 6 chambers).
// If you change it, pass the same value to ESP32_Pressure.py (ROS parameter baudrate).
// Feather ESP32-S3: native USB, the value is ignored.
#define SERIAL_BAUD 115200
// =====================================================================================================================

#define MUX_ADDR 0x70
#define MPR_ADDR 0x18
#define MPR_STATUS_POWERED 0x40
#define MPR_STATUS_BUSY 0x20
#define MPR_STATUS_ERRORS 0x05                     // memory integrity (bit2) | math saturation (bit0)
#define CONVERSION_MIN_US 3000                     // MPRLS conversion is ~5 ms; poll the busy bit after this
#define CONVERSION_TIMEOUT_US 20000

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif
#define NEOPIXEL_LEVEL 24                          // NeoPixel brightness 0-255 (it sits right under the board)
#define ERROR_HOLD_MS 1000                         // a read error marks the chamber failing for this long

const float PSI_TO_PA = 6894.757f;
const uint32_t COUNT_MIN = 0x19999A;               // 10 % of 2^24
const uint32_t COUNT_SPAN = 0xE66666 - 0x19999A;   // 80 % of 2^24

enum State { IDLE, STREAMING };
State state = IDLE;
bool mux_ok = false;
bool present[N_SENSORS];
uint32_t samples = 0, read_errors[N_SENSORS];
uint32_t last_error_ms[N_SENSORS];
uint32_t flash_until_ms = 0;
uint32_t period_us = 0, next_us = 0, stream_start_us = 0;

bool muxSelect(uint8_t port) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << port);
  return Wire.endTransmission() == 0;
}

bool mprStart(uint8_t port) {
  if (!muxSelect(port)) return false;
  Wire.beginTransmission(MPR_ADDR);
  Wire.write(0xAA); Wire.write(0x00); Wire.write(0x00);
  return Wire.endTransmission() == 0;
}

// Read a conversion started by mprStart(): wait for the busy bit to clear, then read.
// Returns NAN on a bus error, a busy timeout (measured from t_start) or an error status bit.
float mprRead(uint8_t port, uint32_t t_start) {
  if (!muxSelect(port)) return NAN;
  uint8_t status;
  while (true) {
    if (Wire.requestFrom((uint8_t)MPR_ADDR, (uint8_t)1) != 1) return NAN;
    status = Wire.read();
    if (!(status & MPR_STATUS_BUSY)) break;
    if (micros() - t_start > CONVERSION_TIMEOUT_US) return NAN;
    delayMicroseconds(100);
  }
  if (!(status & MPR_STATUS_POWERED) || (status & MPR_STATUS_ERRORS)) return NAN;
  if (Wire.requestFrom((uint8_t)MPR_ADDR, (uint8_t)4) != 4) return NAN;
  status = Wire.read();
  uint32_t count = ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
  if (status & (MPR_STATUS_BUSY | MPR_STATUS_ERRORS)) return NAN;
  float psi = ((float)count - (float)COUNT_MIN) * 25.0f / (float)COUNT_SPAN;
  return psi * PSI_TO_PA;
}

bool probe(uint8_t port) {
  if (!mprStart(port)) return false;
  return !isnan(mprRead(port, micros()));
}

void probeMissing() {
  Wire.beginTransmission(MUX_ADDR);
  mux_ok = Wire.endTransmission() == 0;
  for (int k = 0; k < N_SENSORS; k++) {
    if (!present[k]) present[k] = mux_ok && probe(MUX_PORT[k]);
  }
}

void report() {
  Serial.print("# sensors "); Serial.print(N_SENSORS);
  Serial.print(", mode "); if (SAMPLE_HZ > 0) { Serial.print(SAMPLE_HZ, 1); Serial.print(" Hz fixed"); } else Serial.print("free run");
  Serial.print(", mux "); Serial.println(mux_ok ? "ok" : "NOT FOUND at 0x70");
  for (int k = 0; k < N_SENSORS; k++) {
    Serial.print("# chamber "); Serial.print(k + 1); Serial.print(" (mux port "); Serial.print(MUX_PORT[k]);
    Serial.print("): "); Serial.print(present[k] ? "ok" : "NOT RESPONDING");
    Serial.print(", read errors "); Serial.println(read_errors[k]);
  }
  Serial.print("# mux ports with an MPRLS (0x18):");
  for (uint8_t port = 0; port < 8; port++) {
    if (!muxSelect(port)) { Serial.print(" (mux not answering)"); break; }
    Wire.beginTransmission(MPR_ADDR);
    if (Wire.endTransmission() == 0) { Serial.print(' '); Serial.print(port); }
  }
  Serial.println();
  if (state == STREAMING && samples > 1) {
    Serial.print("# streamed "); Serial.print(samples); Serial.print(" samples at ");
    Serial.print(samples * 1e6 / (float)(micros() - stream_start_us), 1); Serial.println(" Hz");
  }
}

// ===== LED STATUS =====================================================================================================
int failingChamber() {                             // 1-based number of the lowest failing chamber, 0 if none
  uint32_t now = millis();
  for (int k = 0; k < N_SENSORS; k++) {
    if (!present[k] || (last_error_ms[k] && now - last_error_ms[k] < ERROR_HOLD_MS)) return k + 1;
  }
  return 0;
}

void setNeoPixel(uint8_t r, uint8_t g, uint8_t b) {
#ifdef PIN_NEOPIXEL
  static int32_t last = -1;
  int32_t rgb = ((int32_t)r << 16) | ((int32_t)g << 8) | b;
  if (rgb == last) return;                         // only talk to the pixel when the colour changes
  last = rgb;
  rgbLedWrite(PIN_NEOPIXEL, r, g, b);
#endif
}

void updateLeds() {
  uint32_t now = millis();
  int bad = failingChamber();
  bool red;
  if (!mux_ok) {
    red = (now / 100) % 2;                                         // fast blink
    setNeoPixel(NEOPIXEL_LEVEL, 0, 0);
  } else if (bad) {
    uint32_t cycle = bad * 400 + 1200, t = now % cycle;             // 'bad' blinks of 200 ms, then a pause
    red = t < (uint32_t)bad * 400 && (t % 400) < 200;
    setNeoPixel(NEOPIXEL_LEVEL, NEOPIXEL_LEVEL / 3, 0);
  } else if (state == STREAMING) {
    red = true;
    setNeoPixel(0, NEOPIXEL_LEVEL, 0);
  } else {
    red = (now % 1000) < 500;                                      // slow blink
    setNeoPixel(0, 0, NEOPIXEL_LEVEL);
  }
  if ((int32_t)(flash_until_ms - now) > 0) setNeoPixel(NEOPIXEL_LEVEL, NEOPIXEL_LEVEL, NEOPIXEL_LEVEL);
  digitalWrite(LED_BUILTIN, red ? HIGH : LOW);
}
// =====================================================================================================================

void startStreaming() {
  probeMissing();
  state = STREAMING;
  samples = 0;
  stream_start_us = next_us = micros();
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Wire.begin();
  Wire.setClock(400000);
  delay(50);
  for (int k = 0; k < N_SENSORS; k++) { present[k] = false; read_errors[k] = 0; last_error_ms[k] = 0; }
  probeMissing();
  if (SAMPLE_HZ > 0) period_us = (uint32_t)(1e6 / SAMPLE_HZ);
  report();
  if (START_STREAMING) startStreaming();
}

void handleCommand() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 's' || c == 'i' || c == '?') flash_until_ms = millis() + 150;
    if (c == 's' && state != STREAMING) {
      startStreaming();
    } else if (c == 'i') {
      state = IDLE;
    } else if (c == '?') {
      probeMissing();
      report();
    }
  }
}

void loop() {
  handleCommand();
  updateLeds();
  if (state != STREAMING) { delay(2); return; }
  if (SAMPLE_HZ > 0) {                                             // fixed rate
    if ((int32_t)(micros() - next_us) < 0) return;
    next_us += period_us;
    if ((int32_t)(micros() - next_us) > (int32_t)(4 * period_us)) next_us = micros() + period_us;   // fell behind
  }

  float p[N_SENSORS];
  uint32_t t_start = micros();
  for (int k = 0; k < N_SENSORS; k++) {                            // trigger every sensor ...
    p[k] = (present[k] && mprStart(MUX_PORT[k])) ? 0.0f : NAN;
  }
  delayMicroseconds(CONVERSION_MIN_US);                            // ... let them convert in parallel ...
  for (int k = 0; k < N_SENSORS; k++) {                            // ... then read each one
    if (!isnan(p[k])) p[k] = mprRead(MUX_PORT[k], t_start);
    if (isnan(p[k])) { read_errors[k]++; last_error_ms[k] = millis() | 1; }
  }

  for (int k = 0; k < N_SENSORS; k++) {
    if (k) Serial.print(' ');
    if (isnan(p[k])) Serial.print("nan"); else Serial.print(p[k], 1);
  }
  Serial.println();
  samples++;
}
