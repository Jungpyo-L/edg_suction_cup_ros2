// Suction-cup chamber pressure: Adafruit Feather ESP32-S3 + PCA9548A I2C mux + one Honeywell MPRLS per chamber.
// Talks to src/ESP32_Pressure.py over USB serial (115200):
//   's' start streaming, 'i' idle, '?' print sensor status, rate and a mux port scan (lines starting with '#').
// Streaming: one line per sample, N_SENSORS absolute pressures in Pa separated by spaces (exactly N values; the old
// PCA_MPR_state_3_sensors.ino printed sensor 2 twice). A sensor that fails a read reports nan for that sample.
//
// Replaces PCA_MPR_state_ch{3,4,5,6}.ino, which needed the unavailable Adafruit_MPRLS_JP library, hung in setup when
// one sensor did not answer, and read the sensors one after another (~50 Hz with 4 sensors). Here all sensors are
// triggered together, then read, so a sample takes about one conversion time (~5 ms) and lines leave at a fixed
// SAMPLE_HZ (144 Hz, the simulator's sensor model).
//
// MPRLS (0-25 psi, I2C 0x18): write 0xAA 0x00 0x00 to start a conversion; read 4 bytes = status + 24-bit count.
// Status: bit6 powered, bit5 busy, bit2 memory error, bit0 math saturation.
// psi = (count - 10 % of 2^24) * 25 / (80 % of 2^24); Pa = psi * 6894.757.

#include <Wire.h>

// ---- configuration -------------------------------------------------------------------------------------------------
#define N_SENSORS 3                                // chambers on this cup
const uint8_t MUX_PORT[N_SENSORS] = {0, 1, 2};     // PCA9548A port of chamber 1, 2, 3 (... in output order)
#define SAMPLE_HZ 144.0
// ---------------------------------------------------------------------------------------------------------------------

#define PCA_ADDR 0x70
#define MPR_ADDR 0x18
#define MPR_STATUS_POWERED 0x40
#define MPR_STATUS_BUSY 0x20
#define MPR_STATUS_ERRORS 0x05                     // memory integrity (bit2) | math saturation (bit0)
#define CONVERSION_TIMEOUT_US 20000

const float PSI_TO_PA = 6894.757f;
const uint32_t COUNT_MIN = 0x19999A;               // 10 % of 2^24
const uint32_t COUNT_SPAN = 0xE66666 - 0x19999A;   // 80 % of 2^24

enum State { IDLE, STREAMING };
State state = IDLE;
bool present[N_SENSORS];
uint32_t samples = 0, read_errors[N_SENSORS];
uint32_t period_us, next_us, stream_start_us;

bool muxSelect(uint8_t port) {   // PCA9548A: 8 ports
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(1 << port);
  return Wire.endTransmission() == 0;
}

bool mprStart(uint8_t port) {
  if (!muxSelect(port)) return false;
  Wire.beginTransmission(MPR_ADDR);
  Wire.write(0xAA); Wire.write(0x00); Wire.write(0x00);
  return Wire.endTransmission() == 0;
}

// Read one finished conversion; returns NAN on a bus error, a busy timeout or an error status bit.
float mprRead(uint8_t port) {
  if (!muxSelect(port)) return NAN;
  uint32_t t0 = micros();
  uint8_t status = MPR_STATUS_BUSY;
  while (true) {
    if (Wire.requestFrom((uint8_t)MPR_ADDR, (uint8_t)1) != 1) return NAN;
    status = Wire.read();
    if (!(status & MPR_STATUS_BUSY)) break;
    if (micros() - t0 > CONVERSION_TIMEOUT_US) return NAN;
    delayMicroseconds(200);
  }
  if (!(status & MPR_STATUS_POWERED) || (status & MPR_STATUS_ERRORS)) return NAN;
  if (Wire.requestFrom((uint8_t)MPR_ADDR, (uint8_t)4) != 4) return NAN;
  Wire.read();                                      // status again
  uint32_t count = ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
  float psi = ((float)count - (float)COUNT_MIN) * 25.0f / (float)COUNT_SPAN;
  return psi * PSI_TO_PA;
}

bool probe(uint8_t port) {
  if (!mprStart(port)) return false;
  delay(10);
  return !isnan(mprRead(port));
}

void report() {
  Serial.print("# sensors "); Serial.print(N_SENSORS); Serial.print(", target "); Serial.print(SAMPLE_HZ, 1);
  Serial.println(" Hz");
  for (int k = 0; k < N_SENSORS; k++) {
    Serial.print("# chamber "); Serial.print(k + 1); Serial.print(" (mux port "); Serial.print(MUX_PORT[k]);
    Serial.print("): "); Serial.print(present[k] ? "ok" : "NOT RESPONDING");
    Serial.print(", read errors "); Serial.println(read_errors[k]);
  }
  Serial.print("# mux ports with an MPRLS (0x18):");
  for (uint8_t port = 0; port < 8; port++) {
    if (!muxSelect(port)) { Serial.print(" mux?"); break; }
    Wire.beginTransmission(MPR_ADDR);
    if (Wire.endTransmission() == 0) { Serial.print(' '); Serial.print(port); }
  }
  Serial.println();
  if (state == STREAMING && samples > 1) {
    Serial.print("# streamed "); Serial.print(samples); Serial.print(" samples at ");
    Serial.print(samples * 1e6 / (float)(micros() - stream_start_us), 1); Serial.println(" Hz");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
#ifdef PIN_I2C_POWER
  pinMode(PIN_I2C_POWER, OUTPUT);                   // Feather ESP32-S2/S3: STEMMA QT / I2C power switch
  digitalWrite(PIN_I2C_POWER, HIGH);
  delay(10);
#endif
  Wire.begin();
  Wire.setClock(400000);
  delay(50);
  for (int k = 0; k < N_SENSORS; k++) {
    read_errors[k] = 0;
    present[k] = probe(MUX_PORT[k]);
  }
  period_us = (uint32_t)(1e6 / SAMPLE_HZ);
  report();
}

void handleCommand() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 's' && state != STREAMING) {
      state = STREAMING; samples = 0; stream_start_us = next_us = micros();
      digitalWrite(LED_BUILTIN, HIGH);
    } else if (c == 'i') {
      state = IDLE; digitalWrite(LED_BUILTIN, LOW);
    } else if (c == '?') {
      report();
    }
  }
}

void loop() {
  handleCommand();
  if (state != STREAMING) { delay(2); return; }
  if ((int32_t)(micros() - next_us) < 0) return;    // fixed-rate sampling
  next_us += period_us;
  if ((int32_t)(micros() - next_us) > (int32_t)(4 * period_us)) next_us = micros() + period_us;   // fell behind: resync

  for (int k = 0; k < N_SENSORS; k++) mprStart(MUX_PORT[k]);     // trigger every sensor ...
  delayMicroseconds(3000);                                        // ... most of a conversion (mprRead polls the rest) ...
  float p[N_SENSORS];
  for (int k = 0; k < N_SENSORS; k++) {                           // ... then read each
    p[k] = mprRead(MUX_PORT[k]);
    if (isnan(p[k])) read_errors[k]++;
  }
  for (int k = 0; k < N_SENSORS; k++) {
    if (k) Serial.print(' ');
    if (isnan(p[k])) Serial.print("nan"); else Serial.print(p[k], 1);
  }
  Serial.println();
  samples++;
}
