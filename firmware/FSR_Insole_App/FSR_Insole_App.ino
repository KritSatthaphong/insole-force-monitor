// FSR_Insole_App.ino  v1.4  (2026-11-12)  research build: 200 Hz, sample counter + acquisition timestamp
// Smart Insole -> Insole Force Monitor web app.
// Streams raw mV of 7 FlexiForce A401 channels + BMI270 IMU at 200 Hz over BOTH:
//   - USB serial (921600 baud on a CH343/UART port; native USB ignores the baud rate):
//       "D,<ms>,<mV1>..<mV7>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<apk>,<seq>,<us>"   + "#FOOT,L" and "#TIMING,..." every 2 s
//       ms  = millis() when the frame was finished (as in v1.0 - v1.3)
//       ax..az = mean accel of the frame (mg), gx..gz = mean gyro (0.1 deg/s), apk = peak |a - g| in the frame (mg)
//       seq = sample counter, uint32, +1 per acquisition cycle (first sample after boot = 0)
//       us  = micros() taken just BEFORE the 7-channel ADC burst of that cycle (acquisition start, uint32, wraps every 71.6 min;
//             the app unwraps it). The 7 channels are read one after the other during the following adc_us (see #TIMING).
//       Lines without IMU: "D,<ms>,<mV1>..<mV7>,<seq>,<us>". v1.0-v1.3 lines (without seq,us) are still understood by the app.
//   - BLE (Nordic UART service 6E400001...): NEW packet "v2", batched (4 samples per notification, 50 notifications/s):
//       characteristic 6E400005 (notify), little-endian:
//         byte 0   = 2     packet version
//         byte 1   = n     number of samples in this notification (normally 4)
//         byte 2   = 14    firmware version (major*10 + minor -> 1.4)
//         byte 3   = flags bit0 = IMU present (otherwise the 7 IMU values are 0)
//         n x 36-byte sample: uint32 seq, uint32 us (same meaning as above), 7 x uint16 mV, 7 x int16 IMU (ax,ay,az,gx,gy,gz,apk)
//       The v1.0-v1.3 characteristics (6E400003 / 6E400004, 50 Hz, 16 bytes) are no longer created: 400 notifications per second
//       would exceed what phones and browsers can take. The app still reads v1 from older firmware.
//       Needs an ATT MTU >= 151 (Chrome / Android negotiate 247+; the app reports a short packet if not). Use USB if in doubt.
// The app converts mV -> N with the per-channel calibration, so no calibration lives here.
//
// 200 Hz = one frame every 5000 us. The ADC burst is unchanged from v1.3 (8 reads averaged per channel). Whether it fits in
// 5 ms on your board is reported by the "#TIMING" line (adc_us_max, frame_us_max, late, over): check it in a 30 s test before the lab.
// If late/over keep counting up, lower OVERSAMPLE (e.g. 4) and re-flash; the app records the oversample value in the metadata.
//
// IMU: SparkFun BMI270 (Qwiic) at 0x68, SDA GPIO23, SCL GPIO22, 400 kHz. Accel +-16 g, gyro +-2000 deg/s, ODR 200 Hz.
// It is read at 200 Hz (about one reading per frame, the frame sends the mean of the readings since the last frame) plus the peak
// dynamic |a - g| (heel-strike shock). Gravity g is tracked in sensor axes: rotated with the gyro every reading and pulled
// towards the accelerometer while |a| is close to 1 g (foot still). The mean accel values still include gravity.
// Mounting on the insole: sensor +Y points down (ay = -1 g standing), +X points to the heel.
// Gyro zero offset (GYRO_BIAS, deg/s) is subtracted from every reading: mean of 3 x 10 s still measurements of this
// BMI270 (rounds 4-6: gx +0.034/+0.033/+0.024, gy -1.179/-1.173/-1.173, gz +0.179/+0.181/+0.180 dps; sd ~0.12 dps).
// Re-measure it if the IMU board is replaced.
//
// Board: Waveshare ESP32-C6-DEV-KIT-N8, Arduino "ESP32C6 Dev Module", core 3.3.x,
//        USB CDC On Boot: Enabled. Library: "SparkFun BMI270 Arduino Library" (Library Manager). BLE is part of the core.
// Channel -> insole site (heel to toe): ch1 S1, ch2 S4, ch3 S6, ch4 S11, ch5 S13, ch6 S10, ch7 S14.
//
// Commands (USB serial or BLE write):
//   ?    info line (IMU status, |a| at rest: should be ~1.00 g when the board lies still) + timing line
//   fL   set this board to LEFT  foot (saved in flash, board restarts, BLE name "Insole-L")
//   fR   set this board to RIGHT foot (saved in flash, board restarts, BLE name "Insole-R")
// Flash the same sketch on both boards, then send fL to one and fR to the other.

#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>
#include "SparkFun_BMI270_Arduino_Library.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#if defined(CONFIG_BLUEDROID_ENABLED)
#include <BLE2902.h>
#endif

#define FW_VERSION   "1.4"
#define FW_BYTE      14                                        // major * 10 + minor, sent in the BLE v2 header
#define NUS_SERVICE  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX       "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"   // app -> board (write)
#define NUS_FSR2     "6E400005-B5A3-F393-E0A9-E50E24DCCA9E"   // board -> app (notify, batched v2: FSR + IMU + seq + us)

const uint8_t  N_CH            = 7;
const uint8_t  FSR_PIN[N_CH]   = {0, 1, 2, 3, 4, 5, 6};   // S1..S7 = GPIO0..6 (ADC1)
const uint16_t SAMPLE_HZ       = 200;
const uint32_t FRAME_US        = 1000000UL / SAMPLE_HZ;    // 5000 us
const uint8_t  OVERSAMPLE      = 8;                        // ADC reads averaged per channel (unchanged since v1.0)
const uint32_t FOOT_PERIOD_MS  = 2000;
const uint8_t  IMU_SDA         = 23;
const uint8_t  IMU_SCL         = 22;
const uint16_t IMU_HZ          = 200;
const float    GYRO_BIAS[3]    = {0.030f, -1.175f, 0.180f};   // deg/s, subtracted from gx, gy, gz
const uint8_t  BATCH_N         = 4;                        // samples per BLE notification
const uint8_t  REC_BYTES       = 36;                       // seq 4 + us 4 + 7 x mV 14 + 7 x imu 14
const uint16_t BLE_MIN_MTU     = 4 + BATCH_N * REC_BYTES + 3;   // 151

// IMU frame accumulator (declared before the functions that use it)
struct ImuFrame {
  float ax, ay, az, gx, gy, gz;   // sums: g, deg/s
  float pk2;                      // peak |a - g|^2 (g^2)
  uint8_t n;
};

// One acquisition cycle, kept until the BLE batch is full
struct Sample {
  uint32_t seq, us;
  uint16_t mv[N_CH];
  int16_t imu[7];
};

// Timing diagnostics (cumulative since boot), printed in "#TIMING"
struct Diag {
  uint32_t frames, late, over, resync;       // late: frame started > 1 ms after its schedule; over: frame work took > 5 ms; resync: fell > 25 ms behind
  uint32_t adcMax, frameMax, lateMax;        // us
  uint64_t adcSum;
} diag = {};

Preferences prefs;
char foot = 'L';
BLEServer* server = nullptr;
BLECharacteristic* fsrChar = nullptr;
volatile bool bleConnected = false;
volatile bool restartAdv = false;
uint32_t seq = 0;                 // sample counter: +1 per acquisition cycle
uint16_t mv[N_CH];
String serialBuf;
Sample batch[BATCH_N];
uint8_t nBatch = 0;
uint16_t peerMtu = 0;             // 0 = not checked yet in this connection
uint32_t bleSince = 0;

BMI270 imu;
bool imuOk = false;
float imuRestG = 0;               // |a| averaged at boot (should be ~1.00 g)
float grav[3] = {0, -1, 0};       // gravity estimate in sensor axes (g); +Y points down on the insole
const float GRAV_K = 0.02f;       // pull towards the accelerometer per reading while still
ImuFrame acc = {};
int16_t imuOut[7] = {0, -1000, 0, 0, 0, 0, 0};     // last frame sent: mg, 0.1 deg/s, mg (insole flat: ay = -1 g)

void printTiming() {
  Serial.printf("#TIMING,hz=%u,oversample=%u,frames=%lu,adc_us_avg=%lu,adc_us_max=%lu,frame_us_max=%lu,late=%lu,late_us_max=%lu,over=%lu,resync=%lu,ble_mtu=%u\n",
                SAMPLE_HZ, OVERSAMPLE, (unsigned long)diag.frames,
                (unsigned long)(diag.frames ? diag.adcSum / diag.frames : 0), (unsigned long)diag.adcMax, (unsigned long)diag.frameMax,
                (unsigned long)diag.late, (unsigned long)diag.lateMax, (unsigned long)diag.over, (unsigned long)diag.resync, peerMtu);
}

void printInfo() {
  Serial.printf("#FOOT,%c\n", foot);
  Serial.printf("#INFO,FSR_Insole_App v%s,ch=%u,hz=%u,ble=Insole-%c,%s,imu=%s,|a|=%.2fg,gyro_bias=%.3f/%.3f/%.3f dps,oversample=%u,ts=micros_before_adc_burst,mac=%012llX,build=%s %s\n",
                FW_VERSION, N_CH, SAMPLE_HZ, foot, bleConnected ? "ble_connected" : "ble_idle",
                imuOk ? "BMI270" : "none", imuRestG, GYRO_BIAS[0], GYRO_BIAS[1], GYRO_BIAS[2], OVERSAMPLE,
                (unsigned long long)ESP.getEfuseMac(), __DATE__, __TIME__);
  printTiming();
}

void handleCommand(String c) {
  c.trim();
  if (c == "?") {
    printInfo();
  } else if (c == "fL" || c == "fR") {
    char f = c.charAt(1);
    prefs.putChar("foot", f);
    Serial.printf("#FOOT,%c (saved, restarting)\n", f);
    delay(200);
    ESP.restart();
  } else if (c.length()) {
    Serial.printf("# unknown command: %s (use ?, fL, fR)\n", c.c_str());
  }
}

class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer* s) { bleConnected = true; nBatch = 0; peerMtu = 0; bleSince = millis(); }
  void onDisconnect(BLEServer* s) { bleConnected = false; restartAdv = true; nBatch = 0; }
};

class RxCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) { handleCommand(String(c->getValue().c_str())); }
};

void setupBLE() {
  char name[12];
  snprintf(name, sizeof(name), "Insole-%c", foot);
  BLEDevice::init(name);
  BLEDevice::setMTU(247);                                   // batched packets are 148 bytes
  server = BLEDevice::createServer();
  server->setCallbacks(new ServerCB());
  BLEService* svc = server->createService(NUS_SERVICE);
  fsrChar = svc->createCharacteristic(NUS_FSR2, BLECharacteristic::PROPERTY_NOTIFY);
#if defined(CONFIG_BLUEDROID_ENABLED)
  fsrChar->addDescriptor(new BLE2902());
#endif
  BLECharacteristic* rx = svc->createCharacteristic(
      NUS_RX, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCB());
  svc->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
}

bool setupIMU() {
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(400000);
  if (imu.beginI2C(BMI2_I2C_PRIM_ADDR) != BMI2_OK) return false;   // 0x68

  bmi2_sens_config ac;
  ac.type = BMI2_ACCEL;
  ac.cfg.acc.odr = BMI2_ACC_ODR_200HZ;
  ac.cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
  ac.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
  ac.cfg.acc.range = BMI2_ACC_RANGE_16G;          // heel strike can exceed the default +-8 g when running

  bmi2_sens_config gc;
  gc.type = BMI2_GYRO;
  gc.cfg.gyr.odr = BMI2_GYR_ODR_200HZ;
  gc.cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
  gc.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
  gc.cfg.gyr.ois_range = BMI2_GYR_OIS_2000;
  gc.cfg.gyr.range = BMI2_GYR_RANGE_2000;
  gc.cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;

  if (imu.setConfig(ac) != BMI2_OK || imu.setConfig(gc) != BMI2_OK) return false;

  // scale self-check: |a| at rest should be ~1.00 g (if it reads ~0.5 or ~2, the range setting did not reach the conversion)
  float s = 0, sv[3] = {0, 0, 0};
  uint8_t n = 0;
  for (uint8_t i = 0; i < 40; i++) {
    delay(5);
    if (imu.getSensorData() != BMI2_OK) continue;
    s += sqrtf(imu.data.accelX * imu.data.accelX + imu.data.accelY * imu.data.accelY + imu.data.accelZ * imu.data.accelZ);
    sv[0] += imu.data.accelX; sv[1] += imu.data.accelY; sv[2] += imu.data.accelZ;
    n++;
  }
  imuRestG = n ? s / n : 0;
  float m = sqrtf(sv[0] * sv[0] + sv[1] * sv[1] + sv[2] * sv[2]);
  if (m > 0.5f) for (uint8_t k = 0; k < 3; k++) grav[k] = sv[k] / m;   // start the gravity estimate from the boot pose
  return n > 0;
}

// Read the IMU at 200 Hz, between FSR frames
void pollIMU() {
  static uint32_t next = micros();
  if (!imuOk || (int32_t)(micros() - next) < 0) return;
  next += 1000000UL / IMU_HZ;
  if ((int32_t)(micros() - next) > 20000) next = micros();   // fell behind: resync
  if (imu.getSensorData() != BMI2_OK) return;
  static uint32_t lastUs = 0;
  uint32_t nowUs = micros();
  float dt = lastUs ? (nowUs - lastUs) * 1e-6f : 1.0f / IMU_HZ;
  if (dt > 0.05f) dt = 0.05f;
  lastUs = nowUs;
  float ax = imu.data.accelX, ay = imu.data.accelY, az = imu.data.accelZ;
  float gx = imu.data.gyroX - GYRO_BIAS[0], gy = imu.data.gyroY - GYRO_BIAS[1], gz = imu.data.gyroZ - GYRO_BIAS[2];
  acc.ax += ax; acc.ay += ay; acc.az += az;
  acc.gx += gx; acc.gy += gy; acc.gz += gz;

  // gravity in sensor axes: a world-fixed vector seen from the rotating sensor, d(grav)/dt = -w x grav
  const float D2R = 0.01745329f;
  float wx = gx * D2R, wy = gy * D2R, wz = gz * D2R;
  float cx = wy * grav[2] - wz * grav[1], cy = wz * grav[0] - wx * grav[2], cz = wx * grav[1] - wy * grav[0];
  grav[0] -= cx * dt; grav[1] -= cy * dt; grav[2] -= cz * dt;
  float an = sqrtf(ax * ax + ay * ay + az * az);
  if (fabsf(an - 1.0f) < 0.1f) {         // quasi-static: correct the drift towards the measured gravity
    grav[0] += GRAV_K * (ax - grav[0]); grav[1] += GRAV_K * (ay - grav[1]); grav[2] += GRAV_K * (az - grav[2]);
  }
  float gn = sqrtf(grav[0] * grav[0] + grav[1] * grav[1] + grav[2] * grav[2]);
  if (gn > 0.1f) { grav[0] /= gn; grav[1] /= gn; grav[2] /= gn; }

  float dx = ax - grav[0], dy = ay - grav[1], dz = az - grav[2];
  float m2 = dx * dx + dy * dy + dz * dz;   // dynamic acceleration, gravity removed
  if (m2 > acc.pk2) acc.pk2 = m2;
  acc.n++;
}

int16_t clamp16(float v) {
  if (v > 32767.0f) return 32767;
  if (v < -32768.0f) return -32768;
  return (int16_t)lroundf(v);
}

// Close the IMU frame: mean of the readings since the last FSR frame (keeps the last values if none arrived)
void closeImuFrame() {
  if (acc.n) {
    float k = 1.0f / acc.n;
    imuOut[0] = clamp16(acc.ax * k * 1000.0f);
    imuOut[1] = clamp16(acc.ay * k * 1000.0f);
    imuOut[2] = clamp16(acc.az * k * 1000.0f);
    imuOut[3] = clamp16(acc.gx * k * 10.0f);
    imuOut[4] = clamp16(acc.gy * k * 10.0f);
    imuOut[5] = clamp16(acc.gz * k * 10.0f);
    imuOut[6] = clamp16(sqrtf(acc.pk2) * 1000.0f);
  }
  acc = {};
}

void readChannels() {
  for (uint8_t i = 0; i < N_CH; i++) {
    uint32_t sum = 0;
    for (uint8_t k = 0; k < OVERSAMPLE; k++) sum += analogReadMilliVolts(FSR_PIN[i]);
    mv[i] = (uint16_t)((sum + OVERSAMPLE / 2) / OVERSAMPLE);
  }
}

static inline void put32(uint8_t* p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }
static inline void put16(uint8_t* p, uint16_t v) { p[0] = v; p[1] = v >> 8; }

// BLE v2: send the collected samples in one notification
void sendBatch() {
  uint8_t pkt[4 + BATCH_N * REC_BYTES];
  pkt[0] = 2; pkt[1] = nBatch; pkt[2] = FW_BYTE; pkt[3] = imuOk ? 1 : 0;
  uint8_t* p = pkt + 4;
  for (uint8_t s = 0; s < nBatch; s++, p += REC_BYTES) {
    put32(p, batch[s].seq); put32(p + 4, batch[s].us);
    for (uint8_t i = 0; i < N_CH; i++) put16(p + 8 + 2 * i, batch[s].mv[i]);
    for (uint8_t i = 0; i < 7; i++) put16(p + 22 + 2 * i, (uint16_t)batch[s].imu[i]);
  }
  fsrChar->setValue(pkt, 4 + nBatch * REC_BYTES);
  fsrChar->notify();
  nBatch = 0;
}

void setup() {
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxBufferSize(8192);        // native USB: ride out short host stalls without losing lines (must be set before begin)
#else
  Serial.setTxBufferSize(2048);
#endif
  Serial.begin(921600);                // UART/CH343 port: 200 Hz lines need more than 115200; native USB ignores the baud rate
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);            // native USB: never block when no PC is listening
#endif
  analogReadResolution(12);
  for (uint8_t i = 0; i < N_CH; i++) analogSetPinAttenuation(FSR_PIN[i], ADC_11db);

  prefs.begin("insole", false);
  foot = prefs.getChar("foot", 'L');
  if (foot != 'L' && foot != 'R') foot = 'L';

  imuOk = setupIMU();
  setupBLE();
  delay(300);
  printInfo();
}

void loop() {
  static uint32_t nextUs = micros();
  static uint32_t lastFoot = 0;

  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { handleCommand(serialBuf); serialBuf = ""; }
    else if (serialBuf.length() < 32) serialBuf += ch;
  }

  if (restartAdv) { restartAdv = false; delay(50); BLEDevice::startAdvertising(); }

  pollIMU();

  uint32_t startUs = micros();
  int32_t lateUs = (int32_t)(startUs - nextUs);
  if (lateUs < 0) return;
  nextUs += FRAME_US;
  if ((int32_t)(micros() - nextUs) > 5 * (int32_t)FRAME_US) { nextUs = micros(); diag.resync++; }  // fell behind: resync (seq keeps counting, us shows the gap)
  if (lateUs > 1000) { diag.late++; if ((uint32_t)lateUs > diag.lateMax) diag.lateMax = lateUs; }

  uint32_t t0 = micros();              // acquisition timestamp: just before the ADC burst
  readChannels();
  uint32_t adcUs = micros() - t0;
  if (imuOk) closeImuFrame();
  uint32_t ms = millis();

  if (imuOk) {
    Serial.printf("D,%lu,%u,%u,%u,%u,%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%lu,%lu\n", (unsigned long)ms,
                  mv[0], mv[1], mv[2], mv[3], mv[4], mv[5], mv[6],
                  imuOut[0], imuOut[1], imuOut[2], imuOut[3], imuOut[4], imuOut[5], imuOut[6],
                  (unsigned long)seq, (unsigned long)t0);
  } else {
    Serial.printf("D,%lu,%u,%u,%u,%u,%u,%u,%u,%lu,%lu\n", (unsigned long)ms,
                  mv[0], mv[1], mv[2], mv[3], mv[4], mv[5], mv[6], (unsigned long)seq, (unsigned long)t0);
  }
  if (ms - lastFoot >= FOOT_PERIOD_MS) { lastFoot = ms; Serial.printf("#FOOT,%c\n", foot); printTiming(); }

  if (bleConnected && fsrChar) {
    Sample& s = batch[nBatch++];
    s.seq = seq; s.us = t0;
    for (uint8_t i = 0; i < N_CH; i++) s.mv[i] = mv[i];
    for (uint8_t i = 0; i < 7; i++) s.imu[i] = imuOk ? imuOut[i] : 0;
#if defined(CONFIG_BLUEDROID_ENABLED)
    if (!peerMtu && millis() - bleSince > 1500) {            // MTU is negotiated just after connecting
      peerMtu = server->getPeerMTU(server->getConnId());
      if (peerMtu && peerMtu < BLE_MIN_MTU) Serial.printf("#WARN,BLE MTU %u is below %u: 200 Hz packets will be cut, use USB\n", peerMtu, BLE_MIN_MTU);
    }
#endif
    if (nBatch >= BATCH_N) sendBatch();
  }
  seq++;

  uint32_t used = micros() - t0;       // acquisition + output of this frame
  diag.frames++; diag.adcSum += adcUs;
  if (adcUs > diag.adcMax) diag.adcMax = adcUs;
  if (used > diag.frameMax) diag.frameMax = used;
  if (used > FRAME_US) diag.over++;
}
