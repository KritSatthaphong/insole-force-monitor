// FSR_Insole_App.ino  v1.2  (2026-10-29)
// Smart Insole -> Insole Force Monitor web app.
// Streams raw mV of 7 FlexiForce A401 channels + BMI270 IMU at 50 Hz over BOTH:
//   - USB serial (115200):  "D,<ms>,<mV1>,...,<mV7>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<apk>"   + "#FOOT,L" every 2 s
//       ax..az = mean accel of the frame (mg), gx..gz = mean gyro (0.1 deg/s), apk = peak |a| in the frame (mg)
//       (no IMU found -> the line stops after mV7, like v1.0)
//   - BLE (Nordic UART service):
//       TX  6E400003: 16-byte notify = uint16 seq + 7 x uint16 mV (little-endian), unchanged from v1.0
//       IMU 6E400004: 16-byte notify = uint16 seq + 7 x int16 (ax, ay, az, gx, gy, gz, apk; units as above),
//                     sent just before the TX packet with the same seq
// The app converts mV -> N with the per-channel calibration (RF 39k), so no cal lives here.
//
// IMU: SparkFun BMI270 (Qwiic) at 0x68, SDA GPIO23, SCL GPIO22, 400 kHz. Accel +-16 g, gyro +-2000 deg/s, ODR 200 Hz.
// It is read at 200 Hz; each 20 ms frame sends the mean of its ~4 readings (for orientation / stride integration)
// plus the peak |a| (the heel-strike shock, which a 50 Hz average would smear).
// Gyro zero offset (GYRO_BIAS, deg/s) is subtracted from every reading: mean of 3 x 10 s still measurements of this
// BMI270 (rounds 4-6: gx +0.034/+0.033/+0.024, gy -1.179/-1.173/-1.173, gz +0.179/+0.181/+0.180 dps; sd ~0.12 dps).
// Re-measure it if the IMU board is replaced.
//
// Board: Waveshare ESP32-C6-DEV-KIT-N8, Arduino "ESP32C6 Dev Module", core 3.3.x,
//        USB CDC On Boot: Enabled. Library: "SparkFun BMI270 Arduino Library" (Library Manager). BLE is part of the core.
// Channel -> insole site (heel to toe): ch1 S1, ch2 S4, ch3 S6, ch4 S11, ch5 S13, ch6 S10, ch7 S14.
//
// Commands (USB serial or BLE write):
//   ?    info line (includes IMU status and |a| at rest: should be ~1.00 g when the board lies still)
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

#define FW_VERSION   "1.2"
#define NUS_SERVICE  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX       "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"   // app -> board (write)
#define NUS_TX       "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"   // board -> app (notify, FSR)
#define NUS_IMU      "6E400004-B5A3-F393-E0A9-E50E24DCCA9E"   // board -> app (notify, IMU)

const uint8_t  N_CH            = 7;
const uint8_t  FSR_PIN[N_CH]   = {0, 1, 2, 3, 4, 5, 6};   // S1..S7 = GPIO0..6 (ADC1)
const uint16_t SAMPLE_HZ       = 50;
const uint8_t  OVERSAMPLE      = 8;                        // ADC reads averaged per channel
const uint32_t FOOT_PERIOD_MS  = 2000;
const uint8_t  IMU_SDA         = 23;
const uint8_t  IMU_SCL         = 22;
const uint16_t IMU_HZ          = 200;
const float    GYRO_BIAS[3]    = {0.030f, -1.175f, 0.180f};   // deg/s, subtracted from gx, gy, gz

// IMU frame accumulator (declared before the functions that use it)
struct ImuFrame {
  float ax, ay, az, gx, gy, gz;   // sums: g, deg/s
  float pk2;                      // peak |a|^2 (g^2)
  uint8_t n;
};

Preferences prefs;
char foot = 'L';
BLEServer* server = nullptr;
BLECharacteristic* txChar = nullptr;
BLECharacteristic* imuChar = nullptr;
volatile bool bleConnected = false;
volatile bool restartAdv = false;
uint16_t seq = 0;
uint16_t mv[N_CH];
String serialBuf;

BMI270 imu;
bool imuOk = false;
float imuRestG = 0;               // |a| averaged at boot (should be ~1.00 g)
ImuFrame acc = {};
int16_t imuOut[7] = {0, 0, 1000, 0, 0, 0, 1000};   // last frame sent: mg, 0.1 deg/s, mg

void printInfo() {
  Serial.printf("#FOOT,%c\n", foot);
  Serial.printf("#INFO,FSR_Insole_App v%s,ch=%u,hz=%u,ble=Insole-%c,%s,imu=%s,|a|=%.2fg,gyro_bias=%.3f/%.3f/%.3f dps\n",
                FW_VERSION, N_CH, SAMPLE_HZ, foot, bleConnected ? "ble_connected" : "ble_idle",
                imuOk ? "BMI270" : "none", imuRestG, GYRO_BIAS[0], GYRO_BIAS[1], GYRO_BIAS[2]);
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
  void onConnect(BLEServer* s) { bleConnected = true; }
  void onDisconnect(BLEServer* s) { bleConnected = false; restartAdv = true; }
};

class RxCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) { handleCommand(String(c->getValue().c_str())); }
};

void setupBLE() {
  char name[12];
  snprintf(name, sizeof(name), "Insole-%c", foot);
  BLEDevice::init(name);
  server = BLEDevice::createServer();
  server->setCallbacks(new ServerCB());
  BLEService* svc = server->createService(NUS_SERVICE);
  txChar = svc->createCharacteristic(NUS_TX, BLECharacteristic::PROPERTY_NOTIFY);
  imuChar = svc->createCharacteristic(NUS_IMU, BLECharacteristic::PROPERTY_NOTIFY);
#if defined(CONFIG_BLUEDROID_ENABLED)
  txChar->addDescriptor(new BLE2902());
  imuChar->addDescriptor(new BLE2902());
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
  float s = 0;
  uint8_t n = 0;
  for (uint8_t i = 0; i < 40; i++) {
    delay(5);
    if (imu.getSensorData() != BMI2_OK) continue;
    s += sqrtf(imu.data.accelX * imu.data.accelX + imu.data.accelY * imu.data.accelY + imu.data.accelZ * imu.data.accelZ);
    n++;
  }
  imuRestG = n ? s / n : 0;
  return n > 0;
}

// Read the IMU at 200 Hz, between FSR frames
void pollIMU() {
  static uint32_t next = micros();
  if (!imuOk || (int32_t)(micros() - next) < 0) return;
  next += 1000000UL / IMU_HZ;
  if ((int32_t)(micros() - next) > 20000) next = micros();   // fell behind: resync
  if (imu.getSensorData() != BMI2_OK) return;
  float ax = imu.data.accelX, ay = imu.data.accelY, az = imu.data.accelZ;
  acc.ax += ax; acc.ay += ay; acc.az += az;
  acc.gx += imu.data.gyroX - GYRO_BIAS[0];
  acc.gy += imu.data.gyroY - GYRO_BIAS[1];
  acc.gz += imu.data.gyroZ - GYRO_BIAS[2];
  float m2 = ax * ax + ay * ay + az * az;
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

void setup() {
  Serial.begin(115200);
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

  if ((int32_t)(micros() - nextUs) < 0) return;
  nextUs += 1000000UL / SAMPLE_HZ;
  if ((int32_t)(micros() - nextUs) > 5 * (int32_t)(1000000UL / SAMPLE_HZ)) nextUs = micros();  // fell behind: resync

  readChannels();
  if (imuOk) closeImuFrame();
  uint32_t ms = millis();

  if (imuOk) {
    Serial.printf("D,%lu,%u,%u,%u,%u,%u,%u,%u,%d,%d,%d,%d,%d,%d,%d\n", (unsigned long)ms,
                  mv[0], mv[1], mv[2], mv[3], mv[4], mv[5], mv[6],
                  imuOut[0], imuOut[1], imuOut[2], imuOut[3], imuOut[4], imuOut[5], imuOut[6]);
  } else {
    Serial.printf("D,%lu,%u,%u,%u,%u,%u,%u,%u\n", (unsigned long)ms,
                  mv[0], mv[1], mv[2], mv[3], mv[4], mv[5], mv[6]);
  }
  if (ms - lastFoot >= FOOT_PERIOD_MS) { lastFoot = ms; Serial.printf("#FOOT,%c\n", foot); }

  if (bleConnected && txChar) {
    if (imuOk && imuChar) {            // IMU first, so the app has it when the FSR packet with the same seq arrives
      uint8_t ip[2 + 2 * 7];
      ip[0] = seq & 0xFF; ip[1] = seq >> 8;
      for (uint8_t i = 0; i < 7; i++) {
        uint16_t u = (uint16_t)imuOut[i];
        ip[2 + 2 * i] = u & 0xFF; ip[3 + 2 * i] = u >> 8;
      }
      imuChar->setValue(ip, sizeof(ip));
      imuChar->notify();
    }
    uint8_t pkt[2 + 2 * N_CH];
    pkt[0] = seq & 0xFF; pkt[1] = seq >> 8;
    for (uint8_t i = 0; i < N_CH; i++) { pkt[2 + 2 * i] = mv[i] & 0xFF; pkt[3 + 2 * i] = mv[i] >> 8; }
    txChar->setValue(pkt, sizeof(pkt));
    txChar->notify();
  }
  seq++;
}
