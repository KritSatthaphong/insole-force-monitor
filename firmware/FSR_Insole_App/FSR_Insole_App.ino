// FSR_Insole_App.ino  v1.0  (2026-10-27)
// Smart Insole -> Insole Force Monitor web app.
// Streams raw mV of 7 FlexiForce A401 channels at 50 Hz over BOTH:
//   - USB serial (115200):  "D,<ms>,<mV1>,...,<mV7>"   + "#FOOT,L" every 2 s
//   - BLE (Nordic UART service): 16-byte notify = uint16 seq + 7 x uint16 mV (little-endian)
// The app converts mV -> N with the per-channel calibration (RF 39k), so no cal lives here.
//
// Board: Waveshare ESP32-C6-DEV-KIT-N8, Arduino "ESP32C6 Dev Module", core 3.3.x,
//        USB CDC On Boot: Enabled. No extra libraries needed (BLE is part of the core).
// Channel -> insole site (heel to toe): ch1 S1, ch2 S4, ch3 S6, ch4 S11, ch5 S13, ch6 S10, ch7 S14.
//
// Commands (USB serial or BLE write):
//   ?    info line
//   fL   set this board to LEFT  foot (saved in flash, board restarts, BLE name "Insole-L")
//   fR   set this board to RIGHT foot (saved in flash, board restarts, BLE name "Insole-R")
// Flash the same sketch on both boards, then send fL to one and fR to the other.

#include <Arduino.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#if defined(CONFIG_BLUEDROID_ENABLED)
#include <BLE2902.h>
#endif

#define FW_VERSION   "1.0"
#define NUS_SERVICE  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX       "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"   // app -> board (write)
#define NUS_TX       "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"   // board -> app (notify)

const uint8_t  N_CH            = 7;
const uint8_t  FSR_PIN[N_CH]   = {0, 1, 2, 3, 4, 5, 6};   // S1..S7 = GPIO0..6 (ADC1)
const uint16_t SAMPLE_HZ       = 50;
const uint8_t  OVERSAMPLE      = 8;                        // ADC reads averaged per channel
const uint32_t FOOT_PERIOD_MS  = 2000;

Preferences prefs;
char foot = 'L';
BLEServer* server = nullptr;
BLECharacteristic* txChar = nullptr;
volatile bool bleConnected = false;
volatile bool restartAdv = false;
uint16_t seq = 0;
uint16_t mv[N_CH];
String serialBuf;

void printInfo() {
  Serial.printf("#FOOT,%c\n", foot);
  Serial.printf("#INFO,FSR_Insole_App v%s,ch=%u,hz=%u,ble=Insole-%c,%s\n",
                FW_VERSION, N_CH, SAMPLE_HZ, foot, bleConnected ? "ble_connected" : "ble_idle");
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
#if defined(CONFIG_BLUEDROID_ENABLED)
  txChar->addDescriptor(new BLE2902());
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

  if ((int32_t)(micros() - nextUs) < 0) return;
  nextUs += 1000000UL / SAMPLE_HZ;
  if ((int32_t)(micros() - nextUs) > 5 * (int32_t)(1000000UL / SAMPLE_HZ)) nextUs = micros();  // fell behind: resync

  readChannels();
  uint32_t ms = millis();

  Serial.printf("D,%lu,%u,%u,%u,%u,%u,%u,%u\n", (unsigned long)ms,
                mv[0], mv[1], mv[2], mv[3], mv[4], mv[5], mv[6]);
  if (ms - lastFoot >= FOOT_PERIOD_MS) { lastFoot = ms; Serial.printf("#FOOT,%c\n", foot); }

  if (bleConnected && txChar) {
    uint8_t pkt[2 + 2 * N_CH];
    pkt[0] = seq & 0xFF; pkt[1] = seq >> 8;
    for (uint8_t i = 0; i < N_CH; i++) { pkt[2 + 2 * i] = mv[i] & 0xFF; pkt[3 + 2 * i] = mv[i] >> 8; }
    txChar->setValue(pkt, sizeof(pkt));
    txChar->notify();
  }
  seq++;
}
