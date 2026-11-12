// Host stub of the Arduino-ESP32 API, enough for FSR_UTM_Cal.ino. Simulated clock (SIM_SPEED x real time).
#pragma once
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cctype>
#include <string>
#include <deque>
#include <mutex>
#include <thread>
#include <chrono>
#include <condition_variable>
#include <vector>
#include <strings.h>
using std::min; using std::max;
double sim_now_s();           // simulated seconds since start
void sim_sleep_s(double s);
int sim_serial_read();        // -1 if none
int sim_serial_avail();
void sim_serial_write(const uint8_t*, size_t);
uint16_t sim_adc_mv(uint8_t pin);
inline unsigned long millis() { return (unsigned long)(sim_now_s() * 1000.0); }
inline void delay(uint32_t ms) { sim_sleep_s(ms / 1000.0); }
inline bool isDigit(char c) { return c >= '0' && c <= '9'; }
inline void analogReadResolution(int) {}
#define ADC_11db 3
inline void analogSetPinAttenuation(int, int) {}
inline uint32_t analogReadMilliVolts(uint8_t pin) { return sim_adc_mv(pin); }
#define OUTPUT 1
#define HIGH 1
#define LOW 0
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}

class String {
 public:
  std::string s;
  String() {}
  String(const char* c) : s(c ? c : "") {}
  String(const std::string& x) : s(x) {}
  size_t length() const { return s.size(); }
  char operator[](size_t i) const { return i < s.size() ? s[i] : 0; }
  String& operator+=(char c) { s += c; return *this; }
  bool operator==(const char* c) const { return s == c; }
  bool operator!=(const char* c) const { return s != c; }
  bool operator==(const String& o) const { return s == o.s; }
  const char* c_str() const { return s.c_str(); }
  void trim() { size_t a = s.find_first_not_of(" \t\r\n"); if (a == std::string::npos) { s.clear(); return; } size_t b = s.find_last_not_of(" \t\r\n"); s = s.substr(a, b - a + 1); }
  void toLowerCase() { for (auto& c : s) c = tolower(c); }
  int indexOf(char c) const { size_t p = s.find(c); return p == std::string::npos ? -1 : (int)p; }
  String substring(size_t a) const { return a >= s.size() ? String("") : String(s.substr(a)); }
  String substring(size_t a, size_t b) const { if (a >= s.size()) return String(""); return String(s.substr(a, b - a)); }
  void replace(const char* f, const char* t) { std::string F(f), T(t); size_t p = 0; while ((p = s.find(F, p)) != std::string::npos) { s.replace(p, F.size(), T); p += T.size(); } }
  long toInt() const { return atol(s.c_str()); }
  float toFloat() const { return atof(s.c_str()); }
};

struct SerialStub {
  void setTxBufferSize(size_t) {}
  void begin(uint32_t) {}
  explicit operator bool() const { return true; }
  int available() { return sim_serial_avail(); }
  int read() { return sim_serial_read(); }
  size_t write(const uint8_t* b, size_t n) { sim_serial_write(b, n); return n; }
  void println(const char* c) { sim_serial_write((const uint8_t*)c, strlen(c)); sim_serial_write((const uint8_t*)"\n", 1); }
};
extern SerialStub Serial;

// ---- FreeRTOS ----
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(x) (x)
#define pdTRUE 1
#define portMAX_DELAY 0xFFFFFFFF
inline TickType_t xTaskGetTickCount() { return (TickType_t)(sim_now_s() * 1000.0); }
inline void vTaskDelayUntil(TickType_t* last, TickType_t inc) { *last += inc; double t = *last / 1000.0, n = sim_now_s(); if (t > n) sim_sleep_s(t - n); }
struct QueueS { std::mutex m; std::deque<std::vector<uint8_t>> d; size_t len, item; };
typedef QueueS* QueueHandle_t;
inline QueueHandle_t xQueueCreate(size_t len, size_t item) { auto q = new QueueS; q->len = len; q->item = item; return q; }
inline int xQueueSend(QueueHandle_t q, const void* p, TickType_t) { std::lock_guard<std::mutex> l(q->m); if (q->d.size() >= q->len) return 0; q->d.emplace_back((const uint8_t*)p, (const uint8_t*)p + q->item); return 1; }
inline int xQueueReceive(QueueHandle_t q, void* p, TickType_t wait) {
  double t0 = sim_now_s();
  for (;;) { { std::lock_guard<std::mutex> l(q->m); if (!q->d.empty()) { memcpy(p, q->d.front().data(), q->item); q->d.pop_front(); return 1; } }
    if (sim_now_s() - t0 >= wait / 1000.0) return 0; sim_sleep_s(0.002); }
}
inline void xQueueReset(QueueHandle_t q) { std::lock_guard<std::mutex> l(q->m); q->d.clear(); }
inline size_t uxQueueMessagesWaiting(QueueHandle_t q) { std::lock_guard<std::mutex> l(q->m); return q->d.size(); }
typedef std::recursive_mutex* SemaphoreHandle_t;
inline SemaphoreHandle_t xSemaphoreCreateMutex() { return new std::recursive_mutex; }
inline int xSemaphoreTake(SemaphoreHandle_t s, TickType_t) { s->lock(); return 1; }
inline void xSemaphoreGive(SemaphoreHandle_t s) { s->unlock(); }
inline void xTaskCreate(void (*f)(void*), const char*, int, void* a, int, void*) { std::thread(f, a).detach(); }
typedef std::mutex portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED {}
#define portENTER_CRITICAL(m) (m)->lock()
#define portEXIT_CRITICAL(m) (m)->unlock()
