// FSR_UTM_Hold.ino v1.0 - SIMPLE FlexiForce A401 calibration on the UTM: you signal each hold, you type the force.
// Board: ESP32C6 Dev Module, Arduino-ESP32 3.3.x, USB CDC On Boot: Enabled
// Works in the Arduino Serial Monitor (line ending "Newline") or with utm_pc_logger.py (saves files).
// No raw stream, no Shimadzu export, no time syncing: one CAL row per hold, with the force you type.
//
// ONE SHIMADZU RUN (2 holds, e.g. UTM method 100 N hold 30 s -> 200 N hold 30 s):
//   run          next run of the plan (or run UP03 / run DN05 / run FREE). Puck clear: 2 s zero point (auto)
//   (start the UTM)
//   h            type h + Enter when the UTM display says the 1st hold has started
//                -> the board measures until <dwell - 3 s>, mean of the last 2 s, drift since 3-5 s
//                -> prompt: type the force the UTM shows NOW (still inside the hold), e.g. 100.4
//   h            same for the 2nd hold
//   (repeat run ... for the next Shimadzu run; save when the channel is finished)
//
// Commands (not case sensitive):
//   z         zero all channels (all unloaded)          s<n>   channel 1..7
//   t<s>      UTM hold time (dwell) in s, e.g. t30, t10 c<n>   repeat / cycle number (c1, c2, c3)
//   p<N>      peak P for the plan (default 1000)         plan   list the runs with their 2 targets
//   run [ID]  start a Shimadzu run (zero point first)    h / h<N>  measure a hold (h<N> = other target)
//   x         cancel a hold that is running              n<text>   note
//   list      print all points of this channel           save   close the channel (prints the table)
//   l         live view on/off                           ?      help
// At the force prompt: number = UTM force in N, Enter or . = the target, a = redo the hold, x = discard.
// Settings (t, p, step, c, channel) are kept in flash.
//
// CAL row (also read by insole-app Calibration check and tools/check_cal.py):
//   CAL,ms,ch,cycle,run,run_id,hold,dir,set_N,utm_N,mean_mV,sd_mV,min_mV,max_mV,early_mV,drift_mV,trend_mV,
//   base_mV,rs_kOhm,g_uS,xtalk_mV,flags,redo,win_s
//   dir: Z zero point, U loading (1st hold of a run, or higher than the 1st), D unloading (lower than the 1st)
//   flags: bit1 ADC clipped (> 3250 mV), bit2 still moving in the last 2 s (hold ended early?)

#include <Arduino.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>
#include <Preferences.h>
#include "esp_timer.h"

#define FW_VERSION "FSR_UTM_Hold v1.0 simple hold calibration (2026-11-11)"

// ---------------- Hardware ----------------
const uint8_t NCH = 7;
const uint8_t FSR_PINS[NCH] = {0, 1, 2, 3, 4, 5, 6};                 // ch1..ch7 = GPIO0..6
const char *const SITE_OF_CH[NCH] = {"S1", "S4", "S6", "S11", "S13", "S10", "S14"};
// RF per channel in ohm: MUST equal the fitted (measured) resistor of each channel
const float RF_OHM[NCH] = {39000, 39000, 39000, 39000, 39000, 39000, 39000};
// provisional display only: loading curves of insole-app DEFAULT_CAL
const float PROV_A[NCH] = {3.58966f, 3.98233f, 3.22000f, 2.72739f, 4.28822f, 2.80304f, 3.41093f};
const float PROV_B[NCH] = {1.02654f, 1.01871f, 1.00616f, 1.02718f, 1.01217f, 1.03127f, 0.99299f};
const uint16_t ADC_TOP_MV = 3250;
const uint16_t NEAR_MV = 3000;
const float DEAD_MV = 3.0f;

// ---------------- Measurement ----------------
const uint32_t PERIOD_MS = 5;          // 200 Hz
const uint8_t OVERSAMPLE = 4;
const float MARGIN_S = 3.0f;           // board stops measuring this long before the UTM hold ends
const float AVG_S = 2.0f;              // reported value = mean of the last 2 s
const float ZERO_S = 2.0f;             // automatic zero point at run start
const float FULL_SCALE_N = 1000.0f;
const uint16_t MAX_PTS = 160;

// =====================================================================
struct Stat { double s[NCH], ss[NCH]; uint32_t n; uint16_t mn, mx; uint8_t flags; };

struct Point {
  uint32_t ms;
  uint8_t ch, cycle, hold, flags, redo;
  uint16_t run;
  char runId[8], dir;
  float setN, utmN, mean, sd, early, drift, trend, base, g, xtalk, win;
  uint16_t mn, mx;
};

struct Settings { uint32_t magic; float dwellS, pN, stepN; uint8_t cycle, selCh; };

struct PlanRun { char id[6]; char kind; float t1, t2; };

// =====================================================================
Preferences prefs;
Settings S;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
volatile uint16_t latestMv[NCH];
volatile uint32_t accWant = 0, accN = 0;
volatile uint8_t accCh = 0;
Stat acc;

float baseMV[NCH];
bool zeroed[NCH] = {false};
char note[100] = "";

PlanRun plan[32];
uint8_t nPlan = 0;
int8_t nextPlan = 0;

bool sessOn = false;          // channel session open (start .. save)
uint8_t sessCh = 0;
Point pts[MAX_PTS];
uint16_t nPts = 0;
uint16_t runNo = 0;
int8_t runPlan = -1;          // plan index of the current run, -1 = free
char runId[8] = "";
uint8_t holdNo = 0;           // holds measured in the current run
float hold1N = -1;
bool liveOn = false;
String rx;
uint32_t lastLive = 0;

// =====================================================================
void out(const char *fmt, ...) {
  char buf[320];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n < 0) return;
  if ((size_t)n >= sizeof(buf)) { n = sizeof(buf) - 1; buf[n - 1] = '\n'; }
  Serial.write((const uint8_t *)buf, n);
}

// 200 Hz sampler: latest values + accumulator for measurements
void samplerTask(void *) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(PERIOD_MS));
    uint16_t mv[NCH];
    for (uint8_t ch = 0; ch < NCH; ch++) {
      uint32_t sum = 0;
      for (uint8_t k = 0; k < OVERSAMPLE; k++) sum += analogReadMilliVolts(FSR_PINS[ch]);
      mv[ch] = (sum + OVERSAMPLE / 2) / OVERSAMPLE;
    }
    if (accN < accWant) {
      if (accN == 0) { acc.mn = 0xFFFF; acc.mx = 0; acc.flags = 0; }
      for (uint8_t ch = 0; ch < NCH; ch++) { double v = mv[ch]; acc.s[ch] += v; acc.ss[ch] += v * v; }
      uint16_t v = mv[accCh];
      if (v < acc.mn) acc.mn = v;
      if (v > acc.mx) acc.mx = v;
      if (v > ADC_TOP_MV) acc.flags |= 0x02;
      accN++;
      acc.n = accN;
    }
    portENTER_CRITICAL(&mux);
    for (uint8_t ch = 0; ch < NCH; ch++) latestMv[ch] = mv[ch];
    portEXIT_CRITICAL(&mux);
  }
}

uint16_t latest(uint8_t ch) {
  portENTER_CRITICAL(&mux);
  uint16_t v = latestMv[ch];
  portEXIT_CRITICAL(&mux);
  return v;
}

// true if 'x' arrived (cancel)
bool cancelPending() {
  while (Serial.available()) { char c = Serial.read(); if (c == 'x' || c == 'X') return true; }
  return false;
}

// collect n frames into st; 0 ok, 1 cancelled, 2 timeout
int collect(uint32_t n, uint8_t ch, Stat &st, bool cancellable) {
  accWant = 0;
  memset(&acc, 0, sizeof(acc));
  accCh = ch;
  accN = 0;
  accWant = n;
  uint32_t t0 = millis();
  while (accN < n) {
    if (millis() - t0 > n * PERIOD_MS + 1000) { accWant = 0; return 2; }
    if (cancellable && cancelPending()) { accWant = 0; return 1; }
    delay(10);
  }
  accWant = 0;
  st = acc;
  return 0;
}

void merge(Stat &a, const Stat &b) {
  for (uint8_t i = 0; i < NCH; i++) { a.s[i] += b.s[i]; a.ss[i] += b.ss[i]; }
  a.n += b.n;
  if (b.mn < a.mn) a.mn = b.mn;
  if (b.mx > a.mx) a.mx = b.mx;
  a.flags |= b.flags;
}

float meanOf(const Stat &st, uint8_t ch) { return st.n ? st.s[ch] / st.n : 0; }
float sdOf(const Stat &st, uint8_t ch) {
  if (!st.n) return 0;
  double m = st.s[ch] / st.n, v = st.ss[ch] / st.n - m * m;
  return v > 0 ? sqrt(v) : 0;
}

float condUS(uint8_t ch, float mv) {
  if (mv - baseMV[ch] < DEAD_MV) return 0.0f;
  return (mv / baseMV[ch] - 1.0f) / RF_OHM[ch] * 1e6f;
}
float provN(uint8_t ch, float g) { return g > 0 ? PROV_A[ch] * powf(g, PROV_B[ch]) : 0.0f; }

// waits until millis() reaches t (cancellable); prints a countdown every 5 s
bool waitUntil(uint32_t t, uint32_t t0, uint8_t ch) {
  uint32_t lastP = millis();
  while ((int32_t)(millis() - t) < 0) {
    if (cancelPending()) return false;
    if (millis() - lastP >= 5000) {
      lastP = millis();
      uint16_t v = latest(ch);
      out("INFO,  %2lu s  S%u %u mV (~%.0f N prov)\n", (unsigned long)((millis() - t0) / 1000), ch + 1, v, provN(ch, condUS(ch, v)));
    }
    delay(10);
  }
  return true;
}

// =====================================================================
void saveSettings() { prefs.putBytes("set", &S, sizeof(S)); }

void loadSettings() {
  if (prefs.getBytesLength("set") == sizeof(S)) prefs.getBytes("set", &S, sizeof(S));
  if (S.magic != 0x484F4C44 || S.selCh >= NCH || S.dwellS < 5) {
    S.magic = 0x484F4C44; S.dwellS = 30; S.pN = 1000; S.stepN = 100; S.cycle = 1; S.selCh = 0;
  }
}

void buildPlan() {
  nPlan = 0;
  float g[24];
  int ng = 0;
  for (float f = S.stepN; f <= S.pN + 0.01f && ng < 21; f += S.stepN) g[ng++] = f;
  for (int i = 0; i + 1 < ng; i += 2) {
    PlanRun &r = plan[nPlan];
    snprintf(r.id, sizeof(r.id), "UP%02u", (unsigned)(nPlan + 1));
    r.kind = 'U'; r.t1 = g[i]; r.t2 = g[i + 1]; nPlan++;
  }
  if (ng % 2 == 1 && ng >= 2) {
    PlanRun &r = plan[nPlan];
    snprintf(r.id, sizeof(r.id), "UP%02u", (unsigned)(nPlan + 1));
    r.kind = 'U'; r.t1 = g[ng - 2]; r.t2 = g[ng - 1]; nPlan++;
  }
  uint8_t nUp = nPlan;
  for (int i = ng - 2; i >= 0; i--) {
    PlanRun &r = plan[nPlan];
    snprintf(r.id, sizeof(r.id), "DN%02u", (unsigned)(nPlan - nUp + 1));
    r.kind = 'D'; r.t1 = S.pN; r.t2 = g[i]; nPlan++;
  }
  if (nextPlan >= nPlan) nextPlan = 0;
}

void printPlan() {
  out("PLAN,P %.0f N, step %.0f N, UTM hold %.0f s, cycle %u. In TRAPEZIUM: A1 Load to t1, A2 Hold %.0f s, A3 Load/Unload to t2, A4 Hold %.0f s\n",
      S.pN, S.stepN, S.dwellS, S.cycle, S.dwellS, S.dwellS);
  for (uint8_t i = 0; i < nPlan; i++)
    out("PLAN,%s,%s t1 %.0f N -> t2 %.0f N%s\n", plan[i].id, plan[i].kind == 'U' ? "loading   " : "descending", plan[i].t1,
        plan[i].t2, i == nextPlan ? "   <- next" : "");
}

// =====================================================================
void printHeader() {
  out("FILE_OPEN,UTM,%u\n", sessCh + 1);
  out("# fw,%s\n", FW_VERSION);
  out("# mode,utm_hold,channel,%u\n", sessCh + 1);
  out("# site,%s\n", SITE_OF_CH[sessCh]);
  out("# note,%s\n", note);
  out("# rate_hz,%lu,oversample,%u,adc_atten,11dB\n", 1000UL / PERIOD_MS, OVERSAMPLE);
  out("# rf_ohm,%.0f\n", RF_OHM[sessCh]);
  out("# base_mV,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n", baseMV[0], baseMV[1], baseMV[2], baseMV[3], baseMV[4], baseMV[5], baseMV[6]);
  out("# utm_hold_s,%.0f,board_window,%.0f..%.0f s after h,early_window,3..5 s\n", S.dwellS, S.dwellS - MARGIN_S - AVG_S, S.dwellS - MARGIN_S);
  out("# p_N,%.0f,step_N,%.0f,cycle,%u\n", S.pN, S.stepN, S.cycle);
  out("# flags,bit1=adc_top_3250_mV,bit2=moving_in_last_2s\n");
  out("# CAL,ms,ch,cycle,run,run_id,hold,dir,set_N,utm_N,mean_mV,sd_mV,min_mV,max_mV,early_mV,drift_mV,trend_mV,"
      "base_mV,rs_kOhm,g_uS,xtalk_mV,flags,redo,win_s\n");
}

void printPoint(const Point &p) {
  out("CAL,%lu,%u,%u,%u,%s,%u,%c,%.1f,%.2f,%.2f,%.2f,%u,%u,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.2f,%u,%u,%.1f\n",
      (unsigned long)p.ms, p.ch + 1, p.cycle, p.run, p.runId, p.hold, p.dir, p.setN, p.utmN, p.mean, p.sd, p.mn, p.mx,
      p.early, p.drift, p.trend, p.base, p.g > 0 ? 1000.0f / p.g : -1.0f, p.g, p.xtalk, p.flags, p.redo, p.win);
}

void addPoint(const Point &p) {
  printPoint(p);
  if (nPts < MAX_PTS) pts[nPts++] = p;
  else out("WARN,point table full (%u): the row above is printed / saved but not kept for list. save soon\n", MAX_PTS);
}

void openSession() {
  sessOn = true;
  sessCh = S.selCh;
  nPts = 0;
  runNo = 0;
  printHeader();
  out("INFO,channel S%u (GPIO%u, site %s, RF %.0f) open\n", sessCh + 1, FSR_PINS[sessCh], SITE_OF_CH[sessCh], RF_OHM[sessCh]);
}

// after save: plain CAL table to copy from the Serial Monitor; while open: LIST lines (not saved twice by the logger)
void listPoints(bool final) {
  out("INFO,--- S%u: %u points%s ---\n", sessCh + 1, nPts, final ? " (copy from the CAL header line into Excel)" : "");
  if (!final) { for (uint16_t i = 0; i < nPts; i++) { out("LIST,"); printPoint(pts[i]); } return; }
  out("CAL,ms,ch,cycle,run,run_id,hold,dir,set_N,utm_N,mean_mV,sd_mV,min_mV,max_mV,early_mV,drift_mV,trend_mV,"
      "base_mV,rs_kOhm,g_uS,xtalk_mV,flags,redo,win_s\n");
  for (uint16_t i = 0; i < nPts; i++) printPoint(pts[i]);
  out("INFO,--- end of table ---\n");
}

void saveSession() {
  if (!sessOn) { out("WARN,nothing open\n"); return; }
  out("# end,points,%u,runs,%u\n", nPts, runNo);
  out("FILE_CLOSE,rows,0,dropped,0,PASS\n");
  listPoints(true);
  sessOn = false;
  if (S.selCh + 1 < NCH) { S.selCh++; saveSettings(); }
  out("INFO,S%u closed. Next channel S%u selected (s<n> to change). Move the puck, z, then run\n", sessCh + 1, S.selCh + 1);
}

// =====================================================================
String readLine() {
  String s;
  for (;;) {
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n') { s.trim(); return s; }
      if (c != '\r') s += c;
    }
    delay(5);
  }
}

bool parseNum(const String &s, float &v) {
  if (!s.length()) return false;
  uint8_t dots = 0;
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '.') { if (++dots > 1) return false; }
    else if (!isDigit(c)) return false;
  }
  if (s == ".") return false;
  v = s.toFloat();
  return true;
}

bool zeroAll() {
  out("ZERO,keep all FSRs unloaded, measuring 2 s...\n");
  Stat st;
  if (collect(400, 0, st, false)) { out("ERR,zero timed out\n"); return false; }
  bool ok = true;
  for (uint8_t ch = 0; ch < NCH; ch++) {
    float m = meanOf(st, ch);
    bool good = m > 250 && m < 350;
    out("ZERO,S%u,mean_mV,%.1f,sd_mV,%.2f,%s\n", ch + 1, m, sdOf(st, ch), good ? "ok" : "OUT OF 250-350 mV: loaded or not connected, kept old");
    if (good) { baseMV[ch] = m; zeroed[ch] = true; } else ok = false;
  }
  return ok;
}

float xtalkOf(const Stat &st, uint8_t ch) {
  float x = 0;
  for (uint8_t o = 0; o < NCH; o++) if (o != ch && zeroed[o]) x = fmaxf(x, fabsf(meanOf(st, o) - baseMV[o]));
  return x;
}

// zero point at the start of a run (puck clear, UTM not started)
void runZero() {
  uint8_t ch = sessCh;
  Stat st;
  if (collect((uint32_t)(ZERO_S * 1000 / PERIOD_MS), ch, st, false)) { out("ERR,zero point timed out\n"); return; }
  Point p = {};
  p.ms = millis(); p.ch = ch; p.cycle = S.cycle; p.run = runNo; p.hold = 0; p.dir = 'Z';
  strncpy(p.runId, runId, sizeof(p.runId) - 1);
  p.setN = 0; p.utmN = 0; p.mean = meanOf(st, ch); p.sd = sdOf(st, ch); p.mn = st.mn; p.mx = st.mx;
  p.early = p.mean; p.base = baseMV[ch]; p.g = 0; p.xtalk = xtalkOf(st, ch); p.flags = st.flags; p.win = ZERO_S;
  float off = p.mean - baseMV[ch];
  addPoint(p);
  out("INFO,zero point S%u: %.1f mV, %+.1f mV vs z -> %s\n", ch + 1, p.mean, off,
      fabsf(off) <= 5 ? "OK" : "CHECK: still loaded / not recovered (wait, or z again before the next run)");
}

void startRun(String arg) {
  arg.trim();
  arg.toUpperCase();
  if (!zeroed[S.selCh]) { out("WARN,z first with all sensors unloaded\n"); return; }
  if (!sessOn) openSession();
  if (S.selCh != sessCh) { out("WARN,channel S%u is open: save first, then s<n>\n", sessCh + 1); return; }
  runPlan = -1;
  if (!arg.length()) runPlan = nextPlan;
  else if (arg != "FREE") {
    for (uint8_t i = 0; i < nPlan; i++) if (arg == plan[i].id) runPlan = i;
    if (runPlan < 0) { out("WARN,unknown run %s (plan = list, run FREE = no targets)\n", arg.c_str()); return; }
  }
  runNo++;
  holdNo = 0;
  hold1N = -1;
  strncpy(runId, runPlan >= 0 ? plan[runPlan].id : "FREE", sizeof(runId) - 1);
  out("# run,%s,%u\n", runId, runNo);
  if (runPlan >= 0) {
    const PlanRun &r = plan[runPlan];
    out("INFO,=== run %u: %s on S%u. TRAPEZIUM: A1 Load to %.0f N, A2 Hold %.0f s, A3 %s to %.0f N, A4 Hold %.0f s ===\n",
        runNo, r.id, sessCh + 1, r.t1, S.dwellS, r.kind == 'U' ? "Load" : "Unload", r.t2, S.dwellS);
    nextPlan = (runPlan + 1) % nPlan;
  } else {
    out("INFO,=== run %u: FREE on S%u (no targets: type h, then the force) ===\n", runNo, sessCh + 1);
  }
  out("INFO,puck clear? zero point 2 s...\n");
  runZero();
  out("INFO,start the UTM. Type h + Enter when the UTM display shows hold 1 has started\n");
}

// one hold: h typed when the UTM hold started
void measureHold(float targetN) {
  uint8_t ch = sessCh;
  uint8_t redo = 0;
  if (targetN < 0 && runPlan >= 0) targetN = holdNo == 0 ? plan[runPlan].t1 : (holdNo == 1 ? plan[runPlan].t2 : -1);
  const float endS = S.dwellS - MARGIN_S;
  const float e0 = endS >= 10 ? 3.0f : 1.0f, e1 = endS >= 10 ? 5.0f : 2.0f;
  for (;;) {
    uint32_t t0 = millis();
    out("INFO,hold %u%s: measuring %.0f s (stops %.0f s before the UTM hold ends). x = cancel\n", holdNo + 1,
        targetN >= 0 ? "" : " (no target)", endS, MARGIN_S);
    Stat early, a, b;
    if (!waitUntil(t0 + (uint32_t)(e0 * 1000), t0, ch)) { out("INFO,hold cancelled\n"); return; }
    int r = collect((uint32_t)((e1 - e0) * 1000 / PERIOD_MS), ch, early, true);
    if (r == 0 && !waitUntil(t0 + (uint32_t)((endS - AVG_S) * 1000), t0, ch)) r = 1;
    if (r == 0) r = collect((uint32_t)(AVG_S * 500 / PERIOD_MS), ch, a, true);   // first half of the last 2 s
    if (r == 0) r = collect((uint32_t)(AVG_S * 500 / PERIOD_MS), ch, b, true);   // second half
    if (r == 1) { out("INFO,hold cancelled, nothing saved\n"); return; }
    if (r == 2) { out("ERR,sampler timeout, nothing saved\n"); return; }
    float trend = meanOf(b, ch) - meanOf(a, ch);
    Stat w = a;
    merge(w, b);
    float mean = meanOf(w, ch), sd = sdOf(w, ch);
    float earlyMv = meanOf(early, ch);
    float g = condUS(ch, mean);
    uint8_t flags = w.flags;
    float exc = mean - baseMV[ch];
    if (fabsf(trend) > fmaxf(6.0f, 0.01f * exc)) flags |= 0x04;
    out("MEAS,S%u,mean_mV,%.1f,sd_mV,%.2f,min,%u,max,%u,drift_mV,%+.1f (since %.0f-%.0f s),est_N_prov,%.0f\n", ch + 1, mean, sd,
        w.mn, w.mx, mean - earlyMv, e0, e1, provN(ch, g));
    if (flags & 0x02) out("WARN,CLIPPED (> %u mV): this force is out of range for RF %.0f\n", ADC_TOP_MV, RF_OHM[ch]);
    else if (w.mx > NEAR_MV) out("WARN,near the ADC rail (> %u mV)\n", NEAR_MV);
    if (flags & 0x04) out("WARN,signal still moving in the last 2 s (%+.1f mV/s): did the UTM hold end early? a = redo\n", trend);
    if (exc < DEAD_MV) out("WARN,no response: puck off the sensor, wrong channel or not connected?\n");

    // force typed by the user
    float utmN = 0;
    bool redoReq = false, discard = false;
    for (;;) {
      while (Serial.available()) Serial.read();
      if (targetN >= 0) out("PROMPT,type the UTM force NOW (Enter or . = %.0f), a = redo, x = discard\n", targetN);
      else out("PROMPT,type the UTM force NOW in N, a = redo, x = discard\n");
      String in = readLine();
      in.toLowerCase();
      if (in == "x") { discard = true; break; }
      if (in == "a") { redoReq = true; break; }
      float v;
      if ((!in.length() || in == ".") && targetN >= 0) v = targetN;
      else if (!parseNum(in, v)) { out("WARN,type a number in N, e.g. 102.5\n"); continue; }
      if (v > FULL_SCALE_N * 1.5f) { out("WARN,above 1.5x full scale, retype\n"); continue; }
      if (targetN >= 0 && fabsf(v - targetN) > fmaxf(20.0f, 0.2f * targetN)) {
        out("PROMPT,%.1f N is far from the target %.0f N. y = accept, anything else = retype\n", v, targetN);
        String c = readLine();
        if (c != "y" && c != "Y") continue;
      }
      utmN = v;
      break;
    }
    if (discard) { out("INFO,hold discarded\n"); return; }
    if (redoReq) { redo++; out("INFO,redo: type h when the next stable hold starts\n"); return; }

    Point p = {};
    p.ms = millis(); p.ch = ch; p.cycle = S.cycle; p.run = runNo; p.hold = holdNo + 1;
    strncpy(p.runId, runId, sizeof(p.runId) - 1);
    if (utmN < 1.0f) p.dir = 'Z';
    else if (hold1N < 0 || utmN > hold1N + 0.5f) p.dir = 'U';
    else p.dir = 'D';
    p.setN = targetN >= 0 ? targetN : utmN; p.utmN = utmN; p.mean = mean; p.sd = sd; p.mn = w.mn; p.mx = w.mx;
    p.early = earlyMv; p.drift = mean - earlyMv; p.trend = trend; p.base = baseMV[ch]; p.g = g;
    p.xtalk = xtalkOf(w, ch); p.flags = flags; p.redo = redo; p.win = AVG_S;
    addPoint(p);
    if (holdNo == 0) hold1N = utmN;
    holdNo++;
    if (holdNo == 1) out("INFO,hold 1 saved (%c %.1f N). Type h when hold 2 starts\n", p.dir, utmN);
    else out("INFO,hold %u saved (%c %.1f N). Run %s done: run for the next one, save to finish S%u\n", holdNo, p.dir, utmN,
             runId, ch + 1);
    return;
  }
}

// =====================================================================
void printHelp() {
  out("--- %s ---\n", FW_VERSION);
  out("z zero | s<n> channel | t<s> UTM hold time | c<n> cycle | p<N> peak | plan | run [ID] | h / h<N> hold | x cancel\n");
  out("n<text> note | list | save | l live | ?\n");
  out("INFO,channel S%u (site %s, RF %.0f), UTM hold %.0f s, P %.0f N, cycle %u, channel %s, next run %s\n", S.selCh + 1,
      SITE_OF_CH[S.selCh], RF_OHM[S.selCh], S.dwellS, S.pN, S.cycle, sessOn ? "OPEN" : "closed", plan[nextPlan].id);
}

void handleCommand(String cmd) {
  cmd.trim();
  if (!cmd.length()) return;
  String lc = cmd;
  lc.toLowerCase();
  char c = lc[0];
  String arg = lc.substring(1);
  arg.trim();
  float v;
  if (lc == "plan") printPlan();
  else if (lc == "list") { if (sessOn) listPoints(false); else out("WARN,nothing open\n"); }
  else if (lc == "save") saveSession();
  else if (lc.startsWith("run")) startRun(cmd.substring(3));
  else if (lc == "x") out("INFO,nothing to cancel (x works during a hold)\n");
  else if (c == 'h') {
    if (!sessOn || !runNo) { out("WARN,run first\n"); return; }
    float t = -1;
    if (arg.length() && !parseNum(arg, t)) { out("WARN,use h or h<N>, e.g. h250\n"); return; }
    bool wl = liveOn; liveOn = false;
    measureHold(t);
    liveOn = wl;
  } else if (c == 'z') {
    if (sessOn) { out("WARN,channel open: zero only between channels (save first). Each run has its own zero point\n"); return; }
    zeroAll();
  } else if (c == 's' && arg.length()) {
    if (sessOn) { out("WARN,save first, then change channel\n"); return; }
    int n = arg.toInt();
    if (n < 1 || n > NCH) { out("WARN,channel s1..s7\n"); return; }
    S.selCh = n - 1; saveSettings();
    out("INFO,selected S%u (GPIO%u, site %s, RF %.0f)\n", S.selCh + 1, FSR_PINS[S.selCh], SITE_OF_CH[S.selCh], RF_OHM[S.selCh]);
  } else if (c == 't') {
    if (!parseNum(arg, v) || v < 8 || v > 600) { out("WARN,t8..t600 (UTM hold time in s)\n"); return; }
    S.dwellS = v; saveSettings();
    out("INFO,UTM hold %.0f s: board measures until %.0f s after h, mean of %.0f-%.0f s\n", v, v - MARGIN_S, v - MARGIN_S - AVG_S, v - MARGIN_S);
  } else if (c == 'c') {
    if (!parseNum(arg, v) || v < 1 || v > 99) { out("WARN,c1..c99\n"); return; }
    S.cycle = (uint8_t)v; saveSettings(); out("INFO,cycle %u\n", S.cycle);
  } else if (c == 'p') {
    if (sessOn) { out("WARN,save first\n"); return; }
    if (!parseNum(arg, v) || v < 200 || v > 1500) { out("WARN,p200..p1500\n"); return; }
    S.pN = v; saveSettings(); nextPlan = 0; buildPlan(); printPlan();
  } else if (c == 'n') {
    String t = cmd.substring(1);
    t.trim(); t.replace(",", ";");
    if (sessOn) out("# note,%lu,%s\n", (unsigned long)millis(), t.c_str());
    else { strncpy(note, t.c_str(), sizeof(note) - 1); out("INFO,note '%s'\n", note); }
  } else if (lc == "l") liveOn = !liveOn;
  else if (c == '?') printHelp();
  else out("WARN,unknown command '%s', ? for help\n", cmd.c_str());
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  out("%s\n", FW_VERSION);
  prefs.begin("utmhold", false);
  loadSettings();
  buildPlan();
  analogReadResolution(12);
  for (uint8_t ch = 0; ch < NCH; ch++) { analogSetPinAttenuation(FSR_PINS[ch], ADC_11db); baseMV[ch] = 300.0f; }
  xTaskCreate(samplerTask, "sampler", 4096, nullptr, 3, nullptr);
  delay(200);
  zeroAll();
  printHelp();
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n') { handleCommand(rx); rx = ""; }
    else if (ch != '\r') rx += ch;
  }
  if (liveOn && millis() - lastLive >= 200) {
    lastLive = millis();
    uint8_t ch = S.selCh;
    uint16_t v = latest(ch);
    float g = condUS(ch, v);
    out("LIVE,S%u,%u mV,%.2f uS,~%.0f N prov%s\n", ch + 1, v, g, provN(ch, g), v > ADC_TOP_MV ? ",CLIPPED" : "");
  }
  delay(2);
}
