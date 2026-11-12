// FSR_UTM_Cal.ino v3.0 - FlexiForce A401 calibration on the Shimadzu AG-X plus (TRAPEZIUM X)
// Board: ESP32C6 Dev Module, Arduino-ESP32 3.3.x, USB CDC On Boot: Enabled
// PC side: utm_pc_logger.py v3 (not the Arduino Serial Monitor). Files are written on the PC.
//
// TWO WAYS TO RECORD
// 1) RUN mode (new, for the 4-Area Single methods in TRAPEZIUM X):
//      run [ID]   opens one file per machine run, measures an unloaded baseline, then records
//                 all 7 channels at 200 Hz continuously (ramps, both holds, return, recovery).
//                 The UTM does the timing. The ESP32 never waits for typed forces.
//      k[text]    event mark (type k + Enter at the moment you click Start in TRAPEZIUM)
//      end        unloaded post-baseline, hold summary, close the file
//    The board also detects the holds itself (flat FSR signal) and prints a QC line per hold,
//    so you see at once whether the run gave 2 clean plateaus. The real force comes later
//    from the Shimadzu raw export, paired offline by tools/utm_pair.py.
//    Run plan from P (peak) and step:  UP01 100->200 ... UP05 900->1000,  DN01 1000->900 ... DN09 1000->100.
//    plan = show the plan with the TRAPEZIUM Area settings; run with no ID = next run not done.
//    Any other ID (run PILOT1, run COND) = free run with the same recording and hold detection.
// 2) POINT mode (v2.x, unchanged): start, m / m<N> (hold, then type the UTM force), save.
//
// Other commands (not case sensitive):
//   z            zero all channels (all unloaded; accepts 250-350 mV per channel)
//   s<n>         select channel 1..7
//   n<text>      note (into the next header, or a '# note' line in the open file)
//   set          show settings;  set <key> <value>: p step dwell ramp rest rep hold avg base
//   meta         show metadata;  meta <key> <value> (e.g. meta cell 20kN SN1234);  meta <key> = delete
//   done <ID> / undo <ID>   mark a plan run as done / not done
//   x            abort the open run (file is kept, marked ABORTED)
//   r            free recording (RAW), save to stop
//   l            live view on/off,   i  indicator test,   ?  help
// Settings, metadata and the done list are kept in flash (survive a reset).
//
// Serial protocol to the PC (one complete line per write):
//   FILE_OPEN,<RUN|UTM|RAW>,<ch>[,<run_id>,<rep>]   PC opens a file
//   '#' header/notes, R raw frames, EV events, BASE baselines, HOLD on-board holds, CAL points
//   FILE_CLOSE,rows,N,dropped,N,PASS|CHECK           PC closes the file
//   anything else (PROMPT, INFO, WARN, ERR, ZERO, LIVE, STAT, PLAN, MEAS) is screen only
//
// Front end (all channels): Vout = VREF*(1 + RF/Rs)  ->  G = (Vout/base - 1)/RF, base = unloaded
// output (= VREF). RF_OHM[] MUST equal the resistor fitted on each channel.

#include <Arduino.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>
#include <Preferences.h>
#include "esp_timer.h"
#include "esp_random.h"

#define FW_VERSION "FSR_UTM_Cal v3.0 run mode + hold detection, RF per channel (2026-11-11)"

// ---------------- Pins / channels ----------------
const uint8_t NCH = 7;
const uint8_t FSR_PINS[NCH] = {0, 1, 2, 3, 4, 5, 6};          // ch1..ch7 = GPIO0..6
// insole site of each channel (workbook sensor S<n> = channel n)
const char *const SITE_OF_CH[NCH] = {"S1", "S4", "S6", "S11", "S13", "S10", "S14"};
const uint32_t SERIAL_BAUD = 921600;   // ignored by native USB

// ---------------- Front end ----------------
// RF per channel in ohm. If you change a resistor (e.g. 27k on ch3, ch4, ch6, ch7 against
// clipping), change it here BEFORE recording, measure it with a meter and write the measured value.
// 2026-11-12: ch3, ch4, ch6, ch7 changed to 30k (they clipped below 1000 N at 39k); ch1, ch2, ch5 stay 39k.
// Replace the nominal values with the values measured by a meter (e.g. 29870) for the best accuracy.
const float RF_OHM[NCH] = {39000, 39000, 30000, 30000, 39000, 30000, 30000};
// Provisional display only (live view, hold QC): loading curves aU, bU of insole-app DEFAULT_CAL.
const float PROV_A[NCH] = {3.58966f, 3.98233f, 3.22000f, 2.72739f, 4.28822f, 2.80304f, 3.41093f};
const float PROV_B[NCH] = {1.02654f, 1.01871f, 1.00616f, 1.02718f, 1.01217f, 1.03127f, 0.99299f};
const float DEAD_MV = 3.0f;
const uint16_t ADC_TOP_MV = 3250;     // ADC clips ~3300 mV: above this = clipped (flag bit1)
const uint16_t NEAR_MV = 3000;        // near-rail warning (screen only, derivable from mV)

// ---------------- Point mode (v2.x) ----------------
const float FULL_SCALE_N = 1000.0f;   // typed forces above 1.2x this are rejected
const float ZERO_N = 1.0f;
const uint32_t FIRST_MS = 1000;
const float CONFIRM_FRAC = 0.2f;

// ---------------- Hold detection (run mode) ----------------
const uint8_t BLOCK_N = 20;           // 20 frames = 100 ms blocks
const float ONSET_MV = 8.0f;          // loaded = block mean > base + this
const float CONFIRM_S = 1.0f;         // flat this long -> hold
const float MIN_HOLD_S = 5.0f;        // shorter flat parts are not reported
const float EARLY_FROM_S = 3.0f, EARLY_TO_S = 5.0f;   // early window for 30 s drift
const float GUARD_S = 0.7f;           // late window ends this long before the detected hold end

// ---------------- Timing ----------------
const uint32_t PERIOD_MS = 5;         // 200 Hz
const uint8_t OVERSAMPLE = 4;
const uint16_t QUEUE_LEN = 1024;
const uint32_t SEND_MS = 100;
const uint32_t LIVE_MS = 200;
const uint32_t STAT_MS = 2000;        // status line during a run

// ---------------- Indicator (optional) ----------------
enum { IND_NONE, IND_LED, IND_BUZZER };
const uint8_t IND_TYPE = IND_NONE;
const int IND_PIN = 10;

// =====================================================================
// Types (all defined before the first function)
struct Frame {
  uint64_t t_us;
  uint32_t idx;
  uint16_t mv[NCH];
  uint8_t flags;            // bit1 = some channel above ADC_TOP_MV
};

struct Meas {
  float mean[NCH], sd[NCH], first;
  uint16_t mn, mx;
  uint32_t f0, f1;
  uint8_t flags;
};

struct Settings {
  uint32_t magic;
  float pN, stepN, dwellS, rampNps, restS, holdS, avgS, baseS;
  uint16_t rep;
  uint8_t selCh;
  uint32_t done[NCH];       // bit i = plan run i done for this repeat
};

struct MetaKV { char k[16]; char v[48]; };
const uint8_t NMETA = 12;

const uint8_t MAX_RUNS = 32;
struct PlanRun {
  char id[6];               // UP01, DN03
  char kind;                // 'U' loading template, 'D' descending template
  float t1, t2;             // Area 1 / Area 3 targets
};

// ==== TRACKER BEGIN (no Arduino calls: unit-tested on the PC) ====
struct Block {              // BLOCK_N frames of the tracked channel
  uint64_t t_us;            // time of the last frame
  uint32_t f0, f1;          // first / last frame index
  uint32_t n;
  uint64_t sum, ss;
  uint16_t mn, mx;
  uint8_t flags;
  float chMean[NCH];        // block mean of every channel (wrong-channel check)
};

struct WinStat {
  uint32_t n, f0, f1;
  double mean, sd;
  uint16_t mn, mx;
  uint8_t flags;
  uint64_t t0, t1;
};

struct HoldInfo {
  int k;                    // 1, 2, ... in this run
  uint64_t t0, t1;          // estimated start / end of the plateau (us)
  float dur;                // s
  bool earlyOk;
  float early;              // mean mV, EARLY_FROM..EARLY_TO s after start
  WinStat late;             // last avg s before t1 - guard
  float drift;              // late - early (mV)
  uint8_t flags;            // bit1 clip, bit2 short vs dwell, bit3 no early window, bit4 window cut, bit5 run ended in hold
};

enum { TE_ONSET = 1, TE_HOLD_START = 2, TE_HOLD_END = 4, TE_UNLOAD = 8 };

struct Tracker {
  // configuration
  float base = 300, onsetMV = ONSET_MV, flatMVps = 7.5f, confirmS = CONFIRM_S, minHoldS = MIN_HOLD_S;
  float earlyFrom = EARLY_FROM_S, earlyTo = EARLY_TO_S, avgS = 2, guardS = GUARD_S, dwellS = 30;
  float blockS = BLOCK_N * PERIOD_MS / 1000.0f, halfS = 0.5f, exitFactor = 1.5f, leaveS = 1.5f;
  // state
  static const int RING = 200;      // 20 s of blocks
  Block ring[RING];
  uint32_t total = 0;
  uint8_t state = 0;                // 0 unloaded, 1 moving, 2 hold, 3 leaving the hold (end not yet confirmed)
  int above = 0, below = 0, flatRun = 0, steepRun = 0, holds = 0;
  uint64_t aboveT = 0, belowT = 0, flatT = 0, holdT0 = 0, candT1 = 0, leaveT = 0;
  uint64_t onsetT = 0, unloadT = 0;
  bool earlyOk = false;
  float early = 0, slope = 0, ref = 0, tol = 4;
  HoldInfo last;

  void reset(float b) {
    base = b; total = 0; state = 0; above = below = flatRun = steepRun = holds = 0;
    earlyOk = false; slope = 0;
  }
  const Block &back(int k) const { return ring[(total - 1 - k) % RING]; }
  int avail() const { return total < (uint32_t)RING ? (int)total : RING; }
  static float bmean(const Block &b) { return b.n ? (float)((double)b.sum / b.n) : 0.0f; }

  // stats of the blocks whose end time is in (w0, w1]
  bool win(uint64_t w0, uint64_t w1, WinStat &w) const {
    w.n = 0; w.mn = 0xFFFF; w.mx = 0; w.flags = 0; w.f0 = w.f1 = 0; w.t0 = w.t1 = 0; w.mean = w.sd = 0;
    double s = 0, ss = 0;
    for (int k = avail() - 1; k >= 0; k--) {        // oldest first
      const Block &b = back(k);
      if (b.t_us <= w0 || b.t_us > w1) continue;
      if (!w.n) { w.f0 = b.f0; w.t0 = b.t_us; }
      w.f1 = b.f1; w.t1 = b.t_us;
      w.n += b.n; s += (double)b.sum; ss += (double)b.ss;
      if (b.mn < w.mn) w.mn = b.mn;
      if (b.mx > w.mx) w.mx = b.mx;
      w.flags |= b.flags;
    }
    if (!w.n) return false;
    w.mean = s / w.n;
    double var = ss / w.n - w.mean * w.mean;
    w.sd = var > 0 ? sqrt(var) : 0;
    return true;
  }

  // The signal left the plateau at block 0: plateau level = blocks 1.5-2.5 s back; the hold end
  // is the newest block still within tol of that level.
  void findEnd() {
    const uint64_t tNow = back(0).t_us;
    double s = 0, ss = 0;
    int n = 0;
    for (int k = 0; k < avail(); k++) {
      const Block &b = back(k);
      if (b.t_us + 1500000ULL > tNow) continue;
      if (b.t_us + 2500000ULL <= tNow || b.t_us < holdT0) break;
      double m = bmean(b);
      s += m; ss += m * m; n++;
    }
    if (n < 3) {                                     // hold younger than 2.5 s: use the hold start
      ref = bmean(back(0)); tol = 4; candT1 = tNow; return;
    }
    ref = (float)(s / n);
    double var = ss / n - (double)ref * ref;
    tol = fmaxf(4.0f, 4.0f * (float)sqrt(var > 0 ? var : 0));
    candT1 = tNow - 1500000ULL;
    for (int k = 0; k < avail(); k++) {
      const Block &b = back(k);
      if (b.t_us + 1500000ULL <= tNow) break;
      if (fabsf(bmean(b) - ref) <= tol) { candT1 = b.t_us; break; }
    }
  }

  int endHold(uint64_t t1) {
    float dur = (float)((double)(t1 - holdT0) / 1e6);
    state = 1; flatRun = 0; steepRun = 0;
    if (dur < minHoldS) return 0;                   // short pause: ignore
    HoldInfo h;
    h.k = ++holds; h.t0 = holdT0; h.t1 = t1; h.dur = dur; h.flags = 0;
    h.earlyOk = earlyOk; h.early = early;
    uint64_t w1 = t1 - (uint64_t)(guardS * 1e6f);
    uint64_t w0 = w1 - (uint64_t)(avgS * 1e6f);
    if (w0 < holdT0) { w0 = holdT0; h.flags |= 0x10; }
    if (!win(w0, w1, h.late)) h.flags |= 0x10;
    h.flags |= h.late.flags & 0x02;
    if (dwellS > 0 && dur < 0.9f * dwellS) h.flags |= 0x04;
    if (!h.earlyOk) h.flags |= 0x08;
    h.drift = h.earlyOk && h.late.n ? (float)(h.late.mean - h.early) : 0.0f;
    last = h;
    return TE_HOLD_END;
  }

  // run ended while a hold was open: report it up to the newest block (flag bit5)
  int flush() {
    if (state < 2 || !total) return 0;
    uint64_t t1 = state == 3 ? candT1 : back(0).t_us;
    int e = endHold(t1);
    if (e) last.flags |= 0x20;
    return e;
  }

  int push(const Block &b) {
    ring[total % RING] = b;
    total++;
    int ev = 0;
    float m = bmean(b);
    bool haveSlope = false;
    if (total >= 10) {
      const Block &a = back(5);
      double s1 = 0, s0 = 0;
      for (int k = 0; k < 5; k++) { s1 += bmean(back(k)); s0 += bmean(back(k + 5)); }
      double dt = (double)(b.t_us - a.t_us) / 1e6;
      if (dt > 0) { slope = (float)((s1 - s0) / 5.0 / dt); haveSlope = true; }
    }
    bool hi = m > base + onsetMV;

    if (state == 0) {
      if (hi) {
        if (above++ == 0) aboveT = b.t_us;
        if (above >= 2) { state = 1; onsetT = aboveT; flatRun = steepRun = 0; below = 0; ev |= TE_ONSET; }
      } else above = 0;
      return ev;
    }
    if (!hi) {
      if (below++ == 0) belowT = b.t_us;
      if (below >= 5) {
        if (state == 2) { findEnd(); ev |= endHold(candT1); }
        else if (state == 3) ev |= endHold(candT1);
        state = 0; above = 0; below = 0; unloadT = belowT;
        return ev | TE_UNLOAD;
      }
    } else below = 0;
    if (!haveSlope) return ev;
    float as = fabsf(slope);
    if (state == 1) {
      if (as < flatMVps) {
        if (flatRun++ == 0) flatT = b.t_us;
        if (flatRun * blockS >= confirmS - 1e-3f) {
          state = 2; steepRun = 0; earlyOk = false;
          holdT0 = flatT - (uint64_t)(halfS * 1e6f);
          ev |= TE_HOLD_START;
        }
      } else flatRun = 0;
    } else if (state >= 2) {
      if (!earlyOk && b.t_us >= holdT0 + (uint64_t)(earlyTo * 1e6f)) {
        WinStat w;
        if (win(holdT0 + (uint64_t)(earlyFrom * 1e6f), holdT0 + (uint64_t)(earlyTo * 1e6f), w)) {
          earlyOk = true; early = (float)w.mean;
        }
      }
      if (state == 2) {
        if (as > exitFactor * flatMVps) {
          if (++steepRun >= 3) { findEnd(); state = 3; leaveT = b.t_us; }
        } else steepRun = 0;
      } else {                                       // leaving: confirm, or back to the hold
        float dev = fabsf(m - ref);
        if (dev > 3 * tol) ev |= endHold(candT1);
        else if (b.t_us >= leaveT + (uint64_t)(leaveS * 1e6f)) {
          if (dev > tol) ev |= endHold(candT1);
          else { state = 2; steepRun = 0; }
        }
      }
    }
    return ev;
  }
};
// ==== TRACKER END ====

// =====================================================================
// Globals
QueueHandle_t q, blockQ;
SemaphoreHandle_t outLock;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
Frame latest;
Preferences prefs;
Settings S;
MetaKV meta[NMETA];
PlanRun plan[MAX_RUNS];
uint8_t nPlan = 0;
Tracker trk;

float baseMV[NCH];
bool zeroed[NCH] = {false};
char note[120] = "";
uint32_t bootId = 0;
uint16_t fileSeq = 0;

volatile bool logging = false, stopReq = false, fileOpen = false;
volatile uint32_t dropped, rows, blockDropped;
int8_t runCh = -1;                  // channel of the open file (-1 = free recording)

// point mode session
bool calOn = false;
uint8_t calCycle = 1;
uint16_t calStep = 0, calSaved = 0;
float prevN = -1;
char prevDir = 'U';
bool cycleLoaded = false, newCyclePending = false;

// run mode session
bool runOn = false;
int8_t runPlanIdx = -1;             // -1 = free run (PILOT, COND, ...)
char runId[16] = "";
char runUid[24] = "";
uint16_t evN = 0;
uint32_t runStartMs = 0, lastStat = 0;
float preBase[NCH];
bool nearWarned = false, clipWarned = false, chanWarned = false;
uint8_t holdsSeen = 0;
uint32_t lastRunEndMs = 0;
char lastRunId[16] = "";
float lastRunPeak = 0;

// tracking (sampler -> loop)
volatile bool trackOn = false;
volatile uint8_t trackCh = 0;

// accumulator filled by the sampler
volatile uint32_t accWant = 0, accN = 0;
volatile uint8_t accCh = 0;
double accS[NCH], accSS[NCH];
uint16_t accMin, accMax;
uint32_t accF0, accF1;
uint64_t accT0, accT1;
uint8_t accFlags;

bool liveOn = false;
String rx;
uint32_t lastLive = 0;

// =====================================================================
// Output: one locked write per call so lines never mix
void outWrite(const char *s, size_t n) {
  xSemaphoreTake(outLock, portMAX_DELAY);
  Serial.write((const uint8_t *)s, n);
  xSemaphoreGive(outLock);
}

void out(const char *fmt, ...) {
  char buf[360];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n < 0) return;
  if ((size_t)n >= sizeof(buf)) { n = sizeof(buf) - 1; buf[n - 1] = '\n'; }
  outWrite(buf, n);
}

// =====================================================================
// Sampler: 200 Hz, all channels; feeds the raw queue, the point accumulator and the hold tracker
void samplerTask(void *) {
  TickType_t last = xTaskGetTickCount();
  uint32_t frameIdx = 0;
  Block blk;
  uint8_t bn = 0;
  float chSum[NCH];
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(PERIOD_MS));
    Frame fr;
    fr.t_us = esp_timer_get_time();
    fr.idx = frameIdx++;
    fr.flags = 0;
    for (uint8_t ch = 0; ch < NCH; ch++) {
      uint32_t sum = 0;
      for (uint8_t k = 0; k < OVERSAMPLE; k++) sum += analogReadMilliVolts(FSR_PINS[ch]);
      fr.mv[ch] = (sum + OVERSAMPLE / 2) / OVERSAMPLE;
      if (fr.mv[ch] > ADC_TOP_MV) fr.flags |= 0x02;
    }

    if (accN < accWant) {
      if (accN == 0) { accMin = 0xFFFF; accMax = 0; accFlags = 0; accF0 = fr.idx; accT0 = fr.t_us; }
      for (uint8_t ch = 0; ch < NCH; ch++) { double v = fr.mv[ch]; accS[ch] += v; accSS[ch] += v * v; }
      uint16_t v = fr.mv[accCh];
      if (v < accMin) accMin = v;
      if (v > accMax) accMax = v;
      if (v > ADC_TOP_MV) accFlags |= 0x02;
      accF1 = fr.idx; accT1 = fr.t_us;
      accN++;
    }

    if (trackOn) {
      uint8_t tc = trackCh;
      uint16_t v = fr.mv[tc];
      if (bn == 0) {
        blk.f0 = fr.idx; blk.n = 0; blk.sum = blk.ss = 0; blk.mn = 0xFFFF; blk.mx = 0; blk.flags = 0;
        for (uint8_t ch = 0; ch < NCH; ch++) chSum[ch] = 0;
      }
      blk.sum += v; blk.ss += (uint64_t)v * v; blk.n++;
      if (v < blk.mn) blk.mn = v;
      if (v > blk.mx) blk.mx = v;
      if (v > ADC_TOP_MV) blk.flags |= 0x02;
      for (uint8_t ch = 0; ch < NCH; ch++) chSum[ch] += fr.mv[ch];
      if (++bn >= BLOCK_N) {
        blk.t_us = fr.t_us; blk.f1 = fr.idx;
        for (uint8_t ch = 0; ch < NCH; ch++) blk.chMean[ch] = chSum[ch] / bn;
        if (xQueueSend(blockQ, &blk, 0) != pdTRUE) blockDropped++;
        bn = 0;
      }
    } else bn = 0;

    portENTER_CRITICAL(&mux);
    latest = fr;
    portEXIT_CRITICAL(&mux);

    if (logging && xQueueSend(q, &fr, 0) != pdTRUE) dropped++;
  }
}

Frame getLatest() {
  Frame fr;
  portENTER_CRITICAL(&mux);
  fr = latest;
  portEXIT_CRITICAL(&mux);
  return fr;
}

// =====================================================================
// Raw rows to the PC in batches of complete lines; closes the file when asked
static_assert(NCH == 7, "formatFrame assumes 7 channels");
int formatFrame(char *p, size_t room, const Frame &fr) {
  return snprintf(p, room, "R,%llu,%lu,%u,%u,%u,%u,%u,%u,%u,%u\n",
                  (unsigned long long)fr.t_us, (unsigned long)fr.idx,
                  fr.mv[0], fr.mv[1], fr.mv[2], fr.mv[3], fr.mv[4], fr.mv[5], fr.mv[6], fr.flags);
}

void writerTask(void *) {
  static char buf[1024];
  size_t len = 0;
  uint32_t lastSend = millis();
  Frame fr;
  for (;;) {
    if (xQueueReceive(q, &fr, pdMS_TO_TICKS(20)) == pdTRUE) {
      char line[96];
      int n = formatFrame(line, sizeof(line), fr);
      if (n > 0 && (size_t)n < sizeof(line)) {
        if (len + n > sizeof(buf)) { outWrite(buf, len); len = 0; lastSend = millis(); }
        memcpy(buf + len, line, n);
        len += n;
        rows++;
      }
    }
    if (len && millis() - lastSend >= SEND_MS) { outWrite(buf, len); len = 0; lastSend = millis(); }
    if (fileOpen && stopReq && !logging && uxQueueMessagesWaiting(q) == 0) {
      if (len) { outWrite(buf, len); len = 0; }
      out("FILE_CLOSE,rows,%lu,dropped,%lu,%s\n", (unsigned long)rows, (unsigned long)dropped,
          dropped == 0 ? "PASS" : "CHECK");
      fileOpen = false;
      stopReq = false;
    }
  }
}

// =====================================================================
// Settings, metadata, plan (flash)
void defaultSettings() {
  S.magic = 0x55544D33;   // 'UTM3'
  S.pN = 1000; S.stepN = 100; S.dwellS = 30; S.rampNps = 10; S.restS = 120;
  S.holdS = 10; S.avgS = 2; S.baseS = 3; S.rep = 1; S.selCh = 0;
  for (uint8_t i = 0; i < NCH; i++) S.done[i] = 0;
}

void saveSettings() { prefs.putBytes("set", &S, sizeof(S)); }
void saveMeta() { prefs.putBytes("meta", meta, sizeof(meta)); }

void loadSettings() {
  memset(meta, 0, sizeof(meta));
  size_t n = prefs.getBytesLength("set");
  if (n == sizeof(S)) prefs.getBytes("set", &S, sizeof(S));
  if (n != sizeof(S) || S.magic != 0x55544D33 || S.selCh >= NCH) defaultSettings();
  if (prefs.getBytesLength("meta") == sizeof(meta)) prefs.getBytes("meta", meta, sizeof(meta));
  for (uint8_t i = 0; i < NMETA; i++) { meta[i].k[sizeof(meta[i].k) - 1] = 0; meta[i].v[sizeof(meta[i].v) - 1] = 0; }
}

// UP runs pair the grid points (100,200) (300,400) ...; DN runs go P -> each lower grid point
void buildPlan() {
  nPlan = 0;
  float g[40];
  int ng = 0;
  for (float f = S.stepN; f <= S.pN + 0.01f && ng < 40; f += S.stepN) g[ng++] = f;
  for (int i = 0; i + 1 < ng && nPlan < MAX_RUNS; i += 2) {
    PlanRun &r = plan[nPlan];
    snprintf(r.id, sizeof(r.id), "UP%02u", (unsigned)(nPlan + 1));
    r.kind = 'U'; r.t1 = g[i]; r.t2 = g[i + 1];
    nPlan++;
  }
  if (ng % 2 == 1 && ng >= 2 && nPlan < MAX_RUNS) {      // odd count: last run repeats one point
    PlanRun &r = plan[nPlan];
    snprintf(r.id, sizeof(r.id), "UP%02u", (unsigned)(nPlan + 1));
    r.kind = 'U'; r.t1 = g[ng - 2]; r.t2 = g[ng - 1];
    nPlan++;
  }
  uint8_t nUp = nPlan;
  for (int i = ng - 2; i >= 0 && nPlan < MAX_RUNS; i--) {
    PlanRun &r = plan[nPlan];
    snprintf(r.id, sizeof(r.id), "DN%02u", (unsigned)(nPlan - nUp + 1));
    r.kind = 'D'; r.t1 = S.pN; r.t2 = g[i];
    nPlan++;
  }
}

int findPlan(const char *id) {
  for (uint8_t i = 0; i < nPlan; i++) if (!strcasecmp(plan[i].id, id)) return i;
  return -1;
}

bool isDone(uint8_t ch, int i) { return i >= 0 && i < 32 && (S.done[ch] >> i) & 1u; }

void areaText(const PlanRun &r, char *buf, size_t n) {
  if (r.kind == 'U')
    snprintf(buf, n, "A1 Load/Force to %.0f N | A2 Hold/Force %.0f s | A3 Load/Force to %.0f N | A4 Hold/Force %.0f s",
             r.t1, S.dwellS, r.t2, S.dwellS);
  else
    snprintf(buf, n, "A1 Load/Force to %.0f N | A2 Hold/Force %.0f s | A3 Unload/Force to %.0f N | A4 Hold/Force %.0f s",
             r.t1, S.dwellS, r.t2, S.dwellS);
}

void printPlan() {
  out("PLAN,P %.0f N, step %.0f N, dwell %.0f s, ramp %.1f N/s, repeat %u, channel S%u (site %s)\n",
      S.pN, S.stepN, S.dwellS, S.rampNps, S.rep, S.selCh + 1, SITE_OF_CH[S.selCh]);
  uint8_t nd = 0;
  for (uint8_t i = 0; i < nPlan; i++) {
    char a[160];
    areaText(plan[i], a, sizeof(a));
    bool d = isDone(S.selCh, i);
    nd += d;
    out("PLAN,%s,%s,%s\n", plan[i].id, d ? "done" : "----", a);
  }
  out("PLAN,%u of %u runs done for S%u repeat %u. Ramp rate %.1f N/s in Areas 1 and 3 (same for every run)\n",
      nd, nPlan, S.selCh + 1, S.rep, S.rampNps);
}

const char *metaGet(const char *k) {
  for (uint8_t i = 0; i < NMETA; i++) if (meta[i].k[0] && !strcasecmp(meta[i].k, k)) return meta[i].v;
  return "";
}

// =====================================================================
// Header
void writeHeader(const char *mode) {
  out("# fw,%s\n", FW_VERSION);
  out("# start_ms,%lu\n", (unsigned long)millis());
  out("# mode,%s,channel,%d\n", mode, runCh + 1);
  out("# file_uid,%s\n", runUid);
  if (runCh >= 0) out("# site,%s\n", metaGet("site")[0] ? metaGet("site") : SITE_OF_CH[runCh]);
  out("# note,%s\n", note);
  out("# rate_hz,%lu,oversample,%u,adc_atten,11dB\n", 1000UL / PERIOD_MS, OVERSAMPLE);
  out("# rf_ohm,%.0f\n", RF_OHM[runCh >= 0 ? runCh : S.selCh]);
  out("# rf_ohm_ch,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f\n", RF_OHM[0], RF_OHM[1], RF_OHM[2], RF_OHM[3],
      RF_OHM[4], RF_OHM[5], RF_OHM[6]);
  out("# site_of_ch,%s,%s,%s,%s,%s,%s,%s\n", SITE_OF_CH[0], SITE_OF_CH[1], SITE_OF_CH[2], SITE_OF_CH[3],
      SITE_OF_CH[4], SITE_OF_CH[5], SITE_OF_CH[6]);
  out("# fsr_pins,%u,%u,%u,%u,%u,%u,%u\n", FSR_PINS[0], FSR_PINS[1], FSR_PINS[2], FSR_PINS[3],
      FSR_PINS[4], FSR_PINS[5], FSR_PINS[6]);
  out("# base_mV,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n", baseMV[0], baseMV[1], baseMV[2], baseMV[3],
      baseMV[4], baseMV[5], baseMV[6]);
  out("# base_convention,G_uS=(mV/base_mV-1)/RF*1e6; base_mV = unloaded output (= VREF); zero = z\n");
  for (uint8_t i = 0; i < NMETA; i++) if (meta[i].k[0]) out("# meta,%s,%s\n", meta[i].k, meta[i].v);
  out("# flags,bit1=adc_top_%u_mV\n", ADC_TOP_MV);
  out("# R,t_us,frame,S1_mV,S2_mV,S3_mV,S4_mV,S5_mV,S6_mV,S7_mV,flags\n");
}

void writeRunHeader(const char *tmpl, float t1, float t2, float restS) {
  out("# run_id,%s\n", runId);
  out("# run_uid,%s\n", runUid);
  out("# repeat_id,%u\n", S.rep);
  out("# template,%s\n", tmpl);
  if (t1 > 0) out("# targets_N,%.0f,%.0f\n", t1, t2);
  if (runPlanIdx >= 0) { char a[160]; areaText(plan[runPlanIdx], a, sizeof(a)); out("# areas,%s\n", a); }
  out("# p_N,%.0f\n", S.pN);
  out("# dwell_s,%.1f\n", S.dwellS);
  out("# ramp_N_per_s,%.2f\n", S.rampNps);
  out("# rest_s_since_last_run,%.1f\n", restS);
  out("# prev_run,%s,peak_N,%.0f\n", lastRunId[0] ? lastRunId : "none", lastRunPeak);
  out("# hold_detect,block_ms,%u,onset_mV,%.1f,flat_mV_per_s,%.2f,confirm_s,%.1f,min_hold_s,%.1f,"
      "early_s,%.1f,%.1f,avg_s,%.1f,guard_s,%.1f\n", BLOCK_N * (unsigned)PERIOD_MS, ONSET_MV, trk.flatMVps,
      CONFIRM_S, MIN_HOLD_S, EARLY_FROM_S, EARLY_TO_S, S.avgS, GUARD_S);
  out("# EV,ev_id,t_us,frame,type,text\n");
  out("# BASE,kind,t0_us,t1_us,frame0,frame1,n,S1_mV,S2_mV,S3_mV,S4_mV,S5_mV,S6_mV,S7_mV,sd_mV,d_zero_mV,d_pre_mV\n");
  out("# HOLD,k,label,target_N,t0_us,t1_us,dur_s,early_mV,late_mV,sd_mV,min_mV,max_mV,drift_mV,n,"
      "frame0,frame1,win_s,est_N_prov,flags\n");
  out("# hold_flags,bit1=adc_top,bit2=short_vs_dwell,bit3=no_early_window,bit4=window_cut,bit5=ended_in_hold\n");
}

// =====================================================================
void startRec(const char *kind, int8_t ch, const char *mode) {
  runCh = ch;
  snprintf(runUid, sizeof(runUid), "%08lX-%03u", (unsigned long)bootId, (unsigned)(++fileSeq));
  evN = 0;
  dropped = rows = 0;
  xQueueReset(q);
  if (!strcmp(kind, "RUN")) out("FILE_OPEN,RUN,%d,%s,%u\n", ch + 1, runId, S.rep);
  else out("FILE_OPEN,%s,%d\n", kind, ch + 1);
  writeHeader(mode);
  fileOpen = true;
  logging = true;
}

void stopRec() {
  if (!fileOpen) return;
  logging = false;
  stopReq = true;
  uint32_t t0 = millis();
  while (fileOpen && millis() - t0 < 3000) delay(10);
  if (fileOpen) out("ERR,file did not close within 3 s\n");
}

// Collect n frames into accS/accSS (+min/max of channel ch). false on timeout.
bool collect(uint32_t n, uint8_t ch) {
  accWant = 0;
  for (int i = 0; i < NCH; i++) accS[i] = accSS[i] = 0;
  accCh = ch;
  accN = 0;
  accWant = n;
  uint32_t t0 = millis();
  while (accN < n) {
    if (millis() - t0 > n * PERIOD_MS + 1000) { accWant = 0; return false; }
    delay(10);
  }
  accWant = 0;
  return true;
}

void accStats(int i, uint32_t n, float &mean, float &sd) {
  double m = accS[i] / n;
  double var = accSS[i] / n - m * m;
  mean = m;
  sd = var > 0 ? sqrt(var) : 0;
}

void indSet(bool on) { if (IND_TYPE != IND_NONE) digitalWrite(IND_PIN, on ? HIGH : LOW); }
void indPulse(uint8_t n, uint16_t ms) {
  for (uint8_t i = 0; i < n; i++) { indSet(true); delay(ms); indSet(false); delay(ms); }
}
void indStart() { if (IND_TYPE == IND_LED) indSet(true); else indPulse(1, 80); }
void indEnd(bool warn) {
  indSet(false);
  delay(150);
  if (warn) indPulse(3, 300); else indPulse(2, 80);
}

bool zeroFSR() {
  out("ZERO,keep all FSRs unloaded, measuring 2 s...\n");
  const uint32_t n = 400;
  indStart();
  if (!collect(n, 0)) { out("ERR,zero timed out\n"); indEnd(true); return false; }
  bool ok = true;
  for (uint8_t ch = 0; ch < NCH; ch++) {
    float m, sd;
    accStats(ch, n, m, sd);
    bool good = m > 250 && m < 350;
    out("ZERO,S%u,GPIO%u,mean_mV,%.1f,sd_mV,%.2f,%s\n", ch + 1, FSR_PINS[ch], m, sd,
        good ? "ok" : "OUT OF 250-350 mV: loaded or not connected, kept old");
    if (good) { baseMV[ch] = m; zeroed[ch] = true; } else ok = false;
  }
  indEnd(!ok);
  return ok;
}

float condUS(uint8_t ch, float mv, float base) {
  if (mv - base < DEAD_MV) return 0.0f;
  return (mv / base - 1.0f) / RF_OHM[ch] * 1e6f;
}

float provN(uint8_t ch, float gUS) { return gUS > 0 ? PROV_A[ch] * powf(gUS, PROV_B[ch]) : 0.0f; }

// =====================================================================
// Line input
void flushInput() { while (Serial.available()) Serial.read(); }

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

bool parseForce(const String &s, float &v) {
  if (!s.length()) return false;
  uint8_t dots = 0;
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '.') { if (++dots > 1) return false; }
    else if (!isDigit(c)) return false;
  }
  v = s.toFloat();
  return true;
}

bool abortPending() {
  while (Serial.available()) if (Serial.read() == 'x') return true;
  return false;
}

// =====================================================================
// POINT mode (v2.x): hold timer starts now. 0 ok, 1 cancel, 2 sampler timeout.
int measureStep(uint8_t ch, Meas &m) {
  uint32_t holdMs = (uint32_t)(S.holdS * 1000), avgMs = (uint32_t)(S.avgS * 1000);
  uint32_t t0 = millis();
  out("INFO,holding %.0f s, hands off the setup... (x = cancel)\n", S.holdS);
  indStart();
  const uint32_t nFirst = FIRST_MS / PERIOD_MS, nAvg = avgMs / PERIOD_MS;
  if (!collect(nFirst, ch)) return 2;
  float sd;
  accStats(ch, nFirst, m.first, sd);
  uint32_t avgStart = t0 + holdMs - avgMs;
  while ((int32_t)(millis() - avgStart) < 0) {
    if (abortPending()) return 1;
    delay(20);
  }
  if (!collect(nAvg, ch)) return 2;
  for (uint8_t i = 0; i < NCH; i++) accStats(i, nAvg, m.mean[i], m.sd[i]);
  m.mn = accMin; m.mx = accMax; m.f0 = accF0; m.f1 = accF1; m.flags = accFlags;
  return 0;
}

void measurePoint(float targetN) {
  const uint8_t ch = S.selCh;
  uint8_t redo = 0;
  for (;;) {
    Meas m;
    int r = measureStep(ch, m);
    if (r == 1) { indSet(false); out("INFO,hold cancelled, nothing saved\n"); return; }
    if (r == 2) { indSet(false); out("ERR,sampler timeout, nothing saved, m to retry\n"); return; }

    float g = condUS(ch, m.mean[ch], baseMV[ch]);
    float exc = m.mean[ch] - baseMV[ch];
    float creepPct = exc > DEAD_MV ? (m.mean[ch] - m.first) / exc * 100.0f : 0.0f;
    float xtalk = 0;
    for (uint8_t o = 0; o < NCH; o++)
      if (o != ch && zeroed[o]) xtalk = fmaxf(xtalk, fabsf(m.mean[o] - baseMV[o]));
    out("MEAS,S%u,mean_mV,%.1f,sd_mV,%.2f,min_mV,%u,max_mV,%u,rs_kOhm,%.2f,g_uS,%.2f,"
        "est_N_provisional,%.0f,creep_pct,%.1f\n",
        ch + 1, m.mean[ch], m.sd[ch], m.mn, m.mx, g > 0 ? 1000.0f / g : -1.0f, g, provN(ch, g), creepPct);
    bool sat = m.flags & 0x02;
    if (sat) out("WARN,ADC above %u mV in window: saturating at this force\n", ADC_TOP_MV);
    indEnd(sat);

    float utmN = 0;
    bool redoReq = false, discard = false;
    for (;;) {
      flushInput();
      if (targetN >= 0)
        out("PROMPT,UTM force in N at end of hold (Enter = %.1f, 0 = zero point), a = redo, x = discard\n", targetN);
      else
        out("PROMPT,UTM force in N at end of hold (0 = zero point), a = redo, x = discard\n");
      String in = readLine();
      if (in == "x") { discard = true; break; }
      if (in == "a") { redoReq = true; break; }
      float v;
      if ((!in.length() || in == ".") && targetN >= 0) v = targetN;
      else if (!parseForce(in, v)) { out("WARN,type a number in N, e.g. 102.5\n"); continue; }
      if (v > FULL_SCALE_N * 1.2f) { out("WARN,above 1.2x full scale, retype\n"); continue; }
      if (targetN >= 0 && fabsf(v - targetN) > fmaxf(20.0f, CONFIRM_FRAC * targetN)) {
        out("PROMPT,%.1f N is far from target %.1f N. y = accept, anything else = retype\n", v, targetN);
        if (readLine() != "y") continue;
      }
      utmN = v;
      break;
    }
    if (discard) { out("INFO,point discarded\n"); return; }
    if (redoReq) { redo++; out("INFO,redo: keep the UTM at the force, hold starts now\n"); continue; }

    char dir;
    if (utmN < ZERO_N) dir = 'Z';
    else if (prevN < 0 || utmN > prevN + 0.5f) dir = 'U';
    else if (utmN < prevN - 0.5f) dir = 'D';
    else dir = prevDir;
    bool warn = false;
    if (dir != 'Z' && g <= 0) { out("WARN,no response at load: puck off the sensor or not connected?\n"); warn = true; }
    if (dir == 'Z' && fabsf(exc) > 5.0f) { out("WARN,zero offset %.1f mV (residual load or drift)\n", exc); warn = true; }
    if (warn) {
      String in;
      for (;;) {
        flushInput();
        out("PROMPT,y (or Enter / .) = save anyway, a = redo, x = discard\n");
        in = readLine();
        if (in == "x" || in == "a" || in == "y" || in == "." || !in.length()) break;
      }
      if (in == "x") { out("INFO,point discarded\n"); return; }
      if (in == "a") { redo++; out("INFO,redo: hold starts now\n"); continue; }
    }

    if (dir == 'Z') { if (cycleLoaded) newCyclePending = true; }
    else {
      if (newCyclePending) { calCycle++; newCyclePending = false; }
      cycleLoaded = true;
    }
    prevN = utmN;
    prevDir = dir;
    uint16_t stepNo = calStep++;
    float setN = targetN >= 0 ? targetN : utmN;

    out("CAL,%lu,%u,%u,%u,%c,%.1f,%.2f,%.2f,%.2f,%u,%u,%.2f,%.2f,%.3f,%.3f,%.2f,%u,%u,%lu,%lu\n",
        (unsigned long)millis(), ch + 1, calCycle, stepNo, dir, setN, utmN,
        m.mean[ch], m.sd[ch], m.mn, m.mx, m.first, baseMV[ch],
        g > 0 ? 1000.0f / g : -1.0f, g, xtalk, m.flags, redo, (unsigned long)m.f0, (unsigned long)m.f1);
    calSaved++;
    out("INFO,saved point %u (cycle %u, %c, %.1f N). Set next force then m / m<N>, or save\n",
        calSaved, calCycle, dir, utmN);
    return;
  }
}

void startCalibration() {
  if (!zeroed[S.selCh]) { out("WARN,run z first with the sensor unloaded\n"); return; }
  calOn = true;
  calCycle = 1;
  calStep = calSaved = 0;
  prevN = -1;
  prevDir = 'U';
  cycleLoaded = newCyclePending = false;
  out("INFO,POINT calibration S%u (GPIO%u), RF %.0f, hold %.0f s, avg %.1f s\n", S.selCh + 1,
      FSR_PINS[S.selCh], RF_OHM[S.selCh], S.holdS, S.avgS);
  startRec("UTM", S.selCh, "utm_step");
  out("# protocol,manual,dir and cycle from typed forces\n");
  out("# hold_ms,%lu,first_ms,%lu,avg_ms,%lu\n", (unsigned long)(S.holdS * 1000), (unsigned long)FIRST_MS,
      (unsigned long)(S.avgS * 1000));
  out("# CAL,ms,ch,cycle,step,dir,set_N,utm_N,mean_mV,sd_mV,min_mV,max_mV,first_mV,base_mV,"
      "rs_kOhm,g_uS,xtalk_mV,flags,redo,frame0,frame1\n");
  out("INFO,set the UTM force, wait until stable, then m (or m<N>). save = finish channel\n");
}

// =====================================================================
// RUN mode
void ev(const char *type, uint64_t t_us, const char *text) {
  Frame fr = getLatest();
  int64_t df = ((int64_t)fr.t_us - (int64_t)t_us) / (int64_t)(PERIOD_MS * 1000);
  long frame = (long)fr.idx - (long)df;
  evN++;
  out("EV,%s.E%02u,%llu,%ld,%s,%s\n", runUid, evN, (unsigned long long)t_us, frame, type, text);
}

// baseline of all channels over S.baseS seconds; returns false if the run channel is not unloaded
bool baseline(const char *kind, float *meanOut) {
  uint8_t ch = runCh;
  uint32_t n = (uint32_t)(S.baseS * 1000) / PERIOD_MS;
  ev(!strcmp(kind, "pre") ? "BASE_PRE_START" : "BASE_POST_START", getLatest().t_us, "");
  if (!collect(n, ch)) { out("ERR,baseline timed out\n"); return false; }
  float m[NCH], sd[NCH];
  for (uint8_t i = 0; i < NCH; i++) accStats(i, n, m[i], sd[i]);
  float dz = m[ch] - baseMV[ch];
  float dp = !strcmp(kind, "post") ? m[ch] - preBase[ch] : 0;
  char dpS[16] = "";
  if (!strcmp(kind, "post")) snprintf(dpS, sizeof(dpS), "%.2f", dp);
  out("BASE,%s,%llu,%llu,%lu,%lu,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%s\n", kind,
      (unsigned long long)accT0, (unsigned long long)accT1, (unsigned long)accF0, (unsigned long)accF1,
      (unsigned long)n, m[0], m[1], m[2], m[3], m[4], m[5], m[6], sd[ch], dz, dpS);
  ev(!strcmp(kind, "pre") ? "BASE_PRE_END" : "BASE_POST_END", accT1, "");
  if (meanOut) for (uint8_t i = 0; i < NCH; i++) meanOut[i] = m[i];
  bool ok = m[ch] > 250 && m[ch] < 350 && fabsf(dz) <= 5.0f && sd[ch] < 5.0f;
  if (!strcmp(kind, "post")) ok = ok && fabsf(dp) <= 5.0f;
  out("INFO,%s-baseline S%u: %.1f mV (sd %.2f), %+.1f mV vs zero%s%s -> %s\n", kind, ch + 1, m[ch], sd[ch], dz,
      !strcmp(kind, "post") ? ", vs pre " : "", dpS, ok ? "OK" : "CHECK");
  return ok;
}

void stopTracking() {
  trackOn = false;
  delay(5);
  xQueueReset(blockQ);
}

void beginRun(String arg) {
  if (fileOpen || calOn) { out("WARN,a file is open, end / save first\n"); return; }
  uint8_t ch = S.selCh;
  if (!zeroed[ch]) { out("WARN,run z first with all sensors unloaded\n"); return; }
  int idx = -1;
  arg.trim();
  if (!arg.length()) {
    for (uint8_t i = 0; i < nPlan; i++) if (!isDone(ch, i)) { idx = i; break; }
    if (idx < 0) { out("INFO,all %u plan runs done for S%u repeat %u (set rep <n> for the next repeat)\n", nPlan, ch + 1, S.rep); return; }
  } else {
    if (arg.length() > 12) { out("WARN,run ID max 12 characters\n"); return; }
    for (size_t i = 0; i < arg.length(); i++)
      if (!isalnum(arg[i]) && arg[i] != '_' && arg[i] != '-') { out("WARN,run ID: letters, digits, _ or - only\n"); return; }
    idx = findPlan(arg.c_str());
  }
  float restS = lastRunEndMs ? (millis() - lastRunEndMs) / 1000.0f : -1;
  if (restS >= 0 && restS < S.restS) {
    flushInput();
    out("PROMPT,only %.0f s since the last run (rest %.0f s). y = start anyway, anything else = cancel\n", restS, S.restS);
    if (readLine() != "y") { out("INFO,run cancelled\n"); return; }
  }
  if (idx >= 0 && isDone(ch, idx)) {
    flushInput();
    out("PROMPT,%s is already done for S%u repeat %u. y = record again, anything else = cancel\n", plan[idx].id, ch + 1, S.rep);
    if (readLine() != "y") { out("INFO,run cancelled\n"); return; }
  }

  runPlanIdx = idx;
  if (idx >= 0) strncpy(runId, plan[idx].id, sizeof(runId) - 1);
  else { strncpy(runId, arg.c_str(), sizeof(runId) - 1); for (char *p = runId; *p; p++) *p = toupper(*p); }
  runId[sizeof(runId) - 1] = 0;
  holdsSeen = 0; nearWarned = clipWarned = chanWarned = false;

  // tracker settings for this run
  trk.avgS = S.avgS;
  trk.dwellS = S.dwellS;
  trk.flatMVps = fmaxf(2.5f, 0.25f * S.rampNps * 3.0f);   // ramps are ~3 mV/N x rate

  const char *tmpl = idx < 0 ? "FREE" : (plan[idx].kind == 'U' ? "UP" : "DN");
  float t1 = idx >= 0 ? plan[idx].t1 : 0, t2 = idx >= 0 ? plan[idx].t2 : 0;
  out("INFO,=== RUN %s  channel S%u (GPIO%u, site %s, RF %.0f)  repeat %u ===\n", runId, ch + 1, FSR_PINS[ch],
      SITE_OF_CH[ch], RF_OHM[ch], S.rep);
  startRec("RUN", ch, "run");
  writeRunHeader(tmpl, t1, t2, restS);
  runOn = true;
  runStartMs = millis();
  ev("RUN_START", getLatest().t_us, runId);

  out("INFO,puck clear of the sensor? measuring pre-baseline %.0f s...\n", S.baseS);
  bool ok = baseline("pre", preBase);
  if (!ok) {
    flushInput();
    out("PROMPT,pre-baseline not recovered / loaded. y = continue anyway, x = abort run\n");
    String in = readLine();
    if (in != "y") {
      ev("ABORT", getLatest().t_us, "pre-baseline");
      out("# end,run,%s,status,ABORTED\n", runId);
      runOn = false;
      stopRec();
      out("INFO,run aborted, file kept. Wait, z if needed, then run again\n");
      return;
    }
    out("# note,%lu,pre-baseline accepted by operator\n", (unsigned long)millis());
  }
  trk.reset(preBase[ch]);
  trackCh = ch;
  xQueueReset(blockQ);
  trackOn = true;

  if (idx >= 0) {
    char a[160];
    areaText(plan[idx], a, sizeof(a));
    out("INFO,TRAPEZIUM method %s: %s. Rate %.1f N/s. Change points: ramps end on Sensor/Force, holds on Duration\n",
        plan[idx].kind == 'U' ? "LOADING" : "DESCENDING", a, S.rampNps);
  }
  out("INFO,recording. Start the UTM now and type k (+Enter) when you click Start. end = finish after the UTM returned, x = abort\n");
}

// one on-board hold -> HOLD row + screen line
void reportHold() {
  uint8_t ch = runCh;
  const HoldInfo &h = trk.last;
  holdsSeen = h.k;
  char label = 'H';
  float target = 0;
  if (runPlanIdx >= 0) {
    const PlanRun &r = plan[runPlanIdx];
    if (h.k == 1) { label = r.kind == 'U' ? 'U' : 'P'; target = r.t1; }
    else if (h.k == 2) { label = r.kind == 'U' ? 'U' : 'D'; target = r.t2; }
    else label = 'X';
  }
  float estN = provN(ch, condUS(ch, h.late.mean, trk.base));
  char t[24];
  snprintf(t, sizeof(t), "hold %d", h.k);
  ev("HOLD_END", h.t1, t);
  out("HOLD,%d,%c,%.0f,%llu,%llu,%.2f,%.2f,%.2f,%.2f,%u,%u,%.2f,%lu,%lu,%lu,%.2f,%.0f,%u\n", h.k, label, target,
      (unsigned long long)h.t0, (unsigned long long)h.t1, h.dur, h.earlyOk ? h.early : 0.0f, h.late.mean,
      h.late.sd, h.late.mn, h.late.mx, h.drift, (unsigned long)h.late.n, (unsigned long)h.late.f0,
      (unsigned long)h.late.f1, h.late.n * (PERIOD_MS / 1000.0f), estN, h.flags);
  bool good = !(h.flags & 0x16);
  out("INFO,hold %d (%c %.0f N): %.1f s, last %.1f s %.1f mV sd %.2f (~%.0f N prov), drift %+.1f mV since %.0f-%.0f s -> %s%s%s%s\n",
      h.k, label, target, h.dur, h.late.n * (PERIOD_MS / 1000.0f), h.late.mean, h.late.sd, estN, h.drift,
      EARLY_FROM_S, EARLY_TO_S, good ? "OK" : "CHECK", (h.flags & 2) ? " clipped" : "",
      (h.flags & 4) ? " short" : "", (h.flags & 0x10) ? " window cut" : "");
}

// tracker events -> file rows and screen
void handleBlocks() {
  Block b;
  while (xQueueReceive(blockQ, &b, 0) == pdTRUE) {
    if (!trackOn || !runOn) continue;
    int e = trk.push(b);
    uint8_t ch = runCh;
    if (b.mx > NEAR_MV && !nearWarned) { out("WARN,S%u above %u mV: near the ADC rail\n", ch + 1, NEAR_MV); nearWarned = true; }
    if ((b.flags & 0x02) && !clipWarned) { out("WARN,S%u above %u mV: CLIPPED, this force is out of range for RF %.0f\n", ch + 1, ADC_TOP_MV, RF_OHM[ch]); clipWarned = true; }
    if (e & TE_ONSET) { ev("ONSET", trk.onsetT, ""); out("INFO,load detected on S%u\n", ch + 1); }
    if (e & TE_HOLD_START) {
      char t[24];
      snprintf(t, sizeof(t), "hold %u", holdsSeen + 1);
      ev("HOLD_START", trk.holdT0, t);
      float m = Tracker::bmean(b);
      out("INFO,hold %u started: %.0f mV (~%.0f N provisional)\n", holdsSeen + 1, m, provN(ch, condUS(ch, m, trk.base)));
    }
    if (!chanWarned) {               // another channel loaded much more than the selected one?
      float exc = b.chMean[ch] - preBase[ch];
      for (uint8_t o = 0; o < NCH && !chanWarned; o++) {
        float oe = b.chMean[o] - preBase[o];
        if (o != ch && oe > 50.0f && oe > 3.0f * fmaxf(exc, 10.0f)) {
          out("WARN,S%u is loaded (+%.0f mV) but the selected S%u is not (+%.0f mV): WRONG CHANNEL or puck off the sensor? "
              "x = abort, then s%u\n", o + 1, oe, ch + 1, exc, o + 1);
          ev("WRONG_CHANNEL?", b.t_us, "");
          chanWarned = true;
        }
      }
    }
    if (e & TE_HOLD_END) reportHold();
    if (e & TE_UNLOAD) {
      ev("UNLOAD", trk.unloadT, "");
      out("INFO,S%u unloaded (%u hold(s)). Type end when the UTM has returned (recording continues)\n", ch + 1, holdsSeen);
    }
  }
}

void endRun(bool aborted) {
  if (!runOn) return;
  if (!aborted) {
    Frame fr = getLatest();
    float exc = (float)fr.mv[runCh] - preBase[runCh];
    if (exc > 20.0f) {
      flushInput();
      out("PROMPT,S%u still loaded (%+.0f mV vs pre-baseline). Wait until the UTM has returned. y = end anyway (post-baseline under load), anything else = keep recording\n",
          runCh + 1, exc);
      if (readLine() != "y") { out("INFO,still recording. Type end when unloaded\n"); return; }
    }
  }
  // let the tracker see the last blocks, report a hold that is still open
  uint32_t t0 = millis();
  while (uxQueueMessagesWaiting(blockQ) && millis() - t0 < 200) delay(5);
  handleBlocks();
  if (trk.flush()) { out("WARN,run ended during a hold: last hold reported up to now (flag bit5)\n"); reportHold(); }
  stopTracking();
  uint8_t expect = runPlanIdx >= 0 ? 2 : 0;
  bool ok = !aborted;
  if (aborted) {
    ev("ABORT", getLatest().t_us, "operator");
  } else {
    out("INFO,post-baseline %.0f s (UTM returned, puck clear)...\n", S.baseS);
    bool bok = baseline("post", nullptr);
    if (!bok) ok = false;
    if (expect && holdsSeen != expect) {
      out("WARN,%u hold(s) detected, %u expected: check the method / the export\n", holdsSeen, expect);
      ok = false;
    }
  }
  ev("RUN_END", getLatest().t_us, aborted ? "aborted" : (ok ? "ok" : "check"));
  out("# end,run,%s,holds,%u,expected,%u,status,%s\n", runId, holdsSeen, expect,
      aborted ? "ABORTED" : (ok ? "OK" : "CHECK"));
  runOn = false;
  stopRec();
  lastRunEndMs = millis();
  strncpy(lastRunId, runId, sizeof(lastRunId) - 1);
  lastRunPeak = runPlanIdx >= 0 ? fmaxf(plan[runPlanIdx].t1, plan[runPlanIdx].t2) : 0;
  if (runPlanIdx >= 0 && ok) { S.done[runCh] |= 1u << runPlanIdx; saveSettings(); }
  out("INFO,run %s %s. %s\n", runId, aborted ? "ABORTED (file kept)" : (ok ? "OK, marked done" : "CHECK (not marked done; done <ID> to accept)"),
      "Rest, then run for the next one (plan = list)");
}

void printStat() {
  Frame fr = getLatest();
  uint8_t ch = runCh;
  const char *st = trk.state == 0 ? "unloaded" : (trk.state == 1 ? "moving" : (trk.state == 2 ? "HOLD" : "leaving hold"));
  float g = condUS(ch, fr.mv[ch], trk.base);
  char age[24] = "";
  if (trk.state >= 2) snprintf(age, sizeof(age), ",%.0f s", (double)(fr.t_us - trk.holdT0) / 1e6);
  out("STAT,%s,%.0f s,S%u,%u mV,~%.0f N prov,%s%s,slope %+.1f mV/s\n", runId, (millis() - runStartMs) / 1000.0f,
      ch + 1, fr.mv[ch], provN(ch, g), st, age, trk.slope);
}

// =====================================================================
void saveAndNext() {
  if (runOn) { endRun(false); return; }
  bool wasCal = calOn;
  if (!wasCal && !fileOpen) { out("WARN,nothing open to save\n"); return; }
  if (wasCal) out("# end,points,%u,cycles,%u\n", calSaved, calCycle);
  stopRec();
  calOn = false;
  if (!wasCal) return;
  out("INFO,S%u done: %u points, %u cycle(s)\n", S.selCh + 1, calSaved, calCycle);
  if (S.selCh + 1 < NCH) {
    S.selCh++;
    saveSettings();
    out("INFO,next channel S%u selected (s<n> to change). Move the puck, unload, z, then start\n", S.selCh + 1);
  } else {
    out("INFO,S7 was the last channel\n");
  }
}

void printSettings() {
  out("INFO,set p %.0f | step %.0f | dwell %.1f | ramp %.2f | rest %.0f | rep %u | hold %.1f | avg %.1f | base %.1f\n",
      S.pN, S.stepN, S.dwellS, S.rampNps, S.restS, S.rep, S.holdS, S.avgS, S.baseS);
  out("INFO,(p,step: N; dwell,rest,hold,avg,base: s; ramp: N/s; rep: repeat number; hold/avg = point mode)\n");
}

void doSet(String a) {
  a.trim();
  if (!a.length()) { printSettings(); return; }
  int sp = a.indexOf(' ');
  if (sp < 0) { out("WARN,use set <key> <value>, e.g. set p 800\n"); return; }
  String k = a.substring(0, sp), v = a.substring(sp + 1);
  v.trim();
  float x;
  if (!parseForce(v, x)) { out("WARN,value must be a plain number\n"); return; }
  bool replan = false, clearDone = false;
  if (k == "p") { if (x < 100 || x > 1500) { out("WARN,p 100..1500 N\n"); return; } S.pN = x; replan = clearDone = true; }
  else if (k == "step") { if (x < 25 || x > 500) { out("WARN,step 25..500 N\n"); return; } S.stepN = x; replan = clearDone = true; }
  else if (k == "dwell") { if (x < 5 || x > 600) { out("WARN,dwell 5..600 s\n"); return; } S.dwellS = x; }
  else if (k == "ramp") { if (x < 0.5f || x > 200) { out("WARN,ramp 0.5..200 N/s\n"); return; } S.rampNps = x; }
  else if (k == "rest") { if (x > 3600) { out("WARN,rest 0..3600 s\n"); return; } S.restS = x; }
  else if (k == "rep") { if (x < 1 || x > 99) { out("WARN,rep 1..99\n"); return; } S.rep = (uint16_t)x; clearDone = true; }
  else if (k == "hold") { if (x < 3 || x > 600) { out("WARN,hold 3..600 s\n"); return; } S.holdS = x; }
  else if (k == "avg") { if (x < 0.5f || x > 10) { out("WARN,avg 0.5..10 s\n"); return; } S.avgS = x; }
  else if (k == "base") { if (x < 1 || x > 30) { out("WARN,base 1..30 s\n"); return; } S.baseS = x; }
  else { out("WARN,unknown key (p step dwell ramp rest rep hold avg base)\n"); return; }
  int ng = (int)floorf(S.pN / S.stepN + 0.01f);
  if (ng < 2 || ng > 21) {
    out("WARN,p / step gives %d grid points (2..21 allowed), setting not changed\n", ng);
    loadSettings();       // back to the stored values
    return;
  }
  if (S.holdS < S.avgS + 1.5f) S.holdS = S.avgS + 1.5f;
  if (clearDone) { for (uint8_t i = 0; i < NCH; i++) S.done[i] = 0; out("INFO,done list cleared (new plan / repeat)\n"); }
  if (replan) buildPlan();
  saveSettings();
  printSettings();
}

void doMeta(String a) {
  a.trim();
  if (!a.length()) {
    out("INFO,metadata (written to every file header):\n");
    for (uint8_t i = 0; i < NMETA; i++) if (meta[i].k[0]) out("INFO,  %s = %s\n", meta[i].k, meta[i].v);
    out("INFO,suggested keys: foot board sensor site fixture puck cell cell_N vref_mV operator temp_C method\n");
    return;
  }
  int sp = a.indexOf(' ');
  String k = sp < 0 ? a : a.substring(0, sp), v = sp < 0 ? String("") : a.substring(sp + 1);
  k.toLowerCase();
  v.trim();
  v.replace(",", ";");
  if (k == "clear") { memset(meta, 0, sizeof(meta)); saveMeta(); out("INFO,metadata cleared\n"); return; }
  if (k.length() >= sizeof(meta[0].k)) { out("WARN,key max %u characters\n", (unsigned)sizeof(meta[0].k) - 1); return; }
  for (size_t i = 0; i < k.length(); i++) if (!isalnum(k[i]) && k[i] != '_') { out("WARN,key: letters, digits, _\n"); return; }
  if (v.length() >= sizeof(meta[0].v)) { out("WARN,value max %u characters\n", (unsigned)sizeof(meta[0].v) - 1); return; }
  int slot = -1, empty = -1;
  for (uint8_t i = 0; i < NMETA; i++) {
    if (meta[i].k[0] && !strcasecmp(meta[i].k, k.c_str())) slot = i;
    if (!meta[i].k[0] && empty < 0) empty = i;
  }
  if (!v.length()) {
    if (slot >= 0) { meta[slot].k[0] = 0; saveMeta(); out("INFO,meta %s deleted\n", k.c_str()); }
    return;
  }
  if (slot < 0) slot = empty;
  if (slot < 0) { out("WARN,metadata full (%u keys), delete one\n", NMETA); return; }
  strncpy(meta[slot].k, k.c_str(), sizeof(meta[slot].k) - 1);
  strncpy(meta[slot].v, v.c_str(), sizeof(meta[slot].v) - 1);
  saveMeta();
  out("INFO,meta %s = %s\n", meta[slot].k, meta[slot].v);
}

void doDone(String a, bool set) {
  a.trim();
  int i = findPlan(a.c_str());
  if (i < 0) { out("WARN,unknown plan run '%s' (plan = list)\n", a.c_str()); return; }
  if (set) S.done[S.selCh] |= 1u << i; else S.done[S.selCh] &= ~(1u << i);
  saveSettings();
  out("INFO,%s %s for S%u repeat %u\n", plan[i].id, set ? "marked done" : "marked not done", S.selCh + 1, S.rep);
}

void printHelp() {
  out("--- %s ---\n", FW_VERSION);
  out("RUN mode:   plan | run [ID] | k[text] mark | end | x abort\n");
  out("POINT mode: start | m / m<N> | save\n");
  out("z zero | s<n> channel | n<text> note | set [key value] | meta [key value] | done/undo <ID> | r raw | l live | i | ?\n");
  out("INFO,selected S%u (site %s, RF %.0f), repeat %u, note '%s', file %s\n", S.selCh + 1, SITE_OF_CH[S.selCh],
      RF_OHM[S.selCh], S.rep, note, runOn ? "RUN open" : (calOn ? "POINT open" : (fileOpen ? "RAW open" : "closed")));
  printSettings();
}

void printLive() {
  Frame fr = getLatest();
  uint8_t ch = S.selCh;
  float g = condUS(ch, fr.mv[ch], baseMV[ch]);
  out("LIVE,S%u,mV,%u,kOhm,%.1f,uS,%.2f,N_prov,%.0f%s\n", ch + 1, fr.mv[ch], g > 0 ? 1000.0f / g : -1.0f, g,
      provN(ch, g), fr.mv[ch] > ADC_TOP_MV ? ",ADC_TOP" : "");
}

void handleCommand(String cmd) {
  cmd.trim();
  if (!cmd.length()) return;
  String lc = cmd;
  lc.toLowerCase();
  int sp = lc.indexOf(' ');
  String word = sp < 0 ? lc : lc.substring(0, sp);
  String rest = sp < 0 ? String("") : cmd.substring(sp + 1);
  char c = lc[0];
  bool rec = fileOpen || calOn || runOn;

  if (word == "start") {
    if (rec) { out("WARN,a file is open, end / save first\n"); return; }
    startCalibration();
  } else if (word == "save" || word == "end") {
    saveAndNext();
  } else if (word == "run") {
    beginRun(rest);
  } else if (word == "plan") {
    printPlan();
  } else if (word == "set") {
    if (rec && rest.length()) { out("WARN,end / save first, then change settings\n"); return; }
    doSet(sp < 0 ? String("") : lc.substring(sp + 1));
  } else if (word == "meta") {
    if (rec && rest.length()) { out("WARN,end / save first (metadata goes into the header)\n"); return; }
    String r = rest; r.trim();
    doMeta(r);
  } else if (word == "done" || word == "undo") {
    if (rec) { out("WARN,end / save first\n"); return; }
    doDone(rest, word == "done");
  } else if (lc == "x") {
    if (runOn) endRun(true);
    else out("WARN,no run open (x cancels a run; inside a point hold it cancels the hold)\n");
  } else if (c == 'k') {
    if (!fileOpen) { out("WARN,no file open\n"); return; }
    String t = cmd.substring(1);
    t.trim();
    t.replace(",", ";");
    ev("MARK", getLatest().t_us, t.length() ? t.c_str() : "mark");
    out("INFO,mark E%02u at %.2f s\n", evN, (millis() - runStartMs) / 1000.0f);
  } else if (c == 's') {
    if (rec) { out("WARN,end / save first, then change channel\n"); return; }
    int n = lc.substring(1).toInt();
    if (n < 1 || n > NCH) { out("WARN,channel s1..s7\n"); return; }
    S.selCh = n - 1;
    saveSettings();
    out("INFO,selected S%u (GPIO%u, site %s, RF %.0f)\n", S.selCh + 1, FSR_PINS[S.selCh], SITE_OF_CH[S.selCh], RF_OHM[S.selCh]);
  } else if (c == 'm') {
    if (!calOn) { out("WARN,start first (point mode). For machine runs use run\n"); return; }
    float target = -1;
    String t = lc.substring(1);
    t.trim();
    if (t.length() && !parseForce(t, target)) { out("WARN,use m or m<N>, e.g. m100\n"); return; }
    bool wasLive = liveOn;
    liveOn = false;
    measurePoint(target);
    liveOn = wasLive;
  } else if (c == 'z') {
    if (rec) { out("WARN,end / save first (zero must not change inside a file)\n"); return; }
    zeroFSR();
  } else if (c == 'n') {
    String t = cmd.substring(1);
    t.trim();
    t.replace(",", ";");
    if (fileOpen) out("# note,%lu,%s\n", (unsigned long)millis(), t.c_str());
    else { strncpy(note, t.c_str(), sizeof(note) - 1); out("INFO,note '%s'\n", note); }
  } else if (lc == "r") {
    if (rec) { out("WARN,a file is open, end / save first\n"); return; }
    startRec("RAW", -1, "free");
    out("INFO,recording (RAW) to the PC, save to stop\n");
  } else if (lc == "l") {
    liveOn = !liveOn;
  } else if (lc == "i") {
    out("INFO,indicator: start, done, warning\n");
    indStart(); delay(1000); indEnd(false); delay(800); indStart(); delay(500); indEnd(true);
  } else if (c == '?') {
    printHelp();
  } else {
    out("WARN,unknown command '%s', ? for help\n", cmd.c_str());
  }
}

// =====================================================================
void setup() {
  outLock = xSemaphoreCreateMutex();
  Serial.setTxBufferSize(4096);
  Serial.begin(SERIAL_BAUD);
  while (!Serial && millis() < 3000) {}
  out("%s\n", FW_VERSION);

  prefs.begin("utmcal", false);
  loadSettings();
  buildPlan();
  bootId = esp_random();

  if (IND_TYPE != IND_NONE) { pinMode(IND_PIN, OUTPUT); indSet(false); }
  analogReadResolution(12);
  for (uint8_t ch = 0; ch < NCH; ch++) {
    analogSetPinAttenuation(FSR_PINS[ch], ADC_11db);
    baseMV[ch] = 300.0f;
  }

  q = xQueueCreate(QUEUE_LEN, sizeof(Frame));
  blockQ = xQueueCreate(64, sizeof(Block));
  if (!q || !blockQ || !outLock) { Serial.println("ERR,alloc failed"); while (true) delay(1000); }
  xTaskCreate(samplerTask, "sampler", 4096, nullptr, 3, nullptr);
  xTaskCreate(writerTask, "writer", 4096, nullptr, 2, nullptr);

  delay(200);
  zeroFSR();
  printHelp();
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n') { handleCommand(rx); rx = ""; }
    else if (ch != '\r') rx += ch;
  }
  handleBlocks();
  uint32_t now = millis();
  if (runOn && now - lastStat >= STAT_MS) { lastStat = now; printStat(); }
  if (liveOn && now - lastLive >= LIVE_MS) { lastLive = now; printLive(); }
  delay(2);
}
