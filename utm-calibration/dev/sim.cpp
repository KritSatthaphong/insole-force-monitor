// Host simulation of FSR_UTM_Cal.ino: pty serial, simulated FSR front end + Shimadzu force profile.
#include "Arduino.h"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <random>
#include <atomic>
SerialStub Serial;
static std::chrono::steady_clock::time_point T0 = std::chrono::steady_clock::now();
static double SPEED = 5;
double sim_now_s() { return std::chrono::duration<double>(std::chrono::steady_clock::now() - T0).count() * SPEED; }
void sim_sleep_s(double s) { if (s > 0) std::this_thread::sleep_for(std::chrono::duration<double>(s / SPEED)); }
static int mfd = -1;
static std::mutex inM; static std::deque<int> inQ;
int sim_serial_avail() { std::lock_guard<std::mutex> l(inM); return (int)inQ.size(); }
int sim_serial_read() { std::lock_guard<std::mutex> l(inM); if (inQ.empty()) return -1; int c = inQ.front(); inQ.pop_front(); return c; }

// ---------------- UTM profile ----------------
struct Seg { double t, f; };
static std::mutex pm;
static std::vector<Seg> prof;           // UTM time (s) -> force
static std::atomic<double> utmStart{-1};
static std::atomic<bool> utmArmed{false};
static std::string simRunId; static int simRunN = 0;
static double SIM_DELAY = 0.35;         // operator: k pressed, UTM starts this much later
static int SIM_CH = 0;
static double profF(double t) {
  std::lock_guard<std::mutex> l(pm);
  if (prof.empty() || t <= prof[0].t) return 0;
  for (size_t i = 1; i < prof.size(); i++) if (t <= prof[i].t) { double a = (t - prof[i-1].t) / (prof[i].t - prof[i-1].t); return prof[i-1].f + a * (prof[i].f - prof[i-1].f); }
  return 0;
}
static double profEnd() { std::lock_guard<std::mutex> l(pm); return prof.empty() ? 0 : prof.back().t; }
static void setProfile(const std::string& id) {
  double t1 = 0, t2 = 0;
  if (id.size() == 4 && id[0] == 'U') { int n = atoi(id.c_str() + 2); t1 = (2 * n - 1) * 100; t2 = 2 * n * 100; }
  else if (id.size() == 4 && id[0] == 'D') { int n = atoi(id.c_str() + 2); t1 = 1000; t2 = 1000 - n * 100; }
  else { t1 = 300; t2 = 500; }
  double r = 10, t = 2;                 // 2 s approach
  std::vector<Seg> p{{0, 0}, {t, 0}};
  t += t1 / r; p.push_back({t, t1}); t += 30; p.push_back({t, t1});
  t += fabs(t2 - t1) / r; p.push_back({t, t2}); t += 30; p.push_back({t, t2});
  t += t2 / 200; p.push_back({t, 0}); t += 3; p.push_back({t, 0});
  std::lock_guard<std::mutex> l(pm); prof = p;
}
// overshoot at hold entry and force noise (UTM side)
static std::mt19937 rng(7);
static double utmForce(double tu) {
  double F = profF(tu);
  std::lock_guard<std::mutex> l(pm);
  for (size_t i = 2; i + 1 < prof.size(); i++) if (prof[i].f == prof[i+1].f && prof[i].f > 0 && tu > prof[i].t && tu < prof[i+1].t) F += 0.012 * prof[i].f * exp(-(tu - prof[i].t) / 0.8);
  return F;
}
// ---------------- FSR model ----------------
static double fsrG(double tu) {          // conductance (uS) of the loaded sensor at UTM time tu
  double F = utmForce(tu);
  if (F <= 0.2) return 0;
  double Fp = 0, tch = 0;
  { std::lock_guard<std::mutex> l(pm);
    for (size_t i = 1; i < prof.size(); i++) { if (prof[i-1].t > tu) break; double te = std::min(tu, prof[i].t); double f1 = prof[i-1].f + (te - prof[i-1].t) / (prof[i].t - prof[i-1].t) * (prof[i].f - prof[i-1].f); Fp = std::max(Fp, std::max(prof[i-1].f, f1)); if (prof[i].f != prof[i-1].f) tch = te; } }
  double G = pow(F / 3.2, 1 / 1.02);
  if (F < Fp - 1) G *= 1 + 0.10 * (1 - F / Fp);                 // unloading reads higher
  double tau = std::max(0.0, tu - tch);
  G *= 1 + 0.010 * log10(1 + tau / 0.3);                         // creep
  return G;
}
static const double BASE[7] = {294.6, 296.1, 292.3, 297.0, 291.8, 299.2, 295.5};
uint16_t sim_adc_mv(uint8_t pin) {
  static std::normal_distribution<double> nd(0, 2.0);
  static std::mutex rm;
  double us = utmStart.load(), tu = us >= 0 ? sim_now_s() - us : -1;
  double G = tu >= 0 ? fsrG(tu) : 0;
  double exc = BASE[SIM_CH] * 39000 * G * 1e-6;
  double mv = BASE[pin] + (pin == SIM_CH ? exc : 0.002 * exc);
  { std::lock_guard<std::mutex> l(rm); mv += nd(rng); }
  if (mv > 3300) mv = 3300 + (mv - 3300) * 0.002;
  return (uint16_t)std::max(0.0, mv);
}
// ---------------- UTM export (Shimadzu-like csv, 20 Hz) ----------------
static void utmThread(std::string id, int n) {
  double us; while ((us = utmStart.load()) < 0) sim_sleep_s(0.01);
  std::vector<std::string> rows; std::normal_distribution<double> nd(0, 0.25);
  double end = profEnd(); char b[96];
  for (int k = 0;; k++) { double tu = k * 0.05; while (sim_now_s() - us < tu) sim_sleep_s(0.005); if (tu > end) break;
    double F = utmForce(tu) + nd(rng); snprintf(b, sizeof b, "%.2f,%.4f,%.4f", tu, F, F / 4000.0); rows.push_back(b); }
  char fn[64]; snprintf(fn, sizeof fn, "utm_export_%s_%d.csv", id.c_str(), n);
  FILE* f = fopen(fn, "w");
  fprintf(f, "\"Test Name\",\"sim %s\"\n\"Batch\",\"1\"\n\n\"Time\",\"Force\",\"Stroke\"\n\"sec\",\"N\",\"mm\"\n", id.c_str());
  for (auto& r : rows) fprintf(f, "%s\n", r.c_str());
  fclose(f);
  fprintf(stderr, "[sim] wrote %s (%zu rows)\n", fn, rows.size());
}
static std::string outLine;
void sim_serial_write(const uint8_t* b, size_t n) {
  for (size_t i = 0; i < n; i++) {
    char c = b[i];
    if (c == '\n') {
      if (outLine.rfind("FILE_OPEN,RUN,", 0) == 0) {
        char id[16] = ""; sscanf(outLine.c_str(), "FILE_OPEN,RUN,%*d,%15[^,]", id);
        simRunId = id; simRunN++; utmStart = -1; setProfile(simRunId); utmArmed = true;
        std::thread(utmThread, simRunId, simRunN).detach();
      }
      outLine.clear();
    } else outLine += c;
  }
  size_t off = 0;
  while (off < n) { ssize_t w = ::write(mfd, b + off, n - off); if (w > 0) off += w; else std::this_thread::sleep_for(std::chrono::microseconds(200)); }
}
static void inputThread() {
  std::string line; char c;
  for (;;) { ssize_t r = ::read(mfd, &c, 1);
    if (r == 1) { { std::lock_guard<std::mutex> l(inM); inQ.push_back((uint8_t)c); }
      if (c == '\n') { if (!line.empty() && (line[0] == 'k' || line[0] == 'K') && utmArmed && utmStart < 0) { double t = sim_now_s() + SIM_DELAY; utmStart = t; fprintf(stderr, "[sim] UTM started at board t=%.3f s\n", t); } line.clear(); }
      else if (c != '\r') line += c; }
    else std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
}
#include "sketch.inc"
int main() {
  if (getenv("SIM_SPEED")) SPEED = atof(getenv("SIM_SPEED"));
  if (getenv("SIM_CH")) SIM_CH = atoi(getenv("SIM_CH")) - 1;
  if (getenv("SIM_DELAY")) SIM_DELAY = atof(getenv("SIM_DELAY"));
  mfd = posix_openpt(O_RDWR | O_NOCTTY); grantpt(mfd); unlockpt(mfd);
  struct termios tio; int sfd = open(ptsname(mfd), O_RDWR | O_NOCTTY); tcgetattr(sfd, &tio); cfmakeraw(&tio); tcsetattr(sfd, TCSANOW, &tio);
  FILE* pf = fopen("pty_name", "w"); fprintf(pf, "%s\n", ptsname(mfd)); fclose(pf);
  fprintf(stderr, "[sim] pty %s\n", ptsname(mfd));
  std::thread(inputThread).detach();
  setup();
  for (;;) loop();
}
