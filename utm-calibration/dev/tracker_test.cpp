#include <cstdint>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>
const uint8_t NCH = 7; const uint32_t PERIOD_MS = 5; const uint8_t BLOCK_N = 20;
const float ONSET_MV = 8.0f, CONFIRM_S = 1.0f, MIN_HOLD_S = 5.0f, EARLY_FROM_S = 3.0f, EARLY_TO_S = 5.0f, GUARD_S = 0.7f;
#include "tracker.inc"
// force profile: list of (time, force) segments, linear interpolation
struct P { double t, f; };
double prof(const std::vector<P>& p, double t){ if(t<=p[0].t) return p[0].f; for(size_t i=1;i<p.size();i++) if(t<=p[i].t){double a=(t-p[i-1].t)/(p[i].t-p[i-1].t); return p[i-1].f+a*(p[i].f-p[i-1].f);} return p.back().f; }
int run(const char* name, std::vector<P> p, double creep_mVps, double noise, double clipAt=3300, float flat=7.5f, bool logc=false){
  Tracker t; t.reset(300); t.dwellS=30; t.flatMVps=flat; t.avgS=2;
  std::mt19937 g(1); std::normal_distribution<double> nd(0,noise);
  double T = p.back().t + 10; Block b{}; int bn=0; uint32_t idx=0; int nh=0;
  printf("== %s\n", name);
  double holdStart=-1;
  for (double tt=0; tt<T; tt+=0.005, idx++){
    double F = prof(p, tt);
    // 2.8 mV/N, creep: grows after each change (crude): use derivative sign memory
    double mv = 300 + 2.8*F + nd(g);
    static double lastF=0, since=0; if (fabs(F-lastF)>1e-6) since=0; else since+=0.005; lastF=F;
    if (F>1) mv += logc ? 0.012*(mv-300)*log10(1+since/0.2) : creep_mVps*std::min(since,30.0);
    if (mv>clipAt) mv=clipAt; uint16_t v = (uint16_t)lround(std::max(0.0,mv));
    if(bn==0){ b=Block{}; b.f0=idx; b.mn=0xFFFF; }
    b.sum+=v; b.ss+=(uint64_t)v*v; b.n++; if(v<b.mn)b.mn=v; if(v>b.mx)b.mx=v; if(v>3250)b.flags|=2;
    if(++bn>=BLOCK_N){ b.t_us=(uint64_t)llround(tt*1e6); b.f1=idx; for(int c=0;c<NCH;c++) b.chMean[c]=300; bn=0;
      int e=t.push(b);
      if(e&TE_ONSET) printf("  ONSET %.2f s\n", t.onsetT/1e6);
      if(e&TE_HOLD_START) printf("  HOLD_START %.2f s\n", t.holdT0/1e6);
      if(e&TE_HOLD_END){ auto&h=t.last; nh++; printf("  HOLD_END k=%d %.2f..%.2f dur %.2f early %.1f late %.2f (n %u, %.2f..%.2f s) sd %.2f drift %.2f flags 0x%02x  -> late F_equiv %.1f N\n",
        h.k,h.t0/1e6,h.t1/1e6,h.dur,h.early,h.late.mean,h.late.n,h.late.t0/1e6-0.1,h.late.t1/1e6,h.late.sd,h.drift,h.flags,(h.late.mean-300)/2.8);}
      if(e&TE_UNLOAD) printf("  UNLOAD %.2f s\n", t.unloadT/1e6);
    }
  }
  return nh;
}
int main(){
  // UP01: 5 s idle, ramp 10 N/s to 100 (10 s), hold 30, ramp to 200, hold 30, fast return 1 s
  run("UP01 100->200, 10 N/s, creep 0.1 mV/s", {{0,0},{5,0},{15,100},{45,100},{55,200},{85,200},{86,0}}, 0.1, 2.0);
  run("DN05 1000->500 with clipping at peak", {{0,0},{5,0},{105,1000},{135,1000},{185,500},{215,500},{216,0}}, 0.15, 2.0, 3300);
  run("UP05 900->1000, noise 4 mV", {{0,0},{5,0},{95,900},{125,900},{135,1000},{165,1000},{166,0}}, 0.1, 4.0);
  run("UP at 2 N/s ramp (flat threshold 7.5 too high?)", {{0,0},{5,0},{55,100},{85,100},{135,200},{165,200},{166,0}}, 0.1, 2.0);
  run("UP at 2 N/s, flat 2.5", {{0,0},{5,0},{55,100},{85,100},{135,200},{165,200},{166,0}}, 0.1, 2.0, 3300, 2.5f);
  run("UP at 2 N/s, flat 2.5, noise 4", {{0,0},{5,0},{55,100},{85,100},{135,200},{165,200},{166,0}}, 0.1, 4.0, 3300, 2.5f);
  run("UP03 500->600 10 N/s, log creep 1.2%/decade", {{0,0},{5,0},{55,500},{85,500},{95,600},{125,600},{126,0}}, 0, 2.0, 3300, 7.5f, true);
  run("DN09 1000->100 log creep", {{0,0},{5,0},{105,1000},{135,1000},{225,100},{255,100},{256,0}}, 0, 2.0, 3300, 7.5f, true);
  run("short 3 s pause in ramp (should be ignored)", {{0,0},{5,0},{10,50},{13,50},{18,100},{48,100},{49,0}}, 0.0, 2.0);
}
