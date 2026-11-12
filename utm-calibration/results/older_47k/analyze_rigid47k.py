# Analysis of UTM_S1_20261026_191028 (rigid plate, RF 47k, sensor S1)
import csv, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SRC = "UTM_S1_191028_CAL_rigidplate_RF47k.csv"
RF = 47000.0
ADC_TOP = 3300.0   # ESP32-C6 ADC clips at ~3300 mV
r = list(csv.DictReader(open(SRC)))
F = np.array([float(x['utm_N']) for x in r]); V = np.array([float(x['mean_mV']) for x in r])
SD = np.array([float(x['sd_mV']) for x in r]); FI = np.array([float(x['first_mV']) for x in r])
D = np.array([x['dir'] for x in r]); FL = np.array([int(x['flags']) for x in r])
vb = float(r[0]['base_mV'])
G = np.where(V - vb > 3, (V / vb - 1) / RF * 1e6, 0)
ok = (V < ADC_TOP - 20) & (F > 0)   # only clipped points excluded
up, dn = ok & (D == 'U'), ok & (D == 'D')

# combined (mid-line) power fit F = a*G^n, used by the demo firmware
n, la = np.polyfit(np.log(G[ok]), np.log(F[ok]), 1); a = np.exp(la)
fitF = lambda g: a * np.power(g, n)
err = fitF(G) - F
FS = F[ok].max()                                    # highest unsaturated force
res = {}
res['fit'] = f"F = {a:.3f} * G^{n:.3f}  (G in uS, Vb {vb:.1f} mV, RF 47k)"
res['rmse_all'] = np.sqrt(np.mean(err[ok]**2))
res['maxerr'] = np.abs(err[ok]).max()
for name, m in (('up', up), ('dn', dn)):
    nn, ll = np.polyfit(np.log(G[m]), np.log(F[m]), 1)
    res['rmse_' + name] = np.sqrt(np.mean((np.exp(ll) * G[m]**nn - F[m])**2))
# straight line V(F) on the up branch: linearity
s, c = np.polyfit(F[up], V[up], 1)
res['lin_slope'] = s; res['lin_int'] = c
res['nonlin_maxN'] = np.abs((s * F[up] + c - V[up]) / s).max()
# hysteresis: down - up at common forces
grid = np.arange(25, min(F[up].max(), F[dn].max()) + 1, 25)
o = np.argsort(F[dn])
hyst_mV = np.interp(grid, F[dn][o], V[dn][o]) - np.interp(grid, F[up], V[up])
hyst_N = hyst_mV / s
res['hyst_maxN'] = hyst_N.max(); res['hyst_pctFS'] = hyst_N.max() / FS * 100
# noise and creep in N (local slope)
slope_local = np.gradient(np.interp(F, F[up], V[up]), F) if False else s
sdN = SD / s
res['sdN_med'] = np.median(sdN[ok]); res['sdN_max'] = sdN[ok].max()
exc = V - vb
creep = np.where(exc > 3, (V - FI) / exc * 100, 0)
res['creep_med'] = np.median(np.abs(creep[ok])); res['creep_max'] = np.abs(creep[ok]).max()
res['zero_mV'] = [round(v - vb, 2) for v, d in zip(V, D) if d == 'Z']
res['sat_from'] = (ADC_TOP - c) / s
res['FS'] = FS
for k, v in res.items(): print(k, v)

# ---------------- figure ----------------
C_UP, C_DN, INK, MUTED, GRID = "#2a78d6", "#eb6834", "#0b0b0b", "#52514e", "#e4e3df"
plt.rcParams.update({"font.size": 10, "axes.edgecolor": MUTED, "axes.labelcolor": INK,
                     "xtick.color": MUTED, "ytick.color": MUTED, "axes.spines.top": False,
                     "axes.spines.right": False, "axes.grid": True, "grid.color": GRID,
                     "grid.linewidth": 0.8, "legend.frameon": False})
fig, ax = plt.subplots(2, 2, figsize=(12, 9), dpi=150)
fig.suptitle("FSR S1 (FlexiForce A401) on UTM, rigid plate, RF 47k, 1 cycle", color=INK, fontsize=13, x=0.01, ha="left")

# A: force vs voltage
A = ax[0, 0]
A.axhspan(ADC_TOP, 3600, color="#f2f1ee", zorder=0)
A.text(15, 3560, "ADC clipping (> 3300 mV)", color=MUTED, fontsize=8, va="top")
A.plot(F[up], V[up], "-o", color=C_UP, lw=2, ms=6, label="loading (up)")
A.plot(F[dn], V[dn], "-o", color=C_DN, lw=2, ms=6, label="unloading (down)")
sat = ~ok & (F > 0)
A.plot(F[sat], V[sat], "x", color=MUTED, ms=8, mew=2, label="clipped at 3.3 V (excluded)")
gg = np.linspace(1, G[ok].max(), 200)
A.plot(fitF(gg), vb * (1 + RF * gg * 1e-6), "--", color=INK, lw=1.2, label="mid-line fit")
A.set_xlabel("UTM force (N)"); A.set_ylabel("FSR output Vout (mV)")
A.set_title("A  Force vs output voltage", loc="left", color=INK)
A.set_xlim(0, 1050); A.set_ylim(250, 3600); A.legend(loc="lower right")

# B: error of mid-line fit
B = ax[0, 1]
B.axhspan(-res['rmse_all'], res['rmse_all'], color="#f2f1ee", zorder=0)
B.axhline(0, color=MUTED, lw=1)
B.plot(F[up], err[up], "-o", color=C_UP, lw=2, ms=6, label="loading")
B.plot(F[dn], err[dn], "-o", color=C_DN, lw=2, ms=6, label="unloading")
B.text(20, res['rmse_all'] + 1, f"±RMSE {res['rmse_all']:.1f} N", color=MUTED, fontsize=8)
B.set_xlabel("UTM force (N)"); B.set_ylabel("Estimated - true force (N)")
B.set_title("B  Accuracy of one calibration curve (both directions)", loc="left", color=INK)
B.set_xlim(0, 850); B.legend(loc="lower left")

# C: hysteresis
Cx = ax[1, 0]
Cx.plot(grid, hyst_N, "-o", color=INK, lw=2, ms=5)
Cx.set_xlabel("Force (N)"); Cx.set_ylabel("Unloading - loading (N)")
Cx.set_title(f"C  Hysteresis (max {res['hyst_maxN']:.0f} N = {res['hyst_pctFS']:.1f}% of {FS:.0f} N)", loc="left", color=INK)
Cx.set_xlim(0, 850); Cx.set_ylim(0, None)

# D: noise and creep
Dx = ax[1, 1]
Dx.plot(F[up], sdN[up], "-o", color=C_UP, lw=2, ms=6, label="loading")
Dx.plot(F[dn], sdN[dn], "-o", color=C_DN, lw=2, ms=6, label="unloading")
Dx.set_xlabel("UTM force (N)"); Dx.set_ylabel("SD within 2 s hold (N)")
Dx.set_title(f"D  Noise per 200 Hz sample (median {res['sdN_med']:.2f} N)", loc="left", color=INK)
Dx.set_xlim(0, 850); Dx.set_ylim(0, None); Dx.legend(loc="upper right")

fig.tight_layout(rect=(0, 0, 1, 0.96))
fig.savefig("S1_rigid47k_analysis.png")
print("saved")
