# fit_cal.py - per-channel F = a*G^b fit from UTM rigid-plate CAL files (RF 39k).
# Usage: python fit_cal.py "<folder with UTM_S*_CAL*.csv>"
# Prints the DEFAULT_CAL lines to paste into insole-app/index.html (or type a/b into the Calibration dialog).
# Excludes saturated points (flags != 0). S6 run 190330: loading leg above 60 N is excluded (sensor read ~350 mV
# from 95 to 785 N, then jumped to saturation - looks like an intermittent contact).
import csv, glob, os, sys
import numpy as np

D = sys.argv[1] if len(sys.argv) > 1 else "."
RF = 39000.0
files = {1: "UTM_S1_*_CAL*.csv", 2: "UTM_S2_*_CAL*.csv", 3: "UTM_S3_*_CAL*.csv", 4: "UTM_S4_*_CAL*.csv",
         5: "UTM_S5_*_CAL*.csv", 6: "UTM_S6_20261027_190330_CAL.csv", 7: "UTM_S7_*_CAL*.csv"}

for ch, pat in files.items():
    fns = sorted(f for f in glob.glob(os.path.join(D, pat)) if f.endswith(".csv"))
    if not fns:
        print(f"// S{ch}: no file matching {pat}")
        continue
    fn = fns[-1]
    rows = list(csv.DictReader(open(fn)))
    rows = [r for r in rows if r["dir"] in "UD" and float(r["utm_N"]) > 0 and r["flags"] == "0"]
    if ch == 6 and "190330" in fn:
        rows = [r for r in rows if not (r["dir"] == "U" and float(r["utm_N"]) > 60)]
    F = np.array([float(r["utm_N"]) for r in rows])
    V = np.array([float(r["mean_mV"]) for r in rows])
    Vb = np.array([float(r["base_mV"]) for r in rows])
    G = (V / Vb - 1) / RF * 1e6                      # uS
    b, la = np.polyfit(np.log(G), np.log(F), 1)      # log-log start
    a = np.exp(la)
    for _ in range(50):                              # Gauss-Newton on force residuals (N)
        p = a * G**b
        J = np.vstack([G**b, a * G**b * np.log(G)]).T
        d = np.linalg.lstsq(J, F - p, rcond=None)[0]
        a += d[0]; b += d[1]
    e = F - a * G**b
    rmse = np.sqrt((e**2).mean())
    print(f'  {{ a: {a:.5f}, b: {b:.5f}, base: {Vb[0]:.2f}, rmse: {rmse:.1f}, note: "S{ch} {os.path.basename(fn)}, '
          f'{len(F)} pts, max err {abs(e).max():.1f} N, Fmax {F.max():.0f} N" }},')
