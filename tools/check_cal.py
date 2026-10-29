# check_cal.py - does the app's mV -> N conversion match a UTM CAL log? (same model as insole-app, RF 39k)
# Usage: python check_cal.py <UTM_..._CAL.csv> [channel 1-7, default = ch column of the file] [--skip F_N ...]
# Runs the app's default coefficients for that channel (hysteresis model and single curve) over the CAL hold points,
# in test order, with base = the file's base_mV, and prints bias / RMSE / max error by branch.
# Saturated points (>= 3250 mV) and 0 N points are left out. Needs numpy.
import csv, sys
import numpy as np

RF, SAT_MV, DEAD_MV, FS = 39000, 3250, 4, 1000
# copied from insole-app/index.html DEFAULT_CAL (S1-S7test results.xlsx)
CAL = [
    dict(aU=3.58966, bU=1.02654, aD=2.16552, bD=1.11225, a=2.77212, b=1.07015),
    dict(aU=3.98233, bU=1.01871, aD=2.04133, bD=1.13729, a=2.82156, b=1.08045),
    dict(aU=3.22000, bU=1.00616, aD=3.22000, bD=1.00616, a=3.22000, b=1.00616),
    dict(aU=2.72739, bU=1.02718, aD=1.60231, bD=1.11501, a=2.12201, b=1.06744),
    dict(aU=4.28822, bU=1.01217, aD=2.15866, bD=1.13229, a=2.99311, b=1.07616),
    dict(aU=2.80304, bU=1.03127, aD=2.71891, bD=1.03706, a=2.80304, b=1.03127),
    dict(aU=3.41093, bU=0.99299, aD=1.73110, bD=1.10614, a=2.28257, b=1.06399),
]


def g_of(mv, base):
    return (mv / base - 1) / RF * 1e6 if mv > base + DEAD_MV else 0.0


def hyst_force(st, G, c):
    """Same as hystForce() in the app (G0 = 0, no preload)."""
    U = lambda g: c["aU"] * g ** c["bU"]
    if not G > 0.3:
        st["on"] = False
        return U(G) if G > 0 else 0.0
    if not st.get("on"):
        st.update(on=True, up=True, Gr=0.0, Fr=0.0, Gp=None, Fp=0.0, Ge=0.0, Fe=0.0)
    F = 0.0
    for _ in range(2):
        if st["up"]:
            if st["Gp"] is not None and G >= st["Gp"]:
                st["Gr"], st["Fr"], st["Gp"] = st["Gp"], st["Fp"], None
            if st["Gp"] is not None:
                den = U(st["Gp"]) - U(st["Gr"])
                F = st["Fr"] + (st["Fp"] - st["Fr"]) * ((U(G) - U(st["Gr"])) / den if den > 0 else 1)
            else:
                F = st["Fr"] + U(G) - (U(st["Gr"]) if st["Gr"] > 0 else 0)
        else:
            F = st["Fr"] * (G / st["Gr"]) ** c["bD"]
        dz = 0.5 + 0.02 * st["Ge"]
        if st["up"] and G < st["Ge"] - dz:
            st.update(up=False, Gr=st["Ge"], Gp=st["Ge"], Fr=st["Fe"], Fp=st["Fe"])
            continue
        if not st["up"] and G > st["Ge"] + dz:
            st.update(up=True, Gr=st["Ge"], Fr=st["Fe"])
            continue
        break
    if (G > st["Ge"]) if st["up"] else (G < st["Ge"]):
        st["Ge"], st["Fe"] = G, F
    return F


def main():
    args = sys.argv[1:]
    skip = set()
    if "--skip" in args:
        i = args.index("--skip")
        skip = {float(x) for x in args[i + 1:]}
        args = args[:i]
    if not args:
        sys.exit(__doc__ or "usage: python check_cal.py <UTM_..._CAL.csv> [channel] [--skip F_N ...]")
    path = args[0]
    rows = list(csv.reader(open(path, newline="")))
    head = [h.strip() for h in rows[0]]
    need = ("ch", "dir", "utm_N", "mean_mV", "base_mV")
    if any(k not in head for k in need):
        sys.exit(f"{path}: not an FSR_UTM_Cal CAL log (needs columns {', '.join(need)})")
    col = {k: head.index(k) for k in need}
    pts = [r for r in rows[1:] if r and r[0].strip() == "CAL"]
    if not pts:
        sys.exit(f"{path}: no CAL rows found")
    ch = int(args[1]) if len(args) > 1 else int(pts[0][col["ch"]])
    c = CAL[ch - 1]
    base = float(pts[0][col["base_mV"]])
    st, res = {}, []
    for r in pts:
        F, mv = float(r[col["utm_N"]]), float(r[col["mean_mV"]])
        G = g_of(mv, base)
        fh = hyst_force(st, G, c)
        fs = c["a"] * G ** c["b"] if G > 0 else 0.0
        if F > 0 and mv < SAT_MV and F not in skip:
            res.append((r[col["dir"]].strip(), F, fh, fs))
    print(f"{path}: ch{ch}, base {base:.2f} mV, {len(res)} points (saturated, 0 N and --skip points left out)")
    if not res:
        return
    for name, sel in (("loading", "U"), ("unloading", "D"), ("all", None)):
        q = [x for x in res if sel is None or x[0] == sel]
        if not q:
            print(f"  {name:9s} no points")
            continue
        for model, k in (("hysteresis model", 2), ("single curve", 3)):
            e = np.array([x[k] - x[1] for x in q])
            print(f"  {name:9s} {model:16s} n={len(e):2d}  bias {e.mean():+6.1f} N  RMSE {np.sqrt((e ** 2).mean()):5.1f} N"
                  f"  max|e| {abs(e).max():5.1f} N ({abs(e).max() / FS * 100:.1f} % FS)")
    low = np.array([x[2] - x[1] for x in res if x[1] <= 300])
    if len(low):
        print(f"  0-300 N hysteresis model: bias {low.mean():+.1f} N, RMSE {np.sqrt((low ** 2).mean()):.1f} N")
    worst = max(res, key=lambda x: abs(x[2] - x[1]))
    print(f"  worst point: UTM {worst[1]:g} N ({worst[0]}), app {worst[2]:.1f} N")


if __name__ == "__main__":
    main()
