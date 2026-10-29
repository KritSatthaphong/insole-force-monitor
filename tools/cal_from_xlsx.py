# cal_from_xlsx.py - per-sensor loading / unloading calibration from the UTM summary workbook (RF 39k).
# Usage: python cal_from_xlsx.py "S1-S7test results.xlsx"        (needs: pip install openpyxl numpy)
# Sheet layout: row 1 = S1..s7 (one per channel), row 2 = utm_N | mean_mV pairs, then points in test order
# (0 -> 1000 N loading, then 1000 -> 0 N unloading). Empty mV cells (disconnected pins) and mV >= 3250 (ADC saturated) are skipped.
# Fits F = a*G^b for the loading leg (U), the unloading leg (D) and both together; G (uS) = (mV/base - 1)/RF * 1e6,
# base = mean of the 0 N readings. Prints the DEFAULT_CAL lines for insole-app/index.html and the error of the app's
# hysteresis model replayed over the test sequence.
import sys
import numpy as np
import openpyxl

RF, SAT, MIN_LEG = 39000.0, 3250, 4


def fit(G, F):
    b, la = np.polyfit(np.log(G), np.log(F), 1)
    a = np.exp(la)
    for _ in range(80):                                  # Gauss-Newton on force residuals (N)
        p = a * G**b
        J = np.vstack([G**b, a * G**b * np.log(G)]).T
        d = np.linalg.lstsq(J, F - p, rcond=None)[0]
        a += d[0]; b += d[1]
    return a, b


def replay(G, c):
    """Same branch logic as forceHyst() in index.html: loading curve until the load reverses, then the unloading
    curve scaled through the turning point; reloading heads back to the last peak."""
    U = lambda g: c["aU"] * g ** c["bU"]
    st, out = None, []
    for g in G:
        if g <= 0: st = None; out.append(0.0); continue
        if st is None: st = dict(up=True, Gr=0.0, Fr=0.0, Gp=None, Fp=None, Ge=0.0, Fe=0.0)
        for _ in range(2):
            if st["up"]:
                if st["Gp"] is not None and g >= st["Gp"]:
                    st.update(Gr=st["Gp"], Fr=st["Fp"], Gp=None)
                if st["Gp"] is not None:
                    den = U(st["Gp"]) - U(st["Gr"])
                    F = st["Fr"] + (st["Fp"] - st["Fr"]) * ((U(g) - U(st["Gr"])) / den if den > 0 else 1)
                else:
                    F = st["Fr"] + U(g) - (U(st["Gr"]) if st["Gr"] > 0 else 0)
            else:
                F = st["Fr"] * (g / st["Gr"]) ** c["bD"]
            dz = 0.5 + 0.02 * st["Ge"]
            if st["up"] and g < st["Ge"] - dz:
                st.update(up=False, Gr=st["Ge"], Fr=st["Fe"], Gp=st["Ge"], Fp=st["Fe"]); continue
            if not st["up"] and g > st["Ge"] + dz:
                st.update(up=True, Gr=st["Ge"], Fr=st["Fe"]); continue
            break
        if (st["up"] and g > st["Ge"]) or (not st["up"] and g < st["Ge"]): st.update(Ge=g, Fe=F)
        out.append(F)
    return np.array(out)


ws = openpyxl.load_workbook(sys.argv[1], data_only=True).worksheets[0]
rows = list(ws.iter_rows(values_only=True))
labels, data = rows[0], rows[2:]
for ch in range(7):
    pts = [(r[2 * ch], r[2 * ch + 1]) for r in data if r[2 * ch] is not None]
    F = np.array([p[0] for p in pts], float)
    V = np.array([np.nan if p[1] is None else p[1] for p in pts], float)
    leg = np.where(np.arange(len(F)) <= int(np.argmax(F)), "U", "D")
    base = V[(F == 0) & ~np.isnan(V)].mean()
    G = (V / base - 1) / RF * 1e6
    ok = ~np.isnan(V) & (V < SAT) & (F > 0)
    a, b = fit(G[ok], F[ok])
    c, note = {"a": a, "b": b}, []
    for L in "UD":
        m = ok & (leg == L)
        if m.sum() >= MIN_LEG:
            aL, bL = fit(G[m], F[m])
            e = F[m] - aL * G[m] ** bL
            note.append(f"{'load' if L == 'U' else 'unload'} {m.sum()} pts rmse {np.sqrt((e ** 2).mean()):.1f} N")
        else:
            aL, bL = a, b
            note.append(f"{'load' if L == 'U' else 'unload'} leg missing")
        c["a" + L], c["b" + L] = aL, bL
    seq = ok | (~np.isnan(V) & (F == 0))                 # replay in test order, 0 N points included
    Fs = F[seq]
    e1 = (Fs - a * np.maximum(G[seq], 0) ** b)[Fs > 0]
    eh = (Fs - replay(np.maximum(G[seq], 0), c))[Fs > 0]
    lo = Fs[Fs > 0] <= 300                                # per-sensor range seen in walking
    r = lambda e: np.sqrt((e ** 2).mean()) if len(e) else float("nan")
    print(f'  {{ aU: {c["aU"]:.5f}, bU: {c["bU"]:.5f}, aD: {c["aD"]:.5f}, bD: {c["bD"]:.5f}, a: {a:.5f}, b: {b:.5f}, '
          f'base: {base:.2f}, rmse1: {r(e1):.1f}, rmseH: {r(eh):.1f}, rmse1_300: {r(e1[lo]):.1f}, rmseH_300: {r(eh[lo]):.1f}, '
          f'fmax: {F[ok].max():.0f}, note: "UTM {str(labels[2 * ch]).upper()}: {", ".join(note)}" }},')
