#!/usr/bin/env python3
"""utm_pair.py - pair a Shimadzu TRAPEZIUM X raw export with an FSR_UTM_Cal v3 run file, one row per hold.

    pip install numpy matplotlib
    python utm_pair.py UP01_S1_R1_20261112_101500.csv  UP01_S1_rep1.csv           (ESP32 file, UTM export)
    python utm_pair.py --pairs pairs.csv                                          (many runs)
    python utm_pair.py ESP.csv UTM.csv --out holds_table.csv --plots plots

pairs.csv columns: esp_file,utm_file[,offset_s][,window][,reject][,note]   (paths relative to pairs.csv)

What it does
  1. reads the ESP32 run file (R rows, EV, BASE, HOLD, header) and the UTM export (Time, Force, Stroke;
     the header row is found by name, units sec/ms/min and N/kN/kgf are converted, compression sign fixed)
  2. aligns the two clocks by time (never by row number): coarse search of the offset by correlation of
     UTM force with FSR conductance (starting at the k mark if present), refined on the derivatives
     (ramp starts / ends). Uncertainty = disagreement of separate fits on the first and second half.
  3. finds the holds in the UTM force (|dF/dt| small), refines each hold end, takes the final AVG s
     (minus a guard for the timing uncertainty) in BOTH streams, and the early 3-5 s window for drift
  4. writes one row per hold + the pre/post zero rows to the hold table (appends), with QC flags,
     and a plot per run. Rejected rows stay in the table with CAL = REJ.

The first column is named CAL so insole-app (Calibration check) and tools/check_cal.py read the table
directly: CAL = accepted loading / unloading / zero rows, PEAK = common-peak reference holds (DN runs),
REJ = failed QC. flags = 0 accepted, 2 clipped, 1 other reject (tools/fit_cal.py uses flags == 0).

Manual overrides (each one is logged to annotations_log.csv with --who and the time):
  --offset S           board_time = utm_time + S (skips the automatic alignment)
  --window k=t0:t1     final window of hold k in UTM seconds
  --reject k:reason    reject hold k
"""
import argparse
import csv
import datetime
import io
import os
import re
import sys

import numpy as np

RF_DEFAULT = 39000.0
ADC_TOP = 3250
NEAR_RAIL = 3000
RATE = 200.0
# provisional loading curves (insole-app DEFAULT_CAL aU, bU), only used to express drift in N
PROV = [(3.58966, 1.02654), (3.98233, 1.01871), (3.22000, 1.00616), (2.72739, 1.02718),
        (4.28822, 1.01217), (2.80304, 1.03127), (3.41093, 0.99299)]

COLS = ["CAL", "run_id", "run_uid", "foot", "board_id", "sensor_id", "ch", "site", "repeat_id", "cycle", "stage",
        "dir", "target_N", "set_N", "prior_force_N", "peak_N", "ramp_N_per_s", "dwell_s", "hold_s",
        "utm_t0_s", "utm_t1_s", "utm_N", "utm_sd_N", "utm_min_N", "utm_max_N", "utm_pp_N",
        "mean_mV", "sd_mV", "min_mV", "max_mV", "base_mV", "vref_mV", "rf_ohm", "g_uS", "n_utm", "n_fsr",
        "early_mV", "drift_mV", "drift_utm_N", "drift_force_mV", "overshoot_N", "t_since_load_s",
        "flags", "qc_flags", "alignment_method", "align_offset_s", "align_unc_s", "fixture_id",
        "utm_file", "fsr_file", "analyzed"]

REJECT = {"clip", "pp_big", "no_fsr", "missing_big", "align_big", "window", "manual_reject", "extra_hold",
          "no_window"}


# =====================================================================
# ESP32 run file
def read_esp(path):
    hdr, meta, ev, base, hold, rows = {}, {}, [], [], [], []
    with open(path, newline="") as f:
        for line in f:
            line = line.rstrip("\r\n")
            if line.startswith("R,"):
                rows.append(line)
            elif line.startswith("#"):
                p = line[1:].strip().split(",")
                if p[0] == "meta" and len(p) >= 3:
                    meta[p[1]] = ",".join(p[2:])
                else:
                    hdr.setdefault(p[0], p[1:])
            elif line.startswith("EV,"):
                p = line.split(",")
                ev.append(dict(id=p[1], t=int(p[2]) / 1e6, frame=int(p[3]), type=p[4], text=p[5] if len(p) > 5 else ""))
            elif line.startswith("BASE,"):
                p = line.split(",")
                base.append(dict(kind=p[1], t0=int(p[2]) / 1e6, t1=int(p[3]) / 1e6, n=int(p[6]),
                                 mv=[float(x) for x in p[7:14]], sd=float(p[14]),
                                 d_zero=float(p[15]), d_pre=float(p[16]) if len(p) > 16 and p[16] else None))
            elif line.startswith("HOLD,"):
                hold.append(line.split(","))
    if not rows:
        raise ValueError(f"{path}: no R rows (not an FSR_UTM_Cal run file?)")
    a = np.array([r.split(",")[1:] for r in rows], dtype=np.int64)
    return dict(hdr=hdr, meta=meta, ev=ev, base=base, hold=hold, t=a[:, 0] / 1e6, frame=a[:, 1],
                mv=a[:, 2:9].astype(float), flags=a[:, 9], path=path)


# =====================================================================
# Shimadzu export
TIME_KEYS = ("time", "zeit", "時間", "เวลา")
FORCE_KEYS = ("force", "load", "試験力", "荷重", "แรง")
STROKE_KEYS = ("stroke", "disp", "extension", "ストローク", "変位", "ระยะ")


def _decode(raw):
    for enc in ("utf-8-sig", "utf-16", "cp932", "cp874", "latin-1"):
        try:
            s = raw.decode(enc)
            if enc == "utf-16" and "\x00" in s:
                continue
            return s
        except UnicodeError:
            continue
    return raw.decode("latin-1", "replace")


def _num(x):
    x = x.strip().strip('"').strip()
    try:
        return float(x)
    except ValueError:
        return None


def read_utm(path, force_unit=None, time_unit=None, cols=None):
    text = _decode(open(path, "rb").read())
    lines = [l for l in text.splitlines()]
    sample = "\n".join(lines[:200])
    delim = max(",\t;", key=lambda d: sample.count(d))
    rows = list(csv.reader(io.StringIO(text), delimiter=delim))
    hi = None
    for i, r in enumerate(rows[:200]):
        low = [c.strip().lower() for c in r]
        if any(any(k in c for k in TIME_KEYS) for c in low) and any(any(k in c for k in FORCE_KEYS) for c in low):
            hi = i
            break
    units = ["", "", ""]
    if cols:
        ct, cf, cs = cols
        start = 0
    elif hi is not None:
        low = [c.strip().lower() for c in rows[hi]]
        find = lambda keys: next((j for j, c in enumerate(low) if any(k in c for k in keys)), None)
        ct, cf, cs = find(TIME_KEYS), find(FORCE_KEYS), find(STROKE_KEYS)
        start = hi + 1
        nxt = rows[start] if start < len(rows) else []
        if nxt and all(_num(c) is None for c in nxt if c.strip()):
            units = [nxt[j].strip().lower() if j is not None and j < len(nxt) else "" for j in (ct, cf, cs)]
            start += 1
        for j, k in ((0, ct), (1, cf), (2, cs)):            # units written in the header, e.g. "Force (kN)"
            if k is not None and not units[j]:
                m = re.search(r"[\[(]([^\])]+)[\])]", rows[hi][k])
                if m:
                    units[j] = m.group(1).strip().lower()
    else:
        ct, cf, cs, start = 0, 1, 2, 0
    t, F, st = [], [], []
    for r in rows[start:]:
        if ct is None or cf is None or len(r) <= max(ct, cf):
            continue
        a, b = _num(r[ct]), _num(r[cf])
        if a is None or b is None:
            continue
        t.append(a)
        F.append(b)
        st.append(_num(r[cs]) if cs is not None and cs < len(r) else np.nan)
    if len(t) < 20:
        raise ValueError(f"{path}: fewer than 20 numeric Time/Force rows found (use --utm-cols)")
    t, F, st = np.array(t), np.array(F), np.array([np.nan if v is None else v for v in st], float)
    tu = (time_unit or units[0] or "").lower()
    if not tu:                                       # no unit given: 20 Hz in ms would show steps of ~50
        tu = "ms" if np.median(np.diff(t)) >= 5 else "s"
    if tu in ("ms", "msec"):
        t = t / 1000
    elif tu.startswith("min"):
        t = t * 60
    fu = (force_unit or units[1] or "n").lower()
    if fu == "kn":
        F = F * 1000
    elif fu == "kgf":
        F = F * 9.80665
    big = np.abs(F) > 0.5 * np.abs(F).max()
    sign_flip = bool(np.median(F[big]) < 0)
    if sign_flip:
        F = -F
    o = np.argsort(t, kind="stable")
    t, F, st = t[o], F[o], st[o]
    dt = np.diff(t)
    info = dict(units=units, sign_flip=sign_flip, n=len(t), dt_med=float(np.median(dt)),
                dt_max=float(dt.max()), header_row=hi, delim=repr(delim))
    return t - t[0], F, st, info


# =====================================================================
def smooth(x, n):
    n = max(1, int(n))
    if n == 1:
        return x.copy()
    k = np.ones(n) / n
    p = n // 2
    xp = np.concatenate([np.full(p, x[0]), x, np.full(n - 1 - p, x[-1])])
    return np.convolve(xp, k, mode="valid")


def corr(a, b):
    a = a - a.mean()
    b = b - b.mean()
    d = np.sqrt((a * a).sum() * (b * b).sum())
    return float((a * b).sum() / d) if d > 0 else -1.0


def best_offset(tu, Fu, tb, Gb, lo, hi, step, deriv=False, sel=None):
    """offset tau (board = utm + tau) in [lo, hi] maximizing corr(F_utm(t), G_fsr(t + tau))"""
    grid = np.arange(lo, hi + step / 2, step)
    if sel is None:
        sel = np.ones(len(tu), bool)
    x = Fu[sel]
    if deriv:
        x = np.gradient(smooth(Fu, 5), tu)[sel]
    gb = np.gradient(smooth(Gb, 50), tb) if deriv else Gb
    best, bt = -2, lo
    for tau in grid:
        tt = tu[sel] + tau
        if tt[0] < tb[0] or tt[-1] > tb[-1]:
            continue
        c = corr(x, np.interp(tt, tb, gb))
        if c > best:
            best, bt = c, tau
    return bt, best


def align(esp, tu, Fu, ch, base):
    tb = esp["t"]
    G = np.maximum(esp["mv"][:, ch] / base - 1, 0)
    mark = next((e["t"] for e in esp["ev"] if e["type"] == "MARK"), None)
    Tu = tu[-1]
    if mark is not None:
        lo, hi, method = mark - 15, mark + 15, "xcorr(mark start)"
    else:
        lo, hi, method = tb[0] - Tu, tb[-1], "xcorr(full search)"
    tau, c = best_offset(tu, Fu, tb, G, lo, hi, 0.1)
    if mark is not None and c < 0.9:                 # mark far off: search everything
        tau2, c2 = best_offset(tu, Fu, tb, G, tb[0] - Tu, tb[-1], 0.1)
        if c2 > c:
            tau, c, method = tau2, c2, "xcorr(full search)"
    tau, cd = best_offset(tu, Fu, tb, G, tau - 0.3, tau + 0.3, 0.005, deriv=True)
    half = tu < Tu / 2
    t1, _ = best_offset(tu, Fu, tb, G, tau - 1, tau + 1, 0.005, deriv=True, sel=half)
    t2, _ = best_offset(tu, Fu, tb, G, tau - 1, tau + 1, 0.005, deriv=True, sel=~half)
    unc = max(abs(t1 - tau), abs(t2 - tau)) + 0.005
    return dict(tau=tau, unc=unc, corr=c, corr_d=cd, method=method + "+deriv refine", mark=mark,
                tau_first=t1, tau_second=t2)


# =====================================================================
def find_holds(tu, Fu, thr, fmin, min_s):
    dt = float(np.median(np.diff(tu)))
    Fs = smooth(Fu, round(0.5 / dt))
    sl = smooth(np.gradient(Fs, tu), round(0.5 / dt))
    m = (np.abs(sl) < thr) & (Fs > fmin)
    segs, i, n = [], 0, len(m)
    while i < n:
        if m[i]:
            j = i
            while j < n and m[j]:
                j += 1
            segs.append([i, j - 1])
            i = j
        else:
            i += 1
    merged = []
    for s in segs:                                   # join gaps < 0.5 s at similar level
        if merged and tu[s[0]] - tu[merged[-1][1]] < 0.5 and abs(Fs[s[0]] - Fs[merged[-1][1]]) < max(2, 0.01 * Fs[s[0]]):
            merged[-1][1] = s[1]
        else:
            merged.append(s)
    out = []
    Fr = smooth(Fu, max(1, round(0.2 / dt)))
    for a, b in merged:
        if tu[b] - tu[a] < min_s:
            continue
        # refine the end: walk forward while the force stays at the plateau level
        e0 = np.searchsorted(tu, tu[b] - 4.0)
        e1 = np.searchsorted(tu, tu[b] - 1.0)
        L = float(np.median(Fu[max(a, e0):max(a + 1, e1)]))
        tol = max(2.0, 0.01 * L)
        j = max(a, e1)
        while j + 1 < len(tu) and abs(Fr[j + 1] - L) <= tol:
            j += 1
        out.append(dict(i0=a, i1=j, t0=float(tu[a]), t1=float(tu[j]), level=L))
    return out


def stats(x):
    return (float(np.mean(x)), float(np.std(x)), float(np.min(x)), float(np.max(x))) if len(x) else (np.nan,) * 4


def fmt(v, nd=3):
    if v is None or (isinstance(v, float) and not np.isfinite(v)):
        return ""
    if isinstance(v, float):
        return f"{v:.{nd}f}"
    return str(v)


def analyze(esp_path, utm_path, a, offset=None, windows=None, rejects=None, note=""):
    esp = read_esp(esp_path)
    H, M = esp["hdr"], esp["meta"]
    ch = int(H.get("mode", ["run", "channel", "1"])[2]) - 1
    if ch < 0:
        raise ValueError(f"{esp_path}: free RAW recording without a channel; pairing needs a RUN file")
    rf_ch = [float(x) for x in H.get("rf_ohm_ch", [])] or [float(H.get("rf_ohm", [RF_DEFAULT])[0])] * 7
    rf = rf_ch[ch]
    pre = next((b for b in esp["base"] if b["kind"] == "pre"), None)
    post = next((b for b in esp["base"] if b["kind"] == "post"), None)
    base = pre["mv"][ch] if pre else float(H.get("base_mV", [300] * 7)[ch])
    tmpl = H.get("template", ["FREE"])[0]
    targets = [float(x) for x in H.get("targets_N", [])]
    dwell = float(H.get("dwell_s", [30])[0])
    ramp = float(H.get("ramp_N_per_s", [10])[0])
    P = float(H.get("p_N", [1000])[0])
    hd = H.get("hold_detect", [])
    avg = a.avg or (float(hd[hd.index("avg_s") + 1]) if "avg_s" in hd else 2.0)
    rep = H.get("repeat_id", [""])[0]
    run_id = H.get("run_id", [os.path.basename(esp_path).split("_")[0]])[0]

    tu, Fu, stroke, uinfo = read_utm(utm_path, a.force_unit, a.time_unit, a.utm_cols)
    if offset is not None:
        al = dict(tau=offset, unc=a.manual_unc, corr=np.nan, corr_d=np.nan, method="manual offset", mark=None,
                  tau_first=np.nan, tau_second=np.nan)
    else:
        al = align(esp, tu, Fu, ch, base)
    tau, unc = al["tau"], al["unc"]
    tb = esp["t"] - tau                               # board samples on the UTM clock
    mv = esp["mv"][:, ch]
    frame = esp["frame"]

    thr = a.hold_thr or max(0.5, 0.2 * ramp)
    holds = find_holds(tu, Fu, thr, max(5.0, 0.02 * P), a.min_hold or max(5.0, 0.5 * dwell))
    onset_i = int(np.argmax(Fu > 2.0)) if (Fu > 2.0).any() else 0
    t_onset = tu[onset_i]
    guard = a.guard + 2 * unc
    cm = re.match(r"\s*([\d.]+)", M.get("cell_N", M.get("cell_n", "")))
    cell_N = float(cm.group(1)) if cm else 0.0

    # expected holds of the template
    exp = []
    if tmpl == "UP" and len(targets) == 2:
        exp = [("hold1", "U", targets[0]), ("hold2", "U", targets[1])]
    elif tmpl == "DN" and len(targets) == 2:
        exp = [("peak", "P", targets[0]), ("hold2", "D", targets[1])]
    assign = {}
    used = set()
    for k, (stg, d, tgt) in enumerate(exp):          # match by force level, in time order
        cand = [i for i in range(len(holds)) if i not in used and (not used or i > max(used))]
        if cand:
            i = min(cand, key=lambda i: abs(holds[i]["level"] - tgt))
            assign[i] = (stg, d, tgt)
            used.add(i)
    count_bad = bool(exp) and len(holds) != len(exp)

    rows, prev_level, plotinfo = [], 0.0, []
    common = dict(run_id=run_id, run_uid=H.get("run_uid", [""])[0], foot=M.get("foot", ""), board_id=M.get("board", ""),
                  sensor_id=M.get("sensor", ""), ch=ch + 1, site=H.get("site", [""])[0], repeat_id=rep, cycle=rep,
                  dwell_s=dwell, base_mV=base, vref_mV=M.get("vref_mV", M.get("vref_mv", "")), rf_ohm=rf,
                  alignment_method=al["method"], align_offset_s=tau, align_unc_s=unc,
                  fixture_id=M.get("fixture", ""), utm_file=os.path.basename(utm_path),
                  fsr_file=os.path.basename(esp_path), analyzed=datetime.datetime.now().isoformat(timespec="seconds"))

    for k, h in enumerate(holds, 1):
        qc = []
        stg, d, tgt = assign.get(k - 1, ("extra", "X", np.nan))
        if not exp:                                  # free run: direction from the level change
            stg, d, tgt = f"hold{k}", ("U" if h["level"] >= prev_level else "D"), np.nan
        elif (k - 1) not in assign:
            qc.append("extra_hold")
        if count_bad:
            qc.append("hold_count")
        w1 = h["t1"] - guard
        w0 = w1 - avg
        if windows and k in windows:
            w0, w1 = windows[k]
            qc.append("manual_window")
        if w0 < h["t0"]:
            qc.append("window")
        su = (tu > w0) & (tu <= w1)
        sb = (tb > w0) & (tb <= w1)
        uN, uSD, uMin, uMax = stats(Fu[su])
        mN, mSD, mMin, mMax = stats(mv[sb])
        n_exp_u = avg / max(uinfo["dt_med"], 1e-3)
        if su.sum() < 0.8 * n_exp_u:
            qc.append("utm_n")
        nb = int(sb.sum())
        miss = int(frame[sb][-1] - frame[sb][0] + 1 - nb) if nb else 0
        if nb < 0.5 * avg * RATE:
            qc.append("no_fsr")
        elif miss > 0.05 * avg * RATE:
            qc.append("missing_big")
        elif miss > 0:
            qc.append("missing")
        if nb and mMax > ADC_TOP:
            qc.append("clip")
        elif nb and mMax > NEAR_RAIL:
            qc.append("near_rail")
        pp = uMax - uMin
        if np.isfinite(tgt):
            tol = max(2.0, 0.01 * tgt)
            if abs(uN - tgt) > tol:
                qc.append("setpoint")
            if cell_N and tgt < 0.02 * cell_N:
                qc.append("below_2pct_cell")
        else:
            tol = max(2.0, 0.01 * abs(uN))
        if pp > 3 * tol:
            qc.append("pp_big")
        elif pp > tol:
            qc.append("pp")
        if unc > 0.5:
            qc.append("align_big")
        elif unc > 0.1:
            qc.append("align")
        hold_s = h["t1"] - h["t0"]
        if hold_s < 0.9 * dwell:
            qc.append("short_hold")
        # early window -> drift
        e0, e1 = h["t0"] + a.early[0], h["t0"] + a.early[1]
        ue = Fu[(tu > e0) & (tu <= e1)]
        be = mv[(tb > e0) & (tb <= e1)]
        early_mV = float(be.mean()) if len(be) else np.nan
        drift_mV = mN - early_mV
        drift_N = uN - float(ue.mean()) if len(ue) else np.nan
        A, B = PROV[ch]
        G = (mN / base - 1) / rf * 1e6 if mN > base else 0
        sens = base * rf * 1e-6 * (G / (B * uN)) if uN > 0 and G > 0 else np.nan   # dmV/dN on the provisional curve
        drift_force_mV = drift_N * sens if np.isfinite(sens) else np.nan
        sel = (tu >= h["t0"] - 1) & (tu <= h["t0"] + 3)
        over = (Fu[sel].max() - h["level"]) if d in "UPX" else (h["level"] - Fu[sel].min())
        # ramp rate into this hold: middle 60 % of the change from the previous plateau
        prev_t = holds[k - 2]["t1"] if k > 1 else t_onset
        rs = (tu > prev_t) & (tu < h["t0"])
        ramp_meas = np.nan
        if rs.sum() > 10:
            f0, f1 = (holds[k - 2]["level"] if k > 1 else 0.0), h["level"]
            lo_, hi_ = sorted((f0 + 0.2 * (f1 - f0), f0 + 0.8 * (f1 - f0)))
            mid = rs & (Fu > lo_) & (Fu < hi_)
            if mid.sum() > 5:
                ramp_meas = abs(np.polyfit(tu[mid], Fu[mid], 1)[0])
        if rejects and k in rejects:
            qc.append("manual_reject")
        rej = any(q in REJECT for q in qc)
        rec = "REJ" if rej else ("PEAK" if d == "P" else "CAL")
        flags = 2 if "clip" in qc else (1 if rej else 0)
        peak = float(Fu[: np.searchsorted(tu, h["t1"]) + 1].max())
        row = dict(common, CAL=rec, stage=stg, dir=d, target_N=tgt, set_N=tgt if np.isfinite(tgt) else uN,
                   prior_force_N=prev_level, peak_N=peak, ramp_N_per_s=ramp_meas, hold_s=hold_s,
                   utm_t0_s=w0, utm_t1_s=w1, utm_N=uN, utm_sd_N=uSD, utm_min_N=uMin, utm_max_N=uMax, utm_pp_N=pp,
                   mean_mV=mN, sd_mV=mSD, min_mV=mMin, max_mV=mMax, g_uS=G, n_utm=int(su.sum()), n_fsr=nb,
                   early_mV=early_mV, drift_mV=drift_mV, drift_utm_N=drift_N, drift_force_mV=drift_force_mV,
                   overshoot_N=over, t_since_load_s=w1 - t_onset, flags=flags, qc_flags=";".join(qc))
        rows.append(row)
        plotinfo.append((h, w0, w1, rec, stg))
        prev_level = uN

    for b, stg in ((pre, "base_pre"), (post, "base_post")):
        if not b:
            continue
        qc = ["utm_zero_nominal"]
        if abs(b["d_zero"]) > 5:
            qc.append("zero_offset")
        if b["d_pre"] is not None and abs(b["d_pre"]) > 5:
            qc.append("not_recovered")
        rows.append(dict(common, CAL="CAL", stage=stg, dir="Z", target_N=0.0, set_N=0.0, prior_force_N="",
                         peak_N="", ramp_N_per_s="", hold_s=b["t1"] - b["t0"], utm_t0_s=b["t0"] - tau,
                         utm_t1_s=b["t1"] - tau, utm_N=0.0, utm_sd_N="", utm_min_N="", utm_max_N="", utm_pp_N="",
                         mean_mV=b["mv"][ch], sd_mV=b["sd"], min_mV="", max_mV="", g_uS=0.0, n_utm=0, n_fsr=b["n"],
                         early_mV="", drift_mV="", drift_utm_N="", drift_force_mV="", overshoot_N="",
                         t_since_load_s="", flags=0, qc_flags=";".join(qc)))
    order = {"base_pre": 0}
    rows.sort(key=lambda r: (order.get(r["stage"], 1), r["stage"] == "base_post"))
    board_holds = [(int(h[4]) / 1e6 - tau, int(h[5]) / 1e6 - tau) for h in esp["hold"]]
    return dict(rows=rows, al=al, uinfo=uinfo, holds=plotinfo, tu=tu, Fu=Fu, tb=tb, mv=mv, ch=ch, base=base,
                rf=rf, run_id=run_id, rep=rep, board_holds=board_holds, esp=esp)


# =====================================================================
def plot(res, path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("  (matplotlib not installed: no plot)")
        return
    C_UTM, C_FSR, INK, MUTED, GRID = "#2a78d6", "#eb6834", "#0b0b0b", "#52514e", "#e4e3df"
    plt.rcParams.update({"font.size": 9, "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True,
                         "grid.color": GRID, "axes.edgecolor": MUTED, "legend.frameon": False})
    fig, ax = plt.subplots(2, 1, figsize=(11, 6.5), dpi=130, sharex=True)
    al = res["al"]
    fig.suptitle(f"{res['run_id']}  S{res['ch'] + 1}  repeat {res['rep']}   offset {al['tau']:.3f} s "
                 f"± {al['unc']:.3f} s ({al['method']})", x=0.01, ha="left", fontsize=10, color=INK)
    ax[0].plot(res["tu"], res["Fu"], color=C_UTM, lw=1.2, label="UTM force")
    for h, w0, w1, rec, stg in res["holds"]:
        for A in ax:
            A.axvspan(h["t0"], h["t1"], color="#f2f1ee", zorder=0)
            A.axvspan(w0, w1, color="#cfe3f7" if rec != "REJ" else "#f7d4cf", zorder=0)
        ax[0].text((h["t0"] + h["t1"]) / 2, h["level"], f"{stg}\n{rec}", ha="center", va="bottom", fontsize=8, color=MUTED)
    for t0, t1 in res["board_holds"]:
        ax[1].plot([t0, t1], [res["base"] - 20] * 2, color=MUTED, lw=3, solid_capstyle="butt")
    ax[0].set_ylabel("UTM force (N)")
    ax[0].legend(loc="upper right")
    sel = (res["tb"] >= res["tu"][0] - 5) & (res["tb"] <= res["tu"][-1] + 5)
    ax[1].plot(res["tb"][sel], res["mv"][sel], color=C_FSR, lw=0.8, label=f"FSR S{res['ch'] + 1} (aligned)")
    ax[1].axhline(ADC_TOP, color=MUTED, lw=0.8, ls="--")
    ax[1].text(res["tu"][0], ADC_TOP, " clip 3250 mV", color=MUTED, fontsize=7, va="bottom")
    ax[1].set_ylabel("FSR output (mV)")
    ax[1].set_xlabel("UTM time (s)   [shaded: hold; blue = final window used, red = rejected; grey bars = on-board holds]")
    ax[1].legend(loc="upper right")
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(path)
    plt.close(fig)


def log_annotation(folder, who, esp, utm, kind, value, note):
    p = os.path.join(folder, "annotations_log.csv")
    new = not os.path.exists(p)
    with open(p, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["time", "who", "fsr_file", "utm_file", "override", "value", "note"])
        w.writerow([datetime.datetime.now().isoformat(timespec="seconds"), who, os.path.basename(esp),
                    os.path.basename(utm), kind, value, note])


def parse_windows(s):
    out = {}
    for part in filter(None, re.split(r"[;\s]+", s or "")):
        k, v = part.split("=")
        t0, t1 = v.split(":")
        out[int(k)] = (float(t0), float(t1))
    return out


def parse_rejects(s):
    out = {}
    for part in filter(None, re.split(r";", s or "")):
        k, _, why = part.partition(":")
        out[int(k)] = why.strip()
    return out


def main():
    ap = argparse.ArgumentParser(description="Pair TRAPEZIUM X raw exports with FSR_UTM_Cal v3 run files")
    ap.add_argument("esp", nargs="?", help="ESP32 run file (UP01_S1_R1_....csv)")
    ap.add_argument("utm", nargs="?", help="Shimadzu raw export csv for the same run")
    ap.add_argument("--pairs", help="csv with esp_file,utm_file[,offset_s][,window][,reject][,note]")
    ap.add_argument("--out", default="holds_table.csv", help="hold table (rows are appended)")
    ap.add_argument("--plots", default="pair_plots", help="folder for one PNG per run ('' = none)")
    ap.add_argument("--offset", type=float, help="manual offset: board_time = utm_time + OFFSET (s)")
    ap.add_argument("--manual-unc", type=float, default=0.1, help="timing uncertainty to assume with --offset (s)")
    ap.add_argument("--window", help="manual final window(s): 'k=t0:t1' in UTM seconds, ';' between holds")
    ap.add_argument("--reject", help="manual reject(s): 'k:reason', ';' between holds")
    ap.add_argument("--who", default=os.environ.get("USERNAME", os.environ.get("USER", "")), help="name for the audit log")
    ap.add_argument("--note", default="", help="note for the audit log")
    ap.add_argument("--avg", type=float, help="final window length (s), default = avg_s of the run file (2 s)")
    ap.add_argument("--guard", type=float, default=0.3, help="gap before the hold end (s), + 2x timing uncertainty")
    ap.add_argument("--early", type=float, nargs=2, default=(3.0, 5.0), help="early window after the hold start (s)")
    ap.add_argument("--hold-thr", type=float, help="|dF/dt| below this = hold (N/s), default 0.2 x ramp rate")
    ap.add_argument("--min-hold", type=float, help="shortest hold to keep (s), default max(5, dwell/2)")
    ap.add_argument("--force-unit", help="override the UTM force unit (N, kN, kgf)")
    ap.add_argument("--time-unit", help="override the UTM time unit (s, ms, min)")
    ap.add_argument("--utm-cols", type=int, nargs=3, metavar=("T", "F", "S"), help="0-based Time Force Stroke columns")
    a = ap.parse_args()

    jobs = []
    if a.pairs:
        base_dir = os.path.dirname(os.path.abspath(a.pairs))
        for r in csv.DictReader(open(a.pairs, newline="")):
            j = lambda p: p if os.path.isabs(p) else os.path.join(base_dir, p)
            off = r.get("offset_s", "").strip()
            jobs.append((j(r["esp_file"].strip()), j(r["utm_file"].strip()), float(off) if off else None,
                         r.get("window", ""), r.get("reject", ""), r.get("note", "")))
    elif a.esp and a.utm:
        jobs.append((a.esp, a.utm, a.offset, a.window or "", a.reject or "", a.note))
    else:
        ap.error("give ESP and UTM files, or --pairs")

    out_dir = os.path.dirname(os.path.abspath(a.out))
    if a.plots:
        os.makedirs(a.plots, exist_ok=True)
    new = not os.path.exists(a.out)
    with open(a.out, "a", newline="") as fo:
        w = csv.DictWriter(fo, fieldnames=COLS, extrasaction="ignore")
        if new:
            w.writeheader()
        for esp, utm, off, win, rej, note in jobs:
            print(f"== {os.path.basename(esp)}  +  {os.path.basename(utm)}")
            try:
                windows, rejects = parse_windows(win), parse_rejects(rej)
                res = analyze(esp, utm, a, off, windows, rejects, note)
            except Exception as e:                       # keep going with the other runs
                print(f"  ERROR: {e}")
                continue
            for kind, val in (("offset", off), ("window", win), ("reject", rej)):
                if val not in (None, ""):
                    log_annotation(out_dir, a.who, esp, utm, kind, val, note)
            al, ui = res["al"], res["uinfo"]
            print(f"  UTM: {ui['n']} rows, dt {ui['dt_med'] * 1000:.0f} ms (max gap {ui['dt_max'] * 1000:.0f} ms), "
                  f"units {ui['units']}, sign flipped: {ui['sign_flip']}")
            mk = "" if al["mark"] is None else f", k mark at board {al['mark']:.3f} s"
            print(f"  alignment: board = utm + {al['tau']:.3f} s  +/- {al['unc']:.3f} s  (first half {al['tau_first']:.3f}, "
                  f"second half {al['tau_second']:.3f}, corr {al['corr']:.4f}){mk}")
            for r in res["rows"]:
                print(f"  {r['CAL']:4s} {r['stage']:9s} {r['dir']} target {fmt(r['target_N'], 0):>5s} N  UTM {fmt(r['utm_N'], 2):>8s} N "
                      f"(pp {fmt(r['utm_pp_N'], 2):>5s})  FSR {fmt(r['mean_mV'], 2):>8s} mV (sd {fmt(r['sd_mV'], 2)})  "
                      f"drift {fmt(r['drift_mV'], 1):>5s} mV / {fmt(r['drift_utm_N'], 2):>5s} N  {r['qc_flags']}")
                w.writerow({k: fmt(v) if isinstance(v, float) else v for k, v in r.items()})
            if a.plots:
                p = os.path.join(a.plots, os.path.splitext(os.path.basename(esp))[0] + "_pair.png")
                plot(res, p)
                print(f"  plot: {p}")
    print(f"hold table: {os.path.abspath(a.out)}")


if __name__ == "__main__":
    main()
