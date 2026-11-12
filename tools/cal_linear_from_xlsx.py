#!/usr/bin/env python3
"""Per-sensor LINEAR calibration (mV -> N) from 'Final Test Results.xlsx' (sheet 'PVC' = test directly on the insole).

Model used by the app (calibration model "Linear"):
    N = max(0, m * (mV - baseline))        (0 N when mV <= baseline + 4 mV)
where baseline = the zero-load mV set by the app's Zero (auto-zero on connect), and m (N per mV) is fitted here:
all loading points of both PVC runs of a sensor are pooled, saturated points (mV >= 3250) and missing readings are dropped,
and the line is forced through (zero-load mV, 0 N), because that is how it is applied after zeroing.

Also printed for reference: the free fit F = m_free * mV + c (an Excel trendline) and a 0-300 N only fit.

Usage:  python tools/cal_linear_from_xlsx.py "data/Final Test Results.xlsx" [sheet=PVC] > data/linear_cal_pvc.json
Needs:  pip install openpyxl numpy
"""
import json
import sys

import numpy as np
import openpyxl

SAT_MV = 3250
SITES = ["S1", "S4", "S6", "S11", "S13", "S10", "S14"]      # channel 1..7


def read_sheet(path, sheet):
    ws = openpyxl.load_workbook(path, data_only=True)[sheet]
    rows = list(ws.iter_rows(min_row=3, values_only=True))
    out = []
    for s in range(7):
        pts = [(r[2 * s], r[2 * s + 1]) for r in rows
               if r[2 * s] is not None and isinstance(r[2 * s + 1], (int, float))]
        out.append(np.array(pts, float))
    return out


def fit_sensor(p):
    force, mv = p[:, 0], p[:, 1]
    zero = mv[force == 0]
    b0 = float(zero.mean()) if len(zero) else 300.0
    ok = mv < SAT_MV
    res = {"zero_mV": round(b0, 2), "n_points": int(len(force)), "n_saturated_dropped": int((~ok).sum())}
    for label, sel in (("full", ok), ("0_300N", ok & (force <= 300))):
        x, y = mv[sel] - b0, force[sel]
        m = float((x * y).sum() / (x * x).sum())                    # through (baseline, 0)
        r = y - m * x
        mf, c = np.polyfit(mv[sel], y, 1)                           # free line (trendline), for reference
        rf = y - (mf * mv[sel] + c)
        res[label] = {"m_N_per_mV": round(m, 5), "rmse_N": round(float(np.sqrt((r ** 2).mean())), 1),
                      "max_abs_err_N": round(float(np.abs(r).max()), 1), "n": int(sel.sum()),
                      "free_fit_m": round(float(mf), 5), "free_fit_c_N": round(float(c), 1),
                      "free_fit_rmse_N": round(float(np.sqrt((rf ** 2).mean())), 1)}
    return res


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "data/Final Test Results.xlsx"
    sheet = sys.argv[2] if len(sys.argv) > 2 else "PVC"
    data = read_sheet(path, sheet)
    out = {"source": path.replace("\\", "/").split("/")[-1], "sheet": sheet, "model": "N = max(0, m*(mV - baseline))",
           "applied_fit": "full", "sensors": []}
    for ch, p in enumerate(data, 1):
        r = fit_sensor(p)
        r.update({"channel": ch, "sensor_id": SITES[ch - 1]})
        out["sensors"].append(r)
    print(json.dumps(out, indent=2))
    print("// JS: const LIN_M = [" + ", ".join(f"{s['full']['m_N_per_mV']:.5f}" for s in out["sensors"]) + "];",
          file=sys.stderr)


if __name__ == "__main__":
    main()
