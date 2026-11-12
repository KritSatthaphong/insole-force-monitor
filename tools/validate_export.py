#!/usr/bin/env python3
"""Validate a research export of the Insole Force Monitor (acceptance tests A-E, offline).

Usage:   python tools/validate_export.py P01_R01.zip          (or the folder / the three files' folder)
Needs:   numpy (pip install numpy)

Tests
  A  CSV structure: required columns, 7 sensors, field count per row, row count = metadata, IDs consistent across the 3 files
  B  Timing: sample_seq order / missing / duplicates vs metadata and events, device time monotonic, interval + jitter, duration
  C  Force consistency: Fz_N = sum of the 7 S*_N (tolerance for 3-decimal rounding)
  D  CoP consistency: recomputed from the 7 forces and the sensor positions in the metadata, blank when Fz <= 10 N, CAD transform in the outline box
  E  Metadata reproducibility: S*_N rebuilt from raw mV with the calibration in the metadata (linear model; before any config-change event)
Exit code 0 = no FAIL (NOT TESTED / SKIPPED do not fail).
"""
import csv
import io
import json
import sys
import zipfile
from pathlib import Path

import numpy as np

SITES = ["S1", "S4", "S6", "S11", "S13", "S10", "S14"]
REQUIRED = (["t_ms", "dev_ms", "foot", "src"] + [f"{s}_mV" for s in SITES] + [f"{s}_N" for s in SITES] +
            ["Fz_N", "COPx_mm", "COPy_mm", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps", "apk_g",
             "sample_seq", "dev_us", "time_src", "host_rx_utc_ms", "sensor_valid_mask", "saturation_mask"])
results = []


def record(test, name, status, detail=""):
    results.append((test, name, status, detail))


def load(path):
    p = Path(path)
    files = {}
    if p.suffix.lower() == ".zip":
        z = zipfile.ZipFile(p)
        bad = z.testzip()
        record("ZIP", "zip integrity (CRC-32 of every member)", "PASS" if bad is None else "FAIL", f"first bad member: {bad}" if bad else f"{len(z.namelist())} members: {', '.join(z.namelist())}")
        for n in z.namelist():
            files[n.split("/")[-1]] = z.read(n).decode("utf-8")
    else:
        d = p if p.is_dir() else p.parent
        for f in d.iterdir():
            if f.suffix.lower() in (".csv", ".json"):
                files[f.name] = f.read_text(encoding="utf-8")
    return files


def parse_csv(text):
    lines = text.splitlines()
    comments = [l for l in lines if l.startswith("#")]
    body = [l for l in lines if l and not l.startswith("#")]
    rdr = csv.reader(io.StringIO("\n".join(body)))
    head = next(rdr)
    rows = list(rdr)
    return comments, head, rows


def col(head, rows, name, dtype=float):
    i = head.index(name)
    out = []
    for r in rows:
        v = r[i]
        out.append(np.nan if v == "" else dtype(v))
    return np.array(out, float)


def main(path):
    files = load(path)
    meta_name = next((n for n in files if n.endswith("_metadata.json")), None)
    if not meta_name:
        print("no *_metadata.json found")
        return 2
    meta = json.loads(files[meta_name])
    rid = meta["recording_id"]
    events_name = f"{rid}_events.csv"
    # ---------- A ----------
    record("A", "three file types present with the same Recording ID",
           "PASS" if events_name in files and any(n.startswith(f"{rid}_insole_") for n in files) else "FAIL", f"id={rid}, files={sorted(files)}")
    ev_rows = list(csv.DictReader(io.StringIO(files.get(events_name, "event_type,dev_ms,host_utc_ms,sample_seq,details\n"))))
    record("A", "events.csv header = event_type,dev_ms,host_utc_ms,sample_seq,details",
           "PASS" if ev_rows == [] or list(ev_rows[0].keys()) == ["event_type", "dev_ms", "host_utc_ms", "sample_seq", "details"] else "FAIL")
    non_ascii = [n for n, t in files.items() if any(ord(c) > 127 for c in t)]
    record("A", "all three file types are plain ASCII (safe to open on any locale)", "PASS" if not non_ascii else "FAIL", f"non-ASCII in: {non_ascii}" if non_ascii else "")
    ev_types = [e["event_type"] for e in ev_rows]
    need = ["RECORD_START"] + ([] if meta.get("recovered_log") else ["RECORD_STOP"])      # a recovered (crashed) recording never reached Stop
    record("A", "RECORD_START" + ("" if meta.get("recovered_log") else " and RECORD_STOP") + " events present" + (" (recovered log: no stop)" if meta.get("recovered_log") else ""),
           "PASS" if all(n in ev_types for n in need) else "FAIL", f"{len(ev_rows)} events: " + str({t: ev_types.count(t) for t in set(ev_types)}))
    for foot in meta["foot"]:
        name = f"{rid}_insole_{foot}.csv"
        if name not in files:
            record("A", f"{name} present", "FAIL")
            continue
        comments, head, rows = parse_csv(files[name])
        tag = f"[{foot}] "
        miss = [c for c in REQUIRED if c not in head]
        record("A", tag + "required columns", "PASS" if not miss else "FAIL", f"missing: {miss}" if miss else f"{len(head)} columns")
        record("A", tag + "every row has the header's field count", "PASS" if all(len(r) == len(head) for r in rows) else "FAIL")
        record("A", tag + "foot column only " + foot, "PASS" if all(r[head.index("foot")] == foot for r in rows) else "FAIL")
        exp = meta["feet"][foot]["samples"]
        record("A", tag + "row count = metadata samples", "PASS" if exp == len(rows) else "FAIL", f"csv {len(rows)} vs metadata {exp}")
        record("A", tag + f"recording id {rid} in CSV comment", "PASS" if any(rid in c for c in comments) else "FAIL")
        if not rows:
            continue
        # ---------- B ----------
        seq = col(head, rows, "sample_seq")
        src = rows[0][head.index("time_src")]
        if np.isnan(seq).all():
            record("B", tag + "sample_seq", "NOT TESTED" if src == "none" else "FAIL", "Demo / simulated source: no MCU sequence" if src == "none" else "sample_seq column is empty")
        else:
            # same rule as the app: compare with the highest sequence received so far; a forward jump of d adds d-1 missing,
            # equal = duplicate, lower = out of order (a late sample does not reduce 'missing'); a drop to < 1000 after > 2000 = MCU restart
            missing = dup = ooo = restarts = 0
            last = None
            for s in seq:
                if last is None:
                    last = s
                    continue
                d = s - last
                if d == 1:
                    last = s
                elif d > 1:
                    missing += int(d - 1)
                    last = s
                elif d == 0:
                    dup += 1
                elif s < 1000 and last > 2000:
                    restarts += 1
                    last = s
                else:
                    ooo += 1
            m = meta["feet"][foot]
            record("B", tag + "sequence: missing / duplicate / out-of-order match metadata",
                   "PASS" if (missing, dup, ooo) == (m["missing_samples"], m["duplicate_samples"], m["out_of_order_samples"]) else "FAIL",
                   f"csv {missing}/{dup}/{ooo} vs metadata {m['missing_samples']}/{m['duplicate_samples']}/{m['out_of_order_samples']}")
            drop_ev = sum(json.loads(e["details"]).get("missing", 0) for e in ev_rows if e["event_type"] == "PACKET_DROP" and json.loads(e["details"]).get("foot") == foot)
            record("B", tag + "PACKET_DROP events add up to the missing samples", "PASS" if drop_ev == missing else "FAIL", f"events {drop_ev} vs seq gaps {missing}")
        if src in ("mcu_us", "mcu_ms"):
            dev = col(head, rows, "dev_ms")
            dd = np.diff(dev)
            back = int(np.sum(dd < 0))
            wrapped = src == "mcu_us" and np.nanmax(col(head, rows, "dev_us")) > 2 ** 32
            if not np.isnan(seq).all():     # arrival order is preserved: a late or repeated sample legitimately steps back in time
                ok = back <= dup + ooo
                record("B", tag + "device timestamp monotonic (uint32 rollover handled); every backwards step is a duplicate / out-of-order sample",
                       "PASS" if ok else "FAIL", f"{back} backwards steps, {dup + ooo} duplicate/out-of-order samples" + ("; stream crossed the micros() wrap" if wrapped else ""))
            else:
                record("B", tag + "device timestamp monotonic", "PASS" if back == 0 else "FAIL", f"{back} backwards steps")
            nom = meta["feet"][foot]["nominal_sampling_hz"] or 0
            if nom:
                per = 1000.0 / nom
                ok = dd[(dd > 0) & (dd < 3 * per)]
                med = float(np.median(ok)) if len(ok) else float("nan")
                record("B", tag + "median sample interval = 1/nominal rate", "PASS" if abs(med - per) < 0.1 * per else "FAIL",
                       f"median {med:.3f} ms (nominal {per:.3f} ms), jitter sd {float(np.std(ok)):.3f} ms, max {float(ok.max()):.3f} ms, longest gap {float(dd.max()):.1f} ms")
            tt = col(head, rows, "t_ms")
            dur_dev, dur_t = (dev[-1] - dev[0]) / 1000, (tt[-1] - tt[0]) / 1000
            record("B", tag + "recording duration: device clock vs t_ms", "PASS" if abs(dur_dev - dur_t) < max(0.05, 0.002 * dur_dev) else "FAIL",
                   f"device {dur_dev:.3f} s, t_ms {dur_t:.3f} s")
            if src == "mcu_us":
                us = col(head, rows, "dev_us")
                record("B", tag + "dev_us = dev_ms x 1000", "PASS" if np.allclose(us, dev * 1000, atol=1.0) else "FAIL")
        else:
            record("B", tag + "device timestamp", "NOT TESTED", f"time_src = {src}: no measured device time in this recording")
        # ---------- C ----------
        N = np.column_stack([col(head, rows, f"{s}_N") for s in SITES])
        Fz = col(head, rows, "Fz_N")
        err = np.abs(Fz - N.sum(axis=1))
        record("C", tag + "Fz_N = sum of the 7 S*_N", "PASS" if err.max() <= 0.005 else "FAIL", f"max |diff| {err.max():.4f} N over {len(rows)} rows (tolerance 0.005 N = 3-decimal rounding)")
        # ---------- D ----------
        pos = np.array([s["position_cop_frame_mm"] for s in meta["feet"][foot]["sensors"]])
        cad = np.array([s["position_cad_mm"] for s in meta["feet"][foot]["sensors"]])
        cx, cy = col(head, rows, "COPx_mm"), col(head, rows, "COPy_mm")
        on = Fz > 10
        with np.errstate(invalid="ignore", divide="ignore"):
            rx, ry = (N * pos[:, 0]).sum(axis=1) / N.sum(axis=1), (N * pos[:, 1]).sum(axis=1) / N.sum(axis=1)
        blank_ok = bool(np.all(np.isnan(cx[~on])) and np.all(np.isnan(cy[~on])) and not np.any(np.isnan(cx[on])))
        record("D", tag + "COP blank exactly when Fz_N <= 10 N (never 0,0)", "PASS" if blank_ok else "FAIL", f"{int(on.sum())} loaded rows, {int((~on).sum())} blank rows")
        if on.any():
            e = max(np.abs(cx[on] - rx[on]).max(), np.abs(cy[on] - ry[on]).max())
            record("D", tag + "COP recomputed from S*_N and sensor positions", "PASS" if e <= 0.06 else "FAIL", f"max |diff| {e:.4f} mm (tolerance 0.06 mm)")
            M = np.array(meta["coordinates"]["transform_app_to_cad_3x3"])
            xc, yc = cx[on] + M[0, 2], cy[on] + M[1, 2]
            inside = np.all((xc >= -1) & (xc <= 91.8) & (yc >= -1) & (yc <= 273)) and np.allclose(pos + M[:2, 2], cad, atol=0.01)
            record("D", tag + "transform to the CAD frame: COP inside the outline box, sensor positions consistent", "PASS" if inside else "FAIL")
        # ---------- E ----------
        mv = np.column_stack([col(head, rows, f"{s}_mV") for s in SITES])
        sens = meta["feet"][foot]["sensors"]
        if sens[0]["calibration"] and sens[0]["calibration"]["model"] == "linear":
            host = col(head, rows, "host_rx_utc_ms")
            chg = [float(e["host_utc_ms"]) for e in ev_rows if e["event_type"] in ("ZERO", "TARE_CHANGE", "CALIBRATION_CHANGE") and json.loads(e["details"]).get("foot") in (foot, None)]
            sel = host < min(chg) if chg else np.ones(len(rows), bool)
            m_ = np.array([s["calibration"]["m_N_per_mV"] for s in sens])
            b_ = np.array([s["calibration"]["baseline_mV"] for s in sens])
            t_ = np.array([s["calibration"]["tare_N"] for s in sens])
            dead = sens[0]["calibration"]["dead_band_mV"]
            F = np.where(mv > b_ + dead, m_ * (mv - b_), 0.0)
            Nr = np.maximum(0, F - t_)
            e = np.abs(Nr[sel] - N[sel]).max() if sel.any() else 0
            record("E", tag + "S*_N rebuilt from raw mV with the metadata calibration (linear)", "PASS" if e <= 0.0006 else "FAIL",
                   f"max |diff| {e:.5f} N on {int(sel.sum())} rows" + (f"; rows after the first config-change event excluded ({len(chg)} such events)" if chg else ""))
        else:
            record("E", tag + "force reconstruction", "NOT TESTED", "curve model (stateful hysteresis) not reproduced by this script")
    print(f"\nValidation of {path}  (recording {rid}, data_source {meta['data_source']}, hardware={meta['is_hardware_recording']})\n")
    w = max(len(r[1]) for r in results)
    for t, n, s, d in results:
        print(f"[{t}] {s:<10} {n:<{w}}  {d}")
    bad = [r for r in results if r[2] == "FAIL"]
    print(f"\n{len([r for r in results if r[2] == 'PASS'])} PASS, {len(bad)} FAIL, {len([r for r in results if r[2] == 'NOT TESTED'])} NOT TESTED")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]) if len(sys.argv) > 1 else print(__doc__))
