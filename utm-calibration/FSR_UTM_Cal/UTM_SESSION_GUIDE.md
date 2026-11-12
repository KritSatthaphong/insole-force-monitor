# UTM calibration session guide (FSR_UTM_Cal v3.0)

Shimadzu AG-X plus, TRAPEZIUM X **Single** method with 4 Areas. One machine run = one ESP32 file + one Shimadzu raw export, paired afterwards on the PC.

## 0. Files

| File | What it is |
|---|---|
| `FSR_UTM_Cal/FSR_UTM_Cal.ino` | v3.0 sketch. Adds RUN mode (continuous 200 Hz recording of a whole machine run), run plan, `k` marks, pre/post baselines, on-board hold detection + QC, per-channel RF, settings kept in flash. POINT mode (`start` / `m` / `save`) unchanged from v2.1 |
| `FSR_UTM_Cal/utm_pc_logger.py` | v3 logger. Run files, events with PC time, `runs_log.csv`, `holds_log.csv`, reconnect after a cable drop, board-restart detection. **v2 logger does not save the new lines: replace it** |
| `tools/utm_pair.py` | after the session: pairs each ESP32 run file with its Shimadzu export, one row per hold → `holds_table.csv` + plot per run |
| `dev/` | PC simulation + unit test of the sketch (for development, not needed in the lab) |

## 1. Before the lab (once)

1. Arduino IDE: board **ESP32C6 Dev Module**, **USB CDC On Boot: Enabled**, core 3.3.12. Open `FSR_UTM_Cal.ino`, check `RF_OHM[]` = resistors actually fitted (measure them; 27k on some channels → write it there). Upload.
2. `pip install pyserial numpy matplotlib`
3. Close the Serial Monitor. `python utm_pc_logger.py --out D:\capstone\utm_2026xxxx`
4. Type once (stored in flash, written into every file header):

```
meta foot L
meta board B1
meta fixture rigid steel + acrylic puck 25.4 mm centred
meta cell 20 kN SN xxxxx (or the low-force cell)
meta cell_N 20000
meta operator Krit
meta vref_mV 291
set p 1000
set step 100
set dwell 30
set ramp 10
set rest 120
set rep 1
plan
```

`meta sensor <A401 ID>` per sensor. `cell_N` makes the pairing tool flag holds below 2 % of the cell rating (below 400 N on a 20 kN cell, see §6).

## 2. TRAPEZIUM X methods (make and save two templates)

Compression, N. Chart **Force vs Time**. Samplings: time-based, **50 ms**. Ramps end on **Sensor / Force / target**, holds end on **Duration / 30 s**. Same rate (10 N/s) in Areas 1 and 3 of every run.

| Template | Area 1 | Area 2 | Area 3 | Area 4 |
|---|---|---|---|---|
| LOADING (UP) | Load / Force → lower target | Hold / Force, 30 s | Load / Force → upper target | Hold / Force, 30 s, then end action |
| DESCENDING (DN) | Load / Force → P | Hold / Force, 30 s | **Unload** / Force → lower target | Hold / Force, 30 s, then end action |

`plan` prints the exact targets for every run, e.g. `UP03: A1 Load/Force to 500 N | A2 Hold 30 s | A3 Load/Force to 600 N | A4 Hold 30 s`. Before each run, change only the two targets.

After every run: **Save Raw Data to CSV File** (Time, Force, Stroke). Name it like the ESP32 file: `UP03_S1_R1.csv`.

## 3. Per sensor

1. Puck centred on the sensor, tail taped, LiPo **disconnected**, same USB supply as the test.
2. `s<n>` and check the line `selected S<n> (GPIOx, site ..., RF ...)`. `meta sensor <ID>`.
3. Puck clear → `z` (all 7 must say ok).
4. Conditioning: `run COND` → `k` at each Start → 5× 0→P→0 (as the lab sets it up) → `end`. One board file can hold several UTM starts. Then rest 60 s.
5. Pilot (first sensor only): `run PILOT1` with one 30 s hold. Check that the board shows `hold 1 ... OK` and the export shows ramp → flat plateau. Then one UP and one DN run, check both traces, then start the matrix.
6. Matrix: `run` (takes the next run not done) ×14. Order: UP01..UP05, then DN01..DN09.
7. Next repeat: `set rep 2` (clears the done list), run all 14 again.

## 4. Per run (about 4–6 min)

```
run              (or run UP03)  -> pre-baseline 3 s, puck must be clear
k                (Enter at the moment you click Start in TRAPEZIUM)
...              (watch: load detected, hold 1 started, hold 1 ... OK, hold 2 ... OK, unloaded)
end              (after the UTM has returned)  -> post-baseline 3 s, close file
```

- `x` aborts the run (file kept, marked ABORTED, not done).
- `end` while the sensor is still loaded asks first; anything but `y` keeps recording.
- If the board says `CHECK`, the run is not marked done. Look at the export; repeat it, or accept it with `done UP03`.
- `n<text>` writes a note into the open file (fixture moved, puck re-centred, UTM alarm, ...).
- The board warns if the rest since the last run is shorter than `set rest`. Use one fixed rest for every run.

**Messages to act on**

| Message | Meaning / action |
|---|---|
| `WARN,S2 is loaded ... but the selected S1 is not: WRONG CHANNEL` | `x`, then `s2`, `run` again |
| `WARN,... above 3000 mV: near the ADC rail` | still valid, note it |
| `WARN,... above 3250 mV: CLIPPED` | that force is out of range for this RF. The hold is rejected later. Lower RF (27k) or lower P for this sensor |
| `pre-baseline ... CHECK` | sensor not recovered or still loaded. Wait, `x`, run again |
| `hold k ... CHECK short` | hold shorter than 0.9 × dwell. Check the Area durations |
| `n hold(s) detected, 2 expected` | method or change point wrong. Check the export before going on |

## 5. After the session: pairing

```
python tools\utm_pair.py --pairs pairs.csv --out holds_table.csv --plots pair_plots
```

`pairs.csv` (paths relative to this file):

```
esp_file,utm_file,offset_s,window,reject,note
utm_data\UP01_S1_R1_20261112_101500.csv,trapezium\UP01_S1_R1.csv,,,,
utm_data\DN05_S1_R1_20261112_104210.csv,trapezium\DN05_S1_R1.csv,,,,
```

Each run is aligned by time (correlation of UTM force with FSR conductance, starting at the `k` mark, refined on the ramp starts and ends). Timing uncertainty = disagreement of separate fits on the first and second half. Per hold: final 2 s (minus a guard of 0.3 s + 2 × uncertainty) in both streams, early 3–5 s window for 30 s drift, overshoot, measured ramp rate, time since load. **Open every plot once.** Blue = window used, red = rejected.

`holds_table.csv`: first column `CAL` = accepted U/D/Z rows, `PEAK` = common-peak holds of DN runs, `REJ` = failed QC (rows are kept). The table opens in insole-app (Calibration → Calibration check) and in `tools/check_cal.py` directly.

**QC flags** (the `qc_flags` column)

| Rejects (REJ) | Warnings only |
|---|---|
| `clip` FSR > 3250 mV in the window | `setpoint` mean force off target by > max(2 N, 1 %) |
| `pp_big` UTM peak-to-peak > 3 × max(2 N, 1 %) | `pp` peak-to-peak > max(2 N, 1 %) |
| `no_fsr`, `missing_big` FSR samples missing (> 5 %) | `missing` a few frames missing |
| `align_big` timing uncertainty > 0.5 s | `align` uncertainty > 0.1 s |
| `window` final window does not fit inside the hold | `near_rail` > 3000 mV |
| `extra_hold` plateau the template did not expect | `short_hold`, `hold_count`, `utm_n` |
| `manual_reject` | `below_2pct_cell` target < 2 % of `cell_N` |

Manual fixes (each logged to `annotations_log.csv` with `--who`): `--offset 25.36`, `--window "2=100.0:102.0"` (UTM seconds), `--reject "1:puck moved"`. Same fields as columns in `pairs.csv`.

## 6. Validity notes from the handoff

- The AG-X plus manual gives force-control error > 2 % of the cell rating: > 400 N for a 20 kN cell. 100–400 N holds on the 20 kN cell are flagged `below_2pct_cell`. Use a calibrated lower-capacity cell for them, or report them as reference-limited. A 1 kN cell has no headroom at 1000 N.
- Workbook sensors S3, S4, S6, S7 may clip near 800–1000 N at 39k. Watch the CLIPPED warning on UP05 / DN runs. If you change RF, change `RF_OHM[]` and re-flash **before** recording.
- Check S6 (ch6) contact and S4 before the full batch.
- Keep the same puck, backing, USB supply, ramp rate, dwell and rest within a comparison. Change only one thing at a time and note it.

## 7. What has been checked

| Item | Status |
|---|---|
| Sketch v3.0 compiles, ESP32-C6, core 3.3.12, IDE prototype generator (also v2.1) | done here |
| Hold detector: unit test with simulated 10 N/s and 2 N/s runs, noise 2–4 mV, creep, short pauses | done here |
| Full workflow: real sketch code on a PC simulation of the board + logger + pairing (UP01, DN05, abort, wrong channel, end under load, point mode, RAW) | done here (simulated, not hardware) |
| Pairing: offset recovered within 25 ms of the simulated truth; exports in N / kN, `,` / tab, UTF-8 / UTF-16, s / ms, negative compression sign | done here (simulated exports) |
| Flash + real ADC + real USB on the board | **not yet: do the bench check below** |
| Real TRAPEZIUM X export format | **not yet: send me one export from the pilot** |

**Bench check before the lab (10 min):** flash, start the logger, `?` (shows v3.0), `z`, `s1`, `set dwell 30`, `run PILOT1`, `k`, put a mass (1–2 kg) on the puck for ~35 s, lift it, put it back ~35 s, lift, `end`. You should see `load detected`, `hold 1 ... OK`, `hold 2 ... OK`, `FILE_CLOSE ... PASS` and in the PC line `200.00 Hz`, `frame gaps 0`. Then `s2` with the mass on S1 and `run PILOT2`: the WRONG CHANNEL warning must appear (`x` to abort). Also check one ADC value near the top with a meter (short one FSR, compare `l` live mV with the meter at the ADC pin).
