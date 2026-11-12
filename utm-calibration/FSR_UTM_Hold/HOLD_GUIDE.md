# FSR_UTM_Hold v1.0 – simple UTM calibration (you signal each hold, you type the force)

No Shimadzu export and no syncing. One CAL row per hold. Two holds per Shimadzu run.

## Setup (5 min)
1. Arduino IDE: ESP32C6 Dev Module, USB CDC On Boot **Enabled**. Check `RF_OHM[]` (line ~45) = the fitted resistors. Upload.
2. Either Serial Monitor (line ending **Newline**), or, better (saves the file): close the Serial Monitor and run
   `python utm_pc_logger.py COM7 --baud 115200 --out utm_data`   (`pip install pyserial` once).
   Use the COM port the Arduino IDE shows. On the UART port (CH343) `--baud 115200` is required; on the native USB port it is ignored.
   If the board is silent, add `--no-dtr`.
3. Type: `t30` (Shimadzu hold time, or `t10`), `c1` (repeat number), `s1`, then `z` with the puck clear.

## TRAPEZIUM method (4 Areas, Single)
A1 Load/Force → t1 | A2 Hold/Force, Duration 30 s | A3 Load (or Unload)/Force → t2 | A4 Hold/Force, Duration 30 s.
`plan` lists t1/t2 for every run: UP01 100→200 … UP05 900→1000, DN01 1000→900 … DN09 1000→100.

## Each Shimadzu run
| You | Board |
|---|---|
| `run` (puck clear, UTM not started) | 2 s zero point, prints the targets of the next run |
| start the UTM | |
| `h` + Enter as soon as the display shows **hold 1** | measures 27 s (3 s before the hold ends), countdown every 5 s |
| at the prompt type the force on the display **now**, e.g. `100.4` (Enter = target) | saves CAL row |
| `h` when **hold 2** starts, type the force | saves CAL row |

Next run: `run` again. `run DN03` jumps to a run, `run FREE` = no targets. Channel finished: `save` (prints the table, selects the next channel) → move puck → `z` → `run`.

- At the force prompt: `a` = redo (then `h` at the next stable hold), `x` = discard. During the measurement: `x` = cancel.
- `list` shows the points so far. `l` = live mV / provisional N.
- Type `h` within ~2 s of the hold start. If you are late, the window still ends 3 s before the UTM hold ends only if you were at most 3 s late.
- WARN `CLIPPED` = force out of range for this RF (point flag 2). WARN `still moving` = the hold ended early or you were late (flag 4) → redo.
- WARN `no response` = puck off the sensor or wrong channel (`s<n>`).
- Zero point `CHECK` = sensor not recovered from the previous run: wait longer before `run`.

Output columns: `CAL,ms,ch,cycle,run,run_id,hold,dir,set_N,utm_N,mean_mV,sd_mV,min_mV,max_mV,early_mV,drift_mV,trend_mV,base_mV,rs_kOhm,g_uS,xtalk_mV,flags,redo,win_s`
(dir Z/U/D; drift = last 2 s − mean at 3–5 s; trend = change inside the last 2 s). The `_CAL.csv` file opens in insole-app (Calibration check) and `check_cal.py`.
