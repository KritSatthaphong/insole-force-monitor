# Research-grade data export (app 1.5.1-research, firmware 1.4, 2026-11-12)

Scope of this release: **logging, export structure, metadata and data integrity**, plus the two changes you asked for explicitly
(linear calibration from the PVC test, 200 Hz). Not touched: ADC averaging, CoP formula and origin, hysteresis model code, gait/IMU
analysis, real-time visualisation. Backups of the previous files: `backup_20261112_before_research_export/` (local) and the Git tag
`backup-before-research-export` (GitHub).

## 1. Audit: what existed, what was missing

| Topic | Before (app 1.4 / firmware 1.3) | Verdict |
|---|---|---|
| Acquisition | 50 Hz, 7 channels x 8 ADC reads averaged, 12 bit, 11 dB, `analogReadMilliVolts`; IMU 200 Hz averaged per frame | kept, rate raised to 200 Hz |
| USB line | `D,<ms>,<7 mV>[,<7 IMU>]`, `ms = millis()` after the ADC burst, no sample counter | added `seq` and `us` at the END of the line |
| BLE | v1: 16-byte notify, 16-bit counter, **no timestamp**, time = counter x 20 ms | new batched v2 packet with `seq` + `us`; v1 still read |
| `dev_ms` in CSV | USB: millis (integer); BLE: counter x 20 ms **written as if it were a device time** | BLE v1 `dev_ms` is now blank and `time_src = seq_est` |
| Calibration | per-sensor power curve `F = a*G^b` with hysteresis (rigid-plate UTM data) | kept as selectable "Curve"; **Linear** (PVC) is the new default |
| Force sum / CoP | `Fz = sum Fi`, `COP = sum(ri*Fi)/sum(Fi)`, blank when Fz <= 10 N | unchanged |
| CoP origin (**active**) | **middle of the insole outline box** (45.365, 136.005 mm from the CAD corner), set on 2026-11-12. CSVs recorded before that used the CAD corner. | preserved; transform to the corner frame exported |
| CSV | one combined file, both feet, 2 `#` comment lines (refreshed at Stop), N with 1 decimal | existing 29 columns unchanged; 6 appended; 3 decimals |
| Config snapshot | CSV comment written again at Stop (silently replaces the start values) | frozen snapshot + events |
| Events, IDs, metadata, per-foot files | none | new |

## 2. Change log

**Firmware `FSR_Insole_App.ino` 1.3 -> 1.4** (compiled for ESP32-C6 with USB CDC On Boot enabled and disabled; not flashed)
- `SAMPLE_HZ` 50 -> 200 (frame 5000 us). ADC burst, IMU code, gyro bias unchanged.
- Sample counter `seq` (uint32) and `us = micros()` taken just before the ADC burst; appended to the USB line; sent in BLE v2.
- BLE v2 characteristic `6E400005`: 4 samples per notification (50 notifications/s). v1 characteristics are no longer created.
- Serial 921600 baud (matters only for the CH343/UART port) and a larger TX buffer; native USB ignores the baud rate.
- `#TIMING` line every 2 s and on `?`: `adc_us_avg/max, frame_us_max, late, late_us_max, over, resync, ble_mtu`; `#INFO` now also has `oversample`, `mac`, `build`.

**App `insole-app/index.html` 1.4 -> 1.5.1-research** (1.5.1, 2026-11-13: USB baud auto-fallback, see below)
- Calibration: new model **Linear** `N = max(0, m*(mV - baseline) - tare)`, 0 N below baseline + 4 mV; selector in the Calibration dialog; old curve model untouched and selectable. A calibration saved in your browser keeps its curve values but switches to Linear once.
- Parser: accepts 9/11/16/18-field `D` lines (anything else is a logged `bad_line`), `#INFO`, `#TIMING`, `#WARN`; BLE v2 and v1; micros() unwrapped across the 71.6 min rollover.
- Sequence tracking: missing / duplicate / out-of-order / MCU restart, per recording.
- Zero now averages 1 s of host time (was a fixed 50 samples), so it still means 1 s at 200 Hz.
- COP trail on the live insole keeps 600 points (was 150) so it shows the same time span at 200 Hz.
- CSV: N, Fz, COP with 3 decimals, `t_ms` 3 decimals, `dev_ms` 3 decimals; columns appended (section 3).
- Study fields (Participant, Run, Condition, Speed; Mass was already there), duplicate-ID warning, Run auto-increments after Stop, live line: ID, source (DEMO flagged), duration, samples, missing, duplicates.
- Research export (ZIP, or 3 separate files) on Stop (`+ ZIP`, default on) and from Logs (`ZIP`, `3 files`). Existing CSV and XML downloads unchanged.
- Events, frozen configuration snapshot, metadata JSON; everything is saved with the log every second, so a crashed/recovered recording can still be exported (metadata says `recovered_log: true`).
- Footer shows the app version (check it after a hard refresh).
- **1.5.1 fix:** USB connect first tries 921600 baud (firmware 1.4 on the CH343/UART port); if no recognisable line arrives within 2.5 s it retries 115200 (older firmware) and, if still silent, shows what it received and what to check. Without this, a board on older firmware looked like "USB - waiting for data".

**New files**: `tools/cal_linear_from_xlsx.py`, `tools/validate_export.py`, `tools/hw_test_usb.py`, `data/linear_cal_pvc.json`, `docs/`, `docs/examples/P00_R01/` (simulated).

## 3. CSV schema (`<ID>_insole_L.csv`, one row = one sample of that insole, as received)

`#` comment lines first (read with `comment='#'`): research header, optional `# DEMO` warning, the start-of-recording legacy comment. Pure ASCII.

| Column | Meaning |
|---|---|
| `t_ms` | host time since recording start, mapped from the device clock when there is one (internal app timebase) |
| `foot`, `src` | `L`/`R`; `USB`, `BLE` or `Demo` |
| `S1_mV S4_mV S6_mV S11_mV S13_mV S10_mV S14_mV` | raw mV (channel 1..7 in this order) |
| `S*_N` | calibrated force: `max(0, F - tare)`, unscaled |
| `Fz_N` | sum of the seven `S*_N` (unscaled) |
| `COPx_mm`, `COPy_mm` | app CoP, origin = middle of the insole; **blank when Fz <= 10 N**, never (0,0) |
| `dev_ms` | device time in ms; blank when none was measured |
| `ax_g ay_g az_g gx_dps gy_dps gz_dps apk_g` | IMU, sensor axes; accel/gyro = mean of the frame (includes gravity), `apk` = peak dynamic acceleration |
| `Fz_BW_N` | `Fz_N x` body-weight factor, blank if none |
| `sample_seq` | MCU counter (BLE v1: unwrapped 16-bit counter); blank for Demo |
| `dev_us` | MCU `micros()` before the ADC burst, unwrapped; only when `time_src = mcu_us` |
| `time_src` | `mcu_us`, `mcu_ms` (older USB firmware, millis at frame end), `seq_est` (BLE v1: no device time), `none` (Demo) |
| `host_rx_utc_ms` | host receive time, Unix UTC ms; for communication diagnostics, not the sampling clock |
| `sensor_valid_mask` | bit i = sensor i delivered a finite mV > 0 (bit0 S1, bit1 S4, bit2 S6, bit3 S11, bit4 S13, bit5 S10, bit6 S14) |
| `saturation_mask` | bit i = mV >= 3250 (limit of the front end; force is a lower bound) |

Mapping to your list: all requested existing fields keep their names (`t_ms, dev_ms, foot, src, S*_mV, S*_N, Fz_N, COPx_mm, COPy_mm, ax_g..apk_g`). Not available in hardware: raw ADC counts (not exported), disconnected-pin detection.

## 4. Metadata JSON (`<ID>_metadata.json`)

All fields you listed are present. Top level: `schema_version, export_version, recording_id, participant_id, run_id, foot, condition, treadmill_speed_kmh, body_mass_kg, data_source (hardware|demo|mixed), is_hardware_recording, device_id, transport, app_version, firmware_version, source_revision, nominal_sampling_hz, device_time_source, device_time_unit, sample_sequence_definition, time_columns, recording_start_utc, recording_stop_utc, recovered_log, rows_total, files, masks, fz_definition, negative_values_policy, acquisition_note, coordinates, units, csv_columns, feet, configuration_policy, event_counts`.
- `feet.L`: `transport, device_id (ESP32 MAC + USB id, or BLE name), firmware {version, hz, imu, oversample, mac, build}, nominal_sampling_hz, measured_rate_hz, device_duration_s, samples, first/last_sample_seq, missing/duplicate/out_of_order_samples, mcu_restarts, mcu_timing_last (#TIMING), stream_errors, calibration_model, hysteresis_enabled, RF_ohm, global_force_scaling_enabled/coefficient, imu_mounting, sensors[7]`.
- `sensors[i]`: `sensor_id (S1,S4,S6,S11,S13,S10,S14 - not renamed), channel, adc_gpio, position_cop_frame_mm, position_cad_mm, calibration {model, formula, m_N_per_mV, baseline_mV, tare_N, tare_G_uS, dead_band_mV, curve_model_parameters {aU,bU,aD,bD,a,b, in_use}, saturation_mV}`.
- `coordinates`: frame, origin, x/y axis definitions, positive directions (X + = toward the right of the drawing: **medial for the left insole**; Y + = heel -> toe), unit mm, canonical CAD frame, `transform_app_to_cad_3x3 = [[1,0,45.365],[0,1,136.005],[0,0,1]]`, CoP formula and validity rule.
- Global force scaling: `global_force_scaling_enabled` mirrors the app's BW-scale switch (off by default); exported forces are never scaled.
- Source revision: Git tag `v1.5.1-research` (an app cannot know its own commit hash; recordings made with 1.5.0 cite `v1.5.0-research`).
- Full example: `docs/examples/P00_R01/P00_R01_metadata.json` (**simulated**).

## 5. Events CSV (`<ID>_events.csv`)

`event_type,dev_ms,host_utc_ms,sample_seq,details` - `details` is JSON (always has `foot`). Only events that occurred are written; `dev_ms` is empty for host-side events. Written into memory only (never blocks acquisition), saved with the log each second.

| event_type | when |
|---|---|
| `RECORD_START`, `RECORD_STOP` | Record / Stop (a recovered log has no stop) |
| `ZERO`, `TARE_CHANGE` | Zero pressed or auto-zero during the recording: old and new baseline / tare |
| `CALIBRATION_CHANGE` | calibration saved during a recording: old and new values |
| `BLE_DISCONNECT`, `BLE_RECONNECT`, `USB_DISCONNECT` | link lost / restored / closed (`final` true = no more retries or user disconnect) |
| `PACKET_DROP` | forward jump in `sample_seq`: `missing, seq_from, seq_to` |
| `PACKET_DUPLICATE`, `PACKET_OUT_OF_ORDER` | repeated / late sample |
| `STREAM_ERROR` | `bad_line`, `ble_bad_packet`, `ble_short_packet` (MTU too small), `rx_buffer_overflow`, `mcu_restart`, `mcu_warning`; at most one per kind per second, the rest counted |
| `MCU_OVERRUN` | the firmware's `#TIMING` counters `late / over / resync` increased |

## 6. Test results

Legend: **PASS** executed, **FAIL**, **NOT TESTED**. "Simulated" = synthetic lines / mocked Bluetooth through the real app code in a browser; no board was connected.

| Test | Result | Evidence |
|---|---|---|
| Firmware compiles (ESP32-C6, CDC on/off) | PASS | `arduino-cli compile`, 797 KB flash (60 %) |
| Firmware runs on the board, 200 Hz timing, BLE v2 on a real link | **NOT TESTED** | needs hardware |
| A CSV structure | PASS (simulated USB, BLE v2, BLE v1, Demo, recovered log) | `tools/validate_export.py` |
| B Timing: sequence, missing/dup/ooo vs metadata and events, monotonic device time across the micros() wrap, interval/jitter, duration | PASS (simulated USB 200 Hz: median 5.000 ms, jitter sd 0.16 ms; BLE v2) | validator |
| C Fz = sum of forces | PASS, max diff 0.002 N | validator |
| D CoP recomputed, blank rule, CAD transform | PASS, max diff 0.0013 mm | validator |
| E Calibration reproducible from metadata (Linear) | PASS, max diff 0.0005 N | validator |
| ZIP integrity | PASS (CRC-32 of every member read by Python `zipfile`) | validator |
| Recovered (crashed) recording exports | PASS, `recovered_log: true`; its duration check fails only because the synthetic burst fed 3 s of device time in 0.9 s | validator |
| F 30-40 s hardware recording, no unexplained saturation | **NOT TESTED** | needs hardware |
| G Regression: Demo display, curve model, legacy CSV (29 original columns identical), XML, Logs, USB/BLE v1/v2 parsing | PASS (simulated); real USB/BLE **NOT TESTED** | browser tests |
| Throughput | PASS: about 20 000 lines/s parsed and logged (50x the need of two feet at 200 Hz) | browser |

Validator results: simulated USB 200 Hz example 20 PASS / 0 FAIL; BLE v2 21 / 0; BLE v1 17 / 0 / 1 NOT TESTED (no device time); Demo 25 / 0 / 4 NOT TESTED (no seq, no device time). Run it yourself: `python tools/validate_export.py P01_R01.zip`.

## 7. Known issues and hardware-dependent limits

1. **200 Hz ADC timing is unverified.** 56 ADC reads + IMU I2C + output must fit in 5 ms. Read `#TIMING` after 30 s: `late`, `over`, `resync` should stay 0 and `adc_us_max` well below 4000. If not, set `OVERSAMPLE` to 4 in the sketch (the metadata then says so).
2. **BLE at 200 Hz needs ATT MTU >= 151** and has not been tried on a real link. A short packet is reported as `ble_short_packet`. Use USB in the lab if in doubt.
3. **Linear calibration is coarse**: RMS error over 0-1000 N is 13 N (S6) to 38 N (S13), worst single point 28-68 N, and it underestimates at low loads (S1: 99 N reads about 55 N). Raw mV is always exported, so refit offline. The unloading leg and hysteresis are not modelled. `data/linear_cal_pvc.json` has per-sensor numbers including the 0-300 N fit.
4. A bare disconnected pin is not detectable by the hardware; `sensor_valid_mask` only flags missing/zero readings.
5. `t_ms` follows the host clock (it is aligned to the device clock with a smallest-offset rule); use `dev_us` / `dev_ms` for synchronisation with the zebris recording, `host_rx_utc_ms` only for approximate alignment (the PC clock must be correct).
6. At 200 Hz the IMU is about one reading per frame (previously a mean of 4). The gait / IMU **Results** analysis was not changed or re-validated at 200 Hz (e.g. the "still" test is 3 samples = 15 ms).
7. CoP X sign differs by foot in the drawing frame (+X is medial for the left, lateral for the right insole).
8. ZIP is uncompressed; a 40 min two-foot recording is roughly 200 MB of CSV and is built in memory. Chrome may ask to allow multiple downloads: allow it.
9. Firmware 1.4 no longer sends BLE v1 packets, so a pre-1.5 web page cannot receive BLE from it (USB still works).
10. The CSV `Fz` is unscaled and reads far below body weight (sensors cover about 35 of 180 cm2); that is expected, scaling is to be fitted offline.

## 8. Pre-lab checklist

1. Flash `firmware/FSR_Insole_App/FSR_Insole_App.ino` v1.4 (CDC On Boot: Enabled, native USB port VID 303A). Close the Serial Monitor before using the app.
2. **Fastest way: `python tools/hw_test_usb.py COM6`** (find the port with `--list`; close the Serial Monitor and the web app first). It records 35 s over USB, leave the insole unloaded for the first 5 s, then press every sensor / walk, and prints PASS / WARN / FAIL for firmware version, 200 Hz rate, missing samples, MCU timing jitter, `#TIMING` counters, unloaded mV, |a|, channel response and saturation. Zero FAIL = the board is ready. Or by hand: in the Serial Monitor (921600 or any, native USB) send `?`: `v1.4`, `hz=200`, `imu=BMI270`, `|a|` about 1.00 g. Leave it 30 s on a table and check `#TIMING`: `late=0 over=0 resync=0`, `adc_us_max` < 4000. If not, lower `OVERSAMPLE`.
3. Open the live site, hard refresh (Ctrl+Shift+R), footer must say `App 1.5.1-research`.
4. Connect the left board by USB. Message must say "Zeroed" with the insole unloaded and off the floor. Press Zero again if needed (not during a recording).
5. Fill Participant, Run, Condition, Speed, Mass. Never record with Demo (the file name gets `_DEMO` and metadata says `demo`).
6. 30-40 s test recording: the live line must show `missing 0`. Stop, open the ZIP, run `python tools/validate_export.py <file>.zip`. Zero FAIL expected.
7. Check the PC clock, keep the page open and the laptop awake; after each run keep both the ZIP and the original CSV (Logs keeps a copy inside the browser, but clearing site data deletes it).
8. If the status shows BLE problems or `ble_short_packet`, switch to USB.

## 9. How to run and export

- Live: https://kritsatthaphong.github.io/insole-force-monitor/ - or locally `python -m http.server 8765 --directory insole-app` and open http://localhost:8765.
- Record -> Stop & save downloads (always) the combined CSV, and with `+ ZIP` ticked also `P01_R01.zip` containing the folder `P01_R01/` with `P01_R01_insole_L.csv`, `P01_R01_metadata.json`, `P01_R01_events.csv` (`_insole_R.csv` too if the right foot was recorded). Later: Logs -> `ZIP` or `3 files`.
- Linear slopes: `python tools/cal_linear_from_xlsx.py "Final Test Results.xlsx" PVC > data/linear_cal_pvc.json` (needs `openpyxl numpy`).

## 10. Implemented vs already present

- **Already present before**: raw mV and N per sensor, Fz sum, CoP and its centred origin, hysteresis calibration code, Zero/tare, BLE 16-bit counter with wrap handling and loss counter, crash-safe autosave and Logs, IMU columns, XML export, Demo.
- **Implemented now**: everything in section 2.
- **Tests executed**: section 6 (simulated). **Need hardware**: board timing at 200 Hz, BLE v2 over a real link, test F, real saturation behaviour.
- **Remaining risks before the lab**: items 1-3 and 6 of section 7.
