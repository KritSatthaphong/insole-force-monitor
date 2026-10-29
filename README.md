# Insole Force Monitor

Live per-sensor force map for the A401 smart insole (EU42, 7 sensors per foot), both feet side by side.
Each foot stays greyed out until a board (or Demo) is connected to it.

**Open the app:** https://kritsatthaphong.github.io/insole-force-monitor/ (Chrome / Edge on a computer, Chrome on Android,
Bluefy on iPhone). Press **Demo** to try it without hardware.

| Folder | What |
|---|---|
| `firmware/FSR_Insole_App/` | ESP32-C6 sketch: raw mV of 7 channels at 50 Hz over USB serial **and** BLE |
| `insole-app/index.html` | The app (single file, no install) |
| `tools/cal_from_xlsx.py` | Per-sensor loading / unloading fits from the UTM summary workbook (`S1-S7test results.xlsx`) — the app's defaults |
| `tools/fit_cal.py` | Older single-curve fit from the raw UTM CAL CSVs |

## 1. Flash the boards
Arduino IDE → board "ESP32C6 Dev Module", core 3.3.x, **USB CDC On Boot: Enabled** (Disabled also builds; then use
the CH343 port). No extra libraries.
Flash the same sketch on both boards, then in Serial Monitor (115200) send `fL` to the left board and `fR` to the right one
(saved in flash; BLE name becomes `Insole-L` / `Insole-R`). Close Serial Monitor before using the app over USB.

## 2. Open the app
- **Laptop (USB or BLE), Chrome or Edge:** in the `insole-app` folder run `python -m http.server 8765`, open http://localhost:8765.
- **Anywhere (USB or BLE, incl. phones):** open the GitHub Pages link above. In **Chrome on Android** it works as is;
  iPhone Safari has no Web Bluetooth, so use the free **Bluefy** browser on iPhone.

Press **USB** / **Bluetooth** on the foot you want. The app zeroes automatically in the first second if the insole is unloaded;
otherwise press **Zero** with the foot off the insole.

## 3. Notes
- Channel → site (heel to toe): ch1 S1, ch2 S4, ch3 S6, ch4 S11, ch5 S13, ch6 S10, ch7 S14.
- Force: G = (mV/base − 1)/39 kΩ, F = a·G^b with each sensor's own fit from `S1-S7test results.xlsx`.
  **Hysteresis compensation** (on by default): the loading curve is used while a sensor's force rises, the unloading curve
  (scaled through the turning point) while it falls; each step starts again on the loading curve. Over 0–300 N this cuts the
  UTM error from 8–15 N to 2–8 N per sensor. Calibration → "Fit from a UTM workbook" fits a new .xlsx/.csv of the same layout
  in the browser (e.g. the right insole's sensors) into Left, Right or both; Save to use it.
  S3 has no unloading leg and S6 (ch6 = site S10, disconnected pins) no loading leg above 57 N: those channels use one curve.
- ≥ 3250 mV = ADC saturated → shown as `SAT` / `≥ value`.
- vGRF = Σ Fᵢ, the plain sum of the 7 calibrated sensor forces (no area or body-weight scaling), so it is the load on the
  sensors only and reads below the true foot load.
- Zero: channels at 250–350 mV set their base (VREF). A channel above that is preloaded: base stays ≈300 mV and the preload is
  subtracted in newtons. All channels high by the same amount → VREF shifted (warning shown).
- Live COP (x, y in mm, PDF insole coordinates) = Σ(rᵢ Fᵢ)/Σ Fᵢ, with the current step's COP path drawn on the insole.
- Record → Stop & save downloads a CSV of every sample (mV, N, Fz, COP) and opens **Results**: gait events, walking
  speed / cadence / step & stride, vGRF peaks, impulse, loading rate, plantar pressure (peak kPa, PTI, contact area, COP path)
  as mean ± SD per foot, plus a per-step table and "Export steps CSV". Results also appear when a foot disconnects mid-recording.
- Walking speed: fill **Distance** (m walked during the recording) — or **Height** for an estimate. **Mass** adds %BW.

## 4. Logging over Bluetooth to a laptop or phone
Connect each foot with **Bluetooth**, press **● Record**, walk, press **■ Stop & save**. No cable or SD card is needed.
- Rows are saved in the browser's storage about once a second (header shows time · rows · saved). If the tab closes, the phone
  sleeps or the page crashes, the log is kept up to that second and appears under **Logs** as "recovered".
- **Logs** lists every recording in this browser: Download (laptop / Android → Downloads), Share (phone → Files, Drive, e-mail), Delete.
- A dropped BLE link reconnects by itself for ≈40 s without losing zero, calibration or the recording; the status shows
  "% lost" from the packet counter, and the CSV has `dev_ms` (board time) plus BLE loss in its header line.
- The screen is kept awake while a foot streams over BLE or a recording runs (phones stop Bluetooth in the background):
  keep the app in front.
