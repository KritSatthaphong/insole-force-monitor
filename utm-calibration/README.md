# UTM calibration of the FlexiForce A401 channels

Test code for calibrating each insole sensor channel on the Shimadzu AG-X plus (TRAPEZIUM X, Single method, 4 Areas:
load → hold → load / unload → hold). Board: Waveshare ESP32-C6-DEV-KIT-N8, Arduino core 3.3.x, RF 39 kΩ.

| Folder | What it is | Use when |
|---|---|---|
| [`FSR_UTM_Hold/`](FSR_UTM_Hold/) | **Simple version (in use).** Type `h` when the UTM hold starts; the board measures until 3 s before the hold ends and asks for the force on the UTM display. One CAL row per hold, no Shimadzu export, no time syncing. Guide: [`HOLD_GUIDE.md`](FSR_UTM_Hold/HOLD_GUIDE.md) | Fast sessions |
| [`FSR_UTM_Cal/`](FSR_UTM_Cal/) | **Full version v3.0.** Continuous 200 Hz recording of each machine run, run plan, event marks, on-board hold detection and QC, pre/post baselines. Paired afterwards with the Shimadzu raw export by [`tools/utm_pair.py`](tools/utm_pair.py) (time alignment, one row per hold, QC flags, plot). Guide: [`UTM_SESSION_GUIDE.md`](FSR_UTM_Cal/UTM_SESSION_GUIDE.md) | Reference force from the load cell export, full 30 s drift traces |
| `FSR_UTM_*/utm_pc_logger.py` | PC logger for both sketches (`pip install pyserial`). Saves the files; reconnects after a cable drop | Instead of the Serial Monitor |
| [`dev/`](dev/) | PC simulation of the sketch (stub Arduino / FreeRTOS, simulated FSR + UTM) and the hold-detector unit test | Development only |
| [`results/`](results/) | Measured data: S1 cycle 1 at 39 kΩ (loading 0–990 N, 30 s holds) and the older 47 kΩ runs | |

The CAL files (both versions) and the `holds_table.csv` of `utm_pair.py` open directly in insole-app
(Calibration → Calibration check) and in [`../tools/check_cal.py`](../tools/check_cal.py).

Status: both sketches compile for the ESP32-C6 (core 3.3.12). FSR_UTM_Hold has run on the hardware (S1 cycle 1).
FSR_UTM_Cal v3.0 and utm_pair.py were tested on the PC simulation only, not yet with the board and a real TRAPEZIUM export.
