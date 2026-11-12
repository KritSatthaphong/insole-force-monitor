# dev: PC tests of FSR_UTM_Cal.ino (not needed in the lab)

- `tracker_test.cpp`: unit test of the hold detector (the block between `TRACKER BEGIN` / `TRACKER END` in the sketch).
  `sed -n '/==== TRACKER BEGIN/,/==== TRACKER END/p' ../FSR_UTM_Cal/FSR_UTM_Cal.ino > tracker.inc && g++ -O1 -std=c++17 -o t tracker_test.cpp && ./t`
- `sim.cpp` + `inc/`: the real sketch compiled for Linux against stub Arduino / FreeRTOS headers, a pty as the USB port,
  a simulated FSR (power law, unloading hysteresis, log creep, 2 mV noise, ADC clip) and a simulated Shimadzu run
  (starts 0.35 s after a `k` line, writes `utm_export_<ID>_<n>.csv` at 20 Hz).
  `cp ../FSR_UTM_Cal/FSR_UTM_Cal.ino sketch.inc && g++ -O2 -std=c++17 -Iinc -o sim sim.cpp -lpthread`
  Env: SIM_SPEED (clock speed-up, default 5), SIM_CH (loaded channel 1..7), SIM_DELAY (s). The pty name is written to `pty_name`.
- `driver_example.py`: starts sim + utm_pc_logger.py and types a session (run it from a subfolder of dev/).
