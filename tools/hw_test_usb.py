#!/usr/bin/env python3
"""Hardware check of firmware 1.4 over USB (acceptance test F, no browser needed).

Close the Arduino Serial Monitor and the web app first: only one program can hold the port.

    pip install pyserial
    python tools/hw_test_usb.py --list                 # find the port (native USB = VID 303A)
    python tools/hw_test_usb.py COM6                   # 35 s test, board unloaded for the first 5 s, then press / walk
    python tools/hw_test_usb.py COM6 --seconds 40 --baud 921600
    python tools/hw_test_usb.py --simulate             # self-test of this script without a board

What it checks (PASS / WARN / FAIL):
  firmware 1.4 + IMU found | stream rate = 200 Hz | no missing / duplicate samples (sample counter) | sample interval and jitter from the
  MCU micros() timestamp | MCU timing counters (#TIMING: late / over / resync, adc_us_max) | no corrupt lines | unloaded mV near 300 |
  |a| about 1 g | every channel responds when loaded | saturation | micros() timestamp present.
The raw lines are saved to hw_test_<time>.log next to where you run it.
"""
import argparse
import re
import statistics
import sys
import time
from datetime import datetime

SITES = ["S1", "S4", "S6", "S11", "S13", "S10", "S14"]
SAT_MV = 3250


def list_ports():
    from serial.tools import list_ports as lp
    for p in lp.comports():
        tag = "  <- native USB of the ESP32-C6" if p.vid == 0x303A else ("  (CH343 UART bridge)" if p.vid == 0x1A86 else "")
        print(f"{p.device}  vid={p.vid and hex(p.vid)} pid={p.pid and hex(p.pid)}  {p.description}{tag}")


def simulate(seconds, fault=False):
    """Yield (time, line) like a board at 200 Hz (for the self-test); fault=True drops samples and reports late frames."""
    import random
    t = 0.0
    seq = 0
    us0 = 4294967296 - 3_000_000
    yield 0.0, "#INFO,FSR_Insole_App v1.4,ch=7,hz=200,ble=Insole-L,ble_idle,imu=BMI270,|a|=1.00g,gyro_bias=0.030/-1.175/0.180 dps,oversample=8,ts=micros_before_adc_burst,mac=SIM,build=sim"
    while t < seconds:
        loaded = t > 5
        mv = [301 + (int(250 * max(0.0, ((t * 1.3 + i * .4) % 1.0) - .4)) if loaded else 0) + random.randint(-1, 1) for i in range(7)]
        us = (us0 + seq * 5000 + random.randint(-60, 60)) % 4294967296
        if not (fault and seq % 700 == 699):
            yield t, f"D,{us // 1000 % 100000000},{','.join(map(str, mv))},5,-1000,8,3,-2,1,30,{seq},{us}"
        if seq % 400 == 399:
            late = seq // 400 * 3 if fault else 0
            yield t, f"#TIMING,hz=200,oversample=8,frames={seq + 1},adc_us_avg=3900,adc_us_max={5600 if fault else 2950},frame_us_max=3300,late={late},late_us_max=0,over=0,resync=0,ble_mtu=0"
        seq += 1
        t += 0.005


def read_serial(port, baud, seconds):
    import serial
    ser = serial.Serial(port, baud, timeout=0.2)
    ser.reset_input_buffer()
    ser.write(b"?\n")
    t0 = time.time()
    buf = b""
    while time.time() - t0 < seconds:
        chunk = ser.read(4096)
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            yield time.time() - t0, line.decode("ascii", "replace").strip()
    ser.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?")
    ap.add_argument("--seconds", type=float, default=35)
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--simulate", action="store_true", help="self-test without a board")
    ap.add_argument("--fault", action="store_true", help="with --simulate: inject dropped samples and late frames (must report FAIL)")
    a = ap.parse_args()
    if a.list:
        list_ports()
        return 0
    if not a.port and not a.simulate:
        ap.print_help()
        return 2
    src = simulate(a.seconds, a.fault) if a.simulate else read_serial(a.port, a.baud, a.seconds)
    if not a.simulate:
        print(f"Recording {a.seconds:.0f} s from {a.port}: keep the insole UNLOADED for the first 5 s, then press every sensor / walk.\n")
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log = open(f"hw_test_{stamp}.log", "w", encoding="utf-8")

    info = None
    timing = []
    rows = []                 # (t_host, seq, us_unwrapped, [7 mV], imu or None)
    bad = 0
    other = []
    last_us, us_u = None, 0
    for th, line in src:
        log.write(line + "\n")
        if line.startswith("D,"):
            p = line.split(",")
            if len(p) not in (11, 18):
                bad += 1
                continue
            try:
                mv = [float(x) for x in p[2:9]]
                imu = [float(x) for x in p[9:16]] if len(p) == 18 else None
                k = 16 if len(p) == 18 else 9
                seq, us = int(p[k]), int(p[k + 1])
            except ValueError:
                bad += 1
                continue
            if last_us is None:
                us_u = us
            else:
                d = (us - last_us) & 0xFFFFFFFF
                us_u += d if d < 2 ** 31 else d - 2 ** 32
            last_us = us
            rows.append((th, seq, us_u, mv, imu))
        elif line.startswith("#INFO"):
            info = line
        elif line.startswith("#TIMING"):
            timing.append({k: float(v) for k, v in (x.split("=") for x in line[8:].split(",") if "=" in x)})
        elif line.startswith("#WARN") or (line and not line.startswith("#")):
            other.append(line)
    log.close()

    res = []

    def check(name, status, detail=""):
        res.append((status, name, detail))

    # firmware / IMU
    if info is None:
        check("firmware answers '?' (#INFO)", "FAIL", "no #INFO line: wrong port, Serial Monitor open, or old firmware")
    else:
        v = re.search(r"v([\d.]+)", info)
        check("firmware version 1.4", "PASS" if v and v.group(1) == "1.4" else "FAIL", info[:150])
        check("IMU found (BMI270)", "PASS" if "imu=BMI270" in info else "WARN", "" if "imu=BMI270" in info else "no IMU: the IMU columns will be empty")
    n = len(rows)
    if n < 100:
        check("D lines with sample counter and micros() timestamp", "FAIL", f"only {n} valid lines (firmware 1.4 sends 18 fields)")
        return report(res, rows, a.seconds)
    check("D lines with sample counter and micros() timestamp", "PASS", f"{n} lines, {bad} corrupt")
    check("no corrupt lines", "PASS" if bad == 0 else "FAIL", f"{bad} corrupt lines (USB buffer or baud problem)")
    dur = (rows[-1][2] - rows[0][2]) / 1e6
    check("stream rate = 200 Hz (counter span / MCU time)", "PASS" if abs((rows[-1][1] - rows[0][1]) / dur - 200) < 2 else "FAIL",
          f"{(rows[-1][1] - rows[0][1]) / dur:.2f} Hz over {dur:.1f} s of MCU time")
    # sequence
    missing = dup = ooo = 0
    last = None
    for r in rows:
        if last is not None:
            d = r[1] - last
            if d > 1:
                missing += d - 1
                last = r[1]
            elif d == 1:
                last = r[1]
            elif d == 0:
                dup += 1
            else:
                ooo += 1
        else:
            last = r[1]
    check("no missing / duplicate / out-of-order samples", "PASS" if missing == dup == ooo == 0 else "FAIL",
          f"missing {missing} ({100 * missing / (n + missing):.3f} %), duplicate {dup}, out-of-order {ooo}")
    # interval
    iv = [(rows[i + 1][2] - rows[i][2]) / 1000 for i in range(n - 1) if rows[i + 1][1] - rows[i][1] == 1]
    med, sd, mx = statistics.median(iv), statistics.pstdev(iv), max(iv)
    check("sample interval from MCU micros() = 5.000 ms", "PASS" if abs(med - 5) < 0.05 else "FAIL", f"median {med:.3f} ms, jitter sd {sd:.3f} ms, max {mx:.2f} ms")
    check("jitter small (sd < 0.3 ms, no interval > 10 ms)", "PASS" if sd < 0.3 and mx < 10 else "WARN", f"sd {sd:.3f} ms, max {mx:.2f} ms, {sum(1 for x in iv if x > 7.5)} intervals > 7.5 ms")
    # MCU timing counters
    if timing:
        t0_, t1_ = timing[0], timing[-1]
        d = {k: t1_.get(k, 0) - t0_.get(k, 0) for k in ("late", "over", "resync")}
        check("MCU frames on schedule (#TIMING late / over / resync did not increase)", "PASS" if not any(d.values()) else "FAIL",
              f"during the test: late +{d['late']:.0f}, over +{d['over']:.0f}, resync +{d['resync']:.0f}; since boot: late {t1_['late']:.0f}, over {t1_['over']:.0f}")
        am = t1_.get("adc_us_max", 0)
        check("ADC burst fits in the 5 ms frame (adc_us_max)", "PASS" if am < 4000 else ("WARN" if am < 5000 else "FAIL"),
              f"adc_us_avg {t1_.get('adc_us_avg', 0):.0f} us, adc_us_max {am:.0f} us, frame_us_max {t1_.get('frame_us_max', 0):.0f} us (budget 5000). "
              + ("If WARN/FAIL: set OVERSAMPLE to 4 in the sketch." if am >= 4000 else ""))
    else:
        check("#TIMING lines", "WARN", "none received (test shorter than 2 s?)")
    # signals
    rest = [r for r in rows if r[0] < rows[0][0] + 4.5 and r[0] > rows[0][0] + 0.5] or rows[:200]
    rest_mv = [statistics.mean(r[3][i] for r in rest) for i in range(7)]
    off = [SITES[i] for i in range(7) if not 250 < rest_mv[i] < 350]
    check("unloaded mV about 300 on all 7 channels", "PASS" if not off else "FAIL",
          "rest mean: " + ", ".join(f"{SITES[i]} {rest_mv[i]:.0f}" for i in range(7)) + (f"; OUT OF RANGE: {off} (loaded at start, or VREF / amplifier problem)" if off else ""))
    imus = [r[4] for r in rest if r[4]]
    if imus:
        g = statistics.mean((r[0] ** 2 + r[1] ** 2 + r[2] ** 2) ** .5 for r in imus) / 1000
        check("|a| about 1 g at rest", "PASS" if 0.95 < g < 1.05 else "WARN", f"{g:.3f} g")
    span = [max(r[3][i] for r in rows) - min(r[3][i] for r in rows) for i in range(7)]
    dead = [SITES[i] for i in range(7) if span[i] < 30]
    check("every channel responded (mV range > 30 mV during the test)", "PASS" if not dead else "WARN",
          "range: " + ", ".join(f"{SITES[i]} {span[i]:.0f}" for i in range(7)) + (f"; NO RESPONSE: {dead} (not pressed, or loose connector)" if dead else ""))
    sat = [SITES[i] for i in range(7) if any(r[3][i] >= SAT_MV for r in rows)]
    check("no saturation (mV >= 3250)", "PASS" if not sat else "WARN", f"saturated: {sat}" if sat else "none")
    if other:
        check("unexpected text lines", "WARN", "; ".join(other[:3]))
    return report(res, rows, a.seconds)


def report(res, rows, seconds):
    w = max(len(r[1]) for r in res)
    print("\nHardware test (firmware 1.4, USB)\n")
    for status, name, detail in res:
        print(f"[{status:<4}] {name:<{w}}  {detail}")
    f = sum(1 for r in res if r[0] == "FAIL")
    wn = sum(1 for r in res if r[0] == "WARN")
    print(f"\n{sum(1 for r in res if r[0] == 'PASS')} PASS, {wn} WARN, {f} FAIL")
    print("Raw lines saved to the hw_test_*.log file. Next: repeat once with the web app (Record, 30-40 s) and run tools/validate_export.py on its ZIP.")
    return 1 if f else 0


if __name__ == "__main__":
    sys.exit(main())
