#!/usr/bin/env python3
"""utm_pc_logger.py v3 - terminal + PC file logger for FSR_UTM_Cal v3.x (also reads v2.x output).

Close the Arduino Serial Monitor first (one program per port). Type the sketch commands here.

    pip install pyserial
    python utm_pc_logger.py                       (finds the ESP32 port by itself)
    python utm_pc_logger.py COM5 --out D:\\capstone\\utm_data
    python utm_pc_logger.py --pc toon --baud 115200        (Toon's laptop = COM5, Krit's laptop = COM7: --pc krit)

Files in the output folder (default ./utm_data):
    UP01_S1_R1_20261112_101500.csv   one machine run: header, R raw 200 Hz rows, EV events (with PC time),
                                     BASE baselines, HOLD on-board hold summaries, pc_sync / pc_close lines
    UTM_S1_<time>.csv + _CAL.csv     point mode (start / m / save), as in v2.x
    RAW_<time>.csv                   free recording (r / save)
    runs_log.csv                     one line per file: run, channel, repeat, rows, gaps, rate, status
    holds_log.csv                    every on-board HOLD row of every run (quick QC table)

The logger keeps the file open if the USB cable drops and reconnects by itself (missing frames are
counted). If the board restarts, the open file is closed and marked. Ctrl+C quits (open file kept).
"""
import argparse
import csv
import datetime
import os
import sys
import threading
import time

import serial
from serial.tools import list_ports

VERSION = "utm_pc_logger v3.0"
FILE_TYPES = ("R,", "CAL,", "EV,", "HOLD,", "BASE,", "#")
LAPTOPS = {"toon": "COM5", "krit": "COM7"}         # --pc toon / --pc krit


def now_iso():
    return datetime.datetime.now().isoformat(timespec="milliseconds")


def find_port():
    ports = list(list_ports.comports())
    esp = [p for p in ports if p.vid == 0x303A]          # Espressif native USB
    cands = esp or ports
    if len(cands) == 1:
        return cands[0].device
    print("Serial ports found:")
    for p in ports:
        print(f"  {p.device}  {p.description}")
    sys.exit("Give the port, e.g.  python utm_pc_logger.py COM5")


def append_csv(path, head, row):
    new = not os.path.exists(path)
    with open(path, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(head)
        w.writerow(row)


class Logger:
    def __init__(self, outdir, show_raw):
        self.outdir = outdir
        self.show_raw = show_raw
        self.f = self.fcal = None
        self.lock = threading.Lock()
        self.reconnects = 0
        self.warned = False
        self.got = 0
        self.hinted = False
        self.t_start = time.time()

    # ---------------- files ----------------
    def open(self, parts):
        self.close("new file started before the old one was closed")
        os.makedirs(self.outdir, exist_ok=True)
        kind, ch = parts[1], int(parts[2])
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.kind, self.ch = kind, ch
        self.run_id = parts[3] if kind == "RUN" and len(parts) > 3 else ""
        self.rep = parts[4] if kind == "RUN" and len(parts) > 4 else ""
        if kind == "RUN":
            base = f"{self.run_id}_S{ch}_R{self.rep}_{stamp}"
        elif ch > 0:
            base = f"{kind}_S{ch}_{stamp}"
        else:
            base = f"{kind}_{stamp}"
        self.path = os.path.join(self.outdir, base + ".csv")
        self.f = open(self.path, "w", newline="")
        self.calpath = os.path.join(self.outdir, base + "_CAL.csv") if kind == "UTM" else None
        self.fcal = open(self.calpath, "w", newline="") if self.calpath else None
        self.rows = self.cal = self.gaps = self.missing = self.holds = 0
        self.last_idx = self.last_t = self.first_t = None
        self.first_idx = None
        self.back_t = 0
        self.opened = now_iso()
        self.last_sync = time.time()
        self.f.write(f"# pc_logger,{VERSION}\n# pc_open,{self.opened}\n")
        print(f"[PC] recording to {os.path.abspath(self.path)}", flush=True)

    def write(self, line):
        f = self.f
        if line.startswith("R,"):
            p = line.split(",", 3)
            try:
                t, idx = int(p[1]), int(p[2])
            except (ValueError, IndexError):
                self.gaps += 1
                f.write(f"# pc_bad_row,{line[:60]}\n")
                return
            if self.last_idx is not None:
                if idx != self.last_idx + 1:
                    self.gaps += 1
                    if idx > self.last_idx:
                        self.missing += idx - self.last_idx - 1
                if t <= self.last_t:
                    self.back_t += 1
            else:
                self.first_t, self.first_idx = t, idx
            self.last_idx, self.last_t = idx, t
            self.rows += 1
            f.write(line + "\n")
            if time.time() - self.last_sync >= 10:          # PC clock <-> board clock
                self.last_sync = time.time()
                f.write(f"# pc_sync,{self.last_sync:.3f},{t},{idx}\n")
            if self.rows % 2000 == 0:
                f.flush()
            return
        if line.startswith("EV,"):
            f.write(f"{line},{now_iso()}\n")
            f.flush()
            return
        f.write(line + "\n")
        if line.startswith("HOLD,"):
            self.holds += 1
            f.flush()
            if self.kind == "RUN":
                p = line.split(",")
                append_csv(os.path.join(self.outdir, "holds_log.csv"),
                           ["file", "run_id", "ch", "repeat_id", "k", "label", "target_N", "t0_us", "t1_us", "dur_s",
                            "early_mV", "late_mV", "sd_mV", "min_mV", "max_mV", "drift_mV", "n", "frame0", "frame1",
                            "win_s", "est_N_prov", "flags"],
                           [os.path.basename(self.path), self.run_id, self.ch, self.rep] + p[1:])
        if self.fcal:
            if line.startswith("# CAL,"):
                self.fcal.write(line[2:] + "\n")
            elif line.startswith("CAL,"):
                self.fcal.write(line + "\n")
                self.fcal.flush()
                self.cal += 1

    def close(self, why="", status=None):
        if not self.f:
            return
        dur = (self.last_t - self.first_t) / 1e6 if self.rows > 1 else 0
        rate = (self.last_idx - self.first_idx) / dur if dur > 0 else 0
        st = status or ("PASS" if self.gaps == 0 and self.back_t == 0 else "CHECK")
        self.f.write(f"# pc_close,{now_iso()},rows,{self.rows},gaps,{self.gaps},missing_frames,{self.missing},"
                     f"t_backwards,{self.back_t},rate_hz,{rate:.3f},reconnects,{self.reconnects},status,{st}"
                     f"{',' + why if why else ''}\n")
        self.f.close()
        if self.fcal:
            self.fcal.close()
        append_csv(os.path.join(self.outdir, "runs_log.csv"),
                   ["file", "kind", "run_id", "ch", "repeat_id", "pc_open", "pc_close", "rows", "dur_s", "rate_hz",
                    "gaps", "missing_frames", "reconnects", "holds", "cal_points", "status", "note"],
                   [os.path.basename(self.path), self.kind, self.run_id, self.ch, self.rep, self.opened, now_iso(),
                    self.rows, f"{dur:.2f}", f"{rate:.3f}", self.gaps, self.missing, self.reconnects, self.holds,
                    self.cal, st, why])
        note = f" ({why})" if why else ""
        print(f"[PC] saved {os.path.abspath(self.path)}: {self.rows} raw rows ({dur:.1f} s, {rate:.2f} Hz), "
              f"{self.holds} holds, {self.cal} points, frame gaps {self.gaps} (missing {self.missing}) {st}{note}",
              flush=True)
        if self.fcal:
            print(f"[PC] force points only: {os.path.abspath(self.calpath)}", flush=True)
        self.f = self.fcal = None
        self.reconnects = 0

    # ---------------- lines from the board ----------------
    def handle(self, line):
        with self.lock:
            if line.startswith("FILE_OPEN,"):
                self.open(line.split(","))
                return
            if line.startswith("FILE_CLOSE"):
                print(line, flush=True)
                if self.f:
                    self.f.write(f"# {line}\n")
                bad = line.endswith("CHECK")
                self.close(status="CHECK" if bad else None)
                return
            if line.startswith("FSR_UTM_Cal v") and self.f:
                print("[PC] WARNING: the board restarted while a file was open", flush=True)
                self.close("board restarted", status="RESTART")
            is_raw = line.startswith("R,")
            if self.f and line.startswith(FILE_TYPES):
                self.write(line)
            elif is_raw and not self.warned:
                print("[PC] data arriving but no file open here (logger started mid-recording): "
                      "type end / save, then start again", flush=True)
                self.warned = True
            if is_raw and not self.show_raw:
                return
            if line.startswith("#") and not line.startswith(("# note", "# end")):
                return                      # header lines: in the file, not on screen
            print(line, flush=True)


class Port:
    """Serial port that reopens itself after the cable drops."""

    def __init__(self, name, baud, dtr=True):
        self.name, self.baud, self.dtr = name, baud, dtr
        self.ser = None
        self.wlock = threading.Lock()

    def open(self):
        s = serial.Serial()
        s.port, s.baudrate, s.timeout = self.name, self.baud, 0.1
        s.dtr = False                       # do not reset the board when the port opens
        s.rts = False
        s.open()
        if self.dtr:                        # some Windows USB drivers send nothing until DTR is set;
            try:                            # DTR=1 with RTS=0 does not reset the ESP32
                s.dtr = True
            except (OSError, serial.SerialException):
                pass
        self.ser = s

    def write(self, data):
        with self.wlock:
            if not self.ser:
                print("[PC] not connected, command not sent", flush=True)
                return
            try:
                self.ser.write(data)
            except serial.SerialException as e:
                print(f"[PC] write failed: {e}", flush=True)

    def close(self):
        if self.ser:
            try:
                self.ser.close()
            except serial.SerialException:
                pass
        self.ser = None


def reader(port, lg, stop):
    buf = b""
    while not stop.is_set():
        try:
            data = port.ser.read(4096) if port.ser else b""
        except (serial.SerialException, OSError, TypeError) as e:
            print(f"[PC] serial error: {e} - reconnecting (file stays open)...", flush=True)
            with port.wlock:
                port.close()
            buf = b""
            with lg.lock:
                if lg.f:
                    lg.f.write(f"# pc_disconnect,{now_iso()}\n")
            while not stop.is_set():
                time.sleep(1)
                try:
                    with port.wlock:
                        port.open()
                    break
                except (serial.SerialException, OSError):
                    continue
            if stop.is_set():
                break
            with lg.lock:
                lg.reconnects += 1
                if lg.f:
                    lg.f.write(f"# pc_reconnect,{now_iso()}\n")
            print("[PC] reconnected", flush=True)
            continue
        if not data:
            if not lg.got and not lg.hinted and time.time() - lg.t_start > 4:
                lg.hinted = True
                print("[PC] nothing received from the board yet: press RESET on the board, check the COM port "
                      "(the one the Arduino IDE used), close the Serial Monitor. On the UART port (CH343) the baud must match the "
                      "sketch: --baud 115200 for FSR_UTM_Hold", flush=True)
            continue
        lg.got += len(data)
        buf += data
        *lines, buf = buf.split(b"\n")
        for raw in lines:
            lg.handle(raw.decode("ascii", "replace").rstrip("\r"))


def main():
    ap = argparse.ArgumentParser(description=f"PC logger for FSR_UTM_Cal v3.x ({VERSION})")
    ap.add_argument("port", nargs="?", help="serial port, e.g. COM5 or /dev/ttyACM0")
    ap.add_argument("--pc", choices=sorted(LAPTOPS), help="laptop: toon = COM5, krit = COM7 (instead of the port)")
    ap.add_argument("--out", default="utm_data", help="output folder (default ./utm_data)")
    ap.add_argument("--baud", type=int, default=921600, help="only matters for a UART bridge")
    ap.add_argument("--show-raw", action="store_true", help="also print the 200 Hz R rows")
    ap.add_argument("--no-dtr", action="store_true", help="keep DTR low (v2 behaviour)")
    a = ap.parse_args()

    name = a.port or (LAPTOPS[a.pc] if a.pc else None) or find_port()
    port = Port(name, a.baud, dtr=not a.no_dtr)
    try:
        port.open()
    except serial.SerialException as e:
        sys.exit(f"Cannot open {port.name}: {e}\nClose the Arduino Serial Monitor / other programs using the port.")

    lg = Logger(a.out, a.show_raw)
    stop = threading.Event()
    threading.Thread(target=reader, args=(port, lg, stop), daemon=True).start()
    print(f"[PC] {VERSION} connected to {port.name}. Files go to {os.path.abspath(a.out)}", flush=True)
    print("[PC] type commands and press Enter (? = help, plan = run list). Ctrl+C to quit.", flush=True)
    port.write(b"?\n")

    try:
        while not stop.is_set():
            cmd = input()
            port.write((cmd + "\n").encode("ascii", "replace"))
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        stop.set()
        time.sleep(0.3)
        with lg.lock:
            lg.close("logger stopped before end / save", status="CHECK")
        port.close()
        print("[PC] bye")


if __name__ == "__main__":
    main()
