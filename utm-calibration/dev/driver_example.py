import subprocess, time, sys, os
SPEED = 5.0
sim = subprocess.Popen(["../sim"], stderr=open("sim_stderr.txt", "w"), env=dict(os.environ, SIM_SPEED=str(SPEED)))
time.sleep(0.5)
pty = open("pty_name").read().strip()
lg = subprocess.Popen([sys.executable, "/home/user/utm_v3/FSR_UTM_Cal/utm_pc_logger.py", pty, "--out", "data"],
                      stdin=subprocess.PIPE, stdout=open("logger_out.txt", "w"), stderr=subprocess.STDOUT, text=True)
def send(c, wait_sim_s=1.0):
    lg.stdin.write(c + "\n"); lg.stdin.flush(); time.sleep(wait_sim_s / SPEED)
time.sleep(1.5)
for c in ["meta cell 20kN SN-TEST", "meta cell_N 20000", "meta fixture rigid_steel_acrylic_puck_25.4", "meta foot L", "meta board B1",
          "s1", "set", "plan", "n sim test", "Set P 1000"]:
    send(c)
send("run", 5)                 # UP01, pre-baseline 3 s
send("k UTM start", 92)        # profile 86 s
send("end", 6)
send("run DN05", 1)            # within rest -> prompt
send("y", 5)
send("k", 225)
send("end", 6)
send("plan", 1)
send("run pilot1", 5)
send("x", 3)
send("run UP01", 1); send("y", 1); send("y", 5)   # rest prompt + already done prompt
send("k", 30)                  # end the run early (mid hold) to test a short run
send("end", 6)
lg.stdin.close(); time.sleep(0.5); lg.terminate(); time.sleep(0.5); sim.terminate()
