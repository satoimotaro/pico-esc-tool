#!/usr/bin/env python3
# Low-speed characterization with a KEEP-ALIVE hold: repeat throttle every 200ms to feed the
# firmware's 500ms deadman, so the motor gets SUSTAINED power (needed to complete startup + run).
# Reuses esctool.py EscHost. ALWAYS disarms on exit.
import sys, time
sys.path.insert(0, "/home/satoimo/UWR_ESC_ws/pico-esc-tool/host")
from esctool import EscHost, find_pico

IDX = 1
import os
STEPS = [int(x) for x in os.environ.get("STEPS","300,180,120,90,70,55,45").split(",")]
HOLD  = float(os.environ.get("HOLD","3.0"))   # s to hold each level (keep-alive fed)
KA    = 0.2                          # keep-alive throttle resend period (< 500ms deadman)

def hold_and_measure(dev, thr):
    rpms = []; last = (0,0.0,0,0,0)
    t0 = time.time(); nextTele = t0 + 0.8
    while time.time() - t0 < HOLD:
        dev.cmd(f"throttle {IDX} {thr}", timeout=2)   # feed deadman
        if time.time() >= nextTele:
            r = dev.cmd(f"tele {IDX}", timeout=2)
            for ln in r:
                if ln.startswith("tele|"):
                    p = ln.split("|")
                    try:
                        s=(int(p[1]),float(p[2]),int(p[3]),int(p[4]),int(p[5])); rpms.append(s[0]); last=s
                    except Exception: pass
            nextTele = time.time() + 0.5
        time.sleep(KA)
    return rpms, last

def main():
    port = find_pico(None); print(f"# port {port}")
    dev = EscHost(port); armed = False
    try:
        dev.cmd("run", timeout=5); dev.cmd("disconnect", timeout=5); time.sleep(0.4)
        print("# arm bidir…", dev.cmd(f"arm {IDX} bidir", timeout=6)); armed = True
        time.sleep(4.0)
        print("# HELD ramp (keep-alive every 200ms); rpm = MECHANICAL (eRPM/7)")
        print("thr\trpm_min\trpm_max\trpm_last\tsamples\ttemp\tstress")
        for thr in STEPS:
            rpms, last = hold_and_measure(dev, thr)
            nz = [r for r in rpms if r>0]
            print(f"{thr}\t{min(rpms) if rpms else -1}\t{max(rpms) if rpms else -1}\t{last[0]}\t{len(rpms)}(nz={len(nz)})\t{last[3]}\t{last[4]}")
    finally:
        if armed:
            for _ in range(3): dev.cmd(f"throttle {IDX} 0", timeout=2)
            dev.cmd(f"disarm {IDX}", timeout=2); dev.cmd("disarm", timeout=2)
            print("# DISARMED")
        dev.close()

if __name__ == "__main__":
    main()
