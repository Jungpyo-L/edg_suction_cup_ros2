#!/usr/bin/env python3
"""Keyboard jog of the UR TCP in the base frame with the tool orientation held fixed (moveL steps through ur_rtde).

Used to set heights and positions by hand without the pendant: a/d -x/+x, s/w -y/+y, f/r down/up,
6/5/1/2/3/4 select a 50/10/2/0.5/0.1/0.02 mm step, Enter accepts, q aborts (KeyboardInterrupt).
"""
import select
import sys
import termios
import tty

JOG_KEYS = {"d": (1, 0, 0), "a": (-1, 0, 0), "w": (0, 1, 0), "s": (0, -1, 0), "r": (0, 0, 1), "f": (0, 0, -1)}
STEPS_MM = {"6": 50.0, "5": 10.0, "1": 2.0, "2": 0.5, "3": 0.1, "4": 0.02}
JOG_SPEED, JOG_ACC = 0.01, 0.1   # m/s, m/s^2


def _read_key(timeout=0.1):
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if ready else None


def jog(rtde_help, label="", z_floor=None, step_mm=0.5):
    """Jog until Enter; returns the final TCP pose [x, y, z, rx, ry, rz] (m, rad).

    z_floor (m): refuse steps that would go below it (the only guard against pressing into the object).
    """
    print("\n[%s] jog:  a/d -x/+x   s/w -y/+y   f/r down/up (base frame)" % label)
    print("        step 6:50 5:10 1:2 2:0.5 3:0.1 4:0.02 mm   Enter: accept   q: abort")
    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            key = _read_key()
            if key is None:
                continue
            if key in ("\n", "\r"):
                return list(rtde_help.rtde_r.getActualTCPPose())
            if key == "q":
                raise KeyboardInterrupt
            if key in STEPS_MM:
                step_mm = STEPS_MM[key]
                print("  step %.2f mm" % step_mm)
                continue
            if key not in JOG_KEYS:
                continue
            target = list(rtde_help.rtde_r.getActualTCPPose())
            for i, d in enumerate(JOG_KEYS[key]):
                target[i] += d * step_mm / 1000.0
            if z_floor is not None and target[2] < z_floor:
                print("  refused: below the allowed height (%.2f mm)" % (z_floor * 1000))
                continue
            rtde_help.rtde_c.moveL(target, JOG_SPEED, JOG_ACC)
            p = rtde_help.rtde_r.getActualTCPPose()
            print("  x %.2f  y %.2f  z %.2f mm" % (p[0] * 1000, p[1] * 1000, p[2] * 1000))
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
