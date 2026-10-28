#!/usr/bin/env python3
"""Keyboard jog of the UR TCP in the base frame with the tool orientation held fixed (moveL steps through ur_rtde).

Used to set heights and positions by hand without the pendant: a/d -x/+x, s/w -y/+y, f/r down/up,
1-6 select the step: 1 = 0.02, 2 = 0.1, 3 = 0.5, 4 = 2, 5 = 10, 6 = 50 mm, t toggles freedrive (push the arm by hand; on leaving it the tool is
turned back to the orientation it had when the jog started), Enter accepts, q aborts (KeyboardInterrupt).
The teach pendant can also be used in between (the control script and TCP are restored automatically), but freedrive
keeps the connection and is quicker.
"""
import select
import sys
import termios
import tty

JOG_KEYS = {"d": (1, 0, 0), "a": (-1, 0, 0), "w": (0, 1, 0), "s": (0, -1, 0), "r": (0, 0, 1), "f": (0, 0, -1)}
STEPS_MM = {"1": 0.02, "2": 0.1, "3": 0.5, "4": 2.0, "5": 10.0, "6": 50.0}   # 1 = finest ... 6 = coarsest
JOG_SPEED, JOG_ACC = 0.01, 0.1   # m/s, m/s^2


def _read_key(timeout=0.1):
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if ready else None


def ensure_control_script(rtde_help):
    """Moving the arm from the teach pendant stops the ur_rtde control script on the controller, after which moveL
    calls do nothing. Re-upload it if it is not running, and set the TCP again: when the script stops the controller
    falls back to the installation TCP, which would change every pose read afterwards."""
    c = rtde_help.rtde_c
    if not c.isProgramRunning():
        print("  control script not running (teach pendant used?) - re-uploading")
        c.reuploadScript()
        if getattr(rtde_help, "tcp_offset", None) is not None:
            c.setTcp(rtde_help.tcp_offset)
            print("  TCP offset set again: %s" % rtde_help.tcp_offset)
        if not c.isProgramRunning():
            raise RuntimeError("Could not restart the RTDE control script; check the pendant (remote control mode, "
                               "no program running) and restart this tool")


def jog(rtde_help, label="", z_floor=None, step_mm=0.5):
    """Jog until Enter; returns the final TCP pose [x, y, z, rx, ry, rz] (m, rad).

    z_floor (m): refuse steps that would go below it (the only guard against pressing into the object).
    """
    print("\n[%s] jog:  a/d -x/+x   s/w -y/+y   f/r down/up (base frame)" % label)
    print("        step 1:0.02 2:0.1 3:0.5 4:2 5:10 6:50 mm   t: freedrive on/off (move by hand)   Enter: accept   q: abort")
    ensure_control_script(rtde_help)
    home_rot = list(rtde_help.rtde_r.getActualTCPPose())[3:]       # orientation to keep (tool down, set yaw)
    freedrive = False
    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            key = _read_key()
            if key is None:
                continue
            if key == "t":
                if not freedrive:
                    ensure_control_script(rtde_help)
                    rtde_help.rtde_c.teachMode()
                    freedrive = True
                    print("  FREEDRIVE: push the arm by hand, then t again")
                else:
                    rtde_help.rtde_c.endTeachMode()
                    freedrive = False
                    p = list(rtde_help.rtde_r.getActualTCPPose())
                    rtde_help.rtde_c.moveL(p[:3] + home_rot, JOG_SPEED, JOG_ACC)   # square the tool again, same point
                    p = rtde_help.rtde_r.getActualTCPPose()
                    print("  freedrive off, tool squared: x %.2f  y %.2f  z %.2f mm" % (p[0] * 1000, p[1] * 1000, p[2] * 1000))
                continue
            if freedrive:
                if key == "q":
                    raise KeyboardInterrupt                                   # finally ends freedrive
                if key in ("\n", "\r") or key in JOG_KEYS:
                    print("  leave freedrive first (t)")
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
            ensure_control_script(rtde_help)
            if not rtde_help.rtde_c.moveL(target, JOG_SPEED, JOG_ACC):
                print("  move refused by the controller (protective stop or limits?)")
            p = rtde_help.rtde_r.getActualTCPPose()
            print("  x %.2f  y %.2f  z %.2f mm" % (p[0] * 1000, p[1] * 1000, p[2] * 1000))
    finally:
        if freedrive:
            try:
                rtde_help.rtde_c.endTeachMode()
            except Exception:
                pass
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
