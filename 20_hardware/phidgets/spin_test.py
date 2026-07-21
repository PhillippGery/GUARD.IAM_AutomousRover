#!/usr/bin/env python3
"""
spin_test.py — spin one or more DCM4109 motors via their DCC1120 controllers.

Standalone hardware check. No ROS. Ramps each motor to a small duty cycle,
holds, ramps back to zero. Use it to confirm wiring, check spin direction, and
listen for smooth commutation.

Usage:
    python spin_test.py --hub-port 2                  # one motor
    python spin_test.py --hub-port 2 --hub-port 3     # both, one after the other
    python spin_test.py --hub-port 2,3                # same thing, shorter
    python spin_test.py --hub-port 2,3 --duty -0.1    # both in reverse
    python spin_test.py                               # any one controller (legacy)

Label them as you go, e.g. port 2 = BL, port 3 = BR.

Motors run SEQUENTIALLY, not together — one wheel at a time is the point, so
you can watch each shaft and note its direction.

Uses the raw BLDCMotor channel, where TargetVelocity is a DUTY CYCLE from -1.0
to 1.0 (not a real speed). Closed-loop RPM lives in the ROS bridge node.

>>> BEFORE YOU RUN <<<
  - Free the shafts. The gearbox is torquey; nothing should be able to be flung
    or dragged, and keep fingers clear of the shaft and keyway.
  - Bench supply current limit low (~2A) for first runs.
  - E-stop jumper in (board ships shorted; open = failsafe = no motion).
  - 24V on, polarity correct.
"""

import argparse
import time

from Phidget22.Devices.BLDCMotor import BLDCMotor
from Phidget22.PhidgetException import PhidgetException


def parse_ports(values):
    """Accept --hub-port 2 --hub-port 3 and --hub-port 2,3 alike."""
    ports = []
    for v in values or []:
        for part in str(v).split(","):
            part = part.strip()
            if part:
                ports.append(int(part))
    return ports


def spin_one(hub_port, args):
    label = f"hub port {hub_port}" if hub_port is not None else "any controller"
    print(f"\n--- {label} ---")

    ch = BLDCMotor()
    if args.serial is not None:
        ch.setDeviceSerialNumber(args.serial)
    if hub_port is not None:
        ch.setHubPort(hub_port)
        ch.setChannel(0)

    try:
        ch.openWaitForAttachment(args.timeout)
    except PhidgetException as e:
        print(f"  FAILED to attach: {e.details}")
        print("  Run enumerate_phidgets.py to see which ports are populated.")
        return False

    print(f"  Attached: {ch.getDeviceName()} (serial {ch.getDeviceSerialNumber()})")
    print(f"  Spinning at duty {args.duty} for {args.seconds}s. Clear the shaft.")
    for n in (3, 2, 1):
        print(f"    {n}...")
        time.sleep(1.0)

    try:
        ch.setAcceleration(args.accel)
        ch.setTargetVelocity(args.duty)   # duty cycle, not RPM
        time.sleep(args.seconds)
    except PhidgetException as e:
        print(f"  Error while driving: {e.details}")
    finally:
        try:
            ch.setTargetVelocity(0.0)
            time.sleep(0.5)
        except PhidgetException:
            pass
        ch.close()

    print("  Stopped. Note: smooth or stuttering? Which direction?")
    return True


def main():
    ap = argparse.ArgumentParser(description="Phidget DCC1120 spin test")
    ap.add_argument("--hub-port", action="append", default=None,
                    help="VINT hub port. Repeat, or comma-separate (e.g. 2,3). "
                         "Omit to open any one controller.")
    ap.add_argument("--serial", type=int, default=None,
                    help="VINT hub serial (optional; only one hub here)")
    ap.add_argument("--duty", type=float, default=0.1,
                    help="duty cycle -1.0..1.0 (default 0.1). Negative = reverse")
    ap.add_argument("--seconds", type=float, default=2.0,
                    help="hold time per motor (default 2.0)")
    ap.add_argument("--accel", type=float, default=2.0,
                    help="acceleration in duty/s (default 2.0)")
    ap.add_argument("--gap", type=float, default=2.0,
                    help="pause between motors, seconds (default 2.0)")
    ap.add_argument("--timeout", type=int, default=5000,
                    help="attach timeout in ms (default 5000)")
    args = ap.parse_args()

    if not -1.0 <= args.duty <= 1.0:
        ap.error("--duty must be between -1.0 and 1.0")

    ports = parse_ports(args.hub_port)
    if not ports:
        ports = [None]  # legacy behaviour: open whatever's there

    ok = 0
    for i, port in enumerate(ports):
        if spin_one(port, args):
            ok += 1
        if i < len(ports) - 1:
            print(f"\n  (pausing {args.gap}s before next motor)")
            time.sleep(args.gap)

    print(f"\nDone. {ok}/{len(ports)} motor(s) driven.")
    print("Checklist:")
    print("  - Smooth spin?  -> phase/hall wiring is correct.")
    print("  - Stutter/cog?  -> power off, fix phase-to-hall mapping.")
    print("  - Wrong way?    -> record it; the bridge node flips signs in config.")


if __name__ == "__main__":
    main()