#!/usr/bin/env python3
"""
spin_test.py — spin a single DCM4109 via its DCC1120 controller.

Standalone hardware check. No ROS. Ramps the motor to a small duty cycle, holds,
ramps back to zero. Use this to confirm a controller + motor + wiring are good,
to check spin direction, and to check for smooth commutation.

Usage:
    python spin_test.py                       # any controller, 10% duty, 2s
    python spin_test.py --serial 782265 --duty 0.15 --seconds 3
    python spin_test.py --duty -0.1           # reverse direction

Uses the raw BLDCMotor channel, where TargetVelocity is a DUTY CYCLE from -1.0
to 1.0 (not a real speed). This is the simplest possible motion test — closed-
loop velocity in real RPM is handled by the ROS bridge node, not here.

>>> BEFORE YOU RUN <<<
  - Free the shaft. The gearbox is torquey; make sure nothing can be flung or
    dragged, and keep fingers clear of the shaft/keyway.
  - Set a low current limit on your bench supply (~2A) for the first run.
  - Confirm the E-stop jumper is in (board ships shorted; open = failsafe).
  - 24V supply on, polarity correct.
"""

import argparse
import time

from Phidget22.Devices.BLDCMotor import BLDCMotor
from Phidget22.PhidgetException import PhidgetException


def main():
    parser = argparse.ArgumentParser(description="Phidget DCC1120 single-motor spin test")
    parser.add_argument("--serial", type=int, default=None,
                        help="controller serial number (omit to open any one)")
    parser.add_argument("--duty", type=float, default=0.1,
                        help="duty cycle -1.0..1.0 (default 0.1). Negative = reverse")
    parser.add_argument("--seconds", type=float, default=2.0,
                        help="how long to hold the duty (default 2.0)")
    parser.add_argument("--accel", type=float, default=2.0,
                        help="acceleration in duty/s (default 2.0, gentle ramp)")
    parser.add_argument("--timeout", type=int, default=5000,
                        help="attach timeout in ms (default 5000)")
    args = parser.parse_args()

    if not -1.0 <= args.duty <= 1.0:
        parser.error("--duty must be between -1.0 and 1.0")

    ch = BLDCMotor()
    if args.serial is not None:
        ch.setDeviceSerialNumber(args.serial)

    print("Attaching...")
    try:
        ch.openWaitForAttachment(args.timeout)
    except PhidgetException as e:
        print(f"FAILED to attach: {e.details}")
        print("Run attach_test.py first to diagnose. Is 24V power on?")
        return

    print(f"Attached: {ch.getDeviceName()} (serial {ch.getDeviceSerialNumber()})")

    # Safety countdown so nobody's hands are near a shaft about to move.
    print(f"\nSpinning at duty {args.duty} for {args.seconds}s. Clear the shaft.")
    for n in (3, 2, 1):
        print(f"  {n}...")
        time.sleep(1.0)

    try:
        ch.setAcceleration(args.accel)
        ch.setTargetVelocity(args.duty)   # duty cycle, not RPM
        time.sleep(args.seconds)
    except PhidgetException as e:
        print(f"Error while driving: {e.details}")
    finally:
        # Always ramp back to stop and release, even on error/Ctrl-C.
        try:
            ch.setTargetVelocity(0.0)
            time.sleep(0.5)
        except PhidgetException:
            pass
        ch.close()

    print("\nDone. Checklist:")
    print("  - Smooth spin?  -> phase/hall wiring is correct.")
    print("  - Stutter/cog?  -> power off, fix phase-to-hall mapping.")
    print("  - Wrong way?    -> just flip the sign of --duty (software, no rewiring).")


if __name__ == "__main__":
    main()