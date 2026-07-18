#!/usr/bin/env python3
"""
attach_test.py — confirm a Phidget DCC1120 BLDC controller is talking.

Standalone hardware check. No ROS, no motion — it just opens the controller
and prints its identity. Run this FIRST when bringing up a new board or
debugging a connection.

Usage:
    python attach_test.py                # open any single controller on the bus
    python attach_test.py --serial 782265   # target a specific controller

Motor power does NOT need to be on for the controller to attach... except the
DCC1120 wants its supply present to bring up the motor channel, so if this
times out with power off, turn the 24V on and retry.
"""

import argparse
import sys

from Phidget22.Devices.BLDCMotor import BLDCMotor
from Phidget22.PhidgetException import PhidgetException


def main():
    parser = argparse.ArgumentParser(description="Phidget DCC1120 attach test")
    parser.add_argument("--serial", type=int, default=None,
                        help="controller serial number (omit to open any one)")
    parser.add_argument("--timeout", type=int, default=5000,
                        help="attach timeout in ms (default 5000)")
    args = parser.parse_args()

    ch = BLDCMotor()
    if args.serial is not None:
        ch.setDeviceSerialNumber(args.serial)

    print("Waiting for a controller to attach...")
    try:
        ch.openWaitForAttachment(args.timeout)
    except PhidgetException as e:
        print(f"\nFAILED to attach: {e.details}\n")
        print("Common causes:")
        print("  - 'access denied'     -> Linux permissions; install the udev rule (see README)")
        print("  - 'no matching devices' -> controller unpowered, VINT cable loose, or wrong --serial")
        sys.exit(1)

    print("\n=== ATTACHED ===")
    print(f"  Device : {ch.getDeviceName()}")
    print(f"  Serial : {ch.getDeviceSerialNumber()}")
    print(f"  Hub port : {ch.getHubPort()}")
    print(f"  Channel  : {ch.getChannel()}")
    print("================\n")
    print("Comms confirmed. This proves USB -> hub -> VINT cable -> controller.")
    print("It does NOT prove the motor spins — run spin_test.py for that.")

    ch.close()


if __name__ == "__main__":
    main()