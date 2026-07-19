#!/usr/bin/env python3
"""
enumerate_phidgets.py — find which VINT hub ports have motor controllers on them.

Probes each hub port in turn and reports what answers. Unlike attach_test.py
(which opens ONE arbitrary matching device and can't tell you where it is),
this walks every port so you can build a wheel -> hub port map.

    python enumerate_phidgets.py
    python enumerate_phidgets.py --ports 6 --timeout 800

Connect as many controllers as you have (one is fine) and power them, then run
this and note which port each wheel's controller answers on.

NOTE ON ADDRESSING: VINT devices like the DCC1120 report the serial number of
the VINT HUB they're plugged into, not one of their own. Expect every
controller to show the SAME serial — the HUB PORT is what distinguishes them.
Address them as (serial, hub_port), never by serial alone.

This deliberately uses only BLDCMotor, so it works even if the Phidget22
Manager submodule is missing from your install.
"""

import argparse

from Phidget22.Devices.BLDCMotor import BLDCMotor
from Phidget22.PhidgetException import PhidgetException


def probe(hub_port, timeout_ms):
    """Try to open a BLDC controller on one hub port. Return info dict or None."""
    ch = BLDCMotor()
    ch.setHubPort(hub_port)
    ch.setChannel(0)
    try:
        ch.openWaitForAttachment(timeout_ms)
    except PhidgetException:
        try:
            ch.close()
        except PhidgetException:
            pass
        return None

    try:
        info = {
            "name": ch.getDeviceName(),
            "serial": ch.getDeviceSerialNumber(),
            "hub_port": ch.getHubPort(),
            "channel": ch.getChannel(),
        }
    except PhidgetException:
        info = None
    finally:
        try:
            ch.close()
        except PhidgetException:
            pass
    return info


def main():
    ap = argparse.ArgumentParser(description="Map VINT hub ports to motor controllers")
    ap.add_argument("--ports", type=int, default=6,
                    help="number of hub ports to probe (default 6)")
    ap.add_argument("--timeout", type=int, default=800,
                    help="per-port attach timeout in ms (default 800)")
    args = ap.parse_args()

    print(f"Probing hub ports 0..{args.ports - 1}\n")

    found = []
    for port in range(args.ports):
        info = probe(port, args.timeout)
        if info:
            found.append(info)
            print(f"  port {port}: {info['name']}  (serial {info['serial']})")
        else:
            print(f"  port {port}: -")

    print()
    if not found:
        print("No controllers found.")
        print("Check: 24V power on, VINT cables seated, USB connected, udev rules installed.")
        return

    serials = {f["serial"] for f in found}
    print(f"{len(found)} controller(s) on ports: {[f['hub_port'] for f in found]}")
    if len(found) > 1 and len(serials) == 1:
        print(f"All share serial {serials.pop()} — that's the VINT hub's serial, as expected.")
        print("Distinguish the controllers by HUB PORT.")
    elif len(found) > 1:
        print(f"Distinct serials seen: {sorted(serials)}")

    print("\nRecord which WHEEL is on which HUB PORT. That mapping goes in the")
    print("bridge node's config — it's physical, so nothing can auto-detect it.")


if __name__ == "__main__":
    main()