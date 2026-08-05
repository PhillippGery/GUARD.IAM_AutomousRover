# Phidget Motor Setup — read this before you plug anything in

Notes to future me, and anyone else spinning up the drivetrain. This is the
rehash running on Phidget controllers instead of the old Arduino/ESP32 rig I
built for the hackathon, so ignore muscle memory from that version. If a motor
won't move, the answer is almost always somewhere in here.

Two scripts live next to this file:
- `attach_test.py` — checks a controller is talking. No motion.
- `spin_test.py` — actually spins one motor.

## What I'm working with

- **DCM4109_1** — the motor. 24V brushless, NEMA23, 23:1 gearbox, hall sensors
  built in. Torquey and slow, tops out around 170 RPM at the output.
- **DCC1120** — the controller. One per motor. Handles the commutation and
  talks to the PC over VINT.
- **VINT Hub** — the USB bridge. All four controllers hang off this, and it
  plugs into the MiniPC over one USB cable.

One controller per motor, four total, all sharing the single hub.

## Step 0 — deps

```
pip install Phidget22
```

On Linux you also need a udev rule, but that's Step 3.

## Step 1 — wiring (leave everything powered OFF here)

Phase wires, motor to controller terminals:

- RED    -> A (U)
- YELLOW -> B (V)
- BLACK  -> C (W)

Hall: click the 5-pin hall connector into the hall header. Hall only. Leave the
encoder input empty unless I've actually added an encoder module (if I did,
wire it and re-check this).

E-stop: for bench testing, jump the STOP terminals right on the board (it ships
jumpered, so just leave it). On the real rover this feeds into the main e-stop
line instead, so don't leave it board-jumpered once it's on the chassis.

VINT ports: plug the four controllers into the hub. I'm using ports 0 and 2
(or 0 and 1) for one pair, and 5 and 4 (or 5 and 3) for the other. Keeps the
cabling from turning into spaghetti.

Do NOT power up yet. Bring the motors online one at a time: wire one, test it
all the way through, then move to the next. Doing all four at once is how a
wiring mistake slips past you.

## Step 2 — is it even there

```
lsusb
```

Look for a Phidgets line (vendor `06c2`, shows up as the VINT Hub). If it's not
listed, it's a USB or cable problem. Fix that before touching anything else.

## Step 3 — permissions (Linux)

Skip this and every run needs sudo. One-time setup:

```
sudo tee /etc/udev/rules.d/99-libphidget22.rules > /dev/null <<'EOF'
SUBSYSTEM=="usb", ATTRS{idVendor}=="06c2", MODE="0666"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Then unplug and replug the hub's USB so the rule actually takes.

## Step 4 — confirm comms

```
python attach_test.py --serial <serial>
```

Prints the device name, serial, and hub port if the controller's alive. No
motion. This is proving USB -> hub -> VINT cable -> controller. If it says
access denied, Step 3 didn't take. If it says no matching devices, power's off
or the VINT cable is loose.

## Step 5 — spin it

Free the shaft first, set the bench supply current limit low (~2A), turn on the
24V, then:

```
python spin_test.py --serial <serial> --duty 0.1 --seconds 2
```

What I'm watching for:
- Smooth spin = phase and hall wiring are right.
- Stutter or cogging = phase/hall mismatch. Power off, fix it.
- Wrong direction = flip the sign of `--duty`. Software only, no rewiring.

Then repeat Steps 4 and 5 for each motor as I bring them online. Write down
which serial goes to which wheel. The ROS bridge addresses them by serial, so
I'll need that mapping later.