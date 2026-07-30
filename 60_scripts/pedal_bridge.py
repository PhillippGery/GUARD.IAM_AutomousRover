#!/usr/bin/env python3
"""
X-keys XK-3 foot pedal -> arrow keys, for hands-free LeRobot data collection.

The XK-3 pedals emit mouse-button codes (BTN_1/2/3), which pynput's keyboard
listener in collect.py doesn't see. This bridge reads the pedal's input device
and injects the matching ARROW KEY via uinput, so collect.py's existing
keyboard listener catches them exactly as if you'd typed.

    LEFT pedal   (BTN_1) -> LEFT ARROW   (scrap & re-record this episode)
    CENTER pedal (BTN_2) -> SPACEBAR     (pause for a break, between episodes only)
    RIGHT pedal  (BTN_3) -> RIGHT ARROW  (keep this episode, continue)

ESC (stop session) stays on the keyboard on purpose, so a stray pedal tap
can't end a session.

SETUP (once, inside the lerobot env):
    pip install evdev

RUN (in a SECOND terminal, alongside collect.py; needs sudo to read/inject):
    sudo $(which python) pedal_bridge.py

Then run `python collect.py` in your first terminal and drive with your feet.
Ctrl-C here to stop the bridge.
"""

import sys

try:
    from evdev import InputDevice, UInput, ecodes as e, list_devices
except ImportError:
    sys.exit("evdev not installed. Run:  pip install evdev  (inside the lerobot env)")

PEDAL_NAME = "P. I. Engineering XK-3 Foot Pedal"

# Map pedal button code -> key to inject.
BTN_TO_KEY = {
    e.BTN_1: e.KEY_LEFT,    # 257 -> left arrow  : scrap & re-record
    e.BTN_2: e.KEY_SPACE,   # 258 -> spacebar    : pause (between episodes only)
    e.BTN_3: e.KEY_RIGHT,   # 259 -> right arrow : keep & continue
}


def find_pedal():
    """Return the first XK-3 event device that actually reports the pedal buttons.
    The XK-3 shows up as 3 event nodes; only the one exposing BTN_* is useful."""
    for path in list_devices():
        try:
            dev = InputDevice(path)
        except Exception:
            continue
        if PEDAL_NAME in dev.name:
            caps = dev.capabilities().get(e.EV_KEY, [])
            if e.BTN_1 in caps or e.BTN_3 in caps:
                return dev
    return None


def main():
    # Optional explicit override: sudo $(which python) pedal_bridge.py /dev/input/eventNN
    if len(sys.argv) > 1:
        try:
            dev = InputDevice(sys.argv[1])
        except Exception as ex:
            sys.exit(f"[pedal] could not open {sys.argv[1]}: {ex}")
    else:
        dev = find_pedal()

    if dev is None:
        sys.exit("[pedal] XK-3 not found. List devices with `sudo evtest`, find the "
                 "node reporting BTN_* events, and pass it explicitly:\n"
                 "    sudo $(which python) pedal_bridge.py /dev/input/eventNN")

    # Virtual keyboard that can emit the arrow keys + spacebar.
    ui = UInput({e.EV_KEY: [e.KEY_LEFT, e.KEY_RIGHT, e.KEY_SPACE]}, name="xk3-pedal-keys")

    print(f"[pedal] reading: {dev.path}  ({dev.name})")
    print("[pedal] LEFT -> redo | CENTER -> SPACE/pause | RIGHT -> keep")
    print("[pedal] running — drive collect.py with your feet. Ctrl-C to stop.", flush=True)

    # Grab so the raw BTN_* events don't also leak to the desktop as clicks.
    try:
        dev.grab()
    except Exception as ex:
        print(f"[pedal] warning: could not grab device ({ex}); continuing ungrabbed.")

    try:
        for event in dev.read_loop():
            if event.type != e.EV_KEY:
                continue
            key = BTN_TO_KEY.get(event.code)
            if key is None:
                continue  # center pedal or anything else: ignore
            # event.value: 1 = press, 0 = release, 2 = autorepeat. Fire on press only.
            if event.value == 1:
                ui.write(e.EV_KEY, key, 1)   # key down
                ui.syn()
                ui.write(e.EV_KEY, key, 0)   # key up
                ui.syn()
                label = {e.KEY_LEFT: "LEFT/redo",
                         e.KEY_RIGHT: "RIGHT/keep",
                         e.KEY_SPACE: "CENTER/pause"}.get(key, "?")
                print(f"[pedal] {label}", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            dev.ungrab()
        except Exception:
            pass
        ui.close()
        print("\n[pedal] stopped.")


if __name__ == "__main__":
    main()