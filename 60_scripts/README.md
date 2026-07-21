# Scripts

Convenience shell scripts for setup, building, and running GUARDIAN.

| Script | Purpose |
|--------|---------|
| `setup_amd_minipc.sh` | One-shot install of all system dependencies on Ubuntu 22.04 |
| `build_workspace.sh` | Source ROS2, install deps, build workspace |
| `run_guardian.sh` | Interactive launch menu |
| `guardiam_env.sh` | Per-session env: sources ROS2 Jazzy + workspace, sets `ROS_DOMAIN_ID=42`, defines `guardiam_sim`/`guardiam_real`/`guardiam_map`/`cb`/`cbs` aliases |
| `identify_lidar_port.sh` | Print a connected USB-serial device's identifying info (serial number, etc.), one LIDAR at a time |
| `99-guardian-lidar.rules` | udev rules template pinning `/dev/lidar_front`/`/dev/lidar_back` to each LIDAR's own USB serial number |

## Usage

```bash
# First-time setup (run once after cloning)
bash 60_scripts/setup_amd_minipc.sh

# Build workspace
bash 60_scripts/build_workspace.sh

# Launch
bash 60_scripts/run_guardian.sh
```

`setup_amd_minipc.sh` adds `source 60_scripts/guardiam_env.sh` to `~/.bashrc` automatically. To pick it up in your current shell without re-running setup:

```bash
source 60_scripts/guardiam_env.sh
```

## One-time: labeling the front/back LIDARs

The real robot has two identical Scanse Sweep LIDAR units, and `guardian.launch.py`'s real-hardware branch expects them at the stable paths `/dev/lidar_front` and `/dev/lidar_back` — not raw `/dev/ttyUSB0`/`1`, which Linux assigns by USB enumeration order and can silently swap between the two physical units on a reboot or replug.

```bash
# 1. Plug in ONE LIDAR (unplug the other / any other USB-serial adapter)
bash 60_scripts/identify_lidar_port.sh
# Read off the ATTRS{serial} value shown. Physically label that unit
# (tape marked "F" for front, or "B" for back) so the label matches
# which end of the chassis it's actually mounted on.

# 2. Unplug it, plug in the second LIDAR, repeat step 1 for its serial.

# 3. Put both serial numbers into 60_scripts/99-guardian-lidar.rules
#    (replacing REPLACE_WITH_FRONT_SERIAL / REPLACE_WITH_BACK_SERIAL),
#    then install it:
sudo cp 60_scripts/99-guardian-lidar.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# 4. Plug both LIDARs in and confirm:
ls -la /dev/lidar_front /dev/lidar_back
```

Do this once per robot; it survives reboots and USB replugs from then on.
