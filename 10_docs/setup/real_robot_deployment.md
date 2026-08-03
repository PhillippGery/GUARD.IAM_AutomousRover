# Real Robot Deployment — Boot-Time Web Control Interface

Step-by-step for getting the robot to boot up controllable from a browser,
with no SSH/monitor/keyboard needed. Run all of this **on the robot's own
mini PC**, not a dev/lab machine.

At the end: the mini PC boots → drive chain + LIDARs come up automatically
→ the browser control interface (Foxglove) is reachable over ZeroTier →
Nav2/SLAM only starts when you actually press a button.

## 1. Prerequisites

```bash
sudo apt update && sudo apt install -y ros-jazzy-foxglove-bridge
```

If `ros-jazzy-navigation2`, `ros-jazzy-nav2-bringup`, `ros-jazzy-slam-toolbox`
aren't already installed, see [amd_minipc_setup.md](amd_minipc_setup.md).

Also worth running once — out-of-sync `diagnostic_updater`/nav2 packages
have caused `nav2_lifecycle_manager` to crash with a symbol-lookup error
before:

```bash
sudo apt update && sudo apt upgrade -y
```

## 2. Pull and build

```bash
cd ~/GUARD.IAM_AutomousRover && git pull
cd 30_ros2_ws && colcon build --symlink-install
```

## 3. Test before committing to boot

```bash
guardiam_test_boot
```

(or `bash 60_scripts/systemd/test_services.sh` if `guardiam_env.sh` isn't
sourced yet — it should be, via `~/.bashrc`)

This installs the systemd `--user` units and starts them **now**, without
enabling boot-time auto-start. Verify, in order:

1. **Hardware**: `systemctl --user status guardian-hardware` shows
   `active (running)`. Motors respond to Xbox teleop. Both LIDARs publish
   (`ros2 topic hz /scan`, `/scan_back`).
2. **Browser connection**: from any device on the ZeroTier network, open
   `https://app.foxglove.dev`, connect to
   `ws://<mini-pc's-ZeroTier-IP>:8765`. The robot mesh renders (see
   §5 below if it doesn't — this is the one dev-machine-only step).
3. **Mode switching**: click 🗺️ Start Mapping, then 🧭 Start Navigation.
   Each should flip 🟢 Stack Ready to green within a few seconds.
4. **Goal-sending**: use the Publish panel (topic `/goal_pose`) to send a
   goal. The Log panel should show `Navigating — X.XXm remaining...`
   then `Idle` once it arrives.
5. **Waypoints**: drive somewhere, set `target_waypoint_index` to `0` in
   the Parameters panel, click ✓. The Log panel confirms the write.
6. **Demo**: 🚀 Start Demo drives through whatever's in `waypoints.yaml`.
7. **Save Map**: 💾 Save Map while mapping — confirm
   `30_ros2_ws/src/guardian_bringup/maps/guardian_map.{pgm,yaml}` updates.

If anything fails, fix it and re-run `guardiam_test_boot` — it's safe to
run repeatedly (`systemctl restart`, not `start`, so it always picks up
the latest build/unit files).

## 4. Commit to the boot sequence

Once everything above checks out:

```bash
guardiam_install_boot
```

This `systemctl --user enable --now`s the three always-on services
(hardware, foxglove_bridge, web_ops_node) and runs
`loginctl enable-linger` (one `sudo` prompt) — without linger, user
services only run while a login session is active, not from a cold boot
with nobody logged in.

`guardian-stack@.service` (Nav2/SLAM) is **not** enabled here on purpose —
only `guardian-hardware`, `guardian-foxglove-bridge`, and `guardian-ops`
start automatically. Nav2 only ever runs when you press Start
Mapping/Navigation from the browser.

## 5. Reboot and verify

```bash
sudo reboot
```

Wait ~30s, then from a browser: `ws://<mini-pc's-ZeroTier-IP>:8765`. If it
connects and the buttons work with nobody having logged into the mini PC
since reboot, boot persistence is confirmed.

## 6. Foxglove layout — do this once, on any machine connecting for the first time

The Foxglove **layout** (panel arrangement) is separate from the robot
setup above — it lives in your Foxglove account, not on the robot. If
you're setting up a fresh browser/account:

- **3D panel** → Settings → Scene → **Mesh up-axis: Z-up** (do this
  *before* adding the URDF layer — see known-issues note below).
- Custom Layers → **+ Add → URDF** → Source: Topic, Topic:
  `/robot_description`, Control mode: Transforms.
- Settings → Topics → toggle `/map`, `/plan`, `/plan_smoothed` visible.
- Settings → Frame → **Display frame: map** (not `base_link` — goals
  published from the wrong frame get silently rejected by `bt_navigator`).
- Add panel → **Publish**, topic `/goal_pose`
  (`geometry_msgs/msg/PoseStamped`) for sending Nav2 goals.
- Add panel → **Service Call** ×4, one each for `/guardian/save_map`,
  `/guardian/switch_to_mapping`, `/guardian/switch_to_navigation`,
  `/guardian/start_demo`.
- Add panel → **Indicator**, expression `/guardian/stack_ready.data`,
  rule `= true` → green "Ready".
- Add panel → **Log**, then in its Namespaces filter hide
  `foxglove_bridge`/`gz_bridge` (chatty, unrelated) — shows
  `nav_status_node` and `set_waypoint_node` confirmations cleanly.
- Add panel → **Parameters**, filter to `target_waypoint_index` — type an
  index, click the ✓ to record the robot's current pose there.

Full rationale for each of these is in
[web_control_interface_setup.md](web_control_interface_setup.md) if
something doesn't behave as expected.

## Known issues to watch for

- **URDF mesh sometimes vanishes on a mode switch, even though the
  backend is fine.** Reload the browser tab — this has reliably fixed it
  every time it's happened. Not a robot-side problem.
- **A long-running `ros2` CLI daemon can go stale** after enough
  restarts/kills and cause Nav2 nodes to look alive via `ps` but never
  actually configure (`activate_lifecycle_node.sh` loops
  `Node not found`). Fix: `ros2 daemon stop` (auto-restarts fresh on the
  next `ros2` command).
- **First-ever boot against real Phidgets/LIDARs is untested as of this
  writing** — everything above was validated in sim. Budget extra time
  for the first real run of §3.
