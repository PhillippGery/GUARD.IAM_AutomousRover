# Browser Control Interface Setup (Foxglove)

What this gives you: open a browser anywhere on the ZeroTier network (no
SSH, no monitor/keyboard on the robot), see the robot's 3D view + live map,
send Nav2 goals, and press buttons to start mapping, start navigation, or
save the map — all against hardware that auto-starts the moment the mini
PC boots.

Everything below was built and verified **in sim, on a lab PC** (not the
real robot), end-to-end, not just eyeballed:
- Robot mesh + map rendering — confirmed correct against Gazebo's own
  native render.
- 🗺️ Start Mapping / 🧭 Start Navigation / 💾 Save Map buttons — each
  tested by actually clicking them; each does the real thing (verified
  via `systemctl` state and `git diff` on the saved map).
- Nav goal sending — tested by watching `bt_navigator`'s log show `Begin
  navigating from current location (...) to (x, y)` and confirming the
  robot's `/odom` position actually reached the target.

The hardware-launch path (`guardian_hardware.launch.py`,
`guardian-hardware.service`) is written and logically consistent with how
the rest of the stack already works, but has **not been run against real
Phidgets/LIDARs yet** — that first real run is tomorrow's job.

## 1. Prerequisites — install once on the mini PC

```bash
sudo apt update && sudo apt install -y ros-jazzy-foxglove-bridge
```

(If `ros-jazzy-navigation2`, `ros-jazzy-nav2-bringup`, `ros-jazzy-slam-toolbox`,
`ros-jazzy-ros-gz` aren't already on the mini PC, install those too — see
`10_docs/setup/amd_minipc_setup.md`.)

Also worth doing once, if `diagnostic_updater` and friends aren't in sync
with the nav2 packages (this bit us on the lab PC — `nav2_lifecycle_manager`
crashed with a symbol-lookup error until this was run):

```bash
sudo apt update && sudo apt upgrade -y
```

## 2. Pull the code and build

```bash
cd ~/GUARD.IAM_AutomousRover
git pull
cd 30_ros2_ws
colcon build --symlink-install
```

## 3. Install the systemd services

```bash
bash 60_scripts/systemd/install_services.sh
```

This installs 4 **user-level** systemd units (no root needed for the
services themselves — only `loginctl enable-linger` at the end needs
`sudo`, so it's a one-time password prompt):

| Unit | Starts at boot? | What it does |
|---|---|---|
| `guardian-hardware.service` | Yes | Drive chain (mecanum + Phidget bridge) + both LIDARs. **Not yet tested on real hardware.** |
| `guardian-foxglove-bridge.service` | Yes | `foxglove_bridge` on port 8765, bound to `0.0.0.0` (reachable over ZeroTier) |
| `guardian-ops.service` | Yes | `web_ops_node` — exposes the 3 buttons described below |
| `guardian-stack@.service` | No (started on demand) | The actual Nav2/SLAM stack, `mode:=mapping` or `mode:=navigation`. Started/stopped by the ops node, not by systemd directly. |

Check they're up:

```bash
systemctl --user status guardian-hardware guardian-foxglove-bridge guardian-ops
```

## 4. Connect from a browser

From any device on the same ZeroTier network:

1. Go to `https://app.foxglove.dev` (sign in if needed).
2. **Open connection** → enter `ws://<mini-pc's ZeroTier IP>:8765`.
3. If it won't connect: check `foxglove_bridge` is actually running
   (`systemctl --user status guardian-foxglove-bridge`), and double check
   you're using the ZeroTier IP, not `localhost` (that only works if
   you're on the mini PC itself).

## 5. Build the layout (one-time, then it auto-saves)

**3D panel** (usually there by default):
- Settings → **Scene** → **Mesh up-axis** = `Z-up`. **Do this first,
  before adding the URDF layer.** Foxglove's 3D panel defaults this to
  `Y-up`, which silently rotates every mesh (chassis, wheels, LIDARs) 90°
  off its true axis — RViz has no such setting/default mismatch, which is
  why the same URDF rendered correctly there but not here. This is a
  per-panel client-side setting, completely independent of the URDF/TF
  data (which is correct either way) — it explains why *all* meshes were
  affected identically (a global panel setting, not a per-mesh bug) and
  why it has nothing to do with `step_to_description.py` or
  `guardian.urdf.xacro` (both untouched, confirmed via `git diff`). If
  you add the URDF layer before fixing this, or fix this while the layer
  errors out, Foxglove can get stuck showing a spurious `Invalid topic:
  '/robot_description'` error even with the topic correctly selected —
  if that happens, delete the URDF custom layer, reload the page (not
  just the panel), then re-add it fresh.
- Settings → **Custom Layers** → **+ Add** → **URDF**. Set **Source** =
  `Topic`, **Topic** = `/robot_description`, **Control mode** =
  `Transforms`. This renders the actual robot mesh (chassis, wheels,
  LIDARs) — confirmed working, meshes load automatically over the
  websocket (`foxglove_bridge` ≥ v0.7 fetches `package://` assets itself,
  no extra config needed).
- Settings → **Topics** → toggle `/map` visible (the eye icon on the
  right of the row) — this is the actual occupancy grid, separate from
  the "Map" custom layer (which is for satellite/street tiles, not this).
- Settings → **Transforms** → turn **Labels** off (the floating
  `front_left_wheel` etc. text is clutter once the mesh renders) and set
  **Follow mode** = `Position` (not `Pose (position + attitude)` — that
  one tilts the camera to match the robot's exact roll/pitch, which looks
  like a broken/tilted view even though nothing is actually wrong).
- Settings → **Frame** → **Display frame** = `map`, not `base_link`. This
  isn't just a camera setting — it's also the `frame_id` Foxglove stamps
  on anything you publish from the 3D view's click tools. With it left on
  `base_link`, `bt_navigator` rejects every click-published goal with
  `Failed to transform a goal pose provided with frame_id 'base_link' to
  the global frame 'map'` (confirmed in the systemd journal). `Follow
  mode: Position` still works fine with `Display frame: map` — the camera
  just stays map-anchored instead of chasing the robot, which is actually
  the more useful view for placing goals anyway.
- Settings → **Publish** → **2D pose** section → change the topic from
  the default `/move_base_simple/goal` (a ROS1 convention Foxglove ships
  as the default) to **`/goal_pose`** — that's what `bt_navigator`
  actually listens on here (confirmed against `nav2_params.yaml`, no
  remap configured).

**Goal-setting: use a Publish panel, not the 3D view's click tool.**
This was the hardest bug of the night. The 3D view's interactive
"Publish 2D pose" tool (right-click the pose icon in the 3D panel
toolbar → pick the mode → click-drag in the scene) creates a **brand
new, ephemeral DDS publisher on `foxglove_bridge` every time you use
it**, and publishes immediately — before DDS discovery has finished
matching that fresh publisher to `bt_navigator`'s subscriber. The
message gets sent, `ros2 topic echo` can even catch it (a generic
subscriber matches fast enough), but `bt_navigator` specifically misses
it more often than not. No error, no log line — the goal just silently
never arrives. Confirmed by directly checking `ros2 topic info
/goal_pose --verbose`: two separate `foxglove_bridge` publisher GIDs
existed after only two clicks, and neither publish showed up in
`bt_navigator`'s log even after waiting 10+ seconds in a completely idle
system.

The fix: **Add panel → Publish** (not the 3D view tool). Configure once:
Topic = `/goal_pose`, it auto-detects the schema
(`geometry_msgs/msg/PoseStamped`). Edit the JSON body's `pose.position.x`
/ `y` and `header.frame_id: "map"`, then hit the panel's own **Publish**
button. This panel's publisher is created once when the panel is added
and stays alive, so there's no per-click discovery race — confirmed
working end-to-end repeatedly: `bt_navigator` logs `Begin navigating from
current location (...) to (x, y)` immediately, and the robot's `/odom`
position was verified to actually reach the target. Title the panel
something like "🎯 Send Nav Goal (edit x/y, then Publish)" so it's
obvious how to use it without re-deriving this.

**Three Service Call panels** (Add panel → Service Call, once per button).
Each one: click the panel's own title bar first to make sure the sidebar
is actually editing *that* panel before typing (Foxglove's settings
sidebar doesn't always follow panel focus reliably — verify the Service
name field is empty before typing into it, or you'll silently overwrite
a different panel's config):

| Service name | Button title |
|---|---|
| `/guardian/switch_to_mapping` | 🗺️ Start Mapping |
| `/guardian/switch_to_navigation` | 🧭 Start Navigation |
| `/guardian/save_map` | 💾 Save Map |
| `/guardian/start_demo` | 🚀 Start Demo |

All four were tested by actually clicking them (not just configured) —
each returns `success: true` and does the real thing (switches the
running systemd unit instance, saves+rebuilds the map, or launches
`demo_mission_node` via `subprocess.Popen` — fire-and-forget, not
`.run()`, since the demo runs for minutes and a blocking call here would
freeze `web_ops_node` itself for that whole time, same failure mode
`_save_map` used to have; a second click while one's already running
returns `success: false` with the running PID instead of stacking a
duplicate).

**Path, status, and waypoint recording** — three more additions, all
served by `nav_status_node` and `set_waypoint_node` (started
automatically alongside the rest of Nav2 core, in both mapping and
navigation mode):

- **Planned path**: Settings → **Topics** → toggle `/plan` (global) and
  `/plan_smoothed` visible in the 3D panel — Nav2 already publishes
  these, no code needed.
- **Nav status + readiness**: `nav_status_node` publishes two latched
  topics — `/guardian/nav_status` (`std_msgs/String`, re-publishing
  `NavigateToPose`'s action feedback — `distance_remaining` and
  `estimated_time_remaining` are already computed by Nav2, nothing to
  calculate ourselves) and `/guardian/stack_ready` (`std_msgs/Bool` —
  true once `controller_server` and `bt_navigator` both report lifecycle
  state `active`, i.e. actually safe to send a goal or start the demo;
  polled via the same `GetState` service calls
  `nav2_simple_commander.waitUntilNav2Active()` uses internally, so it
  works the same in mapping mode too, unlike checking `amcl` directly).
  Both also go through `get_logger()` on every *change* (not every
  feedback tick — that would spam), so **Add panel → Log**, then in the
  panel's Namespaces list hide `foxglove_bridge`/`gz_bridge` (chatty,
  unrelated), gives a clean timestamped status feed for free instead of
  raw JSON in a Raw Messages panel. Separately, **Add panel → Indicator**
  with Expression `/guardian/stack_ready.data` (rule `= true` → green
  "Ready", fallback → gray "Not Ready") gives an at-a-glance readiness
  light.
- **Waypoint recording from the browser**: `set_waypoint_node` now runs
  in two modes from the same script — the original one-shot CLI
  (`set_waypoint <index>` / `--ros-args -p index:=N`, unchanged) *and*,
  when launched with no `index` param (which is how `guardian.launch.py`
  starts it), a persistent listener on `/guardian/set_waypoint_index`
  (`std_msgs/Int32`). **Add panel → Publish**, Topic =
  `/guardian/set_waypoint_index`, schema auto-detects as `std_msgs/Int32`
  — drive the robot to where waypoint `N` should be, edit `data` to `N`,
  hit Publish, same one-action feel as the CLI alias. Confirmation
  (or the "no transform available" error, if localization isn't up yet)
  shows up in the same Log panel as nav_status, no extra topic needed.

## Known issues / things to double check tomorrow

- **URDF mesh sometimes vanishes on a mode switch (mapping↔navigation),
  even though `/robot_description`, TF, and the URDF layer's own config
  are all fine.** Confirmed on the backend: `robot_state_publisher` never
  restarts on a mode switch (it lives in `guardian_sim.launch.py`, not
  `guardian.launch.py`), `/robot_description` stays latched with the same
  single publisher throughout, and every wheel/laser TF frame keeps
  updating — none of that explains the mesh disappearing, and the URDF
  layer shows no error icon when it happens (unlike the earlier `Invalid
  topic` corruption, which did). This looks like a Foxglove-client-side
  rendering hiccup triggered by the burst of ~15 nodes stopping/starting
  within a couple seconds during a mode switch — outside anything fixable
  from this repo. **Workaround**: reload the browser tab. Confirmed this
  reliably brings the mesh back (not just a coincidence — reproduced and
  fixed this exact way twice in one session). If it recurs after a mode
  switch, reload first before assuming something is actually broken.
- **A long-running `ros2` CLI daemon can go stale and cause very
  confusing failures that look like real Nav2 bugs.** One evening's worth
  of `kill_ros.sh` sweeps and node restarts left the daemon (started
  hours earlier, `ros2 daemon status` / `pgrep -f ros2cli.daemon`) with a
  stale view of the graph: `ros2 node list` stopped showing `map_server`/
  `amcl` at all, even though both were alive and directly
  service-callable. This silently broke `activate_lifecycle_node.sh`'s
  own node-existence check (`ros2 node info <name>`, which goes through
  the same daemon), so it looped `Node not found` forever and never
  configured either node — `map_server`/`amcl` stuck at lifecycle state
  `unconfigured` indefinitely, `demo_mission_node` hanging forever on
  `amcl/get_state service not available, waiting...`, `planner_server`
  failing to activate its costmap ("transform from base_link to map did
  not become available"). **Fix**: `ros2 daemon stop` (it auto-restarts
  fresh on the next `ros2` CLI invocation and immediately sees the
  correct graph). If Nav2 nodes look alive via `ps`/`journalctl` but
  `activate_lifecycle_node.sh` keeps saying `Node not found`, or a
  service is directly callable but doesn't show up in `ros2 node list`,
  suspect a stale daemon before assuming a real Nav2/DDS problem.
- **`kill_ros.sh`'s force-kill pattern used to be a hand-enumerated list
  of binary names, and kept missing new orphans** — `nav2_*` executables,
  `teleop_twist_joy`'s actual `teleop_node` binary, `ros_gz_bridge`'s
  actual `parameter_bridge` binary, and even plain
  `robot_state_publisher` each individually slipped past earlier versions
  of the pattern (a process's *ROS node name* is very often just a
  `--ros-args -r __node:=` remap with nothing to do with its binary's
  actual filename). One `parameter_bridge` orphan survived 45+ minutes
  and multiple `kill_ros.sh` runs this way, ending up as a second live
  publisher on `/scan`/`/odom`/`/tf`/`/clock` racing a freshly-launched
  one. Fixed by matching `--ros-args` generally (every rclpy/rclcpp node
  gets this flag appended regardless of binary name) instead of
  enumerating binaries — a real fix, not another name added to the list.
- **Real hardware not yet tested.** `guardian-hardware.service` has never
  run against actual Phidgets/LIDARs. First boot on the mini PC is the
  real test.
- **Chassis/wheel/LIDAR mesh orientation — resolved, no code change
  needed.** Root cause: Foxglove 3D panel's **Mesh up-axis** setting
  defaults to `Y-up`; ROS/URDF meshes are `Z-up`. Fixed by the Scene
  setting documented in section 5 above — this is a **per-layout
  setting** (saved with the layout, not per-robot or per-mesh), so it
  only needs to be set once when building the layout, and will still be
  correct "the moment we have a new STL file" since it has nothing to do
  with mesh content. Verified from multiple camera angles (top-down,
  oblique, and a close-up side profile) — wheel cylinders' rotation axis
  correctly aligns with their joint's TF axis, chassis frame renders
  rectangular and level, not skewed. `step_to_description.py` and
  `guardian.urdf.xacro` are both untouched at their original committed
  state — confirmed via `git diff` — and should stay that way; this was
  never a mesh-generation or URDF bug.
- **`kill_ros.sh`** was patched this session — its force-kill pattern
  didn't match Nav2's own executables (`nav2_controller`,
  `bt_navigator`, etc., which live under `/opt/ros/*/lib/nav2_*/` and
  don't match any of the script's other patterns), so they could survive
  as orphans after a SIGINT-only shutdown. Fixed by adding `nav2_` to the
  pattern — if you see orphaned Nav2 processes after a shutdown, that's
  the symptom to check for.
- **`ROS_DOMAIN_ID` consistency matters**: every launch (hardware, stack,
  foxglove_bridge, ops node, and any manual `ros2` CLI you run to debug)
  needs to go through `guardiam_env.sh` (which sets `ROS_DOMAIN_ID=42`).
  Mixing a domain-42 process with a domain-0 shell makes them invisible
  to each other with no obvious error — this cost real debugging time
  this session.
- **`save_map.sh`** was fixed this session — it used to break when
  invoked via its *installed* (symlinked) path instead of directly from
  source, because it didn't resolve the symlink chain before computing
  its own directory. Already fixed in the committed script; mentioned
  here so nobody "fixes" it back if the symptom resurfaces elsewhere.
- **Don't put `Restart=on-failure` on a sim/test `guardian-stack@.service`
  unit.** Doing that during testing meant every manual `pkill`/cleanup
  attempt got silently undone by systemd relaunching Gazebo behind our
  back — repeated enough times over one evening to exhaust both RAM and
  swap (`free -h` showed 37 of 39 GB of swap in use), which then caused
  real, confusing symptoms (goal delivery lag, sluggish everything) that
  had nothing to do with any actual code bug. The committed
  `60_scripts/systemd/guardian-stack@.service` (the real-hardware one)
  correctly has no `Restart=` directive for this exact reason — leave it
  that way. If you're improvising a local test unit by hand, don't add
  auto-restart to it either.
- **After a long testing session, check `free -h` before trusting any
  "it's not working" symptom.** Swapped-out memory makes IPC/DDS
  discovery slow and inconsistent in ways that look exactly like
  application bugs. `pkill`/`kill_ros.sh` cleanup between test rounds
  matters more than it seems.
