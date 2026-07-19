# guardian_navigation

**Owner: Phillipp**

High-level autonomous navigation behaviors for GUARDIAN, built on the Nav2 simple-commander API.

## Nodes

| Node | Purpose |
|------|---------|
| `demo_mission_node` | Drives an autonomous demo mission: wait for Nav2, navigate to an intervention waypoint, hold (simulating arm work), then return to base |

## Topics

Uses the Nav2 action interfaces via `BasicNavigator` (no raw topics subscribed/published directly) — internally drives the `navigate_to_pose` action served by `bt_navigator`.

## How to Launch or Run

Requires Nav2 already active (e.g. via `guardian_bringup`'s `guardian_nav.launch.py` or `guardian_sim.launch.py`), then:

```bash
ros2 launch guardian_bringup guardian_sim.launch.py
# in another terminal, once Nav2 lifecycle is started:
ros2 run guardian_navigation demo_mission_node
```

## Known Issues / Dependencies

- Waits on `nav.waitUntilNav2Active()` — will hang if Nav2's lifecycle was never started (`autostart: False` in `nav2_params.yaml`, must click "Startup" in RViz or call the lifecycle manager service manually).
- Waypoint coordinates are hardcoded (`(3.0, 0.0)` and back to origin) — intended as a demo, not a real mission planner.
- Depends on `nav2_simple_commander` and `nav2_bringup`.
