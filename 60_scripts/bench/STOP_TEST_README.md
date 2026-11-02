# Drive stop tests

Goal: measure how the two drive controllers stop, and get clean traces to fit the simulator.

| Run | Launch alias | Node | Label |
|-----|--------------|------|-------|
| baseline | `guardiam_hw` | `phidget_bridge_node` (MotorVelocityController) | `bridge` |
| test | `guardiam_hw_bldc` | `phidget_bldc_node` (BLDCMotor, custom PID, braking on) | `bldc` |

Run both on the same floor, the same day, with the same battery level and robot setup. Do not change any code or params between the two.

## 0. Get the code and build

Use the `feature/bldc-stop-test-clean` branch of the `guardiam3` remote (not `origin`, which is Phillipp's; and not `feature/bldc-stop-test`, which has Jonathan's bridge rewrite merged in):

```
cd ~/GUARD.IAM_AutomousRover
git fetch guardiam3
git checkout -b feature/bldc-stop-test-clean guardiam3/feature/bldc-stop-test-clean   # or just `git checkout feature/bldc-stop-test-clean` if it exists
cb
```

This branch's two drive nodes both publish `/wheel_rpm_measured` (needed by the logger), and the BLDC node has the measured-RPM filter fix (`ema_alpha`, see "Known issue" below).

## Setup and safety

- Hard flat floor, 3 m clear ahead and to the right of a start mark. Tape a line under the robot's front-left corner and run a tape measure along the direction of travel.
- Arms stowed in the same pose every time. Note battery voltage.
- A spotter near the power switch. The robot moves up to 0.6 m/s for 1.5 s per trial.
- To stop: Ctrl+C in the test terminal (sends zero), `killros`, or the power switch.

## 1. First, with the wheels off the ground

The BLDC node's filter bug is fixed, which makes the feedback about 3x stronger than the gains were tuned for. Before driving on the floor, lift the robot, run `guardiam_hw_bldc`, and command a slow forward move (e.g. keyboard or `ros2 topic pub /cmd_vel ...` at 0.2 m/s). All four wheels should settle smoothly near the target with no buzzing or hunting. If they oscillate, stop and lower `kp_custom` in `robot_params.yaml` (try 0.006), rebuild, and retry.

## 2. Calibrate the odometry scale (5 min)

```
guardiam_hw_bldc                       # terminal A
python3 60_scripts/bench/stop_test.py --label bldc_cal --calibrate   # terminal B
```

Three drives at 0.2 m/s. After each, measure total distance from the start mark to the same corner of the robot and type it in. It prints `tape / odom`. If it isn't about 1.0, multiply `position_rescale_factor` (bldc) or `rescale_factor` (bridge) by that number, rebuild, and run it again. Do this before the real runs, because everything else depends on the odom scale.

## 3. Main runs

Run each of these twice, once per controller (labels `bldc` and `bridge`), restarting the launch in between and never running both drive nodes at once:

```
python3 60_scripts/bench/stop_test.py --label bldc   --axes fwd,strafe --speeds 0.4 --repeats 5
python3 60_scripts/bench/stop_test.py --label bridge --axes fwd,strafe --speeds 0.4 --repeats 5
```

Add `--speeds 0.2,0.4,0.6 --axes fwd,strafe,rot --repeats 3` for the full set later.

For every trial: put the robot on the mark, press Enter, then measure.
- fwd / strafe: total distance (m) from the mark to the same robot corner.
- rot: total rotation in degrees (mark a line along the robot's side first).

The script asks for battery voltage and notes at the start, and records the commit hash.

## 4. Look at the results

```
python3 60_scripts/bench/analyze_stop_tests.py ~/stop_tests/bldc_2026* ~/stop_tests/bridge_2026*
```

It prints stop distance, time until still, tape-vs-odom scale, and how close the achieved speed gets to the commanded speed, per controller, axis and speed.

## Known issue found in the first dataset

The first BLDC run (before this branch) read tape/odom = 2.9 on forward trials, nearly constant across speeds. The cause was the BLDC node's velocity filter: `0.1*raw + 0.7*prev` has weights that sum to 0.8, so measured RPM settled at 1/3 of the true speed. Odom under-reported by 3x and the PID pushed wheels past their target. The filter is now `a*raw + (1-a)*prev` with `ema_alpha: 0.3`. The earlier BLDC data is not valid for judging the controller, so redo the BLDC run.
