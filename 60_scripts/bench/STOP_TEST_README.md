# Drive stop tests: how to collect the data

Goal: record how far and how fast the rover stops with each of the two drive controllers, so we can compare them and fit the simulator. Two runs, same floor, same day:

| Run | Launch alias | Node | Label |
|-----|--------------|------|-------|
| 1 | `guardiam_hw` | `phidget_bridge_node` (MotorVelocityController) | `bridge` |
| 2 | `guardiam_hw_bldc` | `phidget_bldc_node` (BLDCMotor, custom PID, active braking) | `bldc` |

Each run takes about 15 minutes. Do not edit any code or parameters between runs.

## Before you start

- **Two people.** One at the keyboard, one standing by the robot (spotter) with a hand near the power switch.
- **Floor.** Hard, flat, same spot for both runs. Clear 3 m ahead and 3 m to the right of the start mark. Nobody walks into that area during a trial.
- **Start mark.** Put a tape line under the front-left corner of the robot, plus a tape measure running along the direction of travel (forward axis, then strafe axis).
- **Arms stowed.** Same pose for both runs, and don't change the payload.
- **Battery.** Charged to about the same level for both runs (note the voltage at the start of each).
- **Latest code.** On the MiniPC:
  ```
  cd ~/GUARD.IAM_AutomousRover
  git pull          # or check out the commit "Add phidget_bldc_node to guardian_drive..."
  cb                # colcon build, then source
  ```

## Safety

- The rover drives up to 0.6 m/s forward or sideways and rotates at 0.6 rad/s, for 1.5 s per trial, then stops. It will move.
- To stop immediately: press **Ctrl+C** in the script terminal (it sends zero velocity), run `killros`, or hit the power switch.
- If a wheel doesn't move, or the robot drifts or lurches on its own, stop and write it down. Do not keep running. The FR wheel has had connection faults before (PhidgetException 0x42).

## Run 1: bridge (current default)

Terminal A:
```
guardiam_hw
```
Wait until the log says all four wheels are driving (no "NOT FOUND" errors).

Terminal B:
```
cd ~/GUARD.IAM_AutomousRover
python3 60_scripts/bench/stop_test.py --label bridge --measure
```

The script tells you each trial ("fwd @ 0.2 rep 1/3", ...) and waits for Enter:
1. Move the robot back so the front-left corner is on the start mark.
2. Make sure the spotter and floor are clear.
3. Press Enter. The robot drives 1.5 s and stops.
4. Measure how far the same corner is past the mark (tape measure, in metres, e.g. `0.12`) and type it in. For rotation trials, skip with Enter. For strafe trials, measure along the strafe direction.
5. Repeat. 27 trials total (fwd, strafe, rot x 3 speeds x 3 repeats).

When it finishes, it prints `saved to ~/stop_tests/bridge_<timestamp>/`.

Shorter first pass if you're short on time (do this before the full run):
```
python3 60_scripts/bench/stop_test.py --label bridge --axes fwd --speeds 0.4 --repeats 5 --measure
```

## Run 2: bldc

1. Stop everything: Ctrl+C in Terminal A, then `killros`. Never run both drive nodes at once.
2. Terminal A:
   ```
   guardiam_hw_bldc
   ```
3. Terminal B: same command, new label:
   ```
   python3 60_scripts/bench/stop_test.py --label bldc --measure
   ```
   Use the same arguments (full run or shortened) as Run 1.

## What to send back

Zip both output folders and send them to Vedant:
```
cd ~ && zip -r stop_tests.zip stop_tests/
```
Also include a note with:
- Date, floor type, battery voltage before each run
- Anything odd: a wheel that lagged, a fault in the log, a trial you had to redo
- Which commit was running (`git log --oneline -1`)

## What's in the output

- `trials.csv`: one row per trial: odom stop distance, odom rotation, time until still, tape-measured distance.
- `ts_N.csv`: time series for trial N: command, wheel RPM x4, odom pose and velocity.

Odometry is computed from wheel encoders, so it can't see slip. That is why the tape measurements matter.
