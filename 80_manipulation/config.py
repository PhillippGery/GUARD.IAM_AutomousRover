"""
Shared config for GUARDIAN bimanual arms — imported by collect.py, teleop_test.py
and infer.py so the arm and camera setup can NEVER drift between collection and
inference.

THREE cameras: two wrist + one fixed overhead (on the pan/tilt gimbal). The
overhead is mounted UPSIDE DOWN by design, so it's rotated 180 in software here
(baked into the recorded observation, so collect and infer stay consistent).
12-dim action, 12-dim state. Cameras don't change those dims.
"""

from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
# Cv2Rotation lives alongside the camera configs. If this import path differs in
# your fork, grep for "class Cv2Rotation" and adjust; some versions also accept a
# plain int (0/90/180/270) for `rotation`.
from lerobot.cameras.configs import Cv2Rotation
from lerobot.robots.so_follower import SO101FollowerConfig
from lerobot.robots.bi_so_follower import BiSOFollower, BiSOFollowerConfig
from lerobot.teleoperators.so_leader import SO101LeaderConfig
from lerobot.teleoperators.bi_so_leader import BiSOLeader, BiSOLeaderConfig

FPS = 20

# ── Capture resolution ─────────────────────────────────────────────────────────
# All three cameras share ONE usb hub (3-4.x), so they share its bandwidth AND
# power. Three 640x480 RAW streams overrun USB2 and one cam (left_wrist) starves
# -> TimeoutError. To CONFIRM that's the cause, drop to 424,240 here and re-run;
# if it works, it was bandwidth. Full-res fixes (pick one):
#   - MJPEG: check `v4l2-ctl -d /dev/cam_left_wrist --list-formats-ext` for MJPG;
#     compressed streams cut bandwidth ~10x. (Needs the camera opened in MJPG.)
#   - Split the load: move ONE camera (e.g. overhead) to a USB port on a DIFFERENT
#     controller, then re-derive its port path and update 99-guardian.rules.
#   - Use a powered hub if it's a power brownout rather than pure bandwidth.
CAM_W, CAM_H = 640, 480

# ── Arm serial ports ─────────────────────────────────────────────────────────
PORT_LEFT_FOLLOWER  = "/dev/guardian_left_follower"
PORT_RIGHT_FOLLOWER = "/dev/guardian_right_follower"
PORT_LEFT_LEADER    = "/dev/guardian_left_leader"
PORT_RIGHT_LEADER   = "/dev/guardian_right_leader"

# ── Calibration ids ──────────────────────────────────────────────────────────
ID_LEFT_FOLLOWER  = "left_arm_follower"
ID_RIGHT_FOLLOWER = "right_follower"
ID_LEFT_LEADER    = "left_arm_leader"
ID_RIGHT_LEADER   = "right_leader"

# ── Cameras (stable udev symlinks, never raw /dev/videoN) ──────────────────────
# Set the overhead angle ONCE before a session (aim_overhead.py, then quit it so
# the ESP32 holds the angle and the camera is freed). Don't leave that script
# running during collection — it holds the camera open.
CAM_LEFT_WRIST  = "/dev/cam_left_wrist"
CAM_RIGHT_WRIST = "/dev/cam_right_wrist"
CAM_OVERHEAD    = "/dev/cam_overhead"

_cameras = {
    "left_wrist":  OpenCVCameraConfig(index_or_path=CAM_LEFT_WRIST,  width=CAM_W, height=CAM_H, fps=FPS,
                                      fourcc="MJPG"),
    "right_wrist": OpenCVCameraConfig(index_or_path=CAM_RIGHT_WRIST, width=CAM_W, height=CAM_H, fps=FPS,
                                      fourcc="MJPG"),
    "overhead":    OpenCVCameraConfig(index_or_path=CAM_OVERHEAD,    width=CAM_W, height=CAM_H, fps=FPS,
                                      rotation=Cv2Rotation.ROTATE_180, fourcc="MJPG"),
}


def make_follower():
    """Bimanual follower. All cameras attach here (all streams recorded regardless
    of which arm they're near — the association is just bookkeeping)."""
    left = SO101FollowerConfig(port=PORT_LEFT_FOLLOWER, id=ID_LEFT_FOLLOWER, cameras=_cameras)
    right = SO101FollowerConfig(port=PORT_RIGHT_FOLLOWER, id=ID_RIGHT_FOLLOWER)
    return BiSOFollower(BiSOFollowerConfig(left, right, id="guardian_follower"))


def make_leader():
    """Bimanual leader (teleop only — not needed at policy inference)."""
    left = SO101LeaderConfig(port=PORT_LEFT_LEADER, id=ID_LEFT_LEADER)
    right = SO101LeaderConfig(port=PORT_RIGHT_LEADER, id=ID_RIGHT_LEADER)
    return BiSOLeader(BiSOLeaderConfig(left, right, id="guardian_leader"))