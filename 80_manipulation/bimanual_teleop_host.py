#!/usr/bin/env python3
"""
GUARD.IAM - Bimanual teleop HOST
Runs on the AMD Ryzen AI MiniPC, next to the SO-101 FOLLOWER arms.

Listens for leader-arm actions sent over ZMQ (WiFi) by bimanual_teleop_client.py
(running on the laptop) and drives the BiSOFollower arms.
Optionally streams a low-res JPEG preview back to the laptop.

Requires (in the conda env, on the MiniPC):
    pip install 'lerobot[feetech,hardware]' pyzmq

Run (on the MiniPC) -- START THIS FIRST, before the client:
    conda activate guardiam_collect
    python3 bimanual_teleop_host.py \
        --left-port /dev/guardian_left_follower \
        --right-port /dev/guardian_right_follower \
        --stream-video

Per-arm calibration ids default to the follower calibration used for collection
and inference. Override with --left-id / --right-id if your follower calibration
files are named differently.
"""

import argparse
import json
import signal
import sys
import time

import cv2
import zmq

from lerobot.robots.bi_so_follower import BiSOFollower, BiSOFollowerConfig
from lerobot.robots.so_follower import SO101FollowerConfig

CMD_PORT = 5555
VIDEO_PORT = 5556
COMMAND_TIMEOUT_S = 0.5  # if no fresh action arrives within this window, hold position
LOOP_HZ = 30


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--left-port", required=True, help="Serial port for the LEFT follower arm")
    parser.add_argument("--right-port", required=True, help="Serial port for the RIGHT follower arm")
    parser.add_argument("--id", default="guardian_follower", help="Base bimanual robot id")
    parser.add_argument("--left-id", default="guardian_follower_left",
                        help="LEFT follower calibration id (matches the .json filename, no extension)")
    parser.add_argument("--right-id", default="guardian_follower_right",
                        help="RIGHT follower calibration id (matches the .json filename, no extension)")
    parser.add_argument("--stream-video", action="store_true", help="Also stream a camera preview back to the client")
    parser.add_argument("--camera-index", type=int, default=0, help="OpenCV index of the camera to preview")
    args = parser.parse_args()

    # Per-arm configs are SO101FollowerConfig OBJECTS (not dicts), each carrying the
    # calibration id that matches its .json file. Mirrors config.py:make_follower().
    robot_config = BiSOFollowerConfig(
        left_arm_config=SO101FollowerConfig(port=args.left_port, id=args.left_id),
        right_arm_config=SO101FollowerConfig(port=args.right_port, id=args.right_id),
        id=args.id,
    )
    robot = BiSOFollower(robot_config)

    print(f"[HOST] Connecting to follower arms ({args.left_port}, {args.right_port})...")
    robot.connect()
    print("[HOST] Follower arms connected.")

    preview_cam = None
    if args.stream_video:
        preview_cam = cv2.VideoCapture(args.camera_index)
        if not preview_cam.isOpened():
            print(f"[HOST] WARNING: could not open camera index {args.camera_index}, disabling video stream")
            preview_cam = None

    ctx = zmq.Context()

    cmd_sock = ctx.socket(zmq.PULL)
    cmd_sock.setsockopt(zmq.CONFLATE, 1)  # only ever keep the newest action queued
    cmd_sock.bind(f"tcp://*:{CMD_PORT}")

    video_sock = None
    if preview_cam is not None:
        video_sock = ctx.socket(zmq.PUB)
        video_sock.setsockopt(zmq.CONFLATE, 1)
        video_sock.bind(f"tcp://*:{VIDEO_PORT}")

    poller = zmq.Poller()
    poller.register(cmd_sock, zmq.POLLIN)

    print(f"[HOST] Listening for actions on tcp://*:{CMD_PORT}")
    if video_sock is not None:
        print(f"[HOST] Streaming preview on tcp://*:{VIDEO_PORT}")

    def shutdown(*_):
        print("\n[HOST] Shutting down...")
        robot.disconnect()
        if preview_cam is not None:
            preview_cam.release()
        cmd_sock.close(0)
        if video_sock is not None:
            video_sock.close(0)
        ctx.term()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)

    last_action = None
    last_cmd_time = time.monotonic()
    period = 1.0 / LOOP_HZ

    while True:
        loop_start = time.monotonic()

        socks = dict(poller.poll(timeout=0))
        if cmd_sock in socks:
            while True:
                try:
                    raw = cmd_sock.recv(flags=zmq.NOBLOCK)
                    last_action = json.loads(raw.decode("utf-8"))
                    last_cmd_time = time.monotonic()
                except zmq.Again:
                    break

        stale = (time.monotonic() - last_cmd_time) > COMMAND_TIMEOUT_S
        if last_action is not None and not stale:
            try:
                robot.send_action(last_action)
            except Exception as e:
                print(f"[HOST] send_action failed: {e}")
        # if stale: connection likely dropped -- hold last physical position,
        # don't keep re-sending a command that may no longer be intentional

        if video_sock is not None:
            ok, frame = preview_cam.read()
            if ok:
                small = cv2.resize(frame, (320, 240))
                _, jpg = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 60])
                try:
                    video_sock.send(jpg.tobytes(), flags=zmq.NOBLOCK)
                except zmq.Again:
                    pass

        elapsed = time.monotonic() - loop_start
        time.sleep(max(0.0, period - elapsed))


if __name__ == "__main__":
    main()
