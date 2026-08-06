#!/usr/bin/env python3
"""
GUARD.IAM - Bimanual teleop CLIENT
Runs on the Ubuntu laptop, next to the SO-101 LEADER arms.

Reads the BiSOLeader arms locally and pushes actions over ZMQ (WiFi) to
bimanual_teleop_host.py running on the MiniPC (follower side).
Optionally displays the host's camera preview so you can see the
follower without standing next to it.

Requires (in the conda env, on the laptop):
    pip install 'lerobot[feetech,hardware]' pyzmq

Run (on the laptop):
    conda activate guardiam_collect
    python3 bimanual_teleop_client.py \
        --left-port /dev/guardian_left_leader \
        --right-port /dev/guardian_right_leader \
        --host-ip <MINIPC_ZEROTIER_IP> \
        --show-video

Per-arm calibration ids default to guardian_leader_left / guardian_leader_right
(the .json files in ~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/).
Override with --left-id / --right-id if your calibration files are named differently.
"""

import argparse
import json
import signal
import sys
import time

import cv2
import numpy as np
import zmq

from lerobot.teleoperators.bi_so_leader import BiSOLeader, BiSOLeaderConfig
from lerobot.teleoperators.so_leader import SO101LeaderConfig

CMD_PORT = 5555
VIDEO_PORT = 5556
LOOP_HZ = 30


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--left-port", required=True, help="Serial port for the LEFT leader arm")
    parser.add_argument("--right-port", required=True, help="Serial port for the RIGHT leader arm")
    parser.add_argument("--id", default="guardian_leader", help="Base bimanual teleop id")
    parser.add_argument("--left-id", default="guardian_leader_left",
                        help="LEFT leader calibration id (matches the .json filename, no extension)")
    parser.add_argument("--right-id", default="guardian_leader_right",
                        help="RIGHT leader calibration id (matches the .json filename, no extension)")
    parser.add_argument("--host-ip", required=True, help="ZeroTier IP of the MiniPC (follower side)")
    parser.add_argument("--show-video", action="store_true", help="Display the host's camera preview")
    args = parser.parse_args()

    # Per-arm configs are SO101LeaderConfig OBJECTS (not dicts), each carrying the
    # calibration id that matches its .json file. This mirrors config.py:make_leader().
    teleop_config = BiSOLeaderConfig(
        left_arm_config=SO101LeaderConfig(port=args.left_port, id=args.left_id),
        right_arm_config=SO101LeaderConfig(port=args.right_port, id=args.right_id),
        id=args.id,
    )
    teleop = BiSOLeader(teleop_config)

    print(f"[CLIENT] Connecting to leader arms ({args.left_port}, {args.right_port})...")
    teleop.connect()
    print("[CLIENT] Leader arms connected.")

    ctx = zmq.Context()

    cmd_sock = ctx.socket(zmq.PUSH)
    cmd_sock.setsockopt(zmq.CONFLATE, 1)
    cmd_sock.connect(f"tcp://{args.host_ip}:{CMD_PORT}")

    video_sock = None
    if args.show_video:
        video_sock = ctx.socket(zmq.SUB)
        video_sock.setsockopt(zmq.CONFLATE, 1)
        video_sock.setsockopt_string(zmq.SUBSCRIBE, "")
        video_sock.connect(f"tcp://{args.host_ip}:{VIDEO_PORT}")

    print(f"[CLIENT] Sending actions to tcp://{args.host_ip}:{CMD_PORT}")
    if video_sock is not None:
        print(f"[CLIENT] Reading preview from tcp://{args.host_ip}:{VIDEO_PORT}")

    def shutdown(*_):
        print("\n[CLIENT] Shutting down...")
        teleop.disconnect()
        cmd_sock.close(0)
        if video_sock is not None:
            video_sock.close(0)
            cv2.destroyAllWindows()
        ctx.term()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)

    period = 1.0 / LOOP_HZ

    while True:
        loop_start = time.monotonic()

        action = teleop.get_action()
        payload = json.dumps(action).encode("utf-8")
        try:
            cmd_sock.send(payload, flags=zmq.NOBLOCK)
        except zmq.Again:
            pass  # host isn't keeping up -- drop this frame, CONFLATE handles the rest

        if video_sock is not None:
            try:
                jpg_bytes = video_sock.recv(flags=zmq.NOBLOCK)
                frame = cv2.imdecode(np.frombuffer(jpg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    cv2.imshow("Follower preview", frame)
                    cv2.waitKey(1)
            except zmq.Again:
                pass

        elapsed = time.monotonic() - loop_start
        time.sleep(max(0.0, period - elapsed))


if __name__ == "__main__":
    main()
