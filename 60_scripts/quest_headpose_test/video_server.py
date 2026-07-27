#!/usr/bin/env python3
"""
GUARD.IAM — webcam video server
===============================
Captures frames from a USB webcam and streams them as JPEG over a WebSocket
so the WebXR page can paint them onto a panel inside the Quest headset.

This runs ALONGSIDE server_stage2.py:
  - server_stage2.py  -> head pose in (port 8765) + servo out (COM3)
  - video_server.py    -> webcam frames out (port 8766)

Run:
    python video_server.py --camera 1
    (try --camera 0 if 1 shows the built-in laptop cam or fails)

List what indices work:
    python video_server.py --list

Dependencies:
    pip install opencv-python websockets
"""

import argparse
import asyncio
import sys

try:
    import cv2
except ImportError:
    print("Missing dependency. Run:  pip install opencv-python")
    sys.exit(1)

try:
    import websockets
except ImportError:
    print("Missing dependency. Run:  pip install websockets")
    sys.exit(1)

VIDEO_PORT = 8766
JPEG_QUALITY = 60      # 1-100; lower = smaller/faster, blurrier
TARGET_FPS = 20        # cap send rate
FRAME_W = 640
FRAME_H = 480


def list_cameras(max_idx=5):
    print("Probing camera indices...")
    for i in range(max_idx):
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)  # DSHOW backend on Windows
        if cap.isOpened():
            ok, _ = cap.read()
            print(f"  index {i}: {'WORKS' if ok else 'opens but no frame'}")
            cap.release()
        else:
            print(f"  index {i}: not available")


class Camera:
    def __init__(self, index):
        # CAP_DSHOW avoids slow open + MSMF quirks on Windows.
        self.cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            print(f"ERROR: could not open camera index {index}.")
            print("Try a different --camera index, or run --list.")
            sys.exit(1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
        print(f"[cam] opened index {index}")

    def read_jpeg(self):
        ok, frame = self.cap.read()
        if not ok:
            return None
        ok, buf = cv2.imencode(".jpg", frame,
                               [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ok:
            return None
        return buf.tobytes()


async def handle_client(websocket, camera):
    print(f"[ws] video client connected: {websocket.remote_address}")
    interval = 1.0 / TARGET_FPS
    try:
        while True:
            jpeg = camera.read_jpeg()
            if jpeg is not None:
                await websocket.send(jpeg)   # binary frame
            await asyncio.sleep(interval)
    except websockets.ConnectionClosed:
        print("[ws] video client disconnected")


async def main_async(camera):
    async def handler(ws):
        await handle_client(ws, camera)
    async with websockets.serve(handler, "0.0.0.0", VIDEO_PORT, max_size=None):
        print(f"[ws] video streaming on ws://0.0.0.0:{VIDEO_PORT}")
        print("Leave this running alongside server_stage2.py.")
        await asyncio.Future()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=1,
                    help="camera index (try 0 or 1)")
    ap.add_argument("--list", action="store_true",
                    help="list available camera indices and exit")
    args = ap.parse_args()

    if args.list:
        list_cameras()
        return

    camera = Camera(args.camera)
    try:
        asyncio.run(main_async(camera))
    except KeyboardInterrupt:
        print("\nBye.")


if __name__ == "__main__":
    main()
