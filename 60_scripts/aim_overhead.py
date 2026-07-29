#!/usr/bin/env python3
"""
GUARD.IAM - overhead cam aim-and-hold (headless-safe)
=====================================================
Live preview + keyboard pan/tilt WITHOUT cv2.imshow, so it runs on the
opencv-python-headless build in the lerobot env (no GUI libs needed).

The camera frames are served as MJPEG on http://localhost:8088 (open it in a
browser tab to watch). Aiming is done from the TERMINAL: this script reads
single keypresses and sends pan/tilt over serial to the ESP32, which holds the
last angle, so the view stays constant for data collection.

Run (inside the lerobot env):
    conda activate lerobot
    python aim_overhead.py
    # then open http://localhost:8088 in a browser, and keep THIS terminal
    # focused to drive the gimbal.

Controls (terminal focused):
    a/d pan   w/s tilt   [ / ] step   space=print angle   h=home   q=quit

Deps:  pip install pyserial   (opencv already in the env; no GUI build required)
"""
import argparse, os, re, sys, time, threading, termios, tty, select
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    import cv2
except ImportError:
    print("Missing opencv. Activate lerobot: conda activate lerobot"); sys.exit(1)
try:
    import serial
except ImportError:
    print("Missing pyserial. Inside the env: pip install pyserial"); sys.exit(1)

PAN_MIN, PAN_MAX = 0, 180
TILT_MIN, TILT_MAX = 20, 160
FRAME_W, FRAME_H = 640, 480
HTTP_PORT = 8088


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def open_cam(path):
    # V4L2 can't open by path string ("can't be used to capture by name"); it
    # needs an integer index. Resolve the udev symlink to /dev/videoN, take N.
    real = os.path.realpath(path)
    m = re.search(r"(\d+)$", real)
    if not m:
        print(f"Couldn't parse a video index from {path} -> {real}"); sys.exit(1)
    index = int(m.group(1))
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"Could not open camera index {index} ({path} -> {real}).")
        print(f"Usually it's busy (a browser tab). Check:  sudo fuser -v {real}")
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    ok, _ = cap.read()
    if not ok:
        print(f"Opened index {index} but got no frame; busy or wrong node.")
        cap.release(); sys.exit(1)
    return cap


# Shared latest-JPEG buffer, written by the capture loop, read by HTTP clients.
class FrameHub:
    def __init__(self):
        self.jpeg = None
        self.lock = threading.Lock()

    def set(self, jpeg):
        with self.lock:
            self.jpeg = jpeg

    def get(self):
        with self.lock:
            return self.jpeg


def make_handler(hub):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):  # silence per-request logging
            pass

        def do_GET(self):
            if self.path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.end_headers()
                self.wfile.write(
                    b"<html><body style='margin:0;background:#111'>"
                    b"<img src='/stream' style='width:100vw'></body></html>")
                return
            if self.path == "/stream":
                self.send_response(200)
                self.send_header(
                    "Content-Type",
                    "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    while True:
                        jpeg = hub.get()
                        if jpeg is not None:
                            self.wfile.write(b"--frame\r\n")
                            self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                            self.wfile.write(jpeg)
                            self.wfile.write(b"\r\n")
                        time.sleep(0.04)  # ~25 fps cap
                except (BrokenPipeError, ConnectionResetError):
                    return
            else:
                self.send_response(404); self.end_headers()
    return Handler


class KeyReader:
    """Non-blocking single-key reads from the terminal (raw mode)."""
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def get(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

    def __exit__(self, *a):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--serial", default="/dev/ttyACM0")
    ap.add_argument("--cam", default="/dev/cam_overhead")
    ap.add_argument("--pan", type=int, default=96)
    ap.add_argument("--tilt", type=int, default=90)
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.serial, 115200, timeout=0.1)
    except Exception as e:
        print(f"Could not open serial {args.serial}: {e}")
        print("Find it with:  ls /dev/ttyACM* /dev/ttyUSB*"); sys.exit(1)
    time.sleep(2.0)  # ESP32-C3 resets on serial open; let it boot

    cap = open_cam(args.cam)
    hub = FrameHub()

    server = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), make_handler(hub))
    threading.Thread(target=server.serve_forever, daemon=True).start()

    pan = clamp(args.pan, PAN_MIN, PAN_MAX)
    tilt = clamp(args.tilt, TILT_MIN, TILT_MAX)
    step = 2

    def send(p, t):
        ser.write(f"{int(p)},{int(t)}\n".encode())

    send(pan, tilt)
    print(f"Preview:  http://localhost:{HTTP_PORT}   (open in a browser tab)")
    print("Keep THIS terminal focused to aim.")
    print("a/d pan, w/s tilt, [ ] step, space=print, h=home, q=quit")

    with KeyReader() as keys:
        while True:
            ok, frame = cap.read()
            if ok:
                cv2.putText(frame, f"pan {pan}  tilt {tilt}  step {step}",
                            (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 255, 120), 2, cv2.LINE_AA)
                enc_ok, buf = cv2.imencode(".jpg", frame,
                                           [cv2.IMWRITE_JPEG_QUALITY, 70])
                if enc_ok:
                    hub.set(buf.tobytes())

            k = keys.get()
            if k is None:
                time.sleep(0.005)
                continue

            moved = True
            if k in ("q", "\x1b"):
                break
            elif k == "a":
                pan = clamp(pan - step, PAN_MIN, PAN_MAX)
            elif k == "d":
                pan = clamp(pan + step, PAN_MIN, PAN_MAX)
            elif k == "w":
                tilt = clamp(tilt + step, TILT_MIN, TILT_MAX)
            elif k == "s":
                tilt = clamp(tilt - step, TILT_MIN, TILT_MAX)
            elif k == "[":
                step = max(1, step - 1); moved = False
            elif k == "]":
                step = min(20, step + 1); moved = False
            elif k == "h":
                pan, tilt = 0, 90
            elif k == " ":
                print(f"LOCK THIS -> pan={pan}  tilt={tilt}"); moved = False
            else:
                moved = False
            if moved:
                send(pan, tilt)

    print(f"\nExiting. Camera holding pan={pan}, tilt={tilt}.")
    print("Use these as HOME_PAN / HOME_TILT for data collection.")
    cap.release(); ser.close(); server.shutdown()


if __name__ == "__main__":
    main()