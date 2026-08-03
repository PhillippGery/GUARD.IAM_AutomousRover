#!/usr/bin/env python3
"""
GUARD.IAM — Stage 2 server
==========================
Stage 1 proved head pose arrives. Stage 2 turns it into servo motion:

    Quest --WiFi/WebSocket--> this server --USB serial--> ESP32-S3 --> 2 servos

Pipeline per head-pose message:
    quaternion -> yaw/pitch (deg)
      -> EMA smoothing (kills the jitter you saw in Stage 1)
      -> deadband (stops servo hunting when you hold still)
      -> map to servo angle (center 90)
      -> clamp to servo limits
      -> send "pan,tilt\n" over serial (rate-limited)

Run:
    python server_stage2.py --port COM5
    (find your ESP32's COM port in Device Manager; on Windows it's COMx)

Run WITHOUT a servo board to just watch the mapped angles:
    python server_stage2.py --no-serial

Dependencies:
    pip install websockets pyserial cryptography
"""

import argparse
import asyncio
import json
import math
import os
import socket
import ssl as ssl_mod
import subprocess
import sys
import threading
import time
import http.server

try:
    import websockets
except ImportError:
    print("Missing dependency. Run:  pip install websockets")
    sys.exit(1)

HERE = os.path.dirname(os.path.abspath(__file__))
HTTPS_PORT = 8443
WS_PORT = 8765
CERT = os.path.join(HERE, "cert.pem")
KEY = os.path.join(HERE, "key.pem")

# ---------- tuning knobs ----------
# Low-pass filter time constant in milliseconds. Higher = smoother but laggier.
# This is frame-rate independent: alpha is computed from this and the actual
# time between samples, so it behaves the same regardless of Quest frame rate.
# 20 ms = light smoothing (teammate's recommendation). Try 30-50 for smoother.
FILTER_TAU_MS = 20.0

DEADBAND_DEG = 1.0    # ignore servo moves smaller than this
SEND_HZ     = 50      # max serial commands per second

# Axis handling.
SWAP_AXES   = True    # True: yaw->tilt swap fixed (turning head pans camera)
INVERT_PAN  = True    # flip if the camera pans the wrong way (left/right reversed)
INVERT_TILT = True    # flip if the camera tilts the wrong way (up/down reversed)

# Gain: camera degrees per head degree. 1.0 = 1:1 (90 deg head -> 90 deg camera).
# Raise above 1.0 to amplify (small head move -> big camera move).
PAN_GAIN    = 1.0
TILT_GAIN   = 1.0

# Home position: where the camera holds while DISARMED (before you press the
# controller button to start tracking). Change these to set the data-collection
# starting frame. Currently center; edit freely.
HOME_PAN    = 180
HOME_TILT   = 56

# Servo travel limits (must match firmware).
PAN_MIN, PAN_MAX   = 0, 180
TILT_MIN, TILT_MAX = 20, 160
PAN_CENTER, TILT_CENTER = 90, 90


# ---------- coordinate conversion ----------
def quat_to_yaw_pitch(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qy + qx * qz)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qx * qx)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    sinp = 2.0 * (qw * qx - qy * qz)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    return math.degrees(yaw), math.degrees(pitch)


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


# ---------- serial link to ESP32 ----------
class ServoLink:
    def __init__(self, port, no_serial=False):
        self.no_serial = no_serial
        self.ser = None
        self._last_send = 0.0
        self._last_pan = None
        self._last_tilt = None
        if no_serial:
            print("[serial] --no-serial: printing angles instead of sending")
            return
        try:
            import serial  # pyserial
        except ImportError:
            print("Missing dependency. Run:  pip install pyserial")
            sys.exit(1)
        try:
            self.ser = serial.Serial(port, 115200, timeout=0.1)
            time.sleep(2.0)  # ESP32-S3 resets on serial open; let it boot
            print(f"[serial] connected on {port}")
        except Exception as e:
            print(f"[serial] could not open {port}: {e}")
            print("[serial] falling back to print-only mode")
            self.no_serial = True

    def send(self, pan, tilt):
        # Rate limit.
        now = time.time()
        if now - self._last_send < 1.0 / SEND_HZ:
            return
        # Deadband: skip tiny changes to stop servo hunting.
        if (self._last_pan is not None
                and abs(pan - self._last_pan) < DEADBAND_DEG
                and abs(tilt - self._last_tilt) < DEADBAND_DEG):
            return
        self._last_send = now
        self._last_pan, self._last_tilt = pan, tilt

        line = f"{int(round(pan))},{int(round(tilt))}\n"
        if self.no_serial:
            print(f"[cmd]   pan {pan:6.1f}  tilt {tilt:6.1f}")
        else:
            try:
                self.ser.write(line.encode())
            except Exception as e:
                print(f"[serial] write failed: {e}")


# ---------- pose -> servo state ----------
class Mapper:
    def __init__(self, link):
        self.link = link
        self.filt_yaw = None
        self.filt_pitch = None
        self.last_t = None
        self.armed = False          # start DISARMED: hold home, ignore head
        self._home_sent = False

    def set_armed(self, armed):
        if armed and not self.armed:
            print("[arm] ARMED — head tracking active")
            # reset filter so it doesn't jump from a stale value
            self.filt_yaw = None
            self.filt_pitch = None
        elif not armed and self.armed:
            print("[arm] DISARMED — holding home position")
            self._home_sent = False
        self.armed = armed

    def send_home(self):
        # Hold the fixed home angle while disarmed. Send once (deadband will
        # suppress repeats anyway, but this avoids spamming).
        if not self._home_sent:
            self.link.send(HOME_PAN, HOME_TILT)
            self._home_sent = True

    def update(self, yaw, pitch):
        # While disarmed, ignore head pose and hold home.
        if not self.armed:
            self.send_home()
            return

        # Time-constant low-pass filter. alpha derived from elapsed time so the
        # smoothing is the same regardless of how fast samples arrive.
        now = time.time()
        if self.filt_yaw is None:
            self.filt_yaw, self.filt_pitch = yaw, pitch
            self.last_t = now
        else:
            dt_ms = max(1.0, (now - self.last_t) * 1000.0)
            self.last_t = now
            # alpha = dt / (tau + dt): standard first-order LPF discretization
            alpha = dt_ms / (FILTER_TAU_MS + dt_ms)
            self.filt_yaw   = alpha * yaw   + (1 - alpha) * self.filt_yaw
            self.filt_pitch = alpha * pitch + (1 - alpha) * self.filt_pitch

        y = self.filt_yaw
        p = self.filt_pitch

        # Swap axes if head yaw is driving the wrong servo.
        if SWAP_AXES:
            y, p = p, y

        # Apply gain (camera degrees per head degree).
        y *= PAN_GAIN
        p *= TILT_GAIN

        if INVERT_PAN:
            y = -y
        if INVERT_TILT:
            p = -p

        pan  = clamp(PAN_CENTER  + y, PAN_MIN,  PAN_MAX)
        tilt = clamp(TILT_CENTER + p, TILT_MIN, TILT_MAX)
        self.link.send(pan, tilt)


# ---------- WebSocket handler ----------
def make_handler(mapper):
    async def handle_client(websocket):
        print(f"\n[ws] Quest connected: {websocket.remote_address}")
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                except json.JSONDecodeError:
                    continue
                if "_status" in data:
                    extra = {k: v for k, v in data.items() if k != "_status"}
                    print(f"[page]  {data['_status']}   {extra if extra else ''}", flush=True)
                    continue
                if "_arm" in data:
                    mapper.set_armed(bool(data["_arm"]))
                    continue
                head = data.get("head_pose", {})
                yaw, pitch = quat_to_yaw_pitch(
                    head.get("qx", 0.0), head.get("qy", 0.0),
                    head.get("qz", 0.0), head.get("qw", 1.0))
                mapper.update(yaw, pitch)
        except websockets.ConnectionClosed:
            print("[ws] Quest disconnected")
    return handle_client


async def ws_main(handler):
    async with websockets.serve(handler, "0.0.0.0", WS_PORT):
        print(f"[ws] WebSocket listening on ws://0.0.0.0:{WS_PORT}")
        await asyncio.Future()


# ---------- HTTPS static server ----------
def serve_https():
    os.chdir(HERE)
    httpd = http.server.HTTPServer(
        ("0.0.0.0", HTTPS_PORT), http.server.SimpleHTTPRequestHandler)
    ctx = ssl_mod.SSLContext(ssl_mod.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(certfile=CERT, keyfile=KEY)
    httpd.socket = ctx.wrap_socket(httpd.socket, server_side=True)
    httpd.serve_forever()


def get_lan_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = "127.0.0.1"
    finally:
        s.close()
    return ip


def ensure_cert():
    if os.path.exists(CERT) and os.path.exists(KEY):
        return
    print("[tls] Generating self-signed certificate...")
    try:
        _make_cert_python()
        print("[tls] Certificate created.")
        return
    except ImportError:
        pass
    subprocess.run([
        "openssl", "req", "-x509", "-newkey", "rsa:2048",
        "-keyout", KEY, "-out", CERT, "-days", "365", "-nodes",
        "-subj", "/CN=guardiam-test",
    ], check=True)


def _make_cert_python():
    from datetime import datetime, timedelta, timezone
    from cryptography import x509
    from cryptography.x509.oid import NameOID
    from cryptography.hazmat.primitives import hashes, serialization
    from cryptography.hazmat.primitives.asymmetric import rsa
    key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
    name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, "guardiam-test")])
    now = datetime.now(timezone.utc)
    cert = (x509.CertificateBuilder().subject_name(name).issuer_name(name)
            .public_key(key.public_key()).serial_number(x509.random_serial_number())
            .not_valid_before(now - timedelta(days=1))
            .not_valid_after(now + timedelta(days=365))
            .sign(key, hashes.SHA256()))
    with open(KEY, "wb") as f:
        f.write(key.private_bytes(serialization.Encoding.PEM,
                serialization.PrivateFormat.TraditionalOpenSSL,
                serialization.NoEncryption()))
    with open(CERT, "wb") as f:
        f.write(cert.public_bytes(serialization.Encoding.PEM))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None, help="ESP32 serial port, e.g. COM5")
    ap.add_argument("--no-serial", action="store_true",
                    help="print mapped angles instead of sending to a board")
    args = ap.parse_args()

    if args.port is None and not args.no_serial:
        print("No --port given. Running in --no-serial mode (angles print only).")
        print("When your ESP32 is connected, re-run with e.g.:  python server_stage2.py --port COM5")
        args.no_serial = True

    ensure_cert()
    link = ServoLink(args.port, no_serial=args.no_serial)
    mapper = Mapper(link)
    handler = make_handler(mapper)

    ip = get_lan_ip()
    print("=" * 60)
    print("GUARD.IAM Stage 2 — head pose -> servos")
    print("=" * 60)
    print(f"On the Quest browser, open:   https://{ip}:{HTTPS_PORT}")
    print(f"WebSocket receiving on:       ws://{ip}:{WS_PORT}")
    print("=" * 60)

    threading.Thread(target=serve_https, daemon=True).start()
    try:
        asyncio.run(ws_main(handler))
    except KeyboardInterrupt:
        print("\nBye.")


if __name__ == "__main__":
    main()
