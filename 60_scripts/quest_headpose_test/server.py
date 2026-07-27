#!/usr/bin/env python3
"""
GUARD.IAM — Stage 1 head-pose test server
==========================================
No motors, no ESP32. Proves the Quest can stream head pose to this laptop.

What it does:
  1. Serves index.html to the Quest browser over HTTPS (WebXR needs HTTPS).
  2. Runs a WebSocket server on port 8765 (same port as the real bridge).
  3. Prints incoming head pose: quaternion + derived yaw/pitch in degrees.

Run:
    python3 server.py

Then on the Quest, open:  https://<this-laptop-ip>:8443
(Accept the security warning — it's your own self-signed cert.)

Dependencies:
    pip install websockets
"""

import asyncio
import json
import math
import ssl
import http.server
import ssl as ssl_mod
import threading
import socket
import subprocess
import os
import sys

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


# ---------- coordinate conversion ----------
def quat_to_yaw_pitch(qx, qy, qz, qw):
    """
    Convert a WebXR head quaternion to yaw (left/right) and pitch (up/down)
    in degrees. WebXR is Y-up, right-handed, -Z forward.

    yaw   = rotation about Y (pan)   -> drives the pan servo
    pitch = rotation about X (tilt)  -> drives the tilt servo
    We drop roll (2-DOF gimbal).
    """
    # yaw (about Y)
    siny_cosp = 2.0 * (qw * qy + qx * qz)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qx * qx)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    # pitch (about X)
    sinp = 2.0 * (qw * qx - qy * qz)
    sinp = max(-1.0, min(1.0, sinp))  # clamp for safety
    pitch = math.asin(sinp)

    return math.degrees(yaw), math.degrees(pitch)


# ---------- WebSocket handler ----------
async def handle_client(websocket):
    print(f"\n[ws] Quest connected: {websocket.remote_address}")
    _last_print = 0.0
    try:
        async for message in websocket:
            try:
                data = json.loads(message)
            except json.JSONDecodeError:
                continue

            # Diagnostic status messages from the page (not head pose).
            if "_status" in data:
                tag = data.get("_status")
                extra = {k: v for k, v in data.items() if k != "_status"}
                print(f"[page]  {tag}   {extra if extra else ''}", flush=True)
                continue

            head = data.get("head_pose", {})
            qx = head.get("qx", 0.0)
            qy = head.get("qy", 0.0)
            qz = head.get("qz", 0.0)
            qw = head.get("qw", 1.0)

            yaw, pitch = quat_to_yaw_pitch(qx, qy, qz, qw)

            # Throttle to ~5 prints/sec and print on its own line so it's
            # clearly visible in PowerShell (which can hide \r-only updates).
            now = asyncio.get_event_loop().time()
            if now - _last_print >= 0.2:
                _last_print = now
                print(f"[head]  yaw(pan): {yaw:+7.1f}    "
                      f"pitch(tilt): {pitch:+7.1f}", flush=True)
    except websockets.ConnectionClosed:
        print("[ws] Quest disconnected")


async def ws_main():
    async with websockets.serve(handle_client, "0.0.0.0", WS_PORT):
        print(f"[ws] WebSocket listening on ws://0.0.0.0:{WS_PORT}")
        await asyncio.Future()  # run forever


# ---------- HTTPS static server (serves index.html) ----------
def serve_https():
    os.chdir(HERE)
    httpd = http.server.HTTPServer(
        ("0.0.0.0", HTTPS_PORT),
        http.server.SimpleHTTPRequestHandler,
    )
    ctx = ssl_mod.SSLContext(ssl_mod.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(certfile=CERT, keyfile=KEY)
    httpd.socket = ctx.wrap_socket(httpd.socket, server_side=True)
    httpd.serve_forever()


def get_lan_ip():
    """Best-effort local IP so we can print the URL to open on the Quest."""
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
    """Generate a self-signed cert if one doesn't exist yet."""
    if os.path.exists(CERT) and os.path.exists(KEY):
        return
    print("[tls] Generating self-signed certificate...")
    try:
        _make_cert_python()
        print("[tls] Certificate created.")
        return
    except ImportError:
        pass  # cryptography not installed; fall back to openssl

    # Fallback: use openssl if the cryptography package isn't available.
    subprocess.run([
        "openssl", "req", "-x509", "-newkey", "rsa:2048",
        "-keyout", KEY, "-out", CERT,
        "-days", "365", "-nodes",
        "-subj", "/CN=guardiam-test",
    ], check=True)
    print("[tls] Certificate created (openssl).")


def _make_cert_python():
    """Generate a self-signed cert using the 'cryptography' package.
    No openssl binary required — works out of the box on Windows.
    Raises ImportError if 'cryptography' isn't installed."""
    from datetime import datetime, timedelta, timezone
    from cryptography import x509
    from cryptography.x509.oid import NameOID
    from cryptography.hazmat.primitives import hashes, serialization
    from cryptography.hazmat.primitives.asymmetric import rsa

    key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
    subject = issuer = x509.Name([
        x509.NameAttribute(NameOID.COMMON_NAME, "guardiam-test"),
    ])
    now = datetime.now(timezone.utc)
    cert = (
        x509.CertificateBuilder()
        .subject_name(subject)
        .issuer_name(issuer)
        .public_key(key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(now - timedelta(days=1))
        .not_valid_after(now + timedelta(days=365))
        .sign(key, hashes.SHA256())
    )
    with open(KEY, "wb") as f:
        f.write(key.private_bytes(
            encoding=serialization.Encoding.PEM,
            format=serialization.PrivateFormat.TraditionalOpenSSL,
            encryption_algorithm=serialization.NoEncryption(),
        ))
    with open(CERT, "wb") as f:
        f.write(cert.public_bytes(serialization.Encoding.PEM))


def main():
    ensure_cert()

    ip = get_lan_ip()
    print("=" * 60)
    print("GUARD.IAM Stage 1 — head-pose test")
    print("=" * 60)
    print(f"On the Quest browser, open:   https://{ip}:{HTTPS_PORT}")
    print("(Accept the security warning — it's your own cert.)")
    print(f"WebSocket will receive on:    ws://{ip}:{WS_PORT}")
    print("Turn your head — angles print below. Ctrl+C to quit.")
    print("=" * 60)

    # HTTPS server in a background thread
    threading.Thread(target=serve_https, daemon=True).start()

    # WebSocket server on the main thread
    try:
        asyncio.run(ws_main())
    except KeyboardInterrupt:
        print("\nBye.")


if __name__ == "__main__":
    main()
