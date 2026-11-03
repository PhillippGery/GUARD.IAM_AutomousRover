#!/usr/bin/env python3
"""Render a top-down animated GIF of the GUARD.IAM sim demo from a recorded bag.

Reads the MCAP file directly (works on bags with no metadata.yaml and on bags
whose footer is missing because the recorder was killed). The original bag is
only opened read-only.

Needs: pip install mcap mcap-ros2-support matplotlib numpy ; ffmpeg on PATH.

Usage:
  python make_gif.py [--bag PATH] [--goals CSV] [--world SDF] [--out GIF]
"""
import argparse
import csv
import math
import os
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import Polygon, FancyArrow

from mcap.stream_reader import StreamReader
from mcap.records import Channel, Message, Schema
from mcap_ros2.decoder import DecoderFactory

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))

FPS = 15
SPEEDUP = 4.0
TRAIL_S = 10.0
REACH_M = 0.3
ROBOT_L, ROBOT_W = 0.5, 0.4
LEAD_IN_S = 1.0     # keep this much still time before first motion
TAIL_S = 2.0        # keep this much after the last goal is reached


# ---------------------------------------------------------------- bag reading
def yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def read_bag(path):
    """Return odom array [t, x, y, yaw, speed] and map->odom array [t, x, y, yaw]."""
    schemas, channels, decoders = {}, {}, {}
    factory = DecoderFactory()
    odom, m2o = [], []
    seen = set()
    truncated = False
    with open(path, "rb") as f:
        try:
            for rec in StreamReader(f).records:
                if isinstance(rec, Schema):
                    schemas[rec.id] = rec
                elif isinstance(rec, Channel):
                    channels[rec.id] = rec.topic
                    decoders[rec.id] = factory.decoder_for(
                        rec.message_encoding, schemas[rec.schema_id])
                elif isinstance(rec, Message):
                    topic = channels[rec.channel_id]
                    seen.add(topic)
                    if topic == "/odom":
                        m = decoders[rec.channel_id](rec.data)
                        p, tw = m.pose.pose, m.twist.twist
                        odom.append((stamp(m.header), p.position.x, p.position.y,
                                     yaw_from_quat(p.orientation),
                                     math.hypot(tw.linear.x, tw.linear.y)))
                    elif topic == "/tf":
                        m = decoders[rec.channel_id](rec.data)
                        for tr in m.transforms:
                            if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                                t = tr.transform
                                m2o.append((stamp(tr.header), t.translation.x,
                                            t.translation.y, yaw_from_quat(t.rotation)))
        except Exception as e:  # missing footer / partial last record
            truncated = True
            print(f"note: stopped reading at byte {f.tell()} ({e}); using records before it")
    for need in ("/odom", "/tf"):
        if need not in seen:
            sys.exit(f"error: topic {need} not found in bag (found: {sorted(seen)})")
    odom = np.array(sorted(odom))
    m2o = np.array(sorted(m2o)) if m2o else None
    return odom, m2o, truncated


def odom_to_map(odom, m2o):
    """Apply time-interpolated map->odom to the odom poses."""
    if m2o is None:
        print("map->odom: not in /tf; map and odom treated as the same frame")
        return odom.copy()
    dx = np.ptp(m2o[:, 1]); dy = np.ptp(m2o[:, 2]); dyaw = np.ptp(np.unwrap(m2o[:, 3]))
    if max(abs(m2o[:, 1]).max(), abs(m2o[:, 2]).max()) < 1e-3 and abs(m2o[:, 3]).max() < 1e-3:
        print("map->odom: identity; map and odom frames match")
        return odom.copy()
    print(f"map->odom: differs from identity (x range {dx:.2f} m, y range {dy:.2f} m, "
          f"yaw range {math.degrees(dyaw):.1f} deg); applying it")
    t = odom[:, 0]
    tx = np.interp(t, m2o[:, 0], m2o[:, 1])
    ty = np.interp(t, m2o[:, 0], m2o[:, 2])
    th = np.interp(t, m2o[:, 0], np.unwrap(m2o[:, 3]))
    c, s = np.cos(th), np.sin(th)
    out = odom.copy()
    out[:, 1] = tx + c * odom[:, 1] - s * odom[:, 2]
    out[:, 2] = ty + s * odom[:, 1] + c * odom[:, 2]
    out[:, 3] = np.arctan2(np.sin(odom[:, 3] + th), np.cos(odom[:, 3] + th))
    return out


# ---------------------------------------------------------------- world / goals
def parse_pose(text):
    v = [float(x) for x in (text or "0 0 0 0 0 0").split()]
    return v + [0.0] * (6 - len(v))


def read_world(path):
    """Return list of (name, corners Nx2) for every box geometry in the world."""
    root = ET.parse(path).getroot()
    boxes = []
    for model in root.iter("model"):
        mp = parse_pose(model.findtext("pose"))
        for link in model.findall("link"):
            lp = parse_pose(link.findtext("pose"))
            for col in link.findall("collision"):
                size = col.find("geometry/box/size")
                if size is None:
                    continue
                sx, sy = [float(v) for v in size.text.split()[:2]]
                cp = parse_pose(col.findtext("pose"))
                # compose planar poses: model * link * collision
                x, y, yaw = mp[0], mp[1], mp[5]
                for p in (lp, cp):
                    x, y = x + math.cos(yaw) * p[0] - math.sin(yaw) * p[1], \
                           y + math.sin(yaw) * p[0] + math.cos(yaw) * p[1]
                    yaw += p[5]
                boxes.append((model.get("name"), rect(x, y, yaw, sx, sy)))
    return boxes


def rect(x, y, yaw, length, width):
    hl, hw = length / 2, width / 2
    pts = np.array([[hl, hw], [-hl, hw], [-hl, -hw], [hl, -hw]])
    c, s = math.cos(yaw), math.sin(yaw)
    return pts @ np.array([[c, s], [-s, c]]) + [x, y]


def read_goals(path):
    with open(path) as f:
        return [dict(num=int(r["goal_num"]), label=r["label"], x=float(r["x"]),
                     y=float(r["y"]), yaw=math.radians(float(r["yaw_deg"])))
                for r in csv.DictReader(f)]


def reach_times(traj, goals):
    """Time each goal is first reached, taking goals in order."""
    times, start = [], 0
    for g in goals:
        d = np.hypot(traj[start:, 1] - g["x"], traj[start:, 2] - g["y"])
        hit = np.nonzero(d < REACH_M)[0]
        if len(hit) == 0:
            times.append(None)
            print(f"warning: goal {g['num']} ({g['label']}) never within {REACH_M} m "
                  f"(closest {d.min():.2f} m)")
            start = len(traj) - 1
        else:
            start += hit[0]
            times.append(traj[start, 0])
    return times


# ---------------------------------------------------------------- rendering
def render(traj, goals, reached, boxes, out_path):
    moving = np.nonzero(traj[:, 4] > 0.02)[0]
    t0 = traj[moving[0], 0] - LEAD_IN_S
    last_goal = max((t for t in reached if t is not None), default=traj[moving[-1], 0])
    t1 = min(last_goal + TAIL_S, traj[-1, 0])
    frame_times = np.arange(t0, t1 + 1e-9, SPEEDUP / FPS)
    print(f"clip: bag t={t0:.1f}..{t1:.1f} s ({t1 - t0:.1f} s real) -> "
          f"{len(frame_times)} frames, {len(frame_times) / FPS:.1f} s GIF")

    allpts = np.vstack([b for _, b in boxes] + [traj[:, 1:3]])
    lo, hi = allpts.min(0) - 0.3, allpts.max(0) + 0.3
    span = max(hi - lo); mid = (hi + lo) / 2

    fig = plt.figure(figsize=(8, 8), dpi=100)
    ax = fig.add_axes([0.08, 0.08, 0.88, 0.75])
    ax.set_aspect("equal")
    ax.set_xlim(mid[0] - span / 2, mid[0] + span / 2)
    ax.set_ylim(mid[1] - span / 2, mid[1] + span / 2)
    ax.set_xlabel("x [m] (map)"); ax.set_ylabel("y [m] (map)")
    ax.grid(True, color="#e6e6e6", lw=0.6); ax.set_axisbelow(True)
    ax.set_facecolor("#fbfbf8")
    for name, b in boxes:
        wall = name.startswith("wall")
        ax.add_patch(Polygon(b, closed=True, fc="#555555" if wall else "#b5835a",
                             ec="#333333", lw=0.8, zorder=2))

    goal_art = []
    for g in goals:
        dot, = ax.plot(g["x"], g["y"], "o", ms=17, mfc="#9e9e9e", mec="white",
                       mew=1.5, zorder=4)
        arr = FancyArrow(g["x"], g["y"], 0.55 * math.cos(g["yaw"]), 0.55 * math.sin(g["yaw"]),
                         width=0.05, head_width=0.18, head_length=0.15,
                         color="#9e9e9e", zorder=3, length_includes_head=True)
        ax.add_patch(arr)
        ax.text(g["x"], g["y"], str(g["num"]), ha="center", va="center",
                color="white", fontsize=10, fontweight="bold", zorder=5)
        goal_art.append((dot, arr))

    trail = LineCollection([], lw=2.5, zorder=6, capstyle="round")
    ax.add_collection(trail)
    body = Polygon(np.zeros((4, 2)), closed=True, fc="#1f6fd1", ec="#0b3d7a",
                   lw=1.2, zorder=7)
    ax.add_patch(body)
    heading, = ax.plot([], [], color="#ffcc00", lw=2.5, zorder=8, solid_capstyle="round")

    fig.text(0.07, 0.955, "GUARD.IAM sim demo: 5-goal Nav2 run", fontsize=15,
             fontweight="bold", va="center")
    panel = fig.text(0.07, 0.885, "", fontsize=11, family="monospace", va="center")
    fig.text(0.97, 0.955, f"{SPEEDUP:g}x speed", fontsize=11, ha="right",
             va="center", color="#666666")

    tmp = tempfile.mkdtemp(prefix="simgif_")
    try:
        for i, t in enumerate(frame_times):
            k = min(np.searchsorted(traj[:, 0], t), len(traj) - 1)
            x, y, yaw, v = traj[k, 1:5]

            body.set_xy(rect(x, y, yaw, ROBOT_L, ROBOT_W))
            heading.set_data([x, x + 0.45 * math.cos(yaw)], [y, y + 0.45 * math.sin(yaw)])

            j0 = np.searchsorted(traj[:, 0], t - TRAIL_S)
            seg = traj[j0:k + 1:3, 1:3]
            if len(seg) > 1:
                segs = np.stack([seg[:-1], seg[1:]], axis=1)
                a = np.linspace(0.05, 0.9, len(segs))
                trail.set_segments(segs)
                trail.set_color([(0.12, 0.44, 0.82, ai) for ai in a])
            else:
                trail.set_segments([])

            cur = None
            for g, rt, (dot, arr) in zip(goals, reached, goal_art):
                done = rt is not None and rt <= t
                col = "#2ca02c" if done else "#9e9e9e"
                dot.set_markerfacecolor(col); arr.set_color(col)
                if not done and cur is None:
                    cur = g
            goal_txt = f"{cur['num']}/{len(goals)} {cur['label']}" if cur else "all reached"
            panel.set_text(f"time {t - t0:6.1f} s   speed {v:4.2f} m/s\n"
                           f"goal {goal_txt}")
            fig.savefig(os.path.join(tmp, f"f{i:05d}.png"))
        plt.close(fig)

        pal = os.path.join(tmp, "palette.png")
        inp = ["-framerate", str(FPS), "-i", os.path.join(tmp, "f%05d.png")]
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", *inp,
                        "-vf", "palettegen=max_colors=128:stats_mode=diff", pal], check=True)
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", *inp, "-i", pal,
                        "-lavfi", "paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle",
                        "-loop", "0", out_path], check=True)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    mb = os.path.getsize(out_path) / 1e6
    print(f"wrote {out_path}: {mb:.2f} MB, {len(frame_times) / FPS:.1f} s, "
          f"{len(frame_times)} frames @ {FPS} fps")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--bag", default=os.path.join(HERE, "bags/demo_v7/demo_v7_0.mcap"))
    ap.add_argument("--goals", default=os.path.join(HERE, "last_run.csv"))
    ap.add_argument("--world", default=os.path.join(
        REPO, "30_ros2_ws/src/guardian_description/worlds/guardian_world.sdf"))
    ap.add_argument("--out", default=os.path.join(HERE, "sim_demo.gif"))
    a = ap.parse_args()

    odom, m2o, _ = read_bag(a.bag)
    traj = odom_to_map(odom, m2o)
    goals = read_goals(a.goals)
    boxes = read_world(a.world)
    reached = reach_times(traj, goals)
    for g, rt in zip(goals, reached):
        if rt is not None:
            print(f"goal {g['num']} {g['label']}: reached at bag t={rt:.1f} s")
    render(traj, goals, reached, boxes, a.out)


if __name__ == "__main__":
    main()
