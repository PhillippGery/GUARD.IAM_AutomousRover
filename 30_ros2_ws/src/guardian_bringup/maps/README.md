# maps

Saved occupancy grid maps (`.yaml` + `.pgm`) for AMCL localization (`mode:=navigation`).

**This directory (`src/guardian_bringup/maps/`) is the canonical, git-tracked home
for maps** — not `install/guardian_bringup/share/.../maps/`, which is gitignored
build output that gets wiped by a clean rebuild.

Run `save_map.sh` after building a map in mapping mode — it saves here directly and
rebuilds `guardian_bringup` so `mode:=navigation` picks up the change immediately.
When you map a real room, commit the resulting `.yaml`/`.pgm` pair so it's not lost.
