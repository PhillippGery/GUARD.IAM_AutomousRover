# Scripts

Convenience shell scripts for setup, building, and running GUARDIAN.

| Script | Purpose |
|--------|---------|
| `setup_amd_minipc.sh` | One-shot install of all system dependencies on Ubuntu 22.04 |
| `build_workspace.sh` | Source ROS2, install deps, build workspace |
| `run_guardian.sh` | Interactive launch menu |
| `guardiam_env.sh` | Per-session env: sources ROS2 Jazzy + workspace, sets `ROS_DOMAIN_ID=42`, defines `guardiam_sim`/`guardiam_real`/`guardiam_map`/`cb`/`cbs` aliases |

## Usage

```bash
# First-time setup (run once after cloning)
bash 60_scripts/setup_amd_minipc.sh

# Build workspace
bash 60_scripts/build_workspace.sh

# Launch
bash 60_scripts/run_guardian.sh
```

`setup_amd_minipc.sh` adds `source 60_scripts/guardiam_env.sh` to `~/.bashrc` automatically. To pick it up in your current shell without re-running setup:

```bash
source 60_scripts/guardiam_env.sh
```
