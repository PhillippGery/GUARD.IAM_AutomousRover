#!/bin/bash
# Saves into the SOURCE tree (src/guardian_bringup/maps/), not
# install/share — install/ is gitignored build output, so a map saved
# there is invisible to git and gets wiped by a clean rebuild. This is
# the one and only place a map actually lives; guardian.launch.py's
# default_map loads it from install/share only because colcon mirrors
# this directory there on build, which is why this script rebuilds
# guardian_bringup at the end — otherwise mode:=navigation would keep
# loading whatever old map was last built into install/.
set -e
MAP_NAME=${1:-guardian_map}
# readlink -f resolves the symlink chain first — this script is invoked
# both directly from source and via its colcon --symlink-install'd copy
# under install/share/ (e.g. from web_ops_node, which locates it through
# get_package_share_directory()); without resolving symlinks first,
# dirname would land inside install/guardian_bringup/ when invoked that
# way, and the WS_DIR math below would cd into the wrong place entirely.
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
SRC_MAPS_DIR="$SCRIPT_DIR/../maps"
MAP_PATH="$SRC_MAPS_DIR/$MAP_NAME"
echo "Saving map to: $MAP_PATH"
ros2 run nav2_map_server map_saver_cli -f "$MAP_PATH" --ros-args -p save_map_timeout:=10.0
echo "Map saved. Files: ${MAP_PATH}.yaml and ${MAP_PATH}.pgm"

echo "Rebuilding guardian_bringup so mode:=navigation picks up the new map..."
WS_DIR="$SCRIPT_DIR/../../.."
(cd "$WS_DIR" && colcon build --symlink-install --packages-select guardian_bringup)
echo "Done — '$MAP_NAME' is now in git-trackable source (src/guardian_bringup/maps/) and loadable by mode:=navigation."
