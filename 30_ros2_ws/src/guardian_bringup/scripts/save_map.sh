#!/bin/bash
MAP_NAME=${1:-guardian_map}
MAP_PATH="$(ros2 pkg prefix guardian_bringup)/share/guardian_bringup/maps/$MAP_NAME"
echo "Saving map to: $MAP_PATH"
ros2 run nav2_map_server map_saver_cli -f "$MAP_PATH" --ros-args -p save_map_timeout:=10.0
echo "Map saved. Files: ${MAP_PATH}.yaml and ${MAP_PATH}.pgm"
