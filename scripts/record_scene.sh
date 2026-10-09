#!/usr/bin/env bash
# Record one rover calibration scene: Airy cloud + ZED rect left image + its camera_info.
#
#   scripts/record_scene.sh rover_scene1 [seconds]     (default 15 s; ~3 s of it is startup)
#
# Needs only zero.launch.py running -- not bringup, nav or YOLO. For native-resolution
# frames, start it with a session-only override that leaves zed_overrides.yaml alone:
#
#   ros2 launch roverrobotics_driver zero.launch.py \
#     zed_param_overrides:='general.grab_resolution:=HD2K;general.pub_resolution:=NATIVE;general.pub_frame_rate:=2.0'
#
# pub_frame_rate 2: a 2K bgra frame is ~11 MB, so 15 Hz would be ~165 MB/s of bag.
# Do NOT run YOLO in that session: yolo_depth_node caches intrinsics once at start.
# Keep the robot and the board still for the whole recording; the cloud is accumulated.
set -euo pipefail
scene=${1:?usage: record_scene.sh <scene_name> [seconds]}
secs=${2:-15}
dir="$(cd "$(dirname "$0")/.." && pwd)/calib_data/$scene"
[[ -e "$dir" ]] && { echo "$dir already exists" >&2; exit 1; }

img=/camera/zed_node/rgb/color/rect/image
# --foreground keeps the recorder in the terminal's process group: without it Ctrl-C
# never reaches it, and the first time rosbag2's keyboard controls touch the tty the
# kernel stops it (state T) -- it then hangs forever and writes nothing.
timeout --foreground --signal=INT "$secs" ros2 bag record -s mcap -o "$dir" \
  --disable-keyboard-controls --topics /rslidar_points "$img" "$img/camera_info" \
  < /dev/null || true
ros2 bag info "$dir" | grep -E "Duration|Topic:"
echo "next: python3 $(dirname "$0")/prepare_scene.py $dir"
