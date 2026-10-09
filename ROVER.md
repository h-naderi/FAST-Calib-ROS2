# FAST-Calib on the Rover Zero 3 (RoboSense Airy + ZED 2i, ROS 2 Jazzy)

Branch `rover-jazzy`, forked from `go2-foxy`. Same half-scale board as the Go2.
Checked out in the rover workspace at `src/fast_calib` (its own git repo, ignored by
wpm-rover). Output is `Rcl`/`Pcl` for `src/fastlivo/config/rover_*_zed.yaml`. Don't
put it anywhere else: the URDF feeds the rest of the stack.

## What differs from go2-foxy

| | |
|---|---|
| OpenCV | System 4.8 (ArUco in `objdetect`, no contrib). The removed `estimatePoseSingleMarkers` / `estimatePoseBoard` are replaced by the `solvePnP` calls they made. No `thirdparty/opencv454`. |
| rosbag2 | `rosbag2_storage::StorageOptions`, `storage_id ""`, so Jazzy's mcap bags open. |
| **Plane alignment** | **Bug fix, affects every tilted LiDAR.** `AngleAxisd` was given an unnormalised axis of length sin(angle), so `R` scaled the board. Exactly invisible on the level XT16. At the Airy's 21° pitch it inflated the hole pattern 6-9% (13 mm RMSE, 27 mm in Pcl on a synthetic scene). |
| Params | `plane_dist_threshold` (upstream's hardcoded 0.01) and `debug_frame` (was `map`, the live SLAM frame on the rover). |
| Topics | Debug clouds are private: `/fast_calib/filtered_cloud`, `/fast_calib/colored_cloud`, ... |
| Launch | Defaults to `qr_params_rover.yaml`, `rviz:=false` (the bundled .rviz is FAST-LIVO2's). |

## Workflow

```bash
# 1. Sensors only, ZED at native res for this session (zed_overrides.yaml untouched).
#    No bringup / YOLO: yolo_depth_node caches intrinsics once at start.
ros2 launch roverrobotics_driver zero.launch.py \
  zed_param_overrides:='general.grab_resolution:=HD2K;general.pub_resolution:=NATIVE;general.pub_frame_rate:=2.0'

# 2. Board upright on a stand ~1.0-1.5 m ahead, whole hole pattern visible to both
#    sensors, robot and board still. One recording per scene; vary tilt between scenes.
src/fast_calib/scripts/record_scene.sh rover_scene1

# 3. frame.png + config/qr_params_rover_rover_scene1.yaml (intrinsics from THIS bag)
python3 src/fast_calib/scripts/prepare_scene.py src/fast_calib/calib_data/rover_scene1

# 4. Run, then check /fast_calib/filtered_cloud + plane_cloud in Foxglove. Tighten
#    the crop box in the scene yaml until it holds just the board (no floor).
ros2 launch fast_calib calib_launch.py params_file:=<printed path>
```

Judge a scene the way CALIB_RESULTS.md does: by per-hole annulus support and agreement
with the other scenes. RMSE is ~0 by construction with the template fit. The extrinsic
does not depend on resolution, so a 2K result goes straight into the 640x360 FAST-LIVO2
config.

`scripts/synth_scene.py` builds a synthetic scene with a known extrinsic (no robot
needed); it should recover truth to about 0.5° / 4 mm.
