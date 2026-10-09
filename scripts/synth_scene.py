#!/usr/bin/env python3
"""Synthetic rover scene with a KNOWN T_cam_lidar -- a regression test needing no robot.

    python3 scripts/synth_scene.py calib_data/synth_scene
    python3 scripts/prepare_scene.py calib_data/synth_scene
    ros2 launch fast_calib calib_launch.py params_file:=<printed path>

Renders the Go2 board into a 2208x1242 ZED-like rect frame and simulates the board
(with holes, 4 mm noise) plus the floor as seen by an Airy pitched 21 deg nose-down,
then writes the same three topics record_scene.sh records. Truth is the URDF-derived
Rcl/Pcl and is saved as <bag>_truth.json. Expect ~0.5 deg / ~4 mm and RMSE ~0.

It exists because the tilt matters: the unnormalised AngleAxis axis in
lidar_detect.hpp scaled the board by 6-9% at this pitch (13 mm RMSE, 27 mm in Pcl)
while being exactly invisible on the level XT16.
"""
import sys, os, json
import numpy as np, cv2
import rosbag2_py
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

out = sys.argv[1].rstrip('/')
rng = np.random.default_rng(0)

# ground truth = the rover yaml's current Rcl/Pcl (URDF-derived)
Rcl = np.array([[0.0, -0.999863, 0.016549], [0.358353, -0.01545, -0.933458], [0.933586, 0.00593, 0.358304]])
U, _, Vt = np.linalg.svd(Rcl); Rcl = U @ Vt
Pcl = np.array([0.06, -0.036481, 0.004568])

# lidar pitched 21 deg nose down: p_w = Rwl p_l
th = np.radians(21.0); c, s = np.cos(th), np.sin(th)
Rwl = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
# board in world (relative to lidar origin): centre, axes; slight yaw for realism
yaw = np.radians(8.0)
centre = np.array([1.2, 0.05, 0.2])
bx = np.array([np.sin(yaw), -np.cos(yaw), 0.0])     # board +x = right, facing the board
by = np.array([0.0, 0.0, 1.0])                       # board +y = up
bz = np.cross(bx, by)                                # toward the robot
Rwb = np.column_stack([bx, by, bz])

W, H = 0.6858, 0.5461
holes = [(sx * 0.26575 / 2, sy * 0.213 / 2) for sx, sy in [(-1, 1), (1, 1), (1, -1), (-1, -1)]]
r = 0.081

# --- lidar: board surface (minus holes) + floor, 20 scans with noise
def board_pts(n):
    u = rng.uniform(-W / 2, W / 2, n); v = rng.uniform(-H / 2, H / 2, n)
    keep = np.ones(n, bool)
    for hx, hy in holes:
        keep &= (u - hx) ** 2 + (v - hy) ** 2 > r ** 2
    u, v = u[keep], v[keep]
    pw = centre + np.outer(u, bx) + np.outer(v, by) + np.outer(rng.normal(0, 0.004, len(u)), bz)
    return pw
def floor_pts(n):
    x = rng.uniform(0.3, 2.5, n); y = rng.uniform(-1, 1, n)
    return np.column_stack([x, y, np.full(n, -0.2425) + rng.normal(0, 0.004, n)])

# --- camera: ZED 2K-ish
fx = fy = 1060.0; cx, cy = 1104.0, 621.0; Wi, Hi = 2208, 1242
K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
# board in camera: p_c = Rcl Rwl^T (Rwb p_b + centre) + Pcl
Rcb = Rcl @ Rwl.T @ Rwb
tcb = Rcl @ Rwl.T @ centre + Pcl
# texture: 1 px = 0.5 mm
ppm = 2000.0
tw, thh = int(W * ppm), int(H * ppm)
tex = np.full((thh, tw), 235, np.uint8)
d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
side = int(0.08 * ppm)
for mid, (mx, my) in {1: (-0.2725, 0.203), 2: (0.2725, 0.203), 3: (-0.2725, -0.203), 4: (0.2725, -0.203)}.items():
    m = cv2.aruco.drawMarker(d, mid, side)
    u0 = int((mx + W / 2) * ppm - side / 2); v0 = int((H / 2 - my) * ppm - side / 2)
    tex[v0:v0 + side, u0:u0 + side] = m
for hx, hy in holes:
    cv2.circle(tex, (int((hx + W / 2) * ppm), int((H / 2 - hy) * ppm)), int(r * ppm), 60, -1)
# texture px -> board metres
A = np.array([[1 / ppm, 0, -W / 2], [0, -1 / ppm, H / 2], [0, 0, 1]])
Hm = K @ np.column_stack([Rcb[:, 0], Rcb[:, 1], tcb]) @ A
img = np.full((Hi, Wi), 120, np.uint8)
warped = cv2.warpPerspective(tex, Hm, (Wi, Hi), flags=cv2.INTER_AREA)
mask = cv2.warpPerspective(np.full_like(tex, 255), Hm, (Wi, Hi))
img[mask > 0] = warped[mask > 0]
img = cv2.GaussianBlur(img, (3, 3), 0.8)
img = np.clip(img.astype(np.int16) + rng.normal(0, 2, img.shape).astype(np.int16), 0, 255).astype(np.uint8)
bgra = cv2.cvtColor(img, cv2.COLOR_GRAY2BGRA)

w = rosbag2_py.SequentialWriter()
w.open(rosbag2_py.StorageOptions(uri=out, storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
topics = {'/rslidar_points': 'sensor_msgs/msg/PointCloud2',
          '/camera/zed_node/rgb/color/rect/image': 'sensor_msgs/msg/Image',
          '/camera/zed_node/rgb/color/rect/image/camera_info': 'sensor_msgs/msg/CameraInfo'}
for i, (n, t) in enumerate(topics.items()):
    w.create_topic(rosbag2_py.TopicMetadata(id=i, name=n, type=t, serialization_format='cdr'))
for k in range(20):
    ns = int(1e9) * 100 + k * int(1e8)
    hdr = Header(stamp=Time(sec=ns // 10**9, nanosec=ns % 10**9), frame_id='rslidar')
    pw = np.vstack([board_pts(6000), floor_pts(20000)])
    pl = pw @ Rwl                                   # p_l = Rwl^T p_w
    w.write('/rslidar_points', serialize_message(point_cloud2.create_cloud_xyz32(hdr, pl.astype(np.float32))), ns)
    if k % 7 == 0:
        h2 = Header(stamp=hdr.stamp, frame_id='camera_left_camera_frame_optical')
        im = Image(header=h2, height=Hi, width=Wi, encoding='bgra8', step=Wi * 4, data=bgra.tobytes())
        ci = CameraInfo(header=h2, height=Hi, width=Wi, distortion_model='plumb_bob',
                        d=[0.0] * 5, k=K.flatten().tolist())
        w.write('/camera/zed_node/rgb/color/rect/image', serialize_message(im), ns)
        w.write('/camera/zed_node/rgb/color/rect/image/camera_info', serialize_message(ci), ns)
del w
json.dump({'Rcl': Rcl.tolist(), 'Pcl': Pcl.tolist(), 'board_centre_lidar': (Rwl.T @ centre).tolist()},
          open(out + '_truth.json', 'w'), indent=1)
print('board centre in lidar frame', Rwl.T @ centre)
