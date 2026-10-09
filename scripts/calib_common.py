"""Shared helpers for the rover calibration scripts (auto_box, joint_solve, render_overlays).

Everything here works from a scene's params yaml (config/qr_params_rover_<scene>.yaml,
written by prepare_scene.py), so a scene is always processed with its own intrinsics,
bag and crop box.
"""
import re

import cv2
import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc

# Outer size of the Go2 half-scale board (qr_params_go2.yaml). Only used to bound crops
# and drawings; the calibration itself uses the hole/marker spacing from the params.
BOARD_W, BOARD_H = 0.6858, 0.5461


def load_scene(path):
    """The ros__parameters of a scene yaml, whatever node key it uses (/** or fast_calib)."""
    doc = yaml.safe_load(open(path))
    p = next(iter(doc.values()))['ros__parameters']
    p['K'] = np.array([[p['fx'], 0, p['cx']], [0, p['fy'], p['cy']], [0, 0, 1.0]])
    p['D'] = np.array([p['k1'], p['k2'], p['p1'], p['p2'], 0.0])
    p['yaml'] = path
    return p


def hole_centres_board(p):
    """Hole centres in the board frame, TL TR BR BL facing the board (the C++ order)."""
    hw, hh = p['delta_width_circles'] / 2, p['delta_height_circles'] / 2
    return np.array([[-hw, hh], [hw, hh], [hw, -hh], [-hw, -hh]])


def camera_board_pose(img, p):
    """Board pose in the camera (R, t, n_markers) from the ArUco markers, as qr_detect.hpp
    does: IDs 1,2,4,3 clockwise from top-left, marker_corner_shift applied."""
    w, h, m = p['delta_width_qr_center'], p['delta_height_qr_center'], p['marker_size']
    obj = []
    for i in range(4):
        xs, ys = (-1 if i % 3 == 0 else 1), (1 if i < 2 else -1)
        obj.append(np.array([[xs * w + (-1 if j % 3 == 0 else 1) * m / 2,
                              ys * h + (1 if j < 2 else -1) * m / 2, 0] for j in range(4)], np.float32))
    ids_board = np.array([1, 2, 4, 3])
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    if hasattr(cv2.aruco, 'ArucoDetector'):          # OpenCV >= 4.7
        corners, ids, _ = cv2.aruco.ArucoDetector(d).detectMarkers(img)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(img, d)
    if ids is None:
        return None, None, 0
    shift = p.get('marker_corner_shift', [0, 0, 0, 0])
    corners = [np.roll(c, -(shift[i - 1] % 4), axis=1) if 1 <= i <= 4 else c
               for c, i in zip(corners, ids.ravel())]
    o, q = [], []
    for c, i in zip(corners, ids.ravel()):
        if i in ids_board:
            o.append(obj[int(np.where(ids_board == i)[0][0])]); q.append(c.reshape(4, 2))
    if len(o) < 4:
        return None, None, len(o)
    ok, rv, tv = cv2.solvePnP(np.vstack(o), np.vstack(q).astype(np.float32), p['K'], p['D'])
    return cv2.Rodrigues(rv)[0], tv.ravel(), len(o)


def read_cloud(bag, max_scans=None, topic='/rslidar_points'):
    """Accumulated x,y,z of up to max_scans lidar messages."""
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    out, n = [], 0
    while r.has_next() and (max_scans is None or n < max_scans):
        _, data, _ = r.read_next()
        msg = deserialize_message(data, PointCloud2)
        out.append(pc.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        n += 1
    return np.vstack(out).astype(float)


def fit_plane(X, normal, centre, bands=(0.08, 0.05, 0.03, 0.03)):
    """Iteratively trimmed least-squares plane, seeded with a normal and a point."""
    for band in bands:
        sel = np.abs((X - centre) @ normal) < band
        centre = X[sel].mean(0)
        normal = np.linalg.svd(X[sel] - centre, full_matrices=False)[2][2]
    return normal, centre


def board_points(cloud, p):
    """Lidar points on the board: the scene's crop box, then a +-3 cm plane band."""
    lo = np.array([p['x_min'], p['y_min'], p['z_min']]); hi = np.array([p['x_max'], p['y_max'], p['z_max']])
    X = cloud[np.all((cloud > lo) & (cloud < hi), 1)]
    c = X.mean(0); n = np.linalg.svd(X - c, full_matrices=False)[2][2]
    n, c = fit_plane(X, n, c)
    return X[np.abs((X - c) @ n) < 0.03]


def kabsch(A, B):
    """R, t minimising |R A + t - B|."""
    ca, cb = A.mean(0), B.mean(0)
    U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, cb - R @ ca


def rot_deg(Ra, Rb):
    """Angle between two rotations [deg]."""
    return np.degrees(np.arccos(np.clip((np.trace(Ra.T @ Rb) - 1) / 2, -1, 1)))


def read_extrinsic(path):
    """Rcl, Pcl from a FAST-LIVO2 yaml or a FAST-Calib calib_result.txt (same layout)."""
    s = open(path).read()
    num = r'[-+0-9.eE]+'
    R = [float(x) for x in re.findall(num, re.search(r'Rcl:\s*\[(.*?)\]', s, re.S).group(1))]
    P = [float(x) for x in re.findall(num, re.search(r'Pcl:\s*\[(.*?)\]', s, re.S).group(1))]
    R = np.array(R).reshape(3, 3)
    U, _, Vt = np.linalg.svd(R)        # hand-typed matrices are only orthonormal to ~1e-5
    return U @ Vt, np.array(P)


def holes_metric(R, P, pts, Rcb, tcb, p):
    """% of board-surface lidar points that land inside an image hole (the Go2's overlay
    check). Independent of the hole fit: the holes come from the ArUco board pose."""
    pc_ = pts @ R.T + P
    n = Rcb[:, 2]
    X = pc_ * ((n @ tcb) / (pc_ @ n))[:, None]        # ray -> camera's board plane
    b = (X - tcb) @ Rcb
    d = np.min(np.linalg.norm(b[:, None, :2] - hole_centres_board(p)[None], axis=2), axis=1)
    return 100.0 * np.mean(d < 0.97 * p['circle_radius'])
