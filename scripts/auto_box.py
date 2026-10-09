#!/usr/bin/env python3
"""Fit a scene's crop box to the board automatically.

    python3 scripts/auto_box.py config/qr_params_rover_<scene>.yaml [--prior <fastlivo yaml>]

Run after prepare_scene.py. The camera finds the board (ArUco), a PRIOR extrinsic
(default: fastlivo's rover_airy_zed.yaml, URDF-assembled, a few degrees off) maps it into
the lidar frame, a trimmed plane fit pins it in the lidar cloud, and the box is that
board +-2 cm. Hand-tuning 18 boxes was the slow, error-prone step; the prior only has to
be good to ~10 cm, so any rough extrinsic works. Also sets plane_dist_threshold 0.02:
the Airy reads the board ~13 mm "thick" (per-ring range offsets), so 0.01 is too tight.
"""
import argparse
import os
import re
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from calib_common import (BOARD_H, BOARD_W, camera_board_pose, fit_plane,  # noqa: E402
                          load_scene, read_cloud, read_extrinsic)

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_PRIOR = os.path.join(PKG, '..', 'fastlivo', 'config', 'rover_airy_zed.yaml')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('scene_yaml')
    ap.add_argument('--prior', default=DEFAULT_PRIOR, help='yaml with a rough Rcl/Pcl')
    a = ap.parse_args()
    p = load_scene(a.scene_yaml)
    R0, P0 = read_extrinsic(a.prior)

    Rcb, tcb, n = camera_board_pose(cv2.imread(p['image_path'], 0), p)
    if Rcb is None:
        sys.exit(f'{a.scene_yaml}: {n} board markers detected, need 4 -- scene unusable')
    outline = np.array([[x, y, 0] for x in (-BOARD_W / 2, BOARD_W / 2) for y in (-BOARD_H / 2, BOARD_H / 2)])
    pred = (outline @ Rcb.T + tcb - P0) @ R0            # board corners in the lidar frame
    lo, hi = pred.min(0) - 0.10, pred.max(0) + 0.10
    X = read_cloud(p['bag_path'])
    X = X[np.all((X > lo) & (X < hi), 1)]

    nrm, c = fit_plane(X, R0.T @ Rcb[:, 2], pred.mean(0))
    ex = R0.T @ Rcb[:, 0]; ex -= ex.dot(nrm) * nrm; ex /= np.linalg.norm(ex); ey = np.cross(nrm, ex)
    u, v, dist = (X - c) @ ex, (X - c) @ ey, (X - c) @ nrm
    on = np.abs(dist) < 0.03
    uc, vc = np.median(u[on]), np.median(v[on])
    board = on & (np.abs(u - uc) < BOARD_W / 2 + 0.04) & (np.abs(v - vc) < BOARD_H / 2 + 0.04)
    blo, bhi = X[board].min(0) - 0.02, X[board].max(0) + 0.02
    inbox = np.all((X > blo) & (X < bhi), 1)
    purity = 100 * np.mean(np.abs(dist[inbox]) < 0.02)

    s = open(a.scene_yaml).read()
    for k, val in zip(['x_min', 'y_min', 'z_min', 'x_max', 'y_max', 'z_max', 'plane_dist_threshold'],
                      list(blo) + list(bhi) + [0.02]):
        s, m = re.subn(rf'^(\s+{k}:\s*)\S+', rf'\g<1>{val:.3f}', s, count=1, flags=re.M)
        if m != 1:
            sys.exit(f'{a.scene_yaml}: no "{k}:" line')
    s = s.replace('    # --- Distance filter, rslidar frame ---',
                  f'    # Crop box fitted by scripts/auto_box.py: board at {np.linalg.norm(c):.2f} m, '
                  f'{purity:.0f}% of box points on its plane.\n    # --- Distance filter, rslidar frame ---', 1) \
        if 'fitted by scripts/auto_box.py' not in s else s
    open(a.scene_yaml, 'w').write(s)
    tilt = np.degrees(np.arccos(abs(nrm @ (R0.T @ Rcb[:, 2]))))
    print(f'{os.path.basename(a.scene_yaml)}: board {np.linalg.norm(c):.2f} m, {board.sum()} pts, '
          f'box purity {purity:.0f}%, camera-vs-lidar board normal under the prior {tilt:.1f} deg')


if __name__ == '__main__':
    main()
