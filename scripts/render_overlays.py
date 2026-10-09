#!/usr/bin/env python3
"""Draw the lidar onto each scene's frame.png to check an extrinsic by eye.

    python3 scripts/render_overlays.py config/qr_params_rover_rover_scene*.yaml \
        --calib output_rover_joint/calib_result.txt \
        [--prior ../fastlivo/config/rover_airy_zed.yaml] [-o output_rover_overlays]

Per scene: <scene>_overlay.jpg (whole frame, points coloured by range) and, with
--prior, <scene>_board_compare.jpg (board close-up, prior left / calib right, the hole
outlines from the ArUco pose in white), plus contact_sheet.jpg of all close-ups.
Reading them: blue = board, red/orange = background seen through a hole. Right means
blue stops at the white circles; blue inside a circle is misalignment.
"""
import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from calib_common import (BOARD_H, BOARD_W, camera_board_pose, hole_centres_board,  # noqa: E402
                          load_scene, read_cloud, read_extrinsic)

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def project(X, R, t, K, shape):
    c = X @ R.T + t
    c = c[c[:, 2] > 0.2]
    uv = c @ K.T
    uv = uv[:, :2] / uv[:, 2:]
    m = (uv[:, 0] >= 0) & (uv[:, 0] < shape[1]) & (uv[:, 1] >= 0) & (uv[:, 1] < shape[0])
    return uv[m], c[m, 2]


def draw(img, uv, z, zlo, zhi):
    out = img.copy()
    col = cv2.applyColorMap(np.clip((z - zlo) / (zhi - zlo) * 255, 0, 255).astype(np.uint8).reshape(-1, 1),
                            cv2.COLORMAP_TURBO).reshape(-1, 3)
    for i in np.argsort(-z):                     # near points drawn last, on top
        cv2.circle(out, (int(uv[i, 0]), int(uv[i, 1])), 2, tuple(int(x) for x in col[i]), -1)
    return out


def label(im, text):
    cv2.putText(im, text, (12, 42), 0, 1.2, (0, 0, 0), 6)
    cv2.putText(im, text, (12, 42), 0, 1.2, (255, 255, 255), 2)
    return im


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('scene_yamls', nargs='+')
    ap.add_argument('--calib', required=True, help='calib_result.txt or FAST-LIVO2 yaml with Rcl/Pcl')
    ap.add_argument('--prior', help='second Rcl/Pcl to compare against in the close-ups')
    ap.add_argument('-o', '--out', default=os.path.join(PKG, 'output_rover_overlays'))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    RJ, PJ = read_extrinsic(a.calib)
    sets = ([('prior', read_extrinsic(a.prior))] if a.prior else []) + [('calibrated', (RJ, PJ))]
    tiles = []
    for y in a.scene_yamls:
        p = load_scene(y)
        name = os.path.basename(y).replace('qr_params_rover_', '').replace('.yaml', '')
        img = cv2.imread(p['image_path'])
        Rcb, tcb, n = camera_board_pose(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), p)
        if Rcb is None:
            print(f'skip {name}: {n} markers')
            continue
        X = read_cloud(p['bag_path'], 25)
        r = np.linalg.norm(X, axis=1)
        X = X[(r > 0.3) & (r < 8)]
        uv, z = project(X[::4], RJ, PJ, p['K'], img.shape)
        cv2.imwrite(os.path.join(a.out, f'{name}_overlay.jpg'),
                    label(draw(img, uv, z, 0.5, 6.0), f'{name}  calibrated'), [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not a.prior:
            print(name, 'done'); continue
        d = np.linalg.norm(tcb)
        box = np.array([[x, y_, 0] for x in (-BOARD_W / 2 - .04, BOARD_W / 2 + .04)
                        for y_ in (-BOARD_H / 2 - .04, BOARD_H / 2 + .04)])
        ob = (p['K'] @ (box @ Rcb.T + tcb).T).T
        ob = ob[:, :2] / ob[:, 2:]
        x0, y0 = np.maximum(ob.min(0).astype(int), 0)
        x1, y1 = np.minimum(ob.max(0).astype(int), [img.shape[1], img.shape[0]])
        pair = []
        for nm, (R, t) in sets:
            uv, z = project(X, R, t, p['K'], img.shape)
            o = draw(img, uv, z, d - 0.15, d + 1.2)
            for hx, hy in hole_centres_board(p):
                ring = np.array([[hx + p['circle_radius'] * np.cos(s), hy + p['circle_radius'] * np.sin(s), 0]
                                 for s in np.linspace(0, 2 * np.pi, 90)])
                pr = (p['K'] @ (ring @ Rcb.T + tcb).T).T
                cv2.polylines(o, [(pr[:, :2] / pr[:, 2:]).astype(np.int32)], True, (255, 255, 255), 2)
            crop = o[y0:y1, x0:x1]
            pair.append(label(cv2.resize(crop, (int(560 * crop.shape[1] / crop.shape[0]), 560)), nm))
        tile = np.hstack([pair[0], np.full((560, 8, 3), 255, np.uint8), pair[1]])
        tile = label(np.vstack([np.full((50, tile.shape[1], 3), 40, np.uint8), tile]), name)
        cv2.imwrite(os.path.join(a.out, f'{name}_board_compare.jpg'), tile, [cv2.IMWRITE_JPEG_QUALITY, 90])
        tiles.append(tile)
        print(name, 'done', flush=True)
    if tiles:
        w = max(t.shape[1] for t in tiles)
        tiles = [cv2.copyMakeBorder(t, 0, 10, 0, w - t.shape[1], cv2.BORDER_CONSTANT, value=(255, 255, 255))
                 for t in tiles]
        rows = [np.hstack(tiles[i:i + 3] + [np.full_like(tiles[0], 255)] * (3 - len(tiles[i:i + 3])))
                for i in range(0, len(tiles), 3)]
        sheet = np.vstack(rows)
        cv2.imwrite(os.path.join(a.out, 'contact_sheet.jpg'),
                    cv2.resize(sheet, (sheet.shape[1] // 2, sheet.shape[0] // 2)), [cv2.IMWRITE_JPEG_QUALITY, 88])


if __name__ == '__main__':
    main()
