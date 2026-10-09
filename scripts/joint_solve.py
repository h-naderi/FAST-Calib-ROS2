#!/usr/bin/env python3
"""Multi-scene calibration: ONE Kabsch fit over every scene's hole-centre pairs.

    python3 scripts/joint_solve.py config/qr_params_rover_rover_scene*.yaml \
        [--prior ../fastlivo/config/rover_airy_zed.yaml] [--holes] [--out output_rover_joint]

Reads each scene's <output_path>/centers.txt (written by the fast_calib node) and fits
all pairs at once, as upstream FAST-Calib's multi_calib does. Scenes without a
centers.txt (no 4-marker detection / no lock) are listed and skipped.

Why joint and not the Go2's average of per-scene results: with every board 1-1.7 m
away, one scene can trade a rotation against a translation, and averaging per-scene
answers keeps that bias. On the rover's 17 scenes the joint fit scored 0.22% on the
holes check (0.23% leave-one-out) against 0.44% for the average.

Reports per-scene spread, a bootstrap 68% interval, leave-one-out centre error and how
the answer converges with scene count; --holes adds the independent overlay check
(reads every bag, slower). Writes <out>/calib_result.txt in FAST-LIVO2's Rcl/Pcl format.
"""
import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from calib_common import (board_points, camera_board_pose, holes_metric, kabsch,  # noqa: E402
                          load_scene, read_cloud, read_extrinsic, rot_deg)

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('scene_yamls', nargs='+')
    ap.add_argument('--prior', help='yaml with an Rcl/Pcl to compare against (e.g. the URDF-assembled one)')
    ap.add_argument('--holes', action='store_true', help='also run the board-points-in-holes check')
    ap.add_argument('--out', default=os.path.join(PKG, 'output_rover_joint'))
    ap.add_argument('--seed', type=int, default=0)
    a = ap.parse_args()

    S = {}
    for y in a.scene_yamls:
        p = load_scene(y)
        f = os.path.join(p['output_path'], 'centers.txt')
        name = os.path.basename(y).replace('qr_params_rover_', '').replace('.yaml', '')
        if not os.path.exists(f):
            print(f'skip {name}: no {f}')
            continue
        c = np.loadtxt(f)
        S[name] = dict(p=p, q=c[:, :3], l=c[:, 3:])
    names = list(S)
    if len(names) < 3:
        sys.exit('need at least 3 scenes')

    def joint(keys):
        return kabsch(np.vstack([S[k]['l'] for k in keys]), np.vstack([S[k]['q'] for k in keys]))

    def rms(R, P, k):
        return 1000 * np.sqrt(np.mean(np.sum((S[k]['l'] @ R.T + P - S[k]['q']) ** 2, 1)))

    R, P = joint(names)
    print(f'\n{len(names)} scenes, {4 * len(names)} pairs\n')
    print('scene             own fit vs joint     joint-fit centre RMS')
    for k in names:
        Rk, Pk = kabsch(S[k]['l'], S[k]['q'])
        print(f'{k:16s}  {rot_deg(R, Rk):5.2f} deg {1000 * np.linalg.norm(Pk - P):5.1f} mm   {rms(R, P, k):5.1f} mm')

    rng = np.random.default_rng(a.seed)
    br, bt = [], []
    for _ in range(1000):
        Rb, Pb = joint(list(rng.choice(names, len(names))))
        br.append(rot_deg(R, Rb)); bt.append(np.linalg.norm(Pb - P))
    loo = [rms(*joint([x for x in names if x != k]), k) for k in names]
    print(f'\nbootstrap 68%: {np.percentile(br, 68):.2f} deg, {1000 * np.percentile(bt, 68):.1f} mm')
    print(f'leave-one-out centre RMS on the held-out scene: {np.mean(loo):.1f} mm '
          f'(in-sample {np.mean([rms(R, P, k) for k in names]):.1f} mm)')
    for n in (4, 8, 12, 16):
        if n < len(names):
            e = [rot_deg(R, joint(list(rng.choice(names, n, replace=False)))[0]) for _ in range(300)]
            print(f'  {n:2d} scenes -> median {np.median(e):.2f} deg from the all-scene answer')

    cands = {'joint': (R, P)}
    if a.prior:
        R0, P0 = read_extrinsic(a.prior)
        cands['prior'] = (R0, P0)
        print(f'\nprior vs joint: {rot_deg(R0, R):.2f} deg, Pcl {1000 * np.linalg.norm(P0 - P):.0f} mm')
    if a.holes:
        print('\nboard points landing in an image hole (lower is better; the Go2 check):')
        pts = {}
        for k in names:
            p = S[k]['p']
            Rcb, tcb, _ = camera_board_pose(cv2.imread(p['image_path'], 0), p)
            pts[k] = (board_points(read_cloud(p['bag_path'], 40), p), Rcb, tcb)
        for c, (Rc, Pc) in cands.items():
            v = [holes_metric(Rc, Pc, *pts[k], S[k]['p']) for k in names]
            print(f'  {c:6s} mean {np.mean(v):.2f}%  worst {max(v):.2f}%')
        v = [holes_metric(*joint([x for x in names if x != k]), *pts[k], S[k]['p']) for k in names]
        print(f'  joint, leave-one-out: mean {np.mean(v):.2f}%')

    os.makedirs(a.out, exist_ok=True)
    with open(os.path.join(a.out, 'calib_result.txt'), 'w') as f:
        f.write(f'# FAST-Calib joint fit over {len(names)} scenes: {" ".join(names)}\n')
        f.write(f'# bootstrap 68%: {np.percentile(br, 68):.2f} deg, {1000 * np.percentile(bt, 68):.1f} mm\n')
        f.write('# T_cam_lidar, p_cam = Rcl * p_lidar + Pcl (FAST-LIVO2 extrin_calib)\n')
        f.write('Rcl: [ ' + ',\n       '.join(', '.join(f'{x:10.6f}' for x in row) for row in R) + ' ]\n')
        f.write('Pcl: [ ' + ', '.join(f'{x:10.6f}' for x in P) + ' ]\n')
    print(f'\nwritten {os.path.join(a.out, "calib_result.txt")}')
    print(open(os.path.join(a.out, 'calib_result.txt')).read().split('\n', 3)[3])


if __name__ == '__main__':
    main()
