# FAST-Calib extrinsic results — Go2 (Hesai XT16 + RealSense D435i)

`T_cam_lidar`, i.e. `p_cam = Rcl * p_lidar + Pcl`. Same convention as
`FAST-LIVO2/config/go2_xt16.yaml`. All figures 2026-08-27.

> **STATUS: not installed.** `go2_xt16.yaml` still carries the older manual
> CloudCompare extrinsic. Nothing here is written into it yet — see
> [Before installing](#before-installing).

## Ranking by overlay cross-validation (preferred)

Each candidate extrinsic is scored on **every** scene by the overlay metric: the
percentage of board-*surface* LiDAR points that project inside an image hole. A
board point landing in a hole is unambiguously wrong, and the hole positions come
from the ArUco board pose, so the test does not reuse the LiDAR fit. Lower is better.

| extrinsic | s1 | s4 | s5 | s7 | s8 | s10 | **mean** |
|---|---|---|---|---|---|---|---|
| **consensus of 5,7,8,10** | 0.01% | 0.13% | 0.32% | 0.52% | 1.25% | 0.72% | **0.49%** |
| consensus of 7,8,10 | 0.02% | 0.21% | 0.88% | 0.54% | 1.19% | 0.77% | 0.60% |
| consensus of all 6 | 0.01% | 0.38% | 0.84% | 0.52% | 1.20% | 0.68% | 0.61% |
| **scene8** (best single) | 0.02% | 0.21% | 1.31% | 0.43% | 1.00% | 0.75% | **0.62%** |
| scene10 | 0.03% | 0.17% | 0.73% | 0.74% | 1.25% | 0.82% | 0.62% |
| scene7 | 0.07% | 0.18% | 1.14% | 0.41% | 1.05% | 0.94% | 0.63% |
| scene5 | 0.46% | 0.04% | 0.87% | 0.85% | 1.31% | 1.33% | 0.81% |
| manual (`go2_xt16.yaml`) | 0.07% | 1.16% | 1.60% | 0.86% | 1.15% | 0.81% | 0.94% |
| scene1 | 0.08% | 0.45% | 2.98% | 0.46% | 1.53% | 0.72% | 1.04% |
| scene4 | 1.22% | 0.18% | 0.85% | 0.96% | 1.30% | 2.07% | 1.10% |

**Best single scene: scene8**, but scene8 / scene10 / scene7 are 0.62 / 0.62 / 0.63%
— a three-way tie inside the metric's noise. Any consensus of the good scenes beats
every individual scene.

**This ranking supersedes the agreement-based one below**, which measures only
distance from the group mean — a scene can sit near the mean and still be wrong, so
that criterion is partly circular. The two agree that scene7/8/10 are the good ones
and scene4 is poor, but they disagree sharply on scene1:

- **scene1 is the worst calibration output here (1.04%), behind even the manual
  extrinsic.** It fits its own scene best of anything (0.08%) and generalises worst
  — the signature of an overfit. Its 0.8° board tilt is the most nearly
  fronto-parallel geometry in the set, which constrains the out-of-plane DOF least.
  Tilt the board.

### Recommended value

Consensus of scenes 5, 7, 8 and 10 — 1.60° from nominal, `Pcl[1]` -49.2 mm vs the
URDF's -61.4 mm:

```
Rcl: [ 0.009513, -0.999852, -0.014363,
      -0.021978,  0.014151, -0.999658,
       0.999713,  0.009825, -0.021840]
Pcl: [ 0.021068, -0.049186, -0.209629]
```

It halves the manual extrinsic's error on this metric (0.49% vs 0.94%).

## Ranking by agreement with the consensus

Ranked by distance from the consensus of all six valid scenes, scoring rotation
and translation on one scale (1° ≈ 17.5 mm of reprojection at 1 m):
`score = Δrot[°] + Δtrans[mm] / 17.5`. Lower is better.

| # | scene | score | Δrot | Δtrans | lock margin | range | tilt | off-square | rings/hole |
|---|---|---|---|---|---|---|---|---|---|
| 1 | **scene7** | 0.77 | 0.56° | 3.7 mm | 1.40× | 1.33 m | 7.6° | 10.5° | 3.4 |
| 2 | **scene8** | 1.40 | 0.94° | 8.0 mm | 1.66× | 1.33 m | 7.2° | 23.5° | 3.4 |
| 3 | **scene10** | 1.72 | 0.95° | 13.5 mm | 1.21× | 1.04 m | 11.3° | 2.7° | 4.4 |
| 4 | **scene1** | 2.38 | 1.48° | 15.8 mm | 1.31× | 1.04 m | 0.8° | 5.5° | 4.4 |
| 5 | **scene4** | 3.22 | 1.22° | 35.0 mm | 1.37× | 1.54 m | 7.2° | 11.7° | 3.0 |
| 6 | **scene5** | 3.61 | 1.71° | 33.2 mm | 1.64× | 1.63 m | 10.1° | 4.9° | 2.8 |

*Lock margin* = worst per-hole annulus support ÷ expected. The aliasing guard
rejects below 0.45×; everything here clears it by a wide margin.

## Consensus

Chordal-mean rotation and per-axis median translation over all six:

```
Rcl: [ 0.009747, -0.999882, -0.011901,
      -0.017313,  0.011731, -0.999781,
       0.999803,  0.009951, -0.017197]
Pcl: [ 0.021068, -0.055470, -0.209629]
```

- **1.32°** from the nominal axis-aligned `Rcl [0,-1,0, 0,0,-1, 1,0,0]`.
- `Pcl[1] = -55.5 mm` vs `go2_description`'s URDF **-61.4 mm** — agreeing to 5.9 mm.
- `Pcl[2]` is the most stable component: -200 to -213 mm across all six, -210 ±3 mm.

## Per-scene results

### 1. scene7 — 0.56° / 3.7 mm from consensus

```
Rcl: [ 0.008857, -0.999957, -0.002765,
      -0.013874,  0.002642, -0.999900,
       0.999865,  0.008894, -0.013850]
Pcl: [ 0.017455, -0.055863, -0.210340]
```

| | |
|---|---|
| bag | `calib_data/go2_scene7_0827_153327` |
| config | `config/qr_params_go2_scene7_fix.yaml` |
| `marker_corner_shift` | `[0,0,3,0]` |
| board range / tilt / off-square | 1.33 m / 7.6° / 10.5° |
| ring pitch → crossings per D160 hole | 3.4 |
| vertical coverage | 579 mm of the 546 mm board |
| per-hole annulus support | 333 / 339 / 265 / 293 (expect ~189 each, guard floor 85) |
| vs nominal `Rcl` | 0.96° |

### 2. scene8 — 0.94° / 8.0 mm from consensus

```
Rcl: [ 0.002504, -0.999995,  0.001845,
      -0.011924, -0.001875, -0.999927,
       0.999926,  0.002482, -0.011929]
Pcl: [ 0.026720, -0.060345, -0.212599]
```

| | |
|---|---|
| bag | `calib_data/go2_scene8_0827_153612` |
| config | `config/qr_params_go2_scene8_fix.yaml` |
| `marker_corner_shift` | `[0,0,3,0]` |
| board range / tilt / off-square | 1.33 m / 7.2° / 23.5° |
| ring pitch → crossings per D160 hole | 3.4 |
| vertical coverage | 606 mm of the 546 mm board |
| per-hole annulus support | 277 / 308 / 228 / 264 (expect ~137 each, guard floor 62) |
| vs nominal `Rcl` | 0.71° |

### 3. scene10 — 0.95° / 13.5 mm from consensus

```
Rcl: [ 0.004291, -0.999740, -0.022395,
      -0.029039,  0.022261, -0.999330,
       0.999569,  0.004938, -0.028936]
Pcl: [ 0.024682, -0.042509, -0.208918]
```

| | |
|---|---|
| bag | `calib_data/go2_scene10_0827_163858` |
| config | `config/qr_params_go2_scene10.yaml` |
| `marker_corner_shift` | `[0,0,0,0]` |
| board range / tilt / off-square | 1.04 m / 11.3° / 2.7° |
| ring pitch → crossings per D160 hole | 4.4 |
| vertical coverage | 545 mm of the 546 mm board |
| per-hole annulus support | 390 / 349 / 293 / 320 (expect ~242 each, guard floor 109) |
| vs nominal `Rcl` | 2.11° |

### 4. scene1 — 1.48° / 15.8 mm from consensus

```
Rcl: [-0.004217, -0.999955,  0.008472,
      -0.009670, -0.008431, -0.999917,
       0.999944, -0.004298, -0.009634]
Pcl: [ 0.036444, -0.055076, -0.213097]
```

| | |
|---|---|
| bag | `calib_data/go2_scene1_0827_132226` |
| config | `config/qr_params_go2_scene1_fix.yaml` |
| `marker_corner_shift` | `[0,0,3,0]` |
| board range / tilt / off-square | 1.04 m / 0.8° / 5.5° |
| ring pitch → crossings per D160 hole | 4.4 |
| vertical coverage | 545 mm of the 546 mm board |
| per-hole annulus support | 405 / 372 / 346 / 327 (expect ~250 each, guard floor 112) |
| vs nominal `Rcl` | 0.78° |

### 5. scene4 — 1.22° / 35.0 mm from consensus

```
Rcl: [ 0.024602, -0.999447, -0.022391,
      -0.006511,  0.022237, -0.999732,
       0.999676,  0.024741, -0.005960]
Pcl: [-0.008925, -0.073120, -0.205709]
```

| | |
|---|---|
| bag | `calib_data/go2_scene4_0827_152557` |
| config | `config/qr_params_go2_scene4_fix.yaml` |
| `marker_corner_shift` | `[0,0,3,0]` |
| board range / tilt / off-square | 1.54 m / 7.2° / 11.7° |
| ring pitch → crossings per D160 hole | 3.0 |
| vertical coverage | 565 mm of the 546 mm board |
| per-hole annulus support | 245 / 226 / 270 / 240 (expect ~165 each, guard floor 74) |
| vs nominal `Rcl` | 1.94° |

### 6. scene5 — 1.71° / 33.2 mm from consensus

```
Rcl: [ 0.022130, -0.999169, -0.034217,
      -0.033233,  0.033472, -0.998887,
       0.999203,  0.023243, -0.032465]
Pcl: [-0.002908, -0.034537, -0.200064]
```

| | |
|---|---|
| bag | `calib_data/go2_scene5_0827_152851` |
| config | `config/qr_params_go2_scene5_fix.yaml` |
| `marker_corner_shift` | `[0,0,3,0]` |
| board range / tilt / off-square | 1.63 m / 10.1° / 4.9° |
| ring pitch → crossings per D160 hole | 2.8 |
| vertical coverage | 575 mm of the 546 mm board |
| per-hole annulus support | 228 / 242 / 296 / 259 (expect ~139 each, guard floor 63) |
| vs nominal `Rcl` | 3.00° |

## Rejected

| scene | bag | reason |
|---|---|---|
| scene2 | `go2_scene2_0827_134836` | vertical clipping — per-hole support 116/**0**/**0**/113, two hole annuli outside the LiDAR's FOV |
| scene3 | `go2_scene3_0827_135418` | vertical clipping — support **36**/127/397/127 |
| scene6 | `go2_scene6_0827_153117` | **camera** side: `[Mono] Unable to find a candidate set that matches target's geometry` at 26.5° off square. Its LiDAR half locked cleanly (230/201/217/252). scene8 passed at 23.5°, so the camera-side limit sits between 23.5° and 26.5°. |
| scene9 | `go2_scene9_0827_153807` | bag contains **no `/lidar_points` at all** — 109 images, 0 scans. Driver was not publishing. |

## Seeing the result

`output_<scene>/colored_cloud.pcd` is the LiDAR cloud coloured by projecting each
point into the camera image with that scene's extrinsic:

```bash
pcl_viewer ~/fastlivo_ws/src/FAST-Calib-ROS2/output_scene7_fix/colored_cloud.pcd
```

More legible for judging alignment are the reprojection overlays in
`output_overlays/` — the LiDAR cloud drawn on the photo, coloured by range (blue =
board surface, red = background seen through the holes):

| file | shows |
|---|---|
| `scene7_compare.png`, `scene10_compare.png` | zoomed side-by-side, consensus vs the current manual extrinsic |
| `scene{7,10}_consensus.png` | full frame, consensus extrinsic |
| `scene{7,10}_manual.png` | full frame, current `go2_xt16.yaml` extrinsic |

**What to look for:** every red (far) run must stay *inside* a hole disc, and every
blue (near) run must stay on solid board. In `scene7_compare.png` the manual
extrinsic pushes red runs past the right rim of the bottom-left hole and above the
top-right hole; the consensus keeps them contained.

The same thing counted rather than eyeballed — board-surface points that project
inside an image hole, which should never happen:

| extrinsic | scene7 | scene8 |
|---|---|---|
| consensus | **62** | **94** |
| scene10's own | 97 | 163 |
| manual (`go2_xt16.yaml`) | 136 | 194 |

Regenerate for any scene with `scratchpad/overlay.py` (see git history of this file's
commit, or ask). While `fast_calib` is running it also publishes the coloured cloud
on `/colored_cloud`, viewable in RViz2 or Foxglove.

## Caveats

1. **RMSE is meaningless with the template fit.** It reads ~1e-4 regardless,
   because both point sets are then exact rectangles and several rigid transforms
   fit all four pairs. Judge a scene on per-hole annulus support and on agreement
   with the other scenes, never on RMSE.
2. **The board's bottom-left ArUco (id 3) was mounted 90° clockwise** for scenes
   1–9. `estimatePoseBoard` matches corners by index, so this silently biased
   those results by ~4.7° / 13 mm until corrected. Every number above already has
   `marker_corner_shift: [0,0,3,0]` applied. The marker was physically rotated
   back before scene10, which is the only scene needing no correction.
3. **A `NO LOCK` before 2026-08-27 meant nothing about the recording.** The
   template's coarse search was centred on the aligned frame's origin rather than
   the board, so any tilted board fell outside its ±150 mm window. Scenes 4, 5, 7
   and 8 all reported `NO LOCK` and all lock after the fix.
4. Each scene constrains the extrinsic with only four point correspondences, so
   single-scene results are weakly determined — particularly in-plane (x, y) and
   rotation. Vary board position and tilt between scenes.

## Before installing

`Pcl[1]` still spreads from -34 to -73 mm across the six scenes. The top three
(scene7/8/10) agree to 0.95° and 13.5 mm, which is close to but not yet at the
~1° / ~1 cm bar. Record 3–4 more scenes at **1.0–1.35 m**, under **~20° off square** (scene6 failed at 26.5°, scene8 passed at 23.5°), centred on the LiDAR, with varied placement — then recompute the
consensus and install that, not any single scene.

Confirm `/lidar_points` is publishing before each recording (see scene9).

## Reproducing

```bash
source /opt/ros/foxy/setup.bash && source ~/fastlivo_ws/install/setup.bash
timeout -s INT 90 ~/fastlivo_ws/install/fast_calib/lib/fast_calib/fast_calib \
  --ros-args -r __node:=fast_calib \
  --params-file ~/fastlivo_ws/src/FAST-Calib-ROS2/config/qr_params_go2_scene7_fix.yaml
```

The node never exits on its own, hence `timeout -s INT`. `-r __node:=fast_calib`
is required — the YAML's root key is `fast_calib`, but the binary names itself
`mono_qr_pattern`.
