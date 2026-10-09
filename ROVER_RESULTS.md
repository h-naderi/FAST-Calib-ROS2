# FAST-Calib results — Rover Zero 3 (RoboSense Airy + ZED 2i left lens)

`T_cam_lidar`, i.e. `p_cam = Rcl * p_lidar + Pcl`, camera = `camera_left_camera_frame_optical`.
Recorded 2026-10-09, 18 scenes in two rooms, Go2 half-scale board, ZED at 2208x1242.

## Result (installed as `src/fastlivo/config/zed_calib.yaml` in wpm-rover)

```yaml
Rcl: [ -0.000257, -0.999998,  0.001974,
        0.383879, -0.001921, -0.923381,
        0.923383,  0.000520,  0.383879 ]
Pcl: [  0.057236, -0.044236, -0.014232 ]
```

One Kabsch fit over all 68 hole-centre pairs of the 17 usable scenes
(`scripts/joint_solve.py`, output in `output_rover_joint/`). Reproduce with:

```bash
python3 scripts/joint_solve.py config/qr_params_rover_rover_scene*.yaml \
    --prior ../fastlivo/config/rover_airy_zed.yaml --holes
```

| | URDF-assembled (`rover_airy_zed.yaml`) | **joint calibration** |
|---|---|---|
| board points landing in an image hole, mean of 17 | 4.12% (worst 7.67%) | **0.22%** (worst 0.73%) |
| same, each scene held out of the fit | — | **0.24%** |
| uncertainty (bootstrap 68%) | — | **0.43°, 7.7 mm** (~2 px at 640x360) |
| offset from the joint answer | 1.78° (1.6° pitch, 0.8° roll), 21 mm: **8–10 px at 640x360** | — |

Visual check: `output_rover_overlays/` (`contact_sheet.jpg`, per-scene
`_board_compare.jpg` prior | calibrated, `_overlay.jpg` full frame). Blue = board,
red = background through a hole; blue inside a white hole outline is misalignment.

## Scenes

| scene | note |
|---|---|
| rover_scene1–10, 13–18 | used |
| rover_scene112 | scene 12 (misnamed at recording time), used |
| rover_scene11 | **excluded**: 3 of 4 markers detected |

Boards 1.1–1.7 m away, square-on, turned, leaned and raised/lowered. Crop boxes were
fitted by `scripts/auto_box.py` (box purity 83–95%), `plane_dist_threshold: 0.02`.
Every scene locked, every hole with per-hole annulus support well above expected.

## What matters on this LiDAR

1. **A single scene is not enough.** Per-scene results scatter 0.5–4.4° (median ~2.5°)
   around the joint answer. The Airy's rings each read range with a fixed offset (on
   the board: −21 to +23 mm, std 10.7 mm) that does not average out over time — 131
   scans of one beam scatter only 3.9 mm — so the board looks 13 mm thick and its
   fitted plane tilts by ~0.7° per scene. Only different rings crossing the board,
   i.e. different board heights and distances, average it out:
   4 scenes ≈ 0.87°, 8 ≈ 0.42°, 12 ≈ 0.25° from the 17-scene answer.
2. **Fit jointly; don't average per-scene results** (the Go2 method). With every board
   at 1–1.7 m a scene trades rotation against translation; averaging keeps that bias.
   Here: joint 0.22% holes / 11.2 mm leave-one-out vs averaged 0.44% / 13.1 mm.
3. RMSE printed by the node is ~0 by construction with the template fit; ignore it.
4. The black markers also read at a different range from the white foam (intensity
   range-walk), visible as a few mm at the board corners. Minor next to (1).

## Before trusting it on another robot

This is **this rover's** extrinsic. The Houston robot's bag has a different ZED
(cx 5 px off — see `camera_houston_zed2i.yaml` in wpm-rover) and its own mounting;
calibrate it the same way rather than reusing these numbers.
