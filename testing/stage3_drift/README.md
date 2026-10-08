# Base-position drift of the leg-kinematics EKF: offline replay in simulation

Measurement of the drift of the base position estimated by `LegKinematicsEkf` (`mpc/ekf.py`, Bloesch et al. [1]) against
the simulator ground truth, for the stage-2 stepping-in-place gait, and its sensitivity to the filter noise parameters.
This is the offline validation described in `docs/PLAN.md`, Stage 3: record a run in Isaac, replay the filter without
Isaac, and compare x, y, z with the simulator.

## Contents

| File | Description |
|---|---|
| `scripts/record_truth.py` | Isaac run (same gait as `scripts/run_stepping.py`) that saves the controller inputs, the contact modes, the online estimate and the simulator base position per tick |
| `scripts/replay_ekf.py` | Offline replay of the filter on a recording, error table, figure and parameter sweeps; needs no Isaac |
| `drift_final.png` | Figure of the default-parameter replay of the three runs |
| `replay_results.txt` | Console output of the replay and the sweeps reported below |

The two scripts are in `scripts/` of the repository, not in this folder.

## Method

Three runs with env seeds 0, 1 and 2 (the sensor-noise realization), each 10 steps in place with the default gait
(`T_ss` 0.30 s, `T_ds` 0.10 s, clearance 2 cm), 200 Hz, 1884 / 1884 / 1891 ticks (9.42 / 9.42 / 9.46 s), no fall.
The controller sees only encoders, AHRS, gyro and accelerometer. The crouch starts at 0.5 s and the gait at 2.0 s.

The filter observes only, so replaying the recorded inputs with other noise parameters is valid open loop. With the
default parameters the replay reproduces the online estimate of the recorded run (maximum difference 0.000 mm).

The error is the estimated base displacement minus the simulator displacement since an anchor time $t_a$:

$$e(t) = \big[\hat r(t) - \hat r(t_a)\big] - \big[r(t) - r(t_a)\big], \qquad t_a = 0.5\ \mathrm{s}.$$

The anchor skips the first ticks. Anchored at the first tick, all runs show an identical jump of about 2 mm
horizontally and 6 mm vertically within the first 0.1 s. This is not drift; its cause was not examined. The error
contains the heading error of the AHRS but not the unobservable start position [1]. The simulator position is the root
of the articulation, the estimate is the base frame; their fixed offset cancels in the displacement to first order.

## Results

Default noise parameters (`accel_density` 0.1, `slip_density` 1e-6, `yaw_std` 1°, `kinematics_std` 2 mm). Error at the end
of the run in mm unless stated; `xy@gait` is the horizontal error when the gait starts (2.0 s), `growth` the slope of
the horizontal error while stepping, `travel` the horizontal displacement of the simulator base since the anchor.

| Seed | x | y | z | xy@gait | xy end | xy max | growth [mm/s] | travel |
|---|---|---|---|---|---|---|---|---|
| 0 | -0.5 | -2.8 | 0.0 | 0.9 | 2.9 | 2.9 | 0.21 | 24.9 |
| 1 | 0.1 | -2.4 | 0.0 | 0.9 | 2.4 | 2.4 | 0.19 | 27.1 |
| 2 | 0.6 | -2.8 | -0.2 | 0.7 | 2.9 | 2.9 | 0.23 | 36.2 |

![Drift of the estimated base position](drift_final.png)

Left: horizontal error over time (dashed: gait starts). Centre: vertical error. Right: direction of the error in the
horizontal plane (dot: end of run).

- The estimated base position stays within 2.4 to 2.9 mm horizontally and 0.2 mm vertically of the simulator over
  9 s. The horizontal error grows by about 0.9 mm during the crouch and by about 0.2 mm/s while stepping, with a
  ripple of about 1 mm at the step rhythm. The vertical error stays within ±0.8 mm, with peaks at touchdowns.
- The simulator base itself translates by 25 to 36 mm over the run, mostly in $+x$, although the gait is nominally in
  place; the distance differs between the seeds. This displacement is 9 to 13 times the estimation error.
- The horizontal error points the same way in all three seeds ($-y$, 2.4 to 2.8 mm, $x$ within 0.6 mm) although the
  travelled distance differs by 45 %. This suggests a systematic bias rather than sensor noise. Possible causes, none
  tested: the leg-kinematics model (contact point, leg asymmetry), the handling of the AHRS orientation, the frame
  offset between the estimate and the simulator root.

### Sensitivity to the noise parameters

Mean over the three runs of the horizontal error at the end of the run in mm (maximum over the run in brackets where
it differs). Defaults in bold; `yaw_std` in rad.

| Parameter | Values | Error [mm] |
|---|---|---|
| `accel_density` | 0.001, 0.01, **0.1**, 1 | 2.9 (3.1), 2.8, 2.7, 2.7 |
| `slip_density` | 1e-8, **1e-6**, 1e-4 | 2.7, 2.7, 3.0 |
| `yaw_std` | 0.0087, **0.0175**, 0.05 | 2.9, 2.7, 2.6 |
| `kinematics_std` | 0.001, **0.002**, 0.005 | 2.7, 2.7, 2.7 |

Changing any of the four parameters by two to three orders of magnitude (one at a time) changes the mean end error by
at most 0.4 mm (2.6 to 3.0 mm). The residual is therefore not a matter of tuning these noise parameters.

## Usage

```bash
uv run python scripts/record_truth.py --seed 0 --record runs/seed0.npz   # also seeds 1 and 2, one process each
uv run python scripts/replay_ekf.py runs/seed0.npz runs/seed1.npz runs/seed2.npz --plot drift_final.png
uv run python scripts/replay_ekf.py runs/*.npz --sweep accel_density 0.001 0.01 0.1 1
uv run python scripts/replay_ekf.py runs/*.npz --set slip_density=1e-5 --anchor 0.5
```

The controller must be built first (`scripts/build_controller.py`); the replay needs only numpy, pinocchio, casadi and
matplotlib.

## Limitations

- Simulation only, three seeds, one gait (10 steps in place, 9.4 s). The paper's figure of up to 10 % of the travelled
  distance on hardware [1] refers to long walks and is not comparable.
- Only the position is scored. The yaw error, the consistency of the filter covariance with the error and the
  bias-estimation option (`estimate_bias`) were not evaluated.
- The sweeps vary one parameter at a time around the defaults.
- The base translation of 25 to 36 mm during stepping in place was not analysed. `docs/PLAN.md` quotes about 0.3 mm
  forward landing drift per step; base displacement and foot landing position are different quantities, and the
  relation between them was not examined.

## References

1. M. Bloesch, M. Hutter, M. Hoepflinger, S. Leutenegger, C. Gehring, C. D. Remy and R. Siegwart, "State Estimation for
   Legged Robots: Consistent Fusion of Leg Kinematics and IMU", *Robotics: Science and Systems*, 2012.
