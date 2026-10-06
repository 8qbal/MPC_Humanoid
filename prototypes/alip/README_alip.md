# ALIP footstep prototype for Robonion (offline, no Isaac)

Files: `alip_prototype.py` (run it), `fig1_nominal_walk.png`, `fig2_lateral_push.png`, `fig3_servo_feasibility.png`, `results.json`.

Run: `python3 alip_prototype.py out` (needs numpy + matplotlib; takes about 2 seconds).

## What it does

It simulates the Angular-Momentum Linear Inverted Pendulum (ALIP) of Gong & Grizzle for a walking Robonion and lets the ALIP footstep rule decide where each swing foot lands. It answers four questions with numbers:

1. Does the ALIP rule give a steady straight walk at a commanded speed? (fig 1)
2. How big a push can the rule recover from by stepping? (fig 2, table below)
3. Can the servos swing the leg fast enough for 0.3 to 0.4 m/s? (fig 3, table below)
4. How much sideways error does a heading error cause?

## Where each equation comes from

| Part | Source |
|---|---|
| Sagittal dynamics, closed-form step map, one-step prediction, foot placement rule | **Paper**, arXiv:2008.10763, Eq. 7, 8, 14, 15 (I fetched and compared them) |
| Lateral axis (y, L' = -Lx) | **My derivation** from L = r x m v at constant height (checked by the periodic-orbit test) |
| Steady-state desired momentum for a given speed and step width | **My derivation**, checked numerically: the simulation settles on exactly step length = v*T, step width = w, speed = v |
| Instant foot switch with continuous angular momentum | **Paper's** "invariance under impacts" claim, used as an assumption |

## Assumptions you must replace with real values

| Quantity | Value used | Where it came from |
|---|---|---|
| Mass | 7.933 kg | sum of link masses in `assets/robonionv2_controller.urdf` (my sum) |
| CoM height H | 0.48 m | constant in Iqbal's `nmpc.py`; the crouched pose is lower, ask Iqbal |
| Step period T | 0.4 s (0.3 swing + 0.1 double support) | Iqbal's gait parameters; double support is folded in |
| Foot spacing w | 0.11 m | 2 x the 0.055 m hip-yaw offset in `joint_info.md`; not measured |
| Reach limits | 0.20 m forward, 0.07 to 0.25 m sideways | **my guess** from 0.2 m + 0.2 m parallelogram bars |
| Leg length for joint speed | 0.38 m | **my guess**; ignores knee flexion and foot lift |

## Results (default assumptions)

**Check that the code does what the maths says:** at 0.3 m/s it settles on step length 0.120 m (= 0.3 x 0.4), step width 0.110 m, speed 0.300 m/s. This checks the code against the equations, not against the robot.

**Max recoverable push impulse by stepping (N s), push 0.15 s after touchdown of step 6, walking at 0.2 m/s:**

| Reach assumption | push forward | push backward | push toward stance side (+y) | push toward swing side (-y) |
|---|---|---|---|---|
| narrow 0.15 m | 0.93 | 3.07 | 0.55 | 0.55 |
| medium 0.25 m | 1.70 | 4.04 | 0.75 | 1.74 |
| wide 0.35 m | 1.70 | 4.53 | 0.94 | 2.93 |

Iqbal's current stepping-in-place limits (from his `PLAN.md`) are about +1.75 / -1.0 N s sagittal and +-0.5 / +1.0 N s lateral. The numbers are the same order of magnitude, so **this does not show that ALIP beats the current robot.** The model has no ankle strategy, no weight-shift phase and idealised feet, and the lateral result depends strongly on the reach you assume. It only says "ALIP stepping could add a comparable amount of push recovery if the legs can reach", which Isaac has to confirm.

**Servo speed needed (rough, +-30%), limit 4.08 rad/s:**

| Speed | Step length | Peak joint speed | Verdict |
|---|---|---|---|
| 0.1 m/s | 0.04 m | 1.1 rad/s | fine |
| 0.2 m/s | 0.08 m | 2.2 rad/s | fine |
| 0.3 m/s | 0.12 m | 3.3 rad/s (80%) | fits |
| 0.4 m/s | 0.16 m | 4.3 rad/s (106%) | **just over the limit** |

Changing the gait timing barely helps: at 0.4 m/s a period of T = 0.45 s still needs 4.2 rad/s, and T >= 0.5 s needs steps longer than the 0.2 m reach. So the 0.3 to 0.4 m/s target in Iqbal's plan is **at the edge of what these servos allow**; 0.3 m/s looks realistic, 0.4 does not unless the real leg geometry is better than my estimate.

**Open-loop footsteps (fixed landing points) fall within 3 steps even with no push.** The pendulum is unstable (growth about 6x per step), so some foot-placement feedback is needed. The real robot also stabilises with ankle and posture control, so this is not a fair comparison to the current robot.

**Heading error to sideways error (geometry):** a heading bias of 0.5 / 1 / 2 / 3 / 5 deg gives 4.4 / 8.7 / 17.5 / 26.2 / 43.7 cm sideways error over a 5 m walk.

## What this means for "make it walk straight"

- ALIP controls **momentum**, not position or heading. After a 0.5 N s lateral push the walk recovers but stays **about 6 cm off the original line** (fig 2). To walk straight you need an extra outer loop on sideways position and on heading.
- Heading (yaw) is exactly what the Bloesch filter cannot measure (Iqbal's `ekf.py` uses the AHRS yaw for it). A 1 degree heading error is about 9 cm per 5 m. So the real accuracy limit for straight walking is the heading sensor, not ALIP.
- ALIP has no yaw state, and Robonion's model has none either. Heading control needs the hip-yaw joint, which is frozen at default today.

## How to hand it to Iqbal

`landing_point(p, sag, lat, t, side, v_des)` in the script is the only function the real controller needs. It takes the CoM state relative to the stance foot and returns where to land the swing foot. It would be called from `gait.py` once forward walking exists (his stage 4).

## Not done and not verified

- Not connected to the NMPC or Isaac; no result here comes from the real model or the real robot.
- Equations were compared with the paper through a fetched text version, not a typeset PDF. Check the signs of Eq. 7, 8, 14, 15 in the PDF yourself before presenting.
- The servo check is a hand estimate; the right check is inverse kinematics with the robot's real leg (the controller URDF, with Pinocchio).
