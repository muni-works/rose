# Humanoid shoulder (J1) heat: keep it simple

Cam's call (2026-10-21): no fans and no ducts.

## How J1 is cooled

The humanoid's J1 is the lymb v10 shoulder pitch, an RS04. It sits inside the torso corner, with its output face flush on the flank at |y| = 186.6.

- **Conduction:** J1 mounts by its back face, through the 41 mm chest adapter, onto the 6.35 mm 5052 torso side plate. That's the same path the desk arm uses. Use a gap pad at the joint at most, and no added parts.
- **Controller derate:** long holds are limited in software.
- **Open air:** the corner is closed. The flank shows a gunmetal bezel ring around the J1 face (`trim-j1ring`).

## Heat by pose

Per side. Copper loss is taken as P = 87 W × (τ / 40 N·m)². These are from the arm model and are rough.

| Pose | J1 torque | Heat |
|---|---:|---:|
| Arms hanging, or T-pose (J2 carries the abduction load) | ~0 N·m | ~0 W |
| Arms forward, no payload | 13.4 N·m | ~10 W |
| Arms forward, 1.0 kg | 21.4 N·m | ~25 W |
| Arms forward, 1.27 kg (rated) | 23.5 N·m | ~30 W |

A passive vent can't shed 30 W from the corner: it would need about 18,600 mm² of grille per side. The plate path plus the motor's thermal mass is what's available.

## Derate (controller)

| Item | Value |
|---|---|
| Continuous J1 limit | **10 W per side (about 13.5 N·m)**: arms forward unloaded, indefinitely |
| Above 10 W | Track an I²t (winding-temperature) estimate per joint. Hold time is limited, not torque. |
| Rated hold (23.5 N·m, about 30 W) | **about 5 min**, then ramp the torque limit down to the continuous level over 30 s and tell the planner to lower the arms or set the load down |
| Recovery | Full hold budget back after about 10 min at ≤ 10 W |

**Basis:** about 350 J/K of motor thermal mass with the plate's conduction. That gives a 40 K winding-temperature rise in about 8 min at 30 W. The 5 min figure keeps margin until the arm's lumped thermal model (plate-only case) replaces these numbers. Update this table when it lands.

**Behaviour:** arms-forward with the rated 1.27 kg is a minutes-long pose, not a continuous one. Carrying should be done with the elbows in (lower J1 torque), or by setting the load down.
