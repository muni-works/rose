# Humanoid shoulder (J1) heat: open ring + conduction, no fans

Cam's call (2026-10-21): "i don't want fans tbh". It replaces the earlier B + A (fan + conduction).

## How J1 is cooled

The humanoid's J1 is the lymb v10 shoulder pitch, an RS04, inside the torso corner. Its output face is flush on the flank at |y| = 186.6.

- **Conduction:** J1 mounts by its back face, through the chest adapter (120 mm square, chamfered), onto the 6.35 mm 5052 torso side plate, on a gap pad (about 3 W/mK).
- **Open ring:** the arm's graphite open ring on the J1 face (OD 133 / ID 123, arm-owned) puts the motor face in air. It breathes through a Ø135 opening in the flank, framed by an inlaid gunmetal bezel (`trim-j1ring`, r 68.5–73.5). Keep about 10 mm of the flank around it clear.
- **Corner:** the crown and the armpit flare clear r 88 about the J1 axis (adapter corners at r 85, plus a 3 mm gap). No fans and no vents.
- **Arm's lumped thermal model:** plate + open ring holds 26.9 N·m continuously, i.e. **1.56 kg arms-forward indefinitely**. J2, at 1.27 kg rated, stays the arm's limit.

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

Thermal protection, per joint.

| Item | Value |
|---|---|
| Continuous J1 | ≤ 26.9 N·m (about 1.56 kg arms-forward), from the arm model; check it on the elbow rig |
| Above that | An I²t winding-temperature estimate, using the RS04's own winding sensor as the truth. Ramp the torque limit down over 30 s and tell the planner to lower the arms. |
| Normal work | Rated 1.27 kg, which J2 limits, never reaches J1's derate |
