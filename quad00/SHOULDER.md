# Humanoid shoulder (J1) heat: conduction + fan

Cam's call (2026-10-21, given directly; it replaces an earlier relayed "keep it simple"): **B + A**, conduction into the chest frame plus a small fan per corner, with the open-ring J1 face.

## How J1 is cooled

The humanoid's J1 is the lymb v10 shoulder pitch, an RS04. It sits inside the torso corner, with its output face flush on the flank at |y| = 186.6.

- **Conduction (B):** J1 mounts by its back face, through the 41 mm chest adapter, onto the 6.35 mm 5052 torso side plate, on a gap pad (about 3 W/mK).
- **Fan (A):** a 40 × 40 × 10 mm fan in each corner (`elec-j1fan.l/r`), behind the drum, between it and the chest's back skin. It moves about 1.3 L/s.
- **Air path:**
  - in through the arm's open graphite ring on the J1 face (OD 133 / ID 123, arm-owned), which breathes through a Ø135 opening in the flank
  - across the can
  - out through a 34 × 34 mm dot grid on the chest's back, behind each shoulder
- **Look:** a gunmetal bezel ring (`trim-j1ring`, r 68.5–73.5) is inlaid flush in the flank around the opening.
- **Corner (lymb v10.3 interface):** the flank sits at |y| 186.6. The crown and the armpit flare clear r 88 about the J1 axis, for J1's 120 mm-square chest adapter (corners at r 85) plus a 3 mm gap.
- **Arm's thermal model (lumped, 2026-10-21):** with the plate plus the open ring, J1 holds 26.9 N·m continuously, i.e. 1.56 kg arms-forward indefinitely. With the fan added it's 33.1 N·m. The fan is margin, not a need; the J2 joint stays the limit.

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

Kept as a safety net, now that the fan carries the steady state.

| Item | Value |
|---|---|
| With the fan running | about 1.3 L/s at ΔT 20 K carries about 30 W, so the rated arms-forward hold can be continuous. The fan curve and real flow are **TBD**. |
| Fan failed or blocked | Fall back to 10 W per side continuous, with an I²t winding-temperature estimate above that: about 5 min at the rated 23.5 N·m, then ramp down over 30 s |
| Monitoring | Read the fan tach. Use the RS04's own winding temperature as the I²t truth. |

Replace these numbers with the arm's lumped thermal model (plate + fan case) when it lands.
