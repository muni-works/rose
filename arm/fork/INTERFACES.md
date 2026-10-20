# Fork joint interfaces and RobStride mechanical answers

Sources: vendor STEP and user manuals, `~/Developer/robstride_assets` (clone of
github.com/RobStride/Product_Information, pulled 2026-10-13). **The manual
drawing (RS06 manual p. 8) is authoritative; it corrects the STEP-only reading
below** — the STEP hole grouping counted the side lugs as a front bolt pattern.

## v9 motors per the manual drawings (RS04 p. 9, RS02 / RS00 p. 8)

| motor | where | nose ahead of the front face | front (stator) | back face | output |
|---|---|---|---|---|---|
| RS04 | J1, J2, J4 | Ø98 x 13.2 (52.2 overall, Ø120 body) | 10x M4 on Ø106 | 10x M4 ▼6 on Ø106 | 8x M5 ▼6.5 on Ø36, 2x Ø5 dowels on Ø35.86 |
| RS02 | J5, J6 | Ø65 x 28 to the flange (41.5 overall, Ø78.5 body) | 9x M3 ▼8 on Ø73 | 4x M4 on Ø52 | 6x M4 on Ø24, 3x Ø4 dowels, Ø43 flange |
| RS00 | J7 | Ø35 x 1 (51 overall, Ø57 body) | 6x M3 on Ø50 | 4x M3 ▼4 on Ø38 | 6x M3 ▼5 on Ø27 |

Thread depths for the RS02 back face and output are not on its drawing; `fab.py` assumes 6 mm.

**v9 mounts every motor by its back face.** The RS04 and RS02 noses are long (13 and 28 mm)
and wide, so a front-face mount would push the holder's plate A far off the output. Back-mounted,
plate B carries the motor and the sleeve; plate A is a clearance plate round the nose (bore =
nose + 4 mm), its outer face 1 mm short of the output. The straddler's plate A bolts to the output
through a 3 mm spacer disc. Every A and B gap in the stack is 4 mm.

## RS06 per the manual drawing

| interface | pattern | turns? |
|---|---|---|
| output flange | 6x M4 deep 6 on Ø24 ±0.1, 3x Ø4 +0.05 dowel holes deep 4, Ø14 +0.05 pilot bore, flange 1.5 ±0.1 proud | yes |
| stator front | 8x M3 deep 8.5 on Ø82 ±0.2, Ø52 −0.05 centring spigot, 3 ±0.1 long | no |
| back cover | Ø70, inner bolt pattern (Ø62 per STEP) | no |
| length | 49 ±0.5 overall, 33 ±0.5 to the lug face | |

## Mechanical answers (what the email would have asked)

**Fault / power-off behaviour.** Manual §4.2.5: *post-power-off anti-backdrive
protection*, on by default (`damper` = 0; set 1 to disable). Turning the motor
fast while unpowered produces a damping torque; RobStride's stated purpose is to
stop back-EMF surges. That is the shorted-phase braking the power-cut sim asked
for. **Unknown: whether it engages at the slow speed of a falling arm** — the
manual says "rotated rapidly". Rig test #1.

**Torque constant and resistance (RS06).** Kt = 1.09 N·m/A rms at the output;
back-EMF 7.6 V rms/krpm (motor side); peak phase current 57 A, rated 14.3 A.
Phase resistance is not published; from the T-N curve (430 → 280 rpm at 57 A,
48 V) R_phase ≈ 0.15 Ω. Shorted-phase output damping B = Kt² / (3 R) ≈ 2.6
N·m·s/rad. The sim used 3.3 — same order; it now has a sourced number.

**Thermal: holding is the binding case.** Rated 11 N·m is *rotating* at 100 rpm
on a 130 x 155 mm aluminium heat sink. **Stalled (holding a pose) the continuous
rating is 8 N·m**: 11 N·m for 200 s, 17 for 15 s, 20 for 11 s, 36 for 1 s.
Rotating: 13 N·m for 1360 s, 15 for 280 s, 18 for 70 s, 23 for 21 s.

**Output bearing.** RS06 output bearing ≈ 35 x 47 mm (STEP part `1_2_04_000326`
— a 61807-class thin-section deep-groove bearing: C ≈ 4.4 kN, C0 ≈ 3.4 kN); rear
support a 634 (4 x 16 x 5). RS04 confirmed 61810 (47 balls Ø3.76 on Ø57.5);
RS03 a 6809. In the fork the child is carried on BOTH sides (output flange and
the sleeve bearing), so the output bearing sees about half the joint's radial
load and almost no moment: ~20 N at the elbow at 2.5 kg, against 3.4 kN static.
Not a constraint. A single-sided (cantilever) mount would put ~2 N·m of moment
on one thin-section bearing — also fine at this payload, but the fork removes it.

**Back cover as a structural mount.** The fork's parent plate B and the sleeve
boss bolt to it. Loads are tens of newtons; M4 into aluminium carries ~1 kN
each. Not a constraint — but it is a die-cast cover, so use the full thread
depth and a washer plate, and check on the rig.

**Rotor inertia.** Not published. Estimate from the 82 mm frame: J ≈ 5e-5 to
1e-4 kg·m² (the sim uses 1e-4); reflected through 9:1, 0.004 – 0.008 kg·m².
Measure on the rig (step torque, no load).

## What this means for the arm

Holding straight out at full reach (783 mm), using the 8 N·m stall rating:

| joint | continuous hold | peak |
|---|---:|---:|
| shoulder (with gas spring) | 1.0 kg | 4.7 kg |
| **elbow** | **0.5 kg** | 6.4 kg |
| wrist pitch | 1.3 kg | 6.0 kg |

**The elbow binds.** It carries 5.6 N·m of its own forearm before any payload.
Options, cheapest first: an elbow counterbalance spring (like the shoulder's);
rate the product by pose (straight-out reach is the worst case; most work is
bent); an RS03 at the elbow (60 N·m peak, heavier). Peak payload is not the
problem — continuous holding is. The old "1.5 kg continuous" from the sim used
the 11 N·m rotating rating and is superseded.

The rated figures assume a 130 x 155 mm heat sink. The fork's parent plates
clamp the stator at both ends and are about that size — keep them solid round
the stator ring for heat.

---

# Original STEP-only reading (kept for the record)

Measured 2026-10-13 from the vendor STEP files
(github.com/RobStride/Product_Information, `RS06-new.step`, `RS00.step`) by
pulling every hole-sized circle and grouping by bolt circle and face. Motor
axis is z; the output face is at the low-z end. Re-check against the printed
drawing in the user manual before cutting metal.

## RS06

| face | z (mm) | pattern | turns? | fork use |
|---|---:|---|---|---|
| output flange | −1.5 | 6× M3 + 6× M4 on Ø24, flange Ø52 | yes | child plate A, via spacer disc |
| stator front ring | 0 | 16× M4 on Ø86 (through to z 33) | no | parent plate A |
| back cover | 47–49 | 4× M4 on Ø62 | no | parent plate B + sleeve boss |

## RS00

| face | z (mm) | pattern | turns? | fork use |
|---|---:|---|---|---|
| output flange | −5 | 6× M4 on Ø17, 6× M4 on Ø28, flange Ø57 | yes | child plate A, via spacer disc |
| stator front ring | −5 … 0.5 | 6× M2.5 on Ø50, 6× M4 on Ø53 | no | parent plate A |
| back cover | 45–46 | 4× M3 on Ø51 | no | parent plate B + sleeve boss |

## What this confirms and what it changes

**Confirmed.** The stator can be held at both ends: the parent's two plates bolt
to the front ring and the back cover, so the motor is a stiff member of the
parent link rather than a cantilever. The back pattern does not turn, so the
sleeve boss bolts there.

**Changed.** On the RS06 the output flange stands only 1.5 mm proud of the
stator ring. Parent plate A sits on that ring, so child plate A cannot reach
the flange directly. A **spacer disc** (Ø48, 3 mm parent plate + 2 mm running
gap → 5 mm thick, 6× M4 on Ø24) passes through a Ø56 clearance hole in parent
plate A. The child plate bolts to the disc. The RS00 needs the same detail
(Ø54 disc, Ø61 clearance hole).

**Cable entry.** The motor has no bore. The cable comes off the parent's inner
face, enters the sleeve boss through a side slot between parent plate B and the
child's bearing, and leaves along the axis through child plate B. It only
twists between the slot and the exit.

## Stack-up, RS06 joint (along the axis, output side first)

```
child plate A   3.0   bolted to the spacer disc
spacer disc     5.0   on the output flange, through parent plate A
parent plate A  3.0   on the stator front ring (Ø56 clearance hole)
motor body     ~49
parent plate B  3.0   on the back cover, 4x M4 on Ø62
sleeve boss         bolted through parent plate B to the back cover; side cable slot
child plate B   3.0   on a flanged plain bushing, on the sleeve
```
