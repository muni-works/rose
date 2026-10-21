# lymb v10.3 ↔ quad00 full torso: interface for rose

This is everything the torso needs to fit the v10.3 arm on the 1703 mm full humanoid. It is checked against rose's torso pass 1 (claude/quad00-torso-v103 @ a2c2207).

- **Frame:** quad00 `k1_structure` body frame for `full`: x fore, y left, z up, mm. The shoulder-pitch axis is at z 434.3.
- **Lifted frame:** the vcad export adds z0 = 999.9 so the soles stand on z = 0; add 999.9 to every z here.
- **Sides:** left side given; the right mirrors in y.
- **Source:** `arm/fork/torso_interface.py` (numbers + mesh), `arm/fork/thermal_v10.py` (heat).

## 1. J1 drum (the arm's J1 = the humanoid's shoulder pitch)

| | value |
|---|---|
| axis | +y (lateral), through x = 0.0, z = 434.3: quad00's own shoulder-pitch axis |
| torso side plate outer face | \|y\| 87.0, J1's mount |
| chest adapter (the desk pedestal's tube, 41.05 mm): 4 x 1/8" 5052 plates, 130 x 130 square | \|y\| 87.0 .. 128.1. Corners reach **r 92** about the axis. This is the biggest thing inside the corner. |
| 1/4" foot, Ø180 | \|y\| 128.1 .. 134.4, r 90 |
| J1 RS04 can, Ø120 | \|y\| 134.4 .. 186.6, r 60 |
| J1 output face = torso flank | **\|y\| 186.6**, flush with the chest cover's flank (rose's `trim-j1ring` sits here) |
| open graphite ring on that face (the arm owns it) | OD 133 / ID 123, i.e. r 61.5 .. 66.5, 2 mm thick (\|y\| 183.6 .. 185.6, 1 mm under the flank face). The motor's black face is open to air inside it. |
| turret disc (J1 output → J2), 1/8" 5052, Ø140 | \|y\| 186.6 .. 189.8, under the arm's turret skin |
| arm skin starting at the flank (turret shell) | r 77 (x ±77 at the flank, tapering up into the J2 puck) |

**Drum radius to clear inside the torso:**
- r 92 for \|y\| 87..128 (the adapter's corners);
- r 90 for 128..134 (the foot);
- r 60 for 134..187 (the can).

Rose's "R ≈ 71.5" covers the can and the open ring, but not the adapter corners and the foot. Either clear r 92 / r 90 inside the corner, or chamfer the adapter tube's corners (a 120 mm square would bring them to r 85).

## 2. J2 roll-stage swept envelope

The envelope covers the turret skin, the turret's neck shell, the J2 puck and its caps, and the top 160 mm of the upper-arm sleeve. It is swept over the full J1 rotation (360°) and J2 over its soft range (-180 .. +10°).

- **|y| range:** 186.1 .. 418.1 (J2 at its far end swings the sleeve top out along the axis).
- **Max radius about the J1 axis:** **162.0 mm**, at |y| ≈ 256 (the J2 axis is at |y| 260.1).

Profile (radius about J1 vs |y|):

| \|y\| | 186 | 196 | 216 | 236 | 256 | 276 | 296 | 316 | 336 | 356 | 376 | 396 | 416 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| r | 84 | 149 | 156 | 160 | 162 | 161 | 158 | 152 | 143 | 131 | 114 | 89 | 47 |

**Mesh:** `arm/fork/interface/j1j2_envelope_l.stl`. Units mm, pre-lift frame above, left side, +0.5 mm on the profile. Clash-check the deltoid cap against it with your usual 5 mm.

## 3. Open-ring puck spec

| joint | ring OD / ID | axial depth | bezel | dot-grid |
|---|---|---|---|---|
| J2, J4 (RS04) | 137 / 123 | 91.5 | gunmetal, 3 mm wide, r 61.5..64.5, both faces | Ø1.3 holes on 2.3 mm hex, 29 % open, round the full circumference, 4 mm clear of each edge |
| J6 (RS02) | 95.5 / 81.5 | 84.4 | same, r 40.75..43.75 | same |
| J1 face (torso flank) | 133 / 123 | 2 (a face ring, not a drum) | rose's `trim-j1ring` | none: only the face is in air |

**Airflow** is natural convection, no fan:
- The open motor faces and the 7-8 mm annulus between the can and the ring are open at both ends.
- Air enters through the lower part of the annulus and the lower dot-grid holes, rises along the hot can, and leaves through the upper holes and annulus (a chimney, whichever way the joint points).
- For J1 on the torso, only the face annulus is open, so air goes in and out through the same face. Keep the flank around the ring free of anything that blocks it for about 10 mm.

## 4. Aluminium flank panels and heat

- **Panels: on the upper arm only.** They go on the flat front and back faces over the J2 and J4 holder plates, 4 per arm:
  - 0.6 mm 5052-H32, Type II anodize in warm black (#1e1813 dye match), satin;
  - 0.25 mm gap pad (~3 W/mK) to the plates.
- **The torso around J1 needs no panel.** J1 already conducts through its back face, the adapter and the 6.35 mm chest frame (see below).

Heat each motor dissipates holding the rated 1.27 kg straight out (model jaws, skin mass included). Copper loss is 1.5 R_line I_rms², with I = T / Kt.

| joint | motor | torque | heat |
|---|---|---|---|
| J1 (torso corner) | RS04 | 24.7 N·m (rose's 23.5 is the bare arm) | **33 W** (30 W at 23.5) |
| J2 | RS04 | 24.7 N·m | 33 W |
| J4 | RS04 | 9.5 N·m | 5 W |
| J6 | RS02 | 4.0 N·m | 9 W |
| J3 | RS02 | 0 straight out; 3.43 N·m at its pose cap | 0 / 6.5 W |
| J7 | RS00 | ~0 | ~0 |

### J1 in the torso corner (`thermal_v10.j1_body`, lumped, every assumption in its docstring)

Conduction runs from J1's back face through the foot (0.10 K/W contact), the 41 mm adapter tube (0.15 K/W), and the torso side plate plus half its decks (0.051 m² a face, efficiency 0.6). From there it goes to enclosed chest air (h 5) and this side's half of the chest shell (0.20 m², h 10). The corner air is enclosed, and no other chest heat is counted.

| case | case → air K/W | J1 continuous | arms-forward hold | time to 120 °C at 23.5 N·m | at 24.7 N·m |
|---|---|---|---|---|---|
| plate only (B) | 2.56 | 23.9 N·m | 1.16 kg | never (steady) | ~119 min |
| **plate + open ring (v10.3)** | 1.92 | **26.9 N·m** | **1.56 kg** | never | never |
| plate + fan 1.3 L/s | 1.10 | 33.1 N·m | 2.41 kg | never | never |
| plate + open ring + fan | 0.97 | 34.7 N·m | 2.62 kg | never | never |

The time is from a one-node transient: the whole RS04 at 850 J/K behind its 0.49 K/W winding resistance. The winding alone heats faster, so treat these times as upper bounds.

**Design of record (Cam, 2026-10-21): no fans in the torso.** J1 is cooled by the open-ring face plus side-plate conduction, with a controller derate above the ring-only 1.56 kg arms-forward hold. The fan rows above are for reference only.

**Conclusion: J1 needs no fan.** With the open ring face that v10.3 already has, the arm holds the rated arms-forward load indefinitely.
- J2 (1.27 kg), not J1, is the limit.
- Plate-only conduction alone is marginal: steady at rose's 23.5 N·m, about 2 h at the skinned 24.7 N·m.
- These are estimates; the elbow rig's heat test checks the model.

## 5. Who owns the deltoid cap

**Recommendation: the arm owns it.** It is the turret shell, on J1's output, so it rotates with J1, which is J2's parent. It already exists in v10.3 as the turret skin and neck shell.

- **Why not the torso:** a torso-owned cap is static, so it would have to clear the whole swept envelope above (r 162 about J1, out to |y| 418). That forces either a huge dome or a slot for the arm.
- **Why the arm:** an arm-owned cap only has to clear J2's own swing. It does, by construction: the coaxial seam is r 69.5 about J2, and the cap stops where the seam circle starts.
- **What rose shapes:** the torso's sloped top and shoulder corner, ending at the flank (|y| 186.6), with
  - ≥ 5 mm to the swept envelope (`interface/j1j2_envelope_l.stl`) everywhere outboard of the flank;
  - inboard of the flank, ≥ 5 mm to the J1 corner contents in section 1 (r 92 / 90 / 60).
- **Arm side, next pass:** I'll shape the turret shell's top surface to continue rose's slope at the rest pose, from the slope angle rose sends me.
- **Clash check:** the arm's soft limits (`soft_limits.pose_ok(..., mount='body_l')`) already keep the arm skin ≥ 5 mm from the torso shell on rose's pass 1. See V10.3.md for the sweep.
