# Sizing the line at 1800 / 2000 mm

2026-08-19. "rose pro should be about 1800 mm, rose max 2000." Height is the
root decision: mass goes as the cube, joint torque as mass × length, and the
actuator that meets the torque sets the envelope everything wraps. So before
any link is re-drawn, here is what each joint needs and what exists to supply
it. Numbers from `sizing.py`; calibration and provenance in its docstring.

## Calibration (URDF effort limits, normalised)

| | height | mass | thigh | knee peak | knee / (m g L_thigh) |
|---|---:|---:|---:|---:|---:|
| Unitree G1 | 1.32 m | 34.1 kg | 332 | 139 | 1.25 |
| Unitree H2 | 1.82 m | 70 kg | 412 | 360 (every hip axis too) | 1.27 |
| rose pro (now) | 1.17 m | 17.6 kg (as modelled — no battery/skin/compute) | 250 | 60 | 1.39 |

Mass as height-cubed density: H2 **11.6 kg/m³**, G1 **14.8**, rose pro as
modelled 11.0. Design at H2 density; the pick must still hold at G1 density,
because commodity actuators are heavier per N·m than Unitree's.

## rose pro at 1800 mm

| | design (11.6) | pessimistic (14.8) |
|---|---:|---:|
| mass | **68 kg** | 86 kg |
| thigh / shank / torso | 405 / 486 / 522 mm | same |
| hip pitch, knee | 349 N·m | 446 N·m |
| hip roll | 269 | 343 |
| hip yaw | 188 | 240 |
| ankle pitch / roll | 68 / 34 | 86 / 43 |
| shoulder pitch | 108 | 138 |
| shoulder roll, elbow | 58 | 74 |
| wrist, head | 17 / 10 | 22 / 13 |

Picks on MyActuator RMD-X V4 (the number after the dash is peak N·m):

| joint | pick | margin at 68 kg | at 86 kg |
|---|---|---:|---:|
| hip pitch, knee | **X15-450** | 1.29 | 1.01 — at the limit |
| hip roll | **X12-320** | 1.19 | 0.93 — would want X15 |
| hip yaw | **X10-200** | 1.06 | 0.83 — would want X12 |
| ankle pitch | **X8-120** | 1.76 | 1.40 |
| ankle roll | X6-60 | 1.76 | 1.40 |
| shoulder pitch | X8-120 | 1.11 | 0.87 |
| shoulder roll, elbow | X6-60 | 1.03 | 0.81 |
| wrist, head | X4-36 | 2.1 | 1.6 |

So: **an 1800 mm pro on the X series is an X15 machine at the hip and knee,**
and it is only the H2's density that makes it fit — at G1 density the hip roll
and yaw step up a size each and the hip pitch/knee have no margin. 22 DOF:
4× X15, 2× X12, 2× X10, 4× X8, 6× X6, 4× X4.

ENVELOPES: see the catalogue table below, filled from vendor data. The hip
pitch pair sits end-to-end on the hip axis (body inboard, as now), so HIP_Y is
set by the X15's axial length, not by taste. This is where the generator gets
its new numbers.

## rose max at 2000 mm

| | design (11.6) | pessimistic (14.8) |
|---|---:|---:|
| mass | **93 kg** | 118 kg |
| thigh / shank / torso | 450 / 540 / 580 mm | |
| hip pitch, knee | **533 N·m** | 679 |
| hip roll / yaw | 410 / 287 | 523 / 366 |
| ankle pitch | 93 | 118 |
| shoulder pitch | 148 | 189 |

rana-140 is 96 N·m peak. **A 2000 mm rose max needs a hip/knee actuator five
to seven times rana-140** — a rana-200-class unit at ~550 N·m peak, which is
the H2's 360 N·m joint again scaled to the extra 180 mm and 23 kg. That is a
new actuator, not a layout; rana-140 covers only the ankle and the arms at
this height. max's current geometry (~1210 mm, R1-class, 25 kg) is a
different machine and its INTERFACES.md targets are the R1's. The re-scope
is: new targets (mass ~93 kg, leg 1060, joint list above), a rana-200 line
item with envelope TBD, and only then `gen-rose1.py`.

## What this means for the sheet

At 1800 / 68 kg the knee moment is 350 N·m. A closed 3 mm 5052 channel 100
square has Z ≈ 3·10⁴ mm³ → ~12 MPa bending at peak, against 190 MPa yield:
the wrap is fine as a *beam*. What does not scale is the joints between
blanks — bolted laps and tabs carrying 350 N·m — and buckling of 3 mm faces
at 100+ mm spans. Expect 4 mm sheet on the leg channels and machined or
welded fittings at the knee and hip output. max's README already says this
("sheet-metal primary structure at rose-2 loads does not transfer"); at 1800
pro is at that load.

## Actuator catalogue (MyActuator RMD-X V4, vendor 2D drawings, Aug 2026)

Local copies: `~/Developer/myactuator_assets/<model>/` (2D PDF, STEP, parameter
sheet). Read off the drawings; the one estimate is flagged.

| model | peak / rated N·m | ratio | mass | OD × length | output ring | stator ring | output pilot / flange | through hole | V | price |
|---|---:|---:|---:|---|---|---|---|---:|---:|---:|
| X4-36 | 34 / 10.5 | 36 | 0.36 | Ø55 × 61 | 6×M3 PCD 26 | 6×M3 PCD 48 | Ø35 | — | 48 | $400 |
| X6-60 | 60 / 20 | 19.6 | 0.82 | Ø80 × 67.5 | 12×M4 PCD 44 | 12×M3 PCD 72.5 | Ø32 / Ø53 | — | 48 | $490 |
| X8-120 | 120 / 43 | 19.6 | 1.40 | Ø96 × 76 (+3 flange) | 12×M4 PCD 55 | 10×Ø3.4/M3 PCD 88 | Ø48 / Ø64 | 12.6 | 48 | $588 |
| X10-200 | 200 / 60 | 20 | 1.64 | Ø124 × 63.5 (+2) | 12×M5 PCD 72 | 16×M4 PCD 116.5 | Ø85 / Ø107 | 12 | 48 | n/a |
| X12-320 | 320 / 85 | 20 | 2.37 | Ø124 × 85 (+3) | 12×M5 PCD 72 | 16×M4 PCD 116.5 | Ø85 / Ø109 | 12.6 | 48 | $1,100 |
| X15-450 | 450 / 145 | 20.25 | 3.50 | Ø166 × 69 (+3; connector +~8 est.) | 12×M5 PCD 100 | 12×Ø5.3 PCD 157 (front) / 16×M5 PCD 154 (rear) | Ø115 / Ø144 | 21.5 | **72** | ~€1,380 |

Two consequences for the layout, read straight off that table:

- **The X15 is a 72 V part.** The rest of the line is 48 V. Four X15s at the
  hip pitch and knee mean a 72 V bus on the legs or a second bus — a power
  architecture decision, not a generator one. Flagging it; the X12-320 at
  320 N·m is the largest 48 V unit and at 68 kg it carries the hip/knee at
  0.92 margin — i.e. not at the pessimistic mass.
- **The X15 is Ø166.** Two hip-pitch bodies end-to-end on the hip axis (as
  now) put HIP_Y at ≥ 69 + 8 + gap ≈ 85 mm each side — the H2's is 80. The
  hip roll X12 (Ø124 along x) and yaw X10 (Ø124 along z) fix the thigh wrap at
  ~Ø124 + 2×(boss + 5) ≈ 150 square at the top. Actuator mass for the pick:
  4×3.5 + 2×2.37 + 2×1.64 + 4×1.4 + 6×0.82 + 4×0.36 = **34.0 kg**, structure
  at 4 mm ≈ 22 kg, battery/compute/hands ~10 kg → ~66 kg. The 68 kg design
  figure holds; it is 2× the G1's actuator fraction (49 %) in absolute terms
  and the same fraction (51 %).
