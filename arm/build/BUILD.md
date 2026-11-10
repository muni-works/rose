# Lymb (Rose fork v9) build package

One arm, one person. Everything a hardware hour needs is here; generated files come from the model
(`python3 arm/fab.py`, `python3 arm/build/measure/poses.py`), so re-run them after any model change.

| file | what |
|---|---|
| [ORDER.md](ORDER.md) | every line to buy, prices, links, lead times (researched 2026-10-20; most lines unconfirmed, re-check at checkout) |
| [../fab/nest/](../fab/nest) | cut files: three 1/8" sheets + one 1/4", nested for the 4040 (390 mm bed) |
| [../fab/parts/](../fab/parts) | one DXF per unique part (20) |
| [../fab/BOM.md](../fab/BOM.md), [../fab/SLEEVES.md](../fab/SLEEVES.md) | parts, screws by interface, the three turned sleeves |
| [measure/](measure) | bring-up, gate, hold and sag scripts + the calibration plate DXF and the 50 poses |
| [RANA-DYNO.md](RANA-DYNO.md) | rana dyno plan (optional for Desk; RS04 baseline first) |

## Cam's checklist

### Today: order (nothing else gates the build)

1. **Motors, now.** RS04 x3, RS02 x2, RS00 x1 (+ EL05 if the gripper is in scope, see "requirements" below). Every
   US source shows backorder / preorder: **4–6 weeks** (Robotics Center) or 10–20 working days (OpenELAB, duties
   included). Order from the source with the shortest confirmed date; split the order if needed. ~$1,390 + shipping.
2. **One RS04 extra (optional, +$295).** Lets the elbow rig and the RS04 dyno baseline run while the arm waits, and
   becomes the spare.
3. Metal, bushings, screws, power, e-stop, harness, measurement kit: ORDER.md lines 7–40, all 1–3 days. ~$650.

### This week: cut and turn (no motors needed)

4. Cut the four nested sheets: `fab/nest/3.2mm_sheet{1,2,3}.dxf`, `6.3mm_sheet1.dxf`. 1/8" single-flute, tabs on.
   Deburr; check one tab-and-slot pair fits before cutting the rest.
5. Cut the calibration plate from the 1/4" offcut: `measure/cal_plate.dxf` (layers: CUT outline and Ø6.6 foot holes,
   DRILL3_CSK6 seats = Ø3 through then 90° countersink to Ø6.0, REAM6 riser dowels). Two risers from 2" bar, seat cone
   on top, height measured with calipers into `BLOCK_H` in `poses.py`, then re-run it.
6. Turn the sleeves: RS04 x2 (+1 for the rig), RS02 x1, per SLEEVES.md. Print hub caps and the base collar (PETG).
7. Dry-assemble all plates with screws only (no motors) to catch any fit issue a week early.

### When motors land: build (~2 days) and measure (~1 day)

8. **Bench each motor alone** (48 V, current-limited supply, e-stop in line): set CAN ids 1,2,4,5,6,7 in RobStride
   Motor Studio, then `python3 measure/bringup.py scan`. **Measure the RS02 back-face and output thread depths now**
   (BOM assumes 6 mm); swap screw lengths if shallower.
9. **Assemble in this order** (each step torques its screws with Loctite 243; the next step needs it):
   1. Foot onto the calibration plate (4 x M6) and the plate clamped to the bench.
   2. J1 RS04 back face onto the foot (10 x M4x10); base collar.
   3. Turret disc onto J1 output (8 x M5x8); towers A/B into the disc by tabs.
   4. Upper arm: J2 and J4 RS04s back-mounted between upper-A/B with sleeves under plate B (M4x10); covers by tabs.
   5. Upper arm into the turret: tower A to J2 output through the RS04 spacer (M5x12); tower B on the bushing + ring.
   6. Forearm: fore-A to J4 output through spacer (M5x12), fore-B on the J4 bushing; fore end plate by tabs.
   7. J5 RS02 cartridge on the fore end plate (M4x8); yoke bridge to its output (M4x8); yoke A/B.
   8. J6 RS02 back-mounted in the yoke with its sleeve; wrist-A to its output via spacer; wrist-B on the bushing.
   9. J7 RS00 on the wrist end plate (M3x6); tool flange + probe adapter (or gripper) on its output (M3x10).
   10. Harness through each sleeve, XT30 daisy chain, 120 Ω at both ends. Hub caps on.
   After each motor goes in: power it alone, jog ±10° by hand in `bringup.py watch`, check nothing rubs.
10. **Zero and map:** copy `measure/joints.example.json` to `joints.json`. Arm straight out on the zero jig (digital
    level on each plate edge, 0.1°), `bringup.py zero <joint>` for each. Move each joint by hand in `watch`: fix
    `sign` until the shown angle follows the model (J2 negative = up).
11. **Measure** (protocol below). Then the lymb exists and is measured.

## Measurement protocol

All runs log CSV into `measure/runs/`; commit them with the arm's serial number.

| test | how | pass |
|---|---|---|
| **Tool-point gate** | `teach.py --load 0`, then `teach.py --load 1.0` (1.0 kg hung on the flange). Hand-guide the Ø6 ball probe (60 mm past the flange) into each seat while the arm floats on gravity feed-forward; Enter records 0.5 s of encoder data. 10 seats x 5 configurations = 50 poses. `report.py runs/*_0kg.csv runs/*_1kg.csv`. | raw ≤ 2.0 mm RMS at both loads, 50/50 poses |
| **Reach** | Straight out (`hold.py --pose out --load 0`), tape from the J2 axis to the jaw tips / probe centre. | 731 mm jaw tips (model), 786 with gripper v2 |
| **Sag** | Dial indicator on the flange top, zero at 0 kg, hang 1.0 kg and 1.27 kg while `hold.py` holds straight out. | record it; model 0.13 mm is structure only, gearbox compliance adds to it |
| **Hold** | `hold.py --pose out --load 1.27 --minutes 30` (gripper-v2 rating). Logs torque and case temperature each second. | no fault, every motor < 80 °C at 30 min, J2/J6 torque within 10 % of model |
| **Power-off drop** | From horizontal with 1 kg, e-stop. Film it. Repeat with `damper` on. | know whether the arm falls; sets the crush-point policy |

Notes: the gate measures the model-plus-encoders error a commanded pose would carry (FK of the measured joint
angles vs the seat's known position), including sag under load. The plate is registered by the foot's own bolt
pattern, so no external metrology is needed. Seat positions are good to the router's ~0.05 mm. `report.py` also
prints a "calibrated" number (6 joint offsets fitted on odd poses, scored on even ones) that shows how much a
software zero would buy. The gate is the raw number.

## Requirements, questioned (for Desk only)

Desk step 2 needs an arm worth buying, measured, with a stated gap. Things the spec carries that Desk doesn't need yet:

1. **Gripper v2 / EL05: not needed to measure a lymb.** The gate uses a probe; hold uses a hanging weight. Build the
   arm with the probe flange, add the gripper after. Saves ~$80 and a part whose identity (EL05 vs "EduLite 05") is unconfirmed.
2. **Clock-spring cassettes on the roll joints: not needed.** Undesigned, and a bench lymb can use service loops
   with J5/J7 limited to ±170° in software. Design them for the shipped product, not the first build.
3. **Hub caps, base collar, cover cosmetics (cortex exterior language): not needed for the measurement.** Printable later
   without disassembly. Don't let the exterior pass gate the first build.
4. **Three sizes (mini / mega) from one loon source: not needed for Desk.** Only "lymb full" is sold in step 2; park mini/mega.
5. **J5 forearm roll: question it.** A 5-DOF arm (mini drops J5) saves an RS02 (~$175), ~0.5 kg at the wrist (holding
   goes up) and a cable cassette. Keep 6-DOF only if a named Desk task needs arbitrary tool orientation.
6. **"Ready to cut" is slightly stale:** the model is now 8.34 kg and holds 1.44 kg straight out (wrist binds), vs 8.21 kg /
   1.49 kg in the paper; `fab.py` regenerated two turret-tower windows (0.11 mm shift). The paper's table should
   quote the measured numbers once they exist.
7. **Margin claim:** at today's reseller prices the motors alone are ~$1,560 with EL05 and shipping, not ~$1,200.
   Factory price (¥1,199/RS04 ≈ $168) is ~1.8x cheaper: worth a direct RobStride quote for 10 sets before the ≥10-lymb gate.

## Open items found while packaging

- `measure/` scripts are untested on hardware (no CAN hardware here). Validated: imports, IK for all 50 poses,
  report math on a synthetic run (0.82 mm raw / 0.57 mm calibrated with injected noise).
- The 50 poses are checked for joint limits but not for self-collision with the foot/plate; teaching is by hand, so
  the operator sees it, but run `audit.py` on them before ever commanding them.
- `teach.py` gravity feed-forward uses model masses; if the arm drifts while floating, raise KD or measure link masses.
