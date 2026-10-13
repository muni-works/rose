# Elbow rig — bill of materials

One RS06 fork joint on a bench stand, test forearm 290 mm to the load bolt.
Prices are estimates (USD, retail) and flagged as such.

| # | item | qty | est. $ | notes |
|---|---|---:|---:|---|
| 1 | RobStride 06 (RS06) | 1 | 260 | Amazon listing |
| 2 | RobStride USB-CAN adapter | 1 | 25 | |
| 3 | 48 V 7.3 A supply (Mean Well LRS-350-48) | 1 | 40 | 350 W covers peak current comfortably |
| 4 | 5052 sheet 3 mm, 400 × 400 | 2 | 50 | cut list: `cut/sheet_3mm_1.dxf`, `cut/sheet_3mm_2.dxf` |
| 5 | 6061 round Ø75 × 25 | 1 | 15 | sleeve boss, see `TURNED.md` |
| 6 | igus GFM-1618-06 flanged bushing | 1 | 5 | ID 16 / OD 18 / flange 24 |
| 7 | DIN 471 retaining ring 16 mm | 2 | 1 | |
| 8 | 2020 extrusion, 250 mm | 2 | 10 | test-forearm bottom bars |
| 9 | 2020 extrusion, 300 mm | 2 | 12 | bench foot rails under the stand plates |
| 10 | 2020 corner brackets + M5 T-nuts | 8 + 16 | 12 | stand plates to rails, bars to plates |
| 11 | M3 × 10 socket head | 8 | 2 | parent A to stator front (thread depth 8.5 — do not exceed) |
| 12 | M4 × 12 socket head | 6 | 2 | child A + spacer disc to the output |
| 13 | M4 × 10 socket head | 4 | 1 | sleeve boss + parent B to back cover |
| 14 | M8 × 70 bolt, nut, spacer tube Ø12 × 58 | 1 | 4 | load bolt across the child plates |
| 15 | weights, 0.5 kg steps to 4 kg + hanger | 1 set | 30 | |
| 16 | high-flex bus cable 2×20 AWG + CAN pair, ~1 m, XT30(2+2) plugs | 1 | 20 | through the sleeve |
| 17 | K-type thermocouple + reader | 1 | 15 | motor case temperature |
| | **total (est.)** | | **~505** | |

## Test plan

1. **Power-off behaviour.** Unpowered, `damper` = 0 (default), drop the test
   arm from horizontal with 1 kg. Film it. Repeat with `damper` = 1. Does the
   anti-backdrive protection engage at a falling-arm speed, and how hard?
2. **Spigot fit and runout.** Dial indicator on the child plate at the load bolt
   while rotating by hand: wobble from the bushing and the plate stack.
3. **Stiffness.** Hang 0.5 – 4 kg at the load bolt, motor holding position;
   measure tip deflection; compare with `fork_checks.py` and the motor's own
   compliance.
4. **Holding heat.** Hold 1 kg horizontal (≈ 4 N·m incl. the test arm) for 30
   min, log case temperature and the motor's reported winding temperature.
   Then 8 N·m. Is the manual's 8 N·m stall rating right with our plates as the
   heat sink?
5. **Cable twist life.** ±150° at 1 Hz with the bus cable through the sleeve,
   overnight; check for jacket wear at the slot and resistance change.
6. **Rotor inertia.** Unloaded current step, measure acceleration → J.
