# Measure on the bench (mini v1)

What the model can't tell us. Each line replaces a **TBD** in [`SPEC.md`](SPEC.md). Record each result in SPEC.md and delete the line here.

| # | Measure | How | Pass | Replaces |
|---|---|---|---|---|
| 1 | **Fan curve, in the head** | Close the head with the fan running at 100% PWM. Read the static pressure at the exhaust grille with a manometer tube through the I/O cutout. Then read the exhaust velocity with a hot-wire anemometer at 5 points on the grille. Repeat at 50% PWM. Also note which way the fan blows. | ≥ 3 CFM (≥ 2.2 m/s average at the grille) at ≤ 100% PWM; air exits the back grille | the fan curve, the fan's flow direction (ASSUMED), the heatsink rating at the fan inlet |
| 2 | **Real thermal at 25 W** | Set `nvpmodel` to the 25 W mode and run `stress-ng` on the CPU plus a GPU burn for 30 min. The room is 25 °C and the head sits closed on the robot. Log `tegrastats` (CPU/GPU/SoC temps, power) every second, and put a thermocouple at the fan inlet and one at the exhaust. Repeat with the head on the desk dock. | No throttling. Tj stays ≥ 10 K under NVIDIA's limit. Fan inlet ≤ 40 °C. Note whether exhaust gets drawn back in through the I/O cutout (inlet temp creeps up over time). | max module Tj (take the limit from the Orin NX thermal design guide), the 25 W pass |
| 3 | **Latch pull-off force** | Seat the head on the neck dock. Pull straight up through the head's centre of mass with a hanging scale until it releases, 5 times. Then pull at the face edge (the peel case), 5 times. Then press the release button and measure the lift-off force (that's the magnets alone). | Pull-off ≥ 3 × head weight × 4 g (fall/shake margin). Release with the button ≤ 15 N. | "pull-off rated by the pawl, TBD N" |
| 4 | Head mass | Weigh the closed head, then the robot | report | total mass |
| 5 | A603 hole pattern | Measure the carrier, or use the Seeed drawing. Drill the printed sled brackets to match. | 4 × M2.5 fit | carrier hole pattern |
| 6 | I/O cutout open area | Photograph the back with the carrier fitted. Measure the open area. | ≥ 35% open (the thermal check assumed this) | the ASSUMED 35% |

Body items (not SPEC TBDs, but cheap to check at the same time):
- **RS06 bolt patterns**: before cutting sheet 1, hold an RS06 against a 1:1 print of `plates/parts/thigh-inner_ll.dxf`. The patterns in `quad.py` MOTORS are ASSUMED.
- **Spine service loops**: cut each loop at 46 mm (from `harness_check.py`). Run head yaw ±57° and pitch −20..+40° 100 times and check the pogo pads still read continuity.
