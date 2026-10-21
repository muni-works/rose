# Measurement scripts: dry run without hardware (2026-10-20)

`LYMB_SIM=1` swaps the CAN adapter for `measure/sim_bus.py`, a simulated RobStride bus. The scripts send their real frames, and each simulated motor answers with feedback frames from a model:
- MIT torque against the model's gravity at the current pose
- inertia and friction
- a first-order case temperature
- encoder noise
- a fixed random zero error per joint, for `report.py` to find

During `teach.py`, `hand()` stands in for Cam's hands and holds the ball in the seat.

```bash
cd arm/fork/build/measure
LYMB_SIM=1 uv run --with python-can python bringup.py scan
yes "" | LYMB_SIM=1 SIM_LOAD_KG=0 uv run --with python-can python teach.py --load 0
yes "" | LYMB_SIM=1 SIM_LOAD_KG=1 uv run --with python-can python teach.py --load 1.0
python3 report.py runs/*_0kg.csv runs/*_1kg.csv
LYMB_SIM=1 SIM_LOAD_KG=1.27 uv run --with python-can python hold.py --load 1.27 --minutes 0.5
```

## Results (v10.2 model)

| script | result |
|---|---|
| `bringup.py scan` | all 6 ids answer (1, 2, 3, 4, 6, 7), no fault |
| `teach.py` 0 kg / 1 kg | 50/50 poses recorded at each load |
| `report.py` | raw 0.56 mm RMS, worst 0.68 mm: **PASS** at both loads, exit code 0 |
| fitted offsets vs injected zero errors | J2 +0.076 vs +0.072°, J6 −0.054 vs −0.055°, J4 −0.015 vs −0.038°, J1 +0.088 vs +0.064° (J1 is weakly observable on the plate) |
| `hold.py` 1.27 kg, straight out | holds J2 at 23.5 N·m (model 23.5), no fault, temperature log written |
| `rig_test.py` zero / tare / heat / duty / torque / repeat / backlash | every command runs and writes its CSV. The arm simulator has no rig load, so heat and torque correctly report **FAIL: no load** |

## Bugs the dry run found, all fixed
1. **The scripts were still v9-shaped.** `arm.py` had J5 hard-coded, and `poses.py` had no J3. Joints now come from the model; `ROSE_ARM` defaults to `v10`, and `ROSE_ARM=` gives v9.
2. **Seat heights ignored v10's desk puck.** The model's table surface is 45 mm below the foot, so every seat would have been off by 45 mm. `poses.py` now finds the table surface in the model, and the foot bolt circle comes from the model too.
3. **Floating without a seat drifts.** With no cone holding the ball, the gravity feed-forward let joints drift up to 3° at 1 kg during the 0.8 s recording. On the real plate the cone holds the ball, so it shouldn't matter, but if the arm sags while Cam guides it, raise KD.
4. **`rig_test.py heat/duty` printed PASS without checking the torque it held.** It now checks the mean holding torque is within 10 % of the target.
5. **The rig's weights were too light.** 35 N·m at 270 mm needs 13.2 kg, and the purchase list had 10 kg. It now lists 14 kg (+$15).

## What can't be checked without hardware
- CAN framing against real firmware.
- The RobStride adapter vs a candleLight adapter.
- Sign conventions and zeroing on real motors.
- Real thermal behaviour.
- The anti-backdrive power-off behaviour.
