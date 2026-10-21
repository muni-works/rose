# Mini lymb as the first measured article: assessment (2026-10-20)

**Recommendation: yes, build a mini lymb first, in parallel with full v10. But it needs an RS06 at the shoulder: in the model, an RS02/RS00-only mini cannot hold its own weight straight out.** Design only; nothing ordered.

## Motors and lead time
In the model (`ROSE_ARM=mini fork_checks.py` with `ROSE_ARM_MOTORS` / `ROSE_ARM_LEN`), the shoulder sets the limit:

| shoulder | lengths | J2 holds straight out | J4 elbow | J6 wrist | mass |
|---|---|---:|---:|---:|---:|
| RS02 | 190/130 | **−0.36 kg** (can't hold itself) | 0.61 | 1.35 | 3.7 kg |
| RS02 | 150/110 | **−0.16 kg** | 0.72 | 1.35 | 3.6 kg |
| RS06 (8 N·m stall, est.) | 190/130 | ~0.3 kg (hand estimate) | 0.61 | 1.35 | ~3.9 kg |
| RS06 | 150/110 | ~0.6 kg (hand estimate) | 0.72 | 1.35 | ~3.8 kg |
| RS04 (today's `mini`) | 150/110 | 4.24 | 0.72 | 1.35 | 4.8 kg |

- An all-RS02/RS00 arm works on the humanoid, where the arms hang, but not as a desk arm rated straight out.
- **Lead time hardly changes:** RS02 and RS00 are sold out at OpenELAB (preorder, 10–20 working days); the RS04 is 4–6 weeks. A mini saves 1–2 weeks, not a month.
- RS06 stock in the US is unconfirmed (Seeed sells it only inside the reBot kit). Check it before choosing the RS06.

## Mini v10 (same kinematics as full v10, scaled)
Proportions follow the mini humanoid (quad00 mini: shoulder→elbow 187 mm, elbow→fingertip 242 mm), so the arm is **190/130 mm**: flange about 536 mm from the shoulder, about 590 mm with a small gripper.

| joint | motor | est. $ |
|---|---|---:|
| J1 yaw / humanoid shoulder pitch | RS02 | 175 |
| J2 pitch / shoulder roll | RS06 | ~200 (unconfirmed) |
| J3 upper-arm roll | RS00 | 152 |
| J4 elbow | RS02 | 175 |
| J6 wrist pitch | RS00 | 152 |
| J7 tool roll | RS00 | 152 |
| **motors** | | **~1,010** |
| plates (2 sheets of 1/8"), sleeves, screws, harness, supply, CAN | | ~350 |
| **per arm** | | **~1,360** (full v10 ~2,180) |

**Predicted:**
- **Hold:** about 0.3 kg straight out, shoulder-limited (estimate); more in bent working poses.
- **Sag:** about 0.07 mm, from the v9 mini structure at these lengths.
- **Not yet confirmed:**
  - The RS06 needs a rating entry in `rose_fork.py` before these numbers are model outputs.
  - The J3 move is part of the full v10 work, which is still running.

## What it proves for Desk
- **The measurement pipeline end to end:** bring-up, zero jig, the 50-pose teach and `report.py`, hold, sag and drop. The same scripts run unchanged; only `joints.json` and the calibration plate (scaled seats) differ.
- **The first measured sim-to-real gap ε,** per joint and at the tool point.
- **The tool-point gate at scale:** ≤ 2.0 × 536/772 ≈ **1.4 mm RMS** at 0 kg and at 0.3 kg.
- **That one model source scales:** mini and full come from the same `rose_fork.py` with a size parameter, the same fab pipeline (DXFs and nests) and the same skin.
- **The fork joint itself:** sleeve and bushing play, the plate stack, power-off behaviour and heat on our plates.

## What it doesn't prove
- The full lymb's 1.27 kg hold rating and the RS04 joints: their heat on our plates, and their sag under load.
- Full-scale stiffness, and the 786 mm reach.
- Shipping-product cost. The mini is not the Desk product, and buyers want full reach.

## Time to a measured mini
- **Order to motors:** 10–20 working days (OpenELAB preorder), so about 3–4 weeks.
- **Cut, turn and dry-fit:** done while waiting.
- **Build:** 1–2 days. **Measure:** 1 day.
- **Result:** a measured mini about **4–5 weeks after approval**, against about 6–7 weeks for full v10.

## Recommendation
1. **Approve full v10 and order both together.** The mini's RS02 and RS00 motors come from a different reseller queue than the full's RS04s, so the two orders don't compete.
2. **If only one arm gets built first, make it the mini,** with the RS06 shoulder. It's the cheapest arm that exercises the whole pipeline and gives a real ε before the RS04s land.
3. **Cheaper still if the goal is only to prove the joint and the scripts:** the RS04 elbow rig (~$530, one motor). It doesn't check the multi-joint kinematics, though.
