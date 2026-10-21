# Elbow rig: test plan and pass criteria

The rig is the lymb's J2/J4 joint as built: same RS04, back-mount, sleeve, bushing and 1/8" plates. The test forearm is 270 mm to the load bolt. Every test is a `rig_test.py` subcommand and logs CSV to `runs/`.

## Bring-up (the measurement scripts end to end)
1. RobStride Motor Studio, through the RobStride USB-CAN module: set CAN id 4 and check the firmware. Leave `damper` at its default.
2. Swap to the candleLight adapter: `python3 ../build/measure/bringup.py scan`. **Pass:** id 4 answers, no fault.
3. Forearm horizontal on a digital level, then `rig_test.py zero` and `rig_test.py tare`. This records tau0, the unloaded arm moment.

## Tests

| # | test | command | pass |
|---|---|---|---|
| 1 | **Torque accuracy**: reported torque vs m·g·L | `torque`, 0.5–10 kg (14 kg on hand for the 35 N·m tests) | within ±10 % at every load ≥ 1 kg. This sets how far `hold.py` and the gravity feed-forward in `teach.py` can be trusted |
| 2 | **Continuous hold, the shoulder's rating**: 25 N·m is what J2 needs at the 1.27 kg straight-out rating | `heat --nm 25 --minutes 30` | 30 min, no fault, case < 80 °C (thermocouple and the motor's own reading agree within 5 °C). If it fails, the lymb's rated payload drops in proportion |
| 3 | **Duty cycle**: work-like loading | `duty --nm 35 --on 10 --off 10 --minutes 30` | no fault; case temperature levels off below 80 °C |
| 4 | **Heat soak, worst case**: 35 N·m continuous | `heat --nm 35 --minutes 30` | informational: time to 80 °C and where the curve flattens. Gives the thermal time constant for the derating model |
| 5 | **Backlash and joint play** at the load bolt | `backlash`, dial at the bolt | output play ≤ 0.10° (≤ 0.47 mm at 270 mm), gearbox plus bushing. The 2.0 mm tool-point gate needs it |
| 6 | **Repeatability** | `repeat`, dial at the bolt | 10 returns within 0.05 mm (spread) |
| 7 | **Stiffness / sag** | dial at the bolt during `torque` | deflection per N·m recorded. Pass: under 1 kg the output stays within 0.2 mm beyond what the model predicts (plates 0.03 mm, plus the motor's own compliance) |
| 8 | **Power-off drop** | forearm horizontal with 1 kg, hit the e-stop, film at 60 fps against a protractor. Repeat with `damper` toggled | informational. Decides whether the lymb needs a brake or a soft-landing policy. Record whether it free-falls and how fast |
| 9 | **Cable twist**: harness through the sleeve | ±150° at 0.5 Hz overnight (`duty`-style loop; edit the angles) | no jacket wear at the slot, bus resistance change < 5 % |

## What a pass means
- **Tests 1–3:** the RS04 joint is good for J2/J4 at the rated hold, on our plates as the heat sink.
- **Tests 5–7:** the fork joint is stiff and tight enough for the ≤ 2.0 mm gate.
- **Test 8:** the crush-point and fall policy can be written.
- **Bring-up and tests 1–7:** the whole script stack (driver, zero, tare, logging) has run on real hardware before the lymb exists.

Any fail goes back to the v10 design before the full order list is unparked.
