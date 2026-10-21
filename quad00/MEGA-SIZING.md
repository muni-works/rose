# Mega humanoid: actuator sizing (heavy-duty, human-scale)

*2026-10-21. Cam's brief: "heavy-duty human-scale: ~1.9–2.0 m, bulkier, 25–50 kg payload, fits doors." The model is `mega_sizing.py`. Everything here is an **estimate** (quasi-static load cases × 1.6 dynamic margin). A whole-body simulation replaces it later.*

## Targets

- **Height 1.95 m.** A standard US door is 2.03 m, so the head clears by about 80 mm. The current mega mockup is 2.10 m and would *not* fit doors, so it comes down.
- **Bulkier, not taller.** About 95 kg robot mass: H1-class 47 kg scaled to 1.95 m (60 kg) × 1.6 for bigger actuators and structure. The full is about 40 kg.
- **Payload:** 50 kg for short lifts (peak cases), 25 kg carried continuously (thermal cases).

## Joint torques and actuator mapping

Actuator choice uses vendor RobStride peak/continuous torque (in `quad.py`) plus a reduction stage (belt or linkage) at 90 % efficiency. "rana" means Cam's own actuator, not yet built.

| joint | peak N.m | continuous N.m | actuator | stage | note |
|---|---:|---:|---|---|---|
| knee pitch | 344 | 73 | rs04 | 4:1 | |
| hip pitch | 394 | 68 | rs04 | 4:1 | |
| hip roll | 204 | 51 | rs04 | 3:1 | |
| hip yaw | 71 | 15 | rs06 | 3:1 | |
| ankle pitch | 273 | 24 | rs04 | 3:1 | |
| ankle roll | 114 | 12 | rs06 | 4:1 | |
| waist pitch | 436 | 80 | rana (custom) | - | |
| waist yaw | 174 | 16 | rs04 | 2:1 | |
| shoulder pitch | 205 | 46 | rs04 | 3:1 | |
| shoulder roll | 123 | 23 | rs06 | 4:1 | |
| upper-arm yaw | 72 | 14 | rs06 | 3:1 | |
| elbow | 138 | 29 | rs04 | 2:1 | |
| wrist | 31 | 6 | rs02 | 3:1 | |
| neck | 8 | 3 | rs02 | direct | |

## What this says

1. **No current RobStride drives the legs directly.** The knee and hip pitch need about 350–400 N·m peak. The RS04 (120 N·m, the largest RobStride) works only through a **3–4:1 stage**, and the waist pitch (about 440 N·m) is past even that.
2. **Speed is the catch (needs checking).** A 4:1 stage divides speed by 4. Walking and stairs need roughly 8–12 rad/s at the knee and hip, so the motor must turn at 32–48 rad/s, about 300–460 rpm. That's likely beyond the RS04's rated speed; check the datasheet. If it is, staged RS04 legs walk slowly and the legs need the custom actuator.
3. **Recommendation: one "rana-L" actuator class for the high-load joints.** About **450 N·m peak / 100 N·m continuous**, at least 12 rad/s, about 3–3.5 kg. Use it for knee, hip pitch and waist pitch: 5 per robot. It's one design, and the same part covers the full at a lower rating, fixing the full's known torque shortfall too.
   - Everything else (hip roll and yaw, ankle, waist yaw, shoulder, elbow) works with **RS04 + 2–3:1 belt or linkage**.
   - Wrist and neck use RS02 or RS06.
4. **Payload realism.** 50 kg is past what any shipping humanoid lifts. Atlas, Optimus and Figure carry about 20–25 kg. Keep **50 kg as short lifts** (two-arm, close to the body) and spec **25 kg carried**.
   - The shoulder (205 N·m peak) and elbow (138) are what limit lifting with arms extended. The rule is "hug the box".
5. **Mass and power (estimate).** About 30 actuators come to about 35–40 kg including stages. Battery: about 2 kWh for 2 h of mixed work, about 12 kg. Together that fits in 95 kg.

## What changes in the model

- The mega variant goes from 2.10 m to **1.95 m** and stops being a pure scale-up of the full. Limb sections get sized by the new actuators (thicker thighs, shanks and upper arms), which fixes the "thin at mega scale" look.
- The pelvis cover comes back on full and mega; it's in progress in the body session.
- Open: rana-L spec and partner (build in-house, or source a 400–500 N·m class actuator), RS04 speed curve, and a whole-body simulation of stairs and lifting.

## Rebuilt (2026-10-21)

The mega variant (\`k1_structure.use('mega')\`) is now **1.95 m**, built on the new actuator envelopes in \`quad.py\`. These are **envelopes only, not real parts**:
- **rana-L** (Ø150 × 95, 450 N·m peak, 3.2 kg) at the hip pitch, hip roll and knee (the "big" slot) and the waist.
- **RS04 + 3:1** (Ø120 × 90, 324 N·m peak) at the ankles and arms.
- RS04 at the hip yaw, RS06 at the wrist.

Limb sections grow with the motors; the structure is motor-driven. Render: \`renders/mega/mega-195-34.png\`.

**Range-of-motion sweep (5° steps):** everything meets its need. The exception is **Shoulder_Roll −5° vs −10° needed** (it hits the shoulder-pitch drum), the same shortfall the full already has. Notes:
- Head pitch is now limited by the chest at +40°, which is exactly the need.
- Waist roll is ±20°, against a ±15° need.

Open items:
- The pelvis cover and the white release look live on the body branch (\`claude/quad00-body-release1\`). Mega picks them up when that merges.
- The single \`waist\` motor slot puts rana-L on waist yaw and roll too, which is oversized: RS04+2:1 would do. Splitting the slot is a follow-up.
