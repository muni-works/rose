# rose

Our humanoid. The name is the thesis: past tense of *rise* — the one that stood up.

Third machine of the pond: **rana** (actuators) → **frog** (quadruped testbed) →
**rose**. frog is the crash-test vehicle; rose's falls are expensive, so the
control stack arrives here already debugged.

## The line

| | height | actuators | compute | what it is |
|---|---:|---|---|---|
| [**rose mini**](mini) | 1150 mm | MyActuator X-series | Orin NX | Booster K1 size. The crashable one. |
| [**rose pro**](pro) | 1450 mm | MyActuator X-series | Orin NX | The workhorse. Sim-validated at 1130 mm today. |
| [**rose max**](max) | 1750 mm | our own rana + torc | **Jetson AGX Thor** | Beefy. Built to jump. |

One design language across all three: **bent 3 mm 5052 sheet** primary structure
(laser blanks + press brake + bolts, no angle stock and no machined blocks),
exposed-skeleton limbs against a shaped torso that *is* the structure, staggered
and graded lightening cutouts, and contact geometry on the feet only. 22 DOF
everywhere — 6 per leg, 4 per arm, 2 in the neck.

Proportions for every variant scale from rose pro's **verified** 1130 mm layout,
which stands in `phyz` at **+0.0 %** ground-force error with COM→COP 0.6 mm.

```bash
python3 family.py           # proportions, mass, actuator selection, jump ceiling
python3 family.py --pack    # + re-run the packing check on each scaled skeleton
```

## Where each one stands

**mini** — 1150 mm, 17.4 kg. Geometrically within 2 % of the validated pro layout,
so it inherits its packing with margin. Was `toad`, a separate 950 mm design on
generic A/B/C actuator SKUs; that geometry is superseded by this line's, and what
survives from it is in [`mini/`](mini) — the vcad kinematic tree, DFM pass and
actuator sizing.

**pro** — the sim-ready machine, and the only one with a verified body. 22 DOF on
X6-60 hips and knees, X4-36 ankles and arms, X4-10 neck; 40 bent-sheet plates,
0 bend-rule violations, 17.58 kg, ~$9,410 of actuators. Loads in `phyz` and stands.
Growing it 1130 → 1450 mm is the open work: the skeleton scales, the actuator
envelopes do not, so the packing check has to be re-run rather than assumed.

**max** — 1750 mm, ~49 kg, rana-190 hips and knees, rana-140 ankles, rana-100 arms,
0.9 kWh at 48 V, Jetson AGX Thor. Pre-geometry: [`max/INTERFACES.md`](max/INTERFACES.md)
is deliberately the first artifact, and geometry starts only against frozen
interfaces.

## What we learned sizing max to jump

Unitree announced **Superman** on 17 Aug 2026 — a claimed 2.0 m standing jump and
12.66 m/s top speed on 0.85 m legs and 360 N·m joints. Company demo, no independent
verification, no hands, and it crashed into an electrical box in the video. Still,
it is the number to design against, so [`family.py`](family.py) puts vendor limits
against it rather than picking a target by feel.

The result is not the one we expected. **rose max is speed-limited, not
torque-limited.** rana-190 has torque to spare — the energy bound alone permits
0.97 m — but its 9.9 rad/s no-load output speed caps the jump at **0.38 m**.

And the reason is electrical, not mechanical. At peak current,

```
P = (Kt · I) · (V − I·R)/Kt = I · (V − I·R)
```

**Kt cancels.** Every actuator in the rana family delivers the same 1.3 kW peak
mechanical power, because peak power is set by the bus and the current limit alone
— 48 V × 30 A — and gearing only trades torque against speed inside that envelope.
The ladder from rana-100 to rana-190 is a *torque* ladder, not a *power* ladder.

So jump height on this machine is an electrical spec:

| target | takeoff | knee rate | knee torque | both knees | vs available |
|---:|---:|---:|---:|---:|---:|
| 0.3 m | 2.43 m/s | 8.9 rad/s | 153 N·m | 2.7 kW | 1.0× |
| 0.5 m | 3.13 m/s | 11.4 rad/s | 211 N·m | 4.8 kW | 1.8× |
| 1.0 m | 4.43 m/s | 16.2 rad/s | 356 N·m | 11.5 kW | 4.4× |
| 2.0 m | 6.26 m/s | 22.9 rad/s | 646 N·m | 29.6 kW | 11.3× |

Three levers, and a bigger rana is not one of them: **raise the bus** (100 V+ leg
drive), **raise the current limit**, or **store energy in series elastics** so the
motors can work slowly and release fast — which is how every good biological and
robotic jumper does it, and the only lever that beats the V·I ceiling instead of
paying for it.

Worth noting what this says about Superman: 360 N·m at ~21 rad/s is ~7.5 kW *per
knee*, roughly 6× a rana-190. Their actuators are power-dense where ours are
torque-dense. That is a design choice we have not made yet, and now we know it is
the choice that matters.

## Honest limits on the above

The jump model is planar, knee-dominated, and first-order — vendor limits in, a
number out, no controller search, in the same spirit as the ollie feasibility work.
It ignores the hip and ankle contributions that would raise it and the transmission
losses that would lower it. Both bounds it reports are *ceilings*: the real machine
lands under the lower one.

`family.py --pack` re-runs pro's real packing check on each scaled skeleton, but it
**cannot check max** — only rana-100 rev D has a detailed housing (⌀100 × 44), and
rana-140/190 envelopes are not modelled. Inventing them would make the check a lie,
so max's packing is blocked on those two drawings.
