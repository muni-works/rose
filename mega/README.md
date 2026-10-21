# rose mega

Top of the **rose** line — our humanoid. The name is the thesis: past tense of
*rise*, the one that stood up. Third machine of the pond: **rana** (actuators)
→ **frog** (quadruped testbed) → **rose**.

The line has three variants, largest last:

| | | |
|---|---|---|
| **rose mini** | 950 mm, 22 DOF, generic A/B/C actuator SKUs | `../rose-mini` |
| **rose full** | **1800 mm**, 23 DOF on MyActuator X-series (X15-450 hip/knee); pushrod ankle, waist yaw; see `../full/` | `../full/` + `vcad/hardware/rose-pro` |
| **rose mega** | **2000 mm** on our own rana + torc; closed sections, pushrod ankles. Geometry here is rose-1, the 1210 mm study; the 2000 mm re-scope is in INTERFACES.md | here |

rose full is the one you can order parts for today; rose mega is the one that is
ours end to end. mini is the crashable one.

**Status: pre-geometry.** [INTERFACES.md](INTERFACES.md) is deliberately the
first artifact; geometry starts only against frozen interfaces.

## What rose inherits (see frog/FINDINGS.md for the full archaeology)

**Hardware, as-is:** the rana family (140 legs / 100 arms, verified EM at
kw 0.933), the 48 V architecture with regen path, one controller PCB in three
FET populations, torc-or-CAN data plane, the fastener/bearing ladder.

**Patterns:** pushrod-driven distal joints (frog's swept-and-debugged four-bar
IS the humanoid ankle), double-shear sandwiches straddling actuators,
bulkhead+spine skeleton with non-structural skin, axial separation of
perpendicular hip actuators, software joint limits near frames.

**Process (the biggest one):** generated mirrors, swept interference, receipts,
consumer-tool export validation, mount-is-the-heatsink thermal design, and
frog as the crash-test vehicle — rose's falls are expensive; frog exists so
the control stack arrives here already debugged.

**What does NOT transfer, honestly:** static stability (rose is a controlled
fall — estimator criticality jumps from important to existential), sheet-metal
primary structure at rose-2 loads (closed sections/machined fittings return
for the big links), and cheap crashes.

## First milestones

1. rose-1 leg on a vertical rail — single-leg hop/balance rig, rana-140 ×2 +
   pushrod ankle. Reuses frog's dyno mount, power cart, and torc/CAN stack.
2. Pelvis + two legs, harness-supported stand.
3. Free standing balance. Then walking. Hopping last — rose earns it.
