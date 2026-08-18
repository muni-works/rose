# rose mini — K1-scale humanoid

Smallest of the **rose** line (formerly `toad`); see `../rose-max` for the line.

A 950 mm, 22-DOF humanoid designed in [vcad](https://vcad.io): sheet-metal
structure, three actuator SKUs, full kinematic tree. Built 2026-07-25.

Sized to the Booster K1 class (roughly 0.95 m, sub-20 kg, 22 DOF). The
proportions and joint layout are designed to that envelope — they are **not**
measured from a real K1, and no Booster drawings were used.

```
cad/k1.loon        parametric source — the thing to edit
cad/k1.vcad        evaluated IR: 13 parts, 23 instances, 22 joints
render/            4096 / 8192 px PNG, transparent background
sheet-metal/       thigh channel STEP + DFM results
analysis/          actuator sizing + mass budget (runnable)
tools/loon2vcad.rs helper: .loon -> .vcad on stdout
```

## The robot

950 mm tall · 318 mm shoulder span · CoM at z = 446 mm, x/y symmetric to
0.003 mm · ~18 kg budgeted.

| chain | DOF | joints |
|---|---|---|
| head | 2 | neck yaw, head pitch |
| arm ×2 | 4 | shoulder pitch/roll, elbow pitch, wrist yaw |
| leg ×2 | 6 | hip yaw/roll/pitch, knee, ankle pitch/roll |

Ground link is the torso. Every child link is authored **in its own joint
frame** — the joints do the placing, not the instance transforms. Each link's
solid also carries the actuator housing that *drives* it, on that joint's axis,
so the linkage is geometrically real rather than implied.

Three actuator SKUs cover all 22 joints: **A** ⌀45×40 (12 Nm) ×4, **B** ⌀55×50
(25 Nm) ×10, **C** ⌀70×58 (55 Nm) ×8. Ankle pitch governs at 27.6 Nm — it owns
the longest lever to the CoM, as it should on a humanoid.

## Status

Geometry, kinematics, DFM and sizing are **done and checked**. Physics sim is
**blocked** on an upstream bug (below).

Two caveats worth carrying forward:

- **Mass is budgeted, not measured.** Don't read a mass off the geometry —
  actuator housings are modelled as solid cylinders, so a density-based number
  is badly wrong. The budget sums to 18.05 kg while the torque cases assume
  17.7 kg; closing that raises the torques ~2%, which changes no SKU choice.
- **No STL/GLB.** `export_cad` walks root parts only and an assembly has none
  (`roots: []`), and the `vcad` CLI fallback wouldn't build — `phyz-contact`
  in the sibling phyz repo is broken at its current main tip. The `.vcad` is
  the source of truth; meshes are one working build away.

## Two upstream bugs found

Both filed as background tasks against the vcad repo.

**1. Booleans against a filleted solid are broken.** Plain primitives are
exact — `shell 2` on a 40 mm cube gives 17344 mm³, watertight. The same shell
on a *filleted* cube gives 128805 mm³ with 365 unpaired edges, and a difference
against a filleted subject gives 652877 mm³ where ~115264 is correct. It can
also drive a part to inverted winding / net negative volume. Workaround used
here: **never fillet a body you will shell or subtract from.** Tell-tale
symptom is the triangle count exploding (447k vs 17k for the same assembly).

**2. Physics FK compounds a 90° error on non-Z joint axes.** In
`create_robot_env` / `gym_reset`, a revolute joint whose axis isn't Z rotates
the child body frame by the axis-realignment rotation and never composes the
inverse out. Correct at depth 1, wrong from depth 2, compounding as
−90°·(n−1) about X, with anchor offsets rotating along. A 3-link chain on Y
gives 0.5 / (0,−0.1,0.5) / (0,−0.1,0.6); the identical chain on Z gives
0.5 / 0.4 / 0.3, all identity. This robot's feet reported hip height. The
CAD-side FK is correct, so the defect is confined to the physics conversion.

There is no practical workaround for #2 — a humanoid can't have all-Z joint
axes — so the sim waits on the fix. It may also corrupt `gym_step` position
targets, not just the reported observation.

## Regenerating

```bash
# .loon -> .vcad  (needs tools/loon2vcad.rs installed as a vcad-loon example)
cargo run -q -p vcad-loon --example loon2vcad -- cad/k1.loon > cad/k1.vcad

# .vcad -> PNG
vcad-render cad/k1.vcad --format png --size 4096 --fill 0.9 \
  --azimuth 38 --elevation 14 -o render/k1-iso.png

# sizing
python3 analysis/actuator_sizing.py
```

`loon2vcad.rs` lives here because nothing in the vcad repo turns loon into a
`.vcad` on disk — it's the missing half of the documented docs-render
workflow. Drop it in `crates/vcad-loon/examples/`. Gotcha:
`eval_vcad_to_value` returns a loon value, not JSON; `eval_vcad` is the one
that returns a serde-serializable `Document`.

PNG renders come from the `vcad-render` CLI rather than the MCP `render_view`
tool on purpose: that artifact store is in-memory and dies with the session.
