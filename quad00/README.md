# quad00 leg

> **Locked (2026-10-18): the mini quad and mini humanoid** -- `quad.py`'s `hybrid` and `humanoid` builds.
> One limb module, four per robot, 12 × RS00, zero gears: the thigh is a box of two 3.2 mm plates with
> the hip and knee RS00 as its spacers (the L-yoke reaches the hip motor's back through a bearing hole in
> the inner plate; the knee output comes through the outer plate to the shin). Thigh 130, shin 170.
> Each limb hangs from a fore-aft ab/ad RS00. Quad: belly-only body, hips 380 x 140, 255 high, front
> yokes flipped so every ab/ad points into the body. Humanoid: the same module on a back plate, legs
> mirrored fore-aft (knees forward), arms with the yoke flipped (elbows back), ~630 mm tall.
> Open before building: the knee runs ~97% of RS00 in the non-backflip cases with the 170 shin; the
> ab/ad lever is 99 mm (89%) -- the inline variant (shin inside the box) cuts it to 35; the humanoid
> needs an ankle and arms outboard of the legs. Everything below this note describes the earlier geared
> leg, kept as the printed prototype.

One leg of an RS00 quadruped, with its hip yoke and ab/ad motor, on a bench bulkhead. All twelve joints of the dog are RobStride RS00.
The knee is driven from the hip through a gear train cut from the same flat stock as everything else.
There are no belts, rods or cables.

![iso](render/leg-iso.png)

```
torso ── ab/ad RS00 ── bent U yoke ─┬─ hip RS00 (outboard flange) ──> thigh-outer plate ┐
                                    └─ knee RS00 (inboard flange) ──> pinion z17        │ thigh fork, 2 × 3.2 mm
                                                                      idler z17         │ idlers on Ø8 clevis pins
                                                                      idler z17         │ + 8×10×8 plain sleeves
                                                                      knee gear z34 ════╧═ shin ── TPU ball paw
```

The upper assembly is **one part**: a U bent from 3.2 mm 5052, plus three motors and 14 screws.

- **All three motor axes lie in one plane**, level with the hip. The ab/ad sits beside the leg, in front
  of the knee motor and 40 mm inboard of the leg plane, so the thigh never sweeps through it.
- **The U's bridge is the only thing over the leg plane.** It spans the thigh's notch from 30 mm above
  the hip, so the thigh swings 88° forward (past horizontal) and 184° back, over the top: 272° of
  travel. `BRIDGE_Z` trades forward swing against metal.
- **The flanges are the motor mounts.** The hip and knee RS00 back-mount to them; the base carries the
  ab/ad output. Flat blank 263 × 105 mm, two 90° bends, inside radius 3.2, K 0.44
  (`out/dxf/yoke-blank_*.dxf`, bend lines on `BEND_UP_90`). The printed U is 6.4 mm thick, grown outward.
- **The ab/ad carries a 40 mm lever** to the leg plane: worst case 59% of RS00 peak (landing), 27% of
  stall standing. It back-mounts on the torso; a bench plate stands in for it.
- **The paw is a TPU ball** that slots over the shin tip and is held by two cross bolts, so it can't spin.

| | |
|---|---|
| links | thigh 148.75 (hip to knee axis), shin 150, standing hip height 255 |
| knee | 2:1, module 2.5, 20°, 3 × 3.2 mm face; 0.2 mm backlash per mesh (`BACKLASH`) |
| stock | 3.2 mm everywhere; gears and shin are 3-layer laminates; the yoke is one bent sheet |
| width | 127 mm over the yoke flanges (the ab/ad body stands 4 mm proud inboard); the leg itself is 17 mm thick |
| ranges | knee folds to 35° interior (shin meets idler 2); hip 88° forward to 184° back from straight down |
| mass, per leg | 620 g aluminium / ~160 g printed, + 3 × RS00 (930 g) |

## Files

```bash
pip install shapely trimesh manifold3d ezdxf mapbox_earcut numpy networkx lxml
python3 leg.py              # checks + out/dxf (router) + out/stl + out/3mf (P1S plates)
python3 leg.py --check      # gear mesh sweep, stack, ranges only
python3 leg.py --assembly   # posed parts + out/assembly/leg.loon
python3 assembly_steps.py   # step renders + out/assembly-guide.html + out/viewer.html (3D)
```

DXF names are `part-Lk_3.2mm_xN`, where N is how many to cut for one leg. `_csk-1` means one of
those N gets 90° M3 countersinks; the countersink circles are on layer `CSK_M3_90DEG`.
STLs are fused laminates, already lying flat for printing, and watertight. `out/3mf/` has them
shelf-packed onto Bambu P1S plates (256 × 256): `plate1_PETG_links`, `plate2_PETG_yoke` (the U and the
bench plate), `plate3_PETG_gears`, and `plate4_TPU_paw`. Open each in Bambu Studio, set the
filament and process, and slice.

The pinion hub layer is 3.6 mm in the STL. In aluminium, cut it from 3.2 mm and add a 0.4 mm shim
under it, so the teeth sit centred between the thigh plates.

## Printed prototype

The goal is fit, backlash, range and stair clearance. It is not built to take load.

- **Material:** PETG, or nylon-CF if you have it. Print the gears flat, at 0.12–0.16 mm layers, with 100% infill on the gears.
- **Knee current limit:** cap the knee motor at about **4 N·m** (PLA/PETG teeth). That is enough to stand
  (~1 N·m) and step slowly. Do not jump on printed teeth. In aluminium at RS00 peak × 1.5 shock, the Lewis safety factor is 2.1.
- **Real hardware:** see [assembly.html](assembly.html) for the full list and the build steps
  (`out/assembly-guide.html` is the same page with the images inlined). Corrections to earlier notes:
  the bushings are plain 8×10×8 sleeves, **not flanged**, because a flange won't fit in the 0.4 mm
  gear-to-plate gap. The shafts are Ø8 clevis pins, head outboard and E-clip inboard, with a 0.4 mm
  thrust washer each side of every gear. Nothing to tighten, and the inboard side stays within 1.5 mm
  of the plate, which is what lets the ab/ad sit beside the thigh.
  The inner thigh plate runs directly on the printed pinion hub (Ø34).

Tests:
1. Backlash at the paw: lock the knee motor, rock the shin, measure.
2. Knee range under power, 35° to straight.
3. On a real 180 mm step, the crouched poses where the stair sweep found a gear strike (hips ≤ 240 mm
   above the nosing line).
4. Standing load under the current limit; watch the teeth for creep.

## Open

- Tooth wear is not checked. Hard-anodise the aluminium version.
- The aluminium build needs a real bushing or bearing for the inner plate on the hub.
- RS00 stall torque is an estimate, scaled from the RS06 manual.
- The torso isn't designed yet; the bench plate stands in for it.
- The bent U's bend allowance assumes K 0.44 for 5052 air bending; confirm with the shop. The ab/ad bolt
  holes end 1.8 mm before the inboard bend starts: ask whether they will distort, or drill after bending.
- 620 g of aluminium per leg; the shin laminate (202 g) is now the heaviest part to lighten. Re-run the RS00
  sizing sweep with the real masses.
- The paw's flat sits on the shin axis, so it only touches the ground with the shin vertical; it is there for printing.

## The whole quad (side-hung)

`body.py` is the body; `quad.py` puts four legs on it (and on the corner-pod alternative) for the viewer.

- **Two flat decks + cross ribs, open sides.** The decks' plan shape is the silhouette: a chest ahead of
  the front hips and a belly between the hip pairs, both ±73 mm to hold the ab/ad pairs, and ±20 mm
  necks where the hip clusters tuck in. The thighs' E-clips pass 4 mm outside the decks, so the legs
  never cross the body, and there is no room for side walls (the decks carry it).
- **Ribs:** a nose wall and a mid wall (two layers each) take the front and rear ab/ad pairs on their
  back faces; rib A closes the belly, rib 0 is the splice where the front and rear deck halves meet, and
  a tail rib closes the tail. Every rib tabs into slots in both decks.
- **Every plate fits the 4040** (largest 307 mm): `out/dxf/body-*.dxf`.
- **Inside:** a 12S2P 21700 pack (44.4 V, 10 Ah, 1.7 kg) in the belly, a Pi 5 + CAN hat and the
  48 V → 5 V supply aft of the splice rib.
- **Width:** leg planes at ±87, 301 mm overall (Go2 is ~310).
- **Mass:** 9.9 kg with aluminium structure (legs 6.5, body 3.4), 7.2 kg printed; CoM 26 mm ahead of
  the hips' midpoint. 9.9 kg is about 9% over the RS00 sizing (9.1 kg), which puts the backflip knee
  at ~100% of RS00 peak: lighten, or drop backflips.
- **Leg-to-leg:** a front thigh swung back over the top and a rear thigh swung forward stop 32 mm apart.
