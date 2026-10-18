# quad00 leg

One leg of an RS00 quadruped, with its hip yoke and ab/ad motor, on a bench bulkhead. All twelve joints of the dog are RobStride RS00.
The knee is driven from the hip through a gear train cut from the same flat stock as everything else.
There are no belts, rods or cables.

![iso](render/leg-iso.png)

```
bulkhead (torso) ── ab/ad RS00 ── arch ══ keyed tenons ══ side plates ─┬─ hip RS00 (outboard) ──> thigh-outer plate ┐
                                                                     └─ knee RS00 (inboard) ──> pinion z17        │ thigh fork, 2 × 3.2 mm
                                                                                                 idler z17         │ idlers on M8 shoulder bolts
                                                                                                 idler z17         │ + 8×10×8 plain sleeves
                                                                                                 knee gear z34 ════╧═ shin ── TPU ball paw
```

The upper assembly is plates and keys only, with no rods or standoffs:

- **The arch is the only bridge** between the two side plates. It sits ahead of the hip, in the
  forward-up sector where a quadruped thigh never goes, so the thigh swings 81° forward (under the
  arch's leg-plane notch) and 184° back, over the top: 265° of travel. That is about Go2's thigh range.
- **The side plates key into the arch** with tapered keys through tusk tenons: the classic flat-pack
  joint. It is 2D-cuttable, self-tightening, and needs no edge-drilled holes or threads in plastic.
- **The ab/ad axis lies in the leg plane** (y = 0), 60 mm above the hip. The arch bolts straight to
  the ab/ad output, so standing load puts almost no torque on ab/ad.
- **The paw is a TPU ball** that slots over the shin tip and is held by two cross bolts, so it can't spin.

| | |
|---|---|
| links | thigh 148.75 (hip to knee axis), shin 150, standing hip height 255 |
| knee | 2:1, module 2.5, 20°, 3 × 3.2 mm face; 0.2 mm backlash per mesh (`BACKLASH`) |
| stock | 3.2 mm everywhere; gears and shin are 3-layer laminates, side plates, arch and bulkhead are 2-layer |
| width | 134 mm over the side plates; the leg itself is 17 mm thick |
| ranges | knee folds to 35° interior (shin meets idler 2); hip 81° forward to 184° back from straight down |
| mass, per leg | 900 g aluminium / ~230 g printed (bulkhead excluded), + 3 × RS00 (930 g) |

## Files

```bash
pip install shapely trimesh manifold3d ezdxf mapbox_earcut numpy networkx lxml
python3 leg.py              # checks + out/dxf (router) + out/stl + out/3mf (P1S plates)
python3 leg.py --check      # gear mesh sweep, stack, ranges only
python3 leg.py --assembly   # posed parts + out/assembly/leg.loon
python3 assembly_steps.py   # step renders + out/assembly-guide.html
```

DXF names are `part-Lk_3.2mm_xN`, where N is how many to cut for one leg. `_csk-1` means one of
those N gets 90° M3 countersinks; the countersink circles are on layer `CSK_M3_90DEG`.
STLs are fused laminates, already lying flat for printing, and watertight. `out/3mf/` has them
shelf-packed onto Bambu P1S plates (256 × 256): `plate1_PETG_links`, `plate2_PETG_yoke` (arch, 2 side
plates, bulkhead), `plate3_PETG_gears-keys`, and `plate4_TPU_paw`. Open each in Bambu Studio, set the
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
  gear-to-plate gap. The shafts are Ø8 × 20 shoulder bolts, with two 1.6 mm washers under each head
  so the shoulder sets the 16.8 mm plate spacing.
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
- The torso isn't designed yet; the bulkhead stands in for it on the bench.
- 900 g of aluminium per leg (yoke included) is heavy. The candidates to lighten are the arch (225 g), the side plates (214 g) and
  the shin laminate (202 g). Re-run the RS00 sizing sweep with the real masses.
- The paw's flat sits on the shin axis, so it only touches the ground with the shin vertical; it is there for printing.
