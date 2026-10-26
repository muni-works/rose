# cortex head: print package (mini v1)

Generated from [`cortex/cortex.loon`](../../cortex/cortex.loon). Regenerate it; don't edit these files by hand:

```
cd quad00/cortex
sh export.sh              # vcad: stl/ proxies + fab/cortex/*.stl|step (model frame)
python3 print_build.py    # watertight tub/floor/exhaust grille + fab/cortex/print/ (laid flat, ready to slice)
sh print_render.sh        # renders/print/*.png
python3 print_check.py fab/cortex/print/*.stl   # wall / overhang / bed-contact report
```

**Slice the files in `print/`.** They are watertight and already in print orientation: drop them on the plate and
don't rotate them. The model-frame `*.stl` here match them. vcad's own STLs for the tub, floor and exhaust grille have
open edges (see `cortex/NOTES.md`). The `*.step` files come from vcad's BRep, so use them for CAD reference, not for slicing.

Printer: Bambu P1S, 0.4 nozzle (a 0.2 nozzle is better for the grilles). Times are **estimates** from part volume,
not a slicer run. Check them when you slice.

## Print order

The sled and floor go first. They are short, they let you fit the real A603, pack and inserts, and the tub's insert
holes have to line up with the floor's countersinks. Start the tub only after the floor checks out.

| # | Plate | File(s) in `print/` | Material | Layer | Orientation (as laid) | Supports | Other settings | Est. time |
|---|---|---|---|---|---|---|---|---|
| 1 | Spine | `cortex-sled-print-l.stl`, `cortex-sled-print-r.stl`, `cortex-floor.stl` | PETG (PA-CF if you have it) | 0.20 | Sled: on its end, the Z-shaped profile flat on the bed (+X up), so the foot and flange bends are in-layer. Floor: desk face down. | Sled: none. Floor: **tree, on build plate only**, for the 3 mm bottom round round the rim (a ~70° overhang). Pockets, cone seats and countersinks all open down onto the bed and need none. | 4 walls, 40% gyroid. Sled: 5 mm brim. | ~1.5–2 h |
| 2 | Tub | `cortex-tub.stl` | PETG, black (ASA is fine in the P1S enclosure) | 0.20 | As assembled, open side down. The outside is support-free. | **Tree (auto), everywhere**, 30° threshold. They land inside the crown, under the top edges of the face opening, the I/O cutout and the exhaust window, and under the glass lip. All of them come out through the open bottom. | 5 walls (= the 2.5 mm wall), 15% gyroid. 5 mm brim, then trim it: the bottom is the face that mates with the floor. | ~5–7 h |
| 3 | Grilles | `cortex-ear-grille-l.stl`, `cortex-ear-grille-r.stl`, `cortex-exhaust-grille.stl` | PETG for the exhaust grille (hot air off the heatsink). PLA is OK for the ears (intake). | 0.12 | Ears: outer face down (flat). Exhaust: standing on its long straight bottom edge, as assembled. It leans less than 30°, so it prints support-free. | none (the Ø1.4 holes bridge themselves) | 100% infill. X-Y hole compensation +0.1. **X-Y contour compensation −0.1**, because the grilles are line-on-line with their windows. Exhaust grille: 5 mm brim. | ~1.5 h |

Hole counts check against SPEC: ear grilles 161 each, exhaust 417, all Ø1.4 on a 2.2 mm hex pitch. The webs are 0.8 mm (2 lines at 0.4).

## Hardware for the printed v1

| Joint | Parts | Notes |
|---|---|---|
| Floor → tub | 4 × M3 heat-set insert (Ø4.0 hole, 7 deep; Ruthex-type M3 × 5.7) in the tub corner blocks; 4 × M3 × 8 countersunk from below | Insert hole size is the insert maker's usual figure. Check it against yours. |
| Sled → floor | 4 × M2.5 × 10 countersunk from below + 4 × M2.5 nut on the foot | Nuts are reached from above with the tub off (SPEC: no tool inside a closed head). |
| Carrier → sled | 4 × M2.5 through the top flanges | **Flanges are blank: drill them to the A603** ([MEASURE.md](../MEASURE.md) #5). |
| Grilles → tub | CA or a bead of epoxy from inside | Flush inserts. Nothing holds them mechanically. |
| Glass → tub | 0.5 mm VHB on the 2 mm lip | See "glass" below. |

## Not printed / not in this package

- **Glass:** 3 mm smoked acrylic, laser-cut from [`glass.dxf`](glass.dxf). The outline is the same as the face opening (zero clearance), so cut it **0.2 mm under per side**. Mask the back black except the screen and the 2 camera windows.
- **Keeper:** 1 mm spring steel (laser), not printed. Strikes: steel discs.
- **Dock body side** (`dock-face`, `dock-pawl`, `dock-release`): these belong to the neck, not the head case. If wanted: pawl and release in PA-CF, pawl −Z up, release +X up.
- **Face frame** (holds the display and cameras behind the glass) and its 4 tub bosses: in SPEC, **not modelled**. For v1, foam-tape the display and cameras to the glass.

## Renders

`cortex/renders/print/`: `tub`, `tub-inside` (corner blocks, inserts, glass lip), `floor-top`, `floor-bottom`, `sled-print`,
`sled-on-floor` (with the pack and carrier proxies), `ear-grilles`, `exhaust-grille`.
