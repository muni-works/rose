# mini v1 fab package

**Status: package only. Nothing has been ordered. Cam approves the design before anything is cut.**

This is the frozen mini humanoid (100 cm, 22 DoF) with the cortex head. Everything below is generated from the model, so regenerate it rather than editing it by hand:

```
cd body
python fab/make.py          # plates + shells + glass  (needs numpy trimesh manifold3d shapely ezdxf rectpack)
(cd ../head && sh export.sh)   # cortex.loon -> head/fab/cortex/*.step|stl and head/stl/* (the head proxies)
python rom_mini.py          # range of motion, with the loon head
python harness_check.py     # cables clear everything; service loop lengths
```

## Freeze

- **Mini geometry:** `k1_design.py` + `k1_structure.py` at this commit. The only change in the cortex port is at the neck dock, compared to the parent commit `b9fa190`. The pogo and latch windows replace the retired USB-C plug cutout in `head-dock` and `trim-headdock`, and the spine leaves the pogo block instead of the plug. Every other mini part has the same volume and bounds.
- **Cortex geometry:** [`head/cortex.loon`](../../head/cortex.loon) at this commit. The Python checks load its parts as STL proxies (`head/stl/`, via `k1_structure.cx_parts`). `cortex.py` is gone.

## What to make

| What | Files | Make it | Qty |
|---|---|---|---|
| Frame plates | [`plates/sheet-1..4.dxf`](plates) nested, [`plates/parts/*.dxf`](plates/parts) one each, [`plates/cutlist.csv`](plates/cutlist.csv) | 1/8" (3.2 mm) 5052-H32 on the Anolex 4040 (390 × 390 sheets, 6 mm edge, 8 mm webs). Layer `CUT` = through-cut, `NOTE` = labels (don't cut). Units mm, true size. | 4 sheets, 31 distinct plates, 48 pieces |
| Body skins | [`shells/*.stl`](shells), [`shells/shells.csv`](shells/shells.csv) | PETG on a P1S, 0.2 mm layers, 5 walls (= the 2 mm skin), 15% gyroid. Soles in TPU 95A. Halves `-a` / `-b` print split face down. | see shells.csv |
| Cortex tub | `head/fab/cortex/cortex-tub.step/.stl` | PETG (production: PC/ABS moulding), open side down (+Z up). Face up is **not** support-free: the back sweep overhangs. Print from `head/fab/cortex/print/`, see [`head/fab/cortex/README.md`](../../head/fab/cortex/README.md) | 1 |
| Cortex floor | `head/fab/cortex/cortex-floor.step/.stl` | v1: PETG or PA-CF, flat bottom down (tree supports on the build plate, for the rim round; the cone seats need none). Production: 3 mm aluminium. | 1 |
| Cortex grilles | `head/fab/cortex/cortex-ear-grilles`, `cortex/cortex-exhaust-grille` (.step/.stl) | PETG, 0.12 mm layers, 0.25 mm nozzle for the Ø1.4 holes (or resin) | 1 + 1 |
| Cortex sled | `head/fab/cortex/cortex-sled-print.step/.stl` (v1, with foot + top flange); `head/fab/cortex/cortex-sled` is the Al part | v1: PETG or PA-CF. Drill to the A603 pattern ([MEASURE](../../head/MEASURE.md) #5). Production: 2 mm aluminium, folded. | 1 pair |
| Latch | `head/fab/cortex/dock-pawl`, `cortex/dock-release` (.step/.stl), plus `cortex-keeper` | Pawl + release in PA-CF (pawl: -Z up; release: +X up). Keeper laser-cut from 1 mm spring steel. | 1 each |
| Face glass | [`head/fab/cortex/glass.dxf`](../../head/fab/cortex/glass.dxf) (126.4 × 79.6, R21.9) | Laser-cut 3 mm smoked acrylic. Mask the back black except the screen and the 2 camera windows. | 1 |
| Everything bought | [`BOM.csv`](BOM.csv) | | |

## Checks (with the loon head)

- **ROM** ([`checks/rom.txt`](checks/rom.txt)): every joint reaches what a K1-class walker needs. Head yaw ±57°, pitch −20..+49° (+40° needed). No collisions.
- **Harness** ([`checks/harness.txt`](checks/harness.txt)): every cable clears every part except its own ends, at rest and across head yaw and pitch. The neck's two service loops need 36 mm of worst-case gap each, so cut them at 46 mm (CORTEX.md guessed 40 and 60–80). The spine runs inside the chest shell, and `make.py` cuts its way through the skin.
- **Plates** all fit the 4040 bed. The longest is `shin-outer` at 322 mm. The nest was checked: every piece is inside the bed and gaps are ≥ 8 mm.

## Read before cutting (open, not fixed here: no new shape work)

1. **Neck magnets vs the dock plate.** cortex.loon puts the neck's N52 Ø8 magnets at y ±30. The body's dock plate is only ±29.7 wide and its trim ±32.7, so the magnets overhang both. This package leaves the magnet seats out of the plate. Pick one: move the magnets to y ±24 in cortex.loon (head strikes too), or widen `head-dock` to ±34.
2. **`chassis-rollmount` (hip-roll RS06 mount)** is clipped to a 2 mm sliver by the shell clip in `k1_design.structure()`. The DXF is cut from the **unclipped** profile (89 × 87). Check it against the chest/pelvis skin before cutting, or exempt it from `clip` like `hipR`.
3. **RS06 bolt patterns are ASSUMED** (`quad.py` MOTORS). Hold a motor against a 1:1 print of `thigh-inner_ll.dxf` first.
4. **Skins are generated, not designed.** They are hollowed to the 2 mm wall the plates were clipped to, with 0.4 mm cleared round everything inside and a 0.5 mm running gap to the next body's shell. Each is split on the axis that loses the least skin, so each half slides off clear of the frame. No bosses, laps or fasteners are modelled: bond with CA, or melt in M3 inserts where a half sits on a plate edge. Check the halves in a slicer before a long print.
5. **Three covers come out empty**: `cover-shoulder`, `cover-shoulderyoke` and `cover-neck`. Their form is only a pad over a motor, so clearing the motor leaves nothing. v1 runs those joints bare. A designed cover is follow-up shape work. The chest skin is printed in 4 pieces to fit the P1S.
6. **Torso rails and latch** are 3.0 mm in the model. Cut them from the same 3.2 stock.
7. **Not in this package:** the limbic board, pack case + BMS, pack dock, foot nodes and the head's dock interface board. These belong to the limbic package (rose `claude/limbic-spec`).

Bench numbers still to get: [MEASURE.md](../../head/MEASURE.md) (fan curve, latch pull-off, real 25 W thermal).
