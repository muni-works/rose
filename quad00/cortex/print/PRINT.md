# Cortex head: desk fit print

Goal: print a cortex head and fit Cam's Jetson Orin NX on its **Seeed reComputer J401** carrier, on the desk, then iterate. Printer: Bambu P1S (256 mm bed, enclosed), **PETG**.

**Print the FULL head first.** The J401 is 100 × 80 mm, the Orin Nano dev-kit footprint, and it doesn't fit the mini: at module height there's only 69–77 mm of depth there. The full head has about 130 mm. Minis will take an Orin Nano or NX on a compact carrier (Seeed A603 class, 87 × 52). The mini STLs are here too for later.

## 1. Fit coupons first (about 1 hour)

`coupons-plate.stl` is one plate, 240 × 156 mm. Settings: PLA or PETG, 0.2 mm layers, 3 walls, 20 % infill, no supports, flat as modelled. The notches along each coupon's edge mark the variant (1 notch = smallest clearance).

| Coupon | Variants | What to do | Lock it into |
|---|---|---|---|
| Inserts | M2.5 heat-set holes 3.4 / 3.6 / 3.8; M3 holes 4.0 / 4.2 / 4.4 | Press an insert into each hole. Keep the hole where it goes in square and holds without cracking the boss. | Carrier and sled standoffs (M2.5); tub-to-floor screws (M3) |
| Board slot | 1.7 / 1.8 / 1.9 / 2.0 wide | Slide in a carrier edge, or any 1.6 mm PCB. Keep the slot that's snug with no rattle. | Board guides |
| Magnet pockets | Ø8.1 / 8.2 / 8.3 × 3.1 deep | Ø8 × 3 magnets or steel discs: press-fit vs drop-in. | Floor strike pockets, neck magnets |
| Dock face | Full-size pad pocket, cone seats, latch slot | Check that the cones seat without rocking and the latch hook clears. | The floor's dock cuts |
| **J401 template** | Plate with the assumed 86 × 58 hole pattern (Ø3.2 with Ø2.2 pilots) and a port-edge ridge | Lay the real J401 on it. Do the holes line up? Do the ports sit on the ridge edge? | `j401-holes` in `cortex-full.loon` |
| Tub-to-floor lip | 0.15 / 0.25 / 0.35 per side (floor strips + wall rings) | Keep the one that drops in by hand with no play. | `c` in `head-print.loon` (0.25 now) |

## 2. Head shells

Regenerate the STLs with `./make_print.sh head-print-full.loon full` (or with no arguments for the mini). Set `VCAD=` to the vcad CLI. The full head is listed first; the mini parts use the same settings.

| STL | Orientation | Settings |
|---|---|---|
| `stl/cortex-full-tub.stl` (158 × 230 × 190) | Upright on its open bottom | PETG, 0.2 mm, 3 walls, 15 % gyroid, enclosure closed, same support rules as the mini tub below |
| `stl/cortex-full-floor.stl`, `-sled`, `-ear-grilles`, `-exhaust-grille`, `-glass` | As the mini rows below | PETG. The sled is four pillars with M2.5 insert holes under the J401's corners. |
| `stl/cortex-mini-tub.stl` (102 × 164 × 118) | Upright on its open bottom | PLA, 0.2 mm, 3 walls, 15 % gyroid. **Tree supports inside only:** the crown roof and the top edge of the face opening. The outside needs none. Bambu: "support on build plate only" off, "tree (auto)", top Z distance 0.2. |
| `stl/cortex-mini-floor.stl` (98 × 158 × 9) | Flat, dock face down | 0.2 mm, 4 walls, 30 % infill, no supports. Has the 2.5 mm registration lip that drops into the tub. |
| `stl/cortex-mini-sled.stl` | Flat | 0.2 mm, 4 walls, 40 % (the product part is 2 mm folded aluminium) |
| `stl/cortex-mini-ear-grilles.stl`, `-exhaust-grille.stl` | Inner face down | **0.12 mm layers** for the 1.4 mm holes, 2 walls, no supports. Contrast colour (the colorway accent) if you have it. |
| `stl/cortex-mini-glass.stl` (optional) | Face down | Smoke PETG, or skip and tape on 3 mm smoked acrylic cut to the opening |

- Mesh repair: Bambu Studio may offer to repair the tub, floor and exhaust grille (a few open edges from the booleans). Accept it.
- Assembly: the floor drops into the tub. Tape it, or use M3 screws once the insert size is locked (bosses come in the next pass).

## 3. Jetson / J401: what's modelled and what isn't

The full head carries the J401 (100 × 80) on four sled pillars, at z 38 above the 3S2P pack. Its port edge faces the back cutout, which is about 94 × 20 mm. The module and fan heatsink sit above it and blow up into the crown. Closest point of the board stack to the tub: 2.3 mm.

**Assumed, check on the template and the desk:**
- The hole pattern: 86 × 58 (dev kit), M2.5/M3
- Port-block height (16 mm) and depth from the edge (24 mm)
- The module and fan-heatsink footprint (60 × 70) and height (32 mm above the board)
- Which J401 it is: the 100 × 80 carrier (modelled) or the 115 × 115 Robotics J401, which needs a remodel but still fits only the full head
- Display thickness, camera depth and FPC exits

**Known small issue:** one sled pillar touches the tub's curled-in bottom by about 0.5 mm. The print's flex absorbs it; it gets fixed with the measured hole pattern.

## 4. What to check on the desk

1. **Board fit.** Does the carrier sit on the sled within the tub with ≥ 2 mm to every wall? Does the tub drop over it from above without catching on any connector?
2. **Connector and cable access.** Do the carrier's ports line up with the back cutout? Can you plug and unplug USB, Ethernet and DC with the head closed? Is there room for the CSI and HDMI ribbons to fold without kinking (≥ 3 mm bend radius)?
3. **Airflow.** The heatsink fan pulls from the open crown space and exhausts aft through the back dot grid. Is the fan ≥ 5 mm from the crown? Run it closed for 10 minutes under load and note the module temperature.
4. **Cameras and lenses.** Hold the cameras behind the glass position. Is the face opening's edge outside the lens FOV? Is the 106 mm baseline symmetric? Is there room behind for the FPCs?
5. **Dock.** Seat the floor on a printed neck dock. Do the cones locate it without rocking? Do the magnets pull it flat (if fitted)? Does the latch slot line up?
6. **Feel.** Weight in the hand, how it sits on the desk, whether the face rake reads right at desk height.

Write the measurements here or in a note, and the next pass folds them in.
