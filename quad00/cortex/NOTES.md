# cortex print prep: working notes (frog/head-print-prep)

Checkpoint log for the 2026-10-26 "print a head case today" push. Newest last.

## 1. State found

- `cortex.loon` last changed in 70969fd; `cortex/stl` and `fab/cortex` are from the same commit (Oct 20).
- `export.sh` points at `~/Developer/vcad/target/release/vcad`, which doesn't exist (the target dir is
  `~/.cache/cargo-target`, and vcad HEAD no longer builds against `~/Developer/tang`: tang-expr is 0.2 there,
  vcad wants ^0.1). Built vcad from a `git archive` copy in /tmp/vb with tang at dab80ad5^ (tang-expr 0.1.1).
- Old exports: `cortex-tub.stl` and `cortex-exhaust-grille.stl` are **not watertight** (the grille has 0 volume).

## 2. What the model is missing for a printed v1 (all SPEC-described, none modelled)

| Gap | SPEC says | Today |
|---|---|---|
| Tub-to-floor fixing | 4 × M3 csk from below into heat-set inserts in tub bosses ("bosses not modelled yet") | add corner bosses + insert holes in the tub, csk holes in the floor |
| Glass seat | "0.5 mm VHB in the stepped face opening" | the step (inset 2 at xfz-3..-8) lies behind the 2.5 mm wall, i.e. in air: the glass has nothing to sit on. Add a ledge ring behind the glass, notched round the display and cameras |
| Sled | 2 mm Al brackets, folded foot, 2 × M2.5 to the floor; carrier on top flanges | modelled as two plain 2 × 35 walls at y ±44..46, *outboard* of the carrier (±43.5), no foot, no flange. Print variant needs a foot and an inward top flange (blank, drill per MEASURE #5) |
| Face frame + its 4 bosses | printed frame behind the glass, screwed to tub bosses | not modelled. Not today: display/cams to the glass with foam/VHB for v1 |
| Grille retention | flush inserts | zero clearance, nothing behind them. CA from inside; slicer XY compensation |
| Glass fit | | glass outline = opening outline, zero clearance. Cut the acrylic ~0.2 mm under per side |

## 3. Decisions (2026-10-26)

- **Interior adds only, exterior untouched** (SPEC: exterior frozen). Each add is taken out of the cavity before the
  cavity comes out of the solid, so nothing is unioned face-to-face with the skin.
  - Corner blocks: Ø18 cylinders on the 4 plan-corner centres, z 3..14. M3 insert hole Ø4.0 × 7, then a Ø3.2 tip to
    z 12. The screw sits 3.5 mm past the corner centre along the diagonal, so the floor's countersink (Ø6.6) clears the
    bottom round by ≥ 2.6 mm.
  - Glass band: the glass outline grown 3 mm, xfz-4.5..-1.0 (tilted). face-opening's existing step then cuts the 2 mm
    lip at xfz-3, as designed. Notched 0.5 mm round the display and camera envelopes (the display's top edge sat
    exactly on the lip line). The glass model is 3.8 thick with its back on the lip. Real stack: 0.5 VHB + 3 mm
    acrylic, so the front sits 0.5 proud. Flagged, not changed.
  - Floor: 4 × M3 csk + 4 × M2.5 csk (sled feet at x -37.5 / -5.5, y ±50).
  - `cortex-sled-print`: new part; the Al `cortex-sled` is unchanged. 3 mm wall y ±43..46, foot y ±46..54 (3 thick),
    flange y ±38..46 at z 35..38, under the carrier edge (±43.5). The pack (±37, top z 36) clears by 1 mm.
- **Clearances re-checked** (manifold min_gap, all internals): the only zero gaps are SPEC's deliberate mounts. Cables
  (proxy not watertight): 0 points inside tub / floor / sled.
- **vcad tessellation bug:** a single countersink in the floor gives 131 open edges. Every leaf is watertight, so
  `print_build.py` runs the top-level booleans in manifold3d. Not fixed in vcad (out of scope today; report to the vcad manager).
- **Bambu Studio CLI:** it loads the configs once they are flattened, with `inherits`, `from: User` and
  `extruder_variant_list` cut to Standard, then segfaults (rc 139) under the sandbox. The print times are estimates.
- fab/README said "tub face up (+X up: support-free)". That's wrong: face up, the back sweep and crown overhang about
  16,700 mm². Corrected there.

## 4. Back panel + displays (2026-10-26, Cam: "close the hole in the back, try a 7-inch; I also have a 4-inch for the mini")

- **Back:** the I/O window stays in the wall (it is the intake) but is now covered by a perforated insert,
  `cortex-back-grille` (Ø1.4 on 2.2 hex, same as the other grilles; ~330 holes, ~510 mm²). The head charges and talks
  on the desk dock through the floor; the carrier's ports are reached only with the tub off. A fully solid back
  fails the SPEC thermal check at 25 W (intake would be only the ear grids' ~80 mm²).
- **7" (Cam's Hosyond): does not fit the mini.** Waveshare-class 7" HDMI (C): 164.9 × 107 × 8, active 154 × 86.
  The mini's face band is ~126 × 82 and the whole head is 121 tall. It wants the full-size head (CORTEX.md already
  routes the 5" (H) there). No geometry written.
- **4" square (Waveshare 4inch HDMI LCD (C), 720×720) — fits, modelled.** Numbers taken from its own STEP
  (waveshare wiki 3D drawing): 84 × 84 × 1 glass, ~69 × 58 PCB, ~10 mm to the connector backs, connectors + 5
  standoffs off one edge. Variant parts: `cortex-tub-d4` (79 × 79 window, 2.5 mm bond land all round for a VHB
  bead under the display's own glass, which sits 1 mm proud as the dead front), `d4-glass` / `d4-body` proxies,
  cameras moved outboard to y ±55 (baseline 110) with Ø7.5 lens holes through the face wall. Ear grids, speakers,
  mics, sled, dock, battery all unchanged and re-checked clear (min gap: sled 1, battery 5.8, tub 5).
  The display is centred on the face band; eye height `ze` unchanged.
- vcad loon front-end bug found: `[cylinder r h]` inside a `fn` body fails ("no matching clause for anonymous with
  1 args"); `[cylinder-n r h n]` works. Worked around; report to the vcad manager.
- 4" open: its HDMI/touch connectors sit on one edge (~12 mm deep) — check which edge Cam's unit has and that it
  clears the sled; its touch runs over USB-C (a second cable to the A603) or I2C.

## 5. Full head, 7-inch (2026-10-26, Cam: "let's do the 7-inch full head")

- **One source, every size:** cortex.loon now takes its four shell planes (and the 7-inch envelope + placement) from
  `size.loon` (mini, the default; output bit-identical to before) or a full-size file copied in as `size.loon`.
  loon gotchas found: `[use m]` binds `m.name` only (bare names need `[use m [a b ...]]`), and unary `[- x]` on a
  module-imported value panics the interpreter (builtins.rs `-` reads args[1]) -- use `[- 0.0 x]`.
- **7-inch (Waveshare 7inch HDMI LCD (C), 164.9 x 107 x 8, active 154.21 x 85.92)** mounts **behind** the face wall:
  its PCB is bigger than any window with a bond land, so it can't sit proud like the 4-inch. Window = active + 2 mm;
  the glass VHBs to the inner face; the tub is the bezel. Active area centred on the outline: ASSUMED.
- **The open call is where the eyes go** (the 7-inch leaves ~10 mm of flat face each side on a straight 1.4x mini).
  Mockups in `mockups/` (`full-above|beside|corner.loon`, `render.sh`, `lineup.png`): above 216 x 146 x 192,
  beside 272 x 146 x 169, corner 229 x 146 x 169. The full body's neck currently sizes its head ~170 x 188 x 127
  (scaled K1 helmet), so the body proportions get re-checked whichever wins.
- Photoreal with the face UI lit on the screen (`mockups/ui_face.py` -> ui-face-1024x600.png, `photoreal.py`) needs
  emissive + image-mapped materials in vcad-render: being added by a subagent in /tmp/vb/vcad as a patch
  (/tmp/vb/vcad-emissive.patch) -- to land in vcad via the vcad manager, not committed from here.
