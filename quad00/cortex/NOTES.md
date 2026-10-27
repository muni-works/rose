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

## 6. Head-design research (2026-10-26, precedent sweep, findings applied)

- **Eyes are the whole expression budget** (Keepon: two dots; no mouth). Baby-schema sizing (Lorenz kindchenschema):
  each eye ~1/5 of the face-band width, centre spacing 1.5-1.8 eye-widths, set HIGH (forehead above, small chin
  below). Applied: ui_face.py eye geometry; eyes at 0.40 h.
- **Eyes must sit INSIDE the camera baseline.** Our eye graphics were at the camera spacing (110 mm) which reads
  as a camera rig, not a face. Graphics and cameras are decoupled: cameras keep the wide baseline, the drawn eyes
  sit inside it.
- **Off-state is the resting identity** (Vector: premium because almost nothing is ever lit). UI contract: black
  background, emissive eyes only, no widgets on the face; state goes to voice/app. Dead front: pane base tinted
  near-black so display-off = shell-black (Nest logic).
- **One accent only** (Braun AB1/ET66): the ear ring (or dock latch button) -- never the face or shell.
- **The skin is the vent** (HomePod): use the Ø1.4 hex pattern as the shell's own texture language; no freeform vents.
- **Uncanny-valley exposure is about heads first** (Mori 1970): the abstract two-eye face is the safe register; never
  go humanlike (no skin, nose, brows).
- **Personality is ~80% motion** (Disney/Pixar): invest in eye animation (blink cadence, saccades, idle gaze) over
  static detail.
- **Bezel metaphors, unify:** mini panel-proud (Vector-like) vs full shell-aperture (Nest-like) -- research favours
  the shell-aperture approach for both (shell = the only visible edge; no screen margin visible).
- Re-check queue if this becomes a spec: Vector's actual face optics (laser display, tint) and Eilik's eye
  proportions (closest shipped analog to the mini).
- **Visor dropped (Cam, after the exploded view):** no curved band, no second curvature. Option I = the egg skull
  IS the face: flat rectangular screen flush in a shallow chamfered recess, ears on the skull's own curve with the
  ring as a recessed groove, shorter chin (eyes at 0.42 h), more dome. Make the skull beautiful; the screen is just
  a rectangle in it.
- **Option J = the direction (Cam: "like the Figure head"):** no literal features (the ":O" read came from round
  eye holes over a mouth-shaped screen). The egg skull gets a FLAT FACE FACET (plane ~9 mm behind the crown), the
  4-inch panel flush in it behind a 1.2 mm reveal, and the EYES are glossy dark glass domes protruding 2.2 mm from
  the pane (21 x 27 mm, 1.45 eye-widths apart, high on the face): catchlights carry the expression, the emissive
  core sits BEHIND the dome in the real product (screen light through translucent dark glass). Off-state: monolith.
  Cameras hide behind the pane. Ears + recessed groove ring are the only character lines; one accent colour only.
  Renderer note: kosm-render 0.2 has no per-hit image emission (the emissive-map patch landed as constant-only,
  985238af), so the lit-face UI renders as geometry emissive slabs for now; the drawn-face pane returns when
  kosm-render gains image maps.
- **The mouth (Cam: "the lack of a mouth is unsettling"):** a soft emissive bar low on the face pane (26 x 7 mm,
  stadium section, 1.1 radiance) -- the waveform/mouth animation lives in firmware (dim glow = speaking, off = a
  dark slot). NOT a physical grille at the mouth: Cam found that horrifying, and the research agrees (a mouth is
  what makes it a creature; a drawn mouth does not hit the uncanny valley, a fake physical one might).
  Physical grilles stay at the EARS only, where speakers/mics honestly live.
- **Eye domes, DIY (off Amazon):** cast acrylic/PC half-domes ("clear acrylic dome 25/30 mm", $1-3) or glass
  cabochons ("20 mm glass dome cabochon", jewelry supply, ~$0.50 in bulk) -- Vector-style optics, factory gloss.
  22 mm matches the current eye. Bond with UV-cure adhesive or a VHB ring. In the model the domes are ONE material
  with the screen light behind them (the drawn eyes on the pane glow through the smoked dome).
- **Cameras behind the domes:** each IMX219 sits directly behind its dome; the dome is the lens window. CAVEAT:
  the IMX219's fixed focus shifts with curved glass in front -- bench-test focus through the actual dome before
  committing (add to MEASURE.md as #7).
- **The face is one tall pane now (J):** 84 x 104, covers nose-to-chin, eyes + mouth all drawn/lit on it; the
  physical domes overlay the drawn eyes. One display, one light source, no bezel part.
- **Texture rule (Cam):** the screen is GLOSS, the shell SATIN. The pane's roughness is a fraction of the skull's
  and a touch lighter, so the face reads as a glass sheet in a plastic head (no ski-mask: the ski-mask read came
  from matte-on-matte with white stickers). Lit elements are FLUSH with the pane and proud of the skull rim by
  ~0.05 mm (z-order: glass front < rim plane < lit elements) -- anything at or below the rim gets swallowed.
- **Cameras move to the EARS (Cam: no room behind the 7"):** one clear Ø5 dot per ear cap, centred in the
  perforation grid, baseline ~115 mm (SPEC wants 106 -- fine). On the mini the same construction works. The dots
  are nearly invisible among the dark grille holes; a dark glass insert seats in each bore.
- **Emissives double-count warning:** MaterialDef emission = color x emissive x strength. Setting emissive=[1,1,1]
  with a high strength blows out to pure white (the "sticker" look). Set the EMISSIVE component to the wanted
  colour and strength 1.
- **J final construction:** egg skull with a flat face facet (plane 9 mm behind the crown), 84 x 92 pane flush in
  it, two Ø22 smoked cabochon domes (off-the-shelf acrylic/glass, $1-3) over Ø19 emissive eye discs, a 30 x 6.5
  stadium mouth bar, ear caps with dot grids + camera dots + recessed groove rings. Off-state: black monolith
  with glass catchlights. On-state: dim blue glows, not white.
- **J in cortex.loon (the production source, cbd174a):** `cortex-tub-j` (egg skull, vertical facet plane 9 mm
  behind the widest front, pane pocket, vents/IO/back-grille/camera-bore cuts), `cortex-face-j` (the recessed
  pane), `cortex-eyes-j` (Ø19 lit discs) + `cortex-domes-j` (Ø21.2 cabochon caps, centres ±16, +x half-spheres),
  `cortex-mouth-j` (30 x 6.5 stadium bar), `cortex-camdots-j` (Ø5 dark dots in the ear grids). All pub.
  z-layout: facet centre 0.48 hh; pane 0.22 hh half-height; eyes +0.11 hh, mouth −0.06 hh. The tub's boolean
  result has the usual open-edge issue (print_build.py rebuilds it watertight via manifold3d; extend its LEAVES
  + build() trees for the J parts next).
  **Materials live in the IMPORTER, not the module** (mini.loon pattern): a module's materials don't export.
  loon bugs found (for the vcad manager): (1) `[difference A B]` is B−A (documented in-file but hostile),
  (2) single-arg `[union X]` poisons the whole module at any later use, (3) a top-level form split across two
  lines breaks the reader (unclosed [), (4) primitives with module-imported values as arguments fail anywhere
  ("no matching clause"), including through [+ x 0.0]; only locally-bound literals work. All worked around.
- **J in the tilted frame (873daf5):** j-tilt = rotate about the ORIGIN, then translate to (xfj, 0, zfj). The
  mockup's double-pivot (translate(-x,-z) INSIDE the fn) double-subtracted for origin-built geometry and threw
  the pane under the chin. The facet plane cuts only u>0 (a +u box through the origin), so the leaning crown
  survives. Mini narrowed: 124 W x 104 D x 126 H (the 4" panel needs ~100 mm of flat face; the old 164 W was
  appliance-wide). Eyes ±16 mm centres, +10 above the facet centre; mouth -19; both flush on the pane recessed
  1.2 mm into the facet. Cameras: Ø4.4 bores + dark glass dots in the ear grids' TOP ROW (they read as grille
  dots, not nostrils).
- **loon lesson (recurring):** top-level forms must be ONE LINE (the reader breaks multi-line forms); fns must
  not take primitives with module-imported args; single-arg [union X] poisons the module; [difference A B] = B-A.
- **K1-class mini (Cam: "shrink it, the screen is the whole face"):** 98 W x 108 D x 118 H (was 124 x 104 x 126).
  The pane is now a ROUNDED-RECT loft (r14, matching the skull's plan corners) spanning the whole flat face,
  edge-to-edge with no bezel reveal; the facet plane sits at the pane's back (u -1.5). The 4" screen (84 wide)
  vs the face (~94 at the facet): ~5 mm of skull each side. Eyes ±16, dome Ø21.2, mouth bar 30 x 6.5 low on the
  pane. K1 reference: the bought Booster K1's helmet is ~110 W x 118 D x 125 H; ours is tighter because the
  screen defines the face.
- **Forehead/chin reduced (Cam: "shorter if possible"):** the KEY fix was j-ph — the pane half-height is now the
  4" screen's real half-height (42.5; it was 26, showing only 60% of the screen). hh 118 -> 108. Result: the pane
  spans z 8.5..93.5 of a 108-tall head: forehead ~14, chin ~9. The screen IS the face, edge-to-edge both ways.
  NOTE: the bottom round (rb 12.5) nearly swallows the chin band — if the chin reads too thin in person, drop rb
  to 0.20*hd and/or lift the whole head 2 mm on the dock.
