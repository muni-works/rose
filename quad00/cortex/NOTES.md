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
