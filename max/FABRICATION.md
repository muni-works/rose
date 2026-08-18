# rose-1 fabrication (v6, 2026-07-29) — sheet metal + machined

Structure is now modeled at true gauge — the mass rollup counts real wall
volumes, no shelling factor. **Honest total: 25.1 kg vs 25 kg target**
(the old ×0.35 estimate was flattering by ~1.4 kg; remaining diet options:
thigh web 3→2.5 mm, lightening holes in the channels — frog mk12 pattern —
and the hcBase/kneeplate perimeters).

## Bent parts (5052-H32, SendCutSend)

| part | gauge | flat (approx) | qty | form |
|---|---|---|---|---|
| thigh channel | 3.0 | 196 × 243 | 2 | C-channel, 2 bends |
| shin channel | 3.0 | 124 × 217 | 2 | C-channel, 2 bends |
| foot pan | 3.0 | 218 × 200 | 2 | pan, 2 bends |
| upper-arm channel | 3.0 | 102 × 50 | 2 | C-channel, 2 bends |
| forearm channel | 2.5 | 88 × 178 | 2 | C-channel, 2 bends |
| neck collar | 3.0 | 268 × 34 | 1 | box collar, 3 bends |
| deck / chest base | 3.0 | — | 1 ea | flat |
| chest shells (front/back/sides) | 2.5 | — | 4 | flat panels |

## Flat waterjet (5052 or 6061)

| part | thickness | qty |
|---|---|---|
| knee plate | 6.0 | 2 |
| pelvis spine | 6.0 | 1 |
| hip ribs | 6.0 | 2 |
| hip carrier base | 6.0 | 2 |
| brackets: brplate/strap/cap, rollout, elplate, yokes, chest bracket, saddles, adapters | 4.0 | ~20 |
| drops (thigh/shin/elbow) | 3.0 | 6 |

## Machined / turned

Cap discs (r34 ×6), crank discs (r20 ×4), rib bearing bosses (×2), yaw
shafts ⌀16 (×2), ankle pins ⌀12 (×4), pushrods ⌀8 (×4), ankle blocks (×2),
deck bosses (×2), waist tower (⌀56 column, ×1).

## Actuator detail now modeled

Each rana carries its rotor/stator seam ring, rear encoder boss (2 mm) and
rear connector block (2 mm) — both rear features live inside the declared
4 mm bolted-gap allowance, and the seam ring's +0.6 mm radial growth was
re-verified against every sweep gate (pushrods moved +1 mm to keep 2 mm).

## Verification state

All static + swept clearances green after conversion (outer envelopes
unchanged, so the v3–v5 sweep results carry over; re-run confirms). Not yet
done: bend reliefs and K-factor flats via `flat_pattern_from_solid`,
per-part DXFs, fastener bodies, bracket structural sizing.
