# Sheet-metal structure

All structural links are 2.0 mm aluminium, folded as U-channels left open on
the outboard face so wiring can be dropped in after assembly rather than
threaded through. Torso and head are 2 mm shelled boxes.

## Validated part: right thigh channel

`k1-thigh-r.step` — folded body, AP214, with true cylindrical bend faces, so a
fab service with a 3D pipeline (SendCutSend) auto-detects bends, angles and
directions with zero data entry.

| parameter | value |
|---|---|
| material | Al-5052 (`Al-soft`) |
| thickness | 2.0 mm |
| base flange | 76 × 200 mm |
| side flanges | 2 × 55 mm, up, off edges 1 and 3 |
| bend relief | on, shop default |
| engraving | `K1 THIGH-R AL5052-2.0` at (8, 100), 6 mm cap height |

### DFM — SendCutSend profile

**`shop_ready: true`, 0 errors, 0 warnings.**

Shop limits the part was checked against:

| limit | value |
|---|---|
| fixed bend radius | 0.97 mm |
| relief depth | 3.51 mm |
| min flange height | 7.95 mm |
| min hole → bend | 6 mm |
| min bend → bend | 12 mm |
| max bend length | 1117.6 mm |
| die width | 12 mm |

Note the bend radius is **fixed by the shop** for this material/thickness —
passing a custom `radius` on a flange is rejected outright. Omit it and the
correct value resolves automatically.

## Not yet done

The remaining channels (shin, upper arm, forearm) share this construction but
have not been individually run through `sheet_metal_create` / DFM. Their
dimensions are in `../cad/k1.loon`.

Flat-pattern DXF was not captured: `sheet_metal_unfold` returned only the
session handle in this build, with the flat pattern and DXF string absent from
the tool result. The STEP is the usable fab artifact here.
