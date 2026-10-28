# Elbow rig: purchase list (for Cam to buy; nothing has been ordered)

One RS04 fork joint, built exactly like the lymb's J2/J4 (v9: back-mounted, sleeve and bushing, 1/8" plates), on a
bench stand, with a 270 mm test forearm to a load bolt. Prices were researched 2026-10-20 on public pages;
"est." means not confirmed on a live page. Check at checkout.

## Order

| # | item | vendor and link | unit | qty | ext. | lead time |
|---|---|---|---:|---:|---:|---|
| 1 | **RobStride RS04** | Robotics Center: https://www.roboticscenter.ai/store/product/robstride-04 | $295 | 1 | $295 | **backorder, 4–6 wk** (page). One search snippet said in stock at $255; ask them before paying |
| 1a | — alternatives | aifitlab $255, ships from China, backordered: https://aifitlab.com/products/robstride-04-motor · OpenELAB €254.75 preorder | | | | pick whichever confirms a ship date first |
| 2 | shipping, RS04 | Robotics Center checkout estimate | $67 | 1 | $67 | |
| 3 | RobStride USB-CAN debug module (IDs, firmware, Motor Studio) | OpenELAB TB-ROBSTRIDE-Debug: https://openelab.com/products/robstride-can-to-usb-debugging | $25 | 1 | $25 | 3–7 d US warehouse, else 10–20 d |
| 4 | candleLight USB-CAN (gs_usb), for the python scripts | CANable 2.0: https://canable.io | ~$40 est. | 1 | $40 | days |
| 5 | 48 V supply, Mean Well LRS-350-48 (enough for one joint) | Jameco: https://www.jameco.com/z/LRS-350-48-MEAN-WELL-350W-48V-7-3A-Single-Output-Enclosed-Switching-Power-Supply_2219840.html | $33 est. | 1 | $33 | in stock (snippet) |
| 6 | e-stop 22 mm, 1NO+1NC, driving a DC contactor or the supply's remote | IDEC / Omron class, e.g. https://ramcoi.com/itemdetail/A22EM11EMO | ~$45 est. | 1 | $45 | call |
| 7 | 48 V DC contactor or relay, ≥10 A | any (Digi-Key) | ~$30 est. | 1 | $30 | days |
| 8 | 5052-H32 1/8" sheet, 24 x 24 in (two 390 mm nests) | McMaster 5052 sheet: https://www.mcmaster.com/products/5052-aluminum-sheet/ | ~$60 est. | 1 | $60 | 1–2 d |
| 9 | 6061 round bar Ø5" x 1" (one RS04 sleeve, Ø119 flange) | McMaster: https://www.mcmaster.com/products/6061-aluminum-rod/ | ~$35 est. | 1 | $35 | 1–2 d |
| 10 | igus iglidur G flanged bushing GFM-1618-03 (or the nearest stocked GFM-1618 length) | https://www.igus.com | ~$1.50 est. | 2 | $3 | 1–3 d |
| 11 | DIN 471 retaining ring 16 mm (pack) | McMaster: https://www.mcmaster.com/products/retaining-rings/ | ~$9 est. | 1 | $9 | 1–2 d |
| 12 | 2020 extrusion, 4 x 300 mm + corner brackets + M5 T-nuts | Misumi / OpenBuilds / Amazon | ~$30 est. | 1 set | $30 | 1–2 d |
| 13 | socket head screws: M4 x 10 (12), M5 x 12 (10), M8 x 70 + nut + Ø12 x 58 spacer tube | McMaster 91290A: https://www.mcmaster.com/products/screws/ | ~$15 est. | 1 lot | $15 | 1–2 d |
| 14 | weights to 14 kg (25 N·m needs 9.4 kg, 35 N·m needs 13.2 kg at 270 mm) + hanger | 1.25/2.5 kg plates + a 1" bar, Amazon | ~$65 est. | 1 | $65 | 1–2 d |
| 15 | dial indicator 0.01 mm + magnetic base | generic set, Amazon | ~$60 est. | 1 | $60 | 1–2 d |
| 16 | K-type thermocouple + reader | Amazon | ~$15 est. | 1 | $15 | 1–2 d |
| 17 | digital level 0.1° | Amazon | ~$25 est. | 1 | $25 | 1–2 d |
| 18 | bus cable 2 x 20 AWG + CAN pair, ~1 m, XT30(2+2) pair, 120 Ω terminators | LCSC C19268030 + any high-flex | ~$20 est. | 1 | $20 | days |
| 19 | Loctite 243 | Amazon | ~$12 est. | 1 | $12 | 1–2 d |
| | **total** | | | | **~$870** (~$560 without the measurement tools, items 14–17, which the lymb reuses) | |

The ~$530 in the earlier plan left out shipping, the second CAN adapter, the contactor and the heavier weights.
The dial, level, thermocouple and weights carry over to the lymb, so the rig's own cost is about $560 including the motor.

## Make (nothing to buy)

- Plates: `cut/sheet_3.175mm_1.dxf` (child A/B, spacer disc) and `cut/sheet_3.175mm_2.dxf` (parent A/B), 1/8" 5052, 390 mm bed. Single parts are in `cut/*_3.175mm.dxf`, with SVG previews.
- Sleeve: the arm's RS04 sleeve, `../fab/SLEEVES.md` and `../fab/sleeve_RS04.svg` (turned on the 4th axis or a lathe).

Regenerate with `python3 arm/fork/rig/elbow_rig.py`. Tests and pass criteria: [RIG-TEST.md](RIG-TEST.md).
