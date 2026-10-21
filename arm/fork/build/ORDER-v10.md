# Lymb v10.3: costed order list, item by item (2026-10-21)

**For Cam to approve line by line. Nothing has been ordered by any agent.**
- Mark each line ✅ / ❌ and buy the ✅ lines yourself.
- Prices are from public pages. **live** = checked on the vendor page 2026-10-20/21. **est.** = not confirmed; check at checkout.
- The cart is in three groups:
  - **A:** needed to build and measure the first lymb (bare, no skin).
  - **B:** the v10.3 skin and thermal parts. The design depends on them, but the first measured article doesn't.
  - **C:** tools and measurement kit, reused for every lymb after.

## A. Build and measure (the critical path)

| # | item | qty | unit | ext. | vendor / link | lead | status | ✅/❌ |
|---|---|---:|---:|---:|---|---|---|---|
| A1 | RobStride RS04 | 3 | $275.00 | $825.00 | Amazon (Cam's cart), Prime | arrives Oct 22 | live (Cam's screenshot) | |
| A2 | RobStride RS02, dual encoder | 2 | $190.00 | $380.00 | Amazon (Cam's cart), Prime | arrives Oct 22 | live | |
| A3 | RobStride RS00 | 1 | (Cam has one coming) | $0 | Amazon | already ordered | | — |
| A4 | candleLight USB-CAN (CANable 2.0 or similar, gs_usb) for the measurement scripts | 1 | ~$40 | $40 | canable.io / Amazon | days | est. | |
| A5 | RobStride USB-CAN debug board (Motor Studio: ids, firmware). Optional, but setting the CAN ids is easiest with it | 1 | $25.09 | $25 | OpenELAB: https://openelab.com/products/robstride-can-to-usb-debugging | preorder 10–20 working days | live (sold out) | |
| A6 | 48 V supply: Mean Well LRS-600-48 (12.5 A; three RS04s can exceed 350 W on a fast loaded move) | 1 | ~$65 | $65 | Digi-Key / Mouser | in stock typ. | est. | |
| A7 | E-stop 22 mm 1NO+1NC + 48 V DC contactor (≥ 20 A) | 1 + 1 | ~$45 + ~$30 | $75 | Digi-Key | days | est. | |
| A8 | 5052-H32 sheet, 1/8", 24 × 48 in: three 381 mm nests, 31 parts incl. the desk pedestal | 1 | ~$115 | $115 | McMaster 5052 sheet / OnlineMetals | 1–2 d | est. | |
| A9 | 5052-H32 plate, 1/4", 12 × 12 in: foot Ø180 | 1 | ~$35 | $35 | McMaster / OnlineMetals | 1–2 d | est. | |
| A10 | 6061 round bar for 3 turned sleeves (2 × RS04 Ø119 flange, 1 × RS02), e.g. Ø5" × 2" + Ø3" × 1" | 1 lot | ~$75 | $75 | McMaster, cut to length | 1–2 d | est. | |
| A11 | igus iglidur GFM-1618-03 (or the nearest stocked GFM-1618) + GFM-1012-03 | 2 + 1 | ~$1.50 | $5 | igus.com | 1–3 d | est. | |
| A12 | DIN 471 retaining rings, 16 mm and 10 mm (packs) | 2 packs | ~$9 | $18 | McMaster | 1–2 d | est. | |
| A13 | Socket head screws (ISO 4762, black 12.9): M3×10 (8), M3×6 (6), M4×10 (44), M4×8 (12), M5×12 (18), M5×8 (10), M6 table bolts (6) | 1 lot | ~$60 | $60 | McMaster 91290A / Bolt Depot | 1–2 d | est. **Hold the M4 RS02 screws until the RS02 thread depth is measured** (assumed 6 mm) | |
| A14 | High-flex harness 2 × 20 AWG + CAN pair, 3 m (model route 1.56 m + 30 % + service) | 1 | ~$40 | $40 | igus chainflex / silicone ribbon | days | est. | |
| A15 | XT30(2+2) connector pairs + two 120 Ω terminators | 8 | ~$1–9 | $25 | LCSC C19268030 / eBay | days–2 wk | est. | |
| A16 | Loctite 243 | 1 | ~$12 | $12 | Amazon | 1–2 d | est. | |
| A17 | 1/8" single-flute O-flute end mills for 5052, 1/4" shank | 2 | ~$25 | $50 | Amazon / Home Depot (Yonico 31310-SC is 3-flute) | days | est. | |
| | **A subtotal** | | | **~$1,875** (motors $1,205 + rest ~$670) | | | | |

## B. v10.3 skin and thermal (not needed for the first measured article)

| # | item | qty | unit | ext. | vendor | lead | status | ✅/❌ |
|---|---|---:|---:|---:|---|---|---|---|
| B1 | Upper-arm flank panels, 0.6 mm 5052-H32, laser cut. Cut them in-house from 0.025" sheet if the 4040 can hold them | 4 | — | ~$30 sheet | McMaster 0.025" 5052 | 1–2 d | est. | |
| B2 | Type II anodize, warm black (#1e1813 dye match), batch: all 1/8" plates (now visible inside the open pucks) + 4 panels | 1 batch | ~$150–300 | $225 | local anodizer, ~1 wk | 1 wk | est. Quote needed | |
| B3 | Gap pad, 0.25 mm, ~3 W/mK, A4 sheet | 1 | ~$35 | $35 | Bergquist / t-Global via Digi-Key | days | est. | |
| B4 | Skin shells: lofted sleeves, turret and neck shells, desk puck base, 3 open-ring pucks with dot-grid, J3/J7 bands. Printed for the first article (MJF PA12 dyed black, or in-house PETG/ASA) | ~12 parts | — | ~$250 MJF / ~$40 PETG | JLC3DP / Xometry MJF, or the shop printer | 1–2 wk / days | est. Part list in V10.3.md. Fastening not yet designed | |
| | **B subtotal** | | | **~$540 (MJF) / ~$330 (PETG)** | | | | |

## C. Tools and measurement kit (one-off; also used by the elbow rig)

| # | item | qty | ext. | vendor | status | ✅/❌ |
|---|---|---:|---:|---|---|---|
| C1 | Dial indicator 0.01 mm + magnetic base | 1 | ~$60 | Amazon | est. | |
| C2 | Ø6 mm ball stylus (M3) for the probe flange | 1 | ~$35 | Amazon / eBay | est. | |
| C3 | Calibration plate stock: the 1/4" offcut from A9, plus 2" 6061 square bar 2 × 60 mm (risers) | 1 | ~$15 | McMaster | est. | |
| C4 | Ø6 h6 dowels + Ø6 H7 reamer, 90° countersink | 1 set | ~$35 | McMaster | est. | |
| C5 | Weights 0.5–2 kg + hanger (1.0 and 1.27 kg loads) | 1 | ~$30 | Amazon | est. | |
| C6 | Digital level 0.1° (zero jig) | 1 | ~$25 | Amazon | est. | |
| C7 | K-type thermocouple + reader | 1 | ~$15 | Amazon | est. | |
| | **C subtotal** | | **~$215** | | | |

## Totals
- **First measured lymb (A + C): ~$2,090.** The motors are $1,205 of it; Cam's RS00 isn't counted.
- **With the skin (+B): ~$2,420–2,630.**
- **Not included:** sales tax, shipping on the est. lines, the elbow rig (on hold), gripper v2 (not needed to measure).

## Schedule if A and C are bought today
| day | |
|---|---|
| Oct 22 | RS04 ×3 and RS02 ×2 arrive (Amazon). Bench each motor, set the ids, measure the RS02 thread depth |
| Oct 22–24 | metal arrives; cut 3 + 1 sheets on the 4040; turn the 3 sleeves; cut the calibration plate |
| ~Oct 26–27 | assemble (BUILD.md, ~2 days), zero, `bringup.py` |
| ~Oct 28 | measure: the 50-pose gate at 0 and 1 kg, hold, sag, drop |
| **~Oct 28–30** | **a measured lymb.** About 1–2 weeks from today, down from 45 days |

**Parts ordered %**, by cost of A + C: 0 % now. 58 % once Cam buys the Amazon motors, 100 % with the rest of A + C.
