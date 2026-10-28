# Lymb v10.3 buy sheet: A + B + C, grouped by vendor (2026-10-21)

Cam buys every line himself; no agent orders.
- **TODAY** = order now.
- **WAIT** = wait for v10.3 final: the skin or panel geometry isn't frozen yet, or a quote needs the final part list.
- Prices: **live** = checked on the vendor page; **est.** = check at checkout.
- Item numbers (A1…) match ORDER-v10.md.

## 1. Amazon: motors first (TODAY, they set the date)
| item | qty | price | ext. | when |
|---|---:|---:|---:|---|
| RobStride 04 QDD 120 N·m Integrated Actuator Module (A1). Check the seller is RobStride or an authorized reseller | 3 | $275.00 live | $825.00 | **TODAY** (15 left; Prime, Oct 22) |
| RobStride 02 QDD 17 N·m Integrated Actuator Module, dual encoders (A2) | 2 | $190.00 live | $380.00 | **TODAY** (Prime, Oct 22) |
| RS00: already ordered by Cam (A3) | — | — | — | done |
| CANable 2.0 / candleLight USB-CAN, gs_usb (A4) | 1 | ~$40 est. | $40 | **TODAY** |
| Loctite 243, 10 ml (A16) | 1 | ~$12 est. | $12 | **TODAY** |
| 1/8" single-flute O-flute end mill for aluminium, 1/4" shank, 2-pack (A17) | 1 | ~$25 est. | $50 | **TODAY** |
| Dial indicator 0.01 mm + magnetic base (C1) | 1 | ~$60 est. | $60 | **TODAY** |
| Ø6 mm ruby or steel ball stylus, M3 thread (C2) | 1 | ~$35 est. | $35 | **TODAY** |
| Weights 0.5–2 kg + hanger (C5) | 1 | ~$30 est. | $30 | **TODAY** |
| Digital level / angle gauge, 0.1° (C6) | 1 | ~$25 est. | $25 | **TODAY** |
| K-type thermocouple + reader (C7) | 1 | ~$15 est. | $15 | **TODAY** |
| **Amazon total** | | | **~$1,492** | |

## 2. McMaster-Carr: https://www.mcmaster.com (TODAY, 1–2 day delivery)
| item | qty | ext. | when |
|---|---:|---:|---|
| 5052-H32 aluminium sheet, 1/8" × 24" × 48" (A8) | 1 | ~$115 est. | **TODAY** |
| 5052-H32 aluminium plate, 1/4" × 12" × 12" (A9; also the calibration plate) | 1 | ~$35 est. | **TODAY** |
| 6061 round rod, Ø5" × 2" + Ø3" × 1" cut-to-length, for the sleeves (A10) | 1 lot | ~$75 est. | **TODAY** |
| 6061 square bar 2" × 2" × 6" (calibration risers, C3) | 1 | ~$15 est. | **TODAY** |
| DIN 471 external retaining rings, 16 mm and 10 mm (A12) | 1 pack each | ~$18 est. | **TODAY** |
| Socket head screws, ISO 4762 black 12.9 (A13): M3×10 ×8, M3×6 ×6, M4×10 ×34, M5×12 ×18, M5×8 ×10, M6 table bolts ×6 | 1 lot | ~$50 est. | **TODAY** |
| RS02 screws, M4×8 ×12 + M4×10 ×10 | 1 lot | ~$10 est. | **WAIT** until the RS02 thread depth is measured on Oct 22 (assumed 6 mm) |
| Ø6 h6 dowel pins ×10, Ø6 H7 reamer, 90° countersink 1/4" (C4) | 1 set | ~$35 est. | **TODAY** |
| 5052-H32 sheet 0.025" × 12" × 24", for the upper-arm panels (B1) | 1 | ~$30 est. | **TODAY** (stock only; the panels are cut after v10.3 final) |
| **McMaster total** | | **~$383** | |

## 3. Digi-Key: https://www.digikey.com (TODAY)
| item | qty | ext. | when |
|---|---:|---:|---|
| Mean Well LRS-600-48 power supply (A6) | 1 | ~$65 est. | **TODAY** |
| 22 mm e-stop, 1NO+1NC, twist release (IDEC XW1E or Omron A22E) (A7) | 1 | ~$45 est. | **TODAY** |
| 48 V DC contactor or relay, ≥ 20 A, 24/48 V coil (A7) | 1 | ~$30 est. | **TODAY** |
| Gap pad, 0.25 mm, ~3 W/mK, sheet (Bergquist GP3000 class) (B3) | 1 | ~$35 est. | **TODAY** (sheet stock; cut to the panels later) |
| **Digi-Key total** | | **~$175** | |

## 4. igus: https://www.igus.com (TODAY)
| item | qty | ext. | when |
|---|---:|---:|---|
| iglidur G flanged bushing GFM-1618-03 (or the nearest stocked GFM-1618 length) (A11) | 2 | ~$3 est. | **TODAY** |
| iglidur G flanged bushing GFM-1012-03 (A11) | 1 | ~$2 est. | **TODAY** |
| chainflex or high-flex cable, 2 × 20 AWG + CAN pair, 3 m (A14) | 1 | ~$40 est. | **TODAY** |
| **igus total** | | **~$45** | |

## 5. LCSC: https://www.lcsc.com (TODAY; 1–2 weeks from China, so it's the slow line)
| item | qty | ext. | when |
|---|---:|---:|---|
| XT30PW(2+2) male + female pairs (C19268030 + mate) + two 120 Ω resistors (A15) | 8 pairs | ~$25 est. | **TODAY**. If it isn't here by assembly, buy XT30(2+2) on Amazon instead |

## 6. OpenELAB: https://openelab.com/products/robstride-can-to-usb-debugging (optional)
| item | qty | ext. | when |
|---|---:|---:|---|
| RobStride CAN-to-USB debug module (A5): Motor Studio, ids, firmware | 1 | $25.09 live (sold out, preorder) | **TODAY** if wanted. Optional: the arm runs on the CANable |

## 7. Skin, machined drums, anodize (v10.8 white = release look, shape frozen at 448ab4b)
| item | ext. | when |
|---|---:|---|
| Skin (B4): 34 print-ready parts in `arm/fork/skin-stl/print/` (watertight, no holes), 291 cm³ MJF PA12, **dyed/painted white #ECECE9** (MJF is grey natively: order "dyed white" if offered, else vapour-smooth + white satin paint in-house), JLC3DP or Xometry (account needed for a quote) | ~$116 est. + ~$40 finish | **READY TO ORDER** |
| 2 SLA trims: J3 hairline ring + J7 Make-green inlay (#39FF14) | ~$20 est. | **READY TO ORDER** |
| J2 drum barrel: 6061, OD 137 × ID 119, 1.0 mm wall, outsourced CNC turning, black anodize | $90–140 est. | **READY TO ORDER** (quote; drawing from `arm/fork/V10.8.md`) |
| Cap discs: 0.050" 5052, Ø134 (J2) ×2, Ø123 (J4) ×2, Ø92 (J6) ×2, laser-cut (SendCutSend), black anodize | $50–90 est. | **READY TO ORDER** |
| J3 band: 0.040" 5052 blank 247 × 42 mm with two 12 mm flanges, laser-cut + bent (SendCutSend), rolled in-house | ~$15 est. | **READY TO ORDER** |
| 6 gunmetal bezel rings, 1.5 mm, turned in-house from 6061 | (stock in A10) | in-house |
| Gap pad, 0.25 mm ~3 W/mK, 200 × 200 mm (already B3 on Digi-Key) | — | (B3) |
| Type II anodize batch (B2): all 1/8" plates + 4 panels + cap discs + J3 band **black**, bezels gunmetal; J2 drum anodized by its turner | ~$225 est. | **READY TO ORDER** (quote after the plates are cut) |
| **Section 7 total** | **~$560–650** | |

## Totals
| group | ext. |
|---|---:|
| Order today (Amazon, McMaster, Digi-Key, igus, LCSC, optional OpenELAB) | **~$2,145** |
| Section 7 (skin, machined drums, anodize) | **~$560–650** |
| **All A + B + C** | **~$2,705–2,795** |

**Parts ordered %** is by cost, and moves when Cam confirms each vendor:

| confirmed | parts ordered |
|---|---:|
| Amazon motors only | ~50 % |
| all of today's lines | ~85 % |
| everything | 100 % |
