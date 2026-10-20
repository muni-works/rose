# Cortex head: parts, layout, wiring (mini)

The head is the **cortex**. It is a detachable module holding the compute, its own battery, the sensors and the face. It sits on a **dock** at the top of the neck and can live on a desk. The **limbic board** stays in the torso and is shared by every rose model. Code: `k1_structure.py` (`CORTEX`, `head_frame`, `cortex`, `harness`, `DOCK_Z`, `PASS_THROUGH`) and the head build in `k1_design.py`.

## Parts

Sizes are from datasheets or retailer spec tables where found. **ASSUMED** marks numbers to confirm against a drawing before any shell is printed.

| Role | Part | Size (mm) | Notes |
|---|---|---|---|
| Compute | NVIDIA Jetson Orin NX 16 GB on Seeed A603 carrier | 87 × 52 board; stack 36 **ASSUMED** | The A603 board outline is 87 × 52. The stack height assumes board, SO-DIMM, module and an 8 mm fan heatsink. Input is 9–20 V on a DC jack. |
| Heatsink | ATS Orin NX fan heatsink (ATS-NVA-3281 class) | 63 × 40 × 8 | |
| Display | Waveshare 3.5" HDMI LCD (E), IPS 640 × 480 | 76.6 × 63.6; active 70.68 × 53.36; thickness 11 **ASSUMED** | HDMI to the A603 over a flat FPC. The 5" (H), at 121 × 89.5, does not fit the mini's face together with the cameras. Use it on the full. |
| Eyes | 2 × Arducam IMX219 (B0287 class) | 25 × 24; depth 13 **ASSUMED** | 22-pin 0.5 mm FFC to the A603 CSI ports. The baseline is 106 mm, flanking the display behind one glass. |
| Ears (out) | 2 × Visaton K 28 WPC BL, 8 Ω 1 W | Ø28 × 5.4 | Behind the side grilles. |
| Amp | 2 × Adafruit 3006 MAX98357A (I2S) | 19.4 × 17.8 × 3.0 | |
| Ears (in) | 2 × Adafruit 3421 SPH0645LM4H (I2S MEMS) | 16.7 × 12.7 × 1.8 | One per ear, above the speaker. |
| Battery | 3S1P 21700 (Molicel P45B class) + 3S BMS | 22 × 66 × 74 | About 11.1 V and 50 Wh. 3S because the A603 needs at least 9 V. |
| Power | 3S charger / power-path board (BQ25713 class) | 30 × 40 × 6 **ASSUMED** | Charges from the dock's 19 V and feeds the A603. |
| Dock | 12-pin pogo pair (Mill-Max 2.54 mm class) | 8 × 32 × 8 | 2 × 19 V, 2 × GND, GbE (4 pairs), 2 spare. The receptacle opening is cut in the neck's dock plate. |
| Back panel | power button, status LED, USB-C (panel extension), RJ45 (panel coupler) | | Ports reach the A603 back edge over short extensions. |

## Layout (head frame, mini)

The head is 104 deep × 164 wide × 121 tall. The flat face band is z +12.5 to +98.8 above the bottom.

- **Front:** the glass face covers the display, with an eye on each side of it.
- **Middle:** the Jetson lies flat behind the display, 4 mm back to leave room for the HDMI FPC.
- **Back:** the 3S pack stands at the back, with the charger above it. The dock pogo block sits in the floor between the pack and the Jetson.
- **Sides:** at each ear, the speaker sits behind the grille, the mic above it and the amp below it. Side vents run along the back half.

## Wiring

Every run is modelled as a tube or ribbon. Each one clears every part except its own two ends.

- **Inside the head:**
  - camera FFCs (16 mm), from each camera over the Jetson top to its CSI connectors
  - HDMI FPC (14 mm), down the 4 mm gap behind the display
  - charger → A603 DC, dock → charger (19 V), dock → A603 (GbE), down the gap between the pack and the Jetson
  - panel → A603 for RJ45 and USB-C
  - amp and mic wires to the carrier header
- **Spine:**
  1. dock → behind the head-pitch drum (head side)
  2. **pitch flex loop**, covering −20..+40°
  3. neck run (yaw side)
  4. **yaw service loop** at the top of the chest, covering ±57°
  5. torso run through a **10 mm pass-through** in the right side plate (x −48, z 210), behind the shoulder drum and above the exhaust fans
  6. down the right side compartment to the limbic board's top edge

## Open items

- **A603 I/O:** confirm the number of CSI ports and their type, and whether I2S is exposed on any header. If not, the mics and speaker need a USB audio codec.
- **Missing drawings:** the stack height, display thickness and charger board size are assumptions.
- **Dock electrical spec:** connector pinout, GbE through pogo pins, and 19 V supply from the limbic board's DC-DC. This belongs to the limbic session (rose branch `claude/limbic-spec`).
- **Service loop lengths:** about 40 mm of slack for the pitch loop and 60–80 mm for the yaw loop. These need a cable clip at each end.
- **Spine cable visibility:** the spine cable shows briefly behind the pitch drum. A neck sleeve or rear cover would hide it.
- **Other sizes:** the full's head should take the 5" (H) display. The tiny has no cortex.
