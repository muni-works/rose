# Cortex head with USB-only I/O (Cam's idea): evaluation

*2026-10-21, body manager. Concept sketch only: the loon model hasn't changed yet. **E** = estimate.*

## Verdict: GO, with two changes

1. **The dock carries body DC power and USB 2.0 on the existing 8 pogo pads; it is not a USB-C plug.**
   - A real USB-C plug in a dock that is mated thousands of times on a moving robot is a reliability risk: USB-C is rated for about 10k cycles, its tongue gets side-loaded, and it isn't designed to blind-mate.
   - USB 3 (5 Gb/s) signals don't survive pogo pins.
   - USB 2.0 High Speed (480 Mb/s) is plenty for the head-to-limbic link.
   - The J3011 takes 9–54 V, so the dock can pass the body's battery voltage straight through. No PD is needed on the body side.
2. **The head shows exactly one USB-C on the outside:** the "desk / extra" port. It does PD in (power) plus USB data.

So the outside of the head shows one USB-C, a dock face and no other ports. That's the clean look Cam wants, without a fragile connector in the dock.

*If Cam wants the dock to literally be USB-C* (so the head charges from any USB-C dock), it can be done with a floating, self-aligning USB-C receptacle inside the cone pair. It costs more (about +$15 E), wears out sooner, and needs the head to swap data roles (it takes power and acts as host at the same time). It's a fallback, not the pick.

## Power (Cam's picks: 4S pack + boost, pogo-pad dock)

**J3011 power rail (input range 12–54 V):**
- Three sources feed it through an ideal-diode OR. The highest voltage wins.
  - **Dock:** the body bus, about 48 V (≤ 54 V), with an input limit and a 2 A fuse.
  - **USB-C PD:** 20 V.
  - **Battery:** a 4S pack (12.0–16.8 V; cutoff 3.1 V per cell = 12.4 V) through a **19 V buck-boost**, so the rail never drops below 15 V across the whole pack range. This needs a 4-switch buck-boost (TI TPS55289 / LM5176 class, about 70 W, 3.7 A out). A plain boost can't do it, because a full pack (16.8 V) is above the lowest output we'd want.
- On the dock or on PD, the J3011 sees ≥ 20 V directly and the buck-boost idles.
- The charger (BQ25731 class, 4S) charges from whichever input is present.
- The 5 V / 3.3 V rails for the display, audio and cameras come off the same rail with small bucks.


Peak draw, estimated:

| Load | Mini (W) | Full (W) |
|---|---:|---:|
| Orin Nano Super, MAXN | 25 | 25 |
| Display (4" / 7") | 3 | 6 |
| 2 speakers + amps, peak | 6 | 6 |
| Stereo cameras + mics | 1 | 1 |
| Fan | 2 | 2 |
| Accessory power to the USB-C port (capped at 5 V / 1.5 A) | 7.5 | 7.5 |
| Conversion losses, about 10 % | 4.5 | 5 |
| **Peak system load** | **about 49** | **about 53** |
| Plus battery charging at the same time (about 0.5 C) | +25 | +45 |
| **Worst-case input** | **about 75** | **about 100** |

- **Standard 100 W PD (20 V / 5 A) covers both heads.** The higher-wattage 140 W mode isn't needed; above about 100 W the charger just slows down.
- **Dock power:** the body bus (about 48 V, RobStride class) runs through an ideal-diode OR, a current-limited input and a 2 A fuse, into the same system power rail.
- **Power path:** a combined charger chip with power-path control (TI BQ25731 class, buck-boost, with a TPS25750 PD controller):
  - Desk: USB-C PD feeds the system rail and charges the pack.
  - Docked: the body supply has priority, and the pack charges from it.
  - Unplugged: the pack feeds the rail with no break; the charger handles the handover.
  - The J3011 takes its supply from the system rail through its XT30 input.
- **⚠️ Battery voltage (now settled by the datasheet: the J3011 input is 12–54 V, not 9–54 V).** A 3S pack can't feed it directly, so **the pack goes to 4S** (12.0–16.8 V; set the cutoff at 3.1 V per cell = 12.4 V). The 20 V PD input and the 48 V body supply both clear 12 V. *(Superseded reasoning follows.)* A 3S pack empties down to 9.0 V, which is right at the J3011's 9 V minimum, so it would brown out under load. Either set the pack's low cutoff to 3.3 V per cell (9.9 V; about 8 % less usable capacity) or move to 4S (12–16.8 V). **I recommend 4S:** same capacity, it's a normal Bambu-printable 21700 layout, and it runs the charger more efficiently.

## Data

**J3011 ports, from Seeed's datasheet:**

| Port | Role | Use in this design |
|---|---|---|
| J38, USB 3.2 Type-C | Host + DisplayPort | **The outside USB-C**, via a 50 mm pigtail. It can also drive an external monitor. |
| J55, USB 2.0 JST 5-pin header | Host | **Dock USB 2.0 to the limbic board.** It's an internal header, so no USB-A pigtail is needed. |
| J39 / J40, 2× USB 3.2 Type-A | Host, 10 Gb/s | Internal spares (for example a USB camera or an AI accelerator) |
| J70, USB 2.0 Micro-B | Device only (recovery and flashing) | Stays internal. Flash with the floor off, or bring it to a service pad. |

The carrier board is 87.8 × 56.7 mm, about the same footprint as the A603 the mini was designed around (87 × 52). It fits the existing mini bay with the board's front edge moved forward 4.7 mm; the clash test showed 23 mm free in front. The assumed 60 × 92 outline is retired.

**Measured from Seeed's STEP (`J40 Mini-asm.stp`, from `reComputer_Mini_3D.7z`):**
- Carrier PCB 87.8 × 56.7 × 1.6 mm.
- **Mounting holes: 4× Ø2.7 (M2.5) on an 81.0 × 49.0 mm rectangle** (5.5 mm in from the port-side corner datum). The sled's carrier plate takes M2.5 heat-set inserts (coupon sizes 3.4 / 3.6 / 3.8).
- **Heatsink:** finned aluminium, 87.0 × 58.7 × 16.7 mm, covering the whole board.
- **Fan:** 46.8 × 46.7 × 7.3 mm, on top of the heatsink.
- **Total stack:** about 36–38 mm above the bottom of the board; the 42 mm enclosure, less its walls, sets the upper limit. That leaves about 11–13 mm of headroom in the mini (the limit there is 49 mm).
- The SO-DIMM Orin Nano module is 69.6 × 45.


- **The Jetson is the USB host everywhere**, and the limbic board in the torso is a USB device. It presents a network link (CDC-NCM) plus a serial port. That keeps the Jetson simple, with no device-mode quirks, and the limbic board's MCU (STM32H7 / RP2350 class, USB 2.0 HS) handles the device side easily.
- **The outside USB-C port** must be wired to a J3011 port that can act as host. The chief's research agent is confirming which J3011 USB-C is host and which is flash/recovery only. On Orin Nano carriers the recovery USB-C is usually the device-mode port, which is still useful for flashing over the same outside port with an internal switch (E).
- **Internal harness:**
  - The J3011's port edge faces aft, 10–20 mm from the back wall, so a 50 mm right-angle USB-C pigtail runs to the panel-mount USB-C.
  - USB 2.0 from the J3011's J55 header (host, JST 5-pin) goes through an internal cable to the power board, then to the dock pads.
- **Removed:** the two T1 Ethernet PHYs and the back I/O cutout. That cutout shrinks to one USB-C plus a status LED.

### Body comms over USB 2.0 instead of T1 Ethernet

- **Bandwidth:** 480 Mb/s raw, about 300 usable, against T1's 100 Mb/s. That's fine. Motor state and commands at 1 kHz for 20 joints come to well under 10 Mb/s.
- **Latency:** USB HS polls every 125 µs. Round trips through an NCM network link are about 0.5–1 ms with jitter. **That's fine because the real-time loop stays on the limbic board** (the cortex split): the head sends setpoints and trajectories at 50–200 Hz and never closes the motor loop.
- **Robustness concerns to handle:**
  - USB is less noise-immune than T1 next to motor drivers. Fix: a common-mode choke and ESD protection at the dock, and a short run, under 30 cm, from the dock to the limbic board.
  - Hot-plug on dock and undock: the limbic board must fail safe within 50 ms when it loses the head. The limbic watchdog already does this.
  - T1 was cleaner for long runs; this design has no long run.

## Mechanical

| | Mini head | Full head |
|---|---|---|
| Outside USB-C | Back face, centred, about 20 mm above the floor seam. It replaces the I/O cutout. | Same, back face |
| Dock | Floor, unchanged: 8 pads, cones ±18, magnets, latch | Same |
| Power board (PD + charger + diode OR) | About 45 × 40 on the floor beside the pack, where the current power board sits (`cortex-power`) | Same, about 50 × 45 |
| Harness | Panel USB-C → 50 mm pigtail → J3011. Dock pads → power board (pins on the board's underside, no cable) → XT30 to the J3011 + USB 2.0 cable → J3011 J55 header | Same, with longer pigtails |

The decased J3011 sits on a carrier plate on the sled (per the BOM memo's interface rule). With a custom carrier later, the ports become board-mounted connectors and the pigtails disappear.

## BOM change per head vs. the current memo (E)

| Change | Qty 10 | Qty 100 | Qty 1k |
|---|---:|---:|---:|
| Power board → PD sink + 4S charger + 19 V buck-boost + diode OR | +13 | +9 | +5 |
| 3S → 4S pack (one more 21700) | +6 | +5 | +3 |
| Remove 2× T1 Ethernet PHY + magnetics | −10 | −7 | −4 |
| Panel USB-C + 2 internal pigtails | +10 | +7 | +3 |
| Remove the I/O shield + cutout finish | −3 | −2 | −1 |
| **Net change per head** | **+16** | **+12** | **+6** |

Cost is effectively flat. The win is the industrial design (one port) and one standard way to power the head on a desk. Neck/torso side: the limbic board loses its T1 PHY (−\$5) and gains a USB device port (MCU-native, about \$0).

## Open items

- The J3011's host vs. recovery USB-C ports (the chief's research agent).
- ~~4S vs. 3S~~: decided, 4S + buck-boost (Cam).
- ~~Dock type~~: decided, pogo pads with body DC + USB 2.0 HS (Cam).
- **Next head CAD pass:** 4S pack envelope (mini 4S1P about 84 × 22 × 70 flat; full 4S2P), power board on the floor, back USB-C cutout replacing the I/O cutout, J3011 board 87.8 × 56.7 on a carrier plate, 8 pads remapped (VBUS ×2, GND ×2, D+, D−, detect, spare).
