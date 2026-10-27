# cortex as a frog node: design brief

Status: v1, 2026-10-27 (Cam: no brow bumps, shape it like a human skull; the 4" panel is square). This brief replaces the "exterior frozen" line in [SPEC.md](SPEC.md) for the outside of the
shell. The inside stays as it is: compute, battery, dock, thermal numbers and assembly. Systems contract (dock, idle,
persona, security): chief `design/head-as-node/spec.md` (`ce079c8`).

## What went wrong last time

The print-prep pass took a robot head and polished parts of it. It never asked what the object is when it sits on a
desk. On the desk it read as a head removed from its body: a tall black tub, ears like ports, a hard base band, and a
face that points over your shoulder. Two weeks went to the ear inserts. The 2 cm orange plug needed 80 booleans and
never once rendered clean. This time the silhouette comes first. A detail is only drawn after the whole object reads
right from 2 m away.

## The object

**pip at rest.** A small companion whose face is a screen. It's built to sit on a desk most days and on a humanoid
some days, and it's the same object in both places. It is not a robot head that also works on a desk, and it is not a
smart speaker.

Three tests, all judged from renders at 1000×1000:

1. **Desk, 60 cm away, seated.** The face looks at you, and from the side it looks like it belongs on the desk, not
   like it fell off something.
2. **Humanoid, 2 m away.** It reads as the head. The neck joint looks intended, not like a puck stuck on a post.
3. **Off, in a photo.** The screen is dark and the object is still beautiful. Shape alone carries it.

## Design moves (decided)

1. **One pebble, no base band.** The shell is a single soft volume: a superellipse in plan, a big crown radius, and
   sides that curve under to a small flat foot. There's no parting-line groove and no recessed band. The tub/floor
   split is the only seam, and it hides in the curve under the foot.
2. **Human skull massing.** ~~Brow domes~~ (dropped in v1). The shell follows a skull's big shapes, not its detail: a flat face plane from chin to brow, a cranium that domes up to its peak behind the face, an egg plan (about 110 across the face, 128 across the parietals), the occiput swelling back at ear height, and a neck recess under the back. The two cameras sit in the brow above the glass on a 52 mm baseline.
3. **Square face.** The 4" 720×720 square panel's own 84 × 84 glass *is* the face, with a 2 mm land round it on the
   face plane (NOTES §4). The screen draws pip's face, so the glass is the face,
   not a window set into a face.
4. **The desk tilt lives in the puck, not the shell.** The shell's face is vertical when docked on the neck. The desk
   dock puck is a 12° wedge, tall at the back, so on a desk the face tilts up toward a seated person. Both docks use
   the same pads, cones and magnets. The old 8° rake in the shell goes away. It was a compromise that was wrong in
   both places.
5. **Sound and air go through one lip.** The ear ovals and orange inserts are gone. There's one continuous slot,
   2.4 mm tall, that wraps the sides and back just under the shell's widest line, where the curve turns under. It is
   the speaker port, the mic port and the air intake. A perforated band on the back of the crown is
   the exhaust. Nothing on the sides draws the eye except the shape.
6. **One accent, and it's light.** The shell is a single material in two finishes: satin moss-black (default) and
   bone. The only colour is the status light: a thin light pipe in the lip slot that glows frog green while pip is
   working as a node and turns off when idle. There's no orange hardware.
7. **Neck join.** On the humanoid, the neck's dock plate sits inside the curve under the foot, so the head appears to
   rest on the neck rather than be bolted to it. The neck trim follows the foot's outline at 1 mm reveal.

## Envelope (mini, v0 massing)

| | Value | Why |
|---|---|---|
| Body | 128 W × 122 D × 134 H, widest at z 80 | Inside: carrier 87 × 52, 3S 21700 pack flat over the dock, 4" display 84 × 84 + 10 deep, about 2.5 mm wall. |
| Brow cameras | Ø6 lenses, y ±26, z 112 | 52 baseline, not SPEC's 106: the panel fills the face between. Check depth at desk range. |

| Face glass | 84 × 84 (the panel's own), R6, centred z 62 | Face plane is flat z 18–104, ≥ ±50 wide. |
| Foot | 84 × 70 flat, desk face | Dock pads, cones and strikes as in SPEC. |
| Lip slot | 2.4 tall at z ≈ 30, sides + back | Open area ~2.4 × 300 = 720 mm² before the speaker/mic gaskets. |
| Exhaust | 70 × 14 perforated band on the back crown | Same Ø1.4 / 2.2 hex as SPEC. |
| Desk puck | 96 × 84 foot, 12° wedge, weighted | Same parts as the neck dock. |

Every number is a v0 massing number. The thermal check (SPEC) has to be redone for the lip + crown-band layout before
any of this is frozen. Intake moves from the I/O cutout to the lip, and that is the open risk.

## Order of work

1. **Massing** (this pass): skull shell, face glass, foot, lip, puck, neck stub. Render desk 3/4, desk side,
   humanoid 3/4 and dark-screen front. Judge against the three tests. Nothing else gets drawn until those pass.
2. Fit internals into the new shell (carrier, pack, display, brow cameras, speakers at the lip). Run the
   clearance sweep.
3. Redo the thermal check for the lip intake and crown exhaust.
4. Details: lens glass, light pipe, exhaust perforation, seam.
5. Print package (reuse `print_build.py`).

## Out of scope

The 7" full-size head (it follows the mini once the mini passes) and anything inside the dock electrical spec.

## v0 massing result (2026-10-27)

Source [`node.loon`](node.loon), renders `sh node_render.sh` → [`renders/node/`](renders/node/).

- **Desk test: passes as a direction.** It reads as a small character, not a part. The domes carry "frog" from
  3/4 and side, and the 12° puck aims the face at a seated person.
- **Humanoid test: open.** The neck stub and torso are placeholders. The join under the foot isn't drawn yet
  (move 7).
- **Off test: weak.** With the screen dark it's a rounded box with bumps. The lip line and the finish split have to
  carry it.
- Not yet modelled: lip slot, exhaust band, lenses in the domes (they're inside the dome surface in v0), light pipe.
  vcad's MCP kernel traps on booleans between the loft and the domes, so every feature is its own root for now.

## v1 skull (2026-10-27)

The shell is generated by [`node_skull.py`](node_skull.py): egg-plan superellipse sections written into `node.loon`.
The brow domes are gone, and the panel's own square glass is the face.

- **Desk:** it reads as a head looking up at you: a domed cranium, cheeks rolling back, the face as a calm dark
  square. Less "frog" than v0. The frog now has to come from pip on the screen and the green light.
- **Humanoid:** the proportion against the torso is right, and the neck sits under the occiput the way a real head's
  does. The join is still a placeholder.
- **Next:** the neck join, the lip slot (now a jaw line under the cheeks?), fitting the internals into the egg (the
  carrier is 87 wide and the face end is narrower), then the thermal pass.

## v1.1 neck join + jaw slot (2026-10-27)

- **Neck join:** the neck's top trim is the foot's own outline, 3% smaller, 1 mm under it (the reveal). It flares
  out of a Ø50 column, so the head rests on the neck instead of being bolted to it. The dock pads, cones and strikes
  sit inside that trim, as in SPEC.
- **Jaw slot:** a 2.4 mm slot along each side, running from the face corner back to under the ear at z 24 and rising
  8° toward the back like a jaw line. There's a green light pipe 2 mm inside it. The back stays clean: a slot round
  the back read as a seam and tessellated badly. The slot is the speaker, mic and intake port.
- **Intake area dropped** to about 2 × 2.4 × 80 ≈ 380 mm² gross, before the speaker and mic gaskets (SPEC needs about
  650 mm² of intake at 25 W). The thermal pass has to find the rest: a deeper slot, a second slot under the foot's
  round, or an exhaust-assisted crown band.
- The neck collar has a visible kink where the trim meets the column. Smooth it with more sections once the neck's
  real diameter comes from the body.

## v1.2 hidden cameras + ear grilles (2026-10-27)

**Cameras: one dead-front glass, cameras behind it.** The options were:

| Option | Hides them? | Cost |
|---|---|---|
| **A. One glass over panel + brow (chosen)** | Yes. Black print with two camera-clear windows; they read as faint marks at most | Face plane runs up to the brow (z 116), so the forehead is taller. One 92 × 101 cover glass instead of the panel's own. The panel's glass sits under it: check the double-glass reflection and the focus through 2 layers (MEASURE #7). |
| B. Cameras inside the panel's black border | Yes | The 4" panel has no border wide enough for an IMX219 lens; needs a custom flex board in the margin. |
| C. Cameras in the eye glyphs (behind holes in the panel) | Yes, until the eyes move | Needs a panel with drilled-through holes. Too exotic for v1. |
| D. Accept them as features (lens rings, "brows") | No | Simple, but that's the robot look the brief walks away from. |

A also tidies the face: one glass edge from chin to brow instead of a panel edge plus two dots.

**Ear grilles:** a 19-dot hex patch (Ø1.2 on a 2.6 pitch, about 10 mm across) on each side where a human ear sits,
behind the middle of the head and above the back end of the jaw slot. The mic port is the centre dot, with a gasket
ring as in SPEC. The speakers stay behind the jaw slot. In this pass the dots are drawn as discs; print prep cuts them
as holes.

## v1.3 ears, no jaw, satin black (2026-10-27)

- **Jaw slot removed** (with its light pipe). The status light needs a new home; the default is the screen itself.
- **Ears moved** to where a human ear sits: at eye height (z 66), just behind the middle of the head's depth
  (x −6). Each is a vertical oval of Ø1.2 dots (about 12 × 22), echoing the ear's outline. The mics and speaker sit
  behind it.
- **Finish:** satin black (`satin_black`, as in cortex.loon) for the printed v1.
- **Intake:** with the jaw slot gone, intake is the two ear patches (~2 × 30 holes × 1.1 mm² ≈ 70 mm² before the
  gaskets), far short of SPEC's ~650 mm² at 25 W. The thermal pass has to add intake under the foot (hidden on the
  neck and on the puck) or around the back.

**Cameras behind the screen: no.** The Waveshare 4inch HDMI LCD (C) is an IPS LCD (its wiki), so behind the glass
sit the LCD cell, polarisers, then an LED backlight with diffuser and reflector sheets and the PCB. The backlight
stack is opaque, so a camera behind it sees nothing. The 3D drawing on the wiki (`4inch-hdmi-lcd_c.zip`) wasn't
downloaded. It would show the same layered module (NOTES §4 measured it at 84 × 84 glass, about 10 mm deep), but no
geometry can make an LCD backlight transparent. Seeing through the screen needs a transparent OLED or a
camera-under-display panel, and neither exists as a 4" HDMI module. The cameras stay behind the brow band of the
cover glass (option A).

## v1.4 hidden intake + internals fit (2026-10-27)

**Intake** is a perforated band round the whole perimeter of the foot's bottom round (z 1.5–7), facing down and out:
3 rows of Ø1.4 on a 2.2 hex, about 450 holes and ~690 mm², which meets SPEC's ~650 at 25 W. On the neck and on the
puck it sits in the shadow under the head. The neck's top trim shrank to the foot's flat (80%) so it doesn't cover
the band. In the renders the band is drawn dark; print prep cuts the holes.

**Internals** come from `python3 node_fit.py`, which writes the parts into node.loon and checks each part's smallest
gap to the inner wall (skin − 2.5 mm wall). Datasheet sizes match SPEC/cortex.loon.

```
part           gap  note
dockboard     29.0  dock board over the pad board, centred on the dock
battery        4.2  3S1P 21700 flat pack 70 x 66 x 22, over the dock
carrier        2.0  Seeed A603 87 x 52 + Orin NX, 18 tall; I/O edge aft
module         8.2  module stack to 28 over the carrier underside
heatsink      18.3  ATS 63 x 40 x 8 fan heatsink, fan up
display        2.7  4in 84 x 84 glass + module, 11.5 deep incl. connectors
cam-l          1.6  IMX219 25 x 24 x 13, lens at z 108.5, y +26
cam-r          1.6  IMX219, y -26
power          2.5  PD sink + 3S charger 30 x 40 x 6, upright on the back wall
speakers       1.0  Visaton K 28 WPC BL, Ø28 x 5.4, outer face at y ±56.2 behind the ear patch
clashes between parts: none
```

- Stack, bottom up: 3 mm floor → dock board → pack flat over the dock (z 9–31) → A603 + Orin NX (z 36–64,
  I/O edge aft) → fan heatsink, fan up into the crown (18 mm of air above it). The 4" panel is behind the glass
  (z 13–97). The cameras sit behind the brow band (lens z 108.5, ±26), the speakers behind the ear patches, and the
  power board stands upright on the back wall.
- To fit the cameras under the crown curve, the panel dropped 3 mm (`fz` 55).
- **Still open:**
  - The exhaust: hot air rises into the crown and has nowhere to go yet. It needs a hidden outlet at the back of
    the crown, or a crown band.
  - The shell is still a solid massing body. Hollowing, the tub/floor split and the bosses come with print prep.
  - Camera FOV through the brow windows isn't checked.
  - The carrier's I/O edge is now only reachable with the shell off: there's no back cutout.

## v1.5 exhaust + printable shell (2026-10-27)

**Exhaust:** a hex field of Ø1.4 holes on a 2.2 pitch, 90 wide × 16 tall, low on the back of the crown (z 84–100)
right behind the heatsink's top. About 400 holes, ~620 mm², which roughly matches the intake. It's out of sight from
the front, side and 3/4 views, and it shows only when you look straight at the back of the head.

**Printable parts:** `python3 node_print.py` builds them straight from the section polygons in manifold3d, without
vcad booleans. It takes about 1 s, and the output is watertight by construction. It writes `../fab/node/`:

| Part | What | Check |
|---|---|---|
| `node-tub.stl` | Hollow 2.5 mm shell, open at the bottom. Cuts: the face window behind the cover glass (86 × 95, R6), 2 ear patches, 3 intake rows on the foot's bottom round angled 45° down and out, and the exhaust field. 849 holes. | watertight, 110 cm³, 122 × 123 × 134 |
| `node-floor.stl` | 3 mm plate that drops into the bottom opening with 0.2 clearance per side | watertight, 17 cm³ |

Renders: `sh node_print_render.sh` → `renders/node/print-*.png` (3/4, back, under, section).

**Not yet in the print parts:**
- Floor retention: there are no corner bosses or heat-set inserts yet, and nothing for the floor to sit on. Add 4
  bosses as in cortex NOTES §3.
- Dock pads, cone seats and strike pockets in the floor.
- Glass seat: the cover glass bonds onto the face plane (0.5 mm VHB) round a 2.5 mm land. There's no pocket.
- Internal mounts for the display, camera, speaker and sled.
- Print orientation: crown up with the open bottom on the bed is the natural choice. Tree supports inside the crown
  come out through the bottom, as with the cortex tub.

## v1.6 rain (2026-10-27)

Target: **IPX4** (splash from any direction) on the humanoid outdoors. The desk doesn't drive this. Opening by opening:

| Opening | Before | Now | Still needed |
|---|---|---|---|
| Face glass | VHB onto the face land | unchanged: a continuous VHB ring is a seal | Unbroken bead all round, no gaps at the corners. |
| Ear patches | holes straight through along y | **holes fall 30° toward the outside** (outer end lower), so water in a hole runs out | An IP67 acoustic membrane (Gore/Saati class) over each mic port and speaker. Holes alone don't stop wind-driven spray. |
| Exhaust | top of the occiput (z 84–100), facing back and **up** | **moved under the occiput's widest line (z 61–75)**, where the surface faces slightly down and the swell above shelters it, level with the heatsink's aft fins. Holes fall 30° outward. | A drip lip inside, below the field, and the fan's aft side gasketed to it, so spray that gets past the holes lands outside the electronics. |
| Intake | foot's bottom round, 45° down | unchanged: it faces the ground, so rain can't fall into it | A drain path: anything that gets in must run out of the lowest intake row, not pool on the floor plate. Keep the floor's top face free of pockets. |
| Tub/floor seam | 0.2 gap | unchanged | A gasket or a glue bead. It's under the head, so it's sheltered. |
| Dock | flush gold pads | unchanged | The neck-side pogo block needs its own seal; pads on the head stay wet-safe. |

Material: **ASA in satin black**, not PETG, for an outdoor head (UV). FDM walls weep at 2.5 mm, so print 5 walls
with no gaps and seal the inside (a brushed epoxy coat or XTC-3D).

Exhaust open area after the move: 9 rows × ~44 ≈ 390 holes, ~600 mm². The 30° drop costs a little effective area
(longer hole); the thermal pass checks it.

## v1.7 MVP print on the P1S: USB-C, floor + sled, internals fitted (2026-10-27)

**Keep DFM simple.** This is a home-printed MVP on a Bambu P1S. The rain extras from v1.6 (membranes, drip lip,
gaskets, sealing coat) are parked for a later outdoor version. The 30° hole angles and the sheltered exhaust stay,
because they cost nothing to print.

**USB-C on the back:** see v1.8 (it's now on the power board itself). The surface faces slightly down there. Its cable runs to
the carrier. It's sized for the **Orin Nano** module you're fitting: the A603 takes Nano or NX in the same footprint,
and the Nano's lower power (7–25 W) eases the thermal budget. **Open:** which port does it wire to, the A603's USB-C
device/recovery port (data + flashing) or a PD power input? As drawn, it's data. Power for the MVP then comes from
the carrier's DC jack with the shell off, or from the dock pads.

**Two prints, no supports except four small trees:**

| File | Print | Notes |
|---|---|---|
| `node-tub.stl` | open bottom on the bed, crown up. 0.2 layer, 5 walls (= 2.5 mm), 15% gyroid. **Tree supports, build plate + inside**: they hold up the crown's inside and the 4 boss undersides, and all come out through the bottom. | ~114 cm³. The face window and holes are already cut. The face plane is vertical: no supports on the outside. |
| `node-floor.stl` | flat, as modelled | ~23 cm³. Plate + 2 sled walls with 45° lips, so no supports. Countersinks for 4 × M3 × 8 open downward. |

**Assembly:**
1. Press 4 × M3 heat-set inserts (Ø4 × 7 holes) into the tub bosses.
2. On the floor, strap the pack (hook-and-loop through the 2 slots). Drill the sled lips to the A603's holes
   (MEASURE #5) and screw the carrier + module + heatsink on top.
3. In the tub, VHB the 4" panel behind the face window, the cameras behind the brow, and the speakers + mics behind
   the ear patches. Screw the USB-C to its pad.
4. Plug the harnesses, lower the tub over the floor, and fit 4 × M3 × 8 countersunk screws from below.
5. Bond the cover glass onto the face land.

**Internals fit** (`node_fit.py`, clearances to the 2.5 mm wall, all ≥ 1 mm, no clashes): battery 4.2, carrier 2.0,
display 2.7, cameras 1.6, speakers 1.0, power 2.5, USB-C 1.0, sled 5.9, bosses 1.0. Renders: `print-fit` /
`print-fit-ear` (section through the printed tub with the parts).

## v1.8 USB-C = PD power in + data (2026-10-27)

The back USB-C is now the head's **power input and its data port**. There's no panel-mount extension: the receptacle
is on the power board.

```
USB-C receptacle (on the power board, through the back wall)
 ├─ VBUS, CC1/CC2 ──> USB-PD sink (TPS25750 class, asks for 20 V) ──> 3S charger / power path (BQ25713 class)
 │                                                                     ├─> pack (3S 21700)
 │                                                                     └─> system rail ──> A603 DC in (9–20 V)
 └─ D+/D- (USB 2) ──> short USB 2 cable ──> A603's USB-C device/recovery port (flashing, USB gadget/ethernet)
```

- **Board:** PD sink + charger on one 40 W × 30 H × 6 mm board. A mid-mount right-angle USB-C sits on its bottom
  edge. It screws flat to a pad inside the back wall (z 27–57, under the exhaust) with 4 × M2.5 self-tapping into
  blind holes. The lower two are only 3.6 deep because the wall is thin there.
- **Wall:** a 9.2 × 3.6 slot through to the receptacle, plus a 12.8 × 7.2 R3 pocket from outside so a plug's
  overmold reaches it (the receptacle face sits 2.7 mm inside the skin).
- **USB 2 only for data**: the SuperSpeed pairs aren't routed. That's enough for flashing, USB-gadget ethernet and
  serial. For USB 3 the receptacle's SS pairs would need routing to the carrier on a proper cable.
- Recovery mode on the A603 needs its REC button held at power-up. For the MVP that's done with the shell off, or
  the button can be brought out to a pinhole later.
- Charging at 20 V PD: about 60 W from a 65 W charger, and the pack + system share it. A 5 V-only charger gives
  trickle power only.
- **Rebuild**: tub 121 cm³, watertight. Fit: power board and receptacle clear everything; nothing clashes.

## v1.9 fixes found in vcad.app (2026-10-27)

- **Right speaker was mirrored outward**, hanging 5 mm outside the head. `[rotate 90 0 0]` points the cylinder
  along −y, so negating y isn't a mirror. Both speakers now span |y| 50.8–56.2.
- **Camera boards poked 0.5 mm through the skin** above the brow: the clearance check skipped anything touching the
  face plane. Each camera is now its 25 × 24 board 5.3 mm behind the glass plus a Ø8 lens barrel to the glass, and
  only the barrels are exempt from the check. The boards clear by 1.2 mm. The panel dropped 1 mm (`fz` 54).
- Assembly for vcad.app: `fab/node/node-assembly.loon` → `node-assembly.vcad`.

## v1.10 sled fixed (2026-10-27)

The sled walls were 28 mm long, off-centre, and missed the carrier's holes. The floor plate is only the foot's flat
(71 × 87), and at the carrier's hole line (y ±40) it runs only from x −14 to 41.

- **Sled:** two full-length walls (x −16..40) beside the pack at y ±35.5–38.5. Their lips point **outward** to y ±43
  with a 45° underside, so they print without supports, and they carry the A603 under its 4 corner holes. The holes
  are ASSUMED 3.5 mm in from the 87 × 52 corners (MEASURE #5). There are Ø2.2 pilot holes in the lips for M2.5
  self-tapping screws.
- **Carrier** moved forward to x −12..40 (I/O still aft), so all 4 holes land on the lips. The module and heatsink
  moved with it, so the heatsink now vents into the open crown ahead of the exhaust.
- **Pack** sits at x −37..33, held by a strap through 2 slots at y ±34.3. **Floor screws** are down to 3: one front
  centre and two behind the walls.
- Fit: carrier 10.4 to the wall, pack 5.1, no clashes. Renders: `sled-carrier.png`, `sled-carrier-front.png`.
  `node_3mf.py` rebuilds the plain 3MF.

## v1.11 the real compute: Jetson Orin Nano Super devkit, and the split shell (2026-10-27)

There's no A603. The compute is the **Jetson Orin Nano Super devkit**. Its numbers come from NVIDIA's P3768 A04
reference files (Allegro ASCII extract + NC drill):
- **Board:** x −4..96, y −17..62, so 100 × 79, R0.6 corners, 1.6 thick.
- **Mounting:** 4 × Ø2.75 NPTH (M2.5) at x {0, 92} × y {0, 58}, with an alternate pair at x 86. The I/O edge is y −17.
- **Stack height:** 31 above the board's underside (datasheet: 34.77 incl. feet). The I/O row height (16) is
  ASSUMED. The 3 × Ø3.8 plated holes at (32.3, 38.7), (73.3, 38.7), (82.3, 15.5) are the module's standoffs.

The 79 × 100 board can't pass through the 71 × 87 foot opening, so **the head now splits at z 30**:

| Part | Print | What |
|---|---|---|
| `node-base.stl` | flat bottom down; tree supports only under the bottom round's first mm | Closed-bottom cup, z 0–30, with the intake band. Holds the pack (strap slots) and **4 Ø7 posts** to z 38 on the devkit's 92 × 58 holes (M2.5 self-tapping pilots). 3 screw columns. A 1.4 × 5 mm locating lip stands above the rim. |
| `node-upper.stl` | seam rim down; tree supports inside the crown | z 31 up: face window, ears, exhaust, USB-C + power-board pad, 3 M3 heat-set bosses (6.5 tall, 5.7 inserts). |

- **Seam:** a 1 mm shadow gap all round at jaw height. The lip behind it locates the shell and blocks light.
- **Fastening:** 3 × M3 × 12 screws from under the base, up the columns into the upper's inserts. One is front
  centre (no web, the display is just ahead) and two are at the back (x −44, y ±30).
- **Devkit placement:** board at head x −37..42, y ±50, z 38, I/O aft. Board-to-wall 2.1. The I/O row's box
  (full width) is at 0.6, but the real connectors sit inboard.
- **Power:** the PD/charger board moved up into the upper shell (z 32–62, USB-C at z 37) and feeds the devkit's DC
  jack (19 V class). Exhaust starts at z 63, above the pad.
- `node-mvp.3mf` has both parts side by side on one P1S plate.

## v1.12 inlaid acrylic face (2026-10-27)

The face is now **3.2 mm (1/8") cast acrylic, 92 × 101.6 mm, R8, inlaid flush** with the face plane. A smoked
transparent sheet is best, but clear works too. Either way, mask the back black except the panel's active area and
the two camera windows.

- **Face plane moved forward 3.5 mm** (x 59.5) so the panel can sit right behind the sheet without crowding the
  devkit.
- **Pocket and land:** a 92.4 × 102 pocket, 3.2 deep, on a 2.5 wide × 1.5 thick land (0.5 VHB for the sheet). The
  land is a frame added behind the face, because the pocket is deeper than the wall. The window through it is
  87 × 96.6, and the 4" panel bonds to the back of the land, showing through.
- **The sheet runs past the seam** (z 14.7–116.3), so on the face the glass hides the jaw seam. The base carries
  the same pocket, and its locating lip stops short of the face, so the sheet and panel (bonded to the upper) slide
  down past it.
- **CNC:** `python3 node_acrylic.py` → `fab/node/node-acrylic.dxf` (R12, mm, seen from the front, origin at the
  sheet centre). Layers: `CUT` is the profile, `MASK_SCREEN` the 71.8 mm active window (centre −11.5), and
  `MASK_CAMS` the Ø8 camera windows at (±26, +43). Mirror X to engrave the mask guide on the back. Cast acrylic,
  single-flute O-flute, ~16–18k rpm, 1–1.5 mm passes.
- Fit: panel 1.5, cameras 4.1, devkit board 1.7, I/O box 0.3 (worst case), no clashes.

## v1.13 fast print (2026-10-27)

The first slice took 13 h 19 m (0.16 mm High Quality, 645 layers, 47.8 g of tree support).
`fab/node/node-mvp-fast.3mf` is a Bambu project with per-object settings for a draft-speed MVP:

| Setting | Was | Now | Why |
|---|---|---|---|
| Process preset | 0.16 High Quality | **0.28 Extra Draft** (pick it in Bambu) | Faster speeds and accelerations |
| Layer height | 0.16 | 0.28 | ~370 layers instead of ~645 |
| Walls | profile default | 2 | 4 lines ≈ the 2.5 mm shell plus gap fill |
| Infill | 15% | 10% grid | The shells are nearly all wall anyway |
| Top/bottom layers | profile default | 4 / 3 | |
| Support | tree, 25° | tree, **30°** | Fewer trees inside the crown |

Re-slice to get the real time. These settings should roughly halve it or better. A 0.6 mm nozzle would roughly halve
it again. `node-mvp.3mf` stays as the plain, settings-free fallback.

## v1.14 a whole 4 × 6" sheet, landscape (2026-10-27)

The face is now **one whole 4 × 6" (152.4 × 101.6 × 3.2) cast acrylic sheet, as bought, with its factory corners.**
No cutting and no CNC. Mask the back as before (`node-acrylic.dxf` is now just the mask guide).

- **Head widened** to ~173 W. Height is unchanged and depth is ~125. The flat face runs z 11–119.5, ±86 wide, and
  the plan squares off toward the face. The crown holds its width higher (n 4) so the sheet's top corners stay
  inside the skin: they sit within 0.25 mm of flush.
- **Sensor plate:** the land behind the sheet is now a full printed plate. It has an 86 mm window for the 4" panel
  (which bonds to its back) and Ø10 holes for the cameras. Future sensors (ToF, ambient light, mic) get their own
  holes in this plate, and the mask stays black over everything except their windows.
- **Stereo cameras moved beside the panel at eye height**, at y ±62 and z 62: a **124 mm baseline**, up from 52,
  which beats SPEC's 106.
- Fit: devkit board 13.7 to the wall, panel 2.7, cameras 6.5. Speakers sit at the wider walls. No clashes against
  the true shell.
- Prints: upper 129 cm³, base 85 cm³. Both are single bodies, watertight, and fit the P1S bed in every 3MF layout.
