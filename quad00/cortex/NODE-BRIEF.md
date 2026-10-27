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
