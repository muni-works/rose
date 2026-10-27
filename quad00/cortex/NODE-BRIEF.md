# cortex as a frog node: design brief

Status: v0, 2026-10-27. This brief replaces the "exterior frozen" line in [SPEC.md](SPEC.md) for the outside of the
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
2. **Frog cue: two brow domes.** Two low, wide domes rise from the crown above the screen, like a frog's eyes above
   the waterline. They hold the **stereo cameras** (52 baseline in v0, see Envelope). That solves the old "stereo impossible behind the
   panel" problem: the cameras look out of the domes through dark glass lenses. On the desk the domes give the
   silhouette its character. On the humanoid they place the eyes where a head's eyes should be. Restraint: the domes
   rise about 9 mm, not cartoon bulbs.
3. **Square face, flush.** It uses the 4" 720×720 square display (it fits the mini; NOTES §4), shown on a dead-front
   glass face that wraps the front with a large corner radius. The screen draws pip's face, so the glass is the face,
   not a window set into a face.
4. **The desk tilt lives in the puck, not the shell.** The shell's face is vertical when docked on the neck. The desk
   dock puck is a 12° wedge, tall at the back, so on a desk the face tilts up toward a seated person. Both docks use
   the same pads, cones and magnets. The old 8° rake in the shell goes away. It was a compromise that was wrong in
   both places.
5. **Sound and air go through one lip.** The ear ovals and orange inserts are gone. There's one continuous slot,
   2.4 mm tall, that wraps the sides and back just under the shell's widest line, where the curve turns under. It is
   the speaker port, the mic port and the air intake. A perforated band on the back of the crown, under the domes, is
   the exhaust. Nothing on the sides draws the eye except the shape.
6. **One accent, and it's light.** The shell is a single material in two finishes: satin moss-black (default) and
   bone. The only colour is the status light: a thin light pipe in the lip slot that glows frog green while pip is
   working as a node and turns off when idle. There's no orange hardware.
7. **Neck join.** On the humanoid, the neck's dock plate sits inside the curve under the foot, so the head appears to
   rest on the neck rather than be bolted to it. The neck trim follows the foot's outline at 1 mm reveal.

## Envelope (mini, v0 massing)

| | Value | Why |
|---|---|---|
| Body | 126 W × 112 D × 114 H to the crown, waist at z 48 | Inside: carrier 87 × 52, 3S 21700 pack flat over the dock, 4" display 84 × 84 + 10 deep, about 2.5 mm wall. |
| Brow domes | Ø34, sunk so ~10 shows, centres x 26, y ±26 | Massing v0 baseline is **52**, not SPEC's 106: at ±53 the domes fall off the crown's shoulder. Depth from 52 mm is fine at desk range; confirm with perception before freezing. |
| Total height | about 118 | |
| Face glass | 68 W × 76 H, R 12, 1 mm proud | v0 is flat. Too small for the 84 mm 4" glass: step 2 either widens the front's flat or takes a smaller square panel. |
| Foot | 84 × 70 flat, desk face | Dock pads, cones and strikes as in SPEC. |
| Lip slot | 2.4 tall at z ≈ 30, sides + back | Open area ~2.4 × 300 = 720 mm² before the speaker/mic gaskets. |
| Exhaust | 70 × 14 perforated band on the back crown | Same Ø1.4 / 2.2 hex as SPEC. |
| Desk puck | 96 × 84 foot, 12° wedge, weighted | Same parts as the neck dock. |

Every number is a v0 massing number. The thermal check (SPEC) has to be redone for the lip + crown-band layout before
any of this is frozen. Intake moves from the I/O cutout to the lip, and that is the open risk.

## Order of work

1. **Massing** (this pass): pebble, domes, face glass, foot, lip, puck, neck stub. Render desk 3/4, desk side,
   humanoid 3/4 and dark-screen front. Judge against the three tests. Nothing else gets drawn until those pass.
2. Fit internals into the new shell (carrier, pack, display, cameras in the domes, speakers at the lip). Run the
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
