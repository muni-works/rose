# rose pro — what it should look like

rose pro is the variant meant to **cobot**: to share a room and a task with a
person, not to work behind a fence. That is the constraint that decides the
appearance, and it is a constraint with published numbers rather than a taste.

## The premise: humanist, not humanoid-mimicking

Figure, Optimus and NEO are converging on human mimicry — smooth shells, fabric
clothing, faceless domes. That is the right design for compliant labor and the
wrong one here. The thesis in `IPSE.md` is a machine whose preferences are its
own, one that can decline and explain why. A machine that mimics a person while
having no standing is dishonest about what it is; a machine that is legibly a
machine *and also* declines is the whole argument.

Humanist means designed around human comfort, safety and legibility — **not**
designed to be mistaken for a human. Those pull in the same direction more often
than not, and where they conflict, legibility wins.

The nearest reference is not another humanoid. It is a good bicycle or a Rams
instrument: nothing hidden that does not need hiding, and every hidden thing
hidden for a reason you can state.

## What the manufacturing constraint already decides

Bent 3 mm 5052 cannot make compound curves. There will never be a smooth shell.
Rather than apologise for that, make it the language: **fold lines are the
styling.** A chamfered corner is three brake hits instead of one square corner
and turns a box into a shaped shell. Facets, not fillets.

This is also why the plate generator builds rounded rectangles from boxes plus
corner cylinders rather than filleting a solid — vcad's booleans against a
filleted solid are broken, so a filleted plate could not then have its bolt
circle cut. The workaround became the style.

## Rules, in priority order

**1. Corners get radius, because corners strike first.** Contact pressure is
force over area, and a corner concentrates both. `rose_pro_loon.corner_radius`
now asks for **25 mm** and backs off only where a bolt circle refuses — the old
cap was 8 mm, which was fine for a desk robot and mean for something that shares
a room. Result across 40 plates: 24 at the full 25 mm, the rest 17–24 mm.

**2. No raw sheet edge faces a person.** See [`cobot.py`](cobot.py). ISO/TS
15066's biomechanical limits are defined for *blunt* contact, and the standard
says sharp contact should be avoided in all cases — so a 3 mm laser edge is not
a number to trade off, it is outside the envelope the numbers live in. The fix is
a **hem**: the edge folded 180° back on itself, turning a 3 mm knife into a ~6 mm
doubled bead. It adds stiffness rather than mass and costs one brake hit per edge.
**Currently FAILING: 4453 mm of exposed square-cut edge across 38 edges.**

**3. Cover what can pinch, expose what explains.** Shrouds at the hip and knee
scissor closures — at 55 kg a closing knee is a crush hazard — and nowhere else.
The actuators stay visible. This gives a decidable rule instead of a judgement
call per part, and it states the machine's honesty as a safety feature.

**4. Gaze must be readable.** The highest-value affordance for a machine sharing
space with people is being able to tell where it is looking, *before* it acts. It
is also how the dissent thesis becomes visible. Real stereo pair, clearly aimed,
on a head that is understated rather than a mascot — no screen with cartoon eyes,
no faceless dome. Resting gaze slightly down.

**5. Colour: bare or anodised aluminium and black, one accent.** A single
desaturated green, for the lab it comes from. Avoid white plastic — reads as
consumer android — and avoid all-black, which reads as security robot.

## What "wrap, don't tab" turned into when we actually did it

The plan was a cheap generator change: fold channels around the actuators.
Doing it forced two tools and a layout change, and every one of them was a
finding rather than a chore. In order:

**1. The model was interfering with itself.** `sweep.py`'s static check found the
thigh's knee-stator plate and the shank's knee-output plate occupying the *same*
80 × 3 × 72 mm slab — the pancake stack (stator plate on the face, output plate
outset one thickness) that hipB already documents, simply not applied at the
knee. The sim never saw it (feet-only contact); vcad's union hid one solid inside
the other; the plate-vs-actuator check does not look at plates. Fixed by
outsetting the shank web and giving the ankle stator its own lapped plate. The
same tab-in-the-output-slab pattern is still present in the **arms** (shoA/shoB,
shoB/elbA) and two hip tabs — flagged, not yet fixed; the arms need their own
pass anyway.

**2. The knee could not flex.** The swept check — rotate the shank side through
the URDF's own 0–132° knee range against the thigh side — showed the *existing*
shank web driving **22 mm into the hip-yaw body at 132°**, with the honest safe
range being **0–93°** on the rounded plates. This is the near-tangent yaw/knee
pair from the layout notes, in the swept case: the yaw body hangs 46 mm above the
knee axis, and in deep flexion the shank folds up behind the thigh through
exactly that volume. It cannot be fixed by shaping the shank; any rear structure
enters that zone above ~110°.

**3. So the knee moved back 55 mm.** That is what the G1's 53 mm knee-back
offset *is* — not styling, clearance for deep flexion — and our number came out
within 2 mm of theirs from a completely independent calculation. `KNEE_BACK` is
now a layout parameter threaded through layout, plates and export. Full 132°
clears the yaw body by 4.7 mm; the whole leg (knee, ankle pitch, ankle roll, hip
yaw) sweeps clean.

**4. The thigh channel is closed at the FRONT and open at the BACK** — the
opposite of first instinct, for the reason in (2): the back is where the shank
goes. Web on the front, side flanges 56 mm off the yaw axis (the X6's connector
boss reaches R50.9 and the body rotates relative to the thigh, so the wrap must
clear it all the way round), knee stator plate bent from the web with a rear
gusset. 118 wide, 99 deep; the G1's thigh is 89 × 180. The shank gets its
front flange only, for the same reason.

**5. Then it fell over.** With the knee back, a straight leg puts the ankle 55 mm
behind the hip — permanently, it is in the link geometry — so the standing COM
sits ~50 mm ahead of the ankle. **That is how a human stands** (COM ahead of
the ankle, held by steady plantarflexor torque). The rig's PD droops under a
steady load and tipped at 0.6 s. Stiffening the leg gains (`IPSE_STAND_LEG_OMEGA`
40 → 80) holds it: **−0.1 % force, COM→COP 4.6 mm, contact never lost**, ankle
PD steady at ~9 N·m — 13 % of what X4-36 has. The G1 does this in its controller
too. The knob is in ipse's `k1_summary`, default unchanged.

**6. rose pro is 3.9 kg of sheet now, up from 3.5.** The wrap costs ~370 g a
side. Worth it; that is the difference between parts and a body.

## 1800 mm

2026-08-19, after the H2 pass: "rose pro should be about 1800 mm, rose max
2000." `SIZING.md` is the decision (X15-450 hip pitch and knee, X12/X10 hip
roll/yaw, X8 ankle and shoulder pitch; ~68 kg), `rose_pro_layout.py` is now
parametric on HEIGHT and the actuator envelopes, and `rose_pro_links.py`
derives every plate from it. What the bigger actuators forced, each found by a
gate and recorded in the code: the hip roll/yaw/knee column moved OUTBOARD of
the D166 pitch body (y 164, as the H2 does); the hip-roll pod sits behind the
hip (its output plate must stand outside the r-104 sweep of the thigh flanges);
hipB is a Z-bracket (stator plate -> leg up through the channel top inside
r 81 -> bridge -> roll output); the yaw output plate sits on a 6 mm spacer; the
thigh's channel top is its own plate, rear-clipped, and the flanges no longer
wrap the yaw body's rear half; the elbow flexes forward only; the arm hangs at
y 255 to clear the hip pods; the chest is +/-165 (chest:waist 1.70).
The `cobot.py` height map is no longer perverse: at 1800 the head cage meets a
standing adult's head. The 1173 mm body is commit 6999477.

## What "she's starting to look heavy" turned into

The wrap made it one body, and one body read heavy. The second pass measured
the Unitree H2 from its own meshes (`UNITREE.md`, "H2, measured") and found the
heaviness was three specific things, none of them "boxy": the leg was widest at
the *knee* (126 × 162 mm, where the H2's is a 46-wide pinch); torso + hip stack
+ thigh wrap was one prism with no width reversal for 48 % of the height; and
the chest kept its full width 70 mm above the shoulder axis. The cut that
answered it — thigh jog, chest over a closed pelvis pod, deck down, windows
with hemmed edges for hole fields, arms as channels — is in `rose_pro_links.py`
and the lineup is `render/lineup-{iso,front,side}.png`.

Two rules came out of it:

- **Every axis-aligned step is a chance to reverse the width.** The H2 reverses
  four times between chest and knee. We cannot taper, but each fold is a step,
  and steps in the right order read as a figure. Order: chest > pelvis >
  thigh-top > knee > shank > ankle.
- **A sweep decides the shape before taste does.** Every odd number in the new
  arm (web top 15 below the roll axis, front flange at y 148, fork inboard)
  is where `sweep.py`'s two new arm sweeps put it. Draw the structure, sweep
  it, and let the clearance tell you what the section is.

## What an H3 would be

The H2 is the first humanoid from Unitree that is shaped like a person rather
than a crash-test dummy — and it is shaped by cast aluminium and injected
shells. An "H3" is that silhouette taken one step further *toward* the human
figure, made in our process, and more honest about what it is. In order of
what the eye reads at 3 m:

1. **Widths in human order, four reversals.** Chest widest at the shoulder
   line; a visible waist with the yaw ring exposed; a closed pelvis pod wider
   than the waist; thigh widest under the hip; knee a pinch; calf a swell;
   ankle a pinch; a shoe. The H2 does all of it with continuous taper. We do it
   with **trapezoid flanges and chamfers** — plates tilted about one in-plane
   axis — so every taper is a fold you can see. That is the one thing an H3 in
   sheet does *better* than the H2: the surface tells you how it was made.
2. **The joints you see are the ones that explain.** Shoulder, hip, wrist and
   the knee's output ring show as dark discs. The knee stator and the hip roll
   bodies — the two scissor closures that can pinch — get faceted shrouds
   (rule 3). Nothing else is covered.
3. **A head that is a helmet, not a die.** Small, closed, low on a collar that
   hides the neck actuators, a visor slot with a real stereo pair behind it,
   resting gaze slightly down. Understated: the H2's head is where it stops
   looking like a machine; ours should still look like one.
4. **Hands.** A faceted mitt with a thumb, hemmed, legible as a hand. A robot
   with no hands reads as a rig.
5. **Arms and shins that are shells, not sticks.** Tapered channels shoulder to
   wrist and knee to ankle, the actuators inside them, the pushrods visible
   because they explain.

What an H3 is *not*: cast, smooth, white plastic, faceless, or pretending.
Every fold is a brake hit; every opening has a hem; every shroud covers a
pinch and nothing else.

## The open geometry work

- **Tilted plates.** Chest:waist is 1.32 against the H2's 1.6 and nothing
  tapers; that is the axis-aligned `Plate` class. One plate tilted about an
  in-plane axis buys chamfered shoulders, a chamfered waist, trapezoid thigh
  flanges and knocked-off hip corners. `sweep.py` is all SDFs; a tilt is one
  transform on the way in.
- **The hip stack and the ankle stack** still read as piles of cylinders. Both
  are layout (three X6-60 bodies between hip pitch and thigh; two X4-36 at the
  ankle). UNITREE.md items B and C — pushrod ankle, pelvis hub — are what fix
  them, not the generator.
- **The head** is a cube on a neck. It was always meant to be understated; it
  now reads as the heaviest thing on the robot next to the H2's.
- **Hip couplings.** hipA's tab is not geometrically connected and hipB cannot
  be one two-bend blank; `rose_pro_bent.py` is owed a pass.

## One finding worth carrying forward

The audit ranks exposed edges by which body region sits at the same height. On
the **1130 mm** geometry, the worst result is that the head cage — the hardest,
most angular structure on the robot — sits at the height of a standing adult's
**abdomen**, whose 110 N limit is the most fragile on the torso.

That is an argument for the 1750 mm height beyond ergonomics or looks: at human
scale, heads meet heads and hands meet hands, and the contact map stops being
perverse. It also means **this audit must be re-run after the re-cut** — its
answer is a function of height, and every number in it is about the 1130 mm body.
