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

## The open geometry work

From the renders, three faults that are shape rather than safety:

- **The arms have no mass.** Four bare actuator cylinders with tab-sized plates
  between them. The proportions pass fixed the *lengths* (upper arm 150→190,
  forearm 130→155) and never gave them *section*.
- **The torso is a perforated slab.** Grading the lightening cutouts was right,
  but it is still a field of holes on a flat plane, and it reads as decoration.
  Replace it with folded ribs that visibly follow load paths into the mounting
  faces: same mass saving, and the pattern explains the structure.
- **Shoulder line too high relative to the head.** A human torso is ~30% of
  height. Shortening and deepening the torso rather than scaling it tall is what
  stops it reading as a fridge with a hat.

## One finding worth carrying forward

The audit ranks exposed edges by which body region sits at the same height. On
the **1130 mm** geometry, the worst result is that the head cage — the hardest,
most angular structure on the robot — sits at the height of a standing adult's
**abdomen**, whose 110 N limit is the most fragile on the torso.

That is an argument for the 1750 mm height beyond ergonomics or looks: at human
scale, heads meet heads and hands meet hands, and the contact map stops being
perverse. It also means **this audit must be re-run after the re-cut** — its
answer is a function of height, and every number in it is about the 1130 mm body.
