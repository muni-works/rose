# What Unitree does, measured — and why rose pro reads as a kit of parts

Research pass, 2026-08-18. Sources are the G1's own URDF (`g1_23dof`, local copy
at `~/Developer/vcad/examples/unitree-g1.urdf`), two G1 teardowns, the H2 spec
reveal, the H1 M107 datasheet, and the DecARt Leg paper (arXiv 2511.10021), which
benchmarks the G1's leg against others with a real metric. Where a number comes
from a URDF it is a design number; where it comes from a teardown it is a
reporter's number.

## The one-sentence diagnosis

**Unitree wraps; we tab.** Every Unitree link is a closed shell with the actuator
*inside* it, and the next joint's stator bolts to the inside of that shell. Our
links are flat plates bolted to the *outside* of bare actuator cylinders, joined
by tabs. The load path in theirs is continuous shell from joint to joint; in ours
it is plate → bolt → flange → cylinder → flange → bolt → plate. That is why theirs
reads as one body and ours reads as parts that could come apart — because it is
the parts that could come apart.

## G1 by the numbers (from its URDF)

| | G1 | rose pro (1130) | note |
|---|---:|---:|---|
| height / mass | 1.32 m / 34.1 kg | 1.13 m / 17.6 kg | |
| hip half-spacing | 116 mm | 75 mm | theirs is wider — bigger actuators |
| hip pitch → knee | 332 mm | 250 mm | |
| knee → ankle | 300 mm | 250 mm | |
| thigh : shank | 1.11 | 1.00 | both near human |
| knee axis vs hip yaw axis | **53 mm behind** | 0 | their leg has an S through the hip |
| ankle pitch→roll stack | **18 mm** | **60 mm** | see below |
| ankle pitch *link* mass | **0.074 kg** | ~0.36 kg (X4-36) | the tell |
| one leg | 7.18 kg = 21 % | | |
| torso + pelvis | 12.4 kg = 36 % | | pelvis alone is 3.8 kg |
| one arm | 3.05 kg, 5 actuators | 1.44 kg, 4 actuators | ours are sticks because they are |
| efforts (N·m) | hip 88, knee 139, ankle 35, arm 25 | 60 / 60 / 34 / 34 | |
| knee effort / (m·g·L_thigh) | 1.25 | 1.39 | we are *not* under-torqued at 1130 |

Three structural facts fall out of that table that no amount of surface styling
would have found:

**1. The ankle actuators are not at the ankle.** The G1's ankle-pitch link masses
74 grams. It is a yoke. Both ankle actuators live *up in the shank* and drive the
foot through a pair of pushrods — the "parallel ankle" the G1 docs describe with
its PR/AB control modes. That is why their whole ankle stacks in 18 mm and ours
takes 60, and why our foot region renders as a pile of cylinders while theirs is
one clean shank ending in a foot. It also puts ~0.7 kg of actuator 300 mm higher
up the leg, which the DecARt paper shows is worth real swing speed.

**2. Each link's mass includes the *next* actuator, and the link is shaped
around it.** hip_yaw_link is 1.70 kg and hip_roll_link 1.52 kg — those are the
knee and yaw motors, respectively, sitting inside the link that carries them.
Their COMs sit 150 mm and 88 mm below the joint origin: the actuator hangs down
*inside* the shell of the link above. Our layout already does the same trick with
the hip yaw body hanging inside the thigh — but we then put a flat plate beside
it instead of a channel around it.

**3. The pelvis is a real part.** 3.8 kg, a hub that both legs *and* the waist
bolt into, with the hip pitch axes 103 mm below it. Ours hangs the hips straight
off the trunk box. A pelvis is what lets a torso be a torso and a hip be a hip.

## What the teardowns add

- Thigh and shin are **high-strength aluminium, topology-optimised with
  diamond cutouts**; the lower shin is the *one* place they use steel, because it
  takes jump and backflip impact directly. Cast/machined, not sheet.
- **Hollow-shaft cabling** everywhere: every motor cable runs through the centre
  of the joint. No external harness. This is a large part of why they look
  finished — there is nothing to see except structure and joint.
- Joints are **49 % of the mass** (17.25 kg across 23) and **66 % of the BOM**.
- Thermal: copper spreaders from the knee joints into the structure, a fan in
  the waist, NTC derating. Passive except at the two hottest points.
- R1 (1.23 m, 25 kg) moved from the G1's metal frame to **composites and
  engineering polymers**. H2 (1.8 m, 70 kg) uses aluminium + titanium + plastics.

## Where Unitree is *going* (H2, Superman)

The H2 abandons the G1's own leg. Knee and ankle motors move **up to just below
the hip and just below the knee** and drive through **steel cable tendons** —
proximal mass, low reflected inertia, explicitly described as "sim2real
friendly". The parallel RSU ankle became a single Achilles tie-rod for pitch.
Hip order standardised to F-A-R (pitch-roll-yaw), same as ours. Waist became a
serial Z-Y-X stack because parallel mechanisms were hard for RL.

Superman (17 Aug 2026): 0.85 m legs, 360 N·m joints, 2.0 m claimed standing
jump. Whatever the leg is, it is not a knee motor at the knee.

## The DecARt result, because it settles a question

The paper's "Fastest Achievable Swing Time" metric, run on real URDFs with real
actuator limits: **every robot with knee actuators at the knee swings slower**
(G1 0.34 s, Booster T1 0.33 s) than every robot with actuators at the root of the
leg (Cassie 0.24 s, DecARt 0.17 s) — and moving the *same* actuator from the knee
to the hip on the *same* leg goes 0.25 → 0.17 s. Actuator placement is a design
variable with a measurable payoff, and it points the same way as our sim2real
thesis: proximal mass is easier to model.

## What this means for rose pro, in order of leverage

**A. Wrap the actuators (generator change, cheap, biggest visual payoff).**
Every limb link becomes a folded channel that encloses its actuator envelope —
for an X6-60 (⌀80 × 67.5) that is a U ~86 mm inside, ~72 mm deep, 3 mm wall,
*two bends*. The next joint's stator bolts to the channel web; the previous
joint's output bolts to its flanges. Same plate table, same bolt circles, same
bend rules — but the plate is now *around* the cylinder instead of *beside* it,
and the load path is continuous. This is what makes it read as one body. Sheet
does closed sections fine; that was never the constraint.

**B. Move the ankle actuators up the shank (kinematic change, medium).** Adopt
the parallel pushrod ankle. rose max's design already commits to pushrod ankles,
so this brings pro into the line's language rather than out of it. Kills the
60 mm ankle stack, drops ~0.7 kg to the shank, cleans the foot completely. It is
a real mechanism to design and a URDF change (closed chain or a mimic joint).

**C. Add a pelvis (layout change, medium).** A hub the two hip-pitch stators
and a waist yaw all bolt into, so the trunk is not the hip mount. Buys a waist
DOF (the G1 has one; we have none) and gives the torso a shape that is not "the
box the legs hang from".

**D. Give the arms section (generator, cheap).** Same channel treatment as A;
they get mass and stop being sticks. Add a shoulder yaw (5-DOF arm) if we want
to match the G1's reach; ours are 4-DOF.

**E. Move the knee up too (architecture, later, max's problem first).** Tendon
or pushrod knee from a hip-mounted motor. This is the H2/DecARt move and it is
where the swing-speed and sim2real wins are. rose max should try it first; if it
works there it comes down to pro.

What we should *not* copy: cast topology-optimised aluminium. That is the wrong
process for a machine whose whole thesis is that it can be cut and bent from
sheet. Copy the *architecture* — wrapped links, proximal actuators, a pelvis,
hidden cabling — in the process we actually have.

## H2, measured — and why the wrapped rose pro read heavy

Second pass, 2026-08-19, after the wrap. Unitree published `h2_description`
(URDF + STL) in `unitree_ros` in 2026; it is sparse-checked-out beside the G1
and rendered with our own `vcad-render` — `pro/reference/h2-full-{iso,side,front}.png`,
composed by `pro/reference/urdf2loon.py`. Every H2 number below is measured
from those meshes (25 mm slices along each link) or read from the URDF; the few
photo estimates are marked.

### The H2 by the numbers

| | H2 | rose pro (before) | rose pro (after) |
|---|---:|---:|---:|
| height / mass | 1.82 m / 75.6 kg (URDF sum) | 1.17 m / 17.6 kg | same |
| hip pitch → knee | 412 | 250 | 250 |
| knee → ankle | **497** | 250 | 250 |
| shank : thigh | **1.21** | 1.00 | 1.00 |
| leg (sole→hip pitch) / height | 53 % | 52 % | 52 % |
| shoulder height / height | 83 % | 77 % | 77 % |
| torso shell height / height | 26 % | 32 % (one box, -80..300) | chest 18 % over waist 14 % |
| chest width : waist width | **1.6** (255 : 160) | 1.0 (206 : 206) | 1.32 (206 : 156) |
| pelvis | separate pod, 216 wide × 196 tall | none — hips hang off the box | 156 × 160 pod, hip rings outside it |
| thigh section, hip end → knee end | 143 × 128 → **72 × 74** | 85 × 74 → 118 × 99 → **126 × 162** | 85 × 74 → 124 × 99 → **100 × 100** |
| shank section, knee → calf → ankle | 46 × 98 → 87 × 96 → **52 × 40** | 66 × 88 flat, ankle 116 × 70 | unchanged |
| upper arm section | 78 → 61 | 3 mm strip | 77 × 80 channel |
| forearm section | 64 → 78 | nothing | 78 × 22 fork |

Three of those rows are the whole diagnosis, and none of them is "boxy":

**1. Our leg was upside-down.** The H2's thigh is widest under the hip and
tapers 2:1 to a knee that is a *pinch* (46 wide) between two cones; the shank
swells to a calf 30 % down and tapers to a 52 × 40 ankle. Ours was narrowest
at the hip roll (85 wide), widened to 118 through the yaw wrap, and hit its
widest and deepest section **at the knee: 126 × 162 mm** — because the knee
stator plate bridged from the front web (x = 59) back to a knee 55 mm behind the
yaw axis, a 160 × 65 slab, and the knee body itself is a D80 cylinder lying
across the link. A leg that gets *bigger* toward the knee reads as a club.

**2. Half the robot was one prism.** Torso box -80..300 plus the hip stack
hipA/hipB (-60..-196, 73–88 wide) plus the thigh wrap (118 wide to -265): a
continuous box from z 300 down to -265 with no width change bigger than 30 mm —
**565 mm, 48 % of the height**. The H2 over the same span goes chest 255 →
waist ~160 → *gap* (the waist yaw/roll links are 60 wide and exposed) → pelvis
216 → thigh 128. Four width reversals; ours had none.

**3. The shoulders were holes in the side of a box that kept going.** The
shoulder pitch axis at z 230 with the deck at 300: 70 mm of full-width box
above the shoulder line. The H2's chest is widest 60 mm *below* its shoulder
pitch axis and narrows to 96 at the neck, so the shoulders are the top corners.

And the arms were not there: the upper arm was a 3 mm strip that did not
reach the roll output it was meant to hang from, and the forearm had no
structural member at all between the elbow output plate and the wrist stator.

### What the H2 does that we can copy in sheet, and what we cannot

Copyable: the *order* of widths (chest > pelvis > thigh-top > knee), a waist
between chest and pelvis, the shoulder line at the top of the chest, a thigh
that is bigger at the hip than the knee, arms with section, one long aperture
per shell face rather than a field of holes, the actuator rings showing as dark
discs at the hip and shoulder and nothing else. Not copyable: continuous
taper (every H2 section changes along its length; ours can only step between
axis-aligned plates), a shank that pinches to 52 × 40 (our ankle actuators are
at the ankle, the H2's are in the shank on a tie-rod — item B in the list
above), a 1.21 shank:thigh (ours is a layout number in rose_pro_layout, not a
generator number).

### What was cut

`rose_pro_links.py`, one pass, every change below passes `sweep.py` (0 real
static overlaps — it was 4 — and six clean joint sweeps, two of them new) and
the generator's own checks (0 body clashes, 0 bend-rule violations):

- **Thigh jog.** The front web stops under the yaw body, bends into a shelf
  that runs back 65 mm, bends down into a 30 mm lower web at x = -6, and the
  knee stator plate bends off that. Knee plate 160 → 98 long; section at the
  knee 126 × 162 → 100 × 100. The leg now reads hip-stack 85 → yaw wrap 124 →
  knee 100 → shank 66: a step-taper, and the knee-back offset is now a visible
  fold, the way it is a visible S in the G1.
- **Thigh windows.** One 65 × 48 rounded-rectangle aperture in each flange,
  over the yaw body, with an 8 mm hem strip around it (the hem is modelled as
  the 3 mm doubled edge it becomes, on the outer face: inward it would come
  within 2 mm of the connector boss sweep).
- **Chest over waist.** Torso split at z 60: waist 156 × 115 × 160 (its side
  walls are the hip-pitch stator plates, grown to the full wall, so the hip
  output plates stand *outside* the pelvis exactly where the H2 shows its hip
  rings), chest 206 × 141 × 210, a ledge frame between them, deck dropped
  300 → 270 (40 above the shoulder axis). Chest:waist 1.32. The round hole
  fields are replaced by one hemmed window per chest face; the waist is closed.
- **Arms.** Upper arm: a channel off the roll output plate — outboard web with
  a D58 clearance hole the elbow body passes through, front flange, elbow
  stator bridged over the body to the web. Forearm: a fork bent off the elbow
  output plate, lapped at the wrist. Their dimensions are the shape of two new
  sweeps (`sho_roll` abduction 0..97°, `elb_pitch` ±132°): the web top sits
  15 mm below the roll axis because a corner at (30, -15) stays outboard of
  y 180 through the whole abduction range; the upper arm's front flange and
  stator bridge live outboard of y 147 because the forearm, a bar offset 37 mm
  from the elbow axis, sweeps everything within r 156 in its own y-band.
- **Four bookkeeping fixes** the gates had been carrying: hipB's bend tab was
  recorded 100 mm too high (punching through the hip stator and output
  plates — two of the four REAL overlaps), hipA's tab started inside the stator
  plate slab, the shoA lap tab sat in the roll output plate's slab, and the
  stereo pair's bores broke out of the cage face (eyes 54 → 48 mm apart).

Net sheet per side 2.96 → 3.43 kg (+0.47: the pelvis enclosure is +0.39, the
arms +0.22, the thigh -0.06). Those are net-of-aperture areas; the old
3.9 kg figure counted the holes as metal. Plate count 49 → 66.

### Honest residue

- The sweeps report **TIGHT** (< 4 mm but > 0) in eleven places, all at
  pancake stacks or at the arm's swept corners; the two worst new ones are the
  rolling output plate's inboard corner 0.6 mm from the shoA lap tab at 52°
  and the upper arm web 1.1 mm from the roll stator plate at 87°. Both are
  rounded-corner-to-rounded-corner at one angle. Not clashes; not comfortable.
- Arm *adduction* is not swept: the URDF allows -97°, which drives the arm
  through the torso. That is a URDF limit to fix, not structure.
- Hip pitch is not swept at all (it never was). The hipA bend tab sweeps a 72
  mm disc in the slab y 78..118 and the new forearm fork hangs at y 122..144,
  4 mm outboard of it.
- The hipA tab is geometrically unconnected (it stops 3 mm above roll_face's
  top edge) and hipB's blank cannot be a single two-bend part as the header
  claims — `rose_pro_bent.py` is owed a pass on both hip couplings.
- Chest:waist 1.32 against the H2's 1.6, and no taper anywhere: that is the
  axis-aligned Plate class. The next extension is a plate tilted about one
  in-plane axis — then the chest can chamfer to the shoulder and the waist,
  the thigh flanges can be trapezoids, and the hip corners can be knocked
  off. Everything in sweep.py is an SDF against a plate; a tilt is one
  transform on the way in.

## Sources

- H2 URDF + meshes: `unitreerobotics/unitree_ros`, `robots/h2_description`
  (local: `~/Developer/pond_assets/reference/unitree_g1/robots/h2_description`)
- H2 spec page: [unitree.com/H2](https://www.unitree.com/mobile/H2/) — 1820 × 456 × 218, ~70 kg, 31 DOF, calf+thigh 1045, forearm+upper arm 690
- H2 reveal: [Humanoids Daily](https://www.humanoidsdaily.com/news/unitree-unveils-h2-humanoid-successor-to-h1)

- G1 URDF: `~/Developer/vcad/examples/unitree-g1.urdf` (g1_23dof)
- G1 teardowns: [chinabizinsider](https://chinabizinsider.com/unitree-g1-teardown-decoding-the-economics-and-engineering-of-the-worlds-best-selling-humanoid-robot/), [Jonver](https://www.jonver.cn/newsdetail_195)
- H2 spec reveal: [Humanoids Daily](https://www.humanoidsdaily.com/news/confirmed-unitree-h2-specs-reveal-31-dof-humanoid-with-major-hardware-overhaul)
- H1 / M107: [Unitree H1](https://www.unitree.com/h1/), [quadruped.de H1 docs](https://www.docs.quadruped.de/projects/h1/html/h1_overview.html)
- G1 ankle PR/AB modes: [quadruped.de G1 docs](https://docs.quadruped.de/projects/g1/html/g1_overview.html)
- DecARt Leg, FAST metric: [arXiv 2511.10021](https://arxiv.org/abs/2511.10021)
- Superman: [TechNode](https://technode.com/2026/08/18/unitree-says-its-new-humanoid-reaches-12-66-m-s-and-jumps-2-meters/)
