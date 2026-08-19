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

## Sources

- G1 URDF: `~/Developer/vcad/examples/unitree-g1.urdf` (g1_23dof)
- G1 teardowns: [chinabizinsider](https://chinabizinsider.com/unitree-g1-teardown-decoding-the-economics-and-engineering-of-the-worlds-best-selling-humanoid-robot/), [Jonver](https://www.jonver.cn/newsdetail_195)
- H2 spec reveal: [Humanoids Daily](https://www.humanoidsdaily.com/news/confirmed-unitree-h2-specs-reveal-31-dof-humanoid-with-major-hardware-overhaul)
- H1 / M107: [Unitree H1](https://www.unitree.com/h1/), [quadruped.de H1 docs](https://www.docs.quadruped.de/projects/h1/html/h1_overview.html)
- G1 ankle PR/AB modes: [quadruped.de G1 docs](https://docs.quadruped.de/projects/g1/html/g1_overview.html)
- DecARt Leg, FAST metric: [arXiv 2511.10021](https://arxiv.org/abs/2511.10021)
- Superman: [TechNode](https://technode.com/2026/08/18/unitree-says-its-new-humanoid-reaches-12-66-m-s-and-jumps-2-meters/)
