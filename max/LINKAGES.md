# rose-1 — torque-path & fastener ledger (v4, 2026-07-29)

Rule adopted this pass: **every joint interface is either a modeled part or a
DECLARED BOLTED face pair with ≤4 mm gap** listed here. Anything not in this
table does not exist as a load path. Generated geometry: `cad/gen-rose1.py`
(v4); the sweep checker re-verifies every clearance on every run.

Hub-joint principle (knee/elbow/all rana outputs): the actuator's
crossed-roller output bearing takes the joint moment, so a single cap plate
bolted to the output flange is the correct attachment — no fork needed.
Where a housing is *held* (stator side), the mount is on the base face.

## Leg — ground up

| # | from → to | modeled part(s) | bolted interface | bolts (per side) |
|---|---|---|---|---|
| L1 | ground → foot | rubber sole + foot plate + toe | bonded sole | — |
| L2 | foot → ankle | roll pin (⌀12, axis X) through foot bracket; pitch pin (⌀12, axis Y) through clevis plates + ankle block | pins in bronze bushings | 2× shoulder bolt M6 |
| L3 | ankle → shin | shin clevis riser (28×36×40) + twin clevis plates | riser is shin material | 4× M4 |
| L4 | ankle torque ← calf pods | crank discs (r20) on both pod output flanges → ⌀8 pushrods → foot horns (inboard + outboard wings) | rod-end spheres at crank pin & horn | M5 rod ends ×4 |
| L5 | calf pods → shin | pod A output inboard, pod B outboard (crank lanes clear both housings, checker-verified) | pod base faces on shin front, machined seats | 6× M4 each (rana-100 BCD) |
| L6 | shin → knee output | shin cap disc (r34) on knee flange + drop member + transition block into shin top | cap-to-flange | 6× M5 (rana-140 BCD) |
| L7 | knee housing → thigh | knee plate (88×8×100) on housing inboard base face, continuous with thigh | plate-to-base bosses | 6× M5 |
| L8 | thigh → hip pitch output | thigh cap disc (r34) on pitch flange + thigh drop member | cap-to-flange | 6× M5 |
| L9 | pitch housing → roll output | 3-piece hip carrier: base plate (y22–30, out of thigh lane) + bridge + adapter strip (r≤14.6 — thigh swept bore starts at r20, +5.4 mm) | adapter-to-roll-flange top+bottom; base-plate-to-housing | 6× M5 + 4× M5 |
| L10 | roll housing → yaw output | yaw carrier saddle (64×32×8) over roll housing top, clamped to ⌀16 yaw shaft | saddle clamp + housing bolts | 2× M5 clamp + 4× M4 |
| L11 | yaw shaft → rib | flanged bearing boss (⌀32/⌀18) in rib bore — leg weight rides this thrust face | boss-to-rib | 4× M4 |
| L12 | yaw housing → rib top | stator base on rib, shaft down through | base flange | 6× M4 |

## Waist & torso

| # | from → to | modeled part(s) | bolts |
|---|---|---|---|
| W1 | ribs → spine | ribs extended to y40, 4 mm overlap with spine plate (one weldment or) | 4× M4 each |
| W2 | spine → tower | ⌀56 tower column z118–206 (was FLOATING 88 mm in v3) | 6× M5 base ring |
| W3 | tower → waist yaw stator | yaw base on tower top | 6× M4 |
| W4 | yaw flange → saddle → yokeA → roll housing | saddle plate (z250–254) + end-cap yoke; roll housing cradled on saddle | 6× M4 flange, 4× M4 yoke |
| W5 | roll output flange → chest | chest bracket (x −6..2, z274–358) up to chest base; bottom corners sized so ±30° roll clears the saddle (+4 mm swept) | 6× M4 flange, 6× M4 base |
| W6 | chest base ↔ shells ↔ deck | sheet-metal panels, tab-and-bolt | M4 pattern, TBD count |

## Arm — shoulder down

| # | from → to | modeled part(s) | bolts |
|---|---|---|---|
| A1 | deck → pitch housing | deck boss (50×16×60), 4 mm bolt gap to stator base | 6× M4 |
| A2 | pitch flange → roll housing | 3-piece bracket: flange plate + top strap + end cap (strap sits outside the roll circle — abduction sweep +30 mm) | 6× M4 flange, 4× M4 cap |
| A3 | roll flange → yaw housing | roll output plate (r≤41 of roll axis so it clears the strap while abducting) | 6× M4 |
| A4 | yaw flange (down) → upper arm | upper arm top face, 4 mm bolt gap | 6× M4 |
| A5 | upper arm → elbow housing | elbow mount plate on housing base face | 4× M4 |
| A6 | elbow flange → forearm | elbow cap disc + drop member + transition into forearm top | 6× M4 |
| A7 | forearm → hand | bolted puck | 4× M3 |

## What the checker now proves about all of this

Every part above is registered in the sweep sets: the carriers yaw with the
leg (+4 mm vs pelvis through ±150°), the shin cap/drop/rods flex with the
knee (166°), the elbow cap/drop/transition flex with the forearm (120° —
the honest limit of the hub-joint elbow; noted), the chest bracket rolls
with the chest (±30° clears the saddle by +4 mm), and the shoulder bracket
survives a full 360° pitch circle (+13 mm).

Mass with all hardware: **24.8 kg vs 25 kg target** (solid-volume upper
bound ×0.35 shelling on structure; actuators and allowances exact).

## Not yet modeled (known, ordered)

1. Individual bolt bodies + heads in the CAD (counts/sizes declared above;
   frog mk14-style flat-head sweep pass pending).
2. Hollow-shaft cable routing (rana ⌀12 bore) — harness path unproven.
3. Ankle pushrod kinematics as a function of pitch/roll angle (four-bar
   transmission ratio) — geometry exists, transmission curve unverified.
4. Structural sizing of the new brackets (closed-form like frog's Bredt
   pass, or FEA) — sections are eyeballed at plausible thickness.
