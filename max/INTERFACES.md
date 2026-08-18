# rose — frozen interfaces (write-once, inherit-everywhere)

This file exists BEFORE any geometry, on the lesson frog taught ten marks in:
interfaces frozen early are what save you later. Anything here changes only by
family-wide decision recorded in rana/frog too.

## Inherited freeze (verbatim from rana-interface.loon)

| interface | value | provenance |
|---|---|---|
| Bus voltage | **48 V** (12S) | frog power arch; precharge+contactor+brake chopper REQUIRED |
| Data plane | **torc (Ethernet) or CAN-FD** — one lineup decision, pending torc jitter measurement | frog torc analysis; PoE carries data+logic only, never phase power |
| Connector set | PWR XT30/XT60 by size · DATA JST-GH 4-pin · encoder JST-GH 6-pin | rana SCHEMATICS.md §3 |
| Encoder | absolute magnetic (AS5047P class), diametral magnet, rear stub | rana |
| Actuators | **rana family only**: 100 / 140 / 190 (Kt 0.166 / 0.333 / 0.606) | rana-interface.loon table |
| Gear ratio policy | chosen PER JOINT (low = backdrivable at ankles/wrists; higher at hips) | family standard |
| Magnetics | 4 mm N42, 1.0 mm airgap, kw 0.933 topologies | rana |
| Fasteners | M3/M4/M5/M6 SHCS ladder, flat-head where a moving part passes over | frog mk14 sweep lesson |
| Bearings | 688-class idlers, thin-section outputs, HK needle in planets | rana/frog |
| Thermal contract | **the mount is the heatsink** — every actuator seat is a conduction path, ≥0.03 flatness, paste | rana solve_thermal (audited): free-air = 267 °C, mounted = 46 °C |

## rose-specific assignments (PREDICTED — sized, not yet swept)

Half-size first (**rose-1**: ~1.2 m, ~25 kg target), full-size (**rose-2**) later.

| joint | qty | actuator | why |
|---|---|---|---|
| Hip (3 DOF cluster) | 2× | rana-140 ×3 | 96 N·m covers squat/stair knee-worst ~80 |
| Knee | 2 | rana-140, femur-mounted | frog mk13 pattern |
| Ankle (pitch+roll) | 2× | driven by knee-level rana-100s via pushrods | frog mk14 four-bar pattern = the Cassie/Digit ankle |
| Shoulder (3 DOF) | 2× | rana-100 ×3 | |
| Elbow | 2 | rana-100 | |
| Wrist/neck | — | rana-100 or direct | later |

## Engineering conventions (process freeze)

- Z-up, mm, loon; models GENERATED (python) wherever mirrored — the humanoid is
  one giant left/right mirror and hand-mirroring caused 3 frog bugs.
- Verifier battery on every mark: COM symmetry, all-pairs interference, SWEPT
  interference on every joint range (frog's bulkhead collision hid at stance
  for 5 marks), consumer-tool validation for every export (STEP failed 3 ways
  in Shapr3D while self-round-trip passed).
- Readiness = the least-verified subsystem. Claims carry receipts; predicted
  stays predicted until hardware measures it.

## Benchmark targets adopted from Unitree R1 (datasheet, 2026-07-28)

| target | value | rose-1 v1 status |
|---|---|---|
| Envelope | 1210 × 357 × **190** | v1 ~1210 tall; 190-deep torso held, hips ~200 |
| Leg length | calf+thigh **675** (56% of height) | 340+335 = 675 ✓ |
| Arm length | upper+forearm **435**, 5 DOF (3 shoulder + 2 elbow) | 215+220 ✓ |
| Waist | **2 DOF**, Y ±150° R ±30° | modelled (rana-100 pair) |
| Joint ranges | knee −10..+148°; hip Y ±157° P −168..+146° R −60..+100° | adopted as sweep targets |
| Routing | hollow shafts + internal cabling | rana ⌀12 bore is the enabler — REQUIRED in v0.2 |
| Output bearings | crossed-roller | rana v0.2 change (was deep-groove) |
| Mass | ~25 kg with battery | rollup pending |

## v2 design notes (2026-07-28)

- **DEVIATION from frog mk13 pattern:** rose's knee actuator sits AT the knee
  as the joint hub (R1/G1 pattern) rather than femur-mounted + linkage. A
  humanoid shin is light and the hub-as-joint deletes a linkage; distal-mass
  cost accepted. Ankles KEEP the remote-drive pattern (twin rana-100 at shin
  top = the calf, pushrods down).
- **Self-audit (printed on every generation):** 6× rana-140 + 16× rana-100 =
  14.6 kg actuators; structure ~7.9 kg shelled; ESTIMATE **28.0 kg vs 25 kg
  target — 3 kg over, actuator-dominated.** Diet paths, in order: shoulder
  roll/yaw and elbow at reduced-current rana-100 builds (fewer magnets/turns —
  same housing), thinner shells, magnesium head/hands. NOT resolved yet.
- Clearance checks green at stance: thigh↔knee-disc +7, shin↔disc +8,
  calf↔shin +5 mm. RANGE SWEEPS STILL PENDING (hip at R1's ±150° will
  find things — that's the next mechanics gate). → CLOSED IN v3, below.

## v3 design notes (2026-07-29) — sweep-audited, mass on target

`cad/gen-rose1.py` now embeds a numeric interference checker (sampled
cylinders/boxes, signed distance, swept through R1 joint ranges) that runs on
every generation. It immediately found two STATIC interpenetrations v2's
hand-written checks missed — the forearm inside the hip-roll actuator, and
the pelvis side walls through the hip-roll envelope — vindicating the
process-freeze rule about swept verification.

Architecture changes the sweeps forced:
- **Pelvis = sagittal spine (y ±40) + hip ribs**, not a box: any wall inside
  |y| < 150 at hip-pod height collides with the roll pods.
- **Hip yaw body raised above the rib on a ⌀16 output shaft.** Proven: R1's
  −168° extension is geometrically impossible with any thigh width while a
  ⌀100 yaw body hangs in the thigh's flexion disc (the thigh centreline
  crosses the yaw axis). Lifting the body out of the disc buys ±146°.
- **Waist stack truly axially separated** (yaw 206–250, roll circle 254–354,
  chest base 358); v2's waist roll clipped the chest base. Chest shells drop
  to z 262 as a skirt that hides the taller column; front panel recedes to
  z 386 so the knee hub clears at full hip flexion (R1's chest does the same).
- **Ankle-drive pods moved to the shin FRONT** — rear pods capped knee
  flexion at ~113°; front pods clear the fold to 165°.
- Arms outboard (shy 170): forearm lane clears the hip-roll pods by 41 mm.

Sweep results (gate ≥2–3 mm, printed every run):
| sweep | result | R1 |
|---|---|---|
| hip pitch | **−145 flexion .. +146 extension** | −168..+146 |
| knee flexion | **165°** | 148 |
| hip yaw vs pelvis/waist | ±150 clear (+6 mm) | ±157 |
| shoulder pitch | 360° clear (+16 mm) | — |
| waist roll ±30 | clear (+8 mm) | ±30 |
| waist yaw, arms at rest | ±150 free | ±150 |
| hip yaw leg-vs-leg | −43° inward (software limit) | software |

The one honest miss vs R1: hip extension stops at ~146°, not 168° — a
fundamental consequence of keeping the thigh concentric with the yaw axis
(R1 offsets the thigh plane outboard of the yaw housing; we chose narrow
stance over the last 22°). Recorded as a spec deviation, not a defect.

Mass: **23.6 kg vs 25 target** (was 28.0; v4 with all joint hardware: 24.8). Diet executed: 11× rana-100-A
reduced-current build (same envelope, 0.38 kg — hip/waist yaw + all arm
joints), shells 8→5 mm, head shelled, battery counted once in the 2.6 kg
allowance. rana-100-A is a new family line item: same lamination/housing,
fewer turns — needs a row in rana-interface.loon before rose-1 ordering.

## v4 (2026-07-29) — every torque path closed

v3 was clearance-verified but mechanically fictional at 8 interfaces (the
frog mk6 lesson, again): floating waist yaw, unattached hip yaw output,
wrong-side knee/elbow attachments, decorative pushrods. v4 models the
complete load path ground→foot→ankle pins→shin→knee cap→thigh→hip carrier→
saddle→rib bearing→pelvis, plus waist tower/saddle/chest-bracket and the
full shoulder bracket chain — all registered in the sweep sets, all green.
The ledger of every interface + bolt spec is **LINKAGES.md** (new process
rule: an interface is a modeled part or a declared-bolted ≤4 mm face pair
in that table, or it doesn't exist). Ranges held: hip −145/+146, knee 166°,
elbow 120° (hub-joint elbow's honest limit). 24.8 kg.

## v5 (2026-07-29) — Unitree H2 restyle

Benchmark switched R1 → **H2** (182 cm / 70 kg / 31 DOF: 6 per leg, 7 per
arm, 3-DOF waist, 2-DOF head; 360 N·m leg joints; segment lengths
unpublished). rose-1 copies its slimness and DOF layout at half scale:
thigh 88→76, shin 50→44, arm sections −15 %, torso narrowed to 168 deep ×
208 wide with side panels trimmed to x ≤ 50 (the knee still clears at full
flexion — re-swept), H2-long feet (200 mm), rounded helmet head. All sweeps
green; extension improved to +151, flexion 142, knee 168°. 23.7 kg.

**H2 DOF gaps = rana family gap.** H2's 3-DOF wrists, 2-DOF neck, and third
waist axis all need an actuator well below rana-100's ⌀100×44 envelope.
Recorded requirement: **rana-60** (~⌀60×32, ~8 N·m @ small ratio) — same
freeze list (48 V, encoder, connector). Until it exists, rose-1 ships 5-DOF
arms, fixed neck, 2-DOF waist — deviations, not omissions. Bulk that
remains (actuator pods at hips/shoulders/shins) is the ⌀100/⌀140 pancake
diameter itself; H2-grade smoothness comes from covers, which the frog
precedent already assigns to a later fairing/skin pass.

## v6 (2026-07-29) — full actuators + sheet metal

Limbs converted to folded 5052 C-channels, plates normalized to waterjet
gauges (6/4/3 mm), shells 2.5 mm, foot a bent pan; actuators carry seam
ring, encoder boss, connector. **Mass fudge eliminated** — walls modeled at
true gauge and counted exactly: 25.1 kg (the ×0.35 estimate had been
flattering by ~1.4 kg). Cut list + gauge table: FABRICATION.md. All sweeps
green (envelopes unchanged). Loon gotcha for the record: `[union]` is
binary — n-ary unions fail with "no matching clause for anonymous with 3
args".
