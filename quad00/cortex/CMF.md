# Cortex colorways (CMF)

*Updated 2026-10-21 per Cam: no brown. Black or white plastics, one fun accent, and a minimal first release.*

Each cortex head keeps its own personality and can move between bodies, even bodies of a different shape. The head's colour says what it's for. Colours, materials and finishes are set by parameters in [`cortex.loon`](cortex.loon) (mini), [`cortex-full.loon`](cortex-full.loon) and [`cortex-mega.loon`](cortex-mega.loon). The geometry is identical across colorways.

- **Pick a colorway:** set `[let colorway "mind"]`.
- **Finish:** `shell-rough` sets the shell's finish, `accent-rough` the ear panels' finish.

## System

- **Two body tones:** black (#121213) or white (#ecece9). No warm or brown tones.
- **One family neon** as the accent.
- **"One flame":** the neon goes **only on the two ear panels**. The exhaust, back, floor and face stay quiet.
- **Release 1 is minimal.** There's no decorative perforation anywhere:
  - **Ear panels:** solid accent plates set into the side windows. Sound and the mic port use a hidden 0.8 mm gap round each panel, so there are no visible holes.
  - **Exhaust:** three plain 2 mm slots in shell colour on the back sweep. It's functional, because the Jetson's heatsink exhausts there.
- Reference: the clean, accent-panel look of Seeed's reBot arm, in our own language. The same restraint applies to the body (torso, neck, limbs): no decorative dot grids, and perforation only where a measured thermal need exists, hidden where it can be.

| Part | Takes |
|---|---|
| Tub (shell) and exhaust insert | Body tone, satin |
| Ear panels | Accent neon, semi-gloss |
| Floor plate | Anodized: black under a black head, clear (silver) under a white head |
| Face glass | Always dead-front black |
| Eyes (drawn on the screen) | The accent neon |

## Colorways

| Name | Shell | Accent | Floor | Read |
|---|---|---|---|---|
| **Mind** (flagship) | Black | Mind orange #ff5f00 | Black | The head is the Mind. Default and release hero. |
| **Make** | Black | Make green #39ff14 | Black | Builders: arms, fab cells |
| **Body** | Black | Body red #ff2052 | Black | Locomotion: quads, humanoids |
| **Play** | Black | Play pink #ff3db8 | Black | Companion and desk heads (Cam, 2026-10-21) |
| **World** | White | World cyan #00f0ff | Clear | Field and outdoor work, people-facing |
| **Power** | White | Power amber #ffd400 | Clear | Energy and charging roles |
| **Silicon** | White | Graphite #4d4d4f | Clear | The quiet one: no neon |

- **Black shells:** the glass blends into the shell, and the face reads as one dark object with lit eyes. This is the strongest look and the flagship.
- **White shells:** the black glass reads as a framed screen, which is friendlier.

## Manufacturing

All cost deltas are rough estimates; get supplier quotes before relying on them.

| Part | Process | How the colour is made | Delta vs. black |
|---|---|---|---|
| Tub, MVP | FDM PETG on the P1S | Black or white filament | $0 |
| Tub, 1k | Injection-moulded PC/ABS, MT-11010 texture | Coloured in the resin. White needs a UV-stabilised grade. | About $0; one extra resin SKU. White shows wear more. |
| Ear panels | MVP: PETG in the accent colour. 1k: moulded PC with fluorescent masterbatch. | True neon needs fluorescent pigment | About +$0.10–0.50 per part |
| Floor plate (5052 Al) | Anodize: black, or clear | | About $0 |
| Face glass | Dead-front print | Black on every colorway | $0 |

Fluorescent pigments fade under UV over a few years. Use a UV-stabilised resin and treat the ear panels as swappable. Recolouring a head is just two panels.
