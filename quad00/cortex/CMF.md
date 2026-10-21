# Cortex colorways (CMF)

Each cortex head keeps its own personality and can move between bodies, even bodies of a different shape. The head's color says what it's for. Colors, materials and finishes are set by parameters in [`cortex.loon`](cortex.loon); the geometry is identical across colorways.

- **Pick a colorway:** set `[let colorway "mind"]`.
- **Finish:** `shell-rough` sets the shell's finish, `accent-rough` the grille inserts' finish.
- **Lineup scene:** [`lineup.loon`](lineup.loon) shows all six heads side by side.

## System

The palette is the master plan's tokens:
- **Two body tones:** warm black (#1e1813, "raised") or cream (#e8e1d2).
- **One family neon** as the accent.

"One flame per page": the neon goes **only on the two ear grille inserts**. It's a signal, not decoration. The exhaust grid, back, floor and face stay quiet.

| Part | Takes |
|---|---|
| Tub (shell) and exhaust grille | body tone, satin |
| Ear grille inserts | accent neon, semi-gloss |
| Floor plate | anodized: warm black (#0b0907) under a black head, tan (#a08d70) under a cream head |
| Face glass | always dead-front black |
| Eyes (drawn on the screen) | the accent neon. The head's color shows up on its face too. |

## Colorways

| Name | Shell | Accent | Floor | Read |
|---|---|---|---|---|
| **Mind** (flagship) | warm black #1e1813 | Mind orange #ff5f00 | warm black | The head is the Mind. Default and release hero. |
| **Make** | warm black | Make green #39ff14 | warm black | Builders: arms, fab cells |
| **Body** | warm black | Body red #ff2052 | warm black | Locomotion: quads, humanoids |
| **World** | cream #e8e1d2 | World cyan #00f0ff | tan | Field and outdoor work, people-facing |
| **Power** | cream | Power amber #ffd400 | tan | Energy and charging roles |
| **Silicon** | cream | Silicon tan #a08d70 | tan | The quiet one: tone on tone, no neon |

**Readability with the dead-front face:**
- **Warm-black shells:** the glass blends into the shell, and the face reads as one dark object with lit eyes (Apple's black-on-black move). This is the strongest look, which is why the flagship uses it.
- **Cream shells:** the black glass reads as a framed screen. That's higher contrast and friendlier.
- **Eyes:** all six neons are bright enough on the black screen, including amber.
- **Silicon:** tan accent on a cream shell is low contrast by design. Its eyes should stay cream or white.

## Manufacturing

All cost deltas below are rough estimates; get supplier quotes before relying on them.

| Part | Process | How the color is made | Delta vs. black |
|---|---|---|---|
| Tub (shell) | Injection-moulded PC/ABS, MT-11010 texture | Colored in the moulding resin (no paint). Cream needs a UV-stabilised grade. | About $0 per unit; one extra resin SKU. Cream shows wear more. |
| Tub, prototypes | 3D printing (MJF / SLS) | MJF is grey and dyed black only. Cream means SLS white plus paint, or SLA plus paint. | About +$15–30 per unit for cream at prototype stage |
| Ear grille inserts | Injection-moulded PC | **Fluorescent masterbatch** (pigment pellets) in the resin. True neon needs fluorescent pigment; normal pigment, paint or anodize can't hit #39ff14 or #00f0ff. | About +$0.10–0.50 per part; same mould, swap the masterbatch |
| Ear grilles, prototypes | 3D printing | SLA in fluorescent resin, or white SLS plus fluorescent paint | About +$5 per pair |
| Floor plate (5052 Al) | Anodize | Black for warm black; light gold or champagne anodize for tan | About $0 |
| Face glass | Dead-front print | Black on every colorway | $0 |

**Caveats:**
- Fluorescent pigments fade under UV over a few years. Use a UV-stabilised resin and treat the grilles as a swappable part. They're separate inserts already, so changing color means changing two small parts and nothing else.
- A head can be recolored later just by swapping its grilles.
- Anodize alone can make orange, red and gold, but not the fluorescent green or cyan. That's why the accents are on the plastic inserts and not on the metal parts.
