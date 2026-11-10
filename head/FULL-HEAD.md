# Cortex heads: mini and full

There are two heads in one family. Each is a complete cortex: Jetson, battery, face, ears and the sealed dock. Both carry the persistent personality, and either one docks on any rose body.

- **Mini head:** [`cortex.loon`](cortex.loon)
- **Full head:** [`cortex-full.loon`](cortex-full.loon), for the 1703 mm humanoid. It is re-proportioned, not scaled up.

## Proportions

| | Mini | Full | Why |
|---|---|---|---|
| Height × width × depth (mm) | 121 × 164 × 104 | 190 × 230 × 160 | |
| Head height / body height | about 1/8 (on the ~1 m mini) | about 1/9 (on 1.70 m) | Small robots get a bigger head, which reads as friendly. The full reads as adult. |
| Width / height | 1.36 | 1.21 | The full is more upright. |
| Face rake | 8° back | 8° back | Same. It looks slightly up on a desk. |
| Back | Braun sweep | Braun sweep | Same shape, scaled to the head. |

## Parts that change

| | Mini | Full |
|---|---|---|
| Display | Waveshare 3.5" (E) 640 × 480 | Waveshare 5" (H), 121 × 89.5 |
| Stereo baseline | 106 mm | 150 mm: about 1.4× the depth resolution at the same range |
| Speakers | 2 × Ø28, 1 W | 2 × Ø40, 2 W |
| Battery | 3S1P 21700, about 50 Wh | 3S2P 21700, about 100 Wh (at the airline carry-on limit) |
| Compute | Orin NX on A603 | Same: one carrier and one software image |
| Ear and exhaust grids | Same 1.4 / 2.2 grid, smaller fields | Larger fields |

**Harness:** the full head's internal cable runs aren't modelled yet; they get re-routed later. The full head's parts are clash-checked: nothing overlaps and everything sits inside the wall.

## Dock: one standard

The full head uses exactly the same dock core as the mini, so any head docks on any body. That's what lets a personality move between bodies, including different body shapes.

- 8 pads
- cones at y ±18
- magnet strikes at y ±30
- latch keeper at x +20

The full head is heavier and sits higher on a taller neck. The full body's neck dock carries a stronger latch spring. The interface itself doesn't change.

## Looking down: gaze by body

A standing robot works below eye level, so its hands, the table and the floor are all below the camera.

- **Full:** from about 1.62 m, hands at table height (0.25–0.55 m ahead) are 57–73° below horizontal. The floor 1.5 m ahead is about 47° below.
- **Mini:** similar angles.

**Fixed 15–25° tilt isn't enough.** The stock IMX219 lens has a 49° vertical FOV, so its centreline would have to point about 50° down to see the hands. Add the face's 8° rake and that's past the neck's pitch range.

**Recommendation:**
1. **Keep the head identical across bodies, and point it with the existing neck-pitch joint.** Both bodies already have one (`head_pitch`, −20…+49°, positive = down). No fixed pitched mount is needed.
2. **Wide stereo lenses (about 120° H × 90° V) on every cortex.** With a 30° down centreline they cover from 15° above the horizon (people's faces) down to 75° (hands at the table).
3. **Default poses:**
   - On a desk: `head_pitch` 0. The face rake already looks slightly up at a seated person.
   - Humanoid standing: `head_pitch` ≈ 35°, which is a camera centreline about 27° down. It sees the work and still sees faces.
   - Close manipulation: up to 49°.

Renders: [`renders/full-head/`](renders/full-head/)
- both heads side by side
- each body at `head_pitch` 0 and 35
