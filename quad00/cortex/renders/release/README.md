# Cortex release renders

All images are **renders of the design. No cortex hardware has been built.**

Caption for every image: **"rendered in vcad"** (`vcad render --photoreal`, vcad main after #958, headless GPU or CPU path tracer).

| File | What | Size | Scene |
|---|---|---|---|
| cortex-hero-16x9.png | Cortex in the Mind colorway on a desk | 1920 × 1080 | `cortex/hero_desk.loon` |
| cortex-hero-square.png | The same shot, square | 1440 × 1440 | `cortex/hero_desk.loon` |
| cortex-lineup.png | All six colorways (Mind, Make, Body, World, Power, Silicon) | 2400 × 850 | `cortex/lineup.loon` |
| mini-cortex-docked.png | Mini humanoid with the cortex docked | 1600 × 1600 | `vcad/mini/mini.loon` |

Settings: exposure 0.55, 128–384 samples, gradient environment. The lineup is cropped to remove a studio softbox from the top edge.

**Note:** the glass face has two flat overlay plates (`cortex-glass-face`). They work around vcad-render smoothing the shading across a flat loft cap; the faint corner streaks that remain come from the same issue.
