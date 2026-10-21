# Reference geometry

**Unitree G1** (`g1_23dof`) and **Unitree H2**, composed from their URDFs at zero
pose and rendered through our own `vcad-render`, so they sit in the same drafting
style as rose and can be compared directly. Meshes are NOT in this repo — they are
Unitree's (BSD-3-Clause, `unitreerobotics/unitree_ros`, `robots/g1_description`
and `robots/h2_description`), sparse-checked-out to
`~/Developer/pond_assets/reference/unitree_g1/`. The `.loon` files here reference
them by absolute path.

```bash
# pull the meshes (once)
cd ~/Developer/pond_assets/reference/unitree_g1
git sparse-checkout add 'robots/h2_description/*'

# regenerate a loon from a URDF
G=~/Developer/pond_assets/reference/unitree_g1/robots
python3 urdf2loon.py $G/h2_description/H2.urdf h2_full.loon --comment "Unitree H2 ..."

# render, SAME camera for every robot (zsh: do not put the flags in a $VAR - it
# does not word-split, and vcad-render gets one argument)
R=~/Developer/vcad/target/release/vcad-render
$R h2_full.loon --azimuth 30 --elevation 15 --size 1400 --fill 0.92 --auto-aspect -o h2-full-iso.png
$R h2_full.loon --azimuth 90 --elevation 0  --size 1400 --fill 0.92 --auto-aspect -o h2-full-side.png
$R h2_full.loon --azimuth 0  --elevation 0  --size 1400 --fill 0.92 --auto-aspect -o h2-full-front.png

# the side-by-side at true relative scale (see lineup.py; heights are sole-to-crown
# from each model's own geometry: G1 1323, rose full 1173, H2 1833)
python3 lineup.py ../render/lineup-front.png g1-full-front.png:1323:G1 \
    ../render/rose-full-front.png:1173:"rose full" h2-full-front.png:1833:H2
```

Traps, recorded because each cost time:

- These STLs are in **metres**, and vcad's `import-mesh` already brings them in
  at the right scale. Wrapping them in `import-mesh-scaled 1000 1000 1000` makes
  every link 355 *metres* tall. Use plain `import-mesh`.
- `--view side` is not a reliable profile: on the H2 loon it produced a frontal
  view. Use explicit `--azimuth 90 --elevation 0` (profile, facing +x = right)
  and `--azimuth 0 --elevation 0` (front).
- `--fill 0.92 --auto-aspect` fills the image with the *projected* bounding box.
  At elevation 0 the drawn height equals the z extent exactly, so panels can be
  put to a common scale; at elevation 15 the footprint tilts in and the drawn
  height is a few percent more than the z extent. `lineup.py` therefore uses
  elevation-0 renders.

What to look at, and why these are here:

- `g1-left-leg-side.png` — the whole "wrap, don't tab" argument in one image. Hip
  pitch disc at top; hip roll link is a curved shell sweeping down and *back*;
  thigh comes forward again (the 10° cant in the URDF, the "S" through the hip);
  knee disc; shank tapering to a yoke; **two pushrods** driving the ankle from
  actuators hidden inside the shank; low flat foot. One continuous shell, hip to
  sole.
- `h2-full-side.png` — the whole "why ours reads heavy" argument. Thigh widest
  under the hip, tapering 2:1 to a knee that is a pinch; shank swelling into a
  calf 30 % down and tapering to a 52 × 40 ankle; chest widest at two-thirds
  height over a visible waist over a separate pelvis pod; shoulders at the top
  corners of the chest; arms with section. Numbers in `../UNITREE.md`.
- `h2-full-front.png` against `../render/rose-full-front.png` — the width order
  chest > pelvis > thigh-top > knee > ankle, which ours now follows in steps.
- `g1-full-*.png` — the body proportions, hand size, and how little of any
  actuator is actually visible.
