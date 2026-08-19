# Reference geometry

**Unitree G1** (`g1_23dof`), composed from its URDF at zero pose and rendered
through our own `vcad-render`, so it sits in the same drafting style as rose and
can be compared directly. Meshes are NOT in this repo — they are Unitree's
(BSD-3-Clause, `unitreerobotics/unitree_ros`, `robots/g1_description`), sparse-
checked-out to `~/Developer/pond_assets/reference/unitree_g1/`. The `.loon` files
here reference them by absolute path.

```bash
# regenerate the loon from the URDF (script inline in the commit that added this)
# then:
R=~/Developer/vcad/target/release/vcad-render
$R g1_full.loon --azimuth 30 --elevation 15 --size 1400 --fill 0.92 --auto-aspect -o g1-full-iso.png
```

One trap, recorded because it cost an hour: these STLs are in **metres**, and
vcad's `import-mesh` already brings them in at the right scale. Wrapping them in
`import-mesh-scaled 1000 1000 1000` makes every link 355 *metres* tall, at which
point the mm translations are noise and the whole robot piles up at the origin.
Use plain `import-mesh`.

What to look at, and why these are here:

- `g1-left-leg-side.png` — the whole argument in one image. Hip pitch disc at
  top; hip roll link is a curved shell sweeping down and *back*; thigh comes
  forward again (the 10° cant in the URDF, the "S" through the hip); knee disc;
  shank tapering to a yoke; **two pushrods** driving the ankle from actuators
  hidden inside the shank; low flat foot. One continuous shell, hip to sole.
- `g1-full-*.png` — the body proportions, hand size, and how little of any
  actuator is actually visible.
