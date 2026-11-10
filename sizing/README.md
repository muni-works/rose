# sizing

- [`family.py`](family.py): the size ladder for mini / full / mega: proportions,
  mass, static knee torque and a jump ceiling. `python3 family.py`. The `--pack`
  check does not currently run.
- [`MEGA-SIZING.md`](MEGA-SIZING.md) and [`mega_sizing.py`](mega_sizing.py): actuator
  sizing for mega at 1.95 m (2026-10-21). The rana actuators it assumes are not built.
- [`rose_full_layout.py`](rose_full_layout.py): the 1800 mm full layout's actuator
  envelopes, from vendor drawings. The arm (`arm/rose_arm_layout.py`) and
  `family.py --pack` import it.
