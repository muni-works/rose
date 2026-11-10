# electronics

| board | what it is |
|---|---|
| [`brain/`](brain) | The compute board: RK3588S2 + STM32H7, split into a compute module and a carrier. Schematic and placement exist; routing and the pre-order checks in its [`DESIGN.md`](brain/DESIGN.md) are open. That file also has the three-board electronics plan. |
| [`limb-controller/`](limb-controller) | One per limb (two arms, two legs, head): that limb's joints, local reflexes and sensors. Schematic written in loon, exported to KiCad, routed ([`DESIGN.md`](limb-controller/DESIGN.md)). |
| [`brand/`](brand) | The Muni logo as SVG and KiCad footprints, for board art. |

Both boards build from their own folder:

```bash
(cd limb-controller && ./build.sh)   # loon -> KiCad schematic, netcheck, ERC, PDF
(cd brain && ./build.sh)             # uses limb-controller's shared footprints and scripts
```
