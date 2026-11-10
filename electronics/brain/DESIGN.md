# rose electronics: the brain, the spine, the limbs - design v0 (2026-10-15)

Three board designs, identical across mini, pro and max. We design all of them; vcad routes all of
them, DDR included (where it can't yet, we fix the router).

| board | qty per robot | job |
|---|---|---|
| **limb controller** (`electronics/limb-controller`, done) | 5: 2 arms, 2 legs, head | the joints of one limb, its local reflexes, its sensors |
| **power spine** | 1 | packs A/B in, e-stop, 48 V out to every limb, standby power |
| **brain** | 1 | H7 realtime core (S0) + Rockchip Linux (S1) + NPU (perception) + the CAN-FD hub |

Sized backward from the k1 stack (`ipse/docs/k1-stack.md`): perception is ~50 GOPS (an NPU covers it),
S0 is a ~530k-weight MLP (an MCU covers it in int8), and **S1's phyz rollouts are the biggest load**
(~1,250 policy steps/s, an estimate of 6-12 A76 cores). Voice (11B) and S2 stay off the robot.

## What makes it awesome

Each item fixes something k1-stack.md says we can't do on the Booster K1.

1. **1 kHz S0, deterministic.** On the K1 the state arrives at 493 Hz and the policy jitters
   ±4-5 ms on a Linux sleep. Here S0 runs on the brain's H7 from a hardware timer, with no OS. The limb
   controllers stream joint state over CAN-FD at 1 kHz. That is Helix 02's S0 rate, which the doc calls
   "not available here". Target: S0 tick jitter < 10 µs.
2. **One clock for the whole body.** The hub broadcasts a sync frame. Every limb controller timestamps
   its encoder reads, IMU samples and CAN frames against it (FDCAN hardware timestamps). The camera
   triggers are slaved to the same clock. Every bus field gets a true sample time, so phyz efference
   compares like with like, with no latency guesswork. Target: < 5 µs skew across the body.
3. **Reflexes at the joint.** Each limb controller runs its own guard envelope: drift, torque, thermal
   and capture-margin hints pushed down from S0. It damps in microseconds if the uplink goes quiet.
   The K1's "state stall 300 ms → damp" becomes 2 ms.
4. **A black box that never loses a trace.** The 2026-10-09 500 Hz traces were lost on the robot. The
   brain writes every bus frame and every command to flash in trace-v3 format. It is a ring buffer,
   held up by a supercap through power loss, and cut from the incident window on any fault, fall or
   e-stop. Hardware minutes are the data advantage, so we keep all of them.
5. **The simulator inside the robot.** phyz runs on the brain: one-step S0 efference at 1 kHz on a CPU
   core, S1 rollouts on the remaining cores, and later on the Mali GPU through tang-gpu (a tang
   project). Surprise channels at every seam come from that.
6. **Hard e-stop that software can't override.** A dedicated wire runs through every limb harness.
   Opening it cuts gate drive on every motor and fires every brake comparator. The firmware can add
   braking and can never inhibit it (per atlas's pack spec). The brain stays powered, so the robot can
   report what happened.
7. **Hot-swap packs with zero downtime.** Two packs, ideal-diode switching between them. The spine reads
   pack-present signals and the BMS over CAN; with one pack, firmware drops the arm current limit to 50 %.
8. **One plug per limb.** A locking hybrid connector carries 48 V, CAN-FD, e-stop and sync. It can't be
   miswired, and swapping a limb takes a minute.
9. **Looks the part.** Black ENIG boards with muni orange, matching the limb controller. The brain sits
   in the torso, where a window can show it, with a light pipe for status (boot, armed, e-stop, recording).

## Brain board

| block | part (LCSC price, 2026-10-14) | why |
|---|---|---|
| Linux SoC | **RK3588S2** ($57) or RK3576 ($23) - picked by the phyz benchmark | S1 + estimators + mapping + network; 6 TOPS NPU for perception |
| DRAM | 2× LPDDR4X, 2-4 GB total (~$44-82) | S1 + map + OS fit in 2 GB; memory is the expensive part, so we buy only what we need |
| storage | eMMC + M.2 NVMe | OS on eMMC, black box and maps on NVMe |
| realtime core | STM32H7 + 8-64 MB HyperRAM | S0 int8 at 1 kHz, clock master, 6 CAN-FD buses (3 FDCAN + 3× MCP251863) |
| accelerator | optional Hailo-8 M.2 ($169) | only if NPU perception falls short |
| links | gigabit Ethernet to a dev machine, WiFi to the offboard S2 and voice | |
| cameras | 2× MIPI CSI, triggered by the body clock | head stereo pair |

The H7 and the SoC talk over SPI or USB at 1 kHz: S1 references down, the bus state up. On a TTL
miss, S0 tracks a stand reference (the k1-stack contract), so if Linux crashes the robot stands and
doesn't fall.

## Before layout: benchmarks

Code: `ipse` branch `claude/brain-bench`, `crates/ipse-sim/examples/brain_bench.rs` (phyz, S0 on a host);
`brain/bench/` here (S0 kernel for the H7, CAN-FD load).

| # | question | how | result so far |
|---|---|---|---|
| 1 | phyz policy steps/s per core (picks RK3588S2 or RK3576) | `brain_bench phyz` on a Pi 5 (A76) | M-series Mac: **580-790 steps/s on one thread**, 230-330 per thread with every core busy (noisy: the Mac was loaded). S1's ~1,250/s is about 2 Mac cores. If an A76 is about half that, ~4 cores: RK3588S2 fits; RK3576's A72s are marginal. **Run on a Pi 5.** |
| 2 | S0 int8 µs per tick on an H7 | `brain/bench/s0_int8.c` (`gen_weights.py` makes the 518 KB weight header) in a Nucleo-H723 project | host: int8 10.6 µs, int8 agrees with f32 to 0.002. H7 number pending. Budget at 1 kHz: < 500 µs |
| 3 | CAN-FD load at 1 kHz | `python3 brain/bench/canfd_load.py` | 1 Mbit/s arbitration, 5 Mbit/s data: arm 29 %, leg 36 %, head 23 %. At 500 k / 2 M: leg 85 % (too high). **Run the data phase at 5 Mbit/s** (MCP2562FD is rated to 8) |

## Open

- Limb-current numbers for max (rana-190) to size the hybrid connector and the spine's copper.
- Board outline and mounting in each torso (the limb controller's lesson: envelope first).
- RK3588S2 vs RK3576 pin-out and DDR layout reference (Rockchip reference + Radxa schematics).

## v0 as designed (2026-10-15)

**Flow** (all scripts here): `lib/gen_lib.py` (symbols and footprints from pin tables) ->
`gen_brain.py` (the circuit as data -> `brain.loon`) -> `build.sh` (vcad schematic, netcheck, ERC)
-> `place.py` -> `fanout.py` -> `route.sh` (vcad HDI route, KiCad DRC). Every pin is looked up by
name in a datasheet table or a KiCad symbol; nothing is typed per pin.

**References** (`reference/`): Radxa ROCK 5C v1.1 schematic, RK806-1, RK860, Rayson LPDDR4X
(JEDEC 200-ball map), Zetta eMMC. The ball list also draws on the RK3588S datasheet Rev 1.6,
which is not kept in this repository.

| block | choice | why |
|---|---|---|
| SoC | RK3588S2, FCCSP1253 0.4 mm | A76 cores for phyz (the S1 benchmark) |
| power | RK806-1 + 3x RK860 (BIG0 -2, BIG1 -3, NPU -2) + SY8089 + SGM2576 | ROCK 5C's tree, values copied |
| DRAM | 2x Winbond W66DP2RQQAGJ, 8 Gb x32 single-rank = 2 GB | in stock at LCSC (659 + 50k); S1 + map + OS fit in 2 GB |
| eMMC | Zetta ZDEMMC32GA, 32 GB | LCSC part |
| SoC input | 12 V from the spine -> LMR33640 -> 5 V VCC_SYSIN (ROCK 5C runs on USB-C 5 V) | |
| realtime | STM32H723VG on its own held-up 12 V -> 5 V -> 3.3 V domain | the H7 outlives the SoC on a power cut |
| CAN-FD | FDCAN1-3 + 3x MCP251863 = 6 buses (5 limbs + BMS) | limb controller's parts |
| black box | W25Q128 16 MB on the H7 | |
| cameras | CSI0 4-lane + DPHY0 RX 4-lane, Pi-style 22-pin FPC, triggers from the H7 | |
| dev link | USB-C OTG0 (maskrom flashing, gadget Ethernet) | no PHY on v0 |
| deferred to v1 | M.2 (no socket footprint yet; PCIE20_0 is free), USB3, HDMI | |

**Layout**: 100 x 76 mm, 8-layer HDI: F sig / In1 sig (fanout) / In2 GND / In3 sig (DDR) /
In4 power regions / In5 GND / In6 sig / B sig + caps. Every used ball of the SoC, the DRAMs and the
eMMC has a laser via in its pad (0.25/0.10, VIPPO): signals F-In1 (microvia), GND F-In2, power
stacked F-In4 into an In4 region per rail. The SoC's and DRAMs' 0201 decoupling is on the back,
under the balls. vcad gained back-side placement, pinned vias, microvias and via-in-pad for this
(ecto/vcad#927).

**ERC**: 5 expected errors (MCP251863's exposed pad is "NC" in KiCad's symbol but is VSS; three
MCP251863 SDO outputs share one SPI bus).

**Open before ordering**: DDR length matching per byte lane (vcad `length_match`), impedance
(40 ohm single / 80 ohm diff for LPDDR4X on this stackup), the fab's HDI rules, and a bring-up
review of every ROCK 5C value copied here.
