# rose electronics: the brain, the spine, the limbs - design v0 (2026-10-15)

Three board designs, identical across mini, pro and max. We design all of them; vcad routes all of
them, DDR included (where it can't yet, we fix the router).

| board | qty per robot | job |
|---|---|---|
| **limb controller** (`arm/controller`, done) | 5: 2 arms, 2 legs, head | the joints of one limb, its local reflexes, its sensors |
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

1. **phyz on an A76** (a Raspberry Pi 5 has the same cores): policy steps/s per core at S1 settings.
   This picks RK3588S2 or RK3576.
2. **S0 MLP int8 on an H7** (a Nucleo-H723, or the limb controller): µs per tick at 710→512→256→128→20.
   If it's > 500 µs, use an H7 with more flash and RAM, or two cores.
3. **CAN-FD at 1 kHz × 5 limbs:** bus load at 5 Mbit/s data phase with 22 joints of state and command.

## Open

- Limb-current numbers for max (rana-190) to size the hybrid connector and the spine's copper.
- Board outline and mounting in each torso (the limb controller's lesson: envelope first).
- RK3588S2 vs RK3576 pin-out and DDR layout reference (Rockchip reference + Radxa schematics).
