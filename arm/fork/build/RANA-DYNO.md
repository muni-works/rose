# rana dyno test plan

Planning only. No build, CAD or purchases implied. RobStride first; rana is optional for Desk.

**Gate** (master plan, sec:rana / tab:arc): rana >= 1.2 x RS04 in *continuous stall torque per kg or per dollar*, on one dyno at 25 C, or rana is dropped.

**Order:** the same rig measures an RS04 first (baseline; the lymb needs these numbers anyway: shoulder hold, current limits for ISO/TS 15066, thermal derate), then rana-100 unchanged. Nothing on the rig changes between DUTs except the adapter flange.

## What the gate needs, before any test

| | RS04 (datasheet) | rana-100 (predicted, upper bound) |
|---|---|---|
| Mass | 1.42 kg | <= 0.75 kg target (weigh it) |
| Price | ~$170 (CNY 1199) | $1,086 est. + $150-300 teeth |
| Continuous | 35 N·m (220x200 sink) / 40 N·m (345x345) | unknown; 36 N·m *peak*, Kt 0.166 N·m/A |
| Peak | 120 N·m | 36 N·m |

- Per dollar is out of reach for the prototype: at RS04 = 35 N·m / $170 = 0.21 N·m/$, rana would need ~270 N·m continuous. **Per kg is the only live path.**
- Per kg: if RS04 measures ~35 N·m / 1.42 kg = 24.6 N·m/kg, rana at 0.75 kg needs >= **22 N·m continuous** (61% of its predicted peak). With a solid 1018 stator (eddy loss) that is unlikely but not ruled out: the dyno decides.
- Both DUTs are bolted to the **same** 220x200x10 mm 6061 heat-sink plate, vertical, still air. Thermal resistance is part of the result, so the mounting is fixed, not tuned per DUT.

## Rig

Inline, one bed: `DUT -> flex coupling -> reaction-torque cell -> flex coupling -> absorber`, on a 20 mm aluminum or steel baseplate with pillow-block alignment.

**Absorber options, cheapest first**
1. **Locked shaft (stall only), $0-40.** A steel arm pinned to the bed through the torque cell. Covers Kt, continuous stall and thermal time constant. Enough to decide the gate on its own.
2. **Second RS04 as load motor, ~$170.** Regenerative absorber in torque or velocity mode over CAN; covers torque-speed, efficiency, backdrive, cogging (absorber drives DUT slowly, DUT unpowered). Needs a 48 V supply that can sink regen: add a brake resistor/dump module (RobStride discharge module or 10 Ohm 200 W resistor + chopper). Limit: 120 N·m peak / 200 rpm, fine for both DUTs. The absorber RS04 later goes into a lymb, so its cost is mostly not sunk.
3. **Hysteresis/particle brake + reaction arm, ~$300-600 used.** Only if option 2's control loop interacts badly with the DUT loop. Not planned.

**Torque sensing (pick one)**
- *Lever + load cell (cheap):* the absorber body floats on a bearing, its 200 mm arm presses an S-beam load cell (50 kg / 500 N). 500 N x 0.2 m = 100 N·m full scale. Calibrate with dead weights on the arm. ~$60.
- *Inline reaction/rotary torque cell (better):* 100-200 N·m, +/-0.1-0.2% FS, mV/V output, ~$250-450 (generic import). Required for cogging and efficiency resolution; the lever is fine for stall.
- Plan: start with the lever (stall gate), add the inline cell before efficiency/cogging.

**Other sensors**
- Position: DUT's own encoder (RS04 internal, rana AS5047P), plus an independent output encoder on the coupling (AS5600/AS5047 or 2500 PPR optical) for backlash. ~$30.
- Temperature: 4 x K-type thermocouples (case near stator, case far side, heat-sink plate, ambient 30 cm away) + the DUT's internal winding/driver temperature. For rana, an NTC or K-type potted into a winding end-turn at build time (note for the rana build, not a change now). MAX31856 boards. ~$60.
- Electrical: DC bus V and I (INA228 shunt or a 2-channel power meter), plus phase current from the drive's telemetry. One clamp current probe for spot-checking drive telemetry. ~$80.
- Ambient: enclosure (or just a closet) held at 25 +/- 2 C; log it. Tests start only after the DUT is within 1 C of ambient.
- DAQ: one MCU (ESP32/RP2040) with HX711/ADS1220 for the load cell / torque cell, thermocouple boards, INA228; streams to a laptop that also talks CAN (RS04) or USB (ODrive S1 for rana). Common timestamp on the laptop. 1 kHz for torque/current, 1 Hz for temperature.

## Tests (run on RS04, then rana, same scripts)

| # | Test | Method | Output |
|---|---|---|---|
| T0 | Calibration | Dead weights on the lever / torque cell, 5 points up and down; zero drift over 1 h | gain, offset, hysteresis |
| T1 | Kt | Locked output; step Iq 0 -> rated in 10 steps, 2 s each, cold, both directions | Kt (N·m/A at output), linearity, saturation knee |
| T2 | **Continuous stall (gate)** | Locked output at 25 C; hold constant Iq until winding reaches its limit (RS04: driver/winding limit per manual; rana: 120 C winding, class-H wire allows more, 120 is the safe cap) or steady state (dT < 1 C / 10 min). Bisect Iq until the steady-state winding temperature is the limit. Also rotate the locked angle 3 positions to average per-phase heating | **T_cont (N·m)**, T_cont/kg, T_cont/$ |
| T3 | Thermal time constant | From T2 heat-up and a cool-down at zero current; fit 1st (or 2-node) RC | tau_heat, tau_cool, R_th |
| T4 | Torque-speed | Absorber in velocity mode, DUT in torque mode, grid 0-max rpm x 0-peak torque, 2 s dwell per point, max duty limited by temperature | T-N envelope at 48 V, peak and 30 s points |
| T5 | Efficiency | Same grid as T4: P_mech = T x w, P_elec = V_bus x I_bus | efficiency map |
| T6 | Backdrive torque | DUT unpowered (and powered, zero torque), absorber spins it 1-60 rpm | breakaway and running friction vs speed |
| T7 | Cogging / ripple | Unpowered: absorber at 1 rpm, 2 revs. Powered: constant Iq at 1 rpm | cogging pk-pk, ripple %, harmonic spectrum |
| T8 | Backlash | Absorber locked, DUT +/- 5% rated torque, output encoder vs motor encoder | backlash (arcmin), torsional stiffness |

T1-T3 decide the gate; T4-T8 are characterization for the lymb models and for rana's next revision. Each test is run 3 times; report the mean and spread. Abort if winding temperature exceeds the limit + 10 C, or case > 80 C.

## Pass / fail

```
G_kg = (T_cont_rana / m_rana) / (T_cont_RS04 / m_RS04)
G_$  = (T_cont_rana / price_rana) / (T_cont_RS04 / price_RS04)
pass if max(G_kg, G_$) >= 1.2   (both T_cont from T2, same rig, same plate, 25 C)
```
- Masses weighed on one scale, with the same fastener set excluded. Prices: RS04 invoice; rana BOM at the quantity stated (prototype $1,086 + teeth, and separately a costed volume estimate, labeled as an estimate).
- If rana fails with the solid 1018 stator, the report states whether the loss budget (eddy vs copper, from T5 at speed and T2 at stall) says a laminated/SMC stator could close the gap. That is an input to the drop decision, not a waiver of it.
- RS04 result is also checked against its datasheet (35 N·m on the 220x200 sink); a > 10% miss means the rig is suspect before the gate is judged.

## Data format

One CSV per run, `dyno/<dut>_<serial>/<test>_<YYYYMMDD-HHMM>_r<n>.csv`, plus a `meta.json` (DUT, serial, mass_kg, price_usd, firmware, sink plate, calibration file, operator, notes).

```
t_s, dut, test, run, cmd_mode, cmd_iq_a, cmd_vel_rps,
torque_nm, torque_src, speed_rps, pos_out_rad, pos_motor_rad,
iq_a, id_a, i_bus_a, v_bus_v, p_elec_w, p_mech_w,
t_winding_c, t_case_near_c, t_case_far_c, t_sink_c, t_ambient_c, t_driver_c,
fault_flags
```
SI units, one row per sample, 1 kHz rows with temperature carried forward. A summary `results.csv` per DUT: `test, run, metric, value, unit, notes`.

## Parts list (estimates, not quotes)

| Item | Est. |
|---|---|
| Baseplate, pillow blocks, 2 flex couplings, DUT adapter flanges (RS04 + rana) | $150 |
| Heat-sink plate 220x200x10 6061 (one, shared) | $25 |
| Locked-shaft arm + pin | $30 |
| Absorber RS04 (reusable in a lymb) | $170 |
| 48 V supply (>= 20 A) + regen dump resistor/chopper | $150 |
| Lever + S-beam 50 kg load cell + HX711/ADS1220 | $60 |
| Inline torque cell 100-200 N·m + amplifier | $350 |
| Output encoder | $30 |
| 4-6 K-type thermocouples + MAX31856 boards | $60 |
| INA228 shunt + clamp probe | $80 |
| MCU, CAN-USB adapter (RobStride or generic), wiring, e-stop | $80 |
| Dead-weight calibration set (to 20 kg) | $60 |
| Guard / enclosure for the rotating shaft | $50 |
| **Phase 1: stall gate only** (lever, locked shaft, no absorber RS04, no torque cell) | **~$750** |
| **Full rig** | **~$1,300** |

rana's own ODrive S1 bench drive is already in the rana BOM and not counted here. The RS04 DUT is a lymb joint, not counted.
