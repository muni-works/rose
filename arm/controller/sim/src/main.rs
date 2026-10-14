//! Arm controller power path, simulated on vcad's MNA solver with behavioural chips.
//!
//! Values come from the schematic (arm_controller.loon). Chips:
//!   U1  LM5060 hot-swap  -> chips::HotSwap   (VIN switch to VBUS_SW, EN / UVLO / OVP pins)
//!   U2  74LVC1G11 AND    -> chips::AndGate   (e-stop loop AND watchdog OK AND MCU enable)
//!   U5  TLV7031          -> chips::Comparator
//!   U6 + Q2 brake FET    -> chips::IdealFet  (gate driver folded into the switch)
//! Analog: bulk C7 220u (+ESR) and C8 2.2u, brake resistor 10R with D4, the
//! comparator divider/reference/hysteresis, UVLO and OVP dividers, R8/C3 hold.
//!
//! Scenario on a 48 V bench supply that cannot sink current:
//!   0 ms     power up; the R8/C3 hold lets the hot-swap turn on at ~200-300 ms
//!   500 ms   back-driven arm returns I_REGEN for 100 ms
//!   800 ms   e-stop pressed (loop opens)
//!
//! LM5060 pin thresholds are ASSUMED (EN 2.0 V, UVLO 1.6 V, OVP 2.0 V) and the
//! inrush limit is set to 5 A - check against the datasheet before trusting
//! the e-stop delay or the UVLO/OVP points.
use vcad_ecad_sim::circuit::chips::{AndGate, ChipCircuit, Comparator, Drive, HotSwap, IdealFet};
use vcad_ecad_sim::circuit::{Circuit, Device, DiodeModel};

#[derive(Clone, Copy)]
struct Net {
    r16: f64,
    r17: f64,
    r20: f64,
    c3: f64,
    r_brake: f64,
    /// diode across R8: C3 charges at once, discharges through R8 only
    bypass: bool,
}

#[derive(Default)]
struct Out {
    inrush: f64,
    v_on: f64,
    v_off: f64,
    v_peak: f64,
    switches: usize,
    e_brake: f64,
    stop_delay: f64,
    trace: Vec<(f64, f64, f64, f64)>,
}

const DT: f64 = 5e-6;
const T_REGEN: (f64, f64) = (0.500, 0.600);
const T_ESTOP: f64 = 0.800;
const T_END: f64 = 1.000;
const T_ON: f64 = 0.450; // bus is up by now

fn run(n: Net, i_regen: f64, trace: bool) -> Out {
    let mut c = Circuit::new();
    let mut nodes = [0usize; 17];
    for x in nodes.iter_mut() {
        *x = c.node();
    }
    let [psu, psu_d, vin, vbus_sw, vbus, esr, brk, div, refn, v33, cmp, uv, ov, en_and, hs_en, estop, wd_ok] = nodes;
    let mcu_en = c.node();
    // supply: 48 V, can't sink (diode), 50 mOhm leads, C1
    c.add(Device::VSource { p: psu, n: 0, v: 48.0 });
    c.add(Device::Diode { p: psu, n: psu_d, model: DiodeModel::silicon() });
    c.add(Device::Resistor { p: psu_d, n: vin, r: 0.05 });
    c.add(Device::Capacitor { p: vin, n: 0, c: 2.2e-6 });
    // UVLO R3 220k / R4 10k, OVP R5 330k / R6 10k (on VIN)
    c.add(Device::Resistor { p: vin, n: uv, r: 220e3 });
    c.add(Device::Resistor { p: uv, n: 0, r: 10e3 });
    c.add(Device::Resistor { p: vin, n: ov, r: 330e3 });
    c.add(Device::Resistor { p: ov, n: 0, r: 10e3 });
    // shunt R14, bulk C7 with ESR, C8, idle motor load ~0.3 A
    c.add(Device::Resistor { p: vbus_sw, n: vbus, r: 0.5e-3 });
    c.add(Device::Resistor { p: vbus, n: esr, r: 0.08 });
    let c7 = c.add(Device::Capacitor { p: esr, n: 0, c: 220e-6 });
    c.add(Device::Capacitor { p: vbus, n: 0, c: 2.2e-6 });
    c.add(Device::Resistor { p: vbus, n: 0, r: 160.0 });
    let regen = c.add(Device::ISource { p: vbus, n: 0, i: 0.0 }); // pushes into VBUS (into p)
    // brake: J3 10R from VBUS, Q2 to ground (chip), D4 freewheel
    c.add(Device::Resistor { p: vbus, n: brk, r: n.r_brake });
    c.add(Device::Diode { p: brk, n: vbus, model: DiodeModel::silicon() });
    // comparator network R16/R17, R20 hysteresis, R18/R19 reference
    c.add(Device::Resistor { p: vbus, n: div, r: n.r16 });
    c.add(Device::Resistor { p: div, n: 0, r: n.r17 });
    c.add(Device::Resistor { p: cmp, n: div, r: n.r20 });
    c.add(Device::VSource { p: v33, n: 0, v: 3.3 });
    c.add(Device::Resistor { p: v33, n: refn, r: 10e3 });
    c.add(Device::Resistor { p: refn, n: 0, r: 10e3 });
    // enable chain inputs and the R8 / C3 hold
    let estop_src = c.add(Device::VSource { p: estop, n: 0, v: 3.3 });
    c.add(Device::VSource { p: wd_ok, n: 0, v: 3.3 });
    c.add(Device::VSource { p: mcu_en, n: 0, v: 3.3 });
    c.add(Device::Resistor { p: en_and, n: hs_en, r: 1e6 });
    c.add(Device::Capacitor { p: hs_en, n: 0, c: n.c3 });
    if n.bypass {
        c.add(Device::Diode { p: en_and, n: hs_en, model: DiodeModel::silicon() });
    }

    let mut cc = ChipCircuit::new(c);
    let hs = cc.add_chip(Box::new(HotSwap::new(vbus_sw, 2.0, 1.6, 2.0, 5.0, 1.5e-3)), &[vin, vbus_sw, hs_en, uv, ov]);
    cc.add_chip(Box::new(AndGate { inputs: 3 }), &[estop, wd_ok, mcu_en, v33, en_and]);
    let cmp_chip = cc.add_chip(Box::new(Comparator::tlv7031()), &[div, refn, v33, cmp]);
    cc.add_chip(Box::new(IdealFet::new(1.5, 1.5e-3, 0)), &[cmp, brk, 0]);
    let mut env = cc.into_env(DT);

    let mut o = Out { v_on: f64::NAN, v_off: f64::NAN, stop_delay: f64::NAN, ..Default::default() };
    let mut high = false;
    let steps = (T_END / DT) as usize;
    for k in 0..steps {
        let t = k as f64 * DT;
        env.env.set_value(regen, if (T_REGEN.0..T_REGEN.1).contains(&t) { i_regen } else { 0.0 });
        env.env.set_value(estop_src, if t >= T_ESTOP { 0.0 } else { 3.3 });
        let s = env.step();
        let vb = s.node_voltages[vbus];
        if t < T_ON {
            o.inrush = o.inrush.max(s.device_currents[c7].abs());
        }
        if t > T_ON && t < T_ESTOP {
            o.v_peak = o.v_peak.max(vb);
        }
        let h = matches!(env.drives(cmp_chip)[3], Drive::Volts(x) if x > 1.0);
        if h != high {
            high = h;
            o.switches += 1;
            if h && o.v_on.is_nan() {
                o.v_on = vb;
            }
            if !h && o.v_off.is_nan() {
                o.v_off = vb;
            }
        }
        let ib = (vb - s.node_voltages[brk]) / n.r_brake;
        o.e_brake += ib.max(0.0) * ib.max(0.0) * 10.0 * DT;
        if t >= T_ESTOP && o.stop_delay.is_nan() {
            if let Drive::Ohms(r) = env.drives(hs)[0] {
                if r > 1e9 {
                    o.stop_delay = t - T_ESTOP;
                }
            }
        }
        if trace && k % 100 == 0 {
            o.trace.push((t, vb, s.node_voltages[hs_en], if h { 1.0 } else { 0.0 }));
        }
    }
    o
}

fn thresholds(n: Net) -> (f64, f64) {
    let par = |a: f64, b: f64| a * b / (a + b);
    let p = par(n.r17, n.r20);
    let a = p / (n.r16 + p);
    let b = par(n.r16, n.r17) / (n.r20 + par(n.r16, n.r17));
    (1.65 / a, (1.65 - 3.3 * b) / a)
}

fn e96() -> Vec<f64> {
    let base = [
        1.00, 1.02, 1.05, 1.07, 1.10, 1.13, 1.15, 1.18, 1.21, 1.24, 1.27, 1.30, 1.33, 1.37, 1.40, 1.43, 1.47, 1.50, 1.54, 1.58, 1.62, 1.65, 1.69, 1.74,
        1.78, 1.82, 1.87, 1.91, 1.96, 2.00, 2.05, 2.10, 2.15, 2.21, 2.26, 2.32, 2.37, 2.43, 2.49, 2.55, 2.61, 2.67, 2.74, 2.80, 2.87, 2.94, 3.01, 3.09,
        3.16, 3.24, 3.32, 3.40, 3.48, 3.57, 3.65, 3.74, 3.83, 3.92, 4.02, 4.12, 4.22, 4.32, 4.42, 4.53, 4.64, 4.75, 4.87, 4.99, 5.11, 5.23, 5.36, 5.49,
        5.62, 5.76, 5.90, 6.04, 6.19, 6.34, 6.49, 6.65, 6.81, 6.98, 7.15, 7.32, 7.50, 7.68, 7.87, 8.06, 8.25, 8.45, 8.66, 8.87, 9.09, 9.31, 9.53, 9.76,
    ];
    [1e3, 1e4, 1e5, 1e6].iter().flat_map(|d| base.iter().map(move |b| b * d)).collect()
}

fn main() {
    let schematic = Net { r16: 316e3, r17: 10e3, r20: 1e6, c3: 220e-9, r_brake: 10.0, bypass: false };
    let vals = e96();
    let mut best = (f64::MAX, schematic);
    for &r17 in vals.iter().filter(|v| (4.99e3..=20e3).contains(*v)) {
        for &r16 in vals.iter().filter(|v| (100e3..=1e6).contains(*v)) {
            for &r20 in vals.iter().filter(|v| (10e3..=10e6).contains(*v)) {
                let n = Net { r16, r17, r20, c3: 390e-9, r_brake: 3.3, bypass: true };
                let (a, b) = thresholds(n);
                let e = (a - 53.5).abs() + (b - 50.5).abs();
                if e < best.0 {
                    best = (e, n);
                }
            }
        }
    }
    let fixed = best.1;
    for (name, n) in [("schematic", schematic), ("fixed", fixed)] {
        let (a, b) = thresholds(n);
        println!(
            "{name:>9}: R16 {:.0}k  R17 {:.2}k  R20 {:.0}k  C3 {:.0}n  brake {:.1} ohm  R8 bypass {}   closed form on {a:.2} V / off {b:.2} V",
            n.r16 / 1e3,
            n.r17 / 1e3,
            n.r20 / 1e3,
            n.c3 * 1e9,
            n.r_brake,
            n.bypass
        );
    }
    println!(
        "\n{:>22} {:>6} {:>8} {:>8} {:>8} {:>8} {:>6} {:>8} {:>10}",
        "case", "regen", "inrush", "on at", "off at", "peak", "sw", "E brake", "e-stop off"
    );
    let mut csv = String::from("t_s,vbus,hs_en,brake\n");
    let mut cases: Vec<(String, Net, f64)> = Vec::new();
    for i in [5.0, 20.0] {
        cases.push(("as drawn".into(), schematic, i));
    }
    for rb in [10.0, 4.7, 3.3] {
        for i in [5.0, 10.0, 15.0] {
            cases.push((format!("fixed divider, {rb} ohm"), Net { r_brake: rb, ..fixed }, i));
        }
    }
    for c3 in [220e-9, 390e-9, 470e-9] {
        cases.push((format!("no bypass, C3 {:.0} nF", c3 * 1e9), Net { c3, bypass: false, ..fixed }, 5.0));
        cases.push((format!("bypass, C3 {:.0} nF", c3 * 1e9), Net { c3, ..fixed }, 5.0));
    }
    for (name, n, i) in &cases {
        let keep = name.contains("3.3 ohm") && *i == 10.0;
        let o = run(*n, *i, keep);
        println!(
            "{name:>22} {:>5.0}A {:>7.2}A {:>7.2}V {:>7.2}V {:>7.2}V {:>6} {:>7.2}J {:>7.0} ms",
            i, o.inrush, o.v_on, o.v_off, o.v_peak, o.switches, o.e_brake, o.stop_delay * 1e3
        );
        if keep {
            for (t, a, b, c) in &o.trace {
                csv.push_str(&format!("{t:.4},{a:.3},{b:.3},{c}\n"));
            }
        }
    }
    std::fs::write("power_path.csv", csv).unwrap();
}
