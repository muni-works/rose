//! rose arm in phyz.
//!
//! The robot is `rose_arm.urdf` / `rose_arm_payload.urdf`, emitted by
//! `arm/rose_arm_export.py` from the same layout the CAD comes from. What URDF
//! cannot say (armature, torque-speed limits, the J2 gas spring, payload) is in
//! `rose_arm_params.json`.
//!
//! Every motor is a torque source limited by a linear torque-speed line from the
//! RobStride peak torque to its no-load speed. The controller is PD plus a
//! gravity feed-forward from RNEA, which is what a real arm runs; the clamp is
//! where the actuators say no. The gas spring is passive: it is a generalized
//! force on J2 from the actual anchor geometry, never clamped and never off.
//!
//! Experiments, each written to out/<name>.csv with a line in out/summary.json:
//!   hold    arm straight out, 2.5 kg, 3 s - with and without the spring
//!   drop    power cut at horizontal, no payload - with and without the spring
//!   reach   folded -> straight out -> folded with 2.5 kg, minimum jerk, 1.2 s legs
//!   roll    elbow at 90, swing the upper-arm roll 90 deg with 2.5 kg
//!   cycle   reach repeated: RMS torque per joint against the rated torque

use std::fs;
use std::io::Write;

use phyz_math::{DVec, Vec3};
use phyz_model::{Model, State};
use phyz_rigid::{aba, forward_kinematics, rnea};
use phyz_urdf::{load_file, UrdfOptions};
use serde_json::{json, Value};

const DT: f64 = 2.5e-4;
const JOINTS: [&str; 7] = [
    "J1_yaw", "J2_sho_pitch", "J3_upper_roll", "J4_elbow", "J5_fore_roll", "J6_wrist_pitch", "J7_tool_roll",
];

struct Motor {
    peak: f64,
    rated: f64,
    no_load: f64,
    kp: f64,
    kd: f64,
    short: f64,
}

struct Arm {
    model: Model,
    motors: Vec<Motor>,
    vidx: Vec<usize>,
    tool: usize,
    flange_local: Vec3,
    spring: Option<(f64, f64, f64)>, // r, d, force
}

fn load(payload: bool, spring: bool, p: &Value) -> Arm {
    load_path(if payload { "rose_arm_payload.urdf" } else { "rose_arm.urdf" }, spring, p)
}

fn load_path(path: &str, spring: bool, p: &Value) -> Arm {
    let opts = UrdfOptions { dt: Some(DT), ..Default::default() };
    let r = load_file(path, &opts).expect("urdf");
    for w in &r.warnings {
        eprintln!("urdf: {w}");
    }
    let mut model = r.model;
    let mut motors = vec![];
    let mut vidx = vec![];
    for name in JOINTS {
        let ji = model.joint_index(name).unwrap_or_else(|| panic!("no joint {name}"));
        let j = &p["joints"][name];
        model.joints[ji].armature = j["armature"].as_f64().unwrap();
        vidx.push(model.v_offsets[ji]);
        let peak = j["peak"].as_f64().unwrap();
        // gains scale with the motor: stiff enough to track, soft enough to be a QDD
        let (kp, kd) = if peak > 30.0 { (220.0, 9.0) } else { (45.0, 1.6) };
        motors.push(Motor { peak, rated: j["rated"].as_f64().unwrap(), no_load: j["no_load"].as_f64().unwrap(), kp, kd, short: j["short"].as_f64().unwrap() });
    }
    let tool = model.body_index("tool").unwrap();
    let pl = &p["payload"]["xyz"];
    let flange_local = Vec3::new(pl[0].as_f64().unwrap(), pl[1].as_f64().unwrap(), pl[2].as_f64().unwrap());
    let s = &p["spring"];
    let spring = spring.then(|| (s["r"].as_f64().unwrap(), s["d"].as_f64().unwrap(), s["force"].as_f64().unwrap()));
    Arm { model, motors, vidx, tool, flange_local, spring }
}

impl Arm {
    /// Gas spring torque about J2 (+y). Base anchor d below the pitch axis, arm anchor r along
    /// the upper arm; constant force pushing them apart. tau = -F r d cos(q) / |A - B|.
    fn spring_tau(&self, q2: f64) -> f64 {
        let Some((r, d, f)) = self.spring else { return 0.0 };
        let (s, c) = q2.sin_cos();
        let l = (r * r + d * d - 2.0 * r * d * s).max(1e-6).sqrt();
        -f * r * d * c / l
    }

    fn flange(&self, st: &State) -> Vec3 {
        let (x, _) = forward_kinematics(&self.model, st);
        let t = &x[self.tool];
        t.pos + t.rot.transpose().mul_vec(self.flange_local)
    }

    fn gravity(&self, st: &State) -> DVec {
        let mut s = st.clone();
        s.v = DVec::zeros(self.model.nv);
        rnea(&self.model, &s, &DVec::zeros(self.model.nv))
    }
}

struct Log {
    name: String,
    rows: Vec<Vec<f64>>,
}

impl Log {
    fn new(name: &str) -> Self {
        Self { name: name.into(), rows: vec![] }
    }
    fn write(&self) {
        let mut f = fs::File::create(format!("out/{}.csv", self.name)).unwrap();
        let mut h = vec!["t".to_string()];
        for j in JOINTS {
            h.push(format!("q_{j}"));
        }
        for j in JOINTS {
            h.push(format!("qd_{j}"));
        }
        for j in JOINTS {
            h.push(format!("tau_{j}"));
        }
        h.extend(["sat".into(), "fx".into(), "fy".into(), "fz".into()]);
        writeln!(f, "{}", h.join(",")).unwrap();
        for r in &self.rows {
            writeln!(f, "{}", r.iter().map(|v| format!("{v:.5}")).collect::<Vec<_>>().join(",")).unwrap();
        }
    }
}

/// Run the arm for `t_end` seconds. `target(t)` gives (q, qd) per joint, or None for power off.
fn run(arm: &Arm, q0: &[f64], t_end: f64, name: &str, target: &dyn Fn(f64) -> Option<([f64; 7], [f64; 7])>) -> (Log, Value) {
    let n = arm.model.nv;
    let mut st = arm.model.default_state();
    for (k, &v) in arm.vidx.iter().enumerate() {
        st.q[v] = q0[k];
    }
    let mut log = Log::new(name);
    let mut peak_frac = [0.0f64; 7];
    let mut peak_tau = [0.0f64; 7];
    let mut peak_speed_frac = [0.0f64; 7];
    let mut sq = [0.0f64; 7];
    let mut sat_time = [0.0f64; 7];
    let mut max_err = [0.0f64; 7];
    let mut flange0 = None;
    let mut z_min = f64::INFINITY;
    let mut drift_max = 0.0f64;
    let steps = (t_end / DT).round() as usize;
    let mut t = 0.0;
    for i in 0..=steps {
        let g = arm.gravity(&st);
        let tgt = target(t);
        let mut ctrl = DVec::zeros(n);
        let mut taus = [0.0; 7];
        let mut sat = 0.0;
        for k in 0..7 {
            let v = arm.vidx[k];
            let m = &arm.motors[k];
            let w = st.v[v];
            let tau = match tgt {
                None if name.starts_with("short") => -m.short * w,   // phases shorted: back-EMF brake
                None => 0.0,
                Some((qd, vd)) => {
                    let spring_ff = if k == 1 { arm.spring_tau(st.q[v]) } else { 0.0 };
                    let raw = g[v] - spring_ff + m.kp * (qd[k] - st.q[v]) + m.kd * (vd[k] - w);
                    max_err[k] = max_err[k].max((qd[k] - st.q[v]).abs());
                    // linear torque-speed line: full peak at stall, zero at no-load, regen side unlimited
                    let avail = if raw * w > 0.0 { (m.peak * (1.0 - w.abs() / m.no_load)).max(0.0) } else { m.peak };
                    if raw.abs() > avail {
                        sat_time[k] += DT;
                        sat = 1.0;
                    }
                    raw.clamp(-avail, avail)
                }
            };
            taus[k] = tau;
            ctrl[v] = tau + if k == 1 { arm.spring_tau(st.q[v]) } else { 0.0 };
            peak_tau[k] = peak_tau[k].max(tau.abs());
            peak_frac[k] = peak_frac[k].max(tau.abs() / m.peak);
            peak_speed_frac[k] = peak_speed_frac[k].max(w.abs() / m.no_load);
            sq[k] += tau * tau * DT;
        }
        let f = arm.flange(&st);
        let f0 = *flange0.get_or_insert(f);
        z_min = z_min.min(f.z);
        if tgt.is_some() && t > 0.5 && name.starts_with("hold") {
            drift_max = drift_max.max((f - f0).norm());
        }
        if i % 40 == 0 {
            let mut row = vec![t];
            row.extend(arm.vidx.iter().map(|&v| st.q[v]));
            row.extend(arm.vidx.iter().map(|&v| st.v[v]));
            row.extend(taus);
            row.extend([sat, f.x, f.y, f.z]);
            log.rows.push(row);
        }
        if i == steps {
            break;
        }
        st.ctrl = ctrl;
        let qdd = aba(&arm.model, &st);
        for k in 0..n {
            st.v[k] += DT * qdd[k];
            st.q[k] += DT * st.v[k];
        }
        t += DT;
    }
    let f_end = arm.flange(&st);
    let rms: Vec<f64> = sq.iter().map(|s| (s / t_end).sqrt()).collect();
    let per: Vec<Value> = (0..7)
        .map(|k| {
            json!({
                "joint": JOINTS[k],
                "peak_tau": peak_tau[k],
                "peak_frac": peak_frac[k],
                "rms_tau": rms[k],
                "rms_over_rated": rms[k] / arm.motors[k].rated,
                "speed_frac": peak_speed_frac[k],
                "sat_s": sat_time[k],
                "max_err_deg": max_err[k].to_degrees(),
            })
        })
        .collect();
    let summary = json!({
        "name": name,
        "joints": per,
        "flange_start": [flange0.unwrap().x, flange0.unwrap().y, flange0.unwrap().z],
        "flange_end": [f_end.x, f_end.y, f_end.z],
        "flange_z_min": z_min,
        "hold_drift_mm": drift_max * 1e3,
        "q_end_deg": arm.vidx.iter().map(|&v| st.q[v].to_degrees()).collect::<Vec<_>>(),
    });
    (log, summary)
}

/// minimum-jerk blend from a to b over [t0, t0 + T]
fn mj(a: &[f64; 7], b: &[f64; 7], t0: f64, dur: f64, t: f64) -> ([f64; 7], [f64; 7]) {
    let s = ((t - t0) / dur).clamp(0.0, 1.0);
    let p = 10.0 * s.powi(3) - 15.0 * s.powi(4) + 6.0 * s.powi(5);
    let dp = if (0.0..=1.0).contains(&((t - t0) / dur)) { (30.0 * s * s - 60.0 * s.powi(3) + 30.0 * s.powi(4)) / dur } else { 0.0 };
    let mut q = [0.0; 7];
    let mut v = [0.0; 7];
    for k in 0..7 {
        q[k] = a[k] + (b[k] - a[k]) * p;
        v[k] = (b[k] - a[k]) * dp;
    }
    (q, v)
}

fn deg(v: [f64; 7]) -> [f64; 7] {
    v.map(f64::to_radians)
}

fn main() {
    fs::create_dir_all("out").unwrap();
    let p: Value = serde_json::from_str(&fs::read_to_string("rose_arm_params.json").unwrap()).unwrap();
    let mut all = vec![];
    let zero = [0.0; 7];
    // folded "home": upper arm 60 deg up, elbow folded 120 down, wrist level
    let home = deg([0.0, -60.0, 0.0, 120.0, 0.0, -60.0, 0.0]);
    let out = zero;

    let mut go = |arm: &Arm, q0: &[f64], t: f64, name: &str, f: &dyn Fn(f64) -> Option<([f64; 7], [f64; 7])>| {
        let (log, s) = run(arm, q0, t, name, f);
        log.write();
        println!("{name}: {}", short(&s));
        all.push(s);
    };

    for spring in [true, false] {
        let tag = if spring { "spring" } else { "bare" };
        let arm = load(true, spring, &p);
        go(&arm, &zero, 3.0, &format!("hold_{tag}"), &|_| Some((zero, [0.0; 7])));
        let a0 = load(false, spring, &p);
        go(&a0, &zero, 1.5, &format!("drop_{tag}"), &|_| None);
        go(&arm, &home, 3.0, &format!("reach_{tag}"), &|t| {
            Some(if t < 1.5 { mj(&home, &out, 0.1, 1.2, t) } else { mj(&out, &home, 1.6, 1.2, t) })
        });
        let bent = deg([0.0, 0.0, 0.0, 90.0, 0.0, 0.0, 0.0]);
        let rolled = deg([0.0, 0.0, 90.0, 90.0, 0.0, 0.0, 0.0]);
        go(&arm, &bent, 2.0, &format!("roll_{tag}"), &|t| Some(mj(&bent, &rolled, 0.2, 1.0, t)));
        go(&arm, &home, 9.0, &format!("cycle_{tag}"), &|t| {
            let c = t % 3.0;
            Some(if c < 1.5 { mj(&home, &out, 0.1, 1.2, c) } else { mj(&out, &home, 1.6, 1.2, c) })
        });
    }
    for spring in [true, false] {
        let tag = if spring { "spring" } else { "bare" };
        let a0 = load(false, spring, &p);
        go(&a0, &zero, 1.5, &format!("short_{tag}"), &|_| None);
    }
    // continuous duty: the reach cycle at each payload, spring on
    for m in [0.0, 0.5, 1.0, 1.5, 2.0, 2.5] {
        let arm = load_path(&format!("rose_arm_payload_{m:.1}.urdf"), true, &p);
        go(&arm, &home, 9.0, &format!("duty_{m:.1}"), &|t| {
            let c = t % 3.0;
            Some(if c < 1.5 { mj(&home, &out, 0.1, 1.2, c) } else { mj(&out, &home, 1.6, 1.2, c) })
        });
    }
    fs::write("out/summary.json", serde_json::to_string_pretty(&all).unwrap()).unwrap();
}

fn short(s: &Value) -> String {
    let j = s["joints"].as_array().unwrap();
    let worst = j
        .iter()
        .max_by(|a, b| a["peak_frac"].as_f64().unwrap().partial_cmp(&b["peak_frac"].as_f64().unwrap()).unwrap())
        .unwrap();
    let sat: f64 = j.iter().map(|x| x["sat_s"].as_f64().unwrap()).sum();
    format!(
        "worst {} {:.1} N.m ({:.0}% of peak), sat {:.2} s, flange z {:.0}->{:.0} mm (min {:.0}), drift {:.1} mm",
        worst["joint"].as_str().unwrap(),
        worst["peak_tau"].as_f64().unwrap(),
        100.0 * worst["peak_frac"].as_f64().unwrap(),
        sat,
        1e3 * s["flange_start"][2].as_f64().unwrap(),
        1e3 * s["flange_end"][2].as_f64().unwrap(),
        1e3 * s["flange_z_min"].as_f64().unwrap(),
        s["hold_drift_mm"].as_f64().unwrap()
    )
}
