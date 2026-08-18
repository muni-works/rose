#!/usr/bin/env python3
"""K1-scale humanoid: quasi-static actuator torque sizing + mass budget.

Three load cases drive the joint torques. Everything here is quasi-static;
a 1.8x dynamic factor is applied when picking the actuator class.

Run:  python3 actuator_sizing.py
"""
import math

g = 9.81
M = 17.7          # gross mass, kg (see mass budget below)

# link geometry, metres (from cad/k1.loon)
L_thigh, L_shin = 0.200, 0.212
com_z, hip_z, knee_z, ank_z = 0.547, 0.468, 0.260, 0.048
DYN = 1.8         # dynamic factor applied to quasi-static peaks


def case_a():
    """Single-leg stance with a 15 deg forward torso lean."""
    lean = math.radians(15)
    out = {}
    for name, z, extra in [("hip pitch", hip_z, 0.03),
                           ("knee", knee_z, 0.03),
                           ("ankle pitch", ank_z, 0.03)]:
        h = com_z - z
        out[name] = M * g * (h * math.sin(lean) + extra)
    return out


def case_b():
    """Two-leg squat to 60 deg knee flexion, half body weight per leg."""
    th = math.radians(60)
    W = M * g / 2
    d_knee = L_thigh * math.sin(th)
    arms = {"hip pitch": d_knee * 0.5,
            "knee": d_knee,
            "ankle pitch": L_shin * math.sin(th / 2) + d_knee}
    return {k: W * d for k, d in arms.items()}, arms


def case_c():
    """Arm holding a 1 kg payload at full extension."""
    reach = 0.180 + 0.170 + 0.104
    m_arm = 1.4
    return {"shoulder pitch": (1.0 * reach + m_arm * reach * 0.45) * g,
            "elbow": (1.0 * (0.170 + 0.104)) * g}, reach


def stance_roll():
    """Single-leg stance: hip roll carries the lateral CoM offset."""
    return M * g * (0.060 + 0.030)


# ── actuator catalogue: 3 SKUs ───────────────────────────────────────────
CLASSES = {"A": dict(torque=12, mass=0.30, size="D45x40"),
           "B": dict(torque=25, mass=0.45, size="D55x50"),
           "C": dict(torque=55, mass=0.75, size="D70x58")}

# joint -> (count, class, quasi-static peak Nm)
JOINTS = [("ankle pitch", 2, "C", 27.6), ("knee", 2, "C", 18.1),
          ("hip pitch", 2, "C", 16.0),   ("hip roll", 2, "C", 16.0),
          ("hip yaw", 2, "B", 6.0),      ("ankle roll", 2, "B", 9.0),
          ("shoulder pitch", 2, "B", 7.3), ("shoulder roll", 2, "B", 7.3),
          ("elbow", 2, "B", 2.7),        ("wrist yaw", 2, "A", 1.2),
          ("neck yaw", 1, "A", 0.5),     ("head pitch", 1, "A", 0.8)]

OTHER_MASS = [("sheet-metal structure (2 mm Al-5052)", 2.60),
              ("battery, 14S 6 Ah Li-ion", 1.45),
              ("compute + IMU + sensors", 0.80),
              ("wiring, connectors, fasteners", 0.95),
              ("covers / foot pads", 0.55)]


def main():
    print("=== Case A: single-leg stance, 15 deg lean ===")
    for k, v in case_a().items():
        print(f"  {k:16s}{v:6.1f} Nm")
    print(f"  {'hip roll':16s}{stance_roll():6.1f} Nm")

    print("\n=== Case B: 60 deg squat, per leg ===")
    tq, arms = case_b()
    for k in tq:
        print(f"  {k:16s}{tq[k]:6.1f} Nm   (moment arm {arms[k]*1000:.0f} mm)")

    print("\n=== Case C: arm, 1 kg payload at full extension ===")
    tq, reach = case_c()
    for k, v in tq.items():
        print(f"  {k:16s}{v:6.1f} Nm")
    print(f"  reach {reach*1000:.0f} mm")

    print(f"\n=== Actuator selection (dynamic factor {DYN}x) ===")
    print(f"{'joint':17s}{'n':>3}{'cls':>5}{'peak':>7}{'x1.8':>7}{'rated':>7}{'SF':>6}{'kg':>7}")
    n_tot = m_act = 0.0
    for name, n, cls, peak in JOINTS:
        c = CLASSES[cls]
        m = n * c["mass"]
        n_tot += n
        m_act += m
        print(f"{name:17s}{n:>3}{cls:>5}{peak:>7.1f}{peak*DYN:>7.1f}"
              f"{c['torque']:>7}{c['torque']/peak:>6.1f}{m:>7.2f}")
    print(f"{'TOTAL':17s}{n_tot:>3.0f}{'':>5}{'':>7}{'':>7}{'':>7}{'':>6}{m_act:>7.2f}")

    print("\n=== Mass budget ===")
    total = m_act
    print(f"  {'actuators (22)':38s}{m_act:>6.2f} kg")
    for k, v in OTHER_MASS:
        print(f"  {k:38s}{v:>6.2f} kg")
        total += v
    print(f"  {'-'*38}{'':>6}")
    print(f"  {'TOTAL':38s}{total:>6.2f} kg   (assumed {M} kg)")

    print("\n=== SKU rollup ===")
    for cls, c in CLASSES.items():
        n = sum(n for _, n, k, _ in JOINTS if k == cls)
        print(f"  {cls}  {c['size']:>8}  {c['torque']:>3} Nm  x{n:<3} "
              f"{n*c['mass']:>5.2f} kg")


if __name__ == "__main__":
    main()
