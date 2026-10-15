"""Mass and joint-torque budget for the rose line at a given standing height.

Why this exists: "rose full should be about 1800 mm, rose mega 2000" is a height
decision, and height decides everything else - mass goes roughly as the cube,
joint torque as mass x length (the fourth power), and the actuator that fits
the torque sets the envelope the sheet has to wrap. So the first thing a
re-layout needs is a number for each joint, with its provenance, before any
link length is chosen. Run: python3 sizing.py [height_mm ...]

CALIBRATION. Three robots whose URDF effort limits we have, normalised the same
way (knee peak / (m g L_thigh)):
    G1   1.32 m  34.1 kg  thigh 0.332  knee 139   -> 1.25
    H2   1.82 m  70 kg    thigh 0.412  knee 360   -> 1.27   (hips also 360)
    rose full 1.17 m 17.6 kg thigh 0.250 knee 60   -> 1.39
So knee and hip pitch want ~1.3 m g L_thigh peak, and Unitree gives every hip
axis the same actuator as the knee. Ankle pitch: G1 35/34 kg, H2 67/70 kg
-> ~1 N.m per kg. Shoulder pitch: G1 25, H2 130 -> ~1.6 N.m/kg at H2 scale.

MASS. Height-cubed density: rose full as modelled 17.6 kg at 1.17 m -> 11.0
kg/m^3 (it is mostly air: no battery, skin or compute in the model yet); H2 70
kg at 1.82 -> 11.6; G1 34 at 1.32 -> 14.8. 11.6 is the design figure (a full
machine at H2 density), 14.8 the pessimistic one (G1 density: heavier
actuators per N.m, which commodity actuators are). Both are printed, and the
actuator pick must hold at the pessimistic one.
"""
import sys

G = 9.81
# proportions adopted from the H2 (UNITREE.md "H2, measured"), as fractions of height
P = dict(leg=0.53,           # sole -> hip pitch
         thigh=0.225,        # hip pitch -> knee   (H2 412/1833)
         shank=0.270,        # knee -> ankle       (H2 497/1833)
         ankle=0.030,        # ankle -> sole
         shoulder=0.83,      # sole -> shoulder pitch
         torso=0.29,         # hip pitch -> shoulder pitch
         upper_arm=0.126,    # shoulder -> elbow   (H2 231/1833)
         forearm=0.131)      # elbow -> wrist

# peak torque per joint as multiples: (k_mgL on m*g*L_thigh) or (k_m on m)
TORQUE = {
    'hip_pitch':  ('mgL', 1.3),
    'hip_roll':   ('mgL', 1.0),   # H2 gives it 1.3 too; G1 1.0 - take the lighter
    'hip_yaw':    ('mgL', 0.7),   # G1: same as pitch; H2 same. Honest floor 0.7
    'knee':       ('mgL', 1.3),
    'ank_pitch':  ('m',   1.0),
    'ank_roll':   ('m',   0.5),
    'sho_pitch':  ('m',   1.6),
    'sho_roll':   ('m',   0.85),
    'elbow':      ('m',   0.85),
    'wrist':      ('m',   0.25),
    'head':       ('m',   0.15),
}

# what exists. (peak N.m, mass kg, OD mm, length mm). X-series from vendor data
# where we have it (X4/X6: STEP + manual); the larger ones are from the vendor's
# published tables - see SIZING.md for provenance and what is still estimated.
# rana from mega/INTERFACES.md.
CATALOGUE = {
    # name:     (peak N.m, mass kg, OD mm, axial length mm)   source
    'X4-36':    (34,  0.36, 55,  61.0),     # vendor STEP, V4 manual
    'X6-60':    (60,  0.82, 80,  67.5),     # vendor STEP, V4 manual
    'X8-120':   (120, 1.40, 96,  76.0),     # RMD-X8-P20-120 2D-A0 (Aug 2026)
    'X10-200':  (200, 1.64, 124, 63.5),     # RMD-X10-P20-200-E spec V1.0 (Aug 2026)
    'X12-320':  (320, 2.37, 124, 85.0),     # RMD-X12-P20-320 2D-A0 (Aug 2026)
    'X15-450':  (450, 3.50, 166, 69.0),     # RMD-X15-P20-450 2D-A0 (Aug 2026); 72 V
    'rana-100': (None, 0.55, None, None),   # mega/INTERFACES.md
    'rana-140': (96,  None, None, None),    # mega/INTERFACES.md
}


def budget(h_mm, density=11.6):
    h = h_mm / 1000.0
    m = density * h ** 3
    L = P['thigh'] * h
    out = {}
    for j, (kind, k) in TORQUE.items():
        out[j] = k * (m * G * L if kind == 'mgL' else m)
    return m, L, out


def pick(nm, family):
    opts = [(t, n) for n, (t, *_r) in CATALOGUE.items() if n.startswith(family) and t]
    for t, n in sorted(opts):
        if t >= nm:
            return n, t
    return 'NONE (%.0f needed)' % nm, None


def main():
    hs = [float(a) for a in sys.argv[1:]] or [1173.0, 1800.0, 2000.0]
    for h in hs:
        for dens, tag in ((11.6, 'H2 density, design'), (14.8, 'G1 density, pessimistic')):
            m, L, t = budget(h, dens)
            print('\n=== %.0f mm, %s: mass %.0f kg, thigh %.0f mm, shank %.0f mm, torso %.0f mm ==='
                  % (h, tag, m, L * 1000, P['shank'] * h, P['torso'] * h))
            for j, nm in t.items():
                fam = 'rana' if h >= 1900 else 'X'
                n, _ = pick(nm, fam)
                print('  %-10s %6.0f N.m   -> %s' % (j, nm, n))


if __name__ == '__main__':
    main()
