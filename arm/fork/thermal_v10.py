"""lymb v10.3 steady-state thermal estimate: does the skin cost continuous torque?

Lumped network per enclosed motor, winding -> case -> ambient (25 C), winding limit 120 C (Class B
insulation is 130 C; 10 K kept back). Every number below is an ASSUMPTION unless it says manual.

  manual (RS spec book 2026-10-09, ~/Developer/robstride_assets):
    RS04  rated 40 N.m on a 345 x 345 mm Al sink plate, 27 A pk rated phase current, 0.16 ohm line
    RS02  rated  7 N.m on a 150 x 150 mm plate,          7 A pk,                      0.55 ohm
    RS00  rated  5 N.m on a 150 x 150 mm plate,          4.7 A pk,                    1.5 ohm
  copper loss at the rated point  P = 1.5 x R_line x I_rms^2  (3-phase, line-to-line resistance)
  the manual's sink plate:  R_ref = 1 / (h x eta x 2 a^2),  h = 10 W/m2K (free convection + radiation),
                            eta = 0.7 (plate fin efficiency)
  so the motor's own winding -> mounting-face resistance   R_int = 95 K / P - R_ref

  in the arm, case -> ambient conductance G:
    bare (no skin)    h x (A_can + eta_p x A_plates)          eta_p = 0.5 for the long thin 3 mm plates
    skinned           the can and its plates -> enclosed air -> shell -> ambient, in series:
                      1 / (1 / (h_in x (A_can + eta_p A_plates)) + 1 / (h x A_shell)),  h_in = 5 W/m2K
    vented            + h x phi x A_vent in parallel: the perforated hub caps (1.3 mm holes, 2.3 mm hex,
                      29 % open) pass phi = 0.5 of an open face's free convection (ASSUMED)
  allowed loss  P_max = 95 K / (R_int + 1/G); torque scales with sqrt(P) (copper loss ~ I^2), so the
  derating against the manual's rated condition is f = sqrt(P_max / P_rated), capped at 1.
  The model's hold numbers use STALL-continuous ratings (fork_checks RATING: 25 / 4.4 / 3.6 N.m),
  taken as valid on the manual's sink; the skin's effect is f_skin / f_bare applied to them.

    ROSE_ARM=v10 uv run -q --with numpy --with manifold3d --with trimesh --with scipy python arm/fork/thermal_v10.py
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np
import rose_fork as F
import fork_checks as C
import skin_v10 as S

H, ETA_REF, ETA_P, H_IN, PHI, DT = 10.0, 0.7, 0.5, 5.0, 0.5, 95.0
MANUAL = {'RS04': (40.0, 27.0, 0.16, 0.345), 'RS02': (7.0, 7.0, 0.55, 0.150), 'RS00': (5.0, 4.7, 1.5, 0.150)}
FLANK = float(os.environ.get('LYMB_FLANK', 0.6))
BARREL = float(os.environ.get('LYMB_BARREL', 0.6))
PHI_CAP = float(os.environ.get('LYMB_PHI_CAP', 1.0))   # 1.0: open motor faces behind a bezel (v10.3); 0.5: perforated 1.3 / 2.3 hex caps
ALPANEL = os.environ.get('LYMB_ALPANEL', '1') == '1'
OPEN = os.environ.get('LYMB_OPEN_DRUM', '1') == '1'
VENT_OPEN = (math.pi / 4 * 1.3 ** 2) / (math.sqrt(3) / 2 * 2.3 ** 2)


def motor_numbers(name):
    T, ipk, R, a = MANUAL[name]
    P = 1.5 * R * (ipk / math.sqrt(2)) ** 2
    R_ref = 1 / (H * ETA_REF * 2 * a * a)
    return P, R_ref, DT / P - R_ref


def sheet_area(pred):
    A = 0.0
    for p in F.build():
        if p[1] in ('sheet', 'cover') and pred(p):
            A += 2 * (F.area2d(p[5]) - sum(F.area2d(h) for h in (p[7] if len(p) > 7 else [])))
    return A


def main(vent_scale=1.0, quiet=False):
    parts = {nm: m for nm, l, m, _ in S.skin_meshes()}
    A = lambda nm, k=1.0: parts[nm].area * 1e-6 * k if nm in parts else 0.0
    is_end = lambda p: abs(p[6][0]) > 1e-6          # extruded along x: an end plate / bridge / flange
    S_ = F.stations()
    J = [  # joint, motor, can area, plates area, shell area, vent faces area
        ('J2', F.J2M, sheet_area(lambda p: p[0] == 'upper' and not is_end(p)) + 0.5 * sheet_area(lambda p: p[0] == 'turret'),
         A('puck-J2') + A('upper', 0.5) + A('turret', 0.5), True),
        ('J3', F.J3M, sheet_area(lambda p: p[0] in ('upper', 'upperd') and is_end(p)), A('upper', 0.5) + A('upperd', 0.3), 'band'),
        ('J4', F.J4M, sheet_area(lambda p: p[0] == 'upperd' and not is_end(p)), A('puck-J4') + A('upperd', 0.4) + A('fore', 0.3), True),
        ('J6', F.J6M, sheet_area(lambda p: p[0] == 'yoke'), A('puck-J6') + A('fore', 0.4) + A('wrist', 0.5), True),
        ('J7', F.J7M, sheet_area(lambda p: p[0] in ('wrist', 'tool') and is_end(p)), A('wrist', 0.5), 'band'),
    ]
    rows = []
    f_out = {}
    for j, m, Ap, Ash, vented in J:
        name = F.NAME[id(m)]
        P, R_ref, R_int = motor_numbers(name)
        Acan = math.pi * m['d'] * m['L'] + 2 * math.pi * (m['d'] / 2) ** 2
        G_bare = H * (Acan + ETA_P * Ap)
        G_skin = 1 / (1 / (H_IN * (Acan + ETA_P * Ap)) + 1 / (H * Ash))
        Rp = {'J2': F.R_LOBE_SH, 'J4': F.R_LOBE_EL, 'J6': F.R_LOBE6W}.get(j, 0.0) * 1e3 + 2.5
        A_vent = 2 * min(math.pi * (0.55 * Rp * 1e-3) ** 2 * vent_scale, math.pi * (m['d'] / 2) ** 2) if vented is True else 0.0
        G_vent = G_skin + H * PHI * A_vent
        if vented == 'band':
            # roll cartridges: a dot-grid band in the sleeve right over the can (the barrel sees air at PHI)
            G_vent = G_skin + H * PHI * math.pi * m['d'] * m['L']
        if vented is True and OPEN:
            # open drum: the puck is a graphite RING round the can, both motor end faces open to air (the
            # motor's own dark face is the puck's face) and the 8.5 mm annulus round the can open at both
            # ends (h on the can's barrel taken at 0.6 h); the plates inside the sleeves stay enclosed
            ends = 2 * math.pi * (m['d'] / 2) ** 2
            barrel = math.pi * m['d'] * m['L']
            # + dot-grid panels on both sleeve flanks over the holder plates (FLANK of the plate area
            #   sees air through 29 %-open panels, taken at PHI of free convection)
            if ALPANEL:
                # v10.3 choice: the sleeve's flat front/back faces over the holder plates are 1 mm 5052
                # panels, warm-black anodized, on a 0.5 mm gap-pad to the plates: the plates keep their own
                # free-convection area (the panel is the same size, eta_p as bare); pad + panel add 0.15 K/W
                G_pl = 1 / (1 / (H * ETA_P * Ap) + 0.15)
                G_vent = H * (PHI_CAP * ends + BARREL * barrel) + G_pl
            else:
                G_vent = H * (ends + BARREL * barrel) + H * PHI * FLANK * ETA_P * Ap + 1 / (1 / (H_IN * ETA_P * Ap * (1 - FLANK)) + 1 / (H * Ash))
        f = lambda G: min(1.0, math.sqrt(DT / (R_int + 1 / G) / P))
        fb, fs, fv = f(G_bare), f(G_skin), f(G_vent)
        f_out[j] = (fb, fs, fv)
        rows.append((j, name, P, R_int, 1 / G_bare, 1 / G_skin, 1 / G_vent if vented else None, fb, fs, fv))
    if not quiet:
        print('vents: 1.3 mm holes on a 2.3 mm hex, %.0f %% open; caps at J2 / J4 x %.2f of the hub-cap area' % (100 * VENT_OPEN, vent_scale))
        print('%-4s %-5s %7s %7s %9s %9s %9s %7s %7s %7s' % ('jt', 'motor', 'P_rat W', 'R_int', 'R bare', 'R skin', 'R vented', 'f bare', 'f skin', 'f vent'))
        for j, name, P, Ri, Rb, Rs, Rv, fb, fs, fv in rows:
            print('%-4s %-5s %7.1f %7.2f %9.2f %9.2f %9s %7.2f %7.2f %7.2f' % (j, name, P, Ri, Rb, Rs, '%.2f' % Rv if Rv else '-', fb, fs, fv))
    return f_out


def j2_hold(f_ratio):
    st = F.RATING[id(F.J2M)][1] * f_ratio
    return (st - C.moment(0.0, 0.0)) / (C.G * (C.flange_x()))


if __name__ == '__main__':
    fo = main()
    fb, fs, fv = fo['J2']
    print('\nJ2 hold, model jaws: bare %.2f kg; skinned, no vents %.2f kg; vented %.2f kg   (skin derating = f_config / f_bare)'
          % (j2_hold(1.0), j2_hold(fs / fb), j2_hold(fv / fb)))
    for j, x0, m in (('J4', F.UPPER, F.J4M), ('J6', F.W, F.J6M)):
        fb, fs, fv = fo[j]
        st = F.RATING[id(m)][1] * fv / fb
        print('  %s hold, model jaws, vented: %.2f kg (bare %.2f)' % (j, (st - C.moment(x0, 0.0)) / (C.G * (C.flange_x() - x0)),
                                                                   (F.RATING[id(m)][1] - C.moment(x0, 0.0)) / (C.G * (C.flange_x() - x0))))
    fb, fs, fv = fo['J3']
    print('  J3 continuous, vented: %.2f N.m against the pose cap %.2f N.m' % (F.RATING[id(F.J3M)][1] * fv / fb, 0.8 * F.RATING[id(F.J3M)][1]))
    for j, (fb, fs, fv) in fo.items():
        print('  %s derating with the skin: %.0f %% unvented, %.0f %% vented' % (j, 100 * (1 - fs / fb), 100 * (1 - fv / fb)))
