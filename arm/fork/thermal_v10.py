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
FLANK = float(os.environ.get('LYMB_FLANK', 0.0))
BARREL = float(os.environ.get('LYMB_BARREL', 1.0))   # v10.3: open annulus + a dot-grid round the ring's circumference
J3_GRID = float(os.environ.get('LYMB_J3_GRID', 0.95))   # J3's grid open area vs the old 4-face 29 % grid: v10.5 = 3 faces (front clean) at 1.4 / 2.2 hex (37 %) = 0.75 x 37 / 29 = 0.95
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
    global J3_GRID
    if 'LYMB_J3_GRID' not in os.environ and S.J3_OPEN.get('ref'):
        J3_GRID = S.J3_OPEN['v105'] / S.J3_OPEN['ref']      # the v10.5 field's open area vs the v10.3 grid's
        if not quiet:
            print('J3 grid: %.0f mm2 open (v10.5 fields, 1.4 / 2.2, 3 faces) vs %.0f mm2 (v10.3, 1.3 / 2.3, 4 faces): x %.2f'
                  % (S.J3_OPEN['v105'], S.J3_OPEN['ref'], J3_GRID))
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
            G_vent = G_skin + H * PHI * math.pi * m['d'] * m['L'] * (J3_GRID if j == 'J3' else 1.0)   # v10.5: J3's grid on the inner face + back half only
        if vented is True and OPEN:
            # open drum: the puck is a graphite RING round the can, both motor end faces open to air (the
            # motor's own dark face is the puck's face) and the 8.5 mm annulus round the can open at both
            # ends (h on the can's barrel taken at 0.6 h); the plates inside the sleeves stay enclosed
            ends = 2 * math.pi * (m['d'] / 2) ** 2
            barrel = math.pi * m['d'] * m['L']
            # + dot-grid panels on both sleeve flanks over the holder plates (FLANK of the plate area
            #   sees air through 29 %-open panels, taken at PHI of free convection)
            if ALPANEL and j in ('J2', 'J4'):   # panels over the J2 / J4 holder plates only (the upper arm)
                # v10.3 choice: the sleeve's flat front/back faces over the holder plates are 1 mm 5052
                # panels, warm-black anodized, on a 0.5 mm gap-pad to the plates: the plates keep their own
                # free-convection area (the panel is the same size, eta_p as bare); pad + panel add 0.15 K/W
                G_pl = 1 / (1 / (H * ETA_P * Ap) + 0.20)   # 0.25 mm pad + 0.6 mm panel (pad ~3 W/mK)
                G_vent = H * (PHI_CAP * ends + BARREL * (0.8 if (j == 'J2' and os.environ.get('LYMB_MOUNT') == 'body') else 1.0) * barrel) + G_pl   # v10.4: the deltoid hood covers a third of J2's dot-grid
            else:   # J6: open ring, its holder plates stay enclosed under the sleeve
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


def j1_body(verbose=True):
    """J1 on the humanoid, inside rose's torso corner (claude/quad00-torso-v103). Lumped, steady state +
    one-node transient. ASSUMPTIONS (all mine, to be checked on the elbow rig's heat test):
      J1 = RS04: R_int 0.49 K/W (backed out above), Kt 2.1 N.m/A_rms, R_line 0.16 ohm -> P = 0.24 I_rms^2;
         winding limit 120 C, ambient 25 C (95 K).
      (B) conduction, built in: J1 back face -> 1/4" foot (contact 0.10 K/W) -> 41 mm adapter tube, 4 x 1/8" 5052
         plates 130 wide (R = L / k A, k 138 W/mK) -> torso side plate (6.35 mm 5052, 26 600 mm2 a face) + half
         of each deck it carries (35 200 / 15 300 mm2): 0.051 m2 a face, both faces, fin efficiency 0.6 ->
         enclosed chest air (h_in 5 W/m2K) -> this side's half of the chest shell (0.20 m2, h 10). No other
         heat in the chest is counted (the cortex is in the head; the limbic board, battery are NOT added).
      the corner: J1's can (barrel + back face) -> enclosed corner air (h_in 5) -> the same chest shell path.
      (open ring) J1's output end: the annulus between the can (r 60) and the graphite ring (r 66.5) open to
         the room, h 10 on 2 x the annulus (both walls); the output face itself conducts into the turret disc +
         towers (0.020 m2 a face of 5052, efficiency 0.5) which sit under the turret skin (h_in 5 inside, its
         0.034 m2 outside at h 10).
      (fan) 1.3 L/s of 25 C air ducted along J1's barrel: annular duct 10 mm deep round the r 60 can ->
         v 0.34 m/s, h = 10 + 3.9 sqrt(v / 0.05) = 20 W/m2K on barrel + back face (0.031 m2); air-side limit
         m cp = 1.57 W/K, so G_fan = m cp (1 - exp(-h A / m cp)).
      transient: one node, the whole RS04 (1.42 kg, c 600 J/kgK -> 850 J/K) behind R_int; the winding's own
         ~0.3 kg of copper heats faster, so the time is an UPPER bound.
    """
    H_, HI, k = 10.0, 5.0, 138.0
    P_r, R_ref, R_int = motor_numbers('RS04')
    m = F.J1M
    A_can = math.pi * m['d'] * m['L'] + math.pi * (m['d'] / 2) ** 2
    shell = 1 / (H_ * 0.20)
    # (B) plate path
    R_tube = 0.041 / (k * 4 * 0.130 * 0.003175)
    A_fr = 2 * 0.051 * 0.6
    R_plate = 0.10 + R_tube + 1 / (HI * A_fr) + shell
    # the corner: can -> enclosed air -> shell (shares the shell: taken in parallel, conservative-ish)
    R_corner = 1 / (HI * A_can) + shell
    # open ring at the output end + the turret's metal under its skin
    ann = math.pi * (0.0665 ** 2 - 0.060 ** 2)
    G_ring = H_ * 2 * ann + 1 / (1 / (HI * 0.5 * 2 * 0.020) + 1 / (H_ * 0.034))
    # fan
    mcp, h_f = 1.2e-3 * 1.3 * 1005, 10 + 3.9 * math.sqrt(0.34 / 0.05)
    G_fan = mcp * (1 - math.exp(-h_f * 0.031 / mcp))
    cases = {'plate only (B)': 1 / R_plate + 1 / R_corner,
             'plate + open ring': 1 / R_plate + 1 / R_corner + G_ring,
             'plate + fan 1.3 L/s': 1 / R_plate + G_fan + 1 / R_corner,
             'plate + open ring + fan': 1 / R_plate + G_fan + 1 / R_corner + G_ring}
    T_hold = 23.5          # rose's arms-forward rated hold (1.27 kg); C.moment(0, 1.27) gives the same
    M0 = C.moment(0.0, 0.0)
    out = {}
    for nm, G in cases.items():
        R = R_int + 1 / G
        P_max = DT / R
        T_cont = 2.1 * math.sqrt(P_max / 0.24)
        kg = (T_cont - M0) / (C.G * C.flange_x())
        P_h = 0.24 * (T_hold / 2.1) ** 2
        Css = 1.42 * 600
        if P_h * R <= DT:
            t_lim = float('inf')
        else:
            t_lim = -Css * R * math.log(1 - DT / (P_h * R))
        T2 = C.moment(0.0, 1.27)       # the arm model's own arms-forward torque at 1.27 kg (skin mass included)
        P2 = 0.24 * (T2 / 2.1) ** 2
        t2 = float('inf') if P2 * R <= DT else -Css * R * math.log(1 - DT / (P2 * R))
        out[nm] = (1 / G, T_cont, kg, P_h, t_lim, T2, t2)
    if verbose:
        print('\nJ1 on the humanoid (torso corner), RS04: holding %.1f N.m dissipates %.0f W' % (T_hold, 0.24 * (T_hold / 2.1) ** 2))
        T2 = list(out.values())[0][5]
        print('%-26s %8s %10s %12s %16s %16s' % ('case', 'R c-a K/W', 'T_cont N.m', 'hold kg fwd', 't@23.5 N.m', 't@%.1f N.m' % T2))
        f_ = lambda tl: 'never (steady)' if tl == float('inf') else '%.0f min' % (tl / 60)
        for nm, (Rca, Tc, kg, Ph, tl, T2, t2) in out.items():
            print('%-26s %8.2f %10.1f %12.2f %16s %16s' % (nm, Rca, Tc, kg, f_(tl), f_(t2)))
    return out


if __name__ == '__main__':
    fo = main()
    j1_body()
    import json
    json.dump({j: round(fv / fb, 3) for j, (fb, fs, fv) in fo.items()}, open(os.path.join(HERE, 'thermal_v10.json'), 'w'), indent=1)
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
