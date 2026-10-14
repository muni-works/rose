"""Search for an OPEN sheet 'bone' for the rose arm links - structure you can see into,
with no closed shell - and score it against the closed box.

Every candidate is flat plates only (router, no brake). Scored on the upper arm and
forearm at their waist sections, with the loads from rose_arm_layout:
  sag    flange deflection from bending, arm straight out, 2.5 kg
  twist  flange deflection from torsion, elbow at 90 (the roll joint loads the link in torsion)
  mass   both links
  open   fraction of the link's side/face area you can see through
  core   clear inner channel for the cable bundle (mm^2), and whether it is enclosed on 4 sides

The physics that decides it: bending is easy (two side plates far apart do it), torsion
is not. An open section's torsion constant is sum(b t^3 / 3) - for 2 mm sheet that is
three orders of magnitude below a closed one. A LATTICE face (laser-cut triangulated
plate) closes the section in shear while staying see-through: Warren diagonals give an
equivalent shear stiffness  G t_eq = E A_d b cos^2(theta) / (d s)  per face.

Run:  python3 arm/rose_arm_bone.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rose_arm_layout as A

E, G = 69e9, 26e9
RHO = A.SHEET_RHO
LIMIT_SAG = 0.5e-3     # m at the flange
LIMIT_TWIST = 0.5e-3   # m at the flange


def lattice_face(b, L, t, w, bay=None, xbrace=False):
    """Triangulated face plate b wide, L long: two edge rails + diagonals. Returns (G*t_eq, mass, open fraction)."""
    s = bay or b                           # bay length ~ width: 45 deg diagonals
    d = math.hypot(s, b)
    A_d = w * t
    nd = 2 if xbrace else 1
    Gt = nd * E * A_d * b * (s / d) ** 2 / (d * s)
    n_bays = max(1, round(L / s))
    area_solid = 2 * w * L + nd * n_bays * w * d
    m = RHO * t * area_solid
    open_frac = max(0.0, 1 - area_solid / (b * L))
    return Gt, m, open_frac


def section(kind, W, H, L, **k):
    """Return dict(I, J, mass, open, core, enclosed) for a link of outer W (across) x H (deep) x L long."""
    ts = k.get('ts', 0.002)        # side plate
    tf = k.get('tf', 0.0015)       # face / skin
    w = k.get('w', 0.008)          # lattice strut width
    win = k.get('win', 0.30)       # window fraction in side plates
    I_sides = 2 * ts * H ** 3 / 12 * (1 - 0.5 * win)
    if kind == 'box':
        I = (tf * H ** 3 / 6 + W * tf * H ** 2 / 2) * (1 - A.CUTOUT)
        J = 4 * (W * H) ** 2 * tf / (2 * (W + H)) * (1 - A.CUTOUT) ** 2
        m = RHO * tf * 2 * (W + H) * L * (1 - A.CUTOUT)
        return dict(I=I, J=J, mass=m, open=A.CUTOUT, core=W * H, enclosed=True)
    if kind == 'fin':
        t = 0.003
        return dict(I=t * H ** 3 / 12, J=H * t ** 3 / 3, mass=RHO * t * H * L * (1 - win), open=1.0,
                    core=0.0, enclosed=False)
    if kind == 'twin':
        J = 2 * H * ts ** 3 / 3
        m = RHO * ts * 2 * H * L * (1 - win) + 3 * RHO * 0.003 * W * H * 0.4    # 3 spacer plates
        return dict(I=I_sides, J=J, mass=m, open=1 - (2 * H * (1 - win)) / (2 * (W + H)), core=W * H, enclosed=False)
    if kind == 'ribs':
        J = 2 * H * ts ** 3 / 3 * 3.0          # diaphragms every ~W restrain warping: generous x3
        m = RHO * ts * 2 * H * L * (1 - win) + RHO * 0.002 * W * H * 0.6 * max(2, L / W)
        return dict(I=I_sides, J=J, mass=m, open=1 - (2 * H * (1 - win)) / (2 * (W + H)), core=W * H, enclosed=False)
    if kind in ('truss', 'truss1', 'trussX'):
        faces = 1 if kind == 'truss1' else 2
        Gt, mf, of = lattice_face(W, L, tf, w, xbrace=(kind == 'trussX'))
        I = I_sides + faces * 2 * (w * tf) * (H / 2) ** 2
        if faces == 2:
            # Bredt with mixed walls: sum of l / t_eff, sides real sheet, faces G t_eq / G
            J = 4 * (W * H) ** 2 / (2 * H / (ts * (1 - win) ** 2) + 2 * W / (Gt / G))
        else:
            J = 2 * H * ts ** 3 / 3 + W * tf ** 3 / 3
        m = RHO * ts * 2 * H * L * (1 - win) + faces * mf
        open_frac = (2 * H * win + faces * W * of + (2 - faces) * W) / (2 * (W + H))
        return dict(I=I, J=J, mass=m, open=open_frac, core=(W - 2 * w) * (H - 2 * w), enclosed=(faces == 2))
    if kind == 'cage':
        # four lattice faces on four corner rails: a space frame cut flat and tabbed together
        Gt_w, m_w, o_w = lattice_face(W, L, tf, w, xbrace=k.get('x', True))
        Gt_h, m_h, o_h = lattice_face(H, L, tf, w, xbrace=k.get('x', True))
        I = 4 * (2 * w * tf) * (H / 2) ** 2          # corner rails (one from each meeting face)
        J = 4 * (W * H) ** 2 / (2 * H / (Gt_h / G) + 2 * W / (Gt_w / G))
        m = 2 * m_w + 2 * m_h
        open_frac = (2 * W * o_w + 2 * H * o_h) / (2 * (W + H))
        return dict(I=I, J=J, mass=m, open=open_frac, core=(W - 2 * w) * (H - 2 * w), enclosed=True)
    raise ValueError(kind)


def flange(sec_u, sec_f):
    """Same load cases as rose_arm_layout.stiffness(), per section."""
    U, F, Hd, R_ = A.UPPER, A.FORE, A.HAND, A.REACH
    m_e = A.moment_past(U)
    F_e = m_e / (R_ - U)
    m_w = A.moment_past(U + F)
    F_w = m_w / Hd
    th_u = (F_e * U ** 2 / 2 + m_e * U) / (E * sec_u['I'])
    d_u = (F_e * U ** 3 / 3 + m_e * U ** 2 / 2) / (E * sec_u['I'])
    th_f = (F_w * F ** 2 / 2 + m_w * F) / (E * sec_f['I'])
    d_f = (F_w * F ** 3 / 3 + m_w * F ** 2 / 2) / (E * sec_f['I'])
    sag = d_u + th_u * (F + Hd) + d_f + th_f * Hd
    twist = m_e * U / (G * sec_u['J']) * (R_ - U) + m_w * F / (G * sec_f['J']) * Hd
    return sag, twist


CANDIDATES = [
    ('closed box (current)', 'box', {}),
    ('single fin, 3 mm', 'fin', {}),
    ('twin side plates, spacers', 'twin', {}),
    ('twin plates + bulkhead ribs', 'ribs', {}),
    ('twin plates + 1 lattice face', 'truss1', {}),
    ('twin plates + 2 lattice faces, 6 mm struts', 'truss', {'w': 0.006}),
    ('twin plates + 2 lattice faces, 8 mm struts', 'truss', {'w': 0.008}),
    ('twin plates + 2 lattice faces, 12 mm struts', 'truss', {'w': 0.012}),
    ('twin plates + 2 X-braced faces, 6 mm', 'trussX', {'w': 0.006}),
]


def main():
    Wu, Hu = A.UA_W, A.UA_W * A.WAIST
    Wf, Hf = A.FA_W, A.FA_W * A.WAIST
    print('loads: elbow %.1f N.m, wrist %.1f N.m (2.5 kg, arm straight out)' % (A.moment_past(A.UPPER), A.moment_past(A.UPPER + A.FORE)))
    print('sections at the waist: upper %.0f x %.0f, forearm %.0f x %.0f mm\n' % (Wu * 1e3, Hu * 1e3, Wf * 1e3, Hf * 1e3))
    hdr = '%-44s %7s %8s %7s %6s %7s  %s'
    print(hdr % ('candidate', 'mass', 'sag', 'twist', 'open', 'core', 'verdict'))
    rows = []
    for name, kind, k in CANDIDATES:
        su = section(kind, Wu, Hu, A.UPPER, **k)
        sf = section(kind, Wf, Hf, A.FORE, **k)
        sag, tw = flange(su, sf)
        m = su['mass'] + sf['mass']
        ok = sag < LIMIT_SAG and tw < LIMIT_TWIST
        verdict = ('PASS' if ok else 'fails ' + ('twist' if tw >= LIMIT_TWIST else 'sag')) + \
                  ('' if su['enclosed'] else ', cables exposed')
        rows.append((name, m, sag, tw, su['open'], su['core'], ok, su['enclosed']))
        print('%-44s %5.2f kg %6.2f mm %5.1f mm %4.0f %% %5.0f mm2  %s' % (
            name, m, sag * 1e3, tw * 1e3, 100 * su['open'], su['core'] * 1e6, verdict))
    print('\n--- sweep: twin side plates + two lattice faces ---')
    found = []
    for ts in (0.002, 0.003):
        for win in (0.15, 0.30):
            for tf in (0.0015, 0.002, 0.003):
                for w in (0.006, 0.008, 0.010, 0.012):
                    for kind in ('truss', 'trussX'):
                        k = dict(ts=ts, win=win, tf=tf, w=w)
                        su = section(kind, Wu, Hu, A.UPPER, **k)
                        sf = section(kind, Wf, Hf, A.FORE, **k)
                        sag, tw = flange(su, sf)
                        if sag < LIMIT_SAG and tw < LIMIT_TWIST:
                            found.append((su['mass'] + sf['mass'], kind, k, sag, tw, su['open'], su['core']))
    for waist in (A.WAIST, 1.0):
        Wu2, Hu2, Wf2, Hf2 = Wu, A.UA_W * waist, Wf, A.FA_W * waist
        for tf in (0.0015, 0.002, 0.003):
            for w in (0.006, 0.008, 0.010, 0.012, 0.014):
                for kind, extra in (('cage', {}), ('truss', {'ts': 0.002, 'win': 0.3}), ('trussX', {'ts': 0.002, 'win': 0.3})):
                    k = dict(tf=tf, w=w, **extra)
                    su = section(kind, Wu2, Hu2, A.UPPER, **k)
                    sf = section(kind, Wf2, Hf2, A.FORE, **k)
                    sag, tw = flange(su, sf)
                    if sag < LIMIT_SAG and tw < LIMIT_TWIST:
                        k['waist'] = waist
                        found.append((su['mass'] + sf['mass'], kind, k, sag, tw, su['open'], su['core']))
    found.sort(key=lambda r: r[0])
    for m, kind, k, sag, tw, op, core in found[:8]:
        print('  %.2f kg  %-6s waist %3.0f%%  sides %s  faces %.1f mm struts %2.0f mm   sag %.2f  twist %.2f  open %2.0f%%  core %4.0f mm2'
              % (m, kind, 100 * k.get('waist', A.WAIST), 'lattice  ' if kind == 'cage' else '%.1f w%2.0f%%' % (k['ts'] * 1e3, k['win'] * 100),
                 k['tf'] * 1e3, k['w'] * 1e3, sag * 1e3, tw * 1e3, op * 100, core * 1e6))
    if found:
        m, kind, k, sag, tw, op, core = found[0]
        rows.append(('BEST lattice bone', m, sag, tw, op, core, True, True))
    best = min((r for r in rows if r[6] and r[7] and r[4] > 0.4), key=lambda r: r[1], default=None)
    print('\nlightest open-looking section that passes both and still cradles the cables:')
    print('  %s - %.2f kg, %.0f %% see-through' % (best[0], best[1], 100 * best[4]) if best else '  none')


if __name__ == '__main__':
    main()


# ------------------------------------------------------------------ 2020 bones (2026-10-12)
# 2020 B-type slot, 6063-T5: I = 0.70 cm^4 each axis, A = 1.6 cm^2, 0.44 kg/m. Torsion constant of the
# open slotted profile is small; J ~ 0.10 cm^4 (four open slots, conservative).
I20, A20, KG20, J20 = 0.70e-8, 1.6e-4, 0.44, 0.10e-8


def bar2020(kind, W, H, L, s=0.060, t_bh=0.003, t_web=0.0015):
    """W x H outer envelope, bars at the corners of the envelope minus 10 mm. Bulkheads every s
    (flat 3 mm plates, tabbed into the slots) make adjacent bars act as a Vierendeel panel:
    G t_eq = 24 E I_bar / (s^2 b)."""
    b_h, b_w = H - 0.020, W - 0.020
    if kind == '1x':
        return dict(I=I20, J=J20, mass=KG20 * L, open=1.0, core=0.0, enclosed=False)
    if kind == '2x':          # two bars, one above the other (bending), bulkheads between: one Vierendeel wall
        I = 2 * (I20 + A20 * (b_h / 2) ** 2)
        J = 2 * J20 + 2 * (A20 * (b_h / 2) ** 2) * 0.0 + 0.0
        # one shear wall is not a closed cell: torsion is the bars' own plus the wall's warping restraint, ~ open
        m = 2 * KG20 * L + RHO * t_bh * 0.03 * b_h * L / s
        return dict(I=I, J=J * 3, mass=m, open=0.9, core=0.0, enclosed=False)
    if kind == '2x+webs':     # two bars + a flat 1.5 mm web each side: a closed box made of bars and plates
        I = 2 * (I20 + A20 * (b_h / 2) ** 2) + 2 * t_web * b_h ** 3 / 12 * 0.7
        J = 4 * (0.02 * b_h) ** 2 / (2 * b_h / (t_web * 0.49) + 2 * 0.02 / 0.004)
        m = 2 * KG20 * L + 2 * RHO * t_web * b_h * L * 0.7
        return dict(I=I, J=J, mass=m, open=0.3, core=0.0, enclosed=True)
    if kind == '4x':          # four corner bars + bulkhead plates: four Vierendeel walls, a closed cell
        I = 4 * (I20 + A20 * (b_h / 2) ** 2)
        Gt_h = 24 * E * I20 / (s ** 2 * b_h)
        Gt_w = 24 * E * I20 / (s ** 2 * b_w)
        J = 4 * (b_w * b_h) ** 2 / (2 * b_h / (Gt_h / G) + 2 * b_w / (Gt_w / G)) + 4 * J20
        n_bh = max(2, round(L / s) + 1)
        m = 4 * KG20 * L + n_bh * RHO * t_bh * (b_w * b_h) * 0.35       # ring bulkheads, centre open
        return dict(I=I, J=J, mass=m, open=0.75, core=(b_w - 0.02) * (b_h - 0.02), enclosed=False)
    raise ValueError(kind)


def main_2020():
    print('\n--- 2020 bones (links at full depth: upper %.0f x %.0f, forearm %.0f x %.0f) ---' % (
        A.UA_W * 1e3, A.UA_W * 1e3, A.FA_W * 1e3, A.FA_W * 1e3))
    print('%-46s %7s %8s %8s  %s' % ('candidate', 'mass', 'sag', 'twist', 'verdict'))
    for name, kind, k in [('single 2020', '1x', {}), ('two 2020, stacked, bulkheads', '2x', {}),
                          ('two 2020 + flat 1.5 mm side webs', '2x+webs', {}),
                          ('four 2020 + bulkheads every 60', '4x', {'s': 0.060}),
                          ('four 2020 + bulkheads every 40', '4x', {'s': 0.040}),
                          ('four 2020 + bulkheads every 80', '4x', {'s': 0.080})]:
        su = bar2020(kind, A.UA_W, A.UA_W, A.UPPER, **k)
        sf = bar2020(kind, A.FA_W, A.FA_W, A.FORE, **k)
        sag, tw = flange(su, sf)
        ok = sag < LIMIT_SAG and tw < LIMIT_TWIST
        print('%-46s %5.2f kg %6.2f mm %6.2f mm  %s' % (name, su['mass'] + sf['mass'], sag * 1e3, tw * 1e3,
              'PASS' if ok else 'fails ' + ('twist' if tw >= LIMIT_TWIST else 'sag')))


if __name__ == '__main__':
    main_2020()
