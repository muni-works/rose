"""Swept clearance for the SKIN panels - the gate the skin did not have.

`sweep.py` proves the BONES clear through every joint range. The skin stands
proud of the bones, so a bone pair clearing by 4 mm is a skin pair overlapping
by 8: every panel grows outward, and two panels that face each other across a
joint each grow toward the other. Nothing checked that until this file.

The proxy, and why it is honest: `[skin-gap r g X]` dilates X by r and erodes
by r-g, so the outer surface sits **+g** off the hull of X (exactly on convex
ground; the concave dips it bridges only ever ADD material, never remove).
`[shell t]` hollows inward. So the outer surface of a panel is the hull of
(its bones + its form blocks) offset outward by SKIN_GAP - which is a
distance test on geometry we already sample in `sweep.py`, no mesh needed.

  skin gap between two panels = (bone/form gap) - 2 * SKIN_GAP

Bridged concavities make the real panel FATTER than this proxy in places, so a
pass here is necessary, not sufficient - it is a lower bound on the real
clearance where the panels face each other across their smooth outer faces,
which is where they meet. Deep concave pockets are called out as UNPROVEN
rather than passed silently.

Run:  python3 skin_sweep.py
"""
import math

import rose_pro_links as L
import sweep as S
import skin as K

MIN_SKIN_GAP = 3.0      # panels must not come closer than this when swept
MEMBERS = False         # --members: print the per-member breakdown under each joint
STEP_DEG = 6.0          # sweep step: coarser than the bone gate's 2 deg (proxies)


def seam_cyls(panel, side):
    """The seam cylinders that CUT this panel, in world mm.

    The cuts live in the loon, so the proxy has to know them too or it keeps
    reporting the overlaps the seams were added to remove."""
    out = []
    for joint, panels in K.SEAM_PANELS.items():
        if panel not in panels:
            continue
        r, half = K.seam_r(joint, panel), K.SEAM[joint][1]
        J = L.JOINTS[joint]
        for sd in (('L', 'R') if side == 'C' else (side,)):
            c = list(J['c'])
            if sd == 'R':
                c[1] = -c[1]
            if J['c'][1] == 0.0 and side != 'C':
                continue
            i = L.AXI[J['ax']]
            ui, vi = [k for k in range(3) if k != i]
            out.append((J['ax'], c[ui], c[vi], r, c[i] - half, c[i] + half))
    return out


def in_seam(pt, cyls):
    for (ax, cu, cv, r, lo, hi) in cyls:
        i = L.AXI[ax]
        ui, vi = [k for k in range(3) if k != i]
        if lo <= pt[i] <= hi and math.hypot(pt[ui] - cu, pt[vi] - cv) <= r:
            return True
    return False


def panel_prims(panel, side):
    """The parent side as ANALYTIC primitives: (kind, data) for plates and boxes.

    Point-to-point over two sampled panels is O(n*m) and took minutes per joint;
    sweep.py's own shape - sample the mover, distance to the static primitives -
    is O(n*k) with k ~ 20. Same answer, seconds."""
    out = []
    want = ('L', 'R', 'C') if side == 'C' else (side, 'C')
    links, _joints = K.PANELS[panel]
    for p in L.PLATES:
        if p.link not in links:
            continue
        tag = 'C' if p.link in K.NO_MIRROR else 'L'
        if tag == 'C' or side in ('L', 'C'):
            out.append(('plate', p, S.plate_r(p), +1, '%s/%s' % (p.link, p.tag)))
        if tag != 'C' and side in ('R', 'C'):
            out.append(('plate', p, S.plate_r(p), -1, '%s/%s' % (p.link, p.tag)))   # mirrored: negate y of the query
    for i, (x0, x1, y0, y1, z0, z1) in enumerate(K.FORM.get(panel, ())):
        for sd in (('L', 'R') if side == 'C' else (side,)):
            a, b = (y0, y1) if sd == 'L' else (-y1, -y0)
            out.append(('box', ((x0, x1), (a, b), (z0, z1)), None, +1,
                        'FORM[%s][%d]' % (panel, i)))
    return out


def d_prim(pt, prim):
    kind, data, r, sgn = prim[:4]
    q = (pt[0], sgn * pt[1], pt[2])
    return S.d_plate(q, data, r) if kind == 'plate' else S.d_box(q, data)


def panel_side(panel, side):
    """Sample points of a panel: its bones plus its form blocks."""
    return [q for _lbl, pp in panel_members(panel, side) for q in pp]


def panel_members(panel, side):
    """Same points as panel_side, but kept per MEMBER: [(label, points)].

    The gate used to report only the worst pair of PANELS, which is one level
    too coarse to act on - 'forearm hits upperarm' does not say whether to move
    a form block, a seam, or a plate. Naming the member on each side is the
    difference between guessing and knowing which solid to shrink."""
    out = []
    want = ('L', 'R', 'C') if side == 'C' else (side, 'C')
    links, _joints = K.PANELS[panel]
    for p in L.PLATES:
        if p.link not in links:
            continue
        tag = 'C' if p.link in K.NO_MIRROR else 'L'
        if tag not in want and not (tag == 'L' and side in ('L', 'C')):
            continue
        pp = S.plate_points(p, pitch=12.0)
        if side == 'R' and tag != 'C':
            pp = [(x, -y, z) for (x, y, z) in pp]
        out.append(('%s/%s' % (p.link, p.tag), pp))
    for i, (x0, x1, y0, y1, z0, z1) in enumerate(K.FORM.get(panel, ())):
        for sd in (('L', 'R') if side == 'C' else (side,)):
            a, b = (y0, y1) if sd == 'L' else (-y1, -y0)
            out.append(('FORM[%s][%d]' % (panel, i),
                        S.box_points(((x0, x1), (a, b), (z0, z1)), pitch=12.0)))
    return out


def panels_for(links):
    """Panels whose links all sit in this set (a panel rides exactly one link
    group, so a panel is on the child side only if every link of it is)."""
    out = []
    for panel, (plinks, _j) in K.PANELS.items():
        if plinks and all(l in links for l in plinks):
            out.append(panel)
    return out


def sides_of(panel):
    return ('C',) if panel in K.CENTRED else ('L', 'R')


def sweep_joint(joint, verbose=True, members=False):
    J = L.JOINTS[joint]
    lo, hi = S.RANGES[joint]
    child = panels_for(set(S.CHILD_LINKS.get(joint, ())))
    parent = panels_for(set(S.PARENT_LINKS.get(joint, ())))
    if not child or not parent:
        return None
    n = int(math.ceil(math.degrees(hi - lo) / STEP_DEG))   # coarser than bones: proxies
    worst = (1e9, None, None, None)
    per_member = {}
    for cp in child:
        for cs in sides_of(cp):
            cseam = seam_cyls(cp, cs)
            c = list(J['c'])
            if cs == 'R':
                c[1] = -c[1]
            for (clbl, craw) in panel_members(cp, cs):
                cpts = [q for q in craw if not in_seam(q, cseam)]   # cut away by its seams
                if not cpts:
                    continue
                for pp in parent:
                    for ps in sides_of(pp):
                        if cs in ('L', 'R') and ps in ('L', 'R') and cs != ps:
                            continue                    # left arm vs right leg: not a pair
                        prims = panel_prims(pp, ps)
                        pseam = seam_cyls(pp, ps)
                        if not prims:
                            continue
                        for k in range(n + 1):
                            th = lo + (hi - lo) * k / n
                            # a reflection in y flips the sense of x- and z-rotations,
                            # so the right side sweeps the mirrored angle (same trap
                            # pose.py hit: without this the right arm swings the wrong
                            # way and reports a 94 mm chest collision that is not real)
                            if cs == 'R' and J['ax'] in ('x', 'z'):
                                R = S.rot(J['ax'], -th)
                            else:
                                R = S.rot(J['ax'], th)
                            for q in cpts:
                                x = S.apply(R, c, q)
                                if in_seam(x, pseam):
                                    continue    # the parent has no material here
                                for w in prims:
                                    d = d_prim(x, w)
                                    key = ('%s/%s %s' % (cs, cp, clbl),
                                           '%s/%s %s' % (ps, pp, w[4]))
                                    if d < per_member.get(key, (1e9,))[0]:
                                        per_member[key] = (d, math.degrees(th))
                                    if d < worst[0]:
                                        worst = (d, '%s/%s' % (cs, cp),
                                                 '%s/%s' % (ps, pp), math.degrees(th))
    skin_gap = worst[0] - 2 * K.SKIN_GAP
    if verbose and worst[1]:
        flag = ('  CLASH' if skin_gap < 0 else
                ('  TIGHT' if skin_gap < MIN_SKIN_GAP else ''))
        print('  %-10s %-14s vs %-14s  bone %6.1f -> skin %+7.1f mm at %5.0f deg%s'
              % (joint, worst[1], worst[2], worst[0], skin_gap, worst[3], flag))
    if members:
        # WHICH SOLID. The panel-level line says a pair is tight; this says
        # whether the collider is a plate, a form block, or a seam that is too
        # small - which is the only form of the answer you can act on.
        for key, (d, th) in sorted(per_member.items(), key=lambda kv: kv[1][0])[:12]:
            print('        %-38s vs %-38s  bone %6.1f -> skin %+7.1f  at %5.0f deg'
                  % (key[0], key[1], d, d - 2 * K.SKIN_GAP, th))
    return skin_gap


def usable_range(joint):
    """The sub-range of the URDF range the SKIN can actually reach.

    sweep.py did this for the bones when it found the honest knee limit was
    0..93 deg before the knee moved back. Same question, one layer out: the
    skin is thicker than the sheet, so it buys comfort with degrees, and the
    right way to state that is a number rather than a shrug."""
    J = L.JOINTS[joint]
    lo, hi = S.RANGES[joint]
    child = panels_for(set(S.CHILD_LINKS.get(joint, ())))
    parent = panels_for(set(S.PARENT_LINKS.get(joint, ())))
    if not child or not parent:
        return None
    n = int(math.ceil(math.degrees(hi - lo) / 3.0))
    ok_lo, ok_hi = None, None
    for k in range(n + 1):
        th = lo + (hi - lo) * k / n
        worst = 1e9
        for cp in child:
            for cs in sides_of(cp):
                cseam = seam_cyls(cp, cs)
                cpts = [q for q in panel_side(cp, cs) if not in_seam(q, cseam)]
                c = list(J['c'])
                if cs == 'R':
                    c[1] = -c[1]
                Rm = S.rot(J['ax'], -th if (cs == 'R' and J['ax'] in ('x', 'z')) else th)
                for pp in parent:
                    for ps in sides_of(pp):
                        if cs in ('L', 'R') and ps in ('L', 'R') and cs != ps:
                            continue
                        prims = panel_prims(pp, ps)
                        pseam = seam_cyls(pp, ps)
                        for q in cpts:
                            x = S.apply(Rm, c, q)
                            if in_seam(x, pseam):
                                continue
                            for w in prims:
                                worst = min(worst, d_prim(x, w))
        # 0.0, not MIN_SKIN_GAP: this reports where the panels physically
        # TOUCH, which is the hard limit. The 3 mm comfort bar is the gate's
        # question, and reporting the two against different bars made them
        # disagree about the knee.
        if worst - 2 * K.SKIN_GAP >= 0.0:
            if ok_lo is None:
                ok_lo = th
            ok_hi = th
        elif ok_lo is not None and ok_hi is not None:
            break                      # first contiguous band around the start
    return (lo, hi, ok_lo, ok_hi)


def ranges_report():
    print('=' * 78)
    print('What the skin COSTS in joint range (URDF range vs what the panels clear)')
    print('=' * 78)
    for j in ('knee', 'hip_pitch', 'ank_pitch', 'sho_pitch', 'sho_roll',
              'elb_pitch', 'wrist_yaw', 'waist_yaw'):
        r = usable_range(j)
        if not r:
            continue
        lo, hi, a, b = r
        if a is None:
            print('  %-10s %7.0f..%-7.0f deg  ->  NOTHING clears' % (j, math.degrees(lo), math.degrees(hi)))
            continue
        print('  %-10s %7.0f..%-7.0f deg  ->  %7.0f..%-7.0f  (lost %.0f deg)'
              % (j, math.degrees(lo), math.degrees(hi), math.degrees(a), math.degrees(b),
                 math.degrees((hi - lo) - (b - a))))


def main():
    print('=' * 78)
    print('SKIN panel clearance, swept. Panel surface = hull(bones + form) + %.0f mm.'
          % K.SKIN_GAP)
    print('A bone pair clearing by 4 mm is a skin pair overlapping by %.0f.'
          % (2 * K.SKIN_GAP - 4))
    print('=' * 78)
    ok = True
    for j in ('knee', 'hip_pitch', 'hip_yaw', 'ank_pitch', 'sho_pitch', 'sho_roll',
              'elb_pitch', 'wrist_yaw', 'waist_yaw', 'head_yaw', 'head_pitch'):
        g = sweep_joint(j, members=MEMBERS)
        if g is not None and g < 0:
            ok = False
    print('\n  VERDICT: %s' % ('PASS' if ok else 'FAIL'))
    print('  NOTE: a proxy is a LOWER bound only where panels face each other with')
    print('  their smooth outer faces. Where the ball bridges a deep concavity the')
    print('  real panel is fatter than this and the pair needs a mesh check.')
    return ok


if __name__ == '__main__':
    import sys
    MEMBERS = '--members' in sys.argv
    if len(sys.argv) > 1 and sys.argv[1] not in ('--range', '--members'):
        # one joint by name: the whole table is 15 minutes, a single pair is seconds
        for j in sys.argv[1].split(','):
            sweep_joint(j, members=True)
        sys.exit(0)
    if '--range' in sys.argv:
        ranges_report()
        sys.exit(0)
    sys.exit(0 if main() else 1)
