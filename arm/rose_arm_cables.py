"""Cables for the rose arm, modelled faithfully enough to find pinches, snags and kinks.

Harness (per arm):
  BUS  one daisy chain, base -> J1 -> J2 ... J7 -> gripper: 2 x 20 AWG (48 V) + CAN twisted pair,
       high-flex PUR jacket, D 5.5 mm. Each RobStride has its connector on the body at the
       STATOR end; the chain plugs in and out there.
  CAM  USB from the wrist camera back to the base, D 4.5 mm, bundled with the bus.
  The bundle envelope is modelled as D 10 mm (two cables side by side).

Routing:
  * Inside a link the bundle lies in the inner-facing slot of the upper inside 2020 bar, under a
    cover strip - enclosed, nothing to snag.
  * The RobStride bodies are solid on axis (checked against the vendor STEP), so nothing passes
    through a joint. Every joint is crossed by a SERVICE LOOP outside the motor: from the motor's
    connector (stator end, on the parent) to a clip on the child link just past the output face,
    both at radius R_motor + 8 mm. The loop has fixed length; its shape at any joint angle is the
    circular arc of that length through the two clips, bulging radially away from the joint axis.
    That is the first-order shape of a cable loop between two clips.

Checks, every one swept over the joint's range and over real + random poses:
  bend   arc radius >= R_BEND (6.5 x the 5.5 mm cable, centreline, dynamic flexing)
  taut   chord never reaches the loop length
  pinch  every loop point clears every body by >= the bundle radius + 2 mm
  snag   how far the loop stands proud of the joint (radial bulge past the motor + clip radius)

Run:  python3 arm/rose_arm_cables.py      (writes sim/out/cables.json)
"""
import json
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rose_arm_layout as A
import rose_arm_geom as Gm
from rose_arm_geom import add, sub, mul, dot, cross, norm, unit

D_BUS, D_CAM = 0.0055, 0.0045
R_BUNDLE = 0.005
R_BEND = 6.5 * D_BUS            # 36 mm
CLIP_GAP = float(os.environ.get('CLIP_GAP', '0.006'))   # clip standoff from the motor skin, default
# per joint, from the stand-off sweep (2026-10-12): the shoulder and base want the loop higher
GAP_J = {'J1_yaw': 0.009, 'J2_sho_pitch': 0.009}
PINCH = R_BUNDLE + 0.002
SLACK = 1.08                    # loop length over its longest chord
SNAG = 0.030                    # a loop standing more than this proud of its clip radius gets a guard

# joint limits the cables are designed to (deg). Where these differ from the mechanical
# limits in rose_arm_export.LIMITS, the cable is the reason - see the report.
LIMITS = {
    'J1_yaw': (-170, 170), 'J2_sho_pitch': (-120, 90), 'J3_upper_roll': (-100, 100),
    'J4_elbow': (-10, 150), 'J5_fore_roll': (-110, 110), 'J6_wrist_pitch': (-120, 120),
    'J7_tool_roll': (-180, 180),
}

JOINT_OF = {j[0]: j for j in A.joints(True)}
OUT_END = {'J2_sho_pitch': 'a'}            # which end of the body is the output face
PARENT = {l[1]: l[2] for l in Gm.LINKS if l[1]}
CHILD = {l[1]: l[0] for l in Gm.LINKS if l[1]}

# direction from the joint axis toward the parent / child link body, zero pose. The clip sits on
# that side so the cable enters its own link immediately. Roll joints get the side the search picks.
DIRS = {
    'J1_yaw': ((0, -1, 0), (0, -1, 0)),
    'J2_sho_pitch': ((0, 0, -1), (1, 0, 0)),
    'J4_elbow': ((-1, 0, 0), (1, 0, 0)),
    'J6_wrist_pitch': ((-1, 0, 0), (1, 0, 0)),
}
ROLL = ('J3_upper_roll', 'J5_fore_roll', 'J7_tool_roll')
ROLL_SIDES = [(0, 0, 1), (0, 0, -1), (0, 1, 0), (0, -1, 0)]


def motor_geom(j):
    label, a, b, r, model = A.seg(JOINT_OF[j])
    out = a if OUT_END.get(j) == 'a' else b
    stat = b if OUT_END.get(j) == 'a' else a
    axis = unit(sub(out, stat))
    return stat, out, axis, r


def clips(j, side=None):
    """(parent clip, child clip) in zero-pose world coords."""
    stat, out, axis, r = motor_geom(j)
    rl = r + R_BUNDLE + GAP_J.get(j, CLIP_GAP)
    if side is None:
        dp, dc = DIRS[j]
    elif isinstance(side[0], (tuple, list)):
        dp, dc = side
    else:
        dp, dc = side, side
    pc = add(add(stat, mul(axis, 0.004)), mul(unit(dp), rl))
    cc = add(add(out, mul(axis, 0.012)), mul(unit(dc), rl))
    return pc, cc


def arc_points(p, c, L, centre_axis_pt, axis, n=24):
    """Circular arc of length L from p to c, bulging away from the joint axis. Returns (pts, R, ok)."""
    ch = sub(c, p)
    cl = norm(ch)
    if cl >= L * 0.999:
        return [p, c], float('inf'), False
    # solve L/cl = th / (2 sin(th/2)) for th in (0, 2 pi)
    lo, hi = 1e-6, 2 * math.pi - 1e-6
    target = L / max(cl, 1e-9)
    for _ in range(80):
        mid = (lo + hi) / 2
        if mid / (2 * math.sin(mid / 2)) < target:
            lo = mid
        else:
            hi = mid
    th = (lo + hi) / 2
    R = L / th
    m = mul(add(p, c), 0.5)
    # bulge: away from the axis line, perpendicular to the chord
    rel = sub(m, centre_axis_pt)
    radial = sub(rel, mul(axis, dot(rel, axis)))
    e1 = unit(ch)
    bdir = sub(radial, mul(e1, dot(radial, e1)))
    if norm(bdir) < 1e-6:
        bdir = cross(e1, axis)
    bdir = unit(bdir)
    # arc centre sits on the far side of the chord from the bulge
    h = R * math.cos(th / 2)
    cen = sub(m, mul(bdir, h))
    a0 = sub(p, cen)
    a0u, b0 = unit(a0), bdir
    pts = []
    # rotate from p to c through the bulge: param angle from start
    start = math.atan2(dot(a0, b0), dot(a0, e1))
    end_v = sub(c, cen)
    end = math.atan2(dot(end_v, b0), dot(end_v, e1))
    # go the long way round through +bulge when th > pi
    da = end - start
    if da > 0:
        da -= 2 * math.pi
    if abs(abs(da) - th) > 1e-3:
        da = (end - start) % (2 * math.pi)
    for k in range(n + 1):
        t = start + da * k / n
        pts.append(add(cen, add(mul(e1, R * math.cos(t)), mul(b0, R * math.sin(t)))))
    return pts, R, True


def loop_length(j, lo, hi, side=None):
    """Fixed loop length: SLACK x the longest chord over the range."""
    pc, cc = clips(j, side)
    stat, out, axis, r = motor_geom(j)
    child = CHILD[j]
    o = Gm.ORIGIN[child]
    worst = 0.0
    for d in range(int(lo), int(hi) + 1, 2):
        R_ = Gm.rot(Gm.AXIS[j], math.radians(d))
        c = add(Gm.mv(R_, sub(cc, o)), o)
        worst = max(worst, norm(sub(c, pc)))
    return SLACK * worst


def link_route(link, entry, exit_):
    """Inside a link: clip -> the inner slot of the upper inside bar -> along it -> out to the next connector."""
    profs = {'upper_a': 'ua_A', 'upper_b': 'ua_B', 'fore_c': 'fa_C', 'fore_d': 'fa_D', 'hand': 'hand'}
    if link in profs:
        prof = {p[0]: p for p in A.profiles(True)}[profs[link]]
        _, (x0, x1), (y0, y1), ends, _ = prof
        H = 2 * max((e[2] if e[0] == 'lobe' else e[1]) for e in ends)
        ys = y0 + A.BONE['knuckle_t'] + 0.010 + 0.016          # just inboard of the inner face of the bar
        zs = A.BASE_H + H / 2 - 0.010 - 0.016
        a = (entry[0], ys, zs)
        b = (exit_[0], ys, zs)
        return [entry, a, b, exit_]
    return [entry, exit_]


def cable_path(q, sides, lengths):
    """Full harness at pose q: list of (kind, joint_or_link, points_world)."""
    T = Gm.fk(q)
    segs = []
    order = ['J1_yaw', 'J2_sho_pitch', 'J3_upper_roll', 'J4_elbow', 'J5_fore_roll', 'J6_wrist_pitch', 'J7_tool_roll']
    base_in = (-0.075, 0.0, 0.004)
    prev_exit = {}
    for j in order:
        pc, cc = clips(j, sides.get(j))
        prev_exit[j] = (pc, cc)
    # base: entry -> J1 connector
    segs.append(('link', 'base', [Gm.xform(T, 'base', p) for p in [base_in, (-0.05, -0.02, 0.006), prev_exit['J1_yaw'][0]]]))
    for i, j in enumerate(order):
        pc, cc = prev_exit[j]
        par, chi = PARENT[j], CHILD[j]
        P = Gm.xform(T, par, pc)
        C = Gm.xform(T, chi, cc)
        stat, out, axis, r = motor_geom(j)
        ax_w = Gm.xdir(T, par, axis)
        ap_w = Gm.xform(T, par, stat)
        pts, R, ok = arc_points(P, C, lengths[j], ap_w, ax_w)
        segs.append(('loop', j, pts, R, ok))
        nxt = order[i + 1] if i + 1 < len(order) else None
        if nxt:
            exit_ = prev_exit[nxt][0]
        else:   # gripper motor connector on the tool
            exit_ = (A.GRIP_X - A.R('EduLite') - 0.006, A.HA_YC, A.BASE_H + 0.01)
        route = link_route(chi, cc, exit_)
        segs.append(('link', chi, [Gm.xform(T, chi, p) for p in route]))
    return segs


def dist_to_bodies(pts, bodies, skip_links=()):
    best = (float('inf'), None)
    for link, mat, P, lo, hi in bodies:
        if link in skip_links:
            continue
        for p in pts:
            # AABB prefilter
            dx = max(lo[0] - p[0], 0, p[0] - hi[0])
            dy = max(lo[1] - p[1], 0, p[1] - hi[1])
            dz = max(lo[2] - p[2], 0, p[2] - hi[2])
            if dx * dx + dy * dy + dz * dz > best[0] ** 2:
                continue
            for b in P:
                d = (p[0] - b[0]) ** 2 + (p[1] - b[1]) ** 2 + (p[2] - b[2]) ** 2
                if d < best[0] ** 2:
                    best = (math.sqrt(d), (link, mat))
    return best


def upstream(link):
    """The link and everything between it and the base."""
    out, par = [], {l[0]: l[2] for l in Gm.LINKS}
    while link:
        out.append(link)
        link = par[link]
    return out


def downstream(link):
    kids = {}
    for n, j, p in Gm.LINKS:
        kids.setdefault(p, []).append(n)
    out, st = [], [link]
    while st:
        x = st.pop()
        out.append(x)
        st += kids.get(x, [])
    return out


def loop_check(q, sides, lengths, j, bodies=None):
    """One joint's loop at pose q: bend radius, taut, pinch distance, bulge."""
    T = Gm.fk(q)
    pc, cc = clips(j, sides.get(j))
    par, chi = PARENT[j], CHILD[j]
    P = Gm.xform(T, par, pc)
    C = Gm.xform(T, chi, cc)
    stat, out, axis, r = motor_geom(j)
    ax_w = Gm.xdir(T, par, axis)
    ap_w = Gm.xform(T, par, stat)
    pts, R, ok = arc_points(P, C, lengths[j], ap_w, ax_w)
    rl = r + R_BUNDLE + GAP_J.get(j, CLIP_GAP)
    bulge = 0.0
    for p in pts:
        rel = sub(p, ap_w)
        rad = norm(sub(rel, mul(ax_w, dot(rel, ax_w))))
        bulge = max(bulge, rad - rl)
    inner = pts[2:-2]
    if bodies is None:
        bodies = Gm.posed_clouds(T)
    side_p, side_c = upstream(par), downstream(chi)
    scissor = float('inf')
    for p in inner:
        dp, _ = dist_to_bodies([p], [b for b in bodies if b[0] in side_p])
        dc, _ = dist_to_bodies([p], [b for b in bodies if b[0] in side_c])
        scissor = min(scissor, max(dp, dc))       # the cable is trapped only if BOTH sides are close
    third, who = dist_to_bodies(inner, [b for b in bodies if b[0] not in (par, chi)])
    rest, _ = dist_to_bodies(inner, [b for b in bodies if b[0] in (par, chi)])
    clear = min(scissor, third)
    return dict(R=R, taut=not ok, clear=clear, scissor=scissor, third=third, rest=rest,
                who=who if third < scissor else ('scissor', ''), bulge=bulge)


def ring_dirs(j, n=8):
    """n unit directions in the plane perpendicular to joint j's axis."""
    ax = Gm.AXIS[j]
    u, v = Gm.perp_basis(ax)
    return [tuple(round(c, 6) for c in add(mul(u, math.cos(2 * math.pi * k / n)), mul(v, math.sin(2 * math.pi * k / n))))
            for k in range(n)]


def choose_roll_sides(lengths_for):
    """For every joint pick the (parent clip, child clip) angles with the best worst case over the
    joint's FULL range: bend radius against R_BEND and pinch clearance against PINCH."""
    sides = {}
    for j in Gm.JOINT_NAMES:
        best = None
        lo, hi = LIMITS[j]
        D = ring_dirs(j)
        cands = [(a, b) for a in D for b in D]
        for sd in cands:
            L = loop_length(j, lo, hi, sd)
            worstR, worstC = float('inf'), float('inf')
            for d in range(int(lo), int(hi) + 1, max(10, (int(hi) - int(lo)) // 9)):
                c = loop_check({j: math.radians(d)}, {j: sd}, {j: L}, j)
                worstR = min(worstR, c['R'])
                worstC = min(worstC, c['clear'])
                if min(worstR / R_BEND, worstC / PINCH) < (best[0] if best else -1):
                    break
            score = min(worstR / R_BEND, worstC / PINCH)
            if best is None or score > best[0]:
                best = (score, sd, L)
        sides[j] = best[1]
    return sides


def sweep(limits, sides, lengths, step=5):
    rows = {}
    for j in Gm.JOINT_NAMES:
        lo, hi = limits[j]
        worst = dict(R=float('inf'), clear=float('inf'), bulge=0.0, taut=False, at_R=None, at_C=None, who=None)
        for d in range(int(lo), int(hi) + 1, step):
            c = loop_check({j: math.radians(d)}, sides, lengths, j)
            if c['R'] < worst['R']:
                worst['R'], worst['at_R'] = c['R'], d
            if c['clear'] < worst['clear']:
                worst['clear'], worst['at_C'], worst['who'] = c['clear'], d, c['who']
            worst['bulge'] = max(worst['bulge'], c['bulge'])
            worst['taut'] |= c['taut']
        rows[j] = worst
    return rows


def sim_poses():
    import csv
    out = []
    d = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sim', 'out')
    for n in ('reach_spring', 'roll_spring', 'cycle_spring', 'short_spring'):
        f = os.path.join(d, n + '.csv')
        if not os.path.exists(f):
            continue
        rows = list(csv.DictReader(open(f)))
        for r in rows[::10]:
            out.append({j: float(r['q_' + j]) for j in Gm.JOINT_NAMES})
    return out


ADJ = None


def self_collides(bodies):
    """Coarse: any two primitives of NON-adjacent links whose boxes overlap by more than 2 mm."""
    global ADJ
    if ADJ is None:
        par = {l[0]: l[2] for l in Gm.LINKS}
        ADJ = {(a, b) for a in par for b in par if a == b or par.get(a) == b or par.get(b) == a
               or par.get(par.get(a)) == b or par.get(par.get(b)) == a}
    for i, (la, _, _, loa, hia) in enumerate(bodies):
        for lb, _, _, lob, hib in bodies[i + 1:]:
            if (la, lb) in ADJ:
                continue
            if all(min(hia[k], hib[k]) - max(loa[k], lob[k]) > 0.002 for k in range(3)):
                return True
    return False


def pose_check(poses, sides, lengths):
    """Every cable point (loops and in-link runs) against every body of OTHER links, per pose.
    Poses where the arm collides with itself are skipped - the arm cannot be there."""
    worst = (float('inf'), None, None)
    global N_VALID
    N_VALID = 0
    for q in poses:
        T = Gm.fk(q)
        bodies = Gm.posed_clouds(T)
        if self_collides(bodies):
            continue
        N_VALID += 1
        for seg in cable_path(q, sides, lengths):
            kind, name, pts = seg[0], seg[1], seg[2]
            if kind == 'link':
                skip = (name,)
                pts = pts[1:-1] or pts
            else:          # a loop rests on its own two links by design; scissor is checked per joint
                skip = (PARENT[name], CHILD[name])
                pts = pts[2:-2]
            d, who = dist_to_bodies(pts, bodies, skip)
            if d < worst[0]:
                worst = (d, (kind, name, who), {k: round(math.degrees(v)) for k, v in q.items()})
    return worst


def loop_vs_world(j, sd, L, poses_bodies):
    """Worst clearance of joint j's loop to links other than its own two, over pre-posed bodies."""
    worst = float('inf')
    for q, T, bodies in poses_bodies:
        pc, cc = clips(j, sd)
        par, chi = PARENT[j], CHILD[j]
        P = Gm.xform(T, par, pc)
        C = Gm.xform(T, chi, cc)
        stat, out, axis, r = motor_geom(j)
        pts, R, ok = arc_points(P, C, L, Gm.xform(T, par, stat), Gm.xdir(T, par, axis))
        d, _ = dist_to_bodies(pts[2:-2], [b for b in bodies if b[0] not in (par, chi)])
        worst = min(worst, d)
        if worst < PINCH:
            break
    return worst


def refine(j, sides, poses_bodies):
    """Re-pick joint j's clip angles so the loop passes its own sweep AND clears the rest of the arm."""
    lo, hi = LIMITS[j]
    D = ring_dirs(j)
    best = None
    for sd in [(a, b) for a in D for b in D]:
        L = loop_length(j, lo, hi, sd)
        wR, wC = float('inf'), float('inf')
        for d in range(int(lo), int(hi) + 1, max(10, (int(hi) - int(lo)) // 12)):
            c = loop_check({j: math.radians(d)}, {j: sd}, {j: L}, j)
            wR, wC = min(wR, c['R']), min(wC, c['clear'])
        own = min(wR / R_BEND, wC / PINCH)
        if own < 1.0 or (best and own < best[0]):
            continue
        world = loop_vs_world(j, sd, L, poses_bodies)
        score = min(own, world / PINCH)
        if best is None or score > best[0]:
            best = (score, sd, L)
    if best:
        sides[j] = best[1]
    return best


def random_poses(n, limits, seed=7):
    rnd = random.Random(seed)
    return [{j: math.radians(rnd.uniform(*limits[j])) for j in Gm.JOINT_NAMES} for _ in range(n)]


def report(title, limits, sides, lengths, rows):
    print('\n=== %s ===' % title)
    print('  %-15s %-12s %6s %8s %8s %8s  %s' % ('joint', 'range', 'loop', 'min R', 'clear', 'bulge', 'verdict'))
    bad = []
    for j in Gm.JOINT_NAMES:
        w = rows[j]
        lo, hi = limits[j]
        v = []
        if w['taut']:
            v.append('TAUT')
        if w['R'] < R_BEND:
            v.append('kinks (R %.0f at %d deg)' % (w['R'] * 1e3, w['at_R']))
        if w['clear'] < PINCH:
            v.append('PINCH %.1f mm (%s) at %d deg' % (w['clear'] * 1e3, w['who'][0] if w['who'] else '?', w['at_C']))
        if w['bulge'] > SNAG:
            v.append('stands %.0f mm proud' % (w['bulge'] * 1e3))
        if v:
            bad.append(j)
        print('  %-15s %4d..%-4d  %4.0f mm %6.0f mm %6.1f mm %6.0f mm  %s' % (
            j, lo, hi, lengths[j] * 1e3, w['R'] * 1e3, w['clear'] * 1e3, w['bulge'] * 1e3, ', '.join(v) or 'ok'))
    return bad


def main():
    print('bundle D %.0f mm, bend limit R %.0f mm (6.5 x bus D), pinch limit %.0f mm, loop slack %.2f'
          % (2 * R_BUNDLE * 1e3, R_BEND * 1e3, PINCH * 1e3, SLACK))
    sides = choose_roll_sides(None)
    lengths = {j: loop_length(j, *LIMITS[j], sides.get(j)) for j in Gm.JOINT_NAMES}
    rows = sweep(LIMITS, sides, lengths)
    bad = report('full mechanical ranges, clip angles searched', LIMITS, sides, lengths, rows)

    # cable-driven limits: shrink each failing joint's range symmetrically about its centre (and
    # for the elbow, to one-sided folding) until it passes
    lim2 = dict(LIMITS)
    for j in []:
        lo, hi = lim2[j]
        for _ in range(20):
            L = loop_length(j, lo, hi, sides.get(j))
            lengths[j] = L
            r = sweep({j: (lo, hi)}, sides, {**lengths, j: L}, step=5) if False else None
            w = dict(R=float('inf'), clear=float('inf'), taut=False)
            for d in range(int(lo), int(hi) + 1, 5):
                c = loop_check({j: math.radians(d)}, sides, {**lengths, j: L}, j)
                w['R'] = min(w['R'], c['R'])
                w['clear'] = min(w['clear'], c['clear'])
                w['taut'] |= c['taut']
            if w['R'] >= R_BEND and w['clear'] >= PINCH and not w['taut']:
                break
            mid = (lo + hi) / 2
            span = max(20, (hi - lo) - 20)
            lo, hi = int(round(mid - span / 2)), int(round(mid + span / 2))
        lim2[j] = (lo, hi)
    lengths2 = {j: loop_length(j, *lim2[j], sides.get(j)) for j in Gm.JOINT_NAMES}
    rows2 = rows
    bad2 = []

    poses = sim_poses() + random_poses(300, lim2)
    valid = []
    for q in poses:
        T = Gm.fk(q)
        b = Gm.posed_clouds(T)
        if not self_collides(b):
            valid.append((q, T, b))
    for it in range(4):
        w = pose_check(poses, sides, lengths2)
        if w[0] >= PINCH or w[1][0] != 'loop':
            break
        j = w[1][1]
        print('  harness: %s loop %.1f mm from %s - re-picking its clips against %d reachable poses'
              % (j, w[0] * 1e3, w[1][2][0], len(valid)))
        b = refine(j, sides, valid)
        lengths2[j] = loop_length(j, *LIMITS[j], sides.get(j))
        print('    -> score %.2f' % (b[0] if b else float('nan')))
    rows2 = sweep(LIMITS, sides, lengths2)
    report('after harness refinement', LIMITS, sides, lengths2, rows2)
    w = pose_check(poses, sides, lengths2)
    print('\n=== whole harness over %d reachable poses (of %d sim + random) ===' % (N_VALID, len(poses)))
    print('  closest any cable comes to another link: %.1f mm  (%s %s vs %s)' % (
        w[0] * 1e3, w[1][0], w[1][1], w[1][2][0] if w[1][2] else '?'))
    print('  at pose %s' % w[2])
    print('  %s' % ('PINCH' if w[0] < PINCH else 'clear'))

    out = dict(sides={j: sides[j] for j in sides}, lengths=lengths2, limits=lim2,
               rows={j: {k: v for k, v in rows2[j].items() if k != 'who'} for j in rows2},
               pose_worst=dict(d=w[0], where=[w[1][0], w[1][1], w[1][2][0] if w[1][2] else None], pose=w[2]),
               full_ranges_failing=bad, cable_ranges_failing=bad2)
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sim', 'out', 'cables_gap%02d.json' % round(CLIP_GAP * 1e3))
    json.dump(out, open(path, 'w'), indent=2, default=str)


if __name__ == '__main__':
    main()
