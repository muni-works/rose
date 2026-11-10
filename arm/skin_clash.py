"""Skin clash check for the v10.2 skin (skin_v10.skin_parts): every pair of skin parts on links that move
relative to each other, swept over only the joints between them (the exterior session's clash.py rule),
at the structure's own joint ranges (audit.joint_range), counting a clash as > 1 mm^3 of shared volume.
Pairs more than two joints apart are skipped, except the desk puck against the wrist and forearm.

    ROSE_ARM=v10 uv run -q --with numpy --with manifold3d --with trimesh --with scipy python arm/skin_clash.py
"""
import itertools
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np
from manifold3d import Manifold
import rose_fork as F
import skin_v10 as S

ORDER = [l[0] for l in F.LINKS]
JOINT = {l[0]: l[1] for l in F.LINKS}
LIM = {'J1_yaw': (-180, 180, 45), 'J2_sho_pitch': (-180, 25, 15), 'J3_upper_roll': (-180, 180, 45),
       'J4_elbow': (-150, 130, 10), 'J6_wrist_pitch': (-130, 130, 10), 'J7_tool_roll': (-180, 180, 45)}


def chain(la, lb):
    i, j = sorted((ORDER.index(la), ORDER.index(lb)))
    return [JOINT[ORDER[k]] for k in range(i + 1, j + 1) if JOINT[ORDER[k]] in LIM]


def posed(m, link, T):
    R, t = T[link]
    V = (m.vertices - 0.0) @ np.array(R).T + np.array(t) * 1e3
    return Manifold(S._mm(type(m)(V, m.faces, process=False)))


def main():
    parts = S.skin_meshes()
    pairs = []
    for (na, la, ma, _), (nb, lb, mb, _) in itertools.combinations(parts, 2):
        if la == lb:
            continue
        js = chain(la, lb)
        far = {'base'} & {la, lb} and {'fore', 'wrist'} & {la, lb}
        if not js or (len(js) > 2 and not far):
            continue
        if far:
            js = ['J2_sho_pitch', 'J4_elbow']
        pairs.append((na, la, ma, nb, lb, mb, js))
    print('%d pairs' % len(pairs))
    bad = 0
    for na, la, ma, nb, lb, mb, js in pairs:
        grids = [range(LIM[j][0], LIM[j][1] + 1, LIM[j][2]) for j in js]
        worst = (0.0, None)
        for combo in itertools.product(*grids):
            q = {j: math.radians(a) for j, a in zip(js, combo)}
            T = F.fk(q)
            v = (posed(ma, la, T) ^ posed(mb, lb, T)).volume()
            if v > worst[0]:
                worst = (v, combo)
        if worst[0] > 1.0:
            bad += 1
            print('  CLASH %-10s (%s) x %-10s (%s): %.0f mm^3 at %s = %s' % (na, la, nb, lb, worst[0], js, worst[1]))
    print('%d clashing pairs' % bad)


def ranges():
    """Each joint alone (others at zero): the contiguous range round 0 the skin allows, vs the structure's."""
    parts = S.skin_meshes()
    for j, (lo, hi, _) in LIM.items():
        rel = [(a, b) for a, b in itertools.combinations(parts, 2) if a[1] != b[1] and j in chain(a[1], b[1])]
        def ok(d):
            T = F.fk({j: math.radians(d)})
            return all((posed(a[2], a[1], T) ^ posed(b[2], b[1], T)).volume() <= 1.0 for a, b in rel)
        a_ = 0
        while a_ - 5 >= lo and ok(a_ - 5):
            a_ -= 5
        b_ = 0
        while b_ + 5 <= hi and ok(b_ + 5):
            b_ += 5
        print('  %-15s skin %5d .. %-5d  (structure %d .. %d)' % (j, a_, b_, lo, hi))


if __name__ == '__main__':
    main()
    print('\nranges with the skin on, each joint alone:')
    ranges()
