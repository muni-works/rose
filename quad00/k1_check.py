"""checks for the mini humanoid (the K1 on our stack): ankle linkage reach/transmission/collisions, shin
strength, and joint sweeps of both chains against the rest of the robot. Usage: python3 k1_check.py"""
import itertools, math, numpy as np
import quad as Q
from manifold3d import Manifold, Mesh

toM = lambda m: Manifold(Mesh(vert_properties=np.asarray(m.vertices, np.float32), tri_verts=np.asarray(m.faces, np.uint32)))
JOINT = ({'pivot-ankle', 'foot'}, {'shin', 'paw'})


def ankle():
    e = Q.MOTORS['rs00']; W = Q.MOTORS['rs06']['D'] + 7
    DM = max(80.0, (W / 2 + e['D'] / 2 + 4) / math.sin(math.radians(127)))
    G = Q.ankle2_geom(245.0, DM, 1.2 * e['D'], e)
    worst, fails = 1.0, []
    for p, r in itertools.product(range(-20, 51, 5), range(-20, 21, 5)):
        s = Q.ankle2_ik(G, p, r)
        if any(x is None for x in s): fails.append((p, r)); continue
        worst = min(worst, *(x[1] for x in s))
    print(f'ankle (2 x RS00): reach {"all" if not fails else fails} of pitch -20..50 x roll +/-20; worst transmission {worst:.2f}')
    bad = {}
    for rel in (0, 60, 127):
        for p, r in itertools.product((-20, 0, 25, 50), (-20, 0, 20)):
            parts = Q.inline_parts(0, rel, 'ankle', 51, 245, mot='rs06', end_mot='rs00', dual=True, ank=(p - rel, r))
            for n in [n for n in parts if n == 'yoke' or n.endswith(('-abad', '-hip'))]: del parts[n]
            P = {n: toM(g) for n, g in parts.items()}
            h = [(a, b) for a, b in itertools.combinations(P, 2) if (P[a] ^ P[b]).volume() > 5 and not (
                ('shin' in (a, b) and any(x.startswith(('rs0', 'pivot')) for x in (a, b))) or
                (any(x.startswith('crank') for x in (a, b)) and any(x.startswith(('rs0', 'link')) for x in (a, b))) or
                ('foot' in (a, b) and any(x.startswith(('link', 'pivot')) for x in (a, b))))]
            if h: bad[(rel, p, r)] = h
    print('  collisions (knee x pitch x roll):', bad or 'none')


def sweep(label, kws):
    bad = {}
    for kw in kws:
        H = Q.humanoid_k1(**kw)
        sc = Q.build(H); M = {}
        for node in sc.graph.nodes_geometry:
            T, g = sc.graph[node]; m = sc.geometry[g]
            if m.extents.max() >= 1: M[node] = toM(m.copy().apply_transform(T))
        h = sorted({(a.split('.')[0], b.split('.')[0]) for a, b in itertools.combinations(M, 2)
                    if {a.split('.')[0], b.split('.')[0]} not in JOINT and (M[a] ^ M[b]).volume() > 5})
        if h: bad[str(kw)] = h[:4]
    print(f'{label}: clean {len(kws) - len(bad)}/{len(kws)}', '' if not bad else bad)


if __name__ == '__main__':
    ankle()
    sweep('hip pitch (leg pose th)', [dict(leg_pose=(p, 20.0)) for p in (-60, -30, 0, 30, 60, 90)])
    sweep('knee', [dict(knee_bend=k) for k in (5, 40, 80, 120)])
    sweep('shoulder pitch', [dict(arm_pose=(p, 0, 0)) for p in (-150, -90, -30, 30, 60)])
    sweep('shoulder roll (abduction -)', [dict(arm_pose=(0, r, 0)) for r in (-150, -90, -45, -20, 10)])
    sweep('upper-arm yaw', [dict(arm_pose=(0, 0, y)) for y in (-90, -45, 45, 90)])
    sweep('elbow', [dict(elbow=e) for e in (0, 30, 90, 120)])
