"""mini arm checks: the leg-chain arm over shoulder pitch x elbow, shoulder yaw and roll, and the 2-rod wrist."""
import itertools, math, numpy as np, trimesh
import quad as Q
from manifold3d import Manifold, Mesh

toM = lambda m: Manifold(Mesh(vert_properties=np.asarray(m.vertices, np.float32), tri_verts=np.asarray(m.faces, np.uint32)))
KW = dict(leg=(230., 300.), arm=(205., 230.), chest_h=120., legm='rs06', armm='rs06', yawm='rs02', waistm='rs06', wristm='rs00',
          ankle_roll=True, head='rs00', legpk='rs04', rect=True, arm_chain=True, arm_yawm='rs02')
JOINT = ({'pivot-ankle', 'foot'}, {'pivot-wrist', 'hand'})
ARM = ('thigh', 'shin', 'yoke', 'rs06-abad', 'rs06-hip', 'rs06-knee', 'rs00-wrist', 'crank-wrist', 'link-wrist', 'pivot-wrist', 'hand')


def split(H):
    ex = dict(H['extra'])
    arm = {n: m for n, m in ex.items() if n.endswith('.al') and n.split('.')[0].startswith(ARM)}
    rest = {n: m for n, m in ex.items() if n not in arm and not n.endswith('.ar')}
    return arm, rest


def hits(arm, rest, R=np.eye(4)):
    A = {n: toM(m.copy().apply_transform(R)) for n, m in arm.items()}
    B = {n: toM(m) for n, m in rest.items()}
    return sorted({(a.split('.')[0], b.split('.')[0]) for a in A for b in B if {a.split('.')[0], b.split('.')[0]} not in JOINT and (A[a] ^ B[b]).volume() > 5})


if __name__ == '__main__':
    bad = {}
    for th, rel in itertools.product((-60, -30, 0, 30, 60, 90, 120, 150), (0, 40, 80, 120)):
        arm, rest = split(Q.humanoid_family('m', arm_pose=(th, rel), **KW))
        h = hits(arm, rest)
        if h: bad[(th, rel)] = h
    print('shoulder pitch x elbow:', bad or 'clean')
    H = Q.humanoid_family('m', arm_pose=(10, 60), **KW)
    arm, rest = split(H)
    ys = Q.MOTORS['rs06']['D'] / 2
    # yaw axis: vertical (body x) through the shoulder yaw motor; roll axis: fore-aft (body z) through the module's ab/ad
    ext = dict(H['extra'])
    yc = ext['rs02-shoulderyaw.al'].centroid
    rc = ext['rs06-abad.al'].centroid
    mov = ('chassis-shoulderflange', 'chassis-shoulderplate', 'rs02-shoulderyaw-out')
    arm_y = dict(arm, **{n: m for n, m in rest.items() if n.endswith('.al') and n.split('.')[0] in mov})
    rest_y = {n: m for n, m in rest.items() if n not in arm_y}
    for yaw in (-45, -20, 20, 45):
        h = hits(arm_y, rest_y, trimesh.transformations.rotation_matrix(np.radians(yaw), [1, 0, 0], yc))
        print('shoulder yaw', yaw, h or 'clean')
    for roll in (-90, -60, -45, -30, -20, 10, 20):
        h = hits(arm, rest, trimesh.transformations.rotation_matrix(np.radians(roll), [0, 0, 1], rc))
        print('shoulder roll', roll, h or 'clean')
    e = Q.MOTORS['rs00']; W = Q.MOTORS['rs06']['D'] + 7
    DM = max(80.0, (W / 2 + e['D'] / 2 + 4) / math.sin(math.radians(127))) + 20.0
    G = Q.ankle2_geom(230.0, DM, 1.0 * e['D'], e, beta=-5.0)
    worst, fails = 1.0, []
    for p, r in itertools.product(range(-45, 46, 5), range(-20, 21, 5)):
        s = Q.ankle2_ik(G, p, r)
        if any(x is None for x in s): fails.append((p, r)); continue
        worst = min(worst, *(x[1] for x in s))
    print(f'wrist: reach {"all" if not fails else fails} of pitch +/-45 x yaw +/-20, worst transmission {worst:.2f}')
    bad = {}
    for rel in (0, 60, 120):
        for p, r in itertools.product((-45, 0, 45), (-20, 0, 20)):
            parts = Q.inline_parts(10, rel, 'wrist', 205, 230, mot='rs06', end_mot='rs00', dual=True, ank=(p, r))
            P = {n: toM(g) for n, g in parts.items()}
            h = [(a, b) for a, b in itertools.combinations(P, 2) if (P[a] ^ P[b]).volume() > 5 and not (
                ('shin' in (a, b) and any(x.startswith(('rs0', 'pivot')) for x in (a, b))) or
                (any(x.startswith('crank') for x in (a, b)) and any(x.startswith(('rs0', 'link')) for x in (a, b))) or
                ('hand' in (a, b) and any(x.startswith(('link', 'pivot')) for x in (a, b))))]
            if h: bad[(rel, p, r)] = h
    print('wrist collisions (elbow x pitch x yaw):', bad or 'none')
