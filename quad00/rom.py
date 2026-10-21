"""humanoid range-of-motion sweep: intersection volumes between limbs and chassis (body frame, x = spine up)."""
import sys, itertools, numpy as np, trimesh
sys.path.insert(0, sys.argv[1])
import quad as Q, leg as L
from manifold3d import Manifold, Mesh
H = Q.humanoid5()
chassis = {n: m for n, m in H['extra'] if n.startswith('chassis')}
hip_a = H['hip_x']; y0 = H['plane']; y0l = L.RS00_D / 2 + 3.0 - Q.IL_ABAD
wall = hip_a - Q.AB_OUT
hip_x = -wall - 2 * L.T - 40.0
Tl = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1.0]])
rx = lambda d: trimesh.transformations.rotation_matrix(np.radians(d), [1, 0, 0], [0, Q.IL_ABAD, 0])
def M(t):
    return Manifold(Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)))
def limb(kind, s, th, rel, ab):
    out = {}
    for n, g in Q.inline_parts(th, rel).items():
        g = g.copy().apply_transform(rx(ab))
        if kind == 'arm':
            m = trimesh.transformations.translation_matrix([hip_a, s * y0, 0]) @ np.diag([1, s, 1, 1.0])
            if n in Q.YOKE_SIDE:
                m = m @ np.diag([-1, 1, 1, 1.0])
        else:
            m = trimesh.transformations.translation_matrix([hip_x, s * y0l, 0]) @ np.diag([1, s, 1, 1.0]) @ Tl
        out[n] = M(g.apply_transform(m))
    return out
CH = {n: M(m) for n, m in chassis.items()}
def hit(A, B, skip=()):
    r = []
    for (a, ma), (b, mb) in itertools.product(A.items(), B.items()):
        if (a, b) in skip: continue
        v = (ma ^ mb).volume()
        if v > 2: r.append((a, b, round(v)))
    return r
own = {('rs00-abad', 'chassis-pelvis'), ('rs00-abad', 'chassis-nose'), ('yoke', 'chassis-pelvis'), ('yoke', 'chassis-nose')}
def report(tag, kind, grid):
    bad = {}
    for th, rel, ab in grid:
        for s in (1,):
            h = hit(limb(kind, s, th, rel, ab), CH)
            h = [x for x in h if (x[0], x[1]) not in own]
            if h: bad[(th, rel, ab)] = h
    print(tag, len(bad), '/', len(grid))
    for k, v in list(bad.items())[:int(sys.argv[2]) if len(sys.argv) > 2 else 12]:
        print('  ', k, v)
    return bad
if __name__ == '__main__':
    # own-interface sanity (rest pose): which parts touch at all
    print('rest leg vs chassis', hit(limb('leg', 1, *Q.stance(0.96 * 300), 0), CH))
    print('rest arm vs chassis', hit(limb('arm', 1, 100, 70, 0), CH))
    print('leg vs leg rest', hit(limb('leg', 1, *Q.stance(0.96 * 300), 0), limb('leg', -1, *Q.stance(0.96 * 300), 0)))
    print('arm vs leg rest', hit(limb('arm', 1, 100, 70, 0), limb('leg', 1, *Q.stance(0.96 * 300), 0)))
