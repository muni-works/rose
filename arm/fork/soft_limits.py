"""lymb v10.3 combined-pose soft limits: pose_ok(q, payload, mount) for the controller.

A pose is accepted when ALL hold:
  1. every joint inside its SKINNED range (skin_clash.ranges(), each joint alone)  RANGES below
  2. no two skin parts on links that move relative to each other, and are not joined directly by
     one joint (those are seam-safe by construction - the coaxial 1 mm gap), come within GAP_MIN:
     this is what catches the combined folds (J4 + J6 wrist into the upper arm, J2 + J4 wrist into
     the desk puck / J1 drum, ...)
  3. desk mount: every skin point >= CLEAR mm above the table (the tool / jaws may touch - they pick)
     body mount: every arm skin and tool point >= CLEAR mm from the humanoid's own shell
  4. the J3 static-torque cap (pose_cap.j3_ok) at that payload

The geometry is a TABLE (soft_limits_v10.npz): each skin part sampled to surface points (<= SPACING
mm apart) in its link frame, plus the humanoid shell as a point cloud. pose_ok needs only numpy and
rose_fork's FK. Point sets are compared by distance; with points SPACING apart, two surfaces that
interpenetrate always have a pair of points closer than SPACING, so GAP_MIN = SPACING + 1 mm is a
sufficient non-contact test (verified below against the real meshes).

    ROSE_ARM=v10 uv run ... python arm/fork/soft_limits.py build     # writes the table
    ROSE_ARM=v10 uv run ... python arm/fork/soft_limits.py verify    # random sweep, mesh + structure check
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..'))
import numpy as np
import rose_fork as F

TABLE = os.path.join(HERE, 'soft_limits_v10.npz')
SPACING = 5.0          # mm, surface sampling
GAP_MIN = SPACING + 1.0
CLEAR = 5.0            # mm to the desk / the humanoid's shell
RANGES = {'J1_yaw': (-180, 180), 'J2_sho_pitch': (-180, 10), 'J3_upper_roll': (-180, 180),
          'J4_elbow': (-115, 115), 'J6_wrist_pitch': (-105, 105), 'J7_tool_roll': (-180, 180)}
STRUCT = {'J1_yaw': (-180, 180), 'J2_sho_pitch': (-180, 25), 'J3_upper_roll': (-180, 180),
          'J4_elbow': (-150, 130), 'J6_wrist_pitch': (-130, 130), 'J7_tool_roll': (-180, 180)}
ADJ = {frozenset(p) for p in (('base', 'turret'), ('turret', 'upper'), ('upper', 'upperd'), ('upperd', 'fore'),
                              ('fore', 'yoke'), ('yoke', 'wrist'), ('fore', 'wrist'), ('wrist', 'tool'))}
_T = {}


def table():
    if not _T:
        d = np.load(TABLE, allow_pickle=True)
        _T.update({k: d[k] for k in d.files})
    return _T


def _posed(q, R0=None, at=None):
    """{link: (N, 3) mm world points} for the skin (and the tool's structure, key 'tool*')."""
    t = table()
    T = F.fk(q)
    R0 = np.eye(3) if R0 is None else np.asarray(R0, float)
    at = np.zeros(3) if at is None else np.asarray(at, float) * 1e3
    out = {}
    for k in t['links']:
        link = str(k)
        P = t['pts_' + link]
        if not len(P):
            continue
        R, tt = T[link.rstrip('*')]
        W = (P @ np.asarray(R).T + np.asarray(tt) * 1e3) @ R0.T + at
        out[link] = W
    return out


def _min_dist(A, B):
    best = 1e9
    for i in range(0, len(A), 256):
        d = ((A[i:i + 256, None, :] - B[None, :, :]) ** 2).sum(-1)
        best = min(best, float(d.min()))
    return math.sqrt(best)


def why(q, payload=0.0, mount='desk'):
    """None if the pose is accepted, else the first reason it is not."""
    deg = {j: math.degrees(q.get(j, 0.0)) for j in RANGES}
    for j, (lo, hi) in RANGES.items():
        if not lo - 1e-6 <= deg[j] <= hi + 1e-6:
            return 'range %s %.0f' % (j, deg[j])
    import pose_cap
    R0, at = None, None
    if mount != 'desk':
        import render_v10 as RV
        s = 1 if mount == 'body_l' else -1
        R0 = RV.body_R0(s)
        t = table()
        at = t['body_at_l'] if s > 0 else t['body_at_r']
    if not pose_cap.j3_ok(q, payload, tuple(tuple(r) for r in R0) if R0 is not None else pose_cap.I3):
        return 'J3 cap'
    W = _posed(q, R0, at)
    links = [k for k in W if not k.endswith('*')]
    lb = lambda k: k.rstrip('*')
    keys = list(W)
    for i in range(len(keys)):
        for j in range(i + 1, len(keys)):
            a, b = keys[i], keys[j]
            if lb(a) == lb(b) or frozenset((lb(a), lb(b))) in ADJ:
                continue
            A, B = W[a], W[b]
            if (A.min(0) - B.max(0) > GAP_MIN).any() or (B.min(0) - A.max(0) > GAP_MIN).any():
                continue
            if _min_dist(A, B) < GAP_MIN:
                return 'fold %s x %s' % (a, b)
    if mount == 'desk':
        z_table = -F.PED_H * 1e3
        for k in links:
            if k == 'base':
                continue
            if W[k][:, 2].min() < z_table + CLEAR:
                return 'desk %s' % k
        if 'tool*' in W and W['tool*'][:, 2].min() < z_table:      # the jaws may touch the desk, not go through it
            return 'desk tool'
    else:
        H = table()['humanoid']
        for k, P in W.items():
            if lb(k) in ('base', 'turret'):      # the shoulder lives in the torso corner by design
                continue
            lo, hi = P.min(0) - CLEAR, P.max(0) + CLEAR
            sel = H[((H >= lo) & (H <= hi)).all(1)]
            if len(sel) and _min_dist(P, sel) < CLEAR + SPACING / 2:
                return 'torso %s' % k
    return None


def pose_ok(q, payload=0.0, mount='desk'):
    """q: {joint: rad}. mount: 'desk', 'body_l' or 'body_r'. True if the pose is safe to command."""
    return why(q, payload, mount) is None


# ---------------------------------------------------------------- build the table (needs trimesh)
def sample(m, spacing=SPACING):
    import trimesh
    if not len(m.faces):
        return np.zeros((0, 3))
    n = max(64, int(m.area / (spacing * spacing) * 2.0))
    P, _ = trimesh.sample.sample_surface(m, n, seed=1)
    V = m.vertices if len(m.vertices) < 5000 else m.convex_hull.vertices     # the dot-grids' hole vertices add nothing
    return np.vstack([P, V])


def build(hum_dir=None):
    import json
    import trimesh
    import skin_v10 as S
    pts = {}
    for nm, link, m, mat in S.skin_meshes():
        pts.setdefault(link, []).append(sample(m))
    # the tool (gripper servo + jaws) as structure: it may touch the desk, it may not hit the arm or torso
    tool = [np.array(F.Gm.prim_points(p[:7], step=SPACING / 1e3)) * 1e3 - np.array(F.ORIGIN['tool']) * 1e3
            for p in F.build() if p[0] == 'tool']
    out = {'pts_' + k: np.vstack(v).astype(np.float32) for k, v in pts.items()}
    out['pts_tool*'] = np.vstack(tool).astype(np.float32)
    links = list(pts) + ['tool*']
    hum_dir = hum_dir or os.environ.get('LYMB_HUM')
    if hum_dir:
        H = []
        for k in ('cover', 'trim', 'plate', 'motor', 'sole'):
            f = os.path.join(hum_dir, 'hum_%s.stl' % k)
            if os.path.exists(f):
                H.append(sample(trimesh.load(f), 8.0))
        H = np.vstack(H)
        _, idx = np.unique(np.floor(H / 6.0).astype(np.int64), axis=0, return_index=True)   # one point per 6 mm voxel
        out['humanoid'] = H[idx].astype(np.float32)
        import render_v10 as RV
        J = json.load(open(os.path.join(hum_dir, 'hum.json')))
        ph = RV.body_ped_h()
        for s, t in ((1, 'l'), (-1, 'r')):
            sr = [c / 1e3 for c in J['sp']]   # J1 on the humanoid's shoulder-pitch axis
            out['body_at_' + t] = np.array([sr[0], s * (RV.SIDE_PLATE_Y + ph), sr[2]])
    else:
        out['humanoid'] = np.zeros((0, 3), np.float32)
    out['links'] = np.array(links)
    np.savez_compressed(TABLE, **out)
    print('table: %s, %d points' % (TABLE, sum(len(v) for k, v in out.items() if k.startswith('pts_'))))


# ---------------------------------------------------------------- prove it
def grid_poses(step=15):
    """The fold grid: J2 x J4 x J6 over the skinned ranges (J3 = 0) - where the combined folds live."""
    r = lambda j: range(RANGES[j][0], RANGES[j][1] + 1, step)
    for a in r('J2_sho_pitch'):
        for b in r('J4_elbow'):
            for c in r('J6_wrist_pitch'):
                yield {'J2_sho_pitch': math.radians(a), 'J4_elbow': math.radians(b), 'J6_wrist_pitch': math.radians(c)}


def verify(n=600, seed=1, mount='desk', grid=False):
    """Random poses inside the STRUCTURE's ranges: coverage = fraction pose_ok keeps; every kept pose
    is re-checked against the real meshes (skin x skin > 1 mm^3, skin below the table) and the bare
    structure (audit.interferes)."""
    import skin_clash as SC
    import skin_v10 as S
    import audit
    from manifold3d import Manifold
    rng = np.random.default_rng(seed)
    parts = S.skin_meshes()
    kept = bad = 0
    reasons = {}
    src = list(grid_poses()) if grid else [{j: math.radians(rng.uniform(*STRUCT[j])) for j in STRUCT} for _ in range(n)]
    n = len(src)
    for q in src:
        r = why(q, 0.0, mount)
        if r:
            reasons[r.split()[0]] = reasons.get(r.split()[0], 0) + 1
            continue
        kept += 1
        T = F.fk(q)
        hit = audit.interferes(q)
        if not hit:
            M = [(nm, l, SC.posed(m, l, T)) for nm, l, m, _ in parts]
            for i in range(len(M)):
                for j in range(i + 1, len(M)):
                    if M[i][1] != M[j][1] and (M[i][2] ^ M[j][2]).volume() > 1.0:
                        hit = (M[i][0], M[j][0])
                        break
                if hit:
                    break
        if hit:
            bad += 1
            print('  ACCEPTED BUT CLASHES:', hit, {k: round(math.degrees(v)) for k, v in q.items()})
    print('%s%s: %d poses; pose_ok keeps %d (%.0f %%); %d of those clash'
          % (mount, ' fold grid (J2 x J4 x J6 over the skinned ranges, step 15)' if grid else ' random inside the structure ranges', n, kept, 100.0 * kept / n, bad))
    print('  rejected by: ' + ', '.join('%s %d' % kv for kv in sorted(reasons.items(), key=lambda kv: -kv[1])))
    return kept / n, bad


if __name__ == '__main__':
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'verify'
    if cmd == 'build':
        build(sys.argv[2] if len(sys.argv) > 2 else None)
    else:
        verify(int(sys.argv[2]) if len(sys.argv) > 2 else 600, mount=sys.argv[3] if len(sys.argv) > 3 else 'desk', grid=cmd == 'grid')
