"""mini humanoid range-of-motion sweeps: rotate each joint's subtree over the K1's range, report where it first collides.

Bodies are the rigid groups between joints (shells + plates + motor cans). A pose collides when the moving subtree's
overlap with everything else grows past its overlap at the zero pose (adjacent drums overlap coaxially by design)."""
import re
import numpy as np, trimesh
from manifold3d import Manifold, Mesh, OpType
import k1_design as KD, k1_structure as KS

THRESH = 200.0                                  # mm^3 of new overlap = a collision

# body -> name prefixes ({t} = side tag); the order of CHAIN is parent -> child
BODIES = {
    'wroll': ['waist-W2', 'waist-W3', 'motor-waistpitch', 'cover-waistroll'],
    'wyaw': ['waist-W0', 'waist-W1', 'motor-waistroll', 'cover-waist'],
    'torso': ['cover-pack', 'pelvis-deck', 'motor-waistyaw', 'waist-W4', 'trim-waisthub', 'elec-', 'estop-', 'cover-chest', 'accent-waist', 'cover-abdomen', 'cover-pelvis', 'trim-neckbase', 'motor-roll.', 'chassis-rollmount.', 'torso-', 'motor-sp.', 'motor-headyaw'],
    'roll.{t}': ['chassis-hipR.{t}l', 'cover-hipguard.{t}l', 'chassis-hipinner.{t}l', 'motor-pitch.{t}l', 'cover-hip.{t}l', 'puck-hip.{t}l'],
    'pitch.{t}': ['chassis-hipA.{t}l', 'chassis-hipB2.{t}l', 'motor-yaw.{t}l', 'cover-hipyaw.{t}l', 'cover-hipneck.{t}l'],
    'yaw.{t}': ['chassis-thightop.{t}l', 'thigh-', 'motor-knee.{t}l', 'cover-thigh.{t}l', 'cover-knee.{t}l', 'puck-knee.{t}l', 'trim-yawring.{t}l'],
    'knee.{t}': ['shin-', 'motor-ankle', 'cover-calf.{t}l', 'trim-ankle.{t}l'],
    'ankle.{t}': ['cover-shoe.{t}l', 'sole.{t}l', 'node-foot.{t}l'],
    'sp.{t}': ['arm-yokeP', 'motor-sr.{t}a', 'cover-shoulderyoke.{t}a', 'cover-shoulder.{t}a', 'puck-shoulder.{t}a'],
    'sr.{t}': ['arm-upperU', 'motor-ep.{t}a', 'cover-upperarm.{t}a', 'cover-deltoid.{t}a', 'trim-upperarm.{t}a'],
    'ep.{t}': ['arm-elbowE', 'motor-ey.{t}a', 'cover-elbowfork.{t}a', 'cover-elbow.{t}a', 'puck-elbow.{t}a'],
    'wr.{t}': ['wrist-R0', 'wrist-R1', 'motor-wristpitch', 'cover-wristsleeve', 'trim-wristpitch'],
    'wp.{t}': ['wrist-P0', 'wrist-P1', 'motor-wristyaw'],
    'wy.{t}': ['wrist-H0'],
    'ey.{t}': ['wrist-F2', 'motor-wristroll', 'arm-forearmF1', 'cover-forearm.{t}a', 'cover-hand.{t}a', 'trim-wrist.{t}a'],
    'hyaw': ['neck-N', 'motor-headpitch', 'cover-neck', 'cover-headhub'],
    'hpitch': ['head-H1', 'cover-helmet', 'visor-face', 'puck-ear'],
}
LEG = ['roll', 'pitch', 'yaw', 'knee', 'ankle']
ARM = ['sp', 'sr', 'ep', 'ey', 'wr', 'wp', 'wy']

# what a K1-class walker actually uses (deg, K1 sign convention); the sweep reports against these
NEED = {'Hip_Roll': (-15, 40), 'Hip_Pitch': (-100, 30), 'Hip_Yaw': (-45, 45), 'Knee_Pitch': (0, 120),
        'Ankle_Pitch': (-45, 20), 'Ankle_Roll': (-20, 20), 'Shoulder_Pitch': (-180, 60), 'Shoulder_Roll': (-10, 90),
        'Elbow_Pitch': (-90, 90), 'Elbow_Yaw': (-130, 0), 'Head_yaw': (-57, 57), 'Head_pitch': (-20, 40),
        'Wrist_roll': (-90, 90), 'Wrist_pitch': (-45, 45), 'Wrist_yaw': (-30, 30),
        'Waist_yaw': (-45, 45), 'Waist_roll': (-15, 15), 'Waist_pitch': (-15, 40)}
WRIST_LIM = {'Wrist_roll': (-150, 150), 'Wrist_pitch': (-70, 70), 'Wrist_yaw': (-45, 45)}   # ours, deg
WAIST_LIM = {'Waist_yaw': (-90, 90), 'Waist_roll': (-25, 25), 'Waist_pitch': (-25, 60)}     # ours (no K1 joint), deg


def toM(m):
    return Manifold(Mesh(vert_properties=np.asarray(m.vertices, np.float32), tri_verts=np.asarray(m.faces, np.uint32)))


CHEST_PRE = ('waist-W4', 'trim-waisthub', 'elec-', 'estop-', 'cover-chest', 'trim-neckbase', 'torso-', 'motor-sp.', 'motor-headyaw')


def body_of(name):
    b = _body_of(name)
    if b and b.startswith('ey.') and name.startswith('cover-hand') and KS.WRIST() is not None:
        return 'wy.' + b.split('.')[1]                  # 7-DoF arm: the hand rides the wrist yaw
    if b == 'torso' and KS.waist_layout() is not None:
        nm = 'motor-' + name.split('-', 1)[1] if re.match(r'^rs\d\d-', name) else name
        if nm.startswith(CHEST_PRE):
            return 'chest'                              # above the waist: the chest rides the waist pitch
    return b


def TOP():
    """the body the arms and head hang from."""
    return 'chest' if KS.waist_layout() is not None else 'torso'


def _body_of(name):
    if re.match(r'^rs\d\d-', name):                 # motors are named by model; the body map is by role
        name = 'motor-' + name.split('-', 1)[1]
    for side, t in (('', 'l'), ('', 'r')):
        for b, prefixes in BODIES.items():
            for p in prefixes:
                p = p.format(t=t)
                ok = name.startswith(p)
                if ok and '{t}' in b:
                    tag = name.rsplit('.', 1)[-1] if '.' in name else ''
                    if tag and tag[0] != t:
                        continue
                    return b.format(t=t)
                if ok:
                    return b
    return None


def joints(J):
    """(name, moving bodies, centre, axis, URDF range deg) for the left side + head."""
    out = []
    for side, t in (('Left', 'l'),):
        Lg, U = KS.layout(J, side), KS.upper_layout(J, side)
        k1 = lambda n: J[n]['axis'] / np.linalg.norm(J[n]['axis'])
        ax = lambda ours, n: np.asarray(ours, float) * np.sign(np.dot(ours, k1(n)) or 1.0)
        lim = lambda n: tuple(np.degrees(J[n]['lim']))
        chain = [f'{b}.{t}' for b in LEG]
        for i, (jn, c, a) in enumerate((('Hip_Roll', Lg['roll'][0], [1, 0, 0]), ('Hip_Pitch', Lg['pitch'][0], [0, 1, 0]),
                                         ('Hip_Yaw', Lg['yaw'][0], [0, 0, 1]), ('Knee_Pitch', Lg['knee'][0], [0, 1, 0]))):
            n = f'{side}_{jn}'
            out.append((jn, chain[i:], c, ax(a, n), lim(n)))
        for jn, a in (('Ankle_Pitch', [0, 1, 0]), ('Ankle_Roll', [1, 0, 0])):
            n = f'{side}_{jn}'
            out.append((jn, [f'ankle.{t}'], Lg['ankle'], ax(a, n), lim(n)))
        chain = [f'{b}.{t}' for b in ARM]
        for i, (jn, k) in enumerate((('Shoulder_Pitch', 'sp'), ('Shoulder_Roll', 'sr'), ('Elbow_Pitch', 'ep'), ('Elbow_Yaw', 'ey'))):
            n = f'A{side}_{jn}' if jn == 'Shoulder_Pitch' else f'{side}_{jn}'
            c, a = U[k]
            out.append((jn, chain[i:], c, ax(a, n), lim(n)))
        Wr = KS.wrist_layout(J, side)
        if Wr is not None:
            for i, (jn, k) in enumerate((('Wrist_roll', 'roll'), ('Wrist_pitch', 'pitch'), ('Wrist_yaw', 'yaw'))):
                out.append((jn, chain[4 + i:], Wr[k][0], Wr[k][1], WRIST_LIM[jn]))
    W = KS.waist_layout()
    if W is not None:
        up = ['chest', 'hyaw', 'hpitch'] + [f'{b}.{t}' for b in ARM for t in 'lr']
        for jn, k, mv in (('Waist_yaw', 'yaw', ['wyaw', 'wroll']), ('Waist_roll', 'roll', ['wroll']), ('Waist_pitch', 'pitch', [])):
            out.append((jn, mv + up, W[k][0], W[k][1], WAIST_LIM[jn]))
    H = KS.head_layout(J)
    out.append(('Head_yaw', ['hyaw', 'hpitch'], H['yaw'][0], ax_(H['yaw'][1], J['AAHead_yaw']['axis']), tuple(np.degrees(J['AAHead_yaw']['lim']))))
    out.append(('Head_pitch', ['hpitch'], H['pitch'][0], ax_(H['pitch'][1], J['Head_pitch']['axis']), tuple(np.degrees(J['Head_pitch']['lim']))))
    return out


def ax_(ours, k1):
    ours = np.asarray(ours, float)
    return ours * np.sign(np.dot(ours, k1) or 1.0)


def sweep(step=5.0, only=None):
    J, _ = KS.world()
    D = KD.design()
    parts = D + KD.structure(D)
    B = {}
    for n, m in parts:
        b = body_of(n)
        if b is None:
            continue
        B.setdefault(b, []).append(toM(m))
    B = {b: Manifold.batch_boolean(ms, OpType.Add) for b, ms in B.items()}
    rows = []
    for jn, moving, c, a, (lo, hi) in joints(J):
        if only and jn not in only:
            continue
        mov = Manifold.batch_boolean([B[b] for b in moving if b in B], OpType.Add)
        # other limbs are posture (an arm hanging where a leg swings is a pose problem, not a design one)
        limb = lambda b: b.split('.')[0] in LEG and 'leg' or b.split('.')[0] in ARM and 'arm' or None
        mine = limb(moving[0])
        other_side = lambda b: '.' in b and '.' in moving[0] and b.split('.')[1] != moving[0].split('.')[1] and b.split('.')[0] in ('knee', 'ankle')
        stat = {b: M for b, M in B.items() if b not in moving and not (limb(b) and mine and limb(b) != mine) and not other_side(b) and not (jn.startswith('Waist') and limb(b) == 'leg')}   # feet crossing = stance
        S = Manifold.batch_boolean(list(stat.values()), OpType.Add)
        R = lambda deg: mov.transform(trimesh.transformations.rotation_matrix(np.radians(deg), a, c)[:3])
        base = (R(0) ^ S).volume()
        res = {}
        for direction, end in ((-1, lo), (1, hi)):
            reach, hit = end, None
            for deg in np.arange(0, abs(end) + 1e-6, step)[1:] * direction:
                if (R(deg) ^ S).volume() - base > THRESH:
                    Rd = R(deg)
                    hit = max(stat, key=lambda b: (Rd ^ stat[b]).volume())
                    reach = deg - direction * step
                    break
            res[direction] = (reach, hit)
        need = NEED.get(jn)
        ok = need is None or (res[-1][0] <= need[0] + 0.5 and res[1][0] >= need[1] - 0.5)
        rows.append((jn, (lo, hi), res, need, ok))
        print(f'{jn:15s} K1 {lo:6.0f}..{hi:4.0f}  free {res[-1][0]:6.0f}..{res[1][0]:4.0f}  need {need}  '
              f'{"ok" if ok else "SHORT"}  hits: -{res[-1][1]} +{res[1][1]}', flush=True)
    return rows


if __name__ == '__main__':
    import sys
    sweep(only=sys.argv[1:] or None)
