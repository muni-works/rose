"""rose full's 6-DoF hand closing on a can: each finger's three phalanges as real joints, coupled by four-bars and driven by
its palm linear actuator through a slider-crank. Run from body/:  python hand_grasp.py OUTDIR [FRAMES]

Per finger: the actuator pushes a rod to a crank on the proximal phalanx (MCP). A coupler from a palm-side pivot to the
back of the middle phalanx makes PIP follow MCP; a second coupler, from the proximal phalanx to the back of the distal,
makes DIP follow PIP -- both closed numerically, so the coupling is whatever the linkage geometry gives. Each finger
closes until a phalanx touches the can and stops there (the actuator stalls; the others keep closing), so the grasp
adapts. Writes OUTDIR/grasp.loon (a jointed assembly), OUTDIR/grasp.json (vcad-render --animate timeline) and prints
each finger's contact angles and the actuator stroke they took.

Coupler and crank pivots are ASSUMED (mm offsets below), sized to look right, not yet optimised for force transmission.
"""
import sys, os, json, math
sys.path.insert(0, os.getcwd())
import numpy as np, trimesh
from scipy.optimize import brentq
import k1_structure as KS, k1_design as KD
from design import ring, M_

OUT = sys.argv[1]; FRAMES = int(sys.argv[2]) if len(sys.argv) > 2 else 40
os.makedirs(OUT, exist_ok=True)
KS.use('full'); J, M = KS.world(); KD.design()
H = KD.HAND_SPEC['l']; s = H['s']; w, v = H['w'], H['v']
AX = np.array([-s * 1.0, 0, 0])                     # flexion axis: +angle curls the tips to the palm side
PALM, DORS, UP = np.array([0, -s * 1.0, 0]), np.array([0, s * 1.0, 0]), np.array([0, 0, 1.0])
CPL = dict(g=4.0, g_up=0.0, b=6.0, b_dn=0.0)        # MCP->PIP coupler: palm-side ground pivot / dorsal pivot on the middle
                                                     # (searched: stays assembled 0-90 deg MCP, PIP/MCP 0.65-0.75)
CPL2 = dict(g=3.5, g_up=0.0, b=5.0, b_dn=0.0)       # PIP->DIP coupler: on the proximal / dorsal on the distal (same proportions)
CRANK = 7.0                                          # actuator crank on the proximal phalanx, dorsal of the MCP


def bar(segs):
    return KD.hull_rings([ring(c, w, v, a, b, n=2.2) for c, a, b in segs])


def phalanx(p0, p1, r0, r1):
    return bar([(p0, r0 * 1.15, r0), ((p0 + p1) / 2, r0 * 1.12, r0 * 0.98), (p1, r1 * 1.1, r1)])


def R(deg, axis=AX, c=(0, 0, 0)):
    return trimesh.transformations.rotation_matrix(math.radians(deg), axis, c)


def ap(T, p):
    return (T @ np.r_[p, 1.0])[:3]


def track(e, prev, span=60.0, step=0.5):
    """the closure angle nearest the previous one (stay on the branch the linkage is assembled on)."""
    for d in np.arange(0.0, span, step):
        for sg in (1, -1):
            a, b = prev + sg * d, prev + sg * (d + step)
            if e(a) * e(b) <= 0:
                return brentq(e, min(a, b), max(a, b))
    raise ValueError('the four-bar came apart')


def finger_pose(f, a1, step=2.0):
    """MCP angle a1 -> (a2, a3) relative angles at PIP and DIP from the two four-bars (followed from the open hand), and
    each phalanx's world transform."""
    P1, P2, P3 = f[0][0], f[1][0], f[2][0]
    G = P1 + PALM * CPL['g'] + UP * CPL['g_up']; B0 = P2 + DORS * CPL['b'] - UP * CPL['b_dn']; Lc = np.linalg.norm(B0 - G)
    G2_0 = P2 + PALM * CPL2['g'] + UP * CPL2['g_up']; C0 = P3 + DORS * CPL2['b'] - UP * CPL2['b_dn']; Lc2 = np.linalg.norm(C0 - G2_0)
    tr = trimesh.transformations.translation_matrix
    a2 = a3 = 0.0
    for x in list(np.arange(step, a1, step)) + [a1]:
        if x <= 0: break
        T1 = R(x, c=P1); P2w = ap(T1, P2)
        a2 = track(lambda q: np.linalg.norm(ap(R(x + q, c=P2w) @ tr(P2w - P2), B0) - G) - Lc, a2)
        T2 = R(x + a2, c=P2w) @ tr(P2w - P2); G2 = ap(T1, G2_0); P3w = ap(T2, P3)
        a3 = track(lambda q: np.linalg.norm(ap(R(x + a2 + q, c=P3w) @ tr(P3w - P3), C0) - G2) - Lc2, a3)
    T1 = R(a1, c=P1); P2w = ap(T1, P2); T2 = R(a1 + a2, c=P2w) @ tr(P2w - P2); P3w = ap(T2, P3)
    T3 = R(a1 + a2 + a3, c=P3w) @ tr(P3w - P3)
    return a2, a3, (T1, T2, T3)


def stroke(f, x_la, y_la, z_tip, a1):
    """the actuator extension that puts the crank at MCP angle a1 (a slider-crank: rod length fixed)."""
    P1 = f[0][0]; K0 = P1 + DORS * CRANK; K = ap(R(a1, c=P1), K0)
    T0 = np.array([x_la, y_la, z_tip]); Lr = np.linalg.norm(K0 - T0)
    dxy = math.hypot(K[0] - x_la, K[1] - y_la)
    return (z_tip - (K[2] + math.sqrt(max(Lr ** 2 - dxy ** 2, 0.0)))) * -1.0


fingers = H['phalanges']
meshes = [[phalanx(*seg) for seg in f] for f in fingers]
palm = H['palm']
# the object: a 44 mm bottle across the fingers (along x), placed by search so the fingers close to ~40 deg MCP before
# touching, clear of the palm
cw, zk = H['cw'], H['zk']
r_can = 22.0
can_c = np.array([cw[0], cw[1] - s * 70.0, zk - 10.0])
can = trimesh.creation.cylinder(radius=r_can, height=124.0, sections=96, transform=trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0]) @ np.eye(4))
can.apply_translation(can_c)
CAN = M_(can)
assert (M_(palm) ^ CAN).volume() < 1.0, 'the can sits in the palm'


def touches(ms, Ts):
    return any((M_(m.copy().apply_transform(T)) ^ CAN).volume() > 0.3 for m, T in zip(ms, Ts))


x0 = H['wc'][0]
la = [(x0 + 4.0 + fx * 0.96, H['wc'][1] - s * 4.0) for fx in KS.HAND_FX]
z_tip = H['wc'][2] - 24.0 - 8.0 - KS.LA_HAND['L']
report, contact = [], []
for i, f in enumerate(fingers):
    a1 = 0.0
    while a1 < 100.0:
        a2, a3, Ts = finger_pose(f, a1 + 1.0)
        if touches(meshes[i], Ts):
            break
        a1 += 1.0
    a2, a3, _ = finger_pose(f, a1)
    contact.append(a1)
    report.append('finger %d: MCP %5.1f  PIP %5.1f  DIP %5.1f  (PIP/MCP %.2f)  actuator stroke %5.1f mm'
                  % (i, a1, a2, a3, a2 / a1 if a1 else 0, stroke(f, *la[i], z_tip, a1)))
# the thumb: two segments, flexing toward the can's axis; its IP follows at 0.8 (ASSUMED: the flex actuator's linkage)
th = H['thumb']; tm_ = [phalanx(*seg) for seg in th]
td = th[0][1] - th[0][0]; tc = can_c - th[0][0]; tc[0] = 0.0
TAX = np.cross(td, tc); TAX /= np.linalg.norm(TAX)
def thumb_T(b):
    T1 = R(b, TAX, th[0][0]); q = ap(T1, th[1][0])
    return T1, R(1.8 * b, TAX, q) @ trimesh.transformations.translation_matrix(q - th[1][0])
b = 0.0
while b < 90.0 and not touches(tm_, thumb_T(b + 1.0)):
    b += 1.0
report.append('thumb:    flex %5.1f  IP %5.1f' % (b, 0.8 * b))
print('\n'.join(report))

# ---- the jointed assembly (vcad_export's loon form): palm anchors; each phalanx is authored about its proximal pivot.
# Shown palm-down, reaching: the whole scene turns 90 deg about x (the hanging hand's palm faces -y), the bottle on a table.
GR = trimesh.transformations.rotation_matrix(s * math.pi / 2, [1, 0, 0]); Rg = GR[:3, :3]
mats = ['[material "satin_black" 0.09 0.094 0.106 0.1 0.55]', '[material "can" 0.82 0.83 0.85 0.95 0.18]',
        '[material "red" 0.75 0.06 0.05 0.1 0.35]']
parts, inst, jts = [], [], []
def stl(name, m):
    m = m.copy(); m.apply_transform(GR)            # world turned palm-down (local phalanx frames turn about their pivot)
    m.export(os.path.join(OUT, name + '.stl')); return '[import-mesh-scaled 0.001 0.001 0.001 "%s.stl"]' % name
parts.append('[part "palm" %s "satin_black"]' % stl('palm', palm)); inst.append('[instance "palm_i" "palm" 0.0 0.0 0.0]')
parts.append('[part "can" %s "can"]' % stl('can', can)); inst.append('[instance "can_i" "can" 0.0 0.0 0.0]')
band = trimesh.creation.cylinder(radius=r_can + 0.2, height=60.0, sections=96, transform=trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0]))
band.apply_translation(can_c); parts.append('[part "label" %s "red"]' % stl('label', band)); inst.append('[instance "label_i" "label" 0.0 0.0 0.0]')
jts += ['[fixed-joint "fix_can" "palm_i" 0.0 0.0 0.0 "can_i" 0.0 0.0 0.0]', '[fixed-joint "fix_label" "palm_i" 0.0 0.0 0.0 "label_i" 0.0 0.0 0.0]']
cz = (Rg @ can_c)[2] - r_can                                   # the table under the bottle (in the turned frame)
cc = Rg @ can_c
table = trimesh.creation.box(extents=[240.0, 200.0, 18.0]); table.apply_translation([cc[0], cc[1], cz - 9.0])
table.apply_transform(np.linalg.inv(GR))                         # stl() turns it back
mats.append('[material "wood" 0.55 0.42 0.3 0.0 0.6]')
parts.append('[part "table" %s "wood"]' % stl('table', table)); inst.append('[instance "table_i" "table" 0.0 0.0 0.0]')
jts.append('[fixed-joint "fix_table" "palm_i" 0.0 0.0 0.0 "table_i" 0.0 0.0 0.0]')
f3 = lambda p: '%.3f %.3f %.3f' % tuple(Rg @ np.asarray(p, float))
tracks = []
for i, f in enumerate(fingers):
    par, ppiv = 'palm_i', np.zeros(3)
    for k, (seg, m) in enumerate(zip(f, meshes[i])):
        n = 'f%d%d' % (i, k); piv = seg[0]
        parts.append('[part "%s" %s "satin_black"]' % (n, stl(n, m.copy().apply_translation(-piv))))
        inst.append('[instance "%s_i" "%s" 0.0 0.0 0.0]' % (n, n))
        jts.append('[revolute-joint "j%s" %s 0.0 140.0 "%s" %s "%s_i" 0.0 0.0 0.0]' % (n, f3(AX), par, f3(piv - ppiv), n))
        par, ppiv = n + '_i', piv
    keys = [[], [], []]
    for t in range(FRAMES + 1):
        a1 = min(contact[i], 100.0 * t / FRAMES)
        a2, a3, _ = finger_pose(f, a1)
        for k, a in enumerate((a1, a2, a3)):
            keys[k].append({'t': round(t / FRAMES * 3.0, 4), 'value': round(a, 3)})
    tracks += [{'joint': 'jf%d%d' % (i, k), 'keys': keys[k]} for k in range(3)]
par, ppiv = 'palm_i', np.zeros(3)
for k, (seg, m) in enumerate(zip(th, tm_)):
    n = 't%d' % k; piv = seg[0]
    parts.append('[part "%s" %s "satin_black"]' % (n, stl(n, m.copy().apply_translation(-piv))))
    inst.append('[instance "%s_i" "%s" 0.0 0.0 0.0]' % (n, n))
    jts.append('[revolute-joint "j%s" %s 0.0 140.0 "%s" %s "%s_i" 0.0 0.0 0.0]' % (n, f3(TAX), par, f3(piv - ppiv), n))
    par, ppiv = n + '_i', piv
for k, kk in ((0, 1.0), (1, 0.8)):
    tracks.append({'joint': 'jt%d' % k, 'keys': [{'t': round(t / FRAMES * 3.0, 4), 'value': round(kk * min(b, 90.0 * t / FRAMES), 3)} for t in range(FRAMES + 1)]})
src = '\n'.join(mats) + '\n[assembly\n #[' + '\n   '.join(parts) + ']\n #[' + '\n   '.join(inst) + ']\n #[' + '\n   '.join(jts) + ']\n "palm_i"]\n'
open(os.path.join(OUT, 'grasp.loon'), 'w').write(src)
json.dump({'durationS': 3.0, 'fps': FRAMES / 3.0, 'tracks': tracks}, open(os.path.join(OUT, 'grasp.json'), 'w'))
