"""mini arm checks: shoulder pitch-roll-yaw, elbow, and the 2-rod wrist, against the rest of the robot.
Usage: python3 arm_check.py"""
import itertools, math, numpy as np
import quad as Q
from manifold3d import Manifold, Mesh

toM = lambda m: Manifold(Mesh(vert_properties=np.asarray(m.vertices, np.float32), tri_verts=np.asarray(m.faces, np.uint32)))
KW = dict(leg=(230., 300.), arm=(120., 220.), chest_h=120., legm='rs06', armm='rs06', yawm='rs02', waistm='rs06', wristm='rs00',
          ankle_roll=True, head='rs00', legpk='rs04', rect=True, arm_chain=True, arm_yawm='rs02')
JOINT = ({'pivot-ankle', 'foot'}, {'pivot-wrist', 'hand'})
STATIC_ARM = ('rs06-shoulderpitch',)          # inside the chest, does not move with the arm


def hits(shoulder, elbow=60.0, wrist=(0.0, 0.0)):
    H = Q.humanoid_family('m', arm_pose=(0.0, elbow) + tuple(wrist), shoulder=shoulder, **KW)
    ex = dict(H['extra'])
    arm = {n: toM(m) for n, m in ex.items() if n.endswith('.al') and not n.startswith(STATIC_ARM)}
    rest = {n: toM(m) for n, m in ex.items() if not n.endswith(('.al', '.ar')) or n.startswith(STATIC_ARM)}
    return sorted({(a.split('.')[0], b.split('.')[0]) for a in arm for b in rest
                   if {a.split('.')[0], b.split('.')[0]} not in JOINT and (arm[a] ^ rest[b]).volume() > 5})


def sweep(label, poses):
    bad = {p: h for p in poses for h in [hits(*p)] if h}
    ok = [p for p in poses if p not in bad]
    print(f'{label}: clean {len(ok)}/{len(poses)}', '' if not bad else {k: v[:3] for k, v in bad.items()})


if __name__ == '__main__':
    sweep('pitch (elbow 60)', [((p, 0, 0),) for p in range(-180, 181, 30)])
    sweep('roll: abduction is negative (elbow 60)', [((0, r, 0), 60.0) for r in range(-180, 31, 15)])
    sweep('yaw (elbow 90)', [((0, 0, y), 90.0) for y in range(-90, 91, 30)])
    sweep('elbow (arm hanging)', [((0, 0, 0), e) for e in range(0, 131, 30)])
    sweep('elbow (arm 30 forward)', [((-30, 0, 0), e) for e in range(0, 131, 30)])
    sweep('arms up + out', [((p, r, 0), 30.0) for p in (-90, -150) for r in (0, -45, -90)])
