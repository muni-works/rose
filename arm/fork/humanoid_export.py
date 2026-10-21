"""Export the quad00 full (1703 mm, H2-class) humanoid WITHOUT its arms, for the lymb v10 body renders.

Reads quad00 (read-only) at $QUAD00 (default: the claude/quadruped-arm-design worktree), builds the skin
(k1_design.design) and the plate frame (k1_design.structure), drops every arm part (tags .la / .ra: the
shoulder pitch / roll / yaw / elbow motors, arm plates, wrist, hand, deltoid and arm covers), lifts the
body so the soles stand on z = 0, and writes one STL per finish plus the shoulder sockets:

  OUT/hum_<finish>.stl     mm, world frame (x fore, y left, z up)
  OUT/hum.json             {"finishes": {...rgb, metal, rough}, "sr": {"l": [x,y,z], "r": ...},
                            "side_plate_y": |y| of the torso side plate's outer face, "z0": lift}

Run (quad00's deps):
  uv run -q --with numpy --with shapely --with manifold3d --with trimesh --with scipy --with ezdxf \\
     --with matplotlib python arm/fork/humanoid_export.py OUT
"""
import json
import os
import sys

Q = os.environ.get('QUAD00', os.path.expanduser('~/Developer/rose/.claude/worktrees/relaxed-wescoff-d1237a/quad00'))
OUT = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else 'hum')
os.makedirs(OUT, exist_ok=True)
os.chdir(Q)
sys.path.insert(0, Q)
import numpy as np
import trimesh
import k1_structure as KS
import vcad_export as VX

KS.use('full')
import k1_design as KD
J, M = KS.world()
D = KD.design()
C = KD.structure(D)
z0 = -min(m.bounds[0][2] for _, m in D)
groups = {}
import math
HEAD_PITCH = float(os.environ.get('HEAD_PITCH', 35.0))   # rose: eyes down by the neck, ~35 deg at rest
pitched = []
HEAD_PARTS = set()
import re as _re
_fl = os.path.join(Q, 'vcad', 'full', 'full.loon')
FULL_PARTS = set(_re.findall(r'\[part "([^"]+)"', open(_fl).read())) if os.path.exists(_fl) else set()   # quad00's own export: the truth for what the body wears   # v10.2: the old cube head goes; the cortex (quad00/cortex/cortex.loon) sits on the dock
dropped = []
for n, m in D + C:
    tag = n.rpartition('.')[2] if '.' in n else ''
    # v10.8: the head parts to drop are exactly the old helmet head's - everything quad00's own full.loon keeps (the
    # neck sleeve, the saddle cheeks cover-headcheek / cover-headhub, head-H1, head-dock) stays. v10.2-v10.7 dropped
    # 'cover-head*' / 'head-*' too, which took rose's neck pass-2 cheeks off and left the bare pitch drum showing.
    old_head = n.startswith(('cortex-', 'visor-', 'cover-helmet', 'cover-headbutton', 'trim-eye', 'trim-ear', 'trim-headpanel'))
    if FULL_PARTS:
        old_head = old_head or (n.split('.')[0].startswith(('head', 'cover-head', 'trim-head', 'visor')) and n.replace('.', '_') not in FULL_PARTS)
    if tag in ('la', 'ra') or n.split('.')[0] in HEAD_PARTS or old_head:
        dropped.append(n)
        continue
    k = VX.mat_of(n)
    if HEAD_PITCH and VX.body_of(n) == 'hpitch':     # rose's rest pose: everything on the head-pitch body pitched down (v10.8: -35 in trimesh = the cortex's +35 in vcad)
        hp = KS.head_layout(J)['pitch'][0]
        m = m.copy().apply_transform(trimesh.transformations.rotation_matrix(math.radians(-HEAD_PITCH), [0, 1, 0], hp))
        pitched.append(n)
    groups.setdefault(k, []).append(m)
# v10.1: the chest socket. J1 (RS04 + its graphite drum, r ~72.5) runs in through the chest shell to the
# torso side plate. Cut a r = SOCKET_R recess through every cover it crosses, and put a gunmetal bezel
# on the shell's surface round it - J1 then reads flush, like the humanoid's own shoulder drum.
from manifold3d import Manifold, Mesh
SOCKET_R = 75.0
_mm = lambda t: Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32))
_tm = lambda M: trimesh.Trimesh(np.asarray(M.to_mesh().vert_properties)[:, :3], np.asarray(M.to_mesh().tri_verts))
sockets = []
for side in ('Left', 'Right'):
    c = KS.upper_layout(J, side)['sr'][0]
    s_ = 1 if side == 'Left' else -1
    sockets.append((c, s_))
cut_n = 0
for i, m in enumerate(groups.get('cover', []) if os.environ.get('CHEST_SOCKET') else []):
    for c, s_ in sockets:
        cyl = trimesh.creation.cylinder(radius=SOCKET_R, segment=[[c[0], s_ * 90.0, c[2]], [c[0], s_ * 200.0, c[2]]], sections=96)
        if not m.bounds[0][1] - 1 < s_ * 150 < m.bounds[1][1] + 1 and not (m.bounds[0][1] < s_ * 120 < m.bounds[1][1]):
            continue
        try:
            M = Manifold(_mm(m))
            if M.status().name != 'NoError' if hasattr(M.status(), 'name') else False:
                continue
            r = M - Manifold(_mm(cyl))
            if r.volume() < M.volume() - 1.0:
                m = _tm(r)
                cut_n += 1
        except Exception:
            pass
    groups['cover'][i] = m
for c, s_ in (sockets if os.environ.get('CHEST_SOCKET') else []):          # bezel where the recess meets the outermost cover
    V = np.concatenate([mm.vertices for mm in groups.get('cover', [])])
    near = V[(np.hypot(V[:, 0] - c[0], V[:, 2] - c[2]) < SOCKET_R + 20) & (s_ * V[:, 1] > 100) & (s_ * V[:, 1] < 200)]
    y_out = s_ * (np.abs(near[:, 1]).max() if len(near) else 146.0)
    bz = trimesh.creation.annulus(r_min=SOCKET_R - 2.0, r_max=SOCKET_R + 4.0, height=3.0, sections=96)
    bz.apply_transform(trimesh.geometry.align_vectors([0, 0, 1], [0, s_, 0]))
    bz.apply_translation([c[0], y_out - s_ * 1.0, c[2]])
    groups.setdefault('trim', []).append(bz)
print('chest socket: %d cover cuts' % cut_n)
fin = {}
for k, ms in groups.items():
    mesh = trimesh.util.concatenate([x for x in ms if len(x.faces)])
    mesh.apply_translation([0, 0, z0])
    mesh.export(os.path.join(OUT, 'hum_%s.stl' % k))
    fin[k] = VX.MATS.get(k, ('aluminium', 0.75, 0.76, 0.78, 0.9, 0.3))
sr = {}
for side, t in (('Left', 'l'), ('Right', 'r')):
    P = KS.upper_layout(J, side)
    sr[t] = (P['sr'][0] + np.array([0, 0, z0])).tolist()
    sp = P['sp'][0]
m = KS.ARM()
side_plate = abs(KS.upper_layout(J, 'Left')['sp'][0][1]) - m['L'] / 2 - KS.G     # sp stator back face = side plate outer face
HF = KS.head_frame(J, M)
json.dump(dict(cortex=[float(HF['ox']), float(HF['oy']), float(HF['zb'] + z0)], cortex_R=float(HF['R']), cortex_k=1.0 if os.path.exists(os.path.join(Q, 'cortex', 'cortex-full.loon')) else float(HF['hh'] / 120.6),
               head_pitch=(KS.head_layout(J)['pitch'][0] + np.array([0, 0, z0])).tolist(), finishes=fin, sr=sr, z0=float(z0), side_plate_y=float(side_plate), dropped=dropped,
               sp=(KS.upper_layout(J, 'Left')['sp'][0] + np.array([0, 0, z0])).tolist(),
               ey=(KS.upper_layout(J, 'Left')['ey'][0] + np.array([0, 0, z0])).tolist()),
          open(os.path.join(OUT, 'hum.json'), 'w'), indent=1)
print('pitched with the head:', pitched)
print('groups', {k: len(v) for k, v in groups.items()}, 'dropped', len(dropped), 'z0 %.1f' % z0, 'side plate %.1f' % side_plate)
