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
dropped = []
for n, m in D + C:
    tag = n.rpartition('.')[2] if '.' in n else ''
    if tag in ('la', 'ra'):
        dropped.append(n)
        continue
    k = VX.mat_of(n)
    groups.setdefault(k, []).append(m)
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
json.dump(dict(finishes=fin, sr=sr, z0=float(z0), side_plate_y=float(side_plate), dropped=dropped,
               sp=(KS.upper_layout(J, 'Left')['sp'][0] + np.array([0, 0, z0])).tolist(),
               ey=(KS.upper_layout(J, 'Left')['ey'][0] + np.array([0, 0, z0])).tolist()),
          open(os.path.join(OUT, 'hum.json'), 'w'), indent=1)
print('groups', {k: len(v) for k, v in groups.items()}, 'dropped', len(dropped), 'z0 %.1f' % z0, 'side plate %.1f' % side_plate)
