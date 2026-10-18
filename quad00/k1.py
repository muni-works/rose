"""Booster K1 (BSD-3, github.com/BoosterRobotics) for side-by-side comparison: its URDF at the zero pose,
meshes from ~/Developer/booster_assets/robots/K1, mm, Z up, simplified to fit the viewer.
Usage: python3 k1.py  ->  out/assembly/quad-k1.glb"""
import os, xml.etree.ElementTree as ET, numpy as np, trimesh
from manifold3d import Manifold, Mesh

K1 = os.path.expanduser('~/Developer/booster_assets/robots/K1')
HERE = os.path.dirname(os.path.abspath(__file__))


def rpy(r, p, y):
    return trimesh.transformations.euler_matrix(r, p, y, 'sxyz')


def origin(el):
    o = el.find('origin') if el is not None else None
    xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
    ang = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
    return trimesh.transformations.translation_matrix(xyz) @ rpy(*ang)


def simplify(m, tol=0.6):
    try:
        M = Manifold(Mesh(vert_properties=np.asarray(m.vertices, np.float32), tri_verts=np.asarray(m.faces, np.uint32))).simplify(tol)
        g = M.to_mesh()
        out = trimesh.Trimesh(np.asarray(g.vert_properties)[:, :3], np.asarray(g.tri_verts), process=False)
        return out if len(out.faces) else m
    except Exception:
        return m


POSE = {}                                               # joint name -> angle (rad); filled to drop the arms


def build(pose=None):
    pose = POSE if pose is None else pose
    r = ET.parse(os.path.join(K1, 'K1_22dof.urdf')).getroot()
    links = {l.get('name'): l for l in r.findall('link')}
    children = {}
    for j in r.findall('joint'):
        children.setdefault(j.find('parent').get('link'), []).append(j)
    root = (set(links) - {j.find('child').get('link') for j in r.findall('joint')}).pop()
    T = {root: np.eye(4)}
    stack = [root]
    while stack:
        p = stack.pop()
        for j in children.get(p, []):
            c = j.find('child').get('link')
            ax = [float(v) for v in (j.find('axis').get('xyz') if j.find('axis') is not None else '0 0 1').split()]
            T[c] = T[p] @ origin(j) @ trimesh.transformations.rotation_matrix(pose.get(j.get('name'), 0.0), ax)
            stack.append(c)
    scene = trimesh.Scene()
    S = np.diag([1000.0, 1000.0, 1000.0, 1.0])
    for name, l in links.items():
        v = l.find('visual')
        if v is None or v.find('geometry/mesh') is None:
            continue
        f = os.path.join(K1, v.find('geometry/mesh').get('filename'))
        m = trimesh.load(f, force='mesh')
        m.apply_transform(S @ T[name] @ origin(v) @ np.diag([1, 1, 1, 1.0]))
        m.apply_scale(1.0) if False else None
        m = m.copy()
        m.vertices = m.vertices  # metres already scaled by S on the translation only; scale geometry too
        scene.add_geometry(simplify(m), geom_name=f'k1-{name}', node_name=f'k1-{name}')
    scene.apply_translation([0, 0, -scene.bounds[0][2]])
    return scene


def arms_down():
    """the zero pose is a T-pose: find the shoulder-roll signs that bring the arms to the sides."""
    r = ET.parse(os.path.join(K1, 'K1_22dof.urdf')).getroot()
    names = [j.get('name') for j in r.findall('joint') if 'Shoulder_Roll' in j.get('name')]
    best = None
    for s in (1, -1):
        p = {n: (s if 'Left' in n else -s) * 1.35 for n in names}
        w = build(p).extents[1]
        best = min(best or (w, p), (w, p), key=lambda t: t[0])
    return best[1]


if __name__ == '__main__':
    POSE.update(arms_down())
    sc = build()
    out = os.path.join(HERE, 'out', 'assembly', 'quad-k1.glb')
    sc.export(out)
    print('K1', [round(v) for v in sc.extents], 'mm,', sum(len(g.faces) for g in sc.geometry.values()), 'faces,', round(os.path.getsize(out) / 1e6, 2), 'MB')
