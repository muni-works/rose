"""Compose a URDF's visual meshes at zero pose into a loon of import-mesh roots.

This is the script the g1 README promised was "inline in the commit" and was
not. It walks the joint tree from the root link, accumulates each joint's
<origin xyz rpy> (zero joint angles), applies each visual's own origin, and
emits one [root [translate [rotate [import-mesh ...]]]] per visual.

  python3 urdf2loon.py <robot.urdf> <out.loon> [--links name,name,...] [--comment "..."]

Conventions that bit last time, recorded:
  * Unitree STLs are in METRES. vcad's plain import-mesh handles that already;
    do NOT wrap in import-mesh-scaled 1000.
  * loon's [rotate rx ry rz] is degrees, applied about fixed axes X then Y then
    Z - the same order as URDF's rpy, so rpy in degrees passes straight through.
  * The URDF base sits at the PELVIS origin (z=0). rose's trunk origin is at
    the hip-pitch height minus 60. Use the --dz option to shift the whole robot
    (mm) so sole heights line up for a side-by-side.
"""
import math
import os
import sys
import xml.etree.ElementTree as ET


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    # R = Rz(y) Ry(p) Rx(r)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp,     cp * sr,                cp * cr]]


def R_to_rpy(R):
    p = math.atan2(-R[2][0], math.hypot(R[0][0], R[1][0]))
    r = math.atan2(R[2][1], R[2][2])
    y = math.atan2(R[1][0], R[0][0])
    return r, p, y


def mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mulv(A, v):
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]


def origin(el):
    o = el.find('origin') if el is not None else None
    if o is None:
        return [0.0, 0.0, 0.0], rpy_to_R(0, 0, 0)
    xyz = [float(v) for v in o.get('xyz', '0 0 0').split()]
    rpy = [float(v) for v in o.get('rpy', '0 0 0').split()]
    return xyz, rpy_to_R(*rpy)


def main():
    args = sys.argv[1:]
    urdf, out = args[0], args[1]
    only = None
    comment = ''
    dz = 0.0
    i = 2
    while i < len(args):
        if args[i] == '--links':
            only = set(args[i + 1].split(','))
            i += 2
        elif args[i] == '--comment':
            comment = args[i + 1]
            i += 2
        elif args[i] == '--dz':
            dz = float(args[i + 1])
            i += 2
        else:
            i += 1

    base = os.path.dirname(os.path.abspath(urdf))
    tree = ET.parse(urdf)
    root = tree.getroot()
    links = {l.get('name'): l for l in root.findall('link')}
    joints = root.findall('joint')
    children = {}
    child_names = set()
    for j in joints:
        par = j.find('parent').get('link')
        ch = j.find('child').get('link')
        children.setdefault(par, []).append((j, ch))
        child_names.add(ch)
    roots = [n for n in links if n not in child_names]

    lines = ['; %s' % comment] if comment else []
    stack = [(r, [0.0, 0.0, 0.0], rpy_to_R(0, 0, 0)) for r in roots]
    while stack:
        name, t, R = stack.pop()
        link = links.get(name)
        if link is not None and (only is None or name in only):
            for vis in link.findall('visual'):
                mesh = vis.find('geometry/mesh')
                if mesh is None:
                    continue
                fn = mesh.get('filename')
                if fn.startswith('package://'):
                    fn = fn.split('/', 3)[-1]
                path = os.path.normpath(os.path.join(base, fn))
                vt, vR = origin(vis)
                T = [t[k] + mulv(R, vt)[k] for k in range(3)]
                RR = mul(R, vR)
                r, p, y = R_to_rpy(RR)
                lines.append('[root [translate %.3f %.3f %.3f [rotate %.3f %.3f %.3f '
                             '[import-mesh "%s"]]] "aluminum"]'
                             % (T[0] * 1000, T[1] * 1000, T[2] * 1000 + dz,
                                math.degrees(r), math.degrees(p), math.degrees(y), path))
        for (j, ch) in children.get(name, []):
            jt, jR = origin(j)
            T = [t[k] + mulv(R, jt)[k] for k in range(3)]
            stack.append((ch, T, mul(R, jR)))
    open(out, 'w').write('\n'.join(lines) + '\n')
    print('wrote %s (%d roots)' % (out, sum(1 for l in lines if l.startswith('[root'))))


if __name__ == '__main__':
    main()
