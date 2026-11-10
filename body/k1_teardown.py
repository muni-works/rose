"""Booster K1 teardown from its URDF + meshes (BSD-3, BoosterRobotics/booster_assets): for every joint, the
housing around its axis (diameter + width of the material within a radius of the axis, from the parent and
child link meshes), the link lengths, and which RobStride fits inside + carries the K1's rated torque.
Usage: python3 k1_teardown.py"""
import os, math, xml.etree.ElementTree as ET
import numpy as np, trimesh
import k1 as K
import quad as Q

R = ET.parse(os.path.join(K.K1, 'K1_22dof.urdf')).getroot()


def world(pose=None):
    """link frames (mm) at `pose` (joint -> rad) + each link's mesh in world (mm)."""
    pose = pose or {}
    links = {l.get('name'): l for l in R.findall('link')}
    children = {}
    for j in R.findall('joint'):
        children.setdefault(j.find('parent').get('link'), []).append(j)
    root = (set(links) - {j.find('child').get('link') for j in R.findall('joint')}).pop()
    T = {root: np.eye(4)}
    J = {}
    st = [root]
    while st:
        p = st.pop()
        for j in children.get(p, []):
            c = j.find('child').get('link')
            ax = [float(v) for v in (j.find('axis').get('xyz') if j.find('axis') is not None else '0 0 1').split()]
            T[c] = T[p] @ K.origin(j) @ trimesh.transformations.rotation_matrix(pose.get(j.get('name'), 0.0), ax)
            J[j.get('name')] = dict(parent=p, child=c, frame=T[c], axis=T[c][:3, :3] @ np.array(ax),
                                    lim=(float(j.find('limit').get('lower')), float(j.find('limit').get('upper'))) if j.find('limit') is not None else None,
                                    eff=float(j.find('limit').get('effort')) if j.find('limit') is not None else None,
                                    type=j.get('type'))
            st.append(c)
    S = np.diag([1000.0, 1000.0, 1000.0, 1.0])
    M = {}
    for n, l in links.items():
        v = l.find('visual')
        if v is None or v.find('geometry/mesh') is None:
            continue
        m = trimesh.load(os.path.join(K.K1, v.find('geometry/mesh').get('filename')), force='mesh')
        m.apply_transform(S @ T[n] @ K.origin(v))
        M[n] = m
    for j in J.values():
        j['p'] = (S @ j['frame'])[:3, 3]
    return J, M


def housing(j, M, rmax=70.0):
    """the material around a joint axis in its parent + child links: for points within `rmax` of the axis
    and within 60 mm of the joint along it, the radius that holds 95% of them and the axial width."""
    c, a = j['p'], j['axis'] / np.linalg.norm(j['axis'])
    P = np.concatenate([M[n].vertices for n in (j['parent'], j['child']) if n in M])
    d = P - c
    along = d @ a
    rad = np.linalg.norm(d - np.outer(along, a), axis=1)
    m = (rad < rmax) & (np.abs(along) < 60)
    if m.sum() < 20:
        return None
    r95 = float(np.percentile(rad[m], 95))
    core = m & (rad < r95)
    return dict(d=2 * r95, w=float(np.ptp(along[core])))


def main():
    J, M = world()
    print(f'K1: {len([j for j in J.values() if j["type"] != "fixed"])} joints')
    print(f'{"joint":24s} {"housing D x W (mm)":>18s} {"K1 N.m":>7s}  fits (RobStride D x L, peak) ')
    rows = []
    for n, j in J.items():
        if j['type'] == 'fixed' or n.startswith('Right'):
            continue
        h = housing(j, M)
        if h is None:
            continue
        fits = [k for k, m in Q.MOTORS.items() if m['D'] <= h['d'] + 2 and m['L'] <= h['w'] + 2]
        enough = [k for k in fits if Q.MOTORS[k]['peak'] >= 0.9 * (j['eff'] or 0)]
        print(f'{n:24s} {h["d"]:7.0f} x {h["w"]:4.0f}      {j["eff"] or 0:5.0f}   fits {fits or "-"}; strong enough {enough or "NONE"}')
        rows.append((n, h, j['eff'], fits, enough))
    # link lengths
    def dist(a, b): return float(np.linalg.norm(J[a]['p'] - J[b]['p']))
    print('\nlinks (joint to joint, mm):')
    for a, b in (('Left_Hip_Pitch', 'Left_Knee_Pitch'), ('Left_Knee_Pitch', 'Left_Ankle_Pitch'),
                 ('ALeft_Shoulder_Pitch', 'Left_Elbow_Pitch'), ('Left_Elbow_Pitch', 'Left_Elbow_Yaw'),
                 ('Left_Hip_Pitch', 'ALeft_Shoulder_Pitch'), ('Left_Hip_Pitch', 'Right_Hip_Pitch'), ('ALeft_Shoulder_Pitch', 'ARight_Shoulder_Pitch')):
        if a in J and b in J:
            print(f'  {a} -> {b}: {dist(a, b):.0f}')
    return J, M, rows


# our motor for each K1 joint (by the fit table): RS06 hip pitch/roll/knee, RS02 hip yaw, RS00 arms + head;
# the ankle pitch/roll are driven from the shank by two RS00 + rods (as on the K1), not at the joint
OURS = {'Hip_Pitch': 'rs06', 'Hip_Roll': 'rs06', 'Hip_Yaw': 'rs02', 'Knee_Pitch': 'rs06',
        'Shoulder_Pitch': 'rs00', 'Shoulder_Roll': 'rs00', 'Elbow_Pitch': 'rs00', 'Elbow_Yaw': 'rs00',
        'Head_yaw': 'rs00', 'Head_pitch': 'rs00'}


def on_stack():
    """the K1 (arms lowered) as a translucent ghost + our RobStrides coaxial on its joint frames."""
    pose = K.arms_down()
    J, M = world(pose)
    out = []
    for n, m in M.items():
        out.append((f'k1ghost-{n}', K.simplify(m)))
    for n, j in J.items():
        key = next((k for k in OURS if n.endswith(k)), None)
        if key is None:
            continue
        mo = Q.MOTORS[OURS[key]]
        a = j['axis'] / np.linalg.norm(j['axis'])
        out.append((f'{OURS[key]}-{n}', trimesh.creation.cylinder(radius=mo['D'] / 2, segment=[j['p'] - a * mo['L'] / 2, j['p'] + a * mo['L'] / 2], sections=64)))
    # ankle drive: two RS00 in the upper shank, lateral axes, as on our 2-rod ankle
    for side in ('Left', 'Right'):
        k, an = J[f'{side}_Knee_Pitch'], J[f'{side}_Ankle_Pitch']
        for f in (0.38, 0.62):
            p = k['p'] + (an['p'] - k['p']) * f
            a = k['axis'] / np.linalg.norm(k['axis'])
            mo = Q.MOTORS['rs00']
            out.append((f'rs00-{side}_Ankle_drive_{f}', trimesh.creation.cylinder(radius=mo['D'] / 2, segment=[p - a * mo['L'] / 2, p + a * mo['L'] / 2], sections=64)))
    return out


if __name__ == '__main__':
    main()
