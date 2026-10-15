"""Emit a simulation-ready robot description for rose-full (1800 mm layout).

Single source of truth for the kinematics is full/rose_full_layout.py; this
module re-states the same joint origins in PARENT-RELATIVE form (URDF wants
relative, the layout script works in absolute trunk-frame coordinates) and adds
the inertial / collision data a simulator needs.

Writes two files, named for what ipse-sim's loader expects:
  K1_22dof_floating.urdf  - massless `world` root + its own <joint type="floating">,
                            matching the vendor convention. ipse_sim::K1::load_from
                            passes BaseKind::Fixed precisely because the URDF
                            supplies its own free joint; two would stack 12 base DOF.
  K1_22dof.xml            - companion MJCF carrying per-joint `armature`, which URDF
                            cannot express. ipse_sim::vendor::VendorParams reads it as
                            a sibling of the URDF. Without it armature is zero and the
                            computed gains are off by orders of magnitude at the ankle.

Design rules honoured here:
  * every link gets positive mass and a strictly positive-definite inertia tensor,
    or it lands in Inventory::degenerate_inertias and destabilises Featherstone
  * collisions are PRIMITIVES (box/cylinder), never meshes - a mesh-only link gets
    no contact shape at all in phyz
  * feet are BOXES: stand.rs shape_support_points yields 8 corners for a box and
    NOTHING for a cylinder, so cylindrical feet give an empty support polygon
  * joint names carry Hip / Knee / Ankle / Ankle_Pitch substrings and pitch about
    +Y, which is how stand.rs classifies leg DOFs and builds its crouch pose
  * foot links are named left_foot_link / right_foot_link to match the hardcoded
    stand.rs FOOT_LINKS
"""
import math
import os
import sys

# ---------------------------------------------------------------- geometry ---
# Single source: rose_full_layout. Everything here is read from it.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rose_full_layout as LAY

HIP_Y, LEG_Y, SHO_Y = LAY.HIP_Y, LAY.LEG_Y, LAY.SHO_Y
THIGH, SHANK, ANK_DZ = LAY.THIGH, LAY.SHANK, LAY.ANK_DZ
KNEE_BACK = LAY.KNEE_BACK
SOLE = LAY.SOLE

# actuator data: mass kg, ratio, rotor inertia kg.m^2, peak Nm, max rad/s
ACT = {m: (LAY.MODEL[m][2], LAY.MODEL[m][3], LAY.MODEL[m][4] or 6.0e-4,
           LAY.PEAK[m][0], LAY.PEAK[m][1]) for m in LAY.MODEL}


def armature(model):
    _, n, jm, _, _ = ACT[model]
    return n * n * jm


def box_inertia(m, sx, sy, sz):
    return (m*(sy*sy+sz*sz)/12.0, m*(sx*sx+sz*sz)/12.0, m*(sx*sx+sy*sy)/12.0)


def cyl_inertia(m, r, h):
    """Cylinder about its local Z."""
    ixx = m*(3*r*r + h*h)/12.0
    return (ixx, ixx, m*r*r/2.0)


class Link:
    def __init__(self, name, mass, com, inertia, shape):
        self.name, self.mass, self.com, self.inertia, self.shape = \
            name, mass, com, inertia, shape


class Joint:
    def __init__(self, name, parent, child, origin, axis, lo, hi, model):
        self.name, self.parent, self.child = name, parent, child
        self.origin, self.axis, self.lo, self.hi, self.model = origin, axis, lo, hi, model


def bx(m, sx, sy, sz, com):
    return Link.__new__(Link), (sx, sy, sz), com


LINKS, JOINTS = [], []


# Adjacent links meet exactly at their joint plane, so their collision primitives
# touch and the solver sees a self-contact at every joint (26 contacts, 700+ kN).
# The K1 suppresses this with a hardcoded SELF_COLLISION_ARTIFACTS pair list keyed
# on its own link names; rather than fork that table we simply shrink each shape
# along the chain axis so neighbours never touch. Inertia is still computed from
# the TRUE size - only the contact shape is inset.
GAP = 0.004


def add_box(name, mass, size, com, shrink=True):
    sx, sy, sz = size
    col = (sx, sy, max(sz - 2*GAP, 0.002)) if shrink else size
    LINKS.append(Link(name, mass, com, box_inertia(mass, *size), ('box', col)))


def add_cyl(name, mass, r, h, com):
    LINKS.append(Link(name, mass, com, cyl_inertia(mass, r, h),
                      ('cylinder', (r, max(h - 2*GAP, 0.002)))))


def add_joint(name, parent, child, origin, axis, lo, hi, model):
    JOINTS.append(Joint(name, parent, child, origin, axis, lo, hi, model))


# --------------------------------------------------------------- the robot ---
# Link masses at 1800: the actuator each link CARRIES (stator side) plus a
# structure share from SIZING.md (68 kg design mass - 34 kg actuators = 34 kg of
# structure, battery, compute, hands; the plate table will replace the structure
# estimates when it exists). Trunk = 2x hip pitch + 2x shoulder pitch + head yaw
# actuators (11.2 kg) + 6 kg sheet + 10 kg battery/compute + 1 kg head = 28 kg.
J = {lab: org for (lab, org, *_r) in LAY.JOINTS}
Z = dict(hp=LAY.Z_HIP_P, hr=LAY.Z_HIP_R, hy=LAY.Z_HIP_Y, kn=LAY.Z_KNEE, ap=LAY.Z_ANK_P,
         ar=LAY.Z_ANK_R, sp=LAY.Z_SHO_P, sr=LAY.Z_SHO_R, el=LAY.Z_ELB, wr=LAY.Z_WRIST,
         hy2=LAY.Z_HEAD_Y, hp2=LAY.Z_HEAD_P)
M = {m: LAY.MODEL[m][2] for m in LAY.MODEL}

# Trunk (above the waist): 2x shoulder pitch + head yaw + waist yaw OUTPUT side
# structure, 5 kg sheet, 10 kg battery/compute, 1 kg head -> 19.6 kg. Pelvis: the two
# hip-pitch X15 (7.0) + the waist yaw X8 (1.4) + 2.2 kg sheet.
add_box('Trunk', 19.6, (0.200, 0.330, 0.420), (0.0, 0.0, 0.330))
add_box('Pelvis', 10.6, (0.190, 0.200, 0.260), (0.0, 0.0, -0.070 - LAY.Z_WAIST))
add_joint('Waist_Yaw', 'Trunk', 'Pelvis', (0.0, 0.0, LAY.Z_WAIST), (0, 0, 1), -1.0, 1.0, 'X8-120')

for side, s in (('Left', 1.0), ('Right', -1.0)):
    lo_roll, hi_roll = (-0.5, 1.5) if s > 0 else (-1.5, 0.5)
    p = '%s_Hip_Pitch' % side
    r = '%s_Hip_Roll' % side
    y = '%s_Hip_Yaw' % side
    sh = '%s_Shank' % side
    ac = '%s_Ankle_Cross' % side
    ft = '%s_foot_link' % ('left' if s > 0 else 'right')
    dy = s * (LEG_Y - HIP_Y)           # the pitch link reaches OUTBOARD to the leg column

    # hip pitch link: carries the roll actuator (X12) 70 mm outboard of the pitch axis
    add_box(p,  M['X12-320'] + 1.2, (0.130, 0.150, 0.130), (0.0, dy / 2, (Z['hr'] - Z['hp']) / 2))
    # hip roll link: carries the yaw actuator
    add_box(r,  M['X10-200'] + 0.8, (0.130, 0.130, 0.070), (0.0, 0.0, (Z['hy'] - Z['hr']) / 2))
    # thigh (yaw link): carries the knee actuator, a wrapped channel
    add_cyl(y,  M['X15-450'] + 2.8, 0.075, THIGH, (-KNEE_BACK / 2, 0.0, -THIGH / 2))
    # shank: carries BOTH ankle motors (pushrod ankle), high, at z -602 / -704: its COM
    # sits in the upper third. The ankle cross is a pinned yoke, 0.4 kg.
    add_cyl(sh, 2 * M['X8-120'] + 2.0, 0.050, SHANK, (0.0, 0.0, -SHANK * 0.38))
    add_box(ac, 0.4, (0.120, 0.040, 0.050), (0.0, 0.0, -ANK_DZ / 2))
    add_box(ft, 0.6, (0.280, 0.110, 0.022), (0.040, 0.0, -(SOLE - 0.021)), shrink=False)

    add_joint(p,  'Pelvis', p, (0.0, s * HIP_Y, Z['hp'] - LAY.Z_WAIST), (0, 1, 0), -2.0, 1.6, 'X15-450')
    add_joint(r,  p, r, (0.0, dy, Z['hr'] - Z['hp']), (1, 0, 0), lo_roll, hi_roll, 'X12-320')
    add_joint(y,  r, y, (0.0, 0.0, Z['hy'] - Z['hr']), (0, 0, 1), -1.0, 1.0, 'X10-200')
    add_joint('%s_Knee_Pitch' % side, y, sh, (-KNEE_BACK, 0.0, Z['kn'] - Z['hy']), (0, 1, 0), 0.0, 2.3, 'X15-450')
    # both ankle joints are driven through the pushrods by the two X8s (pitch by
    # their sum, roll by their difference); effort/speed/armature of one X8 per axis
    add_joint('%s_Ankle_Pitch' % side, sh, ac, (0.0, 0.0, -SHANK), (0, 1, 0), -0.9, 0.5, 'X8-120')
    add_joint('%s_Ankle_Roll' % side, ac, ft, (0.0, 0.0, -ANK_DZ), (1, 0, 0), -0.4, 0.4, 'X8-120')

    a1, a2, a3 = '%s_Arm_1' % side, '%s_Arm_2' % side, '%s_Arm_3' % side
    hd = '%s_hand_link' % ('left' if s > 0 else 'right')
    add_box(a1, M['X6-60'] + 0.4, (0.100, 0.100, Z['sp'] - Z['sr']), (0.0, 0.0, -(Z['sp'] - Z['sr']) / 2))
    add_cyl(a2, M['X6-60'] + 0.7, 0.040, Z['sr'] - Z['el'], (0.0, 0.0, -(Z['sr'] - Z['el']) / 2))
    add_cyl(a3, M['X4-36'] + 0.6, 0.036, Z['el'] - Z['wr'], (0.0, 0.0, -(Z['el'] - Z['wr']) / 2))
    add_box(hd, 0.4, (0.080, 0.040, 0.150), (0.0, 0.0, -0.075))
    lo_sr, hi_sr = (-1.7, 1.6) if s > 0 else (-1.6, 1.7)
    add_joint('%s_Shoulder_Pitch' % side, 'Trunk', a1, (0.0, s * SHO_Y, Z['sp']), (0, 1, 0), -3.0, 1.5, 'X8-120')
    add_joint('%s_Shoulder_Roll' % side, a1, a2, (0.0, 0.0, Z['sr'] - Z['sp']), (1, 0, 0), lo_sr, hi_sr, 'X6-60')
    # elbow flexes FORWARD only (human; H2 -0.99..3.07): folded back 132 deg the forearm
    # passes through the shoulder-roll pod behind the shoulder (sweep.py found it).
    add_joint('%s_Elbow_Pitch' % side, a2, a3, (0.0, 0.0, Z['el'] - Z['sr']), (0, 1, 0), -2.3, 0.3, 'X6-60')
    add_joint('%s_Wrist_Yaw' % side, a3, hd, (0.0, 0.0, Z['wr'] - Z['el']), (0, 0, 1), -2.4, 2.4, 'X4-36')

add_cyl('Head_1', M['X4-36'] + 0.2, 0.030, 0.060, (0.0, 0.0, 0.030))
add_box('Head_2', 1.2, (0.120, 0.140, 0.170), (0.0, 0.0, 0.085))
add_joint('Head_Yaw', 'Trunk', 'Head_1', (0.0, 0.0, Z['hy2']), (0, 0, 1), -1.0, 1.0, 'X4-36')
add_joint('Head_Pitch', 'Head_1', 'Head_2', (0.0, 0.0, Z['hp2'] - Z['hy2']), (0, 1, 0), -0.35, 0.85, 'X4-36')


# ------------------------------------------------------------------- URDF ----
def geom_xml(shape, indent):
    kind, p = shape
    if kind == 'box':
        return '%s<geometry><box size="%.6f %.6f %.6f"/></geometry>' % (indent, *p)
    return '%s<geometry><cylinder radius="%.6f" length="%.6f"/></geometry>' % (indent, *p)


def urdf():
    o = ['<?xml version="1.0"?>', '<robot name="rose-full">',
         '  <!-- generated by full/rose_full_export.py - do not hand-edit -->',
         '  <link name="world"/>',
         '  <joint name="world_joint" type="floating">',
         '    <parent link="world"/>', '    <child link="Trunk"/>',
         '    <origin xyz="0 0 0" rpy="0 0 0"/>', '  </joint>']
    for l in LINKS:
        ix, iy, iz = l.inertia
        o += ['  <link name="%s">' % l.name,
              '    <inertial>',
              '      <origin xyz="%.6f %.6f %.6f" rpy="0 0 0"/>' % l.com,
              '      <mass value="%.6f"/>' % l.mass,
              '      <inertia ixx="%.8f" ixy="0" ixz="0" iyy="%.8f" iyz="0" izz="%.8f"/>'
              % (ix, iy, iz),
              '    </inertial>',
              '    <visual>',
              '      <origin xyz="%.6f %.6f %.6f" rpy="0 0 0"/>' % l.com,
              geom_xml(l.shape, '      '),
              '    </visual>']
        # Contact geometry ONLY on the feet. Give every link a collision shape and
        # the crouch pose folds thigh into shank, generating dozens of spurious
        # self-contacts and a solver that never converges. The K1 masks those with a
        # hardcoded SELF_COLLISION_ARTIFACTS pair list; restricting contact to the
        # feet is the cleaner equivalent and is what the standing sim actually needs.
        if l.name.endswith('foot_link'):
            o += ['    <collision>',
                  '      <origin xyz="%.6f %.6f %.6f" rpy="0 0 0"/>' % l.com,
                  geom_xml(l.shape, '      '),
                  '    </collision>']
        o += ['  </link>']
    for j in JOINTS:
        _, _, _, eff, vel = ACT[j.model]
        o += ['  <joint name="%s" type="revolute">' % j.name,
              '    <parent link="%s"/>' % j.parent,
              '    <child link="%s"/>' % j.child,
              '    <origin xyz="%.6f %.6f %.6f" rpy="0 0 0"/>' % j.origin,
              '    <axis xyz="%d %d %d"/>' % j.axis,
              '    <limit lower="%.4f" upper="%.4f" effort="%.1f" velocity="%.2f"/>'
              % (j.lo, j.hi, eff, vel),
              '    <dynamics damping="0.1" friction="0.0"/>',
              '  </joint>']
    o.append('</robot>')
    return '\n'.join(o) + '\n'


# ------------------------------------------------------------------- MJCF ----
def mjcf():
    kids = {}
    for j in JOINTS:
        kids.setdefault(j.parent, []).append(j)
    by_name = {l.name: l for l in LINKS}
    # sole depth below the trunk origin, accumulated down the leg chain
    sole = -LAY.Z_SOLE - (SOLE - 0.021)
    spawn = sole + 0.02

    def body(name, joint, depth):
        pad = '  ' * depth
        l = by_name[name]
        out = []
        pos = '0 0 0' if joint is None else '%.6f %.6f %.6f' % joint.origin
        out.append('%s<body name="%s" pos="%s">' % (pad, name, pos))
        if joint is None:
            out.append('%s  <freejoint name="world_joint"/>' % pad)
        else:
            _, _, _, eff, vel = ACT[joint.model]
            out.append('%s  <joint name="%s" type="hinge" axis="%d %d %d" '
                       'range="%.4f %.4f" armature="%.6f" damping="0.1"/>'
                       % (pad, joint.name, *joint.axis, joint.lo, joint.hi,
                          armature(joint.model)))
        ix, iy, iz = l.inertia
        out.append('%s  <inertial pos="%.6f %.6f %.6f" mass="%.6f" diaginertia="%.8f %.8f %.8f"/>'
                   % (pad, *l.com, l.mass, ix, iy, iz))
        if l.shape[0] == 'box':
            sx, sy, sz = l.shape[1]
            flags = '' if name.endswith('foot_link') else ' contype="0" conaffinity="0"'
            out.append('%s  <geom name="%s_col" type="box" size="%.6f %.6f %.6f" pos="%.6f %.6f %.6f"%s/>'
                       % (pad, name, sx/2, sy/2, sz/2, *l.com, flags))
        else:
            r, h = l.shape[1]
            out.append('%s  <geom name="%s_col" type="cylinder" size="%.6f %.6f" pos="%.6f %.6f %.6f" contype="0" conaffinity="0"/>'
                       % (pad, name, r, h/2, *l.com))
        for cj in kids.get(name, []):
            out += body(cj.child, cj, depth + 1)
        out.append('%s</body>' % pad)
        return out

    o = ['<mujoco model="rose-full">',
         '  <!-- generated by full/rose_full_export.py - do not hand-edit -->',
         '  <compiler angle="radian"/>',
         '  <option timestep="0.001" gravity="0 0 -9.81"/>',
         '  <worldbody>']
    o += ['  ' + ln for ln in body('Trunk', None, 1)]
    o[len(o) - len(body('Trunk', None, 1))] = \
        '    <body name="Trunk" pos="0 0 %.4f">' % spawn
    o += ['  </worldbody>', '</mujoco>']
    return '\n'.join(o) + '\n'


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else \
        os.path.expanduser('~/Developer/pond_assets/robots/rose-pro')
    os.makedirs(out, exist_ok=True)
    u = os.path.join(out, 'K1_22dof_floating.urdf')
    m = os.path.join(out, 'K1_22dof.xml')
    open(u, 'w').write(urdf())
    open(m, 'w').write(mjcf())

    total = sum(l.mass for l in LINKS)
    print('wrote %s' % u)
    print('wrote %s' % m)
    print('\nlinks %d (+world), joints %d revolute + 1 floating'
          % (len(LINKS), len(JOINTS)))
    print('total mass %.3f kg' % total)
    print('nq/nv expected: %d' % (len(JOINTS) + 6))
    bad = [l.name for l in LINKS if min(l.inertia) <= 0 or l.mass <= 0]
    print('non-SPD or massless links: %s' % (bad if bad else 'none'))
    names = [l.name for l in LINKS]
    print('duplicate link names: %s'
          % ([n for n in names if names.count(n) > 1] or 'none'))
    print('foot links: %s'
          % [l.name for l in LINKS if l.name.endswith('foot_link')])
    print('armature: ' + '  '.join('%s %.4f' % (m, armature(m)) for m in ('X15-450', 'X12-320', 'X10-200', 'X8-120', 'X6-60', 'X4-36')))


if __name__ == '__main__':
    main()
