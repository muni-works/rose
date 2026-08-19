"""Emit a simulation-ready robot description for rose-pro.

Single source of truth for the kinematics is pro/rose_pro_layout.py; this
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
HIP_Y, SHO_Y = 0.075, 0.150
THIGH, SHANK, ANK_DZ = 0.250, 0.250, 0.060
KNEE_BACK = 0.055    # knee axis behind the hip yaw axis; see rose_pro_layout.KNEE_BACK
SOLE = 0.050

# actuator data, verified from vendor STEP + the X-series V4 manual
ACT = {                      # mass kg, ratio, rotor inertia kg.m^2, peak Nm, max rad/s
    'X6-60': (0.82, 19.612, 0.66e-4, 60.0, 16.0),
    'X4-36': (0.36, 36.0,   0.30e-4, 34.0, 11.6),
    'X4-10': (0.26, 12.6,   0.30e-4, 10.0, 33.2),
}


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
# Masses below are MEASURED from the Fusion solids in pro/rose_pro_links.py
# (plate + angle stock, 2.70 g/cm3) plus the actuator each link carries.
# Trunk = 1776 g frame + 2620 g of actuators it holds (2x hip pitch, 2x shoulder
# pitch, head yaw) + 1500 g battery + 500 g compute.
add_box('Trunk', 6.38, (0.130, 0.190, 0.350), (0.0, 0.0, 0.115))

for side, s in (('Left', 1.0), ('Right', -1.0)):
    lo_roll, hi_roll = (-0.5, 1.5) if s > 0 else (-1.5, 0.5)
    p = '%s_Hip_Pitch' % side
    r = '%s_Hip_Roll' % side
    y = '%s_Hip_Yaw' % side
    sh = '%s_Shank' % side
    ac = '%s_Ankle_Cross' % side
    ft = '%s_foot_link' % ('left' if s > 0 else 'right')

    # link masses = the actuator each link carries (stator side) + its structure
    add_box(p,  0.94, (0.090, 0.090, 0.085), (0.0, 0.0, -0.0425))
    add_box(r,  0.91, (0.090, 0.090, 0.051), (0.0, 0.0, -0.0255))
    add_cyl(y,  0.92, 0.045, 0.114, (0.0, 0.0, -0.057))
    add_cyl(sh, 0.54, 0.038, 0.250, (0.0, 0.0, -0.125))
    add_box(ac, 0.44, (0.060, 0.060, 0.060), (0.0, 0.0, -0.030))
    add_box(ft, 0.17, (0.180, 0.075, 0.022), (0.026, 0.0, -(ANK_DZ - 0.021)), shrink=False)

    add_joint(p,  'Trunk', p, (0.0, s*HIP_Y, -0.060), (0, 1, 0), -2.0, 1.6, 'X6-60')
    add_joint(r,  p, r, (0.0, 0.0, -0.085), (1, 0, 0), lo_roll, hi_roll, 'X6-60')
    add_joint(y,  r, y, (0.0, 0.0, -0.051), (0, 0, 1), -1.0, 1.0, 'X6-60')
    add_joint('%s_Knee_Pitch' % side, y, sh, (-KNEE_BACK, 0.0, -0.114), (0, 1, 0), 0.0, 2.3, 'X6-60')
    add_joint('%s_Ankle_Pitch' % side, sh, ac, (0.0, 0.0, -SHANK), (0, 1, 0), -0.9, 0.5, 'X4-36')
    add_joint('%s_Ankle_Roll' % side, ac, ft, (0.0, 0.0, -ANK_DZ), (1, 0, 0), -0.4, 0.4, 'X4-36')

    a1, a2, a3 = '%s_Arm_1' % side, '%s_Arm_2' % side, '%s_Arm_3' % side
    hd = '%s_hand_link' % ('left' if s > 0 else 'right')
    add_box(a1, 0.43, (0.060, 0.060, 0.065), (0.0, 0.0, -0.0325))
    add_cyl(a2, 0.44, 0.028, 0.125, (0.0, 0.0, -0.0625))
    add_cyl(a3, 0.39, 0.025, 0.155, (0.0, 0.0, -0.0775))
    add_box(hd, 0.065, (0.050, 0.030, 0.075), (0.0, 0.0, -0.0375))
    lo_sr, hi_sr = (-1.7, 1.6) if s > 0 else (-1.6, 1.7)
    add_joint('%s_Shoulder_Pitch' % side, 'Trunk', a1, (0.0, s*SHO_Y, 0.230), (0, 1, 0), -3.0, 1.5, 'X4-36')
    add_joint('%s_Shoulder_Roll' % side, a1, a2, (0.0, 0.0, -0.065), (1, 0, 0), lo_sr, hi_sr, 'X4-36')
    add_joint('%s_Elbow_Pitch' % side, a2, a3, (0.0, 0.0, -0.125), (0, 1, 0), -2.3, 2.3, 'X4-36')
    add_joint('%s_Wrist_Yaw' % side, a3, hd, (0.0, 0.0, -0.155), (0, 0, 1), -2.4, 2.4, 'X4-36')

add_cyl('Head_1', 0.30, 0.030, 0.060, (0.0, 0.0, 0.030))
# head = 201 g cage + stereo pair + interface board
add_box('Head_2', 0.41, (0.078, 0.088, 0.108), (0.0, 0.0, 0.056))
add_joint('Head_Yaw', 'Trunk', 'Head_1', (0.0, 0.0, 0.330), (0, 0, 1), -1.0, 1.0, 'X4-10')
add_joint('Head_Pitch', 'Head_1', 'Head_2', (0.0, 0.0, 0.060), (0, 1, 0), -0.35, 0.85, 'X4-10')


# ------------------------------------------------------------------- URDF ----
def geom_xml(shape, indent):
    kind, p = shape
    if kind == 'box':
        return '%s<geometry><box size="%.6f %.6f %.6f"/></geometry>' % (indent, *p)
    return '%s<geometry><cylinder radius="%.6f" length="%.6f"/></geometry>' % (indent, *p)


def urdf():
    o = ['<?xml version="1.0"?>', '<robot name="rose-pro">',
         '  <!-- generated by pro/rose_pro_export.py - do not hand-edit -->',
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
    sole = 0.060 + 0.085 + 0.051 + 0.114 + SHANK + ANK_DZ + 0.021
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

    o = ['<mujoco model="rose-pro">',
         '  <!-- generated by pro/rose_pro_export.py - do not hand-edit -->',
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
    print('armature: X6-60 %.4f  X4-36 %.4f  X4-10 %.5f kg.m2'
          % (armature('X6-60'), armature('X4-36'), armature('X4-10')))


if __name__ == '__main__':
    main()
