"""Emit simulation and posed-render descriptions of the rose arm from rose_arm_layout.py.

Writes:
  sim/rose_arm.urdf            fixed-base URDF for phyz-urdf; inertials from the mass budget
  sim/rose_arm_params.json     what URDF cannot say: armature, motor limits, spring geometry, payload
  cad/rose-arm-assembly.loon   the same links as a jointed vcad assembly, for render_view joint_state

Each link carries the box(es) it is made of and the actuator whose STATOR bolts to
it (the output side is the next link). Every component is a solid box or cylinder
of its budgeted mass; inertia is summed about the link CoM with the parallel axis
theorem. Joint origins carry no rotation, so a link frame is its joint origin with
world axes in the zero pose (arm along +x, horizontal).
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rose_arm_layout as A

HERE = os.path.dirname(os.path.abspath(__file__))
DOF7 = True
SWEEP = (0.0, 0.5, 1.0, 1.5, 2.0, 2.5)

# rotor inertia (kg m^2) - RobStride does not publish it. ESTIMATED from MyActuator
# rotors of similar diameter (X8 D96 1.5e-4, X6 D80 0.66e-4, X4 D55 0.30e-4 in
# pro/rose_pro_layout.py). Reflected armature is J * N^2.
J_ROTOR = {'RS06': 1.0e-4, 'RS00': 0.30e-4, 'EduLite': 0.15e-4}
FRICTION = {'RS06': 0.30, 'RS00': 0.15, 'EduLite': 0.08}   # N.m backdrive, estimated (QDD, 9:1)
# Shorted-phase (dynamic braking) damping at the output, c = (Kt N)^2 / R. RobStride does not
# publish Kt or R; ESTIMATED from peak torque / ratio / ~40 A and ~0.3 ohm (RS06), ~25 A and
# 0.6 ohm (RS00). Only meaningful if the driver shorts the phases on fault - to be confirmed.
SHORT = {'RS06': 3.3, 'RS00': 0.8, 'EduLite': 0.3}
DAMPING = {'RS06': 0.05, 'RS00': 0.03, 'EduLite': 0.02}    # N.m.s/rad viscous, estimated

JOINTS = {j[0]: j for j in A.joints(DOF7)}
BOXES = {b[0]: b for b in A.boxes(DOF7)}

# link: (joint that drives it, parent link, boxes, stator-mounted actuators)
LINKS = [
    ('base',    None,             None,    ['base_plate'],            ['J1_yaw']),
    ('turret',  'J1_yaw',         'base',  ['col_floor', 'column'],   ['J2_sho_pitch']),
    ('upper_a', 'J2_sho_pitch',   'turret', ['ua_A'],                 ['J3_upper_roll']),
    ('upper_b', 'J3_upper_roll',  'upper_a', ['ua_B'],                ['J4_elbow']),
    ('fore_c',  'J4_elbow',       'upper_b', ['fa_C'],                ['J5_fore_roll']),
    ('fore_d',  'J5_fore_roll',   'fore_c', ['fa_D'],                 ['J6_wrist_pitch']),
    ('hand',    'J6_wrist_pitch', 'fore_d', ['hand'],                 ['J7_tool_roll']),
    ('tool',    'J7_tool_roll',   'hand',  ['grip_plate'],            ['gripper']),
]
LIMITS = {   # deg
    'J1_yaw': (-170, 170), 'J2_sho_pitch': (-120, 90), 'J3_upper_roll': (-170, 170),
    'J4_elbow': (-160, 160), 'J5_fore_roll': (-170, 170), 'J6_wrist_pitch': (-120, 120),
    'J7_tool_roll': (-180, 180),
}


def joint_origin(name):
    """The point the joint axis passes through, in the zero-pose world frame (m)."""
    if name is None:
        return (0.0, 0.0, 0.0)
    j = JOINTS[name]
    org, model, mode = j[1], j[3], j[5]
    if mode == 'mid' or j[2] in ((0, 1, 0), (0, 0, 1)):
        return org
    return org


def box_inertia(m, sx, sy, sz):
    return (m * (sy * sy + sz * sz) / 12, m * (sx * sx + sz * sz) / 12, m * (sx * sx + sy * sy) / 12)


def cyl_inertia(m, r, L, axis):
    ia = m * r * r / 2
    it = m * (3 * r * r + L * L) / 12
    return tuple(ia if axis[i] else it for i in range(3))


def components(link, payload=0.0):
    """(mass, centre (world, zero pose), principal diag in world axes) for everything on a link."""
    _, _, _, boxes, acts = next(l for l in LINKS if l[0] == link)
    out = []
    for b in boxes:
        _, p0, p1, _ = BOXES[b]
        sx, sy, sz = (p1[i] - p0[i] for i in range(3))
        if b == 'base_plate':
            m = A.SHEET_RHO * A.T_PLATE * sx * sy
        elif b == 'col_floor':
            m = A.SHEET_RHO * A.T_PLATE * sx * sy
        elif b == 'grip_plate':
            m = 0.02
        elif b in {q[0] for q in A.profiles(DOF7)}:
            prof = {q[0]: q for q in A.profiles(DOF7)}[b]
            m, xc = A.profile_mass(prof)
            c = (xc, (p0[1] + p1[1]) / 2, A.BASE_H)
            out.append((m, c, box_inertia(m, sx, sy, sz)))
            continue
        else:
            m = A.box_mass(p0, p1)
        c = tuple((p0[i] + p1[i]) / 2 for i in range(3))
        out.append((m, c, box_inertia(m, sx, sy, sz)))
    for a in acts:
        _, pa, pb, r, model = A.seg(JOINTS[a])
        c = tuple((pa[i] + pb[i]) / 2 for i in range(3))
        m = A.KG(model)
        axis = tuple(abs(JOINTS[a][2][i]) for i in range(3))
        out.append((m, c, cyl_inertia(m, r, A.LN(model), axis)))
    if link == 'tool':
        c = (A.GRIP_X, A.HA_YC, A.BASE_H - 0.04)
        out.append((A.M_GRIP_STRUCT, c, box_inertia(A.M_GRIP_STRUCT, 0.06, 0.04, 0.08)))
    if link == 'tool' and payload:  # payload as a point-ish mass at the flange
        out.append((payload, (A.REACH, A.HA_YC, A.BASE_H), box_inertia(payload, 0.05, 0.05, 0.05)))
    if link == 'upper_b':          # harness, lumped where the layout lumps it
        out.append((A.M_HARNESS, (0.35, A.UA_YC, A.BASE_H), box_inertia(A.M_HARNESS, 0.3, 0.02, 0.02)))
    return out


def link_inertial(link, payload=0.0):
    comps = components(link, payload)
    m = sum(c[0] for c in comps)
    com = tuple(sum(c[0] * c[1][i] for c in comps) / m for i in range(3))
    I = [[0.0] * 3 for _ in range(3)]
    for mm, c, d in comps:
        r = [c[i] - com[i] for i in range(3)]
        r2 = sum(v * v for v in r)
        for i in range(3):
            I[i][i] += d[i]
            for k in range(3):
                I[i][k] += mm * ((r2 if i == k else 0.0) - r[i] * r[k])
    return m, com, I


def urdf(payload=0.0):
    L = ['<?xml version="1.0"?>',
         '<!-- GENERATED by arm/rose_arm_export.py from arm/rose_arm_layout.py. Zero pose: arm along +x. -->',
         '<robot name="rose_arm">']
    for name, joint, parent, _, _ in LINKS:
        m, com, I = link_inertial(name, payload)
        o = joint_origin(joint)
        lc = tuple(com[i] - o[i] for i in range(3))
        L.append('  <link name="%s">' % name)
        L.append('    <inertial><origin xyz="%.5f %.5f %.5f"/><mass value="%.4f"/>' % (lc + (m,)))
        L.append('      <inertia ixx="%.6e" ixy="%.6e" ixz="%.6e" iyy="%.6e" iyz="%.6e" izz="%.6e"/></inertial>'
                 % (I[0][0], I[0][1], I[0][2], I[1][1], I[1][2], I[2][2]))
        L.append('  </link>')
    for name, joint, parent, _, _ in LINKS:
        if joint is None:
            continue
        o, po = joint_origin(joint), joint_origin(next(l[1] for l in LINKS if l[0] == parent))
        rel = tuple(o[i] - po[i] for i in range(3))
        model = JOINTS[joint][3]
        lo, hi = LIMITS[joint]
        L.append('  <joint name="%s" type="revolute">' % joint)
        L.append('    <parent link="%s"/><child link="%s"/>' % (parent, name))
        L.append('    <origin xyz="%.5f %.5f %.5f" rpy="0 0 0"/>' % rel)
        L.append('    <axis xyz="%d %d %d"/>' % JOINTS[joint][2])
        L.append('    <limit lower="%.4f" upper="%.4f" effort="%.1f" velocity="%.2f"/>'
                 % (math.radians(lo), math.radians(hi), A.MODEL[model][4], A.MODEL[model][6] * 2 * math.pi / 60))
        L.append('    <dynamics damping="%.3f" friction="%.3f"/>' % (DAMPING[model], FRICTION[model]))
        L.append('  </joint>')
    L.append('</robot>')
    return '\n'.join(L) + '\n'


def params():
    j = {}
    for name, joint, _, _, _ in LINKS:
        if joint is None:
            continue
        model = JOINTS[joint][3]
        _, _, mass, N, peak, rated, rpm, _ = A.MODEL[model]
        j[joint] = dict(model=model, peak=peak, rated=rated, no_load=rpm * 2 * math.pi / 60,
                        armature=J_ROTOR[model] * N * N, short=SHORT[model])
    # gas spring across J2: base anchor D below the pitch axis, arm anchor R along the upper arm.
    # Constant force sized to cancel the arm's self-weight moment at horizontal (|A-B| = sqrt(R^2+D^2)).
    R, D = A.R_SPRING, 0.120
    m_self = A.self_moment_past(0.0, DOF7)
    F = m_self * math.hypot(R, D) / (R * D)
    tool_o = joint_origin('J7_tool_roll')
    return dict(joints=j, spring=dict(r=R, d=D, force=F, self_moment=m_self),
                payload=dict(mass=A.PAYLOAD, xyz=[A.REACH - tool_o[0], A.HA_YC - tool_o[1], 0.0]),
                reach=A.REACH, upper=A.UPPER, fore=A.FORE)


def loon_assembly():
    mm = lambda v: v * 1e3
    parts, inst, jts, welds = [], [], [], []
    for name, joint, parent, boxes, acts in LINKS:
        o = joint_origin(joint)
        shapes = []
        prof = {q[0]: q for q in A.profiles(DOF7)}
        for b in boxes:
            _, p0, p1, _ = BOXES[b]
            if b in prof:
                import rose_arm_loon as LG
                shapes.append(LG.profile_solid(prof[b], o))
                continue
            import rose_arm_loon as LG
            rb = LG.rounded_box(b, p0, p1, o)
            if rb:
                shapes.append(rb)
                continue
            shapes.append('[translate %.1f %.1f %.1f [cube %.1f %.1f %.1f]]' % (
                mm(p0[0] - o[0]), mm(p0[1] - o[1]), mm(p0[2] - o[2]),
                mm(p1[0] - p0[0]), mm(p1[1] - p0[1]), mm(p1[2] - p0[2])))
        act_shapes = []
        for a in acts:
            j = JOINTS[a]
            _, pa, _, r, model = A.seg(j)
            rot = {(1, 0, 0): '[rotate 0 90 0 %s]', (0, 1, 0): '[rotate -90 0 0 %s]', (0, -1, 0): '[rotate 90 0 0 %s]',
                   (0, 0, 1): '%s', (0, 0, -1): '[rotate 180 0 0 %s]'}[tuple(j[4])]
            act_shapes.append('[translate %.1f %.1f %.1f %s]' % (
                mm(pa[0] - o[0]), mm(pa[1] - o[1]), mm(pa[2] - o[2]), rot % ('[cylinder %.1f %.1f]' % (mm(r), mm(A.LN(model))))))
        if name == 'tool':
            gx, gy = A.GRIP_X - o[0], A.HA_YC - o[1]
            for dx in (-0.011, 0.011):
                shapes.append('[translate %.1f %.1f %.1f [cube 6 8 54]]' % (mm(gx + dx) - 3, mm(gy) - 4, mm(-A.R('EduLite') - 0.06)))
        body = shapes[0]
        for s in shapes[1:]:
            body = '[union %s %s]' % (s, body)
        parts.append('    [part "%s" %s "%s"]' % (name, body, 'base' if name in ('base', 'turret') else 'sheet'))
        inst.append('    [instance "i-%s" "%s" %.1f %.1f %.1f]' % ((name, name) + tuple(mm(v) for v in o)))
        if act_shapes:     # actuators as their own dark part, welded to the link (keeps posed renders two-tone)
            ab = act_shapes[0]
            for x in act_shapes[1:]:
                ab = '[union %s %s]' % (x, ab)
            parts.append('    [part "%s_act" %s "actuator"]' % (name, ab))
            inst.append('    [instance "i-%s_act" "%s_act" %.1f %.1f %.1f]' % ((name, name) + tuple(mm(v) for v in o)))
            welds.append('    [fixed-joint "weld_%s" "i-%s" 0 0 0 "i-%s_act" 0 0 0]' % (name, name, name))
        if joint:
            po = joint_origin(next(l[1] for l in LINKS if l[0] == parent))
            lo, hi = LIMITS[joint]
            jts.append('    [revolute-joint "%s" %d %d %d %d %d "i-%s" %.1f %.1f %.1f "i-%s" 0 0 0]' % (
                (joint,) + JOINTS[joint][2] + (lo, hi, parent) + tuple(mm(o[i] - po[i]) for i in range(3)) + (name,)))
    return ('; rose arm as a jointed assembly. GENERATED by arm/rose_arm_export.py - pose it with joint_state.\n'
            '[material "base" 0.55 0.56 0.58 0.9 0.4]\n[material "sheet" 0.78 0.79 0.80 0.9 0.35]\n[material "actuator" 0.10 0.10 0.11 0.6 0.5]\n'
            '[assembly\n  #[\n%s\n  ]\n  #[\n%s\n  ]\n  #[\n%s\n  ]\n  "i-base"]\n'
            % ('\n'.join(parts), '\n'.join(inst), '\n'.join(jts + welds)))


def main():
    os.makedirs(os.path.join(HERE, 'sim'), exist_ok=True)
    with open(os.path.join(HERE, 'sim', 'rose_arm.urdf'), 'w') as f:
        f.write(urdf())
    with open(os.path.join(HERE, 'sim', 'rose_arm_payload.urdf'), 'w') as f:
        f.write(urdf(payload=A.PAYLOAD))
    for m in SWEEP:            # continuous-duty sweep
        with open(os.path.join(HERE, 'sim', 'rose_arm_payload_%.1f.urdf' % m), 'w') as f:
            f.write(urdf(payload=m))
    with open(os.path.join(HERE, 'sim', 'rose_arm_params.json'), 'w') as f:
        json.dump(params(), f, indent=2)
    with open(os.path.join(HERE, 'cad', 'rose-arm-assembly.loon'), 'w') as f:
        f.write(loon_assembly())
    tot = 0.0
    for name, *_ in LINKS:
        m, com, _ = link_inertial(name)
        tot += m
        print('  %-8s %6.3f kg  com (%+.3f %+.3f %+.3f)' % ((name, m) + com))
    print('  total    %6.3f kg   (layout says %.3f)' % (tot, A.arm_mass(DOF7)))
    p = params()['spring']
    print('  spring: %.0f N, r %.0f mm, d %.0f mm, self moment %.1f N.m' % (p['force'], p['r'] * 1e3, p['d'] * 1e3, p['self_moment']))


if __name__ == '__main__':
    main()
