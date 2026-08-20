"""Emit a POSED rose-pro assembly loon.

The link modules stay zero-pose (they are the fabrication truth and the cache
keys); a pose is an ASSEMBLY concern. This wraps each imported root in a chain
of rotate-about-joint transforms, most-proximal outermost, so the same
links/*.loon render standing, striding or waving without touching a plate.

  python3 pose.py stride         -> pro/cad/rose-pro-stride.loon
  python3 pose.py --list

Angles are DEGREES about the joint's own axis, per side. Sign convention is
the URDF's (same as sweep.py's RANGES): y-joints positive = the child swings
FORWARD-DOWN for hip pitch / heel-up for knee (checked visually, 2026-08-19).
Mirroring: a right-side x- or z-axis joint takes the NEGATED angle, so a pose
written for the left reads symmetric ("both arms out" = one number).

Poses here must respect sweep.py's RANGES - single-joint clearance is proven
there; multi-joint combinations are eyeballed on the fast raster render before
any photoreal is spent. The ankle pushrods are visually rigid parallel links:
their cranks counter-rotate with ank_pitch, which this does NOT model - so
poses keep the ankles near zero where the rods read right.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rose_pro_links as L
import rose_pro_loon as R

CAD = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'cad')

# Kinematic chain per link (proximal -> distal). Pelvis is the base.
CHAIN = {
    'pelvisC': (), 'pelvisS': (), 'hipA': ('hip_pitch',),
    'hipB': ('hip_pitch', 'hip_roll'),
    'thigh': ('hip_pitch', 'hip_roll', 'hip_yaw'),
    'shank': ('hip_pitch', 'hip_roll', 'hip_yaw', 'knee'),
    'crankA': ('hip_pitch', 'hip_roll', 'hip_yaw', 'knee'),
    'crankB': ('hip_pitch', 'hip_roll', 'hip_yaw', 'knee'),
    'ankle': ('hip_pitch', 'hip_roll', 'hip_yaw', 'knee', 'ank_pitch'),
    'foot': ('hip_pitch', 'hip_roll', 'hip_yaw', 'knee', 'ank_pitch', 'ank_roll'),
    'trunkS': ('waist_yaw',), 'trunkC': ('waist_yaw',), 'shoC': ('waist_yaw',),
    'shoA': ('waist_yaw', 'sho_pitch'),
    'shoB': ('waist_yaw', 'sho_pitch', 'sho_roll'),
    'elbA': ('waist_yaw', 'sho_pitch', 'sho_roll', 'elb_pitch'),
    'hand': ('waist_yaw', 'sho_pitch', 'sho_roll', 'elb_pitch', 'wrist_yaw'),
    'neck': ('waist_yaw', 'head_yaw'),
    'head': ('waist_yaw', 'head_yaw', 'head_pitch'),
}
# Actuator BODIES ride the link their stator is on (the parent side of their
# own joint); pushrods ride the shank.
ACT_CHAIN = {
    'hip_pitch': (), 'hip_roll': ('hip_pitch',), 'hip_yaw': CHAIN['hipB'],
    'knee': CHAIN['thigh'], 'ank_mA': CHAIN['shank'], 'ank_mB': CHAIN['shank'],
    'ank_pitch': CHAIN['shank'], 'ank_roll': CHAIN['ankle'],
    'waist_yaw': (), 'sho_pitch': CHAIN['trunkS'], 'sho_roll': CHAIN['shoA'],
    'elb_pitch': CHAIN['shoB'], 'wrist_yaw': CHAIN['elbA'],
    'head_yaw': CHAIN['trunkC'], 'head_pitch': CHAIN['neck'],
}

# name -> {joint: deg or (degL, degR)}
POSES = {
    # mid-stride: left leg forward, right pushing off, arms counter-swinging
    'stride': {
        'hip_pitch': (-24.0, 18.0), 'knee': (28.0, 12.0),
        'ank_pitch': (-4.0, 6.0),
        'sho_pitch': (16.0, -20.0), 'elb_pitch': (-14.0, -30.0),
        'head_pitch': 4.0,
    },
    # right arm raised in a wave, weight shifted, head turned to the viewer
    'wave': {
        # abduction caps at 97 deg (sweep RANGES), which is only horizontal - the
        # RAISED arm comes from shoulder PITCH (-120: forward and up past the head),
        # with a little roll so the elbow sits out from the body
        'sho_roll': (4.0, 30.0), 'sho_pitch': (6.0, -120.0),
        'elb_pitch': (-18.0, -45.0), 'wrist_yaw': (0.0, 25.0),
        'waist_yaw': -8.0, 'head_yaw': -14.0, 'hip_pitch': (-3.0, 3.0),
        'knee': (5.0, 0.0),
    },
    # quiet contrapposto: nothing over 15 deg, the pose you photograph
    'stand': {
        'waist_yaw': 10.0, 'head_yaw': -12.0, 'head_pitch': 3.0,
        'hip_pitch': (0.0, -4.0), 'knee': (0.0, 8.0), 'ank_pitch': (0.0, -4.0),
        'sho_pitch': (5.0, -7.0), 'elb_pitch': (-10.0, -16.0),
    },
}


def angle(pose, joint, side):
    a = pose.get(joint, 0.0)
    dl, dr = (a, a) if not isinstance(a, tuple) else a
    d = dl if side != 'R' else dr
    if side == 'R' and L.JOINTS[joint]['ax'] in ('x', 'z'):
        d = -d          # mirrored joint, mirrored positive sense
    return d


def wrap(expr, chain, pose, side):
    """Wrap expr in per-joint rotations, most-proximal OUTERMOST."""
    for joint in reversed(chain):
        d = angle(pose, joint, side)
        if abs(d) < 1e-9:
            continue
        j = L.JOINTS[joint]
        c = list(j['c'])
        if side == 'R':
            c[1] = -c[1]
        rx, ry, rz = {'x': (d, 0, 0), 'y': (0, d, 0), 'z': (0, 0, d)}[j['ax']]
        expr = ('[translate %.4f %.4f %.4f [rotate %.4f %.4f %.4f '
                '[translate %.4f %.4f %.4f %s]]]'
                % (c[0], c[1], c[2], rx, ry, rz, -c[0], -c[1], -c[2], expr))
    return expr


def emit(name):
    pose = POSES[name]
    data = L.dump_json(os.path.expanduser('~/Developer/pond_assets/rose_pro_plates.json'))
    groups = {}
    for pl in data['plates']:
        groups.setdefault(pl['link'], []).append(pl)
    no_mirror = ('neck', 'trunkC', 'head', 'pelvisC')

    lines = ['; rose-pro POSED assembly: "%s". GENERATED by pro/pose.py; links stay zero-pose.' % name,
             '[material "actuator" 0.10 0.10 0.11 0.6 0.5]']
    for link in sorted(groups):
        names = R.link_names(link, groups[link], no_mirror)
        lines.append('[use links.%s [%s]]' % (link, ' '.join(n for n, _, _ in names)))
        for rn, mirror, _ in names:
            side = 'R' if mirror else ('C' if link in no_mirror else 'L')
            # centreline links still yaw with the waist etc: side C uses L angles
            lines.append('[root %s "aluminum"]'
                         % wrap(rn, CHAIN[link], pose, side))
    act_names = ['act_' + n for n, _ in R.actuator_exprs()]
    rods = ['rod_%s_%s' % (s, nm)
            for (nm, x, y, z0, z1) in getattr(L, 'RODS', []) for s in ('L', 'R')]
    lines.append('[use actuators [%s]]' % ' '.join(act_names + rods))
    for an in act_names:
        parts = an.split('_')          # act_L_hip_pitch / act_C_waist_yaw
        side, joint = parts[1], '_'.join(parts[2:])
        lines.append('[root %s "actuator"]' % wrap(an, ACT_CHAIN[joint], pose, side))
    for rn in rods:
        side = rn.split('_')[1]
        lines.append('[root %s "actuator"]' % wrap(rn, CHAIN['shank'], pose, side))

    out = os.path.join(CAD, 'rose-pro-%s.loon' % name)
    open(out, 'w').write('\n'.join(lines) + '\n')
    print('wrote %s' % out)
    return out


if __name__ == '__main__':
    if '--list' in sys.argv:
        print(' '.join(POSES))
    else:
        for nm in (sys.argv[1:] or POSES):
            emit(nm)
