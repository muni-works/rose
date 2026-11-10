"""Arms on rose: the mini fork arm on the full humanoid's shoulders (pro/cad/rose-pro.loon).

rose's own arms (shoA/B/C, elbA, hand and all four arm actuators) are dropped and a mini fork arm is
mounted at each shoulder with its yaw axis pointing forward: J1 becomes the shoulder ROLL (it lifts
the arm sideways), J2 the shoulder pitch, J4 the elbow, J6 the wrist, J7 the hand roll - roll, pitch,
pitch like a human arm, not a chicken wing. J1 lifts the arm now, so it is an RS04 (same three motor
models). The arm's foot and collar are left out; its J1 motor sits behind the shoulder, in the chest.

The mini's 476 mm (shoulder to jaw tips) is about rose's own arm: 227 + 236 mm shoulder to wrist.

Run:  python3 arm/rose_arms.py OUT.loon ['{"L": {...}, "R": {...}}']    then vcad-render OUT.loon
      (VCAD_LOON_PATH=pro/cad so the body's [use links.*] resolve)
"""
import json, math, os, re, sys
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..', '..')
os.environ.setdefault('ROSE_ARM', 'mini')
os.environ.setdefault('ROSE_ARM_ZSH', '0.120')     # shortest turret that clears the RS04 shoulder lobe
os.environ.setdefault('ROSE_ARM_MOTORS', 'RS04,RS04,RS02,RS00,RS00,RS00')   # J1 is a roll joint under gravity now
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, '..'))
import rose_fork as F

BODY = os.path.join(ROOT, 'pro', 'cad', 'rose-pro.loon')
SHO_Y, Z_SHO = 0.255, 0.462            # rose-pro shoulder-pitch output face (act_L_sho_pitch: y 176..255, z 462)
ARM_PARTS = re.compile(r'^\[root (?:[LR]_(?:shoA|shoB|shoC|elbA|hand)_\S+|act_[LR]_(?:sho_pitch|sho_roll|elb_pitch|wrist_yaw)) ')

# a pose people read: left arm offering a can forward, right arm waving
POSE = {
    'L': dict(J1_yaw=8, J2_sho_pitch=-35, J4_elbow=-75, J6_wrist_pitch=-20, J7_tool_roll=0),
    'R': dict(J1_yaw=-70, J2_sho_pitch=-20, J4_elbow=-20, J6_wrist_pitch=-30, J7_tool_roll=0),
}


def mount(side):
    """Rotation and offset that put the arm's base frame on the shoulder: arm z (the yaw axis) forward
    along +x, arm x (straight out at the zero pose) hanging down, arm y (the pitch axes) lateral. The
    J2 axis lands on rose's shoulder-pitch axis."""
    s = 1 if side == 'L' else -1
    ex, ez = (0.0, 0.0, -1.0), (1.0, 0.0, 0.0)
    ey = F.cross(ez, ex)
    R0 = tuple(tuple(c[i] for c in (ex, ey, ez)) for i in range(3))     # columns are the arm axes in world
    at = F.sub((0.0, s * SHO_Y, Z_SHO), F.mv(R0, (0.0, 0.0, F.Z_SH)))
    return R0, at


def build(pose, hold=None):
    hold = {'L': 'can'} if hold is None else hold
    body = [l for l in open(BODY).read().splitlines() if not ARM_PARTS.match(l)]
    mats = '\n'.join(l for l in F.MATERIALS.splitlines() if '"actuator"' not in l)
    L = body + [mats]
    for side in 'LR':
        R0, at = mount(side)
        q = {k: math.radians(v) for k, v in pose[side].items()}
        L.append(F.world_loon(q, at=at, prefix=side.lower() + 'a', header=False, R0=R0,
                              skip={('base', 'base'), ('base', 'printed')}))
        if side in hold:                      # a real object in the jaws, upright, centred on the grasp point
            import scenarios as Sc
            T = F.fk(q)
            gp = F.add(F.mv(R0, F.xf(T, 'tool', Sc.grasp_local(F))), at)
            L.append(Sc.obj_loon(F, hold[side], gp, side.lower() + 'o'))
    return '\n'.join(L) + '\n'


if __name__ == '__main__':
    pose = json.loads(sys.argv[2]) if len(sys.argv) > 2 else POSE
    open(sys.argv[1], 'w').write(build(pose))
    print(sys.argv[1])
