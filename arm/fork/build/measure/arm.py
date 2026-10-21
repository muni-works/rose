"""The lymb as the measurement scripts see it: joint map, model FK, tool point, gravity torques.

joints.json (written during commissioning, see ../BUILD.md step 9) maps each model joint to a
CAN id, a motor model, a sign and a zero offset: q_model = sign * q_motor + offset.
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FORK = os.path.abspath(os.path.join(HERE, '..', '..'))
os.environ.setdefault('ROSE_ARM', 'v10')     # the arm being built; ROSE_ARM= (empty) for v9
sys.path.insert(0, FORK)
sys.path.insert(0, os.path.join(FORK, '..'))
import rose_fork as F            # noqa: E402
import fork_checks as C          # noqa: E402
import scenarios as S            # noqa: E402

G = 9.81
FIXED = {'J5_fore_roll'} if os.environ.get('ROSE_ARM') == 'v10' else set()   # v10 has no forearm roll
JOINTS = [l[1] for l in F.LINKS if l[1] and l[1] not in FIXED]
LINK_OF = {l[1]: l[0] for l in F.LINKS if l[1]}
AXIS_OF = {l[1]: l[3] for l in F.LINKS if l[1]}
ORDER = [l[0] for l in F.LINKS]
PROBE_L = 0.060   # ball centre past the tool flange face, m (Ø6 ball stylus on the flange adapter)
LOAD_KG = 1.0


def probe_local():
    return (F.flange_x() + PROBE_L, 0.0, F.Z_SH)


def tool_point(q_rad):
    """Probe ball centre in the base frame (m) for joint angles in rad."""
    T = F.fk(q_rad)
    return F.xf(T, 'tool', probe_local())


_MASSES = None


def masses():
    global _MASSES
    if _MASSES is None:
        _MASSES = S.link_masses(F, C)
    return _MASSES


def gravity(q_rad, payload=0.0):
    """Holding torque (N.m) each joint must supply, in model sign convention."""
    T = F.fk(q_rad)
    pts = []
    for link, rows in masses().items():
        if link in T:
            pts += [(link, m, F.xf(T, link, c)) for m, c in rows]
    pts.append(('tool', payload, tool_point(q_rad)))
    out = {}
    for j in JOINTS:
        link = LINK_OF[j]
        R, o = T[link]
        a = F.mv(R, AXIS_OF[j])
        down = ORDER[ORDER.index(link):]
        tau = 0.0
        for l, m, p in pts:
            if l in down:
                r = F.sub(p, o)
                f = (0.0, 0.0, -m * G)
                tau += (r[1] * f[2] - r[2] * f[1]) * a[0] + (r[2] * f[0] - r[0] * f[2]) * a[1] + \
                    (r[0] * f[1] - r[1] * f[0]) * a[2]
        out[j] = -tau     # the motor opposes gravity
    return out


def load_map(path=None):
    path = path or os.environ.get('LYMB_JOINTS') or os.path.join(HERE, 'joints.json')
    if os.environ.get('LYMB_SIM') and not os.path.exists(path):
        path = os.path.join(HERE, 'joints.example.json')
    with open(path) as f:
        return json.load(f)


def to_model(jm, j, q_motor):
    return jm[j]['sign'] * q_motor + jm[j]['offset']


def to_motor(jm, j, q_model):
    return (q_model - jm[j]['offset']) / jm[j]['sign']


def deg(q):
    return {k: math.degrees(v) for k, v in q.items()}


def rad(q):
    return {k: math.radians(v) for k, v in q.items()}
