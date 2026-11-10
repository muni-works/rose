"""v9 vs v10 numbers for V10.md: run once per arm (ROSE_ARM unset = v9, ROSE_ARM=v10), prints JSON.

    python3 arm/numbers_v10.py            # v9
    ROSE_ARM=v10 python3 arm/numbers_v10.py
"""
import collections
import io
import json
import os
import sys
import contextlib

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import rose_fork as F
import fork_checks as C
import dfm as D
import audit as A

out = {}
by = collections.defaultdict(float)
for l, m, x in C.masses():
    by[l] += m
out['mass_kg'] = round(sum(by.values()), 2)
out['moving_kg'] = round(sum(by[k] for k in C.MOVING if k in by), 2)
out['reach_mm'] = round(C.flange_x() * 1e3)
out['hold_model'] = {k: round(v, 2) for k, v in C.holds().items()}
C.GRIPPER[0] = 'v2'
out['reach_v2_mm'] = round(C.flange_x() * 1e3)
out['hold_v2'] = {k: round(v, 2) for k, v in C.holds().items()}
C.GRIPPER[0] = 'model'
sag, tw, _ = C.stiffness()
out['sag_mm'], out['twist_mm'] = round(sag * 1e3, 2), round(tw * 1e3, 2)
rng = {}
for j in ('J2_sho_pitch', 'J4_elbow', 'J6_wrist_pitch'):
    rng[j] = A.joint_range(j, -180, 180)
rng['roll'] = A.joint_range('J3_upper_roll' if F.V10 else 'J5_fore_roll', -180, 180)
out['ranges'] = rng
prims = F.build()
proc = collections.Counter(D.classify(p)[0] for p in prims)
out['parts'] = dict(proc)
out['router_unique'] = len({D.key(p) for p in prims if D.classify(p)[0].startswith('router')})
buf = io.StringIO()
with contextlib.redirect_stdout(buf):
    A.main()
txt = buf.getvalue()
sec = lambda name: txt.split('== ' + name)[1].split('\n==')[0].strip().splitlines()[1:]
out['interference'] = [l.strip() for l in sec('INTERFERENCE') if l.strip() != 'none']
out['crush'] = len([l for l in sec('CRUSH') if l.strip() != 'none'])
out['shear'] = len([l for l in sec('SHEAR') if l.strip() != 'none'])
out['stall_J2_Nm'] = round(C.moment(0.0), 1)
print(json.dumps(out, indent=1))
