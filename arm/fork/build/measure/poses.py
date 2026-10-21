"""The 50 gate poses and the calibration plate they are measured on.

Ten conical seats (90° countersinks) are cut in the calibration plate, which the lymb's foot bolts to
through its own 4 x M6 on Ø190, so seat positions are known in the arm's base frame to the router's
accuracy (~0.05 mm), far inside the 2.0 mm gate. Two seats sit on 2" risers (6061 square
bar, faced both ends, height measured with calipers and entered as BLOCK_H, each located by two Ø6 dowels)
to add height. Each seat is visited in 5 different arm
configurations (tool pitch and tool roll vary), so 10 x 5 = 50 poses.

    python3 poses.py            -> poses.csv, cal_plate.dxf, and a reachability summary
"""
import csv
import math
import os

import arm
S, F = arm.S, arm.F
S.grasp_local = lambda _F: arm.probe_local()    # IK solves for the probe ball, not the jaws

HERE = os.path.dirname(os.path.abspath(__file__))
PLATE = (0.380, 0.380)          # 1/4" 5052, one router sheet
BASE_IN_PLATE = (0.125, 0.190)  # foot centre on the plate
BALL_R = 0.003                  # Ø6 mm ball stylus
CSK_D = 0.006                   # countersink top diameter
BLOCK_H = 0.0508                # 2" 6061 riser; replace with its measured height
FOOT_BC = F.R_FOOT_BOLTS          # foot bolt circle radius, from the model (v9 Ø190)
# seats in the arm frame (x, y, on_block); spread in radius and azimuth
SEATS = [(0.16, -0.14, 0), (0.22, -0.16, 0), (0.18, 0.0, 0), (0.24, 0.0, 0), (0.16, 0.14, 0),
         (0.22, 0.16, 0), (0.20, -0.08, 1), (0.20, 0.08, 1), (0.13, -0.17, 0), (0.13, 0.17, 0)]
CONFIGS = [(90, 0), (75, 90), (60, 180), (45, -90), (30, 0)]   # (tool pitch below horizontal deg, J7 roll deg)


def table_z():
    """Model z of the surface the arm stands on (v10's desk puck reaches below the foot)."""
    import rose_arm_geom as Gm
    return min(min(p[2] for p in Gm.prim_points(pr[:7], step=0.01)) for pr in F.build() if pr[0] == 'base')


Z_TABLE = table_z()


def seat_z(on_block):
    """Ball centre height, model frame, when seated in a 90° countersink on the plate."""
    return Z_TABLE + BALL_R * math.sqrt(2) - CSK_D / 2 + (BLOCK_H if on_block else 0.0)


def solve(target, pitch, roll):
    for seed in (S.HOME, dict(S.HOME, J2_sho_pitch=-30, J4_elbow=60, J6_wrist_pitch=60),
                 dict(S.HOME, J2_sho_pitch=-100, J4_elbow=130, J6_wrist_pitch=-10)):
        q, ok = S.ik(F, target, pitch, seed)
        if ok:
            q['J7_tool_roll'] = roll
            q.setdefault('J3_upper_roll', 0.0)
            return q
    return None


def cap_ok(q, payload=0.0):
    """v10: reject poses over the J3 (RS02 upper-arm roll) static cap - arm/fork/pose_cap.py. q in degrees.
    v9 has no J3, so everything passes."""
    if not getattr(F, 'V10', False):
        return True
    import sys
    sys.path.insert(0, os.path.join(HERE, '..', '..'))
    import pose_cap
    return pose_cap.j3_ok({k: math.radians(v) for k, v in q.items()}, payload)


def build():
    rows, misses = [], []
    for i, (x, y, b) in enumerate(SEATS):
        tgt = (x, y, seat_z(b))
        got = 0
        for pitch, roll in CONFIGS + [(p, r) for p in (80, 65, 50, 40, 20) for r in (0, 90)]:
            if got == 5:
                break
            q = solve(tgt, pitch, roll)
            if q is None:
                continue
            if not cap_ok(q):
                continue
            got += 1
            rows.append(dict(pose=len(rows) + 1, seat=i + 1, x=tgt[0], y=tgt[1], z=tgt[2], pitch=pitch,
                             **{j: round(q[j], 3) for j in arm.JOINTS}))
        if got < 5:
            misses.append((i + 1, got))
    return rows, misses


def dxf(path):
    """Outline, foot bolt holes (Ø6.6) and seat centres (drill Ø3 through, then 90° countersink to Ø6)."""
    ox, oy = BASE_IN_PLATE
    ents = []
    W, H = PLATE
    for a, b in [((0, 0), (W, 0)), ((W, 0), (W, H)), ((W, H), (0, H)), ((0, H), (0, 0))]:
        ents.append(('LINE', 'CUT', a, b))
    for k in range(4):
        t = math.radians(45 + 90 * k)
        ents.append(('CIRCLE', 'CUT', (ox + FOOT_BC * math.cos(t), oy + FOOT_BC * math.sin(t)), 0.0033))
    for x, y, b in SEATS:
        if not b:
            ents.append(('CIRCLE', 'DRILL3_CSK6', (ox + x, oy + y), 0.0015))
    # riser dowels: two Ø6 reamed holes per riser, 32 mm apart across the seat
    for x, y, b in SEATS:
        if b:
            for dx in (-0.016, 0.016):
                ents.append(('CIRCLE', 'REAM6', (ox + x + dx, oy + y), 0.003))
    mm = lambda v: f'{v * 1000:.4f}'
    out = ['0', 'SECTION', '2', 'ENTITIES']
    for e in ents:
        if e[0] == 'LINE':
            out += ['0', 'LINE', '8', e[1], '10', mm(e[2][0]), '20', mm(e[2][1]), '11', mm(e[3][0]), '21', mm(e[3][1])]
        else:
            out += ['0', 'CIRCLE', '8', e[1], '10', mm(e[2][0]), '20', mm(e[2][1]), '40', mm(e[3])]
    out += ['0', 'ENDSEC', '0', 'EOF']
    open(path, 'w').write('\n'.join(out) + '\n')


if __name__ == '__main__':
    rows, misses = build()
    with open(os.path.join(HERE, 'poses.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    dxf(os.path.join(HERE, 'cal_plate.dxf'))
    print(f'{len(rows)} poses on {len(SEATS)} seats -> poses.csv, cal_plate.dxf')
    for s, n in misses:
        print(f'  seat {s}: only {n} configurations reachable')
