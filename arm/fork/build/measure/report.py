"""Tool-point gate report: RMS error of model FK at the recorded joint angles against the seats.

    python3 report.py runs/A_0kg.csv runs/B_1kg.csv

Two numbers per run:
  raw         - joint zeros from the zero jig only. This is the gate number printed on the receipt.
  calibrated  - six joint offsets fitted on the odd poses, scored on the even ones (never on the poses
                they were fitted to). It says how much of the raw error a software zero would remove.
Gate: raw RMS <= 2.0 mm at 0 kg and at 1.0 kg.
"""
import csv
import math
import sys

import arm

GATE_MM = 2.0


def err(row, off):
    q = {j: math.radians(float(row[j]) + off.get(j, 0.0)) for j in arm.JOINTS}
    p = arm.tool_point(q)
    return [p[i] - float(row[k]) for i, k in enumerate('xyz')]


def rms(rows, off):
    return math.sqrt(sum(sum(v * v for v in err(r, off)) for r in rows) / len(rows)) * 1000


def fit(rows):
    """Gauss-Newton on 6 joint offsets (deg)."""
    off = {j: 0.0 for j in arm.JOINTS}
    for _ in range(15):
        J, r = [], []
        for row in rows:
            e0 = err(row, off)
            cols = []
            for j in arm.JOINTS:
                o2 = dict(off, **{j: off[j] + 0.01})
                cols.append([(a - b) / 0.01 for a, b in zip(err(row, o2), e0)])
            for k in range(3):
                J.append([c[k] for c in cols])
                r.append(-e0[k])
        n = len(arm.JOINTS)
        A = [[sum(J[i][a] * J[i][b] for i in range(len(J))) + (1e-9 if a == b else 0) for b in range(n)] for a in range(n)]
        bb = [sum(J[i][a] * r[i] for i in range(len(J))) for a in range(n)]
        d = solve(A, bb)
        for j, v in zip(arm.JOINTS, d):
            off[j] += v
    return off


def solve(A, b):
    n = len(b)
    M = [row[:] + [v] for row, v in zip(A, b)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(M[r][c]))
        M[c], M[p] = M[p], M[c]
        for r in range(n):
            if r != c and M[c][c]:
                f = M[r][c] / M[c][c]
                M[r] = [x - f * y for x, y in zip(M[r], M[c])]
    return [M[i][n] / M[i][i] if M[i][i] else 0.0 for i in range(n)]


def main():
    ok = True
    for path in sys.argv[1:]:
        rows = list(csv.DictReader(open(path)))
        raw = rms(rows, {})
        off = fit(rows[0::2])
        cal = rms(rows[1::2], off)
        worst = max(rows, key=lambda r: sum(v * v for v in err(r, {})))
        wmm = math.sqrt(sum(v * v for v in err(worst, {}))) * 1000
        load = rows[0]['load_kg']
        verdict = 'PASS' if raw <= GATE_MM and len(rows) == 50 else 'FAIL'
        ok &= verdict == 'PASS'
        print(f'{path}: {len(rows)} poses at {load} kg   raw {raw:.2f} mm RMS   calibrated {cal:.2f} mm   '
              f'worst pose {worst["pose"]} {wmm:.2f} mm   {verdict}')
        print('   fitted offsets (deg): ' + '  '.join(f'{j[:6]} {v:+.3f}' for j, v in off.items()))
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
