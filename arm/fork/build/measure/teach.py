"""Record the 50 gate poses by hand-guiding the probe ball into each seat.

The arm floats: every joint gets the model's gravity torque as feed-forward and only a light damper,
so Cam can move it by hand. Load, if any, hangs from the tool flange below the probe.
For each pose the script shows the target joint angles; guide the arm near them, let the ball settle
in the countersink, hands off, press Enter. It averages 0.5 s of encoder readings and writes a row.

    python3 teach.py --load 0    -> runs/<stamp>_0kg.csv
    python3 teach.py --load 1.0  -> runs/<stamp>_1kg.csv     (1.0 kg on the flange hook)
"""
import argparse
import csv
import math
import os
import threading
import time

import arm
import robstride as R

KD = {'RS04': 1.0, 'RS02': 0.15, 'RS00': 0.1}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--load', type=float, default=0.0)
    ap.add_argument('--start', type=int, default=1)
    a = ap.parse_args()
    jm = arm.load_map()
    bus = R.open_bus()
    motors = {j: R.Motor(bus, v['id'], v['model']) for j, v in jm.items()}
    poses = list(csv.DictReader(open(os.path.join(arm.HERE, 'poses.csv'))))
    os.makedirs(os.path.join(arm.HERE, 'runs'), exist_ok=True)
    out = os.path.join(arm.HERE, 'runs', time.strftime('%Y%m%d-%H%M') + f'_{a.load:g}kg.csv')
    q = {}
    stop = threading.Event()

    def float_loop():
        for m in motors.values():
            m.enable()
        while not stop.is_set():
            for j, m in motors.items():
                if m.pos is not None:
                    q[j] = arm.to_model(jm, j, m.pos)
            g = arm.gravity(q, a.load) if len(q) == 6 else {j: 0.0 for j in motors}
            for j, m in motors.items():
                tau = g[j] / jm[j]['sign']
                m.mit(m.pos or 0.0, 0.0, 0.0, KD[m.model], tau)
                if m.fault:
                    print(f'\n{j} FAULT 0x{m.fault:02x} - stopping')
                    stop.set()
            time.sleep(0.002)
        for m in motors.values():
            m.stop()

    th = threading.Thread(target=float_loop, daemon=True)
    th.start()
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['pose', 'seat', 'load_kg', 'x', 'y', 'z'] + arm.JOINTS + ['temp_max'])
        try:
            for p in poses[a.start - 1:]:
                tgt = '  '.join(f'{j[:6]} {float(p[j]):+6.1f}' for j in arm.JOINTS)
                input(f'pose {p["pose"]:>2} seat {p["seat"]:>2}  target {tgt}  -> seat the ball, Enter ')
                acc = {j: [] for j in arm.JOINTS}
                t0 = time.time()
                while time.time() - t0 < 0.5:
                    for j in arm.JOINTS:
                        acc[j].append(q[j])
                    time.sleep(0.01)
                qm = {j: sum(v) / len(v) for j, v in acc.items()}
                temp = max(m.temp or 0 for m in motors.values())
                w.writerow([p['pose'], p['seat'], a.load, p['x'], p['y'], p['z']] +
                           [f'{math.degrees(qm[j]):.4f}' for j in arm.JOINTS] + [temp])
                f.flush()
                e = [u - float(p[k]) for u, k in zip(arm.tool_point(qm), 'xyz')]
                print(f'   error {math.sqrt(sum(v * v for v in e)) * 1000:.2f} mm')
        finally:
            stop.set()
            th.join(1)
    print('wrote', out)


if __name__ == '__main__':
    main()
