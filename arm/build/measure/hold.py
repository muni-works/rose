"""Hold and sag: the arm holds a pose under position control while the script logs torque and heat.

    python3 hold.py --pose out --load 1.27 --minutes 30     # rated hold: straight out, gripper-v2 rating
    python3 hold.py --pose out --load 0 --minutes 2         # sag reference

Pose 'out' is every joint at 0 (straight out, the worst case). Sag: zero a dial indicator on the
flange's top face at 0 kg, hang the load, read it - the motors hold position, so the indicator sees
structure + gearbox compliance (model: 0.13 mm structure only). Hold passes if no motor faults and
every case stays under 80 °C for the full time, with torque at J2 / J6 within 10 % of the model.
Writes runs/hold_<stamp>.csv: t, per-joint torque and temperature.
"""
import argparse
import csv
import os
import time

import arm
import robstride as R

KP = {'RS04': 300.0, 'RS02': 60.0, 'RS00': 40.0}
KD = {'RS04': 4.0, 'RS02': 1.0, 'RS00': 0.5}
POSES = {'out': {j: 0.0 for j in arm.JOINTS}}
T_LIMIT = 80.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pose', default='out')
    ap.add_argument('--load', type=float, default=0.0)
    ap.add_argument('--minutes', type=float, default=30)
    a = ap.parse_args()
    jm = arm.load_map()
    bus = R.open_bus()
    motors = {j: R.Motor(bus, v['id'], v['model']) for j, v in jm.items()}
    q = POSES[a.pose]
    g = arm.gravity(q, a.load)
    print('model holding torque N.m: ' + '  '.join(f'{j[:6]} {g[j]:+.2f}' for j in arm.JOINTS))
    for m in motors.values():
        m.enable()
    os.makedirs(os.path.join(arm.HERE, 'runs'), exist_ok=True)
    out = os.path.join(arm.HERE, 'runs', time.strftime('hold_%Y%m%d-%H%M') + f'_{a.load:g}kg.csv')
    t0, last, verdict = time.time(), 0, 'PASS'
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t'] + [f'tau_{j}' for j in arm.JOINTS] + [f'temp_{j}' for j in arm.JOINTS])
        try:
            while time.time() - t0 < a.minutes * 60:
                for j, m in motors.items():
                    m.mit(arm.to_motor(jm, j, q[j]), 0.0, KP[m.model], KD[m.model], g[j] / jm[j]['sign'])
                    if m.fault or (m.temp or 0) > T_LIMIT:
                        verdict = f'FAIL {j} fault 0x{m.fault:02x} temp {m.temp}'
                        raise KeyboardInterrupt
                if time.time() - last > 1.0:
                    last = time.time()
                    w.writerow([round(last - t0, 1)] + [round(motors[j].tau * jm[j]['sign'], 3) for j in arm.JOINTS] +
                               [motors[j].temp for j in arm.JOINTS])
                    f.flush()
                    print(f'{last - t0:6.0f} s  J2 {motors["J2_sho_pitch"].tau:+6.2f} N.m {motors["J2_sho_pitch"].temp:5.1f} °C'
                          f'   J6 {motors["J6_wrist_pitch"].tau:+5.2f} N.m {motors["J6_wrist_pitch"].temp:5.1f} °C', end='\r')
                time.sleep(0.002)
        except KeyboardInterrupt:
            print('\nhold interrupted - lower the arm by hand onto the rest before stopping')
            input('arm supported? Enter to release motors ')
        finally:
            for m in motors.values():
                m.stop()
    print(f'\n{verdict}   wrote {out}')


if __name__ == '__main__':
    main()
