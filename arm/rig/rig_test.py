"""Elbow rig tests: one RS04 fork joint, test forearm 270 mm to the load bolt. Uses ../build/measure/robstride.py.

    python3 rig_test.py tare                    # hold horizontal unloaded: arm moment tau0
    python3 rig_test.py torque                  # 0.5..10 kg steps: reported torque vs m g L + tau0
    python3 rig_test.py heat --nm 25 --minutes 30            # continuous hold at a joint torque
    python3 rig_test.py duty --nm 35 --on 10 --off 10 --minutes 30
    python3 rig_test.py backlash                # +/-3 N.m reversal at a fixed command, dial at the load bolt
    python3 rig_test.py repeat                  # 10 returns to one position, dial at the load bolt

Joint angle 0 = test forearm horizontal (set with a digital level: `python3 rig_test.py zero`).
Every run writes runs/<test>_<stamp>.csv. The pass criteria are in RIG-TEST.md.
"""
import argparse
import csv
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'build', 'measure'))
import robstride as R  # noqa: E402

L = 0.270
G = 9.81
HANGER_KG = 0.20           # weigh yours and edit
KP, KD = 300.0, 4.0
T_LIMIT = 80.0
STATE = os.path.join(HERE, 'runs', 'tare.json')


def writer(name):
    os.makedirs(os.path.join(HERE, 'runs'), exist_ok=True)
    p = os.path.join(HERE, 'runs', f'{name}_{time.strftime("%Y%m%d-%H%M%S")}.csv')
    f = open(p, 'w', newline='')
    return p, f, csv.writer(f)


def hold(m, q, tau_ff, seconds, log=None, t0=None, check=True):
    end = time.time() + seconds
    t0 = t0 or time.time()
    last = 0.0
    while time.time() < end:
        m.mit(q, 0.0, KP, KD, tau_ff)
        if check and (m.fault or (m.temp or 0) > T_LIMIT):
            raise RuntimeError(f'fault 0x{m.fault:02x}, case {m.temp} °C')
        if log and time.time() - last > 1.0:
            last = time.time()
            log.writerow([round(last - t0, 1), round(m.pos, 5), round(m.tau, 3), m.temp])
        time.sleep(0.002)


def tau0():
    try:
        return json.load(open(STATE))['tau0']
    except (OSError, KeyError, ValueError):
        sys.exit('run `rig_test.py tare` first')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('test')
    ap.add_argument('--id', type=int, default=4)
    ap.add_argument('--nm', type=float, default=25.0)
    ap.add_argument('--minutes', type=float, default=30.0)
    ap.add_argument('--on', type=float, default=10.0)
    ap.add_argument('--off', type=float, default=10.0)
    a = ap.parse_args()
    m = R.Motor(R.open_bus(), a.id, 'RS04')
    if a.test == 'zero':
        m.stop(); m.set_zero(); print('zero set at the current angle'); return
    m.enable()
    try:
        if a.test == 'tare':
            hold(m, 0.0, 0.0, 3.0)
            acc = []
            for _ in range(200):
                m.mit(0.0, 0.0, KP, KD, 0.0); acc.append(m.tau); time.sleep(0.005)
            t = sum(acc) / len(acc)
            os.makedirs(os.path.dirname(STATE), exist_ok=True)
            json.dump({'tau0': t}, open(STATE, 'w'))
            print(f'tau0 = {t:.3f} N.m (unloaded test forearm, horizontal)')
        elif a.test == 'torque':
            t0 = tau0()
            p, f, w = writer('torque')
            w.writerow(['load_kg', 'model_nm', 'reported_nm', 'err_pct', 'temp'])
            for kg in (0.5, 1, 2, 3, 4, 6, 8, 10, 13):
                input(f'hang {kg} kg on the hanger (total incl. hanger), Enter ')
                hold(m, 0.0, 0.0, 3.0)
                acc = []
                for _ in range(200):
                    m.mit(0.0, 0.0, KP, KD, 0.0); acc.append(m.tau); time.sleep(0.005)
                rep = sum(acc) / len(acc)
                mod = kg * G * L + t0
                e = (abs(rep) - abs(mod)) / abs(mod) * 100
                w.writerow([kg, round(mod, 3), round(rep, 3), round(e, 1), m.temp]); f.flush()
                print(f'  {kg:4} kg  model {mod:6.2f}  reported {rep:6.2f} N.m  {e:+5.1f} %')
            print('wrote', p)
        elif a.test in ('heat', 'duty'):
            t0 = tau0()
            kg = (a.nm - t0) / (G * L)
            input(f'hang {kg:.2f} kg (incl. hanger) for {a.nm} N.m at the joint, Enter ')
            p, f, w = writer(a.test)
            w.writerow(['t', 'pos', 'tau', 'temp'])
            start = time.time()
            while time.time() - start < a.minutes * 60:
                if a.test == 'heat':
                    hold(m, 0.0, 0.0, a.minutes * 60, w, start)
                    break
                hold(m, 0.0, 0.0, a.on, w, start)
                hold(m, math.radians(-85), 0.0, a.off, w, start)     # hanging: near-zero torque
                f.flush()
            f.close()
            on = [abs(float(r['tau'])) for r in csv.DictReader(open(p)) if abs(float(r['pos'])) < 0.05]
            got = sum(on) / len(on) if on else 0.0
            if abs(got - a.nm) > 0.1 * a.nm:
                print(f'FAIL: joint held {got:.1f} N.m, not {a.nm} - check the load   wrote {p}')
            else:
                print(f'PASS hold: {a.minutes} min at {got:.1f} N.m, no fault, case under {T_LIMIT} °C   wrote {p}')
        elif a.test == 'backlash':
            p, f, w = writer('backlash')
            w.writerow(['tau_ff', 'dial_mm'])
            for tau in (3.0, -3.0, 3.0, -3.0):
                hold(m, 0.0, tau, 2.0, check=False)
                d = float(input(f'  feed-forward {tau:+} N.m: dial reading mm? '))
                w.writerow([tau, d]); f.flush()
            print('wrote', p, ' backlash deg = spread_mm / 270 * 57.3')
        elif a.test == 'repeat':
            p, f, w = writer('repeat')
            w.writerow(['i', 'pos', 'dial_mm'])
            for i in range(10):
                hold(m, math.radians(-60), 0.0, 2.0)
                hold(m, 0.0, 0.0, 3.0)
                d = float(input(f'  return {i + 1}: dial mm? '))
                w.writerow([i + 1, m.pos, d]); f.flush()
            print('wrote', p)
        else:
            sys.exit(f'unknown test {a.test}')
    except (RuntimeError, KeyboardInterrupt) as e:
        print(f'\nFAIL / stopped: {e}')
        input('support the forearm, Enter to release ')
    finally:
        m.stop()


if __name__ == '__main__':
    main()
