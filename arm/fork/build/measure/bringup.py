"""Commissioning: find every motor, print live joint angles, set zeros on the zero jig.

    python3 bringup.py scan                 # ping ids 1..127, list who answers
    python3 bringup.py watch                # live model-frame angles + gravity torque (needs joints.json)
    python3 bringup.py zero J2_sho_pitch    # arm on the zero jig, motor disabled: this angle becomes 0

Zero jig (BUILD.md step 9): straight out, every pitch joint level within 0.1° on a digital level
on the plate edges, J5 / J7 with the flange screw holes square to the table.
"""
import sys
import time

import arm
import robstride as R


def main():
    bus = R.open_bus()
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'watch'
    if cmd == 'scan':
        for i in range(1, 128):
            for model in ('RS04', 'RS02', 'RS00'):
                m = R.Motor(bus, i, model)
                if m.read():
                    print(f'id {i}: answers, pos {m.pos:+.3f} rad, {m.temp:.1f} °C, fault 0x{m.fault:02x}')
                    break
        return
    jm = arm.load_map()
    motors = {j: R.Motor(bus, v['id'], v['model']) for j, v in jm.items()}
    if cmd == 'zero':
        j = sys.argv[2]
        motors[j].stop()
        motors[j].set_zero()
        print(f'{j}: zero set. Set its "offset" in joints.json to 0.')
        return
    while True:
        q = {}
        for j, m in motors.items():
            m.read()
            q[j] = arm.to_model(jm, j, m.pos)
        g = arm.gravity(q)
        print('  '.join(f'{j[:6]} {arm.math.degrees(q[j]):+7.2f}° g{g[j]:+6.1f}' for j in arm.JOINTS), end='\r')
        time.sleep(0.05)


if __name__ == '__main__':
    main()
