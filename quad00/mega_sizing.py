"""mega humanoid actuator sizing: heavy-duty human scale (Cam 2026-10-21): ~1.95 m, bulkier, 25-50 kg payload,
fits doors. Quasi-static joint torques for the governing load cases x a dynamic factor; continuous = the worst
held pose. Segment lengths from Winter's anthropometric fractions of height. All E (estimates) until a sim exists."""
import math
H = 1.95                              # m (US door 2.03 m clear: the head clears with ~80 mm)
M_ROBOT = 95.0                        # kg: H1-class 47 kg x (1.95/1.8)^3 = 60 kg, x1.6 "bulkier" (bigger actuators + structure)
PAY_MAX, PAY_CONT = 50.0, 25.0        # kg: 50 short lifts, 25 carried continuously
g = 9.81
thigh, shank, upper, fore = 0.245 * H, 0.246 * H, 0.186 * H, 0.146 * H + 0.05   # m (forearm to grip)
m_leg = 0.16 * M_ROBOT                # one leg
m_up = M_ROBOT - 2 * m_leg            # trunk + arms + head
DYN = 1.6                             # walking/stairs/stumble margin on peak
def tot(pay): return M_ROBOT + pay
cases = {}
# KNEE: single-leg stance on a stair with 45 deg flex, lever ~ thigh*sin(45)/2 from knee to body-CoM line; peak with 50 kg
lev_k = thigh * math.sin(math.radians(45)) / 2
cases['knee pitch'] = (DYN * (tot(PAY_MAX) - m_leg) * g * lev_k, (tot(PAY_CONT) - 2 * m_leg) * g / 2 * thigh * 0.35)   # cont: double-stance 60 deg squat hold
# HIP PITCH: stoop lift, payload 0.45 m and upper body 0.25 m forward of the hips, shared by 2 hips; single stance adds
cases['hip pitch'] = (DYN * (m_up * g * 0.25 + PAY_MAX * g * 0.45) / 2 * 1.3, (m_up * g * 0.10 + PAY_CONT * g * 0.30) / 2)
# HIP ROLL: single stance, body CoM 0.10 m lateral of the stance hip
cases['hip roll'] = (DYN * (tot(PAY_MAX) - m_leg) * g * 0.10, (tot(PAY_CONT) - m_leg) * g * 0.05)
cases['hip yaw'] = (0.35 * cases['hip roll'][0], 0.3 * cases['hip roll'][1])
# ANKLE: single stance, CoP at the toe 0.12 m ahead of the ankle; roll 0.05 m
cases['ankle pitch'] = (DYN * tot(PAY_MAX) * g * 0.12, tot(PAY_CONT) * g * 0.04 / 2)
cases['ankle roll'] = (DYN * tot(PAY_MAX) * g * 0.05, tot(PAY_CONT) * g * 0.02 / 2)
# WAIST PITCH: 50 kg at 0.40 m + trunk 0.6*m_up at 0.20 m (bent 45 deg)
cases['waist pitch'] = (DYN * (PAY_MAX * g * 0.40 + 0.6 * m_up * g * 0.20), PAY_CONT * g * 0.25 + 0.6 * m_up * g * 0.05)
cases['waist yaw'] = (0.4 * cases['waist pitch'][0], 0.2 * cases['waist pitch'][1])
# SHOULDER PITCH: half the payload per arm held 0.35 m forward (box hugged), arm 6 kg at 0.3 m; peak = arm out at 0.6 m
m_arm = 6.0
cases['shoulder pitch'] = (DYN * (PAY_MAX / 2 * g * 0.45 + m_arm * g * 0.30), PAY_CONT / 2 * g * 0.30 + m_arm * g * 0.15)
cases['shoulder roll'] = (0.6 * cases['shoulder pitch'][0], 0.5 * cases['shoulder pitch'][1])
cases['upper-arm yaw'] = (0.35 * cases['shoulder pitch'][0], 0.3 * cases['shoulder pitch'][1])
cases['elbow'] = (DYN * (PAY_MAX / 2 * g * fore + 2.5 * g * fore / 2), PAY_CONT / 2 * g * fore * 0.7)
cases['wrist'] = (DYN * PAY_MAX / 2 * g * 0.08, PAY_CONT / 2 * g * 0.05)
cases['neck'] = (8.0, 3.0)

# actuators: RobStride (quad.py; vendor peak/cont N.m, kg) + a reduction stage where needed; 'rana' = Cam's own (TBD)
ACT = {'rs04': (120.0, 25.0, 1.42), 'rs06': (36.0, 8.0, 0.62), 'rs02': (17.0, 4.4, 0.38), 'rs00': (14.0, 3.6, 0.31)}
def pick(pk, ct):
    for name, (p, c, kg) in [('rs02', ACT['rs02']), ('rs06', ACT['rs06']), ('rs04', ACT['rs04'])]:
        for n in (1.0, 2.0, 3.0, 4.0):
            if p * n * 0.9 >= pk and c * n * 0.9 >= ct:   # 90 % stage efficiency
                return name, n, kg + (0.0 if n == 1 else 0.25 * n)
    return 'rana (custom)', None, None
if __name__ == '__main__':
    print(f'H {H} m, robot {M_ROBOT} kg + payload {PAY_MAX:.0f} max / {PAY_CONT:.0f} cont; thigh {thigh:.2f} shank {shank:.2f} upper {upper:.2f} fore {fore:.2f} m')
    print('| joint | peak N.m | continuous N.m | actuator | stage | note |'); print('|---|---:|---:|---|---|---|')
    for j, (pk, ct) in cases.items():
        a, n, kg = pick(pk, ct)
        st = '-' if n is None else ('direct' if n == 1 else f'{n:.0f}:1')
        print(f'| {j} | {pk:.0f} | {ct:.0f} | {a} | {st} | |')
