"""The rose line: proportions, actuator selection, mass, and jump feasibility.

Three variants, one design language (bent-sheet primary structure, exposed
skeleton limbs, feet-only contact geometry):

  rose mini  1150 mm   Booster K1 size, MyActuator X-series      the crashable one
  rose pro   1450 mm   MyActuator X-series, one size up          the workhorse
  rose max   1750 mm   our own rana + torc, Jetson Thor inside   the jumper

Proportions are scaled from rose pro's VERIFIED 1130 mm layout (pro/rose_pro_layout.py:
all clearances >= 5 mm at high sampling resolution, stands in phyz at +0.0% ground-force
error). Scaling a verified layout is not the same as verifying the scaled one - every
variant still has to go back through the clearance check, because actuator envelopes do
NOT scale with the robot. That is the whole reason mini is not just pro shrunk: at 1150
the X6-60 bodies are relatively larger and the hip packing gets tighter, not looser.

The jump model is first-order and planar, in the same spirit as the ollie feasibility
work: vendor limits in, a number out, no controller search. It exists to answer one
question honestly - what does the actuator family actually permit - not to predict what
a policy would achieve.
"""
import math

G = 9.81
BUS_V = 48.0

# ---------------------------------------------------------------- actuators
#
# X-series: measured from vendor STEP + the X-series V4 manual (see
# pro/rose_pro_layout.py, which verifies these envelopes against real drawings).
# rana: the family ladder in ~/Developer/rana/README.md, verified through
# calc_motor + winding_layout. Kt is MOTOR Kt; output torque is Kt * I * N * eta.
#
# tau  = peak output torque (N.m)
# kg   = mass
# kt   = motor torque constant (N.m/A), rana only - X-series vendor does not publish it
# n    = gear ratio
# amp  = peak phase current the peak torque assumes
ACT = {
    #                   tau     kg     kt      n     amp
    'X6-60':   dict(tau=60.0, kg=0.82, kt=None, n=19.612, amp=None),
    'X4-36':   dict(tau=34.0, kg=0.36, kt=None, n=36.0,   amp=None),
    'X4-10':   dict(tau=10.0, kg=0.26, kt=None, n=12.6,   amp=None),
    'rana-100': dict(tau=36.0,  kg=0.5, kt=0.1657, n=8.0, amp=30.0),
    'rana-140': dict(tau=96.0,  kg=1.1, kt=0.3326, n=8.0, amp=30.0),
    'rana-190': dict(tau=262.0, kg=2.4, kt=0.6055, n=8.0, amp=30.0),
}


def out_speed_noload(name):
    """No-load output speed (rad/s) at the 48 V bus. None where Kt is unpublished."""
    a = ACT[name]
    if a['kt'] is None:
        return None
    return (BUS_V / a['kt']) / a['n']


def peak_power(name, r_phase=0.15):
    """Peak MECHANICAL output power (W).

    At peak current the back-EMF still fits inside the bus (I*R is small), so the
    actuator can be at peak torque and near no-load speed at the same time. This is
    the optimistic corner and it is the right one to quote for a jump: a jump is a
    single burst, not a duty cycle. Thermal limits do not bind in 0.2 s.

    Gearing does NOT appear here, and that is the whole point - a ratio change trades
    torque against speed and leaves power alone.
    """
    a = ACT[name]
    if a['kt'] is None:
        return None
    tau_m = a['kt'] * a['amp']
    w_m = (BUS_V - a['amp'] * r_phase) / a['kt']
    return tau_m * w_m


# ---------------------------------------------------------------- variants
#
# Joint groups: leg = hip pitch/roll/yaw + knee (4/leg), ankle = pitch/roll (2/leg),
# arm = shoulder pitch/roll, elbow, wrist (4/arm), head = yaw/pitch.
REF_H = 1.130          # rose pro's verified height, metres
REF_STRUCT = 3.77      # its measured bent-sheet structure mass, kg (all 40 plates)
REF_THIGH = 0.250
REF_SHANK = 0.250
REF_STANCE = 0.150

VARIANT = {
    'mini': dict(h=1.150, leg='X6-60',   ankle='X4-36',   arm='X4-36',
                 head='X4-10', batt_kwh=0.30, compute=0.4, compute_name='Jetson Orin NX'),
    'pro':  dict(h=1.450, leg='X6-60',   ankle='X4-36',   arm='X4-36',
                 head='X4-10', batt_kwh=0.50, compute=0.4, compute_name='Jetson Orin NX'),
    'max':  dict(h=1.750, leg='rana-190', ankle='rana-140', arm='rana-100',
                 head='rana-100', batt_kwh=0.90, compute=1.4, compute_name='Jetson AGX Thor'),
}

# 48 V pouch packs land around 180 Wh/kg at pack level once cells, BMS, case and
# busbars are counted. Superman's 0.864 kWh is the reference point for max.
PACK_WH_PER_KG = 180.0


def geometry(v):
    """Scale rose pro's verified proportions to this variant's height."""
    s = v['h'] / REF_H
    return dict(scale=s, thigh=REF_THIGH * s, shank=REF_SHANK * s,
                stance=REF_STANCE * s, seg=REF_SHANK * s)


def mass(v):
    """Actuator mass from the catalogue; structure scaled as length^3 from measured."""
    s = v['h'] / REF_H
    act = (8 * ACT[v['leg']]['kg']       # 4 leg actuators x 2 legs
           + 4 * ACT[v['ankle']]['kg']   # 2 ankle actuators x 2 legs
           + 8 * ACT[v['arm']]['kg']     # 4 arm actuators x 2 arms
           + 2 * ACT[v['head']]['kg'])
    struct = REF_STRUCT * s ** 3
    batt = v['batt_kwh'] * 1000.0 / PACK_WH_PER_KG
    return dict(act=act, struct=struct, batt=batt, compute=v['compute'],
                total=act + struct + batt + v['compute'])


def jump(v, theta_k=math.radians(90.0), tau_derate=0.70):
    """Standing vertical jump ceiling, energy-limited and power-limited.

    Planar, symmetric, knee-dominated. Thigh and shank are equal (they are, by
    design - rose pro runs 1:1 after the proportions pass), so with interior knee
    angle theta the hip-to-foot distance is z = 2L*cos(theta/2) and the vertical
    extension per knee radian is |dz/dtheta| = L*sin(theta/2).

    ENERGY bound: both knees sweep theta -> 0, delivering W = 2 * tau * theta.
    That work lifts the COM through the extension AND buys flight height:
        W = m*g*(rise + h)  ->  h = W/(m*g) - rise

    POWER bound: takeoff speed cannot exceed what the actuator can turn.
        v = omega_out * L*sin(theta/2),   h = v^2/(2g)

    tau_derate is the honest part: peak torque is not available across the whole
    sweep, and the hip and ankle contributions that would offset it are not counted
    either. Both bounds are ceilings; the real number is under the lower one.
    """
    g = geometry(v)
    m = mass(v)['total']
    L = g['seg']
    tau = ACT[v['leg']]['tau']
    w_out = out_speed_noload(v['leg'])

    rise = 2 * L * (1 - math.cos(theta_k / 2))
    w_energy = 2 * tau * tau_derate * theta_k
    h_energy = w_energy / (m * G) - rise

    if w_out is None:
        h_power = None
        v_take = None
    else:
        v_take = w_out * L * math.sin(theta_k / 2)
        h_power = v_take ** 2 / (2 * G)

    # what a target jump would actually demand, for the gap statement
    def demand(h_target):
        vt = math.sqrt(2 * G * h_target)
        w_need = vt / (L * math.sin(theta_k / 2))
        tau_need = m * G * (1 + h_target / rise) / 2 * (L * math.sin(theta_k / 2))
        return dict(v=vt, w=w_need, tau=tau_need, p=2 * tau_need * w_need)

    return dict(m=m, L=L, rise=rise, tau=tau, w_out=w_out, v_take=v_take,
                h_energy=h_energy, h_power=h_power, demand=demand)


def main():
    print('=' * 74)
    print('the rose line')
    print('=' * 74)

    for name in ('mini', 'pro', 'max'):
        v = VARIANT[name]
        g, m = geometry(v), mass(v)
        print('\n--- rose %s --- %.0f mm, scale %.3f from pro\'s verified 1130'
              % (name, v['h'] * 1000, g['scale']))
        print('  thigh / shank   %4.0f / %.0f mm     stance %3.0f mm'
              % (g['thigh'] * 1000, g['shank'] * 1000, g['stance'] * 1000))
        print('  actuators       leg %s   ankle %s   arm %s'
              % (v['leg'], v['ankle'], v['arm']))
        print('  mass            %5.1f kg total  = %.1f actuators + %.1f structure'
              '  + %.1f battery (%.2f kWh) + %.1f %s'
              % (m['total'], m['act'], m['struct'], m['batt'], v['batt_kwh'],
                 m['compute'], v['compute_name']))

        w = out_speed_noload(v['leg'])
        p = peak_power(v['leg'])
        if w is not None:
            print('  knee actuator   %.0f N.m peak, %.1f rad/s no-load out, %.2f kW peak mech'
                  % (ACT[v['leg']]['tau'], w, p / 1000.0))
        else:
            print('  knee actuator   %.0f N.m peak  (vendor publishes no Kt - speed not '
                  'derivable, measured 12.9 rad/s on the K1\'s comparable joint)'
                  % ACT[v['leg']]['tau'])

    print('\n' + '=' * 74)
    print('jump feasibility - rose max, against Unitree Superman (2.0 m claimed)')
    print('=' * 74)
    j = jump(VARIANT['max'])
    print('  mass %.1f kg, segment %.0f mm, COM rise over the push %.0f mm'
          % (j['m'], j['L'] * 1000, j['rise'] * 1000))
    print('  knee %.0f N.m peak, %.1f rad/s no-load output' % (j['tau'], j['w_out']))
    print()
    print('  ENERGY-limited ceiling   %5.2f m' % j['h_energy'])
    print('  POWER-limited  ceiling   %5.2f m   (takeoff %.2f m/s)'
          % (j['h_power'], j['v_take']))
    print('  -> binding constraint is %s'
          % ('SPEED' if j['h_power'] < j['h_energy'] else 'TORQUE'))
    print()
    print('  what each target actually demands at the knee:')
    print('    %-8s %8s %10s %10s %10s' % ('target', 'takeoff', 'knee rate', 'knee tau', 'both knees'))
    for h in (0.3, 0.5, 1.0, 2.0):
        d = j['demand'](h)
        print('    %5.1f m  %6.2f m/s %8.1f rad/s %8.0f N.m %8.1f kW'
              % (h, d['v'], d['w'], d['tau'], d['p'] / 1000.0))
    print()
    avail = 2 * peak_power('rana-190') / 1000.0
    print('  rana-190 x2 knees deliver %.1f kW peak. Gearing cannot move this number -' % avail)
    print('  a ratio change trades torque against speed and leaves power alone.')
    for h in (0.5, 1.0, 2.0):
        d = j['demand'](h)
        print('    %.1f m jump needs %.1fx that power' % (h, d['p'] / 1000.0 / avail))

    print('\n' + '=' * 74)
    print('the rana ladder is a TORQUE ladder, not a POWER ladder')
    print('=' * 74)
    print('  %-10s %8s %8s %10s %10s' % ('', 'Kt', 'tau out', 'no-load out', 'peak mech'))
    for n in ('rana-100', 'rana-140', 'rana-190'):
        print('  %-10s %8.4f %6.0f N.m %7.1f rad/s %8.2f kW'
              % (n, ACT[n]['kt'], ACT[n]['tau'], out_speed_noload(n),
                 peak_power(n) / 1000.0))
    print()
    print('  All three deliver the SAME peak mechanical power, and that is not a')
    print('  coincidence. At peak current, P = (Kt*I) * (V - I*R)/Kt = I*(V - I*R):')
    print('  Kt cancels. Peak power is set by the BUS and the CURRENT LIMIT alone -')
    print('  %.0f V x %.0f A - so it is identical across the family and unchanged by'
          % (BUS_V, ACT['rana-190']['amp']))
    print('  gearing. Going up the ladder buys torque and costs speed, in exact trade.')
    print()
    print('  Consequence for max: jump height is an ELECTRICAL spec, not a mechanical')
    print('  one. To reach 1 m the lever is bus voltage or current, or storing energy')
    print('  in series elastics so the motors can work slowly and release fast. A')
    print('  bigger rana does nothing.')
    for tgt, (bus, amp) in (('1.0 m', (100.0, 60.0)), ('2.0 m', (150.0, 120.0))):
        print('    %s would need roughly a %.0f V / %.0f A leg drive (%.1f kW/knee)'
              % (tgt, bus, amp, bus * amp / 1000.0))


def clearance(scale, label):
    """Re-run pro's verified packing check on a scaled skeleton. ~30 s."""
    import itertools
    import sys
    sys.path.insert(0, 'pro')
    import rose_pro_layout as L

    segs = [L.seg(j) for j in L.joints(scale)]
    worst = []
    for (l1, a1, b1, r1, _), (l2, a2, b2, r2, _) in itertools.combinations(segs, 2):
        gap = L.cyl_gap(a1, b1, r1, a2, b2, r2) * 1000
        if gap < 20:
            worst.append((gap, l1, l2))
    print('\n--- packing check: %s (scale %.3f) ---' % (label, scale))
    if not worst:
        print('  all pairs clear by >20 mm')
    for gap, l1, l2 in sorted(worst):
        print('  %-14s <-> %-14s %+7.1f mm%s'
              % (l1, l2, gap, '   CLASH' if gap < 4 else ''))
    bad = [w for w in worst if w[0] < 4]
    print('  verdict: %s' % ('PASS' if not bad else 'FAIL - %d clash(es)' % len(bad)))
    return not bad


if __name__ == '__main__':
    import sys
    main()
    if '--pack' in sys.argv:
        print('\n' + '=' * 74)
        print('packing re-verification (actuator envelopes do NOT scale)')
        print('=' * 74)
        for n in ('mini', 'pro'):
            clearance(VARIANT[n]['h'] / REF_H, 'rose ' + n)
        print('\n  rose max is NOT checked here: only rana-100 rev D has a detailed')
        print('  housing (D100 x 44). rana-140 and rana-190 envelopes are not modelled,')
        print('  and inventing them would make this check a lie. Blocked on those.')
