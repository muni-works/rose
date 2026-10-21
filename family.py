"""The rose line: proportions, actuator selection, mass, and jump feasibility.

Three variants, one design language (bent-sheet primary structure, exposed
skeleton limbs, feet-only contact geometry):

  rose mini  1150 mm   Booster K1 size, MyActuator X-series      the crashable one
  rose full   1450 mm   MyActuator X-series, one size up          the workhorse
  rose mega   1750 mm   our own rana + torc, Jetson Thor inside   the jumper

Proportions are scaled from rose full's VERIFIED 1130 mm layout (full/rose_full_layout.py:
all clearances >= 5 mm at high sampling resolution, stands in phyz at +0.0% ground-force
error). Scaling a verified layout is not the same as verifying the scaled one - every
variant still has to go back through the clearance check, because actuator envelopes do
NOT scale with the robot. That is the whole reason mini is not just full shrunk: at 1150
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
# full/rose_full_layout.py, which verifies these envelopes against real drawings).
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
    # Bigger X-series SKUs exist and the vendor publishes their torque, but we have
    # no STEP for them, so mass is ESTIMATED (scaled from X6-60 by torque) and the
    # envelope is unmodelled - which means any variant using these cannot have its
    # packing checked. Torque is real; everything else here is a placeholder.
    'X8':      dict(tau=110.0, kg=1.4, kt=None, n=None, amp=None, est=True),
    'X12':     dict(tau=320.0, kg=3.6, kt=None, n=None, amp=None, est=True),
    'X15':     dict(tau=450.0, kg=5.2, kt=None, n=None, amp=None, est=True),
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
REF_H = 1.130          # rose full's verified height, metres
REF_STRUCT = 3.77      # its measured bent-sheet structure mass, kg (all 40 plates)
REF_THIGH = 0.250
REF_SHANK = 0.250
REF_STANCE = 0.150

# Each height is anchored to something real rather than picked off a curve, and
# rounded to the nearest 50 mm.
#
#   mini 1200   Booster K1 is 1150 - same class, squarer number
#   full  1750   a standard adult human
#   mega  2000   a lumberjack: taller AND broader, so it carries a bulk factor too
#
# `bulk` widens stance and shoulders beyond the height scale and thickens the
# structure with it. A lumberjack is not a tall thin person.
VARIANT = {
    'mini': dict(h=1.200, bulk=1.00, leg='X6-60',   ankle='X4-36',   arm='X4-36',
                 head='X4-10', batt_kwh=0.30, compute=0.4, compute_name='Jetson Orin NX'),
    'full':  dict(h=1.750, bulk=1.00, leg='X12',     ankle='X8',      arm='X4-36',
                 head='X4-10', batt_kwh=0.60, compute=0.4, compute_name='Jetson Orin NX'),
    'mega':  dict(h=2.000, bulk=1.15, leg='rana-190', ankle='rana-140', arm='rana-100',
                 head='rana-100', batt_kwh=1.20, compute=1.4, compute_name='Jetson AGX Thor'),
}

# 48 V pouch packs land around 180 Wh/kg at pack level once cells, BMS, case and
# busbars are counted. Superman's 0.864 kWh is the reference point for mega.
PACK_WH_PER_KG = 180.0


def geometry(v):
    """Scale rose full's verified proportions to this variant's height.

    Segment lengths follow height. Lateral spacing follows height AND bulk, which
    is what makes mega broad rather than merely tall.
    """
    s, b = v['h'] / REF_H, v.get('bulk', 1.0)
    return dict(scale=s, bulk=b, thigh=REF_THIGH * s, shank=REF_SHANK * s,
                stance=REF_STANCE * s * b, seg=REF_SHANK * s)


def mass(v):
    """Actuator mass from the catalogue; structure scaled as length^3 from measured."""
    s = v['h'] / REF_H
    act = (8 * ACT[v['leg']]['kg']       # 4 leg actuators x 2 legs
           + 4 * ACT[v['ankle']]['kg']   # 2 ankle actuators x 2 legs
           + 8 * ACT[v['arm']]['kg']     # 4 arm actuators x 2 arms
           + 2 * ACT[v['head']]['kg'])
    # Structure goes as length^3; bulk widens two of those three dimensions.
    struct = REF_STRUCT * s ** 3 * v.get('bulk', 1.0) ** 2
    batt = v['batt_kwh'] * 1000.0 / PACK_WH_PER_KG
    return dict(act=act, struct=struct, batt=batt, compute=v['compute'],
                total=act + struct + batt + v['compute'])


def static(v, theta_k=math.radians(90.0)):
    """Can the knee hold a SINGLE-LEG squat, and how deep?

    Single leg is the case that matters: two-legged standing halves the load, but
    walking spends most of its time on one foot, and every landing lands on one.

    With interior knee angle theta the knee axis sits L*sin(theta/2) horizontally
    off the hip-foot line, so a vertical ground reaction F produces
        tau = F * L * sin(theta/2)
    NOT iterated: a heavier actuator makes the robot heavier, which raises the torque
    it has to hold. The selections below are one pass, not a fixed point, so treat a
    marginal PASS as marginal.

    and the deepest holdable angle inverts it:
        theta_max = 2 * asin(tau_peak / (m*g*L)),  undefined (= full range) if the
    actuator can carry it all the way to the fold.
    """
    g, m = geometry(v), mass(v)['total']
    L, tau = g['seg'], ACT[v['leg']]['tau']
    need_1leg = m * G * L * math.sin(theta_k / 2)
    need_2leg = need_1leg / 2.0
    ratio = tau / (m * G * L)
    theta_max = math.pi if ratio >= 1.0 else 2 * math.asin(ratio)
    return dict(m=m, L=L, tau=tau, need_1leg=need_1leg, need_2leg=need_2leg,
                theta_max=theta_max, ok=need_1leg <= tau)


def jump(v, theta_k=math.radians(90.0), tau_derate=0.70):
    """Standing vertical jump ceiling, energy-limited and power-limited.

    Planar, symmetric, knee-dominated. Thigh and shank are equal (they are, by
    design - rose full runs 1:1 after the proportions pass), so with interior knee
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

    for name in ('mini', 'full', 'mega'):
        v = VARIANT[name]
        g, m = geometry(v), mass(v)
        print('\n--- rose %s --- %.0f mm, scale %.3f from full\'s verified 1130'
              % (name, v['h'] * 1000, g['scale']))
        print('  thigh / shank   %4.0f / %.0f mm     stance %3.0f mm'
              % (g['thigh'] * 1000, g['shank'] * 1000, g['stance'] * 1000))
        print('  actuators       leg %s   ankle %s   arm %s'
              % (v['leg'], v['ankle'], v['arm']))
        est = any(ACT[v[k]].get('est') for k in ('leg', 'ankle', 'arm', 'head'))
        print('  mass            %5.1f kg total%s = %.1f actuators + %.1f structure'
              '  + %.1f battery (%.2f kWh) + %.1f %s'
              % (m['total'], ' (EST)' if est else '      ', m['act'], m['struct'],
                 m['batt'], v['batt_kwh'], m['compute'], v['compute_name']))
        if g['bulk'] != 1.0:
            print('  bulk            x%.2f lateral - broader than a pure scale-up'
                  % g['bulk'])

        st = static(v)
        print('  single-leg 90 squat  needs %.0f N.m at the knee, have %.0f  ->  %s'
              % (st['need_1leg'], st['tau'], 'OK' if st['ok'] else 'SHORT'))
        if st['theta_max'] >= math.pi:
            print('  deepest single-leg squat: full range - torque never binds')
        else:
            print('  deepest single-leg squat the knee holds: %.0f deg of knee bend'
                  % math.degrees(st['theta_max']))

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
    print('jump feasibility - rose mega, against Unitree Superman (2.0 m claimed)')
    print('=' * 74)
    j = jump(VARIANT['mega'])
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
    print('  Consequence for mega: jump height is an ELECTRICAL spec, not a mechanical')
    print('  one. To reach 1 m the lever is bus voltage or current, or storing energy')
    print('  in series elastics so the motors can work slowly and release fast. A')
    print('  bigger rana does nothing.')
    for tgt, (bus, amp) in (('1.0 m', (100.0, 60.0)), ('2.0 m', (150.0, 120.0))):
        print('    %s would need roughly a %.0f V / %.0f A leg drive (%.1f kW/knee)'
              % (tgt, bus, amp, bus * amp / 1000.0))


def clearance(scale, label, lat=1.0):
    """Re-run full's verified packing check on a scaled skeleton. ~30 s."""
    import itertools
    import sys
    sys.path.insert(0, 'full')
    import rose_full_layout as L

    segs = [L.seg(j) for j in L.joints(scale, lat)]
    worst = []
    for (l1, a1, b1, r1, _), (l2, a2, b2, r2, _) in itertools.combinations(segs, 2):
        gap = L.cyl_gap(a1, b1, r1, a2, b2, r2) * 1000
        if gap < 20:
            worst.append((gap, l1, l2))
    print('\n--- packing check: %s (scale %.3f, lateral x%.2f) ---'
          % (label, scale, lat))
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
        for n in ('mini', 'full', 'mega'):
            v = VARIANT[n]
            unmodelled = sorted({v[k] for k in ('leg', 'ankle', 'arm', 'head')
                                 if ACT[v[k]].get('est') or v[k].startswith('rana-1')
                                 and v[k] != 'rana-100'})
            if unmodelled:
                print('\n--- packing check: rose %s --- BLOCKED' % n)
                print('  no modelled envelope for %s. Only these have verified'
                      % ', '.join(unmodelled))
                print('  dimensions: X6-60, X4-36, X4-10 (vendor STEP + V4 manual) and')
                print('  rana-100 rev D (D100 x 44). Inventing the rest would make this')
                print('  check a lie, so it fails closed instead.')
                continue
            clearance(v['h'] / REF_H, 'rose ' + n, v.get('bulk', 1.0))
