"""Pick-and-place scenarios with real-size objects, full and mini arms side by side.

Each arm solves its own IK for the same tasks (objects placed the same distance from its base),
carries the object while its jaws hold it, and every frame gets a static load check: gravity torque
at J2 / J4 / J6 against the motor's stall-continuous rating (holding torque - no dynamics).

Run:  python3 arm/fork/scenarios.py OUT_DIR [workers]     -> render/scenarios.mp4
      python3 arm/fork/scenarios.py --plan                  -> the per-task numbers, no rendering
Frames already in OUT_DIR are skipped, so an interrupted run resumes.
"""
import json, math, os, shutil, subprocess, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, '..'))

FPS = 12
RENDER = os.path.expanduser('~/Developer/vcad/target/release/vcad-render')
G = 9.81
ARMS = {'full': ({}, (0.0, -0.55, 0.0)), 'mini': ({'ROSE_ARM': 'mini'}, (0.0, 0.40, 0.0))}

# real objects: (label, kind, d or (lx, ly), h, kg, colour)
OBJ = {
    'can': ('soda can', 'cyl', 0.066, 0.122, 0.37, (0.80, 0.10, 0.12)),
    'mug': ('mug of coffee', 'cyl', 0.082, 0.095, 0.40, (0.93, 0.92, 0.88)),
    'bottle': ('1 L water bottle', 'cyl', 0.080, 0.230, 1.05, (0.35, 0.60, 0.90)),
    'book': ('hardback book', 'box', (0.160, 0.035), 0.240, 0.70, (0.20, 0.45, 0.30)),
    'far': ('soda can, far away', 'cyl', 0.066, 0.122, 0.37, (0.80, 0.10, 0.12)),
    'big': ('1.5 L bottle', 'cyl', 0.090, 0.300, 1.60, (0.35, 0.60, 0.90)),
}
# tasks: object, pick (x, y) from the arm's base, place (x, y)
TASKS = [
    ('can', (0.30, -0.15), (0.26, 0.20)),
    ('mug', (0.22, 0.18), (0.34, 0.00)),
    ('bottle', (0.30, 0.00), (0.22, -0.22)),
    ('book', (0.40, 0.10), (0.30, -0.18)),
    ('far', (0.58, -0.05), (0.40, 0.25)),
    ('big', (0.32, 0.05), (0.26, -0.22)),
]
HOVER = 0.08          # approach height over the grasp
SEG = [('approach', 1.4), ('descend', 0.6), ('grip', 0.3), ('lift', 0.6), ('carry', 1.4), ('lower', 0.6),
       ('release', 0.3), ('retreat', 0.6)]
LIMITS = dict(J2_sho_pitch=(-180, 40), J4_elbow=(-115, 115), J6_wrist_pitch=(-120, 120))
HOME = dict(J1_yaw=0.0, J2_sho_pitch=-70.0, J4_elbow=110.0, J5_fore_roll=0.0, J6_wrist_pitch=-40.0, J7_tool_roll=0.0)


# ---------------------------------------------------------------- per-arm worker (one motor config per process)
def arm_module():
    import rose_fork as F
    import fork_checks as C
    return F, C


def grasp_local(F):
    s = F.stations()
    return (s['xt'] + F.T_PL + F.EDU['d'] + 0.6 * F.JAW, 0.0, F.Z_SH)


def link_masses(F, C):
    """{link: [(kg, centroid at the zero pose)]} - the same accounting as fork_checks."""
    import rose_arm_geom as Gm
    out = {}
    for p in F.build():
        link, mat, o, u, v, ol, vec = p[:7]
        if mat == 'actuator':
            m = C.ACT_KG.get(round(abs(ol[0][1][0]), 4), 0.0)
            c = F.add(o, F.mul(vec, 0.5))
        else:
            m = F.prim_mass(p) or 0.0
            pts = Gm.prim_points(p[:7], step=0.02)
            c = tuple(sum(q[i] for q in pts) / len(pts) for i in range(3))
        out.setdefault(link, []).append((max(m, 0.0), c))
    return out


def ik(F, target, tool_deg, seed):
    """Joint angles (deg) that put the grasp point at target (arm frame, m) with the tool pitched
    tool_deg below horizontal. Yaw is analytic; J2 / J4 / J6 by damped Newton on the real FK."""
    x, y, z = target
    yaw = math.degrees(math.atan2(y, x))
    gl = grasp_local(F)
    q = dict(seed, J1_yaw=yaw, J5_fore_roll=0.0, J7_tool_roll=0.0)
    names = ('J2_sho_pitch', 'J4_elbow', 'J6_wrist_pitch')

    def resid(qd):
        T = F.fk({k: math.radians(v) for k, v in qd.items()})
        p = F.xf(T, 'tool', gl)
        d = F.xd(T, 'tool', (1.0, 0.0, 0.0))
        r = math.hypot(d[0], d[1])
        ang = math.degrees(math.atan2(-d[2], r))
        return [p[0] - x, p[1] - y, p[2] - z, (ang - tool_deg) * 0.002]

    for _ in range(60):
        r0 = resid(q)
        if max(abs(v) for v in r0) < 1e-4:
            break
        J = []
        for n in names:
            q2 = dict(q); q2[n] += 0.05
            J.append([(a - b) / 0.05 for a, b in zip(resid(q2), r0)])
        # normal equations, 3x3, damped
        A = [[sum(J[i][k] * J[j][k] for k in range(4)) + (1e-6 if i == j else 0) for j in range(3)] for i in range(3)]
        bb = [-sum(J[i][k] * r0[k] for k in range(4)) for i in range(3)]
        dq = solve3(A, bb)
        step = max(abs(v) for v in dq)
        sc = min(1.0, 15.0 / step) if step > 0 else 1.0
        for n, v in zip(names, dq):
            q[n] += v * sc
    r = resid(q)
    ok = max(abs(v) for v in r[:3]) < 0.002 and abs(r[3]) < 0.002 * 2 and \
        all(LIMITS[n][0] <= q[n] <= LIMITS[n][1] for n in names)
    return q, ok


def solve3(A, b):
    import copy
    M = [row[:] + [bv] for row, bv in zip(copy.deepcopy(A), b)]
    for c in range(3):
        p = max(range(c, 3), key=lambda r: abs(M[r][c]))
        M[c], M[p] = M[p], M[c]
        if abs(M[c][c]) < 1e-12:
            return [0.0, 0.0, 0.0]
        for r in range(3):
            if r != c:
                f = M[r][c] / M[c][c]
                M[r] = [a - f * bv for a, bv in zip(M[r], M[c])]
    return [M[i][3] / M[i][i] for i in range(3)]


def best_ik(F, target, seed):
    """Prefer a top-down grasp; tilt toward horizontal until the arm can reach. None if nothing works."""
    for tool in (90, 75, 60, 45, 30, 15, 0):
        for s in (seed, HOME, dict(HOME, J2_sho_pitch=-30, J4_elbow=60, J6_wrist_pitch=60)):
            q, ok = ik(F, target, tool, s)
            if ok:
                return q, tool
    return None, None


def plan_arm(name):
    """Keyposes for every task, the frame schedule, and per-frame loads. Returns JSON-able dict."""
    F, C = arm_module()
    gl = grasp_local(F)
    LM = link_masses(F, C)
    rating = {j: F.RATING[id(m)][1] for j, m in (('J2', F.J2M), ('J4', F.J4M), ('J6', F.J6M))}
    joints = {'J2': ('upper', 'J2_sho_pitch'), 'J4': ('fore', 'J4_elbow'), 'J6': ('wrist', 'J6_wrist_pitch')}
    outboard = {'J2': ('upper', 'fore', 'yoke', 'wrist', 'tool'), 'J4': ('fore', 'yoke', 'wrist', 'tool'),
                'J6': ('wrist', 'tool')}

    def loads(qd, held):
        T = F.fk({k: math.radians(v) for k, v in qd.items()})
        out = {}
        for j, (link, _) in joints.items():
            o = F.xf(T, link, F.ORIGIN[link])
            ax = F.xd(T, link, (0.0, 1.0, 0.0))
            tau = 0.0
            pts = [(m, F.xf(T, l, c)) for l in outboard[j] for m, c in LM.get(l, [])]
            if held:
                pts.append(held)
            for m, c in pts:
                rv = F.sub(c, o)
                fz = (0.0, 0.0, -m * G)
                t = F.cross(rv, fz)
                tau += F.dot(t, ax)
            out[j] = abs(tau) / rating[j]
        return out

    frames, notes = [], []
    seed = dict(HOME)
    for oi, (obj, pick, place) in enumerate(TASKS):
        label, kind, dim, h, kg, col = OBJ[obj]
        zg = h - 0.035 if h > 0.1 else h * 0.6        # grip near the top of tall things
        keys = {}
        fail = None
        for nm, (xy, dz) in (('pick_hover', (pick, HOVER)), ('pick', (pick, 0.0)), ('place_hover', (place, HOVER)),
                             ('place', (place, 0.0))):
            q, tool = best_ik(F, (xy[0], xy[1], zg + dz), seed)
            if q is None:
                fail = '%s out of reach' % ('pick' if 'pick' in nm else 'place')
                break
            keys[nm] = (q, tool)
            seed = q
        lift_ok = True
        if fail is None:
            # holding check at the worst carried pose
            for nm in ('pick_hover', 'place_hover'):
                q = keys[nm][0]
                T = F.fk({k: math.radians(v) for k, v in q.items()})
                gp = F.xf(T, 'tool', gl)
                ld = loads(q, (kg, F.sub(gp, (0.0, 0.0, zg - h / 2))))
                if max(ld.values()) > 1.0:
                    lift_ok = False
                    fail = 'too heavy: %s at %.0f%% of stall-continuous' % (max(ld, key=ld.get), 100 * max(ld.values()))
        notes.append(dict(obj=obj, label=label, kg=kg, fail=fail,
                          tool=None if not keys else keys.get('pick', (None, None))[1]))
        # schedule: if the arm can't do it, it reaches toward the pick and stops (out of reach) or
        # grips and fails to lift (too heavy) - the object stays put
        if 'pick_hover' in keys:
            hv, pk = keys['pick_hover'][0], keys['pick'][0]
        else:
            # point at it: yaw toward the pick, arm stretched as far as it goes
            yaw = math.degrees(math.atan2(pick[1], pick[0]))
            hv = pk = dict(HOME, J1_yaw=yaw, J2_sho_pitch=-15.0, J4_elbow=20.0, J6_wrist_pitch=40.0)
        ph = keys.get('place_hover', (hv,))[0]
        pl = keys.get('place', (pk,))[0]
        can_carry = fail is None
        seq = [('approach', HOME, hv), ('descend', hv, pk), ('grip', pk, pk), ('lift', pk, hv if can_carry else pk),
               ('carry', hv, ph) if can_carry else ('carry', pk, pk),
               ('lower', ph, pl) if can_carry else ('lower', pk, hv),
               ('release', pl, pl) if can_carry else ('release', hv, hv),
               ('retreat', pl, HOME) if can_carry else ('retreat', hv, HOME)]
        for (seg, dur), (_, a, b) in zip(SEG, seq):
            n = int(round(dur * FPS))
            for i in range(n):
                s = (i + 1) / n
                s = s * s * (3 - 2 * s)
                q = {k: a[k] + (b[k] - a[k]) * s for k in a}
                held = can_carry and seg in ('lift', 'carry', 'lower')
                T = F.fk({k: math.radians(v) for k, v in q.items()})
                gp = F.xf(T, 'tool', gl)
                if held:
                    oc = F.sub(gp, (0.0, 0.0, zg - h / 2))
                elif can_carry and seg in ('release', 'retreat'):
                    oc = (place[0], place[1], h / 2)
                else:
                    oc = (pick[0], pick[1], h / 2)
                ld = loads(q, (kg, oc) if held or (not can_carry and seg in ('grip', 'lift', 'carry') and fail.startswith('too')) else None)
                frames.append(dict(task=oi, seg=seg, q=q, obj=oc, held=held, loads=ld))
    return dict(frames=frames, notes=notes)


def obj_loon(F, obj, c, prefix):
    label, kind, dim, h, kg, col = OBJ[obj]
    x, y, z = c
    if kind == 'cyl':
        r = dim / 2
        ol = [('arc', (r, 0.0), (-r, 0.0), (0.0, 0.0), True), ('arc', (-r, 0.0), (r, 0.0), (0.0, 0.0), True)]
    else:
        a, b = dim[0] / 2, dim[1] / 2
        ol = [('line', (-a, -b), (a, -b)), ('line', (a, -b), (a, b)), ('line', (a, b), (-a, b)), ('line', (-a, b), (-a, -b))]
    p = ('obj', 'obj', (x, y, z - h / 2), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), ol, (0.0, 0.0, h))
    from rose_arm_geom import prim_loon
    return '[material "%s_obj" %.2f %.2f %.2f 0.0 0.5]\n[let %so %s]\n[root %so "%s_obj"]\n' % (
        prefix, col[0], col[1], col[2], prefix, prim_loon(p), prefix, prefix)


def emit(name, out):
    """Worker for one arm: plan, then write every frame's loon fragment (arm + its object)."""
    F, C = arm_module()
    at = ARMS[name][1]
    P = plan_arm(name)
    json.dump(dict(notes=P['notes'], frames=[dict(task=f['task'], seg=f['seg'], loads=f['loads'], held=f['held'])
                                             for f in P['frames']]), open(os.path.join(out, name + '.json'), 'w'))
    for i, f in enumerate(P['frames']):
        fn = os.path.join(out, '%s_%04d.frag' % (name, i))
        if os.path.exists(fn):
            continue
        src = F.world_loon({k: math.radians(v) for k, v in f['q'].items()}, at=at, prefix=name[0], header=False)
        obj = TASKS[f['task']][0]
        src += obj_loon(F, obj, F.add(tuple(f['obj']), at), name[0])
        open(fn, 'w').write(src)


# ---------------------------------------------------------------- scene, render, overlay
def scene(out, i):
    import rose_fork as F
    L = [F.MATERIALS, '[material "table" 0.55 0.42 0.30 0.0 0.7]',
         '[let tab [translate -180 -950 -25 [cube 820 1600 25]]]', '[root tab "table"]',
         '[let a0 [translate -180 -950 -25 [sphere 0.3]]]', '[let a1 [translate 640 650 620 [sphere 0.3]]]',
         '[root a0 "base"]', '[root a1 "base"]']
    for name in ARMS:
        L.append(open(os.path.join(out, '%s_%04d.frag' % (name, i))).read())
    return '\n'.join(L) + '\n'


def render(out, w, k, n):
    for i in range(w, n, k):
        png = os.path.join(out, 'r%04d.png' % i)
        if os.path.exists(png):
            continue
        lf = os.path.join(out, 'w%d.loon' % w)
        open(lf, 'w').write(scene(out, i))
        try:
            subprocess.run([RENDER, lf, '--azimuth=-28', '--elevation=20', '--size', '1280x720', '--fill', '0.98', '-o', png],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=600)
        except subprocess.TimeoutExpired:
            pass


def overlay(out, n):
    from PIL import Image, ImageDraw, ImageFont
    data = {a: json.load(open(os.path.join(out, a + '.json'))) for a in ARMS}
    fb = ImageFont.truetype('/System/Library/Fonts/Helvetica.ttc', 22)
    fs = ImageFont.truetype('/System/Library/Fonts/Helvetica.ttc', 17)
    spec = {'full': 'full  RS04 / RS04 / RS04 / RS02 / RS02 / RS00',
            'mini': 'mini  RS02 / RS04 / RS02 / RS00 / RS00'}
    last = None
    for i in range(n):
        src = os.path.join(out, 'r%04d.png' % i)
        if not os.path.exists(src):
            if last is None:
                continue
            src = last
        last = src
        im = Image.open(src).convert('RGB')
        d = ImageDraw.Draw(im)
        task = data['full']['frames'][i]['task']
        obj = TASKS[task][0]
        label, _, _, _, kg, _ = OBJ[obj]
        d.text((24, 18), 'task %d/%d   %s, %.2f kg' % (task + 1, len(TASKS), label, kg), fill=(30, 30, 30), font=fb)
        for col, a in zip((24, 660), ('full', 'mini')):
            fr = data[a]['frames'][i]
            note = data[a]['notes'][task]
            y = 56
            d.text((col, y), spec[a], fill=(30, 30, 30), font=fs); y += 24
            for j in ('J2', 'J4', 'J6'):
                v = fr['loads'][j]
                c = (40, 150, 70) if v < 0.8 else (220, 150, 20) if v <= 1.0 else (210, 40, 40)
                d.text((col, y), j, fill=(60, 60, 60), font=fs)
                d.rectangle((col + 34, y + 4, col + 34 + 200, y + 16), outline=(180, 180, 180))
                d.rectangle((col + 34, y + 4, col + 34 + int(200 * min(v, 1.5) / 1.5), y + 16), fill=c)
                d.line((col + 34 + int(200 / 1.5), y + 1, col + 34 + int(200 / 1.5), y + 19), fill=(90, 90, 90))
                d.text((col + 244, y), '%3.0f%%' % (100 * v), fill=(60, 60, 60), font=fs)
                y += 22
            msg = 'OK' if note['fail'] is None else note['fail'].upper()
            d.text((col, y + 2), msg, fill=(40, 150, 70) if note['fail'] is None else (210, 40, 40), font=fb)
        d.text((24, 690), 'static holding load vs stall-continuous (tick = 100%). real object sizes and masses.',
               fill=(120, 120, 120), font=fs)
        im.save(os.path.join(out, 'o%04d.png' % i))


def plan_print():
    for name, (env, _) in ARMS.items():
        r = subprocess.run([sys.executable, __file__, '--emit-plan', name], env=dict(os.environ, **env),
                           capture_output=True, text=True)
        print(name, r.stdout.strip() or r.stderr[-800:])


if __name__ == '__main__':
    if sys.argv[1] == '--emit-plan':
        P = plan_arm(sys.argv[2])
        for nt in P['notes']:
            pk = [f['loads'] for f in P['frames'] if TASKS.index(next(t for t in TASKS if t[0] == nt['obj'])) == f['task']]
            worst = {j: max(l[j] for l in pk) for j in ('J2', 'J4', 'J6')}
            print('  %-18s %.2f kg  %-40s grasp %s deg  worst %s' % (
                nt['label'], nt['kg'], nt['fail'] or 'ok', nt['tool'], ' '.join('%s %3.0f%%' % (j, 100 * v) for j, v in worst.items())))
        sys.exit()
    if sys.argv[1] == '--plan':
        plan_print(); sys.exit()
    if sys.argv[1] == '--emit':
        emit(sys.argv[2], sys.argv[3]); sys.exit()
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    if len(sys.argv) > 4 and sys.argv[2] == '--render':
        render(out, int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5])); sys.exit()
    k = int(sys.argv[2]) if len(sys.argv) > 2 else 8
    for name, (env, _) in ARMS.items():
        subprocess.run([sys.executable, __file__, '--emit', name, out], env=dict(os.environ, **env), check=True)
    n = min(len(json.load(open(os.path.join(out, a + '.json')))['frames']) for a in ARMS)
    # Frame 0 first, alone: it fills vcad's shape cache (parts are emitted in their own frame, so
    # every later pose is a cache hit) and the workers then never repeat the booleans.
    render(out, 0, n, 1)
    ps = [subprocess.Popen([sys.executable, __file__, out, '--render', str(w), str(k), str(n)]) for w in range(k)]
    for p in ps:
        p.wait()
    overlay(out, n)
    mp4 = os.path.join(HERE, 'render', 'scenarios.mp4')
    subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-framerate', str(FPS), '-i', os.path.join(out, 'o%04d.png'),
                    '-vf', 'scale=1280:-2,format=yuv420p', '-c:v', 'libx264', '-crf', '20', mp4], check=True)
    print(mp4)
