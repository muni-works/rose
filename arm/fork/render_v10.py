"""lymb v10 photoreal renders: (a) desk, skinned; (b) the arms of the full humanoid, skinned; (c) a | b;
(d) v9 next to v10 on the desk.

    uv run -q --with numpy --with manifold3d --with trimesh --with scipy --with pillow \\
        python arm/fork/render_v10.py WORKDIR [a b c d] [--spp N]

WORKDIR holds the scenes (.loon), the skin STLs and hum/ (python arm/fork/humanoid_export.py WORKDIR/hum
first, for b). Finished PNGs go to arm/fork/render/. Each arm config is emitted by a child process with
its own ROSE_ARM env (rose_fork reads it at import).
"""
import json
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
RENDER = os.environ.get('VCAD_RENDER', os.path.expanduser('~/Developer/vcad-fast/target/release/vcad-render'))
OUT = os.path.join(HERE, 'render')

DESK_V10 = dict(J1_yaw=-25, J2_sho_pitch=-50, J3_upper_roll=20, J4_elbow=100, J6_wrist_pitch=-45, J7_tool_roll=0)
DESK_V9 = dict(J1_yaw=-25, J2_sho_pitch=-50, J4_elbow=100, J5_fore_roll=20, J6_wrist_pitch=-45, J7_tool_roll=0)
DESK_MATS = '[material "desk" 0.80 0.79 0.76 0.0 0.62]\n'


# ---------------------------------------------------------------- child: one arm's fragment
def child(spec):
    """spec: dict(q, at, R0, prefix, outdir, skin, skip, tag). Prints the fragment."""
    sys.path.insert(0, HERE)
    import skin_v10 as S
    q = {k: math.radians(v) for k, v in spec['q'].items()}
    skip = set(tuple(s) if isinstance(s, list) else s for s in spec.get('skip', []))
    src = S.frag(q, at=tuple(spec['at']), R0=tuple(tuple(r) for r in spec['R0']), prefix=spec['prefix'],
                 outdir=spec['outdir'], skin=spec['skin'], skip=skip, stl_tag=spec.get('tag'))
    sys.stdout.write(src)


def arm(env, **spec):
    spec.setdefault('R0', [[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    spec.setdefault('skip', [])
    r = subprocess.run([sys.executable, __file__, '--child', json.dumps(spec)], env=dict(os.environ, **env),
                       capture_output=True, text=True)
    if r.returncode:
        raise SystemExit(r.stderr[-3000:])
    return r.stdout


def photo(src):
    sys.path.insert(0, HERE)
    import scenarios as SC
    import rose_fork as F
    for a, b in SC.PHOTO_RENAME.items():
        src = src.replace(a, b)
    return SC.PHOTO_MATERIALS + src


def skin_mats():
    sys.path.insert(0, HERE)
    import skin_v10 as S
    return S.SKIN_MATS


def render(loon, png, az, el, spp, size='1600x1200', fill=0.9, extra=()):
    cmd = [RENDER, loon, '--photoreal', '--spp', str(spp), '--backdrop', 'cyc', '--tonemap', 'neutral',
           '--vignette', '0.15', '--azimuth=%s' % az, '--elevation=%s' % el, '--size', size, '--fill', str(fill),
           '-o', png] + list(extra)
    print(' '.join(cmd), flush=True)
    subprocess.run(cmd, check=True)


V10 = {'ROSE_ARM': 'v10'}
V9 = {'ROSE_ARM': 'full'}


def scene_desk(work, which):
    """(a) v10 skinned on the desk; (d) v9 and v10, bare, side by side."""
    L = []
    if which == 'a':
        # pedestal bottom on the table (z = 0): the arm frame sits PED_H up
        L.append(arm(V10, q=DESK_V10, at=[0, 0, 0.068], prefix='a', outdir=work, skin=True, tag='skin-desk'))
        desk = '[let dk [translate -260 -420 -30 [cube 820 760 30]]]\n[root dk "desk"]\n'
    else:
        L.append(arm(V9, q=DESK_V9, at=[0, 0.30, 0.0], prefix='n', outdir=work, skin=False))
        L.append(arm(V10, q=DESK_V10, at=[0, -0.30, 0.068], prefix='t', outdir=work, skin=False))
        desk = '[let dk [translate -420 -760 -30 [cube 1100 1520 30]]]\n[root dk "desk"]\n'
    import rose_fork as F
    src = photo(F.MATERIALS + '\n' + '\n'.join(L)) + skin_mats() + DESK_MATS + desk
    fn = os.path.join(work, 'desk-%s.loon' % which)
    open(fn, 'w').write(src)
    return fn


def scene_closeup(work):
    """The elbow alone (upper arm, elbow puck, forearm), skinned, cable inside and not drawn."""
    skip = ['base', 'turret', 'yoke', 'wrist', 'tool', 'cable']
    q = dict(DESK_V10, J1_yaw=0, J2_sho_pitch=-20, J3_upper_roll=0, J4_elbow=70)
    L = [arm(V10, q=q, at=[0, 0, 0.045], prefix='e', outdir=work, skin=True, tag='skin-desk', skip=skip)]
    import rose_fork as F
    fn = os.path.join(work, 'closeup.loon')
    open(fn, 'w').write(photo(F.MATERIALS + '\n' + '\n'.join(L)) + skin_mats())
    return fn


def body_R0(s):
    """arm frame -> body: J1 axis (arm z) lateral out, the upper arm (arm x) hanging down,
    J2 axis (arm y) fore-aft - the humanoid's shoulder pitch / roll."""
    ex, ez = (0.0, 0.0, -1.0), (0.0, float(s), 0.0)
    ey = (ez[1] * ex[2] - ez[2] * ex[1], ez[2] * ex[0] - ez[0] * ex[2], ez[0] * ex[1] - ez[1] * ex[0])
    return [[ex[i], ey[i], ez[i]] for i in range(3)]


# humanoid poses, per side (s = +1 left, -1 right); J1 = shoulder pitch (- = forward), J2 = roll
BODY_POSES = {
    'rest': {1: dict(J1_yaw=-8, J2_sho_pitch=-4, J3_upper_roll=90, J4_elbow=-25, J6_wrist_pitch=-10),
             -1: dict(J1_yaw=8, J2_sho_pitch=-4, J3_upper_roll=-90, J4_elbow=-25, J6_wrist_pitch=-10)},
    'reach': {1: dict(J1_yaw=-75, J2_sho_pitch=-4, J3_upper_roll=90, J4_elbow=-15, J6_wrist_pitch=-10),
              -1: dict(J1_yaw=75, J2_sho_pitch=-4, J3_upper_roll=-90, J4_elbow=-15, J6_wrist_pitch=-10)},
    'carry': {1: dict(J1_yaw=-20, J2_sho_pitch=-6, J3_upper_roll=90, J4_elbow=-95, J6_wrist_pitch=0),
              -1: dict(J1_yaw=20, J2_sho_pitch=-6, J3_upper_roll=-90, J4_elbow=-95, J6_wrist_pitch=0)},
    'j1+90': {1: dict(J1_yaw=-90, J2_sho_pitch=-4, J3_upper_roll=90, J4_elbow=0),
              -1: dict(J1_yaw=-90, J2_sho_pitch=-4, J3_upper_roll=-90, J4_elbow=0)},
    'j1-90': {1: dict(J1_yaw=90, J2_sho_pitch=-4, J3_upper_roll=90, J4_elbow=0),
              -1: dict(J1_yaw=90, J2_sho_pitch=-4, J3_upper_roll=-90, J4_elbow=0)},
}
FLANK_Y = 0.1866      # quad00 full: the chest cover's flank at shoulder height (|y|, m) - J1's face sits flush on it
SIDE_PLATE_Y = 0.087  # torso side plate outer face


def body_ped_h():
    """Chest adapter height: J1's output face lands on the flank."""
    import rose_fork as F
    return FLANK_Y - SIDE_PLATE_Y - F.T_FOOT - F.J1M['L']


DESK_POSES = {
    'ready': DESK_V10,
    'pick': dict(J1_yaw=20, J2_sho_pitch=-50, J3_upper_roll=0, J4_elbow=100, J6_wrist_pitch=40, J7_tool_roll=0),   # jaws 2 mm over the desk, pose_ok
    'high': dict(J1_yaw=-40, J2_sho_pitch=-105, J3_upper_roll=0, J4_elbow=35, J6_wrist_pitch=40, J7_tool_roll=0),
}


def scene_desk2(work, pose):
    L = [arm(V10, q=DESK_POSES[pose], at=[0, 0, 0.045], prefix='a', outdir=work, skin=True, tag='skin-desk')]
    import rose_fork as F
    desk = '[let dk [translate -330 -420 -30 [cube 900 840 30]]]\n[root dk "desk"]\n'
    fn = os.path.join(work, 'desk-%s.loon' % pose)
    open(fn, 'w').write(photo(F.MATERIALS + '\n' + '\n'.join(L)) + skin_mats() + DESK_MATS + desk)
    return fn


CORTEX = [('cortex-tub', 'cx_shell'), ('cortex-ear-grilles', 'accent'), ('cortex-exhaust-grille', 'cx_shell'),
          ('cortex-glass', 'cx_glass'), ('cortex-floor', 'cx_floor')]
CX_MATS = ('[material "cx_shell" 0.016 0.012 0.009 0.0 0.45]\n[material "cx_glass" 0.01 0.01 0.012 0.2 0.05]\n'
           '[material "cx_floor" 0.043 0.035 0.027 0.8 0.4]\n')


def scene_body2(work, pose, only=None):
    """Two v10.2 arms on the quad00 full humanoid, the cortex (Make colorway) on the neck dock.
    J1 sinks into the torso's top corner through the chest adapter until its face is flush with the
    flank; J2 then sits at |y| ~ 260 (the humanoid's own shoulder roll is at 255)."""
    H = json.load(open(os.path.join(work, 'hum', 'hum.json')))
    hum, mats = [], {}
    for k, v in H['finishes'].items():
        v = list(v)
        if v[0] == 'satin_black':
            v[1:] = [0.016, 0.012, 0.009, 0.0, 0.45]      # the body tone: warm black #1e1813
        mats[v[0]] = v
        hum.append('[let h_%s [import-mesh-scaled 0.001 0.001 0.001 "hum/hum_%s.stl"]]\n[root h_%s "%s"]' % (k, k, k, v[0]))
    hm = '\n'.join('[material "%s" %s %s %s %s %s]' % tuple(v) for v in mats.values()) + '\n'
    cx = H['cortex']
    cxs = ['[use cortex [%s]]' % ' '.join(n for n, _ in CORTEX)]
    for n, m in CORTEX:
        k = H.get('cortex_k', 1.0)       # cortex.loon is authored at the mini's head size (quad00 vcad_export: hh / 120.6)
        body = n if abs(k - 1.0) < 1e-3 else '[scale %.4f %.4f %.4f %s]' % (k, k, k, n)
        cxs.append('[let hc_%s [translate %.3f %.3f %.3f %s]]\n[root hc_%s "%s"]' % (n.replace('-', '_'), cx[0], cx[1], cx[2], body, n.replace('-', '_'), m))
    import rose_fork as F
    ph = body_ped_h()
    L = []
    for s, t in ((1, 'l'), (-1, 'r')):
        if only and t != only:
            continue
        sr = [c / 1e3 for c in H['sp']]   # v10.3: J1 on the humanoid's shoulder-pitch axis (x, z of quad00's sp; rose's torso v103)
        at = [sr[0], s * (SIDE_PLATE_Y + ph), sr[2]]
        L.append(arm(dict(V10, ROSE_ARM_PED_H='%.5f' % ph, ROSE_ARM_PEDESTAL='0'), q=BODY_POSES[pose][s], at=at, R0=body_R0(s),
                     prefix='b' + t, outdir=work, skin=True, tag='skin-body', skip=[['base', 'base']]))
    src = photo(F.MATERIALS + '\n' + '\n'.join(L)) + skin_mats() + hm + CX_MATS + '\n'.join(cxs) + '\n' + '\n'.join(hum) + '\n'
    fn = os.path.join(work, 'body-%s.loon' % pose)
    open(fn, 'w').write(src)
    return fn


def label(png, text, sub=None):
    from PIL import Image, ImageDraw, ImageFont
    im = Image.open(png).convert('RGB')
    d = ImageDraw.Draw(im)
    fb = ImageFont.truetype('/System/Library/Fonts/Helvetica.ttc', max(18, im.width // 48))
    fs = ImageFont.truetype('/System/Library/Fonts/Helvetica.ttc', max(14, im.width // 70))
    d.text((28, 22), text, fill=(40, 40, 40), font=fb)
    if sub:
        d.text((28, 22 + im.width // 40), sub, fill=(110, 110, 110), font=fs)
    im.save(png)


def side_by_side(a, b, out):
    from PIL import Image
    A, B = Image.open(a).convert('RGB'), Image.open(b).convert('RGB')
    h = max(A.height, B.height)
    A = A.resize((int(A.width * h / A.height), h))
    B = B.resize((int(B.width * h / B.height), h))
    im = Image.new('RGB', (A.width + B.width, h), (255, 255, 255))
    im.paste(A, (0, 0))
    im.paste(B, (A.width, 0))
    im.save(out)


VIEWS = {'34': (-35, 20), 'front': (0, 8), 'side': (-90, 8)}
BVIEWS = {'34': (-30, 10), 'front': (0, 4), 'side': (-90, 4)}


def main():
    work = os.path.abspath(sys.argv[1])
    os.makedirs(work, exist_ok=True)
    spp = int(sys.argv[sys.argv.index('--spp') + 1]) if '--spp' in sys.argv else 256
    only = [a for a in sys.argv[2:] if not a.startswith('--') and not a.isdigit()]
    want = lambda k: not only or k in only
    os.makedirs(OUT, exist_ok=True)
    V = os.environ.get('LYMB_VER', 'v10.3')
    P = lambda n: os.path.join(OUT, 'lymb-%s-%s.png' % (V, n))
    sub = 'warm black #1e1813, graphite pucks, gunmetal caps, Make green at the wrist'
    for v, (az, el) in VIEWS.items():
        if want('desk-' + v):
            render(scene_desk2(work, 'ready'), P('desk-' + v), az, el, spp, size='1400x1200')
            label(P('desk-' + v), 'lymb %s ' % V + 'on the desk (%s)' % v, sub)
    for pose in ('pick', 'high'):
        if want('desk-' + pose):
            render(scene_desk2(work, pose), P('desk-' + pose), -35, 20, spp, size='1400x1200')
            label(P('desk-' + pose), 'lymb %s ' % V + 'on the desk: %s' % pose, sub)
    for v, (az, el) in BVIEWS.items():
        if want('humanoid-' + v):
            render(scene_body2(work, 'rest'), P('humanoid-' + v), az, el, spp, size='1100x1500', fill=0.94)
            label(P('humanoid-' + v), 'lymb %s ' % V + 'as the arms of the 1703 mm humanoid (%s)' % v, 'same part as the desk arm; J1 flush in the torso corner, cortex head (Make)')
    for pose in ('reach', 'carry'):
        if want('humanoid-' + pose):
            render(scene_body2(work, pose), P('humanoid-' + pose), -30, 10, spp, size='1100x1500', fill=0.94)
            label(P('humanoid-' + pose), 'lymb %s ' % V + 'on the humanoid: %s' % pose)
    for pose, az in (('j1+90', -60), ('j1-90', -60)):
        if want('shoulder-' + pose):
            pp = P('shoulder-%s' % pose.replace('+', 'p').replace('-', 'm'))
            render(scene_body2(work, pose, only='l'), pp, az, 12, spp, size='2400x2400', fill=0.94)
            from PIL import Image
            im = Image.open(pp)
            W_, H_ = im.size
            im.crop((int(0.22 * W_), int(0.08 * H_), int(0.78 * W_), int(0.46 * H_))).save(pp)   # the left shoulder
            label(P('shoulder-%s' % pose.replace('+', 'p').replace('-', 'm')), 'lymb %s ' % V + 'shoulder, J1 = %s deg' % pose[2:])
    if want('elbow'):
        render(scene_closeup(work), P('elbow-closeup'), -50, 25, spp, size='1400x1100', fill=0.92)
        label(P('elbow-closeup'), 'lymb %s ' % V + 'elbow: flush graphite puck, gunmetal hub cap, 1 mm coaxial parting gap; J3 ring behind')
    if want('sbs'):
        side_by_side(P('desk-34'), P('humanoid-34'), P('desk-and-humanoid'))


if __name__ == '__main__':
    if sys.argv[1] == '--child':
        child(json.loads(sys.argv[2]))
    else:
        main()
