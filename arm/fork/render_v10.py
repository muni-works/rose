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
    keep = ('upper', 'upperd', 'fore')
    skip = [l for l in ('base', 'turret', 'yoke', 'wrist', 'tool')]
    q = dict(DESK_V10, J1_yaw=0, J2_sho_pitch=-20, J3_upper_roll=0, J4_elbow=70)
    L = [arm(V10, q=q, at=[0, 0, 0.068], prefix='e', outdir=work, skin=True, tag='skin-desk', skip=skip)]
    import rose_fork as F
    src = photo(F.MATERIALS + '\n' + '\n'.join(L)) + skin_mats()
    fn = os.path.join(work, 'closeup.loon')
    open(fn, 'w').write(src)
    return fn


def body_R0(s):
    """arm frame -> body: J1 axis (arm z) lateral out, the upper arm (arm x) hanging down,
    J2 axis (arm y) fore-aft - the humanoid's shoulder pitch / roll."""
    ex, ez = (0.0, 0.0, -1.0), (0.0, float(s), 0.0)
    ey = (ez[1] * ex[2] - ez[2] * ex[1], ez[2] * ex[0] - ez[0] * ex[2], ez[0] * ex[1] - ez[1] * ex[0])
    return [[ex[i], ey[i], ez[i]] for i in range(3)]


BODY_Q = {1: dict(J1_yaw=-12, J2_sho_pitch=-6, J3_upper_roll=90, J4_elbow=-40, J6_wrist_pitch=-15, J7_tool_roll=0),
          -1: dict(J1_yaw=10, J2_sho_pitch=6, J3_upper_roll=-90, J4_elbow=-40, J6_wrist_pitch=-15, J7_tool_roll=0)}


def scene_body(work):
    """(b) two v10 arms on the full humanoid. The J1 back face bolts to the torso side plate through a
    chest adapter (the desk pedestal's tube, 36 mm), so the J2 axis lands on the humanoid's shoulder-roll
    axis (|y| = 255)."""
    H = json.load(open(os.path.join(work, 'hum', 'hum.json')))
    hum = []
    mats = {}
    for k, v in H['finishes'].items():
        mats[v[0]] = v
        hum.append('[let h_%s [import-mesh-scaled 0.001 0.001 0.001 "hum/hum_%s.stl"]]\n[root h_%s "%s"]' % (k, k, k, v[0]))
    hm = '\n'.join('[material "%s" %s %s %s %s %s]' % tuple(v) for v in mats.values()) + '\n'
    import importlib
    L = []
    z_sh = 0.132
    for s, t in ((1, 'l'), (-1, 'r')):
        sr = [c / 1e3 for c in H['sr'][t]]
        R0 = body_R0(s)
        at = [sr[i] - R0[i][2] * z_sh for i in range(3)]     # arm (0, 0, Z_SH) on the shoulder-roll axis
        L.append(arm(dict(V10, ROSE_ARM_PED_H='0.0363'), q=BODY_Q[s], at=at, R0=R0, prefix='b' + t, outdir=work,
                     skin=True, tag='skin-body', skip=[['base', 'base'], 'pedestal']))
    import rose_fork as F
    src = photo(F.MATERIALS + '\n' + '\n'.join(L)) + skin_mats() + hm + '\n'.join(hum) + '\n'
    fn = os.path.join(work, 'body.loon')
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


def main():
    work = os.path.abspath(sys.argv[1])
    os.makedirs(work, exist_ok=True)
    args = [a for a in sys.argv[2:] if not a.startswith('--')]
    spp = int(sys.argv[sys.argv.index('--spp') + 1]) if '--spp' in sys.argv else 256
    args = [a for a in args if not a.isdigit()] or ['a', 'b', 'c', 'd', 'e']
    os.makedirs(OUT, exist_ok=True)
    V = os.environ.get('LYMB_VER', 'v10.1')
    pa, pb = os.path.join(OUT, 'lymb-%s-desk.png' % V), os.path.join(OUT, 'lymb-%s-humanoid.png' % V)
    if 'a' in args:
        render(scene_desk(work, 'a'), pa, -35, 22, spp, size='1600x1400')
        label(pa, 'lymb v10 on the desk', 'J1 yaw / J2 pitch / J3 upper-arm roll / J4 elbow / J6 wrist pitch / J7 tool roll   300 / 200 mm')
    if 'b' in args:
        render(scene_body(work), pb, -30, 12, spp, size='1200x1400', fill=0.94)
        label(pb, 'lymb v10 as the arms of the 1703 mm humanoid', 'same part: J1 = shoulder pitch, J2 = roll, J3 = upper-arm yaw, J4 = elbow')
    if 'c' in args:
        side_by_side(pa, pb, os.path.join(OUT, 'lymb-%s-desk-and-humanoid.png' % V))
    if 'e' in args:
        pe = os.path.join(OUT, 'lymb-%s-elbow-closeup.png' % V)
        render(scene_closeup(work), pe, -50, 25, spp, size='1400x1100', fill=0.92)
        label(pe, 'v10.1 elbow: graphite drum, gunmetal hub cap, the J3 parting band behind it')
    if 'd' in args:
        pd = os.path.join(OUT, 'lymb-v9-vs-%s-desk.png' % V)
        render(scene_desk(work, 'd'), pd, -62, 18, spp, size='1800x1100')
        label(pd, 'v9 (back)  vs  v10 (front), same pose, J2 200 mm over the desk',
              'v9: forearm roll J5, 260 / 220 mm      v10: upper-arm roll J3 just past the shoulder, 300 / 200 mm, 68 mm pedestal')


if __name__ == '__main__':
    if sys.argv[1] == '--child':
        child(json.loads(sys.argv[2]))
    else:
        main()
