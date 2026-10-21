"""Photoreal renders of the lymb exterior: 3/4 hero, side, and a pose sweep strip.

  python3 arm/fork/exterior/render.py OUT [cream|black] [spp]

Env: VCAD_RENDER (default ~/Developer/vcad-fast/target/release/vcad-render, a loon-reading build with
--photoreal), ROSE_ARM=mini for the mini arm. Shop finishes come from scenarios.PHOTO_MATERIALS.
"""
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..'))
import exterior as X
import rose_fork as F
import scenarios

RENDER = os.environ.get('VCAD_RENDER', os.path.expanduser('~/Developer/vcad-fast/target/release/vcad-render'))
HERO = dict(J2_sho_pitch=-35, J4_elbow=70, J6_wrist_pitch=25)
SWEEP = [('reach', dict(J2_sho_pitch=0, J4_elbow=0, J6_wrist_pitch=0)),
         ('raise', dict(J2_sho_pitch=-60, J4_elbow=40, J6_wrist_pitch=30)),
         ('fold', dict(J2_sho_pitch=-80, J4_elbow=115, J6_wrist_pitch=-60)),
         ('down', dict(J2_sho_pitch=30, J4_elbow=60, J6_wrist_pitch=90)),
         ('up', dict(J2_sho_pitch=-90, J4_elbow=0, J6_wrist_pitch=0)),
         ('over', dict(J2_sho_pitch=-150, J4_elbow=-60, J6_wrist_pitch=-90))]


def scene(q, shell):
    src = X.world_loon({k: math.radians(v) for k, v in q.items()}, shell)
    src = src.replace(F.MATERIALS, scenarios.PHOTO_MATERIALS)
    for a, b in scenarios.PHOTO_RENAME.items():
        src = src.replace(a, b)
    return src


def render(out, name, q, shell, spp, view):
    lf = os.path.join(out, name + '.loon')
    open(lf, 'w').write(scene(q, shell))
    png = os.path.join(out, name + '.png')
    cmd = [RENDER, lf, '-o', png, '--photoreal', '--spp', str(spp), '--fov', '30', '--backdrop', 'studio',
           '--tonemap', 'neutral', '--vignette', '0.15', '--edge-radius', '0.6', '--fill', '0.78'] + view
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode:
        print(r.stderr[-800:])
    print(png, flush=True)
    return png


def main(out, shell='cream', spp=128):
    os.makedirs(out, exist_ok=True)
    tag = '%s-%s' % (F.SIZE, shell)
    render(out, 'lymb-%s-34' % tag, HERO, shell, spp, ['--azimuth=-35', '--elevation=20', '--size', '1600x1100'])
    render(out, 'lymb-%s-side' % tag, HERO, shell, spp, ['--azimuth=90', '--elevation=6', '--size', '1600x1100'])
    tiles = [render(out, 'sweep-%s-%s' % (tag, n), q, shell, max(32, spp // 2), ['--azimuth=-35', '--elevation=20', '--size', '700x700'])
             for n, q in SWEEP]
    from PIL import Image
    ims = [Image.open(t) for t in tiles if os.path.exists(t)]
    if ims:
        w, h = ims[0].size
        strip = Image.new('RGB', (w * 3, h * ((len(ims) + 2) // 3)), 'white')
        for i, im in enumerate(ims):
            strip.paste(im.convert('RGB'), ((i % 3) * w, (i // 3) * h))
        strip.save(os.path.join(out, 'lymb-%s-sweep.png' % tag))
        for t in tiles:
            os.remove(t)


if __name__ == '__main__':
    a = sys.argv[1:]
    main(a[0], a[1] if len(a) > 1 else 'cream', int(a[2]) if len(a) > 2 else 128)
