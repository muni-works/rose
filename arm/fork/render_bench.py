"""Render yardstick: the same reference frames, rendered by a named variant, timed, and laid
side by side in one gallery page.

  python3 render_bench.py OUT VARIANT [vcad-render flags...]
  python3 render_bench.py OUT --gallery

OUT holds the emitted scenario (made on first use) and one folder per variant. Each variant
renders the hero still (frame 100) and a short clip (frames 90-149), and writes times.json.
The gallery puts every variant's hero next to the others with its per-frame time and a
clip strip. Env: VCAD_RENDER (binary), BENCH_CLIP=0 to skip the clip.
"""
import html
import json
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scenarios

HERO = 100
CLIP = range(90, 150)
VIEW = ['--azimuth=-28', '--elevation=20', '--size', '1280x720', '--fill', '0.98']


def emitted(out):
    fr = os.path.join(out, 'scene')
    if not os.path.exists(os.path.join(fr, 'mini.json')):
        os.makedirs(fr, exist_ok=True)
        here = os.path.dirname(os.path.abspath(__file__))
        for name, (env, _) in scenarios.ARMS.items():
            subprocess.run([sys.executable, os.path.join(here, 'scenarios.py'), '--emit', name, fr],
                           env=dict(os.environ, **env), check=True, stdout=subprocess.DEVNULL)
    return fr


def run(out, variant, flags):
    fr = emitted(out)
    vd = os.path.join(out, variant)
    os.makedirs(vd, exist_ok=True)
    binary = os.environ.get('VCAD_RENDER', scenarios.RENDER)
    frames = [HERO] + ([i for i in CLIP if i != HERO] if os.environ.get('BENCH_CLIP', '1') != '0' else [])
    times = {}
    for i in frames:
        lf = os.path.join(vd, 's%04d.loon' % i)
        open(lf, 'w').write(scenarios.scene(fr, i))
        t0 = time.time()
        r = subprocess.run([binary, lf, *VIEW, *flags, '-o', os.path.join(vd, 'r%04d.png' % i)],
                           capture_output=True, text=True)
        times[i] = round(time.time() - t0, 2)
        if r.returncode:
            print(r.stderr[-600:])
        print('%s frame %d: %.2f s' % (variant, i, times[i]), flush=True)
    json.dump({'flags': flags, 'binary': binary, 'times': times}, open(os.path.join(vd, 'times.json'), 'w'), indent=1)


def gallery(out):
    rows = []
    for v in sorted(os.listdir(out)):
        tj = os.path.join(out, v, 'times.json')
        if not os.path.exists(tj):
            continue
        t = json.load(open(tj))
        ts = t['times']
        first, rest = ts.get(str(HERO)), [x for k, x in ts.items() if k != str(HERO)]
        warm = sum(rest) / len(rest) if rest else None
        strip = ''.join('<img src="%s/r%04d.png">' % (v, i) for i in list(CLIP)[::10]
                        if os.path.exists(os.path.join(out, v, 'r%04d.png' % i)))
        rows.append('<section><h2>%s</h2><p>%s<br>hero %.1f s%s</p><img class=hero src="%s/r%04d.png">'
                    '<div class=strip>%s</div></section>' % (
                        html.escape(v), html.escape(' '.join(t['flags'])), first,
                        '' if warm is None else ', clip %.2f s/frame' % warm, v, HERO, strip))
    page = ('<!doctype html><meta charset=utf-8><title>Render yardstick</title><style>'
            'body{font:14px system-ui;margin:24px;background:#fafafa}section{margin-bottom:40px}'
            '.hero{width:100%;max-width:1280px;display:block}.strip img{width:12%;margin-right:4px}</style>'
            '<h1>Render yardstick</h1>' + ''.join(rows))
    open(os.path.join(out, 'index.html'), 'w').write(page)
    print(os.path.join(out, 'index.html'))


if __name__ == '__main__':
    out = sys.argv[1]
    if sys.argv[2] == '--gallery':
        gallery(out)
    else:
        run(out, sys.argv[2], sys.argv[3:])
