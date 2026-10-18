"""Render the build steps from out/assembly (run `leg.py --assembly` first).

Each step shows the parts already fitted in grey and the parts going on in this step in
orange, pulled out along the hip axis (Y) by the distance they travel when you fit them.
Writes out/assembly/step-N.loon and render/step-N.png.
"""
import os
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
A = os.path.join(HERE, 'out', 'assembly')
R = os.environ.get('VCAD_RENDER', os.path.expanduser('~/Developer/vcad/target/release/vcad-render'))

HIP = ['side-out', 'rs00-hip']
KNEE = ['side-in', 'rs00-knee']
TRAIN = ['shafts', 'idler-1', 'idler-2', 'shin']
LEG = KNEE + HIP + ['pinion', 'thigh-inner', 'thigh-outer'] + TRAIN
YOKE = ['arch', 'keys', 'rs00-abad', 'bulkhead']

# (title, already fitted, [(new part, (dx, dy, dz) explode)], camera azimuth)
STEPS = [
    ('motors onto the side plates', [], [('side-out', (0, 30, 0)), ('rs00-hip', (0, 0, 0)),
                                         ('side-in', (0, -30, 0)), ('rs00-knee', (0, 0, 0))], 50),
    ('pinion onto the knee motor', KNEE, [('pinion', (0, 35, 0))], 50),
    ('outer thigh plate onto the hip motor', HIP, [('thigh-outer', (0, -35, 0))], -50),
    ('inner thigh plate over the pinion hub', KNEE + ['pinion'], [('thigh-inner', (0, 35, 0))], 50),
    ('shafts, idlers and shin', KNEE + ['pinion', 'thigh-inner'],
     [('shafts', (0, 25, 0)), ('idler-1', (0, 45, 0)), ('idler-2', (0, 45, 0)), ('shin', (0, 45, 0))], 50),
    ('close the fork with the hip side', KNEE + ['pinion', 'thigh-inner'] + TRAIN,
     [('thigh-outer', (0, 45, 0)), ('rs00-hip', (0, 45, 0)), ('side-out', (0, 45, 0))], 50),
    ('arch over the tenons, drive the keys', LEG, [('arch', (40, 0, 0)), ('keys', (0, 0, 30))], 35),
    ('ab/ad motor and bulkhead', LEG + ['arch', 'keys'], [('rs00-abad', (45, 0, 0)), ('bulkhead', (80, 0, 0))], 35),
    ('paw', LEG + YOKE, [('paw', (0, 0, -35))], 35),
]
MAT = {'rs00': 'black', 'shafts': 'steel', 'keys': 'steel', 'paw': 'rubber'}


def mat(name, new):
    if new:
        return 'copper'
    return MAT.get(name.split('-')[0], 'aluminum')


def main():
    os.makedirs(os.path.join(HERE, 'render'), exist_ok=True)
    for i, (title, old, new, az) in enumerate(STEPS, 1):
        lines = [f';; step {i}: {title}']
        lines += [f'[root [import-mesh "{p}.stl"] "{mat(p, False)}"]' for p in old]
        lines += [f'[root [translate {d[0]} {d[1]} {d[2]} [import-mesh "{p}.stl"]] "{mat(p, True)}"]' for p, d in new]
        src = os.path.join(A, f'step-{i}.loon')
        open(src, 'w').write('\n'.join(lines) + '\n')
        png = os.path.join(HERE, 'render', f'step-{i}.png')
        subprocess.run([R, src, f'--azimuth={az}', '--elevation=20', '--size', '900', '--fill', '0.9',
                        '--auto-aspect', '--raytrace', '-o', png], check=True)
        print(f'step {i}: {title} -> {png}')
    # a self-contained copy of the guide (images inlined) for viewers that can't follow relative paths
    import base64, re
    page = open(os.path.join(HERE, 'assembly.html')).read()
    page = re.sub(r'src="(render/step-\d+\.png)"', lambda m: 'src="data:image/png;base64,' +
                  base64.b64encode(open(os.path.join(HERE, m.group(1)), 'rb').read()).decode() + '"', page)
    open(os.path.join(HERE, 'out', 'assembly-guide.html'), 'w').write(page)


if __name__ == '__main__':
    main()
