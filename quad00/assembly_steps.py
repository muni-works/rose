"""Render the build steps from out/assembly (run `leg.py --assembly` first).

Each step shows the parts already fitted in grey and the parts going on in this step in
orange, pulled out along the hip axis (Y) by the distance they travel when you fit them.
Writes out/assembly/step-N.loon and render/step-N.png.
"""
import base64
import os
import re
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
A = os.path.join(HERE, 'out', 'assembly')
R = os.environ.get('VCAD_RENDER', os.path.expanduser('~/Developer/vcad/target/release/vcad-render'))

HIP = ['rs00-hip']
KNEE = ['rs00-knee']
TRAIN = ['shafts', 'idler-1', 'idler-2', 'shin']
LEG = KNEE + HIP + ['pinion', 'thigh-inner', 'thigh-outer'] + TRAIN

# (title, already fitted, [(new part, (dx, dy, dz) explode)], camera azimuth)
STEPS = [
    ('pinion onto the knee motor', KNEE, [('pinion', (0, 35, 0))], 50),
    ('outer thigh plate onto the hip motor', HIP, [('thigh-outer', (0, -35, 0))], -50),
    ('inner thigh plate over the pinion hub', KNEE + ['pinion'], [('thigh-inner', (0, 35, 0))], 50),
    ('shafts, idlers and shin', KNEE + ['pinion', 'thigh-inner'],
     [('shafts', (0, 25, 0)), ('idler-1', (0, 45, 0)), ('idler-2', (0, 45, 0)), ('shin', (0, 45, 0))], 50),
    ('close the fork with the hip side', KNEE + ['pinion', 'thigh-inner'] + TRAIN,
     [('thigh-outer', (0, 45, 0)), ('rs00-hip', (0, 45, 0))], 50),
    ('ab/ad motor onto the yoke', [], [('yoke', (0, 0, 0)), ('rs00-abad', (40, 0, 0))], 35),
    ('yoke over the leg', LEG, [('yoke', (70, 0, 0)), ('rs00-abad', (70, 0, 0))], 35),
    ('paw, and the bench plate', LEG + ['yoke', 'rs00-abad'], [('paw', (0, 0, -35)), ('bulkhead', (40, 0, 0))], 35),
]
MAT = {'rs00': 'black', 'shafts': 'steel', 'paw': 'rubber'}


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
    page = open(os.path.join(HERE, 'assembly.html')).read()
    page = re.sub(r'src="(render/step-\d+\.png)"', lambda m: 'src="data:image/png;base64,' +
                  base64.b64encode(open(os.path.join(HERE, m.group(1)), 'rb').read()).decode() + '"', page)
    open(os.path.join(HERE, 'out', 'assembly-guide.html'), 'w').write(page)
    viewer()


def viewer():
    """out/assembly/leg.glb (every posed part, named) and out/viewer.html (the three.js page, GLB inlined)."""
    import glob
    import trimesh
    scene = trimesh.Scene()
    for f in sorted(glob.glob(os.path.join(A, '*.stl'))):
        n = os.path.basename(f)[:-4]
        m = trimesh.load(f)
        m.merge_vertices()
        scene.add_geometry(m, node_name=n, geom_name=n)
    glb = os.path.join(A, 'leg.glb')
    scene.export(glb)
    page = open(os.path.join(HERE, 'viewer.template.html')).read()
    page = page.replace('__GLB_BASE64__', base64.b64encode(open(glb, 'rb').read()).decode())
    open(os.path.join(HERE, 'out', 'viewer.html'), 'w').write(page)
    print(f'viewer: {glb} + out/viewer.html')


if __name__ == '__main__':
    main()
