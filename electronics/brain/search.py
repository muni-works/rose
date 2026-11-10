"""Placement search for the module: mutate anchor positions, score each
layout with a fast route + KiCad's connectivity count, keep the best.

    BOARD=module python3 search.py [hours] [workers]

Each worker routes in its own copy of this directory; results append to
search.csv (score, anchors JSON). The best layout is printed as a
PLACE_OVERRIDE to apply with place.py.
"""
import json, os, random, re, shutil, subprocess, sys, time
from concurrent.futures import ThreadPoolExecutor

import place

HERE = place.HERE
SCRATCH = os.environ.get('SEARCH_DIR', '/tmp/brain_search')
MOVABLE = {'U_DRAM0': 2.5, 'U_DRAM1': 2.5, 'U_EMMC': 3.0, 'U_PMIC': 4.0}
ROTS = {'U_DRAM0': (0, 180), 'U_DRAM1': (90, 270), 'U_EMMC': (0, 90, 180, 270), 'U_PMIC': (0, 90, 180, 270)}


def score(i, anchors):
    d = os.path.join(SCRATCH, f'w{i}', 'brain')
    if not os.path.isdir(d):
        os.makedirs(os.path.dirname(d), exist_ok=True)
        shutil.copytree(HERE, d, ignore=shutil.ignore_patterns('*.kicad_pcb', '*.ckpt', 'render', 'reference', '*.pdf'))
        os.symlink(os.path.join(HERE, '..', 'arm'), os.path.join(SCRATCH, f'w{i}', 'arm'))
        # pinned DDR copper belongs to the committed placement: drop it
        lp = os.path.join(d, 'module.loon')
        src = open(lp).read()
        b, e = ';; ---- ddr pinned (ddr_pin.py)', ';; ---- end ddr pinned'
        if b in src:
            src = src[:src.index(b)] + src[src.index(e) + len(e) + 1:]
        open(lp, 'w').write(src)
    env = dict(os.environ, BOARD='module', FAST='1', VCAD_ROUTE_CHECKPOINT='0',
               PLACE_OVERRIDE=json.dumps({k: list(v) for k, v in anchors.items()}))
    r = subprocess.run(['sh', 'route.sh'], cwd=d, env=env, capture_output=True, text=True)
    m = re.search(r'unconnected \(KiCad connectivity\): (\d+)', r.stdout + r.stderr)
    return int(m.group(1)) if m else 10 ** 6


def mutate(a):
    a = dict(a)
    for k in random.sample(list(MOVABLE), random.randint(1, 2)):
        x, y, rot, side = a[k]
        s = MOVABLE[k]
        x, y = round(x + random.uniform(-s, s), 2), round(y + random.uniform(-s, s), 2)
        if random.random() < 0.2:
            rot = random.choice(ROTS[k])
        a[k] = (x, y, rot, side)
    return a


def main():
    hours = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0
    workers = int(sys.argv[2]) if len(sys.argv) > 2 else 4
    best = {k: place.MODULE_ANCHORS[k] for k in MOVABLE}
    log = open(os.path.join(HERE, 'search.csv'), 'a')
    with ThreadPoolExecutor(workers) as ex:
        best_score = score(0, best)
        print(f'baseline {best_score}', flush=True)
        log.write(f'{best_score},{json.dumps(best)}\n'); log.flush()
        end, n = time.time() + hours * 3600, 0
        while time.time() < end:
            cands = [mutate(best) for _ in range(workers)]
            for c, s in zip(cands, ex.map(score, range(workers), cands)):
                n += 1
                log.write(f'{s},{json.dumps(c)}\n'); log.flush()
                if s < best_score:
                    best, best_score = c, s
                    print(f'[{n}] new best {s}: {json.dumps(c)}', flush=True)
            print(f'[{n}] best {best_score}', flush=True)
    print('PLACE_OVERRIDE=' + json.dumps({k: list(v) for k, v in best.items()}))


if __name__ == '__main__':
    main()
