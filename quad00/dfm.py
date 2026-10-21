"""DFM pass over the mini humanoid's plates: stock thickness, thin webs in the pocketed profiles (router
rule: web >= 1.5 x stock), and the sheet bill (plates, area, mass). Usage: python3 dfm.py"""
import collections, numpy as np
import quad as Q
import leg as L

WEB = 1.5 * L.T                                         # 4.8 mm minimum web in 3.2 stock


def thin_area(p, w=WEB):
    """area of features narrower than w (lost by an opening of radius w/2)."""
    opened = p.buffer(-w / 2, join_style=1).buffer(w / 2, join_style=1)
    t = p.difference(opened)
    # sharp OUTER corners also fall out of an opening (sub-mm^2 slivers); only count real webs
    return sum(g.area for g in getattr(t, 'geoms', [t]) if g.area > 3.0)


def main(fn=Q.humanoid_mini):
    Q.POCKETED.clear()
    H = fn()
    worst = sorted(((thin_area(p), i) for i, p in enumerate(Q.POCKETED)), reverse=True)
    flagged = [(round(a), i) for a, i in worst if a > 2.0]
    print(f'pocketed profiles: {len(Q.POCKETED)}; with webs under {WEB:.1f} mm: {len(flagged)}', flagged[:8])
    sheet = collections.Counter(); count = collections.Counter()
    for n, m in H['extra']:
        base = n.split('.')[0]
        if base.startswith(('rs0', 'int-', 'pivot', 'head-pivot')):
            continue
        nrm = m.face_normals[np.argmax(m.area_faces)]       # the plate's face normal
        t = float(np.ptp(m.vertices @ nrm))
        key = '3.2' if t < 4.5 else '6.4' if t < 8 else 'assembly'
        count[key] += 1
        sheet[key] += m.volume / (3.2 if key == '3.2' else 6.4 if key == '6.4' else 3.2) / 100.0
    al = sum(m.volume for n, m in H['extra'] if not n.split('.')[0].startswith(('rs0', 'int-'))) * 2.7e-6
    print('parts by stock:', dict(count), '; sheet area cm^2 ~', {k: round(v) for k, v in sheet.items()}, f'; aluminium {al:.1f} kg')


if __name__ == '__main__':
    main()
