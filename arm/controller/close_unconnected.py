"""Close what the router left open, judged by KiCad's own DRC.

    close_unconnected.py board.kicad_pcb drc.json      (KiCad's Python; route.sh loops it)

For each pair KiCad reports unconnected:
  - a pad on a net with an inner plane (GND, +3V3) gets a via to the plane:
    in the pad if it fits, else beside it with a short stub;
  - anything else gets a straight or L-shaped trace between the two ends, on
    the first layer where it clears every other net (dropping through vias
    to an inner signal layer when the outer layers are blocked).
Every addition is checked against tracks, vias and pads of other nets on each
layer it touches. route.sh refills and re-runs DRC afterwards, and
drop_added.py removes anything that still caused a violation.
Prints how many connections it added; tags them in added.txt.
"""
import json, math, sys
import pcbnew

TRACK_W, VIA_D, VIA_DRILL = 0.15, 0.45, 0.25     # mm, the board's defaults
CLEAR, HOLE_GAP = 0.16, 0.26                    # mm, a hair over the rules

mm, to_mm = pcbnew.FromMM, pcbnew.ToMM
board = pcbnew.LoadBoard(sys.argv[1])
report = json.load(open(sys.argv[2]))
COPPER = [pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.In3_Cu, pcbnew.In4_Cu, pcbnew.B_Cu]
SIGNAL = [pcbnew.F_Cu, pcbnew.B_Cu, pcbnew.In2_Cu, pcbnew.In4_Cu]
planes = {}
for z in board.Zones():
    if z.GetLayer() in (pcbnew.In1_Cu, pcbnew.In3_Cu):
        planes[z.GetNetname()] = z.GetLayer()

# obstacles: (net, layer-set, kind, geometry) in mm
segs, discs, rects = [], [], []
for t in board.GetTracks():
    if t.Type() == pcbnew.PCB_VIA_T:
        p = t.GetPosition()
        discs.append((t.GetNetname(), set(COPPER), to_mm(p.x), to_mm(p.y), to_mm(t.GetWidth(pcbnew.F_Cu)) / 2, to_mm(t.GetDrillValue()) / 2))
    else:
        a, b = t.GetStart(), t.GetEnd()
        segs.append((t.GetNetname(), t.GetLayer(), to_mm(a.x), to_mm(a.y), to_mm(b.x), to_mm(b.y), to_mm(t.GetWidth()) / 2))
pads = []
for f in board.GetFootprints():
    for p in f.Pads():
        bb = p.GetBoundingBox()
        lay = {l for l in COPPER if p.IsOnLayer(l)}
        c = p.GetPosition()
        r = (to_mm(bb.GetLeft()), to_mm(bb.GetTop()), to_mm(bb.GetRight()), to_mm(bb.GetBottom()))
        drill = to_mm(max(p.GetDrillSize().x, p.GetDrillSize().y)) / 2
        pads.append((p.GetNetname(), lay, r, to_mm(c.x), to_mm(c.y), drill, p))
        rects.append((p.GetNetname(), lay, r))


def seg_pt(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    t = 0 if l2 == 0 else max(0, min(1, ((px - ax) * dx + (py - ay) * dy) / l2))
    return math.hypot(ax + t * dx - px, ay + t * dy - py)


def seg_seg(a, b, c, d):
    def cross(o, p, q):
        return (p[0] - o[0]) * (q[1] - o[1]) - (p[1] - o[1]) * (q[0] - o[0])
    if (cross(a, b, c) * cross(a, b, d) < 0) and (cross(c, d, a) * cross(c, d, b) < 0):
        return 0.0
    return min(seg_pt(*a, *c, *d), seg_pt(*b, *c, *d), seg_pt(*c, *a, *b), seg_pt(*d, *a, *b))


def rect_seg(r, a, b, hw):
    x0, y0, x1, y1 = r
    if x0 - hw <= a[0] <= x1 + hw and y0 - hw <= a[1] <= y1 + hw:
        return 0.0
    if x0 - hw <= b[0] <= x1 + hw and y0 - hw <= b[1] <= y1 + hw:
        return 0.0
    edges = [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]
    return min(seg_seg(a, b, e0, e1) for e0, e1 in edges)


def seg_clear(net, layer, a, b, hw):
    for n, l, ax, ay, bx, by, w in segs:
        if n != net and l == layer and seg_seg(a, b, (ax, ay), (bx, by)) < hw + w + CLEAR:
            return False
    for n, ls, x, y, r, _ in discs:
        if n != net and layer in ls and seg_pt(x, y, *a, *b) < hw + r + CLEAR:
            return False
    for n, ls, r in rects:
        if n != net and layer in ls and rect_seg(r, a, b, hw + CLEAR) == 0.0:
            return False
    return True


def via_clear(net, x, y, layers=None):
    r = VIA_D / 2
    for l in (layers or COPPER):
        if l in planes.values():
            continue                     # planes: the fill clears its own antipad
        if not seg_clear(net, l, (x, y), (x, y), r):
            return False
    for n, ls, vx, vy, vr, dr in discs:
        if math.hypot(vx - x, vy - y) < VIA_DRILL / 2 + dr + HOLE_GAP:
            return False
    for n, ls, rr, px, py, dr, _ in pads:
        if dr > 0 and math.hypot(px - x, py - y) < VIA_DRILL / 2 + dr + HOLE_GAP:
            return False
    return True


added = []


def add_track(net, layer, a, b):
    t = pcbnew.PCB_TRACK(board)
    t.SetStart(pcbnew.VECTOR2I(mm(a[0]), mm(a[1])))
    t.SetEnd(pcbnew.VECTOR2I(mm(b[0]), mm(b[1])))
    t.SetWidth(mm(TRACK_W))
    t.SetLayer(layer)
    t.SetNet(board.FindNet(net))
    board.Add(t)
    segs.append((net, layer, a[0], a[1], b[0], b[1], TRACK_W / 2))
    added.append(t.m_Uuid.AsString())


def add_via(net, x, y):
    v = pcbnew.PCB_VIA(board)
    v.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
    for l in COPPER:
        v.SetWidth(mm(VIA_D), l)
    v.SetDrill(mm(VIA_DRILL))
    v.SetNet(board.FindNet(net))
    board.Add(v)
    discs.append((net, set(COPPER), x, y, VIA_D / 2, VIA_DRILL / 2))
    added.append(v.m_Uuid.AsString())


def pad_at(net, x, y):
    best = None
    for n, ls, r, px, py, dr, p in pads:
        if n == net and r[0] - 0.05 <= x <= r[2] + 0.05 and r[1] - 0.05 <= y <= r[3] + 0.05:
            d = math.hypot(px - x, py - y)
            if best is None or d < best[0]:
                best = (d, px, py, ls)
    return best


def to_plane(net, x, y, layer):
    """A via from the pad/track end at (x, y) on `layer` down to its net's plane."""
    for ring in (0.0, 0.55, 0.8, 1.1, 1.5, 2.0, 2.5, 3.0):
        for k in range(1 if ring == 0 else 24):
            a = 2 * math.pi * k / 24
            vx, vy = x + ring * math.cos(a), y + ring * math.sin(a)
            if not via_clear(net, vx, vy):
                continue
            if ring and not seg_clear(net, layer, (x, y), (vx, vy), TRACK_W / 2):
                continue
            add_via(net, vx, vy)
            if ring:
                add_track(net, layer, (x, y), (vx, vy))
            return True
    return False


def join(net, a, la, b, lb):
    """Straight or L trace a->b on a shared signal layer, via-hopping if needed."""
    paths = [[a, b], [a, (a[0], b[1]), b], [a, (b[0], a[1]), b]]
    for layer in SIGNAL:
        hop_a = layer not in la
        hop_b = layer not in lb
        if (hop_a and not via_clear(net, *a)) or (hop_b and not via_clear(net, *b)):
            continue
        for path in paths:
            if all(seg_clear(net, layer, p, q, TRACK_W / 2) for p, q in zip(path, path[1:])):
                if hop_a:
                    add_via(net, *a)
                if hop_b:
                    add_via(net, *b)
                for p, q in zip(path, path[1:]):
                    add_track(net, layer, p, q)
                return True
    return False


def end_of(item):
    """(net, (x, y) mm, layer-set) for a DRC item: pads snap to their centre."""
    net = item.get('description', '')
    n = net[net.find('[') + 1:net.find(']')] if '[' in net else ''
    x, y = item['pos']['x'], item['pos']['y']
    hit = pad_at(n, x, y)
    if hit:
        return n, (hit[1], hit[2]), hit[3]
    lay = {l for l in SIGNAL if board.GetLayerName(l) in item.get('description', '')} or {pcbnew.F_Cu}
    return n, (x, y), lay


def same_net_points(net, near, reach=6.0):
    """Copper of `net` within `reach` of `near`, nearest first: (point, layers)."""
    pts = []
    for n, ls, r, px, py, dr, _ in pads:
        if n == net:
            pts.append(((px, py), ls & set(SIGNAL)))
    for n, ls, x, y, r, _ in discs:
        if n == net:
            pts.append(((x, y), set(SIGNAL)))
    for n, l, ax, ay, bx, by, w in segs:
        if n == net:
            pts += [((ax, ay), {l}), ((bx, by), {l})]
    out = [(math.dist(p, near), p, ls) for p, ls in pts if 0.3 < math.dist(p, near) < reach and ls]
    return [(p, ls) for d, p, ls in sorted(out)[:10]]


def join_any(net, a, la):
    for p, lp in same_net_points(net, a):
        if join(net, a, la, p, lp):
            return True
    return False


closed = 0
for v in report.get('unconnected_items', []):
    items = v.get('items', [])
    if len(items) != 2 or 'Zone' in items[0].get('description', ''):
        continue
    (na, a, la), (nb, b, lb) = end_of(items[0]), end_of(items[1])
    net = na or nb
    # a track that ends at its partner on another layer: the missing piece is a via
    if math.dist(a, b) < 1.5 and not (la & lb):
        tip = a if len(la) == 1 and not pad_at(net, *a) else b
        other = b if tip is a else a
        tip_layers = la if tip is a else lb
        if join_any(net, tip, tip_layers):                # a same-net via or pad nearby, on the tip's own layer
            closed += 1
            sys.stderr.write('%-10s joined to nearby copper (%.2f,%.2f)\n' % (net, tip[0], tip[1]))
            continue
        if via_clear(net, *tip):
            add_via(net, *tip)
            if math.dist(tip, other) > 0.05:
                lay = next(iter((lb if tip is a else la) & set(SIGNAL)), pcbnew.F_Cu)
                if seg_clear(net, lay, tip, other, TRACK_W / 2):
                    add_track(net, lay, tip, other)
            closed += 1
            sys.stderr.write('%-10s via at layer change (%.2f,%.2f)\n' % (net, tip[0], tip[1]))
            continue
    if net in planes:
        ok = False
        for (p, ls) in ((a, la), (b, lb)):
            lay = next(iter(ls & set(SIGNAL)), pcbnew.F_Cu)
            ok = to_plane(net, p[0], p[1], lay) or join_any(net, p, ls) or ok
        closed += ok
    else:
        ok = join(net, a, la, b, lb) or join_any(net, a, la) or join_any(net, b, lb)
        closed += ok
    sys.stderr.write('%-10s %s (%.2f,%.2f)-(%.2f,%.2f)\n' % (net or '?', 'closed' if ok else 'BLOCKED', a[0], a[1], b[0], b[1]))
# zero-length tracks are router debris: they connect nothing and confuse the ends
for t in [t for t in list(board.GetTracks()) if t.Type() == pcbnew.PCB_TRACE_T and t.GetStart() == t.GetEnd()]:
    board.Remove(t)
board.Save(sys.argv[1])
open('added.txt', 'a').write('\n'.join(added) + '\n')
print(closed)
