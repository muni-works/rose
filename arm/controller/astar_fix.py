"""Hand-fix router for the last open connections: a fine-grid A* per pair.

    astar_fix.py board.kicad_pcb drc.json       (KiCad's Python; route.sh runs it last)

For each pair KiCad's DRC still calls unconnected, this does what you'd do by
hand in a 6 mm window round the open end:
  - rasterise the signal layers (F, In2, In4, B) at 0.05 mm, marking every
    other-net pad, track and via, grown by clearance and half the trace width;
  - search from the open pad to its partner (pad, via or track), or - for a
    net with an inner plane - to any spot a via fits, which reaches the plane;
  - lay the path as tracks (merged runs) and vias.
Additions go to added.txt so drop_added.py can remove anything DRC rejects.
Logs every fix it makes, so the board's hand work is on record (route.log).
"""
import heapq, json, math, sys
import pcbnew

GRID = 0.05                       # mm
WIN = 3.0                         # mm half-window round the search
TRACK_W, VIA_D, VIA_DRILL = 0.15, 0.45, 0.25      # the board's default via
CLEAR, HOLE_GAP = 0.155, 0.16     # a hair over the board rules (clearance, hole-to-hole)
POWER = {'+12V', '+5V', 'VIN_BUCK', 'VAUX48'}   # the 'Power' class: 0.2 mm clearance
POWER_EXTRA = 0.05
VIA_COST = 1.2                    # mm-equivalent

mm, to_mm = pcbnew.FromMM, pcbnew.ToMM
board = pcbnew.LoadBoard(sys.argv[1])
report = json.load(open(sys.argv[2]))
SIG = [pcbnew.F_Cu, pcbnew.In2_Cu, pcbnew.In4_Cu, pcbnew.B_Cu]
ALL = [pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.In3_Cu, pcbnew.In4_Cu, pcbnew.B_Cu]
planes = {z.GetNetname(): z.GetLayer() for z in board.Zones() if z.GetLayer() in (pcbnew.In1_Cu, pcbnew.In3_Cu)}

pads, tracks, vias = [], [], []
for f in board.GetFootprints():
    for p in f.Pads():
        bb = p.GetBoundingBox()
        pads.append(dict(net=p.GetNetname(), layers={l for l in ALL if p.IsOnLayer(l)},
                         box=(to_mm(bb.GetLeft()), to_mm(bb.GetTop()), to_mm(bb.GetRight()), to_mm(bb.GetBottom())),
                         c=(to_mm(p.GetPosition().x), to_mm(p.GetPosition().y)),
                         drill=to_mm(max(p.GetDrillSize().x, p.GetDrillSize().y)) / 2, ref='%s.%s' % (f.GetReference(), p.GetNumber())))
for t in board.GetTracks():
    if t.Type() == pcbnew.PCB_VIA_T:
        vias.append(dict(net=t.GetNetname(), c=(to_mm(t.GetPosition().x), to_mm(t.GetPosition().y)),
                         r=to_mm(t.GetWidth(pcbnew.F_Cu)) / 2, drill=to_mm(t.GetDrillValue()) / 2))
    else:
        a, b = t.GetStart(), t.GetEnd()
        tracks.append(dict(net=t.GetNetname(), layer=t.GetLayer(), a=(to_mm(a.x), to_mm(a.y)), b=(to_mm(b.x), to_mm(b.y)),
                           hw=to_mm(t.GetWidth()) / 2, obj=t))


def seg_pt(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    l2 = dx * dx + dy * dy
    t = 0 if l2 == 0 else max(0, min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2))
    return math.hypot(a[0] + t * dx - p[0], a[1] + t * dy - p[1])


def box_pt(bx, p):
    dx = max(bx[0] - p[0], 0, p[0] - bx[2])
    dy = max(bx[1] - p[1], 0, p[1] - bx[3])
    return math.hypot(dx, dy)


class Window:
    def __init__(self, net, centre, half=WIN, grid=GRID):
        self.net = net
        self.grid = grid
        self.x0, self.y0 = centre[0] - half, centre[1] - half
        self.n = int(2 * half / grid) + 1
        self.free = {l: bytearray(b'\x01' * self.n * self.n) for l in SIG}   # trace may run here
        self.soft = {l: {} for l in SIG}          # cell -> other-net track ids (rip-up candidates)
        self.viaok = bytearray(b'\x01' * self.n * self.n)                     # a via may sit here
        lo, hi = (self.x0 - 1, self.y0 - 1), (self.x0 + 2 * half + 1, self.y0 + 2 * half + 1)
        near = lambda q: lo[0] < q[0] < hi[0] and lo[1] < q[1] < hi[1]
        my_pads = [pd for pd in pads if near(pd['c'])]
        my_vias = [v for v in vias if near(v['c'])]
        my_tracks = [t for t in tracks if t['net'] != net and (near(t['a']) or near(t['b']) or seg_pt(((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2), t['a'], t['b']) < half * 1.5 + 1)]
        slack = 0.0                     # cell centres only are checked; DRC + drop_added.py catch a clipped corner (a fix goes whole)
        tw = TRACK_W / 2 + CLEAR + slack
        vr = VIA_D / 2 + CLEAR + slack
        n, gr = self.n, grid

        def cells(x0, y0, x1, y1):
            i0, j0 = max(0, int((x0 - self.x0) / gr)), max(0, int((y0 - self.y0) / gr))
            i1, j1 = min(n - 1, int((x1 - self.x0) / gr) + 1), min(n - 1, int((y1 - self.y0) / gr) + 1)
            for j in range(j0, j1 + 1):
                for i in range(i0, i1 + 1):
                    yield i, j, j * n + i, (self.x0 + i * gr, self.y0 + j * gr)

        m = max(tw, vr) + HOLE_GAP + 0.5
        for pd in my_pads:
            b = pd['box']
            for i, j, k, p in cells(b[0] - m, b[1] - m, b[2] + m, b[3] + m):
                if pd['net'] != net:
                    d = box_pt(b, p) - (POWER_EXTRA if pd['net'] in POWER else 0)
                    for l in SIG:
                        if l in pd['layers'] and d < tw:
                            self.free[l][k] = 0
                    if any(l in pd['layers'] for l in ALL if l not in planes.values()) and d < vr:
                        self.viaok[k] = 0
                if pd['drill'] and math.dist(p, pd['c']) < pd['drill'] + VIA_DRILL / 2 + HOLE_GAP:
                    self.viaok[k] = 0
        for v in my_vias:
            c = v['c']
            for i, j, k, p in cells(c[0] - m, c[1] - m, c[0] + m, c[1] + m):
                d = math.dist(p, c) - (POWER_EXTRA if v['net'] in POWER else 0)
                if d < v['drill'] + VIA_DRILL / 2 + HOLE_GAP:
                    self.viaok[k] = 0
                if v['net'] == net:
                    continue
                if d < v['r'] + tw:
                    for l in SIG:
                        self.free[l][k] = 0
                if d < v['r'] + vr:
                    self.viaok[k] = 0
        for t in my_tracks:
            a, b = t['a'], t['b']
            mm_ = t['hw'] + vr
            for i, j, k, p in cells(min(a[0], b[0]) - mm_, min(a[1], b[1]) - mm_, max(a[0], b[0]) + mm_, max(a[1], b[1]) + mm_):
                d = seg_pt(p, a, b) - (POWER_EXTRA if t['net'] in POWER else 0)
                if d < t['hw'] + tw and t['layer'] in self.soft:
                    self.soft[t['layer']].setdefault(k, set()).add(id(t))
                if d < t['hw'] + vr:
                    self.viaok[k] = 0
        self.by_id = {id(t): t for t in my_tracks}

    def pt(self, i, j):
        return (self.x0 + i * self.grid, self.y0 + j * self.grid)

    def idx(self, p):
        return (round((p[0] - self.x0) / self.grid), round((p[1] - self.y0) / self.grid))


RIP_COST = 3.0      # mm-equivalent per cell through another net's track


def search(w, starts, goal, rip=True):
    """starts: [(i, j, layer)]; goal(i, j, layer) -> bool. Returns the node path.
    With rip, other nets' tracks are passable at RIP_COST per cell."""
    D = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
    g, came, heap = {}, {}, []
    target = getattr(goal, 'target', None)
    h = (lambda i, j: math.hypot(i - target[0], j - target[1]) * w.grid) if target else (lambda i, j: 0.0)
    for s in starts:
        g[s] = 0
        heapq.heappush(heap, (0, s))
    while heap:
        _, s = heapq.heappop(heap)
        i, j, l = s
        if goal(i, j, l):
            path = [s]
            while path[-1] in came:
                path.append(came[path[-1]])
            return path[::-1]
        for di, dj, c in D:
            ni, nj = i + di, j + dj
            k2 = nj * w.n + ni
            if 0 <= ni < w.n and 0 <= nj < w.n and w.free[l][k2]:
                soft = w.soft[l].get(k2)
                if soft and not rip:
                    continue
                n = (ni, nj, l)
                ng = g[s] + c * w.grid + (RIP_COST * w.grid if soft else 0)
                if ng < g.get(n, 1e9):
                    g[n], came[n] = ng, s
                    heapq.heappush(heap, (ng + h(ni, nj), n))
        if w.viaok[j * w.n + i] and not any(w.soft[x].get(j * w.n + i) for x in SIG):
            for l2 in SIG:
                if l2 != l and w.free[l2][j * w.n + i]:
                    n = (i, j, l2)
                    ng = g[s] + VIA_COST
                    if ng < g.get(n, 1e9):
                        g[n], came[n] = ng, s
                        heapq.heappush(heap, (ng, n))
    return None


added = []


def lay(w, net, path, end_via):
    """Lay a node path as tracks and vias; returns the board objects created."""
    made = []
    nets = board.FindNet(net)
    def add_track(a, b, layer):
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(pcbnew.VECTOR2I(mm(a[0]), mm(a[1])))
        t.SetEnd(pcbnew.VECTOR2I(mm(b[0]), mm(b[1])))
        t.SetWidth(mm(TRACK_W))
        t.SetLayer(layer)
        t.SetNet(nets)
        board.Add(t)
        added.append(t.m_Uuid.AsString())
        made.append(t)
        tracks.append(dict(net=net, layer=layer, a=a, b=b, hw=TRACK_W / 2, obj=t))
    def add_via(p):
        v = pcbnew.PCB_VIA(board)
        v.SetPosition(pcbnew.VECTOR2I(mm(p[0]), mm(p[1])))
        v.SetWidth(mm(VIA_D))            # (a per-layer SetWidth on a normal padstack zeroes it)
        v.SetDrill(mm(VIA_DRILL))
        v.SetNet(nets)
        board.Add(v)
        added.append(v.m_Uuid.AsString())
        made.append(v)
        vias.append(dict(net=net, c=p, r=VIA_D / 2, drill=VIA_DRILL / 2))
    # merge straight runs; a layer change is a via
    run_start = path[0]
    for k in range(1, len(path)):
        prev, cur = path[k - 1], path[k]
        if cur[2] != prev[2]:
            if run_start[:2] != prev[:2]:
                add_track(w.pt(*run_start[:2]), w.pt(*prev[:2]), prev[2])
            add_via(w.pt(*cur[:2]))
            run_start = cur
            continue
        nxt = path[k + 1] if k + 1 < len(path) else None
        if nxt is None or nxt[2] != cur[2] or (nxt[0] - cur[0], nxt[1] - cur[1]) != (cur[0] - prev[0], cur[1] - prev[1]):
            add_track(w.pt(*run_start[:2]), w.pt(*cur[:2]), cur[2])
            run_start = cur
    if end_via:
        add_via(w.pt(*path[-1][:2]))
    return made


def reroute(t):
    """Reconnect a ripped track's two ends (same net, same layer at the far end)."""
    mid = ((t['a'][0] + t['b'][0]) / 2, (t['a'][1] + t['b'][1]) / 2)
    if math.dist(t['a'], t['b']) > 2 * WIN - 0.5:
        return None
    w = Window(t['net'], mid)
    si, sj = w.idx(t['a'])
    gi, gj = w.idx(t['b'])
    for l in SIG:                       # its own ends are its net's: free them
        w.free[l][sj * w.n + si] = 1
    w.free[t['layer']][gj * w.n + gi] = 1
    path = search(w, [(si, sj, t['layer'])], lambda i, j, l: (i, j, l) == (gi, gj, t['layer']), rip=False)
    if path is None:
        return None
    return lay(w, t['net'], path, False)


def pad_of(net, p):
    for pd in pads:
        if pd['net'] == net and pd['box'][0] - 0.05 <= p[0] <= pd['box'][2] + 0.05 and pd['box'][1] - 0.05 <= p[1] <= pd['box'][3] + 0.05:
            return pd
    return None


fixed = 0
for v in report.get('unconnected_items', []):
    items = v.get('items', [])
    if len(items) != 2:
        continue
    desc = [it.get('description', '') for it in items]
    net = next((d[d.find('[') + 1:d.find(']')] for d in desc if '[' in d), '')
    ends = []
    for it, d in zip(items, desc):
        p = (it['pos']['x'], it['pos']['y'])
        ends.append((d, p, pad_of(net, p)))
    # start from a pad (the stranded end); the other item is the goal
    ends.sort(key=lambda e: e[2] is None)
    (da, pa, pda), (db, pb, pdb) = ends
    if pda is None:
        continue
    # a window round both ends: a long signal gets a coarser grid
    if net in planes and pdb is None:
        w = Window(net, pda['c'])
    else:
        span = max(abs(pa[0] - pb[0]), abs(pa[1] - pb[1])) / 2 + 3.0
        mid = ((pa[0] + pb[0]) / 2, (pa[1] + pb[1]) / 2)
        w = Window(net, mid, span, float(__import__('os').environ.get('ASTAR_GRID', 0)) or (0.1 if span > WIN else GRID))
    starts = []
    for i in range(w.n):
        for j in range(w.n):
            p = w.pt(i, j)
            for l in SIG:
                if l in pda['layers'] and box_pt(pda['box'], p) < 0.01 and w.free[l][j * w.n + i]:
                    starts.append((i, j, l))
    if not starts:
        # the pad's own cells read blocked by a neighbour's clearance: start at its centre
        ci, cj = w.idx(pda['c'])
        starts = [(ci, cj, l) for l in SIG if l in pda['layers']]
        for l in SIG:
            if l in pda['layers']:
                w.free[l][cj * w.n + ci] = 1
    to_plane = net in planes and ('Zone' in db or 'Via' in db or pdb is None or True)
    goal_pad = pdb
    def goal(i, j, l):
        p = w.pt(i, j)
        if goal_pad is not None and l in goal_pad['layers'] and box_pt(goal_pad['box'], p) < 0.01:
            return True
        if 'Via' in db and math.dist(p, pb) < 0.1:
            return True
        if net in planes and w.viaok[j * w.n + i] and math.dist(p, pda['c']) > 0.3:
            return True
        return False
    if goal_pad is not None:
        goal.target = w.idx(goal_pad['c'])
    path = search(w, starts, goal, rip=False) or search(w, starts, goal)
    if path is None:
        sys.stderr.write('%-8s %-12s no path\n' % (net, pda['ref']))
        continue
    end = path[-1]
    p = w.pt(*end[:2])
    end_via = net in planes and not (goal_pad is not None and box_pt(goal_pad['box'], p) < 0.01) and not ('Via' in db and math.dist(p, pb) < 0.1)
    # tracks the path runs through: rip them, lay ours, reroute theirs
    ripped = {tid for (i, j, l) in path for tid in w.soft[l].get(j * w.n + i, ())}
    ripped = [w.by_id[tid] for tid in ripped]
    for t in ripped:
        board.Remove(t['obj'])
        tracks.remove(t)
    mark = len(added)
    made = lay(w, net, path, end_via)
    redone, ok = [], True
    for t in ripped:
        r = reroute(t)
        if r is None:
            ok = False
            break
        redone += r
    if not ok:                          # undo: ours out, theirs back as they were
        for o in made + redone:
            board.Remove(o)
        for t in ripped:
            board.Add(t['obj'])
            tracks.append(t)
        sys.stderr.write('%-8s %-12s path needs a rip-up that cannot be rerouted\n' % (net, pda['ref']))
        continue
    added[mark:] = [' '.join(added[mark:])]        # one fix, one line: DRC keeps it or drops it whole
    fixed += 1
    sys.stderr.write('%-8s %-12s fixed: %d cells, ends (%.2f,%.2f)%s%s\n' % (
        net, pda['ref'], len(path), p[0], p[1], ' + plane via' if end_via else '',
        ', rerouted %s' % ', '.join(sorted({t['net'] for t in ripped})) if ripped else ''))
board.Save(sys.argv[1])
open('added.txt', 'a').write('\n'.join(added) + '\n')
print(fixed)
