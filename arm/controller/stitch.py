"""Ground stitching: a via fence round the board edge plus a grid in open ground.

    stitch.py board.kicad_pcb        (KiCad's Python, on a board whose zones are filled)

A candidate becomes a GND through via only where it lands wholly inside the
filled GND pour on both outer layers (so the fill already cleared it from
every other net there), keeps clear of every track on the inner signal
layers, and keeps hole spacing from every via and pad. Refill afterwards.
Prints the count added.
"""
import math, sys
import pcbnew

FENCE_PITCH, FENCE_INSET = 2.0, 1.9      # mm: just inside the gold rim (art edge ring)
GRID_PITCH = 4.0                         # mm
VIA_D, VIA_DRILL = 0.45, 0.25            # mm, the board's default via
CLEAR, HOLE_GAP = 0.15, 0.25             # mm

mm = pcbnew.FromMM
board = pcbnew.LoadBoard(sys.argv[1])
gnd = board.FindNet('GND')
outer = [pcbnew.F_Cu, pcbnew.B_Cu]
fills = [z for z in board.Zones() if z.GetNetname() == 'GND' and z.GetLayer() in outer]
inner_signal = [l for l in (pcbnew.In2_Cu, pcbnew.In4_Cu)]
plane_layers = {z.GetLayer() for z in board.Zones() if not z.GetLayer() in outer}
tracks = [t for t in board.GetTracks() if t.Type() == pcbnew.PCB_TRACE_T and t.GetLayer() in inner_signal]
holes = [(v.GetPosition(), v.GetDrillValue()) for v in board.GetTracks() if v.Type() == pcbnew.PCB_VIA_T]
holes += [(p.GetPosition(), max(p.GetDrillSize().x, p.GetDrillSize().y))
          for f in board.GetFootprints() for p in f.Pads() if p.GetDrillSize().x > 0]
bb = board.GetBoardEdgesBoundingBox()
x0, y0, x1, y1 = [pcbnew.ToMM(v) for v in (bb.GetLeft(), bb.GetTop(), bb.GetRight(), bb.GetBottom())]


def in_fill(x, y):
    r = VIA_D / 2 + 0.02
    for layer in outer:
        zs = [z for z in fills if z.GetLayer() == layer]
        for k in range(8):
            a = 2 * math.pi * k / 8
            p = pcbnew.VECTOR2I(mm(x + r * math.cos(a)), mm(y + r * math.sin(a)))
            if not any(z.HitTestFilledArea(layer, p, 0) for z in zs):
                return False
        if not any(z.HitTestFilledArea(layer, pcbnew.VECTOR2I(mm(x), mm(y)), 0) for z in zs):
            return False
    return True


def seg_dist(p, a, b):
    ax, ay, bx, by = a.x, a.y, b.x, b.y
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    t = 0 if l2 == 0 else max(0, min(1, ((p.x - ax) * dx + (p.y - ay) * dy) / l2))
    return math.hypot(ax + t * dx - p.x, ay + t * dy - p.y)


def clear_inner(x, y):
    p = pcbnew.VECTOR2I(mm(x), mm(y))
    for t in tracks:
        if seg_dist(p, t.GetStart(), t.GetEnd()) < mm(VIA_D / 2 + CLEAR) + t.GetWidth() / 2:
            return False
    return True


def clear_holes(x, y, extra):
    for (c, d) in holes + extra:
        if math.hypot(pcbnew.ToMM(c.x) - x, pcbnew.ToMM(c.y) - y) < VIA_DRILL / 2 + pcbnew.ToMM(d) / 2 + HOLE_GAP:
            return False
    return True


cands = []
# the fence follows the board's own outline (Edge.Cuts), FENCE_INSET inside it:
# offset each edge both ways and let in_fill() keep the side that is on the board
for d in board.GetDrawings():
    if d.GetLayer() != pcbnew.Edge_Cuts or d.GetShape() != pcbnew.SHAPE_T_SEGMENT:
        continue
    a, b = d.GetStart(), d.GetEnd()
    ax, ay, bx, by = pcbnew.ToMM(a.x), pcbnew.ToMM(a.y), pcbnew.ToMM(b.x), pcbnew.ToMM(b.y)
    L = math.hypot(bx - ax, by - ay)
    if L < 1e-6:
        continue
    nx, ny = -(by - ay) / L, (bx - ax) / L
    for k in range(int(L / FENCE_PITCH) + 1):
        t = (k + 0.5) * FENCE_PITCH / L
        if t > 1:
            break
        px, py = ax + (bx - ax) * t, ay + (by - ay) * t
        cands += [(px + nx * FENCE_INSET, py + ny * FENCE_INSET), (px - nx * FENCE_INSET, py - ny * FENCE_INSET)]
gx = x0 + GRID_PITCH
while gx < x1 - GRID_PITCH / 2:
    gy = y0 + GRID_PITCH
    while gy < y1 - GRID_PITCH / 2:
        cands.append((gx, gy))
        gy += GRID_PITCH
    gx += GRID_PITCH

# rule areas that forbid vias (a copper logo, the gold rim): no stitch lands there
no_via = [z for z in board.Zones() if z.GetIsRuleArea() and z.GetDoNotAllowVias()]


def in_keepout(x, y):
    p = pcbnew.VECTOR2I(mm(x), mm(y))
    return any(z.Outline().Contains(p) for z in no_via)


added = []
for (x, y) in cands:
    if not in_keepout(x, y) and in_fill(x, y) and clear_inner(x, y) and clear_holes(x, y, [(a, mm(VIA_DRILL)) for a in added]):
        v = pcbnew.PCB_VIA(board)
        v.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
        v.SetWidth(mm(VIA_D))
        v.SetDrill(mm(VIA_DRILL))
        v.SetNet(gnd)
        board.Add(v)
        added.append(v.GetPosition())
board.Save(sys.argv[1])
print(len(added))
