"""Finishing pass on a routed board: a clean front and teardrops.

    premium.py board.kicad_pcb        (KiCad's Python; route.sh runs it before the fill)

- Front designators move to the fab layer (the assembly drawing keeps them);
  the front silk is left to the logo and part outlines.
- Teardrops: where an outer-layer trace leaves a via, a filled wedge of the
  same net eases the joint (KiCad's own teardrop tool isn't reachable from
  Python). Their uuids go to added.txt, so drop_added.py removes any that DRC flags.
Prints "<designators moved> <teardrops added>".
"""
import math, sys
import pcbnew

mm, to_mm = pcbnew.FromMM, pcbnew.ToMM
board = pcbnew.LoadBoard(sys.argv[1])

moved = 0
for f in board.GetFootprints():
    ref = f.Reference()
    if ref.GetLayer() == pcbnew.F_SilkS:
        ref.SetLayer(pcbnew.F_Fab)
        moved += 1

vias = [v for v in board.GetTracks() if v.Type() == pcbnew.PCB_VIA_T]
by_pos = {}
for v in vias:
    p = v.GetPosition()
    by_pos[(p.x, p.y)] = v
added, drops = [], 0
for t in list(board.GetTracks()):
    if t.Type() != pcbnew.PCB_TRACE_T or t.GetLayer() not in (pcbnew.F_Cu, pcbnew.B_Cu):
        continue
    for end, other in ((t.GetStart(), t.GetEnd()), (t.GetEnd(), t.GetStart())):
        v = by_pos.get((end.x, end.y))
        if v is None or v.GetNetCode() != t.GetNetCode():
            continue
        vx, vy = to_mm(end.x), to_mm(end.y)
        ox, oy = to_mm(other.x), to_mm(other.y)
        L = math.hypot(ox - vx, oy - vy)
        r = to_mm(v.GetWidth(t.GetLayer())) / 2
        if L < r * 2.2:
            continue
        ux, uy = (ox - vx) / L, (oy - vy) / L
        nx, ny = -uy, ux
        hw = to_mm(t.GetWidth()) / 2
        length = min(r * 2.0, L * 0.6)
        base = r * 0.85
        # wedge: across the via at its centre, tapering to the trace width
        pts = [(vx + nx * base, vy + ny * base), (vx + ux * length + nx * hw, vy + uy * length + ny * hw),
               (vx + ux * length - nx * hw, vy + uy * length - ny * hw), (vx - nx * base, vy - ny * base)]
        s = pcbnew.PCB_SHAPE(board)
        s.SetShape(pcbnew.SHAPE_T_POLY)
        s.SetPolyPoints([pcbnew.VECTOR2I(mm(x), mm(y)) for x, y in pts])
        s.SetFilled(True)
        s.SetWidth(0)
        s.SetLayer(t.GetLayer())
        s.SetNet(t.GetNet())
        board.Add(s)
        added.append(s.m_Uuid.AsString())
        drops += 1
board.Save(sys.argv[1])
open('added.txt', 'a').write('\n'.join(added) + '\n')
print(moved, drops)
