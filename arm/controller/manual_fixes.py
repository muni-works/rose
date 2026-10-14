"""Hand fixes on the routed board - the record of what was done by hand, re-applied
after every route (route.sh runs it before the final fill and DRC).

    manual_fixes.py board.kicad_pcb        (KiCad's Python)

Each fix names the gap the router left and what closes it. A fix whose
copper is already on the board is skipped, so the script can run twice.
Coordinates are mm on the v0.2 rectangle board (150 x 40); after a placement
change, re-check them against drc.rpt.
"""
import sys
import pcbnew

VIA_D, VIA_DRILL = 0.45, 0.25

# (net, [("via", x, y) | ("track", layer, x0, y0, x1, y1, width)], why)
FIXES = [
    ("+12V", [("via", 105.21, 7.615), ("via", 101.01, 11.115),
              ("track", "In2.Cu", 105.21, 7.615, 101.01, 11.115, 0.5)],
     "the In2 run under L1 had no via at either end (and was 0.15 mm on a 1 A rail)"),
]

mm = pcbnew.FromMM
board = pcbnew.LoadBoard(sys.argv[1])
have_vias = {(round(pcbnew.ToMM(t.GetPosition().x), 2), round(pcbnew.ToMM(t.GetPosition().y), 2))
             for t in board.GetTracks() if t.Type() == pcbnew.PCB_VIA_T}
done = 0
for net, items, why in FIXES:
    n = board.FindNet(net)
    if n is None:
        continue
    for it in items:
        if it[0] == "via":
            _, x, y = it
            if (round(x, 2), round(y, 2)) in have_vias:
                continue
            v = pcbnew.PCB_VIA(board)
            v.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
            v.SetWidth(mm(VIA_D))
            v.SetDrill(mm(VIA_DRILL))
            v.SetNet(n)
            board.Add(v)
        else:
            _, layer, x0, y0, x1, y1, w = it
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(pcbnew.VECTOR2I(mm(x0), mm(y0)))
            t.SetEnd(pcbnew.VECTOR2I(mm(x1), mm(y1)))
            t.SetWidth(mm(w))
            t.SetLayer(board.GetLayerID(layer))
            t.SetNet(n)
            board.Add(t)
    done += 1
    print('%-8s %s' % (net, why))
board.Save(sys.argv[1])
print(done)
