"""Move a net from one pad of a part to another on the routed board (a pin swap).

    pin_swap.py board.kicad_pcb REF FROM_PAD TO_PAD      (KiCad's Python)

The loon must make the same change, so the schematic and netlist agree.
"""
import sys
import pcbnew

path, ref, a, b = sys.argv[1:5]
board = pcbnew.LoadBoard(path)
fp = board.FindFootprintByReference(ref)
pa = [p for p in fp.Pads() if p.GetNumber() == a]
pb = [p for p in fp.Pads() if p.GetNumber() == b]
net = pa[0].GetNet()
for p in pb:
    p.SetNet(net)
for p in pa:
    p.SetNetCode(0)
board.Save(path)
print('%s: %s moved from pad %s to pad %s' % (ref, net.GetNetname(), a, b))
