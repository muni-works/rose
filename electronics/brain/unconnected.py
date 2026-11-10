"""Unconnected connections on the board, from KiCad's own connectivity.
kicad-cli's DRC report stops listing near 500, which this board exceeds; the
per-net split below comes from that (capped) list, the total from connectivity.

    python3 unconnected.py brain.kicad_pcb drc.rpt   (KiCad's Python; zones filled by fill_zones.py)
"""
import re, sys
from collections import Counter
import pcbnew

b = pcbnew.LoadBoard(sys.argv[1])
c = b.GetConnectivity()
c.RecalculateRatsnest()
print('unconnected (KiCad connectivity):', c.GetUnconnectedCount(False))
if len(sys.argv) > 2:
    txt = open(sys.argv[2]).read()
    per = Counter(m for blk in txt.split('[unconnected_items]')[1:] for m in re.findall(r'\[([^\]]+)\]', blk.split('\n[')[0])[:1])
    for n, k in per.most_common(12):
        print(f'{k:5d} {n}')
