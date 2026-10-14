"""Remove copper close_unconnected.py or premium.py added that KiCad's DRC still flags.

    drop_added.py board.kicad_pcb drc.json      (KiCad's Python)

Reads the uuids from added.txt; any of them named in a DRC violation (not an
unconnected-items entry) goes. Prints how many were removed.
"""
import json, sys
import pcbnew

groups = [line.split() for line in open('added.txt').read().splitlines() if line.strip()]
added = {u for g in groups for u in g}
report = json.load(open(sys.argv[2]))
flagged = {i['uuid'] for v in report.get('violations', []) for i in v.get('items', []) if i.get('uuid') in added}
# a fix written on one line (astar_fix.py) goes whole: half a route is worse than none
bad = set()
for g in groups:
    if len(g) > 1 and flagged & set(g):
        bad |= set(g)
    else:
        bad |= flagged & set(g)
board = pcbnew.LoadBoard(sys.argv[1])
gone = [t for t in list(board.GetTracks()) + list(board.GetDrawings()) if t.m_Uuid.AsString() in bad]
for t in gone:
    board.Remove(t)
board.Save(sys.argv[1])
print(len(gone))
