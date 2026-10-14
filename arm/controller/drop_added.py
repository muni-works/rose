"""Remove copper close_unconnected.py or premium.py added that KiCad's DRC still flags.

    drop_added.py board.kicad_pcb drc.json      (KiCad's Python)

Reads the uuids from added.txt; any of them named in a DRC violation (not an
unconnected-items entry) goes. Prints how many were removed.
"""
import json, sys
import pcbnew

added = {u for u in open('added.txt').read().split() if u}
report = json.load(open(sys.argv[2]))
bad = {i['uuid'] for v in report.get('violations', []) for i in v.get('items', []) if i.get('uuid') in added}
board = pcbnew.LoadBoard(sys.argv[1])
gone = [t for t in list(board.GetTracks()) + list(board.GetDrawings()) if t.m_Uuid.AsString() in bad]
for t in gone:
    board.Remove(t)
board.Save(sys.argv[1])
print(len(gone))
