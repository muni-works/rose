"""Delete the tracks and vias a KiCad DRC JSON report calls dangling.
    drop_dangling.py board.kicad_pcb drc.json   (KiCad's Python)
Removing a leaf never opens a connection; route.sh loops this with a refill
until nothing dangles. Prints how many items went."""
import json, sys
import pcbnew
report = json.load(open(sys.argv[2]))
dead = {i['uuid'] for v in report.get('violations', [])
        if v.get('type') in ('track_dangling', 'via_dangling') for i in v.get('items', [])[:1]}
board = pcbnew.LoadBoard(sys.argv[1])
gone = [t for t in list(board.GetTracks()) if t.m_Uuid.AsString() in dead]
for t in gone:
    board.Remove(t)
board.Save(sys.argv[1])
print(len(gone))
