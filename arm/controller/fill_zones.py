"""Fill every zone with KiCad's own filler and save (kicad-cli 9 DRC does not refill).
Run with KiCad's Python: route.sh does."""
import sys
import pcbnew
board = pcbnew.LoadBoard(sys.argv[1])
pcbnew.ZONE_FILLER(board).Fill(board.Zones())
board.Save(sys.argv[1])
print('filled %d zones' % len(board.Zones()))
