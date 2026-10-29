#!/bin/sh
# Preview renders of the printed cortex parts -> renders/print/*.png. Run from this folder: sh print_render.sh
# (VCAD_RENDER=path/to/vcad-render to override). Model frame, +Z up = the build direction for the tub, floor and
# exhaust grille; the sled and ear grilles are rotated in the slicer (see ../fab/cortex/README.md).
set -e
R=${VCAD_RENDER:-${CARGO_TARGET_DIR:-$HOME/Developer/vcad/target}/release/vcad-render}
mkdir -p renders/print
tmp=$(mktemp -d .renderXXXX)
trap 'rm -rf "$tmp"' EXIT
cp cortex.loon "$tmp/"
# name | parts (with material) | azimuth | elevation
while IFS='|' read -r name parts az el; do
  [ -z "$name" ] && continue
  {
    echo "[use cortex [$(echo "$parts" | sed 's/:[a-z_]*//g')]]"
    echo '[material "satin_black" 0.09 0.094 0.106 0.1 0.55] [material "petg_grey" 0.42 0.44 0.47 0.1 0.5]'
    echo '[material "pcb_green" 0.05 0.32 0.18 0.1 0.45] [material "cell_pack" 0.13 0.22 0.42 0.1 0.5]'
    for p in $parts; do echo "[root ${p%%:*} \"${p##*:}\"]"; done
  } > "$tmp/$name.loon"
  "$R" "$tmp/$name.loon" --azimuth "$az" --elevation="$el" --size 1200 --fill 0.9 --auto-aspect -o "renders/print/$name.png" >/dev/null 2>&1
  echo "$name"
done <<'EOF'
tub|cortex-tub:satin_black|35|25
tub-inside|cortex-tub:satin_black|215|-40
floor-top|cortex-floor:petg_grey|35|40
floor-bottom|cortex-floor:petg_grey|35|-45
sled-print|cortex-sled-print:petg_grey|35|25
sled-on-floor|cortex-floor:petg_grey cortex-sled-print:satin_black cortex-battery:cell_pack cortex-compute:pcb_green|35|25
ear-grilles|cortex-ear-grilles:satin_black|60|15
exhaust-grille|cortex-exhaust-grille:satin_black|200|20
back-grille|cortex-back-grille:satin_black|205|10
d4-face|cortex-tub-d4:satin_black d4-glass:visor_glass|20|15
d4-inside|cortex-tub-d4:satin_black d4-glass:visor_glass d4-body:pcb_green d4-cams:pcb_green|200|-35
EOF
