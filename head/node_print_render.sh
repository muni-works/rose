#!/bin/sh
# Renders of the printable node parts (run node_print.py first) -> renders/node/print-*.png
set -e
R=${VCAD_RENDER:-${CARGO_TARGET_DIR:-$HOME/Developer/vcad/target}/release/vcad-render}
V=${VCAD:-$(dirname "$R")/vcad}
F=$(cd fab/node && pwd)
tmp=$(mktemp -d .renderXXXX)
trap 'rm -rf "$tmp"' EXIT
cp node.loon "$tmp/"
mkdir -p renders/node
while IFS='|' read -r name az el extra; do
  [ -z "$name" ] && continue
  {
    echo '[material "satin_black" 0.09 0.094 0.106 0.1 0.55] [material "petg_grey" 0.42 0.44 0.47 0.1 0.5]'
    echo "[root [import-mesh \"$F/node-rear.stl\" :units \"mm\"] \"satin_black\"]"
    echo "[root [import-mesh \"$F/node-face.stl\" :units \"mm\"] \"petg_grey\"]"
    case "$name" in *fit*)
      echo '[use node [node-battery node-dk-board node-dk-io node-dk-stack node-display node-stereo-board node-stereo-lens-l node-stereo-lens-r node-power node-speakers node-usb-c]]'
      echo '[material "pcb_green" 0.05 0.32 0.18 0.1 0.45] [material "cell_pack" 0.13 0.22 0.42 0.1 0.5] [material "heatsink" 0.55 0.57 0.6 0.7 0.4]'
      for p in battery:cell_pack dk-board:pcb_green dk-io:heatsink dk-stack:heatsink display:heatsink stereo-board:pcb_green stereo-lens-l:heatsink stereo-lens-r:heatsink power:pcb_green speakers:heatsink usb-c:heatsink; do
        echo "[root node-${p%%:*} \"${p##*:}\"]"; done;;
    esac
  } > "$tmp/$name.loon"
  (cd "$tmp" && "$V" export "$name.loon" "$name.vcad" >/dev/null)
  "$R" "$tmp/$name.vcad" $extra --azimuth="$az" --elevation="$el" --size 1000 --fill 0.8 --auto-aspect -o "renders/node/$name.png" >/dev/null
  echo "$name"
done <<'X'
print-34|30|15|--photoreal
print-back|200|20|--photoreal
print-under|30|-40|--style shaded
print-usb|180|10|--photoreal
print-cut|-90|5|--section y=0 --style shaded
print-fit|-60|20|--section y=0 --style shaded
print-fit-ear|-90|5|--section y=40 --style shaded
X
