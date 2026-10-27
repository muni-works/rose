#!/bin/sh
# Renders of the printable node parts (run node_print.py first) -> renders/node/print-*.png
set -e
R=${VCAD_RENDER:-${CARGO_TARGET_DIR:-$HOME/Developer/vcad/target}/release/vcad-render}
V=${VCAD:-$(dirname "$R")/vcad}
F=$(cd ../fab/node && pwd)
tmp=$(mktemp -d .renderXXXX)
trap 'rm -rf "$tmp"' EXIT
mkdir -p renders/node
while IFS='|' read -r name az el extra; do
  [ -z "$name" ] && continue
  {
    echo '[material "satin_black" 0.09 0.094 0.106 0.1 0.55] [material "petg_grey" 0.42 0.44 0.47 0.1 0.5]'
    echo "[root [import-mesh \"$F/node-tub.stl\" :units \"mm\"] \"satin_black\"]"
    echo "[root [import-mesh \"$F/node-floor.stl\" :units \"mm\"] \"petg_grey\"]"
  } > "$tmp/$name.loon"
  (cd "$tmp" && "$V" export "$name.loon" "$name.vcad" >/dev/null)
  "$R" "$tmp/$name.vcad" $extra --azimuth="$az" --elevation="$el" --size 1000 --fill 0.8 --auto-aspect -o "renders/node/$name.png" >/dev/null
  echo "$name"
done <<'X'
print-34|30|15|--photoreal
print-back|200|20|--photoreal
print-under|30|-40|--style shaded
print-cut|-90|5|--section y=0 --style shaded
X
