#!/bin/sh
# Hero + exploded renders of the printed node (run node_print.py and node_fit.py first) -> renders/node/hero*.png
set -e
R=${VCAD_RENDER:-${CARGO_TARGET_DIR:-$HOME/Developer/vcad/target}/release/vcad-render}
V=${VCAD:-$(dirname "$R")/vcad}
F=$(cd fab/node && pwd)
tmp=$(mktemp -d .renderXXXX)
trap 'rm -rf "$tmp"' EXIT
cp node.loon "$tmp/"
mkdir -p renders/node
USE='[use node [node-glass node-face node-lenses node-puck on-desk node-battery node-dk-board node-dk-io node-dk-stack node-display node-stereo-board node-stereo-lens-l node-stereo-lens-r node-power node-speakers node-usb-c]]'
MAT='[material "satin_black" 0.035 0.037 0.042 0.05 0.6] [material "petg_grey" 0.42 0.44 0.47 0.1 0.5] [material "bone" 0.78 0.76 0.71 0.0 0.55]
[material "glass" 0.01 0.012 0.014 0.4 0.04] [material "face" 0.72 0.95 0.76 0.0 0.5] [material "lens" 0.035 0.04 0.045 0.3 0.08]
[material "puck" 0.62 0.6 0.56 0.0 0.65] [material "pcb_green" 0.05 0.32 0.18 0.1 0.45] [material "cell_pack" 0.13 0.22 0.42 0.1 0.5]
[material "heatsink" 0.55 0.57 0.6 0.7 0.4]'
TUB="[import-mesh \"$F/node-rear.stl\" :units \"mm\"]"
FLR="[import-mesh \"$F/node-face.stl\" :units \"mm\"]"

# hero: the printed head on its desk puck, screen on
{
  echo "$USE"; echo "$MAT"
  echo "[root [on-desk $TUB] \"${BODY:-satin_black}\"] [root [on-desk $FLR] \"satin_black\"]"
  echo '[root [on-desk node-glass] "glass"] [root [on-desk node-face] "face"] [root [on-desk node-lenses] "lens"]'
  echo '[root node-puck "puck"]'
} > "$tmp/hero.loon"

# exploded: the face module pulled forward off the rear shell, the devkit slid half out the front
E='[let fw [fn [x s] [translate x 0.0 0.0 s]]]'
{
  echo "$USE"; echo "$MAT"; echo "$E"
  echo "[root $TUB \"${BODY:-satin_black}\"] [root node-battery \"cell_pack\"]"
  echo '[root [fw 60.0 node-dk-board] "pcb_green"] [root [fw 60.0 node-dk-io] "heatsink"] [root [fw 60.0 node-dk-stack] "heatsink"]'
  echo "[root [fw 130.0 $FLR] \"satin_black\"] [root [fw 130.0 node-glass] \"glass\"] [root [fw 130.0 node-face] \"face\"] [root [fw 130.0 node-display] \"heatsink\"]"
} > "$tmp/exploded.loon"

for n in hero exploded; do (cd "$tmp" && "$V" export "$n.loon" "$n.vcad" >/dev/null); done
"$R" "$tmp/hero.vcad" --photoreal --azimuth=28 --elevation=9 --size 1600 --fill 0.72 --auto-aspect -o renders/node/hero${SUFFIX}.png >/dev/null
"$R" "$tmp/exploded.vcad" --photoreal --azimuth=40 --elevation=18 --size 1600 --fill 0.85 --auto-aspect -o renders/node/hero-exploded.png >/dev/null
"$R" "$tmp/exploded.vcad" --style shaded --azimuth=40 --elevation=18 --size 1600 --fill 0.85 --auto-aspect -o renders/node/hero-exploded-line.png >/dev/null
echo done
