#!/bin/sh
# Full-head mockups: each full-*.loon here is a size.loon for cortex.loon. Renders face 3/4 + front of each, plus a
# lineup of every option next to the mini at one scale. Run from this folder: sh render.sh
# (VCAD / VCAD_RENDER to override the binaries)
set -e
T=${CARGO_TARGET_DIR:-$HOME/Developer/vcad/target}/release
V=${VCAD:-$T/vcad}
R=${VCAD_RENDER:-$T/vcad-render}
here=$(pwd)
tmp=$(mktemp -d "${TMPDIR:-/tmp}/cortex-mock.XXXX")
trap 'rm -rf "$tmp"' EXIT
mats='[material "satin_black" 0.09 0.094 0.106 0.1 0.55] [material "visor_glass" 0.02 0.02 0.025 0.2 0.05] [material "pcb_green" 0.05 0.32 0.18 0.1 0.45]'
mkdir -p "$tmp/lineup"
y=0
for opt in mini above beside corner; do
  d="$tmp/$opt"; mkdir -p "$d"; cp ../cortex.loon "$d/"
  if [ "$opt" = mini ]; then cp ../size.loon "$d/size.loon"; tub=cortex-tub; face="cortex-glass:visor_glass"
  else cp "full-$opt.loon" "$d/size.loon"; tub=cortex-tub-full; face="d7-glass:visor_glass d7-cams:pcb_green"; fi
  {
    echo "[use cortex [$tub $(echo "$face" | sed 's/:[a-z_]*//g')]]"; echo "$mats"
    echo "[root $tub \"satin_black\"]"
    for p in $face; do echo "[root ${p%%:*} \"${p##*:}\"]"; done
  } > "$d/m.loon"
  "$R" "$d/m.loon" --azimuth 30 --elevation 12 --size 1200 --fill 0.9 --auto-aspect -o "$here/$opt-34.png" >/dev/null 2>&1
  "$R" "$d/m.loon" --azimuth 0 --elevation 4 --size 1200 --fill 0.9 --auto-aspect -o "$here/$opt-front.png" >/dev/null 2>&1
  "$V" export "$d/m.loon" "$tmp/lineup/$opt.stl" >/dev/null 2>&1      # tub + face parts, one mesh
  echo "$opt"
done
# lineup at one scale, 330 mm apart along y (left to right: mini, above, beside, corner)
{
  echo "$mats"
  y=-495
  for opt in mini above beside corner; do
    echo "[root [translate 0.0 $y.0 0.0 [import-mesh-scaled 0.001 0.001 0.001 \"$opt.stl\"]] \"satin_black\"]"; y=$((y + 330))
  done
} > "$tmp/lineup/lineup.loon"
"$R" "$tmp/lineup/lineup.loon" --azimuth 20 --elevation 10 --size 1600 --fill 0.92 --auto-aspect -o "$here/lineup.png" >/dev/null 2>&1
echo lineup
