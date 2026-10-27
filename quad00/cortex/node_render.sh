#!/bin/sh
# Massing renders of node.loon -> renders/node/*.png. Run from this folder: sh node_render.sh
set -e
R=${VCAD_RENDER:-${CARGO_TARGET_DIR:-$HOME/Developer/vcad/target}/release/vcad-render}
V=${VCAD:-$(dirname "$R")/vcad}
mkdir -p renders/node
tmp=$(mktemp -d .renderXXXX)
trap 'rm -rf "$tmp"' EXIT
cp node.loon "$tmp/"
HEAD='node-head:satin_black node-lenses:lens node-glass:glass node-face:face node-ears:dots node-intake:dots'
# name | wrap (desk|neck|none) | azimuth | elevation
while IFS='|' read -r name wrap az el; do
  [ -z "$name" ] && continue
  {
    echo "[use node [node-head node-ears node-intake node-dockboard node-battery node-carrier node-module node-heatsink node-display node-cam-l node-cam-r node-power node-speakers node-floor node-lenses node-glass node-face node-puck node-neck node-torso on-desk]]"
    sed -n '/^\[material/p' node.loon
    for p in $HEAD; do
      n=${p%%:*}; m=${p##*:}
      [ "$wrap" = desk ] && n="[on-desk $n]"
      echo "[root $n \"$m\"]"
    done
    [ "$wrap" = desk ] && echo '[root node-puck "puck"]'
    [ "$wrap" = neck ] && echo '[root node-neck "body"] [root node-torso "body"]'
    if [ "$wrap" = cut ]; then
      for p in floor:aluminium dockboard:pcb_green battery:cell_pack carrier:pcb_green module:pcb_green heatsink:heatsink display:heatsink cam-l:pcb_green cam-r:pcb_green power:pcb_green speakers:heatsink; do
        echo "[root node-${p%%:*} \"${p##*:}\"]"
      done
      echo '[material "pcb_green" 0.05 0.32 0.18 0.1 0.45] [material "cell_pack" 0.13 0.22 0.42 0.1 0.5] [material "heatsink" 0.55 0.57 0.6 0.7 0.4] [material "aluminium" 0.75 0.76 0.78 0.9 0.3]'
    fi
  } > "$tmp/$name.loon"
  (cd "$tmp" && "$V" export "$name.loon" "$name.vcad" >/dev/null)
  sec="--photoreal"; [ "$wrap" = cut ] && sec="--section ${SECTION:-y=0} --style shaded"
  "$R" "$tmp/$name.vcad" $sec --azimuth="$az" --elevation="$el" --size 1000 --fill 0.8 --auto-aspect -o "renders/node/$name.png" >/dev/null
  echo "$name"
done <<'X'
desk-34|desk|30|12
desk-side|desk|90|4
desk-front|desk|0|8
human-34|neck|30|10
human-side|neck|90|2
back-34|none|210|15
cut-side|cut|-90|5
cut-34|cut|-60|25
X
