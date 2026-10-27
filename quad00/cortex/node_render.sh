#!/bin/sh
# Massing renders of node.loon -> renders/node/*.png. Run from this folder: sh node_render.sh
set -e
R=${VCAD_RENDER:-${CARGO_TARGET_DIR:-$HOME/Developer/vcad/target}/release/vcad-render}
V=${VCAD:-$(dirname "$R")/vcad}
mkdir -p renders/node
tmp=$(mktemp -d .renderXXXX)
trap 'rm -rf "$tmp"' EXIT
cp node.loon "$tmp/"
HEAD='node-shell:moss_black node-lenses:lens node-glass:glass node-face:face'
# name | wrap (desk|neck|none) | azimuth | elevation
while IFS='|' read -r name wrap az el; do
  [ -z "$name" ] && continue
  {
    echo "[use node [node-shell node-lenses node-glass node-face node-puck node-neck node-torso on-desk]]"
    sed -n '/^\[material/p' node.loon
    for p in $HEAD; do
      n=${p%%:*}; m=${p##*:}
      [ "$wrap" = desk ] && n="[on-desk $n]"
      echo "[root $n \"$m\"]"
    done
    [ "$wrap" = desk ] && echo '[root node-puck "puck"]'
    [ "$wrap" = neck ] && echo '[root node-neck "body"] [root node-torso "body"]'
  } > "$tmp/$name.loon"
  (cd "$tmp" && "$V" export "$name.loon" "$name.vcad" >/dev/null)
  "$R" "$tmp/$name.vcad" --azimuth "$az" --elevation="$el" --size 1000 --fill 0.8 --auto-aspect --photoreal -o "renders/node/$name.png" >/dev/null
  echo "$name"
done <<'X'
desk-34|desk|30|12
desk-side|desk|90|4
desk-front|desk|0|8
human-34|neck|30|10
human-side|neck|90|2
back-34|none|210|15
X
