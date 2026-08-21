"""Emit the SKIN layer: polymer shells over the bone modules.

The skin is geometry-from-bones: vcad's `[skin r ...]` (rolling-ball closing,
minimum curvature radius r everywhere) wrapped over a named group of bone
plates and actuator envelopes, hollowed with `[shell t]`. The bones stay the
single source of truth - fatten SKIN_R and the whole robot gets friendlier;
that one number is both the aesthetic and the ISO/TS 15066 argument.

Output: pro/cad/skin.loon, a module importing the bone/actuator modules and
rooting one shell per panel per side, material "skin". The panels here are
RENDER geometry first (judge the look, iterate r/gap); print-plate splitting,
standoff bosses and windows harden afterwards, panel by panel.

Requires the vcad branch with [skin]/[shell]; until that merges, render with
that worktree's vcad-render.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rose_pro_links as L
import rose_pro_loon as R

CAD = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'cad')

SKIN_R = 12.0      # rolling-ball radius, mm: no skin curvature tighter than this
SKIN_GAP = 6.0     # stand-off from the bones: foam + compliance travel
SKIN_T = 3.0       # printed wall

# Per-panel radius override. The ball must be BIGGER than any aperture it is
# meant to span, or it rolls through: the thigh flange window is 65 x 48, and a
# 12 mm ball (24 across) tunnelled into it and left a hole in the skin, while
# the same ball fell into the jog step and nearly severed the panel. r >= 24
# spans both. This is the rolling-ball rule stated in reverse, and it is why
# the radius is a per-panel number rather than one global.
PANEL_R = {'thigh': 26.0}

# panel -> (bone links, actuator joints) wrapped together. Split where joints
# move: a panel never spans a joint (the skin would tear the sweep).
PANELS = {
    'thigh':   (('thigh',), ('hip_yaw', 'knee')),
    'calf':    (('shank', 'crankA', 'crankB'), ('ank_mA', 'ank_mB')),
    'foot':    (('foot', 'ankle'), ('ank_roll',)),
    # pelvisS/hipA/hipB excluded: thin plates outside the pod skinned into
    # X-shaped webs at each hip (thin-feature aliasing rendered INTO geometry
    # this time). The pod panels + the two actuator bodies carry the form.
    # actuators excluded as well: the closing BRIDGES the two crossed hip
    # cylinders into X-shaped webs (rolling ball in the crotch of a cross).
    # The pod alone is the skin; the hip cans stay dark - the joint reveal.
    'pelvis':  (('pelvisC',), ()),
    # sho_pitch cans excluded: they stretched the grid past the 40M cap and the
# panel evaluated EMPTY (refused resolution). shoC's own cowl covers them.
# shoC excluded too: a group of ONLY thin tilted plates (no bulk) vanishes in
# the SDF at cell = r/6 (3 mm plate ~ 1.5 cells) - the kernel's documented
# thin-feature limit. The cowl is already a cover; it stays bare sheet.
'chest':   (('trunkC', 'trunkS'), ('waist_yaw',)),
    'upperarm': (('shoA', 'shoB'), ('sho_roll', 'elb_pitch')),
    'forearm': (('elbA',), ('wrist_yaw',)),
    'hand':    (('hand',), ()),
    'head':    (('head', 'neck'), ('head_pitch',)),
}
# centreline links (no mirrored copy)
NO_MIRROR = ('neck', 'trunkC', 'head', 'pelvisC')
# panels that are one centred shell, not an L/R pair
CENTRED = ('pelvis', 'chest', 'head')

# WINDOWS: cuts through the finished shell, per panel, side-aware. Reveals,
# not decoration: each shows the joint or mechanism underneath (rule 3 in the
# skeleton/skin thesis). Coordinates derive from the plate table read above.
def windows(panel, side):
    sgn = -1.0 if side == 'R' else 1.0
    if panel == 'thigh':
        # knee ring: a through-cylinder about the knee axis, r 92 over the D166
        # can - the dark knee disc and the bones behind it read through
        y0, y1 = (60.0, 300.0) if side == 'L' else (-300.0, -60.0)
        return ['[translate -60.0 %s -465.0 [rotate -90.0 0.0 0.0 [cylinder 92.0 %s]]]'
                % (R.fnum(y0), R.fnum(y1 - y0))]
    if panel == 'calf':
        # rear slot: the pushrods and cranks stay visible (they explain)
        return ['[translate -230.0 %s -940.0 [cube 90.0 90.0 330.0]]'
                % R.fnum(sgn * 164.0 - 45.0)]
    if panel == 'chest':
        # the chest window, following the bones' own front aperture
        return ['[translate 40.0 -58.0 215.0 [cube 90.0 116.0 115.0]]']
    if panel == 'head':
        # visor: one wide slot across the stereo pair
        return ['[translate 30.0 -62.0 610.0 [cube 110.0 124.0 38.0]]']
    return []


def names_for(link, side, groups):
    """Root names a link contributes on one side (C links render once)."""
    out = []
    for rn, mirror, _ in R.link_names(link, groups[link], NO_MIRROR):
        tag = 'R' if mirror else ('C' if link in NO_MIRROR else 'L')
        if tag == side or tag == 'C':
            out.append(rn)
    return out


def main():
    data = L.dump_json(os.path.expanduser('~/Developer/pond_assets/rose_pro_plates.json'))
    groups = {}
    for pl in data['plates']:
        groups.setdefault(pl['link'], []).append(pl)

    lines = ['; rose-pro SKIN layer. GENERATED by pro/skin.py - edit the panel table there.',
             '; [skin r :gap g bones...] is the rolling-ball wrap: min curvature radius',
             '; r everywhere, standing g off the bones. r and the cobot contact rule are',
             '; the same number.',
             '[material "skin" 0.82 0.80 0.76 0.35 0.15]',
             '']
    use, roots = {}, []
    for panel, (links, joints) in PANELS.items():
        sides = ('C',) if panel in CENTRED else ('L', 'R')
        for side in sides:
            want = ('L', 'R', 'C') if side == 'C' else (side,)
            members = []
            for lk in links:
                ns = [n for w in want for n in names_for(lk, w, groups)]
                ns = sorted(set(ns))
                members += ns
                use.setdefault(lk, set()).update(ns)
            for jt in joints:
                for an, _ in R.actuator_exprs():
                    sd, j = an.split('_', 1)
                    if j == jt and sd in want:
                        members.append('act_' + an)
                        use.setdefault('__act__', set()).add('act_' + an)
            nm = 'skin_%s_%s' % (side, panel)
            # carry panel/side WITH the root: the emission loop below used to read
            # the loop variables, which by then held the LAST panel - so every
            # shell got the head's visor cut (a no-op off the head) and the knee
            # ring, calf slot and per-panel radius never applied at all.
            roots.append((nm, members, panel, side))

    for lk in sorted(use):
        if lk == '__act__':
            lines.append('[use actuators [%s]]' % ' '.join(sorted(use[lk])))
        else:
            lines.append('[use links.%s [%s]]' % (lk, ' '.join(sorted(use[lk]))))
    lines.append('')
    for nm, members, panel, side in roots:
        # loon has no keyword args: [skin-gap r g #[bones...]] (vcad claude/loon-skin-shell)
        shellx = ('[shell %s [skin-gap %s %s #[%s]]]'
                  % (R.fnum(SKIN_T), R.fnum(PANEL_R.get(panel, SKIN_R)),
                     R.fnum(SKIN_GAP), '\n    '.join(members)))
        cuts = windows(panel, side)
        for c in cuts:
            shellx = '[difference %s\n  %s]' % (c, shellx)
        lines.append('[let %s\n  %s]' % (nm, shellx))
        lines.append('[root %s "skin"]' % nm)

    out = os.path.join(CAD, 'skin.loon')
    open(out, 'w').write('\n'.join(lines) + '\n')
    print('wrote %s  (%d panels)' % (out, len(roots)))

    # the skinned ASSEMBLY: every bone root, every actuator, every skin panel.
    # skin.loon's lets are its exports; its own roots stay there.
    asm = ['; rose-pro, skinned. GENERATED by pro/skin.py.',
           '[material "skin" 0.82 0.80 0.76 0.35 0.15]',
           '[material "actuator" 0.10 0.10 0.11 0.6 0.5]']
    for link in sorted(groups):
        ns = [rn for rn, _, _ in R.link_names(link, groups[link], NO_MIRROR)]
        asm.append('[use links.%s [%s]]' % (link, ' '.join(ns)))
        asm += ['[root %s "aluminum"]' % n for n in ns]
    acts = ['act_' + n for n, _ in R.actuator_exprs()] + \
           ['rod_%s_%s' % (sd, nm) for (nm, x, y, z0, z1) in getattr(L, 'RODS', []) for sd in ('L', 'R')]
    asm.append('[use actuators [%s]]' % ' '.join(acts))
    asm += ['[root %s "actuator"]' % n for n in acts]
    asm.append('[use skin [%s]]' % ' '.join(r[0] for r in roots))
    asm += ['[root %s "skin"]' % r[0] for r in roots]
    out2 = os.path.join(CAD, 'rose-pro-skinned.loon')
    open(out2, 'w').write('\n'.join(asm) + '\n')
    print('wrote %s' % out2)


if __name__ == '__main__':
    main()
