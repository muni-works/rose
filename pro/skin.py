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

# FORM BLOCKS. A bone group is mostly air: the rolling ball dips into every
# lightening aperture and between every pair of flanges, so a hollow group
# skins into a bag rather than a limb. Rather than inflate r (which is the
# SAFETY number and balloons the whole panel), each hollow panel gets a filler
# solid inside its bones - form authored with a primitive, the same idea as a
# subdivision cage, in the vocabulary we have. Numbers are the panel's own
# plate bounding box, inset so the block never pokes through the sheet.
# x0 x1 y0 y1 z0 z1, LEFT side; mirrored in y for the right.
FORM = {
    # The upper arm gets a RAIL, not a collar. The elbow seam is r 75 about the
    # elbow axis, so 72% of the old collar (x -58..50 at z 292..305) lived
    # INSIDE the seam and was cut away by the very next difference - it shaped
    # nothing, and the sweep proxy (which does not clip the parent to its own
    # seams) read it as solid and reported the forearm folding onto it at -108
    # deg. What survives the seam is the rear strip beyond x -49; that is all
    # the rail claims to be, and the gate and the geometry now agree.
    'upperarm': [(-58.0, -50.0, 246.0, 302.0, 292.0, 305.0)],  # shoB rear channel
    # The forearm block is TAPERED, in two steps, because the forearm is.
    # elbA is 72 mm deep (y 219..291) only below z 112; above that it is just
    # the two side walls, y 219..252. A single block y 228..286 all the way up
    # therefore stood 34 mm PROUD of the sheet exactly where the forearm folds
    # past the upper arm's front flange - and that overhang, not the sheet, was
    # what the gate kept reporting (bone -4.1, then +3.0 once the block was
    # chopped to z 130, which fixed the symptom by throwing the form away).
    # Stepped to follow the bones, the block reaches z 175 - filling the shell
    # it is there to fill - and clears the upper arm by +7.8 mm at 175 deg of
    # flexion, well outside anything the joint can command.
    'forearm':  [(-38.0, 38.0, 228.0, 286.0, 6.0, 112.0),      # elbA shell, full depth
                 (-38.0, 38.0, 228.0, 244.0, 112.0, 175.0)],   # between the side walls only
    # bottom raised to z 200: at 120 the block's corner (r 189 about the waist
    # axis) swung out over the 194-wide pelvis at 57 deg of yaw. The waist is
    # where a person is narrow for the same reason.
    'chest':    [(-100.0, 96.0, -150.0, 150.0, 200.0, 516.0),  # torso box, inside the walls
                 (-28.0, 58.0, 158.0, 206.0, 430.0, 512.0)],   # shoulder cap, per side
    # stops 60 mm below the wrist axis: a 180 mm mitt swung 138 deg reaches the
    # forearm shell, and the block was what reached first.
    'hand':     [(-40.0, 72.0, 232.0, 280.0, -180.0, -62.0)],
}


def form_exprs(panel, side):
    out = []
    for (x0, x1, y0, y1, z0, z1) in FORM.get(panel, ()):
        for sd in (('L', 'R') if side == 'C' else (side,)):
            a, b = (y0, y1) if sd == 'L' else (-y1, -y0)
            out.append('[translate %s %s %s [cube %s %s %s]]'
                       % (R.fnum(x0), R.fnum(a), R.fnum(z0),
                          R.fnum(x1 - x0), R.fnum(b - a), R.fnum(z1 - z0)))
    return out

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
'chest':   (('trunkC', 'trunkS', 'shoC'), ('waist_yaw',)),
    # shoA and shoB straddle the shoulder ROLL joint - one panel over both would
    # tear when the arm abducts.
    #
    # And shoA gets NO PANEL. Measured: 102 mm between the shoulder pitch and
    # roll axes, against 114 mm of seam the two joints need (each seam is the
    # actuator radius + standoff + air). The link is shorter than its own
    # clearances, so no shell fits there at any thickness - the shoulder
    # mechanism stays an exposed joint cluster, which is at least honest about
    # what it is. The upper arm clears by 21 mm and is the next tightest.
    # Everything below the elbow and everything in the legs has 140-390 mm of
    # slack: this is an ARM problem, not a skin problem.
    'upperarm': (('shoB',), ('elb_pitch',)),
    'forearm': (('elbA',), ('wrist_yaw',)),
    'hand':    (('hand',), ()),
    'head':    (('head', 'neck'), ('head_pitch',)),
}
# centreline links (no mirrored copy)
NO_MIRROR = ('neck', 'trunkC', 'head', 'pelvisC')
# panels that are one centred shell, not an L/R pair
CENTRED = ('pelvis', 'chest', 'head')

# SEAMS. A joint is a GAP in the skin, not a butt joint: both panels either
# side are cut back by a cylinder about the joint's own axis, so the gap is
# invariant under the rotation - it cannot close at any angle in the range.
# That is what skin_sweep.py was built to prove, and what it found missing:
# adjacent panels overlapped at 8 of 9 joints, by 12 mm where the bones merely
# touch (both skins grow SKIN_GAP toward each other) and by 40+ where my form
# blocks crossed. Radius = the joint body's own radius + SKIN_GAP + 6 mm of
# air, so the actuator ring shows in the gap - the reveal and the clearance
# are the same feature.
SEAM = {                       # joint -> (radius, half-length along the axis)
    'knee':       (130.0, 200.0),
    # 75, not 54: the elbow stator and output plates are a pancake pair whose
    # CORNERS sit at r 55, just outside a 54 seam - so both stayed skinned and
    # overlapped at EVERY angle, not just in flexion. A seam has to clear the
    # plate corners at a joint, not just the actuator can.
    'elb_pitch':  (75.0, 120.0),
    'wrist_yaw':  (58.0, 120.0),   # covers the wrist pancake pair, same reason
    # 56, not 54: at 54 the upper arm's rear web survived down to z 351 and met
    # the trunk's shoulder-stator plate at +1.1 mm - the one place the skin was
    # buying clearance with joint range (abduction was capped 1.7 -> 1.55 rad
    # for it). 2 mm more seam takes that band of web out and the pair clears by
    # +3.0 mm at the FULL 1.7 rad. The skin now costs no joint any range at all.
    'sho_roll':   (56.0, 120.0),
    'sho_pitch':  (60.0, 200.0),
    'hip_pitch':  (150.0, 200.0),
    # 110, not 60: the chest floor and the pelvis top are a 200-wide pancake
    # pair that TOUCHES by design, and a 60 seam left both of them skinned.
    # The wide gap is the exposed waist the H2 has, arrived at by the gate.
    'waist_yaw':  (125.0, 200.0),
    'head_pitch': (42.0, 120.0),
    'ank_pitch':  (85.0, 150.0),   # 62 left the foot and calf shells meeting at -52 deg   # the pushrod ankle's pin: small joint, wide yoke
}

# Per-(joint, panel) seam radius. A seam only has to be a cylinder ABOUT the
# joint axis to keep the gap invariant under the rotation - it does not have to
# be the SAME cylinder on both sides. The elbow needs that: the upper arm's
# surviving band is a pair of cheeks reaching r 95 about the elbow axis (they
# have to, the link is only 30 mm of skin tall between its two seams), so it is
# the FOREARM that gets cut back instead. 95 buys the whole flexion range; a
# symmetric 95 would have erased the upper-arm panel entirely.
SEAM_R = {('elb_pitch', 'forearm'): 95.0}


def seam_r(joint, panel):
    return SEAM_R.get((joint, panel), SEAM[joint][0])


# which joints cut which panels (both sides of every joint)
SEAM_PANELS = {
    'knee':       ('thigh', 'calf'),
    'elb_pitch':  ('upperarm', 'forearm'),
    'wrist_yaw':  ('forearm', 'hand'),
    'sho_roll':   ('upperarm',),
    'sho_pitch':  ('chest',),
    'hip_pitch':  ('pelvis', 'thigh'),
    'waist_yaw':  ('chest', 'pelvis'),
    'head_pitch': ('head', 'chest'),
    'ank_pitch':  ('calf', 'foot'),
}


def seam_exprs(panel, side):
    out = []
    for joint, panels in SEAM_PANELS.items():
        if panel not in panels:
            continue
        r, half = seam_r(joint, panel), SEAM[joint][1]
        J = L.JOINTS[joint]
        for sd in (('L', 'R') if side == 'C' else (side,)):
            c = list(J['c'])
            if sd == 'R':
                c[1] = -c[1]
            if J['c'][1] == 0.0 and side != 'C' :
                continue                      # centreline joint, only cuts C panels
            ax = J['ax']
            if ax == 'y':
                e = ('[translate %s %s %s [rotate -90.0 0.0 0.0 [cylinder %s %s]]]'
                     % (R.fnum(c[0]), R.fnum(c[1] - half), R.fnum(c[2]),
                        R.fnum(r), R.fnum(2 * half)))
            elif ax == 'x':
                e = ('[translate %s %s %s [rotate 0.0 90.0 0.0 [cylinder %s %s]]]'
                     % (R.fnum(c[0] - half), R.fnum(c[1]), R.fnum(c[2]),
                        R.fnum(r), R.fnum(2 * half)))
            else:
                e = ('[translate %s %s %s [cylinder %s %s]]'
                     % (R.fnum(c[0]), R.fnum(c[1]), R.fnum(c[2] - half),
                        R.fnum(r), R.fnum(2 * half)))
            out.append(e)
    return out


# WINDOWS: cuts through the finished shell, per panel, side-aware. Reveals,
# not decoration: each shows the joint or mechanism underneath (rule 3 in the
# skeleton/skin thesis). Coordinates derive from the plate table read above.
def windows(panel, side):
    sgn = -1.0 if side == 'R' else 1.0
    # the knee ring is now the knee SEAM (SEAM/SEAM_PANELS above): the reveal
    # and the clearance gap are the same feature, which is the point.
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
            members += form_exprs(panel, side)
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
        cuts = windows(panel, side) + seam_exprs(panel, side)
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
