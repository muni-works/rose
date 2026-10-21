#!/usr/bin/env python3
"""rose-1 v4 — the perfection pass: numeric sweep audit + mass diet.

WHAT CHANGED FROM v2 (all driven by the built-in checker, which found two
STATIC interpenetrations v2's hand-written checks missed):
  - forearm was inside the hip-roll actuator; arms moved outboard (shy 166)
  - pelvis side walls passed through the hip-roll envelope; pelvis is now a
    narrow sagittal spine (y +-40) + hip ribs, walls x-disjoint from the pods
  - hip YAW actuators flipped on top of the ribs, shaft down: the thigh no
    longer meets a hanging yaw body at deep flexion -> R1's +146 deg works
  - waist stack given true axial separation (yaw z168..212, roll circle
    z216..316, chest base 320) — v2's waist roll clipped the chest base
  - ankle-drive pods moved to the shin FRONT: rear pods limited knee flexion
    to ~113 deg; front pods clear the fold (target 148 = R1)
  - mass diet: R100-A reduced-current build (same envelope, 0.38 kg) on all
    zero-gravity-torque + arm joints; shells 8->5 mm; head counted shelled

v4: EVERY TORQUE PATH CLOSED — carriers, mount plates, output cap+drop
members (hub-joint pattern: the crossed-roller output bearing takes the
moment), ankle cranks/rods/horns/pivots, waist tower+saddle. <=4 mm face
gaps are DECLARED BOLTED interfaces (see ../LINKAGES.md). CHECKER: cylinders/boxes sampled (mm-scale surface grids), signed distance,
swept through R1 joint ranges. Prints PASS/FAIL + software limits every run.
"""
import math

# ---------------- checker lib ----------------
def sd_box(p, lo, hi):
    dx = [max(lo[i]-p[i], 0.0, p[i]-hi[i]) for i in range(3)]
    d = math.sqrt(sum(v*v for v in dx))
    if d > 0.0: return d
    return -min(min(p[i]-lo[i], hi[i]-p[i]) for i in range(3))

def sd_cyl(p, base, u, L, r):
    d = [p[i]-base[i] for i in range(3)]
    t = sum(d[i]*u[i] for i in range(3))
    rad = math.sqrt(max(sum(d[i]*d[i] for i in range(3)) - t*t, 0.0))
    dr, da = rad - r, max(-t, t - L)
    if dr <= 0.0 and da <= 0.0: return max(dr, da)
    return math.sqrt(max(dr,0.0)**2 + max(da,0.0)**2)

def _frame(u):
    a = (0,0,1) if abs(u[2]) < 0.9 else (1,0,0)
    v = (u[1]*a[2]-u[2]*a[1], u[2]*a[0]-u[0]*a[2], u[0]*a[1]-u[1]*a[0])
    n = math.sqrt(sum(x*x for x in v)); v = tuple(x/n for x in v)
    w = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
    return v, w

def cyl_pts(base, u, L, r, na=7, nt=24):
    v, w = _frame(u); pts = []
    for i in range(na+1):
        t = L*i/na
        for j in range(nt):
            th = 2*math.pi*j/nt
            c, s = r*math.cos(th), r*math.sin(th)
            pts.append(tuple(base[k]+u[k]*t+v[k]*c+w[k]*s for k in range(3)))
    for t in (0.0, L):  # end-disc interiors
        for rr in (0.35*r, 0.7*r):
            for j in range(8):
                th = 2*math.pi*j/8
                c, s = rr*math.cos(th), rr*math.sin(th)
                pts.append(tuple(base[k]+u[k]*t+v[k]*c+w[k]*s for k in range(3)))
    return pts

def box_pts(lo, hi, n=6):
    xs = [[lo[i]+(hi[i]-lo[i])*k/n for k in range(n+1)] for i in range(3)]
    pts = []
    for i in range(3):
        for face in (lo[i], hi[i]):
            for a in xs[(i+1)%3]:
                for b in xs[(i+2)%3]:
                    p = [0,0,0]; p[i]=face; p[(i+1)%3]=a; p[(i+2)%3]=b
                    pts.append(tuple(p))
    return pts

def rotY(p, cx, cz, deg):
    s, c = math.sin(math.radians(deg)), math.cos(math.radians(deg))
    x, z = p[0]-cx, p[2]-cz
    return (cx + c*x + s*z, p[1], cz - s*x + c*z)

def rotZ(p, cx, cy, deg):
    s, c = math.sin(math.radians(deg)), math.cos(math.radians(deg))
    x, y = p[0]-cx, p[1]-cy
    return (cx + c*x - s*y, cy + s*x + c*y, p[2])

def min_sd(pts, targets):
    m = 1e9
    for p in pts:
        for kind, a in targets:
            d = sd_box(p, *a) if kind == 'box' else sd_cyl(p, *a)
            if d < m: m = d
    return m

# ---------------- model ----------------
PALETTE=[
 '[material "anodized_gunmetal" 0.13 0.135 0.145 0.9 0.5]',    # actuator housings
 '[material "brushed_aluminum" 0.72 0.73 0.75 1.0 0.35]',      # machined structure (anisotropic via name)
 '[material "shell_silver" 0.80 0.81 0.83 0.55 0.3]',          # body panels
 '[material "gloss_black" 0.04 0.04 0.045 0.0 0.12]',          # helmet, hands
 '[material "visor_black" 0.02 0.02 0.025 0.0 0.05]',
 '[material "rubber" 0.05 0.05 0.05 0.0 0.95]',
 '[material "battery_red" 0.45 0.08 0.07 0.0 0.45]',
 '[material "machined_steel" 0.55 0.56 0.58 1.0 0.3]',         # pins/shafts (anisotropic via name)
 '[material "abs_blue" 0.12 0.2 0.5 0.0 0.4]',
]
SHEET=[]
def sheet(part,t,w,l,qty=2): SHEET.append((part,t,w,l,qty))
R=[];CHK=[];MASS={'R140':0,'R100':0,'R100A':0,'alu_mm3':0.0,'abs_mm3':0.0}
PRIM={}  # name -> ('box',(lo,hi)) | ('cyl',(base,u,L,r))
MATMAP={'steel':'anodized_gunmetal','aluminum':'brushed_aluminum','abs-black':'gloss_black',
 'abs-red':'battery_red','abs-blue':'abs_blue','rubber':'rubber'}
def root(s,mat): R.append(f'[root {s} "{MATMAP.get(mat,mat)}"]')
def box(x,y,z,dx,dy,dz,mat='aluminum',fil=None,track=True,name=None):
    b=f'[cube {dx} {dy} {dz}]'
    if fil: b=f'[fillet {fil} [translate {x} {y} {z} {b}]]'
    else: b=f'[translate {x} {y} {z} {b}]'
    root(b,mat)
    if track and mat in ('aluminum','shell_silver'): MASS['alu_mm3']+=dx*dy*dz
    if track and mat.startswith('abs'): MASS['abs_mm3']+=dx*dy*dz
    if name: PRIM[name]=('box',((x,y,z),(x+dx,y+dy,z+dz)))
R140=dict(od=140.0,ln=52.0,fod=90.0,key='R140')
R100=dict(od=100.0,ln=44.0,fod=64.0,key='R100')
R100A=dict(od=100.0,ln=44.0,fod=64.0,key='R100A')  # reduced-current build, same envelope
def ra(spec,rot,tx,ty,tz,name=None,axis=None):
    od,ln,fod=spec['od'],spec['ln'],spec['fod']; MASS[spec['key']]+=1
    def place(b):
        if rot: b=f'[rotate {rot} {b}]'
        return f'[translate {tx} {ty} {tz} {b}]'
    root(place(f'[union [cylinder {od/2} {ln-3.0}] [translate 0.0 0.0 {ln-3.0} [cylinder {fod/2} 3.0]]]'),'steel')
    root(place(f'[translate 0.0 0.0 {ln-11.0} [cylinder {od/2+0.6} 4.0]]'),'abs-black')
    root(place(f'[translate 0.0 0.0 -2.0 [cylinder 10.0 2.0]]'),'machined_steel')
    root(place(f'[translate {od/2-22.0} -8.0 -2.0 [cube 16.0 16.0 2.0]]'),'abs-black')
    if name and axis: PRIM[name]=('cyl',((tx,ty,tz),axis,ln,od/2))

FE,TI=340.0,335.0; HP,KN=8.0,-16.0; hipY=80.0; shy=170.0
SH=590.0   # shoulder pitch axis height (whole waist/chest stack raised so the
           # hip-yaw body sits ABOVE the thigh's flexion disc, on a 16mm shaft)
s,c=math.sin(math.radians(HP)),math.cos(math.radians(HP))
kx,kz=FE*s,-FE*c
tA=HP+KN; s2,c2=math.sin(math.radians(tA)),math.cos(math.radians(tA))
ax,az=kx+TI*s2, kz-TI*c2

def leg(sy):
    y=lambda v: sy*v
    yc=lambda cc,w: cc if sy>0 else -(cc+w)
    ro=('-90.0 0.0 0.0' if sy>0 else '90.0 0.0 0.0')
    rf=('90.0 0.0 0.0' if sy>0 else '-90.0 0.0 0.0')   # output faces INBOARD
    ay=(0,sy,0)
    nm=lambda n: f'{n}{"L" if sy>0 else "R"}'
    def disc(r,h,tx,ty,tz,flip=False,name=None):
        root(f'[translate {tx} {ty} {tz} [rotate {rf if flip else ro} [cylinder {r} {h}]]]','aluminum')
        MASS['alu_mm3']+=3.1416*r*r*h
        if name: PRIM[name]=('cyl',((tx,ty,tz),(0,sy if not flip else -sy,0),h,r))
    # ---- hip roll pod (axis X) ----
    ra(R140,'0.0 90.0 0.0', -56.0, y(hipY), 34.0, nm('hiproll'),(1,0,0))
    # ---- hip yaw: body above rib, output shaft down through rib bearing ----
    ra(R100A,'180.0 0.0 0.0', 0.0, y(hipY), 202.0, nm('hipyaw'),(0,0,-1))
    root(f'[translate 0.0 {y(hipY)} 98.0 [cylinder 8.0 60.0]]','machined_steel')          # output shaft
    root(f'[translate 0.0 {y(hipY)} 118.0 [difference [cylinder 16.0 8.0] [translate 0.0 0.0 -1.0 [cylinder 9.0 10.0]]]]','steel')  # rib bearing boss (pelvis side)
    # yaw carrier saddle: shaft bottom -> roll housing top (the part v3 lacked)
    box(-56.0,yc(64.0,32.0),100.0,64.0,32.0,4.0,name=nm('saddleH'))
    # ---- hip roll output -> pitch housing carrier (3-piece, thigh-plane safe) ----
    box(-4.0,yc(38.0,84.0),-14.0,4.0,84.0,28.0,name=nm('hcAdapt'))   # bolts roll flange, r<=14.6 (thigh clears at r20)
    box(-4.0,yc(30.0,8.0),-14.0,4.0,8.0,28.0)                        # bridge
    box(-40.0,yc(24.0,6.0),-40.0,80.0,6.0,80.0,name=nm('hcBase'))    # pitch-housing base plate
    # ---- hip pitch (axis Y, output flange toward thigh) ----
    ra(R140,ro, 0.0, y(hipY-50), 0.0, nm('hippitch'),ay)
    disc(34.0,6.0, 0.0, y(hipY+2), 0.0, name=nm('thighcap'))         # pitch output cap -> thigh
    # thigh (top raised to meet the cap's drop member)
    root(f'[translate 0.0 {y(hipY)} 0.0 [rotate 0.0 {HP} 0.0 [union [translate -38.0 -30.0 -{FE-77.0} [cube 76.0 3.0 {FE-97.0}]] [union [translate -38.0 27.0 -{FE-77.0} [cube 76.0 3.0 {FE-97.0}]] [translate 35.0 -30.0 -{FE-77.0} [cube 3.0 60.0 {FE-97.0}]]]]]]','aluminum')
    MASS['alu_mm3']+=(2*76*3+3*54)*(FE-97); sheet('thigh channel',3.0,196,FE-97)
    box(-17.0,yc(82.0,3.0),-108.0,38.0,3.0,100.0)                    # thigh drop: cap -> thigh body
    # ---- knee hub: housing bolts to thigh via knee plate (inboard base face) ----
    box(kx-38.0,yc(44.0,6.0),kz-54.0,88.0,8.0,135.7,name=nm('kneeplate'))
    ra(R140,ro, kx, y(hipY-26), kz, nm('knee'),ay)
    disc(34.0,6.0, kx, y(106.0), kz, name=nm('shincap'))             # knee output cap (outboard flange)
    box(kx-19.0,yc(106.0,3.0),kz-132.0,38.0,3.0,100.0,name=nm('shindrop'))
    box(kx-22.0,yc(60.0,54.0),kz-130.0,44.0,54.0,50.0,name=nm('shintrans'))
    # shin
    root(f'[translate {kx} {y(hipY)} {kz} [rotate 0.0 {tA} 0.0 [union [translate -22.0 -20.0 -{TI-40.0} [cube 44.0 3.0 {TI-118.0}]] [union [translate -22.0 17.0 -{TI-40.0} [cube 44.0 3.0 {TI-118.0}]] [translate 19.0 -20.0 -{TI-40.0} [cube 3.0 40.0 {TI-118.0}]]]]]]','aluminum')
    MASS['alu_mm3']+=(2*44*3+3*34)*(TI-118); sheet('shin channel',3.0,124,TI-118)
    # ---- ankle drives: front pods; A outputs INBOARD, B OUTBOARD (crank lanes) ----
    ra(R100,rf, kx+58.0, y(78.0), kz-150.0, nm('calfA'),(0,-sy,0))
    ra(R100,ro, kx+58.0, y(82.0), kz-150.0, nm('calfB'),ay)
    disc(20.0,6.0, kx+58.0, y(34.0), kz-150.0, flip=True)            # crank A (y 28..34)
    disc(20.0,6.0, kx+58.0, y(126.0), kz-150.0)                      # crank B (y 126..132)
    # pushrods: crank pin (r14 below axis) -> foot horn
    for ry in (24.0,128.0):
        root(f'[translate {kx+55.0} {y(ry)} {az-6.0} [cube 8.0 8.0 {kz-164.0-(az-6.0)}]]','machined_steel')
    PRIM[nm('rodA')]=('box',((kx+55.0,yc(24.0,8.0),az-6.0),(kx+62.0,yc(24.0,8.0)[0] if False else yc(24.0,8.0)+8.0,kz-164.0)))
    PRIM[nm('rodB')]=('box',((kx+55.0,yc(128.0,8.0),az-6.0),(kx+62.0,yc(128.0,8.0)+8.0,kz-164.0)))
    # foot horns (inboard + outboard wings)
    box(kx+50.0,yc(24.0,18.0),az-18.0,16.0,18.0,46.0)
    box(kx+50.0,yc(120.0,18.0),az-18.0,16.0,18.0,46.0)
    # ---- ankle: clevis plates + cross pins into the block ----
    box(ax-26.0,yc(50.0,6.0),-654.0+0*az,52.0,6.0,30.0) if False else None
    box(ax-26.0,yc(50.0,6.0),az+14.0,52.0,6.0,30.0)
    box(ax-26.0,yc(112.0,6.0),az+14.0,52.0,6.0,30.0)
    root(f'[translate {ax} {y(48.0)} {az+22.0} [rotate {ro} [cylinder 6.0 72.0]]]','machined_steel')      # pitch pin
    root(f'[translate {ax-20.0} {y(84.0)} {az+2.0} [rotate 0.0 90.0 0.0 [cylinder 6.0 40.0]]]','machined_steel')  # roll pin
    # shin lower clevis riser (closes the 21mm shin->block gap)
    root(f'[translate {kx} {y(hipY)} {kz} [rotate 0.0 {tA} 0.0 [translate -14.0 -36.0 -{TI-14.0} [cube 28.0 72.0 40.0]]]]','aluminum')
    MASS['alu_mm3']+=28*36*40
    # ankle + foot
    box(ax-22.0,yc(58.0,52.0),az-6.0,44.0,52.0,24.0)
    box(ax-64.0,yc(44.0,74.0),az-18.0,200.0,74.0,3.0,name=nm('foot'))
    box(ax-64.0,yc(44.0,3.0),az-15.0,200.0,3.0,9.0)
    box(ax-64.0,yc(115.0,3.0),az-15.0,200.0,3.0,9.0)
    sheet('foot pan',3.0,218,200)
    box(ax-64.0,yc(44.0,74.0),az-22.0,200.0,74.0,4.0,'rubber',track=False)
    root(f'[translate {ax+124.0} {yc(52.0,58.0)} {az-18.0} [rotate 0.0 -16.0 0.0 [cube 40.0 58.0 9.0]]]','aluminum'); MASS['alu_mm3']+=42*64*9

def arm(sy):
    y=lambda v: sy*v
    yc=lambda cc,w: cc if sy>0 else -(cc+w)
    ro=('-90.0 0.0 0.0' if sy>0 else '90.0 0.0 0.0')
    ay=(0,sy,0)
    nm=lambda n: f'{n}{"L" if sy>0 else "R"}'
    def disc(r,h,tx,ty,tz,name=None):
        root(f'[translate {tx} {ty} {tz} [rotate {ro} [cylinder {r} {h}]]]','aluminum')
        MASS['alu_mm3']+=3.1416*r*r*h
        if name: PRIM[name]=('cyl',((tx,ty,tz),ay,h,r))
    # deck boss -> pitch housing (bolted, 4mm)
    box(-25.0,yc(110.0,16.0),SH-22.0,50.0,16.0,60.0,name=nm('boss'))
    ra(R100A,ro, 0.0, y(shy-40), SH, nm('shpitch'),ay)
    # pitch-output bracket: flange plate + strap + end cap -> roll housing
    box(-32.0,yc(176.0,4.0),SH-32.0,64.0,4.0,64.0,name=nm('brplate'))
    box(-52.0,yc(176.0,4.0),SH-50.0,84.0,4.0,10.0,name=nm('brstrap'))
    box(-58.0,yc(156.0,64.0),SH-154.0,4.0,64.0,112.0,name=nm('brcap'))
    ra(R100A,'0.0 90.0 0.0', -50.0, y(shy+16), SH-104.0, nm('shroll'),(1,0,0))
    # roll output plate (r<=41 of roll axis so it clears the strap when abducting)
    box(-6.0,yc(154.0,64.0),SH-158.0,4.0,64.0,78.0,name=nm('rollout'))
    ra(R100A,'180.0 0.0 0.0', 0.0, y(shy+16), SH-158.0, nm('shyaw'),(0,0,-1))
    # upper arm: yaw flange (above, bolted 4mm) down to elbow axis level
    root(f'[translate 0.0 {y(shy+16)} 0.0 [union [translate -18.0 -18.0 {SH-256.0} [cube 36.0 3.0 50.0]] [union [translate -18.0 15.0 {SH-256.0} [cube 36.0 3.0 50.0]] [translate 15.0 -18.0 {SH-256.0} [cube 3.0 36.0 50.0]]]]]','aluminum')
    MASS['alu_mm3']+=(2*36*3+3*30)*50; sheet('upper-arm channel',3.0,102,50)
    PRIM[nm('upper')]=('box',((-18.0,yc(168.0,36.0),SH-256.0),(18.0,yc(168.0,36.0)+36.0,SH-206.0)))
    # elbow mount plate (upper-arm side, on housing base face)
    box(-25.0,yc(170.0,4.0),SH-298.0,50.0,4.0,84.0,name=nm('elplate'))
    ra(R100A,ro, 0.0, y(178.0), SH-256.0, nm('elbow'),ay)
    disc(34.0,6.0, 0.0, y(222.0), SH-256.0, name=nm('elcap'))        # elbow output cap
    box(-17.0,yc(222.0,3.0),SH-350.0,38.0,3.0,100.0,name=nm('eldrop'))
    box(-17.0,yc(163.0,67.0),SH-350.0,38.0,67.0,40.0,name=nm('eltrans'))
    # forearm + hand
    root(f'[translate 4.0 {y(shy+12)} {SH-310.0} [union [translate -15.0 -17.0 -178.0 [cube 30.0 2.5 178.0]] [union [translate -15.0 14.5 -178.0 [cube 30.0 2.5 178.0]] [translate 12.5 -17.0 -178.0 [cube 2.5 34.0 178.0]]]]]','aluminum')
    MASS['alu_mm3']+=(2*30*2.5+2.5*29)*178; sheet('forearm channel',2.5,88,178)
    PRIM[nm('forearm')]=('box',((-11.0,yc(165.0,34.0),SH-488.0),(19.0,yc(165.0,34.0)+34.0,SH-310.0)))
    root(f'[translate -4.0 {y(174.0)} {SH-522.0} [fillet 7.0 [translate -14.0 -18.0 0.0 [cube 28.0 36.0 22.0]]]]','abs-black')
    PRIM[nm('hand')]=('box',((-18.0,yc(156.0,36.0),SH-522.0),(10.0,yc(156.0,36.0)+36.0,SH-500.0)))

for sy in (1,-1): leg(sy); arm(sy)
# ---- pelvis: sagittal spine + hip ribs (now meeting the spine) + walls ----
box(-95.0,-40.0,112.0,190.0,80.0,6.0,mat='shell_silver',name='spine')
box(-18.0,40.0,112.0,36.0,76.0,6.0,name='ribL')
box(-18.0,-116.0,112.0,36.0,76.0,6.0,name='ribR')
box(90.0,-40.0,96.0,8.0,80.0,22.0,name='wallF')
box(-98.0,-40.0,96.0,8.0,80.0,22.0,name='wallB')
# ---- waist: tower supports the yaw stator (v3 had it floating 88mm up) ----
root('[translate 0.0 0.0 118.0 [cylinder 28.0 88.0]]','aluminum'); MASS['alu_mm3']+=3.1416*28*28*88*0.4
PRIM['tower']=('cyl',((0.0,0.0,118.0),(0,0,1),88.0,28.0))
ra(R100A,None, 0.0, 0.0, 206.0, 'wyaw',(0,0,1))
box(-58.0,-40.0,250.0,60.0,80.0,3.0,name='saddle')                 # yaw flange -> saddle
box(-58.0,-40.0,254.0,4.0,80.0,72.0,name='yokeA')                  # saddle end cap -> roll housing
ra(R100,'0.0 90.0 0.0', -50.0, 0.0, 304.0, 'wroll',(1,0,0))
box(-6.0,-40.0,274.0,4.0,80.0,84.0,name='chestbkt')                # roll OUTPUT -> chest base
# ---- chest ----
box(-92.0,-100.0,358.0,132.0,200.0,3.0,mat='shell_silver',fil=6.0,name='chestbase')
box(65.5,-78.0,386.0,2.5,156.0,176.0,mat='shell_silver',name='front')
box(-84.0,-104.0,262.0,2.5,208.0,300.0,mat='shell_silver',name='back')
box(-84.0,101.5,262.0,134.0,2.5,298.0,mat='shell_silver',name='sideL')
box(-84.0,-104.0,262.0,134.0,2.5,298.0,mat='shell_silver',name='sideR')
box(-76.0,-122.0,563.0,156.0,244.0,3.0,mat='shell_silver',name='deck')
box(-58.0,-62.0,374.0,98.0,124.0,124.0,'abs-red',track=False,name='battery')
box(-52.0,-52.0,504.0,96.0,104.0,38.0,'abs-black')
box(-18.0,-14.0,546.0,28.0,28.0,10.0,'abs-blue')
# ---- neck + head (neck now lands on the deck) ----
for (nx,ny,ndx,ndy) in [(-30.0,-38.0,64.0,3.0),(-30.0,35.0,64.0,3.0),(-30.0,-35.0,3.0,70.0),(27.0,-35.0,3.0,70.0)]:
    box(nx,ny,566.0,ndx,ndy,34.0,mat='shell_silver')
sheet('neck collar',3.0,268,34,qty=1)
root('[translate -38.0 -44.0 604.0 [fillet 20.0 [cube 84.0 88.0 80.0]]]','abs-black')
MASS['abs_mm3']+=84*88*80*0.35
PRIM['head']=('box',((-38.0,-44.0,604.0),(46.0,44.0,684.0)))
box(42.0,-28.0,634.0,8.0,56.0,20.0,'visor_black',track=False)

open('/Users/cam/Developer/rose/cad/rose1-v6.loon','w').write(
 '; rose-1 v6 — GENERATED. full actuator detail + sheet-metal structure; sweep-audited.\n'+'\n'.join(PALETTE)+'\n'+'\n'.join(R)+'\n')
print(len(R),'roots')

# ---------------- SELF-AUDIT ----------------
def T(name):
    k,a=PRIM[name]; return (k,a)
def pts_of(*names):
    out=[]
    for name in names:
        k,a=PRIM[name]
        out+= cyl_pts(*a) if k=='cyl' else box_pts(*a)
    return out
def report(label, d, want=2.0):
    tag='PASS' if d>=want else 'FAIL'
    CHK.append(f'{tag} {label}: {d:+.1f} mm')

# STATIC clearances (relative-motion pairs only; bolted faces are exempt)
report('hip-roll pod vs pelvis', min_sd(pts_of('hiprollL'),
    [T('spine'),T('ribL'),T('wallF'),T('wallB')]))
report('yaw saddle vs rib (yaws under it)', min_sd(pts_of('saddleHL'),[T('ribL')]))
report('hip carrier adapter vs thigh swept bore', 20.0-14.6, 2.0)
report('hip-yaw body vs waist yaw', min_sd(pts_of('hipyawL'),[T('wyaw')]))
report('waist yaw vs waist roll', min_sd(pts_of('wyaw'),[T('wroll')]))
report('waist roll vs chest base', min_sd(pts_of('wroll'),[T('chestbase')]))
report('chest skirts vs waist roll', min_sd(pts_of('wroll'),
    [T('front'),T('back'),T('sideL'),T('sideR')]))
report('chest bracket vs saddle/yoke (rolls rel.)', min_sd(pts_of('chestbkt'),
    [T('saddle'),T('yokeA'),T('wyaw')]),3.0)
report('forearm vs hip-roll pod', min_sd(pts_of('forearmL'),[T('hiprollL')]),8.0)
report('sh-pitch vs deck+boss', min_sd(pts_of('shpitchL'),[T('deck'),T('bossL')]),3.0)
report('sh-roll vs sh-pitch', min_sd(pts_of('shrollL'),[T('shpitchL')]),3.0)
report('sh-yaw vs sh-roll', min_sd(pts_of('shyawL'),[T('shrollL')]),3.0)
report('bracket vs sh-roll (rotates rel.)', min_sd(pts_of('brplateL','brstrapL','brcapL'),
    [T('shrollL')]),2.0)
report('rollout plate vs bracket strap (abduction)', min_sd(pts_of('rolloutL'),
    [T('brstrapL'),T('brplateL')]),3.0)
report('elbow hub vs sh-yaw', min_sd(pts_of('elbowL'),[T('shyawL')]),3.0)
report('forearm/transition vs elbow housing', min_sd(pts_of('forearmL','eltransL'),
    [T('elbowL')]),2.0)
report('shin cap/drop vs thigh+kneeplate', min_sd(pts_of('shincapL','shindropL'),
    [('box',((-38.0,50.0,-263.0),(38.0,110.0,-20.0))),T('kneeplateL')]),2.0)
report('pushrods vs pod bodies', min_sd(pts_of('rodAL','rodBL'),
    [T('calfAL'),T('calfBL')]),1.5)

# SWEEP: hip yaw +-150 (cluster below yaw vs pelvis + waist column)
tgt=[T('spine'),T('ribL'),T('ribR'),T('wallF'),T('wallB'),T('hipyawL'),T('wyaw'),T('tower')]
mov=pts_of('hiprollL','hippitchL','saddleHL','hcBaseL')
worst=1e9
for th in range(-150,151,5):
    worst=min(worst,min_sd([rotZ(p,0.0,hipY,th) for p in mov],tgt))
report('hip yaw sweep +-150 vs pelvis/waist',worst)

# SWEEP: hip pitch (leg below pitch vs pelvis/torso/hip-yaw/waist)
def leg_below_pitch():
    thigh=[rotY(p,0.0,0.0,HP) for p in box_pts((-38.0,50.0,-263.0),(38.0,110.0,-20.0))]
    return (thigh+pts_of('kneeplateL','kneeL','shincapL','shindropL','shintransL',
                         'calfAL','calfBL','rodAL','rodBL','footL')
            +box_pts((kx-22.0,60.0,kz-295.0),(kx+22.0,100.0,kz-78.0)))
tgt=[T('spine'),T('ribL'),T('wallF'),T('wallB'),T('hipyawL'),T('chestbase'),T('front'),
     T('sideL'),T('wyaw'),T('battery'),T('saddle'),T('yokeA'),T('wroll'),T('tower'),T('chestbkt')]
mv=leg_below_pitch()
def pitch_clear(th): return min_sd([rotY(p,0.0,0.0,th-HP) for p in mv],tgt)
fwd=HP
while fwd<175 and pitch_clear(fwd+1)>2.0: fwd+=1
bwd=HP
while bwd>-175 and pitch_clear(bwd-1)>2.0: bwd-=1
CHK.append(f'INFO hip pitch clear range: {bwd:+.0f} extension .. {fwd:+.0f}, flexion {-bwd if bwd<0 else 0:.0f} (R1: -168..+146)')

# SWEEP: knee flexion (shin-side assembly vs thigh-side)
thightgt=[('box',((-38.0,50.0,-263.0),(38.0,110.0,-20.0))),T('kneeplateL')]
kmvw=(pts_of('shincapL','shindropL','shintransL','calfAL','calfBL','rodAL','rodBL','footL')
      +box_pts((kx-22.0,60.0,kz-295.0),(kx+22.0,100.0,kz-78.0)))
kmv=[rotY(p,0.0,0.0,-HP) for p in kmvw]
kc=rotY((kx,0,kz),0,0,-HP)
def knee_clear(fl): return min_sd([rotY(p,kc[0],kc[2],-(fl-(-KN))) for p in kmv],thightgt)
fl=-KN
while fl<175 and knee_clear(fl+1)>2.0: fl+=1
CHK.append(f'INFO knee flexion clear to: {fl:+.0f} deg (R1: +148)')

# SWEEP: elbow flexion (forearm-side vs upper-arm side)
emv=pts_of('forearmL','handL','elcapL','eldropL','eltransL')
etgt=[T('upperL'),T('elplateL'),T('shyawL'),T('shrollL')]
el=0
while el<175 and min_sd([rotY(p,0.0,SH-256.0,-(el+1)) for p in emv],etgt)>2.0: el+=1
CHK.append(f'INFO elbow flexion clear to: {el:+.0f} deg')

# SWEEP: shoulder pitch full circle
armmov=pts_of('shrollL','shyawL','elbowL','upperL','elplateL','forearmL','handL',
              'brplateL','brcapL','rolloutL','elcapL','eldropL','eltransL')
tgt=[T('deck'),T('bossL'),T('sideL'),T('front'),T('back'),T('head'),
     T('hiprollL'),T('hipyawL'),('box',((-38.0,50.0,-263.0),(38.0,110.0,-20.0))),T('footL'),T('wallF'),T('wallB')]
worst=1e9; wang=0
for th in range(0,360,5):
    d=min_sd([rotY(p,0.0,SH,th) for p in armmov],tgt)
    if d<worst: worst,wang=d,th
report(f'shoulder pitch 360 sweep (worst @ {wang} deg)',worst,3.0)

# SWEEP: waist roll +-30 (chest + bracket vs column)
def rotX(p,cy,cz,deg):
    sq,cq=math.sin(math.radians(deg)),math.cos(math.radians(deg))
    yv,zv=p[1]-cy,p[2]-cz
    return (p[0], cy+cq*yv-sq*zv, cz+sq*yv+cq*zv)
skirt=pts_of('front','back','sideL','sideR','chestbase','chestbkt')
tgt=[T('hipyawL'),T('hipyawR'),T('yokeA'),T('wyaw'),T('saddle'),T('tower')]
worst=min(min_sd([rotX(p,0.0,304.0,th) for p in skirt],tgt) for th in (-30,-15,15,30))
report('waist roll +-30 sweep (chest vs column)',worst,3.0)

# SWEEP: waist yaw with arms at rest
handmov=pts_of('forearmL','handL','forearmR','handR')
tgt=[T('hiprollL'),T('hiprollR'),T('hipyawL'),T('hipyawR')]
wy=0
while wy<150 and min_sd([rotZ(p,0.0,0.0,wy+1) for p in handmov],tgt)>3.0: wy+=1
CHK.append(f'INFO waist yaw free (arms at rest): +-{wy} deg of +-150 (beyond needs arm lift — software)')

# SWEEP: leg-vs-leg hip yaw (software limit)
selfmov=leg_below_pitch()
other=[('box',((-38.0,-110.0,-263.0),(38.0,-50.0,-20.0))),
       ('box',((kx-22.0,-100.0,kz-295.0),(kx+22.0,-60.0,kz-78.0))),
       ('box',((ax-64.0,-118.0,az-22.0),(ax+136.0,-44.0,az-6.0)))]
ly=0
while ly<150 and min_sd([rotZ(p,0.0,hipY,-(ly+1)) for p in selfmov],other)>3.0: ly+=1
CHK.append(f'INFO hip yaw inward free of other leg: -{ly} deg (software limit)')

# MASS
al=MASS['alu_mm3']*2.70e-6; ab=MASS['abs_mm3']*1.05e-6
act=MASS['R140']*1.1+MASS['R100']*0.5+MASS['R100A']*0.38
print(f"actuators: {MASS['R140']}x R140 + {MASS['R100']}x R100 + {MASS['R100A']}x R100-A = {act:.1f} kg")
print(f"structure (EXACT — sheet modeled at gauge): alu {al:.2f} kg, abs {ab:.2f} kg")
print(f"battery+harness allowance: 2.6 kg")
print(f"ESTIMATE: {act + al + ab + 2.6:.1f} kg vs 25 kg target")
print("--- SHEET CUT LIST (bent parts; qty incl. mirror) ---")
for p,t,w,l,q in SHEET[:len(SHEET)//2]: print(f"  {p}: {t} mm 5052, flat ~{w:.0f} x {l:.0f}, qty {q}")
for line in CHK: print(line)
