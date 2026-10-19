"""Joukowsky Dynamics — miniature flying prototype (about 1:11.5), sized around four 90 mm EDF units.

Run:  python build_mini.py
Writes advanced/data/mini/{parts.json,meshes.bin}, cad/mini/step/*.step, cad/mini/stl/*.stl and a BOM.

The mini keeps the full-size layout (same lofted body, same 90° pod swing about a hinge above the tyre) but uses
off-the-shelf hobby parts: fixed-pitch EDFs instead of variable-pitch proprotors, hobby servos for the conversion
hinges, fixed landing legs, and an ArduPilot flight controller. The wheel pods are about 2.5× deeper than scale so
the EDF fits inside; that is the main geometric compromise.
"""
import json, math, os, sys
import cadquery as cq
from cadquery import Vector as V
import jdcad
from jdcad import *

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
OUT_WEB = os.path.join(ROOT, 'advanced', 'data', 'mini')
OUT_CAD = os.path.join(ROOT, 'cad', 'mini')

EDF_OD, EDF_LEN = 0.098, 0.075          # 90 mm 12-blade EDF unit: casing OD and length
S = (EDF_OD + 0.006) / 1.200            # scale: the full-size duct throat (Ø1.20 m) just clears the EDF casing
GROUND_FULL = -0.23
TIRE_R = 0.740 * S
POD_DEPTH = EDF_LEN + 0.004
WB, TRACK = 1.80 * S, 1.18 * S + 0.012  # pods sit slightly outboard of scale to clear the deeper body pockets
WC_Y = TIRE_R
HINGE = (0.0, TIRE_R + 0.010, -POD_DEPTH / 2 + 0.004)
CHASSIS_Y = 0.032

MATERIALS.update({
    'petg': dict(name='PETG, 40 % gyroid infill (effective density)', rho=640, color='#d8dde3', look='plastic'),
    'tpu': dict(name='TPU 95A, 60 % infill', rho=740, color='#1a1d22', look='rubber'),
    'lwpla': dict(name='Lightweight foaming PLA, 1.9 mm walls', rho=620, color='#c8303f', look='paint'),
    'cf_plate': dict(name='2 mm carbon-fibre plate (CNC cut)', rho=1550, color='#2a313b', look='carbon'),
    'clear_petg': dict(name='Clear PETG, 0.8 mm vase-mode', rho=1270, color='#9fc9e0', look='glass'),
})

def full_to_mini(shape):
    return shape.translate(V(0, -GROUND_FULL, 0)).scale(S)

# --------------------------------------------------------------- body shell from the full-size loft
_BODY = {}
def mini_body():
    if 'skin' in _BODY:
        return _BODY['skin']
    import build_rev08 as B
    outer = full_to_mini(B.body_solid())
    inner = full_to_mini(B.body_solid(0.022))
    skin = B._bool(outer, inner, 'cut')
    for sx in (WB, -WB):          # pod pockets: the deeper pods need room to swing up and out
        for sz in (1, -1):
            skin = B._bool(skin, box(0.150, 0.30, 0.10, (sx, 0.10, sz * (TRACK + 0.010))), 'cut')
    skin = B._bool(skin, box(0.20, 0.06, 0.09, (-0.02, 0.135, 0)), 'cut')     # canopy opening
    _BODY['skin'] = skin
    return skin

def body_section(xa, xb):
    import build_rev08 as B
    return B._bool(mini_body(), box(xb - xa, 1, 1, ((xa + xb) / 2, 0, 0)), 'and')

def canopy():
    import build_rev08 as B
    g = full_to_mini(B.canopy_solid())
    return B._bool(g, full_to_mini(B.canopy_solid(0.012)), 'cut')

# --------------------------------------------------------------- printed / cut parts
def chassis_plate():
    L, Wd = 0.40, 0.112
    p = box(L, 0.002, Wd, (0, 0, 0))
    for sx in (-0.13, -0.06, 0.03, 0.10):    # lightening slots
        for sz in (-0.025, 0.025):
            p = p.cut(box(0.05, 0.01, 0.026, (sx, 0, sz)))
    holes = [(x, z * 0.042) for x in (-0.12, 0.12) for z in (-1, 1)]                                     # landing legs
    holes += [(sx * WB + dx, z * 0.046) for sx in (-1, 1) for dx in (-0.009, 0.009) for z in (-1, 1)]    # hinge masts
    holes += [(0.06 + dx, dz) for dx in (-0.0153, 0.0153) for dz in (-0.0153, 0.0153)]                   # FC plate standoffs
    for x, z in holes:
        p = p.cut(cyl(0.0016, 0.01, 'y', (x, 0, z)))
    return p

def duct_ring():
    """Thin duct around the EDF casing, 8 spokes, and an outer band the TPU tyre stretches over."""
    ri = EDF_OD / 2 + 0.0004
    rb = TIRE_R - 0.003
    inner = tube(ri + 0.003, ri, POD_DEPTH, 'z')
    lip = tube(ri + 0.007, ri - 0.004, 0.006, 'z', (0, 0, POD_DEPTH / 2 - 0.003)).cut(cyl(ri - 0.004, 0.02, 'z', (0, 0, POD_DEPTH / 2)))
    band = tube(rb, rb - 0.002, POD_DEPTH * 0.82, 'z')
    spokes = [rot(box(rb - ri - 0.004, 0.003, POD_DEPTH * 0.82, ((rb + ri) / 2, 0, 0)), 'z', 45 * k + 22.5) for k in range(8)]
    boss = box(0.022, 0.012, 0.020, (0, rb + 0.004, -POD_DEPTH / 2 + 0.012))
    return union(inner, lip, band, boss, *spokes)

def tyre_ring():
    ring = tube(TIRE_R, TIRE_R - 0.003, POD_DEPTH * 0.82, 'z')
    for k in range(36):
        ring = ring.fuse(rot(box(0.006, 0.0015, POD_DEPTH * 0.82, (TIRE_R + 0.0006, 0, 0)), 'z', k * 10))
    return ring

def yoke_arm():
    dz = -POD_DEPTH / 2 - 0.008 - HINGE[2]
    return union(cyl(0.007, 0.024, 'x'), box(0.016, 0.010, abs(dz) + 0.010, (0, 0, dz / 2)),
                 box(0.018, HINGE[1] - (TIRE_R - 0.005) + 0.012, 0.008, (0, -(HINGE[1] - (TIRE_R - 0.005)) / 2, dz)),
                 box(0.022, 0.010, 0.020, (0, -(HINGE[1] - (TIRE_R - 0.005)) - 0.004, dz + 0.012)))

def hinge_block():
    b = box(0.040, 0.030, 0.026)
    b = b.cut(cyl(0.0111, 0.010, 'x', (0.015, 0.006, 0))).cut(cyl(0.0111, 0.010, 'x', (-0.015, 0.006, 0)))
    return b.cut(cyl(0.0042, 0.05, 'x', (0, 0.006, 0)))

def hinge_mast():
    """Upright from the chassis-plate edge to the hinge block (corner-node frame): carries pod weight and thrust into the plate."""
    y0, y1, z = CHASSIS_Y + 0.001 - WC_Y, HINGE[1] - 0.015, HINGE[2] - 0.0285
    web = box(0.030, y1 - y0, 0.006, (0, (y0 + y1) / 2, z))
    foot = box(0.030, 0.003, 0.020, (0, y0 + 0.0015, z - 0.007))
    top = box(0.030, 0.003, 0.026, (0, y1 - 0.0015, HINGE[2] - 0.020))
    gussets = [box(0.003, 0.020, 0.012, (dx, y0 + 0.011, z - 0.006)) for dx in (-0.0135, 0.0135)]
    m = union(web, foot, top, *gussets)
    for dx in (-0.009, 0.009):
        m = m.cut(cyl(0.0016, 0.01, 'y', (dx, y0, 0.046 - TRACK)))
    return m

def leg():
    return union(box(0.010, CHASSIS_Y - 0.004, 0.010, (0, -(CHASSIS_Y - 0.004) / 2, 0)), cyl(0.009, 0.004, 'y', (0, -CHASSIS_Y + 0.006, 0)))

def battery_tray():
    t = box(0.17, 0.003, 0.060)
    return union(t, box(0.17, 0.010, 0.003, (0, 0.005, 0.0315)), box(0.17, 0.010, 0.003, (0, 0.005, -0.0315)))

def fc_plate():
    p = box(0.060, 0.003, 0.060)
    for x in (-0.0153, 0.0153):
        for z in (-0.0153, 0.0153):
            p = p.cut(cyl(0.0016, 0.01, 'y', (x, 0, z)))
    return p

# --------------------------------------------------------------- JDM-M40 custom fan motor (sized by motor_design.py)
MOTOR = json.load(open(os.path.join(OUT_WEB, 'motor.json')))['design']
MOTOR_Z = -0.012          # motor centre on the fan axis, behind the rotor, inside the stator-vane hub
MATERIALS.update({
    'st4140': dict(name='4140 steel, pre-hardened 28–32 HRC (rotor bell / back iron)', rho=7850, color='#7a828b', look='metal'),
    'magwire': dict(name='Class-H magnet wire, Ø0.46 mm, varnished (with slot fill)', rho=3600, color='#c87f4f', look='copper'),
})

def stator():
    M = MOTOR; L = M['L']; bore = 0.0072
    s_ = union(tube(M['r_yo'], bore, L, 'z'),
               *[rot(box(M['depth'] + 0.0004, M['wt'], L, ((M['r_yo'] + M['r_s'] - M['tip']) / 2, 0, 0)), 'z', 360 / M['Ns'] * k) for k in range(M['Ns'])],
               tube(M['r_s'], M['r_s'] - M['tip'], L, 'z'))
    for k in range(M['Ns']):     # slot openings between the tooth tips
        s_ = s_.cut(rot(box(M['tip'] * 3, M['slot_open'], L * 1.2, (M['r_s'] - M['tip'] / 2, 0, 0)), 'z', 360 / M['Ns'] * (k + 0.5)))
    return s_

def tooth_coil():
    """One concentrated coil, drawn as the copper envelope around a tooth (10 turns, axis radial)."""
    M = MOTOR; L = M['L']
    r0, r1 = M['r_yo'] + 0.0003, M['r_s'] - M['tip'] - 0.0003
    r_mid = (r0 + r1) / 2
    c = max(0.0006, (2 * math.pi * r0 / M['Ns'] - M['wt']) / 2 - 0.0003)   # coil build limited by the slot width at the root
    outer = box(r1 - r0, M['wt'] + 2 * c, L + 2 * c, (r_mid, 0, 0))
    return outer.cut(box(r1 - r0 + 0.002, M['wt'] + 0.0002, L + 0.0002, (r_mid, 0, 0)))

def magnet_arc():
    M = MOTOR; L = M['L']; ang = 360 / M['P'] * M['alpha_p']
    ring = tube(M['r_mag_i'] + M['hm'], M['r_mag_i'], L, 'z')
    a = math.radians(ang / 2); R = 0.03
    wedge = extrude_xy([(0, 0), (R * math.cos(a), -R * math.sin(a)), (R, 0), (R * math.cos(a), R * math.sin(a))], L * 1.2, -L * 0.6)
    return ring.intersect(wedge)

def rotor_bell():
    M = MOTOR; Lb = M['L_bell']; ro = M['r_out']; ri = ro - M['t_bell']
    b = tube(ro, ri, Lb, 'z', (0, 0, 0))
    face = cyl(ro, 0.0015, 'z', (0, 0, Lb / 2 + 0.00075))
    for k in range(6):          # cooling vents in the front face
        face = face.cut(rot(cyl(0.0042, 0.004, 'z', (0.0115, 0, Lb / 2 + 0.00075)), 'z', 60 * k))
    boss = tube(0.0055, 0.0025, 0.006, 'z', (0, 0, Lb / 2 - 0.0015))
    return union(b, face, boss)

def motor_shaft():
    return cyl(0.0025, 0.046, 'z', (0, 0, 0.004))

def stator_hub():
    M = MOTOR; L = M['L']
    t = tube(0.0072, 0.0055, L + 0.006, 'z', (0, 0, -0.003))
    fl = tube(0.016, 0.0055, 0.003, 'z', (0, 0, -L / 2 - 0.0075))
    for k in range(4):          # 4 × M3 on a 25 mm circle (drill to suit your fan's motor mount)
        fl = fl.cut(rot(cyl(0.0016, 0.01, 'z', (0.0125, 0, -L / 2 - 0.0075)), 'z', 45 + 90 * k))
    return union(t, fl)

def bearing_685():
    return tube(0.0055, 0.0025, 0.005, 'z')

def build():
    node('mini', 'Miniature prototype', None, system='M00')
    node('m_body', 'Body shell', 'mini', system='M10')
    node('m_frame', 'Frame', 'mini', system='M20')
    node('m_elec', 'Electronics', 'mini', system='M40')
    sysP, sysB = 'M20 Printed / cut parts', 'M40 Purchased components'
    pr = dict(print=True)
    part('JDM-101', 'Body shell, front section', 'M10 Body', 'Shell', 'lwpla', lambda: body_section(0.095, 0.40), world=True,
         shell_t=0.0019, spec=dict(pr, printer='LW-PLA, 0.4 mm nozzle, 2 perimeters, no infill', bed_mm='≥ 220 × 140'),
         process='FDM print, sand + paint', fn='Scaled A1 body, front third (nose to front pods)')
    part('JDM-102', 'Body shell, middle section', 'M10 Body', 'Shell', 'lwpla', lambda: body_section(-0.095, 0.095), world=True,
         shell_t=0.0019, spec=dict(pr, printer='LW-PLA, 2 perimeters'), process='FDM print', fn='Cockpit section with canopy opening')
    part('JDM-103', 'Body shell, rear section', 'M10 Body', 'Shell', 'lwpla', lambda: body_section(-0.40, -0.095), world=True,
         shell_t=0.0019, spec=dict(pr, printer='LW-PLA, 2 perimeters'), process='FDM print', fn='Rear third with diffuser')
    part('JDM-104', 'Canopy', 'M10 Body', 'Shell', 'clear_petg', canopy, world=True, shell_t=0.0008,
         spec=dict(pr, printer='Clear PETG, vase mode or thermoform over a printed buck'), process='Print or vacuum-form')
    for pn in ('JDM-101', 'JDM-102', 'JDM-103', 'JDM-104'):
        place(pn, 'm_body')
    part('JDM-201', 'Chassis plate', sysP, 'Frame', 'cf_plate', chassis_plate, spec=dict(pr, cut='2 mm carbon plate, CNC or hand-cut from the STEP outline'),
         process='CNC-routed carbon plate', fn='Backbone: carries hinges, legs, electronics and battery')
    place('JDM-201', 'm_frame', (0, CHASSIS_Y, 0))
    part('JDM-202', 'Landing leg', sysP, 'Frame', 'petg', leg, spec=dict(pr, printer='PETG 100 % infill, TPU foot pad optional'), process='FDM print')
    for x in (-0.12, 0.12):
        for z in (-0.042, 0.042):
            place('JDM-202', 'm_frame', (x, CHASSIS_Y - 0.0015, z))
    part('JDM-203', 'Battery tray', sysP, 'Frame', 'petg', battery_tray, spec=dict(pr, printer='PETG 40 %'), process='FDM print, velcro straps')
    place('JDM-203', 'm_frame', (-0.01, CHASSIS_Y + 0.003, 0))
    part('JDM-204', 'Flight-controller plate (30.5 mm pattern)', sysP, 'Frame', 'petg', fc_plate, spec=dict(pr, printer='PETG 40 %'), process='FDM print')
    place('JDM-204', 'm_frame', (0.06, CHASSIS_Y + 0.020, 0))
    part('JDM-205', 'Hinge block', sysP, 'Pods', 'petg', hinge_block, spec=dict(pr, printer='PETG or CF-nylon, 60 % infill'), process='FDM print, press-fit 2 × 608 bearings')
    part('JDM-209', 'Hinge mast', sysP, 'Pods', 'petg', hinge_mast, spec=dict(pr, printer='PETG or CF-nylon, 100 % infill, print on its side'),
         process='FDM print, 2 × M3 to the chassis plate', fn='Ties the hinge block to the chassis plate; the body shell carries no load')
    part('JDM-206', 'Pod duct ring (holds the EDF)', sysP, 'Pods', 'petg', duct_ring, spec=dict(pr, printer='PETG 40 %, print lip-up, no supports'),
         process='FDM print', fn='Clamps the EDF casing; bell-mouth inlet lip; servo-arm boss')
    part('JDM-207', 'Tyre ring', sysP, 'Pods', 'tpu', tyre_ring, spec=dict(pr, printer='TPU 95A, 3 perimeters'), process='FDM print, stretch over the duct ring')
    part('JDM-208', 'Pod swing arm', sysP, 'Pods', 'petg', yoke_arm, handed=True, spec=dict(pr, printer='CF-nylon or PETG 80 %, print on side'),
         process='FDM print', fn='Connects the hinge shaft to the pod; swings the pod 90° up and outboard')
    # purchased components
    comp = [
        ('JDM-401', '90 mm 12-blade EDF fan unit: housing, stator vanes and rotor (no motor)', lambda: union(
            tube(EDF_OD / 2, EDF_OD / 2 - 0.002, EDF_LEN, 'z'), tube(0.0215, 0.0205, 0.032, 'z', (0, 0, -0.012)), tube(0.0215, 0.006, 0.010, 'z', (0, 0, 0.016)),
            *[rot(box(0.026, 0.003, 0.010, (0.034, 0, 0.016)), 'z', 30 * k) for k in range(12)],
            *[rot(box(0.026, 0.0015, 0.020, (0.034, 0, -0.016)), 'z', 360 / 7 * k + 8) for k in range(7)]), 0.150),
        ('JDM-402', '100 A ESC (BLHeli_32, 6S)', lambda: box(0.060, 0.012, 0.030), 0.095),
        ('JDM-403', '6S 5000 mAh LiPo, 75C (two in parallel)', lambda: box(0.155, 0.040, 0.048), 0.790),
        ('JDM-404', 'Flight controller (ArduPilot H7, 30.5 mm)', lambda: box(0.036, 0.008, 0.036), 0.012),
        ('JDM-405', 'GNSS + compass module', lambda: cyl(0.014, 0.008, 'y'), 0.025),
        ('JDM-406', 'ExpressLRS receiver', lambda: box(0.020, 0.004, 0.012), 0.004),
        ('JDM-407', 'High-torque digital servo (35 kg·cm at 7.4 V, metal gear)', lambda: union(box(0.040, 0.038, 0.020), cyl(0.0035, 0.006, 'y', (0.010, 0.022, 0))), 0.070),
        ('JDM-408', 'Power distribution board (XT90, 4 × ESC pads)', lambda: box(0.050, 0.004, 0.050), 0.030),
        ('JDM-409', 'Servo BEC, 7.4 V 8 A', lambda: box(0.040, 0.010, 0.020), 0.020),
        ('JDM-410', '608 ball bearing (8 × 22 × 7)', lambda: tube(0.011, 0.004, 0.007, 'x'), 0.012),
        ('JDM-411', 'Hinge shaft, 8 mm steel, 44 mm', lambda: cyl(0.004, 0.044, 'x'), 0.017),
    ]
    sysM = 'M50 Custom fan motor (JDM-M40)'
    M = MOTOR
    part('JDM-501', f'Stator lamination stack, {M["Ns"]} teeth', sysM, 'Motor', 'sisteel', stator, process='Laser-cut 0.2 mm NO20 laminations, bonded',
         spec=dict(laminations=round(M['L'] / 0.0002), od_mm=round(M['r_s'] * 2000, 2), id_mm=14.4, stack_mm=round(M['L'] * 1000, 1), tooth_mm=round(M['wt'] * 1000, 2)),
         fn='Carries the winding; teeth sized for 1.65 T at full flux')
    part('JDM-502', f'Tooth coil, {M["N"]} turns Ø{M["d_wire"] * 1000:.2f} mm', sysM, 'Motor', 'magwire', tooth_coil, mass=MOTOR['m_cu'] / MOTOR['Ns'] * 1.08, process='Hand-wound on the tooth, varnished',
         spec=dict(turns=M['N'], wire_mm=round(M['d_wire'] * 1000, 2), connection=f'{M["conn"]}, {M["paths"]} parallel paths'), fn='Concentrated coil; 12 per motor')
    part('JDM-503', 'Magnet arc, N45SH, radially magnetised', sysM, 'Motor', 'ndfeb', magnet_arc, supplier='buy', process='Purchased (custom arc), bonded in the bell',
         spec=dict(thickness_mm=M['hm'] * 1000, length_mm=M['L'] * 1000, arc_deg=round(360 / M['P'] * M['alpha_p'], 1)), fn='Alternate N / S facing inward')
    part('JDM-504', 'Rotor bell / back iron', sysM, 'Motor', 'st4140', rotor_bell, process='Turned from 4140 tube, vents drilled',
         spec=dict(od_mm=40.0, wall_mm=M['t_bell'] * 1000, length_mm=round(M['L_bell'] * 1000, 1)), fn='Returns the magnet flux and spins the fan through the shaft')
    part('JDM-505', 'Motor shaft, Ø5 mm hardened steel, M5 front thread', sysM, 'Motor', 'steel_stn', motor_shaft, process='Ground rod, threaded and flatted')
    part('JDM-506', '685 hybrid ceramic bearing (5 × 11 × 5)', sysM, 'Motor', 'bearing', bearing_685, supplier='buy', process='Purchased')
    part('JDM-507', 'Stator hub and motor mount', sysM, 'Motor', 'al6061', stator_hub, process='Turned 6061-T6', fn='Holds both bearings; flange bolts into the fan’s motor mount')
    for pn, name, fb, m in comp:
        part(pn, name, sysB, 'Electronics' if pn in ('JDM-402', 'JDM-404', 'JDM-405', 'JDM-406', 'JDM-408', 'JDM-409') else 'Propulsion' if pn in ('JDM-401', 'JDM-403') else 'Pods',
             'electronic', fb, mass=m, supplier='buy', process='Purchased hobby component')
    for k in range(2):           # two packs stacked on the tray, joined by a parallel lead
        place('JDM-403', 'm_elec', (-0.01, CHASSIS_Y + 0.0255 + 0.041 * k, 0.0))
    place('JDM-404', 'm_elec', (0.06, CHASSIS_Y + 0.026, 0))
    place('JDM-405', 'm_elec', (-0.03, 0.142, 0))
    place('JDM-406', 'm_elec', (0.10, CHASSIS_Y + 0.010, 0.03))
    place('JDM-408', 'm_elec', (0.10, CHASSIS_Y + 0.004, -0.02))
    place('JDM-409', 'm_elec', (0.13, CHASSIS_Y + 0.007, 0.02))
    for c, (x, mirror) in {'FL': (WB, False), 'FR': (WB, True), 'RL': (-WB, False), 'RR': (-WB, True)}.items():
        cn = node(f'mc_{c}', f'Corner {c}', 'mini', pos=(x, WC_Y, -TRACK if mirror else TRACK), mirror=mirror, system='M30')
        place('JDM-209', cn)
        place('JDM-205', cn, (0, HINGE[1] + 0.000, HINGE[2] - 0.020))
        for dx in (-0.015, 0.015):
            place('JDM-410', cn, (dx, HINGE[1] + 0.006 - 0.006, HINGE[2] - 0.020))
        place('JDM-407', cn, (0.034, HINGE[1] - 0.004, HINGE[2] - 0.020))
        place('JDM-402', 'm_elec', (x * 0.55, CHASSIS_Y + 0.008, (-1 if mirror else 1) * 0.040))
        yk = node(f'my_{c}', f'Pod swing {c}', cn, pos=HINGE, anim=dict(kind='yoke', type='rot', axis=(1, 0, 0), sign=1), system='M30')
        place('JDM-411', yk)
        place('JDM-208', yk)
        pod = node(f'mp_{c}', f'Pod {c}', yk, pos=(0, -HINGE[1], -HINGE[2]), system='M30')
        place('JDM-206', pod); place('JDM-207', pod); place('JDM-401', pod)
        place('JDM-501', pod, (0, 0, MOTOR_Z)); place('JDM-507', pod, (0, 0, MOTOR_Z)); place('JDM-505', pod, (0, 0, MOTOR_Z))
        place('JDM-504', pod, (0, 0, MOTOR_Z + (MOTOR['L_bell'] - MOTOR['L']) / 2 - 0.0015))
        for zb in (-MOTOR['L'] / 2 - 0.0015, MOTOR['L'] / 2 + 0.0015):
            place('JDM-506', pod, (0, 0, MOTOR_Z + zb))
        for k in range(MOTOR['Ns']):
            place('JDM-502', pod, (0, 0, MOTOR_Z), q_axis((0, 0, 1), 360 / MOTOR['Ns'] * k))
        for k in range(MOTOR['P']):
            place('JDM-503', pod, (0, 0, MOTOR_Z), q_axis((0, 0, 1), 360 / MOTOR['P'] * (k + 0.5)))

META = dict(title='Joukowsky Dynamics miniature prototype', scale=S, edf_od=EDF_OD, edf_len=EDF_LEN, tire_R=TIRE_R, hinge=HINGE,
            wheelbase=2 * WB, track=2 * TRACK, chassis_y=CHASSIS_Y,
            status='Buildable concept. Printed-part masses are from CAD volumes at the stated effective densities.',
            motor=json.load(open(os.path.join(OUT_WEB, 'motor.json'))))

if __name__ == '__main__':
    import build_rev08 as B
    B.body_panel('JD-101-010'); B.glazing('ws')        # OCC quirk: do lofted booleans before any export
    mini_body(); canopy()
    build()
    print(f'scale 1:{1 / S:.1f}, {len(PARTS)} parts, {len(INST)} instances')
    cat = export(OUT_WEB, OUT_CAD, META, step_assembly=True, bom_name='Joukowsky-Dynamics-mini-prototype-BOM.csv',
                 asm_name='Joukowsky-Dynamics-mini-prototype-assembly.step')
    os.makedirs(os.path.join(OUT_CAD, 'stl'), exist_ok=True)
    for pn, P in PARTS.items():
        if not P['spec'].get('print'):
            continue
        shp = S_ = jdcad.S(P['builder']())
        if P['world']:
            bb = shp.BoundingBox()
            shp = shp.translate(V(-(bb.xmin + bb.xmax) / 2, -bb.ymin, -(bb.zmin + bb.zmax) / 2))
        cq.exporters.export(cq.Workplane().add(shp.scale(1000)), os.path.join(OUT_CAD, 'stl', f'{pn}.stl'), tolerance=0.05, angularTolerance=0.2)
        if P['handed']:
            cq.exporters.export(cq.Workplane().add(shp.mirror('XY').scale(1000)), os.path.join(OUT_CAD, 'stl', f'{pn}-LH.stl'), tolerance=0.05, angularTolerance=0.2)
    print('STL written')
