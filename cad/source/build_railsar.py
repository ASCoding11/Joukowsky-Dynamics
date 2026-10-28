"""Tideline ground prototype: a tabletop rail SAR that images a model harbour through a 'cloud' screen.

Run:  python build_railsar.py
Writes satellite/data/proto/{parts.json,meshes.bin}, cad/sat-proto/step, cad/sat-proto/stl and a BOM.

A 60 GHz FMCW radar board rides a 1 m belt-driven rail, stopping every 2.5 mm to record a chirp. Moving the radar
synthesises a long aperture exactly as the satellite does in orbit; back-projection focuses the echoes into an image.
Frame: +X along the rail, +Y up, +Z towards the harbour. Units metres.
"""
import os, sys
import jdcad
from jdcad import *

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
OUT_WEB = os.path.join(ROOT, 'satellite', 'data', 'proto')
OUT_CAD = os.path.join(ROOT, 'cad', 'sat-proto')

RAIL_L, RAIL_Y = 1.20, 0.35        # extrusion length; rail top height above the table
HARBOUR_Z = 1.6                    # harbour board centre distance from the rail
MATERIALS.update({
    'al_ext': dict(name='6063-T5 aluminium extrusion, 20 × 40 V-slot', rho=2700, color='#b8c0c8', look='metal'),
    'pla': dict(name='PLA, 30 % infill (effective density)', rho=520, color='#d8dde3', look='plastic'),
    'mdf': dict(name='MDF board, painted', rho=700, color='#2d5a78', look='plastic'),
    'card': dict(name='Corrugated cardboard (cloud screen)', rho=120, color='#b89a6a', look='fabric'),
    'foil': dict(name='Aluminium foil tape on card', rho=600, color='#dfe5ea', look='metal'),
})

def extrusion():
    e = box(RAIL_L, 0.040, 0.020)
    for y in (-0.010, 0.010):
        e = e.cut(box(RAIL_L + 0.01, 0.006, 0.006, (0, y, 0.0105))).cut(box(RAIL_L + 0.01, 0.006, 0.006, (0, y, -0.0105)))
    return e.cut(cyl(0.0021, RAIL_L + 0.01, 'x', (0, 0.01, 0))).cut(cyl(0.0021, RAIL_L + 0.01, 'x', (0, -0.01, 0)))

def carriage():
    p = box(0.09, 0.006, 0.07)
    for x in (-0.03, 0.03):
        for z in (-0.025, 0.025):
            p = p.cut(cyl(0.0026, 0.02, 'y', (x, 0, z)))
    return p

def vwheel():
    return union(cyl(0.012, 0.011, 'z'), cyl(0.0125, 0.004, 'z'))

def radar_mount():
    base = box(0.07, 0.004, 0.05)
    up = box(0.07, 0.06, 0.004, (0, 0.032, 0.023))
    gus = [box(0.004, 0.03, 0.02, (x, 0.017, 0.012)) for x in (-0.03, 0.03)]
    return union(base, up, *gus)

def radar_board():
    """60 GHz FMCW radar board with antennas-in-package (1 TX, 3 RX), on its MCU baseboard."""
    pcb = box(0.065, 0.040, 0.0016)
    chip = box(0.008, 0.008, 0.0012, (0.0, 0.004, 0.0014))
    base = box(0.065, 0.040, 0.0016, (0, 0, -0.012))
    usb = box(0.012, 0.008, 0.006, (0.026, -0.016, -0.008))
    stand = [cyl(0.0025, 0.012, 'z', (x, y, -0.006)) for x in (-0.028, 0.028) for y in (-0.016, 0.016)]
    return union(pcb, chip, base, usb, *stand)

def stepper():
    return union(box(0.042, 0.042, 0.040), cyl(0.0025, 0.022, 'z', (0, 0, 0.031)), cyl(0.011, 0.002, 'z', (0, 0, 0.021)))

def motor_plate():
    p = box(0.05, 0.06, 0.004)
    return p.cut(cyl(0.011, 0.01, 'z', (0, 0.008, 0)))

def pulley():
    return union(cyl(0.0064, 0.007, 'z'), cyl(0.0085, 0.001, 'z', (0, 0, 0.004)), cyl(0.0085, 0.001, 'z', (0, 0, -0.004)), cyl(0.0065, 0.006, 'z', (0, 0, -0.007)))

def idler():
    return union(cyl(0.0064, 0.007, 'z'), cyl(0.0085, 0.001, 'z', (0, 0, 0.004)), cyl(0.0085, 0.001, 'z', (0, 0, -0.004)))

def belt():
    L = RAIL_L - 0.06
    return union(box(L, 0.0015, 0.006, (0, 0.0064, 0)), box(L, 0.0015, 0.006, (0, -0.0064, 0)))

def end_plate():
    p = box(0.006, 0.08, 0.06)
    return p.cut(cyl(0.0026, 0.02, 'x', (0, 0.01, 0))).cut(cyl(0.0026, 0.02, 'x', (0, -0.01, 0)))

def leg():
    return union(box(0.020, RAIL_Y - 0.02, 0.020, (0, (RAIL_Y - 0.02) / 2, 0)), box(0.12, 0.006, 0.08, (0, 0.003, 0)))

def limit_switch():
    return union(box(0.020, 0.010, 0.006), box(0.016, 0.002, 0.004, (0.002, 0.006, 0)))

def pi_case():
    return union(box(0.094, 0.030, 0.063), box(0.02, 0.008, 0.012, (-0.04, 0.012, 0.0)))

def driver_board():
    return union(box(0.06, 0.004, 0.05), box(0.016, 0.012, 0.016, (0.01, 0.008, 0.005)))

def psu():
    return box(0.11, 0.035, 0.06)

def harbour_board():
    b = box(1.20, 0.012, 0.80)
    quay = box(0.60, 0.012, 0.12, (-0.25, 0.012, 0.30))
    pier = box(0.10, 0.012, 0.35, (0.30, 0.012, 0.18))
    return union(b, quay, pier)

def model_ship(L=0.30, B=0.045):
    hull = box(L, 0.022, B)
    bow = rot(box(B * 0.71, 0.022, B * 0.71), 'y', 45).translate(V(L / 2, 0, 0)).intersect(box(B, 0.03, B * 1.2, (L / 2 + B / 4, 0, 0)))
    house = box(0.035, 0.03, B * 0.8, (-L / 2 + 0.04, 0.026, 0))
    stacks = [box(L * 0.12, 0.016, B * 0.9, (-L / 2 + 0.09 + k * L * 0.14, 0.019, 0)) for k in range(5)]
    return union(hull, bow, house, *stacks)

def reflector():
    """Trihedral corner reflector, 100 mm edge: the radar calibration target."""
    a = 0.1
    return union(box(a, 0.0015, a, (a / 2, 0, a / 2)), box(0.0015, a, a, (0, a / 2, a / 2)), box(a, a, 0.0015, (a / 2, a / 2, 0)))

def cloud_screen():
    s = box(0.90, 0.40, 0.006)
    feet = [box(0.06, 0.02, 0.12, (x, -0.21, 0)) for x in (-0.4, 0.4)]
    return union(s, *feet)

def build():
    node('rig', 'Rail SAR rig', None, system='P00')
    node('carriage', 'Moving carriage', 'rig', pos=(0, RAIL_Y + 0.02, 0), anim=dict(kind='scan', type='trans', axis=(1, 0, 0)), system='P10')
    node('scene', 'Model harbour', 'rig', pos=(0, 0, HARBOUR_Z), system='P30')
    P10, P20, P30 = 'P10 Rail and carriage', 'P20 Electronics', 'P30 Targets and scene'
    part('TP-101', 'Rail, 20 × 40 V-slot extrusion, 1.2 m', P10, 'Rail', 'al_ext', extrusion, supplier='buy', process='Cut to length')
    part('TP-102', 'Carriage plate', P10, 'Carriage', 'al6061', carriage, supplier='buy', process='Purchased V-slot gantry plate')
    part('TP-103', 'V-wheel with bearings', P10, 'Carriage', 'plastic', vwheel, supplier='buy', mass=0.012)
    part('TP-104', 'Radar mount (printed)', P10, 'Carriage', 'pla', radar_mount, spec=dict(print=True, printer='PLA, 3 perimeters, 30 %'), process='FDM print')
    part('TP-105', 'NEMA 17 stepper motor, 1.8°', P10, 'Drive', 'electronic', stepper, supplier='buy', mass=0.28)
    part('TP-106', 'Motor plate (printed)', P10, 'Drive', 'pla', motor_plate, spec=dict(print=True, printer='PLA, 4 perimeters, 50 %'), process='FDM print')
    part('TP-107', 'GT2 pulley, 20 teeth', P10, 'Drive', 'al6061', pulley, supplier='buy', mass=0.008)
    part('TP-108', 'GT2 idler, 20 teeth', P10, 'Drive', 'al6061', idler, supplier='buy', mass=0.006)
    part('TP-109', 'GT2 belt, 6 mm', P10, 'Drive', 'rubber', belt, supplier='buy', mass=0.02)
    part('TP-110', 'End plate (printed)', P10, 'Rail', 'pla', end_plate, spec=dict(print=True, printer='PLA, 4 perimeters, 50 %'), process='FDM print')
    part('TP-111', 'Leg with foot (printed)', P10, 'Rail', 'pla', leg, spec=dict(print=True, printer='PLA, 3 perimeters, 20 %, print on its side'), process='FDM print')
    part('TP-112', 'Limit switch', P10, 'Drive', 'plastic', limit_switch, supplier='buy', mass=0.003)
    part('TP-201', '60 GHz FMCW radar board (58–63.5 GHz, 1 TX / 3 RX) on MCU baseboard', P20, 'Radar', 'pcb', radar_board, supplier='buy', mass=0.025,
         spec=dict(f_start_GHz=58.0, f_stop_GHz=63.5, chirp_us=130, samples=256, EIRP_dBm=10), fn='Transmits a 4 GHz chirp and records the echoes at each rail stop')
    part('TP-202', 'Raspberry Pi 5 in case', P20, 'Control', 'plastic', pi_case, supplier='buy', mass=0.11, fn='Steps the rail, triggers each chirp and logs the data')
    part('TP-203', 'Stepper driver board (TMC2209)', P20, 'Control', 'pcb', driver_board, supplier='buy', mass=0.01)
    part('TP-204', '12 V 3 A power supply', P20, 'Control', 'electronic', psu, supplier='buy', mass=0.25)
    part('TP-301', 'Model harbour board with quay and pier', P30, 'Scene', 'mdf', harbour_board, process='Cut MDF, painted')
    part('TP-302', 'Model container ship, 1:1000 (300 mm)', P30, 'Scene', 'pla', model_ship, spec=dict(print=True, printer='PLA, wrap in foil tape for a strong echo'), process='FDM print')
    part('TP-303', 'Model feeder ship (180 mm)', P30, 'Scene', 'pla', lambda: model_ship(0.18, 0.03), spec=dict(print=True, printer='PLA, foil tape'), process='FDM print')
    part('TP-304', 'Trihedral corner reflector, 100 mm', P30, 'Scene', 'foil', reflector, process='Card + aluminium tape', fn='Known radar cross-section for calibration: 4πa⁴/3λ² ≈ 17 m² at 60 GHz')
    part('TP-305', 'Cloud screen (cardboard, 0.9 × 0.4 m)', P30, 'Scene', 'card', cloud_screen, process='Cut box card',
         fn='Blocks the camera’s view completely; at 60 GHz the radar loses only 1–3 dB through it')
    place('TP-101', 'rig', (0, RAIL_Y, 0))
    for x in (-RAIL_L / 2 + 0.003, RAIL_L / 2 - 0.003):
        place('TP-110', 'rig', (x + (0.003 if x > 0 else -0.003), RAIL_Y, 0))
        place('TP-111', 'rig', (x * 0.92, 0, 0))
        place('TP-112', 'rig', (x * 0.96, RAIL_Y + 0.025, -0.014))
    place('TP-105', 'rig', (-RAIL_L / 2 - 0.03, RAIL_Y, -0.04))
    place('TP-106', 'rig', (-RAIL_L / 2 - 0.03, RAIL_Y, -0.018))
    place('TP-107', 'rig', (-RAIL_L / 2 - 0.03, RAIL_Y, 0.02 - 0.0))
    place('TP-108', 'rig', (RAIL_L / 2 - 0.02, RAIL_Y, 0.02))
    place('TP-109', 'rig', (0, RAIL_Y, 0.02))
    place('TP-102', 'carriage', (0, 0.006, 0))
    for x in (-0.03, 0.03):
        place('TP-103', 'carriage', (x, -0.002, 0.022)); place('TP-103', 'carriage', (x, -0.002, -0.022))
    place('TP-104', 'carriage', (0, 0.011, 0))
    place('TP-201', 'carriage', (0, 0.05, 0.034))
    place('TP-202', 'rig', (-0.35, 0.015, -0.18)); place('TP-203', 'rig', (-0.20, 0.002, -0.18)); place('TP-204', 'rig', (-0.50, 0.0175, -0.20))
    place('TP-301', 'scene', (0, 0.006, 0))
    place('TP-302', 'scene', (-0.25, 0.023, 0.21)); place('TP-302', 'scene', (0.05, 0.023, -0.12), q_axis((0, 1, 0), 25))
    place('TP-303', 'scene', (0.38, 0.023, 0.05), q_axis((0, 1, 0), 90)); place('TP-303', 'scene', (-0.42, 0.023, -0.20), q_axis((0, 1, 0), -15))
    for pos in ((-0.55, 0.012, -0.35), (0.55, 0.012, -0.35), (0.0, 0.012, 0.36)):
        place('TP-304', 'scene', pos, q_axis((0, 1, 0), 225))
    place('TP-305', 'rig', (0, 0.22, 0.80))

META = dict(title='Tideline rail SAR prototype', rail_L=RAIL_L, rail_y=RAIL_Y, harbour_z=HARBOUR_Z,
            radar=dict(f0=58e9, f1=62e9, B=4e9, chirp_s=130e-6, samples=256, fs=2e6, EIRP_dBm=10, NF_dB=12, G_rx_dBi=6, beam_deg=60),
            scan=dict(step_m=0.0025, n=400), status='Buildable bench demonstrator.')

if __name__ == '__main__':
    build()
    export(OUT_WEB, OUT_CAD, META, step_assembly=True, bom_name='Tideline-rail-SAR-BOM.csv', asm_name='Tideline-rail-SAR-assembly.step')
    import cadquery as cq
    os.makedirs(os.path.join(OUT_CAD, 'stl'), exist_ok=True)
    for pn, P in jdcad.PARTS.items():
        if P['spec'].get('print'):
            cq.exporters.export(cq.Workplane().add(S(P['builder']()).scale(1000)), os.path.join(OUT_CAD, 'stl', pn + '.stl'))
    print('STL written')
