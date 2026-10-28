"""Tideline — a Joukowsky Dynamics X-band SAR smallsat for ports and shipping lanes.

Run:  python build_sat.py          (python build_sat.py --fast skips the assembly STEP)
Writes satellite/data/{parts.json,meshes.bin}, cad/sat/step/*.step, a BOM and the assembly STEP.

Frame (as drawn in the workbench): +X along-track (velocity), +Y zenith, -Y nadir, +Z cross-track.
The SAR array looks along -Y; to image, the spacecraft rolls about X so the beam points 15-45° off nadir.
The CAD rest pose is fully deployed. Anim nodes carry their stowed angle, so the workbench can fold the
antenna wings, solar wing and AIS whips back to the launch configuration.
"""
import math, os, sys
import cadquery as cq
from cadquery import Vector as V
import jdcad
from jdcad import *

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
OUT_WEB = os.path.join(ROOT, 'satellite', 'data')
OUT_CAD = os.path.join(ROOT, 'cad', 'sat')

# ---------------------------------------------------------------- top-level dimensions (m)
BX, BY, BZ = 0.80, 0.80, 0.75          # bus: along-track, zenith, cross-track
WALL = 0.020                           # honeycomb wall panels
PANEL_L, PANEL_H, PANEL_T = 0.80, 0.75, 0.045   # SAR array panel: azimuth length, elevation height, thickness
GAP = 0.025                            # hinge gap between panels
N_AZ, N_EL = 4, 34                     # subarrays per panel: 4 in azimuth × 34 rows in elevation (0.7 λ row pitch)
LAM = 299792458 / 9.65e9
SOL = 0.80                             # solar panel edge
SOL_T = 0.020
CELL_D, CELL_H = 0.0186, 0.065         # 18650 Li-ion
BAT_S, BAT_P = 8, 12

MATERIALS.update({
    'honeycomb': dict(name='Al 5056 honeycomb sandwich, 0.5 mm Al 6061 skins (effective density)', rho=180, color='#c9ced3', look='metal'),
    'ant_core': dict(name='CFRP-skinned Al honeycomb antenna panel (effective density)', rho=95, color='#3a424c', look='carbon'),
    'patch': dict(name='Rogers RO4350B patch subarray, copper radiators', rho=1860, color='#c8a25a', look='copper'),
    'trm': dict(name='GaN T/R module (Kovar lid, LTCC substrate)', rho=2600, color='#7f8a94', look='metal'),
    'solar': dict(name='Triple-junction GaAs cells with 100 µm coverglass', rho=900, color='#1c2c5a', look='glass'),
    'sol_core': dict(name='CFRP / Al honeycomb solar substrate (effective density)', rho=110, color='#2a313b', look='carbon'),
    'white': dict(name='Aluminium radiator, white silicate paint (ε 0.9, α 0.2)', rho=2700, color='#e9ecef', look='plastic'),
    'mli': dict(name='Multi-layer insulation, 20 layers aluminised Kapton', rho=300, color='#d8b04c', look='copper'),
    'copv': dict(name='Carbon-overwrapped pressure vessel, Ti liner', rho=1700, color='#2b3138', look='carbon'),
    'cell18650': dict(name='Li-ion 18650 cell, 3.5 Ah high-rate', rho=2700, color='#2f6d8a', look='cell'),
    'kapton': dict(name='Polyimide flex harness', rho=1400, color='#c88a2a', look='plastic'),
})

def electronic(l, w, h, fins=False):
    b = box(l, w, h)
    flange = box(l + 0.012, 0.003, h + 0.012, (0, -w / 2 + 0.0015, 0))
    b = union(b, flange)
    if fins:
        for k in range(int(l / 0.012)):
            b = b.fuse(box(0.002, 0.006, h * 0.8, (-l / 2 + 0.006 + k * 0.012, w / 2 + 0.003, 0)))
    return b

# ---------------------------------------------------------------- structure
def wall_xz():   # top / bottom (zenith / nadir) panels
    return box(BX, WALL, BZ)

def wall_yz():   # ±X panels
    return box(WALL, BY - 2 * WALL, BZ)

def wall_xy():   # ±Z panels
    return box(BX - 2 * WALL, BY - 2 * WALL, WALL)

def shear_deck():
    d = box(BX - 2 * WALL, 0.015, BZ - 2 * WALL)
    return d.cut(cyl(0.09, 0.05, 'y'))

def sep_ring():
    """15-inch launch-adapter ring, spacecraft side (Lightband-class), with its bolt flange."""
    r = 0.3937 / 2
    ring = tube(r + 0.012, r - 0.008, 0.045, 'y', (0, 0.0225, 0))
    fl = tube(r + 0.03, r - 0.008, 0.006, 'y', (0, 0.003, 0))
    for k in range(24):
        fl = fl.cut(rot(cyl(0.003, 0.02, 'y', (r + 0.02, 0.003, 0)), 'y', 15 * k))
    return union(ring, fl)

# ---------------------------------------------------------------- SAR array
def ant_panel():
    """Antenna panel structure, local frame: spans x 0..L, radiating face at y = -T, back at y = 0."""
    p = box(PANEL_L, PANEL_T, PANEL_H, (PANEL_L / 2, -PANEL_T / 2, 0))
    for sx in (0.2, 0.6):          # cut-outs for the panel controller and harness pass-through
        p = p.cut(box(0.12, 0.02, 0.10, (sx, -0.005, 0)))
    return p

def subarray():
    """One radiating subarray: 6 patches on a 0.20 m RO4350B strip, fed by one T/R module."""
    s = box(0.195, 0.0016, 0.0195)
    for k in range(6):
        s = s.fuse(box(0.0093, 0.0006, 0.0093, (-0.0815 + k * 0.0326, -0.0011, 0)))
    return s

def trm():
    t = box(0.060, 0.010, 0.016)
    return t.fuse(box(0.040, 0.004, 0.010, (0, 0.007, 0)))

def tile_ctrl():
    return electronic(0.11, 0.018, 0.09, fins=True)

def ant_radiator():
    return box(PANEL_L - 0.06, 0.0015, PANEL_H - 0.08)

def ant_flex():
    return box(PANEL_L - 0.04, 0.0012, 0.05)

def hinge():
    """Deployment hinge: spring-motor barrel, eddy-current damper and end-of-travel latch."""
    barrel = cyl(0.012, 0.07, 'z')
    leaf_a = box(0.05, 0.004, 0.06, (-0.025, -0.012, 0))
    leaf_b = box(0.05, 0.004, 0.06, (0.025, -0.012, 0))
    damper = cyl(0.009, 0.03, 'z', (0, 0, 0.05))
    latch = box(0.016, 0.012, 0.012, (0.0, 0.012, -0.045))
    return union(barrel, leaf_a, leaf_b, damper, latch)

def hdrm():
    """Hold-down and release mechanism (SMA pin-puller with preloaded cup-cone)."""
    return union(cyl(0.016, 0.045, 'y'), cyl(0.024, 0.006, 'y', (0, -0.0225, 0)), cone(0.012, 0.004, 0.02, 'y', (0, 0.032, 0)))

# ---------------------------------------------------------------- solar
def sol_panel():
    """Solar substrate in its node frame: spans y 0..SOL, z -SOL_T..0 (cells on the -z face)."""
    return box(SOL - 0.02, SOL, SOL_T, (0, SOL / 2, -SOL_T / 2))

def sol_cells():
    c = box(SOL - 0.04, SOL - 0.03, 0.001, (0, 0, 0))
    for k in range(1, 8):
        c = c.cut(box(SOL, 0.002, 0.004, (0, -SOL / 2 + k * SOL / 8, 0)))
    return c

def sol_hinge():
    return union(cyl(0.010, 0.08, 'x'), box(0.07, 0.004, 0.04, (0, 0.01, -0.012)))

# ---------------------------------------------------------------- avionics / payload
def battery_cell():
    return cyl(CELL_D / 2, CELL_H, 'y')

def battery_box():
    w, d = BAT_P * 0.0195 + 0.012, BAT_S * 0.0195 + 0.012
    b = box(w, CELL_H + 0.016, d)
    return b.cut(box(w - 0.006, CELL_H + 0.02, d - 0.006, (0, 0.006, 0)))

def reaction_wheel():
    return union(cyl(0.070, 0.050, 'y'), cyl(0.078, 0.008, 'y', (0, -0.029, 0)))

def mtq():
    return union(cyl(0.012, 0.30, 'x'), cyl(0.016, 0.02, 'x', (0.14, 0, 0)), cyl(0.016, 0.02, 'x', (-0.14, 0, 0)))

def star_tracker():
    return union(box(0.06, 0.06, 0.08), cone(0.034, 0.05, 0.11, 'y', (0, 0.09, 0)).cut(cone(0.03, 0.046, 0.12, 'y', (0, 0.095, 0))))

def sun_sensor():
    return union(box(0.035, 0.008, 0.035), cyl(0.008, 0.004, 'y', (0, 0.006, 0)))

def patch_ant(s=0.08):
    return union(box(s, 0.006, s), box(s * 0.6, 0.002, s * 0.6, (0, 0.004, 0)))

def xband_ant():
    """Gimballed X-band horn for downlink (2-axis, ±70°)."""
    base = cyl(0.045, 0.03, 'z', (0, 0, 0.015))
    yoke = union(box(0.012, 0.09, 0.03, (0.05, 0.045, 0.04)), box(0.012, 0.09, 0.03, (-0.05, 0.045, 0.04)))
    horn = cone(0.018, 0.055, 0.11, 'z', (0, 0.07, 0.09)).cut(cone(0.015, 0.052, 0.12, 'z', (0, 0.07, 0.095)))
    return union(base, yoke, horn)

def ais_whip():
    """AIS VHF monopole: steel tape-spring, a quarter wavelength at 162 MHz, drawn deployed along +y."""
    return union(cyl(0.012, 0.025, 'y', (0, 0.0125, 0)), box(0.012, 0.44, 0.0012, (0, 0.025 + 0.22, 0)))

def tank():
    return union(cyl(0.11, 0.24, 'x'), sphere(0.11, (0.12, 0, 0)), sphere(0.11, (-0.12, 0, 0)), cyl(0.012, 0.05, 'x', (0.25, 0, 0)))

def hall_thruster():
    body = cyl(0.045, 0.06, 'x')
    chan = tube(0.032, 0.018, 0.01, 'x', (-0.032, 0, 0))
    cath = cyl(0.008, 0.06, 'x', (0.0, 0.058, 0))
    return union(body, cath).cut(tube(0.032, 0.018, 0.02, 'x', (-0.03, 0, 0))).fuse(chan.translate(V(0.004, 0, 0)))

def harness():
    return union(box(0.60, 0.012, 0.03, (0, 0, 0)), box(0.012, 0.012, 0.50, (0.29, 0, 0)), box(0.012, 0.012, 0.50, (-0.29, 0, 0)))

def mli_top():
    m = box(BX + 0.004, 0.004, BZ + 0.004)
    return m.cut(cyl(0.21, 0.02, 'y'))

# ---------------------------------------------------------------- assembly
def build():
    y_bot, y_top = -BY / 2, BY / 2
    node('sat', 'Tideline spacecraft', None, system='S00')
    node('bus', 'Bus structure', 'sat', system='S10')
    node('ant_c', 'SAR array, centre panel', 'sat', pos=(-PANEL_L / 2, y_bot - 0.02, 0), system='S20')
    x0 = BX / 2 + GAP
    for side, q in (('px', (0, 0, 0, 1)), ('mx', q_axis((0, 1, 0), 180))):
        sgn = 1 if side == 'px' else -1
        node(f'w2_{side}', f'SAR wing {side.upper()}, inner panel', 'sat', pos=(sgn * x0, y_bot - 0.02, 0), q=q,
             anim=dict(kind='wing2', type='rot', axis=(0, 0, 1), stow=90), system='S20')
        node(f'w3_{side}', f'SAR wing {side.upper()}, outer panel', f'w2_{side}', pos=(PANEL_L + GAP / 2, -PANEL_T - 0.0025, 0),
             anim=dict(kind='wing3', type='rot', axis=(0, 0, 1), stow=180), system='S20')
    node('sol1', 'Solar wing, inner panel', 'sat', pos=(0, y_top, -BZ / 2 - 0.03), anim=dict(kind='sol1', type='rot', axis=(1, 0, 0), stow=180), system='S30')
    node('sol2', 'Solar wing, outer panel', 'sol1', pos=(0, SOL + 0.005, 0.0025), anim=dict(kind='sol2', type='rot', axis=(1, 0, 0), stow=-180), system='S30')
    node('avionics', 'Avionics and power', 'sat', system='S40')
    node('adcs', 'Attitude control', 'sat', system='S50')
    node('prop', 'Propulsion', 'sat', system='S60')
    node('comms', 'Communications', 'sat', system='S70')
    node('payload', 'SAR payload electronics', 'sat', system='S80')
    for k, zc in enumerate((BZ / 2 - 0.10, -(BZ / 2 - 0.10))):
        node(f'ais{k}', f'AIS whip {k + 1}', 'comms', pos=(BX / 2 - 0.08, y_top - 0.06, BZ / 2 + 0.002) if k == 0 else (-(BX / 2 - 0.08), y_top - 0.06, BZ / 2 + 0.002),
             q=q_axis((1, 0, 0), 90), anim=dict(kind='whip', type='rot', axis=(0, 0, 1), stow=90 if k == 0 else -90), system='S70')   # stowed flat along the +Z face
    node('xgim', 'X-band antenna gimbal', 'comms', pos=(0.18, -0.10, BZ / 2), anim=dict(kind='gimbal', type='rot', axis=(1, 0, 0), stow=0), system='S70')

    # -- structure
    S10 = 'S10 Structure'
    part('TL-101-001', 'Nadir panel (array mounting deck)', S10, 'Bus', 'honeycomb', wall_xz, process='Bonded honeycomb sandwich, potted inserts')
    part('TL-101-002', 'Zenith panel', S10, 'Bus', 'honeycomb', wall_xz, process='Bonded honeycomb sandwich')
    part('TL-101-003', '±X side panel', S10, 'Bus', 'honeycomb', wall_yz, process='Bonded honeycomb sandwich')
    part('TL-101-004', '±Z side panel', S10, 'Bus', 'honeycomb', wall_xy, process='Bonded honeycomb sandwich')
    part('TL-101-005', 'Shear deck with tank cut-out', S10, 'Bus', 'honeycomb', shear_deck, process='Bonded honeycomb sandwich')
    part('TL-101-006', 'Launch-adapter ring, 15-inch (spacecraft side)', S10, 'Bus', 'al7075', sep_ring, process='Machined 7075-T7351',
         criticality='Launch load path', fn='Carries all launch loads into the rideshare separation system')
    part('TL-101-007', 'MLI blanket, zenith', 'S10 Structure', 'Thermal', 'mli', mli_top, shell_t=0.004, process='Sewn blanket, grounded')
    place('TL-101-001', 'bus', (0, y_bot + WALL / 2, 0)); place('TL-101-002', 'bus', (0, y_top - WALL / 2, 0))
    place('TL-101-003', 'bus', (BX / 2 - WALL / 2, 0, 0)); place('TL-101-003', 'bus', (-BX / 2 + WALL / 2, 0, 0))
    place('TL-101-004', 'bus', (0, 0, BZ / 2 - WALL / 2)); place('TL-101-004', 'bus', (0, 0, -BZ / 2 + WALL / 2))
    place('TL-101-005', 'bus', (0, 0.0, 0))
    place('TL-101-006', 'bus', (0, y_top, 0))
    place('TL-101-007', 'bus', (0, y_top + 0.002, 0))

    # -- SAR array (5 panels)
    S20 = 'S20 SAR antenna'
    part('TL-201-001', 'Array panel structure (CFRP / honeycomb)', S20, 'Array', 'ant_core', ant_panel, process='Co-cured CFRP skins on Al honeycomb, flatness 0.5 mm',
         criticality='Pointing / flatness', fn='Holds 136 subarrays flat to λ/16 so the beam forms correctly')
    part('TL-201-002', 'Radiating subarray, 6 patches (X-band)', S20, 'Array', 'patch', subarray, process='Etched RO4350B, bonded',
         spec=dict(f_GHz=9.65, patches=6, pitch_mm=32.6), fn='Radiates and receives; one per T/R module')
    part('TL-201-003', 'GaN T/R module, 6 W peak', S20, 'Array', 'trm', trm, supplier='buy', mass=0.028, process='Purchased, screened',
         spec=dict(peak_W=6, NF_dB=3.0, PAE=0.38, phase_bits=6, atten_bits=6), fn='Amplifies the chirp on transmit and the echo on receive; sets phase for beam steering')
    part('TL-201-004', 'Panel controller and power board', S20, 'Array', 'electronic', tile_ctrl, mass=0.45, process='Purchased / in-house PCB',
         fn='Distributes timing, beam-steering words and 28 V to the panel’s T/R modules')
    part('TL-201-005', 'Array back radiator', S20, 'Array', 'white', ant_radiator, process='Painted Al sheet, bonded to TRM heat spreaders',
         fn='Radiates T/R module heat to space after each imaging pass')
    part('TL-201-006', 'Panel flex harness', S20, 'Array', 'kapton', ant_flex, process='Polyimide flex, rigid-flex connectors')
    part('TL-201-007', 'Deployment hinge (spring motor, damper, latch)', S20, 'Deployment', 'ti64', hinge, criticality='Single point: deployment',
         process='Machined Ti-6Al-4V, MoS₂ dry lube', fn='Swings a wing panel open at a controlled rate and latches it flat')
    part('TL-201-008', 'Hold-down release mechanism (SMA pin-puller)', S20, 'Deployment', 'ti64', hdrm, supplier='buy', mass=0.12, criticality='Single point: deployment',
         process='Purchased', fn='Clamps a stowed wing through launch; releases on command')

    def populate(nid):
        place('TL-201-001', nid)
        for i in range(N_AZ):
            for j in range(N_EL):
                z = -PANEL_H / 2 + 0.0145 + j * (PANEL_H - 0.029) / (N_EL - 1)
                place('TL-201-002', nid, (0.1 + i * 0.2, -PANEL_T - 0.0008, z))
                place('TL-201-003', nid, (0.1 + i * 0.2, 0.0055, z))
        place('TL-201-004', nid, (0.40, 0.025, 0.0))
        place('TL-201-005', nid, (0.40, 0.0145, 0))
        place('TL-201-006', nid, (0.40, 0.0156, PANEL_H / 2 - 0.05))
    populate('ant_c')
    for side in ('px', 'mx'):
        populate(f'w2_{side}'); populate(f'w3_{side}')
        for z in (-0.28, 0.28):
            place('TL-201-007', f'w2_{side}', (-GAP / 2, -PANEL_T / 2, z))
            place('TL-201-007', f'w3_{side}', (-GAP / 2, PANEL_T / 2, z), q_axis((1, 0, 0), 180))
        for z in (-0.22, 0.22):   # hold-downs: through the stowed outer panel into the bus side
            place('TL-201-008', f'w3_{side}', (0.62, 0.06, z), q_axis((0, 0, 1), 180))

    # -- solar
    S30 = 'S30 Power generation'
    part('TL-301-001', 'Solar panel substrate', S30, 'Solar wing', 'sol_core', sol_panel, process='CFRP / Al honeycomb')
    part('TL-301-002', 'Solar cell blanket, 0.58 m² triple-junction', S30, 'Solar wing', 'solar', sol_cells, process='Laid-down CIC strings',
         spec=dict(eff_BOL=0.295, cells=96), fn='About 220 W at normal sun per panel at beginning of life')
    part('TL-301-003', 'Solar wing hinge', S30, 'Solar wing', 'ti64', sol_hinge, process='Machined Ti, torsion spring')
    for nid in ('sol1', 'sol2'):
        place('TL-301-001', nid)
        place('TL-301-002', nid, (0, SOL / 2, -SOL_T - 0.0005))
    for x in (-0.28, 0.28):
        place('TL-301-003', 'sol1', (x, 0, 0)); place('TL-301-003', 'sol2', (x, 0, 0))
    place('TL-301-002', 'bus', (0, 0, -BZ / 2 - 0.0005 - 0.0))   # body-mounted cells on the -Z face (covered when stowed)
    for x in (-0.25, 0.25):
        place('TL-201-008', 'bus', (x, y_bot + 0.14, -BZ / 2 - 0.045), q_axis((1, 0, 0), 90))

    # -- avionics and power
    S40 = 'S40 Avionics and power'
    part('TL-401-001', 'Li-ion cell, 18650 high-rate (3.5 Ah, 10 A)', S40, 'Battery', 'cell18650', battery_cell, supplier='buy', mass=0.048,
         spec=dict(V=3.6, Ah=3.5, A_cont=10), process='Purchased, lot-screened')
    part('TL-401-002', f'Battery enclosure, {BAT_S}s{BAT_P}p', S40, 'Battery', 'al6061', battery_box, process='Machined, potted cells')
    part('TL-401-003', 'Power conditioning and distribution unit', S40, 'EPS', 'electronic', lambda: electronic(0.24, 0.10, 0.18, True), mass=3.8,
         fn='Solar MPPT, battery charge, 28 V bus, latching current limiters')
    part('TL-401-004', 'On-board computer (dual, cold-redundant)', S40, 'OBDH', 'electronic', lambda: electronic(0.12, 0.06, 0.12), mass=1.1)
    part('TL-401-005', 'Main harness', S40, 'Harness', 'copper', harness, mass=3.5, process='Space-grade wire, braided')
    place('TL-401-002', 'avionics', (0.18, 0.06 + (CELL_H + 0.016) / 2, 0.15))
    for i in range(BAT_P):
        for j in range(BAT_S):
            place('TL-401-001', 'avionics', (0.18 - (BAT_P - 1) * 0.0195 / 2 + i * 0.0195, 0.06 + 0.008 + CELL_H / 2 + 0.006, 0.15 - (BAT_S - 1) * 0.0195 / 2 + j * 0.0195))
    place('TL-401-003', 'avionics', (-0.20, 0.0075 + 0.05 + 0.003, 0.17))
    place('TL-401-004', 'avionics', (0.17, 0.0075 + 0.03 + 0.003, -0.16))
    place('TL-401-005', 'avionics', (0, 0.33, 0))

    # -- ADCS
    S50 = 'S50 Attitude determination and control'
    part('TL-501-001', 'Reaction wheel, 1.0 N·m·s / 0.1 N·m', S50, 'ADCS', 'electronic', reaction_wheel, supplier='buy', mass=1.9,
         spec=dict(H_Nms=1.0, T_Nm=0.10), fn='Four in a pyramid; slews the spacecraft into imaging attitude')
    part('TL-501-002', 'Magnetorquer rod, 15 A·m²', S50, 'ADCS', 'electronic', mtq, supplier='buy', mass=0.35, fn='Dumps wheel momentum')
    part('TL-501-003', 'Star tracker with baffle', S50, 'ADCS', 'electronic', star_tracker, supplier='buy', mass=0.32, spec=dict(accuracy_arcsec=5))
    part('TL-501-004', 'Coarse sun sensor', S50, 'ADCS', 'electronic', sun_sensor, supplier='buy', mass=0.02)
    part('TL-501-005', 'Fibre-optic gyro IMU', S50, 'ADCS', 'electronic', lambda: electronic(0.09, 0.07, 0.09), supplier='buy', mass=0.75)
    part('TL-501-006', 'GNSS receiver (dual-frequency)', S50, 'ADCS', 'electronic', lambda: electronic(0.10, 0.03, 0.08), supplier='buy', mass=0.25)
    for k, (x, z) in enumerate(((-0.19, -0.19), (0.19, -0.19), (-0.19, 0.19), (0.19, 0.19))):
        tilt = q_rot(((0, 0, 1), 35 * (1 if x > 0 else -1)), ((1, 0, 0), 35 * (1 if z > 0 else -1)))
        place('TL-501-001', 'adcs', (x * 0.9, -0.21, z * 0.9 - 0.0), tilt)
    for a, pos in ((('x', 0)), (('y', 90)), (('z', 0))) if False else ():
        pass
    place('TL-501-002', 'adcs', (0, 0.25, -0.31))
    place('TL-501-002', 'adcs', (0.33, 0.25, 0), q_axis((0, 1, 0), 90))
    place('TL-501-002', 'adcs', (-0.33, 0.08, -0.15), q_axis((0, 0, 1), 90))
    place('TL-501-003', 'adcs', (-0.30, y_top + 0.04, -0.27), q_rot(((1, 0, 0), -25), ((0, 0, 1), 20)))
    place('TL-501-003', 'adcs', (0.30, y_top + 0.04, -0.27), q_rot(((1, 0, 0), -25), ((0, 0, 1), -20)))
    for pos, q in (((0, y_top + 0.004, 0.30), (0, 0, 0, 1)), ((0, 0.20, -BZ / 2 - 0.004), q_axis((1, 0, 0), -90)), ((0, 0.20, BZ / 2 + 0.004), q_axis((1, 0, 0), 90)),
                   ((BX / 2 + 0.004, 0.25, 0.25), q_axis((0, 0, 1), -90)), ((-BX / 2 - 0.004, 0.25, 0.25), q_axis((0, 0, 1), 90)), ((0.30, y_top + 0.004, 0.30), (0, 0, 0, 1))):
        place('TL-501-004', 'adcs', pos, q)
    place('TL-501-005', 'adcs', (0.0, 0.0075 + 0.035 + 0.003, -0.05))
    place('TL-501-006', 'adcs', (-0.05, 0.0075 + 0.015 + 0.003, 0.28))

    # -- propulsion
    S60 = 'S60 Propulsion'
    part('TL-601-001', 'Krypton tank, 10 L COPV, 150 bar', S60, 'Propulsion', 'copv', tank, supplier='buy', mass=3.2, criticality='Pressure vessel',
         spec=dict(volume_L=10, MEOP_bar=150, Kr_capacity_kg=5.5))
    part('TL-601-002', 'Hall-effect thruster, 300 W class', S60, 'Propulsion', 'electronic', hall_thruster, supplier='buy', mass=1.1,
         spec=dict(thrust_mN=17, Isp_s=1350, power_W=300))
    part('TL-601-003', 'Power processing unit', S60, 'Propulsion', 'electronic', lambda: electronic(0.20, 0.06, 0.14, True), supplier='buy', mass=2.4)
    part('TL-601-004', 'Xenon/krypton flow controller', S60, 'Propulsion', 'electronic', lambda: electronic(0.08, 0.04, 0.06), supplier='buy', mass=0.35)
    place('TL-601-001', 'prop', (-0.10, 0.20, 0.05))
    place('TL-601-002', 'prop', (-BX / 2 + 0.01, -0.08, 0.0), q_axis((0, 0, 1), 0))
    place('TL-601-003', 'prop', (-0.12, -0.0075 - 0.03 - 0.003, 0.21), q_axis((1, 0, 0), 180))
    place('TL-601-004', 'prop', (-0.28, 0.0075 + 0.02 + 0.003, -0.05))

    # -- communications
    S70 = 'S70 Communications'
    part('TL-701-001', 'X-band transmitter, 800 Mbit/s, 10 W', S70, 'Comms', 'electronic', lambda: electronic(0.16, 0.05, 0.12, True), supplier='buy', mass=1.3)
    part('TL-701-002', 'X-band gimballed horn antenna', S70, 'Comms', 'al6061', xband_ant, supplier='buy', mass=1.0, spec=dict(gain_dBi=17, range_deg=70))
    part('TL-701-003', 'S-band TT&C transceiver', S70, 'Comms', 'electronic', lambda: electronic(0.10, 0.035, 0.09), supplier='buy', mass=0.45)
    part('TL-701-004', 'S-band patch antenna', S70, 'Comms', 'pcb', patch_ant, supplier='buy', mass=0.06)
    part('TL-701-005', 'AIS receiver (4-channel, 161.975 / 162.025 MHz)', S70, 'Comms', 'electronic', lambda: electronic(0.09, 0.03, 0.07), supplier='buy', mass=0.30,
         fn='Hears ships’ own position reports, to match against radar detections and flag dark vessels')
    part('TL-701-006', 'AIS VHF monopole (tape spring, 0.46 m)', S70, 'Comms', 'steel_stn', ais_whip, supplier='buy', mass=0.08)
    part('TL-701-007', 'L-band relay modem (low-latency ship lists)', S70, 'Comms', 'electronic', lambda: electronic(0.08, 0.025, 0.06), supplier='buy', mass=0.25,
         fn='Sends detection lists through a commercial relay constellation within minutes')
    part('TL-701-008', 'GNSS / L-band patch antenna', S70, 'Comms', 'pcb', lambda: patch_ant(0.06), supplier='buy', mass=0.04)
    place('TL-701-001', 'comms', (0.20, -0.0075 - 0.025 - 0.003, -0.20), q_axis((1, 0, 0), 180))
    place('TL-701-002', 'xgim')
    place('TL-701-003', 'comms', (-0.05, 0.0075 + 0.0175 + 0.003, -0.25))
    place('TL-701-004', 'comms', (0.0, y_top + 0.003, -0.25)); place('TL-701-004', 'comms', (-0.15, -0.20, BZ / 2 + 0.003), q_axis((1, 0, 0), 90))
    place('TL-701-005', 'comms', (0.05, -0.0075 - 0.015 - 0.003, 0.25), q_axis((1, 0, 0), 180))
    place('TL-701-006', 'ais0'); place('TL-701-006', 'ais1')
    place('TL-701-007', 'comms', (-0.20, 0.0075 + 0.0125 + 0.003, -0.25))
    place('TL-701-008', 'comms', (-0.22, y_top + 0.003, 0.25)); place('TL-701-008', 'comms', (0.22, y_top + 0.003, 0.25))

    # -- SAR payload electronics
    S80 = 'S80 SAR payload electronics'
    part('TL-801-001', 'SAR central electronics (chirp, receiver, digitiser, timing)', S80, 'Payload', 'electronic', lambda: electronic(0.30, 0.12, 0.20, True), mass=6.0,
         spec=dict(bandwidth_MHz=300, adc_bits=12, baq_bits=4), fn='Generates the chirp, times every pulse and digitises the echoes')
    part('TL-801-002', 'Payload power unit with supercapacitor bank', S80, 'Payload', 'electronic', lambda: electronic(0.26, 0.10, 0.20, True), mass=7.0,
         fn='Supplies the T/R modules’ pulsed current so the battery sees a smooth load')
    part('TL-801-003', 'Mass memory and on-board processor (1.5 TB, FPGA + GPU)', S80, 'Payload', 'electronic', lambda: electronic(0.18, 0.06, 0.14, True), mass=2.6,
         fn='Stores raw data, forms images and runs ship detection on board')
    place('TL-801-001', 'payload', (0.10, -0.0075 - 0.06 - 0.003, 0.08), q_axis((1, 0, 0), 180))
    place('TL-801-002', 'payload', (-0.17, -0.0075 - 0.05 - 0.003, -0.10), q_axis((1, 0, 0), 180))
    place('TL-801-003', 'payload', (0.24, -0.0075 - 0.03 - 0.003, 0.20), q_axis((1, 0, 0), 180))

META = dict(title='Tideline X-band SAR smallsat', frame='+X along-track, +Y zenith, -Y nadir (array boresight), +Z cross-track',
            f_GHz=9.65, lam=LAM, ant=dict(L=PANEL_L * 5 + GAP * 4, H=PANEL_H, panels=5, trm=5 * N_AZ * N_EL, trm_W=6.0, n_az=N_AZ, n_el=N_EL),
            bus=[BX, BY, BZ], solar=dict(area=3 * (SOL - 0.04) * (SOL - 0.03), eff=0.295), battery=dict(s=BAT_S, p=BAT_P, V=3.6, Ah=3.5, A=10.0),
            status='Concept design. Masses from CAD volumes at stated effective densities, or from catalogue values for purchased units.')

if __name__ == '__main__':
    build()
    print(f'{len(PARTS)} parts, {len(INST)} instances, {len(NODES)} nodes')
    export(OUT_WEB, OUT_CAD, META, step_assembly='--fast' not in sys.argv, bom_name='Tideline-SAR-BOM.csv', asm_name='Tideline-SAR-deployed-assembly.step')
