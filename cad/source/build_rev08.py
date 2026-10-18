"""Joukowsky Dynamics — Rev 08 "wheel-to-proprotor" flying car: every part, CADed.

Run:  python build_rev08.py            (all parts + full-assembly STEP)
      python build_rev08.py --fast     (skip the full-assembly STEP)

Frame: +X forward, +Y up, +Z port, metres. Ground plane y = -0.23 (A1-road baseline).
Each corner frame has its origin at the wheel centre with +Z outboard; starboard corners are
mirror frames, which is why handed parts carry an -LH STEP variant.
"""
import math, json, os, sys
import cadquery as cq
from cadquery import Vector as V
from jdcad import *
import proprotor as PR

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
OUT_WEB = os.path.join(ROOT, 'advanced', 'data')
OUT_CAD = os.path.join(ROOT, 'cad', 'rev08')
SEC = json.load(open(os.path.join(HERE, 'body_sections.json')))

GROUND = -0.23
WC_Y = 0.51                  # wheel-centre height
WB = 1.80                    # half wheelbase
TRACK = 1.18                 # half track (wheel centre plane)
TIRE_R, RIM_R, DUCT_R = 0.74, 0.636, 0.556
HINGE = (0.0, 0.78, -0.08)   # yoke hinge in the corner frame (above the tyre, axis X)
CORNERS = {                  # name: (x, mirror, rotor hand, steer)
    'FL': (WB, False, 'R', True), 'FR': (WB, True, 'R', True),
    'RL': (-WB, False, 'L', False), 'RR': (-WB, True, 'L', False)}
D = PR.DESIGN

# ============================================================== 101 BODY (lofted from A1 surfaces)
def _section_wire(x, yz, inset=0.0):
    ys = [y for y, _ in yz]
    thr = min(0.655, 0.5 * (min(ys) + max(ys)))
    pts = [(y, z) for y, z in yz if y >= thr]
    zb = pts[0][1] * 0.965
    pts = _resample([(0.2, zb)] + pts + [(0.2, -zb)], 48)
    if inset:
        pts = _offset_poly(pts, inset)
    vs = [V(x, y, z) for y, z in pts]
    sp = cq.Edge.makeSpline(vs)
    ln = cq.Edge.makeLine(vs[-1], vs[0])
    return cq.Wire.assembleEdges([sp, ln])

def _resample(pts, n):
    """Arc-length resample of an open polyline to n points (consistent loft parametrisation)."""
    L = [0.0]
    for a, b in zip(pts, pts[1:]):
        L.append(L[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    out, j = [], 0
    for i in range(n):
        s = L[-1] * i / (n - 1)
        while j < len(L) - 2 and L[j + 1] < s:
            j += 1
        t = (s - L[j]) / ((L[j + 1] - L[j]) or 1)
        a, b = pts[j], pts[j + 1]
        out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
    return out

def _offset_poly(pts, t):
    """Offset an open (y, z) section (closed by its bottom chord) inward by t."""
    n = len(pts)
    area = sum(pts[i][1] * pts[(i + 1) % n][0] - pts[(i + 1) % n][1] * pts[i][0] for i in range(n))
    sgn = -1 if area > 0 else 1
    out = []
    for i in range(n):
        a, b, c = pts[i - 1], pts[i], pts[(i + 1) % n]
        nrm = [0.0, 0.0]
        for p, q in ((a, b), (b, c)):
            dy, dz = q[0] - p[0], q[1] - p[1]
            L = math.hypot(dy, dz) or 1
            nrm[0] += -dz / L * sgn; nrm[1] += dy / L * sgn
        L = math.hypot(*nrm) or 1
        out.append((b[0] + nrm[0] / L * t, b[1] + nrm[1] / L * t))
    return out

def _scaled(yz, sy, sz, yc=0.5):
    return [[yc + (y - yc) * sy, z * sz] for y, z in yz]

_BODY = {}
def body_solid(inset=0.0):
    key = round(inset, 4)
    if key in _BODY:
        return _BODY[key]
    secs = SEC['body']
    wires = [_section_wire(-3.10 + inset * 2, _scaled(secs[0]['yz'], 0.78, 0.80), inset)]
    wires += [_section_wire(s['x'], s['yz'], inset) for s in secs]
    wires += [_section_wire(3.08 - inset * 2, _scaled(secs[-1]['yz'], 0.72, 0.78), inset)]
    s = cq.Solid.makeLoft(wires, False)
    _BODY[key] = s
    return s

def body_skin():
    if 'skin' in _BODY:
        return _BODY['skin']
    outer = body_solid()
    inner = body_solid(0.022)
    skin = outer.cut(inner)
    # lower fan-wheel pockets (the fender caps above them ride on the conversion yokes)
    for sx in (WB, -WB):
        for sz in (1, -1):
            skin = skin.cut(box(1.70, 1.2, 0.60, (sx, -0.05, sz * 1.24)))
    # cockpit aperture under the canopy
    skin = skin.cut(box(2.35, 0.6, 1.24, (-0.225, 1.25, 0)))
    _BODY['skin'] = skin
    return skin

BIG = 10.0
def region(xa, xb, za=-BIG, zb=BIG, ya=-BIG, yb=BIG):
    return box(xb - xa, yb - ya, zb - za, ((xa + xb) / 2, (ya + yb) / 2, (za + zb) / 2))

BODY_PANELS = [  # pn, name, region, handed-pair?
    ('JD-101-010', 'Underbody aero tray', region(-2.6, 2.6, -0.9, 0.9, -1, 0.26)),
    ('JD-101-001', 'Front fascia / crash nose', region(2.55, 4)),
    ('JD-101-002', 'Frunk lid / hood', region(1.18, 2.55, -0.80, 0.80, 0.5, 3)),
    ('JD-101-003', 'Front fender, port', region(0.95, 2.55, 0.0, BIG)),
    ('JD-101-004', 'Front fender, starboard', region(0.95, 2.55, -BIG, 0.0)),
    ('JD-101-009', 'Rear fascia / tail', region(-4, -2.55)),
    ('JD-101-008', 'Rear deck lid (parachute bay)', region(-2.55, -1.55, -0.80, 0.80, 0.5, 3)),
    ('JD-101-007', 'Rear quarter panel, port', region(-2.55, -1.15, 0.0, BIG)),
    ('JD-101-011', 'Rear quarter panel, starboard', region(-2.55, -1.15, -BIG, 0.0)),
    ('JD-101-005', 'Rocker / side skirt, port', region(-1.15, 0.95, 0.0, BIG, -1, 0.42)),
    ('JD-101-006', 'Rocker / side skirt, starboard', region(-1.15, 0.95, -BIG, 0.0, -1, 0.42)),
    ('JD-101-012', 'Gull-wing door outer skin, port', region(-1.15, 0.95, 0.60, BIG, 0.42, 3)),
    ('JD-101-013', 'Gull-wing door outer skin, starboard', region(-1.15, 0.95, -BIG, -0.60, 0.42, 3)),
]
def cap_region(sx, sz):
    return box(1.70, 1.2, 0.60, (sx, 1.15, sz * 1.24))

def _occ_bool(a, b, op, fuzzy=0.0):
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut
    from OCP.TopTools import TopTools_ListOfShape
    bld = BRepAlgoAPI_Common() if op == 'and' else BRepAlgoAPI_Cut()
    la, lb = TopTools_ListOfShape(), TopTools_ListOfShape()
    la.Append(a.wrapped); lb.Append(b.wrapped)
    bld.SetArguments(la); bld.SetTools(lb)
    bld.SetRunParallel(False)
    if fuzzy:
        bld.SetFuzzyValue(fuzzy)
    bld.Build()
    if not bld.IsDone() or bld.Shape().IsNull():
        return None
    return cq.Shape.cast(bld.Shape())

def _bool(a, b, op):
    """Serial boolean with fuzzy-tolerance retries (parallel OCC booleans were intermittently null here)."""
    for fz in (0.0, 1e-6, 1e-5, 1e-4):
        r = _occ_bool(a, b, op, fz)
        if r is not None:
            return r
        print('    boolean retry', op, fz)
    raise RuntimeError('boolean failed after retries')

_PANEL_CACHE = {}
def body_panel(pn):
    if not _PANEL_CACHE:
        rem = body_skin()
        for key, sx in (('JD-101-050', WB), ('JD-101-051', -WB)):
            _PANEL_CACHE[key] = _bool(rem, cap_region(sx, 1), 'and')
        for sx in (WB, -WB):
            for sz in (1, -1):
                rem = _bool(rem, cap_region(sx, sz), 'cut')
        for p, n, reg in BODY_PANELS:
            _PANEL_CACHE[p] = _bool(rem, reg, 'and')
            rem = _bool(rem, reg, 'cut')
    return _PANEL_CACHE[pn]

DOOR_HINGE = {'L': (-0.10, 1.64, 0.30), 'R': (-0.10, 1.64, -0.30)}

def build_body():
    node('body', 'Body-in-white & exterior', 'vehicle', system='101')
    node('door_L', 'Gull-wing door, port', 'vehicle', pos=DOOR_HINGE['L'],
         anim=dict(kind='door', type='rot', axis=(1, 0, 0), sign=-1), system='101')
    node('door_R', 'Gull-wing door, starboard', 'vehicle', pos=DOOR_HINGE['R'],
         anim=dict(kind='door', type='rot', axis=(1, 0, 0), sign=1), system='101')
    for pn, name, reg in BODY_PANELS:
        door = 'door outer' in name
        part(pn, name, '101 Body', 'Body panels', 'cfrp_paint', (lambda p=pn: body_panel(p)), world=True,
             shell_t=0.0025, process='Prepreg CFRP, autoclave 135 °C, Class-A clear coat',
             fn='Aerodynamic / Class-A outer surface trimmed from the A1 lofted envelope',
             desc='Lofted through 21 stations sampled from the approved A1-road sculpted body, offset 22 mm '
                  'inboard to form the skin and trimmed at the panel break lines.')
        if door:
            side = 'L' if 'port' in name else 'R'
            h = DOOR_HINGE[side]
            place(pn, f'door_{side}', (-h[0], -h[1], -h[2]))
        else:
            place(pn, 'body')
    # splitter / diffuser / fins
    part('JD-101-020', 'Front splitter', '101 Body', 'Aero devices', 'cfrp',
         lambda: extrude_xy([(0, -0.82), (0.32, -0.70), (0.38, 0), (0.32, 0.70), (0, 0.82)], 0.012),
         process='Pre-preg CFRP, 6 plies', fn='Front downforce / underbody flow conditioning')
    place('JD-101-020', 'body', (2.70, 0.215, 0), q_rot(('x', -90)))
    part('JD-101-021', 'Rear diffuser', '101 Body', 'Aero devices', 'cfrp',
         lambda: rot(extrude_xy([(0, -0.85), (0.55, -0.85), (0.55, 0.85), (0, 0.85)], 0.010), 'z', 0),
         process='Pre-preg CFRP', fn='Pressure recovery of underbody flow')
    place('JD-101-021', 'body', (-3.05, 0.25, 0), q_rot(('x', -90), ('z', -12)))
    part('JD-101-022', 'Diffuser strake', '101 Body', 'Aero devices', 'cfrp',
         lambda: extrude_xy([(0, 0), (0.55, 0), (0.55, 0.16), (0.1, 0.02)], 0.008, -0.004),
         process='Pre-preg CFRP')
    for z in (-0.6, -0.3, 0.0, 0.3, 0.6):
        place('JD-101-022', 'body', (-3.08, 0.12, z), q_rot(('z', -12)))
    # mirror / camera pods with pitot-static probe
    part('JD-101-030', 'Camera mirror pod (with pitot mount)', '101 Body', 'Exterior trim', 'cfrp_paint',
         lambda: union(S(cq.Workplane('YZ').ellipse(0.045, 0.07).extrude(0.16)).translate(V(-0.08, 0, 0)),
                       box(0.05, 0.03, 0.12, (0.02, -0.02, -0.08))),
         handed=True, process='Injection-moulded PC/ABS, painted', fn='Rear-view camera housing; carries pitot probe')
    for side, s in (('L', 1), ('R', -1)):
        place('JD-101-030', 'body', (0.95, 1.06, s * 1.10), q_rot(('y', 0)) if s > 0 else q_rot(('x', 180)))
    part('JD-101-031', 'Flush door handle', '101 Body', 'Exterior trim', 'al7075',
         lambda: union(box(0.16, 0.025, 0.012), box(0.03, 0.02, 0.03, (-0.06, 0, -0.015))),
         process='Machined, anodised', fn='Electrically presented door release')
    for side, s in (('L', 1), ('R', -1)):
        h = DOOR_HINGE[side]
        place('JD-101-031', f'door_{side}', (-0.55 - h[0], 0.98 - h[1], s * 1.13 - h[2]))
    part('JD-101-032', 'Charge-port door', '101 Body', 'Exterior trim', 'cfrp_paint',
         lambda: cyl(0.06, 0.006, 'z'), process='Painted CFRP, motorised hinge', fn='Covers CCS2 inlet')
    place('JD-101-032', 'body', (-2.35, 0.86, 1.155))
    part('JD-101-033', 'Wheel-arch / module pocket liner', '101 Body', 'Exterior trim', 'plastic',
         lambda: tube(0.84, 0.8375, 0.30, 'z').intersect(box(2, 1.0, 1, (0, 0.40, 0))), handed=False, process='Twin-sheet thermoformed TPO',
         fn='Splash / debris shield of the fan-wheel pocket')
    for cx in (WB, -WB):
        for s in (1, -1):
            place('JD-101-033', 'body', (cx, WC_Y, s * 0.80))
    # door internals
    part('JD-101-040', 'Gull-wing door inner frame', '101 Body', 'Doors', 'cfrp', handed=True,
         builder=lambda: union(box(1.9, 0.03, 0.03, (0, 0, 0)), box(1.9, 0.03, 0.03, (0, -0.55, 0.22)),
                               strut((-0.95, 0, 0), (-0.95, -0.55, 0.22), 0.016),
                               strut((0.95, 0, 0), (0.95, -0.55, 0.22), 0.016),
                               strut((0.0, 0, 0), (0.0, -0.55, 0.22), 0.012)),
         process='RTM CFRP hollow section', fn='Door structure carrying glass, skin, hinges and latch')
    part('JD-101-041', 'Gull-wing door hinge', '101 Body', 'Doors', 'ti64',
         lambda: union(cyl(0.018, 0.08, 'x'), box(0.06, 0.012, 0.07, (0, -0.012, 0.035))),
         process='Machined Ti-6Al-4V', fn='Roof-line hinge', criticality='Safety')
    part('JD-101-042', 'Door gas strut', '101 Body', 'Doors', 'steel',
         lambda: union(cyl(0.011, 0.26, 'x', (0.13, 0, 0), False), cyl(0.005, 0.18, 'x', (0.39, 0, 0), False)),
         supplier='buy', process='Purchased gas spring 600 N', fn='Door lift assist')
    part('JD-101-043', 'Door latch & striker', '101 Body', 'Doors', 'steel',
         lambda: union(box(0.06, 0.05, 0.03), cyl(0.008, 0.04, 'z', (0.0, -0.035, 0))),
         supplier='buy', process='Purchased e-latch, FMVSS 206', fn='Electrically released latch', criticality='Safety')
    for side, s in (('L', 1), ('R', -1)):
        n = f'door_{side}'
        place('JD-101-040', n, (0, -0.10, s * 0.25) if s > 0 else (0, -0.10, s * 0.25),
              (0, 0, 0, 1) if s > 0 else q_rot(('x', 180), ('x', 180)))
        for hx in (-0.5, 0.5):
            place('JD-101-041', n, (hx, 0, 0))
        place('JD-101-042', n, (-0.85, -0.25, s * 0.45), q_rot(('z', 30)))
        place('JD-101-043', n, (0.0, -0.62, s * 0.80))

# ============================================================== 102 GLAZING
def canopy_solid(inset=0.0):
    wires = []
    for s in SEC['canopy']:
        pts = s['yz']
        y0 = min(p[0] for p in pts)
        if inset:
            yc = y0
            pts = [(yc + (y - yc) * (1 - inset / max(0.05, y - yc + 0.05)), z * (1 - inset / 0.75)) for y, z in pts]
        vs = [V(s['x'], y, z) for y, z in pts]
        wires.append(cq.Wire.assembleEdges([cq.Edge.makeSpline(vs), cq.Edge.makeLine(vs[-1], vs[0])]))
    return cq.Solid.makeLoft(wires, False)

_GL = {}
def glazing(name):
    if not _GL:
        g = canopy_solid().cut(canopy_solid(0.012))
        _GL['ws'] = g.intersect(region(0.30, 2))
        rest = g.cut(region(0.30, 2))
        _GL['roof'] = rest.intersect(region(-3, 0.30, -0.40, 0.40))
        rest = rest.cut(region(-3, 0.30, -0.40, 0.40))
        _GL['L'] = rest.intersect(region(-3, 0.30, 0, 2))
        _GL['R'] = rest.intersect(region(-3, 0.30, -2, 0))
    return _GL[name]

def build_glazing():
    spec = dict(thickness_mm=12, coating='Hard-coat + hydrophobic + IR-reflective', bird_strike='TBD (CS-23 analogue)')
    part('JD-102-001', 'Windscreen', '102 Glazing', 'Canopy', 'glass', lambda: glazing('ws'), world=True,
         shell_t=0.006, spec=spec, process='Laminated polycarbonate, thermoformed', fn='Forward vision / bird-strike')
    part('JD-102-002', 'Roof glass', '102 Glazing', 'Canopy', 'glass', lambda: glazing('roof'), world=True,
         shell_t=0.006, process='Laminated polycarbonate', fn='Overhead visibility (hover / landing)')
    part('JD-102-003', 'Door glass, port', '102 Glazing', 'Canopy', 'glass', lambda: glazing('L'), world=True,
         shell_t=0.005, process='Laminated polycarbonate', fn='Side visibility')
    part('JD-102-004', 'Door glass, starboard', '102 Glazing', 'Canopy', 'glass', lambda: glazing('R'), world=True,
         shell_t=0.005, process='Laminated polycarbonate', fn='Side visibility')
    place('JD-102-001', 'body'); place('JD-102-002', 'body')
    for side in 'LR':
        h = DOOR_HINGE[side]
        place('JD-102-003' if side == 'L' else 'JD-102-004', f'door_{side}', (-h[0], -h[1], -h[2]))
    part('JD-102-010', 'Canopy perimeter seal', '102 Glazing', 'Canopy', 'rubber',
         lambda: strut((-1.5, 0, 0), (1.5, 0, 0), 0.008), process='Extruded EPDM', supplier='buy')
    place('JD-102-010', 'body', (-0.35, 1.02, 0.73)); place('JD-102-010', 'body', (-0.35, 1.02, -0.73))

# ============================================================== 105 LIGHTING
def build_lighting():
    part('JD-105-001', 'Headlamp housing', '105 Lighting', 'Front lamps', 'plastic', handed=True,
         builder=lambda: box(0.30, 0.07, 0.22).cut(box(0.28, 0.05, 0.20, (0.01, 0, 0))),
         process='Injection-moulded PBT', fn='Carries LED projectors and DRL')
    part('JD-105-002', 'Headlamp lens', '105 Lighting', 'Front lamps', 'lens', handed=True,
         builder=lambda: box(0.012, 0.07, 0.22), process='Injection-moulded PMMA, hard-coated')
    part('JD-105-003', 'LED projector module', '105 Lighting', 'Front lamps', 'electronic',
         lambda: union(cyl(0.022, 0.05, 'x', (0, 0, 0)), box(0.04, 0.05, 0.05, (-0.04, 0, 0))),
         supplier='buy', process='Purchased ECE R112 projector', fn='Low / high beam')
    part('JD-105-004', 'DRL / indicator light blade', '105 Lighting', 'Front lamps', 'lens',
         lambda: box(0.01, 0.012, 0.85), process='Light-guide PMMA')
    part('JD-105-005', 'Tail light bar', '105 Lighting', 'Rear lamps', 'lens',
         lambda: box(0.012, 0.03, 1.70), process='Light-guide PMMA + LED strip', fn='Tail / stop / indicator')
    part('JD-105-006', 'Navigation light (red / green)', '105 Lighting', 'Flight lighting', 'lens',
         lambda: sphere(0.02), supplier='buy', process='Purchased DO-160 nav light',
         fn='Aviation position lights: red port, green starboard')
    part('JD-105-007', 'Anti-collision strobe beacon', '105 Lighting', 'Flight lighting', 'lens',
         lambda: union(cyl(0.035, 0.012, 'y'), sphere(0.025, (0, 0.01, 0))), supplier='buy',
         fn='Anti-collision strobe (flight mode only)')
    part('JD-105-008', 'Landing / taxi light', '105 Lighting', 'Flight lighting', 'electronic',
         lambda: cyl(0.04, 0.03, 'y'), supplier='buy', fn='Down-facing landing light (hover)')
    for s in (1, -1):
        q = (0, 0, 0, 1) if s > 0 else q_rot(('x', 180))
        place('JD-105-001', 'body', (2.95, 0.48, s * 0.62), q)
        place('JD-105-002', 'body', (3.105, 0.48, s * 0.62), q)
        for k in range(3):
            place('JD-105-003', 'body', (3.0, 0.48, s * (0.54 + 0.07 * k)))
        place('JD-105-006', 'body', (3.02, 0.62, s * 0.90))
        place('JD-105-008', 'body', (s * 1.2, 0.205, 0))
    place('JD-105-004', 'body', (3.09, 0.58, 0))
    place('JD-105-005', 'body', (-3.11, 0.88, 0))
    place('JD-105-006', 'body', (-3.12, 0.95, 0))
    place('JD-105-007', 'body', (-0.85, 1.71, 0))

# ============================================================== 110 STRUCTURE
def build_structure():
    node('chassis', 'Primary structure', 'vehicle', system='110')
    def tub():
        outer = box(3.05, 0.62, 1.46, (-0.35, 0.53, 0))
        inner = box(2.95, 0.62, 1.30, (-0.35, 0.62, 0))
        t = outer.cut(inner)
        t = t.cut(box(1.6, 0.30, 0.2, (-0.35, 0.72, 0.70))).cut(box(1.6, 0.30, 0.2, (-0.35, 0.72, -0.70)))
        return t
    part('JD-110-001', 'Carbon monocoque tub', '110 Structure', 'Safety cell', 'cfrp', tub, shell_t=0.006,
         process='Pre-preg CFRP / Nomex honeycomb sandwich, autoclave', criticality='Primary structure',
         fn='Occupant survival cell; carries battery pack, seats, rollover hoop and all corner loads',
         spec=dict(laminate='[0/±45/90]s skins, 20 mm Nomex core', torsional_stiffness_target='> 30 kN·m/deg'))
    place('JD-110-001', 'chassis')

    def subframe(sign):
        x0 = 1.18 * sign
        x1 = 2.55 * sign
        n = [(x0, 0.30, 0.62), (x0, 0.30, -0.62), (x0, 0.80, 0.55), (x0, 0.80, -0.55),
             (x1, 0.30, 0.50), (x1, 0.30, -0.50), (x1, 0.70, 0.45), (x1, 0.70, -0.45)]
        members = [(0, 4), (1, 5), (2, 6), (3, 7), (0, 2), (1, 3), (4, 6), (5, 7), (4, 5), (6, 7),
                   (0, 6), (1, 7), (2, 3), (0, 5)]
        return union(*[strut(n[a], n[b], 0.025, 0.021) for a, b in members])
    part('JD-110-010', 'Front subframe weldment', '110 Structure', 'Subframes', 'al6061', lambda: subframe(1),
         process='TIG-welded 6061 tube, solution treated + aged', criticality='Primary structure',
         fn='Front suspension, steering rack, front fan-wheel conversion hinges, radiators')
    part('JD-110-020', 'Rear subframe weldment', '110 Structure', 'Subframes', 'al6061', lambda: subframe(-1),
         process='TIG-welded 6061 tube', criticality='Primary structure',
         fn='Rear suspension, rear conversion hinges, rear battery stack, parachute canister')
    place('JD-110-010', 'chassis'); place('JD-110-020', 'chassis')
    part('JD-110-030', 'Front crash box', '110 Structure', 'Crash structure', 'al6061',
         lambda: tube(0.06, 0.054, 0.38, 'x').cut(box(0.02, 0.2, 0.2, (0.05, 0, 0))).fuse(tube(0.06, 0.054, 0.16, 'x', (-0.12, 0, 0))),
         process='Extruded multi-cell 6061, crush initiators', fn='Low-speed / pedestrian energy absorption',
         criticality='Safety')
    part('JD-110-031', 'Front bumper beam', '110 Structure', 'Crash structure', 'al7075',
         lambda: S(cq.Workplane('YZ').rect(0.10, 1.40).extrude(0.06)).cut(S(cq.Workplane('YZ').rect(0.08, 1.38).extrude(0.06))),
         process='Roll-formed 7075 box section', criticality='Safety')
    part('JD-110-032', 'Rear bumper beam', '110 Structure', 'Crash structure', 'al7075',
         lambda: S(cq.Workplane('YZ').rect(0.10, 1.50).extrude(0.06)).cut(S(cq.Workplane('YZ').rect(0.08, 1.48).extrude(0.06))),
         process='Roll-formed 7075 box section', criticality='Safety')
    for s in (1, -1):
        place('JD-110-030', 'chassis', (2.75, 0.45, s * 0.42))
    place('JD-110-031', 'chassis', (2.94, 0.45, 0))
    place('JD-110-032', 'chassis', (-3.02, 0.47, 0))
    def hoop():
        pts = [(0, 0.80, 0.62), (0, 1.35, 0.60), (0, 1.62, 0.38), (0, 1.67, 0.0), (0, 1.62, -0.38),
               (0, 1.35, -0.60), (0, 0.80, -0.62)]
        return union(*[strut(a, b, 0.024, 0.019) for a, b in zip(pts, pts[1:])],
                     *[sphere(0.026, p) for p in pts[1:-1]])
    part('JD-110-040', 'Roll-over hoop', '110 Structure', 'Safety cell', 'ti64', hoop,
         process='Mandrel-bent Ti-6Al-4V tube, orbital welded', criticality='Safety',
         fn='Roll-over / inverted-landing protection; parachute harness anchor')
    place('JD-110-040', 'chassis', (-1.45, 0, 0))
    def apillar():
        pts = [(1.12, 0.95, 0.70), (0.80, 1.25, 0.68), (0.30, 1.55, 0.55), (-0.30, 1.65, 0.45)]
        return union(*[strut(a, b, 0.018, 0.014) for a, b in zip(pts, pts[1:])])
    part('JD-110-041', 'A-pillar / canopy frame tube', '110 Structure', 'Safety cell', 'ti64', apillar,
         handed=True, process='Hydroformed Ti tube', criticality='Safety', fn='Canopy frame, windscreen support')
    place('JD-110-041', 'chassis')
    node('chassis_R', 'Primary structure (starboard mirror)', 'chassis', mirror=True, system='110')
    place('JD-110-041', 'chassis_R')
    part('JD-110-050', 'Corner hardpoint bracket', '110 Structure', 'Hardpoints', 'al7075', handed=True,
         builder=lambda: box(0.22, 0.42, 0.02).cut(cyl(0.025, 0.1, 'z', (0.05, 0.12, 0))).cut(cyl(0.025, 0.1, 'z', (0.05, -0.12, 0))),
         process='5-axis machined 7075-T6', criticality='Primary structure',
         fn='Picks up wishbone, conversion-hinge and leg loads into the subframe')
    for cx in (WB, -WB):
        for s in (1, -1):
            place('JD-110-050', 'chassis' if s > 0 else 'chassis_R', (cx, 0.62, 0.66))
    part('JD-110-060', 'Seat rail', '110 Structure', 'Interior structure', 'al6061',
         lambda: S(cq.Workplane('YZ').polyline([(0, 0), (0.03, 0), (0.03, 0.004), (0.004, 0.004), (0.004, 0.035), (0, 0.035)]).close().extrude(0.60)),
         process='Extruded 6061', supplier='make')
    for z in (0.22, 0.48, -0.22, -0.48):
        place('JD-110-060', 'chassis', (-1.05, 0.46, z))
    part('JD-110-070', 'Front bulkhead / firewall', '110 Structure', 'Safety cell', 'cfrp',
         lambda: box(0.012, 0.55, 1.30), shell_t=None, process='Pre-preg CFRP flat laminate')
    part('JD-110-071', 'Rear bulkhead (battery stack wall)', '110 Structure', 'Safety cell', 'cfrp',
         lambda: box(0.012, 0.75, 1.30), process='Pre-preg CFRP flat laminate')
    place('JD-110-070', 'chassis', (1.17, 0.55, 0)); place('JD-110-071', 'chassis', (-1.55, 0.62, 0))
    part('JD-110-090', 'Landing-leg mount bracket', '110 Structure', 'Hardpoints', 'al7075',
         lambda: tube(0.07, 0.052, 0.10, 'y').fuse(box(0.20, 0.02, 0.20, (0, 0.05, 0))),
         process='Machined 7075-T6', criticality='Primary structure')
    for lx in (1.30, -1.30):
        for lz in (0.60, -0.60):
            place('JD-110-090', 'chassis', (lx, 0.29, lz))
    part('JD-110-095', 'Tie-down / jacking point', '110 Structure', 'Hardpoints', 'steel_stn',
         lambda: union(cyl(0.025, 0.03, 'y'), tube(0.02, 0.012, 0.012, 'x', (0, -0.03, 0))),
         process='Machined 17-4PH', fn='Ground handling, aircraft tie-down')
    for lx in (2.2, -2.2):
        for lz in (0.5, -0.5):
            place('JD-110-095', 'chassis', (lx, 0.21, lz))

# ============================================================== 120 COCKPIT
def build_cockpit():
    node('cockpit', 'Cockpit & interior', 'vehicle', system='120')
    def seat_shell():
        base = box(0.50, 0.05, 0.48, (0, 0, 0))
        back = rot(box(0.06, 0.70, 0.48, (0, 0.35, 0)), 'z', 18, (0, 0, 0)).translate(V(-0.26, 0.0, 0))
        side = union(box(0.46, 0.14, 0.03, (0, 0.06, 0.24)), box(0.46, 0.14, 0.03, (0, 0.06, -0.24)))
        return union(base, back, side)
    part('JD-120-001', 'Seat shell', '120 Cockpit', 'Seats', 'cfrp', seat_shell, shell_t=0.004,
         process='Pre-preg CFRP shell, FIA 8862-style geometry', criticality='Safety',
         fn='Occupant restraint shell (road + 14 g vertical crash target)')
    part('JD-120-002', 'Seat cushion', '120 Cockpit', 'Seats', 'foam',
         lambda: box(0.44, 0.06, 0.40), process='Moulded energy-absorbing foam, Alcantara')
    part('JD-120-003', 'Seat back pad', '120 Cockpit', 'Seats', 'foam',
         lambda: box(0.05, 0.56, 0.40), process='Moulded foam, Alcantara')
    part('JD-120-004', 'Headrest', '120 Cockpit', 'Seats', 'foam', lambda: box(0.10, 0.14, 0.24),
         process='Moulded foam')
    part('JD-120-005', '5-point harness assembly', '120 Cockpit', 'Seats', 'textile',
         lambda: union(box(0.004, 0.62, 0.05, (0.02, 0.0, 0.12)), box(0.004, 0.62, 0.05, (0.02, 0.0, -0.12)),
                       box(0.40, 0.004, 0.05, (0.2, -0.31, 0.15)), box(0.40, 0.004, 0.05, (0.2, -0.31, -0.15))),
         supplier='buy', process='Purchased FIA 8853 harness', criticality='Safety')
    part('JD-120-006', 'Harness cam-lock buckle', '120 Cockpit', 'Seats', 'al7075',
         lambda: cyl(0.04, 0.02, 'x'), supplier='buy', criticality='Safety')
    part('JD-120-007', 'Seat slider carriage', '120 Cockpit', 'Seats', 'steel', lambda: box(0.10, 0.03, 0.04),
         supplier='buy')
    for s in (1, -1):
        z = s * 0.35
        place('JD-120-001', 'cockpit', (-0.95, 0.50, z))
        place('JD-120-002', 'cockpit', (-0.92, 0.55, z))
        place('JD-120-003', 'cockpit', (-1.13, 0.86, z), q_rot(('z', 18)))
        place('JD-120-004', 'cockpit', (-1.29, 1.27, z), q_rot(('z', 18)))
        place('JD-120-005', 'cockpit', (-1.08, 0.90, z), q_rot(('z', 18)))
        place('JD-120-006', 'cockpit', (-0.78, 0.66, z))
        for sz in (-0.13, 0.13):
            for sx in (-1.15, -0.80):
                place('JD-120-007', 'cockpit', (sx, 0.495, z + sz))
    part('JD-120-010', 'Dashboard carrier beam', '120 Cockpit', 'Dash', 'al_cast',
         lambda: union(strut((0, 0, -0.68), (0, 0, 0.68), 0.03, 0.025), box(0.08, 0.30, 0.04, (0, -0.15, 0.0))),
         process='Vacuum-cast magnesium-alternative A356', fn='Mounts display, HUD, yoke column and airbags')
    place('JD-120-010', 'cockpit', (0.45, 0.98, 0))
    part('JD-120-011', 'Pillar-to-pillar display', '120 Cockpit', 'Dash', 'electronic',
         lambda: union(box(0.03, 0.12, 1.22), box(0.004, 0.11, 1.18, (0.017, 0, 0))), supplier='buy',
         process='Purchased curved OLED, DO-160 qualified', fn='PFD / MFD / vehicle status, mode-dependent layout')
    place('JD-120-011', 'cockpit', (0.35, 1.07, 0), q_rot(('z', 15)))
    part('JD-120-012', 'HUD projector unit', '120 Cockpit', 'Dash', 'electronic', lambda: box(0.20, 0.08, 0.18),
         supplier='buy', fn='Head-up flight / navigation symbology')
    part('JD-120-013', 'HUD combiner glass', '120 Cockpit', 'Dash', 'glass', lambda: box(0.004, 0.12, 0.22),
         supplier='buy')
    place('JD-120-012', 'cockpit', (0.52, 1.02, 0.35)); place('JD-120-013', 'cockpit', (0.40, 1.13, 0.35), q_rot(('z', -30)))
    part('JD-120-020', 'Steering yoke', '120 Cockpit', 'Controls', 'cfrp',
         lambda: union(box(0.03, 0.05, 0.34), box(0.03, 0.18, 0.04, (0, -0.07, 0.17)), box(0.03, 0.18, 0.04, (0, -0.07, -0.17)),
                       cyl(0.045, 0.05, 'x')),
         process='CFRP + machined Al core, Alcantara grip', fn='Road steering (steer-by-wire); inhibited in flight')
    part('JD-120-021', 'Steer-by-wire column / force-feedback actuator', '120 Cockpit', 'Controls', 'electronic',
         lambda: union(cyl(0.045, 0.22, 'x'), cyl(0.02, 0.12, 'x', (0.15, 0, 0))), supplier='buy',
         criticality='Safety', fn='Dual-wound feedback motor, triple-redundant angle sensing')
    place('JD-120-020', 'cockpit', (0.22, 0.95, 0.35), q_rot(('z', 20)))
    place('JD-120-021', 'cockpit', (0.36, 0.92, 0.35), q_rot(('z', 20)))
    part('JD-120-022', 'Flight sidestick inceptor', '120 Cockpit', 'Controls', 'electronic',
         lambda: union(box(0.07, 0.06, 0.07), cyl(0.016, 0.14, 'y', (0, 0.10, 0)), sphere(0.022, (0, 0.18, 0))),
         supplier='buy', criticality='Flight critical', fn='Pitch / roll (and yaw twist) rate command in flight')
    place('JD-120-022', 'cockpit', (-0.55, 0.78, 0.05))
    part('JD-120-023', 'Thrust / collective lever', '120 Cockpit', 'Controls', 'electronic',
         lambda: union(box(0.18, 0.05, 0.06), rot(cyl(0.014, 0.16, 'y', (0, 0.08, 0)), 'z', -30), sphere(0.02, (0.08, 0.14, 0))),
         supplier='buy', criticality='Flight critical', fn='Vertical speed / thrust command')
    place('JD-120-023', 'cockpit', (-0.60, 0.76, 0.62))
    part('JD-120-024', 'Pedal box', '120 Cockpit', 'Controls', 'al7075', lambda: box(0.20, 0.10, 0.30),
         process='Machined 7075', fn='Accelerator / brake (road); yaw (flight)')
    part('JD-120-025', 'Pedal', '120 Cockpit', 'Controls', 'al7075',
         lambda: union(box(0.015, 0.14, 0.08), cyl(0.008, 0.10, 'z', (0, -0.07, 0))), process='Machined 7075')
    place('JD-120-024', 'cockpit', (0.95, 0.52, 0.35))
    place('JD-120-025', 'cockpit', (0.88, 0.60, 0.29)); place('JD-120-025', 'cockpit', (0.88, 0.60, 0.42))
    part('JD-120-030', 'Centre console', '120 Cockpit', 'Console', 'cfrp',
         lambda: box(0.90, 0.20, 0.18).cut(box(0.86, 0.2, 0.14, (0, 0.03, 0))), process='CFRP + leather')
    place('JD-120-030', 'cockpit', (-0.30, 0.64, 0))
    part('JD-120-031', 'Drive / flight mode selector', '120 Cockpit', 'Console', 'al7075',
         lambda: union(cyl(0.035, 0.03, 'y'), box(0.09, 0.012, 0.09, (0, -0.02, 0))), criticality='Safety',
         process='Machined knob, guarded two-stage', fn='ROAD / PREPARE / CONFIGURE / FLY request (supervised)')
    place('JD-120-031', 'cockpit', (-0.05, 0.76, 0))
    part('JD-120-032', 'Ballistic parachute deploy handle', '120 Cockpit', 'Console', 'steel',
         lambda: union(box(0.10, 0.012, 0.02), strut((0, 0, 0), (0, -0.04, 0), 0.006)), criticality='Safety',
         supplier='buy', process='Purchased (BRS-type) pull-handle, safety pin')
    place('JD-120-032', 'cockpit', (0.05, 1.45, 0), q_rot(('x', 180)))
    part('JD-120-033', 'Overhead switch panel', '120 Cockpit', 'Console', 'electronic',
         lambda: box(0.30, 0.03, 0.22), supplier='make', fn='Avionics master, lights, beacons, emergency')
    place('JD-120-033', 'cockpit', (-0.25, 1.60, 0))
    part('JD-120-034', 'Door trim panel', '120 Cockpit', 'Trim', 'plastic', handed=True,
         builder=lambda: box(1.2, 0.40, 0.02), process='Injection-moulded PP, Alcantara wrap')
    for side, s in (('L', 1), ('R', -1)):
        h = DOOR_HINGE[side]
        place('JD-120-034', f'door_{side}', (-0.15 - h[0], 0.80 - h[1], s * 0.70 - h[2]))
    part('JD-120-035', 'HVAC face vent', '120 Cockpit', 'Trim', 'plastic',
         lambda: box(0.03, 0.04, 0.10).cut(box(0.03, 0.03, 0.08)), supplier='buy')
    for z in (-0.6, -0.15, 0.15, 0.6):
        place('JD-120-035', 'cockpit', (0.40, 1.00, z))
    part('JD-120-036', 'Airbag module', '120 Cockpit', 'Restraints', 'electronic', lambda: box(0.10, 0.08, 0.20),
         supplier='buy', criticality='Safety', fn='Frontal / knee / canopy-curtain airbag')
    for z in (-0.35, 0.35):
        place('JD-120-036', 'cockpit', (0.30, 0.90, z)); place('JD-120-036', 'cockpit', (0.55, 0.65, z))

# ============================================================== 200 ENERGY STORAGE & POWER
CELL_D, CELL_H, CELL_PITCH = 0.046, 0.080, 0.0485
MOD_NX, MOD_NZ, MOD_LAYERS = 8, 7, 1
def build_power():
    node('power', 'Energy storage & power electronics', 'vehicle', system='200')
    part('JD-200-001', 'Battery enclosure lower tray', '200 Energy storage', 'Pack enclosure', 'al6061',
         lambda: box(2.20, 0.13, 1.30).cut(box(2.16, 0.13, 1.26, (0, 0.02, 0))), shell_t=0.004,
         process='Friction-stir-welded 6061 extrusions, IP67', criticality='Safety',
         fn='Structural floor pack enclosure, side-impact and thermal-runaway barrier')
    part('JD-200-002', 'Battery enclosure lid', '200 Energy storage', 'Pack enclosure', 'cfrp',
         lambda: box(2.20, 0.003, 1.30), process='Mica-lined CFRP lid', criticality='Safety')
    place('JD-200-001', 'power', (0, 0.29, 0)); place('JD-200-002', 'power', (0, 0.362, 0))
    part('JD-200-010', 'Battery module housing', '200 Energy storage', 'Battery modules', 'al6061',
         lambda: box(0.40, 0.105, 0.36).cut(box(0.392, 0.105, 0.352, (0, 0.004, 0))), process='Extruded / welded 6061')
    part('JD-200-011', 'Li-ion cell, 4680 format', '200 Energy storage', 'Battery modules', 'cell',
         lambda: union(cyl(CELL_D / 2, CELL_H - 0.003, 'y'), cyl(0.008, 0.003, 'y', (0, CELL_H / 2 - 0.0015 + 0.0015, 0))),
         supplier='buy', mass=0.355, process='Purchased cylindrical cell, tabless',
         spec=dict(nominal_V=3.6, capacity_Ah=26.0, energy_Wh=93.6, chemistry='NMC811 / Si-graphite'))
    part('JD-200-012', 'Module cold plate', '200 Energy storage', 'Battery modules', 'al6061',
         lambda: box(0.39, 0.004, 0.35), process='Roll-bonded aluminium cold plate')
    part('JD-200-013', 'Module busbar', '200 Energy storage', 'Battery modules', 'copper',
         lambda: box(0.38, 0.003, 0.025), process='Laser-cut C11000, nickel plated')
    part('JD-200-014', 'Module BMS slave board', '200 Energy storage', 'Battery modules', 'pcb',
         lambda: box(0.12, 0.012, 0.08), process='SMT FR-4, conformal coated', fn='Cell voltage / temperature sensing, balancing')
    part('JD-200-015', 'Module end plate', '200 Energy storage', 'Battery modules', 'plastic',
         lambda: box(0.006, 0.095, 0.35), process='Glass-filled PA66, UL94 V-0')
    mods = []
    for i in range(5):
        for k in range(3):
            mods.append(((-0.84 + 0.42 * i, 0.250, -0.40 + 0.40 * k), 'floor'))
    for i in range(2):
        for k in range(2):
            mods.append(((-2.36 + 0.42 * i, 0.50, -0.20 + 0.40 * k), 'rear'))
    for m, (pos, where) in enumerate(mods):
        nid = f'bmod_{m:02d}'
        node(nid, f'Battery module {m + 1:02d} ({where})', 'power', pos=pos, system='200')
        place('JD-200-010', nid, (0, 0.0525, 0))
        place('JD-200-012', nid, (0, 0.006, 0))
        place('JD-200-014', nid, (0, 0.108, 0.08))
        place('JD-200-013', nid, (0, 0.100, -0.12)); place('JD-200-013', nid, (0, 0.100, 0.12))
        place('JD-200-015', nid, (0.197, 0.0525, 0)); place('JD-200-015', nid, (-0.197, 0.0525, 0))
        for L in range(MOD_LAYERS):
            for a in range(MOD_NX):
                for b in range(MOD_NZ):
                    place('JD-200-011', nid, (-0.17 + a * CELL_PITCH, 0.051 + L * (CELL_H + 0.006), -0.146 + b * CELL_PITCH))
    elec = [
        ('JD-210-001', 'Battery disconnect unit (BDU)', (0.30, 0.14, 0.25), (1.00, 0.55, 0.0), 'Main contactors, pre-charge, current sensing'),
        ('JD-210-002', 'Main HV contactor', (0.07, 0.09, 0.06), None, 'Isolates pack (+/-)'),
        ('JD-210-003', 'Pyro-fuse', (0.05, 0.05, 0.10), None, 'Crash / short-circuit pyrotechnic disconnect'),
        ('JD-210-004', 'Pre-charge relay & resistor', (0.06, 0.05, 0.08), None, 'Limits inrush to inverter DC links'),
        ('JD-210-005', 'Hall-effect current sensor', (0.05, 0.06, 0.03), None, 'Pack current for SOC / protection'),
        ('JD-210-006', 'HV junction box', (0.26, 0.10, 0.30), (1.45, 0.62, 0.0), 'Fused distribution to 4 inverters, DC-DC, OBC, HVAC'),
        ('JD-210-007', 'DC-DC converter 800 V / 14 V', (0.22, 0.07, 0.18), (1.90, 0.52, -0.25), 'Low-voltage supply 3 kW'),
        ('JD-210-008', 'On-board charger 22 kW', (0.28, 0.09, 0.22), (1.90, 0.52, 0.25), 'AC charging'),
        ('JD-210-009', '12 V LFP auxiliary battery', (0.18, 0.16, 0.14), (2.20, 0.55, 0.0), 'Keep-alive, flight-critical LV backup'),
        ('JD-210-010', 'Insulation monitoring device', (0.10, 0.04, 0.08), (1.45, 0.70, 0.30), 'HV isolation fault detection'),
        ('JD-210-011', 'Manual service disconnect', (0.08, 0.10, 0.08), (-0.30, 0.62, 0.0), 'Lock-out for maintenance'),
        ('JD-210-012', 'CCS2 charge inlet', (0.08, 0.10, 0.10), (-2.35, 0.86, 1.10), 'DC fast charge up to 350 kW'),
    ]
    for pn, name, dims, pos, fn in elec:
        part(pn, name, '210 Power distribution', 'HV distribution', 'electronic',
             (lambda d=dims: box(*d)), supplier='buy' if 'unit' not in name.lower() else 'make', fn=fn,
             criticality='Safety', process='Purchased / qualified HV component' if pos is None or 'CCS' in name else 'Cast Al housing, potted')
        if pos:
            place(pn, 'power', pos)
    for k, z in enumerate((-0.08, 0.08)):
        place('JD-210-002', 'power', (1.00, 0.64, z))
    place('JD-210-003', 'power', (0.92, 0.64, 0.0))
    place('JD-210-004', 'power', (1.08, 0.64, 0.0))
    place('JD-210-005', 'power', (1.12, 0.64, 0.08))
    part('JD-211-001', 'Traction / fan inverter (SiC, 250 kW)', '210 Power distribution', 'Inverters', 'electronic',
         lambda: union(box(0.32, 0.09, 0.22), *[box(0.30, 0.012, 0.004, (0, 0.05, -0.09 + 0.02 * i)) for i in range(10)]),
         process='Cast Al housing, SiC half-bridges, liquid-cooled', criticality='Flight critical',
         fn='One per fan-wheel motor: road torque and flight RPM control')
    for cx in (1.45, -1.45):
        for s in (1, -1):
            place('JD-211-001', 'power', (cx, 0.98 if cx > 0 else 0.92, s * 0.40))
    def hv_run(points):
        return lambda: sweep_circle(points, 0.012)
    runs = {
        'JD-212-001': ('HV cable, JB → FL inverter', [(1.45, 0.62, 0.15), (1.45, 0.80, 0.3), (1.45, 0.95, 0.40)]),
        'JD-212-002': ('HV cable, JB → FR inverter', [(1.45, 0.62, -0.15), (1.45, 0.80, -0.3), (1.45, 0.95, -0.40)]),
        'JD-212-003': ('HV cable, JB → RL inverter', [(1.45, 0.62, 0.12), (0.4, 0.47, 0.66), (-0.8, 0.47, 0.66), (-1.45, 0.88, 0.40)]),
        'JD-212-004': ('HV cable, JB → RR inverter', [(1.45, 0.62, -0.12), (0.4, 0.47, -0.66), (-0.8, 0.47, -0.66), (-1.45, 0.88, -0.40)]),
        'JD-212-005': ('HV cable, BDU → JB', [(1.00, 0.55, 0.0), (1.20, 0.60, 0.0), (1.45, 0.62, 0.0)]),
        'JD-212-006': ('HV cable, rear stack → BDU', [(-1.6, 0.58, 0.0), (-0.5, 0.47, 0.05), (0.85, 0.50, 0.0)]),
        'JD-212-007': ('HV cable, CCS2 inlet → BDU', [(-2.30, 0.86, 1.05), (-2.0, 0.70, 0.75), (-1.0, 0.47, 0.70), (0.9, 0.52, 0.1)]),
    }
    for pn, (name, pts) in runs.items():
        part(pn, name, '210 Power distribution', 'HV harness', 'hv_cable', hv_run(pts), world=True,
             process='Shielded 95 mm² XLPE, orange, HV-interlock', criticality='Safety')
        place(pn, 'power')

# ============================================================== 300 FAN-WHEEL / PROPROTOR MODULE
N_VANES, N_PETALS, N_TREAD, N_WEB = 7, 12, 72, 48
Z_ROTOR = 0.03
def blade_shape(hand='R'):
    st = PR.blade_stations(D, 11)
    wires = []
    def circle_pts(x, r, n=46):
        return [(x, r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n)) for i in range(n)]
    wires.append(spline_wire(circle_pts(0.055, 0.019)))
    wires.append(spline_wire(circle_pts(0.085, 0.019)))
    for r, c, beta, tc in st:
        af = naca4(D['camber'] + '%02d' % max(6, min(40, round(tc * 100))), 24)[:-1]
        b = math.radians(beta)
        pts = []
        for xc, yc in af:
            yy = (0.25 - xc) * c
            zz = yc * c
            pts.append((r, yy * math.cos(b) - zz * math.sin(b), yy * math.sin(b) + zz * math.cos(b)))
        wires.append(spline_wire(pts))
    s = cq.Solid.makeLoft(wires, False)
    if hand == 'L':
        s = s.mirror('XZ')
    return s

def duct_shape():
    prof = [(0.636, -0.16), (0.636, 0.155), (0.628, 0.166), (0.612, 0.170), (0.596, 0.166), (0.582, 0.154),
            (0.570, 0.132), (0.561, 0.100), (0.557, 0.06), (DUCT_R, 0.02), (DUCT_R, -0.10), (0.559, -0.135),
            (0.564, -0.16)]
    return revolve(prof)

def tread_band():
    pr = [(0.722, -0.15), (0.728, -0.15), (0.736, -0.142), (0.740, -0.125), (0.740, 0.125), (0.736, 0.142),
          (0.728, 0.15), (0.722, 0.15)]
    return revolve(pr)

def web_spoke():
    pts = [(0.636, -0.004), (0.678, 0.008), (0.723, 0.026), (0.723, 0.031), (0.678, 0.013), (0.636, 0.001)]
    return extrude_xy(pts, 0.26, -0.13)

def vane_shape():
    af = naca4('0010', 20)[:-1]
    c = 0.075
    pts = [(yc * c, (0.5 - xc) * c) for xc, yc in af]
    v = S(cq.Workplane('YZ').workplane(offset=0.168).polyline(pts).close().extrude(DUCT_R - 0.168 + 0.004))
    return v

def petal_shape(r_h, r_in, half_deg=15.0, t=0.004):
    h = math.tan(math.radians(half_deg))
    pts = [(0, -r_h * h * 0.995), (0, r_h * h * 0.995), (-(r_h - r_in), r_in * h), (-(r_h - r_in), -r_in * h)]
    return union(extrude_xy(pts, t, -t / 2), cyl(0.006, 2 * r_h * h * 0.9, 'y'))

def build_corner_parts():
    """Unique part definitions shared by all four corners."""
    sysm = '300 Fan-wheel module'
    part('JD-300-001', 'Tread band (airless tyre)', sysm, 'Road tyre', 'rubber', tread_band,
         process='Moulded SBR tread on steel-belted shear band', fn='Road contact; forms the duct outer fairing in flight',
         spec=dict(size='1480 × 300 airless', load_index='~105 (≥ 950 kg)', speed='≥ 200 km/h'))
    part('JD-300-002', 'Tread block', sysm, 'Road tyre', 'rubber', lambda: box(0.045, 0.018, 0.12),
         process='Integral with tread band moulding (modelled separately per A1 baseline)')
    part('JD-300-003', 'Airless shear-web spoke', sysm, 'Road tyre', 'pu', web_spoke,
         process='Cast thermoplastic PU web', fn='Load path tread band → rim with designed radial compliance')
    part('JD-300-010', 'Duct shroud / rim', sysm, 'Duct & rim', 'cfrp', duct_shape, shell_t=-0.0045,
         process='Filament-wound CFRP with Ti leading-edge insert', criticality='Flight critical',
         fn='Road rim in drive mode; ducted-fan shroud with bell-mouth inlet in flight',
         spec=dict(throat_diameter_mm=1112, tip_gap_mm=16, inlet_lip_radius_mm=42, exit_area_ratio=1.03))
    part('JD-300-011', 'Duct lip erosion shield', sysm, 'Duct & rim', 'ti64',
         lambda: revolve([(0.565, 0.13), (0.580, 0.155), (0.598, 0.168), (0.614, 0.171), (0.612, 0.168),
                          (0.596, 0.163), (0.581, 0.150), (0.568, 0.128)]),
         process='Hydroformed Ti-6Al-4V', fn='Rain / sand erosion protection of the inlet lip')
    part('JD-300-012', 'Rim lock receiver', sysm, 'Duct & rim', 'steel_stn',
         lambda: tube(0.018, 0.009, 0.03, 'z'), process='Machined 17-4PH, nitrided', criticality='Flight critical',
         fn='Receives conversion lock pin; indexes the rim in flight mode')
    part('JD-300-013', 'Stator vane / wheel spoke', sysm, 'Stator', 'cfrp', vane_shape,
         process='Compression-moulded CFRP, NACA 0010', criticality='Flight critical',
         fn='Wheel spoke (road); outlet guide vane that removes rotor swirl (flight)')
    part('JD-300-014', 'Vane hub ring', sysm, 'Stator', 'al7075', lambda: tube(0.172, 0.150, 0.09, 'z', (0, 0, -0.085)),
         process='Machined 7075-T6', fn='Ties vanes; outer race seat of the rim bearing')
    part('JD-300-015', 'Rim bearing (thin-section 4-point contact)', sysm, 'Stator', 'bearing',
         lambda: tube(0.149, 0.137, 0.035, 'z', (0, 0, -0.09)), supplier='buy',
         process='Purchased thin-section bearing (Kaydon-type)', criticality='Flight critical')
    part('JD-300-016', 'Carbon-ceramic brake ring', sysm, 'Brakes', 'ceramic', lambda: tube(0.630, 0.566, 0.012, 'z'),
         process='C/SiC brake ring, cross-drilled', criticality='Safety',
         fn='Road friction brake; clamped as a rim lock in flight mode')
    part('JD-300-020', 'Iris petal, outboard (inlet collector)', sysm, 'Petal covers', 'cfrp',
         lambda: petal_shape(0.600, 0.100), process='Pre-preg CFRP, 3 mm', criticality='Safety',
         fn='Closed: wheel cover / blade guard. Open (100°): inlet collector extension')
    part('JD-300-021', 'Iris petal, inboard (exit diffuser)', sysm, 'Petal covers', 'cfrp',
         lambda: petal_shape(0.556, 0.140), process='Pre-preg CFRP, 3 mm', criticality='Safety',
         fn='Closed: wheel cover. Open (95°): exit diffuser extension')
    part('JD-300-022', 'Petal hinge actuator (SMA / micro-servo)', sysm, 'Petal covers', 'electronic',
         lambda: box(0.03, 0.02, 0.015), supplier='buy', fn='Opens / closes one petal; position switch')
    # rotor
    part('JD-330-001', 'Proprotor blade, RH', '330 Proprotor', 'Rotor', 'cfrp', lambda: blade_shape('R'),
         process='Pre-preg CFRP spar + foam core, Ni leading-edge sheath', criticality='Flight critical',
         fn='NASA TM-20240002116 design: quadratic chord, inflow-aligned twist, decreasing t/c',
         spec=dict(R_mm=D['R'] * 1000, r0_R=D['r0_R'], B=D['B'], sigma=D['sigma'], r_m=D['r_m'], c2=D['c2'],
                   mu_design=D['mu_design'], airfoils='NACA 24xx, t/c 0.30 → 0.08', rotation='CCW (frame +Z)'))
    part('JD-330-002', 'Proprotor blade, LH', '330 Proprotor', 'Rotor', 'cfrp', lambda: blade_shape('L'),
         process='Pre-preg CFRP spar + foam core, Ni leading-edge sheath', criticality='Flight critical',
         fn='Opposite-rotation blade for diagonal torque balance', spec=dict(rotation='CW (frame +Z)'))
    part('JD-330-010', 'Rotor hub body', '330 Proprotor', 'Hub', 'ti64',
         lambda: polar_cut(cyl(0.095, 0.08, 'z', (0, 0, Z_ROTOR)), cyl(0.029, 0.2, 'x', (0.1, 0, Z_ROTOR)), D['B']),
         process='Forged + machined Ti-6Al-4V', criticality='Flight critical', fn=f"Carries {D['B']} blade grips; torque from motor flange")
    part('JD-330-011', 'Blade grip', '330 Proprotor', 'Hub', 'ti64',
         lambda: tube(0.028, 0.0195, 0.05, 'x', (0.08, 0, 0)).fuse(tube(0.034, 0.0195, 0.008, 'x', (0.064, 0, 0))),
         process='Machined Ti-6Al-4V', criticality='Flight critical')
    part('JD-330-012', 'Pitch bearing (angular contact pair)', '330 Proprotor', 'Hub', 'bearing',
         lambda: tube(0.0285, 0.0195, 0.012, 'x'), supplier='buy', criticality='Flight critical')
    part('JD-330-013', 'Pitch horn', '330 Proprotor', 'Hub', 'ti64',
         lambda: union(box(0.012, 0.035, 0.008, (0, 0.03, 0)), cyl(0.006, 0.012, 'z', (0, 0.045, 0))),
         process='Machined Ti-6Al-4V')
    part('JD-330-014', 'Pitch link', '330 Proprotor', 'Hub', 'steel_stn',
         lambda: union(strut((0, 0, 0), (0, 0, -0.05), 0.0035), sphere(0.006), sphere(0.006, (0, 0, -0.05))),
         process='Machined 17-4PH with rod-end bearings')
    part('JD-330-015', 'Collective spider', '330 Proprotor', 'Hub', 'al7075',
         lambda: union(cyl(0.03, 0.012, 'z'), *[rot(box(0.05, 0.012, 0.008, (0.045, 0, 0)), 'z', 360.0 / D['B'] * k) for k in range(D['B'])]),
         process='Machined 7075-T6')
    part('JD-330-016', 'Collective pitch actuator (EMA)', '330 Proprotor', 'Hub', 'electronic',
         lambda: union(cyl(0.025, 0.06, 'z'), cyl(0.008, 0.03, 'z', (0, 0, 0.04))), supplier='buy',
         criticality='Flight critical', fn='Sets collective: 90° feather (road) → hover → cruise')
    part('JD-330-017', 'Spinner', '330 Proprotor', 'Hub', 'cfrp',
         lambda: revolve([(0, 0.075), (0.097, 0.075), (0.090, 0.10), (0.070, 0.125), (0.040, 0.140), (0, 0.145)], spline=False),
         process='Pre-preg CFRP')
    part('JD-330-018', 'Hub output flange', '330 Proprotor', 'Hub', 'ti64',
         lambda: cyl(0.075, 0.012, 'z', (0, 0, -0.012)), process='Machined Ti-6Al-4V', criticality='Flight critical')
    part('JD-330-019', 'Blade retention nut', '330 Proprotor', 'Hub', 'steel_stn',
         lambda: rot(hexprism(0.034, 0.012), 'y', 90), process='Machined 17-4PH, safety-wired')
    # motor (axial flux) and fixed structure
    part('JD-320-001', 'Motor housing (axial flux)', '320 Motor', 'Motor', 'al_cast',
         lambda: tube(0.135, 0.127, 0.13, 'z', (0, 0, -0.11)).fuse(cyl(0.135, 0.008, 'z', (0, 0, -0.171))),
         process='Cast A356 with integral cooling jacket', criticality='Flight critical',
         spec=dict(type='Yokeless axial flux, dual rotor', peak_kW=230, cont_kW=165, peak_Nm=520, mass_kg_target=26))
    part('JD-320-002', 'Stator core segment set', '320 Motor', 'Motor', 'sisteel',
         lambda: tube(0.122, 0.060, 0.034, 'z', (0, 0, -0.10)), process='Soft-magnetic composite / laminated segments')
    part('JD-320-003', 'Stator coil (concentrated winding)', '320 Motor', 'Motor', 'copper',
         lambda: box(0.050, 0.020, 0.036).cut(box(0.034, 0.008, 0.04)), process='Rectangular Cu wire, potted')
    part('JD-320-004', 'Rotor disc', '320 Motor', 'Motor', 'steel',
         lambda: tube(0.124, 0.040, 0.008, 'z'), process='Machined 4340 back-iron disc', criticality='Flight critical')
    part('JD-320-005', 'NdFeB magnet segment', '320 Motor', 'Motor', 'ndfeb',
         lambda: box(0.040, 0.022, 0.006), supplier='buy', process='Purchased N42SH segment, epoxy-bonded')
    part('JD-320-006', 'Motor shaft', '320 Motor', 'Motor', 'steel', lambda: tube(0.040, 0.030, 0.17, 'z', (0, 0, -0.09)),
         process='Machined 4340, induction hardened', criticality='Flight critical')
    part('JD-320-007', 'Motor main bearing', '320 Motor', 'Motor', 'bearing', lambda: tube(0.052, 0.040, 0.018, 'z'),
         supplier='buy', criticality='Flight critical', process='Hybrid ceramic angular-contact bearing')
    part('JD-320-008', 'Resolver / encoder', '320 Motor', 'Motor', 'electronic', lambda: tube(0.050, 0.030, 0.012, 'z'),
         supplier='buy', fn='Rotor position (road torque + flight RPM)')
    part('JD-320-009', 'Phase-lead terminal block', '320 Motor', 'Motor', 'electronic', lambda: box(0.06, 0.03, 0.04),
         process='Moulded PPS with Cu terminals')
    part('JD-320-010', 'Spindle (fixed axle)', '320 Motor', 'Spindle', 'ti64',
         lambda: union(cyl(0.028, 0.30, 'z', (0, 0, -0.15), centered=True), cyl(0.07, 0.016, 'z', (0, 0, -0.18))),
         process='Forged Ti-6Al-4V', criticality='Flight critical', fn='Fixed axle: carries motor stator, rim bearing, wheel loads')
    part('JD-320-011', 'Tail cone fairing', '320 Motor', 'Spindle', 'cfrp',
         lambda: revolve([(0.0, -0.30), (0.04, -0.295), (0.09, -0.26), (0.125, -0.215), (0.135, -0.175),
                          (0.127, -0.175), (0.117, -0.215), (0.083, -0.255), (0.0, -0.285)]),
         process='Pre-preg CFRP', fn='Hub wake fairing; cooling air exit')
    part('JD-320-012', 'Dog clutch, rotor ring', '320 Motor', 'Clutch', 'steel',
         lambda: union(tube(0.110, 0.080, 0.010, 'z', (0, 0, -0.022)),
                       *[rot(box(0.02, 0.012, 0.008, (0.095, 0, -0.031)), 'z', 30 * k) for k in range(12)]),
         process='Machined 4340, carburised', fn='Road mode: couples rotor to rim (wheel drive)')
    part('JD-320-013', 'Dog clutch, rim ring', '320 Motor', 'Clutch', 'steel',
         lambda: union(tube(0.150, 0.112, 0.008, 'z', (0, 0, -0.040)),
                       *[rot(box(0.02, 0.012, 0.008, (0.105, 0, -0.033)), 'z', 30 * k + 15) for k in range(12)]),
         process='Machined 4340, carburised')
    part('JD-320-014', 'Clutch actuator (electromagnetic)', '320 Motor', 'Clutch', 'electronic',
         lambda: tube(0.120, 0.100, 0.02, 'z'), supplier='buy', criticality='Safety', fn='Engage / release, position-sensed')
    part('JD-320-015', 'Motor cooling manifold', '320 Motor', 'Motor', 'al6061',
         lambda: union(cyl(0.012, 0.06, 'x'), cyl(0.012, 0.06, 'x', (0, 0.04, 0))), process='Brazed Al')
    # brake caliper & rim lock on module fixed frame
    part('JD-360-001', 'Brake caliper (4-piston, rim-mounted)', '360 Brakes', 'Brakes', 'al7075',
         lambda: box(0.10, 0.07, 0.06).cut(box(0.11, 0.075, 0.016)), process='Monobloc machined 7075, Ti pistons',
         criticality='Safety', fn='Road brake + flight rim lock (clamp)')
    part('JD-360-002', 'Brake pad', '360 Brakes', 'Brakes', 'ceramic', lambda: box(0.08, 0.05, 0.012),
         supplier='buy', criticality='Safety')
    part('JD-340-020', 'Rim lock pin', '340 Conversion', 'Locks', 'steel_stn',
         lambda: union(cyl(0.009, 0.07, 'z'), cyl(0.013, 0.01, 'z', (0, 0, -0.035))), process='Ground 17-4PH',
         criticality='Flight critical')
    part('JD-340-021', 'Lock solenoid', '340 Conversion', 'Locks', 'electronic', lambda: cyl(0.022, 0.06, 'z'),
         supplier='buy', criticality='Flight critical', fn='Spring-applied, power-to-release; dual position switches')

def polar_cut(base, cutter, n):
    s = base
    for k in range(n):
        s = s.cut(rot(cutter, 'z', 360.0 * k / n))
    return s

# ============================================================== 340 CONVERSION / 350 SUSPENSION
def yoke_arm():
    h = HINGE
    dz = -0.30 - h[2]          # vertical leg plane (corner z = -0.30) in the hinge frame
    return union(cyl(0.045, 0.17, 'x'),                                          # hinge boss
                 box(0.09, 0.055, abs(dz) + 0.03, (0, 0, dz / 2)),                # top beam over the tyre
                 box(0.10, h[1] + 0.03, 0.05, (0, -h[1] / 2, dz)),                # vertical leg
                 box(0.14, 0.05, 0.08, (0, -h[1], dz + 0.02)))                    # gimbal clevis base

def carrier():
    return union(box(0.03, 0.06, 0.34, (0.11, 0.0, -0.17 - 0.08)), box(0.03, 0.06, 0.34, (-0.11, 0.0, -0.17 - 0.08)),
                 box(0.03, 0.12, 0.12, (0.11, 0.0, 0.0)), box(0.03, 0.12, 0.12, (-0.11, 0.0, 0.0)),
                 box(0.26, 0.80, 0.035, (0, -0.38, -0.36)),
                 box(0.10, 0.06, 0.06, (0, -0.38 + 0.0, -0.40)), box(0.10, 0.06, 0.06, (0, -0.73, -0.40)))

def wishbone(outer, inner_a, inner_b, r=0.016):
    return union(strut(outer, inner_a, r * 0.7), strut(outer, inner_b, r * 0.7),
                 strut(inner_a, inner_b, r * 0.55), sphere(r * 1.4, outer))

def build_suspension_parts():
    sc = '340 Conversion'
    part('JD-340-001', 'Conversion yoke arm', sc, 'Yoke', 'ti64', yoke_arm, handed=True, shell_t=-0.0035,
         process='Additively manufactured Ti-6Al-4V (LPBF) + HIP + machining', criticality='Flight critical',
         fn='Swings the fan-wheel 90° about the hinge axis: wheel plane vertical (road) → horizontal (flight)')
    part('JD-340-002', 'Yoke hinge pin', sc, 'Yoke', 'steel_stn', lambda: cyl(0.022, 0.30, 'x'),
         process='Ground 17-4PH H1025', criticality='Flight critical')
    part('JD-340-003', 'Hinge spherical bearing', sc, 'Yoke', 'bearing', lambda: tube(0.042, 0.022, 0.03, 'x'),
         supplier='buy', criticality='Flight critical')
    part('JD-340-004', 'Conversion rotary actuator (motor + harmonic drive)', sc, 'Yoke', 'electronic',
         lambda: union(cyl(0.065, 0.11, 'x'), cyl(0.045, 0.07, 'x', (-0.09, 0, 0))), supplier='buy',
         criticality='Flight critical', fn='Dual-wound BLDC + 120:1 harmonic drive, no-back brake; 0 → 90° in 6 s',
         spec=dict(output_torque_Nm=2400, stall_torque_Nm=3200, redundancy='dual motor / dual resolver'))
    part('JD-340-005', 'Hinge position RVDT', sc, 'Yoke', 'electronic', lambda: cyl(0.02, 0.03, 'x'), supplier='buy',
         criticality='Flight critical', fn='Redundant conversion-angle feedback (2 per hinge)')
    part('JD-340-010', 'Tilt gimbal trunnion', sc, 'Tilt', 'ti64',
         lambda: union(cyl(0.035, 0.12, 'y'), box(0.10, 0.04, 0.06, (0, 0, 0.03))),
         process='Machined Ti-6Al-4V', criticality='Flight critical', fn='Cruise conversion axis (0–30° forward tilt)')
    part('JD-340-011', 'Tilt rotary actuator', sc, 'Tilt', 'electronic', lambda: cyl(0.045, 0.08, 'y'), supplier='buy',
         criticality='Flight critical')
    part('JD-340-012', 'Tilt position RVDT', sc, 'Tilt', 'electronic', lambda: cyl(0.016, 0.025, 'y'), supplier='buy')
    part('JD-340-013', 'Flexible cable & coolant carrier', sc, 'Tilt', 'plastic',
         lambda: sweep_circle([(0, 0, 0), (0.02, -0.15, 0.05), (0.0, -0.35, 0.02), (0.0, -0.50, -0.05)], 0.018),
         process='PA66 drag-chain with HV / coolant / signal lines', fn='Routes power and coolant across both conversion axes')
    ss = '350 Suspension'
    part('JD-350-001', 'Corner carrier / upright', ss, 'Corner', 'al_cast', carrier, handed=True, shell_t=-0.005,
         process='Vacuum-cast A356-T6, machined', criticality='Primary structure',
         fn='Holds the conversion hinge and actuator; wishbone and steering pick-ups')
    UBJ, LBJ = (0, 0.40, -0.43), (0, 0.05, -0.43)
    part('JD-350-002', 'Upper wishbone', ss, 'Corner', 'steel', handed=True,
         builder=lambda: wishbone(UBJ, (0.22, 0.42, -0.66), (-0.22, 0.42, -0.66)),
         process='TIG-welded 4130 tube', criticality='Primary structure')
    part('JD-350-003', 'Lower wishbone', ss, 'Corner', 'steel', handed=True,
         builder=lambda: wishbone(LBJ, (0.24, 0.03, -0.66), (-0.24, 0.03, -0.66), 0.019),
         process='TIG-welded 4130 tube', criticality='Primary structure')
    part('JD-350-004', 'Ball joint', ss, 'Corner', 'steel', lambda: union(sphere(0.022), cyl(0.012, 0.05, 'y', (0, -0.03, 0))),
         supplier='buy', criticality='Primary structure')
    part('JD-350-005', 'Wishbone bush', ss, 'Corner', 'rubber', lambda: tube(0.022, 0.010, 0.04, 'x'), supplier='buy')
    part('JD-350-006', 'Pushrod', ss, 'Spring / damper', 'steel', handed=True,
         builder=lambda: strut((0, 0.06, -0.44), (0, 0.48, -0.70), 0.012, 0.009), process='4130 tube with rod ends')
    part('JD-350-007', 'Rocker', ss, 'Spring / damper', 'al7075',
         lambda: union(extrude_xy([(0, 0), (0.10, -0.04), (0.06, 0.09)], 0.02, -0.01), cyl(0.012, 0.03, 'z')),
         process='Machined 7075-T6')
    part('JD-350-008', 'Damper body', ss, 'Spring / damper', 'al7075', lambda: cyl(0.025, 0.22, 'z'),
         supplier='buy', process='Purchased 3-way adjustable damper (semi-active)')
    part('JD-350-009', 'Damper rod', ss, 'Spring / damper', 'steel', lambda: cyl(0.008, 0.12, 'z'), supplier='buy')
    part('JD-350-010', 'Coil spring', ss, 'Spring / damper', 'steel',
         lambda: compound(*[tube(0.040, 0.032, 0.010, 'z', (0, 0, -0.08 + 0.025 * k)) for k in range(7)]),
         supplier='buy', process='Purchased Si-Cr spring, 95 N/mm (modelled as coils)')
    part('JD-350-011', 'Anti-roll bar', ss, 'Spring / damper', 'steel',
         lambda: union(strut((0, 0, -0.50), (0, 0, 0.50), 0.013, 0.008), strut((0, 0, 0.50), (0.15, 0, 0.55), 0.011),
                       strut((0, 0, -0.50), (0.15, 0, -0.55), 0.011)), process='Hollow 50CrV4')
    part('JD-350-012', 'ARB drop link', ss, 'Spring / damper', 'steel',
         lambda: union(strut((0, 0, 0), (0, 0.14, 0), 0.006), sphere(0.01), sphere(0.01, (0, 0.14, 0))), supplier='buy')
    part('JD-350-020', 'Steering rack (EPS, steer-by-wire)', ss, 'Steering', 'al_cast',
         lambda: union(cyl(0.032, 0.70, 'z'), cyl(0.05, 0.16, 'x', (0.05, 0, 0.18))), supplier='buy',
         criticality='Safety', fn='Dual-motor rack, triple-redundant position; locked centred in flight')
    part('JD-350-021', 'Tie rod', ss, 'Steering', 'steel', handed=True,
         builder=lambda: union(strut((0.12, 0.20, -0.45), (0.14, 0.24, -0.82), 0.010), sphere(0.016, (0.12, 0.20, -0.45))),
         process='4340 with ball joints', criticality='Safety')
    part('JD-350-022', 'Rear toe link', ss, 'Steering', 'steel', handed=True,
         builder=lambda: union(strut((-0.12, 0.20, -0.45), (-0.14, 0.22, -0.66), 0.010), sphere(0.016, (-0.12, 0.20, -0.45))),
         process='4340 with ball joints')
    part('JD-101-052', 'Hinge nacelle fairing', '101 Body', 'Exterior trim', 'cfrp_paint',
         lambda: (tube(0.12, 0.112, 0.40, 'z', (0, 0, -0.20)).intersect(box(0.40, 0.2, 0.42, (0, 0.10, -0.20)))
                  .fuse(cyl(0.12, 0.008, 'z', (0, 0, 0)).intersect(box(0.4, 0.2, 0.05, (0, 0.10, 0))))),
         handed=True, process='Painted CFRP', fn='Covers the conversion hinge carrier beams')
    part('JD-101-050', 'Fender cap (rides on yoke), front', '101 Body', 'Fender caps', 'cfrp_paint',
         lambda: body_panel('JD-101-050'), world=True, shell_t=0.0025, handed=True,
         process='Prepreg CFRP, Class-A', fn='Fender over the fan-wheel; lifts away with the conversion yoke')
    part('JD-101-051', 'Fender cap (rides on yoke), rear', '101 Body', 'Fender caps', 'cfrp_paint',
         lambda: body_panel('JD-101-051'), world=True, shell_t=0.0025, handed=True,
         process='Prepreg CFRP, Class-A', fn='Fender over the fan-wheel; lifts away with the conversion yoke')

FAST = {}
def fasteners():
    sysf = '900 Hardware'
    lib = [('M5x16 SHCS', 0.005, 0.016, 'socket'), ('M6x20 hex bolt', 0.006, 0.020, 'hex'),
           ('M8x30 hex bolt', 0.008, 0.030, 'hex'), ('M10x45 hex bolt', 0.010, 0.045, 'hex'),
           ('M12x70 hex bolt', 0.012, 0.070, 'hex'), ('M8x25 SHCS', 0.008, 0.025, 'socket')]
    for i, (n, d, L, h) in enumerate(lib):
        pn = f'JD-900-{i + 1:03d}'
        part(pn, n, sysf, 'Fasteners', 'fastener', (lambda d=d, L=L, h=h: bolt_shape(d, L, h)), supplier='buy',
             process='ISO 4762 / ISO 4017, property class 10.9, torque-to-spec + witness mark',
             spec=dict(thread=f'M{int(d * 1000)}x{1.0 if d < 0.007 else 1.25 if d < 0.009 else 1.5 if d < 0.011 else 1.75}',
                       length_mm=L * 1000))
        FAST[n.split('x')[0] + ('s' if 'SHCS' in n else '')] = pn
    for i, d in enumerate((0.006, 0.008, 0.010, 0.012)):
        pn = f'JD-900-{20 + i:03d}'
        part(pn, f'M{int(d * 1000)} nyloc nut', sysf, 'Fasteners', 'fastener', (lambda d=d: nut_shape(d)), supplier='buy',
             process='ISO 10511, class 10')
        FAST[f'nut{int(d * 1000)}'] = pn
        pn = f'JD-900-{30 + i:03d}'
        part(pn, f'M{int(d * 1000)} washer', sysf, 'Fasteners', 'fastener', (lambda d=d: washer_shape(d)), supplier='buy',
             process='ISO 7089 hardened')
        FAST[f'wsh{int(d * 1000)}'] = pn
    part('JD-900-040', 'Quarter-turn panel fastener', sysf, 'Fasteners', 'steel_stn',
         lambda: union(cyl(0.009, 0.003, 'z', (0, 0, 0.0015)), cyl(0.004, 0.014, 'z', (0, 0, -0.007))), supplier='buy',
         process='Dzus-type, captive', fn='Body panel retention, tool-less service access')
    part('JD-900-041', 'Structural blind rivet 4.8', sysf, 'Fasteners', 'al6061',
         lambda: union(cyl(0.005, 0.0015, 'z'), cyl(0.0024, 0.010, 'z', (0, 0, -0.005))), supplier='buy')
    part('JD-900-042', 'Hose clamp', sysf, 'Fasteners', 'steel_stn', lambda: tube(0.024, 0.021, 0.009, 'z'), supplier='buy')
    part('JD-900-043', 'Cushioned P-clip', sysf, 'Fasteners', 'steel_stn', lambda: tube(0.016, 0.012, 0.012, 'x'), supplier='buy')

def bolt_circle(pn, nid, r, n, z, head_up=True, start=0.0, nut=None):
    q = (0, 0, 0, 1) if head_up else q_rot(('x', 180))
    for k in range(n):
        a = math.radians(start + 360.0 * k / n)
        place(pn, nid, (r * math.cos(a), r * math.sin(a), z), q)
        if nut:
            place(nut, nid, (r * math.cos(a), r * math.sin(a), z - (0.03 if head_up else -0.03)), q)

def build_corner(name):
    x, mirror, hand, steer = CORNERS[name]
    zc = -TRACK if mirror else TRACK
    cn = node(f'corner_{name}', f'Corner {name}', 'chassis', pos=(x, WC_Y, zc), mirror=mirror, system='300')
    # --- suspension in the corner frame
    place('JD-350-001', cn, HINGE)
    place('JD-101-052', cn, (0, HINGE[1], HINGE[2]))
    place('JD-350-002', cn); place('JD-350-003', cn)
    for p in ((0, 0.40, -0.43), (0, 0.05, -0.43)):
        place('JD-350-004', cn, p)
    for p in ((0.22, 0.42, -0.66), (-0.22, 0.42, -0.66), (0.24, 0.03, -0.66), (-0.24, 0.03, -0.66)):
        place('JD-350-005', cn, p)
    place('JD-350-006', cn)
    place('JD-350-007', cn, (0, 0.48, -0.70))
    place('JD-350-008', cn, (0.05, 0.50, -0.86)); place('JD-350-009', cn, (0.05, 0.50, -0.74))
    place('JD-350-010', cn, (0.05, 0.50, -0.86))
    place('JD-350-012', cn, (-0.12, 0.30, -0.66))
    place('JD-350-021' if steer else 'JD-350-022', cn)
    place('JD-340-004', cn, (HINGE[0] - 0.21, HINGE[1], HINGE[2]))
    place('JD-340-005', cn, (HINGE[0] + 0.17, HINGE[1], HINGE[2]))
    place('JD-340-005', cn, (HINGE[0] - 0.30, HINGE[1], HINGE[2]))
    place('JD-340-002', cn, HINGE)
    for dx in (-0.11, 0.11):
        place('JD-340-003', cn, (dx, HINGE[1], HINGE[2]))
    for p in ((0.22, 0.42, -0.66), (-0.22, 0.42, -0.66), (0.24, 0.03, -0.66), (-0.24, 0.03, -0.66)):
        place(FAST['M12'], cn, (p[0], p[1] + 0.04, p[2]), q_rot(('x', -90)))
        place(FAST['nut12'], cn, (p[0], p[1] - 0.04, p[2]), q_rot(('x', -90)))
    # --- conversion yoke (anim: 0 road → -90° flight about local X)
    yk = node(f'yoke_{name}', f'Conversion yoke {name}', cn, pos=HINGE,
              anim=dict(kind='yoke', type='rot', axis=(1, 0, 0), sign=1), system='340')
    place('JD-340-001', yk)
    cap = 'JD-101-050' if x > 0 else 'JD-101-051'
    place(cap, yk, (-x, -WC_Y - HINGE[1], -TRACK - HINGE[2]))
    place('JD-340-013', yk, (0.06, -0.02, -0.22))
    tl = node(f'tilt_{name}', f'Cruise tilt gimbal {name}', yk, pos=(0, -HINGE[1], -0.30 - HINGE[2]),
              anim=dict(kind='tilt', type='rot', axis=(0, 1, 0), sign=-1 if mirror else 1), system='340')
    place('JD-340-010', tl, (0, 0, -0.02))
    place('JD-340-011', tl, (0, 0.10, -0.02)); place('JD-340-012', tl, (0, -0.08, -0.02))
    md = node(f'mod_{name}', f'Fan-wheel module {name} (fixed)', tl, pos=(0, 0, 0.30), system='300')
    # fixed: motor stator side, spindle, tail cone, clutch actuator, brake caliper, locks
    for pn, p in (('JD-320-001', (0, 0, 0)), ('JD-320-002', (0, 0, 0)), ('JD-320-010', (0, 0, 0)),
                  ('JD-320-011', (0, 0, 0)), ('JD-320-014', (0, 0, -0.045)), ('JD-320-008', (0, 0, -0.160)),
                  ('JD-320-009', (0, -0.11, -0.19)), ('JD-320-015', (0.0, 0.07, -0.20))):
        place(pn, md, p)
    for k in range(18):
        a = 2 * math.pi * k / 18
        place('JD-320-003', md, (0.091 * math.cos(a), 0.091 * math.sin(a), -0.10), q_rot(('z', math.degrees(a) + 90)))
    for zb in (-0.03, -0.155):
        place('JD-320-007', md, (0, 0, zb))
    place('JD-360-001', md, (0, 0.598, -0.192), q_rot(('z', 0)))
    place('JD-360-002', md, (0, 0.598, -0.178)); place('JD-360-002', md, (0, 0.598, -0.206))
    for a in (45, 135):
        r = 0.60
        place('JD-340-020', md, (r * math.cos(math.radians(a)), r * math.sin(math.radians(a)), -0.215))
        place('JD-340-021', md, (r * math.cos(math.radians(a)), r * math.sin(math.radians(a)), -0.265))
    bolt_circle(FAST['M10'], md, 0.055, 8, -0.192, head_up=False)
    for dx in (-0.035, 0.035):
        place(FAST['M10'], md, (dx, 0.598, -0.15))
    # --- rim (spins in road mode, locked in flight)
    rm = node(f'rim_{name}', f'Rim / duct / tyre {name}', md, anim=dict(kind='rim', type='rot', axis=(0, 0, 1), sign=1),
              system='300')
    for pn in ('JD-300-001', 'JD-300-010', 'JD-300-011', 'JD-300-014', 'JD-300-015'):
        place(pn, rm)
    place('JD-300-016', rm, (0, 0, -0.192))
    place('JD-320-013', rm)
    for k in range(N_TREAD):
        a = 360.0 * k / N_TREAD
        for row, zz in enumerate((-0.068, 0.068)):
            aa = math.radians(a + (2.5 if row else 0))
            place('JD-300-002', rm, (0.749 * math.cos(aa), 0.749 * math.sin(aa), zz), q_rot(('z', math.degrees(aa) - 90)))
    for k in range(N_WEB):
        place('JD-300-003', rm, (0, 0, 0), q_rot(('z', 360.0 * k / N_WEB)))
    for k in range(N_VANES):
        a = 360.0 * k / N_VANES
        place('JD-300-013', rm, (0, 0, -0.085), q_rot(('z', a)))
        for da in (-4, 4):
            aa = math.radians(a + da)
            place(FAST['M8'], rm, (0.16 * math.cos(aa), 0.16 * math.sin(aa), -0.135), q_rot(('x', 180)))
        aa = math.radians(a)
        place(FAST['M6'], rm, (0.645 * math.cos(aa), 0.645 * math.sin(aa), -0.085), q_rot(('y', 90), ('z', a)))
    for a in (45, 135, 225, 315):
        place('JD-300-012', rm, (0.60 * math.cos(math.radians(a)), 0.60 * math.sin(math.radians(a)), -0.175))
    bolt_circle(FAST['M6'], rm, 0.598, 12, -0.205, head_up=False, start=15)
    for k in range(N_PETALS):
        a = 30.0 * k
        ar = math.radians(a)
        po = node(f'petO_{name}_{k}', f'Outboard petal {k + 1}', rm, pos=(0.600 * math.cos(ar), 0.600 * math.sin(ar), 0.176),
                  q=q_rot(('z', a)), anim=dict(kind='petal_out', type='rot', axis=(0, 1, 0), sign=1), system='300')
        place('JD-300-020', po)
        place('JD-300-022', po, (0.012, 0.0, 0.006))
        pi = node(f'petI_{name}_{k}', f'Inboard petal {k + 1}', rm, pos=(0.556 * math.cos(ar), 0.556 * math.sin(ar), -0.178),
                  q=q_rot(('z', a)), anim=dict(kind='petal_in', type='rot', axis=(0, 1, 0), sign=-1, index=(k == 3)),
                  system='300')
        place('JD-300-021', pi)
        place('JD-300-022', pi, (0.012, 0.0, -0.006))
    # --- rotor (spins with rim in road; free in flight)
    rt = node(f'rotor_{name}', f'Proprotor {name} ({hand}H)', md,
              anim=dict(kind='rotor', type='rot', axis=(0, 0, 1), sign=1 if hand == 'R' else -1), system='330')
    for pn in ('JD-330-010', 'JD-330-017', 'JD-330-018', 'JD-320-006', 'JD-320-012'):
        place(pn, rt)
    place('JD-330-015', rt, (0, 0, 0.02)); place('JD-330-016', rt, (0, 0, -0.002))
    for zd in (-0.07, -0.13):
        place('JD-320-004', rt, (0, 0, zd))
        for k in range(24):
            a = 2 * math.pi * (k + 0.5) / 24
            place('JD-320-005', rt, (0.088 * math.cos(a), 0.088 * math.sin(a), zd + (0.007 if zd > -0.1 else -0.007)),
                  q_rot(('z', math.degrees(a) + 90)))
    bolt_circle(FAST['M8'], rt, 0.060, 12, -0.018, head_up=False)
    blade = 'JD-330-001' if hand == 'R' else 'JD-330-002'
    for k in range(D['B']):
        a = 360.0 * k / D['B']
        bn = node(f'blade_{name}_{k}', f'Blade {k + 1} pitch axis', rt, pos=(0, 0, Z_ROTOR), q=q_rot(('z', a)),
                  anim=dict(kind='pitch', type='rot', axis=(1, 0, 0), sign=1 if hand == 'R' else -1), system='330')
        place(blade, bn)
        place('JD-330-011', bn)
        place('JD-330-012', bn, (0.068, 0, 0)); place('JD-330-012', bn, (0.090, 0, 0))
        place('JD-330-013', bn, (0.075, 0, 0.0) if hand == 'R' else (0.075, 0, 0.0),
              (0, 0, 0, 1) if hand == 'R' else q_rot(('x', 180)))
        place('JD-330-019', bn, (0.052, 0, 0))
        ar = math.radians(a + (12 if hand == 'R' else -12))
        place('JD-330-014', rt, (0.06 * math.cos(ar), 0.06 * math.sin(ar), Z_ROTOR + 0.045))

# ============================================================== 600 LANDING LEGS
LEGS = {'FL': (1.30, 0.60), 'FR': (1.30, -0.60), 'RL': (-1.30, 0.60), 'RR': (-1.30, -0.60)}
LEG_LIFT = 0.12              # vehicle rises this much on the legs before conversion
def build_legs():
    sl = '600 Landing legs'
    part('JD-600-001', 'Leg housing', sl, 'Landing leg', 'al7075', lambda: tube(0.055, 0.047, 0.42, 'y', (0, 0.21, 0)),
         process='Machined 7075-T73, hard-anodised bore', criticality='Flight critical')
    part('JD-600-002', 'Leg upper stage', sl, 'Landing leg', 'ti64', lambda: tube(0.045, 0.038, 0.40, 'y', (0, 0.20, 0)),
         process='Machined Ti-6Al-4V', criticality='Flight critical')
    part('JD-600-003', 'Leg lower stage', sl, 'Landing leg', 'steel', lambda: tube(0.036, 0.028, 0.38, 'y', (0, 0.19, 0)),
         process='300M steel, chromed', criticality='Flight critical')
    part('JD-600-004', 'Foot pad (articulated)', sl, 'Landing leg', 'al7075',
         lambda: union(cyl(0.11, 0.02, 'y', (0, -0.01, 0)), sphere(0.03, (0, 0.02, 0))),
         process='Machined 7075 + rubber tread', fn='Ground contact on uneven / soft surfaces')
    part('JD-600-005', 'Leg deploy actuator (ball-screw)', sl, 'Landing leg', 'electronic',
         lambda: union(cyl(0.022, 0.30, 'y', (0, 0.15, 0)), cyl(0.03, 0.08, 'x', (0.03, 0.30, 0))), supplier='buy',
         criticality='Flight critical', fn='Deploys / retracts; carries the car off its tyres for conversion')
    part('JD-600-006', 'Oleo damper cartridge', sl, 'Landing leg', 'steel', lambda: cyl(0.02, 0.18, 'y', (0, 0.09, 0)),
         supplier='buy', fn='Absorbs landing energy (2.5 m/s sink rate target)')
    part('JD-600-007', 'Down-lock & WOW sensor', sl, 'Landing leg', 'electronic', lambda: box(0.05, 0.04, 0.04),
         supplier='buy', criticality='Flight critical', fn='Mechanical down-lock, weight-on-wheels switch (dual)')
    for name, (lx, lz) in LEGS.items():
        ln = node(f'leg_{name}', f'Landing leg {name}', 'chassis', pos=(lx, 0.24, lz), system='600')
        place('JD-600-001', ln)
        place('JD-600-005', ln, (0.075, 0, 0))
        place('JD-600-007', ln, (-0.07, 0.30, 0))
        s1 = node(f'leg1_{name}', f'Leg {name} upper stage', ln, anim=dict(kind='leg', type='trans', axis=(0, -1, 0), stroke=0.27), system='600')
        place('JD-600-002', s1)
        s2 = node(f'leg2_{name}', f'Leg {name} lower stage', s1, anim=dict(kind='leg', type='trans', axis=(0, -1, 0), stroke=0.28), system='600')
        place('JD-600-003', s2); place('JD-600-006', s2, (0, 0.10, 0))
        place('JD-600-004', s2, (0, -0.035, 0))
        bolt_circle(FAST['M10'], ln, 0.08, 6, 0.10, start=30) if False else None
        for k in range(6):
            a = math.radians(30 + 60 * k)
            place(FAST['M10'], 'chassis', (lx + 0.085 * math.cos(a), 0.36, lz + 0.085 * math.sin(a)), q_rot(('x', -90)))

# ============================================================== 400 FLIGHT CONTROL, AVIONICS & SENSORS
def build_avionics():
    node('avionics', 'Flight control & avionics', 'vehicle', system='400')
    units = [
        ('JD-400-001', 'Flight control computer (FCC)', (0.20, 0.06, 0.16), [(1.70, 0.70, -0.20), (1.70, 0.70, 0.0), (1.70, 0.70, 0.20)],
         'Triplex, dissimilar; control laws, thrust allocation, conversion supervision', 'Flight critical'),
        ('JD-400-002', 'Inertial measurement unit (IMU)', (0.08, 0.06, 0.08), [(-0.30, 0.47, -0.06), (-0.30, 0.47, 0.06), (-0.40, 0.47, 0.0)],
         'Triplex MEMS IMU near CG', 'Flight critical'),
        ('JD-400-003', 'Air data computer (ADC)', (0.12, 0.05, 0.10), [(1.70, 0.62, 0.40)], 'Airspeed / altitude / AoA', 'Flight critical'),
        ('JD-400-004', 'Independent flight monitor', (0.16, 0.05, 0.12), [(1.95, 0.70, 0.0)],
         'Dissimilar plausibility monitor & inhibit (per Rev 07 schematic 02, block 450)', 'Flight critical'),
        ('JD-400-005', 'Mode-S / ADS-B transponder', (0.16, 0.05, 0.14), [(2.15, 0.70, 0.25)], 'Cooperative surveillance', ''),
        ('JD-400-006', 'VHF COM radio', (0.16, 0.05, 0.14), [(2.15, 0.70, -0.25)], 'ATC voice / data', ''),
        ('JD-400-007', 'Flight data recorder', (0.14, 0.10, 0.10), [(-2.6, 0.62, 0.0)], 'Crash-protected recorder', ''),
        ('JD-400-008', 'Emergency locator transmitter', (0.10, 0.05, 0.06), [(-2.6, 0.80, 0.2)], '406 MHz ELT', 'Safety'),
        ('JD-400-009', 'Vehicle control unit (VCU, road)', (0.20, 0.05, 0.15), [(1.95, 0.62, -0.40)], 'Road torque vectoring / ABS / ESC', 'Safety'),
        ('JD-400-010', 'Body control module', (0.18, 0.04, 0.12), [(0.60, 0.70, 0.40)], 'Lights, doors, petals, HVAC', ''),
        ('JD-400-011', 'Telematics / LTE-C2 unit', (0.12, 0.03, 0.10), [(0.60, 0.70, -0.40)], 'Connectivity, remote telemetry', ''),
        ('JD-400-012', 'Fuse & relay box (12 V)', (0.20, 0.06, 0.14), [(2.30, 0.62, 0.0)], 'LV distribution', ''),
    ]
    for pn, name, dims, poss, fn, crit in units:
        part(pn, name, '400 Flight control & avionics', 'Avionics bay', 'electronic', (lambda d=dims: box(*d)),
             supplier='buy' if 'computer' not in name.lower() and 'monitor' not in name.lower() else 'make',
             fn=fn, criticality=crit, process='DO-160G environmental / DO-178C & DO-254 DAL A' if crit == 'Flight critical' else 'Automotive grade')
        for p in poss:
            place(pn, 'avionics', p)
    sensors = [
        ('JD-410-001', 'Pitot-static probe (heated)', lambda: union(cyl(0.006, 0.18, 'x', (0.09, 0, 0)), box(0.03, 0.06, 0.012, (0, -0.03, 0))),
         [(1.03, 1.06, 1.17), (1.03, 1.06, -1.17)], 'Dual air data, one per mirror pod'),
        ('JD-410-002', 'GNSS antenna (multi-band)', lambda: cyl(0.045, 0.015, 'y'), [(-1.45, 1.70, 0.25), (-1.45, 1.70, -0.25)], 'Dual GNSS / RTK'),
        ('JD-410-003', 'Radar altimeter', lambda: box(0.10, 0.03, 0.08), [(0.5, 0.205, 0.0), (-0.5, 0.205, 0.0)], 'Height above ground (hover / landing)'),
        ('JD-410-004', 'LiDAR (solid-state)', lambda: box(0.10, 0.05, 0.12), [(2.97, 0.70, 0.0), (-0.3, 1.70, 0.0)], 'Obstacle detection road & landing zone'),
        ('JD-410-005', 'Forward 4D imaging radar', lambda: box(0.04, 0.08, 0.12), [(3.05, 0.40, 0.0)], 'ADAS & detect-and-avoid'),
        ('JD-410-006', 'Surround camera', lambda: union(box(0.04, 0.03, 0.03), cyl(0.01, 0.01, 'x', (0.025, 0, 0))),
         [(3.0, 0.85, 0.3), (3.0, 0.85, -0.3), (-3.08, 0.95, 0.0), (0.95, 1.0, 1.15), (0.95, 1.0, -1.15), (-0.2, 0.205, 0.4),
          (-0.2, 0.205, -0.4), (-0.6, 1.72, 0.0)], '360° vision + downward landing cameras'),
        ('JD-410-007', 'VHF blade antenna', lambda: extrude_xy([(0, 0), (0.12, 0), (0.08, 0.10), (0.03, 0.10)], 0.008, -0.004),
         [(-2.4, 1.12, 0.0)], 'COM / transponder antenna'),
        ('JD-410-008', 'Outside-air temperature probe', lambda: cyl(0.006, 0.04, 'y'), [(2.6, 0.22, 0.3)], 'Density altitude / icing'),
        ('JD-410-009', 'Ultrasonic proximity sensor', lambda: cyl(0.011, 0.018, 'x'),
         [(3.08, 0.32, 0.5), (3.08, 0.32, -0.5), (-3.09, 0.40, 0.5), (-3.09, 0.40, -0.5)], 'Parking / hover-taxi proximity'),
    ]
    for pn, name, fnb, poss, fn in sensors:
        part(pn, name, '400 Flight control & avionics', 'Sensors', 'electronic', fnb, supplier='buy', fn=fn,
             criticality='Flight critical' if pn in ('JD-410-001', 'JD-410-003') else '')
        for p in poss:
            place(pn, 'avionics', p, q_rot(('x', 180)) if p[2] < -0.5 and pn == 'JD-410-001' else (0, 0, 0, 1))
    part('JD-420-001', 'Flight-control data bus harness (CAN-FD / ARINC 825)', '400 Flight control & avionics', 'Harness',
         'hv_cable', lambda: sweep_circle([(1.70, 0.70, 0.0), (0.8, 0.60, 0.1), (-0.3, 0.50, 0.0), (-1.4, 0.70, 0.0),
                                            (-2.3, 0.80, 0.0)], 0.008),
         world=True, process='Triple-routed, segregated twisted-pair', criticality='Flight critical')
    place('JD-420-001', 'avionics')
    part('JD-420-002', '12 V harness backbone', '700 Low voltage', 'Harness', 'hose',
         lambda: sweep_circle([(2.30, 0.62, 0.0), (1.2, 0.80, 0.55), (0.0, 0.95, 0.62), (-1.6, 0.90, 0.55), (-2.6, 0.80, 0.3)], 0.011),
         world=True, process='Taped & convoluted LV harness')
    place('JD-420-002', 'avionics')

# ============================================================== 450 EMERGENCY SYSTEMS
def build_emergency():
    node('emergency', 'Emergency systems', 'vehicle', system='450')
    part('JD-450-001', 'Ballistic parachute canister', '450 Emergency', 'Whole-vehicle parachute', 'cfrp',
         lambda: tube(0.16, 0.152, 0.50, 'y').fuse(cyl(0.16, 0.008, 'y', (0, -0.25, 0))),
         process='Filament-wound CFRP canister', criticality='Safety', fn='Houses a 2,500 kg-rated whole-vehicle recovery parachute')
    part('JD-450-002', 'Parachute pack', '450 Emergency', 'Whole-vehicle parachute', 'textile', lambda: cyl(0.15, 0.40, 'y'),
         supplier='buy', mass=41.0, process='Purchased BRS-class canopy, 2,500 kg / 90 m/s rating', criticality='Safety')
    part('JD-450-003', 'Extraction rocket motor', '450 Emergency', 'Whole-vehicle parachute', 'steel',
         lambda: union(cyl(0.04, 0.32, 'y'), cone(0.04, 0.0, 0.08, 'y', (0, 0.16, 0))), supplier='buy', criticality='Safety')
    part('JD-450-004', 'Parachute riser / bridle', '450 Emergency', 'Whole-vehicle parachute', 'textile',
         lambda: sweep_circle([(0, 0, 0), (0.3, 0.15, 0.2), (0.9, 0.25, 0.5), (1.5, 0.12, 0.6)], 0.008), handed=True,
         supplier='buy', criticality='Safety', fn='4-point harness to roll hoop & front subframe')
    place('JD-450-001', 'emergency', (-2.05, 0.86, 0)); place('JD-450-002', 'emergency', (-2.05, 0.86, 0))
    place('JD-450-003', 'emergency', (-2.30, 0.86, 0.22))
    place('JD-450-004', 'emergency', (-2.05, 1.05, 0.0))
    node('emergency_R', 'Emergency (starboard mirror)', 'emergency', mirror=True, system='450')
    place('JD-450-004', 'emergency_R', (-2.05, 1.05, 0.0))
    part('JD-450-005', 'Fire suppression bottle (battery bay)', '450 Emergency', 'Fire', 'steel',
         lambda: union(cyl(0.05, 0.25, 'x'), sphere(0.05, (0.125, 0, 0)), sphere(0.05, (-0.125, 0, 0))), supplier='buy',
         criticality='Safety', fn='Battery-bay thermal-runaway suppression')
    place('JD-450-005', 'emergency', (-1.85, 0.40, 0.0))

# ============================================================== 500 THERMAL MANAGEMENT
def build_thermal():
    node('thermal', 'Thermal management', 'vehicle', system='500')
    part('JD-500-001', 'Front radiator', '500 Thermal', 'Heat rejection', 'coolant',
         lambda: union(box(0.04, 0.30, 0.50), box(0.05, 0.34, 0.03, (0, 0, 0.26)), box(0.05, 0.34, 0.03, (0, 0, -0.26))),
         process='Brazed Al micro-channel', fn='Motor / inverter / battery loop heat rejection (ground & hover)')
    part('JD-500-002', 'Radiator fan', '500 Thermal', 'Heat rejection', 'plastic',
         lambda: union(tube(0.15, 0.14, 0.05, 'x'), cyl(0.04, 0.05, 'x'), *[rot(box(0.006, 0.10, 0.03, (0, 0.07, 0)), 'x', 72 * k) for k in range(5)]),
         supplier='buy', process='Purchased 48 V brushless fan, 600 W')
    for s in (1, -1):
        place('JD-500-001', 'thermal', (2.45, 0.55, s * 0.60), q_rot(('y', s * 35)))
        place('JD-500-002', 'thermal', (2.36, 0.55, s * 0.55), q_rot(('y', s * 35)))
    comps = [
        ('JD-500-003', 'Electric coolant pump', lambda: union(cyl(0.04, 0.10, 'x'), cyl(0.012, 0.05, 'y', (0.03, 0.04, 0))),
         [(2.05, 0.45, 0.30), (2.05, 0.45, -0.30)], 'Motor / power-electronics loop circulation'),
        ('JD-500-004', 'Coolant expansion reservoir', lambda: box(0.14, 0.12, 0.10), [(2.30, 0.80, 0.40)], 'Degas / expansion'),
        ('JD-500-005', 'Battery chiller (refrigerant ↔ coolant)', lambda: box(0.18, 0.10, 0.08), [(1.75, 0.50, 0.45)], 'Pack cooling for DC fast charge & hover'),
        ('JD-500-006', 'Heat-pump e-compressor', lambda: union(cyl(0.06, 0.20, 'x'), box(0.08, 0.10, 0.10, (0.12, 0, 0))),
         [(1.75, 0.55, -0.45)], 'Cabin heat pump / A/C'),
        ('JD-500-007', 'Cabin HVAC module', lambda: box(0.30, 0.20, 0.45), [(0.85, 0.75, 0.0)], 'Blower, evaporator, PTC'),
        ('JD-500-008', 'Coolant manifold / multi-way valve', lambda: union(box(0.12, 0.06, 0.10), *[cyl(0.012, 0.04, 'y', (dx, 0.04, 0)) for dx in (-0.04, 0, 0.04)]),
         [(1.95, 0.52, 0.0)], 'Routes coolant between battery, motors, cabin'),
    ]
    for pn, name, fnb, poss, fn in comps:
        part(pn, name, '500 Thermal', 'Thermal components', 'electronic' if 'module' in name or 'pump' in name or 'compressor' in name else 'coolant',
             fnb, supplier='buy', fn=fn)
        for p in poss:
            place(pn, 'thermal', p)
    hoses = {
        'JD-510-001': ('Coolant hose, radiator → pump (port)', [(2.45, 0.40, 0.60), (2.3, 0.40, 0.45), (2.05, 0.45, 0.33)]),
        'JD-510-002': ('Coolant hose, radiator → pump (stbd)', [(2.45, 0.40, -0.60), (2.3, 0.40, -0.45), (2.05, 0.45, -0.33)]),
        'JD-510-003': ('Coolant hose, manifold → front inverters', [(1.95, 0.55, 0.0), (1.7, 0.85, 0.2), (1.45, 0.98, 0.35)]),
        'JD-510-004': ('Coolant hose, manifold → rear loop', [(1.95, 0.52, 0.05), (0.9, 0.46, 0.70), (-0.9, 0.46, 0.70), (-1.45, 0.88, 0.40)]),
        'JD-510-005': ('Coolant hose, manifold → battery cold plates', [(1.90, 0.50, -0.05), (1.3, 0.45, -0.2), (1.08, 0.40, -0.30)]),
    }
    for pn, (name, pts) in hoses.items():
        part(pn, name, '500 Thermal', 'Coolant lines', 'hose', (lambda p=pts: sweep_circle(p, 0.014)), world=True,
             process='EPDM, 25 mm ID, spring clamps')
        place(pn, 'thermal')
        place('JD-900-042', 'thermal', pts[0]); place('JD-900-042', 'thermal', pts[-1])

# ============================================================== 700 ROAD-SPECIFIC CHASSIS SYSTEMS
def build_road_systems():
    node('road', 'Road chassis systems', 'chassis', system='350')
    place('JD-350-020', 'road', (WB + 0.14, WC_Y + 0.22, 0))
    place('JD-350-011', 'road', (WB - 0.12, WC_Y + 0.30, 0)); place('JD-350-011', 'road', (-WB - 0.12, WC_Y + 0.30, 0))
    part('JD-360-010', 'Brake-by-wire / ESC hydraulic unit', '360 Brakes', 'Brake control', 'al_cast',
         lambda: union(box(0.16, 0.12, 0.12), cyl(0.03, 0.10, 'y', (0.05, 0.09, 0))), supplier='buy', criticality='Safety')
    place('JD-360-010', 'road', (1.55, 0.82, -0.25))
    part('JD-360-011', 'Brake line (front)', '360 Brakes', 'Brake control', 'steel_stn',
         lambda: sweep_circle([(1.55, 0.82, -0.25), (1.75, 0.95, 0.4), (1.80, 1.20, 0.75)], 0.004), world=True,
         supplier='make', process='Stainless braided PTFE line')
    place('JD-360-011', 'road')

# ============================================================== MAIN
def build_all():
    node('vehicle', 'Joukowsky Dynamics Rev 08 — wheel-to-proprotor flying car', None,
         anim=dict(kind='lift', type='trans', axis=(0, 1, 0)), system='000')
    fasteners()
    build_body(); build_glazing(); build_lighting(); build_structure(); build_cockpit(); build_power()
    build_corner_parts(); build_suspension_parts()
    for c in CORNERS:
        build_corner(c)
    build_legs(); build_avionics(); build_emergency(); build_thermal(); build_road_systems()
    # body panel quarter-turn fasteners along major break lines
    for x in (2.55, 1.18, -1.15, -2.55):
        for z in (-0.7, -0.35, 0.0, 0.35, 0.7):
            place('JD-900-040', 'body', (x, 0.95 if abs(x) > 2 else 1.02, z))
    for x in [i * 0.4 - 2.4 for i in range(13)]:
        for z in (-0.88, 0.88):
            place('JD-900-041', 'body', (x, 0.22, z))
    # seats to rails, subframes to tub
    for s in (1, -1):
        for dx in (-0.2, 0.2):
            for dz in (-0.13, 0.13):
                place(FAST['M10'], 'cockpit', (-0.95 + dx, 0.52, s * 0.35 + dz))
    for x in (1.18, -1.18):
        for p in ((0.30, 0.62), (0.30, -0.62), (0.80, 0.55), (0.80, -0.55)):
            place(FAST['M12'], 'chassis', (x, p[0], p[1]), q_rot(('z', -90 if x > 0 else 90)))
            place(FAST['nut12'], 'chassis', (x - (0.04 if x > 0 else -0.04), p[0], p[1]), q_rot(('z', -90 if x > 0 else 90)))
            place(FAST['wsh12'], 'chassis', (x - (0.03 if x > 0 else -0.03), p[0], p[1]), q_rot(('z', -90 if x > 0 else 90)))
    for m in range(19):
        for p in ((-0.18, 0.17), (0.18, 0.17), (-0.18, -0.17), (0.18, -0.17)):
            place(FAST['M8s'], f'bmod_{m:02d}', (p[0], 0.0, p[1]), q_rot(('x', 180)))

META = dict(
    title='Joukowsky Dynamics Rev 08 — wheel-to-proprotor flying car',
    basis='Builds on the A1-road baseline (Rev 07 concept CAD). Exterior lofted from A1 body surfaces.',
    frame='+X forward, +Y up, +Z port; metres (STEP / BOM in millimetres)',
    ground_y=GROUND, wheel_centre_y=WC_Y, half_wheelbase=WB, half_track=TRACK, hinge=HINGE, leg_lift=LEG_LIFT,
    tire_R=TIRE_R, duct_R=DUCT_R, z_rotor=Z_ROTOR,
    proprotor=dict(D, chord_coeffs=PR.chord_coeffs(), stations=PR.blade_stations(D, 11), performance=PR.performance(D)),
    status='Concept CAD for design exploration. Not a flight-qualified design: no certified loads, '
           'fatigue, aeroelastic, crash or fault-tolerance substantiation.',
)

if __name__ == '__main__':
    import time
    t0 = time.time()
    build_all()
    # OCC quirk: trim the lofted body / canopy before any STEP export or meshing runs in-process
    body_panel('JD-101-010'); glazing('ws')
    print(f'registered {len(PARTS)} unique parts, {len(INST)} instances, {len(NODES)} nodes')
    only = None
    if '--only' in sys.argv:
        only = set(sys.argv[sys.argv.index('--only') + 1].split(','))
    export(OUT_WEB, OUT_CAD, META, step_assembly='--fast' not in sys.argv, only=only)
    print(f'total {time.time() - t0:.0f}s')
