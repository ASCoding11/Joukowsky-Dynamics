"""Joukowsky Dynamics Rev 08 CAD kernel helpers.

A small registry over CadQuery / OpenCascade:
  * part(...)   registers a uniquely numbered part and a builder that returns a B-rep solid
  * node(...)   registers an assembly node (a frame with an optional animated pivot)
  * place(...)  places an instance of a part in a node frame

`export(...)` then builds each unique part once, writes a STEP file per part (with LH variants
for mirrored corners), tessellates each face for the web viewer, and writes the catalog JSON,
packed mesh binary, BOM CSV and a full-assembly STEP.

Frame: +X forward, +Y up, +Z port. Units: metres in the model, millimetres in STEP / BOM.
"""
import math, json, struct, os, csv, time, traceback
import cadquery as cq
from cadquery import Vector as V

# ---------------------------------------------------------------- quaternions
def q_axis(axis, deg):
    a = math.radians(deg) / 2
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z) or 1
    s = math.sin(a) / n
    return (x * s, y * s, z * s, math.cos(a))

def q_mul(a, b):
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)

def q_rot(*steps):
    """q_rot(('x',90),('z',30)) applies x first, then z (extrinsic)."""
    q = (0, 0, 0, 1)
    ax = {'x': (1, 0, 0), 'y': (0, 1, 0), 'z': (0, 0, 1)}
    for s in steps:
        axis = ax[s[0]] if isinstance(s[0], str) else s[0]
        q = q_mul(q_axis(axis, s[1]), q)
    return q

def q_apply(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    ix = w * vx + y * vz - z * vy; iy = w * vy + z * vx - x * vz
    iz = w * vz + x * vy - y * vx; iw = -x * vx - y * vy - z * vz
    return (ix * w + iw * -x + iy * -z - iz * -y,
            iy * w + iw * -y + iz * -x - ix * -z,
            iz * w + iw * -z + ix * -y - iy * -x)

def q_to_axis_angle(q):
    x, y, z, w = q
    w = max(-1.0, min(1.0, w))
    ang = 2 * math.acos(w)
    s = math.sqrt(max(1e-12, 1 - w * w))
    if ang < 1e-9:
        return (0, 0, 1), 0.0
    return (x / s, y / s, z / s), math.degrees(ang)

# ---------------------------------------------------------------- materials (kg/m^3)
MATERIALS = {
    'cfrp':      dict(name='Carbon-fibre / epoxy laminate (T700, 0/±45/90)', rho=1550, color='#2a313b', look='carbon'),
    'cfrp_paint':dict(name='CFRP skin, Class-A clear-coated paint',          rho=1550, color='#b3263a', look='paint'),
    'al7075':    dict(name='Aluminium 7075-T6 (machined)',                   rho=2810, color='#a9b2bb', look='metal'),
    'al6061':    dict(name='Aluminium 6061-T6 (extruded / welded)',          rho=2700, color='#9aa4ad', look='metal'),
    'al_cast':   dict(name='Aluminium A356-T6 (vacuum cast)',                rho=2680, color='#8f9aa3', look='metal'),
    'ti64':      dict(name='Titanium Ti-6Al-4V',                             rho=4430, color='#8b8f99', look='metal'),
    'steel':     dict(name='Alloy steel 4340 (Q&T)',                         rho=7850, color='#6f7780', look='metal'),
    'steel_stn': dict(name='Stainless steel 17-4PH',                         rho=7800, color='#b9c0c6', look='metal'),
    'bearing':   dict(name='Bearing steel 52100',                            rho=7810, color='#c9cfd4', look='metal'),
    'fastener':  dict(name='Steel 10.9 fastener, zinc-flake coated',         rho=7850, color='#7d858c', look='metal'),
    'rubber':    dict(name='Tread compound (SBR / silica)',                  rho=1150, color='#15191f', look='rubber'),
    'pu':        dict(name='Thermoplastic polyurethane (airless web)',       rho=1200, color='#1f252d', look='rubber'),
    'glass':     dict(name='Laminated polycarbonate glazing, hard-coated',   rho=1200, color='#20384a', look='glass'),
    'lens':      dict(name='Optical PMMA lens',                              rho=1180, color='#cfe8ff', look='light'),
    'copper':    dict(name='Copper (C11000) conductor',                      rho=8960, color='#c87f4f', look='copper'),
    'ndfeb':     dict(name='Sintered NdFeB magnet (N42SH)',                  rho=7500, color='#8a96a3', look='metal'),
    'sisteel':   dict(name='Non-oriented electrical steel lamination',       rho=7650, color='#5d6670', look='metal'),
    'cell':      dict(name='Li-ion 4680 cell (NMC811, steel can)',           rho=2700, color='#2f6d8a', look='cell'),
    'pcb':       dict(name='FR-4 PCB assembly',                              rho=1850, color='#1d5a3a', look='pcb'),
    'electronic':dict(name='Electronic unit (Al housing, potted)',           rho=800,  color='#3a4652', look='metal'),
    'plastic':   dict(name='Glass-filled PA66',                              rho=1350, color='#262c33', look='plastic'),
    'foam':      dict(name='Polyurethane seat foam / Alcantara trim',        rho=120,  color='#2b2f36', look='fabric'),
    'textile':   dict(name='Polyester webbing / nylon canopy',               rho=900,  color='#444b55', look='fabric'),
    'hose':      dict(name='EPDM coolant hose',                              rho=1100, color='#111418', look='rubber'),
    'hv_cable':  dict(name='HV cable, orange XLPE jacket',                   rho=3500, color='#e0701d', look='hv'),
    'coolant':   dict(name='Brazed aluminium heat exchanger',                rho=900,  color='#7c8a96', look='metal'),
    'ceramic':   dict(name='Carbon-ceramic brake material',                  rho=2400, color='#3b3f45', look='metal'),
}

# ---------------------------------------------------------------- registry
PARTS, NODES, INST = {}, {}, []
_order = []

def part(pn, name, system, assy, material, builder, desc='', fn='', process='', spec=None,
         world=False, shell_t=None, mass=None, handed=False, criticality='', supplier='make'):
    """Register a part. builder() -> cq.Shape / cq.Workplane in part-local coordinates
    (or vehicle coordinates if world=True; it is then re-centred and instances carry the offset)."""
    assert pn not in PARTS, pn
    PARTS[pn] = dict(pn=pn, name=name, system=system, assy=assy, material=material, builder=builder,
                     desc=desc, fn=fn, process=process, spec=spec or {}, world=world, shell_t=shell_t,
                     mass_override=mass, handed=handed, criticality=criticality, supplier=supplier)
    _order.append(pn)
    return pn

def node(nid, name, parent=None, pos=(0, 0, 0), q=(0, 0, 0, 1), mirror=False, anim=None, system=None):
    """anim: dict(kind=..., type='rot'|'trans', axis=(x,y,z)) — pivot is the node origin."""
    assert nid not in NODES, nid
    NODES[nid] = dict(id=nid, name=name, parent=parent, pos=tuple(pos), q=tuple(q), mirror=mirror,
                      anim=anim, system=system)
    return nid

def place(pn, nid, pos=(0, 0, 0), q=(0, 0, 0, 1), tag=None):
    assert pn in PARTS, pn
    assert nid in NODES, nid
    INST.append([pn, nid, tuple(pos), tuple(q), tag])

# ---------------------------------------------------------------- geometry helpers
def S(x):
    """Coerce Workplane / Shape to a single cq.Shape."""
    if isinstance(x, cq.Workplane):
        vals = [v for v in x.vals() if isinstance(v, cq.Shape)]
        if len(vals) == 1:
            return vals[0]
        return cq.Compound.makeCompound(vals)
    return x

def box(l, w, h, c=(0, 0, 0)):
    return cq.Solid.makeBox(l, w, h, V(c[0] - l / 2, c[1] - w / 2, c[2] - h / 2))

def cyl(r, h, axis='z', c=(0, 0, 0), centered=True):
    d = {'x': V(1, 0, 0), 'y': V(0, 1, 0), 'z': V(0, 0, 1)}[axis] if isinstance(axis, str) else V(*axis)
    p = V(*c) - d * (h / 2 if centered else 0)
    return cq.Solid.makeCylinder(r, h, p, d)

def tube(ro, ri, h, axis='z', c=(0, 0, 0), centered=True):
    return cyl(ro, h, axis, c, centered).cut(cyl(ri, h * 1.01 + 1e-4, axis, c, centered))

def cone(r1, r2, h, axis='z', c=(0, 0, 0)):
    d = {'x': V(1, 0, 0), 'y': V(0, 1, 0), 'z': V(0, 0, 1)}[axis]
    return cq.Solid.makeCone(r1, r2, h, V(*c), d)

def sphere(r, c=(0, 0, 0)):
    return cq.Solid.makeSphere(r, V(*c), angleDegrees1=-90, angleDegrees2=90)

def revolve(pts, spline=False):
    """Revolve an (r, z) profile 360° about the Z axis."""
    wp = cq.Workplane('XZ')
    if spline:
        wp = wp.moveTo(*pts[0]).spline(pts[1:], includeCurrent=True).close()
    else:
        wp = wp.polyline(pts).close()
    return S(wp.revolve(360, (0, 0, 0), (0, 1, 0)))

def extrude_xy(pts, h, z0=0.0):
    """Extrude a closed XY polygon along +Z from z0."""
    return S(cq.Workplane('XY').workplane(offset=z0).polyline(pts).close().extrude(h))

def hexprism(af, h, c=(0, 0, 0), axis='z'):
    s = S(cq.Workplane('XY').polygon(6, af / math.cos(math.pi / 6)).extrude(h).translate((0, 0, -h / 2)))
    if axis == 'x':
        s = s.rotate(V(0, 0, 0), V(0, 1, 0), 90)
    elif axis == 'y':
        s = s.rotate(V(0, 0, 0), V(1, 0, 0), -90)
    return s.translate(V(*c))

def strut(p1, p2, r, ri=None):
    """Round (optionally hollow) member between two points."""
    a, b = V(*p1), V(*p2)
    d = b - a
    L = d.Length
    s = cq.Solid.makeCylinder(r, L, a, d.normalized())
    if ri:
        s = s.cut(cq.Solid.makeCylinder(ri, L * 1.01, a - d.normalized() * (L * 0.005), d.normalized()))
    return s

def sweep_circle(points, r):
    """Swept round section along a smooth spline path (hoses, cables)."""
    pts = [V(*p) for p in points]
    path = cq.Wire.assembleEdges([cq.Edge.makeSpline(pts)])
    t0 = (pts[1] - pts[0]).normalized()
    circ = cq.Wire.makeCircle(r, pts[0], t0)
    return cq.Solid.sweep(circ, [], path, makeSolid=True, isFrenet=False, transitionMode='right')

def union(*shapes):
    shapes = [S(s) for s in shapes if s is not None]
    out = shapes[0]
    for s in shapes[1:]:
        out = out.fuse(s)
    return out.clean()

def compound(*shapes):
    return cq.Compound.makeCompound([S(s) for s in shapes if s is not None])

def rot(shape, axis, deg, c=(0, 0, 0)):
    ax = {'x': (1, 0, 0), 'y': (0, 1, 0), 'z': (0, 0, 1)}[axis] if isinstance(axis, str) else axis
    return S(shape).rotate(V(*c), V(c[0] + ax[0], c[1] + ax[1], c[2] + ax[2]), deg)

def mv(shape, x=0, y=0, z=0):
    return S(shape).translate(V(x, y, z))

def polar(shape, n, axis='z', start=0.0):
    return compound(*[rot(shape, axis, start + 360.0 * k / n) for k in range(n)])

def naca4(code, n=36, closed_te=True):
    """NACA 4-digit section, chord 1, LE at x=0. Returns closed loop TE(upper)->LE->TE(lower)."""
    m, p, t = int(code[0]) / 100, int(code[1]) / 10, int(code[2:]) / 100
    a4 = -0.1036 if closed_te else -0.1015
    beta = [math.pi * i / (n - 1) for i in range(n)]
    xs = [(1 - math.cos(b)) / 2 for b in beta]
    up, lo = [], []
    for x in xs:
        yt = 5 * t * (0.2969 * math.sqrt(x) - 0.1260 * x - 0.3516 * x * x + 0.2843 * x ** 3 + a4 * x ** 4)
        if m and x < p:
            yc = m / p ** 2 * (2 * p * x - x * x); dy = 2 * m / p ** 2 * (p - x)
        elif m:
            yc = m / (1 - p) ** 2 * ((1 - 2 * p) + 2 * p * x - x * x); dy = 2 * m / (1 - p) ** 2 * (p - x)
        else:
            yc = dy = 0
        th = math.atan(dy)
        up.append((x - yt * math.sin(th), yc + yt * math.cos(th)))
        lo.append((x + yt * math.sin(th), yc - yt * math.cos(th)))
    return up[::-1] + lo[1:]

def airfoil_extrude(code, chord, span, axis='z', t_scale=1.0):
    pts = [(x * chord - chord / 4, y * chord * t_scale) for x, y in naca4(code, 24)]
    pts = pts[:-1]
    return S(cq.Workplane('XY').polyline(pts).close().extrude(span))

def loft_sections(wires, ruled=False):
    return cq.Solid.makeLoft(wires, ruled)

def spline_wire(points3, periodic=True):
    pts = [V(*p) for p in points3]
    e = cq.Edge.makeSpline(pts, periodic=periodic)
    return cq.Wire.assembleEdges([e])

# ---------------------------------------------------------------- fasteners (shared library)
def bolt_shape(d, L, head='hex'):
    if head == 'hex':
        hd = hexprism(1.6 * d, 0.65 * d, c=(0, 0, 0.325 * d))
    else:
        hd = cyl(0.75 * d, d, c=(0, 0, 0.5 * d))
    return union(hd, cyl(d / 2, L, c=(0, 0, -L / 2)))

def nut_shape(d):
    return hexprism(1.6 * d, 0.8 * d).cut(cyl(d / 2, d * 2))

def washer_shape(d):
    return tube(d, d * 0.54, 0.18 * d)

# ---------------------------------------------------------------- export
def _mat_mirror_z(p):
    return (p[0], p[1], -p[2])

def _shape_bbox(s):
    bb = s.BoundingBox()
    return [bb.xmin, bb.ymin, bb.zmin, bb.xmax, bb.ymax, bb.zmax]

def tessellate(shape, tol, ang=0.35):
    """Per-face tessellation (vertices are not shared across faces, giving crisp CAD edges).
    Triangles with a vertex outside the B-rep bounding box (degenerate faces from strut unions) are dropped."""
    verts, idx = [], []
    bb = shape.BoundingBox()
    m = 0.02 + 0.02 * max(bb.xlen, bb.ylen, bb.zlen)
    lo, hi = (bb.xmin - m, bb.ymin - m, bb.zmin - m), (bb.xmax + m, bb.ymax + m, bb.zmax + m)
    inside = lambda v: lo[0] <= v[0] <= hi[0] and lo[1] <= v[1] <= hi[1] and lo[2] <= v[2] <= hi[2]
    for f in shape.Faces():
        try:
            vs, tris = f.tessellate(tol, ang)
        except Exception:
            continue
        base = len(verts)
        pts = [(v.x, v.y, v.z) for v in vs]
        ok = [inside(p) for p in pts]
        verts.extend(pts)
        idx.extend((a + base, b + base, c + base) for a, b, c in tris if ok[a] and ok[b] and ok[c])
    used = sorted({i for t in idx for i in t})
    remap = {o: n for n, o in enumerate(used)}
    return [verts[i] for i in used], [(remap[a], remap[b], remap[c]) for a, b, c in idx]

def export(out_web, out_cad, meta, step_assembly=True, only=None, log=print,
           bom_name='Joukowsky-Dynamics-Rev08-BOM.csv', asm_name='Joukowsky-Dynamics-Rev08-road-configuration-assembly.step'):
    os.makedirs(out_web, exist_ok=True)
    os.makedirs(os.path.join(out_cad, 'step'), exist_ok=True)
    if not only:
        for f in os.listdir(os.path.join(out_cad, 'step')):
            os.remove(os.path.join(out_cad, 'step', f))
    t0 = time.time()
    built, report = {}, []
    mirrored_parts = set()
    # which parts appear under mirrored nodes (need LH variant)
    def node_mirrored(nid):
        n = NODES[nid]
        while n:
            if n['mirror']:
                return True
            n = NODES.get(n['parent']) if n['parent'] else None
        return False
    for pn, nid, *_ in INST:
        if node_mirrored(nid):
            mirrored_parts.add(pn)

    binbuf = bytearray()
    parts_out = []
    for i, pn in enumerate(_order):
        P = PARTS[pn]
        if only and pn not in only:
            continue
        status = 'B-rep solid'
        try:
            shp = S(P['builder']())
            if not shp.isValid():
                fixed = shp.fix()
                v0, v1 = abs(shp.Volume()), abs(fixed.Volume())
                if fixed.isValid() and v0 > 0 and abs(v1 - v0) / v0 < 0.2:
                    shp, status = fixed, 'B-rep (shape-healed)'
                else:
                    status = 'B-rep (boolean tolerance warning)'
                log(f'  .. {pn} {P["name"]}: {status}')
        except Exception as e:
            log(f'  !! {pn} {P["name"]}: {e}')
            traceback.print_exc()
            report.append(dict(pn=pn, error=str(e)))
            continue
        if not shp.Solids() and not shp.Faces():
            log(f'  !! {pn} {P["name"]}: empty shape')
            report.append(dict(pn=pn, error='empty shape'))
            continue
        offset = (0, 0, 0)
        if P['world']:
            bb = shp.BoundingBox()
            offset = ((bb.xmin + bb.xmax) / 2, (bb.ymin + bb.ymax) / 2, (bb.zmin + bb.zmax) / 2)
            shp = shp.translate(V(-offset[0], -offset[1], -offset[2]))
        built[pn] = (shp, offset)
        bb = _shape_bbox(shp)
        dims = [bb[3] - bb[0], bb[4] - bb[1], bb[5] - bb[2]]
        try:
            vol = shp.Volume()
        except Exception:
            vol = 0.0
        area = 0.0
        try:
            area = shp.Area()
        except Exception:
            pass
        try:
            c = shp.Center()
            com = [round(c.x, 5), round(c.y, 5), round(c.z, 5)]
        except Exception:
            com = [round((bb[0] + bb[3]) / 2, 5), round((bb[1] + bb[4]) / 2, 5), round((bb[2] + bb[5]) / 2, 5)]
        mat = MATERIALS[P['material']]
        if P['mass_override'] is not None:
            mass = P['mass_override']
        elif P['shell_t'] and P['shell_t'] < 0:          # solid with a laminate skin: area x |t|
            mass = area * -P['shell_t'] * mat['rho']
        elif P['shell_t']:                                # thin solid: half the area is one face
            mass = area / 2 * P['shell_t'] * mat['rho']
        else:
            mass = abs(vol) * mat['rho']
        # STEP per part (+ LH variant for mirrored placements)
        fname = f'{pn}.step'
        cq.exporters.export(cq.Workplane().add(shp.scale(1000)), os.path.join(out_cad, 'step', fname),
                            opt={'write_pcurves': False})
        lh = None
        if pn in mirrored_parts and P['handed']:
            lh = f'{pn}-LH.step'
            cq.exporters.export(cq.Workplane().add(shp.mirror('XY').scale(1000)),
                                os.path.join(out_cad, 'step', lh), opt={'write_pcurves': False})
        # mesh
        diag = math.sqrt(sum(d * d for d in dims)) or 1e-3
        tol = max(0.00025, diag * 0.0035)
        verts, idx = tessellate(shp, tol)
        if not verts:
            log(f'  !! {pn} produced no mesh')
        mn = [min(v[k] for v in verts) for k in range(3)] if verts else [0, 0, 0]
        mx = [max(v[k] for v in verts) for k in range(3)] if verts else [0, 0, 0]
        ext = [max(1e-9, mx[k] - mn[k]) for k in range(3)]
        while len(binbuf) % 4:
            binbuf.append(0)
        voff = len(binbuf)
        for v in verts:
            binbuf += struct.pack('<3h', *[int(round((v[k] - mn[k]) / ext[k] * 65534 - 32767)) for k in range(3)])
        while len(binbuf) % 4:
            binbuf.append(0)
        ioff = len(binbuf)
        wide = len(verts) > 65535
        fmt = '<3I' if wide else '<3H'
        for t in idx:
            binbuf += struct.pack(fmt, *t)
        parts_out.append(dict(
            pn=pn, name=P['name'], system=P['system'], assy=P['assy'], material=P['material'],
            desc=P['desc'], fn=P['fn'], process=P['process'], spec=P['spec'], criticality=P['criticality'],
            supplier=P['supplier'], handed=bool(P['handed'] and pn in mirrored_parts),
            dims_mm=[round(d * 1000, 1) for d in dims], com=com, volume_cm3=round(abs(vol) * 1e6, 2),
            area_cm2=round(area * 1e4, 1), mass_kg=round(mass, 4), basis=status,
            step=fname, step_lh=lh, faces=len(shp.Faces()),
            mesh=dict(voff=voff, nv=len(verts), ioff=ioff, ni=len(idx) * 3, wide=wide, min=mn, ext=ext),
            offset=offset))
        if (i + 1) % 25 == 0:
            log(f'  built {i + 1}/{len(_order)} parts  ({time.time() - t0:.0f}s, mesh {len(binbuf) / 1e6:.1f} MB)')

    pidx = {p['pn']: k for k, p in enumerate(parts_out)}
    nids = list(NODES)
    nidx = {n: k for k, n in enumerate(nids)}
    inst_out, qty = [], {}
    for pn, nid, pos, q, tag in INST:
        if pn not in pidx:
            continue
        off = built[pn][1]
        # world parts: geometry was re-centred; the instance carries the original centre
        p = q_apply(q, off)
        pos2 = (pos[0] + p[0], pos[1] + p[1], pos[2] + p[2])
        inst_out.append([pidx[pn], nidx[nid]] + [round(c, 5) for c in pos2] + [round(c, 6) for c in q])
        qty[pn] = qty.get(pn, 0) + 1
    for p in parts_out:
        p['qty'] = qty.get(p['pn'], 0)
    nodes_out = [dict(id=n, name=NODES[n]['name'], parent=nidx.get(NODES[n]['parent'], -1),
                      pos=[round(c, 5) for c in NODES[n]['pos']], q=[round(c, 6) for c in NODES[n]['q']],
                      mirror=NODES[n]['mirror'], anim=NODES[n]['anim'], system=NODES[n]['system'])
                 for n in nids]

    with open(os.path.join(out_web, 'meshes.bin'), 'wb') as f:
        f.write(binbuf)
    catalog = dict(meta=meta, materials=MATERIALS, nodes=nodes_out, parts=parts_out, instances=inst_out,
                   build=dict(seconds=round(time.time() - t0, 1), errors=report, mesh_bytes=len(binbuf)))
    with open(os.path.join(out_web, 'parts.json'), 'w') as f:
        json.dump(catalog, f, separators=(',', ':'))

    # BOM
    with open(os.path.join(out_cad, bom_name), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['part_number', 'name', 'system', 'assembly', 'qty', 'material', 'process', 'make_buy',
                    'unit_mass_kg', 'total_mass_kg', 'envelope_mm', 'step_file', 'lh_step_file', 'function'])
        for p in parts_out:
            w.writerow([p['pn'], p['name'], p['system'], p['assy'], p['qty'], MATERIALS[p['material']]['name'],
                        p['process'], p['supplier'], p['mass_kg'], round(p['mass_kg'] * p['qty'], 3),
                        'x'.join(str(d) for d in p['dims_mm']), 'step/' + p['step'],
                        ('step/' + p['step_lh']) if p['step_lh'] else '', p['fn']])

    if step_assembly:
        import zipfile
        name = asm_name
        tmp = os.path.join(out_cad, name)
        _export_assembly(built, nids, tmp, log)
        with zipfile.ZipFile(tmp + '.zip', 'w', zipfile.ZIP_DEFLATED, compresslevel=9) as z:
            z.write(tmp, name)
        os.remove(tmp)
        log(f'  zipped assembly: {os.path.getsize(tmp + ".zip") / 1e6:.1f} MB')
    log(f'export done in {time.time() - t0:.0f}s: {len(parts_out)} parts, {len(inst_out)} instances, '
        f'{len(nodes_out)} nodes, mesh {len(binbuf) / 1e6:.2f} MB, errors {len(report)}')
    return catalog

def _compose(a, b):
    """Rigid transforms as (pos, quat): returns a∘b."""
    pa, qa = a; pb, qb = b
    r = q_apply(qa, pb)
    return ((pa[0] + r[0], pa[1] + r[1], pa[2] + r[2]), q_mul(qa, qb))

def _mirror_conj(t):
    """M·T·M for the mirror z -> -z: still a rigid transform."""
    (px, py, pz), (x, y, z, w) = t
    return ((px, py, -pz), (-x, -y, z, w))

def _loc(t):
    p, q = t
    ax, ang = q_to_axis_angle(q)
    return cq.Location(V(p[0] * 1000, p[1] * 1000, p[2] * 1000)) * cq.Location(V(0, 0, 0), V(*ax), ang)

def _export_assembly(built, nids, path, log):
    """Full-assembly STEP in the road configuration. Mirrored corners reference LH (mirrored) solids."""
    root = cq.Assembly(name='Joukowsky-Dynamics-Rev08')
    rh, lh = {}, {}
    for k, (pn, nid, pos, q, tag) in enumerate(INST):
        if pn not in built:
            continue
        shp, off = built[pn]
        chain = []
        n = NODES[nid]
        while n:
            chain.append(n)
            n = NODES.get(n['parent']) if n['parent'] else None
        chain = chain[::-1]
        p = q_apply(q, off)
        T = ((pos[0] + p[0], pos[1] + p[1], pos[2] + p[2]), q)
        mirrored = False
        for n in reversed(chain):
            T = _compose((n['pos'], n['q']), _mirror_conj(T) if n['mirror'] else T)
            if n['mirror']:
                mirrored = not mirrored
        cache = lh if mirrored else rh
        if pn not in cache:
            s = shp.scale(1000)
            cache[pn] = s.mirror('XY') if mirrored else s
        color = cq.Color(MATERIALS[PARTS[pn]['material']]['color'])
        root.add(cache[pn], name=f'{pn}{"-LH" if mirrored else ""}#{k}', loc=_loc(T), color=color)
    root.save(path, exportType='STEP')
    log(f'  assembly STEP written: {os.path.getsize(path) / 1e6:.1f} MB')
