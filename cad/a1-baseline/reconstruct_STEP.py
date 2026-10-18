"""Reconstruct concept CAD with analytic primitives and explicit surface features.

Input is the workbench's geometry/placement JSON, not a dimensional engineering
specification.  Output is a named STEP assembly.  Open formed panels remain
open CAD faces/shells.  Failed analytic feature reconstruction falls back to
source triangle faces and is recorded in the report, never silently omitted.
"""
import argparse
import json
import math
from pathlib import Path

import cadquery as cq
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_Sewing
from OCP.BRepPrimAPI import BRepPrimAPI_MakeRevol
from OCP.GeomAbs import GeomAbs_C2
from OCP.GeomAPI import GeomAPI_PointsToBSplineSurface
from OCP.TColgp import TColgp_Array2OfPnt
from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf


def reconstruct(data, output):
    scale = float(data["mmPerModelUnit"])
    report = {"design": data["design"], "status": data["status"],
              "mmPerModelUnit_assumption": scale, "items": len(data["items"]),
              "geometry_prototypes": len(data["geometries"]), "basis": {},
              "fallbacks": [], "empty_features": [], "open_surface_items": 0,
              "solid_items": 0, "invalid_prototypes": []}

    def polygon(points):
        return cq.Wire.makePolygon([cq.Vector(x * scale, y * scale, 0)
                                    for x, y in points], close=True)

    def triangulated(g):
        v = g["vertices"]
        points = [cq.Vector(*[x * scale for x in v[i:i + 3]])
                  for i in range(0, len(v), 3)]
        faces = []
        for i in range(0, len(g["triangles"]), 3):
            a, b, c = [points[j] for j in g["triangles"][i:i + 3]]
            if (b - a).cross(c - a).Length < 1e-9:
                continue
            try:
                faces.append(cq.Face.makeFromWires(
                    cq.Wire.makePolygon([a, b, c], close=True)))
            except Exception:
                # Degenerate triangles have no area and no physical face.
                pass
        if not faces:
            raise ValueError("No nondegenerate CAD faces")
        sew = BRepBuilderAPI_Sewing(max(1e-6, scale * 1e-7))
        for face in faces:
            sew.Add(face.wrapped)
        sew.Perform()
        shape = cq.Shape.cast(sew.SewedShape())
        shells = shape.Shells()
        if len(shells) == 1:
            try:
                solid = cq.Solid.makeSolid(shells[0])
                if shells[0].Closed() and solid.isValid():
                    return solid, "faceted CAD solid"
            except Exception:
                pass
        return shape, "faceted CAD surface"

    def geometry(g):
        p, t = g["parameters"], g["type"]
        if t == "BoxGeometry":
            return cq.Solid.makeBox(p["width"] * scale, p["height"] * scale,
                                    p["depth"] * scale,
                                    cq.Vector(-p["width"] * scale / 2,
                                              -p["height"] * scale / 2,
                                              -p["depth"] * scale / 2)), "analytic box"
        if t == "CylinderGeometry" and not p.get("openEnded", False):
            r0, r1, h = p["radiusBottom"] * scale, p["radiusTop"] * scale, p["height"] * scale
            base, axis = cq.Vector(0, -h / 2, 0), cq.Vector(0, 1, 0)
            # Low-sided geometry (nuts / studs) is a prism, not a round cylinder.
            if p.get("radialSegments", 32) < 12:
                return triangulated(g)
            if abs(r0 - r1) < 1e-7:
                return cq.Solid.makeCylinder(r0, h, base, axis), "analytic cylinder"
            return cq.Solid.makeCone(r0, r1, h, base, axis), "analytic cone"
        if t == "SphereGeometry":
            return cq.Solid.makeSphere(p["radius"] * scale,
                                      angleDegrees1=-90, angleDegrees2=90), "analytic sphere"
        if t == "TorusGeometry" and p.get("tubularSegments", 48) >= 12:
            return cq.Solid.makeTorus(p["radius"] * scale, p["tube"] * scale), "analytic torus"
        if t == "ExtrudeGeometry":
            solids = []
            for profile in g["profiles"]:
                outer = polygon(profile["outline"])
                holes = [polygon(h) for h in profile["holes"]]
                solids.append(cq.Solid.extrudeLinear(
                    outer, holes, cq.Vector(0, 0, p["depth"] * scale)))
            result = solids[0] if len(solids) == 1 else cq.Compound.makeCompound(solids)
            if g.get("localTransform"):
                v = g["localTransform"]
                trsf = gp_Trsf()
                trsf.SetValues(v[0], v[4], v[8], v[12] * scale,
                               v[1], v[5], v[9], v[13] * scale,
                               v[2], v[6], v[10], v[14] * scale)
                result = result.moved(cq.Location(trsf))
            return result, "profile extrusion / sampled outline"
        if t == "TubeGeometry":
            pts = [cq.Vector(*[v * scale for v in xyz]) for xyz in g["path"]]
            edge = cq.Edge.makeSpline(pts)
            path = cq.Wire.assembleEdges([edge])
            tangent = edge.tangentAt(0)
            profile = cq.Wire.makeCircle(p["radius"] * scale, pts[0], tangent)
            swept = cq.Solid.sweep(profile, [], path, True, False)
            if swept.isValid():
                return swept, "spline-fit tube sweep"
            raise ValueError("Sweep validation failed")
        if t == "LatheGeometry":
            points = [cq.Vector(x * scale, y * scale, 0) for x, y in g["profile"]]
            wire = cq.Wire.makePolygon(points)
            revol = BRepPrimAPI_MakeRevol(wire.wrapped,
                                        gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0)),
                                        p.get("phiLength", math.tau))
            return cq.Shape.cast(revol.Shape()), "revolved CAD surface"
        if g.get("grid"):
            nu, nv = g["grid"]["nu"], g["grid"]["nv"]
            arr = TColgp_Array2OfPnt(1, nu + 1, 1, nv + 1)
            for i in range(nu + 1):
                for j in range(nv + 1):
                    k = (i * (nv + 1) + j) * 3
                    arr.SetValue(i + 1, j + 1,
                                 gp_Pnt(*[v * scale for v in g["vertices"][k:k + 3]]))
            fitted = GeomAPI_PointsToBSplineSurface(arr, 3, 3, GeomAbs_C2,
                                                   max(.001, scale * 1e-5))
            face = BRepBuilderAPI_MakeFace(fitted.Surface(), 1e-7)
            result = cq.Shape.cast(face.Face())
            if result.isValid():
                return result, "B-spline-fitted open CAD panel"
            raise ValueError("Fitted panel validation failed")
        return triangulated(g)

    cache = {}
    for key, g in data["geometries"].items():
        try:
            shape, basis = geometry(g)
            if not shape.isValid():
                raise ValueError("Reconstructed feature invalid")
            if g["type"] in ("TubeGeometry", "BufferGeometry") and basis != "faceted CAD surface":
                # A mathematically valid spline can still overshoot the design.
                # Reject such fits rather than distorting the approved envelope.
                bounds = shape.BoundingBox()
                cad_bounds = [(bounds.xmin, bounds.xmax), (bounds.ymin, bounds.ymax),
                              (bounds.zmin, bounds.zmax)]
                for axis, (low, high) in enumerate(cad_bounds):
                    coords = [v * scale for v in g["vertices"][axis::3]]
                    source_low, source_high = min(coords), max(coords)
                    pad = max(scale * .001, (source_high - source_low) * .05)
                    if low < source_low - pad or high > source_high + pad:
                        raise ValueError("Spline fit exceeded source geometry envelope; retained source faces")
        except Exception as exc:
            report["fallbacks"].append({"geometry": key, "type": g["type"],
                                        "reason": str(exc)[:200]})
            shape, basis = triangulated(g)
        if not shape.isValid():
            report["invalid_prototypes"].append(key)
        cache[key] = (shape, basis)
        report["basis"][basis] = report["basis"].get(basis, 0) + 1
        if len(cache) % 40 == 0:
            print(f"Reconstructed {len(cache)} / {len(data['geometries'])} prototypes", flush=True)

    assembly = cq.Assembly(name="Joukowsky_Dynamics_CONCEPT_NOT_PRODUCTION")
    groups = {}
    scaled_cache = {}
    roles = {"paint": (.48, .06, .12), "carbon": (.08, .1, .12),
             "rubber": (.05, .06, .07), "metal": (.54, .59, .64),
             "copper": (.71, .43, .26), "glass": (.08, .16, .2),
             "ceramic": (.19, .23, .26)}
    for index, item in enumerate(data["items"]):
        shape, basis = cache[item["geometry"]]
        m = item["matrix"]
        scales = [math.sqrt(sum(m[i + j * 4] ** 2 for i in range(3)))
                  for j in range(3)]
        skey = (item["geometry"], *[round(s, 7) for s in scales])
        if skey not in scaled_cache:
            if max(scales) - min(scales) < 1e-6:
                scaled_cache[skey] = shape.scale(scales[0])
            else:
                mat = cq.Matrix([[scales[0], 0, 0, 0], [0, scales[1], 0, 0],
                                 [0, 0, scales[2], 0], [0, 0, 0, 1]])
                scaled_cache[skey] = shape.transformGeometry(mat)
        s = scaled_cache[skey]
        trsf = gp_Trsf()
        trsf.SetValues(m[0] / scales[0], m[4] / scales[1], m[8] / scales[2], m[12] * scale,
                       m[1] / scales[0], m[5] / scales[1], m[9] / scales[2], m[13] * scale,
                       m[2] / scales[0], m[6] / scales[1], m[10] / scales[2], m[14] * scale)
        key = item["assembly"]
        if key not in groups:
            groups[key] = cq.Assembly(name=f"REF_{key}_{item['zone']}")
        name = f"{item['id'].replace(':', '_')}_{item['name'].split(' · ')[0].replace('/', '_')}"
        rgb = roles.get(item["material"], roles["metal"])
        groups[key].add(s, name=name, loc=cq.Location(trsf),
                        color=cq.Color(*rgb))
        if shape.Solids():
            report["solid_items"] += 1
        else:
            report["open_surface_items"] += 1
        if index and index % 1000 == 0:
            print(f"Placed {index} / {len(data['items'])} items", flush=True)
    for group in groups.values():
        assembly.add(group)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    assembly.save(str(output), exportType="STEP", mode="default")
    report["step_bytes"] = output.stat().st_size
    report["assembly_groups"] = len(groups)
    report_path = output.with_suffix(".report.json")
    report_path.write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2), flush=True)
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("input")
    parser.add_argument("output")
    args = parser.parse_args()
    reconstruct(json.loads(Path(args.input).read_text()), args.output)
