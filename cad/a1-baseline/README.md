# Joukowsky Dynamics concept CAD package

This package reconstructs the unedited A1 road configuration of the flying-car concept as a named STEP assembly. It preserves the approved sports-car exterior as a concept envelope and carries forward the illustrated systems. It is a geometric handoff, not a manufactured or flightworthy vehicle design.

## Contents and use

- **STEP assembly:** Open `Joukowsky-Dynamics-A1-road-baseline.step` in a STEP-capable CAD application. It contains 4,263 placed modeled items across 30 assembly groups: 4,235 solid items and 28 open surface items. These are modeled features, not a verified count of purchasable or manufactured parts. Shared prototype geometry is used where possible.
- **Native reconstruction macro:** Run `Joukowsky-Dynamics-editable-concept.FCMacro` in FreeCAD's macro interface, then save the resulting document as `.FCStd`. Boxes, suitable round cylinders, spheres and tori become native editable primitive objects. Other geometry becomes placed feature shapes, including faceted open panels and sampled-profile extrusions. FreeCAD execution was not available for interactive testing in this environment; macro syntax was checked, and the independent STEP reconstruction was tested with OpenCascade.
- **Geometry and parameters:** `Joukowsky-Dynamics-A1-road.cad.json` includes every item ID, geometry prototype, local primitive parameters, world placement, assembly reference, location label and material-appearance role. This is the editable geometric source, not a wiring or manufacturing specification.
- **Individual-item BOM:** The CSV lists each modeled item separately, including repeated cell envelopes, terminal caps, winding loops, rolling elements and fasteners. Generic names and counts reflect the visualization, not selected commercial components.
- **Reconstruction report:** The JSON report identifies geometry bases, fallback cases, validity checks and solid/open-surface counts.
- **STEP reconstruction script:** `reconstruct_STEP.py` can rebuild a STEP file from an exported geometry JSON, including edits. With Python and CadQuery available, run `python reconstruct_STEP.py Joukowsky-Dynamics-A1-road.cad.json regenerated.step`. This is the independent CAD-kernel path used for the baseline package; it is not a browser-side manufacturing solver.

## Geometry basis

The STEP was built with an OpenCascade CAD kernel via CadQuery. Geometry includes analytic boxes, cylinders, spheres and tori; profile extrusions; revolved surfaces; fitted spline tube sweeps; B-spline-fitted open panels; and explicitly faceted CAD solids/surfaces.

Five prototype formed-panel patches were fitted as B-spline CAD surfaces with a numerical fitting tolerance. Three other panel prototypes exceeded their source geometry envelopes when fitted and were retained as source faceted CAD surfaces instead. These are not a recovered original designer's NURBS control net or a validated aerodynamic surface. Extruded curved blade outlines use sampled profile points. Three tube prototypes that could not be reconstructed as valid swept solids were retained as source faceted geometry instead. Nothing was silently omitted because an analytic reconstruction failed.

The macro and STEP are not identical authoring histories. The macro deliberately retains general panels and some complex features as faceted geometry; the baseline STEP additionally includes fitted CAD surfaces and sweeps.

## Units and engineering limits

The disclosure did not establish production dimensions. Coordinates are normalized concept-model units. The package applies an explicit, editable conversion assumption of **1,000 mm per model unit**. This produces a nominal sports-car-scale envelope, not measured or approved dimensions.

The assembly has no solved mates, manufacturing tolerances, fits, thread definitions, joining specifications, wiring ratings, bearing selections, laminate plies, verified wall thicknesses, crash qualifications, thermal validation, rotor stress analysis or flight certification. Some source sheets have zero modeled thickness. Items overlap where the original concept used illustrative enclosures, layers or internal teaching geometry. No automatic Boolean fusion, interference correction or physical assembly constraint solution is implied.

The model reaches the smallest geometry that is actually represented. It does not invent subcomponents, material microstructure or production details to create the appearance of a complete engineered design.

The original visualization's 4,796-element count also included a separate 524-element enlarged inspection module and nine CG / schematic-routing annotations. These are not duplicated as physical items in the 4,263-item CAD vehicle assembly.

## Interactive subdivision

The attached workbench opens in **Part CAD / drill down**. If you have returned to the original visualization, use that button to re-enter CAD mode. Navigate vehicle → zone → assembly → feature family → individual item. Breadcrumbs and **Up one level** reverse the navigation. **Separate children** spreads the immediate children into an inspection layout. Clicking rendered geometry isolates the individual item, including a single instance from a repeated group.

Basic primitives expose editable dimensions. Other formed/faceted features expose local scale controls. A leaf is explicitly marked as the lowest modeled component. A session edit changes only that item and remains in the current tab's configuration-specific CAD session; it is not a persisted cloud CAD document and is not automatically applied to the separate original visualization. Download a native macro or JSON to retain edits before closing the tab.

Current-branch JSON, native macro, BOM and surface STL exports include session edits in assembled coordinates. STL remains triangulated and may contain open surfaces, so it is not automatically printable. The baseline STEP link always downloads the unedited A1 road design, not whatever branch or variant is currently selected.
