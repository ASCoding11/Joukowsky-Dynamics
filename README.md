# Joukowsky Dynamics — dual-mode fan-wheel vehicle

A road car whose four wheels swing up and become ducted proprotors. This repository holds the
interactive concept workbench and the CAD behind it.

The GitHub Pages site (https://ascoding11.github.io/Joukowsky-Dynamics/) opens on a password screen. The workbench is stored
encrypted in `vault.bin` (AES-256-GCM, key derived from the password with PBKDF2) and loads in place once the right password
is entered; a correct entry is remembered for that browser tab only. The unencrypted page is `web/workbench.html.tpl`, kept
out of the served pages on purpose. After changing the workbench, or to change the password, rebuild the vault:
`JD_PASSWORD=<password> python tools/build_vault.py vault.bin`.

To work on it locally, serve the repo (for example `python3 -m http.server`), open it and unlock it, then use the tabs:

| Tab | What it is |
| --- | --- |
| 01–04 | The original Rev 07 workbench: design studio, systems, materials, design register, with the A1-road part CAD drill-down (4,263 modelled items). |
| **05 Blueprint sequence** | A holographic, Iron-Man-style 60-second sequence: scan-in, assembly, road driving, legs down, petals open, wheels swing 90° into proprotors, spin-up, lift-off, partial conversion and cruise. Scrub, play backwards ("Land & stow"), switch Hologram / Material / X-ray. |
| **06 Advanced blueprint** | Every part of the Rev 08 vehicle: a part tree, a sortable BOM, mass & CG, and a 3D assembly where any click opens a datasheet with an orthographic drawing, material, process, mass and STEP links. Configuration slider (road → hover → cruise tilt), explode, half-section. |
| **07 Proprotor & flight** | A live calculator built on NASA/TM-20240002116 *Fundamental Proprotor Design Considerations* (Blaesser, 2024): inflow, angle of attack, chord solution space, edgewise flow, power vs speed, efficiency vs disk area, thrust split and control allocation. |

## Rev 08 CAD (`cad/`)

* `cad/source/build_rev08.py` — the parametric CadQuery / OpenCascade source of **every part**
  (242 unique parts, 3,607 placements, 188 assembly nodes). `python cad/source/build_rev08.py`
  rebuilds everything; `--fast` skips the full-assembly STEP.
* `cad/source/proprotor.py` — blade design from the NASA memo (Eq. 4–8, 12, 23–28): 7 blades,
  R = 540 mm, σ = 0.40 (sized so C_T/σ ≤ 0.12), concave quadratic chord, inflow-aligned twist,
  t/c 0.30 → 0.08 on NACA 24xx sections.
* `cad/source/body_sections.json` — 21 cross-sections sampled from the approved A1-road body
  surfaces; the Rev 08 body is lofted through them, so the exterior matches the A1 car.
* `cad/rev08/step/` — one STEP file per part (millimetres), plus `-LH` variants for handed parts
  used on the starboard corners.
* `cad/rev08/Joukowsky-Dynamics-Rev08-road-configuration-assembly.step.zip` — full assembly.
* `cad/rev08/Joukowsky-Dynamics-Rev08-BOM.csv` — bill of materials with masses and STEP paths.
* `cad/Joukowsky-Dynamics-A1-road-CAD-package.zip`, `cad/a1-baseline/` — the unchanged Rev 07
  A1-road package.

## How the wheel becomes a proprotor

Each corner is one module: an airless tyre on a filament-wound duct rim, seven stator vanes that
double as wheel spokes, a 7-blade proprotor on an axial-flux hub motor, and 12 + 12 iris petals
that cover the disc on the road.

1. **Road** — blades feathered at 90°, the dog clutch couples rotor to rim, so the motor drives the wheel.
2. **Prepare** — four landing legs lift the car 120 mm off its tyres.
3. **Unlock** — clutch releases; the brake caliper and two pins lock the rim; petals open into an
   inlet collector and exit diffuser; collective goes to hover.
4. **Convert** — a harmonic-drive actuator swings each module 90° up and outboard about a hinge
   above the tyre; the fender cap rides on the yoke.
5. **Fly** — rotors spin up (diagonal pairs counter-rotate). For cruise a second gimbal tilts each
   rotor axis up to 30° forward.

## Engineering status

Concept geometry for design exploration, not a flight-qualified design. The CAD mass roll-up is
about 2,220 kg empty (2,400 kg with two occupants). With 3.66 m² of total disk area that gives a
disk loading near 134 lb/ft² and roughly 1.2 MW to hover, so endurance is a few minutes on the
100 kWh pack. The flight tab shows these numbers and the levers (mass, disk area, a deployable
wing for true axial cruise) rather than hiding them. No tolerances, fatigue, aeroelastic, crash or
fault-tolerance substantiation has been done.
