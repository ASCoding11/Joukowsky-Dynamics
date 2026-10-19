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
| **07 Proprotor & flight** | A live calculator built on NASA/TM-20240002116 *Fundamental Proprotor Design Considerations* (Blaesser, 2024). Hover is solved with blade-element momentum theory on the CADed blade (NACA 24xx lift and drag with stall, ducted momentum with the petals as a diffuser, collective trimmed to the weight), plus a battery model with cell internal resistance. A table of 15 physics checks (blade loading, stall, tip Mach, motor power and torque, thrust margin, cell current, voltage sag, endurance, CG, cruise tilt) marks each pass / marginal / fail. |
| **08 Miniature prototype** | A buildable flying model at about 1 : 11.5 with four 90 mm ducted fans: a 3D viewer with an 11-step build sequence that adds and highlights parts, a pod-conversion slider, a configurator (fan, ESC, battery, servo, battery position) with 15 checks computed from the CAD mass and CG, a print / cut list with STL and STEP links, a priced parts list and a wiring diagram. |

## Rev 08 CAD (`cad/`)

* `cad/source/build_rev08.py` — the parametric CadQuery / OpenCascade source of **every part**
  of Rev 08.1 (242 unique parts, 8,015 placements including every battery cell, 192 assembly nodes).
  `python cad/source/build_rev08.py` rebuilds everything; `--fast` skips the full-assembly STEP.
* `cad/source/proprotor.py` — blade design from the NASA memo (Eq. 4–8, 12, 23–28): 7 blades,
  R = 585 mm, σ = 0.36, concave quadratic chord, inflow-aligned twist, t/c 0.30 → 0.08 on NACA 24xx
  sections; it also holds the BEMT hover solver used to size the rotor (C_T/σ 0.099, FoM ≈ 0.75).
* `cad/source/build_mini.py` — the miniature prototype (24 parts). Writes `advanced/data/mini/`,
  `cad/mini/step/`, `cad/mini/stl/` (print-ready, with `-LH` copies of handed parts), a BOM and an
  assembly STEP.
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

## Rev 08.1 changes

Still a two-seat car. Rev 08.1 makes it lighter and the rotors bigger, and replaces the assumed
hover figure of merit with a computed one:

* Tyre radius 740 mm and rotor tip radius 585 mm (was 540 mm), with a thinner duct, tyre and petals.
* High-power 21700 cells (60 A continuous) in a 228s24p, 98.5 kWh pack, replacing 4680 cells that
  would have run at about 12C in hover. 320 kW peak / 210 kW continuous motors, 350 kW SiC inverters.
* Mass roll-up 2,069 kg empty, 2,249 kg with two occupants. Disk loading about 107 lb/ft².
* Hover by BEMT: about 884 kW shaft power, FoM 0.75, thrust margin 1.25 at peak motor power.
  13 of 15 checks pass. Two are marginal: the most-loaded motor needs about 226 kW in hover, above
  its 210 kW continuous rating (within the 320 kW peak), and hover endurance is about 3.8 min after
  a 20 % reserve.

## Miniature prototype

About 1 : 11.5: the same lofted body, hinge position and 90° pod swing, built from hobby parts.
Four 90 mm 6S ducted fans sit in printed duct rings with TPU tyres. Each pod swings on an 8 mm
shaft in two 608 bearings, driven by a 35 kg·cm servo, on a printed mast bolted to a 2 mm carbon
chassis plate. An ArduPilot flight controller, four 100 A ESCs and two 6S 5000 mAh packs in
parallel complete it.

* Default build (with the custom motors below): 5.3 kg, thrust-to-weight 1.80, about 4.4 min of
  hover, about $1,450 in parts plus a radio and charger if you don't own them. 14 of 16 checks
  pass. Two are marginal: thrust-to-weight (below the usual 2 : 1) and voltage sag at full throttle
  (3.27 V per cell). A single pack fails the sag check, which is why the build uses two. With the
  fans' stock motors instead: 5.6 kg, 1.70 : 1, 3.8 min, and a throttle cap is needed so a freshly
  charged pack can't overdrive the ESCs.
* Yaw comes from tilting the front and rear pods opposite ways by ±5°, because the fans' stator
  vanes cancel most of the rotor torque.
* It tests the conversion mechanism, clearances, CG, hover control and the arming interlock. It
  doesn't test road driving (the tyre rings are fixed to the ducts), the variable-pitch proprotor,
  or full-size dynamics.

### JDM-M40 custom fan motor

`cad/source/motor_design.py` sizes a brushless outrunner you build yourself, to fit the 90 mm fan's
motor hub (bell Ø40 mm). It sweeps 2.3 million candidates: slot / pole combination, stack length,
magnet thickness, tooth width, turns, star or delta, and parallel paths. For each one it works out:

* the magnetic circuit: air-gap, tooth, yoke and bell flux density
* the copper and phase resistance
* losses: copper, iron at the electrical frequency, eddy, bearing and windage
* the winding temperature in the fan's exhaust
* rotor bell stress at 115 % speed, and magnet demagnetisation margin

It matches each candidate to the fan's torque curve and keeps the lightest one that passes every
limit; 182 pass. Unbalanced 9-slot combinations are excluded because of their sideways magnetic
pull. It writes `advanced/data/mini/motor.json`, which `build_mini.py` uses for the motor CAD
(JDM-501 to 507) and tab 08 uses for its live operating points.

Result: 12 slots / 10 poles, delta, 10 turns of Ø0.46 mm per tooth (coil order AabBCcaABbcC),
Kv 1,829, 15.8 mΩ line-line. 0.2 mm NO20 laminations, 10 N45SH magnets 1.8 mm thick, a turned
4140 bell, two 685 hybrid ceramic bearings. About 193 g; about 90 % efficient in hover and at
full throttle. The winding runs about 82 °C in hover and reaches about 98 °C after 20 s at full
throttle. A fan with this motor is about 75 g lighter than the same fan with its stock motor, and the Kv is matched to the sagged pack,
so it gives more thrust there.

Prices in the workbench are typical hobby-shop figures, and fan, ESC and servo figures are typical
catalogue values. Check them against the parts you actually buy.

## Engineering status

Concept geometry for design exploration, not a flight-qualified design. The flight tab shows the
hover power, endurance and the levers (mass, disk area, a deployable wing for true axial cruise)
rather than hiding them. No tolerances, fatigue, aeroelastic, crash or fault-tolerance
substantiation has been done.
