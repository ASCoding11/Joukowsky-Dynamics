# Joukowsky Dynamics — dual-mode fan-wheel vehicle

A road car whose four wheels swing up and become ducted proprotors. This repository holds the
interactive concept workbench and the CAD behind it.

The workbench is public on GitHub Pages: https://ascoding11.github.io/Joukowsky-Dynamics/

To work on it locally, serve the repo (for example `python3 -m http.server`), open it, then use the tabs:

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

## Tideline: X-band SAR satellite for ports and shipping lanes (`satellite/`)

A second Joukowsky Dynamics concept, at https://ascoding11.github.io/Joukowsky-Dynamics/satellite/. It's a synthetic-aperture-radar
smallsat that watches ports, anchorages and shipping lanes through cloud, rain and darkness, built the same way as the car:
every part in CAD, and the physics computed live in the browser.

**Design**

* X-band (9.65 GHz), 560 km dawn-dusk sun-synchronous orbit (97.6°).
* A 4.1 × 0.75 m deployable active phased array: a centre panel plus two wings of two panels each, with 680 GaN T/R modules
  (4.1 kW peak), spring hinges with dampers and latches, and SMA hold-down releases.
* A 0.8 m honeycomb bus with a 15-inch rideshare ring.
* Power: a two-panel solar wing plus body-mounted cells (about 600 W), and an 8s12p 18650 battery (1.2 kWh) with a
  supercapacitor payload power unit.
* Four reaction wheels, star trackers, a 300 W krypton Hall thruster, a gimballed X-band horn (800 Mbit/s), S-band TT&C,
  an AIS receiver and an L-band relay modem.
* Wet mass about 134 kg.
* Modes:
  * spotlight, about 1.0 × 0.5 m;
  * stripmap, 3.0 × 2.3 m over a 30 km swath;
  * ScanSAR, about 7 × 12 m over a 120 km swath.

**Workbench tabs**

1. **Mission sequence:** a holographic sequence from separation, wing deployment and the roll to imaging attitude to imaging
   a port, on-board detection, the relay and the downlink.
2. **Advanced blueprint:** part tree, BOM, mass / CG / inertia (deployed and stowed) and a datasheet with a three-view
   drawing and STEP link for every part.
3. **SAR & mission physics:** the SAR radar equation (NESZ), numerically integrated range and azimuth ambiguities, the
   resolution, swath and PRF window, the smallest detectable ship from an X-band sea-clutter model, revisit vs latitude,
   data vs downlink, the X-band link budget, power and battery state of charge over an orbit, array thermal, slew time from
   the CAD inertia, drag, propellant and de-orbit. 27 checks with the CADed defaults: 26 pass. The one marginal is revisit:
   one satellite sees a mid-latitude port about every 39 h, so daily coverage needs a constellation of at least two
   (the satellites slider shows it).
4. **Port analytics:** a simulated SAR scene of a generic port, rendered at the chosen mode's resolution, noise floor and
   sea clutter. A two-pass CA-CFAR detector runs on it, then AIS matching (dark vessels), berth occupancy, the anchorage
   queue and waiting time, and channel-blockage detection. Scenarios: normal, congested, blocked channel, and storm at
   night with an optical comparison.
5. **Ground prototype:** a tabletop rail SAR (a 60 GHz FMCW board on a 1 m belt-driven rail) that images a model harbour
   through a cardboard "cloud", about $380 in parts. The browser simulates the scan and back-projection live.
   `satellite/prototype/rail_sar.py` processes real captures, or `--simulate`; its focus matches theory (5.0 mm
   cross-range and 4.5 cm range).

**CAD**

* `cad/source/build_sat.py`: 44 parts, 1,547 placements. Writes `satellite/data/`, `cad/sat/step/` and the BOM and
  assembly STEP.
* `cad/source/build_railsar.py`: 21 parts, with STL files for the printed ones in `cad/sat-proto/stl/`.

Sea clutter and ship radar cross-section are empirical fits, and the port is generic rather than a real place. Not modelled:
beam spoiling, nadir echoes, quantisation noise, ionosphere and detailed thermal transients.
