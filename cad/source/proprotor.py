"""Proprotor blade design for the Joukowsky fan-wheel, following
NASA/TM-20240002116 "Fundamental Proprotor Design Considerations" (Blaesser, 2024).

All relations are referenced to the memorandum's equation numbers. Lengths in metres.
The same baseline is serialised into parts.json so the browser calculator starts from the
exact geometry that was CADed.
"""
import math

RHO = 1.225

# ------------------------------------------------------------------ baseline design point
DESIGN = dict(
    mtow_kg=2400.0,          # CAD mass roll-up (~2,215 kg) + 2 occupants, rounded up
    n_rotors=4,
    R=0.540,                 # blade tip radius (duct inner radius 0.556 -> 16 mm tip gap)
    r0_R=0.20,               # airfoil starts at r/R = 0.2 (memo §3C; inboard is round shank)
    hub_R=0.13,              # hub / grip radius
    B=7,                     # blades
    sigma=0.40,              # rotor solidity: sized so C_T/sigma <= 0.12 (memo §2B, Leishman limit)
    r_m=0.55,                # radial station of maximum chord, r/R  (Eq. 26)
    c2=-0.45,                # quadratic chord coefficient (normalised by R)  (Eq. 23), concave (memo Fig. 11c)
    mu_design=0.25,          # twist aligned with this inflow ratio (memo §2E); ~hover inflow ratio of the duct
    tip_speed_hover=250.0,   # m/s, M_tip ~ 0.73
    cruise_rpm_ratio=0.92,   # no wing: rotors still carry weight in cruise (partial conversion, edgewise)
    v_cruise=44.4,           # m/s (160 km/h), rotor axes tilted up to 30 deg forward + lifting-body
    tc_root=0.30, tc_tip=0.08,  # monotonically decreasing t/c (memo §3C)
    camber='24',             # NACA 24xx family
    duct_thrust_share=0.45,  # fraction of fan-module thrust carried by the duct lip (ideal duct ~0.5)
)

def chord_coeffs(d=DESIGN):
    """Quadratic chord c(r)=c2 r^2 + c1 r + c0 (normalised by R), Eqs. 23, 26, 28."""
    s_B = d['sigma'] / d['B']
    r0, rm, c2 = d['r0_R'], d['r_m'], d['c2']
    c1 = -2 * c2 * rm                                                      # Eq. 26
    c0 = s_B * math.pi / (1 - r0) - c2 * (1 + r0 + r0 ** 2) / 3 + c2 * rm * (1 + r0)   # Eq. 28 (R = 1)
    return c2, c1, c0

def chord(r, d=DESIGN):
    """Chord in metres at radius r (m)."""
    c2, c1, c0 = chord_coeffs(d)
    x = r / d['R']
    return (c2 * x * x + c1 * x + c0) * d['R']

def phi(r_R, mu):
    """Inflow angle, Eq. 8 (degrees)."""
    return math.degrees(math.atan2(mu, r_R))

def twist(r_R, d=DESIGN):
    """Geometric twist beta(r) aligned with the design inflow ratio (memo §2E), referenced to 3/4 R."""
    return phi(r_R, d['mu_design']) - phi(0.75, d['mu_design'])

def thickness(r_R, d=DESIGN):
    t = (r_R - d['r0_R']) / (1 - d['r0_R'])
    t = min(1, max(0, t))
    return d['tc_root'] + (d['tc_tip'] - d['tc_root']) * (1 - (1 - t) ** 1.6)

def solidity_check(d=DESIGN, n=400):
    """Numerically integrate Eq. 5 and return sigma from Eq. 4."""
    R, r0 = d['R'], d['r0_R'] * d['R']
    Ab = sum(chord(r0 + (R - r0) * (i + 0.5) / n, d) for i in range(n)) * (R - r0) / n
    return d['B'] * Ab / (math.pi * R * R)

def performance(d=DESIGN):
    W = d['mtow_kg'] * 9.80665
    A = math.pi * d['R'] ** 2
    At = A * d['n_rotors']
    T_rotor = W / d['n_rotors']
    DL = W / At                                                # Eq. 2: disk loading == delta p
    omega_h = d['tip_speed_hover'] / d['R']
    T_blade_rotor = T_rotor * (1 - d['duct_thrust_share'])
    CT = T_blade_rotor / (RHO * A * d['tip_speed_hover'] ** 2)  # Eq. 6
    vi_open = math.sqrt(T_rotor / (2 * RHO * A))               # momentum theory, open rotor
    P_ideal_open = W * math.sqrt(W / (2 * RHO * At))
    P_ideal_duct = W ** 1.5 / math.sqrt(4 * RHO * At)          # ideal duct, expansion ratio 1
    fom = 0.72
    omega_c = omega_h * d['cruise_rpm_ratio']
    mu_c = d['v_cruise'] / (omega_c * d['R'])                  # Eq. 7
    return dict(W_N=W, disk_area_m2=At, disk_loading_Pa=DL, disk_loading_psf=DL / 47.88,
                omega_hover=omega_h, rpm_hover=omega_h * 60 / (2 * math.pi), CT=CT,
                CT_sigma=CT / d['sigma'], vi_open=vi_open, lambda_hover=math.sqrt(T_rotor / (RHO * A)) / d['tip_speed_hover'],
                P_ideal_open_kW=P_ideal_open / 1e3, P_ideal_duct_kW=P_ideal_duct / 1e3,
                P_hover_kW=P_ideal_duct / fom / 1e3, fom=fom, omega_cruise=omega_c, mu_cruise=mu_c,
                rpm_cruise=omega_c * 60 / (2 * math.pi), sigma_integrated=solidity_check(d),
                delta_phi_max_mu=math.sqrt(d['r0_R']))

def blade_stations(d=DESIGN, n=11):
    """Radial stations for the lofted blade: (r_m, chord_m, pitch_deg, t_over_c)."""
    R = d['R']
    out = []
    r0 = d['r0_R']
    for i in range(n):
        s = i / (n - 1)
        x = r0 + (0.985 - r0) * s
        out.append((x * R, chord(x * R, d), twist(x, d), thickness(x, d)))
    return out

if __name__ == '__main__':
    import json
    print(json.dumps(performance(), indent=1))
    print('chord coeffs', chord_coeffs())
    for st in blade_stations():
        print(['%.4f' % v for v in st])
