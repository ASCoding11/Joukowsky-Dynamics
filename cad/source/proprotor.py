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
    mtow_kg=2250.0,          # Rev 08.1 CAD mass roll-up (2,069 kg) + 2 occupants (180 kg)
    n_rotors=4,
    R=0.585,                 # blade tip radius (Rev 08.1 duct inner radius 0.600 -> 15 mm tip gap)
    r0_R=0.20,               # airfoil starts at r/R = 0.2 (memo §3C; inboard is round shank)
    hub_R=0.13,              # hub / grip radius
    B=7,                     # blades
    sigma=0.36,              # rotor solidity: BEMT sweep minimum with C_T/sigma ~0.10 (memo §2B limit 0.12)
    r_m=0.55,                # radial station of maximum chord, r/R  (Eq. 26)
    c2=-0.45,                # quadratic chord coefficient (normalised by R)  (Eq. 23), concave (memo Fig. 11c)
    mu_design=0.30,          # twist aligned with this inflow ratio (memo §2E); ~hover inflow ratio of the duct
    tip_speed_hover=240.0,   # m/s, M_tip ~ 0.71
    cruise_rpm_ratio=0.92,   # no wing: rotors still carry weight in cruise (partial conversion, edgewise)
    v_cruise=44.4,           # m/s (160 km/h), rotor axes tilted up to 30 deg forward + lifting-body
    tc_root=0.30, tc_tip=0.08,  # monotonically decreasing t/c (memo §3C)
    camber='24',             # NACA 24xx family
    duct_thrust_share=0.51,  # = 1 - 1/(1 + eta_duct (2 sigma_d - 1)) with eta_duct 0.80, sigma_d 1.15
    eta_duct=0.80,           # duct lip / diffuser effectiveness vs ideal
    sigma_d=1.15,            # exit-area ratio with the inboard petals open as a diffuser  # fraction of fan-module thrust carried by the duct lip (ideal duct ~0.5)
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
    P_ideal_duct = W ** 1.5 / math.sqrt(4 * RHO * At * d['sigma_d'])   # ideal duct with exit-area ratio sigma_d
    trim = trim_hover(T_rotor, d['tip_speed_hover'], d)        # BEMT, defined below; resolved at call time
    fom = trim['FoM'] if trim else float('nan')
    omega_c = omega_h * d['cruise_rpm_ratio']
    mu_c = d['v_cruise'] / (omega_c * d['R'])                  # Eq. 7
    return dict(W_N=W, disk_area_m2=At, disk_loading_Pa=DL, disk_loading_psf=DL / 47.88,
                omega_hover=omega_h, rpm_hover=omega_h * 60 / (2 * math.pi), CT=CT,
                CT_sigma=CT / d['sigma'], vi_open=vi_open, lambda_hover=math.sqrt(T_rotor / (RHO * A)) / d['tip_speed_hover'],
                P_ideal_open_kW=P_ideal_open / 1e3, P_ideal_duct_kW=P_ideal_duct / 1e3,
                P_hover_kW=(4 * trim['P_shaft'] / 1e3) if trim else float('nan'), fom=fom,
                bemt_hover=dict((k, round(v, 4)) for k, v in trim.items()) if trim else None, omega_cruise=omega_c, mu_cruise=mu_c,
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

# ------------------------------------------------------------------ blade-element momentum theory (ducted, hover)
# Each annulus balances blade-element thrust against ducted-rotor momentum. With an ideal duct of exit-area ratio 1
# the wake does not contract; with exit-area ratio σd the rotor sees Δp = ½ρ(v/σd)², so dT_rotor = πρv²r dr/σd², and the
# duct adds η_d·(2σd − 1)·T_rotor (ideal η_d = 1: rotor carries 1/(2σd) of the total).
ETA_MOTOR, ETA_INVERTER = 0.955, 0.985

def polar(alpha_deg, tc):
    """NACA 24xx section at Re ~ 1-2e6: linear lift to stall, flat-plate post-stall, form-factor drag."""
    a = math.radians(alpha_deg)
    a0 = math.radians(-2.0)
    cla = 2 * math.pi * 0.92
    a_st = math.radians(14.0 - 25.0 * max(0.0, tc - 0.12))        # thick inboard sections stall earlier
    cd0 = 0.0065 * (1 + 2.0 * tc + 60 * tc ** 4)
    cl_lin = cla * (a - a0)
    cl_post = 1.1 * math.sin(2 * a)
    cd_post = 0.02 + 1.8 * math.sin(a) ** 2
    if -0.20 < a < a_st:
        return cl_lin, cd0 + 0.010 * (cl_lin - 0.25) ** 2
    blend = min(1.0, (a - a_st) / math.radians(4.0)) if a >= a_st else 1.0
    cl = cl_lin * (1 - blend) + cl_post * blend if a >= a_st else cl_post
    cd = (cd0 + 0.010 * (cl_lin - 0.25) ** 2) * (1 - blend) + cd_post * blend if a >= a_st else cd_post
    return cl, cd

def bemt_hover(theta75, tip_speed, d=DESIGN, eta_duct=None, sigma_d=None, n=36, rho=RHO, gap_loss=0.97):
    """sigma_d: exit-area ratio (open inboard petals form a diffuser). Ideal: T/T_rotor = 2·sigma_d, P = T^1.5/sqrt(4ρAσd)."""
    eta_duct = d['eta_duct'] if eta_duct is None else eta_duct
    sigma_d = d['sigma_d'] if sigma_d is None else sigma_d
    R = d['R']; om = tip_speed / R; r0 = d['r0_R'] * R
    Tr = Q = 0.0; amax = -99.0; stalled = 0.0
    for i in range(n):
        r = r0 + (R - r0) * (i + 0.5) / n; dr = (R - r0) / n
        x = r / R; c = chord(r, d); th = math.radians(twist(x, d) + theta75); tc = thickness(x, d)
        lo, hi = 0.0, 0.9 * tip_speed
        for _ in range(48):                       # bisection on axial velocity at the rotor
            v = 0.5 * (lo + hi)
            phi = math.atan2(v, om * r); alpha = math.degrees(th - phi)
            cl, cd = polar(alpha, tc)
            W2 = v * v + (om * r) ** 2
            dTb = 0.5 * rho * W2 * c * d['B'] * (cl * math.cos(phi) - cd * math.sin(phi)) * dr * gap_loss
            dTm = math.pi * rho * v * v * r * dr / sigma_d ** 2
            if dTb > dTm: lo = v
            else: hi = v
        Tr += dTb
        Q += 0.5 * rho * W2 * c * d['B'] * (cl * math.sin(phi) + cd * math.cos(phi)) * r * dr
        amax = max(amax, alpha)
        if alpha > math.degrees(math.radians(14.0 - 25.0 * max(0.0, tc - 0.12))): stalled += dr / (R - r0)
    T = Tr * (1 + eta_duct * (2 * sigma_d - 1))
    P = Q * om
    A = math.pi * R * R
    fom = T ** 1.5 / math.sqrt(4 * rho * A * sigma_d) / P if P > 0 else 0.0
    return dict(T=T, T_rotor=Tr, Q=Q, P_shaft=P, P_elec=P / (ETA_MOTOR * ETA_INVERTER), FoM=fom, alpha_max=amax,
                stalled_fraction=stalled, omega=om, CT_sigma=Tr / (rho * A * tip_speed ** 2) / d['sigma'])

def trim_hover(T_req, tip_speed, d=DESIGN, **kw):
    """Collective that produces T_req (total, incl. duct) at the given tip speed; None if not reachable."""
    lo, hi = -5.0, 45.0
    if bemt_hover(hi, tip_speed, d, **kw)['T'] < T_req:
        return None
    for _ in range(40):
        m = 0.5 * (lo + hi)
        if bemt_hover(m, tip_speed, d, **kw)['T'] < T_req: lo = m
        else: hi = m
    out = bemt_hover(hi, tip_speed, d, **kw); out['theta75'] = hi
    return out


if __name__ == '__main__':
    import json
    print(json.dumps(performance(), indent=1))
    print('chord coeffs', chord_coeffs())
    for st in blade_stations():
        print(['%.4f' % v for v in st])
