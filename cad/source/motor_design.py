"""JDM-M40: a build-it-yourself brushless outrunner for the miniature prototype's 90 mm ducted fans.

Run:  python motor_design.py          (prints the trade study and writes advanced/data/mini/motor.json)

The motor has to fit the fan's motor hub (bell Ø40 mm), turn the 12-blade rotor to about 37,500 rpm on a sagging
6S pack, and survive minutes of hover in the fan's own exhaust. The script sweeps slot/pole combination, stack
length, magnet thickness, tooth width and turns, sizes each candidate analytically and keeps the lightest one that
passes every limit. Model, per candidate:

* air-gap flux from the magnet load line (N45SH, hot remanence) with Carter and leakage factors
* flux densities in the teeth, stator yoke and rotor bell, each held below its saturation limit
* slot area and copper from the lamination geometry; concentrated double-layer winding, hand-wound fill factor
* back-EMF and Kv from the fundamental flux and the winding factor; phase resistance at temperature
* losses: copper (I²R, two phases conducting), iron (Steinmetz for 0.2 mm NO20 steel at the electrical frequency),
  magnet and bell eddy losses (scaled with frequency²), bearing and windage drag
* fan load: Q = Q_r (ω/ω_r)², thrust T = T_r (ω/ω_r)²; operating point where motor torque meets fan torque
* temperature: lumped convection to the exhaust stream; thermal mass for full-throttle bursts
* rotor bell hoop stress at 115 % of the full-charge speed on the fan; magnet demagnetisation field at burst current
"""
import json, math, os, itertools

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(os.path.dirname(HERE)), 'advanced', 'data', 'mini', 'motor.json')

MU0 = 4e-7 * math.pi
RHO_CU20 = 1.72e-8
# fan: 90 mm 12-blade rotor, catalogue point 3.4 kg static at ~37,500 rpm taking ~1.85 kW of shaft power
FAN = dict(T_r=3.4 * 9.81, rpm_r=37500.0, P_r=1850.0)
FAN['w_r'] = FAN['rpm_r'] * 2 * math.pi / 60
FAN['Q_r'] = FAN['P_r'] / FAN['w_r']
LIMITS = dict(bell_od=0.040, len_max=0.036, bore=0.012, B_tooth=1.65, B_yoke=1.5, B_bell=1.65, fill=0.40,
              T_amb=40.0, I_full_max=85.0, T_wind_max=150.0, T_hover_max=110.0, burst_s=20.0, erpm_max=300e3, sf_bell=2.0)
MAG = dict(Br20=1.33, alpha=-0.0012, mur=1.05, Hcj120=0.90e6, rho=7500.0)   # N45SH
STEEL = dict(rho=7650.0, k=12.0, f0=400.0, a=1.45, b=2.0)                    # NO20: ~12 W/kg at 1 T, 400 Hz
BELL = dict(rho=7850.0, yield_=650e6)                                         # 4140 pre-hardened tube (28–32 HRC), turned
# balanced combinations only: 9N8P / 9N10P have a higher winding factor but an unbalanced magnetic pull that shakes
# the rotor at 35,000 rpm
WINDING_FACTOR = {(12, 10): .933, (12, 14): .933, (12, 8): .866, (9, 6): .866, (6, 4): .866}
# coil order from the star of slots (tooth k sits at k × slot angle electrical; lower case = reversed)
COIL_PATTERN = {(12, 10): 'AabBCcaABbcC', (12, 14): 'AacCBbaACcbB', (12, 8): 'ABCABCABCABC', (9, 6): 'ABCABCABC',
                (9, 8): 'AaABbBCcC', (9, 10): 'AaABbBCcC', (6, 4): 'ABCABC'}

def design(Ns, P, L, hm, wt_frac, N, strands, conn='star', paths=1, g=0.0004, t_bell=0.0024, T_wind=110.0):
    """Size one candidate. Returns None if it does not fit, else a dict of geometry and constants."""
    p = P // 2
    r_out = LIMITS['bell_od'] / 2
    r_mag_o = r_out - t_bell
    r_mag_i = r_mag_o - hm
    r_s = r_mag_i - g                              # stator outer radius
    Br = MAG['Br20'] * (1 + MAG['alpha'] * (100 - 20))   # magnets run ~100 °C
    tau_s = 2 * math.pi * r_s / Ns
    slot_open = 0.0016
    carter = tau_s / (tau_s - (slot_open ** 2 / (5 * g + slot_open)))
    g_eff = g * carter
    Bg = 0.93 * Br * hm / (hm + MAG['mur'] * g_eff)
    tau_p = 2 * math.pi * r_s / P
    alpha_p = 0.86                                 # magnet arc / pole pitch
    wt = wt_frac * tau_s
    B_tooth = Bg * tau_s / wt
    phi_pole = Bg * alpha_p * tau_p * L
    h_y = phi_pole / (2 * LIMITS['B_yoke'] * L) * 1.05
    B_bell = phi_pole / (2 * t_bell * L)
    tip = 0.0009
    r_yo = LIMITS['bore'] / 2 + 0.0012 + h_y       # stator yoke outer radius (bore + insulation sleeve + yoke)
    depth = r_s - tip - r_yo
    if depth < 0.0025 or B_tooth > LIMITS['B_tooth'] or B_bell > LIMITS['B_bell']:
        return None
    slot_area = (math.pi * ((r_s - tip) ** 2 - r_yo ** 2) - Ns * wt * depth) / Ns
    if slot_area <= 0 or wt * Ns > 2 * math.pi * r_yo * 0.92:
        return None
    A_cu_slot = LIMITS['fill'] * slot_area
    A_turn = A_cu_slot / (2 * N)                   # two coil sides per slot (copper per conductor of one coil)
    d_wire = math.sqrt(A_turn / strands / math.pi) * 2
    kw = WINDING_FACTOR[(Ns, P)]
    if (Ns // 3) % paths: return None
    N_ph = Ns // 3 * N // paths                    # series turns per phase (coils split into parallel paths)
    B1 = 4 / math.pi * Bg * math.sin(alpha_p * math.pi / 2)
    phi1 = 2 / math.pi * B1 * tau_p * L
    Ke_ph = p * kw * N_ph * phi1                   # V per mech rad/s, phase peak
    Ke = (math.sqrt(3) if conn == 'star' else 1.0) * Ke_ph * 0.95   # line-line, as a trapezoidal ESC sees it
    Kv = 60 / (2 * math.pi * Ke)
    MLT = 2 * L + 2 * (wt + 0.0022) + math.pi * depth * 0.5
    rho_cu = RHO_CU20 * (1 + 0.00393 * (T_wind - 20))
    R_ph = rho_cu * N_ph * MLT / (A_turn * paths)
    R_ll = (2 * R_ph if conn == 'star' else 2 / 3 * R_ph) + 0.003   # + leads and solder joints
    # masses
    m_cu = 8960 * Ns * 2 * N * A_turn * MLT / 2
    A_lam = math.pi * (r_yo ** 2 - (LIMITS['bore'] / 2 + 0.0012) ** 2) + Ns * (wt * depth + (tau_s - slot_open) * tip)
    m_fe = STEEL['rho'] * A_lam * L * 0.96
    m_tooth = STEEL['rho'] * Ns * wt * depth * L * 0.96
    m_mag = MAG['rho'] * math.pi * (r_mag_o ** 2 - r_mag_i ** 2) * alpha_p * L
    L_bell = L + 0.007
    m_bell = BELL['rho'] * (math.pi * (r_out ** 2 - r_mag_o ** 2) * L_bell + math.pi * r_out ** 2 * 0.0015 * 0.55)
    m_misc = 0.012 + 0.010 + 0.012 + 0.006         # shaft, two bearings, stator hub, adaptor + leads
    m = m_cu + m_fe + m_mag + m_bell + m_misc
    return dict(Ns=Ns, P=P, p=p, L=L, hm=hm, g=g, t_bell=t_bell, r_out=r_out, r_s=r_s, r_mag_i=r_mag_i, r_yo=r_yo,
                depth=depth, tip=tip, wt=wt, slot_open=slot_open, h_y=h_y, Bg=Bg, B_tooth=B_tooth, B_bell=B_bell,
                B_yoke=phi_pole / (2 * h_y * L), N=N, strands=strands, d_wire=d_wire, kw=kw, Ke=Ke, Kv=Kv, R=R_ll,
                MLT=MLT, slot_area=slot_area, m=m, m_cu=m_cu, m_fe=m_fe, m_tooth=m_tooth, m_mag=m_mag, m_bell=m_bell,
                L_bell=L_bell, pattern=COIL_PATTERN[(Ns, P)], alpha_p=alpha_p, conn=conn, paths=paths, N_ph=N_ph)

def losses_fixed(d, w):
    """Speed-dependent losses (W) at mechanical speed w: iron, rotor eddy, bearings and windage."""
    f = d['p'] * w / (2 * math.pi)
    k = STEEL['k'] * (f / STEEL['f0']) ** STEEL['a']
    P_fe = k * (d['m_tooth'] * d['B_tooth'] ** STEEL['b'] + (d['m_fe'] - d['m_tooth']) * d['B_yoke'] ** STEEL['b']) * 1.3   # 1.3: punching damage
    P_eddy = 2.2e-6 * f ** 2 * d['L'] / 0.025      # magnets in 2 axial segments; bell back iron sees slot ripple only
    P_mech = 1.6 * (w / 3927) + 2.4 * (w / 3927) ** 3   # 2 hybrid ceramic bearings + bell windage
    return P_fe, P_eddy, P_mech

def operate(d, V=None, thrust=None):
    """Operating point on the fan. Give either a bus voltage at full duty, or a thrust (N) to find the duty needed."""
    Kt = d['Ke']
    def at_w(w):
        Q = FAN['Q_r'] * (w / FAN['w_r']) ** 2
        Pf = sum(losses_fixed(d, w))
        I = (Q + Pf / w) / Kt
        Vm = w * d['Ke'] + I * d['R']
        return Q, I, Vm, Pf
    if thrust is not None:
        w = FAN['w_r'] * math.sqrt(thrust / FAN['T_r'])
    else:
        lo, hi = 100.0, 9000.0
        for _ in range(80):
            w = (lo + hi) / 2
            if at_w(w)[2] > V: hi = w
            else: lo = w
    Q, I, Vm, Pf = at_w(w)
    P_cu = I ** 2 * d['R']
    P_sh = Q * w
    P_in = Vm * I
    return dict(w=w, rpm=w * 60 / 2 * 1 / math.pi, Q=Q, I=I, V=Vm, P_sh=P_sh, P_in=P_in, P_cu=P_cu, P_fixed=Pf,
                eff=P_sh / P_in, T=FAN['T_r'] * (w / FAN['w_r']) ** 2, f_e=d['p'] * w / (2 * math.pi))

def thermal_area(d):
    return 2 * math.pi * d['r_out'] * d['L_bell'] + math.pi * d['r_out'] ** 2 + 0.6 * 2 * math.pi * d['r_s'] * d['L']   # bell + face + through-flow

def thermal(d, op):
    """Steady winding temperature in the exhaust stream, and the rise over a full-throttle burst."""
    A = thermal_area(d)
    v_air = 18 + 40 * (op['w'] / FAN['w_r'])        # air speed over the hub fairing, m/s
    h = 22 * v_air ** 0.8                           # flat-plate turbulent correlation, W/m²K
    loss = op['P_cu'] + op['P_fixed']
    R_th = 1 / (h * A) + 0.35                       # + winding-to-case resistance, K/W
    C_th = 385 * d['m_cu'] + 460 * (d['m_fe'] + d['m_bell']) + 900 * 0.008
    return dict(dT=loss * R_th, loss=loss, R_th=R_th, C_th=C_th, h=h)

def evaluate(d, V_full=20.2, T_hover=1.37 * 9.81):
    full = operate(d, V=V_full)
    hov = operate(d, thrust=T_hover)
    th_h, th_f = thermal(d, hov), thermal(d, full)
    T_hov = LIMITS['T_amb'] + th_h['dT']
    tau = th_f['R_th'] * th_f['C_th']
    T_burst = T_hov + (th_f['dT'] - th_h['dT']) * (1 - math.exp(-LIMITS['burst_s'] / tau))
    w_nl = 25.2 / d['Ke']                           # full charge, no load (fan removed: never run it like this)
    w_os = 1.15 * operate(d, V=25.2)['w']           # overspeed case: 115 % of full-charge speed on the fan
    r_m = d['r_mag_i'] + d['hm'] / 2
    p_mag = MAG['rho'] * d['hm'] * r_m * w_os ** 2 * d['alpha_p']
    s_bell = p_mag * (d['r_out'] - d['t_bell'] / 2) / d['t_bell'] + BELL['rho'] * (w_os * d['r_out']) ** 2
    I_burst = 1.5 * full['I']
    H_arm = d['N'] * I_burst * 2 / (2 * (d['g'] + d['hm']))   # two coils' MMF across the gap and magnet, A/m
    erpm = w_nl * 60 / (2 * math.pi) * d['p']
    return dict(full=full, hover=hov, T_hover=T_hov, T_burst=T_burst, tau=tau, s_bell=s_bell, sf_bell=BELL['yield_'] / s_bell,
                H_arm=H_arm, demag_margin=MAG['Hcj120'] / H_arm, erpm=erpm, rpm_nl=w_nl * 60 / (2 * math.pi), rpm_os=w_os * 60 / (2 * math.pi))

def ok(d, e):
    return (e['full']['T'] >= FAN['T_r'] * 0.85 and e['full']['I'] <= LIMITS['I_full_max'] and e['T_hover'] <= LIMITS['T_hover_max'] and e['T_burst'] <= LIMITS['T_wind_max']
            and e['sf_bell'] >= LIMITS['sf_bell'] and e['demag_margin'] >= 1.5 and e['erpm'] <= LIMITS['erpm_max']
            and 0.25e-3 <= d['d_wire'] <= 0.65e-3)

def sweep():
    best = None
    tried = passed = 0
    for (Ns, P), L, tb, hm, wf, strands, conn, paths in itertools.product(WINDING_FACTOR, [x / 1000 for x in range(16, 37, 2)], [0.0022, 0.0026, 0.0030],
                                                          [0.0018, 0.0022, 0.0026, 0.0030], [0.42, 0.48, 0.54, 0.60], [1, 2, 4, 8, 12],
                                                          ['star', 'delta'], [1, 2, 4]):
        for N in range(1, 30):
            tried += 1
            d = design(Ns, P, L, hm, wf, N, strands, conn, paths, t_bell=tb)
            if not d: continue
            e = evaluate(d)
            if not ok(d, e): continue
            passed += 1
            score = d['m'] + 0.004 * (e['hover']['P_in'] - e['hover']['P_sh'])   # 4 g per watt of hover loss ≈ battery mass to cover it
            if best is None or score < best[0]:
                best = (score, d, e)
    return best, tried, passed

def curves(d, V_full):
    pts = []
    for k in range(1, 21):
        T = FAN['T_r'] * k / 20 * 1.15
        op = operate(d, thrust=T)
        if op['V'] > V_full * 1.001: break
        pts.append([round(op['T'] / 9.81, 3), round(op['eff'], 4), round(op['I'], 2), round(op['rpm']), round(op['P_cu'] + op['P_fixed'], 1)])
    return pts

if __name__ == '__main__':
    (score, d, e), tried, passed = sweep()
    V_full = 20.2
    print(f'{tried} candidates, {passed} pass; best {d["Ns"]}N{d["P"]}P {d["conn"]} {d["paths"]}p  L {d["L"]*1000:.0f} mm  hm {d["hm"]*1000:.1f} mm  wt {d["wt"]*1000:.1f} mm  N {d["N"]} × {d["strands"]} strands Ø{d["d_wire"]*1000:.2f} mm')
    print(f'  Kv {d["Kv"]:.0f}  R {d["R"]*1000:.1f} mΩ  Bg {d["Bg"]:.2f} T  Bt {d["B_tooth"]:.2f}  Bbell {d["B_bell"]:.2f}  mass {d["m"]*1000:.0f} g')
    for k in ('hover', 'full'):
        o = e[k]
        print(f'  {k}: {o["rpm"]:.0f} rpm  {o["T"]/9.81:.2f} kg  {o["I"]:.1f} A  {o["V"]:.1f} V  eff {o["eff"]:.3f}  Pcu {o["P_cu"]:.0f} W  Pfix {o["P_fixed"]:.0f} W  fe {o["f_e"]:.0f} Hz')
    print(f'  T hover {e["T_hover"]:.0f} °C  T burst {e["T_burst"]:.0f} °C  bell SF {e["sf_bell"]:.1f}  demag margin {e["demag_margin"]:.1f}  erpm {e["erpm"]/1000:.0f}k')
    fe, ed, me = losses_fixed(d, e['hover']['w'])
    out = dict(design={k: (round(v, 7) if isinstance(v, float) else v) for k, v in d.items()},
               fan=FAN, limits=LIMITS, magnet=MAG,
               eval={k: ({kk: round(vv, 5) for kk, vv in v.items()} if isinstance(v, dict) else round(v, 5)) for k, v in e.items()},
               hover_losses=dict(iron=round(fe, 2), eddy=round(ed, 2), mech=round(me, 2)),
               curve=curves(d, V_full), sweep=dict(tried=tried, passed=passed), V_full=V_full,
               fixed_curve=[[round(w), round(sum(losses_fixed(d, w)), 3)] for w in range(0, 6001, 250)],
               thermal=dict(A=round(thermal_area(d), 6), C=round(thermal(d, e['hover'])['C_th'], 3), R_int=0.35, h=[22, 18, 40, 0.8]),
               loss_model=dict(steel=STEEL, eddy_k=2.2e-6, mech=[1.6, 2.4], w_ref=3927))
    with open(OUT, 'w') as f:
        json.dump(out, f, indent=1)
    print('wrote', OUT)
