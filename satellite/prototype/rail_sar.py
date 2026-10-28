"""Rail SAR processor for the Tideline ground prototype.

    python rail_sar.py --simulate            # synthetic harbour, writes rail_sar_sim.png and prints resolution
    python rail_sar.py --data scan.npz       # your own capture: array 'iq' [positions × samples], 'x' positions (m)

The radar is a 60 GHz FMCW board. At every rail stop it sends one linear chirp and records the de-chirped
(beat) signal: an echo from range R appears as a tone at f_b = 2·R·S/c, where S is the chirp slope.
1. Range compression: an FFT of each chirp turns tones into a range profile.
2. Back-projection: for every image pixel, compute its range from each rail position, read that range from
   the profile, undo the carrier phase 4π·f0·R/c and add everything up. Echoes from a real target add in
   phase only at the right pixel, which is what focuses the image, the same as a satellite in orbit.
3. CA-CFAR detection: a pixel is a "ship" when it is well above the mean of its surrounding ring.

Capture is hardware-specific: write each chirp's ADC samples (one RX channel) into iq[i, :] and the rail
position into x[i], then save with numpy.savez('scan.npz', iq=iq, x=x). Only numpy (and matplotlib for
the picture) are needed.
"""
import argparse, math
import numpy as np

C = 299792458.0
RADAR = dict(f0=58e9, B=4e9, T=130e-6, N=256, fs=256 / 130e-6)   # one chirp: 256 samples over 130 µs
RAIL = dict(L=1.0, step=0.0025, height=0.37)                     # radar 0.37 m above the harbour board

def chirp_params(r=RADAR):
    S = r['B'] / r['T']
    t = np.arange(r['N']) / r['fs']
    return S, t

def simulate(targets, r=RADAR, rail=RAIL, snr_db=25, seed=1):
    """targets: list of (x, z, rcs_m2). Returns iq [n_pos × N] and x positions."""
    S, t = chirp_params(r)
    x = np.arange(-rail['L'] / 2, rail['L'] / 2 + 1e-9, rail['step'])
    iq = np.zeros((len(x), r['N']), complex)
    for (tx, tz, rcs) in targets:
        R = np.sqrt((x - tx) ** 2 + tz ** 2 + rail['height'] ** 2)               # [n_pos]
        amp = math.sqrt(rcs) / R ** 2
        az = np.exp(-((x - tx) / np.maximum(tz, 0.1)) ** 2 / (2 * math.tan(math.radians(30)) ** 2))   # ±30° beam
        tau = 2 * R / C
        iq += (amp * az)[:, None] * np.exp(1j * 2 * np.pi * (r['f0'] * tau[:, None] + S * tau[:, None] * t[None, :]))
    p = np.mean(np.abs(iq) ** 2)
    rng = np.random.default_rng(seed)
    iq += math.sqrt(p / 10 ** (snr_db / 10) / 2) * (rng.standard_normal(iq.shape) + 1j * rng.standard_normal(iq.shape))
    return iq, x

def range_compress(iq, r=RADAR, up=8):
    S, _ = chirp_params(r)
    n = r['N'] * up
    win = np.hanning(r['N'])
    prof = np.fft.fft(iq * win[None, :], n=n, axis=1)
    fb = np.fft.fftfreq(n, 1 / r['fs'])
    rng = fb * C / (2 * S)
    keep = rng >= 0
    order = np.argsort(rng[keep])
    return prof[:, keep][:, order], rng[keep][order]

def backproject(prof, rbins, x, xs, zs, r=RADAR, rail=RAIL):
    img = np.zeros((len(zs), len(xs)), complex)
    X, Z = np.meshgrid(xs, zs)
    dr = rbins[1] - rbins[0]
    for i, xp in enumerate(x):
        R = np.sqrt((X - xp) ** 2 + Z ** 2 + rail['height'] ** 2)
        k = (R - rbins[0]) / dr
        k0 = np.clip(np.floor(k).astype(int), 0, len(rbins) - 2)
        f = k - k0
        val = prof[i, k0] * (1 - f) + prof[i, k0 + 1] * f
        img += val * np.exp(-1j * 4 * np.pi * r['f0'] * R / C)
    return np.abs(img)

def cfar(img_db, guard=3, ring=8, k_db=12):
    from numpy.lib.stride_tricks import sliding_window_view as sw
    p = 10 ** (img_db / 10)
    w = 2 * (guard + ring) + 1
    pad = np.pad(p, guard + ring, mode='reflect')
    tot = sw(pad, (w, w)).sum(axis=(2, 3))
    g = 2 * guard + 1
    padg = np.pad(p, guard, mode='reflect')
    inner = sw(padg, (g, g)).sum(axis=(2, 3))
    bg = (tot - inner) / (w * w - g * g)
    return 10 * np.log10(p / bg) > k_db

def harbour_targets():
    """Two foil-taped model ships, two feeders and three corner reflectors, as point scatterers (x, z, rcs)."""
    t = []
    def ship(cx, cz, L, ang, n, rcs):
        for k in range(n):
            s = -L / 2 + L * k / (n - 1)
            t.append((cx + s * math.cos(ang), cz + s * math.sin(ang), rcs))
    facet = 0.05                                   # foil-taped hull facet, m² (about -13 dBsm)
    ship(-0.25, 1.6 + 0.21, 0.30, 0.0, 9, facet)
    ship(0.05, 1.6 - 0.12, 0.30, math.radians(-25), 9, facet)
    ship(0.38, 1.6 + 0.05, 0.18, math.radians(90), 6, facet)
    ship(-0.42, 1.6 - 0.20, 0.18, math.radians(15), 6, facet)
    lam = C / 60e9
    for (x, z) in ((-0.55, 1.25), (0.55, 1.25), (0.0, 1.96)):
        t.append((x, z, 4 * math.pi * 0.1 ** 4 / (3 * lam ** 2)))   # 100 mm trihedral, ≈ 17 m²
    return t

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--simulate', action='store_true')
    ap.add_argument('--data')
    ap.add_argument('--out', default='rail_sar_sim.png')
    a = ap.parse_args()
    if a.data:
        d = np.load(a.data); iq, x = d['iq'], d['x']
    else:
        iq, x = simulate(harbour_targets())
    prof, rb = range_compress(iq)
    xs = np.linspace(-0.8, 0.8, 321); zs = np.linspace(1.0, 2.2, 241)
    img = backproject(prof, rb, x, xs, zs)
    db = 20 * np.log10(img / img.max() + 1e-9)
    det = cfar(db)
    # resolution from the isolated reflector at (0, 1.96)
    iz = np.argmin(abs(zs - 1.96)); ix = np.argmin(abs(xs - 0.0))
    row = db[iz, ix - 30:ix + 31]; col = db[iz - 30:iz + 31, ix]
    pk = db[iz, ix]
    w = lambda v, d: (np.sum(v > pk - 3)) * d
    print(f'cross-range 3 dB width {w(row, xs[1] - xs[0]) * 1000:.1f} mm, range 3 dB width {w(col, zs[1] - zs[0]) * 1000:.1f} mm, '
          f'theory {C / (2 * RADAR["B"]) * 1000 * 1.44:.1f} mm range (Hann) / {C / RADAR["f0"] * 1.96 / (2 * RAIL["L"]) * 1000:.1f} mm cross-range, detections {int(det.sum())} px')
    try:
        import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
        plt.figure(figsize=(7, 5.2)); plt.imshow(np.clip(db, -40, 0), extent=[xs[0], xs[-1], zs[-1], zs[0]], cmap='magma', aspect='auto')
        plt.colorbar(label='dB'); plt.xlabel('along rail, m'); plt.ylabel('range from rail, m'); plt.title('Rail SAR image of the model harbour')
        plt.gca().invert_yaxis(); plt.savefig(a.out, dpi=120, bbox_inches='tight'); print('wrote', a.out)
    except ImportError:
        pass

if __name__ == '__main__':
    main()
