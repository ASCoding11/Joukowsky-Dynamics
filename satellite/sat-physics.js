// Tideline SAR & mission physics: orbit, radar equation, ambiguities, ship detection, revisit, data, link, power,
// thermal, attitude agility, drag, propellant and de-orbit. Everything is recomputed from the inputs and the CAD.
import { $, clamp, rad, deg, db, undb, fmt, esc, lineChart, wireTips, checksTable } from './sat-util.js';

const C = 299792458, KB = 1.380649e-23, T0 = 290, RE = 6371e3, REQ = 6378137, MU = 3.986004418e14, J2 = 1.08263e-3, G0 = 9.80665, SIGMA_SB = 5.670374e-8;

export const PHYS_INPUTS = [
  ['Orbit and constellation', [['h', 'Altitude', 'km', 400, 700, 5], ['nSat', 'Satellites in the constellation', '', 1, 12, 1], ['incMin', 'Nearest incidence angle', '°', 10, 30, 1],
    ['incMax', 'Farthest incidence angle', '°', 30, 55, 1], ['theta', 'Incidence for the checks', '°', 15, 50, 1]]],
  ['Antenna and radar', [['eta', 'Aperture efficiency (taper, losses)', '', .35, .8, .01], ['trmW', 'Peak power per T/R module', 'W', 2, 10, .5], ['duty', 'Transmit duty cycle', '', .04, .25, .01],
    ['NF', 'Receiver noise figure', 'dB', 1.5, 6, .1], ['Ls', 'System losses', 'dB', 1, 6, .1], ['pae', 'T/R module efficiency (PAE)', '', .25, .5, .01]]],
  ['Imaging modes', [['Bspot', 'Spotlight chirp bandwidth', 'MHz', 100, 600, 10], ['spotRes', 'Spotlight azimuth resolution', 'm', .3, 1.5, .05], ['Bstrip', 'Stripmap bandwidth', 'MHz', 40, 200, 5],
    ['Bscan', 'ScanSAR bandwidth', 'MHz', 15, 80, 5], ['nBeams', 'ScanSAR sub-swaths', '', 2, 6, 1]]],
  ['Tasking per orbit', [['nSpot', 'Spotlight scenes (10 s each)', '', 0, 12, 1], ['stripS', 'Stripmap time', 's', 0, 300, 10], ['scanS', 'ScanSAR time', 's', 0, 300, 10]]],
  ['Sea and detection', [['wind', 'Wind speed over the sea', 'm/s', 2, 20, .5], ['thr', 'CFAR detection threshold', 'dB', 8, 18, .5]]],
  ['Power and thermal', [['busW', 'Bus load (avionics, ADCS, heaters)', 'W', 80, 260, 5], ['eclipse', 'Eclipse per orbit (worst season)', 'min', 0, 35, 1], ['eol', 'Solar array end-of-life factor', '', .65, 1, .01],
    ['radFrac', 'Array back exposed as radiator', '', .04, .5, .01], ['heaterW', 'Array survival heaters', 'W', 0, 80, 1]]],
  ['Downlink and storage', [['dl', 'X-band downlink rate', 'Mbit/s', 200, 2000, 50], ['contact', 'Ground contact per orbit', 'min', 2, 25, .5], ['txW', 'X-band RF power', 'W', 2, 20, .5]]],
  ['Attitude and lifetime', [['slew', 'Slew angle between targets', '°', 5, 120, 1], ['wheelT', 'Reaction-wheel torque (each)', 'N·m', .02, .3, .01],
    ['life', 'Mission life', 'years', 1, 10, .5], ['solar', 'Solar activity', 'sel', [{ label: 'Solar minimum', v: .3 }, { label: 'Solar average', v: 1 }, { label: 'Solar maximum', v: 2.5 }]], ['kr', 'Krypton loaded', 'kg', 0, 5.5, .1]]],
];
export function physDefaults() {
  return { h: 560, nSat: 1, incMin: 15, incMax: 45, theta: 35, eta: .55, trmW: 6, duty: .12, NF: 3, Ls: 3.5, pae: .38, Bspot: 300, spotRes: .5, Bstrip: 100, Bscan: 40, nBeams: 4,
    nSpot: 4, stripS: 60, scanS: 120, wind: 7, thr: 13, busW: 140, eclipse: 18, eol: .85, radFrac: .12, heaterW: 30, dl: 800, contact: 9, txW: 10, slew: 30, wheelT: .1, life: 5, solar: 1, kr: 4.5 };
}

// X-band sea clutter (HH), an empirical fit: VV rises with wind and falls with incidence; HH sits below VV by a
// polarisation ratio that grows with incidence. Within a few dB of published X-band data for 20–50° and 3–15 m/s.
export function sigma0(thDeg, U, pol = 'HH') {
  const vv = -6 - .33 * (thDeg - 20) + 13 * Math.log10(Math.max(U, 1) / 7) * (1 + .01 * (thDeg - 20));
  return undb(pol === 'VV' ? vv : vv - .25 * Math.max(0, thDeg - 20));
}
// ship radar cross-section vs length, a fitted rule of thumb (≈ 10 dBsm at 10 m, 35 dBsm at 100 m, 47 dBsm at 300 m)
export const shipRCS = (L) => .03 * L ** 2.5;
const RHO_TAB = [[400, 3.7e-12], [450, 1.6e-12], [500, 7.3e-13], [550, 3.5e-13], [600, 1.8e-13], [650, 9.5e-14], [700, 5.3e-14], [800, 1.7e-14]];
function rhoAt(hkm, f = 1) {
  if (hkm <= 200) return 2.5e-10 * f;
  if (hkm < 400) return Math.exp(Math.log(2.5e-10) + (Math.log(3.7e-12) - Math.log(2.5e-10)) * (hkm - 200) / 200) * f;
  for (let i = 0; i < RHO_TAB.length - 1; i++) {
    const [h0, r0] = RHO_TAB[i], [h1, r1] = RHO_TAB[i + 1];
    if (hkm <= h1) return Math.exp(Math.log(r0) + (Math.log(r1) - Math.log(r0)) * (hkm - h0) / (h1 - h0)) * f;
  }
  return 1.7e-14 * f;
}
const sinc = (x) => (Math.abs(x) < 1e-9 ? 1 : Math.sin(Math.PI * x) / (Math.PI * x));

export function satModel(F, meta, mp, mpStowed, env) {
  const lam = meta.lam, L = meta.ant.L, H = meta.ant.H, nTRM = meta.ant.trm;
  const a = RE + F.h * 1e3, v = Math.sqrt(MU / a), T = 2 * Math.PI * Math.sqrt(a ** 3 / MU), n = 2 * Math.PI / T;
  const inc = deg(Math.acos(-(2 * Math.PI / (365.2422 * 86400)) / (1.5 * n * J2 * (REQ / a) ** 2)));
  const vg = v * RE / a, veff = Math.sqrt(v * vg), nRev = 86400 / T;
  const geo = (thDeg) => {
    const th = rad(thDeg), look = Math.asin(RE * Math.sin(th) / a), gam = th - look;
    return { th, look, gam, R: a * Math.sin(gam) / Math.sin(th), g: RE * gam };
  };
  const lookToGeo = (R) => {   // slant range → look and incidence angles
    const cl = (a * a + R * R - RE * RE) / (2 * a * R); if (cl > 1 || cl < -1) return null;
    const look = Math.acos(cl), si = a * Math.sin(look) / RE; if (si >= 1) return null;
    return { look, th: Math.asin(si), R };
  };
  const A = L * H, G = 4 * Math.PI * F.eta * A / lam ** 2, thEl = .886 * lam / H, thAz = .886 * lam / L;
  const Ppk = nTRM * F.trmW, Pavg = Ppk * F.duty, Fn = undb(F.NF), Ls = undb(F.Ls);
  const nesz = (thDeg, B) => { const g = geo(thDeg); return 4 * (4 * Math.PI) ** 3 * g.R ** 3 * veff * KB * T0 * Fn * Ls * B * Math.sin(g.th) / (Pavg * G * G * lam ** 3 * C); };
  const g0 = geo(F.theta);
  // stripmap swath and PRF window
  const swathG = (thDeg) => { const g = geo(thDeg); return g.R * thEl / Math.cos(g.th); };
  const Bd = 2 * veff * thAz / lam;          // azimuth Doppler bandwidth of the beam
  const prfMin = 1.1 * Bd;
  const slantSwath = swathG(F.theta) * Math.sin(g0.th);
  const prfMax = 1 / (2 * (F.duty / prfMin) + 2 * slantSwath / C + 10e-6);
  const prf = Math.min(Math.max(prfMin * 1.04, prfMin), prfMax), tau = F.duty / prf;
  // azimuth ambiguity-to-signal ratio: two-way pattern sinc⁴(L·f / 2v) aliased at multiples of the PRF
  const aasr = (PRF) => {
    const Bp = .8 * Bd, N = 200, P = (f) => sinc(L * f / (2 * veff)) ** 4;
    let s = 0, amb = 0;
    for (let i = 0; i < N; i++) {
      const f = -Bp / 2 + Bp * (i + .5) / N; s += P(f);
      for (let m = 1; m <= 6; m++) amb += P(f + m * PRF) + P(f - m * PRF);
    }
    return amb / s;
  };
  // range ambiguity-to-signal ratio: echoes from one PRI earlier / later, weighted by the elevation pattern, σ⁰ and R³
  const rasr = (PRF, thDeg = F.theta) => {
    const gc = geo(thDeg); let worst = 0;
    for (const edge of [-.5, 0, .5]) {
      const look0 = gc.look + edge * thEl * .9, cl0 = lookToGeoLook(look0); if (!cl0) continue;
      const P = (x) => { const off = x.look - gc.look; return sinc(H * Math.sin(off) / lam) ** 4 * sigma0(deg(x.th), F.wind) / (x.R ** 3 * Math.sin(x.th)); };
      const S = P(cl0); let amb = 0;
      for (let j = -5; j <= 5; j++) { if (!j) continue; const x = lookToGeo(cl0.R + j * C / (2 * PRF)); if (x && x.R > F.h * 1e3 * 1.001) amb += P(x); }
      worst = Math.max(worst, amb / S);
    }
    return worst;
  };
  function lookToGeoLook(look) { const si = a * Math.sin(look) / RE; if (si >= 1) return null; const th = Math.asin(si), gam = th - look; return { look, th, R: a * Math.sin(gam) / Math.sin(th) }; }
  // modes
  const kw = 1.13;   // resolution broadening from spectral weighting
  const mode = (name, B, dAz, thDeg, swath, looks) => {
    const g = geo(thDeg), dr = kw * C / (2 * B), drg = dr / Math.sin(g.th), N = nesz(thDeg, B);
    const lmin = (U) => { const bg = (sigma0(thDeg, U) + N) * drg * dAz; return Math.max((undb(F.thr) * bg / .03) ** .4, 2 * Math.max(drg, dAz)); };
    const window = name === 'spot' ? 5000 * Math.sin(g.th) : swathG(thDeg) * Math.sin(g.th);
    const rate = 1.2 * B * 8 * Math.min(1, (2 * window / C + tau) * prf);   // complex samples, 4-bit BAQ I and Q
    return { name, B, dr, drg, dAz, nesz: N, neszDb: db(N), swath, lmin, lminNow: lmin(F.wind), rate, rateGbps: rate / 1e9, prf, looks, th: thDeg };
  };
  const strip = mode('strip', F.Bstrip * 1e6, kw * L / 2, F.theta, swathG(F.theta), 1);
  const spot = mode('spot', F.Bspot * 1e6, F.spotRes, F.theta, 5000, 1);
  const scanTh = Math.min(F.incMax, F.theta + 5);
  const scan = mode('scan', F.Bscan * 1e6, (F.nBeams + 1) * kw * L / 2, scanTh, F.nBeams * swathG(scanTh) * .9, 3);
  const scanFarNesz = nesz(F.incMax, F.Bscan * 1e6);
  const spotSweep = deg(lam / (2 * F.spotRes));            // azimuth angle the beam must sweep for the spotlight resolution
  const aMin = 4 * veff * lam * g0.R * Math.tan(g0.th) / C; // minimum antenna area for unambiguous stripmap
  // access and revisit
  const gMin = geo(F.incMin).g, gMax = geo(F.incMax).g, wAccess = 2 * (gMax - gMin);
  const revisitH = (latDeg, ns = F.nSat) => { const c = 2 * Math.PI * RE * Math.cos(rad(Math.min(latDeg, 85))); const passes = Math.min(ns * nRev * 2 * wAccess / c, ns * nRev * 2); return 24 / passes; };
  // data
  const imgS = { spot: F.nSpot * 10, strip: F.stripS, scan: F.scanS };
  const dataGb = (spot.rate * imgS.spot + strip.rate * imgS.strip + scan.rate * imgS.scan) / 1e9;
  const dlGb = F.dl * 1e6 * F.contact * 60 / 1e9, storeGb = 1.5e12 * 8 / 1e9;
  // X-band link at 10° elevation
  const el = rad(10), Rs = Math.sqrt(a * a - (RE * Math.cos(el)) ** 2) - RE * Math.sin(el), lamX = C / 8.2e9;
  const fspl = 20 * Math.log10(4 * Math.PI * Rs / lamX);
  const cn0 = db(F.txW) + 17 - 1.5 - fspl - 3.5 + 32 + 228.6;
  const ebn0 = cn0 - db(F.dl * 1e6), linkMargin = ebn0 - 7.5;
  // power over one orbit (worst-season eclipse centred at mid-orbit)
  const solarA = meta.solar.area, Psun = 1361 * meta.solar.eff * solarA * F.eol, look0 = g0.look;
  const dcImg = Pavg / F.pae + 5 * 20 + 180;     // T/R modules + panel controllers + central electronics
  const events = [];
  for (let i = 0; i < F.nSpot; i++) events.push([600 + i * 150, 10, 'img']);
  if (F.stripS) events.push([1900, F.stripS, 'img']);
  if (F.scanS) events.push([2500, F.scanS, 'img']);
  events.push([3300, 600, 'proc']); events.push([T - F.contact * 60 - 200, F.contact * 60, 'dl']);
  const Ebat = meta.battery.s * meta.battery.p * meta.battery.V * meta.battery.Ah * 3600, Vbat = meta.battery.s * 3.6;
  const ecl0 = T / 2 - F.eclipse * 30, ecl1 = T / 2 + F.eclipse * 30;
  const area = .66, Cd = 2.2, m = mp.m;
  const rho = rhoAt(F.h, F.solar), aDrag = .5 * rho * v * v * Cd * area / m, dvYr = aDrag * 3.156e7;
  const thrustS = aDrag * T / (.017 / m);         // seconds per orbit the 17 mN thruster must fire to cancel drag
  let soc = 1, socMin = 1, eGen = 0, eLoad = 0, iMax = 0, timeline = [];
  const dt = 10;
  for (let t = 0; t < T; t += dt) {
    const ev = events.find(([t0, d]) => t >= t0 && t < t0 + d);
    const kind = ev ? ev[2] : null, lit = !(t >= ecl0 && t < ecl1);
    const cosA = kind === 'img' ? Math.cos(look0) : kind === 'dl' ? Math.cos(rad(50)) : 1;
    const gen = lit ? Psun * cosA : 0;
    const load = F.busW + F.heaterW + (kind === 'img' ? dcImg : 0) + (kind === 'proc' ? 60 : 0) + (kind === 'dl' ? 60 : 0) + (t < thrustS ? 300 : 0);
    const net = gen - load;
    soc = Math.min(1, soc + (net > 0 ? net * .92 : net) * dt / Ebat);
    socMin = Math.min(socMin, soc); eGen += gen * dt; eLoad += load * dt;
    iMax = Math.max(iMax, Math.max(0, load - gen) / Vbat / meta.battery.p);
    if (t % 30 === 0) timeline.push([t / 60, gen, load, soc * 100]);
  }
  // array thermal: heat stored per orbit, and equilibrium temperature with the radiator fraction and heaters
  const mAnt = mp.bySys['S20 SAR antenna'] || 55, Cant = mAnt * 800;
  const qPass = (dcImg - 180 - Pavg) * (imgS.spot + imgS.strip + imgS.scan);
  const dTorbit = qPass / Cant, qAvg = qPass / T;
  const radA = 5 * .74 * .67 * F.radFrac, Tsink = 230;
  const Teq = Math.pow(Tsink ** 4 + (qAvg + F.heaterW) / (.9 * SIGMA_SB * radA), .25) - 273.15;
  const Tcold = Math.pow(Tsink ** 4 + F.heaterW / (.9 * SIGMA_SB * radA), .25) - 273.15;
  // attitude: bang-bang roll slew with a 4-wheel pyramid (≈ 2.3 × one wheel about any axis)
  const Ix = mp.I[0], Tax = 2.3 * F.wheelT, Hax = 2.3 * 1.0, th = rad(F.slew);
  let tSlew = 2 * Math.sqrt(th * Ix / Tax), Hpk = Tax * tSlew / 2;
  if (Hpk > Hax) { const tAcc = Hax / Tax, angAcc = Hax * tAcc / Ix; tSlew = 2 * tAcc + (th - angAcc) / (Hax / Ix); Hpk = Hax; }
  // propellant and de-orbit
  const rp = RE + 300e3, vT = Math.sqrt(MU * (2 / a - 2 / (a + rp))), dvDeorbit = v - vT;
  const dvTot = (F.life * dvYr + dvDeorbit) * 1.1, mProp = m * (1 - Math.exp(-dvTot / (1350 * G0)));
  const decay = (hStart, f) => { let hk = hStart, t = 0; const pts = [[0, hk]]; while (hk > 200 && t < 200) { const aa = RE + hk * 1e3, r = rhoAt(hk, f); const dadt = -r * Math.sqrt(MU * aa) * Cd * area / m; const stepYr = .02; hk += dadt * stepYr * 3.156e7 / 1e3; t += stepYr; if (Math.round(t / .02) % 10 === 0) pts.push([t, hk]); } return { years: t, pts }; };
  const life0 = decay(F.h, F.solar);
  const geoloc = Math.sqrt(.1 ** 2 + (C * 5e-9 / 2 / Math.sin(g0.th)) ** 2 + (1 / Math.tan(g0.th)) ** 2);
  const P = { F, meta, a, v, T, inc, vg, veff, nRev, geo, A, G, Gdb: db(G), thEl, thAz, Ppk, Pavg, Bd, prfMin, prfMax, prf, tau, aasr, rasr, aasrDb: db(aasr(prf)), rasrDb: db(rasr(prf)),
    strip, spot, scan, scanFarNesz, spotSweep, aMin, gMin, gMax, wAccess, revisitH, imgS, dataGb, dlGb, storeGb, Rs, fspl, cn0, ebn0, linkMargin,
    Psun, solarSunW: Psun, dcImg, Ebat, socMin, eGen, eLoad, iMax, timeline, mAnt, dTorbit, qAvg, Teq, Tcold, radA, Ix, tSlew, Hpk, Hax, rho, aDrag, dvYr, thrustS, dvDeorbit, dvTot, mProp,
    life0, decay, geoloc, mp, mpStowed, env, nesz };
  return P;
}

export function physChecks(P) {
  const F = P.F, k = (ok, mid) => (ok ? 'ok' : mid ? 'mid' : 'bad'), hStow = (.4 - P.mpStowed.cg.y);
  const rv = P.revisitH(30), envMax = Math.max(...P.env);
  return [
    ['Antenna area vs ambiguity limit', `${fmt(P.A, 2)} m² (${fmt(P.A / P.aMin, 2)} ×)`, `≥ 1.3 × ${fmt(P.aMin, 2)} m²`, k(P.A >= 1.3 * P.aMin, P.A >= P.aMin), 'An antenna smaller than 4·v·λ·R·tanθ / c cannot have a PRF that suits both range and azimuth.'],
    ['PRF window', `${fmt(P.prf, 0)} Hz in ${fmt(P.prfMin, 0)}–${fmt(P.prfMax, 0)}`, 'minimum < maximum', k(P.prfMin < P.prfMax * .95, P.prfMin < P.prfMax), 'Azimuth sampling needs PRF ≥ 1.1 × Doppler bandwidth; the echo window caps it.'],
    ['Azimuth ambiguities (AASR)', `${fmt(P.aasrDb, 1)} dB`, '≤ −20 dB', k(P.aasrDb <= -20, P.aasrDb <= -17), 'Ghost copies of bright ships displaced along-track; they would show up as false targets.'],
    ['Range ambiguities (RASR, swath edges)', `${fmt(P.rasrDb, 1)} dB`, '≤ −20 dB', k(P.rasrDb <= -20, P.rasrDb <= -17), 'Echoes from the previous and next pulse folding into the image.'],
    ['Noise floor, spotlight', `${fmt(P.spot.neszDb, 1)} dB`, '≤ −15 dB', k(P.spot.neszDb <= -15, P.spot.neszDb <= -13), `NESZ at ${F.theta}° with ${F.Bspot} MHz. Calm sea is about ${fmt(db(sigma0(F.theta, 3)), 0)} dB, so the noise floor must sit near it.`],
    ['Noise floor, stripmap', `${fmt(P.strip.neszDb, 1)} dB`, '≤ −18 dB', k(P.strip.neszDb <= -18, P.strip.neszDb <= -16), 'Sets how well dark targets (oil slicks, calm water) and small boats show against noise.'],
    ['Noise floor, ScanSAR far edge', `${fmt(db(P.scanFarNesz), 1)} dB`, '≤ −17 dB', k(db(P.scanFarNesz) <= -17, db(P.scanFarNesz) <= -15), `At ${F.incMax}° incidence, the worst part of the wide swath.`],
    ['Spotlight resolution', `${fmt(P.spot.drg, 2)} m ground range × ${fmt(P.spot.dAz, 2)} m azimuth`, '≤ 1.0 m both', k(P.spot.drg <= 1 && P.spot.dAz <= 1, P.spot.drg <= 1.5), `Needs a ${fmt(P.spotSweep, 2)}° beam sweep in azimuth, inside the subarrays’ ±2° steering.`],
    ['Stripmap swath', `${fmt(P.strip.swath / 1000, 1)} km`, '≥ 25 km (a whole port)', k(P.strip.swath >= 25e3, P.strip.swath >= 18e3), 'Set by the 0.75 m array height: beamwidth λ / H.'],
    ['ScanSAR swath', `${fmt(P.scan.swath / 1000, 0)} km`, '≥ 100 km (a shipping lane)', k(P.scan.swath >= 100e3, P.scan.swath >= 80e3), `${F.nBeams} sub-swaths; azimuth resolution coarsens to ${fmt(P.scan.dAz, 1)} m.`],
    ['Smallest ship, spotlight', `${fmt(P.spot.lminNow, 1)} m`, '≤ 5 m (small craft in port)', k(P.spot.lminNow <= 5, P.spot.lminNow <= 8), `Ship RCS above (sea clutter + noise) × cell area by ${F.thr} dB, and at least 2 resolution cells long. Wind ${F.wind} m/s.`],
    ['Smallest ship, stripmap', `${fmt(P.strip.lminNow, 1)} m`, '≤ 12 m (fishing and work boats)', k(P.strip.lminNow <= 12, P.strip.lminNow <= 18), 'Most AIS-exempt vessels are under 15 m, so this sets dark-vessel coverage in anchorages.'],
    ['Smallest ship, ScanSAR', `${fmt(P.scan.lminNow, 1)} m`, '≤ 30 m (merchant ships in lanes)', k(P.scan.lminNow <= 30, P.scan.lminNow <= 45), 'Cargo ships and tankers are 80–400 m long.'],
    ['Revisit at 30° latitude', `${fmt(rv, 1)} h with ${F.nSat} satellite${F.nSat > 1 ? 's' : ''}`, '≤ 24 h', k(rv <= 24, rv <= 48), `Access ${F.incMin}–${F.incMax}° on both sides (${fmt(P.wAccess / 1000, 0)} km). Dawn-dusk orbit: passes near 06:00 and 18:00 local.`],
    ['Data per orbit vs downlink', `${fmt(P.dataGb, 0)} of ${fmt(P.dlGb, 0)} Gbit`, '≤ 80 % of downlink', k(P.dataGb <= .8 * P.dlGb, P.dataGb <= P.dlGb), `Raw data after 4-bit compression. ${fmt(P.storeGb / 1000, 0)} Tbit on board holds ${fmt(P.storeGb / Math.max(P.dataGb, 1), 0)} orbits.`],
    ['X-band link margin', `${fmt(P.linkMargin, 1)} dB`, '≥ 3 dB', k(P.linkMargin >= 3, P.linkMargin >= 1), `${F.txW} W into a 17 dBi horn, ${fmt(P.Rs / 1000, 0)} km at 10° elevation, 32 dB/K station, DVB-S2 16APSK 3/4.`],
    ['Energy balance per orbit', `${fmt(P.eGen / P.eLoad, 2)} ×`, '≥ 1.2', k(P.eGen / P.eLoad >= 1.2, P.eGen / P.eLoad >= 1.05), `${fmt(P.eGen / 3600, 0)} Wh generated, ${fmt(P.eLoad / 3600, 0)} Wh used. Imaging draws ${fmt(P.dcImg, 0)} W.`],
    ['Battery depth of discharge', `${fmt((1 - P.socMin) * 100, 1)} %`, '≤ 25 % (27,000 cycles in 5 years)', k(1 - P.socMin <= .25, 1 - P.socMin <= .35), `${fmt(P.Ebat / 3600, 0)} Wh pack, ${P.meta.battery.s}s${P.meta.battery.p}p 18650 cells.`],
    ['Cell current, peak', `${fmt(P.iMax, 1)} A`, '≤ 7 A (10 A rated)', k(P.iMax <= 7, P.iMax <= 10), 'The payload power unit’s supercapacitors smooth the pulses, so cells see the average imaging load.'],
    ['Array temperature (idle / after an orbit)', `${fmt(P.Tcold, 0)} / ${fmt(P.Teq + P.dTorbit, 0)} °C`, '−20 to +45 °C', k(P.Tcold >= -20 && P.Teq + P.dTorbit <= 45, P.Tcold >= -30 && P.Teq + P.dTorbit <= 55), `${fmt(P.radA, 2)} m² of radiator, the rest of the back under MLI. Heats ${fmt(P.dTorbit, 1)} K over an orbit's imaging.`],
    ['Slew between targets', `${fmt(P.tSlew, 1)} s for ${F.slew}°`, '≤ 30 s', k(P.tSlew <= 30, P.tSlew <= 60), `Roll inertia ${fmt(P.Ix, 1)} kg·m² from the CAD; peak momentum ${fmt(P.Hpk, 2)} of ${fmt(P.Hax, 1)} N·m·s.`],
    ['Propellant needed', `${fmt(P.mProp, 2)} kg (Δv ${fmt(P.dvTot, 0)} m/s)`, `≤ ${fmt(F.kr, 1)} kg loaded`, k(P.mProp <= F.kr * .9, P.mProp <= F.kr), `${F.life} years of drag make-up (${fmt(P.dvYr, 1)} m/s per year) + lowering perigee to 300 km at end of life + 10 %.`],
    ['De-orbit within 5 years', P.life0.years <= 5 ? `${fmt(P.life0.years, 1)} y naturally` : `burn reserved; ${fmt(P.life0.years, 0)} y without it`, '≤ 5 years', k(P.life0.years <= 5 || P.mProp <= F.kr, P.life0.years <= 25), 'From a 300 km perigee the spacecraft re-enters within months.'],
    ['Launch mass', `${fmt(P.mp.m, 1)} kg wet`, '≤ 200 kg (typical 15-inch rideshare port)', k(P.mp.m <= 200, P.mp.m <= 230), 'Check the exact allowance with the launch provider.'],
    ['Stowed CG height and envelope', `${fmt(hStow * 1000, 0)} mm · ${P.env.map((d) => fmt(d * 1000, 0)).join(' × ')} mm`, '≤ 500 mm · ≤ 1,200 mm', k(hStow <= .5 && envMax <= 1.2, hStow <= .6 && envMax <= 1.3), 'Measured from the separation plane, with everything folded.'],
    ['Geolocation of detections', `${fmt(P.geoloc, 1)} m`, '≤ 10 m', k(P.geoloc <= 10, P.geoloc <= 25), 'Orbit 0.1 m (dual-frequency GNSS), 5 ns range timing, 1 m sea-level height. Fine enough to match AIS positions.'],
    ['Bandwidth inside the radar allocation', `${F.Bspot} MHz`, '≤ 600 MHz (9.2–10.4 GHz EESS active)', k(F.Bspot <= 600, false), 'Earth-exploration radar band; wider chirps need extra coordination.'],
  ];
}

const chartBox = { el: null };
export function renderPhysics(api) {
  const F = api.F;
  if (!F) return;
  for (const g of PHYS_INPUTS) for (const [k, , u, , , s] of g[1]) { const o = $('fout-' + k); if (o) o.textContent = `${fmt(F[k], s >= 1 ? 0 : s >= .1 ? 1 : 2)}${u ? ' ' + u : ''}`; }
  const P = api.model(), checks = physChecks(P);
  const nBad = checks.filter((c) => c[3] === 'bad').length, nMid = checks.filter((c) => c[3] === 'mid').length;
  const st = (ok, mid) => (ok ? 'ok' : mid ? 'mid' : 'bad');
  $('phys-kpis').innerHTML = [
    ['PHYSICS CHECKS', `${checks.length - nBad - nMid} / ${checks.length} pass`, `${nMid} marginal · ${nBad} failing`, nBad ? 'bad' : nMid ? 'mid' : 'ok'],
    ['SPOTLIGHT', `${fmt(P.spot.drg, 2)} × ${fmt(P.spot.dAz, 2)} m`, `5 × 5 km scene · NESZ ${fmt(P.spot.neszDb, 1)} dB`, ''],
    ['STRIPMAP', `${fmt(P.strip.drg, 1)} × ${fmt(P.strip.dAz, 1)} m`, `${fmt(P.strip.swath / 1000, 0)} km swath · NESZ ${fmt(P.strip.neszDb, 1)} dB`, ''],
    ['SCANSAR', `${fmt(P.scan.drg, 0)} × ${fmt(P.scan.dAz, 0)} m`, `${fmt(P.scan.swath / 1000, 0)} km swath`, ''],
    ['SMALLEST SHIP', `${fmt(P.strip.lminNow, 1)} m`, `stripmap at ${F.wind} m/s wind`, st(P.strip.lminNow <= 12, P.strip.lminNow <= 18)],
    ['REVISIT AT 30°', `${fmt(P.revisitH(30), 1)} h`, `${F.nSat} satellite${F.nSat > 1 ? 's' : ''}`, st(P.revisitH(30) <= 24, P.revisitH(30) <= 48)],
    ['ORBIT', `${fmt(F.h, 0)} km · ${fmt(P.inc, 2)}°`, `${fmt(P.T / 60, 1)} min period · ${fmt(P.v / 1000, 2)} km/s`, ''],
    ['RADAR POWER', `${fmt(P.Ppk / 1000, 2)} kW peak`, `${fmt(P.Pavg, 0)} W average RF · gain ${fmt(P.Gdb, 1)} dBi`, ''],
    ['DATA / ORBIT', `${fmt(P.dataGb, 0)} Gbit`, `${fmt(P.dlGb, 0)} Gbit downlink capacity`, st(P.dataGb <= .8 * P.dlGb, P.dataGb <= P.dlGb)],
    ['WET MASS', `${fmt(P.mp.m, 1)} kg`, `propellant need ${fmt(P.mProp, 2)} kg`, st(P.mp.m <= 200, P.mp.m <= 230)],
  ].map(([k, v, s, c]) => `<div class="kpi ${c}"><small>${k}</small><b>${v}</b><span>${s}</span></div>`).join('') + checksTable(checks);
  // charts
  const ths = []; for (let t = 15; t <= 50.001; t += 1) ths.push(t);
  const C1 = [];
  C1.push(lineChart('Noise floor (NESZ) across incidence angle', 'SAR radar equation', `NESZ = 4(4π)³R³·v·kT·F·L·B·sinθ / (P̄·G²·λ³·c). Wider chirps (finer range resolution) raise it; far-range geometry raises it as R³. Average RF power ${fmt(P.Pavg, 0)} W, antenna gain ${fmt(P.Gdb, 1)} dBi.`,
    { x: [15, 50], y: [-32, -8], xl: 'incidence angle, °', yl: 'NESZ, dB', hlines: [[-15, 'spot −15'], [-18, 'strip −18']], vline: [F.theta, 'checks'], series: [
      { name: `Spotlight ${F.Bspot} MHz`, c: 'var(--s1)', pts: ths.map((t) => [t, db(P.nesz(t, F.Bspot * 1e6))]) },
      { name: `Stripmap ${F.Bstrip} MHz`, c: 'var(--s2)', pts: ths.map((t) => [t, db(P.nesz(t, F.Bstrip * 1e6))]) },
      { name: `ScanSAR ${F.Bscan} MHz`, c: 'var(--s3)', pts: ths.map((t) => [t, db(P.nesz(t, F.Bscan * 1e6))]) },
      { name: 'Sea clutter HH, 3 m/s', c: 'var(--muted)', dash: '4 3', pts: ths.map((t) => [t, db(sigma0(t, 3))]) }] }));
  const us = []; for (let u = 2; u <= 20.001; u += .5) us.push(u);
  C1.push(lineChart('Smallest detectable ship vs wind', 'CFAR against sea clutter + noise', `A ship is found when its radar cross-section beats (σ⁰ + NESZ) × resolution-cell area by ${F.thr} dB, and it spans at least two cells. Rougher sea raises clutter; coarse modes lose small boats first.`,
    { x: [2, 20], y: [0, 60], xl: 'wind speed, m/s', yl: 'ship length, m', vline: [F.wind, 'now'], hlines: [[12, 'fishing boats'], [30, 'merchant ships']], series: [
      { name: 'Spotlight', c: 'var(--s1)', pts: us.map((u) => [u, P.spot.lmin(u)]) }, { name: 'Stripmap', c: 'var(--s2)', pts: us.map((u) => [u, P.strip.lmin(u)]) }, { name: 'ScanSAR', c: 'var(--s3)', pts: us.map((u) => [u, P.scan.lmin(u)]) }] }));
  const lats = []; for (let l = 0; l <= 70.001; l += 1) lats.push(l);
  const ports = [['Singapore', 1.3], ['Suez', 30.0], ['Rotterdam', 51.9], ['Shanghai', 31.2]];
  C1.push(lineChart('Mean revisit vs latitude', 'access swath × passes per day', `One satellite sees a point every ${fmt(P.revisitH(30), 0)} h at 30°; ground tracks converge towards the poles. The ${F.nSat}-satellite line is your setting.`,
    { x: [0, 70], y: [0, 60], xl: 'latitude, °', yl: 'hours between looks', series: [
      { name: `${F.nSat} satellite${F.nSat > 1 ? 's' : ''}`, c: 'var(--s1)', pts: lats.map((l) => [l, P.revisitH(l)]) },
      { name: '1', c: 'var(--muted)', dash: '4 3', pts: lats.map((l) => [l, P.revisitH(l, 1)]) }, { name: '4', c: 'var(--s2)', dash: '4 3', pts: lats.map((l) => [l, P.revisitH(l, 4)]) },
      { name: '8', c: 'var(--s3)', dash: '4 3', pts: lats.map((l) => [l, P.revisitH(l, 8)]) }], points: ports.map(([n, l]) => [l, P.revisitH(l), n]) }));
  C1.push(lineChart('Power over one orbit', 'solar generation vs load, worst-season eclipse', `Imaging bursts draw ${fmt(P.dcImg, 0)} W; the arrays give ${fmt(P.Psun, 0)} W when sun-pointing. The battery covers bursts and the eclipse.`,
    { x: [0, P.T / 60], y: [0, Math.ceil(Math.max(P.dcImg + F.busW + F.heaterW, P.Psun) / 200) * 200], xl: 'minutes into the orbit', yl: 'W', series: [
      { name: 'Generation', c: 'var(--s4)', pts: P.timeline.map((r) => [r[0], r[1]]) }, { name: 'Load', c: 'var(--s2)', pts: P.timeline.map((r) => [r[0], r[2]]) }] }));
  C1.push(lineChart('Battery state of charge over one orbit', `${fmt(P.Ebat / 3600, 0)} Wh pack`, `Lowest point ${fmt(P.socMin * 100, 1)} %: a ${fmt((1 - P.socMin) * 100, 1)} % depth of discharge, refilled before the next orbit.`,
    { x: [0, P.T / 60], y: [Math.min(70, Math.floor(P.socMin * 100 / 5) * 5 - 5), 100], xl: 'minutes into the orbit', yl: 'state of charge, %', series: [{ name: 'SoC', c: 'var(--s1)', pts: P.timeline.map((r) => [r[0], r[3]]) }] }));
  const prfs = []; for (let f = 2500; f <= 7000; f += 50) prfs.push(f);
  C1.push(lineChart('Ambiguities vs PRF (stripmap)', 'antenna patterns, numerically integrated', `Higher PRF pushes azimuth ghosts down but lets range ghosts in. The usable window is ${fmt(P.prfMin, 0)}–${fmt(P.prfMax, 0)} Hz.`,
    { x: [2500, 7000], y: [-60, 0], xl: 'PRF, Hz', yl: 'dB', hlines: [[-20, '−20 dB']], vlines: [[P.prf, 'PRF'], [P.prfMax, 'max']], series: [
      { name: 'AASR', c: 'var(--s1)', pts: prfs.map((f) => [f, db(P.aasr(f))]) }, { name: 'RASR', c: 'var(--s2)', pts: prfs.map((f) => [f, db(P.rasr(f))]) }] }));
  const dMax = P.decay(F.h, 2.5), dMin = P.decay(F.h, .3);
  C1.push(lineChart('Orbit decay with no propulsion', 'drag on a 0.66 m² frontal area', `Natural re-entry from ${fmt(F.h, 0)} km: ${fmt(dMax.years, 1)} y (solar max) to ${fmt(dMin.years, 1)} y (solar min). The thruster holds altitude and lowers the orbit at end of life.`,
    { x: [0, Math.min(60, Math.ceil(Math.max(dMin.years, 5)))], y: [200, F.h + 20], xl: 'years', yl: 'altitude, km', vline: [5, '5-year rule'], series: [
      { name: 'Solar maximum', c: 'var(--s2)', pts: dMax.pts }, { name: 'Solar minimum', c: 'var(--s3)', pts: dMin.pts }, { name: 'Your setting', c: 'var(--s1)', pts: P.life0.pts }] }));
  $('phys-charts').innerHTML = C1.join('');
  wireTips($('phys-charts'));
  $('phys-notes').innerHTML = `
    <div><h4>Why these choices</h4><ul>
      <li><b>X-band (9.65 GHz)</b> gives sub-metre resolution with a 4 m antenna and sees through cloud, rain, smoke and darkness. C-band would suit oil slicks over wide areas; L-band needs a far bigger antenna.</li>
      <li><b>Dawn-dusk sun-synchronous orbit</b> keeps the arrays lit almost all year, so a ${fmt(P.Psun, 0)} W array can feed a ${fmt(P.Ppk / 1000, 1)} kW radar in bursts.</li>
      <li><b>Three modes</b> match the jobs: spotlight for berths and blockages, stripmap for a whole port and its anchorage, ScanSAR for ${fmt(P.scan.swath / 1000, 0)} km of shipping lane.</li>
      <li><b>On-board detection + AIS</b>: ship lists leave within minutes over the relay, and vessels not broadcasting AIS stand out as dark targets.</li></ul></div>
    <div><h4>What limits it</h4><ul>
      <li>A single satellite revisits a mid-latitude port about every ${fmt(P.revisitH(30), 0)} h. Daily monitoring needs ${Math.ceil(P.revisitH(30) / 24 * F.nSat)}+ satellites; near-real-time needs 8 or more.</li>
      <li>Spotlight noise (${fmt(P.spot.neszDb, 1)} dB) is the price of 300 MHz bandwidth; small wooden boats in rough water remain hard to see.</li>
      <li>Dawn-dusk timing means looks near 06:00 and 18:00 local only, unless the constellation adds a second orbit plane.</li></ul></div>
    <div><h4>Method and limits</h4><p>Radar equation for distributed targets with the CAD array area; ambiguities from sinc patterns of a uniformly illuminated aperture; empirical X-band HH sea clutter and a fitted ship-RCS rule; mean revisit from swath geometry; exponential-table atmosphere. Not modelled: beam spoiling for wide swaths, quantisation noise, nadir echoes, antenna pattern errors, terrain, ionosphere and detailed thermal transients.</p></div>`;
}
