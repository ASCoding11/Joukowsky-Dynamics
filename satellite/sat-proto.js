// 05 · Ground prototype: a tabletop rail SAR (60 GHz FMCW on a 1 m belt-driven rail) that images a model harbour.
// The scan is simulated in the browser with the same processing as satellite/prototype/rail_sar.py: FFT range
// compression, then back-projection position by position, so the image sharpens as the synthetic aperture grows.
import { $, clamp, rad, db, undb, fmt, esc, checksTable } from './sat-util.js';

const C = 299792458, KB = 1.380649e-23;
const PR_INPUTS = [
  ['Rail and scan', [['railL', 'Synthetic aperture (rail travel)', 'm', .2, 1.1, .05], ['step', 'Step between chirps', 'mm', .5, 10, .25], ['settle', 'Settle time per step', 'ms', 5, 100, 5]]],
  ['Radar', [['B', 'Chirp bandwidth', 'GHz', 1, 5.5, .1], ['eirp', 'EIRP', 'dBm', 0, 15, .5], ['nf', 'Receiver noise figure', 'dB', 6, 18, .5], ['beam', 'Antenna beamwidth', '°', 20, 90, 5]]],
  ['Scene', [['shipRcs', 'Model-ship facet RCS (foil-taped)', 'dBsm', -30, -5, 1], ['screenDb', 'Cloud screen two-way loss', 'dB', 0, 10, .5]]],
];
const PR_DEFAULT = { railL: 1.0, step: 2.5, settle: 30, B: 4, eirp: 10, nf: 12, beam: 60, shipRcs: -13, screenDb: 2 };
const PRICES = [['60 GHz FMCW radar dev kit with raw ADC output (e.g. BGT60TR13C class)', 1, 150], ['Raspberry Pi 5 (4 GB) + case + 27 W supply', 1, 85], ['20 × 40 V-slot extrusion, 1.5 m', 1, 25],
  ['V-slot gantry plate + 4 V-wheels', 1, 20], ['NEMA 17 stepper (1.8°)', 1, 15], ['TMC2209 stepper driver', 1, 8], ['GT2 belt, 20T pulley and idler', 1, 10], ['Limit switches', 2, 1.5],
  ['12 V 3 A power supply', 1, 12], ['PLA for the printed parts and model ships (≈ 400 g)', 1, 9], ['MDF board 1.2 × 0.8 m + paint', 1, 18], ['Aluminium foil tape', 1, 8],
  ['M3 / M5 hardware, T-nuts', 1, 10], ['Wiring, Dupont leads, USB-C cable', 1, 8]];
const PR = { cfg: { ...PR_DEFAULT }, scan: .5, running: false, inited: false };

function targets(cfg) {
  const t = [], ship = (cx, cz, L, ang, n) => { for (let k = 0; k < n; k++) { const s = -L / 2 + L * k / (n - 1); t.push([cx + s * Math.cos(ang), cz + s * Math.sin(ang), undb(cfg.shipRcs)]); } };
  ship(-0.25, 1.81, .30, 0, 9); ship(0.05, 1.48, .30, rad(-25), 9); ship(0.38, 1.65, .18, rad(90), 6); ship(-0.42, 1.40, .18, rad(15), 6);
  const lam = C / 60e9, refl = 4 * Math.PI * .1 ** 4 / (3 * lam * lam);
  for (const [x, z] of [[-0.55, 1.25], [0.55, 1.25], [0, 1.96]]) t.push([x, z, refl]);
  return t;
}
function fft(re, im) {     // in-place radix-2
  const n = re.length;
  for (let i = 1, j = 0; i < n; i++) { let bit = n >> 1; for (; j & bit; bit >>= 1) j ^= bit; j ^= bit; if (i < j) { [re[i], re[j]] = [re[j], re[i]]; [im[i], im[j]] = [im[j], im[i]]; } }
  for (let len = 2; len <= n; len <<= 1) {
    const ang = -2 * Math.PI / len, wr = Math.cos(ang), wi = Math.sin(ang);
    for (let i = 0; i < n; i += len) { let ur = 1, ui = 0; for (let k = 0; k < len / 2; k++) {
      const a = i + k, b = a + len / 2, tr = re[b] * ur - im[b] * ui, ti = re[b] * ui + im[b] * ur;
      re[b] = re[a] - tr; im[b] = im[a] - ti; re[a] += tr; im[a] += ti; const nr = ur * wr - ui * wi; ui = ur * wi + ui * wr; ur = nr; } }
  }
}
function model(cfg) {
  const f0 = 58e9, B = cfg.B * 1e9, T = 130e-6, N = 256, fs = N / T, S = B / T, lam = C / (f0 + B / 2), H = .37;
  const nPos = Math.max(2, Math.round(cfg.railL / (cfg.step / 1000)) + 1);
  const drRange = 1.44 * C / (2 * B), dxCross = lam * 1.96 / (2 * Math.min(cfg.railL, 2 * 1.96 * Math.tan(rad(cfg.beam / 2))));
  const stepMax = lam / (4 * Math.sin(rad(cfg.beam / 2)));
  const rMax = fs / 2 * C / (2 * S);
  const eirpW = undb(cfg.eirp) / 1000, Gr = undb(6), Fn = undb(cfg.nf), Lscreen = undb(cfg.screenDb);
  const snr1 = (rcs, R) => eirpW * Gr * lam * lam * rcs / ((4 * Math.PI) ** 3 * R ** 4 * KB * 290 * Fn * (1 / T) * Lscreen);
  const inBeam = Math.min(nPos, Math.floor(2 * 1.8 * Math.tan(rad(cfg.beam / 2)) / (cfg.step / 1000)));
  const snrShip = snr1(undb(cfg.shipRcs) * 9, 1.8) * inBeam;
  const scanS = nPos * (cfg.settle / 1000 + T + .004 + (cfg.step / 1000) / .05);
  return { f0, B, T, N, fs, S, lam, H, nPos, drRange, dxCross, stepMax, rMax, snrShip, inBeam, scanS, dataMB: nPos * N * 2 * 2 / 1e6 };
}
function checks(cfg, m) {
  const k = (ok, mid) => (ok ? 'ok' : mid ? 'mid' : 'bad');
  return [
    ['Range resolution', `${fmt(m.drRange * 100, 1)} cm`, '≤ 6 cm (ship clear of the quay)', k(m.drRange <= .06, m.drRange <= .1), `c / 2B with a Hann window, ${fmt(cfg.B, 1)} GHz chirp.`],
    ['Cross-range resolution at 2 m', `${fmt(m.dxCross * 1000, 1)} mm`, '≤ 10 mm', k(m.dxCross <= .01, m.dxCross <= .02), `λ·R / 2L with L = ${fmt(cfg.railL, 2)} m. The satellite's equivalent is L / 2 for the real antenna.`],
    ['Rail sampling', `${fmt(cfg.step, 2)} mm`, `≤ ${fmt(m.stepMax * 1000, 2)} mm (λ / 4 sin(beam/2))`, k(cfg.step <= m.stepMax * 1000 * 1.001, cfg.step <= m.stepMax * 2000), 'Coarser steps alias: ghost copies appear along the rail, just like azimuth ambiguities in orbit.'],
    ['Unambiguous range', `${fmt(m.rMax, 1)} m`, '≥ 3 m', k(m.rMax >= 3, m.rMax >= 2.3), `fs / 2 × c / 2S: ${m.N} samples per ${fmt(m.T * 1e6, 0)} µs chirp.`],
    ['SNR on a model ship at 1.8 m', `${fmt(db(m.snrShip), 1)} dB`, '≥ 20 dB', k(db(m.snrShip) >= 20, db(m.snrShip) >= 12), `Radar equation over ${m.inBeam} chirps in the beam, through the cloud screen (${cfg.screenDb} dB).`],
    ['Scan time', `${fmt(m.scanS, 0)} s for ${m.nPos} chirps`, '≤ 300 s', k(m.scanS <= 300, m.scanS <= 600), 'Move, settle, chirp, repeat. The scene must stay still for the whole scan.'],
    ['Licence-free band', '58–63.5 GHz', '57–64 GHz unlicensed', 'ok', `EIRP ${cfg.eirp} dBm, far below the limit for 60 GHz field-disturbance sensors.`],
  ];
}
let V, A, beamMesh;
export function initProtoTab(api) {
  const { THREE } = api;
  if (!PR.inited) {
    PR.inited = true;
    const scene = new THREE.Scene(); api.lightScene(scene, [1, 2, -1]);
    V = api.makeView('proto', 'pr-viewport', [1.9, 1.5, -1.3], [0, .3, .8], scene, .02, 60);
    const grid = new THREE.GridHelper(6, 60, 0x2f7f8d, 0x14353f); grid.material.transparent = true; grid.material.opacity = .45; scene.add(grid);
    A = api.buildAssembly(api.PROTO.data, api.PROTO.buf); scene.add(A.root);
    beamMesh = new THREE.Mesh(new THREE.ConeGeometry(1, 1, 32, 1, true), new THREE.MeshBasicMaterial({ color: 0x6be1ed, transparent: true, opacity: .07, depthWrite: false, side: THREE.DoubleSide, blending: THREE.AdditiveBlending }));
    scene.add(beamMesh);
    $('pr-scan').addEventListener('input', (e) => { PR.scan = +e.target.value; pose(); });
    $('pr-screen').addEventListener('change', () => pose());
    $('pr-run').addEventListener('click', () => runScan());
    $('pr-reset').addEventListener('click', () => { V.camera.position.set(1.9, 1.5, -1.3); V.controls.target.set(0, .3, .8); });
    V.tick = (dt) => { if (PR.running) stepScan(); };
    renderInputs(); pose(); render(); runScan(true);
  }
}
function pose() {
  const n = A.byId.carriage, L = PR.cfg.railL;
  n.group.position.copy(n.p0); n.group.position.x = (PR.scan - .5) * L;
  const showScreen = $('pr-screen').checked;
  for (const r of A.objs) if (A.D.parts[r.pi].pn === 'TP-305') r.obj.visible = showScreen;
  const rx = n.group.position.x, len = 1.6, half = Math.tan(rad(PR.cfg.beam / 2)) * len;
  beamMesh.scale.set(half, len, half * .4); beamMesh.position.set(rx, .42, .03 + len / 2); beamMesh.rotation.set(-Math.PI / 2, 0, 0);
}
function renderInputs() {
  const c = PR.cfg;
  $('pr-inputs').innerHTML = PR_INPUTS.map(([g, list]) => `<h4>${g.toUpperCase()}</h4>` + list.map(([k, l, u, a, b, s]) =>
    `<label class="fin" for="prin-${k}">${l}<output id="prout-${k}"></output><input id="prin-${k}" type="range" min="${a}" max="${b}" step="${s}" value="${c[k]}" data-k="${k}"></label>`).join('')).join('') +
    `<button class="tl-btn" id="prin-reset">Reset to the CADed rig</button>`;
  $('pr-inputs').querySelectorAll('input').forEach((i) => i.addEventListener('input', () => { c[i.dataset.k] = +i.value; render(); }));
  $('pr-inputs').querySelectorAll('input').forEach((i) => i.addEventListener('change', () => runScan(true)));
  $('prin-reset').addEventListener('click', () => { PR.cfg = { ...PR_DEFAULT }; renderInputs(); render(); runScan(true); });
}
// ---- simulated scan + back-projection (image builds as the carriage moves)
const GX = 200, GZ = 150, X0 = -.8, X1 = .8, Z0 = 1.0, Z1 = 2.2;
let SIM = null;
function runScan(instant = false) {
  const c = PR.cfg, m = model(c), tg = targets(c), up = 4, n = m.N * up;
  const xs = []; for (let i = 0; i < m.nPos; i++) xs.push(-c.railL / 2 + c.railL * i / (m.nPos - 1));
  const win = new Float32Array(m.N); for (let i = 0; i < m.N; i++) win[i] = .5 - .5 * Math.cos(2 * Math.PI * i / (m.N - 1));
  const screen = $('pr-screen').checked ? 1 / Math.sqrt(undb(c.screenDb)) : 1;
  let seed = 7; const rnd = () => { seed = (seed * 16807) % 2147483647; return seed / 2147483647; };
  const gauss = () => Math.sqrt(-2 * Math.log(rnd() + 1e-12)) * Math.cos(2 * Math.PI * rnd());
  SIM = { m, xs, k: 0, img: new Float64Array(GX * GZ * 2), up, n, dr: C / (2 * m.S) * m.fs / n, prof: null, tg, win, screen, gauss, c };
  // noise level from the radar equation: thermal noise per sample relative to a unit echo at 1 m
  const lam = m.lam, eirpW = undb(c.eirp) / 1000, kT = KB * 290 * undb(c.nf) * m.fs;
  SIM.noise = Math.sqrt(kT / (eirpW * undb(6) * lam * lam / (4 * Math.PI) ** 3) / 2);
  PR.running = true; $('pr-status').textContent = 'scanning…';
  if (instant) { while (SIM.k < xs.length) stepScan(true); }
}
function stepScan(silent = false) {
  if (!SIM) return;
  const { m, xs, n, tg, win, screen, c } = SIM, per = silent ? 400 : Math.max(4, Math.ceil(xs.length / 140));
  const re = new Float64Array(n), im = new Float64Array(n), sinB = Math.tan(rad(c.beam / 2));
  for (let q = 0; q < per && SIM.k < xs.length; q++, SIM.k++) {
    const xp = xs[SIM.k];
    re.fill(0); im.fill(0);
    for (const [tx, tz, rcs] of tg) {
      const R = Math.sqrt((xp - tx) ** 2 + tz * tz + m.H * m.H);
      const az = Math.exp(-(((xp - tx) / tz) ** 2) / (2 * sinB * sinB));
      const amp = Math.sqrt(rcs) / (R * R) * az * screen, tau = 2 * R / C;
      for (let s = 0; s < m.N; s++) { const ph = 2 * Math.PI * (m.f0 * tau + m.S * tau * s / m.fs); re[s] += amp * Math.cos(ph); im[s] += amp * Math.sin(ph); }
    }
    for (let s = 0; s < m.N; s++) { re[s] = (re[s] + SIM.noise * SIM.gauss()) * win[s]; im[s] = (im[s] + SIM.noise * SIM.gauss()) * win[s]; }
    fft(re, im);
    // back-project onto the image grid
    for (let j = 0; j < GZ; j++) { const z = Z0 + (Z1 - Z0) * j / (GZ - 1);
      for (let i = 0; i < GX; i++) { const x = X0 + (X1 - X0) * i / (GX - 1), R = Math.sqrt((x - xp) ** 2 + z * z + m.H * m.H);
        const kf = R / SIM.dr, k0 = Math.floor(kf), f = kf - k0; if (k0 < 0 || k0 >= n / 2 - 1) continue;
        const vr = re[k0] * (1 - f) + re[k0 + 1] * f, vi = im[k0] * (1 - f) + im[k0 + 1] * f, ph = -4 * Math.PI * m.f0 * R / C, cr = Math.cos(ph), ci = Math.sin(ph);
        const p = (j * GX + i) * 2; SIM.img[p] += vr * cr - vi * ci; SIM.img[p + 1] += vr * ci + vi * cr; } }
  }
  if (!silent) { PR.scan = SIM.k / xs.length; $('pr-scan').value = PR.scan; pose(); }
  drawImage();
  if (SIM.k >= xs.length) { PR.running = false; $('pr-status').textContent = `${xs.length} chirps · focused`; }
  else $('pr-status').textContent = `${SIM.k} / ${xs.length} chirps`;
}
function drawImage() {
  const cv = $('pr-canvas'), ctx = cv.getContext('2d'), W = cv.width, Hc = cv.height, id = ctx.createImageData(GX, GZ), d = id.data;
  let mx = 1e-30; const mag = new Float64Array(GX * GZ);
  for (let k = 0; k < GX * GZ; k++) { mag[k] = Math.hypot(SIM.img[2 * k], SIM.img[2 * k + 1]); if (mag[k] > mx) mx = mag[k]; }
  for (let j = 0; j < GZ; j++) for (let i = 0; i < GX; i++) {
    const k = (GZ - 1 - j) * GX + i, v = clamp((20 * Math.log10(mag[j * GX + i] / mx + 1e-12) + 40) / 40);
    const r = v < .5 ? 0 : (v - .5) * 2 * 255, g = v * 225, b = 60 + v * 160;   // dark navy → cyan → white
    d[k * 4] = Math.min(255, r + v * v * 80); d[k * 4 + 1] = Math.min(255, g); d[k * 4 + 2] = Math.min(255, b); d[k * 4 + 3] = 255;
  }
  const tmp = document.createElement('canvas'); tmp.width = GX; tmp.height = GZ; tmp.getContext('2d').putImageData(id, 0, 0);
  ctx.imageSmoothingEnabled = true; ctx.drawImage(tmp, 0, 0, W, Hc);
  ctx.fillStyle = 'rgba(214,232,237,.8)'; ctx.font = '10px IBM Plex Mono, monospace';
  ctx.fillText(`${Z1} m`, 4, 12); ctx.fillText(`${Z0} m`, 4, Hc - 4); ctx.fillText('← along rail →', W / 2 - 40, Hc - 4);
}
function render() {
  const c = PR.cfg, m = model(c);
  for (const g of PR_INPUTS) for (const [k, , u, , , s] of g[1]) { const o = $('prout-' + k); if (o) o.textContent = `${fmt(c[k], s >= 1 ? 0 : s >= .1 ? 1 : 2)} ${u}`; }
  const ch = checks(c, m), nb = ch.filter((x) => x[3] === 'bad').length, nm = ch.filter((x) => x[3] === 'mid').length;
  const cost = PRICES.reduce((a, [, q, u]) => a + q * u, 0);
  $('pr-kpis').innerHTML = [
    ['RIG CHECKS', `${ch.length - nb - nm} / ${ch.length} pass`, `${nm} marginal · ${nb} failing`, nb ? 'bad' : nm ? 'mid' : 'ok'],
    ['RESOLUTION', `${fmt(m.drRange * 100, 1)} cm × ${fmt(m.dxCross * 1000, 1)} mm`, 'range × cross-range at 2 m', ''],
    ['SCAN', `${m.nPos} chirps`, `${fmt(m.scanS, 0)} s · ${fmt(m.dataMB, 1)} MB raw`, ''],
    ['PARTS COST', `$${fmt(cost, 0)}`, 'plus a laptop for processing', ''],
  ].map(([k, v, s, cl]) => `<div class="kpi ${cl}"><small>${k}</small><b>${v}</b><span>${s}</span></div>`).join('') + checksTable(ch);
  $('pr-imgnote').innerHTML = `<p>Seen from above: the rail runs along the bottom edge. Each bright dot cluster is a foil-taped model ship; the three strongest spots are the 100 mm corner reflectors (about ${fmt(4 * Math.PI * .1 ** 4 / (3 * (C / 60e9) ** 2), 0)} m² each at 60 GHz). Press <b>Run a scan</b> to watch the image focus as the aperture grows, or change an input to see what breaks.</p>`;
  const REPO = 'https://github.com/ASCoding11/Joukowsky-Dynamics/';
  const D = (window.__sat && window.__sat.PROTO.data) || { parts: [] };
  const printed = D.parts.filter((p) => p.spec && p.spec.print);
  $('pr-sections').innerHTML = `
    <article class="chart-card"><span class="ref">typical 2026 prices, USD</span><h4>Parts list and cost</h4>
      <div class="bom-summary"><span>Total <b>$${fmt(cost, 0)}</b></span><span>Printed parts <b>${printed.length}</b></span><span>Files <a href="${REPO}tree/main/cad/sat-proto" target="_blank" rel="noopener">cad/sat-proto ↗</a></span></div>
      <div class="proto-table"><table class="bom-table"><thead><tr><th>Item</th><th>Qty</th><th>Unit</th><th>Total</th></tr></thead><tbody>
      ${PRICES.map(([n, q, u]) => `<tr><td>${esc(n)}</td><td class="num">${q}</td><td class="num">$${fmt(u, u < 10 ? 2 : 0)}</td><td class="num">$${fmt(q * u, 0)}</td></tr>`).join('')}</tbody></table></div>
      <p>Printed parts: ${printed.map((p) => `<a href="${REPO}raw/main/cad/sat-proto/stl/${p.pn}.stl" target="_blank" rel="noopener">${esc(p.pn)} ${esc(p.name)}</a>`).join(' · ')}.</p></article>
    <div class="flight-notes">
      <div><h4>Build it</h4><ol class="motor-steps">
        <li><b>Rail.</b> Cut the V-slot to 1.2 m, print the two end plates and legs, and bolt them on. Fit the NEMA 17 on its plate at one end and the idler at the other; run the GT2 belt to the gantry plate.</li>
        <li><b>Carriage.</b> Mount the four V-wheels (one pair on eccentric spacers, adjusted until there's no play) and bolt the printed radar mount on top.</li>
        <li><b>Electronics.</b> Wire the TMC2209 to the Pi's GPIO (step, dir, enable) and the limit switches to two inputs. Mount the radar board facing the harbour and connect it to the Pi over USB.</li>
        <li><b>Harbour.</b> Paint the MDF dark, add the quay and pier, and place the printed ships wrapped in foil tape plus the three corner reflectors where the drawing shows. Prop the cardboard “cloud” screen halfway.</li>
        <li><b>Scan.</b> Home the carriage on the left switch. Step 2.5 mm, wait for vibration to settle, trigger one chirp, save the samples and the position; repeat 400 times (about ${fmt(m.scanS / 60, 1)} minutes).</li>
        <li><b>Process.</b> Save the samples as <code>scan.npz</code> and run <code>python rail_sar.py --data scan.npz</code>: range FFT, back-projection, CFAR. Try <code>--simulate</code> first to check your setup.</li>
        <li><b>Validate.</b> The corner reflectors should appear as points about ${fmt(m.drRange * 100, 0)} cm by ${fmt(m.dxCross * 1000, 0)} mm; if they're smeared along the rail, check that the step size and positions are right.</li></ol>
        <p><a href="${REPO}blob/main/satellite/prototype/rail_sar.py" target="_blank" rel="noopener">satellite/prototype/rail_sar.py ↗</a> needs only numpy (and matplotlib for the picture).</p></div>
      <div><h4>What it proves about Tideline</h4><ul>
        <li><b>Synthetic aperture:</b> a small antenna moved along a line gives a resolution set by the line's length, not the antenna's. The satellite does this over about ${fmt(5, 0)} km of orbit.</li>
        <li><b>Seeing through cloud:</b> the cardboard screen hides the harbour from a camera but costs the radar only a couple of dB.</li>
        <li><b>Ambiguities and sampling:</b> raise the step above λ/4 and ghost ships appear, which is the same physics as the satellite's PRF limit.</li>
        <li><b>Detection:</b> the same CFAR detector finds the model ships.</li></ul></div>
      <div><h4>What it doesn't</h4><p>Orbit motion, Earth curvature, range migration over hundreds of kilometres, ionospheric effects, the phased array's beam steering and real sea clutter are not reproduced. 60 GHz is used because it gives centimetre resolution on a table, and the radar boards are cheap and licence-free.</p></div>
    </div>`;
}
