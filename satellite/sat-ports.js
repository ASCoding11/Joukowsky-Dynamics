// 04 · Port analytics: a simulated SAR scene of a generic container port, a CA-CFAR ship detector, AIS matching,
// berth occupancy, anchorage queue, dark vessels and channel-blockage detection. The scene uses the resolution, noise
// floor and sea clutter of the chosen mode from the physics model. Each display pixel holds one resolution cell's
// clutter (decimated, not averaged), so ship-to-clutter ratios match what the sensor actually sees.
import { $, clamp, rad, db, undb, fmt, esc, checksTable } from './sat-util.js';
import { sigma0, shipRCS } from './sat-physics.js';

const NPX = 720;
const VIEW = { spot: 2600, strip: 9000, scan: 30000 };       // metres shown across the canvas
const PORT = {
  coast: (x) => 1150 + 260 * Math.sin(x / 2300) + 120 * Math.sin(x / 700 + 1),
  basins: [{ x0: -2300, x1: -700, y0: 900, y1: 2700 }, { x0: 150, x1: 1850, y0: 900, y1: 2500 }],
  breakwater: [[-3200, -420, 380, 450], [420, 3400, 380, 450]],    // x0, x1, y0, y1
  channel: { x0: -150, x1: 150, y0: -2400, y1: 1000 },            // 300 m dredged approach (Suez-like width) through the breakwater gap
  anchorage: { x0: -5200, x1: 5200, y0: -7200, y1: -3000 },
  laneY: [-11500, -9500],
};
function berths() {
  const out = [];
  for (const b of PORT.basins) {
    for (let y = b.y0 + 220; y + 180 < b.y1; y += 400) { out.push({ x: b.x0 + 45, y, side: -1 }); out.push({ x: b.x1 - 45, y, side: 1 }); }
  }
  return out;
}
const BERTHS = berths();
function isLand(x, y) {
  for (const b of PORT.basins) if (x > b.x0 && x < b.x1 && y > b.y0 - 300 && y < b.y1) return false;
  if (x > -420 && x < 420 && y > PORT.channel.y0 && y < 950) return false;   // entrance and inner harbour, not the quays between the basins
  for (const [x0, x1, y0, y1] of PORT.breakwater) if (x > x0 && x < x1 && y > y0 && y < y1) return true;
  return y > PORT.coast(x);
}
function rng(seed) { let a = seed >>> 0; return () => { a = a + 0x6D2B79F5 | 0; let t = Math.imul(a ^ a >>> 15, 1 | a); t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t; return ((t ^ t >>> 14) >>> 0) / 4294967296; }; }

// ground truth for a scenario: berthed ships, anchorage queue, moving traffic, small craft, and optionally a grounded ship
function scenario(kind, seed) {
  const R = rng(seed), ships = [];
  const occ = kind === 'normal' ? .65 : kind === 'storm' ? .7 : kind === 'congested' ? 1 : .9;
  const queue = kind === 'normal' ? 5 : kind === 'storm' ? 9 : kind === 'congested' ? 26 : 16;
  const add = (x, y, len, ang, type, ais = true) => ships.push({ x, y, len, ang, type, ais, beam: Math.max(4, len * .14) });
  BERTHS.forEach((b) => { if (R() < occ) add(b.x, b.y, 180 + R() * 150, 90, 'berthed'); });   // ≥ 70 m between hulls at 400 m berth spacing
  for (let i = 0; i < queue; i++) {
    const A = PORT.anchorage; add(A.x0 + R() * (A.x1 - A.x0), A.y0 + R() * (A.y1 - A.y0), 150 + R() * 240, R() * 360, 'anchored', !(i % 9 === 4));
  }
  for (let i = 0; i < 9; i++) add(-14000 + R() * 28000, PORT.laneY[0] + R() * (PORT.laneY[1] - PORT.laneY[0]), 120 + R() * 280, R() < .5 ? 0 : 180, 'underway');
  if (kind !== 'blocked') for (let i = 0; i < 2; i++) add((R() - .5) * 120, -1800 + R() * 2400, 160 + R() * 150, 90, 'underway');
  for (let i = 0; i < 22; i++) add((R() - .5) * 12000, -1500 - R() * 9000, 8 + R() * 14, R() * 360, 'small', R() < .45);
  for (let i = 0; i < 8; i++) add((R() - .5) * 2600, 500 + R() * 1800, 15 + R() * 20, R() * 360, 'tug', true);
  if (kind === 'blocked') add(0, -600, 400, 28, 'grounded');   // swung across the fairway, bow and stern on the banks
  return ships.filter((s) => !isLand(s.x, s.y) || s.type === 'berthed');
}

const PT = { scn: 'normal', mode: 'strip', view: 'sar', det: true, seed: 11, inited: false, last: null };
export function initPorts(api) {
  if (PT.inited) return;
  PT.inited = true;
  $('pt-scn').addEventListener('change', (e) => { PT.scn = e.target.value; renderPorts(api); });
  $('pt-mode').addEventListener('change', (e) => { PT.mode = e.target.value; renderPorts(api); });
  $('pt-det').addEventListener('change', (e) => { PT.det = e.target.checked; draw(api); });
  $('pt-new').addEventListener('click', () => { PT.seed = (PT.seed * 7919 + 13) % 100000; renderPorts(api); });
  document.querySelectorAll('[data-ptview]').forEach((b) => b.addEventListener('click', () => { PT.view = b.dataset.ptview; document.querySelectorAll('[data-ptview]').forEach((x) => x.classList.toggle('active', x === b)); draw(api); }));
}

// gamma-ish texture for rough seas (K-distributed clutter) and a cheap value-noise for land and clouds
function noise2(seed) {
  const R = rng(seed), g = new Float32Array(66 * 66); for (let i = 0; i < g.length; i++) g[i] = R();
  return (u, v) => { u = (u % 64 + 64) % 64; v = (v % 64 + 64) % 64; const i = Math.floor(u), j = Math.floor(v), fu = u - i, fv = v - j, s = (t) => t * t * (3 - 2 * t);
    const i1 = (i + 1) % 64, j1 = (j + 1) % 64;   // wrap the lattice so there are no seams
    const a = g[j * 66 + i], b = g[j * 66 + i1], c = g[j1 * 66 + i], d = g[j1 * 66 + i1]; return a + (b - a) * s(fu) + (c - a) * s(fv) + (a - b - c + d) * s(fu) * s(fv); };
}

export function renderPorts(api) {
  if (!api.F) return;
  const P = api.model(), F = api.F, M = P[PT.mode];
  const wind = PT.scn === 'storm' ? 15 : F.wind;
  const S = VIEW[PT.mode], px = S / NPX, x0 = -S / 2, y0 = PT.mode === 'scan' ? -S / 2 - 3500 : PT.mode === 'strip' ? -S / 2 - 1300 : -S / 2 + 400;
  const truth = scenario(PT.scn, PT.seed);
  const R = rng(PT.seed * 3 + PT.mode.length), nz = noise2(PT.seed), nz2 = noise2(PT.seed + 5);
  const s0 = sigma0(M.th, wind), N = M.nesz, Ares = M.drg * M.dAz;
  const img = new Float32Array(NPX * NPX), land = new Uint8Array(NPX * NPX);
  const texK = PT.scn === 'storm' ? 1.6 : .35;     // sea texture strength (rougher sea → spikier clutter)
  for (let j = 0; j < NPX; j++) for (let i = 0; i < NPX; i++) {
    const x = x0 + (i + .5) * px, y = y0 + (NPX - j - .5) * px, k = j * NPX + i;
    let mean;
    if (isLand(x, y)) { land[k] = 1; const t = nz(x / 420, y / 420); mean = undb(-9 + 8 * t + (Math.abs(y - PORT.coast(x)) < px * 1.5 ? 10 : 0)); }
    else { const tex = Math.exp(texK * (nz2(x / 900, y / 900) - .5) * 2.2); mean = s0 * tex; }
    img[k] = (mean + N) * -Math.log(1 - R() * .999999);    // single-look speckle (exponential)
  }
  // quay cranes: bright points along the basin edges
  for (const b of BERTHS) for (let t = -150; t <= 150; t += 75) if (isLand(b.x + b.side * 75, b.y + t)) addPoint(b.x + b.side * 75, b.y + t, 3000);   // on the quay, landward of the berth
  // ships: scatterers along the hull, total RCS from length, relative to one resolution cell; sidelobe cross for bright ones
  function addPoint(x, y, rcs) {
    const i = Math.floor((x - x0) / px), j = NPX - 1 - Math.floor((y - y0) / px);
    if (i < 1 || j < 1 || i >= NPX - 1 || j >= NPX - 1) return;
    const v = rcs / Ares * (.6 + .8 * R());
    img[j * NPX + i] += v;
    const sl = v * .05; img[j * NPX + i - 1] += sl; img[j * NPX + i + 1] += sl; img[(j - 1) * NPX + i] += sl; img[(j + 1) * NPX + i] += sl;
  }
  for (const s of truth) {
    const n = Math.max(2, Math.ceil(s.len / Math.max(px, 3) * 1.5)), rcs = shipRCS(s.len), ca = Math.cos(rad(s.ang)), sa = Math.sin(rad(s.ang));
    for (let k = 0; k < n; k++) { const u = (k / (n - 1) - .5) * s.len, w = (R() - .5) * s.beam; addPoint(s.x + u * ca - w * sa, s.y + u * sa + w * ca, rcs / n * (k > n * .7 ? 1.8 : 1)); }
  }
  // CA-CFAR on the sea (land mask dilated by 2 px), threshold = the physics tab's CFAR setting
  const sea = new Uint8Array(NPX * NPX);
  for (let j = 2; j < NPX - 2; j++) for (let i = 2; i < NPX - 2; i++) { let l = 0; for (let dj = -2; dj <= 2 && !l; dj++) for (let di = -2; di <= 2; di++) if (land[(j + dj) * NPX + i + di]) { l = 1; break; } sea[j * NPX + i] = l ? 0 : 1; }
  // guard and background windows sized in metres, so a 400 m hull never fills its own background ring
  const g = Math.max(2, Math.round(120 / px)), b = clamp(Math.round(400 / px), 8, 120), alpha = undb(F.thr);
  const cfar = (bgMask) => {
    const I1 = integral(img, bgMask), C1 = integral(null, bgMask), det = new Uint8Array(NPX * NPX);
    const cl = (v) => Math.min(NPX - 1, Math.max(0, v));   // windows clipped at the image edge
    for (let j = 0; j < NPX; j++) for (let i = 0; i < NPX; i++) {
      const k = j * NPX + i; if (!sea[k]) continue;
      const o = [cl(i - g - b), cl(j - g - b), cl(i + g + b), cl(j + g + b)], n = [cl(i - g), cl(j - g), cl(i + g), cl(j + g)];
      const so = boxSum(I1, ...o) - boxSum(I1, ...n), co = boxSum(C1, ...o) - boxSum(C1, ...n);
      if (co > 20 && img[k] > alpha * so / co) det[k] = 1;
    }
    return det;
  };
  // three passes: each removes everything detected so far (plus a 1-pixel border) from the background, so bright
  // ships do not raise the threshold for their own hull or their neighbours
  let det = cfar(sea);
  for (let pass = 0; pass < 2; pass++) {
    const bg = new Uint8Array(sea);
    for (let j = 1; j < NPX - 1; j++) for (let i = 1; i < NPX - 1; i++) if (det[j * NPX + i]) for (let dj = -1; dj <= 1; dj++) for (let di = -1; di <= 1; di++) bg[(j + dj) * NPX + i + di] = 0;
    det = cfar(bg);
  }
  // join pixels closer than ~22 m into one object: closes gaps inside a hull, but berths leave ≥ 70 m between hulls
  const objs = components(det, img, px, x0, y0, Math.max(1, Math.round(22 / px)));
  // AIS reports (ships that broadcast, ±15 m), matched to detections
  const ais = truth.filter((s) => s.ais).map((s) => ({ s, x: s.x + (R() - .5) * 30, y: s.y + (R() - .5) * 30 }));
  const tol = Math.max(80, 3 * px);
  for (const o of objs) {
    let best = null, bd = 1e9; for (const a of ais) { const d = Math.hypot(a.x - o.x, a.y - o.y); if (d < bd) { bd = d; best = a; } }
    o.ais = bd < Math.max(tol, (best ? best.s.len / 2 : 0) + tol / 2) ? best : null;
    let tb = null, td = 1e9; for (const s of truth) { const d = Math.hypot(s.x - o.x, s.y - o.y); if (d < td) { td = d; tb = s; } }
    o.truth = td < Math.max(tol, (tb ? tb.len / 2 + tol / 2 : 0)) ? tb : null;
  }
  const inView = truth.filter((s) => s.x > x0 && s.x < x0 + S && s.y > y0 && s.y < y0 + S);
  const found = new Set(objs.filter((o) => o.truth).map((o) => o.truth));
  const big = inView.filter((s) => s.len >= M.lmin(wind));
  const pd = big.length ? big.filter((s) => found.has(s)).length / big.length : 1;
  const fa = objs.filter((o) => !o.truth).length;
  const dark = objs.filter((o) => !o.ais && o.truth).length;
  const inBox = (o, B) => o.x > B.x0 && o.x < B.x1 && o.y > B.y0 && o.y < B.y1;
  const berthUsed = BERTHS.filter((bth) => objs.some((o) => Math.abs(o.x - bth.x) < 150 + px && Math.abs(o.y - bth.y) < 200)).length;
  const berthVisible = BERTHS.filter((bth) => bth.x > x0 && bth.x < x0 + S && bth.y > y0 && bth.y < y0 + S).length;
  const queue = objs.filter((o) => inBox(o, PORT.anchorage) && o.lenEst > 60).length;
  const anchVisible = PORT.anchorage.y1 > y0 && PORT.anchorage.y0 < y0 + S;
  const chan = objs.filter((o) => inBox(o, { x0: PORT.channel.x0 - 100, x1: PORT.channel.x1 + 100, y0: PORT.channel.y0, y1: PORT.channel.y1 }));
  // blocked: a contact lying across the fairway (≥ 30° to its axis) whose cross-channel extent covers ≥ 60 % of its width
  const chanW = PORT.channel.x1 - PORT.channel.x0;
  const blocker = chan.find((o) => o.lenEst * Math.abs(Math.cos(rad(o.ang))) >= .6 * chanW && Math.abs(Math.cos(rad(o.ang))) >= Math.sin(rad(30)));
  const throughput = BERTHS.length * 24 / 30;    // ships per day the berths can turn (30 h average stay)
  window.__pt = PT;
  PT.last = { P, M, S, px, x0, y0, img, land, objs, truth, inView, pd, fa, dark, berthUsed, berthVisible, queue, anchVisible, chan, blocker, throughput, wind, N, s0 };
  draw(api);
  // KPIs and alerts
  const waitH = queue / throughput * 24;
  $('pt-kpis').innerHTML = [
    ['SHIPS DETECTED', `${objs.length}`, `${inView.length} in view · ${fmt(pd * 100, 0)} % of ships ≥ ${fmt(M.lmin(wind), 0)} m found`, pd >= .9 ? 'ok' : pd >= .7 ? 'mid' : 'bad'],
    ['FALSE ALARMS', `${fa}`, `CFAR at ${F.thr} dB over the sea`, fa <= 2 ? 'ok' : fa <= 6 ? 'mid' : 'bad'],
    ['DARK VESSELS', `${dark}`, 'real contacts with no AIS report', dark ? 'mid' : ''],
    ['BERTH OCCUPANCY', berthVisible ? `${berthUsed} / ${berthVisible}` : '—', berthVisible ? `${fmt(berthUsed / berthVisible * 100, 0)} %` : 'berths outside this view', berthVisible && berthUsed / berthVisible > .95 ? 'mid' : ''],
    ['ANCHORAGE QUEUE', anchVisible ? `${queue} ships` : '—', anchVisible ? `≈ ${fmt(waitH, 0)} h wait at ${fmt(throughput, 1)} ships/day` : 'switch to stripmap or ScanSAR to see it', anchVisible && queue > 12 ? 'bad' : anchVisible && queue > 7 ? 'mid' : ''],
    ['CHANNEL', blocker ? 'BLOCKED' : chan.length ? `${chan.length} transiting` : 'clear', blocker ? `${fmt(blocker.lenEst, 0)} m vessel across the fairway` : 'entrance and approach', blocker ? 'bad' : 'ok'],
  ].map(([k, v, s, c]) => `<div class="kpi ${c}"><small>${k}</small><b>${v}</b><span>${s}</span></div>`).join('');
  const al = [];
  if (blocker) al.push(['bad', 'Channel blocked', `A ${fmt(blocker.lenEst, 0)} m vessel lies across the ${fmt(chanW, 0)} m approach channel, ${fmt(90 - Math.abs(((blocker.ang % 180) + 180) % 180 - 90), 0)}° off the fairway axis. ${anchVisible ? queue + ' ships are holding at anchor.' : 'Check the anchorage in stripmap.'}`]);
  if (anchVisible && queue > 12) al.push(['mid', 'Congestion', `${queue} ships at anchor; at about ${fmt(throughput, 1)} ships a day through the berths the queue is about ${fmt(waitH / 24, 1)} days long.`]);
  const nd = objs.filter((o) => !o.ais && o.truth).length;
  if (nd) al.push(['mid', 'Dark vessels', `${nd} radar contacts have no AIS report within ${fmt(tol, 0)} m: candidates for follow-up.`]);
  if (PT.scn === 'storm') al.push(['ok', 'Seeing through the storm', `Cloud and darkness hide everything from optical sensors. The radar still works, but the ${wind} m/s sea raises clutter, so small craft drop out and false alarms rise.`]);
  if (!al.length) al.push(['ok', 'Normal operations', 'Traffic flowing; no blockage or unusual queue.']);
  $('pt-alerts').innerHTML = al.map(([c, t, d]) => `<div class="pt-alert ${c}"><b>${esc(t)}</b><span>${esc(d)}</span></div>`).join('');
  // sections: chokepoints, method
  const POINTS = [['Strait of Malacca / Singapore', 1.3, 'Busiest strait; dense small craft', 'ScanSAR + spotlight on anchorages'], ['Panama Canal', 9.1, 'Locks and Gatun Lake queues', 'Stripmap on both entrances'],
    ['Bab-el-Mandeb', 12.6, 'Red Sea entrance; diversions', 'ScanSAR on the lane'], ['Strait of Hormuz', 26.6, 'Tanker traffic, dark transits', 'ScanSAR + AIS gap analysis'],
    ['Suez Canal', 30.0, 'Single-lane sections; blockages', 'Spotlight on convoy anchorages'], ['Port of Shanghai', 31.2, 'Largest container port', 'Stripmap of Yangshan and anchorages'],
    ['Port of Los Angeles / Long Beach', 33.7, 'Anchorage queues in congestion', 'Stripmap'], ['Strait of Gibraltar', 36.0, 'Atlantic–Mediterranean gateway', 'ScanSAR'],
    ['Bosporus', 41.1, 'Narrow, winding, tanker limits', 'Spotlight'], ['Dover Strait', 51.0, 'Traffic separation scheme', 'ScanSAR'], ['Port of Rotterdam', 51.9, 'Largest port in Europe', 'Stripmap']];
  $('pt-sections').innerHTML = `
    <article class="chart-card"><span class="ref">mean revisit from the physics tab · ${F.nSat} satellite${F.nSat > 1 ? 's' : ''}, ${F.incMin}–${F.incMax}° access, dawn-dusk orbit</span><h4>Chokepoints and ports Tideline watches</h4>
      <div class="proto-table"><table class="bom-table"><thead><tr><th>Place</th><th>Latitude</th><th>Revisit</th><th>With 4 sats</th><th>Why watch it</th><th>Best mode</th></tr></thead><tbody>
      ${POINTS.map(([n, lat, why, m]) => `<tr><td>${esc(n)}</td><td class="num">${fmt(lat, 1)}°</td><td class="num">${fmt(P.revisitH(lat), 0)} h</td><td class="num">${fmt(P.revisitH(lat, 4), 0)} h</td><td>${esc(why)}</td><td>${esc(m)}</td></tr>`).join('')}</tbody></table></div></article>
    <div class="flight-notes">
      <div><h4>How the analysis works</h4><ul>
        <li><b>Detect:</b> cell-averaging CFAR compares each sea pixel with the mean sea level in a ring from ${fmt(g * px, 0)} to ${fmt((g + b) * px, 0)} m around it and flags it above ${F.thr} dB. Land is masked out, and three passes remove ships already found from the background.</li>
        <li><b>Group and measure:</b> flagged pixels are joined into objects; length and heading come from the spread of each object's pixels.</li>
        <li><b>Match AIS:</b> each contact is matched to the nearest AIS report; contacts with none are dark vessels.</li>
        <li><b>Port state:</b> contacts at berths give occupancy; contacts in the anchorage give the queue and, with berth throughput, the waiting time; a long contact lying across the channel raises a blockage alert.</li></ul></div>
      <div><h4>What this simulation is, and is not</h4><p>The port is generic, not a real place. Sea, land and ships are rendered with the selected mode's resolution (${fmt(M.drg, 1)} × ${fmt(M.dAz, 1)} m), noise floor (${fmt(M.neszDb, 1)} dB) and the X-band sea clutter for ${wind} m/s wind, with single-look speckle and a fitted ship-RCS rule. Real scenes add azimuth ghosts of bright ships, moving-ship smear and wakes, and complex land returns. The detector is the same kind used operationally.</p></div>
    </div>`;
}
function integral(img, mask) {
  const W = NPX + 1, I = new Float64Array(W * W);
  for (let j = 0; j < NPX; j++) { let row = 0; for (let i = 0; i < NPX; i++) { const k = j * NPX + i; row += mask[k] ? (img ? img[k] : 1) : 0; I[(j + 1) * W + i + 1] = I[j * W + i + 1] + row; } }
  return I;
}
function boxSum(I, i0, j0, i1, j1) { const W = NPX + 1; return I[(j1 + 1) * W + i1 + 1] - I[j0 * W + i1 + 1] - I[(j1 + 1) * W + i0] + I[j0 * W + i0]; }
function components(det, img, px, x0, y0, rad = 2) {
  const lab = new Int32Array(NPX * NPX), out = []; let n = 0;
  for (let k = 0; k < det.length; k++) {
    if (!det[k] || lab[k]) continue;
    n++; const st = [k]; lab[k] = n; const pts = [];
    while (st.length) {
      const q = st.pop(), i = q % NPX, j = (q / NPX) | 0; pts.push([i, j, img[q]]);
      for (let dj = -rad; dj <= rad; dj++) for (let di = -rad; di <= rad; di++) { const ii = i + di, jj = j + dj; if (ii < 0 || jj < 0 || ii >= NPX || jj >= NPX) continue; const r = jj * NPX + ii; if (det[r] && !lab[r]) { lab[r] = n; st.push(r); } }
    }
    let sw = 0, mi = 0, mj = 0; for (const [i, j, w] of pts) { sw += w; mi += i * w; mj += j * w; } mi /= sw; mj /= sw;
    let cxx = 0, cyy = 0, cxy = 0; for (const [i, j] of pts) { cxx += (i - mi) ** 2; cyy += (j - mj) ** 2; cxy += (i - mi) * (j - mj); } cxx /= pts.length; cyy /= pts.length; cxy /= pts.length;
    const tr = cxx + cyy, dt = Math.sqrt(Math.max(0, (cxx - cyy) ** 2 / 4 + cxy * cxy)), l1 = tr / 2 + dt;
    const ang = -0.5 * Math.atan2(2 * cxy, cxx - cyy) * 180 / Math.PI;   // major axis, degrees from +x (image j points down, so negate)
    out.push({ i: mi, j: mj, x: x0 + (mi + .5) * px, y: y0 + (NPX - mj - .5) * px, n: pts.length, lenEst: (Math.sqrt(12 * l1) + 1) * px, ang, peak: Math.max(...pts.map((p) => p[2])) });
  }
  return out;
}
function draw(api) {
  const L = PT.last; if (!L) return;
  const cv = $('pt-canvas'), ctx = cv.getContext('2d'), id = ctx.createImageData(NPX, NPX), d = id.data;
  if (PT.view === 'sar') {
    const lo = db(L.s0 + L.N) - 6, hi = lo + 32;
    for (let k = 0; k < L.img.length; k++) { const v = clamp((db(L.img[k]) - lo) / (hi - lo)) * 255; d[k * 4] = v * .92; d[k * 4 + 1] = v; d[k * 4 + 2] = v * 1.04; d[k * 4 + 3] = 255; }
  } else {
    const night = PT.scn === 'storm', cover = PT.scn === 'storm' ? 1 : PT.scn === 'normal' ? .55 : .7, nz = noise2(PT.seed + 99), nz3 = noise2(PT.seed + 7);
    for (let j = 0; j < NPX; j++) for (let i = 0; i < NPX; i++) {
      const k = j * NPX + i, x = L.x0 + (i + .5) * L.px, y = L.y0 + (NPX - j - .5) * L.px;
      let r = L.land[k] ? 120 : 24, g = L.land[k] ? 124 : 70, b = L.land[k] ? 118 : 110;
      const cl = clamp((nz(x / 2600, y / 2600) * .65 + nz3(x / 900, y / 900) * .35 - (1 - cover)) * 2.4);
      r = r * (1 - cl) + 236 * cl; g = g * (1 - cl) + 238 * cl; b = b * (1 - cl) + 240 * cl;
      if (night) { r *= .06; g *= .07; b *= .09; }
      d[k * 4] = r; d[k * 4 + 1] = g; d[k * 4 + 2] = b; d[k * 4 + 3] = 255;
    }
    if (!night) for (const s of L.truth) { const i = Math.floor((s.x - L.x0) / L.px), j = NPX - 1 - Math.floor((s.y - L.y0) / L.px); if (i > 0 && j > 0 && i < NPX && j < NPX) { const k = j * NPX + i; if (d[k * 4] < 200) { d[k * 4] = d[k * 4 + 1] = d[k * 4 + 2] = 235; } } }
  }
  ctx.putImageData(id, 0, 0);
  // overlays: zones, detections
  const toPx = (x, y) => [(x - L.x0) / L.px, NPX - (y - L.y0) / L.px];
  ctx.lineWidth = 1; ctx.setLineDash([4, 4]); ctx.strokeStyle = 'rgba(107,225,237,.55)'; ctx.font = '11px IBM Plex Mono, monospace'; ctx.fillStyle = 'rgba(107,225,237,.85)';
  const A = PORT.anchorage, [ax, ay] = toPx(A.x0, A.y1), [bx, by] = toPx(A.x1, A.y0); ctx.strokeRect(ax, ay, bx - ax, by - ay); ctx.fillText('ANCHORAGE', ax + 6, ay + 14);
  const Cn = PORT.channel, [cx0, cy0] = toPx(Cn.x0, Cn.y1), [cx1, cy1] = toPx(Cn.x1, Cn.y0); ctx.strokeRect(cx0, cy0, cx1 - cx0, cy1 - cy0); ctx.fillText('CHANNEL', cx1 + 4, (cy0 + cy1) / 2);
  if (PT.mode === 'scan') { const [lx, ly] = toPx(-15000, PORT.laneY[1]), [mx, my] = toPx(15000, PORT.laneY[0]); ctx.strokeRect(lx, ly, mx - lx, my - ly); ctx.fillText('SHIPPING LANE', lx + 10, ly + 14); }
  ctx.setLineDash([]);
  if (PT.det && PT.view === 'sar') for (const o of L.objs) {
    const r = Math.max(5, o.lenEst / L.px / 2 + 3);
    ctx.strokeStyle = !o.truth ? '#e0675a' : o.ais ? '#5fbf7a' : '#ffb35c'; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.arc(o.i, o.j, r, 0, Math.PI * 2); ctx.stroke();
  }
  if (L.blocker && PT.det) { ctx.strokeStyle = '#e0675a'; ctx.lineWidth = 2.5; ctx.strokeRect(L.blocker.i - 30, L.blocker.j - 30, 60, 60); ctx.fillStyle = '#e0675a'; ctx.fillText('BLOCKED', L.blocker.i + 34, L.blocker.j); }
  // scale bar
  const bar = L.S >= 20000 ? 5000 : L.S >= 8000 ? 2000 : 500, bpx = bar / L.px;
  ctx.fillStyle = 'rgba(6,18,26,.75)'; ctx.fillRect(10, NPX - 36, bpx + 90, 26); ctx.fillStyle = '#d6e8ed'; ctx.fillRect(18, NPX - 22, bpx, 3); ctx.fillText(`${bar >= 1000 ? bar / 1000 + ' km' : bar + ' m'}`, 24 + bpx, NPX - 17);
  $('pt-legend').innerHTML = PT.view === 'sar'
    ? `<span>${PT.mode.toUpperCase()} · ${fmt(L.M.drg, 1)} × ${fmt(L.M.dAz, 1)} m · NESZ ${fmt(L.M.neszDb, 1)} dB · wind ${L.wind} m/s</span><span><i class="g"></i>AIS match <i class="o"></i>dark <i class="r"></i>false alarm</span>`
    : `<span>OPTICAL · ${PT.scn === 'storm' ? 'night and full cloud: nothing visible' : PT.scn === 'normal' ? '55 % cloud cover' : '70 % cloud cover'}</span><span>SAR sees through both</span>`;
}
