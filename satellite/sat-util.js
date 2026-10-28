// Shared helpers for the Tideline workbench: DOM, maths, number formatting and the SVG line chart.
export const $ = (id) => document.getElementById(id);
export const clamp = (v, a = 0, b = 1) => Math.min(b, Math.max(a, v));
export const smooth = (t) => t * t * (3 - 2 * t);
export const ramp = (t, a, b) => smooth(clamp((t - a) / (b - a)));
export const rad = (d) => d * Math.PI / 180;
export const deg = (r) => r * 180 / Math.PI;
export const db = (x) => 10 * Math.log10(x);
export const undb = (x) => 10 ** (x / 10);
export const fmt = (v, d = 1) => Number.isFinite(v) ? v.toLocaleString('en-US', { minimumFractionDigits: d, maximumFractionDigits: d }) : '—';
export const esc = (s) => String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
export const pill = (s) => `<span class="pill ${s}">${s === 'ok' ? '● Pass' : s === 'mid' ? '▲ Marginal' : '■ Fail'}</span>`;
export function checksTable(rows) {
  return `<div class="checks"><table class="checks-table"><thead><tr><th>Check</th><th>Value</th><th>Limit</th><th>Status</th><th>Why it matters</th></tr></thead><tbody>${rows.map(([n, v, l, s, w]) =>
    `<tr class="${s}"><td>${esc(n)}</td><td class="num">${esc(v)}</td><td>${esc(l)}</td><td>${pill(s)}</td><td>${esc(w)}</td></tr>`).join('')}</tbody></table></div>`;
}
export function niceStep(raw) { const p = 10 ** Math.floor(Math.log10(raw)); const f = raw / p; return (f < 1.5 ? 1 : f < 3.5 ? 2 : f < 7.5 ? 5 : 10) * p; }
let chartId = 0;
// one y-axis line chart; series {name, c, pts:[[x,y]], dash}; options: zones, hlines, vline, logY
export function lineChart(title, ref, desc, o) {
  const W = 460, Hh = 250, L = 50, R = 14, T = 12, B = 34, id = 'sch' + (chartId++);
  const sx = (x) => L + (x - o.x[0]) / (o.x[1] - o.x[0]) * (W - L - R), sy = (y) => T + (1 - (y - o.y[0]) / (o.y[1] - o.y[0])) * (Hh - T - B);
  const ticks = (a, b, n = 5) => { const st = niceStep((b - a) / n); const out = []; for (let v = Math.ceil(a / st) * st; v <= b + 1e-9; v += st) out.push(+v.toFixed(6)); return out; };
  let s = `<svg viewBox="0 0 ${W} ${Hh}" id="${id}" data-chart='${esc(JSON.stringify({ x: o.x, L, R, W, s: o.series.filter((q) => q.name).map((q) => ({ n: q.name, c: q.c, p: q.pts })) }))}' role="img" aria-label="${esc(title)}">`;
  s += `<defs><pattern id="hatch-${id}" width="7" height="7" patternUnits="userSpaceOnUse" patternTransform="rotate(45)"><rect width="7" height="7" fill="var(--surface)"/><line x1="0" y1="0" x2="0" y2="7" stroke="var(--muted)" stroke-width="1" opacity=".5"/></pattern></defs>`;
  for (const z of o.zones || []) { const y1 = sy(Math.min(z[1], o.y[1])), y0 = sy(Math.max(z[0], o.y[0])); s += `<rect x="${L}" y="${y1}" width="${W - L - R}" height="${y0 - y1}" fill="url(#hatch-${id})" opacity=".8"/>${z[2] ? `<text x="${W - R - 4}" y="${y1 + 12}" text-anchor="end">${z[2]}</text>` : ''}`; }
  for (const v of o.yTicks || ticks(o.y[0], o.y[1])) s += `<line x1="${L}" x2="${W - R}" y1="${sy(v)}" y2="${sy(v)}" stroke="var(--line)" stroke-width="1"/><text x="${L - 6}" y="${sy(v) + 3}" text-anchor="end">${+v.toFixed(3)}</text>`;
  for (const v of ticks(o.x[0], o.x[1])) s += `<text x="${sx(v)}" y="${Hh - B + 14}" text-anchor="middle">${+v.toFixed(3)}</text>`;
  s += `<line x1="${L}" x2="${W - R}" y1="${Hh - B}" y2="${Hh - B}" stroke="var(--muted)" stroke-width="1" opacity=".5"/>`;
  s += `<text x="${(L + W - R) / 2}" y="${Hh - 4}" text-anchor="middle">${o.xl}</text><text transform="translate(11 ${(T + Hh - B) / 2}) rotate(-90)" text-anchor="middle">${o.yl}</text>`;
  for (const [y, lab] of o.hlines || []) if (y <= o.y[1] && y >= o.y[0]) s += `<line x1="${L}" x2="${W - R}" y1="${sy(y)}" y2="${sy(y)}" stroke="var(--amber)" stroke-dasharray="5 4" stroke-width="1"/><text x="${L + 4}" y="${sy(y) - 4}">${lab}</text>`;
  for (const [xv, lab] of (o.vlines || (o.vline ? [o.vline] : []))) if (xv >= o.x[0] && xv <= o.x[1]) s += `<line x1="${sx(xv)}" x2="${sx(xv)}" y1="${T}" y2="${Hh - B}" stroke="var(--text)" stroke-dasharray="2 3" opacity=".6"/><text x="${sx(xv) + 4}" y="${T + 10}" class="ttl">${lab}</text>`;
  for (const q of o.series) { const d = q.pts.filter(([x, y]) => Number.isFinite(y)).map(([x, y], i) => `${i ? 'L' : 'M'}${sx(x).toFixed(1)} ${sy(clamp(y, o.y[0] - (o.y[1] - o.y[0]) * .05, o.y[1] + (o.y[1] - o.y[0]) * .05)).toFixed(1)}`).join(''); s += `<path d="${d}" fill="none" stroke="${q.c}" stroke-width="2" ${q.dash ? `stroke-dasharray="${q.dash}"` : ''} stroke-linejoin="round" clip-path="url(#clip-${id})"/>`; }
  for (const p of o.points || []) s += `<circle cx="${sx(p[0])}" cy="${sy(p[1])}" r="3.5" fill="${p[3] || 'var(--amber)'}"/><text x="${sx(p[0]) + 6}" y="${sy(p[1]) - 5}" class="ttl">${esc(p[2])}</text>`;
  s += `<clipPath id="clip-${id}"><rect x="${L}" y="${T}" width="${W - L - R}" height="${Hh - T - B}"/></clipPath>`;
  s += `<g class="xh" style="display:none"><line y1="${T}" y2="${Hh - B}" stroke="var(--text)" opacity=".35"/></g></svg>`;
  const named = o.series.filter((q) => q.name);
  const legend = named.map((q) => `<span><i style="background:${q.c}"></i>${esc(q.name)}</span>`).join('');
  return `<article class="chart-card"><span class="ref">${esc(ref)}</span><h4>${esc(title)}</h4>${named.length > 1 ? `<div class="legend">${legend}</div>` : ''}${s}<p>${desc}</p></article>`;
}
export function wireTips(root) {
  root.querySelectorAll('svg[data-chart]').forEach((svg) => {
    const d = JSON.parse(svg.dataset.chart), card = svg.closest('.chart-card');
    let tip = card.querySelector('.chart-tip'); if (!tip) { tip = document.createElement('div'); tip.className = 'chart-tip'; tip.hidden = true; card.appendChild(tip); }
    const xh = svg.querySelector('.xh');
    svg.addEventListener('pointermove', (e) => {
      const r = svg.getBoundingClientRect(), px = (e.clientX - r.left) / r.width * d.W;
      if (px < d.L || px > d.W - d.R || !d.s.length) { tip.hidden = true; xh.style.display = 'none'; return; }
      const x = d.x[0] + (px - d.L) / (d.W - d.L - d.R) * (d.x[1] - d.x[0]);
      const rows = d.s.filter((q) => q.p.length).map((q) => { let best = q.p[0]; for (const p of q.p) if (Math.abs(p[0] - x) < Math.abs(best[0] - x)) best = p; return [q, best]; });
      if (!rows.length) return;
      xh.style.display = ''; xh.querySelector('line').setAttribute('x1', px); xh.querySelector('line').setAttribute('x2', px);
      tip.innerHTML = `x = ${fmt(rows[0][1][0], 2)}<br>` + rows.map(([q, p]) => `<span style="color:${q.c}">■</span> ${esc(q.n)}: ${fmt(p[1], 2)}`).join('<br>');
      tip.hidden = false;
      const cr = card.getBoundingClientRect();
      tip.style.left = Math.min(cr.width - 190, e.clientX - cr.left + 14) + 'px'; tip.style.top = (e.clientY - cr.top + 10) + 'px';
    });
    svg.addEventListener('pointerleave', () => { tip.hidden = true; xh.style.display = 'none'; });
  });
}
