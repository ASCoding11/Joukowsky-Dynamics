// Tideline — X-band SAR smallsat workbench: mission sequence, advanced blueprint, SAR & mission physics,
// port analytics and the ground (rail SAR) prototype. Geometry from cad/source/build_sat.py and build_railsar.py.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { PHYS_INPUTS, physDefaults, satModel, physChecks, renderPhysics } from './sat-physics.js';
import { initPorts, renderPorts } from './sat-ports.js';
import { initProtoTab } from './sat-proto.js';
import { $, clamp, smooth, ramp, rad, fmt, esc } from './sat-util.js';

const REPO = 'https://github.com/ASCoding11/Joukowsky-Dynamics/blob/main/';
const BASE = new URL('./', import.meta.url);

// ------------------------------------------------------------------ tabs
const TABS = { mission: 'mission-panel', blueprint: 'blueprint-panel', physics: 'physics-panel', ports: 'ports-panel', proto: 'proto-panel' };
let activeTab = 'mission';
document.querySelectorAll('[data-tab]').forEach((b) => b.addEventListener('click', () => showTab(b.dataset.tab)));
function showTab(tab) {
  activeTab = tab;
  document.querySelectorAll('[data-tab]').forEach((b) => b.classList.toggle('active', b.dataset.tab === tab));
  for (const [t, id] of Object.entries(TABS)) { const el = $(id); el.hidden = t !== tab; el.classList.toggle('active', t === tab); }
  try { history.replaceState(null, '', '#' + tab); } catch (e) {}
  boot().then(() => activate(tab));
}

// ------------------------------------------------------------------ data
const SAT = {}, PROTO = {};
let bootPromise;
const fetchJSON = (p) => fetch(new URL(p, BASE)).then((r) => (r.ok ? r.json() : Promise.reject(new Error(p + ' ' + r.status))));
const fetchBin = (p) => fetch(new URL(p, BASE)).then((r) => (r.ok ? r.arrayBuffer() : Promise.reject(new Error(p + ' ' + r.status))));
function boot() {
  if (!bootPromise) {
    bootPromise = Promise.all([fetchJSON('data/parts.json'), fetchBin('data/meshes.bin')])
      .then(([d, b]) => { SAT.data = d; SAT.buf = b; buildEngine(); initBlueprint(); initPhysics(); initMission(); })
      .catch((e) => { for (const id of ['seq-loading', 'adv-loading']) { const el = $(id); if (el) el.textContent = 'CAD DATA FAILED TO LOAD: ' + e.message; } throw e; });
  }
  return bootPromise;
}
let protoPromise;
function bootProto() {
  if (!protoPromise) {
    protoPromise = Promise.all([fetchJSON('data/proto/parts.json'), fetchBin('data/proto/meshes.bin')])
      .then(([d, b]) => { PROTO.data = d; PROTO.buf = b; initProtoTab(api); $('pr-loading')?.remove(); })
      .catch((e) => { const el = $('pr-loading'); if (el) el.textContent = 'PROTOTYPE CAD FAILED TO LOAD: ' + e.message; throw e; });
  }
  return protoPromise;
}

// ------------------------------------------------------------------ engine (one renderer, a scene per view)
let renderer, envTex, current = null;
const views = {};
function buildEngine() {
  renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  renderer.setClearColor(0x06121a, 1);
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.localClippingEnabled = true;
  const pm = new THREE.PMREMGenerator(renderer);
  envTex = pm.fromScene(new RoomEnvironment(), 0.04).texture;
  renderer.setAnimationLoop(loop);
  new ResizeObserver(() => resize()).observe(document.body);
}
function lightScene(scene, sunDir = [-.4, .8, -.9]) {
  scene.add(new THREE.HemisphereLight(0xcfe9ff, 0x0b1a22, 0.65));
  const sun = new THREE.DirectionalLight(0xffffff, 1.8); sun.position.set(...sunDir); scene.add(sun);
}
function makeView(name, containerId, camPos, target, scene, near = 0.05, far = 400) {
  const camera = new THREE.PerspectiveCamera(36, 1, near, far);
  camera.position.set(...camPos);
  const el = $(containerId);
  const controls = new OrbitControls(camera, el);
  controls.target.set(...target); controls.enableDamping = true; controls.dampingFactor = 0.08;
  views[name] = { camera, controls, el, scene, tick: null };
  return views[name];
}
function activate(tab) {
  if (tab === 'physics') { current = null; renderPhysics(api); return; }
  if (tab === 'ports') { current = null; initPorts(api); renderPorts(api); return; }
  if (tab === 'proto') { current = null; bootProto().then(() => { if (activeTab === 'proto') attach('proto'); }); return; }
  attach(tab);
}
function attach(name) {
  const v = views[name]; if (!v) return;
  current = name; v.el.appendChild(renderer.domElement); resize();
}
function resize() {
  if (!current) return;
  const v = views[current], w = v.el.clientWidth, h = v.el.clientHeight;
  if (!w || !h) return;
  renderer.setSize(w, h, false); v.camera.aspect = w / h; v.camera.updateProjectionMatrix();
}
const clock = new THREE.Clock();
function loop() {
  const dt = Math.min(clock.getDelta(), 0.1);
  if (!current || document.hidden) return;
  const v = views[current];
  if (v.tick) v.tick(dt);
  v.controls.update();
  const t = performance.now() / 1000;
  for (const m of holoMats) m.uniforms.uTime.value = t;
  renderer.render(v.scene, v.camera);
}

// ------------------------------------------------------------------ materials
const LOOKS = {
  metal: { metalness: .85, roughness: .32 }, paint: { metalness: .25, roughness: .3, clearcoat: 1 },
  carbon: { metalness: .3, roughness: .42 }, glass: { metalness: .2, roughness: .15 },
  rubber: { metalness: 0, roughness: .92 }, light: { metalness: 0, roughness: .1, emissive: 0x9fd8ff, emissiveIntensity: .6 },
  cell: { metalness: .5, roughness: .4 }, pcb: { metalness: .2, roughness: .6 }, plastic: { metalness: .05, roughness: .7 },
  fabric: { metalness: 0, roughness: .95 }, hv: { metalness: .1, roughness: .55 }, copper: { metalness: .9, roughness: .3 },
};
function matFactory(materials) {
  const cache = {};
  return (key) => cache[key] || (cache[key] = (() => {
    const m = materials[key], L = LOOKS[m.look] || LOOKS.metal;
    const mat = new THREE.MeshPhysicalMaterial({ color: new THREE.Color(m.color), envMap: envTex, envMapIntensity: .9, side: THREE.DoubleSide, ...L });
    return mat;
  })());
}
const cadLine = new THREE.LineBasicMaterial({ color: 0x02080c, transparent: true, opacity: .38 });
const holoMats = [];
function holoMat(color, opacity, rimK = 1.6) {
  const m = new THREE.ShaderMaterial({
    transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, side: THREE.DoubleSide, clipping: true,
    uniforms: { uColor: { value: new THREE.Color(color) }, uOpacity: { value: opacity }, uReveal: { value: 99 }, uTime: { value: 0 }, uRim: { value: rimK } },
    vertexShader: `varying vec3 vN; varying vec3 vV; varying vec3 vW;
      #include <clipping_planes_pars_vertex>
      void main(){ vec4 p = vec4(position,1.); vec3 n = normal;
        #ifdef USE_INSTANCING
          p = instanceMatrix*p; n = mat3(instanceMatrix)*n;
        #endif
        vec4 w = modelMatrix*p; vW = w.xyz; vN = normalize(mat3(modelMatrix)*n); vV = cameraPosition - w.xyz;
        vec4 mvPosition = viewMatrix*w;
        #include <clipping_planes_vertex>
        gl_Position = projectionMatrix*mvPosition; }`,
    fragmentShader: `uniform vec3 uColor; uniform float uOpacity; uniform float uReveal; uniform float uTime; uniform float uRim;
      varying vec3 vN; varying vec3 vV; varying vec3 vW;
      #include <clipping_planes_pars_fragment>
      void main(){
        #include <clipping_planes_fragment>
        if (vW.x > uReveal) discard;
        float f = 1. - abs(dot(normalize(vN), normalize(vV)));
        float rim = pow(f, 2.2);
        float band = smoothstep(.25, 0., abs(vW.x - uReveal));
        float scan = .5 + .5*sin(vW.y*60. - uTime*3.);
        float a = uOpacity*(.22 + rim*uRim) + band*.5;
        gl_FragColor = vec4(uColor*(.55 + rim*1.1 + .12*scan) + vec3(.8,1.,1.)*band, a); }`,
  });
  holoMats.push(m);
  return m;
}
const HOLO = () => ({ mesh: holoMat(0x2fb8cc, .10), dense: holoMat(0x2fb8cc, .012, .5), meshHot: holoMat(0xffb35c, .18), line: new THREE.LineBasicMaterial({ color: 0x6be1ed, transparent: true, opacity: .35, blending: THREE.AdditiveBlending, depthWrite: false }) });
const highlightMat = new THREE.MeshBasicMaterial({ color: 0xffb35c, transparent: true, opacity: .55, depthTest: false });
const highlightLine = new THREE.LineBasicMaterial({ color: 0xffe2b8, depthTest: false });
const ghostMat = new THREE.MeshBasicMaterial({ color: 0x6be1ed, transparent: true, opacity: .035, depthWrite: false });

// ------------------------------------------------------------------ generic assembly builder (used for the satellite twice and the prototype)
function decode(p, buf) {
  const m = p.mesh;
  const q = new Int16Array(buf, m.voff, m.nv * 3);
  const pos = new Float32Array(m.nv * 3);
  for (let i = 0; i < pos.length; i++) { const k = i % 3; pos[i] = (q[i] + 32767) / 65534 * m.ext[k] + m.min[k]; }
  const idx = m.wide ? new Uint32Array(buf.slice(m.ioff, m.ioff + m.ni * 4)) : new Uint16Array(buf.slice(m.ioff, m.ioff + m.ni * 2));
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  g.setIndex(new THREE.BufferAttribute(idx, 1));
  g.computeVertexNormals(); g.computeBoundingBox(); g.computeBoundingSphere();
  return g;
}
const GEO_CACHE = new WeakMap();
function geometries(D, buf) {
  if (!GEO_CACHE.has(D)) GEO_CACHE.set(D, D.parts.map((p) => { const geo = decode(p, buf); return { geo, edges: new THREE.EdgesGeometry(geo, 28) }; }));
  return GEO_CACHE.get(D);
}
function buildAssembly(D, buf, { edges = true } = {}) {
  const G = geometries(D, buf), mat = matFactory(D.materials);
  const M = { D, nodes: [], byId: {}, objs: [], root: null, holo: HOLO(), style: 'cad' };
  D.nodes.forEach((n, i) => {
    const g = new THREE.Group(); g.name = n.id;
    g.position.set(...n.pos); g.quaternion.set(...n.q);
    if (n.mirror) g.scale.set(1, 1, -1);
    const rec = { def: n, group: g, q0: g.quaternion.clone(), p0: g.position.clone(), i };
    M.nodes.push(rec); M.byId[n.id] = rec;
  });
  M.nodes.forEach((r) => { if (r.def.parent >= 0) M.nodes[r.def.parent].group.add(r.group); else M.root = r.group; });
  const groups = new Map();
  D.instances.forEach((ins, gi) => { const k = ins[0] + '|' + ins[1]; if (!groups.has(k)) groups.set(k, []); groups.get(k).push(gi); });
  const vv = new THREE.Vector3(), qq = new THREE.Quaternion(), one = new THREE.Vector3(1, 1, 1);
  for (const [key, list] of groups) {
    const [pi, ni] = key.split('|').map(Number);
    const P = D.parts[pi], parent = M.nodes[ni].group, base = mat(P.material);
    const mtx = list.map((gi) => { const s = D.instances[gi]; return new THREE.Matrix4().compose(vv.set(s[2], s[3], s[4]), qq.set(s[5], s[6], s[7], s[8]), one); });
    let obj, line = null;
    if (list.length >= 3) {
      obj = new THREE.InstancedMesh(G[pi].geo, base, list.length);
      mtx.forEach((m, k) => obj.setMatrixAt(k, m));
      obj.instanceMatrix.needsUpdate = true; obj.computeBoundingSphere();
      const src = G[pi].edges.attributes.position.array;
      if (edges && src.length * list.length < 600000) {
        const out = new Float32Array(src.length * list.length);
        mtx.forEach((m, k) => { for (let i = 0; i < src.length; i += 3) { vv.set(src[i], src[i + 1], src[i + 2]).applyMatrix4(m); out[k * src.length + i] = vv.x; out[k * src.length + i + 1] = vv.y; out[k * src.length + i + 2] = vv.z; } });
        const lg = new THREE.BufferGeometry(); lg.setAttribute('position', new THREE.BufferAttribute(out, 3));
        line = new THREE.LineSegments(lg, cadLine);
      }
    } else {
      obj = new THREE.Group();
      mtx.forEach((m) => {
        const me = new THREE.Mesh(G[pi].geo, base); me.matrixAutoUpdate = false; me.matrix.copy(m);
        obj.add(me);
        if (edges) { const li = new THREE.LineSegments(G[pi].edges, cadLine); li.matrixAutoUpdate = false; li.matrix.copy(m); obj.add(li); }
      });
    }
    parent.add(obj); if (line) parent.add(line);
    const rec = { obj, line, pi, ni, inst: list, mtx, base, sys: P.system };
    obj.userData.rec = rec;
    if (!obj.isInstancedMesh) obj.children.forEach((c, k) => { c.userData.rec = rec; c.userData.k = c.isMesh ? (edges ? k / 2 : k) : -1; });
    M.objs.push(rec);
  }
  return M;
}
function setStyle(M, style, hot = null) {
  M.style = style;
  for (const r of M.objs) {
    const isHot = hot && hot(r);
    const m = style === 'holo' ? (isHot ? M.holo.meshHot : r.inst.length > 60 ? M.holo.dense : M.holo.mesh) : r.base;   // dense arrays would add up to white
    if (r.obj.isInstancedMesh) r.obj.material = m; else r.obj.children.forEach((c) => { if (c.isMesh) c.material = m; else c.material = style === 'holo' ? M.holo.line : cadLine; });
    if (r.line) { r.line.material = style === 'holo' ? M.holo.line : cadLine; r.line.visible = !(style === 'holo' && r.inst.length > 60); }
  }
}
// pose: deploy fractions per anim kind (1 = deployed, the CAD rest pose); explode 0–1
function poseSat(M, st) {
  for (const n of M.nodes) {
    const a = n.def.anim; if (!a || a.type !== 'rot') continue;
    const ang = a.kind === 'gimbal' ? rad(st.gimbalAng || 0) : rad(a.stow * (1 - (st[a.kind] ?? 1)));
    n.group.quaternion.copy(n.q0).multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(...a.axis), ang));
  }
  const e = st.explode || 0;
  const NODE_EX = { ant_c: [0, -.45, 0], w2_px: [.35, -.45, 0], w2_mx: [-.35, -.45, 0], sol1: [0, .2, -.55], avionics: [0, .0, 0], comms: [0, 0, 0] };
  for (const [id, v] of Object.entries(NODE_EX)) { const n = M.byId[id]; if (n) n.group.position.copy(n.p0).add(new THREE.Vector3(...v).multiplyScalar(e)); }
  for (const r of M.objs) {
    const nid = M.nodes[r.ni].def.id;
    if (!['bus', 'avionics', 'adcs', 'prop', 'comms', 'payload'].includes(nid)) continue;
    if (!r.dir) { const c = new THREE.Vector3(); r.mtx.forEach((m) => c.add(new THREE.Vector3().setFromMatrixPosition(m))); c.divideScalar(r.mtx.length); r.dir = c.lengthSq() > 1e-6 ? c.clone().normalize() : new THREE.Vector3(0, 1, 0); r.c0 = c; }
    const k = nid === 'bus' ? .25 : .55;
    r.obj.position.copy(r.dir).multiplyScalar(k * e); if (r.line) r.line.position.copy(r.obj.position);
  }
}
// mass properties in the current pose: point masses at each instance's centre of mass, plus each part's own box inertia
function massProps(M, extra = []) {
  M.root.updateMatrixWorld(true);
  const D = M.D, pts = [], v = new THREE.Vector3(), mw = new THREE.Matrix4();
  for (const r of M.objs) {
    const p = D.parts[r.pi], parentW = r.obj.parent.matrixWorld;
    r.mtx.forEach((m) => {
      mw.multiplyMatrices(parentW, new THREE.Matrix4().makeTranslation(r.obj.position.x, r.obj.position.y, r.obj.position.z)).multiply(m);
      v.set(...p.com).applyMatrix4(mw);
      const d = p.dims_mm.map((x) => x / 1000);
      pts.push({ m: p.mass_kg, x: v.x, y: v.y, z: v.z, I: [(d[1] ** 2 + d[2] ** 2) / 12 * p.mass_kg, (d[0] ** 2 + d[2] ** 2) / 12 * p.mass_kg, (d[0] ** 2 + d[1] ** 2) / 12 * p.mass_kg], sys: p.system, pn: p.pn });
    });
  }
  pts.push(...extra);
  const m = pts.reduce((a, b) => a + b.m, 0);
  const cg = { x: pts.reduce((a, b) => a + b.m * b.x, 0) / m, y: pts.reduce((a, b) => a + b.m * b.y, 0) / m, z: pts.reduce((a, b) => a + b.m * b.z, 0) / m };
  const I = [0, 0, 0];
  for (const p of pts) {
    const dx = p.x - cg.x, dy = p.y - cg.y, dz = p.z - cg.z, Ib = p.I || [0, 0, 0];
    I[0] += p.m * (dy * dy + dz * dz) + Ib[0]; I[1] += p.m * (dx * dx + dz * dz) + Ib[1]; I[2] += p.m * (dx * dx + dy * dy) + Ib[2];
  }
  const box = new THREE.Box3();
  for (const p of pts) box.expandByPoint(new THREE.Vector3(p.x, p.y, p.z));
  const bySys = {}; for (const p of pts) bySys[p.sys || 'Propellant'] = (bySys[p.sys || 'Propellant'] || 0) + p.m;
  return { m, cg, I, bySys, pts };
}
function envelope(M) {    // bounding box of every instance's part box, current pose
  M.root.updateMatrixWorld(true);
  const box = new THREE.Box3(), b = new THREE.Box3(), mw = new THREE.Matrix4(), G = geometries(M.D);
  for (const r of M.objs) {
    if (!r.obj.visible) continue;
    r.mtx.forEach((m) => { mw.multiplyMatrices(r.obj.parent.matrixWorld, m); b.copy(G[r.pi].geo.boundingBox).applyMatrix4(mw); box.union(b); });
  }
  return box;
}

// ------------------------------------------------------------------ 01 mission sequence
const SEQ_T = 60;
const STEPS = [
  [0, 'Scan-in', 'Every CADed part resolves, stowed for launch as it rides on the rideshare.'],
  [6, 'Separation', 'The 15-inch separation ring fires and springs push Tideline away at about 0.5 m/s.'],
  [12, 'Detumble', 'Magnetorquers and the reaction wheels kill the tip-off rates; star trackers lock.'],
  [17, 'Solar wing', 'Two SMA hold-downs release and the two-panel wing swings up and latches.'],
  [24, 'SAR array', 'The four wing panels unfold on spring hinges with eddy-current dampers and latch flat to 0.5 mm.'],
  [34, 'Commission', 'AIS whips deploy; the array is calibrated panel by panel against a ground transponder.'],
  [38, 'Slew', 'The wheels roll the spacecraft 30° so the beam points off nadir at the target port.'],
  [42, 'Image the port', 'Stripmap pass: 680 T/R modules transmit a 100 MHz chirp ~5,000 times a second; the echoes are recorded.'],
  [52, 'Detect & relay', 'On-board processing finds the ships, matches AIS and sends a short ship list over the L-band relay.'],
  [56, 'Downlink', 'At the next ground station the full image goes down at 800 Mbit/s over the gimballed X-band horn.'],
];
const seq = { t: 0, playing: false, speed: 1, style: 'holo', camAuto: true };
let MS;   // mission scene objects
function seqState(t) {
  return {
    sol1: ramp(t, 18, 21.5), sol2: ramp(t, 20.5, 24), wing2: ramp(t, 24.5, 29), wing3: ramp(t, 28.5, 33.5), whip: ramp(t, 34, 36),
    roll: -30 * ramp(t, 38.5, 41.5) * (1 - ramp(t, 55, 57.5)), sep: ramp(t, 6, 12), tumble: (1 - ramp(t, 12, 17)) * ramp(t, 7, 9),
    beam: t > 42 && t < 52 ? 1 : 0, relay: t > 52 && t < 56 ? 1 : 0, downlink: t > 56.3 ? 1 : 0, gimbalAng: -50 * ramp(t, 55.5, 57),
  };
}
function initMission() {
  const scene = new THREE.Scene(); lightScene(scene);
  const v = makeView('mission', 'seq-viewport', [6.5, 3.2, 8.5], [0, -1.2, 0], scene, 0.05, 600);
  $('seq-loading').remove();
  const M = buildAssembly(SAT.data, SAT.buf, { edges: true });
  const holder = new THREE.Group(); holder.add(M.root); scene.add(holder);
  // holographic Earth below: lat/long grid shader on a big sphere
  const R = 60, earthC = new THREE.Vector3(0, -R - 14, 0);
  const earthMat = new THREE.ShaderMaterial({
    transparent: true, depthWrite: false, uniforms: { uRot: { value: 0 } },
    vertexShader: 'varying vec3 vP; varying vec3 vN; varying vec3 vW; void main(){ vP = position; vN = normalize(normalMatrix*normal); vec4 w = modelMatrix*vec4(position,1.); vW = w.xyz; gl_Position = projectionMatrix*viewMatrix*w; }',
    fragmentShader: `uniform float uRot; varying vec3 vP; varying vec3 vN; varying vec3 vW;
      float line(float v, float s){ float g = abs(fract(v/s-.5)-.5)/fwidth(v/s); return 1.-min(g,1.); }
      void main(){ vec3 p = normalize(vP); float lat = degrees(asin(p.y)); float lon = degrees(atan(p.z, p.x));
        float a = line(lat, 10.)*.35 + line(lon, 10.)*.35;
        float land = smoothstep(.15,.35, sin(p.x*9.)*sin(p.z*7.+1.3)*cos(p.y*5.) );
        float rim = pow(1.-abs(dot(normalize(vN), vec3(0.,0.,1.))), 2.);
        gl_FragColor = vec4(mix(vec3(.18,.62,.72), vec3(.42,.86,.92), land), a*.55 + land*.07 + rim*.22 + .03); }`,
  });
  const earth = new THREE.Mesh(new THREE.SphereGeometry(R, 128, 96), earthMat); earth.position.copy(earthC); scene.add(earth);
  // port on the globe: quays and ships, arranged so the footprint sweeps over them during imaging
  const port = new THREE.Group(); earth.add(port);
  // put the port where the beam lands mid-pass (T+47): roll the spacecraft, cast the boresight onto the sphere
  const probe = new THREE.Object3D(); probe.rotateX(rad(-30)); probe.updateMatrixWorld(true);
  const o0 = new THREE.Vector3(0, -.46, 0).applyMatrix4(probe.matrixWorld), d0 = new THREE.Vector3(0, -1, 0).transformDirection(probe.matrixWorld);
  const h0 = raySphere(o0, d0, earthC, R) || new THREE.Vector3(0, -14, 0);
  const portDir = h0.clone().sub(earthC).normalize();   // earth rotation is zero at T+47
  const shipMat = new THREE.MeshBasicMaterial({ color: 0xbff6ff }), detMat = new THREE.MeshBasicMaterial({ color: 0xffb35c, transparent: true, opacity: .9, side: THREE.DoubleSide });
  const ships = [], rng = mulberry(7);
  for (let k = 0; k < 42; k++) {
    const u = (rng() - .5) * 7, w = (rng() - .5) * 3.6, len = .25 + rng() * .55;
    const s = new THREE.Mesh(new THREE.BoxGeometry(len, .03, len * .18), shipMat);
    const ring = new THREE.Mesh(new THREE.RingGeometry(len * .9, len * .9 + .035, 24), detMat); ring.rotation.x = -Math.PI / 2; ring.visible = false; s.add(ring);
    s.userData = { u, w, ring, det: false };
    ships.push(s); port.add(s);
  }
  const quayMat = new THREE.LineBasicMaterial({ color: 0x6be1ed, transparent: true, opacity: .8 });
  const quays = new THREE.Group(); port.add(quays);
  // place port objects on the sphere around portDir
  const tA = new THREE.Vector3(1, 0, 0), tB = new THREE.Vector3().crossVectors(portDir, tA).normalize();
  const onSphere = (u, w, h = .02) => portDir.clone().multiplyScalar(R + h).addScaledVector(tA, u).addScaledVector(tB, w).normalize().multiplyScalar(R + h);
  ships.forEach((s) => { s.position.copy(onSphere(s.userData.u, s.userData.w)); s.lookAt(s.position.clone().multiplyScalar(2)); s.rotateX(-Math.PI / 2); s.rotateY(rng() * Math.PI); });
  for (let k = 0; k < 6; k++) { const pts = []; for (let i = 0; i <= 10; i++) pts.push(onSphere(-2.5 + k * .9, 1.6 - i * .16, .01)); quays.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), quayMat)); }
  // beam: a thin fan (narrow in azimuth, wide in elevation) from the array to the ground, and its footprint
  const beamMat = new THREE.MeshBasicMaterial({ color: 0x6be1ed, transparent: true, opacity: .16, blending: THREE.AdditiveBlending, depthWrite: false, side: THREE.DoubleSide });
  const beam = new THREE.Mesh(new THREE.BufferGeometry(), beamMat); scene.add(beam);
  const foot = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), new THREE.MeshBasicMaterial({ color: 0x9ff3ff, transparent: true, opacity: .35, blending: THREE.AdditiveBlending, depthWrite: false, side: THREE.DoubleSide }));
  scene.add(foot);
  const linkMat = new THREE.LineDashedMaterial({ color: 0xffc27a, dashSize: .4, gapSize: .25, transparent: true, opacity: .9 });
  const link = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3(0, 1, 0)]), linkMat); scene.add(link);
  const station = new THREE.Mesh(new THREE.ConeGeometry(.25, .5, 16), new THREE.MeshBasicMaterial({ color: 0xffc27a })); earth.add(station);
  const stDir = new THREE.Vector3(Math.sin(rad(-30)), Math.cos(rad(-30)), 0).normalize();
  station.position.copy(stDir.clone().multiplyScalar(R + .25)); station.lookAt(stDir.clone().multiplyScalar(2 * R)); station.rotateX(Math.PI / 2);
  const relay = new THREE.Mesh(new THREE.OctahedronGeometry(.25), new THREE.MeshBasicMaterial({ color: 0xffc27a })); relay.position.set(-6, 9, -4); scene.add(relay);
  // dispenser ring for separation
  const ring = new THREE.Mesh(new THREE.TorusGeometry(.22, .02, 8, 48), new THREE.MeshBasicMaterial({ color: 0x6be1ed, transparent: true, opacity: .7 }));
  ring.rotation.x = Math.PI / 2; scene.add(ring);
  // when does the footprint reach each ship? The beam's ground point is fixed during imaging (constant roll)
  // while the Earth turns 0.9°/s under it, so sample the pass once and remember the first time each ship is lit
  earth.updateMatrixWorld(true);
  for (const s of ships) {
    s.userData.tDet = Infinity;
    for (let t = 42; t <= 52; t += .05) {
      const wp = s.position.clone().applyAxisAngle(new THREE.Vector3(0, 0, 1), rad((t - 47) * .9)).add(earthC);
      if (wp.distanceTo(h0) < 2.6) { s.userData.tDet = t; break; }
    }
  }
  MS = { hit47: h0.clone(), M, holder, earth, earthMat, port, ships, beam, foot, link, station, relay, ring, R, earthC, portDir, stats: { det: 0 } };
  setStyle(M, 'holo');
  $('seq-steps').innerHTML = STEPS.map(([t, n, d], i) => `<li><button data-step="${i}"><span>T+${String(t).padStart(2, '0')}</span><strong>${esc(n)}</strong><small>${esc(d)}</small></button></li>`).join('');
  $('seq-steps').querySelectorAll('button').forEach((b) => b.addEventListener('click', () => { seq.playing = false; seq.camAuto = true; syncPlay(); setSeqTime(STEPS[+b.dataset.step][0] + .01); }));
  $('seq-play').addEventListener('click', () => { if (seq.t >= SEQ_T - .05) seq.t = 0; seq.playing = !seq.playing; seq.camAuto = true; syncPlay(); });
  $('seq-scrub').addEventListener('input', (e) => { seq.playing = false; syncPlay(); setSeqTime(+e.target.value); });
  $('seq-speed').addEventListener('change', (e) => (seq.speed = +e.target.value));
  document.querySelectorAll('[data-seqstyle]').forEach((b) => b.addEventListener('click', () => {
    seq.style = b.dataset.seqstyle; document.querySelectorAll('[data-seqstyle]').forEach((x) => x.classList.toggle('active', x === b));
    setStyle(M, seq.style); $('seq-viewport').classList.toggle('holo', seq.style === 'holo'); setSeqTime(seq.t);
  }));
  // camera keyframes per step; dragging the view hands control to the user until a step is clicked or Play pressed
  v.el.addEventListener('pointerdown', () => (seq.camAuto = false));
  v.tick = (dt) => {
    if (seq.camAuto) {
      let K = seq.t < 17 ? [[4.2, 2.2, 5.6], [0, 0, 0]] : seq.t < 38 ? [[6.5, 3.2, 8.5], [0, -.6, 0]] : [[9, 6, 16], [-2, -3, -2]];
      if (seq.t >= 38 && seq.t < 52 && MS.hit47) { const h = MS.hit47, tg = h.clone().multiplyScalar(.55); K = [[tg.x + 22, tg.y + 3, tg.z + 19], [tg.x, tg.y, tg.z]]; }   // frame the beam and the port
      const k = 1 - Math.pow(.25, dt);
      v.camera.position.lerp(new THREE.Vector3(...K[0]), k); v.controls.target.lerp(new THREE.Vector3(...K[1]), k);
    }
    if (seq.playing) { seq.t += dt * seq.speed; if (seq.t >= SEQ_T) { seq.t = SEQ_T; seq.playing = false; syncPlay(); } setSeqTime(seq.t); } };
  setSeqTime(0);
  seq.playing = true; syncPlay();
}
function syncPlay() { $('seq-play').textContent = seq.playing ? '❚❚ Pause' : '▶ Play'; }
function mulberry(a) { return () => { a |= 0; a = a + 0x6D2B79F5 | 0; let t = Math.imul(a ^ a >>> 15, 1 | a); t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t; return ((t ^ t >>> 14) >>> 0) / 4294967296; }; }
function setSeqTime(t) {
  seq.t = clamp(t, 0, SEQ_T);
  const S = MS, st = seqState(seq.t);
  poseSat(S.M, st);
  // reveal sweep during scan-in
  const rev = seq.t < 6 ? -2.4 + seq.t / 6 * 5 : 99;
  for (const m of holoMats) m.uniforms.uReveal.value = rev;
  // separation: drift up from the dispenser ring, tumble then settle; roll for imaging
  // the dispenser ring stays with the launcher and recedes
  S.ring.visible = seq.t < 12.5; S.ring.position.set(0, .43 + 4 * st.sep, 0); S.ring.material.opacity = .7 * (1 - st.sep);
  const tum = st.tumble;
  S.holder.rotation.set(rad(25 * tum * Math.sin(seq.t * 1.7)), rad(40 * tum * Math.sin(seq.t * 1.1)), rad(18 * tum * Math.cos(seq.t * 1.3)));
  S.holder.rotateX(rad(st.roll));
  // Earth turns under the spacecraft (along-track = +X)
  S.earth.rotation.z = rad((seq.t - 47) * .9);
  S.holder.updateMatrixWorld(true);
  // beam fan from the array centre along its boresight (-Y body) to the sphere
  const origin = new THREE.Vector3(0, -.46, 0).applyMatrix4(S.holder.matrixWorld);
  const bore = new THREE.Vector3(0, -1, 0).transformDirection(S.holder.matrixWorld).normalize();
  const hit = raySphere(origin, bore, S.earthC, S.R);
  S.beam.visible = S.foot.visible = !!(st.beam && hit);
  if (S.beam.visible) {
    const along = new THREE.Vector3(1, 0, 0).transformDirection(S.holder.matrixWorld).normalize(), side = new THREE.Vector3().crossVectors(bore, along).normalize();
    const dist = origin.distanceTo(hit), wEl = dist * Math.tan(rad(1.6)), wAz = dist * Math.tan(rad(.25));
    const c = [hit.clone().addScaledVector(side, -wEl).addScaledVector(along, -wAz), hit.clone().addScaledVector(side, wEl).addScaledVector(along, -wAz),
      hit.clone().addScaledVector(side, wEl).addScaledVector(along, wAz), hit.clone().addScaledVector(side, -wEl).addScaledVector(along, wAz)];
    const pos = [origin, c[0], c[1], origin, c[1], c[2], origin, c[2], c[3], origin, c[3], c[0]].flatMap((p) => [p.x, p.y, p.z]);
    S.beam.geometry.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3)); S.beam.geometry.computeBoundingSphere();
    S.foot.position.copy(hit); S.foot.lookAt(hit.clone().add(hit.clone().sub(S.earthC))); S.foot.scale.set(wAz * 2 + .25, wEl * 2 * 1.2, 1);
    S.foot.material.opacity = .25 + .15 * Math.sin(performance.now() / 120);
  }
  // ships: detected once the footprint has swept over them
  let det = 0;
  S.port.updateMatrixWorld(true);
  for (const s of S.ships) {
    s.userData.det = seq.t >= s.userData.tDet;
    s.userData.ring.visible = s.userData.det;
    if (s.userData.det) det++;
  }
  S.stats.det = det;
  // relay and downlink lines
  const lp = [];
  if (st.relay) lp.push(new THREE.Vector3(0, .5, 0).applyMatrix4(S.holder.matrixWorld), S.relay.position.clone());
  else if (st.downlink) lp.push(new THREE.Vector3(.18, -.1, .5).applyMatrix4(S.holder.matrixWorld), S.station.getWorldPosition(new THREE.Vector3()));
  S.link.visible = lp.length > 0; S.relay.visible = seq.t > 50;
  if (lp.length) { S.link.geometry.setFromPoints(lp); S.link.computeLineDistances(); S.link.material.dashOffset = -performance.now() / 300; }
  // UI
  const si = STEPS.reduce((a, s, i) => (seq.t >= s[0] ? i : a), 0);
  $('seq-step-code').textContent = `T+${seq.t.toFixed(1).padStart(4, '0')}`;
  $('seq-step-title').textContent = STEPS[si][1];
  $('seq-step-sub').textContent = STEPS[si][2];
  $('seq-steps').querySelectorAll('button').forEach((b, i) => { b.classList.toggle('active', i === si); b.classList.toggle('done', i < si); });
  $('seq-scrub').value = seq.t; $('seq-time').textContent = `${seq.t.toFixed(1).padStart(4, '0')} / ${SEQ_T.toFixed(1)} s`;
  const P = api.model(), soc = 100 - (st.beam ? (seq.t - 42) * .9 : seq.t > 52 ? 9 : 0) + (seq.t > 56 ? (seq.t - 56) * .4 : 0);
  const solarW = seq.t < 18 ? 0 : P.solarSunW * (st.sol1 * .34 + st.sol2 * .33 + .33) * (seq.t > 38 ? Math.cos(rad(Math.abs(st.roll))) : 1);
  $('seq-telemetry').innerHTML = [
    ['ALTITUDE', `${fmt(P.F.h, 0)} km`], ['ATTITUDE', seq.t < 12 ? 'STOWED' : seq.t < 17 ? 'DETUMBLE' : seq.t < 38.5 ? 'SUN-POINTING' : seq.t < 55 ? `ROLL ${fmt(Math.abs(st.roll), 0)}°` : 'DOWNLINK'],
    ['SOLAR', `${fmt(solarW, 0)} W`], ['BATTERY', `${fmt(soc, 0)} %`, soc < 92],
    ['SAR ARRAY', st.wing3 >= 1 ? 'DEPLOYED' : st.wing2 > 0 ? `${fmt((st.wing2 + st.wing3) * 50, 0)} %` : 'STOWED'], ['RADAR', st.beam ? `TX ${fmt(P.strip.prf, 0)} Hz` : 'IDLE', st.beam],
    ['RECORDED', `${fmt(st.beam ? (seq.t - 42) * P.strip.rateGbps : seq.t > 52 ? 10 * P.strip.rateGbps : 0, 1)} Gbit`], ['SHIPS FOUND', `${seq.t >= 42 ? det : 0}`, det > 0 && seq.t >= 42],
  ].map(([k, v, hot]) => `<div><small>${k}</small><b class="${hot ? 'hot' : ''}">${v}</b></div>`).join('');
  const al = $('seq-alert');
  al.hidden = seq.t < 52;
  if (!al.hidden) al.innerHTML = `<span class="eyebrow">SHIP LIST · VIA L-BAND RELAY</span><b>${det} vessels detected</b><span>${Math.round(det * .14)} without AIS (dark) · anchorage queue ${Math.round(det * .3)} · channel clear</span><small>${seq.t < 56 ? 'sent ' + fmt((seq.t - 52) * 60, 0) + ' s after imaging' : 'image downlinking over X-band'}</small>`;
}
function raySphere(o, d, c, R) {
  const oc = o.clone().sub(c), b = oc.dot(d), cc = oc.lengthSq() - R * R, h = b * b - cc;
  if (h < 0) return null;
  const t = -b - Math.sqrt(h);
  return t > 0 ? o.clone().addScaledVector(d, t) : null;
}

// ------------------------------------------------------------------ 02 advanced blueprint
const adv = { config: 1, explode: 0, holo: false, sel: -1, isolate: false, hidden: new Set(), overlays: [], view: '3d', sort: ['tot', -1] };
let BP;
function initBlueprint() {
  const scene = new THREE.Scene(); lightScene(scene);
  const v = makeView('blueprint', 'adv-viewport', [5.2, 2.6, 5.6], [0, -.1, 0], scene, 0.02, 200);
  $('adv-loading').hidden = true;
  const grid = new THREE.GridHelper(8, 32, 0x2f7f8d, 0x14353f); grid.material.transparent = true; grid.material.opacity = .4; grid.position.y = -1.3; scene.add(grid);
  const M = buildAssembly(SAT.data, SAT.buf); scene.add(M.root); BP = { M };
  $('adv-count').textContent = `${SAT.data.parts.length} PARTS · ${SAT.data.instances.length.toLocaleString()} PLACED`;
  buildTree();
  $('adv-search').addEventListener('input', () => { buildTree(); if (adv.view === 'bom') renderBom(); });
  $('adv-filter').addEventListener('change', () => { buildTree(); if (adv.view === 'bom') renderBom(); });
  $('adv-config').addEventListener('input', (e) => { adv.config = +e.target.value; applyBP(); });
  $('adv-explode').addEventListener('input', (e) => { adv.explode = +e.target.value; applyBP(); });
  $('adv-section').addEventListener('change', (e) => { renderer.clippingPlanes = e.target.checked ? [new THREE.Plane(new THREE.Vector3(0, 0, -1), 0)] : []; });
  $('adv-holo').addEventListener('change', (e) => { adv.holo = e.target.checked; refreshIsolation(); });
  $('adv-reset').addEventListener('click', () => {
    adv.config = 1; adv.explode = 0; $('adv-config').value = 1; $('adv-explode').value = 0; $('adv-section').checked = false; renderer.clippingPlanes = [];
    adv.isolate = false; refreshIsolation(); applyBP(); v.camera.position.set(5.2, 2.6, 5.6); v.controls.target.set(0, -.1, 0);
  });
  document.querySelectorAll('[data-advview]').forEach((b) => b.addEventListener('click', () => setAdvView(b.dataset.advview)));
  const ray = new THREE.Raycaster(), ndc = new THREE.Vector2(); let down = null;
  v.el.addEventListener('pointerdown', (e) => (down = [e.clientX, e.clientY]));
  v.el.addEventListener('pointerup', (e) => {
    if (!down || Math.hypot(e.clientX - down[0], e.clientY - down[1]) > 5) return;
    const r = v.el.getBoundingClientRect();
    ndc.set((e.clientX - r.left) / r.width * 2 - 1, -(e.clientY - r.top) / r.height * 2 + 1);
    ray.setFromCamera(ndc, v.camera);
    const targets = [];
    for (const rec of M.objs) { if (!rec.obj.visible) continue; if (rec.obj.isInstancedMesh) targets.push(rec.obj); else rec.obj.children.forEach((c) => c.isMesh && targets.push(c)); }
    const hits = ray.intersectObjects(targets, false).filter((hh) => !renderer.clippingPlanes.length || hh.point.z <= 0.001);
    if (!hits.length) return;
    selectPart(hits[0].object.userData.rec.pi, -1);
  });
  applyBP();
}
function bpState() { const c = adv.config; return { sol1: ramp(c, 0, .35), sol2: ramp(c, .25, .5), wing2: ramp(c, .4, .75), wing3: ramp(c, .65, 1), whip: ramp(c, .85, 1), explode: adv.explode }; }
function applyBP() {
  poseSat(BP.M, bpState());
  $('adv-config-label').textContent = adv.config < .02 ? 'Stowed for launch' : adv.config > .98 ? 'Deployed' : 'Deploying';
  if (adv.view === 'mass') renderMass();
}
function partMatches(p, q, f) {
  if (f === 'crit' && !p.criticality) return false;
  if (f === 'make' && p.supplier !== 'make') return false;
  if (f === 'buy' && p.supplier !== 'buy') return false;
  if (!q) return true;
  return (p.pn + ' ' + p.name + ' ' + p.system + ' ' + p.assy + ' ' + SAT.data.materials[p.material].name + ' ' + p.fn).toLowerCase().includes(q);
}
function buildTree() {
  const D = SAT.data, q = $('adv-search').value.trim().toLowerCase(), f = $('adv-filter').value, sys = {};
  D.parts.forEach((p, i) => { if (!partMatches(p, q, f)) return; ((sys[p.system] ||= {})[p.assy] ||= []).push(i); });
  const open = !!(q || f);
  $('adv-tree').innerHTML = Object.keys(sys).sort().map((s) => {
    const assys = sys[s], all = Object.values(assys).flat(), mass = all.reduce((a, i) => a + D.parts[i].mass_kg * D.parts[i].qty, 0);
    return `<details ${open ? 'open' : ''}><summary><span class="nm">${esc(s)}</span><em>${all.length} · ${fmt(mass, 1)} kg</em><input type="checkbox" class="sysvis" data-sys="${esc(s)}" ${adv.hidden.has(s) ? '' : 'checked'} aria-label="Show ${esc(s)}"></summary>` +
      Object.keys(assys).sort().map((a) => `<details ${open ? 'open' : ''}><summary><span class="nm">${esc(a)}</span><em>${assys[a].length}</em></summary>` +
        assys[a].map((i) => { const p = D.parts[i]; return `<button class="p ${i === adv.sel ? 'sel' : ''}" data-pi="${i}"><span><b>${p.pn}</b>${esc(p.name)}</span><i>×${p.qty}</i></button>`; }).join('') + '</details>').join('') + '</details>';
  }).join('') || '<p class="subtle">No parts match.</p>';
  $('adv-tree').querySelectorAll('.p').forEach((b) => b.addEventListener('click', () => { selectPart(+b.dataset.pi, -1, true); if (adv.view !== '3d') setAdvView('3d'); }));
  $('adv-tree').querySelectorAll('.sysvis').forEach((c) => {
    c.addEventListener('click', (e) => e.stopPropagation());
    c.addEventListener('change', () => { if (c.checked) adv.hidden.delete(c.dataset.sys); else adv.hidden.add(c.dataset.sys); refreshIsolation(); });
  });
}
function clearOverlays() { adv.overlays.forEach((o) => o.parent && o.parent.remove(o)); adv.overlays = []; }
function selectPart(pi, gInst = -1, frame = false) {
  adv.sel = pi; clearOverlays();
  const M = BP.M, G = geometries(SAT.data);
  for (const rec of M.objs) {
    if (rec.pi !== pi) continue;
    rec.mtx.forEach((m) => {
      const me = new THREE.Mesh(G[pi].geo, highlightMat); me.matrixAutoUpdate = false; me.matrix.copy(m); me.renderOrder = 10;
      const li = new THREE.LineSegments(G[pi].edges, highlightLine); li.matrixAutoUpdate = false; li.matrix.copy(m); li.renderOrder = 11;
      const holder = new THREE.Group(); holder.position.copy(rec.obj.position); holder.add(me, li);
      M.nodes[rec.ni].group.add(holder); adv.overlays.push(holder);
    });
  }
  document.querySelectorAll('#adv-tree .p').forEach((b) => b.classList.toggle('sel', +b.dataset.pi === pi));
  const btn = document.querySelector(`#adv-tree .p[data-pi="${pi}"]`);
  if (btn) { btn.closest('details')?.setAttribute('open', ''); btn.closest('details')?.parentElement.closest('details')?.setAttribute('open', ''); btn.scrollIntoView({ block: 'nearest' }); }
  renderSheet(pi); refreshIsolation();
  if (frame) frameOverlays();
}
function frameOverlays() {
  const box = new THREE.Box3(); BP.M.root.updateMatrixWorld(true);
  adv.overlays.forEach((h) => h.children.forEach((o) => { if (o.isMesh) box.union(o.geometry.boundingBox.clone().applyMatrix4(o.matrixWorld)); }));
  if (box.isEmpty()) return;
  const v = views.blueprint, c = box.getCenter(new THREE.Vector3()), r = Math.max(.08, box.getSize(new THREE.Vector3()).length() * .5);
  const dir = v.camera.position.clone().sub(v.controls.target).normalize();
  v.controls.target.copy(c); v.camera.position.copy(c).addScaledVector(dir, r / Math.tan(rad(18)) * 1.2);
}
function refreshIsolation() {
  const M = BP.M, iso = adv.isolate && adv.sel >= 0;
  setStyle(M, adv.holo ? 'holo' : 'cad');
  for (const rec of M.objs) {
    const hide = adv.hidden.has(rec.sys);
    rec.obj.visible = !hide; if (rec.line) rec.line.visible = !hide && !iso;
    if (iso && rec.pi !== adv.sel) { if (rec.obj.isInstancedMesh) rec.obj.material = ghostMat; else rec.obj.children.forEach((c) => { if (c.isMesh) c.material = ghostMat; else c.visible = false; }); }
    else if (!rec.obj.isInstancedMesh) rec.obj.children.forEach((c) => (c.visible = true));
  }
}
function renderSheet(pi) {
  const D = SAT.data, p = D.parts[pi], m = D.materials[p.material];
  const crit = p.criticality ? `<span class="chip crit">${esc(p.criticality.toUpperCase())}</span>` : '';
  const spec = Object.entries(p.spec || {}).map(([k, v]) => `<dt>${esc(k.replace(/_/g, ' ').toUpperCase())}</dt><dd>${esc(v)}</dd>`).join('');
  const where = [...new Set(D.instances.filter((s) => s[0] === pi).map((s) => D.nodes[s[1]].name))].slice(0, 6);
  $('adv-sheet').innerHTML = `
    <div><span class="sheet-pn">${p.pn}</span><h3>${esc(p.name)}</h3>
      <div class="chips"><span class="chip">${esc(p.system)}</span><span class="chip">${esc(p.assy)}</span><span class="chip ${p.supplier === 'buy' ? 'buy' : ''}">${p.supplier === 'buy' ? 'BUY' : 'MAKE'}</span>${crit}</div></div>
    <div class="drawing">${drawing(pi)}</div>
    <p>${esc(p.fn || p.desc || '')}</p>
    <dl class="kv">
      <dt>QUANTITY</dt><dd>${p.qty} per spacecraft</dd>
      <dt>MATERIAL</dt><dd>${esc(m.name)}</dd>
      <dt>PROCESS</dt><dd>${esc(p.process || '—')}</dd>
      <dt>ENVELOPE</dt><dd>${p.dims_mm.map((d) => fmt(d, 1)).join(' × ')} mm</dd>
      <dt>UNIT MASS</dt><dd>${fmt(p.mass_kg, p.mass_kg < 1 ? 3 : 2)} kg · total ${fmt(p.mass_kg * p.qty, 2)} kg</dd>
      <dt>CAD BASIS</dt><dd>${esc(p.basis)} · ${p.faces} B-rep faces</dd>
      <dt>LOCATION</dt><dd>${where.map(esc).join(', ')}</dd>
      ${spec}
    </dl>
    <div class="sheet-actions">
      <button id="sheet-frame">Frame in view</button>
      <button id="sheet-iso">${adv.isolate ? 'Show whole spacecraft' : 'Isolate part'}</button>
      <a href="${REPO}cad/sat/step/${p.step}" target="_blank" rel="noopener">STEP ↗</a>
    </div>
    <p class="sheet-note">Mass from the B-rep volume and an effective material density, or the catalogue mass for purchased units. Concept geometry: no tolerances or qualified loads.</p>`;
  $('sheet-frame').onclick = () => { setAdvView('3d'); frameOverlays(); };
  $('sheet-iso').onclick = () => { adv.isolate = !adv.isolate; refreshIsolation(); renderSheet(pi); if (adv.isolate) frameOverlays(); };
}
function drawing(pi) {
  const G = geometries(SAT.data)[pi], pos = G.edges.attributes.position.array, bb = G.geo.boundingBox, sz = bb.getSize(new THREE.Vector3());
  const W = 320, Hh = 230, pad = 16;
  const views3 = [['FRONT', 0, 1, sz.x, sz.y], ['TOP', 0, 2, sz.x, sz.z], ['SIDE', 2, 1, sz.z, sz.y]];
  const cells = [[pad, pad, 190, 110], [pad, 134, 190, 80], [222, pad, 84, 110]];
  const nseg = pos.length / 6, stride = Math.max(1, Math.ceil(nseg / 5000));
  let out = `<svg viewBox="0 0 ${W} ${Hh}" role="img" aria-label="Orthographic drawing"><rect x="0.5" y="0.5" width="${W - 1}" height="${Hh - 1}" fill="none" stroke="#6be1ed30"/>`;
  views3.forEach(([name, a, b, wa, wb], vi) => {
    const [cx, cy, cw, ch] = cells[vi], s = Math.min(cw / Math.max(wa, 1e-6), ch / Math.max(wb, 1e-6)) * .82;
    const ox = cx + (cw - wa * s) / 2, oy = cy + (ch - wb * s) / 2, mn = [bb.min.x, bb.min.y, bb.min.z];
    let d = '';
    for (let i = 0; i < nseg; i += stride) {
      const k = i * 6;
      d += `M${(ox + (pos[k + a] - mn[a]) * s).toFixed(1)} ${(oy + wb * s - (pos[k + b] - mn[b]) * s).toFixed(1)}L${(ox + (pos[k + 3 + a] - mn[a]) * s).toFixed(1)} ${(oy + wb * s - (pos[k + 3 + b] - mn[b]) * s).toFixed(1)}`;
    }
    out += `<path d="${d}" fill="none" stroke="#6be1ed" stroke-width=".55" opacity=".9"/><text x="${cx}" y="${cy + ch + 9}" fill="#8faeb9" font-size="7" font-family="IBM Plex Mono, monospace">${name} · ${fmt(wa * 1000, 0)} × ${fmt(wb * 1000, 0)} mm</text>`;
  });
  return out + `<text x="${W - pad}" y="${Hh - 8}" text-anchor="end" fill="#6be1ed" font-size="7" font-family="IBM Plex Mono, monospace">${SAT.data.parts[pi].pn} · TIDELINE REV 01</text></svg>`;
}
function setAdvView(vw) {
  adv.view = vw;
  document.querySelectorAll('[data-advview]').forEach((b) => b.classList.toggle('active', b.dataset.advview === vw));
  $('adv-viewport').hidden = vw !== '3d'; $('adv-bom').hidden = vw !== 'bom'; $('adv-mass').hidden = vw !== 'mass';
  if (vw === 'bom') renderBom(); if (vw === 'mass') renderMass(); if (vw === '3d') resize();
}
function renderBom() {
  const D = SAT.data, q = $('adv-search').value.trim().toLowerCase(), f = $('adv-filter').value;
  const cols = [['pn', 'PART NO.'], ['name', 'NAME'], ['system', 'SYSTEM'], ['qty', 'QTY'], ['mat', 'MATERIAL'], ['supplier', 'MAKE / BUY'], ['mass_kg', 'UNIT kg'], ['tot', 'TOTAL kg']];
  let rows = D.parts.map((p, i) => ({ i, p, pn: p.pn, name: p.name, system: p.system, qty: p.qty, mat: D.materials[p.material].name, supplier: p.supplier, mass_kg: p.mass_kg, tot: p.mass_kg * p.qty })).filter((r) => partMatches(r.p, q, f));
  const [k, dir] = adv.sort;
  rows.sort((a, b) => (a[k] > b[k] ? 1 : a[k] < b[k] ? -1 : 0) * dir);
  const tm = rows.reduce((a, r) => a + r.tot, 0), tq = rows.reduce((a, r) => a + r.qty, 0);
  $('adv-bom').innerHTML = `<div class="bom-summary"><span><b>${rows.length}</b> unique parts</span><span><b>${tq.toLocaleString()}</b> pieces</span><span><b>${fmt(tm, 1)} kg</b> dry mass listed</span><span>STEP library: <a href="${REPO}cad/sat" target="_blank" rel="noopener">cad/sat ↗</a></span></div>
    <div style="overflow-x:auto"><table class="bom-table"><thead><tr>${cols.map(([c, l]) => `<th data-k="${c}" class="${c === k ? 'sorted' : ''}">${l}${c === k ? (dir > 0 ? ' ▲' : ' ▼') : ''}</th>`).join('')}</tr></thead><tbody>
    ${rows.map((r) => `<tr data-pi="${r.i}"><td class="pn">${r.pn}</td><td>${esc(r.name)}</td><td>${esc(r.system)}</td><td class="num">${r.qty}</td><td>${esc(r.mat)}</td><td>${r.supplier}</td><td class="num">${fmt(r.mass_kg, 3)}</td><td class="num">${fmt(r.tot, 2)}</td></tr>`).join('')}</tbody></table></div>`;
  $('adv-bom').querySelectorAll('th').forEach((th) => th.addEventListener('click', () => { const c = th.dataset.k; adv.sort = [c, adv.sort[0] === c ? -adv.sort[1] : 1]; renderBom(); }));
  $('adv-bom').querySelectorAll('tbody tr').forEach((tr) => tr.addEventListener('click', () => { setAdvView('3d'); selectPart(+tr.dataset.pi, -1, true); }));
}
function renderMass() {
  const P = api.model(), MP = P.mp, ST = P.mpStowed;
  const bys = Object.entries(MP.bySys).sort((a, b) => b[1] - a[1]), max = bys[0][1];
  $('adv-mass').innerHTML = `<div class="mass-grid"><div>
      <span class="eyebrow">MASS ROLL-UP FROM ${SAT.data.parts.length} CAD PARTS + PROPELLANT</span><h3 style="font-weight:500;margin:6px 0 12px">Dry ${fmt(MP.m - P.F.kr, 1)} kg · wet ${fmt(MP.m, 1)} kg with ${fmt(P.F.kr, 1)} kg of krypton</h3>
      <div class="mass-bars">${bys.map(([s, m]) => `<div class="mass-row"><span>${esc(s)}</span><span class="bar"><i style="width:${(m / max * 100).toFixed(1)}%"></i></span><b>${fmt(m, 1)} kg</b></div>`).join('')}</div></div>
    <div class="flight-notes" style="grid-template-columns:1fr">
      <div><h4>Centre of gravity</h4><p>Deployed: x ${fmt(MP.cg.x * 1000, 0)} mm, y ${fmt(MP.cg.y * 1000, 0)} mm, z ${fmt(MP.cg.z * 1000, 0)} mm from the bus centre.<br>Stowed: ${fmt((0.4 - ST.cg.y) * 1000, 0)} mm below the separation plane, ${fmt(Math.hypot(ST.cg.x, ST.cg.z) * 1000, 0)} mm off its axis.</p></div>
      <div><h4>Moments of inertia (deployed)</h4><p>Roll (about the along-track axis) ${fmt(MP.I[0], 1)} kg·m² · pitch ${fmt(MP.I[2], 1)} kg·m² · yaw ${fmt(MP.I[1], 1)} kg·m².<br>The 4.1 m array dominates pitch and yaw; roll, the axis used to point the beam, stays small, so the wheels slew it quickly.</p></div>
      <div><h4>Stowed for launch</h4><p>Envelope ${P.env.map((d) => fmt(d * 1000, 0)).join(' × ')} mm. Roll / pitch / yaw inertia ${ST.I.map((x) => fmt(x, 1)).join(' / ')} kg·m².</p></div>
    </div></div>`;
}

// ------------------------------------------------------------------ 03 physics (model shared by tabs 01, 03 and 04)
let F, modelCache = null, modelKey = '';
function initPhysics() {
  F = physDefaults(SAT.data.meta);
  $('phys-inputs').innerHTML = PHYS_INPUTS.map(([g, list]) => `<h4>${g.toUpperCase()}</h4>` + list.map(([k, l, u, a, b, s]) => u === 'sel'
    ? `<label class="fin" for="fin-${k}">${l}<select id="fin-${k}" data-k="${k}">${a.map((o) => `<option value="${o.v}" ${o.v === F[k] ? 'selected' : ''}>${esc(o.label)}</option>`).join('')}</select></label>`
    : `<label class="fin" for="fin-${k}">${l}<output id="fout-${k}"></output><input id="fin-${k}" type="range" min="${a}" max="${b}" step="${s}" value="${F[k]}" data-k="${k}"></label>`).join('')).join('') +
    `<button class="tl-btn" id="fin-reset">Reset to the CADed design</button>`;
  $('phys-inputs').querySelectorAll('input, select').forEach((i) => i.addEventListener('input', () => { F[i.dataset.k] = +i.value; renderPhysics(api); }));
  $('fin-reset').addEventListener('click', () => { initPhysics(); renderPhysics(api); });
}
const api = {
  THREE, REPO, views, makeView, lightScene, buildAssembly, setStyle, geometries, attach, PROTO, SAT,
  get F() { return F; },
  model() {
    const key = JSON.stringify(F);
    if (modelCache && key === modelKey) return modelCache;
    if (!api._mp) {   // CAD mass properties: deployed and stowed (krypton added at the tank)
      const tank = SAT.data.instances.find((s) => SAT.data.parts[s[0]].pn === 'TL-601-001');
      api._tank = tank ? { x: tank[2], y: tank[3], z: tank[4] } : { x: 0, y: 0, z: 0 };
      const M = BP.M, save = bpState();
      poseSat(M, { explode: 0 }); api._dep = massProps(M); api._env = null;
      poseSat(M, { sol1: 0, sol2: 0, wing2: 0, wing3: 0, whip: 0, explode: 0 }); api._sto = massProps(M);
      const e = envelope(M); api._env = [e.max.x - e.min.x, e.max.y - e.min.y, e.max.z - e.min.z];
      poseSat(M, save);
    }
    const kr = { m: F.kr, ...api._tank, I: [0, 0, 0], sys: 'Propellant (krypton)' };
    const withKr = (mp) => { const pts = [...mp.pts, kr]; return recompute(pts); };
    const mp = withKr(api._dep), mpStowed = withKr(api._sto);
    modelCache = satModel(F, SAT.data.meta, mp, mpStowed, api._env); modelKey = key;
    return modelCache;
  },
  checks() { const P = api.model(); return physChecks(P); },
  setStep: (t) => setSeqTime(t), showTab,
};
function recompute(pts) {
  const m = pts.reduce((a, b) => a + b.m, 0);
  const cg = { x: pts.reduce((a, b) => a + b.m * b.x, 0) / m, y: pts.reduce((a, b) => a + b.m * b.y, 0) / m, z: pts.reduce((a, b) => a + b.m * b.z, 0) / m };
  const I = [0, 0, 0];
  for (const p of pts) { const dx = p.x - cg.x, dy = p.y - cg.y, dz = p.z - cg.z, Ib = p.I || [0, 0, 0]; I[0] += p.m * (dy * dy + dz * dz) + Ib[0]; I[1] += p.m * (dx * dx + dz * dz) + Ib[1]; I[2] += p.m * (dx * dx + dy * dy) + Ib[2]; }
  const bySys = {}; for (const p of pts) bySys[p.sys] = (bySys[p.sys] || 0) + p.m;
  return { m, cg, I, bySys, pts };
}
window.__sat = api;

// deep link (#mission / #blueprint / #physics / #ports / #proto)
const deep = location.hash.replace('#', '');
showTab(TABS[deep] ? deep : 'mission');
