// Joukowsky Dynamics Rev 08 — blueprint sequence, advanced part blueprint and proprotor / flight dynamics.
// Geometry comes from the CadQuery build (cad/source/build_rev08.py): every part is a B-rep solid,
// tessellated per face into advanced/data/meshes.bin and catalogued in advanced/data/parts.json.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { EffectComposer } from 'three/addons/postprocessing/EffectComposer.js';
import { RenderPass } from 'three/addons/postprocessing/RenderPass.js';
import { UnrealBloomPass } from 'three/addons/postprocessing/UnrealBloomPass.js';
import { OutputPass } from 'three/addons/postprocessing/OutputPass.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';

const REPO = 'https://github.com/ASCoding11/Joukowsky-Dynamics/blob/main/';
const BASE = new URL('./', import.meta.url);
const $ = (id) => document.getElementById(id);
const clamp = (v, a = 0, b = 1) => Math.min(b, Math.max(a, v));
const smooth = (t) => t * t * (3 - 2 * t);
const ramp = (t, a, b) => smooth(clamp((t - a) / (b - a)));
const rad = THREE.MathUtils.degToRad;
const deg = THREE.MathUtils.radToDeg;
const fmt = (v, d = 1) => Number.isFinite(v) ? v.toLocaleString('en-US', { minimumFractionDigits: d, maximumFractionDigits: d }) : '—';
const esc = (s) => String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

// ------------------------------------------------------------------ tabs (works alongside app.js)
const MY_TABS = { sequence: 'sequence-panel', advanced: 'advanced-panel', flight: 'flight-panel', prototype: 'prototype-panel' };
let activeTab = 'model';
document.querySelectorAll('[data-tab]').forEach((b) => b.addEventListener('click', () => showTab(b.dataset.tab)));
function showTab(tab) {
  activeTab = tab;
  for (const [t, id] of Object.entries(MY_TABS)) {
    const el = $(id);
    el.hidden = t !== tab;
    el.classList.toggle('active', t === tab);
  }
  if (!(tab in MY_TABS)) current = null;
  if (tab in MY_TABS) {
    for (const id of ['model-panel', 'systems-panel', 'notes-panel', 'materials-panel']) { const el = $(id); if (el) el.hidden = true; }
    boot().then(() => activate(tab));
  }
}

// ------------------------------------------------------------------ data
let DATA, BUF, bootPromise;
function boot() {
  if (!bootPromise) {
    bootPromise = Promise.all([
      fetch(new URL('data/parts.json', BASE)).then((r) => r.json()),
      fetch(new URL('data/meshes.bin', BASE)).then((r) => (r.ok ? r.arrayBuffer() : Promise.reject(r.status)))
        .catch(() => fetch(new URL('data/meshes.json', BASE)).then((r) => r.json()).then((j) => {   // base64 pack where .bin is not served
          const bin = atob(j.b64), out = new Uint8Array(bin.length);
          for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
          return out.buffer;
        })),
    ]).then(([d, b]) => { DATA = d; BUF = b; buildEngine(); buildModel(); computeMass(); initSequence(); initAdvanced(); initFlight(); })
      .catch((e) => { for (const id of ['seq-loading', 'adv-loading']) { const el = $(id); if (el) el.textContent = 'CAD DATA FAILED TO LOAD: ' + e.message; } throw e; });
  }
  return bootPromise;
}

// ------------------------------------------------------------------ engine (one renderer shared by tabs 05 / 06)
let renderer, scene, composer, bloom, renderPass, envTex, groundMat, platform;
const views = {};
function buildEngine() {
  renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  renderer.setClearColor(0x06121a, 1);
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.localClippingEnabled = true;
  scene = new THREE.Scene();
  const pm = new THREE.PMREMGenerator(renderer);
  envTex = pm.fromScene(new RoomEnvironment(), 0.04).texture;
  scene.add(new THREE.HemisphereLight(0xcfe9ff, 0x0b1a22, 0.6));
  const sun = new THREE.DirectionalLight(0xffffff, 1.6); sun.position.set(6, 10, 7); scene.add(sun);
  // holographic ground grid at the A1 ground plane
  groundMat = new THREE.ShaderMaterial({
    transparent: true, depthWrite: false,
    uniforms: { uOffset: { value: 0 }, uColor: { value: new THREE.Color(0x6be1ed) }, uAlpha: { value: 1 }, uRing: { value: -1 } },
    vertexShader: 'varying vec3 vW; void main(){ vec4 w = modelMatrix*vec4(position,1.); vW=w.xyz; gl_Position=projectionMatrix*viewMatrix*w; }',
    fragmentShader: `uniform float uOffset; uniform vec3 uColor; uniform float uAlpha; uniform float uRing; varying vec3 vW;
      float grid(vec2 p, float s){ vec2 g = abs(fract(p/s-.5)-.5)/fwidth(p/s); return 1.-min(min(g.x,g.y),1.); }
      void main(){ vec2 p = vec2(vW.x + uOffset, vW.z); float r = length(vW.xz);
        float a = grid(p, .5)*.10 + grid(p, 2.5)*.26; float fade = smoothstep(14., 2., r);
        float ring = uRing > 0. ? smoothstep(.10, 0., abs(r - uRing)) * .6 : 0.;
        gl_FragColor = vec4(uColor, (a*fade + ring*fade) * uAlpha); }`,
  });
  const ground = new THREE.Mesh(new THREE.PlaneGeometry(40, 40), groundMat);
  ground.rotation.x = -Math.PI / 2; ground.position.y = DATA.meta.ground_y - 0.002; ground.renderOrder = -1;
  scene.add(ground);
  platform = new THREE.Mesh(new THREE.RingGeometry(3.6, 3.66, 128), new THREE.MeshBasicMaterial({ color: 0x6be1ed, transparent: true, opacity: .35, side: THREE.DoubleSide }));
  platform.rotation.x = -Math.PI / 2; platform.position.y = DATA.meta.ground_y; scene.add(platform);
  composer = new EffectComposer(renderer);
  renderPass = new RenderPass(scene, null); composer.addPass(renderPass);
  bloom = new UnrealBloomPass(new THREE.Vector2(512, 512), 0.55, 0.45, 0.18); composer.addPass(bloom);
  composer.addPass(new OutputPass());
  renderer.setAnimationLoop(loop);
  new ResizeObserver(() => resize()).observe(document.body);
}
function makeView(name, containerId, camPos, target) {
  const camera = new THREE.PerspectiveCamera(36, 1, 0.05, 200);
  camera.position.set(...camPos);
  const el = $(containerId);
  const controls = new OrbitControls(camera, el);
  controls.target.set(...target); controls.enableDamping = true; controls.dampingFactor = 0.08;
  controls.maxDistance = 40; controls.minDistance = 0.4;
  views[name] = { camera, controls, el };
  return views[name];
}
let current = null;
function activate(tab) {
  if (FOCUS.region && FOCUS.view !== tab) exitFocus(true);
  if (tab === 'prototype') {
    current = null;
    bootMini().then(() => { if (activeTab !== 'prototype') return; current = 'prototype'; views.prototype.el.appendChild(renderer.domElement); resize(); });
    return;
  }
  const v = views[tab === 'flight' ? null : tab];
  if (tab === 'flight') { current = null; renderFlight(); return; }
  if (!v) return;
  current = tab;
  v.el.appendChild(renderer.domElement);
  resize();
  if (tab === 'sequence') { applyStyle(seq.style); setSeqTime(seq.t, true); }
  else { applyStyle(adv.holo ? 'holo' : 'cad'); applyAdvState(); }
}
function resize() {
  if (!current) return;
  const v = views[current];
  const w = v.el.clientWidth, h = v.el.clientHeight;
  if (!w || !h) return;
  renderer.setSize(w, h, false);
  composer.setSize(w, h);
  v.camera.aspect = w / h; applyViewOffset(current);
}
// while the tab 05 briefing is open, shift the projection so the model centres in the space left of the panel
function applyViewOffset(view) {
  const v = views[view]; if (!v) return;
  const w = v.el.clientWidth, h = v.el.clientHeight;
  const ui = REGION_UI[view];
  const panel = view === 'sequence' && FOCUS.region === null ? 0 : (view === 'sequence' && ui && !ui.brief.hidden && w > 700 ? ui.brief.offsetWidth + 16 : 0);
  if (panel) v.camera.setViewOffset(w, h, panel / 2, 0, w, h); else v.camera.clearViewOffset();
  v.camera.updateProjectionMatrix();
}
const clock = new THREE.Clock();
function loop() {
  const dt = Math.min(clock.getDelta(), 0.1);
  if (!current || document.hidden) return;
  const v = views[current];
  if (current === 'sequence') tickSequence(dt);
  if (current === 'prototype') { tickProto(dt); v.controls.update(); renderer.render(MINI.scene, v.camera); return; }
  tickFocus(dt, current);
  v.controls.update();
  const t = performance.now() / 1000;
  for (const m of holoMats) m.uniforms.uTime.value = t;
  renderPass.camera = v.camera;
  if (current === 'sequence' && seq.style === 'holo' && seq.bloom) composer.render();
  else renderer.render(scene, v.camera);
  if (current === 'sequence') drawCallouts();
  drawRegionLabels(current);
}

// ------------------------------------------------------------------ materials
const LOOKS = {
  metal: { metalness: .85, roughness: .32 }, paint: { metalness: .25, roughness: .3, clearcoat: 1 },
  carbon: { metalness: .3, roughness: .42 }, glass: { metalness: 0, roughness: .05, transparent: true, opacity: .32 },
  rubber: { metalness: 0, roughness: .92 }, light: { metalness: 0, roughness: .1, emissive: 0x9fd8ff, emissiveIntensity: .6 },
  cell: { metalness: .5, roughness: .4 }, pcb: { metalness: .2, roughness: .6 }, plastic: { metalness: .05, roughness: .7 },
  fabric: { metalness: 0, roughness: .95 }, hv: { metalness: .1, roughness: .55 }, copper: { metalness: .9, roughness: .3 },
};
const cadMats = {};
function cadMat(key) {
  if (cadMats[key]) return cadMats[key];
  const m = DATA.materials[key];
  const L = LOOKS[m.look] || LOOKS.metal;
  const mat = new THREE.MeshPhysicalMaterial({ color: new THREE.Color(m.color), envMap: envTex, envMapIntensity: .9, side: THREE.DoubleSide, ...L });
  if (L.transparent) mat.depthWrite = false;
  return (cadMats[key] = mat);
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
        float band = smoothstep(.35, 0., abs(vW.x - uReveal));
        float scan = .5 + .5*sin(vW.y*42. - uTime*3.);
        float a = uOpacity*(.22 + rim*uRim) + band*.5;
        gl_FragColor = vec4(uColor*(.55 + rim*1.1 + .12*scan) + vec3(.8,1.,1.)*band, a); }`,
  });
  holoMats.push(m);
  return m;
}
function holoLine(color, opacity) {
  const m = new THREE.ShaderMaterial({
    transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, clipping: true,
    uniforms: { uColor: { value: new THREE.Color(color) }, uOpacity: { value: opacity }, uReveal: { value: 99 }, uTime: { value: 0 }, uRim: { value: 0 } },
    vertexShader: `varying vec3 vW;
      #include <clipping_planes_pars_vertex>
      void main(){ vec4 w = modelMatrix*vec4(position,1.); vW = w.xyz; vec4 mvPosition = viewMatrix*w;
        #include <clipping_planes_vertex>
        gl_Position = projectionMatrix*mvPosition; }`,
    fragmentShader: `uniform vec3 uColor; uniform float uOpacity; uniform float uReveal; varying vec3 vW;
      #include <clipping_planes_pars_fragment>
      void main(){
        #include <clipping_planes_fragment>
        if (vW.x > uReveal) discard;
        float band = smoothstep(.35, 0., abs(vW.x - uReveal));
        gl_FragColor = vec4(uColor + vec3(1.)*band, uOpacity + band*.6); }`,
  });
  holoMats.push(m);
  return m;
}
let H; // holo material sets
function makeHoloSets() {
  H = {
    holo: { mesh: holoMat(0x2fb8cc, .10), line: holoLine(0x6be1ed, .42), meshHot: holoMat(0xffb35c, .16), lineHot: holoLine(0xffc27a, .7), shell: holoMat(0x4fd2e4, .07, 3.2),
      ghost: holoMat(0x2fb8cc, .015, .6), ghostLine: holoLine(0x2f8fa0, .05) },
    xray: { mesh: holoMat(0x8fb8ff, .05, 1.2), line: holoLine(0xcfe4ff, .22), meshHot: holoMat(0xff9d6b, .12), lineHot: holoLine(0xffd0a8, .6), shell: holoMat(0xa9c8ff, .04, 2.6),
      ghost: holoMat(0x8fb8ff, .012, .5), ghostLine: holoLine(0x7f9fd0, .04) },
  };
}
const highlightMat = new THREE.MeshBasicMaterial({ color: 0xffb35c, transparent: true, opacity: .55, depthTest: false });
const highlightLine = new THREE.LineBasicMaterial({ color: 0xffe2b8, depthTest: false });
const ghostMat = new THREE.MeshBasicMaterial({ color: 0x6be1ed, transparent: true, opacity: .035, depthWrite: false });
const ghostLine = new THREE.LineBasicMaterial({ color: 0x6be1ed, transparent: true, opacity: .05, depthWrite: false });
const cadHotLine = new THREE.LineBasicMaterial({ color: 0xffb35c, transparent: true, opacity: .9 });

// ------------------------------------------------------------------ model
let root;
const NODE = [];          // { def, group, q0, p0 }
const NODE_BY_ID = {};
const PARTS = [];         // { def, geo, edges, idx }
const OBJS = [];          // render records { obj, line, pi, ni, instIdx:[global instance ids], sys }
const INST_TO_OBJ = [];   // global instance -> [rec, localIndex]
const animNodes = [];
function decode(p, buf = BUF) {
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
function buildModel() {
  makeHoloSets();
  for (const p of DATA.parts) { const geo = decode(p); PARTS.push({ def: p, geo, edges: new THREE.EdgesGeometry(geo, 28), sys: p.system.slice(0, 3) }); }
  DATA.nodes.forEach((n, i) => {
    const g = new THREE.Group(); g.name = n.id;
    g.position.set(...n.pos); g.quaternion.set(...n.q);
    if (n.mirror) g.scale.set(1, 1, -1);
    const rec = { def: n, group: g, q0: g.quaternion.clone(), p0: g.position.clone(), i };
    NODE.push(rec); NODE_BY_ID[n.id] = rec;
    if (n.anim) animNodes.push(rec);
  });
  NODE.forEach((r) => { if (r.def.parent >= 0) NODE[r.def.parent].group.add(r.group); else root = r.group; });
  scene.add(root);
  // group instances by (part, node)
  const groups = new Map();
  DATA.instances.forEach((ins, gi) => {
    const key = ins[0] + '|' + ins[1];
    if (!groups.has(key)) groups.set(key, []);
    groups.get(key).push(gi);
  });
  const m4 = new THREE.Matrix4(), qq = new THREE.Quaternion(), vv = new THREE.Vector3(), one = new THREE.Vector3(1, 1, 1);
  for (const [key, list] of groups) {
    const [pi, ni] = key.split('|').map(Number);
    const P = PARTS[pi], parent = NODE[ni].group;
    const mat = cadMat(P.def.material);
    const mtx = list.map((gi) => { const s = DATA.instances[gi]; return new THREE.Matrix4().compose(vv.set(s[2], s[3], s[4]), qq.set(s[5], s[6], s[7], s[8]), one); });
    let obj, line;
    if (list.length >= 3) {
      obj = new THREE.InstancedMesh(P.geo, mat, list.length);
      mtx.forEach((m, k) => obj.setMatrixAt(k, m));
      obj.instanceMatrix.needsUpdate = true; obj.computeBoundingSphere();
      // merged edges in node space
      const src = P.edges.attributes.position.array;
      if (src.length * list.length < 900000) {
        const out = new Float32Array(src.length * list.length);
        mtx.forEach((m, k) => { for (let i = 0; i < src.length; i += 3) { vv.set(src[i], src[i + 1], src[i + 2]).applyMatrix4(m); out.set([vv.x, vv.y, vv.z], k * src.length + i); } });
        const lg = new THREE.BufferGeometry(); lg.setAttribute('position', new THREE.BufferAttribute(out, 3));
        line = new THREE.LineSegments(lg, cadLine);
      }
    } else {
      obj = new THREE.Group();
      mtx.forEach((m) => {
        const me = new THREE.Mesh(P.geo, mat); me.matrixAutoUpdate = false; me.matrix.copy(m);
        const li = new THREE.LineSegments(P.edges, cadLine); li.matrixAutoUpdate = false; li.matrix.copy(m);
        me.userData.rec = null; obj.add(me, li);
      });
    }
    parent.add(obj); if (line) parent.add(line);
    const rec = { obj, line, pi, ni, inst: list, sys: P.sys, mtx };
    obj.userData.rec = rec;
    if (!obj.isInstancedMesh) obj.children.forEach((c, k) => { c.userData.rec = rec; c.userData.k = c.isMesh ? k / 2 : -1; });
    list.forEach((gi, k) => (INST_TO_OBJ[gi] = [rec, k]));
    OBJS.push(rec);
  }
  assignRegions();
  // rotor-disc glow per module (shown when a rotor spins in flight)
  for (const c of ['FL', 'FR', 'RL', 'RR']) {
    const md = NODE_BY_ID['mod_' + c];
    const disc = new THREE.Mesh(new THREE.RingGeometry(0.13, DATA.meta.duct_R - 0.02, 64, 1), new THREE.MeshBasicMaterial({ color: 0x6be1ed, transparent: true, opacity: 0, depthWrite: false, side: THREE.DoubleSide, blending: THREE.AdditiveBlending }));
    disc.position.z = DATA.meta.z_rotor; disc.userData.disc = true; md.group.add(disc); discs.push(disc);
  }
}
const discs = [];
let styleNow = 'cad';
let hotSystems = new Set();
function applyStyle(style) {
  styleNow = style;
  const set = H[style];
  const fr = FOCUS.region, hv = FOCUS.hover;
  for (const r of OBJS) {
    const inFocus = fr && r.region === fr, ghost = fr && !inFocus;
    const hot = fr ? inFocus : hv ? r.region === hv : hotSystems.has(r.sys);
    const shell = r.sys === '101' || r.sys === '102';
    let mesh = style === 'cad' ? cadMat(PARTS[r.pi].def.material) : hot ? set.meshHot : shell ? set.shell : set.mesh;
    let line = style === 'cad' ? (hot && hv && !fr ? cadHotLine : cadLine) : hot ? set.lineHot : set.line;
    if (ghost) { mesh = style === 'cad' ? ghostMat : set.ghost; line = style === 'cad' ? ghostLine : set.ghostLine; }
    if (r.obj.isInstancedMesh) r.obj.material = mesh; else r.obj.children.forEach((c) => (c.material = c.isMesh ? mesh : line));
    if (r.line) r.line.material = line;
  }
  discs.forEach((d) => d.material.color.set(style === 'cad' ? 0x8aa6b0 : 0x6be1ed));
  groundMat.uniforms.uAlpha.value = style === 'cad' ? .45 : 1;
  platform.visible = style !== 'cad';
  renderer.setClearColor(style === 'cad' ? 0x0b1a23 : 0x050f16, 1);
}
function setReveal(x) { for (const m of holoMats) m.uniforms.uReveal.value = x; }

// ------------------------------------------------------------------ vehicle state -> nodes
const EXPLODE_GROUPS = {
  body: [0, 2.6, 0], door_L: [0, 2.4, 2.4], door_R: [0, 2.4, -2.4], cockpit: [0, 1.5, 0], power: [0, -1.5, 0], avionics: [2.2, 1.2, 0],
  thermal: [3.2, .3, 0], emergency: [-3.0, 1.3, 0], road: [0, -.9, 0],
  corner_FL: [1.4, .3, 2.6], corner_FR: [1.4, .3, -2.6], corner_RL: [-1.4, .3, 2.6], corner_RR: [-1.4, .3, -2.6],
  leg_FL: [.4, -1.3, .9], leg_FR: [.4, -1.3, -.9], leg_RL: [-.4, -1.3, .9], leg_RR: [-.4, -1.3, -.9],
};
const EXPLODE_ORDER = ['power', 'leg_FL', 'leg_FR', 'leg_RL', 'leg_RR', 'road', 'corner_FL', 'corner_FR', 'corner_RL', 'corner_RR', 'cockpit', 'thermal', 'avionics', 'emergency', 'body', 'door_L', 'door_R'];
const SUB_EXPLODE = (id) => {
  if (/^rotor_/.test(id)) return [0, 0, .55];
  if (/^petO_/.test(id)) return [0, 0, .28];
  if (/^petI_/.test(id)) return [0, 0, -.30];
  if (/^blade_/.test(id)) return [.16, 0, 0];
  if (/^yoke_/.test(id)) return [0, .25, 0];
  if (/^bmod_/.test(id)) return [0, .25, 0];
  return null;
};
const _q = new THREE.Quaternion(), _v = new THREE.Vector3(), _ax = new THREE.Vector3();
function applyState(st) {
  for (const r of NODE) {
    const g = r.group, a = r.def.anim, id = r.def.id;
    g.position.copy(r.p0); g.quaternion.copy(r.q0);
    const ex = EXPLODE_GROUPS[id];
    if (ex) {
      const e = typeof st.explode === 'object' ? (st.explode[id] ?? 0) : st.explode;
      g.position.x += ex[0] * e; g.position.y += ex[1] * e; g.position.z += ex[2] * e;
    }
    if (st.sub) { const s = SUB_EXPLODE(id); if (s) g.position.add(_v.set(...s).multiplyScalar(st.sub).applyQuaternion(r.q0)); }
    if (!a) continue;
    _ax.set(...a.axis);
    let ang = null;
    switch (a.kind) {
      case 'lift': g.position.y += st.lift; break;
      case 'yoke': ang = rad(-90 * st.yoke) * a.sign; break;
      case 'tilt': ang = rad(st.tilt) * a.sign; break;
      case 'rim': ang = st.rim * a.sign; break;
      case 'rotor': ang = st.rotorLocked ? st.rim : st.rotor * a.sign; break;
      case 'pitch': ang = rad(st.pitch) * a.sign; break;
      case 'petal_out': ang = rad(100 * st.petals) * a.sign; break;
      case 'petal_in': ang = a.index ? 0 : rad(95 * st.petals) * a.sign; break;
      case 'leg': g.position.addScaledVector(_ax, a.stroke * st.legs); break;
      case 'door': ang = rad(62 * (st.doors || 0)) * a.sign; break;
    }
    if (ang !== null) g.quaternion.multiply(_q.setFromAxisAngle(_ax, ang));
  }
  const op = clamp(st.spin / 3) * (styleNow === 'cad' ? .25 : .35);
  discs.forEach((d) => (d.material.opacity = op));
}
const STATE0 = { explode: 0, sub: 0, lift: 0, yoke: 0, tilt: 0, rim: 0, rotor: 0, rotorLocked: true, pitch: 90, petals: 0, legs: 0, doors: 0, spin: 0 };

// ------------------------------------------------------------------ mass properties (from the CAD)
const MASS = {};
function computeMass() {
  const PAYLOAD = 180;
  const at = (st) => {
    applyState({ ...STATE0, ...st }); root.updateMatrixWorld(true);
    let m = 0; const c = new THREE.Vector3(); const bySys = {};
    const ctr = new THREE.Vector3(), mm = new THREE.Matrix4();
    for (const r of OBJS) {
      const P = PARTS[r.pi], mk = P.def.mass_kg;
      if (P.def.com) ctr.set(...P.def.com); else P.geo.boundingBox.getCenter(ctr);   // B-rep centre of mass
      const node = NODE[r.ni].group;
      r.mtx.forEach((mt) => { _v.copy(ctr).applyMatrix4(mm.multiplyMatrices(node.matrixWorld, mt)); c.addScaledVector(_v, mk); m += mk; });
      bySys[P.def.system] = (bySys[P.def.system] || 0) + mk * r.mtx.length;
    }
    // two occupants at the seat H-points
    c.addScaledVector(_v.set(-0.85, 0.75, 0), PAYLOAD); m += PAYLOAD;
    return { m, cg: c.divideScalar(m), bySys };
  };
  MASS.road = at({});
  MASS.hover = at({ yoke: 1, petals: 1, pitch: 26, legs: 0, lift: 0 });
  MASS.rotors = {};
  root.updateMatrixWorld(true);
  for (const c of ['FL', 'FR', 'RL', 'RR']) { const p = new THREE.Vector3(); NODE_BY_ID['mod_' + c].group.getWorldPosition(p); MASS.rotors[c] = p; }
  MASS.empty = MASS.road.m - PAYLOAD; MASS.payload = PAYLOAD;
  MASS.cells = DATA.parts.find((p) => p.pn === 'JD-200-011')?.qty || 0;
  applyState(STATE0);
}

// ------------------------------------------------------------------ 05 SEQUENCE
const STEPS = [
  { t: 0, code: 'BOOT', title: 'System scan', sub: 'Rev 08 CAD loads as a hologram', hot: [], cam: [[8.5, 4.2, 8.5], [0, .5, 0]],
    text: 'The scan front sweeps nose-ward and reveals 242 unique CADed parts in their exploded positions.' },
  { t: 6, code: 'ASSEMBLE', title: 'Assembly', sub: 'Systems dock into the safety cell', hot: ['200', '300', '110'], cam: [[9.5, 5, 9.5], [0, .5, 0]],
    text: 'Battery, legs, fan-wheel corners, cockpit, avionics, thermal, parachute bay and body panels lock onto the carbon tub.',
    callouts: [['bmod_07', [0, .1, 0], 'ENERGY STORAGE', '19 modules · 5,472 × 21700 cells', -110, 70], ['mod_FL', [0, .5, .2], 'FAN-WHEEL MODULE ×4', 'tyre + duct + 7-blade proprotor', 120, -70],
      ['body', [0, 1.3, 0], 'BODY-IN-WHITE', 'lofted from the A1 surfaces', -140, -60]] },
  { t: 14, code: 'ROAD', title: 'Road mode', sub: '60 km/h · fan hardware stowed and locked', hot: [], cam: [[7.2, 1.4, 5.6], [0, .4, 0]],
    text: 'The airless tyre rolls on the duct rim. Blades sit feathered at 90° and the dog clutch couples them to the rim, so the motor drives the wheel. Petal covers are closed.',
    callouts: [['mod_FL', [0, .2, .22], 'PETAL COVERS CLOSED', 'blade guard in road mode', 110, -80], ['rotor_FL', [0, 0, .05], 'BLADES FEATHERED 90°', 'act as wheel spokes', 130, 30]] },
  { t: 20, code: 'PREPARE', title: 'Stop & prepare', sub: 'Landing legs carry the car off its tyres', hot: ['600'], cam: [[6.6, 1.6, 6.6], [0, .2, 0]],
    text: 'Parking brake, self-test, then four telescoping legs deploy and lift the car 120 mm so each wheel can swing freely.',
    callouts: [['leg2_FL', [0, 0, 0], 'LANDING LEGS ×4', 'ball-screw · oleo · down-lock + WOW', 120, 40]] },
  { t: 27, code: 'UNLOCK', title: 'Unlock & open', sub: 'Clutch release · rim lock · petals open', hot: ['300', '320', '330', '360'], cam: [[4.6, 1.9, 4.9], [1.8, .7, 1.2]],
    text: 'The dog clutch releases the rotor from the rim. The brake caliper and two lock pins index the rim to the module. Iris petals open into an inlet collector and an exit diffuser, and collective drives from 90° to the hover setting.',
    callouts: [['mod_FL', [0, .62, .2], 'IRIS PETALS 12 + 12', 'inlet collector / exit diffuser', 120, -70], ['mod_FL', [0, .6, -.19], 'RIM LOCK', 'caliper clamp + 2 pins', -150, 60],
      ['rotor_FL', [0, 0, .1], 'COLLECTIVE 90° → 26°', 'electromechanical pitch actuator', 130, 40]] },
  { t: 31, code: 'CONVERT', title: 'Wheel → proprotor', sub: 'Yokes swing each wheel 90° up and out', hot: ['340', '300'], cam: [[6.2, 3.6, 7.6], [0, 1, 0]],
    text: 'A harmonic-drive rotary actuator on each hinge swings the module up and outboard. The fender cap rides on the yoke. Redundant RVDTs and locks must agree before arming.',
    callouts: [['yoke_FL', [0, 0, 0], 'CONVERSION HINGE', '2.4 kN·m harmonic drive · dual RVDT', -150, -60], ['yoke_RL', [0, 0, 0], 'FENDER CAP', 'lifts away with the yoke', 120, -80]] },
  { t: 37, code: 'SPIN-UP', title: 'Spin-up', sub: '4 × 320 kW axial-flux motors', hot: ['320', '330'], cam: [[5.2, 6.2, 6.4], [0, 1, 0]],
    text: 'Rotors spin up to hover RPM. Diagonal pairs counter-rotate for yaw balance; the stators remove swirl.',
    callouts: [['mod_FR', [0, 0, .3], 'CCW / CW DIAGONAL PAIRS', 'yaw from differential torque', -150, -70], ['mod_RL', [0, 0, .3], 'DUCTED PROPROTOR', 'R 585 mm · σ 0.36 · B 7', 130, -60]] },
  { t: 41, code: 'LIFT-OFF', title: 'Lift-off & hover', sub: 'Legs retract after weight-off', hot: ['330'], cam: [[8.4, 3.2, 8.4], [0, 2.4, 0]],
    text: 'Thrust balances weight, the car climbs and the legs retract once weight-on-wheels clears.',
    callouts: [['mod_FL', [0, 0, 0], 'HOVER', 'disk loading ≈ 107 lb/ft²', 130, -60]] },
  { t: 47, code: 'TRANSITION', title: 'Partial conversion', sub: 'Modules tilt 25° forward', hot: ['340'], cam: [[-6.5, 4.2, 8.6], [0, 3.6, 0]],
    text: 'Gimbals tilt each rotor axis forward. Thrust vectors carry weight and drag; the body adds a little lift.',
    callouts: [['tilt_FL', [0, 0, 0], 'CRUISE TILT GIMBAL', '0–30° forward', 130, -60]] },
  { t: 53, code: 'CRUISE', title: 'Cruise', sub: '160 km/h', hot: [], cam: [[9, 5, 6], [0, 4, 0]],
    text: 'Edgewise-dominant cruise at μ ≈ 0.19, well inside the memo’s practical limit (μ < 0.7). See tab 07 for power and endurance.',
    callouts: [['mod_RR', [0, 0, 0], 'EDGEWISE μ ≈ 0.19', 'NASA TM Eq. 7', -150, -60]] },
];
const T_END = 60;
const seq = { t: 0, playing: false, dir: 1, speed: 1, style: 'holo', bloom: false, autoCam: true, step: -1, table: null };
function buildTables() {
  const dt = 0.01, n = Math.ceil(T_END / dt) + 2;
  const rim = new Float32Array(n), rot = new Float32Array(n), grd = new Float32Array(n);
  const v = (t) => 16.7 * ramp(t, 14, 16) * (1 - ramp(t, 20, 23));
  const vf = (t) => 44.4 * ramp(t, 47, 53);
  const w = (t) => 2.6 * ramp(t, 37, 41);      // visual rev/s (true 4,420 rpm aliases on screen)
  for (let i = 1; i < n; i++) {
    const t = i * dt;
    rim[i] = rim[i - 1] - v(t) * dt / DATA.meta.tire_R;
    rot[i] = rot[i - 1] + w(t) * 2 * Math.PI * dt;
    grd[i] = grd[i - 1] + (v(t) + vf(t) * .25) * dt;
  }
  // index the rim to a whole turn so the locked rim and petals line up with the yoke
  const total = rim[n - 1], k = Math.round(total / (2 * Math.PI)) * 2 * Math.PI / (total || 1);
  for (let i = 0; i < n; i++) rim[i] *= k;
  seq.table = { dt, rim, rot, grd, v, vf, w };
}
function lookup(arr, t) { const i = clamp(Math.floor(t / seq.table.dt), 0, arr.length - 1); return arr[i]; }
function seqState(t) {
  const ex = {};
  EXPLODE_ORDER.forEach((id, i) => (ex[id] = 1 - ramp(t, 6.2 + i * .38, 8.4 + i * .38)));
  return {
    explode: ex, sub: 0,
    legs: ramp(t, 23, 26.5) - ramp(t, 43, 46.5),
    lift: 0.12 * ramp(t, 23, 26.5) + 2.9 * ramp(t, 41, 47) + 1.0 * ramp(t, 47, 55),
    petals: ramp(t, 27.5, 30), pitch: 90 - 64 * ramp(t, 28.5, 31) + 4 * ramp(t, 47, 53),
    yoke: ramp(t, 31, 37), tilt: 25 * ramp(t, 47, 53), rotorLocked: t < 28,
    rim: lookup(seq.table.rim, t), rotor: lookup(seq.table.rot, t), spin: seq.table.w(t), doors: 0,
  };
}
function initSequence() {
  buildTables();
  const v = makeView('sequence', 'seq-viewport', STEPS[0].cam[0], STEPS[0].cam[1]);
  v.controls.addEventListener('start', () => (seq.autoCam = false));
  $('seq-loading').hidden = true;
  $('seq-steps').innerHTML = STEPS.map((s, i) => `<li><button data-step="${i}"><span>T+${String(s.t).padStart(2, '0')}</span><strong>${esc(s.title)}</strong><small>${esc(s.text)}</small></button></li>`).join('');
  $('seq-steps').querySelectorAll('button').forEach((b) => b.addEventListener('click', () => { seq.autoCam = true; seq.snap = true; setSeqTime(STEPS[+b.dataset.step].t + .01); }));
  $('seq-play').addEventListener('click', () => { if (!seq.playing && seq.t >= T_END - .05 && seq.dir > 0) setSeqTime(0); seq.dir = 1; seq.playing = !seq.playing; seq.autoCam = true; syncPlay(); });
  $('seq-land').addEventListener('click', () => { if (seq.t < 15) setSeqTime(T_END); seq.dir = -1; seq.playing = true; seq.autoCam = true; syncPlay(); });
  $('seq-scrub').addEventListener('input', (e) => { seq.playing = false; syncPlay(); setSeqTime(+e.target.value); });
  $('seq-speed').addEventListener('change', (e) => (seq.speed = +e.target.value));
  $('seq-bloom').addEventListener('change', (e) => (seq.bloom = e.target.checked));
  document.querySelectorAll('[data-seqstyle]').forEach((b) => b.addEventListener('click', () => {
    seq.style = b.dataset.seqstyle;
    document.querySelectorAll('[data-seqstyle]').forEach((x) => x.classList.toggle('active', x === b));
    $('seq-viewport').classList.toggle('holo', seq.style !== 'cad');
    applyStyle(seq.style); setSeqTime(seq.t, true);
  }));
  const reduce = matchMedia('(prefers-reduced-motion: reduce)').matches;
  seq.playing = !reduce; syncPlay();
  initRegionUI('sequence');
}
function syncPlay() { $('seq-play').textContent = seq.playing && seq.dir > 0 ? '❚❚ Pause' : '▶ Play'; $('seq-land').textContent = seq.playing && seq.dir < 0 ? '❚❚ Pause' : '◀ Land & stow'; }
function tickSequence(dt) {
  if (seq.playing) {
    let t = seq.t + dt * seq.speed * seq.dir;
    if (seq.dir < 0 && t <= 14.01) { t = 14.01; seq.playing = false; syncPlay(); }
    if (t >= T_END) { t = T_END; seq.playing = false; syncPlay(); }
    setSeqTime(t);
  }
  if (seq.autoCam && !FOCUS.region) {
    const s = STEPS[Math.max(0, seq.step)], v = views.sequence;
    const lift = seqState(seq.t).lift;
    const tp = _v.set(...s.cam[1]); if (seq.step >= 7) tp.y = Math.max(tp.y, lift + .4);
    const cp = new THREE.Vector3(...s.cam[0]); if (seq.step >= 7) cp.y += lift * .6;
    const k = seq.snap ? 1 : 1 - Math.pow(0.2, dt);           // frame-rate independent ease
    v.controls.target.lerp(tp, k); v.camera.position.lerp(cp, k); seq.snap = false;
  }
}
function setSeqTime(t, force) {
  seq.t = clamp(t, 0, T_END);
  const st = seqState(seq.t);
  applyState(st);
  setReveal(seq.style === 'cad' ? 99 : -3.8 + 7.6 * ramp(seq.t, .5, 5.5));
  groundMat.uniforms.uOffset.value = lookup(seq.table.grd, seq.t);
  groundMat.uniforms.uRing.value = seq.t < 6 ? 1 + seq.t * 2.4 : -1;
  let k = 0; STEPS.forEach((s, i) => { if (seq.t >= s.t) k = i; });
  if (k !== seq.step || force) {
    seq.step = k;
    const s = STEPS[k];
    hotSystems = new Set(s.hot); if (activeTab === 'sequence') applyStyle(seq.style);
    $('seq-step-title').textContent = s.title; $('seq-step-sub').textContent = s.sub;
    $('seq-steps').querySelectorAll('button').forEach((b, i) => { b.classList.toggle('active', i === k); b.classList.toggle('done', i < k); });
    buildCallouts(s.callouts || []);
  }
  $('seq-step-code').textContent = `${STEPS[k].code} · T+${fmt(seq.t, 1)} s`;
  $('seq-scrub').value = seq.t;
  $('seq-time').textContent = `${fmt(seq.t, 1).padStart(4, '0')} / ${T_END}.0 s`;
  telemetry(st);
}
function telemetry(st) {
  const P = flightModel(FLIGHT);
  const road = seq.table.v(seq.t), air = seq.table.vf(seq.t);
  const mode = seq.t < 14 ? 'ASSEMBLY' : seq.t < 20 ? 'ROAD' : seq.t < 31 ? 'PREPARE / CONFIGURE' : seq.t < 41 ? 'READY' : 'FLIGHT';
  const rpm = st.rotorLocked ? Math.abs(road) / DATA.meta.tire_R * 60 / (2 * Math.PI) : P.rpmHover * clamp(st.spin / 2.6);
  const alt = Math.max(0, st.lift - 0.12 * st.legs - (st.lift > .13 ? .12 : st.lift));
  const pw = seq.t < 37 ? road * 0.6 : P.hoverkW * clamp(st.spin / 2.6) * (seq.t > 47 ? 1 - .18 * ramp(seq.t, 47, 53) : 1);
  const items = [['MODE', mode, 'hot'], ['GROUND / AIR SPEED', `${fmt((road + air) * 3.6, 0)} km/h`], ['ALTITUDE AGL', `${fmt(alt, 2)} m`], ['ROTOR SPEED', `${fmt(rpm, 0)} rpm`],
    ['CONVERSION YOKE', `${fmt(st.yoke * 90, 0)}°`], ['CRUISE TILT', `${fmt(st.tilt, 1)}°`], ['COLLECTIVE θ.75', `${fmt(st.pitch, 1)}°`], ['PETALS', `${fmt(st.petals * 100, 0)} %`],
    ['ELECTRICAL POWER', `${fmt(pw, 0)} kW`], ['LEGS', st.legs > .98 ? 'DOWN & LOCKED' : st.legs < .02 ? 'STOWED' : 'IN TRANSIT']];
  $('seq-telemetry').innerHTML = items.map(([k, v, c]) => `<div><small>${k}</small><b class="${c || ''}">${v}</b></div>`).join('');
}
let calloutEls = [];
function buildCallouts(list) {
  $('seq-callouts').innerHTML = ''; calloutEls = [];
  for (const [nid, pt, title, sub, dx, dy] of list) {
    if (!NODE_BY_ID[nid]) continue;
    const el = document.createElement('div'); el.className = 'callout-tag'; el.innerHTML = `${esc(title)}<small>${esc(sub)}</small>`;
    $('seq-callouts').appendChild(el);
    calloutEls.push({ el, node: NODE_BY_ID[nid].group, pt: new THREE.Vector3(...pt), dx, dy });
  }
}
function drawCallouts() {
  const v = views.sequence, w = v.el.clientWidth, h = v.el.clientHeight;
  let svg = '';
  if (FOCUS.region) { calloutEls.forEach((c) => (c.el.style.opacity = 0)); $('seq-leaders').innerHTML = ''; return; }
  for (const c of calloutEls) {
    const p = c.pt.clone().applyMatrix4(c.node.matrixWorld).project(v.camera);
    const vis = p.z < 1 && Math.abs(p.x) < 1.1 && Math.abs(p.y) < 1.1;
    c.el.style.opacity = vis ? 1 : 0;
    if (!vis) continue;
    const x = (p.x + 1) / 2 * w, y = (1 - p.y) / 2 * h;
    const lx = clamp(x + c.dx, 100, w - 100), ly = clamp(y + c.dy, 110, h - 24);
    c.el.style.left = lx + 'px'; c.el.style.top = ly + 'px';
    svg += `<line x1="${x}" y1="${y}" x2="${lx}" y2="${ly}"/><circle cx="${x}" cy="${y}" r="3"/>`;
  }
  $('seq-leaders').innerHTML = svg;
}

// ------------------------------------------------------------------ 06 ADVANCED BLUEPRINT
const adv = { sel: -1, selInst: -1, holo: false, config: 0, explode: 0, view: '3d', hidden: new Set(), overlays: [], sort: ['pn', 1], isolate: false };
function configState(c) {
  // 0 road → 1 converted on the legs → 1.3 airborne hover (legs up) → 2 cruise tilt
  const a = clamp(c), up = ramp(c, 1, 1.3), b = ramp(c, 1.3, 2);
  return { ...STATE0, legs: ramp(a, 0, .35) * (1 - up), lift: .12 * ramp(a, 0, .35) + 1.1 * up, petals: ramp(a, .3, .6), pitch: 90 - 64 * ramp(a, .4, .7) + 4 * b,
    yoke: ramp(a, .55, 1), tilt: 25 * b, rotorLocked: c < .35, spin: 0, explode: adv.explode, sub: adv.explode };
}
function applyAdvState() {
  applyState(configState(adv.config));
  setReveal(99); groundMat.uniforms.uOffset.value = 0; groundMat.uniforms.uRing.value = -1;
  const c = adv.config;
  $('adv-config-label').textContent = c < .05 ? 'Road' : c < .98 ? 'Converting' : c < 1.3 ? 'Lift-off' : c < 1.4 ? 'Hover' : 'Cruise tilt';
}
function initAdvanced() {
  const v = makeView('advanced', 'adv-viewport', [7.5, 3.6, 7.5], [0, .5, 0]);
  $('adv-loading').hidden = true;
  $('adv-count').textContent = `${DATA.parts.length} PARTS · ${DATA.instances.length.toLocaleString()} PLACED`;
  buildTree();
  $('adv-search').addEventListener('input', () => { buildTree(); if (adv.view === 'bom') renderBom(); });
  $('adv-filter').addEventListener('change', () => { buildTree(); if (adv.view === 'bom') renderBom(); });
  $('adv-config').addEventListener('input', (e) => {
    const lift0 = configState(adv.config).lift;
    adv.config = +e.target.value; applyAdvState();
    const dy = configState(adv.config).lift - lift0;          // keep the car in frame as it lifts off
    v.controls.target.y += dy; v.camera.position.y += dy;
    if (FOCUS.region && FOCUS.view === 'advanced') { clearTimeout(adv.refocus); adv.refocus = setTimeout(() => focusRegion('advanced', FOCUS.region), 220); }
  });
  $('adv-explode').addEventListener('input', (e) => { adv.explode = +e.target.value; applyAdvState(); });
  $('adv-section').addEventListener('change', (e) => { renderer.clippingPlanes = e.target.checked ? [new THREE.Plane(new THREE.Vector3(0, 0, -1), 0)] : []; });
  $('adv-holo').addEventListener('change', (e) => { adv.holo = e.target.checked; applyStyle(adv.holo ? 'holo' : 'cad'); refreshIsolation(); });
  $('adv-reset').addEventListener('click', () => {
    adv.config = 0; adv.explode = 0; $('adv-config').value = 0; $('adv-explode').value = 0; $('adv-section').checked = false; renderer.clippingPlanes = [];
    adv.isolate = false; refreshIsolation(); applyAdvState(); v.camera.position.set(7.5, 3.6, 7.5); v.controls.target.set(0, .5, 0);
  });
  document.querySelectorAll('[data-advview]').forEach((b) => b.addEventListener('click', () => setAdvView(b.dataset.advview)));
  initRegionUI('advanced');
  // picking
  const ray = new THREE.Raycaster(); const ndc = new THREE.Vector2(); let down = null;
  v.el.addEventListener('pointerdown', (e) => (down = [e.clientX, e.clientY]));
  v.el.addEventListener('pointerup', (e) => {
    if (!down || Math.hypot(e.clientX - down[0], e.clientY - down[1]) > 5) return;
    const r = v.el.getBoundingClientRect();
    ndc.set((e.clientX - r.left) / r.width * 2 - 1, -(e.clientY - r.top) / r.height * 2 + 1);
    ray.setFromCamera(ndc, v.camera);
    const targets = [];
    for (const rec of OBJS) { if (!rec.obj.visible) continue; if (rec.obj.isInstancedMesh) targets.push(rec.obj); else rec.obj.children.forEach((c) => c.isMesh && targets.push(c)); }
    const hits = ray.intersectObjects(targets, false).filter((hh) => !renderer.clippingPlanes.length || hh.point.z <= 0.001);
    if (!hits.length) return;
    const h0 = hits[0], rec = h0.object.userData.rec;
    const k = h0.object.isInstancedMesh ? h0.instanceId : h0.object.userData.k;
    selectPart(rec.pi, rec.inst[k]);
  });
}
function partMatches(p, q, f) {
  if (f === 'crit' && !p.criticality) return false;
  if (f === 'make' && p.supplier !== 'make') return false;
  if (f === 'buy' && p.supplier !== 'buy') return false;
  if (f === 'handed' && !p.handed) return false;
  if (!q) return true;
  return (p.pn + ' ' + p.name + ' ' + p.system + ' ' + p.assy + ' ' + DATA.materials[p.material].name + ' ' + p.fn).toLowerCase().includes(q);
}
function buildTree() {
  const q = $('adv-search').value.trim().toLowerCase(), f = $('adv-filter').value;
  const sys = {};
  DATA.parts.forEach((p, i) => { if (!partMatches(p, q, f)) return; ((sys[p.system] ||= {})[p.assy] ||= []).push(i); });
  const open = !!(q || f);
  $('adv-tree').innerHTML = Object.keys(sys).sort().map((s) => {
    const assys = sys[s]; const n = Object.values(assys).flat().length;
    const mass = Object.values(assys).flat().reduce((a, i) => a + DATA.parts[i].mass_kg * DATA.parts[i].qty, 0);
    return `<details ${open ? 'open' : ''}><summary><span class="nm">${esc(s)}</span><em>${n} · ${fmt(mass, 0)} kg</em><input type="checkbox" class="sysvis" data-sys="${esc(s)}" ${adv.hidden.has(s) ? '' : 'checked'} aria-label="Show ${esc(s)}"></summary>` +
      Object.keys(assys).sort().map((a) => `<details ${open ? 'open' : ''}><summary><span class="nm">${esc(a)}</span><em>${assys[a].length}</em></summary>` +
        assys[a].map((i) => { const p = DATA.parts[i]; return `<button class="p ${i === adv.sel ? 'sel' : ''}" data-pi="${i}"><span><b>${p.pn}</b>${esc(p.name)}</span><i>×${p.qty}</i></button>`; }).join('') + '</details>').join('') + '</details>';
  }).join('') || '<p class="subtle">No parts match.</p>';
  $('adv-tree').querySelectorAll('.p').forEach((b) => b.addEventListener('click', () => { selectPart(+b.dataset.pi, -1, true); if (adv.view !== '3d') setAdvView('3d'); }));
  $('adv-tree').querySelectorAll('.sysvis').forEach((c) => {
    c.addEventListener('click', (e) => e.stopPropagation());
    c.addEventListener('change', () => { if (c.checked) adv.hidden.delete(c.dataset.sys); else adv.hidden.add(c.dataset.sys); refreshIsolation(); });
  });
}
function clearOverlays() { adv.overlays.forEach((o) => o.parent && o.parent.remove(o)); adv.overlays = []; }
function selectPart(pi, gInst = -1, frame = false) {
  adv.sel = pi; adv.selInst = gInst;
  clearOverlays();
  const P = PARTS[pi];
  for (const rec of OBJS) {
    if (rec.pi !== pi) continue;
    rec.mtx.forEach((m) => {
      const me = new THREE.Mesh(P.geo, highlightMat); me.matrixAutoUpdate = false; me.matrix.copy(m); me.renderOrder = 10;
      const li = new THREE.LineSegments(P.edges, highlightLine); li.matrixAutoUpdate = false; li.matrix.copy(m); li.renderOrder = 11;
      NODE[rec.ni].group.add(me, li); adv.overlays.push(me, li);
    });
  }
  document.querySelectorAll('#adv-tree .p').forEach((b) => b.classList.toggle('sel', +b.dataset.pi === pi));
  const btn = document.querySelector(`#adv-tree .p[data-pi="${pi}"]`);
  if (btn) { btn.closest('details')?.setAttribute('open', ''); btn.closest('details')?.parentElement.closest('details')?.setAttribute('open', ''); btn.scrollIntoView({ block: 'nearest' }); }
  renderSheet(pi);
  refreshIsolation();
  if (frame) frameOverlays();
}
function frameOverlays() {
  const box = new THREE.Box3(); root.updateMatrixWorld(true);
  adv.overlays.forEach((o) => { if (o.isMesh) { const b = o.geometry.boundingBox.clone().applyMatrix4(o.matrixWorld); box.union(b); } });
  if (box.isEmpty()) return;
  const v = views.advanced, c = box.getCenter(new THREE.Vector3()), r = Math.max(.15, box.getSize(new THREE.Vector3()).length() * .5);
  const dir = v.camera.position.clone().sub(v.controls.target).normalize();
  v.controls.target.copy(c); v.camera.position.copy(c).addScaledVector(dir, r / Math.tan(rad(18)) * 1.15);
}
function refreshIsolation() {
  if (FOCUS.region) { applyStyle(styleNow); return; }
  const iso = adv.isolate && adv.sel >= 0;
  for (const rec of OBJS) {
    const sys = PARTS[rec.pi].def.system;
    const hide = adv.hidden.has(sys);
    rec.obj.visible = !hide; if (rec.line) rec.line.visible = !hide && !iso;
    if (rec.obj.isInstancedMesh) { if (iso) rec.obj.material = rec.pi === adv.sel ? rec.obj.material : ghostMat; }
    else rec.obj.children.forEach((c) => { if (iso) { if (c.isMesh) c.material = rec.pi === adv.sel ? c.material : ghostMat; else c.visible = rec.pi === adv.sel; } else c.visible = true; });
  }
  if (!iso) applyStyle(adv.holo ? 'holo' : 'cad');
}
function renderSheet(pi) {
  const p = DATA.parts[pi], m = DATA.materials[p.material];
  const crit = p.criticality ? `<span class="chip ${p.criticality === 'Safety' ? 'safe' : 'crit'}">${esc(p.criticality.toUpperCase())}</span>` : '';
  const spec = Object.entries(p.spec || {}).map(([k, v]) => `<dt>${esc(k.replace(/_/g, ' ').toUpperCase())}</dt><dd>${esc(v)}</dd>`).join('');
  const insts = DATA.instances.filter((s) => s[0] === pi).length;
  const where = [...new Set(DATA.instances.filter((s) => s[0] === pi).map((s) => DATA.nodes[s[1]].name))].slice(0, 6);
  $('adv-sheet').innerHTML = `
    <div><span class="sheet-pn">${p.pn}</span><h3>${esc(p.name)}</h3>
      <div class="chips"><span class="chip">${esc(p.system)}</span><span class="chip">${esc(p.assy)}</span><span class="chip ${p.supplier === 'buy' ? 'buy' : ''}">${p.supplier === 'buy' ? 'BUY' : 'MAKE'}</span>${crit}${p.handed ? '<span class="chip">LH / RH</span>' : ''}</div></div>
    <div class="drawing" id="sheet-drawing">${drawing(pi)}</div>
    <p>${esc(p.fn || p.desc || '')}</p>
    <dl class="kv">
      <dt>QUANTITY</dt><dd>${p.qty} per vehicle (${insts} placements)</dd>
      <dt>MATERIAL</dt><dd>${esc(m.name)}</dd>
      <dt>PROCESS</dt><dd>${esc(p.process || '—')}</dd>
      <dt>ENVELOPE</dt><dd>${p.dims_mm.map((d) => fmt(d, 1)).join(' × ')} mm</dd>
      <dt>UNIT MASS</dt><dd>${fmt(p.mass_kg, p.mass_kg < 1 ? 3 : 2)} kg · total ${fmt(p.mass_kg * p.qty, 2)} kg</dd>
      <dt>VOLUME / AREA</dt><dd>${fmt(p.volume_cm3, 1)} cm³ · ${fmt(p.area_cm2, 0)} cm²</dd>
      <dt>CAD BASIS</dt><dd>${esc(p.basis)} · ${p.faces} B-rep faces · ${p.mesh.ni / 3} display triangles</dd>
      <dt>LOCATION</dt><dd>${where.map(esc).join(', ')}${where.length >= 6 ? '…' : ''}</dd>
      ${spec}
    </dl>
    ${p.desc && p.fn ? `<p>${esc(p.desc)}</p>` : ''}
    <div class="sheet-actions">
      <button id="sheet-frame">Frame in view</button>
      <button id="sheet-iso">${adv.isolate ? 'Show full vehicle' : 'Isolate part'}</button>
      <button id="sheet-region">Expand its region</button>
      <a href="${REPO}cad/rev08/step/${p.step}" target="_blank" rel="noopener">STEP${p.step_lh ? ' (RH)' : ''} ↗</a>
      ${p.step_lh ? `<a href="${REPO}cad/rev08/step/${p.step_lh}" target="_blank" rel="noopener">STEP (LH) ↗</a>` : ''}
    </div>
    <p class="sheet-note">Mass is computed from the B-rep volume (or laminate area × ply thickness for skins) and the material density. Concept geometry only: no tolerances, fits or qualified loads.</p>`;
  const rb = REGION_UI.advanced?.brief; if (rb) $('adv-sheet').prepend(rb);   // keep the region briefing above the datasheet
  $('sheet-frame').onclick = () => { setAdvView('3d'); frameOverlays(); };
  $('sheet-region').onclick = () => { const rec = OBJS.find((r) => r.pi === pi); if (rec) { setAdvView('3d'); focusRegion('advanced', rec.region, rec); } };
  $('sheet-iso').onclick = () => { adv.isolate = !adv.isolate; refreshIsolation(); renderSheet(pi); if (adv.isolate) frameOverlays(); };
}
function drawing(pi) {
  const P = PARTS[pi], pos = P.edges.attributes.position.array, bb = P.geo.boundingBox;
  const sz = bb.getSize(new THREE.Vector3());
  const W = 320, Hh = 230, pad = 16;
  const views3 = [['SIDE', 0, 1, sz.x, sz.y], ['TOP', 0, 2, sz.x, sz.z], ['END', 2, 1, sz.z, sz.y]];
  const cells = [[pad, pad, 190, 110], [pad, 134, 190, 80], [222, pad, 84, 110]];
  const nseg = pos.length / 6, stride = Math.max(1, Math.ceil(nseg / 5000));
  let out = `<svg viewBox="0 0 ${W} ${Hh}" role="img" aria-label="Orthographic drawing">`;
  out += `<rect x="0.5" y="0.5" width="${W - 1}" height="${Hh - 1}" fill="none" stroke="#6be1ed30"/>`;
  views3.forEach(([name, a, b, wa, wb], vi) => {
    const [cx, cy, cw, ch] = cells[vi];
    const s = Math.min(cw / Math.max(wa, 1e-6), ch / Math.max(wb, 1e-6)) * .82;
    const ox = cx + (cw - wa * s) / 2, oy = cy + (ch - wb * s) / 2;
    const mn = [bb.min.x, bb.min.y, bb.min.z];
    let d = '';
    for (let i = 0; i < nseg; i += stride) {
      const k = i * 6;
      const x1 = ox + (pos[k + a] - mn[a]) * s, y1 = oy + wb * s - (pos[k + b] - mn[b]) * s;
      const x2 = ox + (pos[k + 3 + a] - mn[a]) * s, y2 = oy + wb * s - (pos[k + 3 + b] - mn[b]) * s;
      d += `M${x1.toFixed(1)} ${y1.toFixed(1)}L${x2.toFixed(1)} ${y2.toFixed(1)}`;
    }
    out += `<path d="${d}" fill="none" stroke="#6be1ed" stroke-width=".55" opacity=".9"/>`;
    out += `<text x="${cx}" y="${cy + ch + 9}" fill="#8faeb9" font-size="7" font-family="IBM Plex Mono, monospace">${name} · ${fmt(wa * 1000, 0)} × ${fmt(wb * 1000, 0)} mm</text>`;
  });
  out += `<text x="${W - pad}" y="${Hh - 8}" text-anchor="end" fill="#6be1ed" font-size="7" font-family="IBM Plex Mono, monospace">${DATA.parts[pi].pn} · REV 08 · THIRD-ANGLE</text></svg>`;
  return out;
}
function setAdvView(vw) {
  adv.view = vw;
  document.querySelectorAll('[data-advview]').forEach((b) => b.classList.toggle('active', b.dataset.advview === vw));
  $('adv-viewport').hidden = vw !== '3d'; $('adv-bom').hidden = vw !== 'bom'; $('adv-mass').hidden = vw !== 'mass';
  if (vw === 'bom') renderBom(); if (vw === 'mass') renderMass(); if (vw === '3d') resize();
}
function renderBom() {
  const q = $('adv-search').value.trim().toLowerCase(), f = $('adv-filter').value;
  const cols = [['pn', 'PART NO.'], ['name', 'NAME'], ['system', 'SYSTEM'], ['qty', 'QTY'], ['mat', 'MATERIAL'], ['supplier', 'MAKE / BUY'], ['mass_kg', 'UNIT kg'], ['tot', 'TOTAL kg'], ['env', 'ENVELOPE mm']];
  let rows = DATA.parts.map((p, i) => ({ i, p, pn: p.pn, name: p.name, system: p.system, qty: p.qty, mat: DATA.materials[p.material].name, supplier: p.supplier, mass_kg: p.mass_kg, tot: p.mass_kg * p.qty, env: p.dims_mm[0] * p.dims_mm[1] * p.dims_mm[2] }))
    .filter((r) => partMatches(r.p, q, f));
  const [k, dir] = adv.sort;
  rows.sort((a, b) => (a[k] > b[k] ? 1 : a[k] < b[k] ? -1 : 0) * dir);
  const tm = rows.reduce((a, r) => a + r.tot, 0), tq = rows.reduce((a, r) => a + r.qty, 0);
  $('adv-bom').innerHTML = `<div class="bom-summary"><span><b>${rows.length}</b> unique parts</span><span><b>${tq.toLocaleString()}</b> pieces</span><span><b>${fmt(tm, 1)} kg</b> listed mass</span><span>STEP library: <a href="${REPO}cad/rev08" target="_blank" rel="noopener">cad/rev08 ↗</a></span></div>
    <div style="overflow-x:auto"><table class="bom-table"><thead><tr>${cols.map(([c, l]) => `<th data-k="${c}" class="${c === k ? 'sorted' : ''}">${l}${c === k ? (dir > 0 ? ' ▲' : ' ▼') : ''}</th>`).join('')}</tr></thead><tbody>
    ${rows.map((r) => `<tr data-pi="${r.i}"><td class="pn">${r.pn}</td><td>${esc(r.name)}</td><td>${esc(r.system)}</td><td class="num">${r.qty}</td><td>${esc(r.mat)}</td><td>${r.supplier}</td><td class="num">${fmt(r.mass_kg, 3)}</td><td class="num">${fmt(r.tot, 2)}</td><td class="num">${r.p.dims_mm.map((d) => Math.round(d)).join('×')}</td></tr>`).join('')}</tbody></table></div>`;
  $('adv-bom').querySelectorAll('th').forEach((th) => th.addEventListener('click', () => { const c = th.dataset.k; adv.sort = [c, adv.sort[0] === c ? -adv.sort[1] : 1]; renderBom(); }));
  $('adv-bom').querySelectorAll('tbody tr').forEach((tr) => tr.addEventListener('click', () => { setAdvView('3d'); selectPart(+tr.dataset.pi, -1, true); }));
}
function renderMass() {
  const bys = Object.entries(MASS.road.bySys).sort((a, b) => b[1] - a[1]);
  const max = bys[0][1];
  const r = MASS.rotors, xf = (r.FL.x + r.FR.x) / 2, xr = (r.RL.x + r.RR.x) / 2;
  const split = flightModel(FLIGHT);
  $('adv-mass').innerHTML = `<div class="mass-grid"><div>
      <span class="eyebrow">MASS ROLL-UP FROM ${DATA.parts.length} CAD PARTS</span><h3 style="font-weight:500;margin:6px 0 12px">Empty ${fmt(MASS.empty, 0)} kg · gross ${fmt(MASS.road.m, 0)} kg with two occupants</h3>
      <div class="mass-bars">${bys.map(([s, m]) => `<div class="mass-row"><span>${esc(s)}</span><span class="bar"><i style="width:${(m / max * 100).toFixed(1)}%"></i></span><b>${fmt(m, 1)} kg</b></div>`).join('')}</div></div>
    <div class="flight-notes" style="grid-template-columns:1fr">
      <div><h4>Centre of gravity</h4><p>Road: x ${fmt(MASS.road.cg.x * 1000, 0)} mm, y ${fmt(MASS.road.cg.y * 1000, 0)} mm, z ${fmt(MASS.road.cg.z * 1000, 0)} mm.<br>Hover (modules converted): x ${fmt(MASS.hover.cg.x * 1000, 0)} mm, y ${fmt(MASS.hover.cg.y * 1000, 0)} mm.</p></div>
      <div><h4>Rotor centroid vs CG</h4><p>Front rotors at x ${fmt(xf * 1000, 0)} mm, rear at x ${fmt(xr * 1000, 0)} mm. The CG sits ${fmt(Math.abs(MASS.hover.cg.x - (xf + xr) / 2) * 1000, 0)} mm ${MASS.hover.cg.x > (xf + xr) / 2 ? 'forward' : 'aft'} of the rotor centroid, so the ${MASS.hover.cg.x > (xf + xr) / 2 ? 'front' : 'rear'} pair carries ${fmt(Math.max(split.kFront, 1 - split.kFront) * 100, 1)} % of hover thrust. Equal loading is optimal (memo Eq. 44); the imbalance costs ${fmt((split.splitPenalty - 1) * 100, 2)} % hover power.</p></div>
      <div><h4>Against the 2,400 kg design gross mass</h4><p>${MASS.road.m <= 2400 ? 'Within' : 'Over'} the rotor sizing mass by ${fmt(Math.abs(2400 - MASS.road.m), 0)} kg. The largest contributors are the battery, the four fan-wheel modules (tyre, duct, petals) and the structure: these are the mass-reduction targets.</p></div>
    </div></div>`;
}

// ------------------------------------------------------------------ 07 PROPROTOR & FLIGHT DYNAMICS
const RHO = 1.225, G0 = 9.80665;
// Component limits used by the physics checks (from the CAD part specs)
const CELL = { V: 3.6, Ah: 5.0, Wh: 18.0, Acont: 60, Apeak: 100, R: 0.010 };      // high-power 21700 cell
const MOTOR = { peakKW: 320, contKW: 210, peakNm: 900, contNm: 600, eta: 0.955 }, INV_ETA = 0.985, ACCESSORY_KW = 6;
let FLIGHT;
const INPUTS = [
  ['Vehicle', [['mass', 'Gross mass (2 occupants)', 'kg', 1400, 3000, 10], ['battery', 'Battery energy', 'kWh', 40, 160, 1], ['usable', 'Usable state-of-charge window', '', .6, .95, .01],
    ['reserve', 'Landing reserve kept', '', 0, .4, .01]]],
  ['Rotor geometry (CADed baseline)', [['R', 'Tip radius R', 'm', .35, .75, .005], ['B', 'Blades B', '', 3, 10, 1], ['sigma', 'Solidity σ', '', .1, .6, .005],
    ['r0', 'Airfoil start r₀/R', '', .1, .35, .01], ['rm', 'Max-chord station r_m/R', '', .2, 1, .01], ['c2', 'Chord concavity c₂', '', -1.5, 0, .01], ['muD', 'Twist design inflow μ_d', '', 0, .8, .01]]],
  ['Duct', [['etaD', 'Duct effectiveness η_d', '', .3, 1, .01], ['sigd', 'Exit-area ratio σ_d (petals as diffuser)', '', 1, 1.4, .01]]],
  ['Operating point', [['tip', 'Hover tip speed ωR', 'm/s', 150, 300, 1], ['vC', 'Cruise speed', 'km/h', 40, 260, 1], ['rpmC', 'Cruise RPM / hover RPM', '', .6, 1.1, .01],
    ['thC', 'Cruise collective θc (for chart C)', '°', -10, 50, .5]]],
  ['Airframe', [['cda', 'Drag area C_D·A', 'm²', .6, 3, .05], ['clS', 'Body lift C_L·S', 'm²', 0, 4, .05], ['tiltMax', 'Tilt limit', '°', 0, 90, 1]]],
];
function initFlight() {
  const pp = DATA.meta.proprotor;
  FLIGHT = { mass: Math.round(MASS.road.m / 10) * 10, battery: Math.round(MASS.cells * CELL.Wh / 100) / 10, usable: .9, reserve: .2,
    R: pp.R, B: pp.B, sigma: pp.sigma, r0: pp.r0_R, rm: pp.r_m, c2: pp.c2, muD: pp.mu_design, etaD: pp.eta_duct ?? .8, sigd: pp.sigma_d ?? 1.15,
    tip: pp.tip_speed_hover, vC: pp.v_cruise * 3.6, rpmC: pp.cruise_rpm_ratio, thC: 6, cda: 1.6, clS: 1.8, tiltMax: 30, crMin: .05, ctMin: 0 };
  $('flight-inputs').innerHTML = INPUTS.map(([g, list]) => `<h4>${g.toUpperCase()}</h4>` + list.map(([k, l, u, a, b, s]) =>
    `<label class="fin" for="fin-${k}">${l}<output id="fout-${k}"></output><input id="fin-${k}" type="range" min="${a}" max="${b}" step="${s}" value="${FLIGHT[k]}" data-k="${k}" data-u="${u}"></label>`).join('')).join('') +
    `<button class="tl-btn" id="fin-reset">Reset to CADed design</button>`;
  $('flight-inputs').querySelectorAll('input').forEach((i) => i.addEventListener('input', () => { FLIGHT[i.dataset.k] = +i.value; renderFlight(); }));
  $('fin-reset').addEventListener('click', () => { initFlight(); renderFlight(); });
  renderFlight();
}
function chordCoeffs(F) {               // Eq. 26 & 28 (R = 1)
  const c1 = -2 * F.c2 * F.rm;
  const c0 = F.sigma / F.B * Math.PI / (1 - F.r0) - F.c2 * (1 + F.r0 + F.r0 ** 2) / 3 + F.c2 * F.rm * (1 + F.r0);
  return [F.c2, c1, c0];
}
const chordAt = (F, x) => { const [a, b, c] = chordCoeffs(F); return a * x * x + b * x + c; };
const phiDeg = (x, mu) => deg(Math.atan2(mu, x));                        // Eq. 8
const twistDeg = (F, x) => phiDeg(x, F.muD) - phiDeg(.75, F.muD);         // memo §2E, referenced to 3/4 R
const tcAt = (F, x) => { const t = clamp((x - F.r0) / (1 - F.r0)); return .30 + (.08 - .30) * (1 - (1 - t) ** 1.6); };
const stallDeg = (tc) => 14 - 25 * Math.max(0, tc - .12);
// NACA 24xx at Re ~1–2e6: linear lift to stall, blended into flat-plate post-stall; form-factor profile drag
function polar(aDeg, tc) {
  const a = rad(aDeg), a0 = rad(-2), cla = 2 * Math.PI * .92, ast = rad(stallDeg(tc));
  const cd0 = .0065 * (1 + 2 * tc + 60 * tc ** 4), clLin = cla * (a - a0), cdLin = cd0 + .01 * (clLin - .25) ** 2;
  const clP = 1.1 * Math.sin(2 * a), cdP = .02 + 1.8 * Math.sin(a) ** 2;
  if (a > -.2 && a < ast) return [clLin, cdLin];
  if (a >= ast) { const b = Math.min(1, (a - ast) / rad(4)); return [clLin * (1 - b) + clP * b, cdLin * (1 - b) + cdP * b]; }
  return [clP, cdP];
}
// Ducted-rotor BEMT in hover: each annulus balances blade-element thrust against dT = πρv²r dr / σd²; the duct adds η_d(2σd − 1)·T_rotor
function bemt(F, th75, tip, n = 30) {
  const R = F.R, om = tip / R, r0 = F.r0 * R, dr = (R - r0) / n, gap = .97;
  let Tr = 0, Q = 0, Qp = 0, amax = -99, stall = 0; const st = [];
  for (let i = 0; i < n; i++) {
    const r = r0 + (i + .5) * dr, x = r / R, c = chordAt(F, x) * R, th = rad(twistDeg(F, x) + th75), tc = tcAt(F, x);
    let lo = 0, hi = .9 * tip, v = 0, phi = 0, alpha = 0, cl = 0, cd = 0, W2 = 0, dTb = 0;
    for (let k = 0; k < 40; k++) {
      v = (lo + hi) / 2; phi = Math.atan2(v, om * r); alpha = deg(th - phi); [cl, cd] = polar(alpha, tc);
      W2 = v * v + (om * r) ** 2;
      dTb = .5 * RHO * W2 * c * F.B * (cl * Math.cos(phi) - cd * Math.sin(phi)) * dr * gap;
      if (dTb > Math.PI * RHO * v * v * r * dr / F.sigd ** 2) lo = v; else hi = v;
    }
    Tr += dTb;
    Q += .5 * RHO * W2 * c * F.B * (cl * Math.sin(phi) + cd * Math.cos(phi)) * r * dr;
    Qp += .5 * RHO * W2 * c * F.B * cd * Math.cos(phi) * r * dr;
    amax = Math.max(amax, alpha); if (alpha > stallDeg(tc)) stall += 1 / n;
    st.push({ x, alpha, phi: deg(phi), v, dT: dTb / dr });
  }
  const T = Tr * (1 + F.etaD * (2 * F.sigd - 1)), A = Math.PI * R * R, P = Q * om;
  return { T, Tr, Q, P, Pp: Qp * om, om, amax, stall, st, fom: T ** 1.5 / Math.sqrt(4 * RHO * A * F.sigd) / P, CTs: Tr / (RHO * A * tip * tip) / F.sigma };
}
function trimHover(F, Treq, tip) {
  let lo = -5, hi = 45;
  if (bemt(F, hi, tip).T < Treq) return null;
  for (let k = 0; k < 32; k++) { const m = (lo + hi) / 2; if (bemt(F, m, tip).T < Treq) lo = m; else hi = m; }
  const s = bemt(F, hi, tip); s.th = hi; return s;
}
// largest thrust one rotor can make inside the motor power/torque limits and before the blade stalls
function maxThrust(F, tip, limKW) {
  let lo = -5, hi = 45, best = null;
  for (let k = 0; k < 30; k++) {
    const m = (lo + hi) / 2, s = bemt(F, m, tip);
    const ok = s.P / 1e3 <= limKW && s.Q <= MOTOR.peakNm && s.stall <= .15;   // thick root sections may stall first
    if (ok) { lo = m; best = s; } else hi = m;
  }
  return best;
}
// series/parallel pack with cell internal resistance: terminal current for a given electrical power
function pack(Pkw, F) {
  const N = MASS.cells, S = 12 * Math.max(1, Math.round(N / 288)), P = N / S;
  const Voc = S * CELL.V * (F.battery / (N * CELL.Wh / 1000)), Rp = S / P * CELL.R, Pw = Pkw * 1e3;
  const disc = Voc * Voc - 4 * Rp * Pw;
  if (disc < 0) return { ok: false, S, P, Voc, Rp, I: Infinity, Icell: Infinity, C: Infinity, V: 0, eff: 0 };
  const I = (Voc - Math.sqrt(disc)) / (2 * Rp);
  return { ok: true, S, P, Voc, Rp, I, Icell: I / P, C: I / P / CELL.Ah, V: Voc - I * Rp, eff: (Voc - I * Rp) / Voc, heatKW: I * I * Rp / 1e3 };
}
const _fmCache = { key: null, val: null };
function flightModel(F) {                // memoised: BEMT trims are too heavy to redo every animation frame
  const key = JSON.stringify(F);
  if (_fmCache.key === key) return _fmCache.val;
  _fmCache.key = key; _fmCache.val = flightModelRaw(F);
  return _fmCache.val;
}
function flightModelRaw(F) {
  const W = F.mass * G0, A = Math.PI * F.R ** 2, At = 4 * A;
  const om = F.tip / F.R, dl = W / At;
  // CG-based thrust split (memo Eq. 44: equal disk loading is optimal)
  let kFront = .5, splitPenalty = 1;
  if (MASS.rotors) {
    const xf = (MASS.rotors.FL.x + MASS.rotors.FR.x) / 2, xr = (MASS.rotors.RL.x + MASS.rotors.RR.x) / 2;
    kFront = clamp((MASS.hover.cg.x - xr) / (xf - xr));
    splitPenalty = (2 * (kFront / 2) ** 1.5 + 2 * ((1 - kFront) / 2) ** 1.5) / (4 * .25 ** 1.5);
  }
  const Tavg = W / 4, Theavy = W * Math.max(kFront, 1 - kFront) / 2;
  const hv = trimHover(F, Tavg, F.tip), hvHeavy = trimHover(F, Theavy, F.tip);
  const hoverShaftKW = hv ? 4 * hv.P / 1e3 * splitPenalty : Infinity;
  const hoverkW = hoverShaftKW / (MOTOR.eta * INV_ETA) + ACCESSORY_KW;
  const tmax = maxThrust(F, F.tip, MOTOR.peakKW);
  const margin = tmax && hvHeavy ? tmax.T / Theavy : 0;
  // forward flight, calibrated to the BEMT hover split between induced and profile power
  const vh = Math.sqrt(Tavg / (2 * RHO * A));
  const kInd = hv ? (hv.P - hv.Pp) / (Tavg * vh) : 1.2;
  const cdEff = hv ? 8 * hv.Pp / (F.sigma * RHO * A * F.tip ** 3) : .012;
  const powerAt = (V, rpmRatio = F.rpmC) => {
    const q = .5 * RHO * V * V, D = q * F.cda, L = q * F.clS;
    const Tz = Math.max(W - L, 0), T = Math.hypot(D, Tz), tilt = deg(Math.atan2(D, Tz));
    const Ti = T / 4, a = rad(tilt), Vax = V * Math.sin(a), Ved = V * Math.cos(a);
    let vi = Math.sqrt(Ti / (2 * RHO * A));
    for (let k = 0; k < 40; k++) vi = Ti / (2 * RHO * A * Math.max(1e-3, Math.hypot(Ved, Vax + vi)));   // Glauert
    const tipC = F.tip * rpmRatio, mu = Ved / tipC;
    const Pi = 4 * kInd * Ti * vi, Pp = 4 * F.sigma * cdEff / 8 * RHO * A * tipC ** 3 * (1 + 4.65 * mu * mu), Pc = 4 * Ti * Vax;
    const shaft = (Pi + Pp) * splitPenalty + Pc;
    return { V, tilt, T, mu, vi, Pi: Pi / 1e3, Pp: Pp / 1e3, Ppar: D * V / 1e3, Pshaft: shaft / 1e3, P: shaft / 1e3 / (MOTOR.eta * INV_ETA) + ACCESSORY_KW };
  };
  const sweep = []; for (let v = 0; v <= 75; v += 1.5) sweep.push(powerAt(v, v < 1 ? 1 : F.rpmC));
  sweep[0] = { ...sweep[0], P: hoverkW, Pshaft: hoverShaftKW };
  const Vc = F.vC / 3.6, cr = powerAt(Vc);
  const bH = pack(hoverkW, F), bC = pack(cr.P, F);
  const Euse = F.battery * F.usable * (1 - F.reserve);
  const hoverMin = bH.ok ? Euse * bH.eff / hoverkW * 60 : 0, cruiseMin = bC.ok ? Euse * bC.eff / cr.P * 60 : 0;
  const lamH = hv ? hv.st.reduce((a, s) => a + s.v, 0) / hv.st.length / F.tip : .25;
  const lamC = (Vc * Math.sin(rad(cr.tilt)) + cr.vi) / (F.tip * F.rpmC);
  const roots = [chordAt(F, F.r0), chordAt(F, 1)];
  const ductShare = 1 - 1 / (1 + F.etaD * (2 * F.sigd - 1));
  return { W, A, At, om, dl, hv, hvHeavy, tmax, margin, hoverkW, hoverShaftKW, perMotorKW: hvHeavy ? hvHeavy.P / 1e3 : Infinity, perMotorNm: hvHeavy ? hvHeavy.Q : Infinity,
    CTs: hv ? hv.CTs : NaN, fom: hv ? hv.fom : NaN, lamH, lamC, rpmHover: om * 60 / (2 * Math.PI), Mtip: F.tip / 340.3, kFront, splitPenalty, ductShare, kInd, cdEff,
    cr: { ...cr }, sweep, E: Euse, bH, bC, hoverMin, cruiseMin, rangeKm: cruiseMin / 60 * F.vC, roots, installed: 4 * MOTOR.peakKW, cont: 4 * MOTOR.contKW };
}
function physicsChecks(F, M) {
  const cg = MASS.rotors ? Math.abs(MASS.hover.cg.x - ((MASS.rotors.FL.x + MASS.rotors.RL.x) / 2)) * 1000 : 0;
  const c = (ok, mid) => (ok ? 'ok' : mid ? 'mid' : 'bad');
  return [
    ['Hover trim exists', M.hv ? `θ.75 = ${fmt(M.hv.th, 1)}°` : 'none', 'BEMT finds a collective that carries the weight', c(!!M.hv, false), 'If no collective lifts the car, the rotor is too small or the mass too high.'],
    ['Blade loading C_T/σ', fmt(M.CTs, 3), '≤ 0.12 (memo §2B)', c(M.CTs <= .12, M.CTs <= .13), 'Above ~0.12 the sections run near stall everywhere.'],
    ['Stall margin', `α_max ${fmt(M.hv?.amax, 1)}°`, '≤ 12° (stall 12–14°)', c(M.hv && M.hv.amax <= 12 && M.hv.stall === 0, M.hv && M.hv.amax <= 14), 'Highest section angle of attack in the hover trim.'],
    ['Tip Mach number', fmt(M.Mtip, 2), '≤ 0.72', c(M.Mtip <= .72, M.Mtip <= .8), 'Compressibility drag and noise rise quickly above ~0.75.'],
    ['Motor power, most-loaded rotor', `${fmt(M.perMotorKW, 0)} kW`, `${MOTOR.contKW} kW cont / ${MOTOR.peakKW} kW peak`, c(M.perMotorKW <= MOTOR.contKW, M.perMotorKW <= MOTOR.peakKW), 'Above continuous rating, hover is limited by motor temperature.'],
    ['Motor torque, most-loaded rotor', `${fmt(M.perMotorNm, 0)} N·m`, `${MOTOR.contNm} cont / ${MOTOR.peakNm} peak`, c(M.perMotorNm <= MOTOR.contNm, M.perMotorNm <= MOTOR.peakNm), 'Shaft torque Q = P / ω from BEMT.'],
    ['Thrust margin for control', `${fmt(M.margin, 2)} × hover`, '≥ 1.25', c(M.margin >= 1.25, M.margin >= 1.1), 'Spare thrust on the heaviest-loaded rotor at peak motor power; needed to hold attitude in gusts.'],
    ['Cell current in hover', M.bH.ok ? `${fmt(M.bH.Icell, 0)} A (${fmt(M.bH.C, 1)}C)` : 'pack cannot deliver', `≤ ${CELL.Acont} A cont`, c(M.bH.ok && M.bH.Icell <= CELL.Acont, M.bH.ok && M.bH.Icell <= CELL.Apeak), `${M.bH.S}s ${fmt(M.bH.P, 0)}p pack; current from P = V·I with internal resistance.`],
    ['Pack voltage under hover load', M.bH.ok ? `${fmt(M.bH.V, 0)} V of ${fmt(M.bH.Voc, 0)} V` : '—', '≥ 80 % of open-circuit', c(M.bH.ok && M.bH.eff >= .8, M.bH.ok && M.bH.eff >= .7), `Battery heat ${fmt(M.bH.heatKW, 0)} kW at this load.`],
    ['Hover endurance (after reserve)', `${fmt(M.hoverMin, 1)} min`, '≥ 4 min for take-off + landing', c(M.hoverMin >= 4, M.hoverMin >= 2), `${fmt(F.reserve * 100, 0)} % kept for landing.`],
    ['CG over rotor centroid', `${fmt(cg, 0)} mm off`, '≤ 50 mm', c(cg <= 50, cg <= 120), `Thrust split ${fmt(M.kFront * 100, 1)} / ${fmt((1 - M.kFront) * 100, 1)} % front / rear.`],
    ['Cruise tilt within gimbal limit', `${fmt(M.cr.tilt, 1)}°`, `≤ ${F.tiltMax}°`, c(M.cr.tilt <= F.tiltMax, M.cr.tilt <= F.tiltMax + 5), 'Rotor axes must tilt enough to balance drag.'],
    ['Cruise power vs continuous rating', `${fmt(M.cr.Pshaft, 0)} kW`, `≤ ${M.cont} kW`, c(M.cr.Pshaft <= M.cont, M.cr.Pshaft <= M.installed), 'Total shaft power at the cruise speed.'],
    ['Model sanity: figure of merit', fmt(M.fom, 3), '0.60 – 0.82 is typical', c(M.fom >= .6 && M.fom <= .82, M.fom >= .5 && M.fom <= .88), 'Computed, not assumed. Outside this band, check the inputs.'],
    ['Seats', '2 occupants', '2 by design', 'ok', 'Payload 180 kg included in the gross mass.'],
  ];
}
function renderFlight() {
  if (!FLIGHT) return;
  const F = FLIGHT, M = flightModel(F);
  for (const g of INPUTS) for (const [k, , u, , , s] of g[1]) { const o = $('fout-' + k); if (o) o.textContent = `${fmt(F[k], s >= 1 ? 0 : s >= .1 ? 1 : s >= .01 ? 2 : 3)}${u ? ' ' + u : ''}`; }
  const checks = physicsChecks(F, M);
  const nBad = checks.filter((c) => c[3] === 'bad').length, nMid = checks.filter((c) => c[3] === 'mid').length;
  const st = (v, okMax, midMax) => (v <= okMax ? 'ok' : v <= midMax ? 'mid' : 'bad');
  const kpis = [
    ['PHYSICS CHECKS', `${checks.length - nBad - nMid} / ${checks.length} pass`, `${nMid} marginal · ${nBad} failing`, nBad ? 'bad' : nMid ? 'mid' : 'ok'],
    ['DISK LOADING', `${fmt(M.dl / 47.88, 0)} lb/ft²`, `${fmt(M.dl, 0)} Pa · Eq. 2`, st(M.dl / 47.88, 60, 120)],
    ['HOVER POWER (BEMT)', `${fmt(M.hoverkW, 0)} kW`, `${fmt(M.hoverShaftKW, 0)} kW shaft · FoM ${fmt(M.fom, 3)}`, st(M.perMotorKW, MOTOR.contKW, MOTOR.peakKW)],
    ['HOVER COLLECTIVE', M.hv ? `${fmt(M.hv.th, 1)}°` : '—', `${fmt(M.rpmHover, 0)} rpm · M${fmt(M.Mtip, 2)}`, M.hv ? 'ok' : 'bad'],
    ['THRUST MARGIN', `${fmt(M.margin, 2)} ×`, 'peak power, heaviest rotor', st(-M.margin, -1.25, -1.1)],
    ['HOVER ENDURANCE', `${fmt(M.hoverMin, 1)} min`, `${fmt(M.E, 0)} kWh after reserve`, M.hoverMin >= 4 ? 'ok' : M.hoverMin >= 2 ? 'mid' : 'bad'],
    ['CRUISE POWER / RANGE', `${fmt(M.cr.P, 0)} kW · ${fmt(M.rangeKm, 0)} km`, `${fmt(M.cruiseMin, 1)} min at ${fmt(F.vC, 0)} km/h`, M.rangeKm >= 40 ? 'ok' : M.rangeKm >= 15 ? 'mid' : 'bad'],
    ['CELL CURRENT (HOVER)', M.bH.ok ? `${fmt(M.bH.Icell, 0)} A` : '—', `${fmt(M.bH.C, 1)}C · limit ${CELL.Acont} A`, M.bH.ok ? st(M.bH.Icell, CELL.Acont, CELL.Apeak) : 'bad'],
    ['CRUISE μ (EDGEWISE)', fmt(M.cr.mu, 3), 'Eq. 7 · practical μ < 0.7', st(M.cr.mu, .5, .7)],
    ['ROOT / TIP CHORD', `${fmt(M.roots[0] * F.R * 1000, 0)} / ${fmt(M.roots[1] * F.R * 1000, 0)} mm`, 'Eq. 23, 28', M.roots[0] >= F.crMin && M.roots[1] >= F.ctMin ? 'ok' : 'bad'],
  ];
  $('flight-kpis').innerHTML = kpis.map(([k, v, s, c]) => `<div class="kpi ${c}"><small>${k}</small><b>${v}</b><span>${s}</span></div>`).join('') +
    `<div class="checks"><table class="checks-table"><thead><tr><th>Check</th><th>Value</th><th>Limit</th><th>Status</th><th>Why it matters</th></tr></thead><tbody>${checks.map(([n, v, l, s, w]) =>
      `<tr class="${s}"><td>${esc(n)}</td><td class="num">${esc(v)}</td><td>${esc(l)}</td><td><span class="pill ${s}">${s === 'ok' ? '● Pass' : s === 'mid' ? '▲ Marginal' : '■ Fail'}</span></td><td>${esc(w)}</td></tr>`).join('')}</tbody></table></div>`;
  const C = [];
  const xs = []; for (let x = F.r0; x <= 1.0001; x += .02) xs.push(+x.toFixed(3));
  if (M.hv) C.push(lineChart('Spanwise thrust loading in hover (BEMT)', 'blade-element momentum', `Thrust per metre of span along one blade at the trimmed collective (${fmt(M.hv.th, 1)}°). The duct lip adds ${fmt(M.ductShare * 100, 0)} % on top of the rotor.`,
    { x: [0, 1], y: [0, Math.ceil(Math.max(...M.hv.st.map((s) => s.dT / F.B)) / 1000 + .5) * 1000], xl: 'r/R', yl: 'N per m per blade', series: [{ name: 'dT/dr', c: 'var(--s1)', pts: M.hv.st.map((s) => [s.x, s.dT / F.B]) }] }));
  C.push(lineChart('Inflow angle across the blade', 'Fig. 5a · Eq. 8', 'Inflow angle φ = atan(μR/r). The hover curve is the BEMT solution, including induced inflow, which is large because of the high disk loading.',
    { x: [0, 1], y: [0, 90], xl: 'r/R', yl: 'φ, deg', series: [
      { name: 'Hover (BEMT)', c: 'var(--s1)', pts: M.hv ? M.hv.st.map((s) => [s.x, s.phi]) : [] },
      { name: `Cruise λ = ${fmt(M.lamC, 2)}`, c: 'var(--s2)', pts: xs.map((x) => [x, phiDeg(x, M.lamC)]) },
      { name: 'μ = 0.1 / 0.5 / 0.9', c: 'var(--muted)', dash: '3 3', pts: xs.map((x) => [x, phiDeg(x, .1)]) },
      { name: '', c: 'var(--muted)', dash: '3 3', pts: xs.map((x) => [x, phiDeg(x, .5)]) },
      { name: '', c: 'var(--muted)', dash: '3 3', pts: xs.map((x) => [x, phiDeg(x, .9)]) }] }));
  const mus = []; for (let m = .05; m <= .9001; m += .025) mus.push(+m.toFixed(3));
  C.push(lineChart('Inflow at ¾R and root-to-tip change', 'Fig. 5b · Eq. 32–33', `Δφ peaks at μ = √(r₀/R) = ${fmt(Math.sqrt(F.r0), 2)}. A large Δφ means no single twist suits the whole operating range.`,
    { x: [0, .9], y: [0, 60], xl: 'advance / inflow ratio μ', yl: 'deg', vline: [Math.sqrt(F.r0), '√r₀'], series: [
      { name: 'φ at r/R = 0.75', c: 'var(--s1)', pts: mus.map((m) => [m, phiDeg(.75, m)]) },
      { name: 'Δφ root → tip', c: 'var(--s2)', pts: mus.map((m) => [m, phiDeg(F.r0, m) - phiDeg(1, m)]) }] }));
  C.push(lineChart('Section angle of attack, hover vs cruise', 'Fig. 8 · Eq. 12', 'Hover: α from the BEMT trim. Cruise: α(r) = β(r) − φ(r) + θc with the chosen collective. Hatched: stall beyond ±15°; below about −2° the cambered sections stop lifting.',
    { x: [0, 1], y: [-20, 40], xl: 'r/R', yl: 'α, deg', zones: [[15, 40, 'Stalled flow'], [-15, -2, 'Negative thrust'], [-20, -15, '']], series: [
      { name: M.hv ? `Hover, trimmed θc = ${fmt(M.hv.th, 1)}°` : 'Hover (no trim)', c: 'var(--s1)', pts: M.hv ? M.hv.st.map((s) => [s.x, s.alpha]) : [] },
      { name: `Cruise, θc = ${fmt(F.thC, 1)}°`, c: 'var(--s2)', pts: xs.map((x) => [x, twistDeg(F, x) - phiDeg(x, M.lamC) + F.thC]) }] }));
  C.push(lineChart('Blade chord distribution', 'Eq. 23, 26, 28', `Quadratic chord with maximum at r_m/R = ${fmt(F.rm, 2)} meeting σ = ${fmt(F.sigma, 3)} with B = ${F.B}. Filled band is the blade planform (chord centred on the pitch axis).`,
    { x: [0, 1], y: [-.3, .3], xl: 'r/R', yl: 'c/R (planform ±c/2)', band: xs.map((x) => [x, chordAt(F, x) / 2]), series: [
      { name: 'c(r)/R', c: 'var(--s1)', pts: xs.map((x) => [x, chordAt(F, x)]) }], yTicks: [-.3, -.2, -.1, 0, .1, .2, .3] }));
  C.push(lineChart('Geometric twist', 'memo §2E', `β(r) aligned with the design inflow ratio μ_d = ${fmt(F.muD, 2)} and referenced to zero at ¾R. Collective is added on top (Eq. 9).`,
    { x: [0, 1], y: [-10, 40], xl: 'r/R', yl: 'β, deg', series: [{ name: 'β(r)', c: 'var(--s3)', pts: xs.map((x) => [x, twistDeg(F, x)]) }] }));
  C.push(solutionSpace(F));
  C.push(polarChart(M.cr.mu));
  C.push(lineChart('Electrical power vs airspeed', 'BEMT-calibrated momentum + profile + parasite', `Induced and profile parts are calibrated to the BEMT hover solution (induced factor ${fmt(M.kInd, 2)}, effective C_d ${fmt(M.cdEff, 4)}). Rotor axes tilt to balance drag; cruise at ${fmt(F.vC, 0)} km/h needs ${fmt(M.cr.tilt, 1)}° tilt.`,
    { x: [0, 270], y: [0, Math.max(1200, Math.min(3000, M.hoverkW * 1.15))], xl: 'airspeed, km/h', yl: 'kW', vline: [F.vC, 'cruise'], hlines: [[M.installed, 'motor peak'], [M.cont, 'continuous']], series: [
      { name: 'Total electrical', c: 'var(--s1)', pts: M.sweep.map((s) => [s.V * 3.6, s.P]) },
      { name: 'Induced (shaft)', c: 'var(--s2)', pts: M.sweep.map((s) => [s.V * 3.6, s.Pi]) },
      { name: 'Profile (shaft)', c: 'var(--s3)', pts: M.sweep.map((s) => [s.V * 3.6, s.Pp]) },
      { name: 'Parasite', c: 'var(--s4)', pts: M.sweep.map((s) => [s.V * 3.6, s.Ppar]) }] }));
  const T = M.cr.T, Vc = F.vC / 3.6, areas = []; for (let a = 1; a <= 30.01; a += .5) areas.push(a);
  const etaP = (Aa) => 2 * Vc / (Vc + Math.sqrt(2 * T / (RHO * Aa) + Vc * Vc));
  C.push(lineChart('Ideal propulsive efficiency vs disk area', 'Eq. 3, 45', `Froude efficiency at the cruise thrust (${fmt(T / 1000, 1)} kN) and ${fmt(F.vC, 0)} km/h. Adding area has diminishing returns; the fan-wheels give ${fmt(M.At, 2)} m².`,
    { x: [0, 30], y: [0, 1], xl: 'total disk area, m²', yl: 'η_p', vline: [M.At, 'Rev 08.1'], series: [{ name: 'η_p (ideal, upper bound)', c: 'var(--s1)', pts: areas.map((a) => [a, etaP(a)]) }] }));
  $('flight-charts').innerHTML = C.join('');
  wireTips();
  const r = MASS.rotors, arms = ['FL', 'FR', 'RL', 'RR'].map((k) => [k, r[k].x - MASS.hover.cg.x, r[k].z - MASS.hover.cg.z]);
  const kq = M.hv ? M.hv.Q / (M.W / 4) : 0;   // reaction torque per unit thrust, m
  const spin = { FL: 1, FR: -1, RL: -1, RR: 1 };
  $('flight-notes').innerHTML = `
    <div><h4>Findings for the Rev 08.1 design</h4><ul>
      <li>Bigger rotors (R ${fmt(F.R * 1000, 0)} mm, ${fmt(M.At, 2)} m² total) and a lighter car bring disk loading to ${fmt(M.dl / 47.88, 0)} lb/ft². That is still several times an open-proprotor eVTOL, so hover power dominates.</li>
      <li>BEMT trims hover at θ.75 = ${M.hv ? fmt(M.hv.th, 1) : '—'}° with a computed figure of merit of ${fmt(M.fom, 3)}; ${fmt(M.hoverkW, 0)} kW electrical. The open petals form a diffuser (σ_d ${fmt(F.sigd, 2)}), so the duct carries ${fmt(M.ductShare * 100, 0)} % of the thrust.</li>
      <li>High-power 21700 cells keep hover current at ${M.bH.ok ? fmt(M.bH.Icell, 0) : '—'} A per cell (${fmt(M.bH.C, 1)}C). The old 4680 pack would have needed about 12C from cells rated near 3–5C.</li>
      <li>Endurance is ${fmt(M.hoverMin, 1)} min hovering or ${fmt(M.rangeKm, 0)} km at cruise after a ${fmt(F.reserve * 100, 0)} % reserve: a short-hop vehicle. A deployable wing (memo's axial-cruise case) is the next big lever.</li></ul></div>
    <div><h4>Hover control allocation</h4><p>Rows give force and moments per newton of each rotor's thrust (moment arms from the hover CG; yaw from BEMT reaction torque, k_Q ≈ ${fmt(kq * 1000, 1)} mm).</p>
      <div style="overflow-x:auto"><table class="alloc"><tr><th></th>${arms.map((a) => `<th>${a[0]}</th>`).join('')}</tr>
      <tr><th>Thrust</th>${arms.map(() => '<td>1</td>').join('')}</tr>
      <tr><th>Roll (m)</th>${arms.map((a) => `<td>${fmt(a[2], 3)}</td>`).join('')}</tr>
      <tr><th>Pitch (m)</th>${arms.map((a) => `<td>${fmt(-a[1], 3)}</td>`).join('')}</tr>
      <tr><th>Yaw (m)</th>${arms.map((a) => `<td>${fmt(spin[a[0]] * kq, 4)}</td>`).join('')}</tr></table></div>
      <p>Four rotors and four control axes: a single rotor failure leaves the vehicle under-actuated, so the ballistic parachute covers that case.</p></div>
    <div><h4>Method and limits</h4><p>Hover: blade-element momentum theory over 30 annuli of the CADed blade (chord, twist, thickness), NACA 24xx lift and drag with stall, ducted momentum with exit-area ratio σ_d and duct effectiveness η_d, 3 % tip-gap loss; collective trimmed to the CG-weighted thrust. Forward flight: Glauert inflow and advance-ratio profile power calibrated to that hover solution. Battery: ${M.bH.S}s pack with 10 mΩ cells. Not modelled: aeroelasticity, duct-lip separation in crosswind, acoustics, transients and thermal time constants.</p></div>`;
}
// --- chart helpers (SVG, one y-axis, theme tokens, hover crosshair)
let chartId = 0;
function lineChart(title, ref, desc, o) {
  const W = 460, Hh = 250, L = 46, R = 14, T = 12, B = 34, id = 'ch' + (chartId++);
  const sx = (x) => L + (x - o.x[0]) / (o.x[1] - o.x[0]) * (W - L - R), sy = (y) => T + (1 - (y - o.y[0]) / (o.y[1] - o.y[0])) * (Hh - T - B);
  const ticks = (a, b, n = 5) => { const st = niceStep((b - a) / n); const out = []; for (let v = Math.ceil(a / st) * st; v <= b + 1e-9; v += st) out.push(+v.toFixed(6)); return out; };
  let s = `<svg viewBox="0 0 ${W} ${Hh}" id="${id}" data-chart='${esc(JSON.stringify({ x: o.x, y: o.y, L, R, T, B, W, H: Hh, s: o.series.filter((q) => q.name).map((q) => ({ n: q.name, c: q.c, p: q.pts.filter((_, i) => i % 1 === 0) })) }))}' role="img" aria-label="${esc(title)}">`;
  s += `<defs><pattern id="hatch-${id}" width="7" height="7" patternUnits="userSpaceOnUse" patternTransform="rotate(45)"><rect width="7" height="7" fill="var(--surface)"/><line x1="0" y1="0" x2="0" y2="7" stroke="var(--muted)" stroke-width="1" opacity=".5"/></pattern></defs>`;
  for (const z of o.zones || []) { const y1 = sy(Math.min(z[1], o.y[1])), y0 = sy(Math.max(z[0], o.y[0])); s += `<rect x="${L}" y="${y1}" width="${W - L - R}" height="${y0 - y1}" fill="url(#hatch-${id})" opacity=".8"/>${z[2] ? `<text x="${W - R - 4}" y="${y1 + 12}" text-anchor="end">${z[2]}</text>` : ''}`; }
  for (const v of o.yTicks || ticks(o.y[0], o.y[1])) s += `<line x1="${L}" x2="${W - R}" y1="${sy(v)}" y2="${sy(v)}" stroke="var(--line)" stroke-width="1"/><text x="${L - 6}" y="${sy(v) + 3}" text-anchor="end">${+v.toFixed(3)}</text>`;
  for (const v of ticks(o.x[0], o.x[1])) s += `<text x="${sx(v)}" y="${Hh - B + 14}" text-anchor="middle">${+v.toFixed(3)}</text>`;
  s += `<line x1="${L}" x2="${W - R}" y1="${Hh - B}" y2="${Hh - B}" stroke="var(--muted)" stroke-width="1" opacity=".5"/>`;
  s += `<text x="${(L + W - R) / 2}" y="${Hh - 4}" text-anchor="middle">${o.xl}</text><text transform="translate(11 ${(T + Hh - B) / 2}) rotate(-90)" text-anchor="middle">${o.yl}</text>`;
  if (o.band) { const top = o.band.map(([x, y]) => `${sx(x)},${sy(y)}`), bot = o.band.slice().reverse().map(([x, y]) => `${sx(x)},${sy(-y)}`); s += `<polygon points="${top.concat(bot).join(' ')}" fill="var(--s1)" opacity=".18"/>`; }
  for (const [y, lab] of o.hlines || []) if (y <= o.y[1]) s += `<line x1="${L}" x2="${W - R}" y1="${sy(y)}" y2="${sy(y)}" stroke="var(--amber)" stroke-dasharray="5 4" stroke-width="1"/><text x="${L + 4}" y="${sy(y) - 4}">${lab} ${fmt(y, 0)}</text>`;
  if (o.vline && o.vline[0] >= o.x[0] && o.vline[0] <= o.x[1]) s += `<line x1="${sx(o.vline[0])}" x2="${sx(o.vline[0])}" y1="${T}" y2="${Hh - B}" stroke="var(--text)" stroke-dasharray="2 3" opacity=".6"/><text x="${sx(o.vline[0]) + 4}" y="${T + 10}" class="ttl">${o.vline[1]}</text>`;
  for (const q of o.series) { const d = q.pts.filter(([x, y]) => Number.isFinite(y)).map(([x, y], i) => `${i ? 'L' : 'M'}${sx(x).toFixed(1)} ${sy(clamp(y, o.y[0] - (o.y[1] - o.y[0]) * .05, o.y[1] + (o.y[1] - o.y[0]) * .05)).toFixed(1)}`).join(''); s += `<path d="${d}" fill="none" stroke="${q.c}" stroke-width="2" ${q.dash ? `stroke-dasharray="${q.dash}"` : ''} stroke-linejoin="round" clip-path="url(#clip-${id})"/>`; }
  s += `<clipPath id="clip-${id}"><rect x="${L}" y="${T}" width="${W - L - R}" height="${Hh - T - B}"/></clipPath>`;
  s += `<g class="xh" style="display:none"><line y1="${T}" y2="${Hh - B}" stroke="var(--text)" opacity=".35"/></g></svg>`;
  const legend = o.series.filter((q) => q.name).map((q) => `<span><i style="background:${q.c}"></i>${esc(q.name)}</span>`).join('');
  return `<article class="chart-card"><span class="ref">${esc(ref)}</span><h4>${esc(title)}</h4>${o.series.filter((q) => q.name).length > 1 ? `<div class="legend">${legend}</div>` : ''}${s}<p>${desc}</p></article>`;
}
function niceStep(raw) { const p = 10 ** Math.floor(Math.log10(raw)); const f = raw / p; return (f < 1.5 ? 1 : f < 3.5 ? 2 : f < 7.5 ? 5 : 10) * p; }
function wireTips() {
  document.querySelectorAll('#flight-charts svg[data-chart]').forEach((svg) => {
    const d = JSON.parse(svg.dataset.chart), card = svg.closest('.chart-card');
    let tip = card.querySelector('.chart-tip'); if (!tip) { tip = document.createElement('div'); tip.className = 'chart-tip'; tip.hidden = true; card.appendChild(tip); }
    const xh = svg.querySelector('.xh');
    svg.addEventListener('pointermove', (e) => {
      const r = svg.getBoundingClientRect(), px = (e.clientX - r.left) / r.width * d.W;
      if (px < d.L || px > d.W - d.R) { tip.hidden = true; xh.style.display = 'none'; return; }
      const x = d.x[0] + (px - d.L) / (d.W - d.L - d.R) * (d.x[1] - d.x[0]);
      const rows = d.s.map((q) => { let best = q.p[0]; for (const p of q.p) if (Math.abs(p[0] - x) < Math.abs(best[0] - x)) best = p; return [q, best]; });
      xh.style.display = ''; xh.querySelector('line').setAttribute('x1', px); xh.querySelector('line').setAttribute('x2', px);
      tip.innerHTML = `x = ${fmt(rows[0][1][0], 2)}<br>` + rows.map(([q, p]) => `<span style="color:${q.c}">■</span> ${esc(q.n)}: ${fmt(p[1], 2)}`).join('<br>');
      tip.hidden = false;
      const cr = card.getBoundingClientRect();
      tip.style.left = Math.min(cr.width - 190, e.clientX - cr.left + 14) + 'px'; tip.style.top = (e.clientY - cr.top + 10) + 'px';
    });
    svg.addEventListener('pointerleave', () => { tip.hidden = true; xh.style.display = 'none'; });
  });
}
function solutionSpace(F) {
  const W = 460, Hh = 250, L = 46, R = 14, T = 12, B = 34;
  const sx = (x) => L + x * (W - L - R), sy = (y) => T + (-y / 1.5) * (Hh - T - B);
  let cells = '';
  const nx = 60, ny = 40;
  for (let i = 0; i < nx; i++) for (let j = 0; j < ny; j++) {
    const rm = (i + .5) / nx, c2 = -(j + .5) / ny * 1.5;
    if (rm < F.r0) continue;
    const G = { ...F, rm, c2 };
    const ok = chordAt(G, F.r0) >= F.crMin && chordAt(G, 1) >= F.ctMin && chordAt(G, rm) > 0;
    if (ok) cells += `<rect x="${sx(i / nx).toFixed(1)}" y="${sy(-j / ny * 1.5).toFixed(1)}" width="${((W - L - R) / nx + .4).toFixed(1)}" height="${((Hh - T - B) / ny + .4).toFixed(1)}" fill="var(--s1)" opacity=".28"/>`;
  }
  let s = `<svg viewBox="0 0 ${W} ${Hh}" role="img" aria-label="Quadratic chord solution space">${cells}`;
  for (const v of [0, -.3, -.6, -.9, -1.2, -1.5]) s += `<line x1="${L}" x2="${W - R}" y1="${sy(v)}" y2="${sy(v)}" stroke="var(--line)"/><text x="${L - 6}" y="${sy(v) + 3}" text-anchor="end">${v}</text>`;
  for (const v of [0, .2, .4, .6, .8, 1]) s += `<text x="${sx(v)}" y="${Hh - B + 14}" text-anchor="middle">${v}</text>`;
  s += `<circle cx="${sx(F.rm)}" cy="${sy(F.c2)}" r="5" fill="var(--amber)" stroke="var(--surface)" stroke-width="2"/><text x="${sx(F.rm) + 9}" y="${sy(F.c2) + 4}" class="ttl">Rev 08 blade</text>`;
  s += `<text x="${(L + W - R) / 2}" y="${Hh - 4}" text-anchor="middle">radial station of maximum chord r_m/R</text><text transform="translate(11 ${(T + Hh - B) / 2}) rotate(-90)" text-anchor="middle">c₂</text></svg>`;
  return `<article class="chart-card"><span class="ref">Fig. 12 · Eq. 26–28</span><h4>Quadratic chord solution space</h4>${s}<p>Shaded: (r_m, c₂) pairs that meet σ = ${fmt(F.sigma, 3)} with root chord ≥ ${F.crMin}R and tip chord ≥ ${F.ctMin}R (the memo's example constraints). The marker is the CADed blade.</p></article>`;
}
function polarChart(mu) {
  // memo Fig. 3 convention: freestream from the top, counter-clockwise rotor; azimuth ψ from the right-hand side
  const W = 300, c = 150, Rr = 120, top = 1 + mu;
  let cells = '';
  const col = (v) => {
    if (v < 0) return 'rgb(224,103,90)';
    const t = clamp(v / top), a = [10, 30, 42], b = [140, 236, 245];
    return `rgb(${a.map((x, i) => Math.round(x + (b[i] - x) * t)).join(',')})`;
  };
  for (let i = 0; i < 18; i++) for (let j = 0; j < 72; j++) {
    const r0 = i / 18, r1 = (i + 1) / 18, p0 = j / 72 * 2 * Math.PI, p1 = (j + 1) / 72 * 2 * Math.PI;
    const v = (r0 + r1) / 2 + mu * Math.cos((p0 + p1) / 2);        // V⊥LE / (ωR) = r/R + μ cos ψ
    const P = (r, p) => `${(c + Rr * r * Math.cos(p)).toFixed(1)},${(c - Rr * r * Math.sin(p)).toFixed(1)}`;
    cells += `<polygon points="${P(r0, p0)} ${P(r1, p0)} ${P(r1, p1)} ${P(r0, p1)}" fill="${col(v)}" stroke="${col(v)}" stroke-width=".3"/>`;
  }
  const rev = mu > 0 ? `<circle cx="${c - Rr * mu / 2}" cy="${c}" r="${Rr * mu / 2}" fill="none" stroke="var(--text)" stroke-dasharray="4 3"/>` : '';
  const s = `<svg viewBox="0 -8 ${W} 300" role="img" aria-label="Edgewise velocity polar">${cells}${rev}
    <line x1="${c}" y1="-6" x2="${c}" y2="22" stroke="var(--text)"/><path d="M${c - 4} 16 L${c} 24 L${c + 4} 16 Z" fill="var(--text)"/><text x="${c + 8}" y="8" class="ttl">V∞</text>
    <text x="${c - Rr - 4}" y="${c + 4}" text-anchor="end">retreating</text><text x="${c + Rr + 4}" y="${c + 4}">advancing</text>
    <text x="${c}" y="288" text-anchor="middle">dashed: reverse-flow circle, diameter μR</text></svg>`;
  return `<article class="chart-card"><span class="ref">Fig. 3 · Eq. 7</span><h4>Edgewise flow at cruise, μ = ${fmt(mu, 3)}</h4>${s}<p>Velocity normal to the leading edge over tip speed, r/R + μ cos ψ: dark near zero, bright at the advancing tip, red where the flow reverses on the retreating side.</p></article>`;
}

// ------------------------------------------------------------------ REGION EXPLORER (click a region: it flies in and explodes)
const REGIONS = [
  { id: 'FL', name: 'Fan-wheel FL', long: 'Fan-wheel / proprotor corner, front port', kind: 'fan' },
  { id: 'FR', name: 'Fan-wheel FR', long: 'Fan-wheel / proprotor corner, front starboard', kind: 'fan' },
  { id: 'RL', name: 'Fan-wheel RL', long: 'Fan-wheel / proprotor corner, rear port', kind: 'fan' },
  { id: 'RR', name: 'Fan-wheel RR', long: 'Fan-wheel / proprotor corner, rear starboard', kind: 'fan' },
  { id: 'battery', name: 'Battery', long: 'Battery pack · 19 modules', kind: 'grid' },
  { id: 'power', name: 'Power electronics', long: 'HV distribution & inverters', kind: 'radial' },
  { id: 'cockpit', name: 'Cockpit', long: 'Cockpit, controls & restraints', kind: 'radial' },
  { id: 'avionics', name: 'Avionics', long: 'Flight control, sensors & harness', kind: 'radial' },
  { id: 'thermal', name: 'Thermal', long: 'Thermal management', kind: 'radial' },
  { id: 'legs', name: 'Landing legs', long: 'Landing legs ×4', kind: 'radial' },
  { id: 'emergency', name: 'Parachute', long: 'Ballistic parachute & fire suppression', kind: 'radial' },
  { id: 'structure', name: 'Structure', long: 'Carbon tub, subframes & road chassis', kind: 'radial' },
  { id: 'body', name: 'Body', long: 'Body panels, doors, glazing & lighting', kind: 'shell' },
];
const REGION_BY_ID = Object.fromEntries(REGIONS.map((r) => [r.id, r]));
const FOCUS = { region: null, view: null, hover: null, f: 0, target: 0, fly: null, labels: [], rec: null, briefT: 0 };
function topNode(nid) {
  let n = NODE[nid];
  while (n.def.parent >= 0) {
    const p = NODE[n.def.parent];
    if (p.def.id === 'vehicle' || p.def.id === 'chassis') return n.def.id === 'chassis_R' ? 'chassis' : n.def.id;
    n = p;
  }
  return n.def.id;
}
function assignRegions() {
  for (const r of OBJS) {
    const top = topNode(r.ni), sys = r.sys;
    let id = 'structure';
    if (/^corner_/.test(top)) id = top.slice(7);
    else if (/^leg_/.test(top)) id = 'legs';
    else if (top === 'body' || /^door_/.test(top)) id = 'body';
    else if (top === 'power' || /^bmod_/.test(top)) id = sys === '210' ? 'power' : 'battery';
    else if (top === 'cockpit') id = 'cockpit';
    else if (top === 'avionics') id = 'avionics';
    else if (top === 'thermal') id = 'thermal';
    else if (top === 'emergency' || top === 'emergency_R') id = 'emergency';
    r.region = id;
  }
}
// axial slot (metres along the module axle, + outboard / up in flight) for the fan-wheel exploded view
function fanSlot(rec) {
  const p = PARTS[rec.pi].def, nid = NODE[rec.ni].def.id, a = p.assy;
  if (/^petO_/.test(nid)) return 1.15;
  if (/^petI_/.test(nid)) return -1.25;
  if (a === 'Road tyre') return 0.78;
  if (a === 'Duct & rim') return 0.42;
  if (p.system.startsWith('330')) return /^blade_/.test(nid) ? 0.12 : 0.0;
  if (a === 'Stator') return -0.28;
  if (a === 'Clutch') return -0.46;
  if (p.system.startsWith('320') && a === 'Motor') return -0.64;
  if (a === 'Brakes' || p.system.startsWith('360')) return -0.84;
  if (a === 'Locks') return -0.98;
  if (a === 'Spindle') return -1.06;
  return null;   // yoke, tilt gimbal, suspension, fender cap: pushed radially away instead
}
function recWorldCentroid(rec, out) {
  const P = PARTS[rec.pi], ctr = new THREE.Vector3(), m = new THREE.Matrix4(), v = new THREE.Vector3();
  if (P.def.com) ctr.set(...P.def.com); else P.geo.boundingBox.getCenter(ctr);
  out.set(0, 0, 0);
  rec.obj.updateWorldMatrix(true, false);
  for (const mt of rec.mtx) out.add(v.copy(ctr).applyMatrix4(m.multiplyMatrices(rec.obj.matrixWorld, mt)));
  return out.divideScalar(rec.mtx.length);
}
function recLocalCentroid(rec) {
  const P = PARTS[rec.pi], ctr = new THREE.Vector3(), v = new THREE.Vector3(), out = new THREE.Vector3();
  if (P.def.com) ctr.set(...P.def.com); else P.geo.boundingBox.getCenter(ctr);
  for (const mt of rec.mtx) out.add(v.copy(ctr).applyMatrix4(mt));
  return out.divideScalar(rec.mtx.length);
}
function focusRegion(view, id, pickedRec = null) {
  if (!REGION_BY_ID[id]) return;
  if (FOCUS.region) resetOffsets();
  const R = REGION_BY_ID[id];
  root.updateMatrixWorld(true);
  const recs = OBJS.filter((r) => r.region === id);
  const C = new THREE.Vector3(); let mt = 0;
  const cw = new Map();
  for (const r of recs) { const c = recWorldCentroid(r, new THREE.Vector3()); cw.set(r, c); const m = PARTS[r.pi].def.mass_kg * r.mtx.length + 1e-3; C.addScaledVector(c, m); mt += m; }
  C.divideScalar(mt);
  let axis = null, modC = null;
  if (R.kind === 'fan') {
    const md = NODE_BY_ID['mod_' + id].group; modC = md.getWorldPosition(new THREE.Vector3());
    axis = new THREE.Vector3(0, 0, 1).transformDirection(md.matrixWorld);
    C.copy(modC);
  }
  const box = new THREE.Box3();
  for (const r of recs) {
    const c = cw.get(r), off = new THREE.Vector3();
    if (R.kind === 'fan') {
      const s = fanSlot(r);
      if (s !== null) off.copy(axis).multiplyScalar(s * 1.05);
      else { off.copy(c).sub(modC); off.addScaledVector(axis, -off.dot(axis)); off.setLength(.55).addScaledVector(axis, -.7); }
    } else if (R.kind === 'grid') {
      off.copy(c).sub(C); off.y = 0; off.multiplyScalar(.85); off.y = .35 + (PARTS[r.pi].def.pn === 'JD-200-002' ? .5 : PARTS[r.pi].def.pn === 'JD-200-001' ? -.45 : 0);
    } else {
      off.copy(c).sub(C);
      const L = off.length();
      off.multiplyScalar(R.kind === 'shell' ? .35 : .9); if (L > 1e-4) off.addScaledVector(off.clone().normalize(), R.kind === 'shell' ? .35 : .22);
    }
    const parent = r.obj.parent;
    const p0 = parent.worldToLocal(c.clone()), p1 = parent.worldToLocal(c.clone().add(off));
    r.focusOff = p1.sub(p0);
    box.expandByPoint(c); box.expandByPoint(c.clone().add(off));
  }
  const refocus = FOCUS.region === id && FOCUS.view === view;
  FOCUS.region = id; FOCUS.view = view; FOCUS.f = refocus ? 1 : 0; FOCUS.target = 1; FOCUS.recs = recs; FOCUS.hover = null;
  // camera: frame the exploded envelope from the current viewing direction
  const v = views[view], ctr = box.getCenter(new THREE.Vector3()), radius = Math.max(.6, box.getSize(new THREE.Vector3()).length() * .5);
  const dir = v.camera.position.clone().sub(v.controls.target).normalize();
  if (R.kind === 'fan') {                       // look across the axle so the exploded stack spreads out on screen
    const d = axis.clone().cross(new THREE.Vector3(0, 1, 0));
    if (d.lengthSq() < .01) d.set(1, 0, 0);
    d.normalize(); if (d.x < 0) d.negate();
    dir.copy(d).add(new THREE.Vector3(0, .45, 0)).addScaledVector(axis, .35).normalize();
  }
  FOCUS.fly = { target: ctr, pos: ctr.clone().addScaledVector(dir, radius / Math.tan(rad2(18)) * (R.kind === 'fan' ? 1.25 : 1.0)) };
  buildRegionLabels(recs);
  applyStyle(styleNow);
  if (view === 'sequence') { seq.autoCam = false; }
  renderBrief(true);
  const ui = REGION_UI[view]; ui.brief.hidden = false; ui.back.hidden = false; views[view].el.classList.add('focused');
  if (view === 'advanced') $('adv-sheet').scrollTop = 0;
  applyViewOffset(view);
  ui.chips.querySelectorAll('button').forEach((b) => b.classList.toggle('active', b.dataset.region === id));
}
const rad2 = (d) => d * Math.PI / 180;
function resetOffsets() { for (const r of OBJS) if (r.focusOff) { r.obj.position.set(0, 0, 0); if (r.line) r.line.position.set(0, 0, 0); r.focusOff = null; } }
function exitFocus(immediate = false) {
  if (!FOCUS.region) return;
  if (immediate) { finishExit(); return; }
  FOCUS.target = 0;
}
function finishExit() {
  const view = FOCUS.view;
  resetOffsets();
  FOCUS.region = null; FOCUS.labels.forEach((l) => l.el.remove()); FOCUS.labels = []; FOCUS.f = 0;
  views[view]?.el.classList.remove('focused');
  const ui = REGION_UI[view]; if (ui) { ui.brief.hidden = true; setTimeout(() => applyViewOffset(view)); ui.back.hidden = true; ui.svg.innerHTML = ''; ui.chips.querySelectorAll('button').forEach((b) => b.classList.remove('active')); }
  if (view === 'sequence') seq.autoCam = true;
  if (view === 'advanced') refreshIsolation(); else applyStyle(styleNow);
}
function tickFocus(dt, view) {
  if (!FOCUS.region || FOCUS.view !== view) return;
  const k = 1 - Math.pow(.02, dt);
  FOCUS.f += (FOCUS.target - FOCUS.f) * k;
  if (FOCUS.target === 0 && FOCUS.f < .01) { finishExit(); return; }
  const e = smooth(clamp(FOCUS.f));
  for (const r of FOCUS.recs) { r.obj.position.copy(r.focusOff).multiplyScalar(e); if (r.line) r.line.position.copy(r.obj.position); }
  if (FOCUS.fly && FOCUS.target === 1) {
    const v = views[view], kc = 1 - Math.pow(.08, dt);
    v.controls.target.lerp(FOCUS.fly.target, kc); v.camera.position.lerp(FOCUS.fly.pos, kc);
    if (v.camera.position.distanceTo(FOCUS.fly.pos) < .02) FOCUS.fly = null;
  }
  FOCUS.briefT += dt; if (FOCUS.briefT > .25) { FOCUS.briefT = 0; renderBrief(false); }
}
function buildRegionLabels(recs) {
  FOCUS.labels.forEach((l) => l.el.remove()); FOCUS.labels = [];
  const groups = new Map();
  for (const r of recs) {
    const p = PARTS[r.pi].def, nid = NODE[r.ni].def.id;
    const key = /^petO_/.test(nid) ? 'Petals · inlet collector' : /^petI_/.test(nid) ? 'Petals · exit diffuser' : /^blade_/.test(nid) ? 'Proprotor blades' : p.assy;
    const g = groups.get(key) || { key, mass: 0, n: 0, recs: [] };
    g.mass += p.mass_kg * r.mtx.length; g.n += r.mtx.length; g.recs.push(r); groups.set(key, g);
  }
  const list = [...groups.values()].sort((a, b) => b.mass - a.mass).slice(0, 9);
  const ui = REGION_UI[FOCUS.view];
  for (const g of list) {
    const rep = g.recs.reduce((a, b) => (PARTS[b.pi].def.mass_kg * b.mtx.length > PARTS[a.pi].def.mass_kg * a.mtx.length ? b : a));
    const el = document.createElement('div'); el.className = 'callout-tag region-tag';
    el.innerHTML = `${esc(g.key.toUpperCase())}<small>${g.n} part${g.n > 1 ? 's' : ''} · ${fmt(g.mass, g.mass < 10 ? 1 : 0)} kg</small>`;
    ui.layer.appendChild(el);
    FOCUS.labels.push({ el, obj: rep.obj, pt: recLocalCentroid(rep) });
  }
}
function drawRegionLabels(view) {
  const ui = REGION_UI[view];
  if (!ui) return;
  if (!FOCUS.region || FOCUS.view !== view) { if (ui.svg.innerHTML) ui.svg.innerHTML = ''; return; }
  const v = views[view], w = v.el.clientWidth, h = v.el.clientHeight;
  const cx = FOCUS.fly ? w / 2 : w / 2;
  let svg = ''; const used = [];
  const op = clamp((FOCUS.f - .55) / .35);
  const pts = FOCUS.labels.map((l) => { const p = l.pt.clone().applyMatrix4(l.obj.matrixWorld).project(v.camera); return { l, x: (p.x + 1) / 2 * w, y: (1 - p.y) / 2 * h, vis: p.z < 1 }; });
  pts.sort((a, b) => a.y - b.y);
  for (const q of pts) {
    if (!q.vis) { q.l.el.style.opacity = 0; continue; }
    const right = q.x >= cx;
    const maxX = w - (view === 'sequence' && w > 700 ? ui.brief.offsetWidth + 24 : 0) - 110;     // keep clear of the briefing panel
    let lx = clamp(q.x + (right ? 150 : -150), 110, maxX), ly = clamp(q.y - 30, 70, h - 70);
    for (const u of used) if (Math.abs(u.x - lx) < 190 && Math.abs(u.y - ly) < 40) ly = u.y + 42;
    used.push({ x: lx, y: ly });
    q.l.el.style.left = lx + 'px'; q.l.el.style.top = ly + 'px'; q.l.el.style.opacity = op;
    svg += `<line x1="${q.x.toFixed(1)}" y1="${q.y.toFixed(1)}" x2="${lx.toFixed(1)}" y2="${(ly - 4).toFixed(1)}" opacity="${op}"/><circle cx="${q.x.toFixed(1)}" cy="${q.y.toFixed(1)}" r="3" opacity="${op}"/>`;
  }
  ui.svg.innerHTML = svg;
}
// what the region is doing right now (sequence time or configuration slider)
function currentPhase(view) {
  if (view === 'sequence') {
    const t = seq.t;
    return t < 14 ? 'assembly' : t < 20 ? 'road' : t < 27 ? 'prepare' : t < 31 ? 'unlock' : t < 37 ? 'convert' : t < 41 ? 'spinup' : t < 47 ? 'hover' : 'cruise';
  }
  const c = adv.config;
  return c < .05 ? 'road' : c < .35 ? 'prepare' : c < .55 ? 'unlock' : c < .98 ? 'convert' : c < 1.4 ? 'hover' : 'cruise';
}
function currentState(view) { return view === 'sequence' ? seqState(seq.t) : configState(adv.config); }
function regionBrief(id, view) {
  const ph = currentPhase(view), st = currentState(view), M = flightModel(FLIGHT), R = REGION_BY_ID[id];
  const flying = ph === 'spinup' || ph === 'hover' || ph === 'cruise';
  const road = view === 'sequence' ? seq.table.v(seq.t) : 0;
  const elec = flying ? (ph === 'cruise' ? M.cr.P : M.hoverkW) * (ph === 'spinup' && view === 'sequence' ? clamp(st.spin / 2.6) : 1) : road > .5 ? 12 + road * 0.9 : 1.5;
  const rpm = st.rotorLocked ? road / DATA.meta.tire_R * 60 / (2 * Math.PI) : M.rpmHover * (view === 'sequence' ? clamp(st.spin / 2.6) : flying ? 1 : 0);
  const PH = { assembly: 'Assembly', road: 'Road mode', prepare: 'Stop & prepare', unlock: 'Unlock & open', convert: 'Converting', spinup: 'Spin-up', hover: 'Hover', cruise: 'Cruise' }[ph];
  let now = '', figs = [];
  if (R.kind === 'fan') {
    const front = id[0] === 'F', share = front ? M.kFront / 2 : (1 - M.kFront) / 2;
    const T = M.W * share / (ph === 'cruise' ? Math.cos(rad2(M.cr.tilt)) : 1);
    const per = elec / 4 * (share / .25);
    const txt = {
      assembly: 'The corner module docks onto its hardpoint bracket: suspension carrier, conversion yoke, then the wheel module itself.',
      road: `Rolling at ${fmt(road * 3.6, 0)} km/h. The dog clutch couples the rotor to the rim, so the axial-flux motor drives the tyre through the stator spokes. Blades are feathered at 90° and the petals are closed over them.`,
      prepare: 'The landing legs take the weight, so this wheel is unloaded. Parking brake on; the module runs its self-test (clutch, lock pins, petal switches, resolver).',
      unlock: 'The clutch releases the rotor from the rim. The brake caliper and two lock pins index the rim to the module. Petals open into an inlet collector and an exit diffuser; collective drives from 90° towards the hover setting.',
      convert: `The harmonic-drive actuator is swinging the module about the hinge above the tyre: ${fmt(st.yoke * 90, 0)}° of 90°. The fender cap rides on the yoke; two RVDTs must agree before the next step.`,
      spinup: `The motor accelerates the 7-blade rotor towards ${fmt(M.rpmHover, 0)} rpm. The seven stator vanes straighten the swirl. ${front === (id[1] === 'L') ? 'This diagonal pair spins counter-clockwise.' : 'This diagonal pair spins clockwise.'}`,
      hover: `Carrying ${fmt(share * 100, 1)} % of the vehicle weight: ${fmt(T / 1000, 1)} kN, of which the duct provides about ${fmt(M.ductShare * 100, 0)} %. Disk loading ${fmt(M.dl / 47.88, 0)} lb/ft².`,
      cruise: `The gimbal tilts the rotor axis ${fmt(st.tilt, 1)}° forward. Edgewise advance ratio μ ≈ ${fmt(M.cr.mu, 2)} (NASA TM Eq. 7), collective ${fmt(st.pitch, 1)}°.`,
    };
    now = txt[ph];
    figs = [['Rotor speed', `${fmt(rpm, 0)} rpm`], ['Motor power', `${fmt(per, 0)} kW`], ['Thrust', flying ? `${fmt(T / 1000, 2)} kN` : '0 kN'], ['Conversion yoke', `${fmt(st.yoke * 90, 0)}°`],
      ['Cruise tilt', `${fmt(st.tilt, 1)}°`], ['Collective θ.75', `${fmt(st.pitch, 1)}°`], ['Petals', `${fmt(st.petals * 100, 0)} %`], ['Clutch', st.rotorLocked ? 'Engaged (wheel drive)' : 'Released (rotor free)']];
  } else if (id === 'battery') {
    const E = FLIGHT.battery, bp = pack(elec, FLIGHT), I = bp.I, C = elec / E;
    now = { assembly: 'Nineteen modules drop into the floor tray and the rear stack behind the seats; the mica-lined lid seals the pack.',
      road: `Supplying about ${fmt(elec, 0)} kW for driving: a gentle ${fmt(C, 2)}C. Cold plates run on the low-temperature loop.`,
      prepare: 'Contactors closed, pre-charge complete. The BMS confirms state of charge and cell temperatures before conversion is allowed.',
      unlock: 'Small loads only (actuators, petals, pumps). The BMS reserves energy for the flight and the landing.',
      convert: 'Small loads only (conversion actuators). The BMS holds the flight reserve.',
      spinup: `Ramping to ${fmt(elec, 0)} kW as the rotors spin up; the chiller pre-cools the cold plates.`,
      hover: `Discharging at ${fmt(C, 1)}C: about ${fmt(I, 0)} A, ${fmt(bp.Icell, 0)} A per cell against a 60 A rating. After the landing reserve, hover lasts about ${fmt(M.hoverMin, 1)} min.`,
      cruise: `About ${fmt(elec, 0)} kW (${fmt(C, 1)}C, ${fmt(I, 0)} A). Roughly ${fmt(M.rangeKm, 0)} km at this speed after the landing reserve.` }[ph];
    figs = [['Cells', `${MASS.cells.toLocaleString()} × 21700 high-power`], ['Energy', `${fmt(E, 1)} kWh`], ['Pack power', `${fmt(elec, 0)} kW`], ['Pack current', `${fmt(I, 0)} A`], ['Per cell', `${fmt(bp.Icell, 1)} A of ${CELL.Acont} A`], ['C-rate', `${fmt(C, 2)}C`], ['Pack mass', `${fmt(MASS.road.bySys['200 Energy storage'], 0)} kg`]];
  } else if (id === 'power') {
    now = flying ? `Four SiC inverters each deliver about ${fmt(elec / 4, 0)} kW to their fan motors, controlling RPM for thrust allocation. The junction box fuses every branch; the insulation monitor watches the 800 V system.`
      : ph === 'road' ? `The inverters run the motors in torque mode for driving (≈${fmt(elec, 0)} kW total) with regenerative braking.` : 'HV system live; the DC-DC converter supplies 14 V to avionics, actuators and the 12 V backup battery.';
    figs = [['Inverters', '4 × 350 kW SiC'], ['Per inverter', `${fmt(elec / 4, 0)} kW`], ['Bus voltage', '820 V nominal (228s)'], ['DC-DC', '3 kW to 14 V'], ['On-board charger', '22 kW AC'], ['DC fast charge', 'CCS2 up to 350 kW']];
  } else if (id === 'cockpit') {
    now = flying ? 'The sidestick commands pitch and roll rates and the thrust lever commands climb. The steering yoke is locked centred; the HUD shows flight symbology. The parachute handle is armed overhead.'
      : ph === 'road' ? 'Steer-by-wire yoke and pedals are active. The sidestick and thrust lever are inhibited. The mode selector reads ROAD.'
      : 'The driver has selected PREPARE / CONFIGURE on the guarded mode selector. Each step needs confirmation; harnesses must be latched.';
    figs = [['Occupants', '2 × 90 kg'], ['Mode selector', flying ? 'FLY' : ph === 'road' ? 'ROAD' : 'CONFIGURE'], ['Primary display', 'Pillar-to-pillar OLED'], ['Restraints', '5-point harness + 4 airbags'], ['Parachute handle', flying ? 'ARMED' : 'SAFE']];
  } else if (id === 'avionics') {
    now = flying ? 'Three dissimilar flight computers run the control laws and allocate thrust to the four rotors at high rate. The independent monitor can inhibit an unsafe command. Radar altimeters and LiDAR watch the ground.'
      : 'The flight computers supervise conversion: every actuator move needs agreement between redundant position sensors and lock switches. On the road the VCU runs traction and stability control.';
    figs = [['Flight computers', '3 (triplex)'], ['IMUs', '3'], ['Air data', '2 pitot-static probes'], ['Height', '2 radar altimeters'], ['Surveillance', 'ADS-B / Mode-S'], ['Cameras', '8 surround']];
  } else if (id === 'thermal') {
    const q = elec * .08;
    now = `About ${fmt(q, 0)} kW of heat from motors, inverters and the battery goes to two front radiators and the chiller. ${flying ? 'In hover the radiator fans run hard because there is little ram air.' : ''}`;
    figs = [['Heat load', `${fmt(q, 0)} kW`], ['Radiators', '2 front, angled 35°'], ['Pumps', '2 electric'], ['Battery cooling', 'Chiller + cold plates']];
  } else if (id === 'legs') {
    const down = st.legs > .98, load = down && !flying ? M.W / 4 : 0;
    now = down ? `Down and locked. Each leg carries about ${fmt(load / 1000, 1)} kN, keeping the tyres 120 mm off the ground so the wheels can swing up.` : st.legs > .02 ? 'In transit: the ball-screw drives the two telescoping stages.' : flying ? 'Stowed after lift-off; the weight-on-wheels switches confirmed the car is airborne.' : 'Stowed flush with the underbody.';
    figs = [['Legs', '4 telescoping, 2 stages'], ['Stroke', '550 mm'], ['Load per leg', `${fmt(load / 1000, 1)} kN`], ['State', down ? 'Down & locked' : st.legs > .02 ? 'In transit' : 'Stowed']];
  } else if (id === 'emergency') {
    now = flying ? 'Armed. If the vehicle becomes uncontrollable (a single rotor failure leaves it under-actuated) the rocket extracts a whole-vehicle canopy through the rear deck lid.' : 'Safe on the ground; the fire-suppression bottle protects the battery bay at all times.';
    figs = [['Canopy rating', '2,500 kg'], ['Deployment', 'Rocket extraction'], ['Riser', '4-point to roll hoop & subframe'], ['Battery fire', 'Suppression bottle']];
  } else if (id === 'structure') {
    now = 'The carbon tub carries the battery, seats and roll hoop. Welded aluminium subframes pick up the suspension, steering rack and conversion hinges; corner hardpoints feed hinge and leg loads into the tub.';
    figs = [['Tub', 'CFRP / Nomex sandwich'], ['Subframes', '6061 welded tube'], ['Roll hoop', 'Ti-6Al-4V'], ['Structure mass', `${fmt(MASS.road.bySys['110 Structure'], 0)} kg`]];
  } else if (id === 'body') {
    now = 'Panels are trimmed from the body lofted through the A1 surfaces. Fender caps ride on the conversion yokes; gull-wing doors hinge on the roof line; the canopy is laminated polycarbonate.';
    figs = [['Panels', 'CFRP, Class-A paint'], ['Doors', '2 gull-wing'], ['Glazing', '4 polycarbonate panes'], ['Body mass', `${fmt(MASS.road.bySys['101 Body'], 0)} kg`]];
  }
  return { PH, now, figs };
}
const REGION_UI = {};
function initRegionUI(view) {
  const host = views[view].el;
  const svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg'); svg.setAttribute('class', 'hud-leaders region-leaders'); host.appendChild(svg);
  const layer = document.createElement('div'); layer.className = 'hud-callouts'; host.appendChild(layer);
  const chips = document.createElement('div'); chips.className = 'region-chips';
  chips.innerHTML = '<span>EXPLORE A REGION</span>' + REGIONS.map((r) => `<button data-region="${r.id}">${esc(r.name)}</button>`).join('');
  host.appendChild(chips);
  const back = document.createElement('button'); back.className = 'region-back'; back.textContent = '← Back to full vehicle'; back.hidden = true; host.appendChild(back);
  // tab 06 shows the briefing in the datasheet column; tab 05 overlays it at the right edge and shifts the model left
  const brief = document.createElement('aside'); brief.className = 'region-brief' + (view === 'advanced' ? ' in-sheet' : ''); brief.hidden = true;
  if (view === 'advanced') $('adv-sheet').prepend(brief); else host.appendChild(brief);
  const tip = document.createElement('div'); tip.className = 'region-tip'; tip.hidden = true; host.appendChild(tip);
  REGION_UI[view] = { svg, layer, chips, back, brief, tip };
  for (const el of [chips, back, brief]) for (const ev of ['pointerdown', 'pointerup', 'wheel', 'dblclick', 'contextmenu']) el.addEventListener(ev, (e) => e.stopPropagation());
  chips.querySelectorAll('button').forEach((b) => b.addEventListener('click', () => (FOCUS.region === b.dataset.region ? exitFocus() : focusRegion(view, b.dataset.region))));
  back.addEventListener('click', () => exitFocus());
  host.addEventListener('keydown', (e) => { if (e.key === 'Escape') exitFocus(); });
  // hover + click picking
  const ray = new THREE.Raycaster(), ndc = new THREE.Vector2();
  let down = null, lastMove = 0;
  const pick = (e) => {
    const r = host.getBoundingClientRect();
    ndc.set((e.clientX - r.left) / r.width * 2 - 1, -(e.clientY - r.top) / r.height * 2 + 1);
    ray.setFromCamera(ndc, views[view].camera);
    const targets = [];
    for (const rec of OBJS) { if (!rec.obj.visible || (FOCUS.region && rec.region !== FOCUS.region)) continue; if (rec.obj.isInstancedMesh) targets.push(rec.obj); else rec.obj.children.forEach((c) => c.isMesh && targets.push(c)); }
    const hit = ray.intersectObjects(targets, false).find((hh) => !renderer.clippingPlanes.length || hh.point.z <= .001);
    return hit ? { rec: hit.object.userData.rec, hit } : null;
  };
  host.addEventListener('pointerdown', (e) => (down = [e.clientX, e.clientY]));
  host.addEventListener('pointermove', (e) => {
    if (e.buttons || FOCUS.region || performance.now() - lastMove < 120 || e.target.closest('.region-chips, .region-brief, .region-back')) return;
    lastMove = performance.now();
    const h = pick(e), id = h ? h.rec.region : null;
    if (id !== FOCUS.hover) { FOCUS.hover = id; applyStyle(styleNow); if (view === 'advanced' && adv.isolate) refreshIsolation(); }
    tip.hidden = !id;
    if (id) { const r = host.getBoundingClientRect(); tip.textContent = `${REGION_BY_ID[id].long} · ${view === 'advanced' ? 'double-click' : 'click'} to expand`; tip.style.left = (e.clientX - r.left + 14) + 'px'; tip.style.top = (e.clientY - r.top + 14) + 'px'; }
  });
  host.addEventListener('pointerleave', () => { tip.hidden = true; if (FOCUS.hover && !FOCUS.region) { FOCUS.hover = null; applyStyle(styleNow); } });
  const open = (e) => {
    const h = pick(e); if (!h) return;
    tip.hidden = true;
    if (!FOCUS.region) focusRegion(view, h.rec.region, h.rec);
    else if (view === 'sequence') showPartFromRegion(h.rec.pi);
  };
  if (view === 'sequence') host.addEventListener('pointerup', (e) => { if (!down || Math.hypot(e.clientX - down[0], e.clientY - down[1]) > 5 || e.target.closest('.region-chips, .region-brief, .region-back')) return; open(e); });
  else host.addEventListener('dblclick', (e) => { if (!e.target.closest('.region-chips, .region-brief, .region-back')) open(e); });
}
function showPartFromRegion(pi) { showTab('advanced'); document.querySelector('[data-tab="advanced"]')?.click(); setTimeout(() => { setAdvView('3d'); selectPart(pi, -1, true); }, 60); }
function renderBrief(full) {
  if (!FOCUS.region) return;
  const ui = REGION_UI[FOCUS.view], R = REGION_BY_ID[FOCUS.region], B = regionBrief(FOCUS.region, FOCUS.view);
  if (full || !ui.brief.querySelector('.rb-now')) {
    const groups = new Map();
    for (const r of FOCUS.recs) { const p = PARTS[r.pi].def; const g = groups.get(p.assy) || []; if (!g.includes(r.pi)) g.push(r.pi); groups.set(p.assy, g); }
    const mass = FOCUS.recs.reduce((a, r) => a + PARTS[r.pi].def.mass_kg * r.mtx.length, 0);
    const pieces = FOCUS.recs.reduce((a, r) => a + r.mtx.length, 0);
    ui.brief.innerHTML = `<span class="eyebrow">REGION · ${esc(R.id.toUpperCase())}</span><h3>${esc(R.long)}</h3>
      <div class="rb-phase"></div><p class="rb-now"></p><dl class="kv rb-figs"></dl>
      <div class="rb-sum">${groups.size} sub-assemblies · ${[...groups.values()].flat().length} unique parts · ${pieces.toLocaleString()} pieces · ${fmt(mass, 1)} kg</div>
      <div class="rb-groups">${[...groups.entries()].sort().map(([a, list]) => `<details><summary>${esc(a)} <em>${list.length}</em></summary>${list.map((pi) => `<button data-pi="${pi}"><b>${PARTS[pi].def.pn}</b> ${esc(PARTS[pi].def.name)}</button>`).join('')}</details>`).join('')}</div>
      <p class="rb-hint">${FOCUS.view === 'sequence' ? 'Keep the sequence playing to watch this region change. Click a part to open its datasheet.' : 'Move the configuration slider to see this region in other modes. Click a part to select it.'}</p>`;
    ui.brief.querySelectorAll('button[data-pi]').forEach((b) => b.addEventListener('click', () => {
      const pi = +b.dataset.pi;
      if (FOCUS.view === 'advanced') { selectPart(pi, -1, false); } else showPartFromRegion(pi);
    }));
  }
  ui.brief.querySelector('.rb-phase').textContent = `NOW · ${B.PH.toUpperCase()}`;
  ui.brief.querySelector('.rb-now').textContent = B.now;
  ui.brief.querySelector('.rb-figs').innerHTML = B.figs.map(([k, v]) => `<dt>${esc(k.toUpperCase())}</dt><dd>${esc(v)}</dd>`).join('');
}

// ------------------------------------------------------------------ 08 miniature prototype
// Geometry from cad/source/build_mini.py (advanced/data/mini/*). The configurator swaps catalogue-class hobby parts
// and recomputes mass, CG (from the CAD instance positions in the flight pose), thrust, current, hover time and cost.
const MINI = { data: null, buf: null, scene: null, root: null, nodes: {}, recs: [], conv: 0, explode: 0, step: 0, shell: true, cad: null, cfg: null, anim: null };
// typical catalogue figures for 12-blade alloy-housing EDF units at their rated cell count (static, full throttle)
const EDFS = [
  { label: '64 mm EDF, 4S', S: 4, T: 1.15, A: 52, m: .125, od: .070, price: 45 },
  { label: '70 mm EDF, 6S', S: 6, T: 2.1, A: 78, m: .235, od: .077, price: 70 },
  { label: '80 mm EDF, 6S', S: 6, T: 2.8, A: 92, m: .330, od: .088, price: 95 },
  { label: '90 mm EDF, 6S (CADed)', S: 6, T: 3.4, A: 100, m: .420, od: .098, price: 110 },
];
const ESCS = [{ label: '60 A', A: 60, m: .060, price: 35 }, { label: '80 A', A: 80, m: .075, price: 45 }, { label: '100 A (CADed)', A: 100, m: .095, price: 55 }, { label: '120 A', A: 120, m: .115, price: 75 }];
const SERVOS = [{ label: '20 kg·cm', tq: 20, m: .055, price: 18 }, { label: '25 kg·cm', tq: 25, m: .062, price: 22 }, { label: '35 kg·cm (CADed)', tq: 35, m: .070, price: 28 }, { label: '60 kg·cm (large case)', tq: 60, m: .150, price: 55 }];
const PROTO_DEFAULT = { edf: 3, motor: 1, esc: 2, servo: 2, S: 6, mAh: 5000, C: 75, packs: 2, batX: 0, kInst: .92, usable: .8, misc: .19 };
const PROTO_INPUTS = [
  ['Propulsion', [['edf', 'Ducted fan (×4)', 'sel', EDFS], ['motor', 'Fan motor (×4)', 'sel', [{ label: 'JDM-M40, built yourself (CADed)', v: 1 }, { label: 'Stock motor bought with the fan', v: 0 }]], ['esc', 'ESC (×4)', 'sel', ESCS], ['kInst', 'Installed thrust (inlet + pod losses)', '', .8, 1, .01]]],
  ['Battery', [['S', 'Cells in series', 'sel', [{ label: '4S (14.8 V)', v: 4 }, { label: '6S (22.2 V)', v: 6 }]], ['mAh', 'Capacity per pack', 'mAh', 3000, 10000, 100], ['C', 'Continuous C rating', 'C', 25, 100, 5],
    ['packs', 'Packs in parallel', '', 1, 2, 1], ['usable', 'Capacity used before landing', '', .6, .9, .01], ['batX', 'Battery position (+ = forward)', 'mm', -40, 40, 1]]],
  ['Pods and airframe', [['servo', 'Conversion servo (×4)', 'sel', SERVOS], ['misc', 'Wiring, screws, paint', 'kg', .05, .5, .01]]],
];
const FIL = { lwpla: ['LW-PLA', 40, 10], petg: ['PETG', 22, 18], tpu: ['TPU 95A', 35, 9], clear_petg: ['Clear PETG', 25, 12] };   // name, $/kg, g/h printed
const BUY = { 'JDM-404': 75, 'JDM-405': 30, 'JDM-406': 15, 'JDM-408': 15, 'JDM-409': 15, 'JDM-410': 1.2, 'JDM-411': 3 };
const EXTRAS = [['M3 bolts, nylock nuts and heat-set inserts (assorted kit)', 1, 15], ['XT90 connectors, XT90 parallel lead, 10 / 12 AWG silicone wire', 1, 30], ['Servo extension leads, 300 mm', 4, 2],
  ['Battery straps (velcro, 250 mm)', 2, 3], ['2 mm carbon plate blank (400 × 150) or cut service', 1, 35]];
const MOTOR_COST = { 'JDM-501': 20, 'JDM-502': 0, 'JDM-503': 1.8, 'JDM-504': 31, 'JDM-505': 4, 'JDM-506': 7, 'JDM-507': 15 };   // per item: laser-cut stack, wire costed below, magnet arc, bell blank + shop turning, …
const MOTOR_EXTRAS = [['Class-H magnet wire Ø0.46 mm, 250 g spool (winds all four)', 1, 15], ['Impregnating varnish + high-temp magnet epoxy', 1, 18], ['3.5 mm bullet connectors, silicone leads', 4, 3]];
const TOOLS = [['ExpressLRS transmitter (skip if you own one)', 1, 90], ['6S balance charger', 1, 70], ['LiPo fire-resistant bag', 1, 15]];

function fetchPack(path, root = BASE) {
  return fetch(new URL(path + '.bin', root)).then((r) => (r.ok ? r.arrayBuffer() : Promise.reject(new Error(path + ' ' + r.status))))
    .catch(() => fetch(new URL(path + '.json', root)).then((r) => r.json()).then((j) => {
      const bin = atob(j.b64), out = new Uint8Array(bin.length);
      for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
      return out.buffer;
    }));
}
let miniPromise;
function bootMini() {
  if (!miniPromise) {
    const json = (r) => (r.ok ? r.json() : Promise.reject(new Error('mini parts ' + r.status)));
    miniPromise = Promise.all([fetch(new URL('data/mini/parts.json', BASE)).then(json), fetchPack('data/mini/meshes')])
      .then(([d, b]) => { MINI.data = d; MINI.buf = b; buildMini(); measureMini(); initProto(); $('proto-loading').remove(); })
      .catch((e) => { const el = $('proto-loading'); if (el) el.textContent = 'PROTOTYPE CAD FAILED TO LOAD: ' + e.message; throw e; });
  }
  return miniPromise;
}
const MINI_EXPLODE = {
  'JDM-101': [.07, .14, 0], 'JDM-102': [0, .15, 0], 'JDM-103': [-.07, .14, 0], 'JDM-104': [0, .22, 0],
  'JDM-202': [0, -.03, 0], 'JDM-203': [0, .03, 0], 'JDM-204': [0, .05, 0], 'JDM-403': [0, .08, 0], 'JDM-404': [0, .09, 0], 'JDM-405': [0, .2, 0],
  'JDM-402': [0, .05, 0], 'JDM-406': [0, .06, 0], 'JDM-408': [0, .05, 0], 'JDM-409': [0, .06, 0],
  'JDM-209': [0, -.02, -.03], 'JDM-205': [0, .03, 0], 'JDM-410': [0, .06, 0], 'JDM-407': [.05, .03, 0], 'JDM-411': [0, .05, .02], 'JDM-208': [0, .02, .03],
  'JDM-206': [0, 0, .05], 'JDM-207': [0, 0, .11], 'JDM-401': [0, 0, -.02],
  'JDM-507': [0, 0, -.07], 'JDM-506': [0, 0, -.085], 'JDM-501': [0, 0, -.1], 'JDM-502': [0, 0, -.1], 'JDM-503': [0, 0, -.13], 'JDM-504': [0, 0, -.13], 'JDM-505': [0, 0, -.16],
};
function buildMini() {
  const D = MINI.data, sc = MINI.scene = new THREE.Scene();
  sc.add(new THREE.HemisphereLight(0xcfe9ff, 0x0b1a22, 0.7));
  const sun = new THREE.DirectionalLight(0xffffff, 1.7); sun.position.set(.6, 1.2, .8); sc.add(sun);
  const grid = new THREE.GridHelper(2.4, 48, 0x2f7f8d, 0x14353f); grid.material.transparent = true; grid.material.opacity = .55; sc.add(grid);
  const ring = new THREE.Mesh(new THREE.RingGeometry(.5, .504, 128), new THREE.MeshBasicMaterial({ color: 0x6be1ed, transparent: true, opacity: .35, side: THREE.DoubleSide }));
  ring.rotation.x = -Math.PI / 2; ring.position.y = .0005; sc.add(ring);
  const mats = {};
  const mat = (k) => mats[k] || (mats[k] = (() => {
    const m = D.materials[k], L = LOOKS[m.look] || LOOKS.metal;
    const x = new THREE.MeshPhysicalMaterial({ color: new THREE.Color(m.color), envMap: envTex, envMapIntensity: .9, side: THREE.DoubleSide, ...L });
    if (L.transparent) x.depthWrite = false;
    return x;
  })());
  MINI.hot = new THREE.MeshPhysicalMaterial({ color: 0xffb35c, emissive: 0x6a3200, emissiveIntensity: .6, metalness: .2, roughness: .45, side: THREE.DoubleSide });
  const geos = D.parts.map((p) => decode(p, MINI.buf)), edges = geos.map((g) => new THREE.EdgesGeometry(g, 28));
  const nodes = D.nodes.map((n) => { const g = new THREE.Group(); g.userData.id = n.id; g.position.set(...n.pos); g.quaternion.set(...n.q); if (n.mirror) g.scale.set(1, 1, -1); return { def: n, group: g }; });
  nodes.forEach((r) => { if (r.def.parent >= 0) nodes[r.def.parent].group.add(r.group); else MINI.root = r.group; MINI.nodes[r.def.id] = r; });
  sc.add(MINI.root);
  D.instances.forEach((s) => {
    const p = D.parts[s[0]], me = new THREE.Mesh(geos[s[0]], mat(p.material)), holder = new THREE.Group();
    me.position.set(s[2], s[3], s[4]); me.quaternion.set(s[5], s[6], s[7], s[8]);
    me.add(new THREE.LineSegments(edges[s[0]], cadLine));
    holder.add(me); nodes[s[1]].group.add(holder);
    MINI.recs.push({ me, holder, p, pn: p.pn, base: me.material, ex: new THREE.Vector3(...(MINI_EXPLODE[p.pn] || [0, 0, 0])) });
  });
}
function poseMini(conv, explode) {
  for (const c of ['FL', 'FR', 'RL', 'RR']) MINI.nodes['my_' + c].group.quaternion.setFromAxisAngle(new THREE.Vector3(1, 0, 0), rad(-90 * conv));
  for (const r of MINI.recs) r.holder.position.copy(r.ex).multiplyScalar(explode);
}
// CAD-derived numbers the configurator needs: instance positions in the flight pose, thrust lines, hinge arms
function measureMini() {
  poseMini(1, 0); MINI.root.updateMatrixWorld(true);
  const items = [], v = new THREE.Vector3();
  for (const r of MINI.recs) items.push({ pn: r.pn, m: r.p.mass_kg, p: r.me.localToWorld(v.set(...r.p.com)).clone() });
  MINI.recs.forEach((r, i) => { let g = r.holder.parent; while (g && !/^m[cyp]_/.test(g.userData.id || '')) g = g.parent; items[i].corner = g ? g.userData.id.slice(3) : null; });
  const hinge = new THREE.Vector3(), hingeR = new THREE.Vector3();
  MINI.nodes.my_FL.group.getWorldPosition(hinge);
  const edf = items.filter((it) => it.pn === 'JDM-401');
  const fl = edf.find((it) => it.corner === 'FL');
  const pod = items.filter((it) => it.corner === 'FL' && (['JDM-206', 'JDM-207', 'JDM-401', 'JDM-208', 'JDM-411'].includes(it.pn) || it.pn.startsWith('JDM-50')));
  const motorM = items.filter((it) => it.corner === 'FL' && it.pn.startsWith('JDM-50')).reduce((a, b) => a + b.m, 0);
  const podM = pod.reduce((a, b) => a + b.m, 0), podZ = pod.reduce((a, b) => a + b.m * b.p.z, 0) / podM;
  poseMini(0, 0); MINI.root.updateMatrixWorld(true);
  MINI.nodes.my_FL.group.getWorldPosition(hingeR);
  const podRoad = MINI.recs.filter((r) => r.pn === 'JDM-206').map((r) => r.me.getWorldPosition(new THREE.Vector3())).find((p) => p.x > 0 && p.z > 0);
  MINI.cad = {
    items, xf: edf.filter((e) => e.p.x > 0).reduce((a, b) => a + b.p.x, 0) / 2, xr: edf.filter((e) => e.p.x < 0).reduce((a, b) => a + b.p.x, 0) / 2,
    thrustArm: Math.abs(fl.p.z - hinge.z), podArm: Math.abs(podZ - hinge.z), podRadius: Math.hypot(podRoad.y - hingeR.y, podRoad.z - hingeR.z),
    podCadM: podM, motorM, fanM: MINI.data.parts.find((p) => p.pn === 'JDM-401').mass_kg, chassisY: MINI.data.meta.chassis_y, hingeY: hinge.y,
  };
  poseMini(MINI.conv, MINI.explode);
}
// JDM-M40 on the 90 mm fan: torque balance Ke·I = Q_fan(ω) + P_fixed(ω)/ω, voltage Vm = Ke·ω + I·R (motor_design.py)
function motorOnFan(T, Vb, kInst) {
  const MT = MINI.data.meta.motor, D = MT.design, F = MT.fan, fc = MT.fixed_curve;
  const pfix = (w) => { const s = fc[1][0] - fc[0][0], i = clamp(Math.floor(w / s), 0, fc.length - 2), t = (w - fc[i][0]) / s; return fc[i][1] + t * (fc[i + 1][1] - fc[i][1]); };
  const at = (w) => { const Q = F.Q_r * (w / F.w_r) ** 2, Pf = pfix(w), I = (Q + Pf / w) / D.Ke, Vm = w * D.Ke + I * D.R;
    return { w, rpm: w * 30 / Math.PI, Q, I, Vm, Pf, Pcu: I * I * D.R, T: kInst * F.T_r * (w / F.w_r) ** 2, eff: Q * w / (Vm * I) }; };
  if (T != null) { const r = at(F.w_r * Math.sqrt(T / (kInst * F.T_r))); if (r.Vm <= Vb) return { ...r, sat: false }; }
  let lo = 50, hi = 9000;
  for (let k = 0; k < 50; k++) { const mid = (lo + hi) / 2; if (at(mid).Vm > Vb) hi = mid; else lo = mid; }
  return { ...at(lo), sat: T != null };
}
function motorThermal(r) {           // lumped convection to the exhaust stream; returns winding rise (K) and time constant (s)
  const MT = MINI.data.meta.motor, th = MT.thermal, h = th.h[0] * (th.h[1] + th.h[2] * r.w / MT.fan.w_r) ** th.h[3];
  const R = 1 / (h * th.A) + th.R_int;
  return { dT: (r.Pcu + r.Pf) * R, tau: R * th.C };
}
function protoModel(c) {
  const e = EDFS[c.edf], esc = ESCS[c.esc], sv = SERVOS[c.servo], cad = MINI.cad;
  const sleeve = e.od < .0975 ? .025 : 0;                      // printed adaptor sleeve for a smaller casing in the 98 mm duct ring
  const custom = c.motor === 1 && c.edf === 3;                  // the JDM-M40 is sized for the 90 mm fan only
  const packWh = c.S * 3.7 * c.mAh / 1000, packM = packWh * .0071 + .015, packVol = packWh * 2.68;
  const over = { 'JDM-401': custom ? cad.fanM : e.m + sleeve, 'JDM-402': esc.m, 'JDM-407': sv.m };
  if (!custom) for (const it of cad.items) if (it.pn.startsWith('JDM-50')) over[it.pn] = 0;
  const pts = [], bat = cad.items.filter((it) => it.pn === 'JDM-403').sort((a, b) => a.p.y - b.p.y)[0];
  for (let k = 0; k < c.packs; k++) pts.push([packM, bat.p.x + c.batX / 1000, bat.p.y + k * .041, bat.p.z]);   // packs stack on the tray
  for (const it of cad.items) {
    if (it.pn === 'JDM-403') continue;
    pts.push([over[it.pn] ?? it.m, it.p.x, it.p.y, it.p.z]);
  }
  pts.push([c.misc, 0, cad.chassisY + .01, 0]);
  const m = pts.reduce((a, b) => a + b[0], 0);
  const cg = { x: pts.reduce((a, b) => a + b[0] * b[1], 0) / m, y: pts.reduce((a, b) => a + b[0] * b[2], 0) / m, z: pts.reduce((a, b) => a + b[0] * b[3], 0) / m };
  const Iyaw = pts.reduce((a, b) => a + b[0] * ((b[1] - cg.x) ** 2 + (b[3] - cg.z) ** 2), 0);
  const W = m * 9.81, kf = clamp((cg.x - cad.xr) / (cad.xf - cad.xr));
  const Tf = W * kf / 2, Tr = W * (1 - kf) / 2;
  // battery: open-circuit 3.85 V per cell at mid charge; resistance scales with capacity and C rating; 4 mΩ of wiring and connectors
  const Ah = c.mAh / 1000 * c.packs, Rpack = c.S * .003 * (5000 / c.mAh) * Math.sqrt(60 / c.C) / c.packs + .004, Voc = c.S * 3.85, Vr = e.S * 3.7;
  // fixed-pitch fan: thrust ∝ rpm², power ∝ rpm³, so at a fixed voltage I ∝ T^1.5; full-throttle thrust ∝ V², current ∝ V²
  const op = (Tlist, V0 = Voc) => {
    let V = V0, out = null;
    for (let k = 0; k < 40; k++) {
      const s = V / Vr, Tmax = e.T * 9.81 * c.kInst * s * s, Imax = e.A * s * s;
      const per = Tlist.map((T) => { const x = T == null ? 1 : Math.min(1, T / Tmax); return { x, I: Imax * x ** 1.5, sat: T != null && T > Tmax }; });
      const I = per.reduce((a, b) => a + b.I, 0) + 1.5;    // + flight controller, receiver, GNSS and servo holding current
      const Vn = Math.max(.6 * V0, V0 - I * Rpack);
      out = { V: Vn, I, per, Tmax, Imax };
      if (Math.abs(Vn - V) < 1e-5) break;
      V = .5 * (V + Vn);
    }
    return out;
  };
  const opM = (Tlist, V0 = Voc) => {                          // custom motor: motor, fan and battery solved together
    let V = V0, out = null;
    for (let k = 0; k < 40; k++) {
      const Vb = .97 * V, fmax = motorOnFan(null, Vb, c.kInst);
      const per = Tlist.map((T) => { const r = motorOnFan(T, Vb, c.kInst); return { ...r, x: r.T / fmax.T, I: r.Vm * r.I / (.97 * V), Im: r.I }; });
      const I = per.reduce((a, b) => a + b.I, 0) + 1.5;
      const Vn = Math.max(.6 * V0, V0 - I * Rpack);
      out = { V: Vn, I, per, Tmax: fmax.T, Imax: fmax.I, fmax };
      if (Math.abs(Vn - V) < 1e-5) break;
      V = .5 * (V + Vn);
    }
    return out;
  };
  const hv = custom ? opM([Tf, Tf, Tr, Tr]) : op([Tf, Tf, Tr, Tr]), full = custom ? opM([null, null, null, null]) : op([null, null, null, null]);
  const fresh = (custom ? opM : op)([null, null, null, null], c.S * 4.15);   // full throttle on a freshly charged pack
  // throttle cap that holds a fresh pack's current to 85 % of the ESC rating (current ∝ duty^1.5·V² for a fan load)
  const cap = Math.min(1, (.85 * esc.A / fresh.Imax) ** (1 / 3));
  const twr = 4 * full.Tmax / W, xMax = Math.max(...hv.per.map((p) => p.x));
  const hoverMin = c.usable * Ah / hv.I * 60;
  const podM = cad.podCadM - cad.fanM - cad.motorM + (custom ? cad.fanM + cad.motorM : e.m + sleeve);
  const conv = podM * cad.podRadius * 100;                                   // kg·cm, worst case (pod arm horizontal)
  const holdHover = Math.abs(Math.max(Tf, Tr) / 9.81 * cad.thrustArm - podM * cad.podArm) * 100;
  const holdFull = Math.abs(full.Tmax / 9.81 * cad.thrustArm - podM * cad.podArm) * 100;
  const dYaw = rad(5), Myaw = 2 * Tf * Math.sin(dYaw) * (cad.xf - cg.x) + 2 * Tr * Math.sin(dYaw) * (cg.x - cad.xr), aYaw = Myaw / Iyaw;
  const Df = e.od - .008, Afan = Math.PI / 4 * (Df * Df - (.4 * Df) ** 2), dl = W / (4 * Afan);
  return { custom, fresh, cap, e, esc, sv, m, W, cg, kf, Tf, Tr, hv, full, twr, xMax, hoverMin, Ah, packWh, packM, packVol, sleeve, podM, conv, holdHover, holdFull, Myaw, aYaw, Iyaw, dl, Afan, Rpack, Voc };
}
function protoChecks(c, M) {
  const k = (ok, mid) => (ok ? 'ok' : mid ? 'mid' : 'bad'), cad = MINI.cad;
  const dx = (M.cg.x - (cad.xf + cad.xr) / 2) * 1000, Ipack = M.full.I / M.Ah, Ihv = M.hv.I / M.Ah;
  return [
    ['Fan fits the pod duct ring', `${fmt(M.e.od * 1000, 0)} mm casing`, '≤ 98 mm bore', k(M.e.od <= .0985 && !M.sleeve, M.e.od <= .0985), M.sleeve ? `Smaller casing: print a ${fmt((.0984 - M.e.od) / 2 * 1000, 1)} mm-wall adaptor sleeve (adds ${fmt(M.sleeve * 4000, 0)} g) and expect extra inlet loss.` : 'The duct ring is CADed around a 98 mm casing with a 0.4 mm clamp gap.'],
    ...(c.motor === 1 && c.edf !== 3 ? [['Custom motor matches the fan', 'JDM-M40 on a smaller fan', '90 mm fan only', 'bad', 'The JDM-M40 is sized for the 90 mm rotor; the stock motor is used in the numbers instead.']] : []),
    ['Cells vs fan rating', `${c.S}S on a ${M.e.S}S fan`, `= ${M.e.S}S`, k(c.S === M.e.S, c.S < M.e.S), c.S > M.e.S ? 'Over-volting spins the fan past its rated speed: motor and rotor can fail. Do not fly.' : c.S < M.e.S ? 'Under-volting is safe but thrust falls with the square of voltage.' : 'Matches the fan’s rated voltage.'],
    ['Thrust-to-weight (full throttle, sagged)', `${fmt(M.twr, 2)} : 1`, '≥ 2.0 (≥ 1.6 marginal)', k(M.twr >= 2, M.twr >= 1.6), `${fmt(4 * M.full.Tmax / 9.81, 2)} kg of thrust at ${fmt(M.full.V, 1)} V for ${fmt(M.m, 2)} kg. Below about 1.6 there is no margin to hold attitude in wind.`],
    ['Hover throttle, most-loaded fan', `${fmt(M.xMax * 100, 0)} % of max thrust`, '≤ 55 % (≤ 70 % marginal)', k(M.xMax <= .55, M.xMax <= .7), `Front / rear thrust split ${fmt(M.kf * 100, 0)} / ${fmt((1 - M.kf) * 100, 0)} % from the CG.`],
    ['Hover time', `${fmt(M.hoverMin, 1)} min`, '≥ 3 min (≥ 2 marginal)', k(M.hoverMin >= 3, M.hoverMin >= 2), `${fmt(M.hv.I, 0)} A total in hover from ${fmt(M.Ah, 1)} Ah, landing with ${fmt((1 - c.usable) * 100, 0)} % left. Ducted fans trade endurance for compactness.`],
    ['Battery current at full throttle', `${fmt(Ipack, 0)}C (${fmt(M.full.I, 0)} A)`, `≤ ${c.C}C rating`, k(Ipack <= c.C * .8, Ipack <= c.C), `Hover draws ${fmt(Ihv, 0)}C. Real packs deliver well under their label, hence the 80 % target.`],
    ['Voltage sag at full throttle', `${fmt(M.full.V / c.S, 2)} V per cell`, '≥ 3.4 V (≥ 3.2 marginal)', k(M.full.V / c.S >= 3.4, M.full.V / c.S >= 3.2), `Pack resistance ${fmt(M.Rpack * 1000, 0)} mΩ including wiring.`],
    ['ESC current, full throttle', `${fmt(M.full.Imax, 0)} A per fan`, `≤ 85 % of ${M.esc.A} A`, k(M.full.Imax <= .85 * M.esc.A, M.full.Imax <= M.esc.A), 'Fan current at full throttle and sagged voltage.'],
    ['ESC current on a fresh pack', `${fmt(M.fresh.Imax, 0)} A per fan`, `≤ 85 % of ${M.esc.A} A`, M.fresh.Imax <= .85 * M.esc.A ? 'ok' : M.cap >= .85 ? 'mid' : 'bad',
      M.fresh.Imax <= .85 * M.esc.A ? 'A fully charged pack sags less, so the fans spin faster and draw more.' : `A fully charged pack sags less and the fans draw more. Cap the throttle at ${fmt(M.cap * 100, 0)} % (ArduPilot MOT_SPIN_MAX) or set an ${fmt(.85 * M.esc.A, 0)} A current limit in the ESCs; thrust on a sagged pack is not affected.`],
    ['Battery fits the tray and body', `${fmt(M.packVol * c.packs, 0)} cm³`, '≤ 620 cm³', k(M.packVol * c.packs <= 620, M.packVol * c.packs <= 720), `${c.packs} × ${c.S}S ${c.mAh} mAh, about ${fmt(M.packM * 1000, 0)} g each. The tray is 170 × 60 mm with about 60 mm of height under the shell.`],
    ['CG over the thrust centre (fore-aft)', `${fmt(dx, 0)} mm ${dx >= 0 ? 'forward' : 'aft'}`, '≤ 8 mm (≤ 20 marginal)', k(Math.abs(dx) <= 8, Math.abs(dx) <= 20), `Slide the battery to move it: about ${fmt(M.packM * c.packs / M.m, 2)} mm of CG per mm of battery travel.`],
    ['CG lateral', `${fmt(M.cg.z * 1000, 1)} mm`, '≤ 3 mm', k(Math.abs(M.cg.z) <= .003, Math.abs(M.cg.z) <= .006), 'Mirror-image layout; route wiring to keep it centred.'],
    ['Servo torque, pod swing', `${fmt(M.conv, 1)} kg·cm`, `≤ ½ of ${M.sv.tq} kg·cm`, k(M.conv <= M.sv.tq / 2, M.conv <= M.sv.tq / 1.3), `Pod mass ${fmt(M.podM * 1000, 0)} g at ${fmt(MINI.cad.podRadius * 1000, 0)} mm from the hinge, worst case. Convert on the ground with the fans stopped.`],
    ['Servo holding torque, full thrust', `${fmt(M.holdFull, 1)} kg·cm`, `≤ 70 % of ${M.sv.tq} kg·cm`, k(M.holdFull <= .7 * M.sv.tq, M.holdFull <= M.sv.tq), `In flight the thrust line sits ${fmt(MINI.cad.thrustArm * 1000, 0)} mm outboard of the hinge, so the servo holds thrust × offset minus pod weight × ${fmt(MINI.cad.podArm * 1000, 0)} mm. Hover: ${fmt(M.holdHover, 1)} kg·cm.`],
    ['Yaw authority (±5° differential pod tilt)', `${fmt(M.aYaw, 1)} rad/s²`, '≥ 3 (≥ 1.5 marginal)', k(M.aYaw >= 3, M.aYaw >= 1.5), `EDF stators cancel most fan torque, so yaw comes from tilting front and rear pods opposite ways: ${fmt(M.Myaw, 2)} N·m on ${fmt(M.Iyaw * 1000, 0)} g·m².`],
    ['Registration', `${fmt(M.m, 2)} kg`, 'over 250 g', 'ok', 'Register it with your aviation authority (FAA in the US) before flying outdoors, and fly within recreational rules.'],
  ];
}
// --- build steps: each adds parts; the viewer shows parts up to the current step and highlights the new ones
const PRINTED = (p) => p.spec && p.spec.print;
const PROTO_STEPS = [
  { t: 'Overview', time: '', add: [], all: true, pose: null,
    what: 'The finished miniature: the scaled Rev 08.1 body on a carbon chassis plate, four wheel pods that each hold a ducted fan, and a servo-driven hinge above every tyre. Use Next to build it part by part.',
    doIt: ['Pick your parts in the configurator below; every check updates. The defaults are the CADed build.', 'Download the STL / STEP files from the print list.', 'Budget a weekend for printing and a weekend for assembly and setup.'], check: '' },
  { t: 'Print and cut the parts', time: 'print', add: [], preview: true, pose: null,
    what: 'Every orange part is made by you: printed in LW-PLA, PETG, TPU or clear PETG, plus one carbon plate.',
    doIt: ['Body sections (JDM-101 to 103): LW-PLA at about 240 °C so it foams; 2 perimeters, no infill. Sand and paint after assembly.',
      'Hinge masts, blocks, arms and duct rings: PETG (or CF-nylon for the masts and arms). Print the duct rings lip-up without supports.',
      'Tyre rings: TPU 95A, 3 perimeters. Canopy: clear PETG in vase mode, or thermoform over a printed buck.',
      'Chassis plate: send the JDM-201 STEP outline to a CNC plate service, or cut 2 mm carbon with a diamond blade and a dust mask.',
      'Print one RH and one LH of each handed part (the -LH files).'],
    check: 'Test-fit a 608 bearing in a hinge block and the fan casing in a duct ring before printing the rest.' },
  { t: 'Chassis plate, legs and trays', time: '40 min', add: ['JDM-201', 'JDM-202', 'JDM-203', 'JDM-204'], pose: 0,
    what: 'The 2 mm carbon plate is the backbone. Everything that carries load bolts to it; the body shell carries none.',
    doIt: ['Melt M3 heat-set inserts into the legs, battery tray and flight-controller plate.', 'Bolt the four legs under the plate at the outer leg holes (M3 × 10).',
      'Bolt the battery tray on top, centred, and the FC plate ahead of it on the four standoff holes using 10 mm nylon standoffs.'],
    check: 'The plate sits level on its legs on a flat table.' },
  { t: 'Hinge masts and hinge blocks', time: '40 min', add: ['JDM-209', 'JDM-205', 'JDM-410'], pose: 0,
    what: 'Each mast lifts a hinge block 90 mm above the plate so the pod can swing up and out over the tyre.',
    doIt: ['Press two 608 bearings into each hinge block (warm the block if they are tight).', 'Glue each block onto its mast’s top flange with epoxy and add two M3 screws.',
      'Bolt each mast foot to the plate edge with two M3 × 12 through the mast holes. Masts face outboard.'],
    check: 'The four bearing bores line up front-to-back with a straight 8 mm rod.' },
  { t: 'Swing arms, hinge shafts and servos', time: '1 h', add: ['JDM-208', 'JDM-411', 'JDM-407'], pose: .5,
    what: 'The swing arm is the miniature conversion yoke: it turns on an 8 mm shaft in the two bearings and carries the pod.',
    doIt: ['Slide the 8 mm shaft through the bearings and the arm; lock it with a grub screw on the flat.', 'Mount each servo beside its hinge block and link the horn to the arm (1 : 1, ball links).',
      'Before linking, centre each servo with a tester and set the travel so 0° is road and 90° is flight.'],
    check: 'Each arm swings 0–90° by hand with no binding and no slop at the shaft.' },
  { t: 'Pods: duct ring, fan and tyre', time: '1 h', add: ['JDM-206', 'JDM-401', 'JDM-207'], pose: 0,
    what: 'Each pod is a printed duct ring around a 90 mm ducted fan, with a TPU tyre stretched over the outer band.',
    doIt: ['Slide the fan into the duct ring from the exhaust side so the inlet sits behind the bell-mouth lip; clamp with the two M3 pinch bolts.',
      'Stretch the TPU tyre ring over the band (warm it in hot water first).', 'Bolt each pod to its swing arm at the servo-arm boss; run the three motor wires along the arm with a loop at the hinge.'],
    check: 'Spin each fan by hand: no rubbing. Swing each pod 0–90°: the wires don’t pull tight.' },
  { t: 'Build and fit the fan motors', time: '≈ 3 h per motor', add: ['JDM-501', 'JDM-502', 'JDM-503', 'JDM-504', 'JDM-505', 'JDM-506', 'JDM-507'], pose: 0,
    what: 'Each fan is driven by a JDM-M40: a 40 mm outrunner you wind and assemble yourself, sized for this fan and this battery. The full procedure, winding table and checks are in the “Custom fan motor” section below.',
    doIt: ['Wind 12 tooth coils (10 turns of 0.46 mm wire each), connect them as the winding table shows, varnish and bake.',
      'Bond 10 magnets in the 4140 bell, alternating poles, press the bearings into the hub and assemble the rotor on its shaft.',
      'Measure Kv with a drill and a multimeter, balance the motor with the fan rotor fitted, then bolt it into the fan’s motor mount.'],
    check: 'All four motors within 3 % of each other on Kv and phase resistance; each runs 3 min at hover throttle on the bench below 80 °C.' },
  { t: 'Power and electronics', time: '2 h', add: ['JDM-402', 'JDM-408', 'JDM-409', 'JDM-404', 'JDM-405', 'JDM-406'], pose: 0,
    what: 'Two packs in parallel feed a distribution board, four ESCs and a separate servo BEC; the flight controller runs everything. One pack sags too far at full throttle (see the checks).',
    doIt: ['Solder the PDB: XT90 input, four ESC pads, a 5 V feed to the flight controller and the BEC input.', 'Mount the ESCs on the plate near their pods; solder motor wires (swap two to reverse a fan).',
      'Wire ESC signals to FC outputs 1–4 (front-right, rear-left, front-left, rear-right) and servos to outputs 5–8. Power the servos from the BEC, not the FC.',
      'Mount the GNSS on the roof mast, arrow forward. Receiver on a UART; antennas out of the carbon’s shadow.'],
    check: 'Smoke-stopper first power-up with no fans connected. The FC boots and the servos centre.' },
  { t: 'Battery and centre of gravity', time: '20 min', add: ['JDM-403'], pose: 1,
    what: 'With the pods in flight position, the CG must sit over the centre of the four fans.',
    doIt: ['Stack the two packs on the tray, join them with an XT90 parallel lead and strap them down with two velcro straps. Only parallel packs at the same voltage (within 0.05 V per cell).', 'Set the pods to 90°. Balance the model on two fingertips under the front and rear fan centres.',
      'Slide the battery until it balances; mark the position on the tray. The configurator shows the predicted spot.'],
    check: 'CG within 8 mm of the fan centre fore-aft and 3 mm side-to-side (see the checks table).' },
  { t: 'Body shell and canopy', time: '1.5 h', add: ['JDM-101', 'JDM-102', 'JDM-103', 'JDM-104'], pose: 0,
    what: 'The three body sections and canopy are cosmetic and aerodynamic. They clip over the frame with magnets so you can reach the battery.',
    doIt: ['Glue the canopy into the middle section.', 'Glue 6 × 3 mm magnets in the section joints and matching ones on the masts.',
      'Check the pod pockets: each pod must swing 0–90° without touching the shell. Trim if needed.'],
    check: 'Run the conversion with the shell on: no contact anywhere.' },
  { t: 'Set up and test conversion', time: '2 h', add: [], all: true, pose: 'anim',
    what: 'Set up ArduPilot, then prove the conversion logic on the bench with the fans unpowered.',
    doIt: ['Flash ArduCopter. Frame: Quad X. Calibrate accelerometers, compass and radio. Set the ESC protocol to DShot.',
      'Add a short Lua script: a switch slews all four pod servos 0° ↔ 90° over 2 s and blocks arming unless all pods are at 90°.',
      'Map yaw to differential pod tilt in the same script (±5°, front pair opposite the rear pair) and turn down the motors’ own yaw mixing; EDF stators make fan-torque yaw weak.',
      'Cap the throttle with MOT_SPIN_MAX (or set a current limit in the ESCs) to the value the checks table gives, so a freshly charged pack cannot overdrive the ESCs.',
      'Set a battery failsafe (land at 3.5 V per cell), a radio failsafe (land) and a geofence.'],
    check: 'Pods convert both ways on the switch. Arming refuses in road position. Tilting the model makes the motors respond correctly in the motor-test page.' },
  { t: 'First hover', time: '', add: [], all: true, pose: 1,
    what: 'Tethered first, then free hover in an open field. Ducted fans are dangerous: treat them like spinning blades.',
    doIt: ['Tie the model to two ground stakes with 1 m lines. Safety glasses on; stand 5 m back; keep hands out of the inlets.',
      'Arm in Stabilize, raise throttle slowly to light on the skids, check that it doesn’t try to flip or yaw. Fix motor order or direction if it does.',
      'Untethered: hover at 1 m for 30 s, land, check temperatures and the pack voltage. Then tune (AutoTune on roll and pitch) and only then try a slow forward flight.'],
    check: 'Stable hover with the throttle near the predicted hover point. Land when the timer reaches the predicted hover time minus one minute.' },
];
function initProto() {
  const v = makeView('prototype', 'proto-viewport', [.52, .3, .58], [0, .08, 0]);
  v.controls.minDistance = .25; v.controls.maxDistance = 4;
  MINI.cfg = { ...PROTO_DEFAULT };
  $('proto-conv').addEventListener('input', (e) => { MINI.anim = null; MINI.conv = +e.target.value; poseMini(MINI.conv, MINI.explode); labelConv(); });
  $('proto-explode').addEventListener('input', (e) => { MINI.explode = +e.target.value; poseMini(MINI.conv, MINI.explode); });
  $('proto-shell').addEventListener('change', (e) => { MINI.shell = e.target.checked; applyStep(); });
  $('proto-reset').addEventListener('click', () => { v.camera.position.set(.52, .3, .58); v.controls.target.set(0, .08, 0); });
  $('proto-prev').addEventListener('click', () => setStep(MINI.step - 1));
  $('proto-next').addEventListener('click', () => setStep(MINI.step + 1));
  $('proto-step-list').innerHTML = PROTO_STEPS.map((s, i) => `<li><button data-pstep="${i}"><span>${String(i).padStart(2, '0')}</span>${esc(s.t)}</button></li>`).join('');
  $('proto-step-list').querySelectorAll('button').forEach((b) => b.addEventListener('click', () => setStep(+b.dataset.pstep)));
  renderProtoInputs(); setStep(0); renderProto();
}
function labelConv() { $('proto-conv').value = MINI.conv; $('proto-conv-label').textContent = MINI.conv < .02 ? 'Road' : MINI.conv > .98 ? 'Flight' : `${fmt(MINI.conv * 90, 0)}°`; }
function setStep(i) {
  MINI.step = clamp(i, 0, PROTO_STEPS.length - 1);
  const s = PROTO_STEPS[MINI.step];
  MINI.anim = s.pose === 'anim' ? { t: 0 } : null;
  if (typeof s.pose === 'number') { MINI.conv = s.pose; poseMini(MINI.conv, MINI.explode); labelConv(); }
  $('proto-step-code').textContent = `STEP ${MINI.step} / ${PROTO_STEPS.length - 1}`;
  $('proto-prev').disabled = MINI.step === 0; $('proto-next').disabled = MINI.step === PROTO_STEPS.length - 1;
  $('proto-step-list').querySelectorAll('button').forEach((b, k) => { b.classList.toggle('active', k === MINI.step); b.classList.toggle('done', k < MINI.step); });
  const parts = [...new Set(s.add)].map((pn) => MINI.data.parts.find((p) => p.pn === pn)).filter(Boolean);
  const list = s.preview ? MINI.data.parts.filter(PRINTED) : parts;
  const time = s.time === 'print' ? `≈ ${fmt(printHours(), 0)} h of printing` : s.time;
  $('proto-step').innerHTML = `<span class="eyebrow">STEP ${MINI.step}${time ? ' · ' + esc(time.toUpperCase()) : ''}</span><h3>${esc(s.t)}</h3><p>${esc(s.what)}</p>
    <ol>${s.doIt.map((d) => `<li>${esc(d)}</li>`).join('')}</ol>${s.check ? `<p class="proto-check"><b>Check before moving on.</b> ${esc(s.check)}</p>` : ''}
    ${list.length ? `<div class="proto-parts">${list.map((p) => `<span><b>${esc(p.pn)}</b> ${esc(p.name)} × ${p.qty}</span>`).join('')}</div>` : ''}`;
  applyStep();
}
function applyStep() {
  const s = PROTO_STEPS[MINI.step], shown = new Set(), hot = new Set(s.add);
  if (s.all) for (const r of MINI.recs) shown.add(r.pn);
  else if (s.preview) for (const r of MINI.recs) { if (PRINTED(r.p)) { shown.add(r.pn); hot.add(r.pn); } }
  else for (let k = 0; k <= MINI.step; k++) PROTO_STEPS[k].add.forEach((pn) => shown.add(pn));
  if (MINI.step === 0) for (const r of MINI.recs) shown.add(r.pn);
  for (const r of MINI.recs) {
    const body = r.p.system.startsWith('M10');
    r.holder.visible = shown.has(r.pn) && (MINI.shell || !body || hot.has(r.pn));
    r.me.material = hot.has(r.pn) ? MINI.hot : r.base;
  }
}
function tickProto(dt) {
  if (!MINI.anim) return;
  MINI.anim.t += dt;
  const ph = MINI.anim.t % 7;            // road 1 s → 2 s swing up → 1.5 s flight → 2 s swing down → 0.5 s
  MINI.conv = ph < 1 ? 0 : ph < 3 ? smooth((ph - 1) / 2) : ph < 4.5 ? 1 : ph < 6.5 ? 1 - smooth((ph - 4.5) / 2) : 0;
  poseMini(MINI.conv, MINI.explode); labelConv();
}
function renderProtoInputs() {
  const C = MINI.cfg;
  $('proto-inputs').innerHTML = PROTO_INPUTS.map(([g, list]) => `<h4>${g.toUpperCase()}</h4>` + list.map(([k, l, u, a, b, s]) => u === 'sel'
    ? `<label class="fin" for="pin-${k}">${l}<select id="pin-${k}" data-k="${k}">${a.map((o, i) => `<option value="${o.v ?? i}" ${(o.v ?? i) === C[k] ? 'selected' : ''}>${esc(o.label)}</option>`).join('')}</select></label>`
    : `<label class="fin" for="pin-${k}">${l}<output id="pout-${k}"></output><input id="pin-${k}" type="range" min="${a}" max="${b}" step="${s}" value="${C[k]}" data-k="${k}"></label>`).join('')).join('') +
    `<button class="tl-btn" id="pin-reset">Reset to the CADed build</button>`;
  $('proto-inputs').querySelectorAll('input, select').forEach((i) => i.addEventListener('input', () => { C[i.dataset.k] = +i.value; renderProto(); }));
  $('pin-reset').addEventListener('click', () => { MINI.cfg = { ...PROTO_DEFAULT }; renderProtoInputs(); renderProto(); });
}
function protoCost(c, M) {
  const rows = [], add = (grp, pn, name, qty, unit, note = '') => rows.push({ grp, pn, name, qty, unit, total: qty * unit, note });
  for (const p of MINI.data.parts) {
    if (p.supplier === 'buy' || p.pn.startsWith('JDM-50')) continue;
    if (p.material === 'cf_plate') continue;   // costed with the plate blank in the extras
    const f = FIL[p.material] || FIL.petg, g = p.mass_kg * 1000 * p.qty * 1.15;
    add('Printed', p.pn, `${p.name} (${f[0]}, ${fmt(g, 0)} g incl. waste)`, 1, g / 1000 * f[1], `${fmt(g / f[2], 1)} h`);
  }
  for (const p of MINI.data.parts.filter((q) => q.supplier === 'buy' || q.pn.startsWith('JDM-50'))) {
    if (p.pn.startsWith('JDM-50')) { if (M.custom) add('Fan motors (JDM-M40)', p.pn, p.name, p.qty, MOTOR_COST[p.pn] ?? 5); continue; }
    if (p.pn === 'JDM-401') add('Purchased', p.pn, M.custom ? '90 mm 12-blade fan unit, no motor (housing + rotor)' : M.e.label.replace(' (CADed)', '') + ' ducted fan unit with motor', 4, M.custom ? 45 : M.e.price);
    else if (p.pn === 'JDM-402') add('Purchased', p.pn, `${M.esc.A} A ESC, BLHeli_32 / AM32`, 4, M.esc.price);
    else if (p.pn === 'JDM-403') add('Purchased', p.pn, `${c.S}S ${c.mAh} mAh ${c.C}C LiPo`, c.packs, Math.round(.75 * M.packWh + 10));
    else if (p.pn === 'JDM-407') add('Purchased', p.pn, `${M.sv.tq} kg·cm metal-gear digital servo`, 4, M.sv.price);
    else add('Purchased', p.pn, p.name, p.qty, BUY[p.pn] ?? 10);
  }
  if (M.sleeve) add('Printed', 'JDM-206A', 'Fan adaptor sleeve (PETG)', 4, .5);
  if (M.custom) for (const [n, q, u] of MOTOR_EXTRAS) add('Fan motors (JDM-M40)', '', n, q, u);
  for (const [n, q, u] of EXTRAS) add('Hardware', '', n, q, u);
  for (const [n, q, u] of TOOLS) add('If you don’t own them', '', n, q, u);
  return rows;
}
function printHours() { return MINI.data.parts.filter((p) => PRINTED(p) && FIL[p.material]).reduce((a, p) => a + p.mass_kg * 1000 * p.qty * 1.15 / FIL[p.material][2], 0); }
function renderProto() {
  if (!MINI.cad) return;
  const c = MINI.cfg, M = protoModel(c);
  for (const g of PROTO_INPUTS) for (const [k, , u, , , s] of g[1]) { const o = $('pout-' + k); if (o) o.textContent = `${fmt(c[k], s >= 1 ? 0 : 2)}${u ? ' ' + u : ''}`; }
  const checks = protoChecks(c, M), nBad = checks.filter((x) => x[3] === 'bad').length, nMid = checks.filter((x) => x[3] === 'mid').length;
  const rows = protoCost(c, M), build = rows.filter((r) => r.grp !== 'If you don’t own them').reduce((a, b) => a + b.total, 0), tools = rows.filter((r) => r.grp === 'If you don’t own them').reduce((a, b) => a + b.total, 0);
  const printH = printHours();
  const st = (v, a, b) => (v >= a ? 'ok' : v >= b ? 'mid' : 'bad');
  $('proto-kpis').innerHTML = [
    ['PHYSICS CHECKS', `${checks.length - nBad - nMid} / ${checks.length} pass`, `${nMid} marginal · ${nBad} failing`, nBad ? 'bad' : nMid ? 'mid' : 'ok'],
    ['ALL-UP MASS', `${fmt(M.m, 2)} kg`, `CAD roll-up + ${fmt(c.misc * 1000, 0)} g wiring`, ''],
    ['THRUST / WEIGHT', `${fmt(M.twr, 2)} : 1`, `${fmt(4 * M.full.Tmax / 9.81, 1)} kg thrust`, st(M.twr, 2, 1.6)],
    ['HOVER TIME', `${fmt(M.hoverMin, 1)} min`, `${fmt(M.hv.I, 0)} A at ${fmt(M.hv.V, 1)} V`, st(M.hoverMin, 3, 2)],
    ['HOVER POWER', `${fmt(M.hv.I * M.hv.V, 0)} W`, `${fmt(M.m * 1000 / (M.hv.I * M.hv.V), 2)} g/W`, ''],
    ['BUILD COST', `$${fmt(build, 0)}`, `+ $${fmt(tools, 0)} radio & charger if needed`, ''],
    ['PRINT TIME', `${fmt(printH, 0)} h`, 'one 0.4 mm FDM printer', ''],
    ['DISK LOADING', `${fmt(M.dl, 0)} Pa`, `full-size car: ${fmt(flightModel(FLIGHT).dl, 0)} Pa`, ''],
  ].map(([k, v, s, cl]) => `<div class="kpi ${cl}"><small>${k}</small><b>${v}</b><span>${s}</span></div>`).join('') +
    `<div class="checks"><table class="checks-table"><thead><tr><th>Check</th><th>Value</th><th>Limit</th><th>Status</th><th>Why it matters</th></tr></thead><tbody>${checks.map(([n, v, l, s, w]) =>
      `<tr class="${s}"><td>${esc(n)}</td><td class="num">${esc(v)}</td><td>${esc(l)}</td><td><span class="pill ${s}">${s === 'ok' ? '● Pass' : s === 'mid' ? '▲ Marginal' : '■ Fail'}</span></td><td>${esc(w)}</td></tr>`).join('')}</tbody></table></div>`;
  const STL = REPO.replace('/blob/', '/raw/') + 'cad/mini/stl/', STEP = REPO.replace('/blob/', '/raw/') + 'cad/mini/step/';
  const printed = MINI.data.parts.filter(PRINTED);
  const grp = (g) => rows.filter((r) => r.grp === g);
  const table = (list) => list.map((r) => `<tr><td class="pn">${esc(r.pn)}</td><td>${esc(r.name)}</td><td class="num">${r.qty}</td><td class="num">$${fmt(r.unit, r.unit < 10 ? 2 : 0)}</td><td class="num">$${fmt(r.total, 0)}</td></tr>`).join('');
  const groupSum = (g) => grp(g).reduce((a, b) => a + b.total, 0);
  const full = MINI.data.meta, S = full.scale;
  $('proto-sections').innerHTML = `
    <article class="chart-card"><span class="ref">cad/mini · ${printed.length} made parts · STL for printing, STEP for editing</span><h4>Print and cut list</h4>
      <div class="proto-table"><table class="bom-table"><thead><tr><th>Part</th><th>Name</th><th>Qty</th><th>Material and settings</th><th>Mass each</th><th>Files</th></tr></thead><tbody>${printed.map((p) =>
        `<tr><td class="pn">${esc(p.pn)}</td><td>${esc(p.name)}</td><td class="num">${p.qty}${p.handed ? ' (2 RH + 2 LH)' : ''}</td><td>${esc(p.spec.printer || p.spec.cut || '')}</td><td class="num">${fmt(p.mass_kg * 1000, 0)} g</td>
        <td><a href="${STL}${p.pn}.stl" target="_blank" rel="noopener">STL</a>${p.handed ? ` · <a href="${STL}${p.pn}-LH.stl" target="_blank" rel="noopener">LH</a>` : ''} · <a href="${STEP}${p.pn}.step" target="_blank" rel="noopener">STEP</a></td></tr>`).join('')}</tbody></table></div>
      <p>Full assembly: <a href="${REPO.replace('/blob/', '/raw/')}cad/mini/Joukowsky-Dynamics-mini-prototype-assembly.step.zip" target="_blank" rel="noopener">assembly STEP (zip)</a> · <a href="${REPO}cad/mini/Joukowsky-Dynamics-mini-prototype-BOM.csv" target="_blank" rel="noopener">BOM CSV</a>. Masses are CAD volumes at the stated effective densities.</p></article>
    <article class="chart-card"><span class="ref">prices are typical 2026 hobby-shop figures in USD · your configuration</span><h4>Parts list and cost</h4>
      <div class="bom-summary"><span>Build <b>$${fmt(build, 0)}</b></span><span>Printed <b>$${fmt(groupSum('Printed'), 0)}</b></span><span>Purchased <b>$${fmt(groupSum('Purchased'), 0)}</b></span>${M.custom ? `<span>Fan motors <b>$${fmt(groupSum('Fan motors (JDM-M40)'), 0)}</b></span>` : ''}<span>Hardware <b>$${fmt(groupSum('Hardware'), 0)}</b></span><span>Radio, charger, bag <b>$${fmt(tools, 0)}</b></span></div>
      <div class="proto-table"><table class="bom-table"><thead><tr><th>Part</th><th>Item</th><th>Qty</th><th>Unit</th><th>Total</th></tr></thead><tbody>
        ${['Purchased', 'Fan motors (JDM-M40)', 'Printed', 'Hardware', 'If you don’t own them'].map((g) => `<tr class="grp"><td colspan="5">${esc(g.toUpperCase())}</td></tr>${table(grp(g))}`).join('')}</tbody></table></div></article>
    <article class="chart-card"><span class="ref">battery packs in parallel · one PDB · four ESCs · servo BEC · ArduPilot flight controller</span><h4>Wiring diagram</h4>${wiringSvg(c, M)}
      <p>Motor outputs follow ArduPilot Quad X order: 1 front-right, 2 rear-left, 3 front-left, 4 rear-right. Servos take outputs 5–8 but their power comes from the BEC, because four servos holding against thrust can draw more than a flight controller’s 5 V rail can give. Leave a loop in each motor lead where it crosses the hinge.</p></article>
    ${motorSection(c, M)}
    <div class="flight-notes">
      <div><h4>What the miniature proves</h4><ul>
        <li>The pod swing: hinge position, clearances to the body and tyre, and the actuator torque, with the full-size layout.</li>
        <li>That four pods in their flight position can hover the car body, with the CG over the thrust centre and the battery where the configurator predicts.</li>
        <li>Vectored yaw by differential pod tilt, which the full-size car also needs because its ducts have stator vanes.</li>
        <li>The conversion logic: arming locked out until all four pods report flight position.</li></ul></div>
      <div><h4>What it does not prove</h4><ul>
        <li>Road driving: the tyre rings are fixed to the ducts, so the miniature rolls on its pods only when pushed. The full-size wheel drive is not scaled.</li>
        <li>Proprotor aerodynamics: off-the-shelf fans are fixed-pitch, so the NASA-memo blade design, collective trim and petal diffuser are not tested.</li>
        <li>Dynamics at full scale: Froude scaling at 1 : ${fmt(1 / S, 1)} would need ${fmt(2249 * S ** 3, 2)} kg; this build is ${fmt(M.m / (2249 * S ** 3), 1)} × heavier, so time scales and gust response do not carry over directly.</li>
        <li>Battery heat, crash safety and the parachute.</li></ul></div>
      <div><h4>Scale data</h4><table class="alloc">
        <tr><th></th><th>Full size</th><th>Miniature</th></tr>
        <tr><th>Length scale</th><td>1</td><td>1 : ${fmt(1 / S, 1)}</td></tr>
        <tr><th>Wheelbase</th><td>${fmt(full.wheelbase / S * 1000, 0)} mm</td><td>${fmt(full.wheelbase * 1000, 0)} mm</td></tr>
        <tr><th>Track</th><td>${fmt((full.track - .024) / S * 1000, 0)} mm</td><td>${fmt(full.track * 1000, 0)} mm</td></tr>
        <tr><th>Tyre radius</th><td>740 mm</td><td>${fmt(full.tire_R * 1000, 1)} mm</td></tr>
        <tr><th>Mass</th><td>2,249 kg</td><td>${fmt(M.m, 2)} kg</td></tr>
        <tr><th>Disk loading</th><td>${fmt(flightModel(FLIGHT).dl, 0)} Pa</td><td>${fmt(M.dl, 0)} Pa</td></tr>
        <tr><th>Conversion time</th><td>6 s</td><td>2 s (Froude ${fmt(6 * Math.sqrt(S), 1)} s)</td></tr></table>
        <p>Pods are about 2.5 × deeper than scale so a ducted fan fits; the pockets in the body are cut deeper to match.</p></div>
    </div>`;
}
// --- 08 · custom fan motor section
const MOTOR_STEPS = [
  ['Laminations', 'Order the stator as a stack of 0.2 mm NO20 electrical-steel laminations from a laser or wire-EDM lamination service (STEP outline JDM-501), or buy a 12-tooth stator blank with the same outside diameter, bore and stack height. Stack them on a 14.4 mm mandrel with the teeth aligned, bond with thin bonding varnish, deburr the slot edges, then insulate the slots with 0.1 mm Nomex paper or a coat of epoxy powder.'],
  ['Wind the teeth', 'Wind every tooth in the same direction: 10 turns of Ø0.46 mm class-H (200 °C) magnet wire, tight and in layers, starting at the yoke. Mark each start lead with a dab of paint. A printed winding comb holding the lamination stack makes this a 40-minute job.'],
  ['Connect the coils', 'Follow the winding table: in each phase the four coils are joined in parallel, with “reversed” coils connected finish-to-start. Then join the three phases in delta (A finish → B start, B finish → C start, C finish → A start) and solder the three leads. Sleeve every joint.'],
  ['Varnish', 'Dip or trickle impregnating varnish into the winding and bake it as the varnish datasheet says. This locks the turns against vibration and adds a second layer of insulation.'],
  ['Rotor bell', 'Turn the 4140 tube to 40.0 mm outside diameter with a 34.0 mm bore, face it to length, and drill the six vent holes and the shaft boss (drawing JDM-504). The bore must run true within 0.02 mm, or the magnets will rub.'],
  ['Magnets', 'Bond the 10 arcs inside the bell with high-temperature magnet epoxy, poles alternating N / S facing inward, using a printed spacer ring to keep the gaps even. Check the alternation with a compass before the epoxy sets. Centrifugal force presses outrunner magnets into the bell, so the glue only has to locate them.'],
  ['Hub and bearings', 'Press the two 685 hybrid ceramic bearings into the turned 6061 hub with retaining compound, then press the stator onto the hub, leads to the rear.'],
  ['Assemble', 'Push the shaft through the bearings, press the bell onto the shaft’s flat and lock it with a grub screw, and fit the rear collar with 0.05 mm of end play. Turn it by hand: it should cog evenly with no rubbing.'],
  ['Check Kv', 'Spin the motor with a drill at a known speed and measure the line-to-line AC voltage. Kv ≈ rpm ÷ (1.41 × V rms). Expect __KV__ rpm/V ± 5 %. Measure the three line-to-line resistances: they must match within 3 %.'],
  ['Balance and test', 'Fit the fan rotor and balance the pair on a magnetic balancer. Bench-run on a current-limited supply, then 3 min at hover throttle in the duct with an IR thermometer: winding below 80 °C. Repeat for all four motors.'],
];
function windingSvg(D) {
  const cx = 150, cy = 150, ph = { A: 'var(--s1)', B: 'var(--s2)', C: 'var(--s3)' };
  let s = `<svg viewBox="0 0 300 300" class="winding" role="img" aria-label="Winding layout: ${D.Ns} teeth and ${D.P} magnets">`;
  for (let k = 0; k < D.P; k++) {   // magnets on the bell
    const a0 = (k / D.P) * 2 * Math.PI, a1 = a0 + 2 * Math.PI / D.P * .86, r0 = 118, r1 = 132;
    const pt = (r, a) => `${(cx + r * Math.cos(a)).toFixed(1)} ${(cy + r * Math.sin(a)).toFixed(1)}`;
    s += `<path d="M${pt(r0, a0)} A${r0} ${r0} 0 0 1 ${pt(r0, a1)} L${pt(r1, a1)} A${r1} ${r1} 0 0 0 ${pt(r1, a0)}Z" class="${k % 2 ? 'ms' : 'mn'}"/>`;
    const am = (a0 + a1) / 2; s += `<text x="${cx + 140 * Math.cos(am)}" y="${cy + 140 * Math.sin(am) + 3}" text-anchor="middle">${k % 2 ? 'S' : 'N'}</text>`;
  }
  [...D.pattern].forEach((ch, k) => {
    const a = (k + .5) / D.Ns * 2 * Math.PI - Math.PI / 2, P = ch.toUpperCase(), rev = ch !== P;
    const x = cx + 82 * Math.cos(a), y = cy + 82 * Math.sin(a);
    s += `<g transform="translate(${x.toFixed(1)} ${y.toFixed(1)}) rotate(${(a * 180 / Math.PI + 90).toFixed(1)})"><rect x="-10" y="-22" width="20" height="44" rx="3" fill="${ph[P]}" opacity="${rev ? .45 : .9}"/></g>`;
    s += `<text x="${cx + 82 * Math.cos(a)}" y="${cy + 82 * Math.sin(a) + 4}" text-anchor="middle" class="ttl">${P}${rev ? '′' : ''}</text>`;
    s += `<text x="${cx + 50 * Math.cos(a)}" y="${cy + 50 * Math.sin(a) + 3}" text-anchor="middle">${k + 1}</text>`;
  });
  s += `<circle cx="${cx}" cy="${cy}" r="34" class="hub"/><text x="${cx}" y="${cy + 4}" text-anchor="middle">Ø14.4</text></svg>`;
  return s;
}
function motorSection(c, M) {
  const MT = MINI.data.meta.motor, D = MT.design, E = MT.eval;
  const teeth = { A: [], B: [], C: [] };
  [...D.pattern].forEach((ch, k) => teeth[ch.toUpperCase()].push(`${k + 1}${ch === ch.toUpperCase() ? '' : '′'}`));
  const cadM = MINI.cad.motorM;
  let ops = '', checks = '', chart = '';
  if (M.custom) {
    const hv = M.hv.per.reduce((a, b) => (b.Im > a.Im ? b : a)), fu = M.full.fmax, fc = M.fresh.fmax;
    const tH = motorThermal(hv), tF = motorThermal(fu), Th = 40 + tH.dT, Tb = Th + (tF.dT - tH.dT) * (1 - Math.exp(-20 / tF.tau));
    const row = (n, r, V, T) => `<tr><th>${n}</th><td>${fmt(r.rpm, 0)}</td><td>${fmt(r.T / 9.81, 2)}</td><td>${fmt(r.I, 1)}</td><td>${fmt(V, 1)}</td><td>${fmt(r.eff * 100, 1)} %</td><td>${fmt(r.Pcu + r.Pf, 0)} W</td><td>${T == null ? '—' : fmt(T, 0) + ' °C'}</td></tr>`;
    ops = `<div style="overflow-x:auto"><table class="alloc"><tr><th></th><th>rpm</th><th>thrust kg</th><th>motor A</th><th>motor V</th><th>efficiency</th><th>losses</th><th>winding</th></tr>
      ${row('Hover (most-loaded fan)', hv, hv.Vm, Th)}${row('Full throttle, this battery', fu, fu.Vm, Tb)}${row('Full throttle, freshly charged pack', fc, fc.Vm, null)}</table></div>
      <p>Winding temperature: steady in hover with 40 °C air; full throttle is a 20 s burst starting from hover temperature (thermal time constant ${fmt(tF.tau, 0)} s).</p>`;
    const k = (ok, mid) => (ok ? 'ok' : mid ? 'mid' : 'bad'), erpm = fc.rpm * D.p, os = fc.rpm * 1.15;
    const rows = [
      ['Fits the fan’s motor hub', 'Ø40 × 26.5 mm bell', 'hub Ø41 × 32 mm', 'ok', 'Same envelope as the 40 mm motors these fans are sold for.'],
      ['Winding temperature, hover', `${fmt(Th, 0)} °C`, '≤ 110 °C', k(Th <= 110, Th <= 130), `${fmt(hv.Pcu, 0)} W copper + ${fmt(hv.Pf, 0)} W iron, eddy and bearing losses into the fan’s exhaust.`],
      ['Winding temperature, 20 s full throttle', `${fmt(Tb, 0)} °C`, '≤ 150 °C (wire rated 200 °C)', k(Tb <= 150, Tb <= 180), 'Class-H wire and varnish; magnets are N45SH (rated to 150 °C).'],
      ['Motor current, full throttle', `${fmt(fu.I, 0)} A`, `≤ 85 % of ${M.esc.A} A ESC`, k(fu.I <= .85 * M.esc.A, fu.I <= M.esc.A), 'Current through the ESC at full duty on the sagged pack.'],
      ['Efficiency in hover', `${fmt(hv.eff * 100, 1)} %`, '≥ 85 %', k(hv.eff >= .85, hv.eff >= .8), 'Shaft power ÷ electrical input at the hover operating point.'],
      ['ESC electrical speed', `${fmt(erpm / 1000, 0)}k erpm`, '≤ 300k (AM32 / BLHeli_32)', k(erpm <= 300e3, erpm <= 400e3), `${D.P / 2} pole pairs at ${fmt(fc.rpm, 0)} rpm on a freshly charged pack. Iron runs at ${fmt(fc.rpm * D.p / 60, 0)} Hz, which is why the laminations are 0.2 mm.`],
      ['Bearing speed', `${fmt(os, 0)} rpm at 115 %`, '≤ 60,000 rpm (685 hybrid ceramic)', k(os <= 60000, os <= 70000), 'Ceramic balls run cooler and faster than steel; keep them lightly greased, not packed.'],
      ['Rotor bell strength', `safety factor ${fmt(E.sf_bell, 1)}`, '≥ 2 on yield at 115 % speed', k(E.sf_bell >= 2, E.sf_bell >= 1.5), `Hoop stress from the spinning bell plus magnet pressure, at ${fmt(E.rpm_os, 0)} rpm (4140 pre-hardened, 650 MPa yield).`],
      ['Magnet demagnetisation', `margin ${fmt(E.demag_margin, 1)}`, '≥ 1.5 at 150 % current, 120 °C', k(E.demag_margin >= 1.5, E.demag_margin >= 1.2), 'Armature field at burst current compared with N45SH coercivity when hot.'],
      ['Flux densities', `gap ${fmt(D.Bg, 2)} T · tooth ${fmt(D.B_tooth, 2)} T · bell ${fmt(D.B_bell, 2)} T`, 'tooth ≤ 1.65, bell ≤ 1.65', 'ok', 'Held below saturation so Kv and torque stay linear.'],
    ];
    const pill = (s) => `<span class="pill ${s}">${s === 'ok' ? '● Pass' : s === 'mid' ? '▲ Marginal' : '■ Fail'}</span>`;
    checks = `<div class="checks proto-table"><table class="checks-table"><thead><tr><th>Check</th><th>Value</th><th>Limit</th><th>Status</th><th>Why it matters</th></tr></thead><tbody>${rows.map(([n, v, l, s, w]) => `<tr><td>${esc(n)}</td><td class="num">${esc(v)}</td><td>${esc(l)}</td><td>${pill(s)}</td><td>${esc(w)}</td></tr>`).join('')}</tbody></table></div>`;
    const pts = []; for (let i = 1; i <= 24; i++) { const r = motorOnFan(fu.T * i / 24, 1e9, c.kInst); pts.push(r); }
    chart = `<div class="charts">${lineChart('Efficiency across the throttle range', 'JDM-M40 on the 90 mm fan', `Motor efficiency against the fan's thrust, up to full throttle on this battery (${fmt(fu.T / 9.81, 2)} kg). The dashed line is hover.`,
      { x: [0, Math.ceil(fu.T / 9.81 * 2) / 2], y: [.6, 1], xl: 'thrust per fan, kg', yl: 'efficiency', vline: [hv.T / 9.81, 'hover'], series: [{ name: 'efficiency', c: 'var(--s1)', pts: pts.map((r) => [r.T / 9.81, r.eff]) }] })}
      ${lineChart('Current and losses across the throttle range', 'torque balance with the fan load', 'Motor current rises with thrust^1.5. Losses are copper (grows with current²) plus iron, eddy and bearing losses (grow with speed).',
      { x: [0, Math.ceil(fu.T / 9.81 * 2) / 2], y: [0, Math.ceil(fu.I / 20) * 20], xl: 'thrust per fan, kg', yl: 'A  or  W', vline: [hv.T / 9.81, 'hover'], series: [{ name: 'motor current, A', c: 'var(--s2)', pts: pts.map((r) => [r.T / 9.81, r.I]) }, { name: 'losses, W', c: 'var(--s3)', pts: pts.map((r) => [r.T / 9.81, r.Pcu + r.Pf]) }] })}</div>`;
  }
  const spec = [['Type', `outrunner, ${D.Ns} slots / ${D.P} poles, sensorless`], ['Kv', `${fmt(D.Kv, 0)} rpm/V`], ['Winding', `${D.N} turns Ø${fmt(D.d_wire * 1000, 2)} mm per tooth · ${D.conn}, ${D.paths} parallel paths`],
    ['Resistance (line-line)', `${fmt(D.R * 1000, 1)} mΩ hot`], ['Bell', `Ø40.0 × ${fmt(D.L_bell * 1000 + 1.5, 1)} mm, 4140 steel, ${fmt(D.t_bell * 1000, 1)} mm wall`], ['Stator', `Ø${fmt(D.r_s * 2000, 1)} × ${fmt(D.L * 1000, 0)} mm, 0.2 mm NO20, ${fmt(D.wt * 1000, 2)} mm teeth`],
    ['Magnets', `${D.P} × N45SH arcs, ${fmt(D.hm * 1000, 1)} mm thick, air gap ${fmt(D.g * 1000, 1)} mm`], ['Shaft / bearings', 'Ø5 mm, M5 front thread · 2 × 685 hybrid ceramic'], ['Mass', `${fmt(cadM * 1000, 0)} g (CAD), ${fmt(D.m * 1000, 0)} g (sizing estimate)`]];
  return `<article class="chart-card motor-card"><span class="ref">cad/source/motor_design.py · ${fmt(MT.sweep.tried / 1e6, 1)} million candidate designs, ${MT.sweep.passed} passed every limit, the lightest kept</span>
    <h4>Custom fan motor: JDM-M40</h4>
    <p>A 40 mm outrunner you build yourself, sized for this fan on this battery rather than bought off the shelf. The sizing script swept slot / pole combinations, stack length, magnet thickness, tooth width, turns and connection. For each candidate it worked out the magnetic circuit, copper, losses, temperature, rotor stress and demagnetisation, and matched it to the fan's torque curve. Unbalanced 9-slot combinations were excluded because their sideways magnetic pull shakes the rotor at 35,000 rpm.${M.custom ? '' : ' <b>Select the JDM-M40 under Fan motor to see its numbers on your configuration.</b>'}</p>
    <div class="motor-grid"><div><table class="alloc spec">${spec.map(([k, v]) => `<tr><th>${k}</th><td>${esc(v)}</td></tr>`).join('')}</table></div>
      <div class="motor-wind">${windingSvg(D)}<p>Coil order around the stator: ${D.pattern.split('').map((ch) => ch === ch.toUpperCase() ? ch : ch.toUpperCase() + '′').join(' ')} (′ = connected reversed). Phase A: teeth ${teeth.A.join(', ')} · B: ${teeth.B.join(', ')} · C: ${teeth.C.join(', ')}; each phase's four coils in parallel, phases in delta.</p></div></div>
    ${ops}${checks}${chart}
    <h4>Build one motor</h4><ol class="motor-steps">${MOTOR_STEPS.map(([t, d]) => `<li><b>${esc(t)}.</b> ${esc(d.replace('__KV__', fmt(D.Kv, 0)))}</li>`).join('')}</ol>
    <p>You need a small lathe (or a machine shop for the bell and hub, about $25 each), a soldering iron, a multimeter, a drill for the Kv test and a magnetic prop balancer. The STEP files for the stator outline, bell, hub and shaft are in <a href="${REPO}cad/mini/step/" target="_blank" rel="noopener">cad/mini/step</a>.</p></article>`;
}
function wiringSvg(c, M) {
  const box = (x, y, w, h, t, s, cl = '') => `<g class="wb ${cl}"><rect x="${x}" y="${y}" width="${w}" height="${h}" rx="3"/><text x="${x + w / 2}" y="${y + h / 2 - (s ? 3 : -4)}" text-anchor="middle" class="ttl">${t}</text>${s ? `<text x="${x + w / 2}" y="${y + h / 2 + 11}" text-anchor="middle">${s}</text>` : ''}</g>`;
  const ln = (d, cl) => `<path d="${d}" class="${cl}" fill="none"/>`;
  const Y = [20, 100, 180, 260];
  let s = `<svg viewBox="0 0 760 350" class="wiring" role="img" aria-label="Wiring diagram: battery to PDB, four ESCs and fans, servo BEC, flight controller">`;
  s += ln('M110 165 H170', 'pw') + Y.map((y) => ln(`M290 165 H330 V${y + 20} H380`, 'pw')).join('') + ln('M230 200 V260', 'pw');
  s += Y.map((y) => ln(`M480 ${y + 20} H540`, 'ph')).join('');
  s += Y.map((y, i) => ln(`M430 ${y + 40} V${y + 52} H625 V${150 + i * 12} H640`, 'sg')).join('');
  s += ln('M260 130 V8 H620 V190 H640', 'lv') + ln('M290 282 H756 V40 H750', 'lv') + ln('M695 140 V60', 'sg') + ln('M695 200 V240', 'sg');
  s += box(10, 140, 100, 50, `${c.S}S LiPo`, `${c.mAh} mAh × ${c.packs}`) + box(170, 130, 120, 70, 'PDB', 'XT90 · 4 ESC pads') + box(170, 260, 120, 44, 'Servo BEC', '7.4 V · 8 A');
  ['FR · out 1', 'RL · out 2', 'FL · out 3', 'RR · out 4'].forEach((n, i) => { s += box(380, Y[i], 100, 40, 'ESC', `${n} · ${M.esc.A} A`) + box(540, Y[i], 70, 40, 'EDF', `${fmt(M.e.od * 1000 - 8, 0)} mm`); });
  s += box(640, 140, 110, 60, 'Flight controller', 'ArduPilot H7', 'fc') + box(640, 20, 110, 40, '4 pod servos', `out 5–8 · ${M.sv.tq} kg·cm`) + box(640, 240, 110, 40, 'GNSS · receiver', 'UART');
  s += `<g class="wl"><path d="M10 334 H34" class="pw"/><text x="40" y="338">battery power</text><path d="M150 334 H174" class="lv"/><text x="180" y="338">5–7.4 V</text>
    <path d="M260 334 H284" class="sg"/><text x="290" y="338">signal</text><path d="M350 334 H374" class="ph"/><text x="380" y="338">3-phase motor</text></g></svg>`;
  return s;
}

// open the remembered tab, or a deep link (#sequence / #advanced / #flight / #prototype)
const deep = location.hash.replace('#', '');
const startTab = MY_TABS[deep] ? deep : MY_TABS[document.body.dataset.startTab] ? document.body.dataset.startTab : null;
if (startTab) document.querySelector(`[data-tab="${startTab}"]`)?.click();
window.__jd = { _dbg: { OBJS, PARTS, NODE }, focus: (v, id) => focusRegion(v, id), unfocus: () => exitFocus(true), seek: (t) => { seq.playing = false; seq.snap = true; syncPlay(); setSeqTime(t); }, style: (s) => document.querySelector(`[data-seqstyle="${s}"]`).click(), showTab, get DATA() { return DATA; }, MASS, flightModel: () => flightModel(FLIGHT), MINI, protoModel: () => protoModel(MINI.cfg), protoChecks: () => protoChecks(MINI.cfg, protoModel(MINI.cfg)), setStep: (i) => setStep(i), camTo: (t) => { const v = views.prototype; v.controls.target.copy(t); v.camera.position.set(t.x + .09, t.y + .05, t.z + .16); } };
