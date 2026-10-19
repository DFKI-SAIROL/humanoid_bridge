// Per-motor jog panel for the G1. Vanilla ES modules, no build step, no runtime CDN.
//
// Vendored deps are pinned under ./vendor/ (three 0.180.0, urdf-loader 0.13.1).
// Refetch commands live in vendor/README.md.
//
// Layering: the motor list and the control panel are built first and work alone.
// The 3D view is loaded afterwards by dynamic import and is allowed to fail.

// ------------------------------------------------------------------ constants

// The only shipped description with all 29 body joints AND the Dex3 hands.
const URDF_FILE = 'g1_29dof_with_hand.urdf';
const ASSETS_BASE = '/assets/';
const URDF_URL = ASSETS_BASE + URDF_FILE;

const API = {
  catalog: '/api/catalog',
  start: '/api/control/start',
  stop: '/api/control/stop',
  select: '/api/select',
  target: '/api/target',
  mode: '/api/mode',
  stream: '/api/stream',
};

const PING_MS = 250; // server stops commanding after 500 ms without a ping
const TARGET_MIN_INTERVAL_MS = 50; // ~20 POST /api/target per second
const HTTP_TIMEOUT_MS = 4000;
const WS_BACKOFF_MS = [250, 500, 1000, 2000, 4000, 8000];
const STALE_AFTER_MS = 1000; // no frame for this long: readouts are history
const SELECT_SETTLE_MS = 1000; // ignore stream `selected` this long after a local pick
const CATALOG_RETRY_MS = 3000;

const DIVERGE_WARN = 0.05; // rad
const DIVERGE_BAD = 0.15; // rad
const DIVERGE_FULL = 0.4; // rad that fills the divergence bar

const GROUP_ORDER = ['body', 'left_hand', 'right_hand'];
const GROUP_LABELS = { body: 'Body', left_hand: 'Left hand', right_hand: 'Right hand' };

// -------------------------------------------------------------- dom helpers

const el = (id) => document.getElementById(id);
const fmt = (v) => (Number.isFinite(v) ? v.toFixed(3) : '—');
const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);

function setText(node, value) {
  if (node && node.textContent !== value) node.textContent = value;
}

function setClass(node, name, on) {
  if (node) node.classList.toggle(name, !!on);
}

// -------------------------------------------------------------------- state

const state = {
  motors: [],
  byName: new Map(),
  byUrdf: new Map(),
  selected: null, // config name, e.g. "L_ELBOW"
  selectPending: null, // name of an in-flight POST /api/select
  lastSelectAt: 0, // local selections win over in-flight stream frames
  started: false,
  limp: false,
  linkUp: false,
  meas: new Map(), // name -> {q_cmd, q_meas, dq, tau}
  lastFrameAt: 0,
  dirty: false,
};

// ------------------------------------------------------------------ banners

const bannerNodes = new Map();

function setBanner(id, level, html) {
  let node = bannerNodes.get(id);
  if (!node) {
    node = document.createElement('div');
    bannerNodes.set(id, node);
    el('banners').appendChild(node);
  }
  node.className = `banner is-${level}`;
  node.innerHTML = html;
}

function clearBanner(id) {
  const node = bannerNodes.get(id);
  if (node) {
    node.remove();
    bannerNodes.delete(id);
  }
}

// --------------------------------------------------------------- http client

async function postJSON(path, body) {
  const init = {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body || {}),
  };
  if (typeof AbortSignal !== 'undefined' && AbortSignal.timeout) {
    init.signal = AbortSignal.timeout(HTTP_TIMEOUT_MS);
  }
  const res = await fetch(path, init);
  if (!res.ok) throw new Error(`HTTP ${res.status} from ${path}`);
  return res.json();
}

async function getJSON(path) {
  const res = await fetch(path, { headers: { Accept: 'application/json' } });
  if (!res.ok) throw new Error(`HTTP ${res.status} from ${path}`);
  return res.json();
}

// ----------------------------------------------------------------- commands

async function control(which) {
  const path = which === 'start' ? API.start : API.stop;
  try {
    const reply = await postJSON(path);
    if (reply && reply.ok === false) {
      setBanner('control', 'warn', `<b>${which.toUpperCase()} refused</b>${escapeHtml(reply.message || '')}`);
    } else {
      clearBanner('control');
    }
  } catch (err) {
    if (which === 'stop') {
      // Last resort: drop the socket so the server's deadman trips within 500 ms.
      dropSocket();
      setBanner('control', 'bad',
        '<b>STOP request failed &mdash; socket dropped instead</b>' +
        'The deadman switch stops the robot within 500&nbsp;ms of the lost link. ' +
        `Reason: ${escapeHtml(err.message)}`);
    } else {
      setBanner('control', 'bad', `<b>START failed</b>${escapeHtml(err.message)}`);
    }
  }
}

async function selectMotor(name, push = true) {
  const motor = state.byName.get(name);
  if (!motor) return;
  if (state.selected === name && !push) return;
  state.selected = name;
  renderSelection();
  if (!push) return;
  state.selectPending = name;
  state.lastSelectAt = performance.now();
  try {
    const reply = await postJSON(API.select, { name });
    if (reply && reply.ok === false) {
      setBanner('select', 'warn', `<b>Server refused select</b>${escapeHtml(name)}`);
    } else {
      clearBanner('select');
    }
  } catch (err) {
    setBanner('select', 'bad', `<b>Select failed</b>${escapeHtml(err.message)}`);
  } finally {
    if (state.selectPending === name) state.selectPending = null;
  }
}

async function setLimp(limp) {
  try {
    const reply = await postJSON(API.mode, { limp });
    if (reply && reply.ok === false) {
      setBanner('mode', 'warn', '<b>Server refused the limp-mode change</b>');
    } else {
      clearBanner('mode');
    }
  } catch (err) {
    setBanner('mode', 'bad', `<b>Limp-mode change failed</b>${escapeHtml(err.message)}`);
    el('chk-limp').checked = state.limp; // put the box back where the robot actually is
  }
}

// Throttled target sender: at most one POST per TARGET_MIN_INTERVAL_MS while
// dragging, and always one final POST on release.
let targetTimer = null;
let targetPending = null;
let targetLastSent = 0;
let targetSeq = 0;

function queueTarget(name, q) {
  targetPending = { name, q };
  if (targetTimer !== null) return;
  const wait = Math.max(0, TARGET_MIN_INTERVAL_MS - (performance.now() - targetLastSent));
  targetTimer = setTimeout(flushTarget, wait);
}

function flushTarget() {
  targetTimer = null;
  if (!targetPending) return;
  const { name, q } = targetPending;
  targetPending = null;
  targetLastSent = performance.now();
  sendTarget(name, q);
}

function sendTargetNow(name, q) {
  if (targetTimer !== null) {
    clearTimeout(targetTimer);
    targetTimer = null;
  }
  targetPending = null;
  targetLastSent = performance.now();
  sendTarget(name, q);
}

async function sendTarget(name, q) {
  const seq = ++targetSeq;
  try {
    const reply = await postJSON(API.target, { name, q });
    clearBanner('target');
    if (reply && reply.ok === false) {
      setBanner('target', 'warn', '<b>Server refused the target</b>');
      return;
    }
    // Adopt the server's clamp, but never yank a slider the user is holding.
    if (reply && Number.isFinite(reply.q) && seq === targetSeq) applyClamped(name, reply.q);
  } catch (err) {
    setBanner('target', 'bad', `<b>Target failed</b>${escapeHtml(err.message)}`);
  }
}

function escapeHtml(text) {
  const div = document.createElement('div');
  div.textContent = String(text == null ? '' : text);
  return div.innerHTML;
}

// ----------------------------------------------------- websocket + deadman

let ws = null;
let pingTimer = null;
let reconnectTimer = null;
let backoffIndex = 0;

function connect() {
  clearTimeout(reconnectTimer);
  reconnectTimer = null;
  const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
  setLink(false, 'connecting…', 'is-warn');
  try {
    ws = new WebSocket(`${proto}//${location.host}${API.stream}`);
  } catch (err) {
    console.error('[motor-probe] websocket construction failed', err);
    scheduleReconnect();
    return;
  }
  ws.onopen = () => {
    backoffIndex = 0;
    setLink(true, 'link live', 'is-ok');
    clearBanner('link');
    startPing();
  };
  ws.onmessage = onStreamMessage;
  ws.onerror = () => { /* onclose always follows; report there */ };
  ws.onclose = () => {
    stopPing();
    ws = null;
    onLinkLost();
    scheduleReconnect();
  };
}

function onLinkLost() {
  state.started = false;
  setLink(false, 'link down', 'is-bad');
  setBanner('link', 'bad',
    '<b>Stream is down &mdash; the robot is NOT being commanded</b>' +
    'No ping reaches the server, so the deadman switch has stopped it. ' +
    'Readouts below are the last values received. Reconnecting…');
  markStale(true);
}

function scheduleReconnect() {
  if (reconnectTimer !== null) return;
  const base = WS_BACKOFF_MS[Math.min(backoffIndex, WS_BACKOFF_MS.length - 1)];
  backoffIndex += 1;
  const wait = base + Math.floor(Math.random() * 200);
  reconnectTimer = setTimeout(() => {
    reconnectTimer = null;
    connect();
  }, wait);
}

// The deadman ping. It exists only while a socket is genuinely OPEN: no socket,
// no timer, and every tick re-checks readyState before sending.
function startPing() {
  stopPing();
  sendPing();
  pingTimer = setInterval(sendPing, PING_MS);
}

function stopPing() {
  if (pingTimer !== null) {
    clearInterval(pingTimer);
    pingTimer = null;
  }
}

function sendPing() {
  if (!ws || ws.readyState !== WebSocket.OPEN) {
    stopPing();
    return;
  }
  try {
    ws.send('{"type":"ping"}');
  } catch (err) {
    console.error('[motor-probe] ping failed', err);
    stopPing();
    dropSocket();
  }
}

function dropSocket() {
  stopPing();
  if (ws) {
    try { ws.close(); } catch { /* already closing */ }
  }
}

function onStreamMessage(event) {
  let msg;
  try {
    msg = JSON.parse(event.data);
  } catch {
    return;
  }
  if (!msg || typeof msg !== 'object' || !Array.isArray(msg.motors)) return;

  state.lastFrameAt = performance.now();
  state.started = !!msg.started;
  state.limp = !!msg.limp;

  // Safety-critical, so never gated on requestAnimationFrame: a stalled
  // renderer must not leave a stale "COMMANDING ROBOT" on screen.
  renderStartedPill();

  for (const m of msg.motors) {
    if (m && typeof m.name === 'string') state.meas.set(m.name, m);
  }

  // One selection model. Adopt the server's view, but not while our own select is
  // in flight and not from frames still in transit when we made it — a stale
  // `selected` would otherwise bounce the selection and reset the jog inputs.
  if (typeof msg.selected === 'string' && msg.selected !== state.selected
      && !state.selectPending
      && performance.now() - state.lastSelectAt > SELECT_SETTLE_MS) {
    selectMotor(msg.selected, false);
  }

  state.dirty = true;
}

// -------------------------------------------------------------- top bar ui

function setLink(up, text, cls) {
  state.linkUp = up;
  const pill = el('pill-conn');
  pill.className = `pill ${cls}`;
  setText(pill, text);
  renderStartedPill();
}

function renderStartedPill() {
  const pill = el('pill-started');
  if (!state.linkUp) {
    pill.className = 'pill is-bad';
    setText(pill, 'STOPPED (no link)');
    return;
  }
  if (state.started) {
    pill.className = 'pill is-warn';
    setText(pill, 'COMMANDING ROBOT');
  } else {
    pill.className = 'pill';
    setText(pill, 'not commanding');
  }
}

function markStale(stale) {
  setClass(el('rows'), 'is-stale', stale);
}

// ------------------------------------------------------------- motor list

const rowEls = new Map(); // motor name -> {row, meas, cmd, dv, fill, group, dvClass, last}
const groupHeads = new Map();

function buildList() {
  const host = el('rows');
  host.textContent = '';
  rowEls.clear();
  groupHeads.clear();

  for (const group of GROUP_ORDER) {
    const motors = state.motors.filter((m) => m.group === group);
    if (!motors.length) continue;

    const head = document.createElement('div');
    head.className = 'grouphead';
    head.textContent = `${GROUP_LABELS[group] || group} (${motors.length})`;
    host.appendChild(head);
    groupHeads.set(group, head);

    for (const motor of motors) {
      host.appendChild(buildRow(motor));
    }
  }
  setText(el('list-count'), `(${state.motors.length})`);
}

function buildRow(motor) {
  const row = document.createElement('button');
  row.type = 'button';
  row.className = 'row';
  row.dataset.name = motor.name;

  const nm = document.createElement('span');
  nm.className = 'nm';
  nm.textContent = motor.name;
  const uj = document.createElement('span');
  uj.className = 'uj';
  uj.textContent = motor.urdf_joint;
  nm.appendChild(uj);

  const meas = numCell('meas');
  const cmd = numCell('cmd');

  const divCell = document.createElement('span');
  divCell.className = 'divcell';
  const dv = document.createElement('b');
  dv.className = 'dv';
  dv.textContent = '—';
  const bar = document.createElement('span');
  bar.className = 'bar';
  const fill = document.createElement('span');
  fill.className = 'fill';
  bar.appendChild(fill);
  divCell.append(dv, bar);

  row.append(nm, meas.wrap, cmd.wrap, divCell);
  row.addEventListener('click', () => selectMotor(motor.name));

  rowEls.set(motor.name, {
    row,
    group: motor.group,
    meas: meas.value,
    cmd: cmd.value,
    dv,
    fill,
    dvClass: '',
    haystack: `${motor.name} ${motor.urdf_joint} ${motor.group}`.toLowerCase(),
    last: {},
  });
  return row;
}

function numCell(label) {
  const wrap = document.createElement('span');
  wrap.className = 'num';
  const value = document.createElement('b');
  value.textContent = '—';
  const tag = document.createElement('i');
  tag.textContent = label;
  wrap.append(value, tag);
  return { wrap, value };
}

function updateRows() {
  let classesChanged = false;

  for (const [name, r] of rowEls) {
    const m = state.meas.get(name);
    const qm = m ? Number(m.q_meas) : NaN;
    const qc = m ? Number(m.q_cmd) : NaN;
    const d = Number.isFinite(qm) && Number.isFinite(qc) ? Math.abs(qc - qm) : NaN;

    const sMeas = fmt(qm);
    const sCmd = fmt(qc);
    const sDiv = fmt(d);
    if (r.last.meas !== sMeas) { r.meas.textContent = sMeas; r.last.meas = sMeas; }
    if (r.last.cmd !== sCmd) { r.cmd.textContent = sCmd; r.last.cmd = sCmd; }
    if (r.last.dv !== sDiv) { r.dv.textContent = sDiv; r.last.dv = sDiv; }

    const pct = Number.isFinite(d) ? Math.round(clamp(d / DIVERGE_FULL, 0, 1) * 100) : 0;
    if (r.last.pct !== pct) { r.fill.style.width = `${pct}%`; r.last.pct = pct; }

    const cls = divergenceClass(d);
    if (cls !== r.dvClass) {
      r.row.classList.remove('dv-warn', 'dv-bad');
      if (cls) r.row.classList.add(cls);
      r.dvClass = cls;
      classesChanged = true;
    }
  }

  if (classesChanged && el('chk-diverging').checked) applyFilter();
}

function divergenceClass(d) {
  if (!Number.isFinite(d)) return '';
  if (d >= DIVERGE_BAD) return 'dv-bad';
  if (d >= DIVERGE_WARN) return 'dv-warn';
  return '';
}

function applyFilter() {
  const needle = el('list-filter').value.trim().toLowerCase();
  const divergingOnly = el('chk-diverging').checked;
  const visibleByGroup = new Map();

  for (const [, r] of rowEls) {
    let show = !needle || r.haystack.includes(needle);
    if (show && divergingOnly && !r.dvClass) show = false;
    r.row.hidden = !show;
    if (show) visibleByGroup.set(r.group, true);
  }
  for (const [group, head] of groupHeads) head.hidden = !visibleByGroup.get(group);
}

// ----------------------------------------------------------- control panel

const panel = {
  motor: null,
  held: false, // user is dragging the slider or typing in the number box
  lastLocalEdit: 0,
};

function renderSelection() {
  const motor = state.byName.get(state.selected) || null;
  const changed = (panel.motor && panel.motor.name) !== (motor && motor.name);
  panel.motor = motor;

  for (const [name, r] of rowEls) setClass(r.row, 'is-selected', name === state.selected);

  const slider = el('sel-slider');
  const num = el('sel-num');

  if (!motor) {
    setText(el('sel-name'), 'none');
    setText(el('sel-urdf'), '');
    el('sel-group').hidden = true;
    el('sel-hint').hidden = false;
    el('sel-controls').hidden = true;
    slider.disabled = true;
    num.disabled = true;
    viewer.setSelected(null);
    return;
  }

  setText(el('sel-name'), motor.name);
  setText(el('sel-urdf'), motor.urdf_joint);
  const tag = el('sel-group');
  tag.hidden = false;
  setText(tag, GROUP_LABELS[motor.group] || motor.group);
  el('sel-hint').hidden = true;
  el('sel-controls').hidden = false;

  const step = Math.max(0.0005, (motor.q_max - motor.q_min) / 2000);
  slider.min = motor.q_min;
  slider.max = motor.q_max;
  slider.step = step;
  slider.disabled = false;
  num.min = motor.q_min;
  num.max = motor.q_max;
  num.step = step;
  num.disabled = false;

  setText(el('sel-min'), fmt(motor.q_min));
  setText(el('sel-max'), fmt(motor.q_max));
  setText(el('sel-limits'),
    `q ∈ [${fmt(motor.q_min)}, ${fmt(motor.q_max)}] rad  ·  ` +
    `dq≤${motor.dq_limit}  ·  tau≤${motor.tau_limit}  ·  ` +
    `kp=${motor.kp} kd=${motor.kd}  ·  ${motor.group} idx ${motor.index}`);

  // Seed the inputs from where the motor is right now — only on a real change of
  // motor, so a re-render never yanks a value the user just dialled in.
  if (changed) {
    const m = state.meas.get(motor.name);
    const seed = m && Number.isFinite(Number(m.q_cmd)) ? Number(m.q_cmd)
      : m && Number.isFinite(Number(m.q_meas)) ? Number(m.q_meas)
        : 0;
    setInputs(clamp(seed, motor.q_min, motor.q_max));
  }

  viewer.setSelected(motor.urdf_joint);
  updatePanelReadouts();
}

function setInputs(q) {
  el('sel-slider').value = String(q);
  el('sel-num').value = q.toFixed(4);
}

function applyClamped(name, q) {
  if (!panel.motor || panel.motor.name !== name) return;
  if (panel.held) return;
  if (performance.now() - panel.lastLocalEdit < 400) return;
  setInputs(q);
}

function updatePanelReadouts() {
  const motor = panel.motor;
  if (!motor) return;
  const m = state.meas.get(motor.name);
  const qm = m ? Number(m.q_meas) : NaN;
  const qc = m ? Number(m.q_cmd) : NaN;
  const d = Number.isFinite(qm) && Number.isFinite(qc) ? Math.abs(qc - qm) : NaN;

  setText(el('sel-meas'), fmt(qm));
  setText(el('sel-cmd'), fmt(qc));
  setText(el('sel-div'), fmt(d));
  setText(el('sel-dq'), m ? fmt(Number(m.dq)) : '—');
  setText(el('sel-tau'), m ? fmt(Number(m.tau)) : '—');

  const cell = el('sel-div-cell');
  const cls = divergenceClass(d);
  setClass(cell, 'is-warn', cls === 'dv-warn');
  setClass(cell, 'is-bad', cls === 'dv-bad');
}

// `source` is the input the user is touching: the other one mirrors it, and the
// one being edited is never rewritten under the cursor.
function onJog(raw, final, source) {
  const motor = panel.motor;
  if (!motor) return;
  const q = clamp(Number(raw), motor.q_min, motor.q_max);
  if (!Number.isFinite(q)) return;
  panel.lastLocalEdit = performance.now();
  if (source === 'num') el('sel-slider').value = String(q);
  else el('sel-num').value = q.toFixed(4);
  if (final) sendTargetNow(motor.name, q);
  else queueTarget(motor.name, q);
}

// ------------------------------------------------------------- render loop

function frame() {
  requestAnimationFrame(frame);

  if (state.dirty) {
    state.dirty = false;
    updateRows();
    updatePanelReadouts();
    if (!el('chk-limp').matches(':focus')) el('chk-limp').checked = state.limp;
    setClass(el('limpstrip'), 'is-on', state.limp);
    viewer.applyPose(state.meas);
    markStale(false);
  } else if (state.linkUp && state.lastFrameAt && performance.now() - state.lastFrameAt > STALE_AFTER_MS) {
    markStale(true);
  }

  viewer.tick();
}

// -------------------------------------------------------------- 3D viewer

const NO_VIEWER = { ok: false, setSelected() {}, applyPose() {}, tick() {} };

// Replaced by the real viewer once the vendored libs and the URDF both load.
let viewer = NO_VIEWER;
let viewerTeardown = null; // drops the WebGL context if 3D is disabled later

function disable3D(title, detail, err) {
  if (err) console.error('[motor-probe] 3D disabled:', err);
  viewer = NO_VIEWER;
  if (viewerTeardown) {
    const teardown = viewerTeardown;
    viewerTeardown = null;
    try { teardown(); } catch (e) { console.error('[motor-probe] viewer teardown failed', e); }
  }
  document.body.classList.add('no-3d');
  setText(el('viewport-msg'), '');
  setBanner('viewer', 'warn', `<b>${title}</b>${detail}`);
}

async function initViewer() {
  const host = el('viewport');
  let THREE;
  let OrbitControls;
  let URDFLoader;
  try {
    THREE = await import('three');
    ({ OrbitControls } = await import('three/addons/controls/OrbitControls.js'));
    URDFLoader = (await import('urdf-loader')).default;
  } catch (err) {
    disable3D('3D libraries did not load from <code>static/vendor/</code>.',
      'Refetch them with the commands in <code>static/vendor/README.md</code>. ' +
      'The motor list is fully functional without them &mdash; selecting and jogging still work.',
      err);
    return;
  }

  let renderer;
  try {
    renderer = new THREE.WebGLRenderer({ antialias: true });
  } catch (err) {
    disable3D('WebGL is unavailable in this browser.',
      'The motor list is fully functional without it.', err);
    return;
  }
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  host.appendChild(renderer.domElement);

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100);
  camera.position.set(1.6, 1.2, 2.2);

  scene.add(new THREE.HemisphereLight(0xffffff, 0x445566, 2.0));
  const key = new THREE.DirectionalLight(0xffffff, 1.6);
  key.position.set(2.5, 4, 3);
  scene.add(key);
  const fill = new THREE.DirectionalLight(0xffffff, 0.5);
  fill.position.set(-2.5, 1.5, -2);
  scene.add(fill);

  const grid = new THREE.GridHelper(4, 16);
  grid.material.transparent = true;
  grid.material.opacity = 0.5;
  scene.add(grid);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.12;
  controls.target.set(0, 0.8, 0);

  // If 3D is disabled later (a missing URDF), drop the context instead of
  // leaving a hidden canvas rendering an empty scene every frame.
  viewerTeardown = () => {
    controls.dispose();
    renderer.domElement.remove();
    renderer.dispose();
  };

  // Match the page theme so the pane never looks pasted on.
  function applyViewerTheme() {
    const css = getComputedStyle(document.body);
    const read = (name, fallback) => (css.getPropertyValue(name).trim() || fallback);
    renderer.setClearColor(new THREE.Color(read('--surface', '#ffffff')), 1);
    grid.material.color = new THREE.Color(read('--line-strong', '#a9b3c1'));
  }
  applyViewerTheme();
  document.addEventListener('motorprobe:theme', applyViewerTheme);
  // Also follow the OS theme when the user has not picked one explicitly.
  if (window.matchMedia) {
    const dark = window.matchMedia('(prefers-color-scheme: dark)');
    if (dark.addEventListener) dark.addEventListener('change', applyViewerTheme);
  }

  function resize() {
    const w = host.clientWidth || 1;
    const h = host.clientHeight || 1;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
  resize();
  if (typeof ResizeObserver !== 'undefined') new ResizeObserver(resize).observe(host);
  else window.addEventListener('resize', resize);

  // ---- load the robot description -----------------------------------------

  let robot = null;
  let meshErrors = 0;

  const manager = new THREE.LoadingManager();
  manager.onError = (url) => {
    if (url !== URDF_URL) meshErrors += 1;
  };
  // Meshes arrive asynchronously, long after the URDF parse callback, so all
  // mesh work happens here — once every STL has settled.
  manager.onLoad = () => {
    if (!robot) return;
    setSelected(null);
    const meshCount = prepareMeshes();
    groundRobot();
    frameRobot();

    if (!meshCount) {
      setBanner('meshes', 'warn',
        '<b>The description parsed but no meshes loaded</b>' +
        'Run <code>scripts/fetch_g1_description.sh</code>. Selecting and jogging still work.');
    } else if (meshErrors > 0) {
      setBanner('meshes', 'warn',
        `<b>${meshErrors} mesh${meshErrors === 1 ? '' : 'es'} failed to load</b>` +
        'The model is incomplete. Refetch the description with ' +
        '<code>scripts/fetch_g1_description.sh</code>. Selecting and jogging still work.');
    } else {
      clearBanner('meshes');
    }

    setText(el('viewport-msg'),
      meshCount ? 'Click a joint to select it · drag to orbit · scroll to zoom' : '');

    const selected = state.byName.get(state.selected);
    if (selected) setSelected(selected.urdf_joint);
  };

  const loader = new URDFLoader(manager);
  loader.parseVisual = true;
  loader.parseCollision = false;

  setText(el('viewport-msg'), `Loading ${URDF_FILE}…`);

  loader.load(URDF_URL, onRobot, null, (err) => {
    disable3D(`Robot description not found at <code>${URDF_URL}</code>.`,
      'Fetch it with <code>scripts/fetch_g1_description.sh</code>. ' +
      'The motor list is fully functional without it &mdash; selecting and jogging still work.',
      err);
  });

  function onRobot(loaded) {
    robot = loaded;
    robot.rotation.x = -Math.PI / 2; // URDF is Z-up, three is Y-up
    scene.add(robot);
    setText(el('viewport-msg'), 'Loading meshes…');
  }

  // Returns the mesh count. Idempotent: manager.onLoad may fire more than once.
  function prepareMeshes() {
    let count = 0;
    robot.traverse((o) => {
      if (o.isURDFJoint) o.ignoreLimits = true; // show q_meas truthfully, past URDF limits
      if (!o.isMesh || !o.material) return;
      count += 1;
      if (o.userData.probePrepared) return;
      o.userData.probePrepared = true;
      // Clone: the URDF shares one material across links, and a highlight must not spread.
      o.material = Array.isArray(o.material) ? o.material.map((m) => m.clone()) : o.material.clone();
    });
    return count;
  }

  // Stand the robot on the grid. The URDF root is the pelvis, so left alone the
  // model floats and the floor cuts through its hips. Done once: the bounding
  // box shrinks as joints bend, and re-grounding every frame would make it bob.
  function groundRobot() {
    if (!robot || robot.userData.grounded) return;
    const box = new THREE.Box3().setFromObject(robot);
    if (box.isEmpty()) return;
    robot.position.y -= box.min.y;
    robot.userData.grounded = true;
  }

  function frameRobot() {
    if (!robot) return;
    const box = new THREE.Box3().setFromObject(robot);
    if (box.isEmpty()) return;
    const size = box.getSize(new THREE.Vector3());
    const center = box.getCenter(new THREE.Vector3());
    const radius = Math.max(size.x, size.y, size.z, 0.1) * 0.5;
    controls.target.copy(center);
    camera.position.set(center.x + radius * 1.4, center.y + radius * 0.7, center.z + radius * 2.6);
    camera.near = radius / 100;
    camera.far = radius * 200;
    camera.updateProjectionMatrix();
    controls.update();
  }

  // ---- picking -------------------------------------------------------------

  const raycaster = new THREE.Raycaster();
  const pointer = new THREE.Vector2();

  // Nearest URDF joint above a mesh. Non-commandable links resolve to a fixed
  // joint (or nothing), which is exactly what we want to ignore.
  function nearestJoint(obj) {
    let node = obj;
    while (node && node !== scene) {
      if (node.isURDFJoint) return node;
      node = node.parent;
    }
    return null;
  }

  function pick(event) {
    if (!robot) return null;
    const rect = host.getBoundingClientRect();
    if (!rect.width || !rect.height) return null;
    pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
    raycaster.setFromCamera(pointer, camera);
    for (const hit of raycaster.intersectObject(robot, true)) {
      const joint = nearestJoint(hit.object);
      if (joint) return joint;
    }
    return null;
  }

  let downAt = null;
  renderer.domElement.addEventListener('pointerdown', (e) => {
    downAt = { x: e.clientX, y: e.clientY };
  });
  renderer.domElement.addEventListener('pointerup', (e) => {
    const start = downAt;
    downAt = null;
    if (!start) return;
    if (Math.abs(e.clientX - start.x) > 4 || Math.abs(e.clientY - start.y) > 4) return; // orbit, not click
    const joint = pick(e);
    if (!joint) return;
    const motor = state.byUrdf.get(joint.name);
    if (motor) selectMotor(motor.name); // non-commandable joints: do nothing
  });

  const tip = el('hovertip');
  let hoverAt = 0;
  renderer.domElement.addEventListener('pointermove', (e) => {
    const now = performance.now();
    if (now - hoverAt < 40) return;
    hoverAt = now;
    const joint = pick(e);
    const motor = joint ? state.byUrdf.get(joint.name) : null;
    renderer.domElement.style.cursor = motor ? 'pointer' : 'default';
    if (!joint) {
      tip.hidden = true;
      return;
    }
    const rect = host.getBoundingClientRect();
    tip.hidden = false;
    tip.textContent = motor ? `${motor.name}  •  ${joint.name}` : `${joint.name} (not commandable)`;
    tip.style.left = `${e.clientX - rect.left}px`;
    tip.style.top = `${e.clientY - rect.top}px`;
  });
  renderer.domElement.addEventListener('pointerleave', () => { tip.hidden = true; });

  // ---- selection highlight -------------------------------------------------

  const highlighted = [];
  const HIGHLIGHT = new THREE.Color(0xff9500);

  function setSelected(urdfJoint) {
    for (const mesh of highlighted) {
      const saved = mesh.userData.probeHighlight;
      if (!saved) continue;
      if (mesh.material.emissive) mesh.material.emissive.copy(saved.emissive);
      mesh.material.emissiveIntensity = saved.intensity;
      delete mesh.userData.probeHighlight;
    }
    highlighted.length = 0;
    if (!robot || !urdfJoint) return;

    const joint = robot.joints ? robot.joints[urdfJoint] : null;
    if (!joint) return;

    // Only the meshes this joint moves directly, not the whole downstream chain.
    joint.traverse((o) => {
      if (!o.isMesh || !o.material || nearestJoint(o) !== joint) return;
      if (!o.material.emissive) return;
      o.userData.probeHighlight = {
        emissive: o.material.emissive.clone(),
        intensity: o.material.emissiveIntensity == null ? 1 : o.material.emissiveIntensity,
      };
      o.material.emissive.copy(HIGHLIGHT);
      o.material.emissiveIntensity = 1;
      highlighted.push(o);
    });
  }

  const reduceMotion = window.matchMedia
    && window.matchMedia('(prefers-reduced-motion: reduce)').matches;

  function pulse() {
    if (!highlighted.length) return;
    const k = reduceMotion ? 0.75 : 0.55 + 0.35 * Math.sin(performance.now() / 260);
    for (const mesh of highlighted) mesh.material.emissiveIntensity = k;
  }

  function applyPose(meas) {
    if (!robot) return;
    for (const motor of state.motors) {
      const m = meas.get(motor.name);
      if (!m) continue;
      const q = Number(m.q_meas);
      if (Number.isFinite(q)) robot.setJointValue(motor.urdf_joint, q);
    }
  }

  if (!viewerTeardown) return; // disable3D already tore this viewer down

  viewer = {
    ok: true,
    setSelected,
    applyPose,
    tick() {
      pulse();
      controls.update();
      renderer.render(scene, camera);
    },
  };

  if (state.selected) {
    const motor = state.byName.get(state.selected);
    if (motor) setSelected(motor.urdf_joint);
  }
}

// ------------------------------------------------------------------- theme

function applyTheme(theme) {
  if (theme) document.documentElement.setAttribute('data-theme', theme);
  else document.documentElement.removeAttribute('data-theme');
  document.dispatchEvent(new CustomEvent('motorprobe:theme'));
}

function initTheme() {
  let saved = null;
  try { saved = localStorage.getItem('motorprobe.theme'); } catch { /* private mode */ }
  if (saved === 'dark' || saved === 'light') applyTheme(saved);

  el('btn-theme').addEventListener('click', () => {
    const current = document.documentElement.getAttribute('data-theme')
      || (window.matchMedia && window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light');
    const next = current === 'dark' ? 'light' : 'dark';
    applyTheme(next);
    try { localStorage.setItem('motorprobe.theme', next); } catch { /* private mode */ }
  });
}

// -------------------------------------------------------------------- wiring

function wireControls() {
  el('btn-start').addEventListener('click', () => control('start'));
  el('btn-stop').addEventListener('click', () => control('stop'));

  // Escape stops, wherever focus happens to be.
  window.addEventListener('keydown', (e) => {
    if (e.key !== 'Escape') return;
    e.preventDefault();
    control('stop');
  });

  el('chk-limp').addEventListener('change', (e) => setLimp(e.target.checked));

  const slider = el('sel-slider');
  slider.addEventListener('pointerdown', () => { panel.held = true; });
  slider.addEventListener('input', (e) => onJog(e.target.value, false, 'slider'));
  slider.addEventListener('change', (e) => { panel.held = false; onJog(e.target.value, true, 'slider'); });
  slider.addEventListener('pointerup', (e) => { panel.held = false; onJog(e.target.value, true, 'slider'); });
  slider.addEventListener('keyup', (e) => onJog(e.target.value, true, 'slider'));
  slider.addEventListener('blur', () => { panel.held = false; });

  const num = el('sel-num');
  num.addEventListener('focus', () => { panel.held = true; });
  num.addEventListener('input', (e) => {
    if (e.target.value === '' || !Number.isFinite(Number(e.target.value))) return;
    onJog(e.target.value, false, 'num');
  });
  num.addEventListener('change', (e) => {
    const motor = panel.motor;
    if (!motor) return;
    const q = clamp(Number(e.target.value), motor.q_min, motor.q_max);
    if (!Number.isFinite(q)) { setInputs(Number(slider.value)); return; }
    setInputs(q); // commit: snap the typed text to the clamped value
    onJog(q, true, 'num');
  });
  num.addEventListener('blur', () => { panel.held = false; });

  el('list-filter').addEventListener('input', applyFilter);
  el('chk-diverging').addEventListener('change', applyFilter);

  // Losing the tab must stop the robot: close the socket so the deadman trips now.
  window.addEventListener('pagehide', dropSocket);
  // Timers are throttled in background tabs, so ping the moment we are back.
  document.addEventListener('visibilitychange', () => {
    if (!document.hidden) sendPing();
  });
}

// ---------------------------------------------------------------------- boot

async function loadCatalog() {
  const data = await getJSON(API.catalog);
  const motors = (data && data.motors) || [];
  if (!Array.isArray(motors) || !motors.length) throw new Error('catalog is empty');

  state.motors = motors;
  state.byName = new Map(motors.map((m) => [m.name, m]));
  state.byUrdf = new Map(motors.map((m) => [m.urdf_joint, m]));
}

async function boot() {
  initTheme();
  wireControls();

  for (;;) {
    try {
      await loadCatalog();
      clearBanner('catalog');
      break;
    } catch (err) {
      setBanner('catalog', 'bad',
        `<b>Cannot read the motor catalog from <code>${API.catalog}</code></b>` +
        `Is the probe server running? Retrying every ${CATALOG_RETRY_MS / 1000} s. ` +
        `Reason: ${escapeHtml(err.message)}`);
      await new Promise((resolve) => setTimeout(resolve, CATALOG_RETRY_MS));
    }
  }

  buildList();
  renderSelection();
  connect();
  requestAnimationFrame(frame);
  initViewer(); // async, reports its own failures, never blocks the list
}

boot();
