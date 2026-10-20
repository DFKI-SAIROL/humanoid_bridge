// Robot console page. Vanilla ES module, no build step, no runtime CDN.
//
// Owns the one WebSocket per tab: the hello carries this tab's client id, the
// 250 ms ping is the jog deadman. Renders the safety bar, Control, Processes and
// Camera; the Jog tab lives in jog.js and gets every snapshot from here.
// Rules: docs/robot_console.md.

import { jogBoot, jogOnLink, jogOnSnapshot, jogShow } from './jog.js';

const PING_MS = 250;
const WS_BACKOFF_MS = [250, 500, 1000, 2000, 4000];
const HTTP_TIMEOUT_MS = 4000;
const STALE_AFTER_MS = 1500;
const CAMERA_PERIOD_MS = 200;
const EVENTS_SHOWN = 15;
const GROUP_LABELS = { 'bring-up': 'Bring-up', policy: 'Policy', run: 'Run' };
const TABS = ['control', 'processes', 'jog', 'camera'];

const el = (id) => document.getElementById(id);

const s = {
  client: null,
  ws: null,
  pingTimer: null,
  reconnectTimer: null,
  backoff: 0,
  snap: null,
  linkUp: false,
  lastFrameAt: 0,
  tab: 'control',
  jogBooted: false,
  cards: new Map(), // process name -> card refs
  cameraOn: false,
};

// ------------------------------------------------------------------ helpers

function setText(node, value) {
  if (node && node.textContent !== value) node.textContent = value;
}

// tty output carries terminal escapes (colours, bracketed paste): drop them for display.
function plain(text) {
  return String(text == null ? '' : text)
    .replace(/\x1b\[[0-9;?]*[ -/]*[@-~]/g, '')
    .replace(/\x1b\][^\x07]*\x07/g, '');
}

function escapeHtml(text) {
  const div = document.createElement('div');
  div.textContent = String(text == null ? '' : text);
  return div.innerHTML;
}

function make(tag, className, text) {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (text != null) node.textContent = text;
  return node;
}

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

// --------------------------------------------------------------------- http

async function post(path, body) {
  const init = {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ ...(body || {}), client: s.client }),
  };
  if (typeof AbortSignal !== 'undefined' && AbortSignal.timeout) {
    init.signal = AbortSignal.timeout(HTTP_TIMEOUT_MS);
  }
  const res = await fetch(path, init);
  if (res.status === 401) throw new Error('no token: reopen the URL the console printed');
  // 409 is a refusal with a reason.
  if (!res.ok && res.status !== 409) throw new Error(`HTTP ${res.status} from ${path}`);
  return res.json();
}

// Run an action; show its refusal or failure as a banner under `id`.
async function act(id, label, path, body) {
  try {
    const reply = await post(path, body);
    if (reply && reply.ok === false) {
      setBanner(id, 'warn', `<b>${escapeHtml(label)} refused</b>${escapeHtml(reply.message || '')}`);
      return false;
    }
    clearBanner(id);
    return true;
  } catch (err) {
    setBanner(id, 'bad', `<b>${escapeHtml(label)} failed</b>${escapeHtml(err.message)}`);
    return false;
  }
}

// ----------------------------------------------------------- safety actions

async function estop(mode) {
  try {
    const reply = await post('/api/estop', { mode });
    if (reply && reply.ok === false) {
      setBanner('estop', 'bad',
        `<b>E-STOP ${mode.toUpperCase()} NOT DELIVERED &mdash; use the remote</b>` +
        `${escapeHtml(reply.message || '')}`);
    } else {
      clearBanner('estop');
    }
  } catch (err) {
    setBanner('estop', 'bad',
      `<b>E-STOP ${mode.toUpperCase()} REQUEST FAILED &mdash; use the remote (L1+R1)</b>` +
      `${escapeHtml(err.message)}`);
  }
}

function confirmMotion(title, text) {
  const dialog = el('confirm');
  setText(el('confirm-title'), title);
  setText(el('confirm-text'), text);
  return new Promise((resolve) => {
    dialog.addEventListener('close', () => resolve(dialog.returnValue === 'ok'), { once: true });
    dialog.returnValue = 'cancel';
    dialog.showModal();
  });
}

async function arm() {
  const ok = await confirmMotion('Arm at the measured pose?',
    'The bridge takes the robot at the pose it holds now, at full gains, ramping over 2 s. ' +
    'Is the robot clear and in the gantry?');
  if (ok) act('arm', 'Arm', '/api/arm', { confirm: true });
}

async function ready() {
  const ok = await confirmMotion('Move to the ready pose?',
    'Every joint, legs included, moves to ready_q over 2 s. Is the robot clear?');
  if (ok) act('ready', 'Ready pose', '/api/ready', { confirm: true });
}

// ----------------------------------------------------------- the websocket

function connect() {
  clearTimeout(s.reconnectTimer);
  s.reconnectTimer = null;
  const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
  setLink(false, 'connecting…', 'is-warn');
  try {
    s.ws = new WebSocket(`${proto}//${location.host}/api/stream`);
  } catch (err) {
    console.error('[console] websocket construction failed', err);
    scheduleReconnect();
    return;
  }
  s.ws.onopen = () => {
    s.backoff = 0;
    setLink(true, 'link live', 'is-ok');
    clearBanner('link');
  };
  s.ws.onmessage = onMessage;
  s.ws.onerror = () => { /* onclose follows */ };
  s.ws.onclose = () => {
    stopPing();
    s.ws = null;
    s.client = null;
    setLink(false, 'link down', 'is-bad');
    setBanner('link', 'bad',
      '<b>No link to the console</b>This tab shows old values and has lost control. ' +
      'A jog has stopped; processes keep running. The remote still works. Reconnecting…');
    scheduleReconnect();
  };
}

function scheduleReconnect() {
  if (s.reconnectTimer !== null) return;
  const base = WS_BACKOFF_MS[Math.min(s.backoff, WS_BACKOFF_MS.length - 1)];
  s.backoff += 1;
  s.reconnectTimer = setTimeout(() => {
    s.reconnectTimer = null;
    connect();
  }, base + Math.floor(Math.random() * 200));
}

function startPing() {
  stopPing();
  sendPing();
  s.pingTimer = setInterval(sendPing, PING_MS);
}

function stopPing() {
  if (s.pingTimer !== null) {
    clearInterval(s.pingTimer);
    s.pingTimer = null;
  }
}

function sendPing() {
  if (!s.ws || s.ws.readyState !== WebSocket.OPEN) {
    stopPing();
    return;
  }
  try {
    s.ws.send('{"type":"ping"}');
  } catch (err) {
    stopPing();
    try { s.ws.close(); } catch { /* closing */ }
  }
}

function onMessage(event) {
  let msg;
  try {
    msg = JSON.parse(event.data);
  } catch {
    return;
  }
  if (!msg || typeof msg !== 'object') return;
  if (msg.type === 'hello') {
    s.client = msg.client;
    startPing();
    return;
  }
  if (msg.type === 'state') onState(msg);
}

function setLink(up, text, cls) {
  s.linkUp = up;
  const pill = el('pill-conn');
  pill.className = `pill ${cls}`;
  setText(pill, text);
  jogOnLink(up);
}

// ------------------------------------------------------------------ render

function onState(snap) {
  s.snap = snap;
  s.lastFrameAt = performance.now();
  if (!s.jogBooted) {
    s.jogBooted = true;
    jogBoot({ client: () => s.client }, snap.robot && snap.robot.urdf, snap.robot && snap.robot.model);
    if (s.tab === 'jog') jogShow();
  }
  renderSafety(snap);
  renderControl(snap);
  renderProcesses(snap);
  renderEvents(snap);
  jogOnSnapshot(snap);
}

function statePill(state) {
  if (!state) return ['no /bridge_state', 'is-bad'];
  if (state.startsWith('LATCHED')) return [state, 'is-bad'];
  if (state === 'ARMED') return [state, 'is-warn'];
  if (state.startsWith('STOPPED')) return [state, 'is-warn'];
  return [state, ''];
}

function renderSafety(snap) {
  const robot = snap.robot || {};
  const bridge = snap.bridge || {};
  const control = snap.control || {};
  setText(el('robot-name'), robot.name || 'Robot');
  document.title = `${robot.name || 'Robot'} console`;

  const [text, cls] = statePill(bridge.state);
  const pill = el('pill-state');
  pill.className = `pill state-pill ${cls}`;
  setText(pill, text);

  // null: this robot's lowstate has no remote field.
  const remote = el('pill-remote');
  remote.hidden = bridge.remote == null;
  remote.className = `pill ${bridge.remote ? 'is-ok' : 'is-bad'}`;
  setText(remote, bridge.remote ? 'remote' : 'no remote');

  const btn = el('btn-control');
  if (control.yours) {
    btn.className = 'pill pill-btn is-ok';
    setText(btn, 'you have control');
  } else if (control.held) {
    btn.className = 'pill pill-btn is-warn';
    setText(btn, 'another tab has control — take over');
  } else {
    btn.className = 'pill pill-btn';
    setText(btn, 'take control');
  }

  const age = bridge.lowstate_age;
  setText(el('brand-sub'),
    `${control.tabs || 0} tab${control.tabs === 1 ? '' : 's'} · ` +
    (age == null ? 'no /lowstate' : `/lowstate ${(age * 1000).toFixed(0)} ms old`));

  const latched = !!(bridge.state && bridge.state.startsWith('LATCHED'));
  el('btn-reset').disabled = !(control.yours && latched);
}

function fmtAge(age) {
  if (age == null) return '—';
  return age < 1 ? `${(age * 1000).toFixed(0)} ms` : `${age.toFixed(1)} s`;
}

function renderControl(snap) {
  const bridge = snap.bridge || {};
  const control = snap.control || {};
  setText(el('f-state'), bridge.state || '—');
  setText(el('f-age'), fmtAge(bridge.lowstate_age));
  const rpy = bridge.rpy;
  setText(el('f-rpy'), rpy ? `${rpy[0].toFixed(3)} / ${rpy[1].toFixed(3)} rad` : '—');
  setText(el('f-tabs'), String(control.tabs || 0));
  el('latch-note').hidden = !!(snap.robot && snap.robot.latch_valid);

  const latched = !!(bridge.state && bridge.state.startsWith('LATCHED'));
  const streaming = (snap.processes || []).some((p) => p.commands_robot
    && ['starting', 'ready', 'prompt', 'stopping'].includes(p.state));
  const jogging = !!(snap.jog && snap.jog.active);
  const blocked = !control.yours || latched || streaming || jogging;
  el('btn-arm').disabled = blocked || !!bridge.armed;
  el('btn-ready').disabled = blocked;
  el('btn-jog-start').disabled = !control.yours || !bridge.armed || streaming || jogging;
  el('btn-jog-stop').disabled = !jogging;
}

function renderEvents(snap) {
  const host = el('events');
  const events = (snap.events || []).slice(-EVENTS_SHOWN).reverse();
  const key = events.map((e) => e.t + e.text).join('|');
  if (host.dataset.key === key) return;
  host.dataset.key = key;
  host.textContent = '';
  for (const event of events) {
    const item = make('li', `ev is-${event.level}`);
    const time = new Date(event.t * 1000).toLocaleTimeString();
    item.append(make('time', null, time), make('span', null, event.text));
    host.appendChild(item);
  }
}

// --------------------------------------------------------------- processes

const PROC_CLASS = {
  ready: 'is-ok', starting: 'is-warn', prompt: 'is-warn', stopping: 'is-warn',
  external: 'is-ok', exited: 'is-bad', stopped: '',
};

function renderProcesses(snap) {
  const procs = snap.processes || [];
  el('proc-empty').hidden = procs.length > 0;
  const host = el('proc-groups');
  for (const proc of procs) {
    let card = s.cards.get(proc.name);
    if (!card) {
      card = buildCard(proc);
      s.cards.set(proc.name, card);
      groupSection(host, proc.group).appendChild(card.root);
    }
    updateCard(card, proc, snap);
  }
}

function groupSection(host, group) {
  let section = host.querySelector(`[data-group="${group}"]`);
  if (!section) {
    section = make('section', 'procgroup');
    section.dataset.group = group;
    section.appendChild(make('h2', null, GROUP_LABELS[group] || group));
    section.appendChild(make('div', 'grid'));
    host.appendChild(section);
  }
  return section.querySelector('.grid');
}

function buildCard(proc) {
  const root = make('section', 'card proc');
  const head = make('div', 'prochead');
  const title = make('h3', null, proc.label || proc.name);
  const pill = make('span', 'pill');
  head.append(title, pill);
  if (proc.commands_robot) head.appendChild(make('span', 'tag tag-robot', 'commands the robot'));
  root.appendChild(head);

  const inputs = new Map();
  const names = Object.keys(proc.params || {});
  if (names.length) {
    const form = make('div', 'params');
    for (const name of names) {
      const spec = proc.params[name];
      const label = make('label', null, name);
      let input;
      if (spec.choices) {
        input = make('select');
        for (const choice of spec.choices) {
          const option = make('option', null, choice);
          option.value = choice;
          input.appendChild(option);
        }
        if (spec.default) input.value = spec.default;
      } else {
        input = make('input');
        input.type = 'text';
        input.value = spec.default || '';
        if (spec.pattern) input.title = `must match ${spec.pattern}`;
      }
      label.appendChild(input);
      form.appendChild(label);
      inputs.set(name, input);
    }
    root.appendChild(form);
  }

  const buttons = make('div', 'btnrow');
  const start = make('button', 'btn btn-start', 'Start');
  start.type = 'button';
  const stop = make('button', 'btn', 'Stop');
  stop.type = 'button';
  buttons.append(start, stop);
  root.appendChild(buttons);

  const requires = make('ul', 'requires');
  root.appendChild(requires);
  const prompt = make('div', 'prompt');
  prompt.hidden = true;
  root.appendChild(prompt);
  const status = make('p', 'hint procstatus');
  root.appendChild(status);

  const details = make('details', 'logbox');
  details.appendChild(make('summary', null, 'log'));
  const pre = make('pre');
  details.appendChild(pre);
  root.appendChild(details);

  const card = { root, pill, start, stop, requires, prompt, status, details, pre, inputs,
    seq: 0, fetching: false, promptKey: '' };

  start.addEventListener('click', () => {
    const params = {};
    for (const [name, input] of inputs) params[name] = input.value;
    act(`proc-${proc.name}`, `Start ${proc.label || proc.name}`, '/api/process/start',
      { name: proc.name, params });
  });
  stop.addEventListener('click', () => {
    act(`proc-${proc.name}`, `Stop ${proc.label || proc.name}`, '/api/process/stop', { name: proc.name });
  });
  details.addEventListener('toggle', () => { if (details.open) fetchLog(card, proc.name); });
  return card;
}

function updateCard(card, proc, snap) {
  card.pill.className = `pill ${PROC_CLASS[proc.state] || ''}`;
  setText(card.pill, proc.state + (proc.state === 'exited' && proc.exit_code != null ? ` (${proc.exit_code})` : ''));

  const yours = !!(snap.control && snap.control.yours);
  const running = ['starting', 'ready', 'prompt', 'stopping'].includes(proc.state);
  const missing = (proc.requires || []).filter((r) => !r.ok);
  card.start.disabled = !yours || running || proc.state === 'external' || missing.length > 0;
  card.stop.disabled = !running || proc.state === 'stopping';
  for (const input of card.inputs.values()) input.disabled = running;

  const reqKey = JSON.stringify(proc.requires || []);
  if (card.requires.dataset.key !== reqKey) {
    card.requires.dataset.key = reqKey;
    card.requires.textContent = '';
    for (const r of proc.requires || []) {
      card.requires.appendChild(make('li', r.ok ? 'is-ok' : 'is-bad', `${r.ok ? '✓' : '✗'} ${r.cond}`));
    }
  }

  const prompt = proc.prompt;
  const promptKey = prompt ? prompt.text + prompt.answers.join() : '';
  if (promptKey !== card.promptKey) {
    card.promptKey = promptKey;
    card.prompt.textContent = '';
    card.prompt.hidden = !prompt;
    if (prompt) {
      card.prompt.appendChild(make('p', null, plain(prompt.text)));
      const row = make('div', 'btnrow');
      for (const answer of prompt.answers) {
        const b = make('button', 'btn', answer);
        b.type = 'button';
        b.addEventListener('click', () => act(`proc-${proc.name}`, `Answer ${answer}`,
          '/api/process/answer', { name: proc.name, answer }));
        row.appendChild(b);
      }
      card.prompt.appendChild(row);
    }
  }
  for (const b of card.prompt.querySelectorAll('button')) b.disabled = !yours;

  let status = '';
  if (proc.state === 'external') status = 'running outside the console';
  else if (missing.length && !running) status = `waiting for ${missing.map((r) => r.cond).join(', ')}`;
  if (proc.partial) status = (status ? status + ' · ' : '') + plain(proc.partial);
  setText(card.status, status);

  if (card.details.open && proc.log_seq > card.seq) fetchLog(card, proc.name);
}

async function fetchLog(card, name) {
  if (card.fetching) return;
  card.fetching = true;
  try {
    const res = await fetch(`/api/process/log?name=${encodeURIComponent(name)}&since=${card.seq}`);
    if (!res.ok) return;
    const data = await res.json();
    if (data.lines && data.lines.length) {
      const atBottom = card.pre.scrollTop + card.pre.clientHeight >= card.pre.scrollHeight - 4;
      card.pre.textContent = (card.pre.textContent + data.lines.map(plain).join('\n') + '\n')
        .split('\n').slice(-400).join('\n');
      if (atBottom) card.pre.scrollTop = card.pre.scrollHeight;
    }
    card.seq = data.seq || card.seq;
  } catch { /* the next snapshot retries */ } finally {
    card.fetching = false;
  }
}

// ------------------------------------------------------------------ camera

function cameraLoop() {
  if (s.tab !== 'camera' || !s.snap || !s.snap.robot || !s.snap.robot.camera) {
    s.cameraOn = false;
    if (s.snap && s.snap.robot && !s.snap.robot.camera) setText(el('camera-msg'), 'No camera configured.');
    return;
  }
  s.cameraOn = true;
  const img = el('camera-img');
  img.onload = () => {
    setText(el('camera-msg'), `frame at ${new Date().toLocaleTimeString()}`);
    setTimeout(cameraLoop, CAMERA_PERIOD_MS);
  };
  img.onerror = () => {
    setText(el('camera-msg'), 'No frame from the camera: is the camera server running?');
    setTimeout(cameraLoop, 1000);
  };
  img.src = `/api/camera/frame.jpg?t=${Date.now()}`;
}

// -------------------------------------------------------------------- tabs

function showTab(tab) {
  if (!TABS.includes(tab)) tab = 'control';
  s.tab = tab;
  for (const name of TABS) el(`tab-${name}`).hidden = name !== tab;
  for (const b of document.querySelectorAll('.tab')) b.classList.toggle('is-active', b.dataset.tab === tab);
  try { localStorage.setItem('robot_console.tab', tab); } catch { /* private mode */ }
  if (tab === 'jog' && s.jogBooted) jogShow();
  if (tab === 'camera' && !s.cameraOn) cameraLoop();
}

// ------------------------------------------------------------------- theme

function applyTheme(theme) {
  if (theme) document.documentElement.setAttribute('data-theme', theme);
  else document.documentElement.removeAttribute('data-theme');
  document.dispatchEvent(new CustomEvent('motorprobe:theme'));
}

function initTheme() {
  let saved = null;
  try { saved = localStorage.getItem('robot_console.theme'); } catch { /* private mode */ }
  if (saved === 'dark' || saved === 'light') applyTheme(saved);
  el('btn-theme').addEventListener('click', () => {
    const current = document.documentElement.getAttribute('data-theme')
      || (window.matchMedia && window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light');
    const next = current === 'dark' ? 'light' : 'dark';
    applyTheme(next);
    try { localStorage.setItem('robot_console.theme', next); } catch { /* private mode */ }
  });
}

// ------------------------------------------------------------------- wiring

function wire() {
  el('btn-estop-damp').addEventListener('click', () => estop('damp'));
  el('btn-estop-hold').addEventListener('click', () => estop('hold'));
  el('btn-reset').addEventListener('click', () => act('reset', 'Reset', '/api/reset'));
  el('btn-control').addEventListener('click', () => act('control', 'Take control', '/api/control/take'));
  el('btn-arm').addEventListener('click', arm);
  el('btn-ready').addEventListener('click', ready);
  el('btn-stop').addEventListener('click', () => act('stop', 'Stop', '/api/stop'));

  // Escape is E-STOP DAMP, wherever focus is, dialogs included.
  window.addEventListener('keydown', (e) => {
    if (e.key !== 'Escape') return;
    e.preventDefault();
    estop('damp');
  });

  for (const b of document.querySelectorAll('.tab')) {
    b.addEventListener('click', () => showTab(b.dataset.tab));
  }

  // Closing the tab must stop a jog now, not after the server notices.
  window.addEventListener('pagehide', () => { if (s.ws) s.ws.close(); });
  // Background tabs throttle timers: ping as soon as the tab is back.
  document.addEventListener('visibilitychange', () => { if (!document.hidden) sendPing(); });

  setInterval(() => {
    if (s.linkUp && s.lastFrameAt && performance.now() - s.lastFrameAt > STALE_AFTER_MS) {
      const pill = el('pill-state');
      pill.className = 'pill state-pill is-bad';
      setText(pill, 'stale');
    }
  }, 500);
}

function boot() {
  initTheme();
  wire();
  let saved = 'control';
  try { saved = localStorage.getItem('robot_console.tab') || 'control'; } catch { /* private mode */ }
  showTab(saved);
  connect();
}

boot();
