// Application wiring: mission inputs -> backend planner job -> 2D/3D/profile views.
import { api } from './api.js';
import { MapView } from './map-view.js';
import { ChartDock } from './charts.js';
import { fmt, el, icon } from './format.js';

const $ = (sel, root = document) => root.querySelector(sel);
const $$ = (sel, root = document) => [...root.querySelectorAll(sel)];

const state = {
  meta: null,
  regions: [],
  presets: [],
  uiMode: 'mission',
  view: '2d',
  pick: 'start',
  points: {
    start: { lon: null, lat: null, info: null, error: null, altMode: 'agl', altValue: 150 },
    goal: { lon: null, lat: null, info: null, error: null, altMode: 'agl', altValue: 150 },
  },
  job: null,
  result: null,
  resultStale: false,
  hoverIndex: -1,
};

let mapView, charts;

// ======================================================================= boot
async function boot() {
  const dot = $('#server-dot');
  try {
    const [meta, regions, presets] = await Promise.all([api.meta(), api.regions(), api.presets()]);
    state.meta = meta; state.regions = regions.regions; state.presets = presets.presets;
    dot.classList.add('ok'); dot.title = 'Backend connected';
  } catch (e) {
    dot.classList.add('bad'); dot.title = `Backend unreachable: ${e.message}`;
    $('#run-status').innerHTML = `<span class="err">Backend unreachable — start it with <code>python -m mission_ui.server</code></span>`;
    return;
  }
  applyDefaults(state.meta.defaults);

  mapView = new MapView($('#map'), { terrainMeta: { ...state.meta.terrain, tilePort: state.meta.tile_port } });
  charts = new ChartDock($('#chart'), $('#dock-readout'));
  window.__app = { state, mapView, charts };  // debugging/validation hook
  await mapView.ready;
  mapView.setRegions(state.regions);
  mapView.setPickMode(state.pick);

  for (const p of state.presets) $('#preset-select').append(el('option', { value: p.id }, p.label));
  wireMission();
  wireMap();
  wireDock();
  wireModes();
  renderAll();
  setInterval(heartbeat, 10000);
}

function applyDefaults(d) {
  $('#in-min-agl').value = d.min_agl_m; $('#in-target-agl').value = d.target_agl_m; $('#in-buffer').value = d.lateral_buffer_m;
  $('#in-tol-xy').value = d.goal_tolerance_xy_m; $('#in-tol-alt').value = d.goal_tolerance_alt_m;
  $('#in-max-exp').value = d.max_expansions; $('#in-max-time').value = d.max_search_time_s;
  $('#in-feedback').value = d.max_feedback_passes; $('#in-corridor').value = d.max_corridor_deviation_m;
  $('#in-smoothing').checked = d.smoothing;
  for (const k of ['start', 'goal']) { state.points[k].altValue = d[`${k}_alt`].value; state.points[k].altMode = d[`${k}_alt`].mode; }
}

async function heartbeat() {
  const dot = $('#server-dot');
  try { await api.health(); dot.className = 'server-dot ok'; } catch { dot.className = 'server-dot bad'; }
}

// ===================================================================== inputs
const num = (id) => parseFloat($(id).value);
const safety = () => ({ min: num('#in-min-agl'), target: num('#in-target-agl'), buffer: num('#in-buffer') });

function card(which) { return $(`.point-card[data-point="${which}"]`); }

function wireMission() {
  for (const which of ['start', 'goal']) {
    const c = card(which);
    $$('input[data-coord]', c).forEach((inp) => inp.addEventListener('change', () => {
      const lat = parseFloat($('input[data-coord="lat"]', c).value);
      const lon = parseFloat($('input[data-coord="lon"]', c).value);
      if (isFinite(lat) && isFinite(lon)) setPoint(which, lon, lat, { fly: true });
    }));
    $('input[data-alt="value"]', c).addEventListener('input', (e) => {
      state.points[which].altValue = parseFloat(e.target.value); onMissionEdited(); renderAll();
    });
    $$('[data-altmode]', c).forEach((b) => b.addEventListener('click', () => setAltMode(which, b.dataset.altmode)));
    $('.pick-btn', c).addEventListener('click', () => setPick(state.pick === which ? null : which));
  }
  for (const id of ['#in-min-agl', '#in-target-agl', '#in-tol-xy', '#in-tol-alt', '#in-max-exp', '#in-max-time', '#in-feedback', '#in-corridor', '#in-smoothing']) {
    $(id).addEventListener('input', () => { onMissionEdited(); renderAll(); });
  }
  $('#in-buffer').addEventListener('change', () => { refreshInfo('start'); refreshInfo('goal'); onMissionEdited(); });

  $('#btn-swap').addEventListener('click', () => {
    const s = state.points.start, g = state.points.goal;
    const a = { lon: s.lon, lat: s.lat }, b = { lon: g.lon, lat: g.lat };
    setPoint('start', b.lon, b.lat); setPoint('goal', a.lon, a.lat);
  });
  $('#btn-reset').addEventListener('click', resetMission);
  $('#preset-select').addEventListener('change', (e) => applyPreset(e.target.value));
  $('#btn-run').addEventListener('click', runPlanner);
  document.addEventListener('keydown', (e) => { if (e.key === 'Escape' && state.pick) setPick(null); });
}

function setPick(which) {
  state.pick = which;
  mapView.setPickMode(which);
  renderPickUI();
}

function nextPick() {
  if (state.points.start.lon == null) return 'start';
  if (state.points.goal.lon == null) return 'goal';
  return null;
}

function setPoint(which, lon, lat, { fly = false } = {}) {
  const p = state.points[which];
  if (lon == null || lat == null) { p.lon = p.lat = null; p.info = null; p.error = null; mapView.setMissionPoint(which, null); }
  else { p.lon = lon; p.lat = lat; mapView.setMissionPoint(which, { lon, lat }); refreshInfo(which); }
  if (fly && lon != null) mapView.fitTo([state.points.start, state.points.goal].filter((q) => q.lon != null));
  $('#preset-select').value = '';
  onMissionEdited();
  renderAll();
}

const pending = {};
async function refreshInfo(which) {
  const p = state.points[which];
  if (p.lon == null) return;
  pending[which]?.abort();
  const ctl = new AbortController();
  pending[which] = ctl;
  p.info = null; p.error = null; p.loading = true;
  renderPoint(which);
  try {
    const info = await api.point(p.lon, p.lat, safety().buffer, ctl.signal);
    p.info = info;
    p.error = info.region_id ? (info.planner_ground_m == null ? 'No valid planner terrain here (ROI edge / NoData)' : null)
      : 'Outside every planner region ROI — the planner cannot run here';
  } catch (e) {
    if (e.name === 'AbortError') return;
    p.error = e.message;
  }
  p.loading = false;
  renderAll();
}

function setAltMode(which, mode) {
  const p = state.points[which];
  if (p.altMode === mode) return;
  const g = p.info?.planner_ground_m;
  if (g != null && isFinite(p.altValue)) p.altValue = Math.round(mode === 'msl' ? g + p.altValue : p.altValue - g);
  p.altMode = mode;
  $('input[data-alt="value"]', card(which)).value = p.altValue;
  onMissionEdited();
  renderPoint(which);
}

function resolvedAlt(which) {
  const p = state.points[which]; const g = p.info?.planner_ground_m;
  if (g == null || !isFinite(p.altValue)) return null;
  return p.altMode === 'agl' ? { msl: g + p.altValue, agl: p.altValue, ground: g } : { msl: p.altValue, agl: p.altValue - g, ground: g };
}

function applyPreset(id) {
  const pr = state.presets.find((x) => x.id === id);
  if (!pr) return;
  for (const which of ['start', 'goal']) {
    const p = state.points[which];
    p.altMode = 'agl'; p.altValue = pr.alt_agl_m;
    $('input[data-alt="value"]', card(which)).value = pr.alt_agl_m;
    p.lon = pr[which].lon; p.lat = pr[which].lat;
    mapView.setMissionPoint(which, pr[which]);
    refreshInfo(which);
  }
  setPick(null);
  mapView.fitTo([pr.start, pr.goal]);
  $('#preset-select').value = id;
  onMissionEdited();
  renderAll();
}

function resetMission() {
  setPoint('start', null, null); setPoint('goal', null, null);
  state.result = null; state.resultStale = false;
  mapView.setResult(null); charts.setResult(null);
  setPick('start');
  renderAll();
}

function onMissionEdited() {
  if (state.result) state.resultStale = true;
  const s = state.points.start, g = state.points.goal;
  if (!state.result) mapView.setDirectLine(s.lon != null ? s : null, g.lon != null ? g : null);
}

// ====================================================================== map
function wireMap() {
  mapView.addEventListener('pick', (e) => {
    const { which, lon, lat, drag } = e.detail;
    setPoint(which, lon, lat);
    if (!drag) setPick(nextPick());
  });
  let hoverCtl = null, hoverTimer = 0;
  mapView.addEventListener('cursor', (e) => {
    const ro = $('#cursor-readout');
    if (!e.detail) { ro.textContent = ''; return; }
    const { lon, lat, zoom, terrain } = e.detail;
    const base = `${lat.toFixed(5)}, ${lon.toFixed(5)} · z${zoom.toFixed(1)}`;
    if (terrain != null) { ro.textContent = `${base} · terrain ${terrain.toFixed(0)} m${mapView.exaggeration !== 1 ? ' (exaggerated)' : ''}`; return; }
    ro.textContent = base;
    clearTimeout(hoverTimer);
    hoverTimer = setTimeout(async () => {
      hoverCtl?.abort(); hoverCtl = new AbortController();
      try {
        const r = await api.hoverPoint(lon, lat, hoverCtl.signal);
        if (r.display_dem_m != null) ro.textContent = `${base} · DEM ${r.display_dem_m.toFixed(0)} m${r.region_id ? ` · ${r.region_id} ROI` : ''}`;
      } catch { /* aborted */ }
    }, 180);
  });
  mapView.addEventListener('routehover', (e) => setHover(e.detail.index, 'map'));

  $$('[data-view]').forEach((b) => b.addEventListener('click', () => {
    state.view = b.dataset.view;
    $$('[data-view]').forEach((x) => x.setAttribute('aria-checked', String(x === b)));
    document.body.classList.toggle('view3d', state.view === '3d');
    $('#exag-wrap').hidden = state.view !== '3d';
    mapView.setView(state.view);
    setHover(-1);
  }));
  $('#exag-select').addEventListener('change', (e) => mapView.setExaggeration(parseFloat(e.target.value)));
  $$('[data-layer]').forEach((c) => c.addEventListener('change', () => mapView.setLayerVisible(c.dataset.layer, c.checked)));
  $$('[data-stage]').forEach((c) => c.addEventListener('change', () => mapView.setStageVisible(c.dataset.stage, c.checked)));
}

function setHover(index, source) {
  state.hoverIndex = index;
  mapView.showHover(index);
  if (source !== 'chart') charts.showIndex(index);
}

// ===================================================================== dock
function wireDock() {
  $$('[role="tab"]').forEach((t) => t.addEventListener('click', () => {
    $$('[role="tab"]').forEach((x) => x.setAttribute('aria-selected', String(x === t)));
    charts.setTab(t.dataset.tab);
  }));
  charts.addEventListener('hover', (e) => setHover(e.detail.index, 'chart'));
  $('#btn-dock').addEventListener('click', () => {
    const d = $('#dock');
    d.classList.toggle('collapsed');
    $('#btn-dock').setAttribute('aria-expanded', String(!d.classList.contains('collapsed')));
    setTimeout(() => mapView.resize(), 220);
  });
}

function wireModes() {
  $$('[data-uimode]').forEach((b) => b.addEventListener('click', () => {
    state.uiMode = b.dataset.uimode;
    $$('[data-uimode]').forEach((x) => x.setAttribute('aria-checked', String(x === b)));
    document.body.classList.toggle('eng', state.uiMode === 'engineering');
    if (state.uiMode !== 'engineering') {
      // Leaving engineering: hide diagnostic stages and fall back to a mission tab.
      for (const s of ['search', 'profiled']) { $(`[data-stage="${s}"]`).checked = false; mapView.setStageVisible(s, false); }
      $('[data-stage="final"]').checked = true; mapView.setStageVisible('final', true);
      const sel = $('[role="tab"][aria-selected="true"]');
      if (sel?.classList.contains('eng-only')) $('[data-tab="profile"]').click();
    }
    renderResults();
  }));
}

// ================================================================== planner
function buildRequest() {
  const s = state.points.start, g = state.points.goal;
  return {
    start: { lon: s.lon, lat: s.lat, alt: { mode: s.altMode, value: s.altValue }, heading_deg: null },
    goal: { lon: g.lon, lat: g.lat, alt: { mode: g.altMode, value: g.altValue } },
    safety: { min_agl_m: num('#in-min-agl'), target_agl_m: num('#in-target-agl'), lateral_buffer_m: num('#in-buffer') },
    tolerance: { xy_m: num('#in-tol-xy'), alt_m: num('#in-tol-alt') },
    budget: { max_expansions: num('#in-max-exp'), max_search_time_s: num('#in-max-time'), max_feedback_passes: num('#in-feedback') },
    smoothing: { enabled: $('#in-smoothing').checked, max_corridor_deviation_m: num('#in-corridor') },
  };
}

function validation() {
  const issues = [];
  const { min, target } = safety();
  for (const which of ['start', 'goal']) {
    const p = state.points[which];
    if (p.lon == null) { issues.push(`Place the ${which} point`); continue; }
    if (p.loading) { issues.push(`Resolving ${which} terrain…`); continue; }
    if (p.error) { issues.push(`${which}: ${p.error}`); continue; }
    const a = resolvedAlt(which);
    if (a && a.agl < min) issues.push(`${which} is ${a.agl.toFixed(0)} m AGL (< min ${min} m)`);
  }
  const s = state.points.start.info, g = state.points.goal.info;
  if (s?.region_id && g?.region_id && s.region_id !== g.region_id) issues.push('Start and goal must be in the same planner region');
  if (!(target >= min)) issues.push('Target AGL must be ≥ min AGL');
  return issues;
}

async function runPlanner() {
  if (validation().length || state.job?.state === 'running' || state.job?.state === 'queued') return;
  const body = buildRequest();
  try {
    state.job = await api.plan(body);
    state.jobError = null;
  } catch (e) {
    state.job = null; state.jobError = e.message;
    renderRun(); return;
  }
  renderRun();
  const t0 = performance.now();
  let misses = 0;
  while (true) {
    await new Promise((r) => setTimeout(r, 700));
    let j;
    try {
      j = await api.job(state.job.id, 10000);
      misses = 0;
    } catch (e) {
      // A slow response is not a failure: keep polling and say so instead of looking frozen.
      misses += 1;
      $('#run-status').textContent = `Waiting for server… (${misses})`;
      if (misses >= 30) { state.jobError = `Lost contact with the server: ${e.message}`; break; }
      continue;
    }
    state.job = j;
    renderRun();
    if (j.state === 'done' || j.state === 'failed') break;
  }
  if (state.job?.state === 'done') {
    state.result = state.job.result; state.resultStale = false;
    state.result.clientReceiveMs = performance.now() - t0;
    mapView.setResult(state.result);
    charts.setResult(state.result);
    $('#chart-empty').hidden = !!state.result.stages.final;
    if (state.result.stages.final) mapView.fitRoute();
    for (const s of ['search', 'profiled']) mapView.setStageVisible(s, $(`[data-stage="${s}"]`).checked);
    // No route: show the diagnostic best-partial chain so the failure is visible on the map.
    if (!state.result.stages.final && state.result.stages.search_best_partial) mapView.setStageVisible('search', true);
  } else if (state.job?.state === 'failed') {
    state.jobError = state.job.error;
  }
  renderAll();
}

// =================================================================== render
function renderAll() {
  renderPoint('start'); renderPoint('goal');
  renderPickUI(); renderRun(); renderResults();
  const regionId = state.points.start.info?.region_id || state.points.goal.info?.region_id;
  const reg = state.regions.find((r) => r.region_id === regionId);
  $('#region-chip').textContent = reg ? `${reg.name} · ${reg.crs}` : 'No region';
  const s = state.points.start, g = state.points.goal;
  const ml = $('#mission-line');
  if (s.info?.x_m != null && g.info?.x_m != null) {
    const d = Math.hypot(g.info.x_m - s.info.x_m, g.info.y_m - s.info.y_m);
    ml.textContent = `Direct distance ${fmt.km(d)} (input geometry, not a route)`;
  } else ml.textContent = '';
}

function renderPickUI() {
  for (const which of ['start', 'goal']) {
    card(which).classList.toggle('picking', state.pick === which);
    $('.pick-btn', card(which)).setAttribute('aria-pressed', String(state.pick === which));
  }
  $('#pick-hint').innerHTML = state.pick
    ? `Click the map to place <b>${state.pick.toUpperCase()}</b>. <kbd>Esc</kbd> cancels.`
    : 'Drag markers or use <b>Pick</b> to move a point. Coordinates can be typed.';
}

function renderPoint(which) {
  const p = state.points[which], c = card(which);
  const lat = $('input[data-coord="lat"]', c), lon = $('input[data-coord="lon"]', c);
  if (document.activeElement !== lat) lat.value = p.lat != null ? p.lat.toFixed(5) : '';
  if (document.activeElement !== lon) lon.value = p.lon != null ? p.lon.toFixed(5) : '';
  $$('[data-altmode]', c).forEach((b) => b.setAttribute('aria-checked', String(b.dataset.altmode === p.altMode)));
  $('[data-alt-unit]', c).textContent = p.altMode === 'agl' ? 'm AGL' : 'm MSL';
  const ro = $('[data-readout]', c);
  const altInput = $('input[data-alt="value"]', c);
  altInput.removeAttribute('aria-invalid');
  if (p.lon == null) { ro.textContent = ''; return; }
  if (p.loading) { ro.textContent = 'Resolving planner terrain…'; return; }
  if (p.error) { ro.innerHTML = ''; ro.append(el('span', { class: 'err' }, p.error)); return; }
  const i = p.info, a = resolvedAlt(which), min = safety().min;
  const lines = [
    el('div', {}, 'UTM ', el('span', { class: 'val' }, `${i.x_m.toFixed(0)} E  ${i.y_m.toFixed(0)} N`), ` · ${i.region_id}`),
    el('div', {}, 'Planner terrain ', el('span', { class: 'val' }, fmt.m(i.planner_ground_m, 1)), ` (buffer ${i.lateral_buffer_m} m)`),
  ];
  if (a) {
    const other = p.altMode === 'agl' ? `= ${a.msl.toFixed(1)} m MSL` : `= ${a.agl.toFixed(1)} m AGL`;
    const row = el('div', {}, 'Altitude ', el('span', { class: 'val' }, other));
    if (a.agl < min) { row.append(el('span', { class: 'err' }, `  below min AGL ${min} m`)); altInput.setAttribute('aria-invalid', 'true'); }
    lines.push(row);
  }
  ro.replaceChildren(...lines);
}

function renderRun() {
  const btn = $('#btn-run'), st = $('#run-status');
  const running = state.job && (state.job.state === 'queued' || state.job.state === 'running');
  const issues = validation();
  btn.disabled = running || issues.length > 0;
  btn.classList.toggle('busy', !!running);
  const stageNames = { queued: 'Queued', loading_terrain: 'Loading planner terrain', search_and_profile: 'A* search + terrain-following profile',
    smoothing: 'Corridor-safe smoothing', packaging: 'Packaging trajectory', done: 'Done' };
  $('#run-label').textContent = running ? 'Planning…' : (state.result && !state.resultStale ? 'Re-run planner' : 'Run planner');
  const chip = $('#job-chip');
  if (running) {
    st.textContent = `${stageNames[state.job.stage] || state.job.stage} · ${state.job.elapsed_s.toFixed(1)} s`;
    chip.hidden = false; chip.textContent = `${state.job.id} · ${state.job.elapsed_s.toFixed(0)} s`;
  } else {
    chip.hidden = true;
    if (state.jobError) { st.innerHTML = ''; st.append(el('span', { class: 'err' }, state.jobError)); }
    else if (issues.length) st.textContent = issues[0];
    else if (state.job?.state === 'done') st.textContent = `Completed in ${state.job.elapsed_s.toFixed(1)} s (${state.job.id})`;
    else st.textContent = 'Ready';
  }
}

function kv(label, value, small) {
  return el('div', {}, el('dt', {}, label), el('dd', {}, value, small ? el('small', {}, ` ${small}`) : null));
}

function renderResults() {
  const box = $('#results');
  const r = state.result;
  if (!r) { box.hidden = true; return; }
  box.hidden = false;
  const st = r.status;
  const cls = st.verdict === 'SAFE' ? 'safe' : st.verdict === 'SAFE_WITH_ADVISORIES' ? 'advisory' : 'bad';
  const ico = cls === 'safe' ? 'check' : cls === 'advisory' ? 'alert' : 'x';
  const title = { SAFE: 'Route found · terrain safe', SAFE_WITH_ADVISORIES: 'Route found · terrain safe', UNSAFE: 'Unsafe trajectory', NO_ROUTE: 'No route' }[st.verdict];
  const v = $('#verdict');
  v.className = `verdict ${cls}${state.resultStale ? ' stale' : ''}`;
  v.replaceChildren(icon(ico), el('div', {},
    el('div', { class: 'v-title' }, title, state.resultStale ? el('span', { class: 'stale-tag' }, 'inputs changed') : null),
    el('div', { class: 'v-sub' }, st.headline)));

  const f = r.flight, kvBox = $('#kv-main');
  if (f) {
    kvBox.hidden = false;
    kvBox.replaceChildren(
      kv('Route length', fmt.km(f.distance_m), `×${f.route_to_straight_ratio.toFixed(2)}`),
      kv('Flight time', fmt.s(f.flight_time_s), '@ 40 m/s'),
      kv('Min terrain clearance', fmt.m(f.min_agl_m, 1), 'AGL'),
      kv('Mean clearance', fmt.m(f.mean_agl_m, 0), 'AGL'),
      kv('Altitude range', `${f.min_msl_m.toFixed(0)}–${f.max_msl_m.toFixed(0)}`, 'm MSL'),
      kv('Max climb / descent', `${fmt.signed(f.max_climb_rate_mps)} / ${fmt.signed(f.max_descent_rate_mps)}`, 'm/s'),
      kv('Max bank', fmt.deg(f.max_abs_bank_deg), `limit ${r.limits.max_bank_deg}°`),
      kv('Planning time', fmt.s(r.timing.total_s), `${fmt.int(r.search.expanded_nodes)} exp.`),
    );
  } else {
    kvBox.hidden = false;
    kvBox.replaceChildren(
      kv('Search result', r.search.termination_reason), kv('Expansions', fmt.int(r.search.expanded_nodes)),
      kv('Search time', fmt.s(r.search.runtime_s)), kv('Closest to goal', fmt.m(r.search.closest_xy_distance_to_goal_m, 0)));
  }

  const checks = $('#checks');
  checks.replaceChildren(...st.checks
    .filter((c) => state.uiMode === 'engineering' || !c.advisory || !c.ok)
    .map((c) => {
      const k = c.ok ? 'ok' : c.advisory ? 'warn' : 'fail';
      return el('li', { class: k }, icon(c.ok ? 'check' : c.advisory ? 'alert' : 'x'), el('span', {}, c.label), el('span', { class: 'c-val' }, c.value),
        c.note && !c.ok ? el('span', { class: 'c-note' }, c.note) : null);
    }));

  renderEngineering(r);
}

function table(caption, rows) {
  return el('table', { class: 'eng-table' }, el('caption', {}, caption),
    el('tbody', {}, rows.filter(Boolean).map(([k, v]) => el('tr', {}, el('th', {}, k), el('td', {}, v)))));
}

function renderEngineering(r) {
  const box = $('#eng-details');
  if (state.uiMode !== 'engineering') { box.replaceChildren(); return; }
  const s = r.search, p = r.profile, sm = r.smoothing, t = r.timing, m = r.mission;
  const rejects = Object.entries(s.rejected_reason_counts || {}).sort((a, b) => b[1] - a[1]).slice(0, 4);
  const nodes = [
    table('Mission (resolved by backend)', [
      ['Region / CRS', `${m.region_id} · ${m.crs}`],
      ['Start MSL / AGL', `${m.start.z_msl_m.toFixed(1)} / ${m.start.agl_m.toFixed(1)} m`],
      ['Start heading', `${m.start.heading_deg.toFixed(1)}° (${m.start.heading_source.replace(/_/g, ' ')})`],
      ['Goal MSL / AGL', `${m.goal.z_msl_m.toFixed(1)} / ${m.goal.agl_m.toFixed(1)} m`],
      ['Min / target AGL', `${m.min_agl_m} / ${m.target_agl_m} m`],
      ['Lateral buffer', `${m.lateral_buffer_m} m`],
      ['Goal tolerance', `±${m.goal_tolerance.xy_m} m XY, ±${m.goal_tolerance.alt_m} m alt`],
    ]),
    table('A* pose search', [
      ['Termination', s.termination_reason],
      ['Expanded / generated', `${fmt.int(s.expanded_nodes)} / ${fmt.int(s.generated_neighbors)}`],
      ['Rejected', fmt.int(s.rejected_neighbors)],
      ...rejects.map(([k, v]) => [`  ${k}`, fmt.int(v)]),
      ['Max open size', fmt.int(s.max_open_size)],
      ['Unique keys', fmt.int(s.unique_search_keys)],
      ['Search runtime', fmt.s(s.runtime_s)],
      ['Primitives', fmt.int(s.primitive_count)],
      ['Goal error XY / Z', `${fmt.num(s.goal_xy_error_m)} / ${fmt.num(s.goal_z_error_m)} m`],
      ['Max search altitude', fmt.m(s.maximum_altitude_msl_m)],
    ]),
  ];
  if (p) nodes.push(table('Terrain-following profile', [
    ['Status', p.status], ['Min AGL (validated)', fmt.m(p.min_agl_m, 2)], ['Refinement passes', p.refinement_passes],
    ['Runtime', fmt.s(p.runtime_s)], p.failure_reason_detail ? ['Failure', p.failure_reason_detail] : null,
  ]));
  if (sm) nodes.push(table('Local B-spline smoothing', [
    ['Junctions smoothed', `${sm.junctions_smoothed} / ${sm.junctions_total}`],
    ['Roll rate raw → smoothed', `${sm.raw_max_bank_rate_deg_s.toFixed(1)} → ${sm.smoothed_max_bank_rate_deg_s.toFixed(1)} °/s`],
    ['Max corridor deviation', fmt.m(sm.max_corridor_deviation_m, 2)],
    ['Corridor safe', sm.corridor_safe ? 'yes' : 'NO'],
  ]));
  if (r.flight) nodes.push(table('Flight telemetry (final)', [
    ['Mean MSL / ground', `${r.flight.mean_msl_m.toFixed(0)} / ${r.flight.mean_ground_m.toFixed(0)} m`],
    ['Max roll rate', `${r.flight.max_abs_roll_rate_deg_s.toFixed(1)} °/s`],
    ['Max load factor', `${r.flight.max_load_factor_g.toFixed(2)} g`],
    ['Samples (sent / source)', `${r.stages.final.n} / ${r.stages.final.n_source_samples}`],
  ]));
  nodes.push(table('Timing', [
    ['Region load', fmt.s(t.region_load_s)], ['Buffered field', fmt.s(t.buffered_field_s)],
    ['Search + profile', fmt.s(t.search_and_profile_s)], t.smoothing_s != null ? ['Smoothing', fmt.s(t.smoothing_s)] : null,
    ['Backend total', fmt.s(t.total_s)],
  ]));
  const evs = (r.events || []).filter((e) => e.lon != null);
  if (evs.length) {
    nodes.push(el('div', { class: 'eng-events' }, evs.map((e) => el('button', { class: 'btn btn-sm', onclick: () => mapView.fitTo([{ lon: e.lon - 0.01, lat: e.lat - 0.01 }, { lon: e.lon + 0.01, lat: e.lat + 0.01 }]) },
      el('span', {}, e.kind.replace(/_/g, ' ')), el('span', {}, e.value != null ? `${e.value.toFixed(1)} ${e.unit}` : e.detail || '')))));
  }
  box.replaceChildren(...nodes);
}

boot();
