// Trajectory charts (uPlot, canvas). All series are backend telemetry columns.
import uPlot from '../vendor/uplot/uPlot.esm.js';

const AXIS = { stroke: '#94A3B8', grid: { stroke: 'rgba(148,163,184,.12)', width: 1 }, ticks: { stroke: 'rgba(148,163,184,.25)', width: 1 }, font: '11px "Fira Code", monospace', labelFont: '11px "Fira Sans", sans-serif' };

const constant = (n, v) => new Array(n).fill(v);

function tabSpec(tab, res) {
  const c = res.stages.final.columns;
  const n = c.s.length;
  const km = c.s.map((v) => v / 1000);
  const minAgl = res.mission.min_agl_m, target = res.mission.target_agl_m;
  const lim = res.limits;
  switch (tab) {
    case 'profile':
      return {
        data: [km, c.ground, c.ground.map((g) => g + minAgl), c.ground.map((g) => g + target), c.z],
        series: [
          {},
          { label: 'Planner terrain', stroke: '#A08463', width: 1, fill: 'rgba(139,115,85,.45)', unit: 'm MSL' },
          { label: `Safety floor (+${minAgl} m)`, stroke: '#EF4444', width: 1, dash: [5, 4], unit: 'm MSL' },
          { label: `Target (+${target} m)`, stroke: 'rgba(226,232,240,.5)', width: 1, dash: [1, 3], unit: 'm MSL' },
          { label: 'Aircraft altitude', stroke: '#38BDF8', width: 2, unit: 'm MSL' },
        ],
        yLabel: 'Altitude MSL (m)',
        fillFrom: 1,
      };
    case 'agl':
      return {
        data: [km, c.agl, constant(n, minAgl), constant(n, target)],
        series: [
          {},
          { label: 'AGL', stroke: '#38BDF8', width: 2, fill: 'rgba(56,189,248,.10)', unit: 'm' },
          { label: 'Min AGL', stroke: '#EF4444', width: 1, dash: [5, 4], unit: 'm' },
          { label: 'Target AGL', stroke: 'rgba(226,232,240,.5)', width: 1, dash: [1, 3], unit: 'm' },
        ],
        yLabel: 'Height above planner terrain (m)',
        yMin: 0,
      };
    case 'vertical':
      return {
        data: [km, c.vz, constant(n, lim.max_climb_rate_mps), constant(n, -lim.max_descent_rate_mps)],
        series: [
          {},
          { label: 'Vertical rate', stroke: '#2DD4BF', width: 1.5, unit: 'm/s' },
          { label: 'Climb limit', stroke: 'rgba(239,68,68,.8)', width: 1, dash: [5, 4], unit: 'm/s' },
          { label: 'Descent limit', stroke: 'rgba(239,68,68,.8)', width: 1, dash: [5, 4], unit: 'm/s' },
        ],
        yLabel: 'v_z (m/s)',
      };
    case 'lateral':
      return {
        data: [km, c.bank, c.roll_rate, constant(n, lim.max_bank_deg), constant(n, -lim.max_bank_deg)],
        series: [
          {},
          { label: 'Bank', stroke: '#C084FC', width: 1.5, unit: '°' },
          { label: 'Roll rate', stroke: 'rgba(148,163,184,.8)', width: 1, unit: '°/s' },
          { label: 'Bank limit', stroke: 'rgba(239,68,68,.8)', width: 1, dash: [5, 4], unit: '°' },
          { label: '', stroke: 'rgba(239,68,68,.8)', width: 1, dash: [5, 4], unit: '°' },
        ],
        yLabel: 'Bank (°) · roll rate (°/s)',
      };
  }
  return null;
}

export class ChartDock extends EventTarget {
  constructor(container, readout) {
    super();
    this.container = container;
    this.readout = readout;
    this.tab = 'profile';
    this.result = null;
    this.plot = null;
    this.syncing = false;
    new ResizeObserver(() => this._resize()).observe(container);
  }

  setResult(result) {
    this.result = result?.stages?.final ? result : null;
    this._render();
  }

  setTab(tab) { this.tab = tab; this._render(); }

  _resize() {
    if (this.plot) this.plot.setSize({ width: this.container.clientWidth, height: this.container.clientHeight });
  }

  _render() {
    this.plot?.destroy();
    this.plot = null;
    this.readout.textContent = '';
    if (!this.result) return;
    const spec = tabSpec(this.tab, this.result);
    const self = this;
    const w = this.container.clientWidth, h = this.container.clientHeight;
    const opts = {
      width: w, height: h,
      padding: [8, 12, 0, 4],
      legend: { show: false },
      cursor: { drag: { x: true, y: false }, points: { size: 7 } },
      scales: { x: { time: false }, y: spec.yMin != null ? { range: (u, min, max) => [spec.yMin, max * 1.08] } : {} },
      axes: [
        { ...AXIS, label: 'Distance along track (km)', labelSize: 18, size: 36 },
        { ...AXIS, label: spec.yLabel, labelSize: 18, size: 58 },
      ],
      series: spec.series.map((s, i) => (i === 0 ? { label: 'km' } : { ...s, points: { show: false } })),
      hooks: {
        setCursor: [(u) => {
          const i = u.cursor.idx;
          self._updateReadout(i);
          if (!self.syncing) self.dispatchEvent(new CustomEvent('hover', { detail: { index: i == null ? -1 : i } }));
        }],
        setSelect: [(u) => {
          if (u.select.width < 4) return;
          const a = u.posToVal(u.select.left, 'x'), b = u.posToVal(u.select.left + u.select.width, 'x');
          u.setScale('x', { min: a, max: b });
          u.setSelect({ width: 0, height: 0 }, false);
        }],
      },
    };
    if (spec.fillFrom) {
      // Terrain area fills down to the plot floor.
      opts.series[1].fillTo = (u) => u.scales.y.min;
    }
    this.plot = new uPlot(opts, spec.data, this.container);
    this.plot.over.addEventListener('dblclick', () => this.plot.setScale('x', { min: spec.data[0][0], max: spec.data[0][spec.data[0].length - 1] }));
    this.plot.over.setAttribute('title', 'Drag to zoom · double-click to reset');
  }

  _updateReadout(i) {
    if (i == null || !this.result) { this.readout.textContent = ''; return; }
    const c = this.result.stages.final.columns;
    const parts = [
      `s ${(c.s[i] / 1000).toFixed(2)} km`,
      `t ${c.t[i].toFixed(0)} s`,
      `alt ${c.z[i].toFixed(0)} m MSL`,
      `terrain ${c.ground[i].toFixed(0)} m`,
      `AGL ${c.agl[i].toFixed(0)} m`,
      `hdg ${c.heading[i].toFixed(0)}°`,
      `vz ${c.vz[i] >= 0 ? '+' : ''}${c.vz[i].toFixed(1)} m/s`,
      `bank ${c.bank[i].toFixed(1)}°`,
    ];
    this.readout.textContent = parts.join('  ·  ');
  }

  // Map -> chart cursor sync.
  showIndex(i) {
    if (!this.plot) return;
    this.syncing = true;
    if (i < 0) {
      this.plot.setCursor({ left: -10, top: -10 });
    } else {
      const x = this.plot.valToPos(this.plot.data[0][i], 'x');
      const yv = this.plot.data[this.tab === 'profile' ? 4 : 1][i];
      this.plot.setCursor({ left: x, top: this.plot.valToPos(yv, 'y') });
    }
    this.syncing = false;
  }
}
