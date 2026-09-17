// MapLibre map controller: one map, two views.
//  2D: flat hillshade + elevation tint, mission picking, route ground track.
//  3D: GPU terrain (same tile pyramid, quadtree LOD), route at true MSL via Route3DLayer.

import * as maplibregl from '../vendor/maplibre-gl/maplibre-gl.mjs';
import { Route3DLayer } from './route3d-layer.js';
import { prefersReducedMotion } from './format.js';

const COLORS = { route: '#38BDF8', near: '#F59E0B', start: '#22C55E', goal: '#C084FC', search: '#A78BFA', profiled: '#94A3B8' };
const EMPTY = { type: 'FeatureCollection', features: [] };

function tintRamp() {
  // Muted hypsometric tint tuned for a dark UI (route colours must stay dominant).
  return ['interpolate', ['linear'], ['elevation'],
    -10, '#132033', 0.5, '#132033', 1, '#23332b', 200, '#2c3f2f', 500, '#3b4a33', 900, '#4d5238',
    1300, '#5f5840', 1800, '#6d6150', 2500, '#7c7568', 3500, '#a3a09a', 4500, '#c9c9c6'];
}

export class MapView extends EventTarget {
  constructor(container, { terrainMeta }) {
    super();
    this.view = '2d';
    this.exaggeration = 1.5;
    this.pickMode = null;
    this.minAgl = 100;
    this.finalCols = null;
    this.markers = {};
    // Terrain tiles come from a second port when the server offers one, so a burst of
    // tile requests never occupies the browser's per-host connection slots used by the API.
    const tileOrigin = terrainMeta.tilePort ? `${location.protocol}//${location.hostname}:${terrainMeta.tilePort}` : location.origin;
    const tiles = [tileOrigin + '/api/terrain/{z}/{x}/{y}.png'];
    const demSource = {
      type: 'raster-dem', tiles, tileSize: terrainMeta.tileSize, encoding: 'terrarium',
      minzoom: terrainMeta.minzoom, maxzoom: terrainMeta.maxzoom, bounds: terrainMeta.bounds,
      attribution: 'Terrain: Copernicus GLO-30 (© DLR/Airbus, ESA)',
    };
    this.map = new maplibregl.Map({
      container,
      attributionControl: { compact: true },
      maxPitch: 80,
      hash: false,
      fadeDuration: 0,
      bounds: terrainMeta.bounds,
      fitBoundsOptions: { padding: 40 },
      style: {
        version: 8,
        sources: {
          'dem-shade': demSource,
          // Terrain mesh source declares 128 px tiles so MapLibre selects one zoom level finer DEM tiles
          // for the 3D surface than for 2D shading (same cached 256 px tiles, ~2x vertical-detail in 3D).
          'dem-terrain': { ...demSource, tileSize: 128 },
          osm: { type: 'raster', tiles: ['https://tile.openstreetmap.org/{z}/{x}/{y}.png'], tileSize: 256, maxzoom: 19, attribution: '© OpenStreetMap contributors' },
          roi: { type: 'geojson', data: EMPTY },
          direct: { type: 'geojson', data: EMPTY },
          'stage-search': { type: 'geojson', data: EMPTY },
          'stage-profiled': { type: 'geojson', data: EMPTY },
          'route-final': { type: 'geojson', data: EMPTY },
          events: { type: 'geojson', data: EMPTY },
        },
        sky: { 'sky-color': '#0B1120', 'horizon-color': '#1E293B', 'fog-color': '#0F172A', 'sky-horizon-blend': 0.6, 'horizon-fog-blend': 0.6, 'fog-ground-blend': 0.85, 'atmosphere-blend': 0 },
        layers: [
          { id: 'bg', type: 'background', paint: { 'background-color': '#0B1120' } },
          { id: 'relief', type: 'color-relief', source: 'dem-shade', paint: { 'color-relief-color': tintRamp(), 'color-relief-opacity': 1 } },
          { id: 'hillshade', type: 'hillshade', source: 'dem-shade', paint: {
            'hillshade-method': 'igor', 'hillshade-exaggeration': 0.6, 'hillshade-illumination-direction': 315,
            'hillshade-shadow-color': 'rgba(2,6,14,0.85)', 'hillshade-highlight-color': 'rgba(203,213,225,0.35)', 'hillshade-accent-color': 'rgba(2,6,14,0.5)' } },
          { id: 'osm', type: 'raster', source: 'osm', layout: { visibility: 'none' }, paint: { 'raster-opacity': 0.55, 'raster-saturation': -0.6 } },
          { id: 'roi-fill', type: 'fill', source: 'roi', paint: { 'fill-color': '#38BDF8', 'fill-opacity': 0.03 } },
          { id: 'roi-line', type: 'line', source: 'roi', paint: { 'line-color': '#94A3B8', 'line-width': 1.2, 'line-dasharray': [4, 3], 'line-opacity': 0.8 } },
          { id: 'direct', type: 'line', source: 'direct', paint: { 'line-color': '#E2E8F0', 'line-width': 1.2, 'line-dasharray': [2, 3], 'line-opacity': 0.55 } },
          { id: 'stage-search', type: 'line', source: 'stage-search', layout: { visibility: 'none', 'line-join': 'round' }, paint: { 'line-color': COLORS.search, 'line-width': 2, 'line-dasharray': [1.5, 1.5] } },
          { id: 'stage-profiled', type: 'line', source: 'stage-profiled', layout: { visibility: 'none', 'line-join': 'round' }, paint: { 'line-color': COLORS.profiled, 'line-width': 1.5 } },
          { id: 'route-casing', type: 'line', source: 'route-final', layout: { 'line-join': 'round', 'line-cap': 'round' }, paint: { 'line-color': '#05080f', 'line-width': 6, 'line-opacity': 0.85 } },
          { id: 'route-line', type: 'line', source: 'route-final', layout: { 'line-join': 'round', 'line-cap': 'round' }, paint: {
            'line-color': ['case', ['get', 'near'], COLORS.near, COLORS.route], 'line-width': 3 } },
          { id: 'events', type: 'circle', source: 'events', paint: { 'circle-radius': 4.5, 'circle-color': ['match', ['get', 'kind'], 'min_agl', COLORS.near, 'profile_failure', '#EF4444', '#E2E8F0'], 'circle-stroke-color': '#05080f', 'circle-stroke-width': 2 } },
        ],
      },
    });
    this.map.addControl(new maplibregl.NavigationControl({ visualizePitch: true }), 'bottom-right');
    this.map.addControl(new maplibregl.ScaleControl({ unit: 'metric' }), 'bottom-right');
    this.route3d = new Route3DLayer({ colors: COLORS });
    this.hoverMarker = new maplibregl.Marker({ element: Object.assign(document.createElement('div'), { className: 'mk-hover' }) });

    this.ready = new Promise((resolve) => this.map.once('load', resolve));
    this.ready.then(() => this._wire());
  }

  _wire() {
    const map = this.map;
    map.addLayer(this.route3d);
    this.route3d.setVisible(false);
    map.on('click', (e) => {
      const target = this.pickMode;
      if (!target) return;
      this.dispatchEvent(new CustomEvent('pick', { detail: { which: target, lon: e.lngLat.lng, lat: e.lngLat.lat } }));
    });
    let raf = 0;
    map.on('mousemove', (e) => {
      if (raf) return;
      raf = requestAnimationFrame(() => {
        raf = 0;
        this.dispatchEvent(new CustomEvent('cursor', { detail: { lon: e.lngLat.lng, lat: e.lngLat.lat, zoom: map.getZoom(),
          terrain: this.view === '3d' ? map.queryTerrainElevation(e.lngLat) : null } }));
        this._hoverRoute(e.point);
      });
    });
    map.on('mouseout', () => this.dispatchEvent(new CustomEvent('cursor', { detail: null })));
  }

  // ---------------------------------------------------------------- layers
  setLayerVisible(id, on) {
    const ids = id === 'roi' ? ['roi-fill', 'roi-line'] : [id];
    if (id === 'tile-debug') { this.map.showTileBoundaries = on; return; }
    for (const l of ids) if (this.map.getLayer(l)) this.map.setLayoutProperty(l, 'visibility', on ? 'visible' : 'none');
  }

  setStageVisible(stage, on) {
    if (stage === 'final') {
      this.setLayerVisible('route-casing', on); this.setLayerVisible('route-line', on);
      this.route3d.setVisible(on && this.view === '3d');
      this.finalVisible = on;
    } else {
      this.setLayerVisible(`stage-${stage}`, on);
    }
  }

  setRegions(regions) {
    this.map.getSource('roi').setData({ type: 'FeatureCollection', features: regions.map((r) => ({
      type: 'Feature', properties: { id: r.region_id, name: r.name }, geometry: { type: 'Polygon', coordinates: [r.roi_polygon] } })) });
  }

  // --------------------------------------------------------------- mission
  setPickMode(which) {
    this.pickMode = which;
    this.map.getContainer().classList.toggle('picking', !!which);
  }

  setMissionPoint(which, lonlat) {
    if (!lonlat) { this.markers[which]?.remove(); delete this.markers[which]; return; }
    let mk = this.markers[which];
    if (!mk) {
      const elx = document.createElement('div');
      elx.className = `mk mk-${which}`;
      elx.textContent = which === 'start' ? 'S' : 'G';
      elx.setAttribute('role', 'img');
      elx.setAttribute('aria-label', which === 'start' ? 'Start point' : 'Goal point');
      mk = new maplibregl.Marker({ element: elx, draggable: true });
      mk.on('dragend', () => {
        const ll = mk.getLngLat();
        this.dispatchEvent(new CustomEvent('pick', { detail: { which, lon: ll.lng, lat: ll.lat, drag: true } }));
      });
      this.markers[which] = mk;
    }
    mk.setLngLat([lonlat.lon, lonlat.lat]).addTo(this.map);
  }

  setDirectLine(a, b) {
    const data = a && b ? { type: 'Feature', geometry: { type: 'LineString', coordinates: [[a.lon, a.lat], [b.lon, b.lat]] } } : EMPTY;
    this.map.getSource('direct').setData(data);
  }

  // ------------------------------------------------------------ trajectory
  setResult(result) {
    const stages = result?.stages || {};
    const lineOf = (st) => st ? { type: 'Feature', geometry: { type: 'LineString', coordinates: st.columns.lon.map((x, i) => [x, st.columns.lat[i]]) } } : EMPTY;
    this.map.getSource('stage-search').setData(lineOf(stages.search || stages.search_best_partial));
    this.map.getSource('stage-profiled').setData(lineOf(stages.profiled));
    const fin = stages.final;
    this.finalCols = fin ? fin.columns : null;
    this.minAgl = result?.mission?.min_agl_m ?? 100;
    if (fin) {
      // Split into runs by near-floor class so colour is data-driven without per-vertex styling.
      const c = fin.columns, feats = [];
      let run = [[c.lon[0], c.lat[0]]], near = c.agl[0] < this.minAgl + 20;
      for (let i = 1; i < c.lon.length; i++) {
        const n = c.agl[i] < this.minAgl + 20;
        run.push([c.lon[i], c.lat[i]]);
        if (n !== near || i === c.lon.length - 1) {
          feats.push({ type: 'Feature', properties: { near }, geometry: { type: 'LineString', coordinates: run } });
          run = [[c.lon[i], c.lat[i]]]; near = n;
        }
      }
      this.map.getSource('route-final').setData({ type: 'FeatureCollection', features: feats });
      this.route3d.setTrajectory({ lon: c.lon, lat: c.lat, z: c.z, ground: c.ground, agl: c.agl }, this.minAgl);
    } else {
      this.map.getSource('route-final').setData(EMPTY);
      this.route3d.clear();
    }
    const ev = (result?.events || []).filter((e) => e.lon != null && (e.kind === 'min_agl' || e.kind === 'profile_failure'));
    this.map.getSource('events').setData({ type: 'FeatureCollection', features: ev.map((e) => ({
      type: 'Feature', properties: { kind: e.kind }, geometry: { type: 'Point', coordinates: [e.lon, e.lat] } })) });
    this.setDirectLine(null, null);
  }

  fitTo(points, { duration } = {}) {
    const pts = points.filter(Boolean);
    if (!pts.length) return;
    const b = new maplibregl.LngLatBounds();
    for (const p of pts) b.extend([p.lon, p.lat]);
    this.map.fitBounds(b, { padding: { top: 70, bottom: 50, left: 60, right: 60 }, maxZoom: 13,
      duration: prefersReducedMotion() ? 0 : (duration ?? 700), pitch: this.map.getPitch(), bearing: this.map.getBearing() });
  }

  fitRoute() {
    const c = this.finalCols;
    if (!c) return;
    let w = 180, s = 90, e = -180, n = -90;
    for (let i = 0; i < c.lon.length; i++) { w = Math.min(w, c.lon[i]); e = Math.max(e, c.lon[i]); s = Math.min(s, c.lat[i]); n = Math.max(n, c.lat[i]); }
    this.fitTo([{ lon: w, lat: s }, { lon: e, lat: n }]);
  }

  // ----------------------------------------------------------------- hover
  _hoverRoute(point) {
    const c = this.finalCols;
    if (!c || this.view !== '2d' || this.finalVisible === false) return;
    // Nearest sample in screen space (stride keeps this < 1 ms for 5k points).
    let best = -1, bestD = 14 * 14;
    const stride = c.lon.length > 2500 ? 2 : 1;
    for (let i = 0; i < c.lon.length; i += stride) {
      const p = this.map.project([c.lon[i], c.lat[i]]);
      const d = (p.x - point.x) ** 2 + (p.y - point.y) ** 2;
      if (d < bestD) { bestD = d; best = i; }
    }
    this.dispatchEvent(new CustomEvent('routehover', { detail: { index: best } }));
  }

  showHover(index) {
    const c = this.finalCols;
    if (!c || index < 0) { this.hoverMarker.remove(); this.route3d.setHover(-1); return; }
    if (this.view === '2d') {
      this.hoverMarker.setLngLat([c.lon[index], c.lat[index]]).addTo(this.map);
      this.route3d.setHover(-1);
    } else {
      this.hoverMarker.remove();
      this.route3d.setHover(index);
    }
  }

  // ------------------------------------------------------------------ view
  setView(view) {
    if (view === this.view) return;
    this.view = view;
    const map = this.map;
    const duration = prefersReducedMotion() ? 0 : 900;
    this.hoverMarker.remove();
    // In 3D the draped line becomes a subdued ground-track shadow under the airborne route.
    map.setPaintProperty('route-line', 'line-opacity', view === '3d' ? 0.4 : 1);
    map.setPaintProperty('route-line', 'line-width', view === '3d' ? 2 : 3);
    map.setPaintProperty('route-casing', 'line-opacity', view === '3d' ? 0.3 : 0.85);
    if (view === '3d') {
      map.setTerrain({ source: 'dem-terrain', exaggeration: this.exaggeration });
      this.route3d.setExaggeration(this.exaggeration);
      this.route3d.setVisible(this.finalVisible !== false);
      map.easeTo({ pitch: 62, bearing: map.getBearing() || -20, duration });
    } else {
      map.setTerrain(null);
      this.route3d.setVisible(false);
      this.route3d.setHover(-1);
      map.easeTo({ pitch: 0, bearing: 0, duration });
    }
    for (const mk of Object.values(this.markers)) mk.setDraggable(view === '2d');
  }

  setExaggeration(e) {
    this.exaggeration = e;
    if (this.view === '3d') this.map.setTerrain({ source: 'dem-terrain', exaggeration: e });
    this.route3d.setExaggeration(e);
  }

  resize() { this.map.resize(); }
}
