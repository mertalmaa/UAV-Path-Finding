// MapLibre custom WebGL2 layer that draws the planner trajectory at its true
// MSL altitude above the 3D terrain.
//
// Rendering only: every vertex comes from backend trajectory columns
// (lon, lat, z_msl, planner ground). Positions are converted to Web Mercator
// with MapLibre's own MercatorCoordinate and expressed relative to a local
// origin (float64 on the CPU) so float32 GPU precision stays sub-metre.
// Altitude is multiplied by the terrain exaggeration so route and terrain
// share one vertical scale.

import { MercatorCoordinate } from '../vendor/maplibre-gl/maplibre-gl.mjs';

const LINE_VS = `#version 300 es
precision highp float;
uniform mat4 u_matrix;
uniform vec2 u_viewport;
uniform float u_width;
in vec3 a_pos; in vec3 a_prev; in vec3 a_next; in float a_side; in vec4 a_color;
out vec4 v_color; out float v_edge;
vec2 toScreen(vec4 c) { return (c.xy / c.w) * 0.5 * u_viewport; }
void main() {
  vec4 cur = u_matrix * vec4(a_pos, 1.0);
  vec4 prv = u_matrix * vec4(a_prev, 1.0);
  vec4 nxt = u_matrix * vec4(a_next, 1.0);
  vec2 sc = toScreen(cur), sp = toScreen(prv), sn = toScreen(nxt);
  vec2 d1 = sc - sp, d2 = sn - sc;
  vec2 dir = normalize((length(d1) > 1e-6 ? normalize(d1) : vec2(0.0)) + (length(d2) > 1e-6 ? normalize(d2) : vec2(0.0)) + vec2(1e-9, 0.0));
  vec2 normal = vec2(-dir.y, dir.x);
  vec2 offset = normal * u_width * 0.5 * a_side;
  cur.xy += offset / (0.5 * u_viewport) * cur.w;
  gl_Position = cur;
  v_color = a_color; v_edge = a_side;
}`;
const LINE_FS = `#version 300 es
precision highp float;
in vec4 v_color; in float v_edge; out vec4 fragColor;
void main() {
  float a = 1.0 - smoothstep(0.65, 1.0, abs(v_edge));
  fragColor = vec4(v_color.rgb * v_color.a * a, v_color.a * a);
}`;
const FLAT_VS = `#version 300 es
precision highp float;
uniform mat4 u_matrix; uniform float u_point_size;
in vec3 a_pos; in vec4 a_color; out vec4 v_color;
void main() { gl_Position = u_matrix * vec4(a_pos, 1.0); gl_PointSize = u_point_size; v_color = a_color; }`;
const FLAT_FS = `#version 300 es
precision highp float;
uniform bool u_round; in vec4 v_color; out vec4 fragColor;
void main() {
  if (u_round) { vec2 p = gl_PointCoord * 2.0 - 1.0; float r = dot(p, p); if (r > 1.0) discard;
    float ring = step(0.45, r); fragColor = vec4(mix(vec3(1.0), v_color.rgb, ring), 1.0); return; }
  fragColor = vec4(v_color.rgb * v_color.a, v_color.a);
}`;

function compile(gl, vs, fs) {
  const prog = gl.createProgram();
  for (const [type, src] of [[gl.VERTEX_SHADER, vs], [gl.FRAGMENT_SHADER, fs]]) {
    const sh = gl.createShader(type);
    gl.shaderSource(sh, src); gl.compileShader(sh);
    if (!gl.getShaderParameter(sh, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(sh));
    gl.attachShader(prog, sh);
  }
  gl.linkProgram(prog);
  if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(prog));
  return prog;
}

const hex = (h, a = 1) => {
  const n = parseInt(h.slice(1), 16);
  return [((n >> 16) & 255) / 255, ((n >> 8) & 255) / 255, (n & 255) / 255, a];
};

// Column-major 4x4 multiply (Float64) and translation.
function mul(a, b) {
  const o = new Float64Array(16);
  for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) {
    let s = 0; for (let k = 0; k < 4; k++) s += a[k * 4 + r] * b[c * 4 + k]; o[c * 4 + r] = s;
  }
  return o;
}
const translation = (x, y, z) => { const m = new Float64Array(16); m[0] = m[5] = m[10] = m[15] = 1; m[12] = x; m[13] = y; m[14] = z; return m; };

export class Route3DLayer {
  constructor({ id = 'route-3d', colors = {} } = {}) {
    this.id = id;
    this.type = 'custom';
    this.renderingMode = '3d';
    this.colors = { route: '#38BDF8', near: '#F59E0B', start: '#22C55E', goal: '#C084FC', ...colors };
    this.data = null;         // { lon[], lat[], z[], ground[], agl[] }
    this.minAgl = 100;
    this.exaggeration = 1;
    this.hoverIndex = -1;
    this.visible = true;
    this.buffers = null;
  }

  onAdd(map, gl) {
    this.map = map; this.gl = gl;
    this.lineProg = compile(gl, LINE_VS, LINE_FS);
    this.flatProg = compile(gl, FLAT_VS, FLAT_FS);
    this._rebuild();
  }

  onRemove() { this._freeBuffers(); }

  setTrajectory(cols, minAgl) {
    this.data = cols; this.minAgl = minAgl; this.hoverIndex = -1; this._rebuild();
  }
  clear() { this.data = null; this._rebuild(); }
  setExaggeration(e) { if (e !== this.exaggeration) { this.exaggeration = e; this._rebuild(); } }
  setHover(i) { if (i !== this.hoverIndex) { this.hoverIndex = i; this._rebuildHover(); this.map?.triggerRepaint(); } }
  setVisible(v) { this.visible = v; this.map?.triggerRepaint(); }

  _merc(lon, lat, alt) {
    const m = MercatorCoordinate.fromLngLat([lon, lat], alt * this.exaggeration);
    return [m.x - this.origin[0], m.y - this.origin[1], m.z];
  }

  _freeBuffers() {
    if (!this.buffers || !this.gl) return;
    for (const b of Object.values(this.buffers)) if (b && b.buf) this.gl.deleteBuffer(b.buf);
    this.buffers = null;
  }

  _upload(data) {
    const gl = this.gl; const buf = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, buf); gl.bufferData(gl.ARRAY_BUFFER, data, gl.STATIC_DRAW);
    return buf;
  }

  _rebuild() {
    if (!this.gl) return;
    this._freeBuffers();
    const d = this.data;
    if (!d || d.lon.length < 2) { this.map?.triggerRepaint(); return; }
    const n = d.lon.length;
    const o = MercatorCoordinate.fromLngLat([d.lon[0], d.lat[0]], 0);
    this.origin = [o.x, o.y];
    const air = new Array(n), gnd = new Array(n);
    for (let i = 0; i < n; i++) {
      air[i] = this._merc(d.lon[i], d.lat[i], d.z[i]);
      gnd[i] = this._merc(d.lon[i], d.lat[i], d.ground[i] ?? d.z[i]);
    }
    this.air = air; this.gnd = gnd;
    const cRoute = hex(this.colors.route), cNear = hex(this.colors.near);
    const colorAt = (i) => (d.agl[i] != null && d.agl[i] < this.minAgl + 20 ? cNear : cRoute);

    // Screen-space thick line: 2 vertices per sample, stride 3+3+3+1+4 = 14.
    const line = new Float32Array(n * 2 * 14);
    let k = 0;
    for (let i = 0; i < n; i++) {
      const p = air[i], pv = air[Math.max(0, i - 1)], nx = air[Math.min(n - 1, i + 1)], c = colorAt(i);
      for (const side of [-1, 1]) { line.set([...p, ...pv, ...nx, side, ...c], k); k += 14; }
    }
    // Clearance curtain: planner ground -> aircraft altitude.
    const curtain = new Float32Array(n * 2 * 7);
    k = 0;
    for (let i = 0; i < n; i++) {
      const c = colorAt(i);
      curtain.set([...gnd[i], c[0], c[1], c[2], 0.10], k); k += 7;
      curtain.set([...air[i], c[0], c[1], c[2], 0.30], k); k += 7;
    }
    // Start / goal masts.
    const cs = hex(this.colors.start), cg = hex(this.colors.goal);
    const masts = new Float32Array([
      ...gnd[0], ...cs, ...air[0], ...cs, ...gnd[n - 1], ...cg, ...air[n - 1], ...cg,
    ]);
    const tops = new Float32Array([...air[0], ...cs, ...air[n - 1], ...cg]);
    this.buffers = {
      line: { buf: this._upload(line), count: n * 2 },
      curtain: { buf: this._upload(curtain), count: n * 2 },
      masts: { buf: this._upload(masts), count: 4 },
      tops: { buf: this._upload(tops), count: 2 },
      hover: null,
    };
    this._rebuildHover();
    this.map?.triggerRepaint();
  }

  _rebuildHover() {
    if (!this.buffers || !this.gl) return;
    if (this.buffers.hover) { this.gl.deleteBuffer(this.buffers.hover.buf); this.buffers.hover = null; }
    const i = this.hoverIndex;
    if (i < 0 || !this.air || i >= this.air.length) return;
    const w = [1, 1, 1, 1];
    const arr = new Float32Array([...this.gnd[i], ...w, ...this.air[i], ...w]);
    this.buffers.hover = { buf: this._upload(arr), count: 2, point: this.air[i] };
  }

  _attrib(prog, name, size, stride, offset) {
    const gl = this.gl; const loc = gl.getAttribLocation(prog, name);
    if (loc < 0) return;
    gl.enableVertexAttribArray(loc);
    gl.vertexAttribPointer(loc, size, gl.FLOAT, false, stride, offset);
    return loc;
  }

  render(gl, options) {
    this.renderCount = (this.renderCount || 0) + 1;
    // mainMatrix maps Web Mercator [0..1] (z conformal) to clip space; modelViewProjectionMatrix is in world pixels.
    const mvp = options.defaultProjectionData.mainMatrix;
    this._lastMvp = mvp;
    if (!this.visible || !this.buffers) return;
    const matrix = new Float32Array(mul(Float64Array.from(mvp), translation(this.origin[0], this.origin[1], 0)));
    const vp = [gl.drawingBufferWidth, gl.drawingBufferHeight];
    const dpr = window.devicePixelRatio || 1;
    const used = [];
    const flat = this.flatProg;

    gl.enable(gl.BLEND);
    gl.blendFunc(gl.ONE, gl.ONE_MINUS_SRC_ALPHA);
    gl.enable(gl.DEPTH_TEST);
    gl.depthFunc(gl.LEQUAL);

    // Curtain (no depth write so the route line is never hidden by it).
    gl.useProgram(flat);
    gl.uniformMatrix4fv(gl.getUniformLocation(flat, 'u_matrix'), false, matrix);
    gl.uniform1f(gl.getUniformLocation(flat, 'u_point_size'), 1);
    gl.uniform1i(gl.getUniformLocation(flat, 'u_round'), 0);
    gl.depthMask(false);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.buffers.curtain.buf);
    used.push(this._attrib(flat, 'a_pos', 3, 28, 0), this._attrib(flat, 'a_color', 4, 28, 12));
    gl.drawArrays(gl.TRIANGLE_STRIP, 0, this.buffers.curtain.count);
    gl.depthMask(true);

    // Masts + hover drop line.
    gl.bindBuffer(gl.ARRAY_BUFFER, this.buffers.masts.buf);
    this._attrib(flat, 'a_pos', 3, 28, 0); this._attrib(flat, 'a_color', 4, 28, 12);
    gl.lineWidth(1);
    gl.drawArrays(gl.LINES, 0, this.buffers.masts.count);
    if (this.buffers.hover) {
      gl.bindBuffer(gl.ARRAY_BUFFER, this.buffers.hover.buf);
      this._attrib(flat, 'a_pos', 3, 28, 0); this._attrib(flat, 'a_color', 4, 28, 12);
      gl.drawArrays(gl.LINES, 0, 2);
    }
    for (const l of used) if (l != null) gl.disableVertexAttribArray(l);

    // Route line.
    const lp = this.lineProg;
    gl.useProgram(lp);
    gl.uniformMatrix4fv(gl.getUniformLocation(lp, 'u_matrix'), false, matrix);
    gl.uniform2f(gl.getUniformLocation(lp, 'u_viewport'), vp[0], vp[1]);
    gl.uniform1f(gl.getUniformLocation(lp, 'u_width'), 4.0 * dpr);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.buffers.line.buf);
    const la = [
      this._attrib(lp, 'a_pos', 3, 56, 0), this._attrib(lp, 'a_prev', 3, 56, 12), this._attrib(lp, 'a_next', 3, 56, 24),
      this._attrib(lp, 'a_side', 1, 56, 36), this._attrib(lp, 'a_color', 4, 56, 40),
    ];
    gl.drawArrays(gl.TRIANGLE_STRIP, 0, this.buffers.line.count);
    for (const l of la) if (l != null) gl.disableVertexAttribArray(l);

    // Start / goal / hover points (always on top for readability).
    gl.useProgram(flat);
    gl.disable(gl.DEPTH_TEST);
    gl.uniform1i(gl.getUniformLocation(flat, 'u_round'), 1);
    gl.uniform1f(gl.getUniformLocation(flat, 'u_point_size'), 12 * dpr);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.buffers.tops.buf);
    const ta = [this._attrib(flat, 'a_pos', 3, 28, 0), this._attrib(flat, 'a_color', 4, 28, 12)];
    gl.drawArrays(gl.POINTS, 0, 2);
    if (this.buffers.hover) {
      gl.uniform1f(gl.getUniformLocation(flat, 'u_point_size'), 11 * dpr);
      gl.bindBuffer(gl.ARRAY_BUFFER, this.buffers.hover.buf);
      this._attrib(flat, 'a_pos', 3, 28, 0); this._attrib(flat, 'a_color', 4, 28, 12);
      gl.drawArrays(gl.POINTS, 1, 1);
    }
    for (const l of ta) if (l != null) gl.disableVertexAttribArray(l);
    gl.enable(gl.DEPTH_TEST);
  }

  // Screen projection of a trajectory sample using the same matrix as render().
  // Used by tests to verify terrain/trajectory alignment.
  projectSample(i, useGround = false) {
    const mvp = this._lastMvp;
    const p = (useGround ? this.gnd : this.air)[i];
    const m = mul(Float64Array.from(mvp), translation(this.origin[0], this.origin[1], 0));
    const x = m[0] * p[0] + m[4] * p[1] + m[8] * p[2] + m[12];
    const y = m[1] * p[0] + m[5] * p[1] + m[9] * p[2] + m[13];
    const w = m[3] * p[0] + m[7] * p[1] + m[11] * p[2] + m[15];
    const c = this.map.getCanvas();
    return { x: (x / w + 1) / 2 * c.clientWidth, y: (1 - y / w) / 2 * c.clientHeight };
  }
}
