// Thin HTTP client. The backend is the source of truth for every planner value.
async function request(path, opts = {}) {
  const res = await fetch(path, { headers: { 'Content-Type': 'application/json' }, ...opts });
  let body = null;
  try { body = await res.json(); } catch { /* non-JSON */ }
  if (!res.ok) {
    const err = new Error((body && body.error) || `HTTP ${res.status}`);
    err.status = res.status;
    throw err;
  }
  return body;
}

export const api = {
  health: () => request('/api/health'),
  meta: () => request('/api/meta'),
  regions: () => request('/api/regions'),
  presets: () => request('/api/presets'),
  point: (lon, lat, lateralBufferM, signal) =>
    request(`/api/point?lon=${lon}&lat=${lat}&lateral_buffer_m=${lateralBufferM}`, { signal }),
  hoverPoint: (lon, lat, signal) => request(`/api/point?lon=${lon}&lat=${lat}&planner=0`, { signal }),
  plan: (body) => request('/api/plan', { method: 'POST', body: JSON.stringify(body) }),
  job: (id, timeoutMs = 10000) => request(`/api/jobs/${encodeURIComponent(id)}`, { signal: AbortSignal.timeout(timeoutMs) }),
};
