export const fmt = {
  m: (v, d = 0) => (v == null || !isFinite(v) ? '—' : `${v.toFixed(d)} m`),
  km: (m, d = 2) => (m == null || !isFinite(m) ? '—' : `${(m / 1000).toFixed(d)} km`),
  num: (v, d = 1) => (v == null || !isFinite(v) ? '—' : v.toFixed(d)),
  int: (v) => (v == null || !isFinite(v) ? '—' : Math.round(v).toLocaleString('en-US')),
  s: (v) => (v == null || !isFinite(v) ? '—' : v < 60 ? `${v.toFixed(1)} s` : `${Math.floor(v / 60)} min ${Math.round(v % 60)} s`),
  deg: (v, d = 1) => (v == null || !isFinite(v) ? '—' : `${v.toFixed(d)}°`),
  signed: (v, d = 1) => (v == null || !isFinite(v) ? '—' : `${v >= 0 ? '+' : ''}${v.toFixed(d)}`),
  ll: (v) => (v == null || !isFinite(v) ? '—' : v.toFixed(5)),
};

export function el(tag, attrs = {}, ...children) {
  const node = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (v == null || v === false) continue;
    if (k === 'class') node.className = v;
    else if (k === 'html') node.innerHTML = v;
    else if (k.startsWith('on')) node.addEventListener(k.slice(2), v);
    else node.setAttribute(k, v === true ? '' : v);
  }
  for (const c of children.flat()) if (c != null) node.append(c.nodeType ? c : document.createTextNode(c));
  return node;
}

export const icon = (name) => {
  const s = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
  s.setAttribute('aria-hidden', 'true');
  const u = document.createElementNS('http://www.w3.org/2000/svg', 'use');
  u.setAttribute('href', `#i-${name}`);
  s.append(u);
  return s;
};

export const prefersReducedMotion = () => window.matchMedia('(prefers-reduced-motion: reduce)').matches;
