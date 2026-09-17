// Minimal observable store: one state object, shallow-merge updates, keyed listeners.
export function createStore(initial) {
  let state = initial;
  const listeners = new Set();
  return {
    get: () => state,
    set(patch) {
      const prev = state;
      state = { ...state, ...(typeof patch === 'function' ? patch(state) : patch) };
      for (const fn of listeners) fn(state, prev);
    },
    subscribe(fn) { listeners.add(fn); return () => listeners.delete(fn); },
  };
}
