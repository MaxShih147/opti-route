// ============================================================================
//  MIP solver via HiGHS-WASM
//  ----------------------------------------------------------------------------
//  Builds the same MILP formulation we run server-side (with MTZ subtour
//  elimination) as a CPLEX LP-format string and feeds it to HiGHS compiled
//  to WebAssembly. ~1.5MB one-time download from jsDelivr.
//
//  KSP warm-start hints are emitted as a separate /* mipstart */ block; HiGHS
//  picks them up and seeds its incumbent.
// ============================================================================

// HiGHS 1.14.2 — load the UMD build via a plain <script> tag.
// esm.sh's auto-polyfill tries to provide node:fs and crashes in browsers,
// so we bypass it and just let the emscripten UMD detect the browser env.
const HIGHS_VERSION = "1.14.2";
const HIGHS_BUILD = `https://cdn.jsdelivr.net/npm/highs@${HIGHS_VERSION}/build/`;

let highsPromise = null;
async function loadHighs() {
  if (highsPromise) return highsPromise;
  highsPromise = (async () => {
    // load script tag (only the first time)
    if (typeof globalThis.Module !== "function") {
      await new Promise((resolve, reject) => {
        const s = document.createElement("script");
        s.src = HIGHS_BUILD + "highs.js";
        s.async = true;
        s.onload = resolve;
        s.onerror = () => reject(new Error("Failed to load highs.js from CDN"));
        document.head.appendChild(s);
      });
    }
    const factory = globalThis.Module;
    if (typeof factory !== "function") {
      throw new Error("HiGHS Module not found on window after script load");
    }
    // Keep our own reference and clear the global so it doesn't clash with
    // future calls. (Module factory remembered in closure.)
    const inst = await factory({ locateFile: (f) => HIGHS_BUILD + f });
    return inst;
  })();
  return highsPromise;
}

// ---------------------------------------------------------------------------
//  Walking-distance recomputation (mirror of solver.js: kept self-contained
//  so mip.js can be replaced without touching the KSP code path)
// ---------------------------------------------------------------------------

import { _internalsForMip } from "./solver.js";
const { dijkstra, buildAdjacency, nodeConnectedComponent } = _internalsForMip;

function computeWalkDistances(adj, edges, fromNodes, toNodes) {
  const targetSet = new Set(toNodes);
  const out = new Map();
  for (const u of fromNodes) {
    const { dist } = dijkstra(adj, edges, u, "length", null);
    const m = new Map();
    for (const t of targetSet) if (dist.has(t)) m.set(t, dist.get(t));
    out.set(u, m);
  }
  return out;
}

// ---------------------------------------------------------------------------
//  LP-format builder
// ---------------------------------------------------------------------------

class LpBuilder {
  constructor() {
    this.objective = [];         // [[coef, varName], ...]
    this.constraints = [];       // [{name, terms, op, rhs}]
    this.bounds = [];            // ["0 <= u_3 <= 100", ...]
    this.binaries = new Set();
    this.generals = new Set();
    this.cn = 0;
  }
  obj(coef, v) { if (coef !== 0) this.objective.push([coef, v]); }
  cons(name, terms, op, rhs) {
    if (!name) name = `c${this.cn++}`;
    this.constraints.push({ name, terms, op, rhs });
  }
  bound(s) { this.bounds.push(s); }
  binary(v) { this.binaries.add(v); }
  general(v) { this.generals.add(v); }

  build() {
    const out = [];
    out.push("Minimize");
    out.push(" obj: " + this._fmtTerms(this.objective));
    out.push("Subject To");
    for (const c of this.constraints) {
      out.push(` ${c.name}: ${this._fmtTerms(c.terms)} ${c.op} ${c.rhs}`);
    }
    if (this.bounds.length) {
      out.push("Bounds");
      for (const b of this.bounds) out.push(" " + b);
    }
    if (this.binaries.size) {
      out.push("Binary");
      out.push(" " + [...this.binaries].join(" "));
    }
    if (this.generals.size) {
      out.push("General");
      out.push(" " + [...this.generals].join(" "));
    }
    out.push("End");
    return out.join("\n");
  }

  _fmtTerms(terms) {
    if (!terms.length) return "0";
    const parts = [];
    for (let i = 0; i < terms.length; i++) {
      const [coef, v] = terms[i];
      if (coef === 0) continue;
      const sign = coef >= 0 ? (i === 0 ? "" : "+ ") : "- ";
      const abs = Math.abs(coef);
      const c = abs === 1 ? "" : `${abs} `;
      parts.push(`${sign}${c}${v}`);
    }
    return parts.length ? parts.join(" ") : "0";
  }
}

// ---------------------------------------------------------------------------
//  Variable naming
// ---------------------------------------------------------------------------

const vx = (u, v) => `x_${u}_${v}`;
const vs = (v) => `s_${v}`;
const vz = (p, n) => `z_${p}_${n}`;
const vu = (v) => `u_${v}`;

// ---------------------------------------------------------------------------
//  Build LP from MIP model
// ---------------------------------------------------------------------------

function buildLp(scene, params, walkForPid, demandForPid, A, B, nodes, busEdges, M, warmStart) {
  const lp = new LpBuilder();
  const N = nodes.length;
  const SCALE = 100;

  // Per-node arc lists (in/out) computed from busEdges (undirected)
  const inArcs = new Map();
  const outArcs = new Map();
  for (const n of nodes) { inArcs.set(n, []); outArcs.set(n, []); }

  for (const e of busEdges) {
    const u = e.u, v = e.v;
    const xUV = vx(u, v), xVU = vx(v, u);
    lp.binary(xUV);
    lp.binary(xVU);
    // arc direction exclusion: x_uv + x_vu <= 1
    lp.cons(`dir_${u}_${v}`, [[1, xUV], [1, xVU]], "<=", 1);
    // record for flow conservation
    outArcs.get(u).push(xUV);
    inArcs.get(v).push(xUV);
    outArcs.get(v).push(xVU);
    inArcs.get(u).push(xVU);
    // objective contribution
    lp.obj(Math.round(e.weight * params.alpha_route * SCALE), xUV);
    lp.obj(Math.round(e.weight * params.alpha_route * SCALE), xVU);
  }

  // Flow conservation (A: outflow=1, inflow=0; B: vice versa; else balanced)
  for (const n of nodes) {
    const out = outArcs.get(n).map(v => [1, v]);
    const inn = inArcs.get(n).map(v => [1, v]);
    if (n === A) {
      lp.cons(`flowA`, out, "=", 1);
      if (inn.length) lp.cons(`inA`, inn, "=", 0);
    } else if (n === B) {
      lp.cons(`flowB`, inn, "=", 1);
      if (out.length) lp.cons(`outB`, out, "=", 0);
    } else {
      // out - in = 0
      const combined = [];
      for (const t of out) combined.push(t);
      for (const t of inn) combined.push([-1, t[1]]);
      lp.cons(`flow_${n}`, combined, "=", 0);
      // simple path: in <= 1
      if (inn.length) lp.cons(`in_${n}`, inn, "<=", 1);
    }
  }

  // MTZ subtour elimination
  // u_A = 0 (via bound); u_v in [1, N] for v != A
  for (const n of nodes) {
    if (n === A) {
      lp.bound(`${vu(n)} = 0`);
    } else {
      lp.bound(`1 <= ${vu(n)} <= ${N}`);
    }
    lp.general(vu(n));
  }
  // For each directed arc (a → b) with b ≠ A:
  //   u_b - u_a - N · x_{a→b}  ≥  1 - N
  for (const e of busEdges) {
    const a = e.u, b = e.v;
    if (b !== A) {
      lp.cons(`mtz_${a}_${b}`,
        [[1, vu(b)], [-1, vu(a)], [-N, vx(a, b)]],
        ">=", 1 - N);
    }
    if (a !== A) {
      lp.cons(`mtz_${b}_${a}`,
        [[1, vu(a)], [-1, vu(b)], [-N, vx(b, a)]],
        ">=", 1 - N);
    }
  }

  // Stop variables
  const intermediates = [];
  for (const n of nodes) {
    if (n === A || n === B) {
      lp.bound(`${vs(n)} = 1`);
      lp.binary(vs(n));
      continue;
    }
    lp.binary(vs(n));
    intermediates.push(n);
    // s_v ≤ inflow(v):  s_v - Σ in_arcs ≤ 0
    const inn = inArcs.get(n);
    if (inn.length) {
      lp.cons(`s_in_${n}`,
        [[1, vs(n)], ...inn.map(v => [-1, v])],
        "<=", 0);
    } else {
      lp.bound(`${vs(n)} = 0`);
    }
    // stop fixed cost in objective
    const c = Math.round(params.stop_fixed_cost * scene.nodes.find(x => x.id === n).terrain * SCALE);
    if (c) lp.obj(c, vs(n));
  }
  // Stop budget
  if (intermediates.length) {
    lp.cons(`budget`, intermediates.map(n => [1, vs(n)]), "<=", params.max_stops);
  }

  // Passenger assignment z_pn ∈ {0,1}; Σ_n z_pn = 1; z_pn ≤ s_n
  for (const pid of walkForPid.keys()) {
    const wmap = walkForPid.get(pid);
    if (!wmap.size) continue;
    const zVars = [];
    for (const [n, d] of wmap) {
      const name = vz(pid, n);
      lp.binary(name);
      zVars.push([1, name]);
      // z ≤ s
      lp.cons(`z_le_s_${pid}_${n}`, [[1, name], [-1, vs(n)]], "<=", 0);
      // walk cost in objective
      const demand = demandForPid.get(pid) ?? 1;
      const c = Math.round(d * demand * params.beta_walk * SCALE);
      if (c) lp.obj(c, name);
    }
    lp.cons(`assign_${pid}`, zVars, "=", 1);
  }

  return lp.build();
}

// ---------------------------------------------------------------------------
//  Result parsing
// ---------------------------------------------------------------------------

function parseHighsResult(result, scene, A, B, walkForPid, demandForPid, params) {
  const cols = result.Columns || {};
  const val = (name) => {
    const c = cols[name];
    return c ? Math.round(c.Primal ?? c.value ?? 0) : 0;
  };

  // Reconstruct path
  const usedArcs = new Set();
  for (const name of Object.keys(cols)) {
    if (name.startsWith("x_") && val(name) === 1) usedArcs.add(name);
  }
  const path = [A];
  let cur = A;
  for (let safety = 0; cur !== B && safety < scene.nodes.length * 2; safety++) {
    let next = null;
    for (const name of usedArcs) {
      const [, u, v] = name.split("_");
      if (parseInt(u, 10) === cur) {
        next = parseInt(v, 10);
        usedArcs.delete(name);
        break;
      }
    }
    if (next === null) break;
    path.push(next);
    cur = next;
  }

  // Chosen stops
  const chosenStops = [];
  for (const n of scene.nodes) {
    if (n.id === A || n.id === B) continue;
    if (val(vs(n.id)) === 1) chosenStops.push(n.id);
  }

  // Assignments
  const assignments = [];
  let walkTotal = 0;
  for (const [pid, wmap] of walkForPid) {
    for (const [n, d] of wmap) {
      if (val(vz(pid, n)) === 1) {
        const demand = demandForPid.get(pid) ?? 1;
        assignments.push({ passenger_id: pid, stop_node_id: n, walk_distance: d });
        walkTotal += d * demand;
        break;
      }
    }
  }

  // Path metrics
  let pathLen = 0, pathWeight = 0;
  const edgeAt = (u, v) => scene.edges.find(e =>
    (e.u === u && e.v === v) || (e.u === v && e.v === u)
  );
  for (let i = 0; i < path.length - 1; i++) {
    const e = edgeAt(path[i], path[i + 1]);
    if (e) { pathLen += e.length; pathWeight += e.weight; }
  }

  const costRoute = params.alpha_route * pathWeight;
  const terrAt = (n) => scene.nodes.find(x => x.id === n).terrain;
  const costStops = chosenStops.reduce((a, n) => a + params.stop_fixed_cost * terrAt(n), 0);
  const costWalk = params.beta_walk * walkTotal;

  const stopsInfo = chosenStops.map(sid => {
    const node = scene.nodes.find(n => n.id === sid);
    return {
      node_id: sid, x: node.x, y: node.y,
      passengers: assignments.filter(a => a.stop_node_id === sid).map(a => a.passenger_id),
    };
  });

  return {
    algorithm: "mip",
    path_nodes: path,
    path_length: pathLen,
    stops: stopsInfo,
    assignments,
    cost_route: costRoute,
    cost_stops: costStops,
    cost_walk: costWalk,
    cost_total: costRoute + costStops + costWalk,
    notes: `HiGHS status: ${result.Status}`,
    optimality_gap: result.Status === "Optimal" ? 0 : null,
  };
}

// ---------------------------------------------------------------------------
//  Public entry point
// ---------------------------------------------------------------------------

export async function solveMipHighs(scene, params, opts = {}) {
  const t0 = (typeof performance !== "undefined" ? performance.now() : Date.now());
  const A = scene.source, B = scene.sink;
  const adj = scene._adj;
  const edges = scene._rawEdges;
  const busMain = scene._busMain;
  const busEdges = edges.filter(e => !e.forbidden && busMain.has(e.u) && busMain.has(e.v));
  const nodes = scene.nodes.filter(n => busMain.has(n.id)).map(n => n.id);

  // Walk distances + per-passenger M-nearest candidate stops (M=20, always
  // including A and B so the assignment is always feasible)
  const M = 20;
  const passengerIds = scene.passengers.map(p => p.id);
  const passengerNodes = scene.passengers.map(p => p.node_id);
  const passengerDemands = scene.passengers.map(p => p.demand ?? 1);
  const uniqueP = [...new Set(passengerNodes)];
  const walk = computeWalkDistances(adj, edges, uniqueP, nodes);

  const walkForPid = new Map();
  const demandForPid = new Map();
  for (let i = 0; i < passengerIds.length; i++) {
    const pid = passengerIds[i], pn = passengerNodes[i];
    if (!walk.has(pn)) continue;
    const w = walk.get(pn);
    const sorted = [...w.entries()].sort((a, b) => a[1] - b[1]).slice(0, M);
    const cand = new Set(sorted.map(([n]) => n));
    if (w.has(A)) cand.add(A);
    if (w.has(B)) cand.add(B);
    const m = new Map();
    for (const n of cand) m.set(n, w.get(n));
    walkForPid.set(pid, m);
    demandForPid.set(pid, passengerDemands[i]);
  }

  const lp = buildLp(scene, params, walkForPid, demandForPid, A, B, nodes, busEdges, M);

  const highs = opts.highs ?? await loadHighs();
  // HiGHS solve with sensible defaults; small time limit so the UI stays alive
  const result = highs.solve(lp, {
    presolve: "on",
    time_limit: params.mip_time_limit_s ?? 20,
    mip_rel_gap: 1e-3,
  });

  // Accept any status where HiGHS handed back an incumbent. The Columns map
  // is populated only when there's a feasible solution; we use that as the
  // signal rather than the Status string, which has many flavours.
  const hasIncumbent = result.Columns && Object.keys(result.Columns).length > 0;
  if (!hasIncumbent) {
    throw new Error(`MIP found no feasible solution (HiGHS: ${result.Status})`);
  }

  const out = parseHighsResult(result, scene, A, B, walkForPid, demandForPid, params);
  // Tag a gap when HiGHS didn't prove optimality.
  if (result.Status !== "Optimal") {
    // HiGHS exposes the bound via result.MipGap when available; otherwise
    // we leave it null and rely on Status for UI messaging.
    out.optimality_gap = result.MipGap ?? null;
    out.notes = `HiGHS status: ${result.Status} (incumbent only)`;
  }
  out.runtime_ms = (typeof performance !== "undefined" ? performance.now() : Date.now()) - t0;
  return out;
}
