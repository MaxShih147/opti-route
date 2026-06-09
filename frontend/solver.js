// ============================================================================
//  opti-route — fully-static solver (no backend)
//  Ported from backend/{graph_gen,algorithms}/*.py
//
//  Exports:
//    generateCity({rows, cols, n_passengers, seed, edge_drop_rate,
//                  forbidden_zones, arterial_count}) → scene
//    solveKsp(scene, {max_stops, alpha_route, beta_walk, stop_fixed_cost,
//                     k_paths, corridor_hops}) → result
//    editScene(scene, action)                          → mutated scene
// ============================================================================

// ---------------------------------------------------------------------------
//  Seeded RNG  (Mulberry32 — small, fast, good enough for procgen)
// ---------------------------------------------------------------------------

function makeRng(seed) {
  let s = seed >>> 0;
  function rand() {
    s = (s + 0x6D2B79F5) >>> 0;
    let t = s;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  }
  function uniform(lo, hi) { return lo + (hi - lo) * rand(); }
  function randint(lo, hi) { return Math.floor(lo + (hi - lo + 1) * rand()); }
  function gauss(mu = 0, sigma = 1) {
    // Box–Muller
    const u = Math.max(1e-12, rand());
    const v = rand();
    return mu + sigma * Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
  }
  function choice(arr) { return arr[Math.floor(rand() * arr.length)]; }
  function choices(values, weights) {
    const total = weights.reduce((a, b) => a + b, 0);
    let r = rand() * total;
    for (let i = 0; i < values.length; i++) {
      r -= weights[i];
      if (r <= 0) return values[i];
    }
    return values[values.length - 1];
  }
  return { rand, uniform, randint, gauss, choice, choices };
}

// ---------------------------------------------------------------------------
//  Multi-octave value noise (for terrain field)
// ---------------------------------------------------------------------------

function valueNoise2D(rows, cols, octaves, rng) {
  const field = Array.from({ length: rows }, () => new Float64Array(cols));
  let amp = 1.0, totalAmp = 0.0;
  for (let o = 0; o < octaves; o++) {
    const resR = Math.max(2, Math.floor(rows / Math.pow(2, octaves - o)));
    const resC = Math.max(2, Math.floor(cols / Math.pow(2, octaves - o)));
    const coarse = [];
    for (let r = 0; r < resR; r++) {
      const row = new Float64Array(resC);
      for (let c = 0; c < resC; c++) row[c] = rng.rand();
      coarse.push(row);
    }
    for (let r = 0; r < rows; r++) {
      for (let c = 0; c < cols; c++) {
        const fr = r * (resR - 1) / Math.max(1, rows - 1);
        const fc = c * (resC - 1) / Math.max(1, cols - 1);
        const r0 = Math.floor(fr), c0 = Math.floor(fc);
        const r1 = Math.min(r0 + 1, resR - 1), c1 = Math.min(c0 + 1, resC - 1);
        const dr = fr - r0, dc = fc - c0;
        const v =
          coarse[r0][c0] * (1 - dr) * (1 - dc) +
          coarse[r1][c0] * dr * (1 - dc) +
          coarse[r0][c1] * (1 - dr) * dc +
          coarse[r1][c1] * dr * dc;
        field[r][c] += amp * v;
      }
    }
    totalAmp += amp;
    amp *= 0.5;
  }
  for (let r = 0; r < rows; r++)
    for (let c = 0; c < cols; c++) field[r][c] /= totalAmp;
  return field;
}

// ---------------------------------------------------------------------------
//  Geometry helpers
// ---------------------------------------------------------------------------

function pointInDisk(px, py, cx, cy, r) {
  const dx = px - cx, dy = py - cy;
  return dx * dx + dy * dy <= r * r;
}

function amoebaPoints(cx, cy, baseR, rng, n = 28) {
  const harmonics = [
    [rng.uniform(0.10, 0.22), rng.uniform(0, 2 * Math.PI), 2],
    [rng.uniform(0.06, 0.16), rng.uniform(0, 2 * Math.PI), 3],
    [rng.uniform(0.04, 0.10), rng.uniform(0, 2 * Math.PI), 5],
  ];
  const pts = [];
  for (let i = 0; i < n; i++) {
    const theta = 2 * Math.PI * i / n;
    let r = baseR;
    for (const [amp, phase, k] of harmonics) r *= 1 + amp * Math.sin(k * theta + phase);
    pts.push([cx + r * Math.cos(theta), cy + r * Math.sin(theta)]);
  }
  return pts;
}

function pointInPolygon(px, py, poly) {
  let inside = false;
  const n = poly.length;
  if (n < 3) return false;
  for (let i = 0, j = n - 1; i < n; j = i++) {
    const [xi, yi] = poly[i], [xj, yj] = poly[j];
    if ((yi > py) !== (yj > py)) {
      const xCross = (xj - xi) * (py - yi) / (yj - yi) + xi;
      if (px < xCross) inside = !inside;
    }
  }
  return inside;
}

// ---------------------------------------------------------------------------
//  Graph data structure
// ---------------------------------------------------------------------------
//  Stored as: nodes (array of {id, x, y, terrain, row, col}),
//             edges (array of {u, v, length, weight, forbidden, arterial}),
//             adj   (Map<nodeId, Array<[neighborId, edgeIndex]>>)
//
//  We index edges so each (u,v) and (v,u) lookup is fast for both
//  bus and pedestrian graphs.

function buildAdjacency(nodes, edges) {
  const adj = new Map();
  for (const n of nodes) adj.set(n.id, []);
  for (let i = 0; i < edges.length; i++) {
    const e = edges[i];
    adj.get(e.u).push([e.v, i]);
    adj.get(e.v).push([e.u, i]);
  }
  return adj;
}

// ---------------------------------------------------------------------------
//  Dijkstra (single source, all-targets, optional edge filter)
// ---------------------------------------------------------------------------
//  edgeFilter(edge) → bool decides which edges are traversable.
//  weightKey: "weight" (bus cost) or "length" (pedestrian distance).

class MinHeap {
  constructor() { this.h = []; }
  push(item) {
    const h = this.h;
    h.push(item);
    let i = h.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (h[p][0] <= h[i][0]) break;
      [h[p], h[i]] = [h[i], h[p]];
      i = p;
    }
  }
  pop() {
    const h = this.h;
    if (!h.length) return null;
    const top = h[0];
    const last = h.pop();
    if (h.length) {
      h[0] = last;
      const n = h.length;
      let i = 0;
      while (true) {
        const l = 2 * i + 1, r = 2 * i + 2;
        let smallest = i;
        if (l < n && h[l][0] < h[smallest][0]) smallest = l;
        if (r < n && h[r][0] < h[smallest][0]) smallest = r;
        if (smallest === i) break;
        [h[i], h[smallest]] = [h[smallest], h[i]];
        i = smallest;
      }
    }
    return top;
  }
  size() { return this.h.length; }
}

function dijkstra(adj, edges, source, weightKey, edgeFilter) {
  const dist = new Map();
  const prev = new Map();
  dist.set(source, 0);
  const heap = new MinHeap();
  heap.push([0, source]);
  while (heap.size()) {
    const [d, u] = heap.pop();
    if (d > (dist.get(u) ?? Infinity)) continue;
    for (const [v, eIdx] of adj.get(u)) {
      const e = edges[eIdx];
      if (edgeFilter && !edgeFilter(e)) continue;
      const nd = d + e[weightKey];
      if (nd < (dist.get(v) ?? Infinity)) {
        dist.set(v, nd);
        prev.set(v, [u, eIdx]);
        heap.push([nd, v]);
      }
    }
  }
  return { dist, prev };
}

function dijkstraPath(adj, edges, source, target, weightKey, edgeFilter) {
  const { dist, prev } = dijkstra(adj, edges, source, weightKey, edgeFilter);
  if (!dist.has(target)) return null;
  const path = [];
  let cur = target;
  while (cur !== source) {
    path.push(cur);
    const p = prev.get(cur);
    if (!p) return null;
    cur = p[0];
  }
  path.push(source);
  path.reverse();
  return path;
}

function pathMetrics(path, adj, edges) {
  let L = 0, W = 0;
  for (let i = 0; i < path.length - 1; i++) {
    const u = path[i], v = path[i + 1];
    for (const [n, eIdx] of adj.get(u)) {
      if (n === v) {
        L += edges[eIdx].length;
        W += edges[eIdx].weight;
        break;
      }
    }
  }
  return [L, W];
}

// ---------------------------------------------------------------------------
//  Yen's K-shortest simple paths
// ---------------------------------------------------------------------------
//  Returns up to k simple paths from source to target, ordered by cost.

function yenKShortest(adj, edges, source, target, k, edgeFilter) {
  const A = [];  // accepted paths
  const B = [];  // candidate paths (cost, path)
  const first = dijkstraPath(adj, edges, source, target, "weight", edgeFilter);
  if (!first) return [];
  A.push(first);

  for (let kk = 1; kk < k; kk++) {
    const prevPath = A[kk - 1];
    for (let i = 0; i < prevPath.length - 1; i++) {
      const spurNode = prevPath[i];
      const rootPath = prevPath.slice(0, i + 1);

      // Edges to ignore: edges that are the (i, i+1) hop of any accepted
      // path whose root matches our current rootPath.
      const forbiddenEdges = new Set();
      for (const p of A) {
        if (p.length > i && arraysEqual(p.slice(0, i + 1), rootPath)) {
          // edge between p[i] and p[i+1]
          for (const [n, eIdx] of adj.get(p[i])) {
            if (n === p[i + 1]) { forbiddenEdges.add(eIdx); break; }
          }
        }
      }
      const rootSet = new Set(rootPath.slice(0, -1));   // nodes to exclude

      const customFilter = (e) => {
        if (edgeFilter && !edgeFilter(e)) return false;
        // skip the chosen-out edges
        return true;
      };

      // run Dijkstra from spurNode, skipping nodes in rootSet and forbiddenEdges
      const { dist, prev } = dijkstraSpur(
        adj, edges, spurNode, "weight",
        customFilter, rootSet, forbiddenEdges
      );
      if (!dist.has(target)) continue;
      // reconstruct spur path
      const spurPath = [];
      let cur = target;
      while (cur !== spurNode) {
        spurPath.push(cur);
        const p = prev.get(cur);
        if (!p) break;
        cur = p[0];
      }
      spurPath.push(spurNode);
      spurPath.reverse();

      const totalPath = rootPath.slice(0, -1).concat(spurPath);
      const cost = pathMetrics(totalPath, adj, edges)[1];

      // dedupe candidates
      const key = totalPath.join(",");
      if (!B.some(c => c[2] === key)) B.push([cost, totalPath, key]);
    }
    if (!B.length) break;
    B.sort((a, b) => a[0] - b[0]);
    A.push(B.shift()[1]);
  }
  return A;
}

function dijkstraSpur(adj, edges, source, weightKey, edgeFilter, excludedNodes, excludedEdges) {
  const dist = new Map();
  const prev = new Map();
  dist.set(source, 0);
  const heap = new MinHeap();
  heap.push([0, source]);
  while (heap.size()) {
    const [d, u] = heap.pop();
    if (d > (dist.get(u) ?? Infinity)) continue;
    for (const [v, eIdx] of adj.get(u)) {
      if (excludedEdges.has(eIdx)) continue;
      if (v !== source && excludedNodes.has(v)) continue;
      const e = edges[eIdx];
      if (edgeFilter && !edgeFilter(e)) continue;
      const nd = d + e[weightKey];
      if (nd < (dist.get(v) ?? Infinity)) {
        dist.set(v, nd);
        prev.set(v, [u, eIdx]);
        heap.push([nd, v]);
      }
    }
  }
  return { dist, prev };
}

function arraysEqual(a, b) {
  if (a.length !== b.length) return false;
  for (let i = 0; i < a.length; i++) if (a[i] !== b[i]) return false;
  return true;
}

// ---------------------------------------------------------------------------
//  Connected components (used to identify bus subgraph component for A)
// ---------------------------------------------------------------------------

function nodeConnectedComponent(adj, edges, source, edgeFilter) {
  const seen = new Set([source]);
  const stack = [source];
  while (stack.length) {
    const u = stack.pop();
    for (const [v, eIdx] of adj.get(u)) {
      if (edgeFilter && !edgeFilter(edges[eIdx])) continue;
      if (!seen.has(v)) { seen.add(v); stack.push(v); }
    }
  }
  return seen;
}

// ---------------------------------------------------------------------------
//  Corridor expansion (BFS hops on the full graph)
// ---------------------------------------------------------------------------

function corridorNodes(adj, path, hops) {
  const out = new Set(path);
  let frontier = new Set(path);
  for (let h = 0; h < hops; h++) {
    const nxt = new Set();
    for (const u of frontier) {
      for (const [v] of adj.get(u)) if (!out.has(v)) nxt.add(v);
    }
    for (const v of nxt) out.add(v);
    frontier = nxt;
  }
  return out;
}

// ---------------------------------------------------------------------------
//  Walking-distance matrix  (Dijkstra from each unique passenger node)
// ---------------------------------------------------------------------------

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
//  p-median / UFLP-with-budget  (greedy add → 1-swap → 1-drop)
// ---------------------------------------------------------------------------

function solvePmedian({
  passengerIds, passengerNodes, passengerDemands,
  candidateStops, walkDist,
  fixedStops, maxStops,
  stopFixedCost, terrainAtNode, betaWalk,
}) {
  const INF = Infinity;
  const stopCost = (s) => stopFixedCost * (terrainAtNode.get(s) ?? 1.0);
  const pidToDemand = new Map();
  for (let i = 0; i < passengerIds.length; i++) {
    pidToDemand.set(passengerIds[i], passengerDemands[i] ?? 1);
  }
  const fixedSet = new Set(fixedStops);

  function bestWalk(pn, active) {
    let bd = INF, bs = -1;
    const dPn = walkDist.get(pn);
    if (!dPn) return [bd, bs];
    for (const s of active) {
      const d = dPn.get(s) ?? INF;
      if (d < bd) { bd = d; bs = s; }
    }
    return [bd, bs];
  }

  function totalCost(activeIntermediates) {
    const active = new Set([...fixedSet, ...activeIntermediates]);
    let walkSum = 0;
    const assign = new Map();
    const perP = new Map();
    for (let i = 0; i < passengerIds.length; i++) {
      const pid = passengerIds[i], pn = passengerNodes[i];
      const [d, s] = bestWalk(pn, active);
      assign.set(pid, s);
      const dEff = d < INF ? d : 0;
      perP.set(pid, dEff);
      walkSum += dEff * pidToDemand.get(pid);
    }
    let stopSum = 0;
    for (const s of activeIntermediates) stopSum += stopCost(s);
    return [betaWalk * walkSum + stopSum, assign, perP];
  }

  let chosen = new Set();
  let [curCost, curAssign, curPerP] = totalCost(chosen);
  const candidatePool = candidateStops.filter(c => !fixedSet.has(c));

  // greedy add
  let improved = true;
  while (improved && chosen.size < maxStops) {
    improved = false;
    let bestGain = 0, bestPick = -1, bestState = null;
    for (const c of candidatePool) {
      if (chosen.has(c)) continue;
      const trial = new Set(chosen); trial.add(c);
      const [tCost, tAssign, tPerP] = totalCost(trial);
      const gain = curCost - tCost;
      if (gain > bestGain + 1e-9) {
        bestGain = gain; bestPick = c; bestState = [tCost, tAssign, tPerP];
      }
    }
    if (bestPick !== -1) {
      chosen.add(bestPick);
      [curCost, curAssign, curPerP] = bestState;
      improved = true;
    }
  }

  // 1-swap
  let moved = true, safety = 0;
  while (moved && safety < 50) {
    safety++;
    moved = false;
    outer:
    for (const sOut of [...chosen]) {
      for (const sIn of candidatePool) {
        if (chosen.has(sIn)) continue;
        const trial = new Set(chosen); trial.delete(sOut); trial.add(sIn);
        const [tCost, tAssign, tPerP] = totalCost(trial);
        if (tCost + 1e-9 < curCost) {
          chosen = trial;
          [curCost, curAssign, curPerP] = [tCost, tAssign, tPerP];
          moved = true; break outer;
        }
      }
    }
  }

  // 1-drop
  moved = true;
  while (moved) {
    moved = false;
    for (const sOut of [...chosen]) {
      const trial = new Set(chosen); trial.delete(sOut);
      const [tCost, tAssign, tPerP] = totalCost(trial);
      if (tCost + 1e-9 < curCost) {
        chosen = trial;
        [curCost, curAssign, curPerP] = [tCost, tAssign, tPerP];
        moved = true; break;
      }
    }
  }

  let totalWalk = 0;
  for (const [pid, d] of curPerP) totalWalk += d * pidToDemand.get(pid);
  let totalStopFixed = 0;
  for (const s of chosen) totalStopFixed += stopCost(s);

  return {
    chosenStops: [...chosen].sort((a, b) => a - b),
    assignment: curAssign,
    walkPerPassenger: curPerP,
    totalWalk, totalStopFixed,
  };
}

// ---------------------------------------------------------------------------
//  CITY GENERATOR
// ---------------------------------------------------------------------------

export function generateCity({
  rows = 12, cols = 16, n_passengers = 100, seed = 42,
  edge_drop_rate = 0.15, forbidden_zones = 2, arterial_count = 4,
  cell_size = 60,
}) {
  const cs = cell_size;
  const R = rows, C = cols;
  const rng = makeRng(seed);
  const jitter = 0.25;
  const terrainAmp = 0.8;
  const arterialMinSpan = 5;
  const terrainOctaves = 3;
  const forbiddenRadiusCells = [1, 2];

  // nodes
  const nodes = [];
  const nodeIdOf = new Map();
  const terrainField = valueNoise2D(R, C, terrainOctaves, rng);
  let nid = 0;
  for (let r = 0; r < R; r++) {
    for (let c = 0; c < C; c++) {
      const jx = (rng.rand() - 0.5) * 2 * jitter * cs;
      const jy = (rng.rand() - 0.5) * 2 * jitter * cs;
      const terrain = 1.0 + (terrainField[r][c] * 2 - 1) * terrainAmp;
      nodes.push({
        id: nid, x: c * cs + jx + cs, y: r * cs + jy + cs,
        terrain, row: r, col: c,
      });
      nodeIdOf.set(`${r},${c}`, nid);
      nid++;
    }
  }

  // ---- candidate edges (lattice + arterials, with random drop) ----
  const allEdges = [];
  function pushEdge(u, v, opts = {}) {
    const a = nodes[u], b = nodes[v];
    const length = Math.hypot(b.x - a.x, b.y - a.y);
    let terrainW = (a.terrain + b.terrain) / 2;
    if (opts.arterial) terrainW *= 0.7;
    allEdges.push({
      u, v, length, terrain: terrainW,
      weight: length * terrainW,
      forbidden: false,
      arterial: !!opts.arterial,
    });
  }

  const lattice = [];
  for (let r = 0; r < R; r++) {
    for (let c = 0; c < C; c++) {
      const u = nodeIdOf.get(`${r},${c}`);
      for (const [dr, dc] of [[0, 1], [1, 0]]) {
        const nr = r + dr, nc = c + dc;
        if (nr < R && nc < C) {
          lattice.push([u, nodeIdOf.get(`${nr},${nc}`)]);
        }
      }
    }
  }
  for (const [u, v] of lattice) {
    if (rng.rand() < edge_drop_rate) continue;
    pushEdge(u, v);
  }

  let attempts = 0, added = 0;
  const existingEdge = new Map();   // "u,v" → idx for quick "has_edge"
  for (let i = 0; i < allEdges.length; i++) {
    const e = allEdges[i];
    existingEdge.set(`${e.u},${e.v}`, i);
    existingEdge.set(`${e.v},${e.u}`, i);
  }
  while (added < arterial_count && attempts < arterial_count * 20) {
    attempts++;
    const u = rng.randint(0, nid - 1);
    const v = rng.randint(0, nid - 1);
    if (u === v) continue;
    const ua = nodes[u], va = nodes[v];
    if (Math.max(Math.abs(ua.row - va.row), Math.abs(ua.col - va.col)) < arterialMinSpan) continue;
    if (existingEdge.has(`${u},${v}`)) continue;
    pushEdge(u, v, { arterial: true });
    const idx = allEdges.length - 1;
    existingEdge.set(`${u},${v}`, idx);
    existingEdge.set(`${v},${u}`, idx);
    added++;
  }

  // ---- forbidden zones (with hard caps) ----
  const w = (C - 1) * cs + 2 * cs;
  const h = (R - 1) * cs + 2 * cs;
  const scale = R * C;
  const maxCov = scale < 200 ? 4 : 8;
  const MAX_PAIR_OVERLAP = 3;

  const nodeXY = nodes.map(n => [n.x, n.y, n.id]);

  function coveredSet(points) {
    const s = new Set();
    for (const [x, y, n] of nodeXY) if (pointInPolygon(x, y, points)) s.add(n);
    return s;
  }
  function pointInAnyZone(px, py, zones) {
    for (const z of zones) {
      if (Math.hypot(px - z.cx, py - z.cy) > z.r * 1.3) continue;
      if (pointInPolygon(px, py, z.points)) return true;
    }
    return false;
  }

  const forbiddenZones = [];
  const zoneCoveredSets = [];
  for (let zi = 0; zi < forbidden_zones; zi++) {
    let placed = false;
    for (let attempt = 0; attempt < 40 && !placed; attempt++) {
      const rc = rng.randint(2, Math.max(2, R - 3));
      const cc = rng.randint(2, Math.max(2, C - 3));
      const cx = cc * cs + cs;
      const cy = rc * cs + cs;
      for (let radCells = forbiddenRadiusCells[1]; radCells >= 1; radCells--) {
        const radius = radCells * cs;
        const points = amoebaPoints(cx, cy, radius, rng);
        const covered = coveredSet(points);
        if (covered.size > maxCov) continue;
        let overlap = false;
        for (const s of zoneCoveredSets) {
          let inter = 0;
          for (const v of covered) if (s.has(v)) inter++;
          if (inter > MAX_PAIR_OVERLAP) { overlap = true; break; }
        }
        if (overlap) continue;
        forbiddenZones.push({ cx, cy, r: radius, points });
        zoneCoveredSets.push(covered);
        placed = true;
        break;
      }
    }
  }

  // mark forbidden edges (any endpoint or midpoint inside a zone)
  for (const e of allEdges) {
    const a = nodes[e.u], b = nodes[e.v];
    const mx = (a.x + b.x) / 2, my = (a.y + b.y) / 2;
    if (pointInAnyZone(mx, my, forbiddenZones) ||
        pointInAnyZone(a.x, a.y, forbiddenZones) ||
        pointInAnyZone(b.x, b.y, forbiddenZones)) {
      e.forbidden = true;
    }
  }

  // ---- connectivity: keep largest full-graph component ----
  const adjAll = buildAdjacency(nodes, allEdges);
  // find largest connected component
  const seen = new Set();
  let largest = new Set();
  for (const n of nodes) {
    if (seen.has(n.id)) continue;
    const comp = new Set();
    const stack = [n.id];
    while (stack.length) {
      const u = stack.pop();
      if (comp.has(u)) continue;
      comp.add(u);
      for (const [v] of adjAll.get(u)) if (!comp.has(v)) stack.push(v);
    }
    for (const v of comp) seen.add(v);
    if (comp.size > largest.size) largest = comp;
  }
  const keptNodes = nodes.filter(n => largest.has(n.id));
  const keptEdges = allEdges.filter(e => largest.has(e.u) && largest.has(e.v));

  // ---- bus subgraph: largest component within (no forbidden) ----
  const adj = buildAdjacency(keptNodes, keptEdges);
  const busFilter = (e) => !e.forbidden;
  // find largest bus component
  const busSeen = new Set();
  let busMain = new Set();
  for (const n of keptNodes) {
    if (busSeen.has(n.id)) continue;
    const comp = nodeConnectedComponent(adj, keptEdges, n.id, busFilter);
    for (const v of comp) busSeen.add(v);
    if (comp.size > busMain.size) busMain = comp;
  }

  // pick A and B as closest-to-corner nodes IN the bus component
  function distSq(n, cx, cy) { return (n.x - cx) ** 2 + (n.y - cy) ** 2; }
  const candidatesAB = keptNodes.filter(n => busMain.has(n.id));
  let source = candidatesAB[0].id, sink = candidatesAB[0].id;
  let bestA = Infinity, bestB = Infinity;
  for (const n of candidatesAB) {
    const dA = distSq(n, cs, cs);
    const dB = distSq(n, (C - 1) * cs + cs, (R - 1) * cs + cs);
    if (dA < bestA) { bestA = dA; source = n.id; }
    if (dB < bestB) { bestB = dB; sink = n.id; }
  }

  // ---- passengers — hotspots + demand ----
  const nHotspots = Math.max(3, Math.min(8, Math.floor((R * C) / 30)));
  const hotspotCenters = [];
  for (let i = 0; i < nHotspots; i++) {
    for (let t = 0; t < 30; t++) {
      const hx = rng.uniform(cs * 1.5, w - cs * 1.5);
      const hy = rng.uniform(cs * 1.5, h - cs * 1.5);
      if (!pointInAnyZone(hx, hy, forbiddenZones)) {
        hotspotCenters.push([hx, hy]); break;
      }
    }
  }
  const sigma = cs * 1.3;
  const passengers = [];
  for (let i = 0; i < n_passengers; i++) {
    let px = 0, py = 0;
    for (let t = 0; t < 8; t++) {
      if (hotspotCenters.length && rng.rand() < 0.75) {
        const [hx, hy] = rng.choice(hotspotCenters);
        px = hx + rng.gauss(0, sigma);
        py = hy + rng.gauss(0, sigma);
      } else {
        px = rng.uniform(cs * 0.5, w - cs * 0.5);
        py = rng.uniform(cs * 0.5, h - cs * 0.5);
      }
      px = Math.max(cs * 0.4, Math.min(w - cs * 0.4, px));
      py = Math.max(cs * 0.4, Math.min(h - cs * 0.4, py));
      if (!pointInAnyZone(px, py, forbiddenZones)) break;
    }
    const demand = rng.choices([1, 2, 3, 4, 5, 6], [44, 25, 14, 8, 5, 4]);
    let nearest = candidatesAB[0].id, bestD = Infinity;
    for (const n of keptNodes) {
      const d = (n.x - px) ** 2 + (n.y - py) ** 2;
      if (d < bestD) { bestD = d; nearest = n.id; }
    }
    passengers.push({ id: i, x: px, y: py, node_id: nearest, demand });
  }

  return {
    nodes: keptNodes.map(n => ({
      id: n.id, x: n.x, y: n.y, terrain: n.terrain,
    })),
    edges: keptEdges,
    passengers,
    source, sink,
    forbidden_zones: forbiddenZones,
    width: w, height: h,
    // internal indices (not serialized)
    _adj: adj, _busMain: busMain, _rawEdges: keptEdges,
  };
}

// ---------------------------------------------------------------------------
//  SOLVE — KSP (Yen + corridor + p-median + path repair)
// ---------------------------------------------------------------------------

export function solveKsp(scene, {
  max_stops = 5, alpha_route = 1.0, beta_walk = 1.0, stop_fixed_cost = 50,
  k_paths = 6, corridor_hops = 3,
}) {
  const t0 = performance.now();
  const adj = scene._adj;
  const edges = scene._rawEdges;
  const A = scene.source, B = scene.sink;
  const busFilter = (e) => !e.forbidden;
  const busMain = scene._busMain;

  if (!busMain.has(A) || !busMain.has(B)) {
    throw new Error("no bus-feasible path between A and B");
  }

  // K candidate paths
  const paths = yenKShortest(adj, edges, A, B, k_paths, busFilter);
  if (!paths.length) throw new Error("no bus-feasible path between A and B");

  const terrainAt = new Map(scene.nodes.map(n => [n.id, n.terrain]));
  const passengerIds = scene.passengers.map(p => p.id);
  const passengerNodes = scene.passengers.map(p => p.node_id);
  const passengerDemands = scene.passengers.map(p => p.demand ?? 1);
  const uniqueP = [...new Set(passengerNodes)];

  let best = null;
  let bestCost = Infinity;
  let bestIdx = -1;

  for (let idx = 0; idx < paths.length; idx++) {
    const path = paths[idx];
    const [pathLen, pathWeight] = pathMetrics(path, adj, edges);
    const costRoute = alpha_route * pathWeight;

    const corridor = corridorNodes(adj, path, corridor_hops);
    const onPathSet = new Set(path);
    const onPath = path.filter(n => n !== A && n !== B);
    const offPath = [...corridor].filter(n => !onPathSet.has(n) && busMain.has(n));
    const candidates = onPath.concat(offPath);

    const walk = computeWalkDistances(adj, edges, uniqueP, candidates.concat([A, B]));

    const pmed = solvePmedian({
      passengerIds, passengerNodes, passengerDemands,
      candidateStops: candidates,
      walkDist: walk,
      fixedStops: [A, B],
      maxStops: max_stops,
      stopFixedCost: stop_fixed_cost,
      terrainAtNode: terrainAt,
      betaWalk: beta_walk,
    });

    let finalPath = path, finalLen = pathLen, finalWeight = pathWeight;
    const chosen = pmed.chosenStops;
    if (chosen.length) {
      // sort stops by projection along original path
      const pathPos = new Map(path.map((n, i) => [n, i]));
      const orderKey = (s) => {
        if (pathPos.has(s)) return pathPos.get(s);
        const visited = new Map([[s, 0]]);
        let q = [s];
        while (q.length) {
          const nxt = [];
          for (const u of q) {
            if (pathPos.has(u)) return pathPos.get(u);
            for (const [w] of adj.get(u)) {
              if (!visited.has(w)) {
                visited.set(w, visited.get(u) + 1);
                nxt.push(w);
              }
            }
          }
          q = nxt;
        }
        return 0;
      };
      const ordered = [...chosen].sort((a, b) => orderKey(a) - orderKey(b));
      const waypoints = [A, ...ordered, B];
      const repaired = [A];
      let ok = true;
      let repairLen = 0, repairWeight = 0;
      for (let i = 0; i < waypoints.length - 1; i++) {
        const seg = dijkstraPath(adj, edges, waypoints[i], waypoints[i + 1], "weight", busFilter);
        if (!seg) { ok = false; break; }
        for (let j = 1; j < seg.length; j++) repaired.push(seg[j]);
        const [sl, sw] = pathMetrics(seg, adj, edges);
        repairLen += sl; repairWeight += sw;
      }
      if (ok) {
        finalPath = repaired;
        finalLen = repairLen;
        finalWeight = repairWeight;
      } else {
        continue;  // skip this candidate
      }
    }

    const costRouteFinal = alpha_route * finalWeight;
    const costWalk = beta_walk * pmed.totalWalk;
    const costStops = pmed.totalStopFixed;
    const total = costRouteFinal + costStops + costWalk;

    if (total < bestCost) {
      bestCost = total;
      best = { finalPath, finalLen, pmed, costRoute: costRouteFinal, costWalk, costStops };
      bestIdx = idx;
    }
  }

  if (!best) {
    // fall back to bare geodesic
    const path = paths[0];
    const [pathLen, pathWeight] = pathMetrics(path, adj, edges);
    return {
      algorithm: "ksp",
      path_nodes: path,
      path_length: pathLen,
      stops: [],
      assignments: [],
      cost_route: alpha_route * pathWeight,
      cost_stops: 0,
      cost_walk: 0,
      cost_total: alpha_route * pathWeight,
      runtime_ms: performance.now() - t0,
      notes: "fell back to bare geodesic",
    };
  }

  const stopsInfo = best.pmed.chosenStops.map(sid => {
    const node = scene.nodes.find(n => n.id === sid);
    return {
      node_id: sid, x: node.x, y: node.y,
      passengers: [...best.pmed.assignment.entries()]
        .filter(([_, v]) => v === sid).map(([k]) => k),
    };
  });
  const assignments = [...best.pmed.assignment.entries()].map(([pid, sid]) => ({
    passenger_id: pid, stop_node_id: sid,
    walk_distance: best.pmed.walkPerPassenger.get(pid) ?? 0,
  }));

  return {
    algorithm: "ksp",
    path_nodes: best.finalPath,
    path_length: best.finalLen,
    stops: stopsInfo,
    assignments,
    cost_route: best.costRoute,
    cost_stops: best.costStops,
    cost_walk: best.costWalk,
    cost_total: best.costRoute + best.costStops + best.costWalk,
    runtime_ms: performance.now() - t0,
    notes: `Evaluated ${paths.length} candidate paths · winner #${bestIdx}`,
  };
}

// ---------------------------------------------------------------------------
//  MIP — delegated to mip.js (HiGHS-WASM)
// ---------------------------------------------------------------------------

export async function solveMip(scene, params) {
  const { solveMipHighs } = await import("./mip.js");
  return solveMipHighs(scene, params);
}

// Shared graph helpers exposed for mip.js
export const _internalsForMip = {
  dijkstra, buildAdjacency, nodeConnectedComponent,
};

// ---------------------------------------------------------------------------
//  Scene edits (move A/B, add/move/delete passenger)
// ---------------------------------------------------------------------------

export function editScene(scene, action) {
  if (!scene) throw new Error("no scene");
  function snapToNearest(x, y) {
    let best = scene.nodes[0].id, bestD = Infinity;
    for (const n of scene.nodes) {
      const d = (n.x - x) ** 2 + (n.y - y) ** 2;
      if (d < bestD) { bestD = d; best = n.id; }
    }
    return best;
  }
  const { type } = action;
  if (type === "set_source") {
    scene.source = action.node_id ?? snapToNearest(action.x, action.y);
  } else if (type === "set_sink") {
    scene.sink = action.node_id ?? snapToNearest(action.x, action.y);
  } else if (type === "add_passenger") {
    const nn = snapToNearest(action.x, action.y);
    const newId = (scene.passengers.reduce((m, p) => Math.max(m, p.id), -1) + 1);
    scene.passengers.push({
      id: newId, x: action.x, y: action.y, node_id: nn, demand: 1,
    });
  } else if (type === "move_passenger") {
    for (const p of scene.passengers) {
      if (p.id === action.passenger_id) {
        p.x = action.x; p.y = action.y;
        p.node_id = snapToNearest(action.x, action.y);
        break;
      }
    }
  } else if (type === "delete_passenger") {
    scene.passengers = scene.passengers.filter(p => p.id !== action.passenger_id);
  }
  return scene;
}
