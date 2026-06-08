// Bus Route Optimizer — frontend.
// Single-page, vanilla JS, SVG canvas. Talks to the FastAPI backend over /api.

const $ = (sel) => document.querySelector(sel);
const $$ = (sel) => Array.from(document.querySelectorAll(sel));
const svgNS = "http://www.w3.org/2000/svg";

let scene = null;
let editMode = "none";
let lastResults = []; // history of solves, for comparison
let activeAlgo = null; // which algorithm's solution is currently rendered on the map
const ALGO_LABELS = {
  ksp: "K-最短路徑啟發式",
  mip: "混合整數規劃 (MILP)",
};
// short labels used in the comparison table to avoid line wrapping
const ALGO_SHORT = { ksp: "K", mip: "M" };

function status(text, kind = "") {
  const el = $("#status");
  el.textContent = text;
  el.className = "status " + kind;
}

// ---------------- SVG rendering ----------------

const SVG = $("#canvas");
const layers = {};

function setupSVG() {
  SVG.innerHTML = "";
  for (const name of ["edges", "forbidden", "terrain", "walk", "route", "passengers", "stops", "nodes", "endpoints", "labels"]) {
    const g = document.createElementNS(svgNS, "g");
    g.setAttribute("class", "layer-" + name);
    SVG.appendChild(g);
    layers[name] = g;
  }
}
setupSVG();

function fitToScene() {
  const w = scene.width;
  const h = scene.height;
  SVG.setAttribute("viewBox", `0 0 ${w} ${h}`);
  SVG.setAttribute("preserveAspectRatio", "xMidYMid meet");
}

function clear(layer) { while (layer.firstChild) layer.removeChild(layer.firstChild); }

function el(tag, attrs = {}, parent) {
  const e = document.createElementNS(svgNS, tag);
  for (const [k, v] of Object.entries(attrs)) e.setAttribute(k, v);
  if (parent) parent.appendChild(e);
  return e;
}

function renderScene() {
  fitToScene();
  const nodeById = {};
  for (const n of scene.nodes) nodeById[n.id] = n;

  // forbidden zones — amoeba polygons
  clear(layers.forbidden);
  for (const z of scene.forbidden_zones || []) {
    if (z.points && z.points.length > 2) {
      const pts = z.points.map(p => p.join(",")).join(" ");
      el("polygon", { points: pts, class: "forbidden-zone" }, layers.forbidden);
    } else {
      // legacy fallback: disk shape
      el("circle", { cx: z.cx, cy: z.cy, r: z.r, class: "forbidden-zone" }, layers.forbidden);
    }
  }

  // terrain overlay — vivid cyan (cheap) → vivid coral (expensive)
  // bigger radius so neighbouring dots overlap into a smooth wash.
  clear(layers.terrain);
  const tg = el("g", { class: "terrain-overlay" }, layers.terrain);
  for (const n of scene.nodes) {
    const t = n.terrain; // 0.2 .. 1.8 typical
    const norm = Math.max(0, Math.min(1, (t - 0.4) / 1.2));
    // gradient: cyan #00d4ff → coral #ff5a3a
    const r = Math.floor(0 + 255 * norm);
    const g = Math.floor(212 - 122 * norm);
    const b = Math.floor(255 - 197 * norm);
    el("circle", { cx: n.x, cy: n.y, r: 14, fill: `rgb(${r},${g},${b})` }, tg);
  }

  // edges
  clear(layers.edges);
  for (const e of scene.edges) {
    const a = nodeById[e.u], b = nodeById[e.v];
    if (!a || !b) continue;
    const cls = e.forbidden ? "edge forbidden" : (e.arterial ? "edge arterial" : "edge");
    el("line", { x1: a.x, y1: a.y, x2: b.x, y2: b.y, class: cls }, layers.edges);
  }

  // nodes (small invisible-ish dots, clickable)
  clear(layers.nodes);
  for (const n of scene.nodes) {
    const c = el("circle", { cx: n.x, cy: n.y, r: 2.5, class: "node", "data-id": n.id }, layers.nodes);
    c.addEventListener("click", (ev) => onNodeClick(n, ev));
  }

  // endpoints
  clear(layers.endpoints);
  const A = nodeById[scene.source], B = nodeById[scene.sink];
  if (A) {
    el("circle", { cx: A.x, cy: A.y, r: 9, class: "node src" }, layers.endpoints);
    el("text", { x: A.x, y: A.y - 14, "text-anchor": "middle", class: "label" }, layers.endpoints).textContent = "A";
  }
  if (B) {
    el("circle", { cx: B.x, cy: B.y, r: 9, class: "node sink" }, layers.endpoints);
    el("text", { x: B.x, y: B.y - 14, "text-anchor": "middle", class: "label" }, layers.endpoints).textContent = "B";
  }

  // passengers — radius scaled by demand (sqrt so big groups don't dominate)
  clear(layers.passengers);
  for (const p of scene.passengers) {
    const demand = Math.max(1, p.demand || 1);
    const r = 2.5 + Math.sqrt(demand - 1) * 2.2;
    const c = el("circle", {
      cx: p.x, cy: p.y, r,
      class: "passenger", "data-pid": p.id, "data-demand": demand,
    }, layers.passengers);
    c.addEventListener("click", (ev) => onPassengerClick(p, ev));
  }
}

function renderSolution(result) {
  const nodeById = {};
  for (const n of scene.nodes) nodeById[n.id] = n;

  SVG.classList.add("has-solution");

  // route polyline
  clear(layers.route);
  if (result && result.path_nodes && result.path_nodes.length > 1) {
    const pts = result.path_nodes.map(id => `${nodeById[id].x},${nodeById[id].y}`).join(" ");
    el("polyline", { points: pts, class: "edge route", fill: "none" }, layers.route);
  }

  // stops on top of nodes
  clear(layers.stops);
  if (result && result.stops) {
    for (const s of result.stops) {
      el("circle", { cx: s.x, cy: s.y, r: 7, class: "node stop" }, layers.stops);
    }
  }

  // walk lines passenger -> stop
  clear(layers.walk);
  if (result && result.assignments) {
    const stopById = {};
    if (result.stops) for (const s of result.stops) stopById[s.node_id] = s;
    // also A and B can host passengers (implicit stops)
    stopById[scene.source] = nodeById[scene.source];
    stopById[scene.sink] = nodeById[scene.sink];
    const passById = Object.fromEntries(scene.passengers.map(p => [p.id, p]));
    for (const a of result.assignments) {
      const p = passById[a.passenger_id];
      const s = stopById[a.stop_node_id];
      if (!p || !s) continue;
      el("line", { x1: p.x, y1: p.y, x2: s.x, y2: s.y, class: "walk-line" }, layers.walk);
    }
  }
}

function clearSolution() {
  clear(layers.route); clear(layers.stops); clear(layers.walk);
  SVG.classList.remove("has-solution");
}

// ---------------- Interactions ----------------

function onNodeClick(node, ev) {
  ev.stopPropagation();
  if (editMode === "set_source") {
    apiEdit({ action: "set_source", node_id: node.id });
  } else if (editMode === "set_sink") {
    apiEdit({ action: "set_sink", node_id: node.id });
  }
}

function onPassengerClick(p, ev) {
  ev.stopPropagation();
  if (editMode === "delete_passenger") {
    apiEdit({ action: "delete_passenger", passenger_id: p.id });
  }
}

function onCanvasClick(ev) {
  if (editMode !== "add_passenger") return;
  const pt = SVG.createSVGPoint();
  pt.x = ev.clientX; pt.y = ev.clientY;
  const ctm = SVG.getScreenCTM().inverse();
  const loc = pt.matrixTransform(ctm);
  apiEdit({ action: "add_passenger", x: loc.x, y: loc.y });
}
SVG.addEventListener("click", onCanvasClick);

$$(".mode").forEach(btn => btn.addEventListener("click", () => {
  $$(".mode").forEach(b => b.classList.remove("active"));
  btn.classList.add("active");
  editMode = btn.dataset.mode;
  SVG.style.cursor = (editMode === "none") ? "default" : "crosshair";
}));

// ---------------- API ----------------

async function apiGet(path) {
  const r = await fetch(path);
  if (!r.ok) throw new Error(`${path} ${r.status}`);
  return r.json();
}
async function apiPost(path, body) {
  const r = await fetch(path, {
    method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  if (!r.ok) {
    const text = await r.text();
    let detail = text;
    try { detail = JSON.parse(text).detail || text; } catch {}
    const err = new Error(detail);
    err.status = r.status;
    throw err;
  }
  return r.json();
}

async function loadScene() {
  scene = await apiGet("/api/scene");
  renderScene(); clearSolution();
  status(`scene: ${scene.nodes.length} 節點 · ${scene.edges.length} 邊 · ${scene.passengers.length} 乘客`);
}

async function regenerate() {
  status("生成中…", "busy");
  // 城市規模 → row × col 約略平方分解，城市偏橫向：cols ≥ rows
  const scale = Math.max(16, +$("#g-scale").value);
  const cols = Math.max(4, Math.ceil(Math.sqrt(scale)));
  const rows = Math.max(4, Math.floor(scale / cols));
  const params = {
    rows,
    cols,
    n_passengers: +$("#g-pass").value,
    seed: Math.floor(Math.random() * 1000000),
    edge_drop_rate: (+$("#g-drop").value) / 100,
    forbidden_zones: +$("#g-forb").value,
    arterial_count: 4,
  };
  try {
    scene = await apiPost("/api/generate", params);
    renderScene(); clearSolution();
    lastResults = []; activeAlgo = null; renderResultsTable();
    status(`已生成 · ${scene.nodes.length} 節點 · ${scene.edges.length} 邊 · ${scene.passengers.length} 乘客`);
  } catch (e) { status("生成失敗: " + e.message, "error"); }
}

async function apiEdit(body) {
  try {
    scene = await apiPost("/api/edit", body);
    renderScene(); clearSolution();
  } catch (e) { status(e.message, "error"); }
}

async function solve(algo) {
  const btn = $(`button.solver[data-algo="${algo}"]`);
  btn.disabled = true;
  status(`求解中 (${ALGO_LABELS[algo]}) …`, "busy");
  const kpInput = $("#p-kp");
  const corrInput = $("#p-corr");
  const params = {
    algorithm: algo,
    max_stops: +$("#p-k").value,
    alpha_route: +$("#p-alpha").value,
    beta_walk: +$("#p-beta").value,
    stop_fixed_cost: +$("#p-stop").value,
    k_paths: kpInput ? +kpInput.value : 6,
    corridor_hops: corrInput ? +corrInput.value : 3,
    mip_time_limit_s: 20.0,
  };
  try {
    const result = await apiPost("/api/solve", params);
    renderSolution(result);
    pushResult(result);
    let s = `${ALGO_LABELS[algo]} · 總成本 ${result.cost_total.toFixed(1)} · ${result.runtime_ms.toFixed(0)}ms`;
    if (result.optimality_gap != null) s += ` · gap ${(result.optimality_gap*100).toFixed(1)}%`;
    status(s);
  } catch (e) {
    const msg = (e && e.message) || "";
    let pretty;
    if (/no bus-feasible path|no path|disconnected/i.test(msg)) {
      pretty = `${ALGO_LABELS[algo]} · 無解 — A 與 B 在禁區阻隔下不連通`;
    } else if (/no feasible solution|INFEASIBLE/i.test(msg)) {
      pretty = `${ALGO_LABELS[algo]} · 無解 — 模型在限制下無可行解`;
    } else if (e && e.status === 422) {
      pretty = `${ALGO_LABELS[algo]} · 無解 — ${msg}`;
    } else {
      pretty = `求解失敗: ${msg}`;
    }
    status(pretty, "error");
  } finally {
    btn.disabled = false;
  }
}

// ---------------- Results comparison ----------------

function pushResult(r) {
  // keep at most one per algorithm (latest)
  lastResults = lastResults.filter(x => x.algorithm !== r.algorithm);
  lastResults.push(r);
  activeAlgo = r.algorithm;
  renderResultsTable();
}

function showResult(algo) {
  const r = lastResults.find(x => x.algorithm === algo);
  if (!r) return;
  activeAlgo = algo;
  renderSolution(r);
  renderResultsTable();
  let s = `${ALGO_LABELS[algo]} · 總成本 ${r.cost_total.toFixed(1)} · ${r.runtime_ms.toFixed(0)}ms`;
  if (r.optimality_gap != null) s += ` · gap ${(r.optimality_gap*100).toFixed(1)}%`;
  status(s);
}

function renderResultsTable() {
  const wrap = $("#results-table");
  const empty = $("#results-empty");
  if (lastResults.length === 0) {
    wrap.innerHTML = ""; empty.style.display = "block"; return;
  }
  empty.style.display = "none";
  const best = Math.min(...lastResults.map(r => r.cost_total));
  let html = `<table>
    <thead>
      <tr>
        <th class="col-algo" rowspan="2">方法</th>
        <th class="col-group" colspan="4">成本</th>
        <th class="col-group">運算時間</th>
      </tr>
      <tr>
        <th>總計</th><th>路線</th><th>設站</th><th>步行</th>
        <th class="col-unit">ms</th>
      </tr>
    </thead>
    <tbody>`;
  for (const r of lastResults) {
    const isBest = Math.abs(r.cost_total - best) < 1e-3;
    const isActive = r.algorithm === activeAlgo;
    const cls = [isBest ? "best" : "", isActive ? "active" : ""].filter(Boolean).join(" ");
    html += `<tr class="${cls}" data-algo="${r.algorithm}" title="${ALGO_LABELS[r.algorithm]}">
      <td class="algo">${ALGO_SHORT[r.algorithm]}</td>
      <td>${r.cost_total.toFixed(0)}</td>
      <td>${r.cost_route.toFixed(0)}</td>
      <td>${r.cost_stops.toFixed(0)}</td>
      <td>${r.cost_walk.toFixed(0)}</td>
      <td>${r.runtime_ms.toFixed(0)}</td>
    </tr>`;
  }
  html += "</tbody></table>";
  wrap.innerHTML = html;
  // bind row clicks → switch SVG to that algorithm's result
  wrap.querySelectorAll("tr[data-algo]").forEach(tr => {
    tr.addEventListener("click", () => showResult(tr.dataset.algo));
  });
}

// ---------------- Wire up controls ----------------

// "生成城市" button: each click generates a fresh random seed internally.
$("#btn-regen").addEventListener("click", regenerate);
$$(".solver").forEach(btn => btn.addEventListener("click", () => solve(btn.dataset.algo)));

loadScene().catch(e => status("載入失敗: " + e.message, "error"));
