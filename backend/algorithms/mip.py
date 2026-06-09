"""
Algorithm A — MIP (CP-SAT) for the joint route + stops + assignment problem.

Decision variables (all binary):
  x_{u→v}   : bus traverses directed arc u→v
              (each undirected bus-edge expanded into two arcs; ≤ 1 of them on)
  s_v       : node v hosts a stop  (s_A = s_B = 1 are forced)
  z_{p,v}   : passenger p walks to stop at node v

Constraints:
  Flow conservation:  A is a source of 1 unit, B is a sink, others conserve.
                      This forces the x-arcs to form a simple A→B walk.
  Visited ⇒ stop:     s_v ≤ Σ inflow(v)   for v ∉ {A,B}
  Stop budget:        Σ_{v∉{A,B}} s_v  ≤ K
  Assignment:         Σ_v z_{p,v} = 1 for every passenger;  z_{p,v} ≤ s_v

Objective (minimize, costs scaled to integers for CP-SAT):
    α · Σ w_uv · x_{u→v}                       (route)
  + Σ_{v ∉ {A,B}} (stop_fixed · terrain_v) · s_v   (stop building)
  + β · Σ_{p,v} walk(p→v) · z_{p,v}             (passenger walking)

Use:
  - Optimal benchmark for small/medium instances (≤ ~200 nodes, ~50 passengers).
  - Reports the optimality gap vs. heuristics from algorithms B and C.

Limitations:
  - Variable count ≈ 2·|E| + |V| + |P|·|V|. CP-SAT handles a few tens of
    thousands of booleans within a 20–60s budget; bigger needs LP-based MIP.
"""
from __future__ import annotations
import time
import networkx as nx
from ortools.sat.python import cp_model

from ..models import ProblemInstance, SolveResult, StopInfo, Assignment
from .common import compute_walk_distances, bus_subgraph


SCALE = 100  # cost scaling factor — keep numerics in reasonable integer range


def solve_mip(inst: ProblemInstance) -> SolveResult:
    t0 = time.perf_counter()
    G = inst.G
    p = inst.params
    Gbus = bus_subgraph(G)

    if not nx.has_path(Gbus, inst.source, inst.sink):
        raise RuntimeError("no bus-feasible path between A and B")

    A, B = inst.source, inst.sink

    # ---- Phase 2: KSP corridor restriction ----
    # Run KSP first (also used for warm-start hints below). The corridor is
    # the union of nodes within `corridor_hops_mip` BFS hops of any KSP
    # candidate path. Restricting the MIP to this subgraph cuts arc count
    # ~50% on default scenes without losing the optimum in practice.
    from itertools import islice
    from .ksp import solve_ksp
    try:
        ksp_hint = solve_ksp(inst)
    except Exception:
        ksp_hint = None

    corridor_nodes: set[int] | None = None
    if ksp_hint is not None:
        # Re-derive corridor from the Yen path family (not just the winning one)
        try:
            yen_paths = list(islice(
                nx.shortest_simple_paths(Gbus, A, B, weight="weight"),
                p.k_paths,
            ))
        except Exception:
            yen_paths = [ksp_hint.path_nodes]
        # 4-hop BFS neighborhood around the Yen path family. Slightly wider
        # than KSP's own corridor_hops so MIP almost always has access to
        # any node the true optimum needs while still cutting node count.
        seed_nodes = set()
        for path in yen_paths:
            seed_nodes.update(path)
        corridor_nodes = set(seed_nodes)
        frontier = set(seed_nodes)
        for _ in range(5):
            nxt = set()
            for u in frontier:
                if u in Gbus:
                    for v in Gbus.neighbors(u):
                        if v not in corridor_nodes:
                            nxt.add(v)
            corridor_nodes |= nxt
            frontier = nxt
        corridor_nodes.add(A)
        corridor_nodes.add(B)

    if corridor_nodes is None or len(corridor_nodes) >= Gbus.number_of_nodes() * 0.95:
        # corridor not useful (or KSP failed); use full bus graph
        nodes = list(Gbus.nodes())
        Hbus = Gbus
    else:
        nodes = list(corridor_nodes)
        Hbus = Gbus.subgraph(corridor_nodes).copy()
        if not nx.has_path(Hbus, A, B):
            # corridor disconnects A↔B; fall back to full
            nodes = list(Gbus.nodes())
            Hbus = Gbus

    model = cp_model.CpModel()

    # ---- arc variables ----
    x: dict[tuple[int, int], cp_model.IntVar] = {}
    arc_weight: dict[tuple[int, int], float] = {}
    for u, v, d in Hbus.edges(data=True):
        x[(u, v)] = model.NewBoolVar(f"x_{u}_{v}")
        x[(v, u)] = model.NewBoolVar(f"x_{v}_{u}")
        arc_weight[(u, v)] = d["weight"]
        arc_weight[(v, u)] = d["weight"]
        # at most one direction per undirected edge
        model.Add(x[(u, v)] + x[(v, u)] <= 1)

    in_arcs: dict[int, list[cp_model.IntVar]] = {n: [] for n in nodes}
    out_arcs: dict[int, list[cp_model.IntVar]] = {n: [] for n in nodes}
    for (u, v), var in x.items():
        out_arcs[u].append(var)
        in_arcs[v].append(var)

    # ---- flow conservation (tightened: no flow into A, none out of B) ----
    for n in nodes:
        out_sum = sum(out_arcs[n]) if out_arcs[n] else 0
        in_sum = sum(in_arcs[n]) if in_arcs[n] else 0
        if n == A:
            model.Add(out_sum == 1)
            if in_arcs[n]:
                model.Add(in_sum == 0)
        elif n == B:
            model.Add(in_sum == 1)
            if out_arcs[n]:
                model.Add(out_sum == 0)
        else:
            model.Add(out_sum - in_sum == 0)
            if in_arcs[n]:
                model.Add(in_sum <= 1)

    # ---- MTZ subtour elimination ----
    # Potential variable u_v: strictly increases along the path.
    # u_A = 0, u_v ∈ [1, N] for v ≠ A.  For each arc (a→b) with b ≠ A:
    #     x_{a→b} = 1  ⇒  u_b ≥ u_a + 1
    # implemented via big-M:  u_b ≥ u_a + 1 - N · (1 − x_{a→b})
    # This forbids any cycle, including those not connected to A→B.
    N = len(nodes)
    u_var: dict[int, cp_model.IntVar] = {}
    for n in nodes:
        if n == A:
            u_var[n] = model.NewIntVar(0, 0, f"u_{n}")
        else:
            u_var[n] = model.NewIntVar(1, N, f"u_{n}")
    for (a, b), var in x.items():
        if b == A:
            continue  # impossible by flow constraints; skip
        model.Add(u_var[b] >= u_var[a] + 1 - N * (1 - var))

    # ---- stop variables ----
    s = {n: model.NewBoolVar(f"s_{n}") for n in nodes}
    model.Add(s[A] == 1)
    model.Add(s[B] == 1)

    for n in nodes:
        if n in (A, B):
            continue
        if in_arcs[n]:
            model.Add(s[n] <= sum(in_arcs[n]))
        else:
            model.Add(s[n] == 0)

    intermediates = [s[n] for n in nodes if n not in (A, B)]
    if intermediates:
        model.Add(sum(intermediates) <= p.max_stops)

    # ---- passenger assignment ----
    # restrict candidates per passenger to e.g. their K-nearest stops (keeps z small)
    unique_p_nodes = list(set(inst.passenger_nodes))
    walk = compute_walk_distances(G, unique_p_nodes, nodes)
    demand_for_pid = dict(zip(inst.passenger_ids, inst.passenger_demands))

    # For each passenger we only need z to plausible candidates — to keep the MIP
    # tractable, restrict to the M-nearest reachable nodes from the passenger.
    M = 20  # candidate stops per passenger (Phase 1: was 40)
    z: dict[tuple[int, int], cp_model.IntVar] = {}
    walk_for_pid: dict[int, dict[int, float]] = {}
    for pid, pn in zip(inst.passenger_ids, inst.passenger_nodes):
        if pn not in walk:
            continue
        # M-nearest stops by walking distance + always include A and B so a
        # passenger always has a feasible assignment (they're free stops).
        nearest_items = sorted(walk[pn].items(), key=lambda kv: kv[1])[:M]
        candidate_nodes = {n for n, _ in nearest_items}
        if A in walk[pn]:
            candidate_nodes.add(A)
        if B in walk[pn]:
            candidate_nodes.add(B)
        walk_for_pid[pid] = {n: walk[pn][n] for n in candidate_nodes}
        for n in walk_for_pid[pid]:
            z[(pid, n)] = model.NewBoolVar(f"z_{pid}_{n}")

    # each passenger assigned to exactly one stop (out of its M candidates)
    for pid in inst.passenger_ids:
        cand = [z[(pid, n)] for n in walk_for_pid.get(pid, {}).keys()]
        if not cand:
            continue
        model.Add(sum(cand) == 1)

    # z_p,n ≤ s_n
    for (pid, n), zvar in z.items():
        model.Add(zvar <= s[n])

    # ---- objective ----
    terms = []
    # route cost
    for arc, var in x.items():
        c = int(round(arc_weight[arc] * p.alpha_route * SCALE))
        terms.append(c * var)
    # stop fixed cost (intermediates only — A,B free)
    terrain = {n: G.nodes[n]["terrain"] for n in nodes}
    for n in nodes:
        if n in (A, B):
            continue
        c = int(round(p.stop_fixed_cost * terrain[n] * SCALE))
        if c:
            terms.append(c * s[n])
    # walk cost — weighted by passenger demand (population at the point)
    for (pid, n), zvar in z.items():
        d = walk_for_pid[pid][n]
        demand = demand_for_pid.get(pid, 1)
        c = int(round(d * demand * p.beta_walk * SCALE))
        if c:
            terms.append(c * zvar)

    model.Minimize(sum(terms))

    # ---- KSP warm-start hints ----
    # Phase 1: seed the search with KSP's solution so CP-SAT starts with a
    # tight upper bound. Skip arcs/nodes not in the corridor subgraph.
    if ksp_hint is not None:
        ksp_arcs = set()
        for i in range(len(ksp_hint.path_nodes) - 1):
            u, v = ksp_hint.path_nodes[i], ksp_hint.path_nodes[i + 1]
            ksp_arcs.add((u, v))
        for arc, var in x.items():
            model.AddHint(var, 1 if arc in ksp_arcs else 0)
        hint_stops = {sinfo.node_id for sinfo in ksp_hint.stops}
        for n in nodes:
            if n in (A, B):
                continue
            model.AddHint(s[n], 1 if n in hint_stops else 0)
        hint_assign = {a.passenger_id: a.stop_node_id for a in ksp_hint.assignments}
        for (pid, n), zvar in z.items():
            model.AddHint(zvar, 1 if hint_assign.get(pid) == n else 0)
        path_pos: dict[int, int] = {}
        for i, n in enumerate(ksp_hint.path_nodes):
            path_pos.setdefault(n, i)
        for n, uvar in u_var.items():
            if n == A:
                continue
            if n in path_pos:
                model.AddHint(uvar, min(path_pos[n], N))

    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = p.mip_time_limit_s
    solver.parameters.num_search_workers = 16          # Phase 2a: was 8 (Mac Studio M3 Ultra)
    solver.parameters.linearization_level = 2          # Phase 1: tighter LP cuts
    status = solver.Solve(model)

    if status not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        raise RuntimeError(f"MIP found no feasible solution (status={solver.StatusName(status)})")

    # ---- reconstruct path ----
    used_arcs = {(u, v) for (u, v), var in x.items() if solver.Value(var) == 1}
    path = [A]
    cur = A
    safety = 0
    while cur != B and safety < len(nodes) * 2:
        safety += 1
        nxt = None
        for (u, v) in used_arcs:
            if u == cur:
                nxt = v
                break
        if nxt is None:
            break
        path.append(nxt)
        used_arcs.discard((cur, nxt))
        cur = nxt

    # ---- reconstruct stops and assignments ----
    chosen_stops = [n for n in nodes if n not in (A, B) and solver.Value(s[n]) == 1]

    assignments = []
    walk_total = 0.0
    walk_per_p: dict[int, float] = {}
    assigned_stop: dict[int, int] = {}
    for pid, pn in zip(inst.passenger_ids, inst.passenger_nodes):
        for n in walk_for_pid.get(pid, {}):
            if solver.Value(z.get((pid, n), 0)) == 1:
                d = walk_for_pid[pid][n]
                demand = demand_for_pid.get(pid, 1)
                assignments.append(Assignment(passenger_id=pid, stop_node_id=n, walk_distance=d))
                walk_total += d * demand
                walk_per_p[pid] = d
                assigned_stop[pid] = n
                break

    # path length
    path_len = sum(G[path[i]][path[i+1]]["length"] for i in range(len(path)-1))
    path_weight = sum(G[path[i]][path[i+1]]["weight"] for i in range(len(path)-1))

    cost_route = p.alpha_route * path_weight
    cost_stops = sum(p.stop_fixed_cost * terrain[n] for n in chosen_stops)
    cost_walk = p.beta_walk * walk_total

    stops_info = [
        StopInfo(
            node_id=s_id,
            x=G.nodes[s_id]["x"],
            y=G.nodes[s_id]["y"],
            passengers=[pid for pid, sid in assigned_stop.items() if sid == s_id],
        )
        for s_id in chosen_stops
    ]

    # optimality gap
    obj = solver.ObjectiveValue()
    bound = solver.BestObjectiveBound()
    gap = (obj - bound) / max(1.0, abs(obj))

    status_name = solver.StatusName(status)
    notes = (
        f"CP-SAT {status_name}; "
        f"obj={obj/SCALE:.2f}, bound={bound/SCALE:.2f}, gap={gap*100:.2f}%; "
        f"M={M} candidates per passenger."
    )

    return SolveResult(
        algorithm="mip",
        path_nodes=path,
        path_length=path_len,
        stops=stops_info,
        assignments=assignments,
        cost_route=cost_route,
        cost_stops=cost_stops,
        cost_walk=cost_walk,
        cost_total=cost_route + cost_stops + cost_walk,
        runtime_ms=(time.perf_counter() - t0) * 1000,
        optimality_gap=gap,
        notes=notes,
    )
