"""
Algorithm C — K-shortest paths + p-median (the recommended workhorse).

Pipeline:
  1. Generate K candidate routes A→B using Yen's K-shortest-paths.
  2. For each route, build the candidate-stop set:
       path nodes  ∪  nodes within `corridor_radius_hops` of the path
     (the corridor expansion lets the algorithm pick stops *near* the route
      instead of strictly on it — this is what bends the effective service area
      toward passenger clusters).
  3. Solve the p-median subproblem for each path's candidate set.
  4. Total cost = α·route + Σstop_fixed + β·walk.  Return the best.

Why this design:
  - The pure two-phase baseline locks in the geodesic A→B path; if passengers
    cluster off-axis, walking cost dominates and there is nothing to do.
  - Yen's K paths give a small *diverse* portfolio of routes — including ones
    that detour through denser passenger regions, paid for by extra route cost.
  - p-median on each portfolio member picks the best stops conditional on
    that route. Pick the portfolio winner.
  - Far cheaper than full MIP, and easy to explain to a planner.
"""
from __future__ import annotations
import time
from itertools import islice
import networkx as nx

from ..models import ProblemInstance, SolveResult, StopInfo, Assignment
from .common import compute_walk_distances, solve_pmedian, path_metrics, bus_subgraph


def _k_shortest_paths(G: nx.Graph, s: int, t: int, k: int, weight: str = "weight") -> list[list[int]]:
    """Wrap networkx Yen's K-shortest simple paths (returns up to k)."""
    return list(islice(nx.shortest_simple_paths(G, s, t, weight=weight), k))


def _corridor_nodes(G: nx.Graph, path: list[int], hops: int) -> set[int]:
    """Nodes within `hops` BFS-hops of any path node (used as candidate stops)."""
    out: set[int] = set(path)
    frontier = set(path)
    for _ in range(hops):
        nxt = set()
        for u in frontier:
            for v in G.neighbors(u):
                if v not in out:
                    nxt.add(v)
        out |= nxt
        frontier = nxt
    return out


def solve_ksp(
    inst: ProblemInstance,
    k_paths: int | None = None,
    corridor_hops: int | None = None,
) -> SolveResult:
    """Use overrides if passed; otherwise read from inst.params."""
    t0 = time.perf_counter()
    G = inst.G
    p = inst.params
    if k_paths is None:
        k_paths = p.k_paths
    if corridor_hops is None:
        corridor_hops = p.corridor_hops

    Gbus = bus_subgraph(G)
    if not nx.has_path(Gbus, inst.source, inst.sink):
        raise RuntimeError("no bus-feasible path between A and B")

    # Generate candidate paths
    paths = _k_shortest_paths(Gbus, inst.source, inst.sink, k_paths, weight="weight")

    terrain_at = {n: G.nodes[n]["terrain"] for n in G.nodes()}
    unique_p_nodes = list(set(inst.passenger_nodes))

    best = None
    best_cost = float("inf")
    best_path_idx = -1
    diagnostics: list[dict] = []

    for idx, path in enumerate(paths):
        path_len, path_weight = path_metrics(Gbus, path)
        cost_route = p.alpha_route * path_weight

        # candidate stops = corridor around the path
        corridor = _corridor_nodes(G, path, corridor_hops)
        # exclude A/B (they're fixed); also prefer path nodes by listing first
        on_path = [n for n in path if n != inst.source and n != inst.sink]
        off_path = [n for n in corridor if n not in set(path)]
        candidates = on_path + off_path

        walk = compute_walk_distances(
            G,
            unique_p_nodes,
            candidates + [inst.source, inst.sink],
        )

        pmed = solve_pmedian(
            passenger_ids=inst.passenger_ids,
            passenger_nodes=inst.passenger_nodes,
            candidate_stops=candidates,
            walk_dist=walk,
            fixed_stops=[inst.source, inst.sink],
            max_stops=p.max_stops,
            stop_fixed_cost=p.stop_fixed_cost,
            terrain_at_node=terrain_at,
            beta_walk=p.beta_walk,
        )

        # If some chosen stops are OFF the original path, the bus must detour to
        # visit them. We do a cheap repair: shortest path A → s1 → ... → sk → B
        # where intermediates are ordered by projection along the original path.
        # This may inflate route cost — captured below.
        chosen = pmed.chosen_stops
        if chosen:
            # order stops by their position along the original geodesic
            path_pos = {n: i for i, n in enumerate(path)}
            # for off-path stops, snap to nearest on-path node for ordering
            def order_key(s):
                if s in path_pos:
                    return path_pos[s]
                # nearest on-path neighbor by BFS
                visited = {s: 0}
                q = [s]
                while q:
                    nxt = []
                    for u in q:
                        if u in path_pos:
                            return path_pos[u]
                        for w in G.neighbors(u):
                            if w not in visited:
                                visited[w] = visited[u] + 1
                                nxt.append(w)
                    q = nxt
                return 0
            ordered = sorted(chosen, key=order_key)
            waypoints = [inst.source] + ordered + [inst.sink]
            repaired = [waypoints[0]]
            ok = True
            repair_len = 0.0
            repair_weight = 0.0
            for a, b in zip(waypoints, waypoints[1:]):
                try:
                    seg = nx.dijkstra_path(Gbus, a, b, weight="weight")
                except nx.NetworkXNoPath:
                    ok = False
                    break
                # avoid duplicate node at junction
                repaired.extend(seg[1:])
                sl, sw = path_metrics(Gbus, seg)
                repair_len += sl
                repair_weight += sw
            if ok:
                # dedupe consecutive (already handled by extend[1:]) and use repaired
                final_path = repaired
                final_len = repair_len
                final_weight = repair_weight
            else:
                final_path = path
                final_len = path_len
                final_weight = path_weight
        else:
            final_path = path
            final_len = path_len
            final_weight = path_weight

        cost_route_final = p.alpha_route * final_weight
        cost_walk = p.beta_walk * pmed.total_walk
        cost_stops = pmed.total_stop_fixed
        total = cost_route_final + cost_stops + cost_walk

        diagnostics.append({
            "path_idx": idx,
            "geodesic_len": path_len,
            "final_len": final_len,
            "total_cost": total,
            "n_stops": len(pmed.chosen_stops),
        })

        if total < best_cost:
            best_cost = total
            best = (final_path, final_len, pmed, cost_route_final, cost_walk, cost_stops)
            best_path_idx = idx

    assert best is not None
    final_path, final_len, pmed, cost_route, cost_walk, cost_stops = best

    stops_info = [
        StopInfo(
            node_id=s,
            x=G.nodes[s]["x"],
            y=G.nodes[s]["y"],
            passengers=[pid for pid, sid in pmed.assignment.items() if sid == s],
        )
        for s in pmed.chosen_stops
    ]
    assignments = [
        Assignment(passenger_id=pid, stop_node_id=sid, walk_distance=pmed.walk_cost_per_passenger[pid])
        for pid, sid in pmed.assignment.items()
    ]

    notes = (
        f"Evaluated {len(paths)} candidate paths (corridor hops={corridor_hops}). "
        f"Winner: path #{best_path_idx}. "
        f"Per-path totals: " + ", ".join(f"#{d['path_idx']}={d['total_cost']:.1f}" for d in diagnostics)
    )

    return SolveResult(
        algorithm="ksp",
        path_nodes=final_path,
        path_length=final_len,
        stops=stops_info,
        assignments=assignments,
        cost_route=cost_route,
        cost_stops=cost_stops,
        cost_walk=cost_walk,
        cost_total=cost_route + cost_stops + cost_walk,
        runtime_ms=(time.perf_counter() - t0) * 1000,
        notes=notes,
    )
