"""
Shared solver utilities:
  - distance computation (passengers → candidate stops via Dijkstra)
  - p-median / facility-location subproblem (greedy + 1-swap local search)
  - cost evaluation
"""
from __future__ import annotations
from dataclasses import dataclass
import networkx as nx


@dataclass
class PMedianResult:
    chosen_stops: list[int]              # node ids (excluding A, B which are implicit)
    assignment: dict[int, int]           # passenger_id -> stop_node_id (may be A or B)
    walk_cost_per_passenger: dict[int, float]
    total_walk: float
    total_stop_fixed: float


def compute_walk_distances(
    G: nx.Graph,
    from_nodes: list[int],
    to_nodes: list[int],
) -> dict[int, dict[int, float]]:
    """
    For each `from_node`, run single-source Dijkstra to all `to_nodes`.
    Returns {from_node: {to_node: dist}}.
    Uses edge attribute 'length' (un-weighted by terrain — pedestrian walks
    real-world distance, not weighted by terrain).
    """
    target_set = set(to_nodes)
    out: dict[int, dict[int, float]] = {}
    for u in from_nodes:
        dists = nx.single_source_dijkstra_path_length(G, u, weight="length")
        out[u] = {t: dists[t] for t in target_set if t in dists}
    return out


def solve_pmedian(
    passenger_ids: list[int],
    passenger_nodes: list[int],         # parallel to passenger_ids
    candidate_stops: list[int],         # nodes eligible to host a stop
    walk_dist: dict[int, dict[int, float]],  # walk_dist[passenger_node][stop_node]
    fixed_stops: list[int],             # always-on stops (A, B)
    max_stops: int,                     # max intermediate stops (excludes fixed)
    stop_fixed_cost: float,
    terrain_at_node: dict[int, float],  # for stop terrain surcharge
    beta_walk: float,
) -> PMedianResult:
    """
    Greedy add + 1-swap local search for the p-median / UFLP-with-budget subproblem.

    Cost being minimized here:
        β · Σ walk_dist(p, nearest_active_stop(p))
      + Σ (stop_fixed_cost · terrain_at(s))   over chosen intermediates
    """
    INF = float("inf")

    def stop_cost(s: int) -> float:
        return stop_fixed_cost * terrain_at_node.get(s, 1.0)

    def best_walk(pn: int, active: set[int]) -> tuple[float, int]:
        bd, bs = INF, -1
        d_pn = walk_dist.get(pn, {})
        for s in active:
            d = d_pn.get(s, INF)
            if d < bd:
                bd, bs = d, s
        return bd, bs

    def total_cost(active_intermediates: set[int]) -> tuple[float, dict[int, int], dict[int, float]]:
        active = set(fixed_stops) | active_intermediates
        walk_sum = 0.0
        assign: dict[int, int] = {}
        per_p: dict[int, float] = {}
        for pid, pn in zip(passenger_ids, passenger_nodes):
            d, s = best_walk(pn, active)
            assign[pid] = s
            per_p[pid] = d if d < INF else 0.0
            walk_sum += d if d < INF else 0.0
        stop_sum = sum(stop_cost(s) for s in active_intermediates)
        return beta_walk * walk_sum + stop_sum, assign, per_p

    # ----- Greedy add -----
    chosen: set[int] = set()
    cur_cost, cur_assign, cur_per_p = total_cost(chosen)

    candidate_pool = [c for c in candidate_stops if c not in fixed_stops]

    improved = True
    while improved and len(chosen) < max_stops:
        improved = False
        best_gain = 0.0
        best_pick = -1
        best_state = None
        for c in candidate_pool:
            if c in chosen:
                continue
            trial = chosen | {c}
            t_cost, t_assign, t_per_p = total_cost(trial)
            gain = cur_cost - t_cost
            if gain > best_gain + 1e-9:
                best_gain = gain
                best_pick = c
                best_state = (t_cost, t_assign, t_per_p)
        if best_pick != -1:
            chosen.add(best_pick)
            cur_cost, cur_assign, cur_per_p = best_state
            improved = True

    # ----- 1-swap local search -----
    moved = True
    safety = 0
    while moved and safety < 50:
        safety += 1
        moved = False
        chosen_list = list(chosen)
        for s_out in chosen_list:
            for s_in in candidate_pool:
                if s_in in chosen:
                    continue
                trial = (chosen - {s_out}) | {s_in}
                t_cost, t_assign, t_per_p = total_cost(trial)
                if t_cost + 1e-9 < cur_cost:
                    chosen = trial
                    cur_cost, cur_assign, cur_per_p = t_cost, t_assign, t_per_p
                    moved = True
                    break
            if moved:
                break

    # ----- 1-drop pass (in case greedy over-added) -----
    moved = True
    while moved:
        moved = False
        for s_out in list(chosen):
            trial = chosen - {s_out}
            t_cost, t_assign, t_per_p = total_cost(trial)
            if t_cost + 1e-9 < cur_cost:
                chosen = trial
                cur_cost, cur_assign, cur_per_p = t_cost, t_assign, t_per_p
                moved = True
                break

    total_walk = sum(cur_per_p.values())
    total_stop_fixed = sum(stop_cost(s) for s in chosen)

    return PMedianResult(
        chosen_stops=sorted(chosen),
        assignment=cur_assign,
        walk_cost_per_passenger=cur_per_p,
        total_walk=total_walk,
        total_stop_fixed=total_stop_fixed,
    )


def path_metrics(G: nx.Graph, path: list[int]) -> tuple[float, float]:
    """Return (total length, total weight) of a node-sequence path."""
    L = W = 0.0
    for i in range(len(path) - 1):
        d = G[path[i]][path[i + 1]]
        L += d["length"]
        W += d["weight"]
    return L, W


def bus_subgraph(G: nx.Graph) -> nx.Graph:
    """
    Return G with forbidden edges removed but all nodes preserved.
    Pedestrians use the full G (they can step around barriers).

    Built manually rather than via nx.edge_subgraph, which would drop any
    node whose every incident edge is forbidden — that produced spurious
    "source/sink not in graph" errors when forbidden zones isolated an
    endpoint in small scenes.
    """
    H = nx.Graph()
    H.add_nodes_from(G.nodes(data=True))
    for u, v, d in G.edges(data=True):
        if not d.get("forbidden", False):
            H.add_edge(u, v, **d)
    return H
