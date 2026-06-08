"""
Algorithm B — Two-phase baseline.

Phase 1: Single shortest path A → B (Dijkstra, edge weight = length × terrain),
         strictly avoiding forbidden edges.
Phase 2: Treat path nodes as candidate stops. Solve p-median (at most K) over
         them, with each passenger's walking cost being graph distance from
         passenger's snapped node to the stop in the FULL graph (pedestrians
         can route around barriers, bus cannot).

Strength: very fast — one global Dijkstra + small UFLP.
Weakness: the path itself is chosen without considering passengers, so when
          passengers cluster far from the geodesic A-B route, the result is
          suboptimal. Used as a baseline to quantify how much detour helps.
"""
from __future__ import annotations
import time
import networkx as nx

from ..models import ProblemInstance, SolveResult, SolveParams, StopInfo, Assignment
from .common import compute_walk_distances, solve_pmedian, path_metrics, bus_subgraph


def solve_two_phase(inst: ProblemInstance) -> SolveResult:
    t0 = time.perf_counter()
    G = inst.G
    p = inst.params

    Gbus = bus_subgraph(G)
    if inst.source not in Gbus or inst.sink not in Gbus:
        raise RuntimeError("source/sink not in bus-traversable subgraph")
    if not nx.has_path(Gbus, inst.source, inst.sink):
        raise RuntimeError("no bus-feasible path between A and B (forbidden zones cut graph)")

    # Phase 1: shortest A→B path
    path = nx.dijkstra_path(Gbus, inst.source, inst.sink, weight="weight")
    path_len, path_weight = path_metrics(Gbus, path)
    cost_route = p.alpha_route * path_weight

    # Phase 2: p-median on path nodes
    candidate_stops = [n for n in path if n != inst.source and n != inst.sink]
    # walk distances in FULL graph (pedestrian)
    unique_p_nodes = list(set(inst.passenger_nodes))
    walk = compute_walk_distances(G, unique_p_nodes, candidate_stops + [inst.source, inst.sink])

    terrain_at = {n: G.nodes[n]["terrain"] for n in G.nodes()}
    pmed = solve_pmedian(
        passenger_ids=inst.passenger_ids,
        passenger_nodes=inst.passenger_nodes,
        passenger_demands=inst.passenger_demands,
        candidate_stops=candidate_stops,
        walk_dist=walk,
        fixed_stops=[inst.source, inst.sink],
        max_stops=p.max_stops,
        stop_fixed_cost=p.stop_fixed_cost,
        terrain_at_node=terrain_at,
        beta_walk=p.beta_walk,
    )

    cost_walk = p.beta_walk * pmed.total_walk
    cost_stops = pmed.total_stop_fixed
    total = cost_route + cost_stops + cost_walk

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

    return SolveResult(
        algorithm="two_phase",
        path_nodes=path,
        path_length=path_len,
        stops=stops_info,
        assignments=assignments,
        cost_route=cost_route,
        cost_stops=cost_stops,
        cost_walk=cost_walk,
        cost_total=total,
        runtime_ms=(time.perf_counter() - t0) * 1000,
        notes="Phase 1 Dijkstra → Phase 2 greedy p-median + 1-swap local search.",
    )
