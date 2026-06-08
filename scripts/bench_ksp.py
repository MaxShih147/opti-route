"""
Sweep KSP parameters and measure gap to MIP optimum.

For each (scene_size, seed):
  1. Run MIP with a generous time limit -> reference cost
  2. Run two_phase as baseline
  3. Sweep KSP: k_paths × corridor_hops
  4. Report gap = (ksp_cost - mip_cost) / mip_cost
"""
from __future__ import annotations
import json
import time
from itertools import product

from backend.graph_gen import GenParams, generate_city
from backend.models import ProblemInstance, SolveParams
from backend.algorithms.two_phase import solve_two_phase
from backend.algorithms.ksp import solve_ksp
from backend.algorithms.mip import solve_mip


def build_instance(rows: int, cols: int, n_passengers: int, seed: int, params: SolveParams):
    p = GenParams(
        rows=rows, cols=cols, n_passengers=n_passengers, seed=seed,
        forbidden_zones=2, forbidden_radius_cells=(1, 3),
    )
    G, _, passengers, src, sink, _ = generate_city(p)
    inst = ProblemInstance(
        G=G, source=src, sink=sink,
        passenger_ids=[pp["id"] for pp in passengers],
        passenger_nodes=[pp["node_id"] for pp in passengers],
        candidate_stops=list(G.nodes()),
        terrain={n: G.nodes[n]["terrain"] for n in G.nodes()},
        params=params,
    )
    return inst, G.number_of_nodes(), G.number_of_edges()


def run(scene_label, rows, cols, n_pass, seeds, k_path_grid, corridor_grid, mip_time):
    print(f"\n========== Scene: {scene_label}  ({rows}×{cols}, {n_pass} pass) ==========")
    rows_out = []

    for seed in seeds:
        params = SolveParams(algorithm="mip", max_stops=4, alpha_route=1.0,
                             beta_walk=1.0, stop_fixed_cost=50.0,
                             mip_time_limit_s=mip_time)
        inst, n_nodes, n_edges = build_instance(rows, cols, n_pass, seed, params)
        print(f"\n-- seed={seed}  graph={n_nodes}n/{n_edges}e --")

        # MIP reference
        params.algorithm = "mip"
        try:
            mip_res = solve_mip(inst)
            print(f"  MIP        total={mip_res.cost_total:8.2f}  "
                  f"runtime={mip_res.runtime_ms:7.0f}ms  "
                  f"gap_internal={(mip_res.optimality_gap or 0)*100:.1f}%")
        except Exception as e:
            print(f"  MIP FAILED: {e}")
            continue
        ref = mip_res.cost_total

        # two_phase baseline
        params.algorithm = "two_phase"
        tp = solve_two_phase(inst)
        gap_tp = (tp.cost_total - ref) / ref * 100
        print(f"  two_phase  total={tp.cost_total:8.2f}  "
              f"runtime={tp.runtime_ms:6.0f}ms  gap={gap_tp:+6.1f}%")

        # KSP sweep
        for k_paths, corr in product(k_path_grid, corridor_grid):
            params.algorithm = "ksp"
            t0 = time.perf_counter()
            ksp_res = solve_ksp(inst, k_paths=k_paths, corridor_hops=corr)
            dt = (time.perf_counter() - t0) * 1000
            gap = (ksp_res.cost_total - ref) / ref * 100
            print(f"  KSP k={k_paths:2d} c={corr}    total={ksp_res.cost_total:8.2f}  "
                  f"runtime={dt:6.0f}ms  gap={gap:+6.1f}%")
            rows_out.append({
                "scene": scene_label, "seed": seed,
                "k_paths": k_paths, "corridor_hops": corr,
                "ksp_total": ksp_res.cost_total, "mip_ref": ref,
                "gap_pct": gap, "ksp_ms": dt,
            })

    return rows_out


if __name__ == "__main__":
    all_rows = []

    # --- Small scene: MIP can reach OPTIMAL ---
    all_rows += run(
        scene_label="small",
        rows=6, cols=8, n_pass=12,
        seeds=[3, 17, 42],
        k_path_grid=[3, 6, 10, 20],
        corridor_grid=[0, 1, 2, 3],
        mip_time=60.0,
    )

    # --- Medium scene: MIP probably hits time limit ---
    all_rows += run(
        scene_label="medium",
        rows=10, cols=12, n_pass=20,
        seeds=[3, 17, 42],
        k_path_grid=[3, 6, 10, 20],
        corridor_grid=[0, 1, 2, 3],
        mip_time=60.0,
    )

    # --- Default scene ---
    all_rows += run(
        scene_label="default",
        rows=12, cols=16, n_pass=30,
        seeds=[42, 7, 99],
        k_path_grid=[6, 10, 20],
        corridor_grid=[1, 2, 3],
        mip_time=90.0,
    )

    # save raw
    with open("/tmp/bench_ksp.json", "w") as f:
        json.dump(all_rows, f, indent=2)

    # summary by (scene, k, corridor) averaging across seeds
    print("\n\n========== SUMMARY (avg over seeds) ==========")
    from collections import defaultdict
    bucket = defaultdict(list)
    for r in all_rows:
        bucket[(r["scene"], r["k_paths"], r["corridor_hops"])].append(r)
    print(f"{'scene':10s} {'k':>3s} {'corr':>4s} {'avg_gap%':>10s} {'avg_ms':>8s}")
    for key in sorted(bucket.keys()):
        rs = bucket[key]
        avg_gap = sum(r["gap_pct"] for r in rs) / len(rs)
        avg_ms = sum(r["ksp_ms"] for r in rs) / len(rs)
        print(f"{key[0]:10s} {key[1]:3d} {key[2]:4d} {avg_gap:+10.2f} {avg_ms:8.1f}")

    print("\nraw data: /tmp/bench_ksp.json")
