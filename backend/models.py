"""Shared data models — Pydantic for API, plain dataclasses for solver internals."""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Optional
from pydantic import BaseModel, Field


# ---------- API request/response models ----------

class Node(BaseModel):
    id: int
    x: float
    y: float
    terrain: float = 1.0  # cost multiplier at this location


class Edge(BaseModel):
    u: int
    v: int
    length: float
    forbidden: bool = False


class Passenger(BaseModel):
    id: int
    x: float
    y: float
    node_id: int  # snapped to nearest road node


class Scene(BaseModel):
    nodes: list[Node]
    edges: list[Edge]
    passengers: list[Passenger]
    source: int  # node id for A
    sink: int    # node id for B
    width: float = 1000.0
    height: float = 1000.0


class SolveParams(BaseModel):
    algorithm: str = Field(..., description="'two_phase' | 'iterative' | 'mip'")
    max_stops: int = 5
    alpha_route: float = 1.0     # weight on route length
    beta_walk: float = 1.0       # weight on passenger walking
    stop_fixed_cost: float = 50.0
    max_iter: int = 10           # for iterative
    mip_time_limit_s: float = 20.0


class StopInfo(BaseModel):
    node_id: int
    x: float
    y: float
    passengers: list[int]


class Assignment(BaseModel):
    passenger_id: int
    stop_node_id: int
    walk_distance: float


class SolveResult(BaseModel):
    algorithm: str
    path_nodes: list[int]
    path_length: float
    stops: list[StopInfo]
    assignments: list[Assignment]
    cost_route: float
    cost_stops: float
    cost_walk: float
    cost_total: float
    runtime_ms: float
    iterations: Optional[int] = None
    optimality_gap: Optional[float] = None
    notes: str = ""


# ---------- Internal solver dataclasses ----------

@dataclass
class ProblemInstance:
    """Pre-processed problem instance for solvers."""
    G: object  # networkx.Graph
    source: int
    sink: int
    passenger_nodes: list[int]  # one node per passenger (snapped)
    passenger_ids: list[int]
    candidate_stops: list[int]  # nodes eligible to host a stop
    terrain: dict[int, float]   # node -> terrain cost factor
    params: SolveParams
    # APSP from each passenger node to every candidate stop (lazy filled)
    walk_dist: dict[tuple[int, int], float] = field(default_factory=dict)
