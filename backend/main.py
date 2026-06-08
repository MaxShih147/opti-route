"""
FastAPI backend for the bus-route-and-stops optimizer.

Endpoints:
  POST /api/generate   -- (re)generate a random city scene with given params
  POST /api/solve      -- run a chosen algorithm on the current scene
  POST /api/edit       -- mutate the current scene (move/add/delete passenger,
                          toggle forbidden edge, set A/B)
  GET  /api/scene      -- get the current scene
  GET  /                -- serve the frontend

State:
  A single in-memory scene (this is a single-user demo). Re-generating wipes it.
"""
from __future__ import annotations
from pathlib import Path
from typing import Optional
import networkx as nx
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from .graph_gen import GenParams, generate_city, graph_to_scene
from .models import ProblemInstance, SolveParams, SolveResult
from .algorithms import solve_two_phase, solve_ksp, solve_mip

APP_ROOT = Path(__file__).resolve().parent.parent
FRONTEND_DIR = APP_ROOT / "frontend"


app = FastAPI(title="Bus Route Optimizer")


# ---- in-memory state ----

class SceneState:
    def __init__(self):
        self.G: Optional[nx.Graph] = None
        self.scene_dict: Optional[dict] = None
        self.gen_params: Optional[GenParams] = None

    def regenerate(self, p: GenParams):
        G, _, passengers, source, sink, forb = generate_city(p)
        self.G = G
        self.gen_params = p
        self.scene_dict = graph_to_scene(G, passengers, source, sink, forb, p)

    def to_problem(self, params: SolveParams) -> ProblemInstance:
        assert self.G is not None
        passengers = self.scene_dict["passengers"]
        return ProblemInstance(
            G=self.G,
            source=self.scene_dict["source"],
            sink=self.scene_dict["sink"],
            passenger_ids=[p["id"] for p in passengers],
            passenger_nodes=[p["node_id"] for p in passengers],
            candidate_stops=list(self.G.nodes()),
            terrain={n: self.G.nodes[n]["terrain"] for n in self.G.nodes()},
            params=params,
        )


STATE = SceneState()
STATE.regenerate(GenParams())  # seed with a default scene


# ---- API models ----

class GenerateRequest(BaseModel):
    rows: int = 12
    cols: int = 16
    edge_drop_rate: float = 0.12
    arterial_count: int = 4
    n_passengers: int = 30
    forbidden_zones: int = 2
    seed: int = 42


class EditRequest(BaseModel):
    action: str  # 'set_source' | 'set_sink' | 'add_passenger' | 'move_passenger' | 'delete_passenger'
    node_id: Optional[int] = None
    passenger_id: Optional[int] = None
    x: Optional[float] = None
    y: Optional[float] = None


# ---- routes ----

@app.post("/api/generate")
def api_generate(req: GenerateRequest):
    p = GenParams(
        rows=req.rows,
        cols=req.cols,
        edge_drop_rate=req.edge_drop_rate,
        arterial_count=req.arterial_count,
        n_passengers=req.n_passengers,
        forbidden_zones=req.forbidden_zones,
        seed=req.seed,
    )
    STATE.regenerate(p)
    return STATE.scene_dict


@app.get("/api/scene")
def api_scene():
    if STATE.scene_dict is None:
        raise HTTPException(404, "no scene")
    return STATE.scene_dict


@app.post("/api/solve")
def api_solve(params: SolveParams) -> SolveResult:
    if STATE.G is None:
        raise HTTPException(400, "no scene generated yet")
    inst = STATE.to_problem(params)
    if params.algorithm == "two_phase":
        return solve_two_phase(inst)
    if params.algorithm == "ksp":
        return solve_ksp(inst)
    if params.algorithm == "mip":
        return solve_mip(inst)
    raise HTTPException(400, f"unknown algorithm: {params.algorithm}")


@app.post("/api/edit")
def api_edit(req: EditRequest):
    if STATE.scene_dict is None or STATE.G is None:
        raise HTTPException(400, "no scene generated")
    G = STATE.G

    def snap_to_nearest(x: float, y: float) -> int:
        return min(G.nodes(), key=lambda n: (G.nodes[n]["x"] - x) ** 2 + (G.nodes[n]["y"] - y) ** 2)

    if req.action == "set_source":
        if req.node_id is None:
            if req.x is None or req.y is None:
                raise HTTPException(400, "need node_id or (x,y)")
            req.node_id = snap_to_nearest(req.x, req.y)
        STATE.scene_dict["source"] = req.node_id
    elif req.action == "set_sink":
        if req.node_id is None:
            if req.x is None or req.y is None:
                raise HTTPException(400, "need node_id or (x,y)")
            req.node_id = snap_to_nearest(req.x, req.y)
        STATE.scene_dict["sink"] = req.node_id
    elif req.action == "add_passenger":
        if req.x is None or req.y is None:
            raise HTTPException(400, "need (x,y)")
        nn = snap_to_nearest(req.x, req.y)
        new_id = (max((p["id"] for p in STATE.scene_dict["passengers"]), default=-1) + 1)
        STATE.scene_dict["passengers"].append(
            {"id": new_id, "x": req.x, "y": req.y, "node_id": nn}
        )
    elif req.action == "move_passenger":
        if req.passenger_id is None or req.x is None or req.y is None:
            raise HTTPException(400, "need passenger_id and (x,y)")
        for p in STATE.scene_dict["passengers"]:
            if p["id"] == req.passenger_id:
                p["x"] = req.x
                p["y"] = req.y
                p["node_id"] = snap_to_nearest(req.x, req.y)
                break
    elif req.action == "delete_passenger":
        if req.passenger_id is None:
            raise HTTPException(400, "need passenger_id")
        STATE.scene_dict["passengers"] = [
            p for p in STATE.scene_dict["passengers"] if p["id"] != req.passenger_id
        ]
    else:
        raise HTTPException(400, f"unknown action: {req.action}")
    return STATE.scene_dict


# ---- static frontend ----

if FRONTEND_DIR.exists():
    app.mount("/static", StaticFiles(directory=FRONTEND_DIR), name="static")

    @app.get("/")
    def index():
        return FileResponse(FRONTEND_DIR / "index.html")
