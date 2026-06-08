"""
City-like undirected graph generator.

Strategy:
  1. Start with an N x M lattice; jitter node positions for organic feel.
  2. Randomly drop a fraction of edges (variable block structure).
  3. Add a few long "arterial" edges (scale-free-ish, hub roads).
  4. Sprinkle terrain cost field via low-frequency value noise.
  5. Carve a few polygon forbidden zones (marks edges inside as forbidden).

The resulting graph keeps Manhattan-style readability while exhibiting
multiple A-B alternatives and detour opportunities — what algorithms need
to differentiate themselves.
"""
from __future__ import annotations
import math
import random
from dataclasses import dataclass

import networkx as nx


@dataclass
class GenParams:
    rows: int = 12
    cols: int = 16
    cell_size: float = 60.0           # spacing between grid nodes (pixels)
    jitter: float = 0.25              # 0..0.5 of cell_size
    edge_drop_rate: float = 0.12      # fraction of lattice edges removed
    arterial_count: int = 4           # extra long edges
    arterial_min_span: int = 5        # min Chebyshev distance for arterial
    terrain_octaves: int = 3
    terrain_amplitude: float = 0.8    # multiplicative range [1-amp, 1+amp]
    forbidden_zones: int = 2
    forbidden_radius_cells: tuple[int, int] = (1, 3)
    n_passengers: int = 30
    seed: int = 42


def _value_noise_2d(rows: int, cols: int, octaves: int, rng: random.Random) -> list[list[float]]:
    """Simple value noise — sum of low-res random grids upscaled bilinearly."""
    field = [[0.0] * cols for _ in range(rows)]
    amp = 1.0
    total_amp = 0.0
    for o in range(octaves):
        res_r = max(2, rows // (2 ** (octaves - o)))
        res_c = max(2, cols // (2 ** (octaves - o)))
        coarse = [[rng.random() for _ in range(res_c)] for _ in range(res_r)]
        for r in range(rows):
            for c in range(cols):
                fr = r * (res_r - 1) / max(1, rows - 1)
                fc = c * (res_c - 1) / max(1, cols - 1)
                r0, c0 = int(fr), int(fc)
                r1, c1 = min(r0 + 1, res_r - 1), min(c0 + 1, res_c - 1)
                dr, dc = fr - r0, fc - c0
                v = (
                    coarse[r0][c0] * (1 - dr) * (1 - dc)
                    + coarse[r1][c0] * dr * (1 - dc)
                    + coarse[r0][c1] * (1 - dr) * dc
                    + coarse[r1][c1] * dr * dc
                )
                field[r][c] += amp * v
        total_amp += amp
        amp *= 0.5
    return [[v / total_amp for v in row] for row in field]


def _point_in_disk(px: float, py: float, cx: float, cy: float, r: float) -> bool:
    return (px - cx) ** 2 + (py - cy) ** 2 <= r * r


def _amoeba_points(cx: float, cy: float, base_r: float, rng: random.Random,
                   n: int = 28) -> list[list[float]]:
    """
    Smooth irregular blob (amoeba-like) sampled at n equally-spaced angles.
    Sums a few low-harmonic sines to deform a circle gently in/out.
    """
    harmonics = [
        (rng.uniform(0.10, 0.22), rng.uniform(0, 2 * math.pi), 2),
        (rng.uniform(0.06, 0.16), rng.uniform(0, 2 * math.pi), 3),
        (rng.uniform(0.04, 0.10), rng.uniform(0, 2 * math.pi), 5),
    ]
    pts = []
    for i in range(n):
        theta = 2 * math.pi * i / n
        r = base_r
        for amp, phase, k in harmonics:
            r *= 1 + amp * math.sin(k * theta + phase)
        pts.append([cx + r * math.cos(theta), cy + r * math.sin(theta)])
    return pts


def _point_in_polygon(px: float, py: float, poly: list[list[float]]) -> bool:
    inside = False
    n = len(poly)
    if n < 3:
        return False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > py) != (yj > py):
            x_cross = (xj - xi) * (py - yi) / (yj - yi) + xi
            if px < x_cross:
                inside = not inside
        j = i
    return inside


def generate_city(p: GenParams) -> tuple[nx.Graph, dict, list[dict], int, int, list[dict]]:
    """
    Returns:
        G:                networkx.Graph with node attrs (x, y, terrain) and edge attrs (length, forbidden)
        node_xy:          {node_id: (x, y)}
        passengers:       list of {id, x, y, node_id}
        source, sink:     node ids for A and B
        forbidden_zones:  list of {cx, cy, r} for frontend rendering
    """
    rng = random.Random(p.seed)
    R, C = p.rows, p.cols
    cs = p.cell_size

    terrain_field = _value_noise_2d(R, C, p.terrain_octaves, rng)

    G = nx.Graph()
    node_id_of = {}  # (r, c) -> id
    nid = 0
    for r in range(R):
        for c in range(C):
            jx = (rng.random() - 0.5) * 2 * p.jitter * cs
            jy = (rng.random() - 0.5) * 2 * p.jitter * cs
            x = c * cs + jx + cs
            y = r * cs + jy + cs
            terrain = 1.0 + (terrain_field[r][c] * 2 - 1) * p.terrain_amplitude
            G.add_node(nid, x=x, y=y, terrain=terrain, row=r, col=c)
            node_id_of[(r, c)] = nid
            nid += 1

    # forbidden zones (amoeba-shaped smooth blobs)
    forbidden_zones = []
    for _ in range(p.forbidden_zones):
        rc = rng.randint(2, max(2, R - 3))
        cc = rng.randint(2, max(2, C - 3))
        rad_cells = rng.randint(*p.forbidden_radius_cells)
        cx = cc * cs + cs
        cy = rc * cs + cs
        radius = rad_cells * cs
        points = _amoeba_points(cx, cy, radius, rng)
        forbidden_zones.append({
            "cx": cx, "cy": cy, "r": radius,
            "points": points,
        })

    def _point_in_any_zone(px, py):
        for z in forbidden_zones:
            # quick bounding-circle reject, then exact polygon test
            if (px - z["cx"]) ** 2 + (py - z["cy"]) ** 2 > (z["r"] * 1.3) ** 2:
                continue
            if _point_in_polygon(px, py, z["points"]):
                return True
        return False

    def is_forbidden_edge(u, v):
        ux, uy = G.nodes[u]["x"], G.nodes[u]["y"]
        vx, vy = G.nodes[v]["x"], G.nodes[v]["y"]
        mx, my = (ux + vx) / 2, (uy + vy) / 2
        # sample midpoint + endpoints — covers both crossing and entering
        if _point_in_any_zone(mx, my):
            return True
        if _point_in_any_zone(ux, uy):
            return True
        if _point_in_any_zone(vx, vy):
            return True
        return False

    # lattice edges (4-connected) with random drop
    candidate_edges = []
    for r in range(R):
        for c in range(C):
            u = node_id_of[(r, c)]
            for dr, dc in [(0, 1), (1, 0)]:
                nr, nc = r + dr, c + dc
                if 0 <= nr < R and 0 <= nc < C:
                    v = node_id_of[(nr, nc)]
                    candidate_edges.append((u, v))

    for (u, v) in candidate_edges:
        if rng.random() < p.edge_drop_rate:
            continue
        ux, uy = G.nodes[u]["x"], G.nodes[u]["y"]
        vx, vy = G.nodes[v]["x"], G.nodes[v]["y"]
        length = math.hypot(vx - ux, vy - uy)
        # weight = length * average terrain of endpoints
        terrain_w = (G.nodes[u]["terrain"] + G.nodes[v]["terrain"]) / 2
        forb = is_forbidden_edge(u, v)
        G.add_edge(u, v, length=length, terrain=terrain_w, forbidden=forb,
                   weight=length * terrain_w)

    # arterial long edges — connect distant nodes (hub-like)
    attempts = 0
    added = 0
    while added < p.arterial_count and attempts < p.arterial_count * 20:
        attempts += 1
        u = rng.randrange(nid)
        v = rng.randrange(nid)
        if u == v:
            continue
        ur, uc = G.nodes[u]["row"], G.nodes[u]["col"]
        vr, vc = G.nodes[v]["row"], G.nodes[v]["col"]
        if max(abs(ur - vr), abs(uc - vc)) < p.arterial_min_span:
            continue
        if G.has_edge(u, v):
            continue
        ux, uy = G.nodes[u]["x"], G.nodes[u]["y"]
        vx, vy = G.nodes[v]["x"], G.nodes[v]["y"]
        length = math.hypot(vx - ux, vy - uy)
        terrain_w = (G.nodes[u]["terrain"] + G.nodes[v]["terrain"]) / 2 * 0.7  # arterials cheaper per length
        forb = is_forbidden_edge(u, v)
        G.add_edge(u, v, length=length, terrain=terrain_w, forbidden=forb,
                   weight=length * terrain_w, arterial=True)
        added += 1

    # Forbidden edges are KEPT in the graph but flagged.
    #   - Bus path algorithms must skip them (strict).
    #   - Pedestrians can use them (they walk around obstacles physically).
    # Verify full-graph connectivity, else keep largest component.
    if not nx.is_connected(G):
        largest = max(nx.connected_components(G), key=len)
        G = G.subgraph(largest).copy()

    # Build bus subgraph (forbidden edges removed) and find its largest
    # connected component — A and B MUST live in here, otherwise the bus has
    # no feasible route between them.
    H = nx.Graph()
    H.add_nodes_from(G.nodes(data=True))
    for u, v, d in G.edges(data=True):
        if not d.get("forbidden", False):
            H.add_edge(u, v, **d)

    bus_components = list(nx.connected_components(H))
    if not bus_components:
        raise RuntimeError("bus subgraph has no edges; loosen forbidden zone params")
    bus_main = max(bus_components, key=len)

    nodes_remaining = list(G.nodes())
    if len(nodes_remaining) < 4:
        raise RuntimeError("generated graph too small after pruning; loosen params")

    # A near top-left, B near bottom-right — but both confined to the bus's
    # largest connected component so a feasible path is guaranteed.
    def dist_to_corner(n, cx, cy):
        return (G.nodes[n]["x"] - cx) ** 2 + (G.nodes[n]["y"] - cy) ** 2

    bus_candidates = [n for n in nodes_remaining if n in bus_main]
    source = min(bus_candidates, key=lambda n: dist_to_corner(n, cs, cs))
    sink = min(bus_candidates, key=lambda n: dist_to_corner(n, (C - 1) * cs + cs, (R - 1) * cs + cs))

    # passengers: scatter around the map, then snap to nearest node
    passengers = []
    w = (C - 1) * cs + 2 * cs
    h = (R - 1) * cs + 2 * cs
    for i in range(p.n_passengers):
        # bias toward areas around but not on direct A-B line — makes problem interesting
        px = rng.uniform(cs * 0.5, w - cs * 0.5)
        py = rng.uniform(cs * 0.5, h - cs * 0.5)
        # avoid placing inside forbidden zones
        if _point_in_any_zone(px, py):
            # retry once, else accept anyway (the node snap will pull them out)
            px = rng.uniform(cs * 0.5, w - cs * 0.5)
            py = rng.uniform(cs * 0.5, h - cs * 0.5)
        # snap to nearest remaining node
        nn = min(nodes_remaining, key=lambda n: (G.nodes[n]["x"] - px) ** 2 + (G.nodes[n]["y"] - py) ** 2)
        passengers.append({"id": i, "x": px, "y": py, "node_id": nn})

    node_xy = {n: (G.nodes[n]["x"], G.nodes[n]["y"]) for n in G.nodes()}
    return G, node_xy, passengers, source, sink, forbidden_zones


def graph_to_scene(G: nx.Graph, passengers, source, sink, forbidden_zones, p: GenParams) -> dict:
    """Serialize to dict matching Scene model + extras for frontend."""
    nodes = [
        {"id": n, "x": G.nodes[n]["x"], "y": G.nodes[n]["y"], "terrain": G.nodes[n]["terrain"]}
        for n in G.nodes()
    ]
    edges = [
        {
            "u": u, "v": v,
            "length": d["length"],
            "weight": d["weight"],
            "arterial": d.get("arterial", False),
            "forbidden": d.get("forbidden", False),
        }
        for u, v, d in G.edges(data=True)
    ]
    return {
        "nodes": nodes,
        "edges": edges,
        "passengers": passengers,
        "source": source,
        "sink": sink,
        "forbidden_zones": forbidden_zones,
        "width": (p.cols - 1) * p.cell_size + 2 * p.cell_size,
        "height": (p.rows - 1) * p.cell_size + 2 * p.cell_size,
    }
