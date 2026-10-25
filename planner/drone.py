"""Drone delivery routing.

1. grid_path: 8-connected A* on a city grid, around no-fly zones.
2. smooth: drop waypoints wherever a straight segment stays clear.
3. plan_fleet: OR-Tools vehicle routing over the smoothed path lengths, with
   payload capacity, battery range, delivery time windows, and dropped
   packages reported instead of failing.
4. conflicts: flag two drones closer than a separation distance at the same time.

Coordinates are grid cells (x, y); each cell is CELL_M meters. Cell (x, y)
covers [x-0.5, x+0.5] x [y-0.5, y+0.5].
"""
import bisect
import heapq
import math
from typing import NamedTuple

from ortools.constraint_solver import pywrapcp, routing_enums_pb2

CELL_M = 50
DIAG = math.sqrt(2)
DROP_PENALTY = 10_000_000  # meters; larger than any route, so the solver drops only when it must


class Map(NamedTuple):
    width: int
    height: int
    blocked: frozenset  # cells inside a no-fly zone


class Package(NamedTuple):
    id: str
    cell: tuple
    grams: int
    window: tuple  # (earliest, latest) delivery time, seconds after dispatch


class Route(NamedTuple):
    drone: int
    stops: list         # [(package id, delivery time s)]
    distance_m: int
    grams: int
    timeline: list      # [(t s, x, y)] for the conflict check and plotting


def make_map(width, height, zones):
    """zones: ("rect", x0, y0, x1, y1) or ("circle", cx, cy, r), in cells."""
    blocked = set()
    for z in zones:
        if z[0] == "rect":
            _, x0, y0, x1, y1 = z
            blocked |= {(x, y) for x in range(x0, x1 + 1) for y in range(y0, y1 + 1)}
        else:
            _, cx, cy, r = z
            blocked |= {(x, y) for x in range(cx - r, cx + r + 1) for y in range(cy - r, cy + r + 1)
                        if (x - cx) ** 2 + (y - cy) ** 2 <= r * r}
    return Map(width, height, frozenset((x, y) for x, y in blocked if 0 <= x < width and 0 <= y < height))


def free(m, c):
    return 0 <= c[0] < m.width and 0 <= c[1] < m.height and c not in m.blocked


def octile(a, b):
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return max(dx, dy) + (DIAG - 1) * min(dx, dy)


def grid_path(m, start, goal):
    """Shortest 8-connected path of cells from start to goal.

    The heuristic is octile distance: the exact cost on an empty grid, so
    no-fly zones can only make the true cost larger and it never overestimates.
    Diagonal steps may not cut the corner of a blocked cell.
    """
    if not (free(m, start) and free(m, goal)):
        raise ValueError(f"start {start} or goal {goal} is outside the map or in a no-fly zone")
    g, parent, closed = {start: 0.0}, {start: None}, set()
    frontier = [(octile(start, goal), start)]
    while frontier:
        _, cur = heapq.heappop(frontier)
        if cur == goal:
            path = []
            while cur:
                path.append(cur)
                cur = parent[cur]
            return path[::-1]
        if cur in closed:
            continue
        closed.add(cur)
        x, y = cur
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                nxt = (x + dx, y + dy)
                if (dx, dy) == (0, 0) or not free(m, nxt):
                    continue
                if dx and dy and not (free(m, (x + dx, y)) and free(m, (x, y + dy))):
                    continue
                ng = g[cur] + (DIAG if dx and dy else 1)
                if ng < g.get(nxt, math.inf):
                    g[nxt], parent[nxt] = ng, cur
                    heapq.heappush(frontier, (ng + octile(nxt, goal), nxt))
    raise ValueError(f"no path from {start} to {goal}")


def clear(m, a, b):
    """True if the straight segment between cell centers a and b crosses only free cells.

    Exact grid traversal (Amanatides-Woo): walk cell to cell, stepping in x or y
    depending on which cell boundary the segment reaches first. Boundaries sit
    at half-cells, so in parametric time the k-th x boundary is at
    (2k+1) / (2|dx|); comparing (2k+1)*|dy| with (2l+1)*|dx| keeps it in
    integers, with no float ties. Passing exactly through a corner needs both
    side cells free, the same no-corner-cutting rule as grid_path.
    """
    (x, y), (bx, by) = a, b
    ax, ay = abs(bx - x), abs(by - y)
    sx, sy = (bx > x) - (bx < x), (by > y) - (by < y)
    ix = iy = 0  # x and y boundaries crossed so far
    if not free(m, (x, y)):
        return False
    while (ix, iy) != (ax, ay):
        tx, ty = (2 * ix + 1) * ay, (2 * iy + 1) * ax  # next x / y boundary, cross-multiplied
        if iy == ay or (ix < ax and tx < ty):
            x, ix = x + sx, ix + 1
        elif ix == ax or ty < tx:
            y, iy = y + sy, iy + 1
        else:  # exactly through a corner
            if not (free(m, (x + sx, y)) and free(m, (x, y + sy))):
                return False
            x, y, ix, iy = x + sx, y + sy, ix + 1, iy + 1
        if not free(m, (x, y)):
            return False
    return True


def smooth(m, path):
    """Keep only the waypoints needed: from each kept point, jump to the farthest
    following point it can see. Each shortcut replaces a chain of segments, so by
    the triangle inequality the path never gets longer."""
    out, i = [path[0]], 0
    while i < len(path) - 1:
        j = i + 1
        while j + 1 < len(path) and clear(m, path[i], path[j + 1]):
            j += 1
        out.append(path[j])
        i = j
    return out


def length(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def plan_fleet(m, hub, packages, drones=4, payload_g=1200, range_m=12_000, speed_mps=29,
               service_s=60, horizon_s=3600, time_limit_s=2):
    """Assign packages to drones (one sortie each from the hub) and order the stops.

    Returns (routes, dropped package ids).
    ponytail: one dispatch wave, one sortie per drone. Multi-sortie needs a
    vehicle per (drone, trip) with each trip starting after the previous returns.
    """
    nodes = [hub] + [p.cell for p in packages]
    n = len(nodes)
    legs, dist = {}, [[0] * n for _ in range(n)]
    for i in range(n):
        for j in range(i + 1, n):
            pts = smooth(m, grid_path(m, nodes[i], nodes[j]))
            legs[i, j], legs[j, i] = pts, pts[::-1]
            dist[i][j] = dist[j][i] = round(length(pts) * CELL_M)

    manager = pywrapcp.RoutingIndexManager(n, drones, 0)
    routing = pywrapcp.RoutingModel(manager)
    node = manager.IndexToNode

    dist_cb = routing.RegisterTransitCallback(lambda a, b: dist[node(a)][node(b)])
    routing.SetArcCostEvaluatorOfAllVehicles(dist_cb)
    routing.AddDimension(dist_cb, 0, range_m, True, "Distance")  # battery range per sortie

    demand = [0] + [p.grams for p in packages]
    demand_cb = routing.RegisterUnaryTransitCallback(lambda a: demand[node(a)])
    routing.AddDimensionWithVehicleCapacity(demand_cb, 0, [payload_g] * drones, True, "Payload")

    def flight_s(a, b):
        i, j = node(a), node(b)
        return math.ceil(dist[i][j] / speed_mps) + (service_s if i else 0)

    time_cb = routing.RegisterTransitCallback(flight_s)
    routing.AddDimension(time_cb, horizon_s, horizon_s, False, "Time")  # slack = hovering until the window opens
    time = routing.GetDimensionOrDie("Time")
    for k, p in enumerate(packages, 1):
        idx = manager.NodeToIndex(k)
        time.CumulVar(idx).SetRange(*p.window)
        routing.AddDisjunction([idx], DROP_PENALTY)

    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    params.time_limit.FromSeconds(time_limit_s)
    sol = routing.SolveWithParameters(params)
    if sol is None:
        raise RuntimeError("OR-Tools found no solution")

    routes, served = [], set()
    for v in range(drones):
        idx, seq, times = routing.Start(v), [], []
        while not routing.IsEnd(idx):
            seq.append(node(idx))
            times.append(sol.Value(time.CumulVar(idx)))
            idx = sol.Value(routing.NextVar(idx))
        seq.append(0)
        if len(seq) == 2:
            continue  # drone stays home
        stops = [(packages[k - 1].id, t) for k, t in zip(seq[1:], times[1:])]
        served |= set(seq)

        # Timeline: fly each smoothed leg at constant speed, hover at a stop until
        # its delivery time, then spend service_s dropping the package.
        timeline, t = [(times[0], *hub)], times[0]
        for leg, (a, b) in enumerate(zip(seq, seq[1:])):
            pts = legs[a, b]
            for p, q in zip(pts, pts[1:]):
                t += math.dist(p, q) * CELL_M / speed_mps
                timeline.append((t, *q))
            if b:
                t = times[leg + 1] + service_s
                timeline.append((t, *nodes[b]))
        routes.append(Route(v, stops, sum(dist[a][b] for a, b in zip(seq, seq[1:])),
                            sum(demand[k] for k in seq), timeline))

    dropped = [p.id for k, p in enumerate(packages, 1) if k not in served]
    return routes, dropped


def position(timeline, t):
    """Interpolated (x, y) at time t, or None if not flying then."""
    times = [p[0] for p in timeline]
    if t < times[0] or t > times[-1]:
        return None
    i = bisect.bisect_right(times, t) - 1
    if i == len(times) - 1:
        return timeline[i][1:]
    (t0, x0, y0), (t1, x1, y1) = timeline[i], timeline[i + 1]
    f = (t - t0) / (t1 - t0) if t1 > t0 else 0
    return x0 + (x1 - x0) * f, y0 + (y1 - y0) * f


def conflicts(routes, hub, sep_m=150, hub_radius_m=300, dt=1.0):
    """Pairs of drones closer than sep_m at the same moment.

    Positions are sampled every dt seconds. Near the hub (within hub_radius_m)
    is skipped: takeoff and landing are sequenced by the hub's own pads.
    Returns [(drone a, drone b, first time s, closest distance m)].
    """
    if not routes:
        return []
    end = max(r.timeline[-1][0] for r in routes)
    near_hub = lambda p: math.dist(p, hub) * CELL_M < hub_radius_m
    found = {}
    t = 0.0
    while t <= end:
        pos = [(r.drone, position(r.timeline, t)) for r in routes]
        pos = [(d, p) for d, p in pos if p and not near_hub(p)]
        for i, (a, pa) in enumerate(pos):
            for b, pb in pos[i + 1:]:
                gap = math.dist(pa, pb) * CELL_M
                if gap < sep_m:
                    first, closest = found.get((a, b), (t, gap))
                    found[a, b] = (first, min(closest, gap))
        t += dt
    return [(a, b, round(first), round(gap)) for (a, b), (first, gap) in found.items()]


def deconflict(routes, packages, hub, step_s=15, max_delay_s=900, **sep):
    """Stagger departures so no two drones conflict.

    Drones are cleared one at a time. Each one's whole flight is delayed in
    step_s increments until it conflicts with no drone already cleared, as long
    as every delivery stays inside its window. Returns (routes, still conflicting
    drone ids).
    ponytail: greedy first-fit on whole-flight delays. Per-leg delays or
    altitude layers would resolve more conflicts with less waiting.
    """
    close = {p.id: p.window[1] for p in packages}
    cleared, stuck = [], []
    for r in routes:
        placed = None
        for delay in range(0, max_delay_s + 1, step_s):
            moved = r._replace(stops=[(i, t + delay) for i, t in r.stops],
                               timeline=[(t + delay, x, y) for t, x, y in r.timeline])
            if any(t > close[i] for i, t in moved.stops):
                break  # any later departure misses a window too
            if not conflicts(cleared + [moved], hub, **sep):
                placed = moved
                break
        if placed is None:
            placed = r
            stuck.append(r.drone)
        cleared.append(placed)
    return cleared, stuck
