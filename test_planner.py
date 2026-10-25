import pytest

from planner.ship import (COLS, DOCK, ROWS, TRUCK_MIN, State, balanced, heights,
                          load_manifest, moves, travel)

EMPTY = (0,) * COLS


def test_load_manifest():
    cols, base = load_manifest("JSONs/shipCase2.json")
    assert base[:4] == (2, 1, 1, 0)             # NAN cells at the bottom are obstacles
    assert cols[0] == ((40, cols[0][0][1]),)    # col 1 holds one 40-weight container
    assert sum(len(s) for s in cols) == 4


def test_travel_clears_stacks():
    h = [0] * COLS
    h[1] = 5                                     # tall stack between columns 0 and 2
    assert travel(0, 1, 2, 1, h) == 5 + 2 + 5    # up to level 6, across 2, down to 1
    assert travel(0, 1, 2, 1, [0] * COLS) == 2   # nothing in the way: straight across
    assert travel(DOCK, ROWS + 1, 0, 1, h) == 1 + ROWS


def test_moves_are_legal():
    full = ((9, "x"),) * ROWS
    state = State((((1, "a"), (2, "b")), ()) + (full,) * (COLS - 2), DOCK, unload=("b",))
    out = list(moves(state, EMPTY))
    assert all(m.name != "a" for m, _, _ in out)  # buried container never lifted
    assert {(m.src, m.dst) for m, _, _ in out if m.src == 0} == {(0, 1), (0, DOCK)}  # full cols refused
    m, nxt, cost = next(o for o in out if o[0].dst == DOCK)
    assert nxt.unload == () and nxt.cols[0] == ((1, "a"),)
    assert cost == travel(DOCK, ROWS + 1, 0, 2, heights(state.cols, EMPTY)) + 1 + (ROWS + 1 - 2) + TRUCK_MIN


def test_balanced():
    ship = lambda l, r: (((l, "l"),),) + ((),) * (COLS - 2) + (((r, "r"),),)
    assert balanced(ship(90, 100))
    assert not balanced(ship(89, 100))
    assert balanced(((),) * COLS)


# --- crane search -------------------------------------------------------------
from planner.crane import plan_balance, plan_transfer, sift_target


def replay(cols, base, plan, unload=(), load=()):
    """Re-apply a plan through moves() so every step is checked as legal."""
    state = State(cols, DOCK, tuple(sorted(unload)), tuple(sorted(load)))
    for move, cost in plan:
        state, step = next((n, c) for m, n, c in moves(state, base) if m == move)
        assert step == cost
    return state


@pytest.mark.parametrize("case", [1, 2, 3])
def test_astar_matches_ucs_balance(case):
    cols, base = load_manifest(f"JSONs/shipCase{case}.json")
    kind, plan, cost, expanded = plan_balance(cols, base)
    _, _, ucs_cost, ucs_expanded = plan_balance(cols, base, informed=False)
    assert kind == "balance" and cost == ucs_cost      # heuristic never overestimated
    assert expanded <= ucs_expanded                    # and it actually helped
    assert balanced(replay(cols, base, plan).cols)


def test_sift_when_unbalanceable():
    cols, base = load_manifest("JSONs/shipCase5.json")   # 96 of 113 total weight in one box
    kind, plan, _, _ = plan_balance(cols, base)
    final = replay(cols, base, plan)
    assert kind == "sift"
    assert tuple(tuple(w for w, _ in s) for s in final.cols) == sift_target(cols, base)


def test_astar_matches_ucs_transfer():
    cols, base = load_manifest("JSONs/shipCase3.json")
    unload, load = [cols[0][0][1]], [(42, "New")]   # unload the buried box in column 1
    plan, cost, _ = plan_transfer(cols, base, unload, load)
    _, ucs_cost, _ = plan_transfer(cols, base, unload, load, informed=False)
    assert cost == ucs_cost
    _, fast_cost, _ = plan_transfer(cols, base, unload, load, weight=2)
    assert cost <= fast_cost <= 2 * cost                # weighted A* stays within its bound
    final = replay(cols, base, plan, unload, load)
    assert (42, "New") in [c for s in final.cols for c in s]
    assert unload[0] not in [n for s in final.cols for _, n in s]


# --- drone routing ------------------------------------------------------------

from planner.drone import (Package, Route, clear, conflicts, deconflict, free, grid_path,
                           length, make_map, plan_fleet, smooth)

CITY = make_map(40, 40, [("rect", 10, 0, 12, 30), ("circle", 28, 20, 5)])  # wall with a gap at the top, and a stadium
HUB = (2, 2)


def test_grid_path_avoids_zones_and_smoothing_helps():
    path = grid_path(CITY, HUB, (20, 5))
    assert all(free(CITY, c) for c in path)
    assert max(y for _, y in path) > 30                     # had to go around the wall
    smoothed = smooth(CITY, path)
    assert len(smoothed) < len(path) and length(smoothed) <= length(path)
    for a, b in zip(smoothed, smoothed[1:]):                # independent fine check of every segment
        for i in range(1001):
            x, y = a[0] + (b[0] - a[0]) * i / 1000, a[1] + (b[1] - a[1]) * i / 1000
            assert free(CITY, (round(x), round(y)))


def test_fleet_respects_limits():
    pkgs = [Package("near", (5, 20), 500, (0, 3600)),
            Package("far", (35, 35), 400, (0, 3600)),
            Package("east", (20, 10), 600, (0, 3600)),
            Package("heavy", (6, 6), 5000, (0, 3600)),          # over payload: must be dropped
            Package("too_soon", (38, 2), 300, (0, 10))]         # can't get there in 10 s: dropped
    routes, dropped = plan_fleet(CITY, HUB, pkgs, drones=2, payload_g=1200, range_m=8000, time_limit_s=1)
    assert dropped == {"heavy": "over payload", "too_soon": "no drone or time slot this wave"}
    windows = {p.id: p.window for p in pkgs}
    for r in routes:
        assert r.grams <= 1200 and r.distance_m <= 8000
        assert all(windows[i][0] <= t <= windows[i][1] for i, t in r.stops)
    assert sorted(i for r in routes for i, _ in r.stops) == ["east", "far", "near"]


def test_conflicts():
    cross = lambda d, t0: [Route(d, [], 0, 0, [(t0, 0, 20), (t0 + 100, 40, 20)]),
                           Route(d + 1, [], 0, 0, [(t0, 20, 0), (t0 + 100, 20, 40)])]
    assert [c[:2] for c in conflicts(cross(0, 0), HUB)] == [(0, 1)]     # both reach (20, 20) at t=50
    a, b = cross(0, 0)[0], cross(0, 200)[1]                              # same paths, 200 s apart
    assert conflicts([a, b], HUB) == []


def test_clear_is_exact():
    m = make_map(5, 5, [("rect", 1, 1, 1, 1)])     # one blocked cell, spanning [0.5, 1.5]^2
    assert not clear(m, (0, 0), (2, 2))             # through the blocked cell's center
    assert not clear(m, (0, 1), (2, 1))
    assert clear(m, (0, 2), (2, 4))                 # passes the cell without touching it
    assert not clear(m, (2, 0), (0, 2))             # through the corners of the blocked cell
    assert clear(m, (0, 0), (4, 1))                 # slope 1/4 stays under the cell (y <= 0.5 for x <= 2)


def test_deconflict_staggers_departures():
    same = [Route(d, [(f"p{d}", 100)], 0, 0, [(0, 0, 20), (100, 40, 20)]) for d in range(3)]
    pkgs = [Package(f"p{d}", (40, 20), 0, (0, 3600)) for d in range(3)]
    fixed, stuck = deconflict(same, pkgs, (0, 0))
    assert stuck == [] and conflicts(fixed, (0, 0)) == []
    assert sorted(r.timeline[0][0] for r in fixed) == [0, 15, 30]
    late = [r._replace(stops=[(r.stops[0][0], 100)]) for r in same]
    tight = [p._replace(window=(0, 100)) for p in pkgs]   # no room to wait: one flies, two stay stuck
    _, stuck = deconflict(late, tight, (0, 0))
    assert len(stuck) == 2
