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
