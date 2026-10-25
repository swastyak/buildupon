"""Benchmarks.

1. Original greedy balancer (balance.py) vs the A* planner, scored by the same crane-time model.
2. Drone grid A* + smoothing: pure Python vs the C++ extension (if built).

    python bench.py
"""
import contextlib
import io
import json
import random
import time

import balance as greedy
from planner import drone
from planner.crane import plan_balance
from planner.ship import DOCK, State, balanced, load_manifest, moves

CASES = [f"JSONs/shipCase{i}.json" for i in range(1, 6)]


def run_greedy(path):
    """Run the original algorithm untouched; return its move list and its own minute count."""
    greedy.moves, greedy.move_Dict, greedy.tot_distance = 0, {}, 0
    greedy.all_best_balances.clear()
    with open(path) as f:
        manifest = json.load(f)
    with contextlib.redirect_stdout(io.StringIO()):
        greedy.balance(manifest)
    parse = lambda key: tuple(int(n) for n in key.strip("[]").split(","))
    return [(parse(m["origin"]), parse(m["destination"])) for m in greedy.move_Dict.values()], greedy.tot_distance


def score(path, cell_moves):
    """Replay greedy's (row, col) moves through the ship model.

    Returns (minutes, final cols, problem). problem names the first move the
    ship model rejects: lifting a non-top or empty cell, or setting a box down
    anywhere but on top of a stack.
    """
    cols, base = load_manifest(path)
    state, total = State(cols, DOCK), 0
    for k, ((r1, c1), (r2, c2)) in enumerate(cell_moves, 1):
        s, d = c1 - 1, c2 - 1
        top = base[s] + len(state.cols[s])
        if r1 != top:
            what = "an empty cell" if r1 > top else "a buried box"
            return None, state.cols, f"move {k} lifts {what} at [{r1:02},{c1:02}]"
        if r2 != base[d] + len(state.cols[d]) + 1:
            return None, state.cols, f"move {k} sets a box down at [{r2:02},{c2:02}], not on top of a stack"
        state, cost = next((n, c) for m, n, c in moves(state, base) if (m.src, m.dst) == (s, d))
        total += cost
    return total, state.cols, None


def main():
    print("| Case | Greedy moves | Greedy min (its own count) | Greedy min (crane model) | A* plan | A* moves | A* min | A* ms | Notes |")
    print("|---|---|---|---|---|---|---|---|---|")
    for i, path in enumerate(CASES, 1):
        notes = []
        try:
            g_moves, g_own = run_greedy(path)
            g_min, g_cols, problem = score(path, g_moves)
            if problem:
                notes.append(f"greedy: {problem}")
            elif not balanced(g_cols):
                notes.append("greedy ends unbalanced")
        except Exception as e:  # the original code can crash; that is a finding too
            g_moves, g_own, g_min = [], "-", None
            notes.append(f"greedy crashed: {type(e).__name__}: {e}")

        cols, base = load_manifest(path)
        t = time.perf_counter()
        kind, plan, a_min, _ = plan_balance(cols, base)
        ms = (time.perf_counter() - t) * 1000

        if g_min:
            notes.append(f"A* saves {100 * (g_min - a_min) / g_min:.0f}%")
        g_min = g_min if g_min is not None else "invalid"
        print(f"| {i} | {len(g_moves)} | {g_own} | {g_min} | {kind} | {len(plan)} | {a_min} | {ms:.0f} | {'; '.join(notes)} |")


def grid_bench(size=1000, pairs=10, seed=0):
    """Same random start/goal pairs on a size x size map, timed in Python and C++."""
    s = size / 100  # scale the demo city's no-fly zones up to this map size
    m = drone.make_map(size, size, [("rect", int(30 * s), 0, int(34 * s), int(70 * s)),
                                    ("circle", int(62 * s), int(60 * s), int(12 * s)),
                                    ("rect", int(70 * s), int(20 * s), int(95 * s), int(28 * s))])
    rng = random.Random(seed)
    cells = []
    while len(cells) < 2 * pairs:
        c = (rng.randrange(size), rng.randrange(size))
        if drone.free(m, c):
            cells.append(c)
    jobs = list(zip(cells[::2], cells[1::2]))

    t = time.perf_counter()
    py = [drone.smooth(m, drone.grid_path(m, a, b)) for a, b in jobs]
    py_s = time.perf_counter() - t

    print(f"\n## Grid A* + smoothing, {size}x{size} map, {pairs} paths\n")
    print("| Version | Total | Per path |")
    print("|---|---|---|")
    print(f"| Python | {py_s:.2f} s | {1000 * py_s / pairs:.0f} ms |")
    if drone._fastgrid is None:
        print("| C++ | not built (see setup.py) | |")
        return
    t = time.perf_counter()
    g = drone._fastgrid.Grid(m.width, m.height, list(m.blocked))
    cpp = [g.smooth(g.path(a, b)) for a, b in jobs]
    cpp_s = time.perf_counter() - t
    assert cpp == py, "C++ and Python paths differ"
    print(f"| C++ (incl. building the grid) | {cpp_s:.2f} s | {1000 * cpp_s / pairs:.0f} ms |")
    print(f"\nC++ is {py_s / cpp_s:.0f}x faster; both return identical paths.")


if __name__ == "__main__":
    main()
    grid_bench()
