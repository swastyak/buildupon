"""Original greedy balancer (balance.py) vs the A* planner, scored by the same crane-time model.

    python bench.py
"""
import contextlib
import io
import json
import time

import balance as greedy
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


if __name__ == "__main__":
    main()
