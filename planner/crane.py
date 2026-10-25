"""A* crane planner: balance a ship, or unload/load containers, in minimum crane minutes."""
import heapq
import itertools

from .ship import BALANCE_RATIO, COLS, DOCK, MID, ROWS, TRUCK_MIN, State, balanced, moves, side_weights

CENTER_OUT = [MID - 1 - i // 2 if i % 2 == 0 else MID + i // 2 for i in range(COLS)]  # 5,6,4,7,...


def zero(state):
    return 0


class NoPlan(Exception):
    pass


def astar(start, base, is_goal, h, limit=500_000):
    """Return (plan, cost, expanded). plan is a list of (Move, minutes).

    With h=lambda s: 0 this is uniform-cost search. h must never overestimate
    the remaining cost, or the plan may not be optimal.
    """
    tie = itertools.count()
    best = {start: 0}
    parent = {start: None}
    frontier = [(h(start), next(tie), start)]
    expanded = 0
    while frontier:
        _, _, state = heapq.heappop(frontier)
        g = best[state]
        if is_goal(state):
            plan = []
            while parent[state]:
                prev, move, cost = parent[state]
                plan.append((move, cost))
                state = prev
            return plan[::-1], g, expanded
        expanded += 1
        if expanded > limit:
            raise NoPlan(f"gave up after {limit} expansions")
        for move, nxt, cost in moves(state, base):
            if g + cost < best.get(nxt, float("inf")):
                best[nxt] = g + cost
                parent[nxt] = (state, move, cost)
                heapq.heappush(frontier, (g + cost + h(nxt), next(tie), nxt))
    raise NoPlan("no reachable goal state")


# --- balance -----------------------------------------------------------------

def balance_h(state):
    """Lower bound on minutes left to balance.

    Net weight must cross from the heavy side to the light side, so some
    container that is on the heavy side now must end on the light side. Its
    carry is at least its column's horizontal distance to the first column
    across the centerline, and travel is never shorter than horizontal distance.
    """
    if balanced(state.cols):
        return 0
    left, right = side_weights(state.cols)
    if left > right:
        return min(MID - c for c in range(MID) if state.cols[c])
    return min(c - (MID - 1) for c in range(MID, COLS) if state.cols[c])


def balanceable(cols, base):
    """True if some split of the containers between sides meets the rule and fits."""
    weights = [w for s in cols for w, _ in s]
    total = sum(weights)
    cap_left = sum(ROWS - b for b in base[:MID])
    cap_right = sum(ROWS - b for b in base[MID:])
    splits = {(0, 0)}  # (left weight, left count)
    for w in weights:
        splits |= {(lw + w, n + 1) for lw, n in splits}
    for lw, n in splits:
        rw = total - lw
        if n <= cap_left and len(weights) - n <= cap_right:
            if max(lw, rw) == 0 or min(lw, rw) / max(lw, rw) >= BALANCE_RATIO:
                return True
    return False


def sift_target(cols, base):
    """Heaviest containers nearest the center, filling level by level (the original SIFT layout)."""
    weights = sorted((w for s in cols for w, _ in s), reverse=True)
    target = [[] for _ in range(COLS)]
    slots = [c for level in range(1, ROWS + 1) for c in CENTER_OUT if level > base[c]]
    for w, c in zip(weights, slots):
        target[c].append(w)
    return tuple(map(tuple, target))


def plan_balance(cols, base, informed=True):
    """Balance if possible, else SIFT. informed=False runs uniform-cost search (for checking)."""
    start = State(cols, DOCK)
    if balanceable(cols, base):
        return "balance", *astar(start, base, lambda s: balanced(s.cols), balance_h if informed else zero)

    target = sift_target(cols, base)
    as_weights = lambda s: tuple(tuple(w for w, _ in col) for col in s.cols)

    def sift_h(s):
        """In each column, every container at or above the first cell that differs
        from the target must be lifted at least once. Each such container then
        travels at least to the nearest other column whose target holds its
        weight, or 2 if it can only come back to its own column (out and back)."""
        now, total = as_weights(s), 0
        for c in range(COLS):
            first_bad = next((i for i, w in enumerate(now[c]) if i >= len(target[c]) or target[c][i] != w), None)
            if first_bad is None:
                continue
            for w in now[c][first_bad:]:
                total += min(abs(c - d) if d != c else 2 for d in range(COLS) if w in target[d])
        return total

    return "sift", *astar(start, base, lambda s: as_weights(s) == target, sift_h if informed else zero)


# --- unload / load -----------------------------------------------------------

def unload_h(state):
    """Each container still to unload must be carried from its column to the dock
    (horizontal distance col+1) plus the truck handoff; each load needs the handoff
    plus at least one column of travel. Moves are separate, so the bounds add up."""
    total = 0
    for name in state.unload:
        total += min(c + 1 for c in range(COLS) if any(n == name for _, n in state.cols[c])) + TRUCK_MIN
    return total + len(state.load) * (TRUCK_MIN + 1)


def plan_transfer(cols, base, unload=(), load=(), informed=True):
    start = State(cols, DOCK, tuple(sorted(unload)), tuple(sorted(load)))
    return astar(start, base, lambda s: not s.unload and not s.load, unload_h if informed else zero)
