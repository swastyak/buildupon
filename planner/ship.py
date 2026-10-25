"""Ship state, legal crane moves, and the crane-time cost model.

Coordinates: columns 0..11 (port side 0-5, starboard side 6-11), levels 1..8
counted from the bottom. The dock (truck transfer point) is a virtual column
DOCK=-1 sitting above the ship at level ROWS+1, left of column 0.
"""
import json
from typing import NamedTuple

ROWS, COLS = 8, 12
MID = COLS // 2           # columns < MID are the left side
DOCK = -1                 # crane park spot and truck transfer point
TRUCK_MIN = 2             # minutes to hand a container between dock and truck
BALANCE_RATIO = 0.9       # lighter side must weigh >= 90% of the heavier side


class State(NamedTuple):
    cols: tuple           # per column: tuple of (weight, name), bottom up, NAN excluded
    crane: int            # column the crane is over (DOCK or 0..11)
    unload: tuple = ()    # names still to unload, sorted
    load: tuple = ()      # (weight, name) still to load, sorted


class Move(NamedTuple):
    src: int              # column or DOCK (load from truck)
    dst: int              # column or DOCK (unload to truck)
    weight: int
    name: str


def load_manifest(path):
    """Read a manifest JSON ({"[RR,CC]": {"weight", "description"}}) into (cols, base).

    base[c] is the number of NAN (unusable) cells at the bottom of column c.
    """
    with open(path) as f:
        data = json.load(f)
    data = data.get("manifest", data)  # loadUnload files wrap it
    cols, base = [], []
    for c in range(1, COLS + 1):
        cells = [data["[%02d,%02d]" % (r, c)] for r in range(1, ROWS + 1)]
        nan = 0
        while nan < ROWS and cells[nan]["description"] == "NAN":
            nan += 1
        stack, gap = [], False
        for cell in cells[nan:]:
            desc = cell["description"]
            if desc == "NAN":
                raise ValueError(f"{path}: NAN above a usable cell in column {c}")
            if desc == "UNUSED":
                gap = True
            elif gap:
                raise ValueError(f"{path}: floating container in column {c}")
            else:
                stack.append((int(cell["weight"]), desc))
        cols.append(tuple(stack))
        base.append(nan)
    return tuple(cols), tuple(base)


def heights(cols, base):
    return [b + len(s) for s, b in zip(cols, base)]


def travel(a, la, b, lb, h):
    """Crane minutes from (column a, level la) to (column b, level lb).

    The crane goes up to a clearance level, across, then down. It must pass
    above every stack strictly between a and b. DOCK is at level ROWS+1.
    """
    lo, hi = sorted((a, b))
    between = [h[c] + 1 for c in range(max(lo + 1, 0), hi)]
    clear = max([la, lb] + between)
    return (clear - la) + abs(a - b) + (clear - lb)


def level_of(col, h):
    """Level the crane hook sits at when parked over a column (its top) or the dock."""
    return ROWS + 1 if col == DOCK else h[col]


def moves(state, base):
    """Yield (Move, next_state, cost) for every legal move from state."""
    h = heights(state.cols, base)
    crane, crane_lvl = state.crane, level_of(state.crane, h)
    open_cols = [c for c in range(COLS) if h[c] < ROWS]

    for s in range(COLS):
        if not state.cols[s]:
            continue
        w, name = state.cols[s][-1]
        reach = travel(crane, crane_lvl, s, h[s], h)
        h2 = h[:]
        h2[s] -= 1
        lifted = state.cols[:s] + (state.cols[s][:-1],) + state.cols[s + 1:]
        for d in open_cols:
            if d == s:
                continue
            cost = reach + travel(s, h[s], d, h2[d] + 1, h2)
            cols = lifted[:d] + (lifted[d] + ((w, name),),) + lifted[d + 1:]
            yield Move(s, d, w, name), state._replace(cols=cols, crane=d), cost
        if name in state.unload:
            rest = list(state.unload)
            rest.remove(name)
            cost = reach + travel(s, h[s], DOCK, ROWS + 1, h2) + TRUCK_MIN
            yield Move(s, DOCK, w, name), state._replace(cols=lifted, crane=DOCK, unload=tuple(rest)), cost

    for item in sorted(set(state.load)):
        rest = list(state.load)
        rest.remove(item)
        reach = travel(crane, crane_lvl, DOCK, ROWS + 1, h) + TRUCK_MIN
        for d in open_cols:
            cost = reach + travel(DOCK, ROWS + 1, d, h[d] + 1, h)
            cols = state.cols[:d] + (state.cols[d] + (item,),) + state.cols[d + 1:]
            yield Move(DOCK, d, *item), State(cols, d, state.unload, tuple(rest)), cost


def side_weights(cols):
    left = sum(w for s in cols[:MID] for w, _ in s)
    right = sum(w for s in cols[MID:] for w, _ in s)
    return left, right


def balanced(cols):
    left, right = side_weights(cols)
    return max(left, right) == 0 or min(left, right) / max(left, right) >= BALANCE_RATIO
