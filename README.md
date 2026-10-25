# Crane planner

Plans container crane moves to balance, unload, and load a ship, in minimum crane time.

The project started as the CS 179M senior project at UC Riverside (Winter 2022, team of 3,
original repo: [luccavp12/CS179m](https://github.com/luccavp12/CS179m)). That version is a
Flask web app with a greedy balancer (`balance.py`, `operations.py`, `hello.py`). It is kept
here unchanged except for an import guard.

The 2026 extension (`planner/`) replaces the greedy with an A* search and benchmarks the two
against each other.

## How it works

- **State** (`planner/ship.py`): 12 columns x 8 levels, each column a stack of
  `(weight, name)`. Unusable (`NAN`) cells are treated as fixed obstacles. The state is
  hashable, so the search can remember which states it has already seen.
- **Moves:** lift the top container of a column and set it on another column that has room,
  or hand it to or from the truck at the dock.
- **Cost:** crane minutes. The crane goes up, across, and down, clearing every stack in
  between by one level. Empty travel to the next pickup counts, plus 2 minutes per truck
  handoff.
- **Balanced:** the lighter side weighs at least 90% of the heavier side (`BALANCE_RATIO`).
- **Search** (`planner/crane.py`): one A* function with a different goal and heuristic per
  job:
  - *Balance:* some container now on the heavy side must cross the centerline, so the plan
    costs at least the horizontal distance from the nearest occupied heavy-side column.
  - *SIFT* (when no split of the weights can balance): heaviest containers nearest the
    center. Every container above the first misplaced cell in its column must be lifted
    and carried at least to the nearest column where its weight belongs.
  - *Unload/load:* every unload must reach the dock and every load must leave it, plus the
    truck handoff.

  None of the heuristics ever overestimates. The tests check this by comparing A* against
  uniform-cost search, which must give the same plan cost.

## Results

`python bench.py` runs the original greedy and the A* planner on the five test manifests,
scoring both with the same crane-time model:

| Case | Greedy moves | Greedy min | A* moves | A* min | A* runtime | Notes |
|---|---|---|---|---|---|---|
| 1 | 1 | 15 | 1 | 15 | 1 ms | same plan |
| 2 | 2 | 22 | 2 | 22 | 65 ms | same plan |
| 3 | 2 | 25 | 2 | 25 | 144 ms | same plan |
| 4 | 6 | 57 | 4 | 43 | 132 ms | A* 25% less crane time: greedy stacks blockers onto a column, then has to move them off again |
| 5 | 10 | invalid | 5 | 43 | 6.6 s | can't be balanced (one box weighs 96 of 113 total). Greedy's 2nd move lifts the cell it just emptied. A* plans the SIFT layout. |

The greedy's own minute count (4, 6, 8, 26, 28) leaves out empty crane travel and the climb
over stacks, so the table rescores its moves with the same model as A*.

## Run

```
python -m venv .venv
.venv\Scripts\activate          # Linux/macOS: source .venv/bin/activate
pip install -r requirements.txt
pytest -q
python bench.py
```
