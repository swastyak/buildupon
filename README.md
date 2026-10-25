# Crane + drone delivery planner

Plans container crane moves to balance, unload, and load a ship in minimum crane time, then
routes a drone fleet to deliver the unloaded packages.

The project started as the CS 179M senior project at UC Riverside (Winter 2022, team of 3,
original repo: [luccavp12/CS179m](https://github.com/luccavp12/CS179m)). That version is a
Flask web app with a greedy balancer (`balance.py`, `operations.py`, `hello.py`). It is kept
here unchanged except for an import guard.

The 2026 extension (`planner/`) replaces the greedy with an A* search, benchmarks the two
against each other, and adds drone delivery routing.

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

## Drone delivery routing (`planner/drone.py`)

- **Paths:** 8-connected A* on a grid of 50 m cells, around no-fly zones (rectangles and
  circles). The heuristic is octile distance, the exact cost on an empty grid, so it never
  overestimates. Diagonal steps can't cut the corner of a no-fly cell.
- **Smoothing:** from each waypoint, jump to the farthest later waypoint with a clear straight
  line. The line check is an exact grid traversal (Amanatides-Woo) in integer arithmetic.
  Shortcuts never make a path longer.
- **Fleet:** Google OR-Tools vehicle routing over the smoothed path lengths, with payload
  capacity, battery range per sortie, delivery time windows, and drop penalties so
  packages that can't be delivered are reported instead of making the solve fail.
- **Separation:** samples every drone's position each second and flags pairs closer than
  150 m, ignoring the area right around the hub. `deconflict` then delays departures in
  15 s steps until no conflicts remain, as long as every delivery stays in its window.

On a 5 km x 5 km map with 3 no-fly zones, 15 packages and 6 drones: planning takes 2.9 s
(2 s is the OR-Tools search limit). All 6 drones leaving at t=0 caused 6 separation
conflicts on the way out of the hub; staggering departures 15-45 s cleared all of them.

Simplifications: one sortie per drone per wave (no multi-trip), and a flat 2D map (no
altitude layers or wind).

## Run

```
python -m venv .venv
.venv\Scripts\activate          # Linux/macOS: source .venv/bin/activate
pip install -r requirements.txt
pytest -q
python bench.py
```
