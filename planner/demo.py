"""End to end: unload a ship with the crane planner, then deliver its packages by drone.

    python -m planner.demo JSONs/shipCase3.json --seed 1
"""
import argparse
import math
import random

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from .crane import plan_transfer
from .drone import CELL_M, Package, conflicts, deconflict, free, make_map, plan_fleet
from .ship import DOCK, load_manifest

ZONES = {
    "airport corridor": ("rect", 30, 0, 34, 70),
    "stadium": ("circle", 62, 60, 12),
    "prison": ("rect", 70, 20, 95, 28),
}
CITY = make_map(100, 100, ZONES.values())
HUB = (3, 3)                     # next to the port
MIN_DROP_M = 500                 # closer than this, a drone isn't worth it


def clock(seconds):
    """Seconds after dispatch as +MmSSs."""
    m, s = divmod(round(seconds), 60)
    return f"+{m}m{s:02}s"


def where(col):
    return "truck" if col == DOCK else f"column {col + 1}"


def packages_from(containers, rng):
    """Each container holds 1-3 packages with seeded random drop-offs.
    About a third get a 15-minute delivery slot; the rest just need to arrive within the hour."""
    pkgs = []
    for name in containers:
        for k in range(rng.randint(1, 3)):
            cell = HUB
            while not free(CITY, cell) or math.dist(cell, HUB) * CELL_M < MIN_DROP_M:
                cell = (rng.randrange(CITY.width), rng.randrange(CITY.height))
            start = rng.choice([None, None, rng.randrange(0, 2700, 300)])
            window = (0, 3600) if start is None else (start, start + 900)
            pkgs.append(Package(f"{name}-{k + 1}", cell, rng.randrange(200, 1100), window))
    return pkgs


def plot(routes, packages, dropped, out):
    fig, ax = plt.subplots(figsize=(8, 8))
    grid = [[(x, y) in CITY.blocked for x in range(CITY.width)] for y in range(CITY.height)]
    ax.imshow(grid, origin="lower", cmap="Reds", alpha=0.35, extent=(-0.5, CITY.width - 0.5, -0.5, CITY.height - 0.5))
    for r in routes:
        xs, ys = zip(*[(x, y) for _, x, y in r.timeline])
        n = len(r.stops)
        ax.plot(xs, ys, lw=1.8, label=f"drone {r.drone}: {n} stop{'s' * (n > 1)}, {r.distance_m / 1000:.1f} km")
    for name, z in ZONES.items():
        cx, cy = ((z[1] + z[3]) / 2, (z[2] + z[4]) / 2) if z[0] == "rect" else z[1:3]
        ax.text(cx, cy, name, ha="center", va="center", fontsize=8, color="darkred")
    cells = {p.id: p.cell for p in packages}
    done = [cells[i] for r in routes for i, _ in r.stops]
    if done:
        ax.scatter(*zip(*done), s=30, c="black", zorder=3, label="delivered")
    if dropped:
        ax.scatter(*zip(*[cells[i] for i in dropped]), s=40, marker="x", c="gray", zorder=3, label="not delivered")
    ax.scatter(*HUB, s=200, marker="*", c="gold", edgecolors="black", zorder=4, label="hub")
    km = CITY.width * CELL_M / 1000
    ax.set(title=f"Drone deliveries ({km:.0f} x {km:.0f} km, red = no-fly)", xticks=[], yticks=[])
    ax.legend(loc="center right", fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=120)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("manifest")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--drones", type=int, default=6)
    ap.add_argument("--unload", type=int, default=3, help="how many containers to unload (seeded pick)")
    ap.add_argument("--weight", type=float, default=2.0, help="weighted A*; plan costs at most weight x optimal (1 = exact)")
    ap.add_argument("--out", default="routes.png")
    args = ap.parse_args()
    rng = random.Random(args.seed)

    cols, base = load_manifest(args.manifest)
    on_board = [name for stack in cols for _, name in stack]
    containers = rng.sample(on_board, min(args.unload, len(on_board)))
    plan, minutes, _ = plan_transfer(cols, base, unload=containers, weight=args.weight)
    bound = "optimal" if args.weight == 1 else f"weighted A*, at most {args.weight:g}x optimal"
    print(f"Crane: unload {', '.join(containers)} in {len(plan)} moves, {minutes} crane minutes ({bound})")
    for i, (m, cost) in enumerate(plan, 1):
        print(f"  {i:2}. {m.name:<10} {where(m.src)} -> {where(m.dst)}  ({cost} min)")

    packages = packages_from(containers, rng)
    routes, dropped = plan_fleet(CITY, HUB, packages, drones=args.drones)
    before = len(conflicts(routes, HUB))
    routes, stuck = deconflict(routes, packages, HUB)
    print(f"\nDrones: {len(packages)} packages, {len(packages) - len(dropped)} delivered this wave, "
          f"{len(dropped)} not delivered")
    print(f"Separation: {before} conflicts before staggering takeoffs, "
          f"{len(conflicts(routes, HUB))} after" + (f" (drones {stuck} could not be cleared)" if stuck else ""))
    for r in routes:
        stops = ", ".join(f"{i} at {clock(t)}" for i, t in r.stops)
        print(f"  drone {r.drone}: takes off at {clock(r.timeline[0][0])}, "
              f"{r.distance_m / 1000:.1f} km, {r.grams} g  [{stops}]")
    for i, why in dropped.items():
        print(f"  not delivered: {i} ({why})")

    plot(routes, packages, dropped, args.out)
    print(f"\nSaved {args.out}")


if __name__ == "__main__":
    main()
