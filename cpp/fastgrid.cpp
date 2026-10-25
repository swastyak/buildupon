// Grid A* and line-of-sight smoothing for planner/drone.py, exposed to Python with pybind11.
//
// Same algorithm and the same tie-breaking as the Python version: the open list
// pops the smallest (f, x, y), and neighbors are tried in the same order, so
// both return identical paths. Cells are (x, y); cell (x, y) covers
// [x-0.5, x+0.5] x [y-0.5, y+0.5].
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

namespace py = pybind11;
using Cell = std::pair<int, int>;

static const double DIAG = std::sqrt(2.0);

class Grid {
public:
    Grid(int width, int height, const std::vector<Cell>& blocked)
        : w_(width), h_(height), blocked_(static_cast<size_t>(width) * height, 0) {
        for (auto [x, y] : blocked)
            if (inside(x, y)) blocked_[index(x, y)] = 1;
    }

    bool free(int x, int y) const { return inside(x, y) && !blocked_[index(x, y)]; }

    // 8-connected A* with the octile-distance heuristic; diagonal steps may not cut a blocked corner.
    std::vector<Cell> path(Cell start, Cell goal) const {
        if (!free(start.first, start.second) || !free(goal.first, goal.second))
            throw std::invalid_argument("start or goal is outside the map or in a no-fly zone");
        const double inf = std::numeric_limits<double>::infinity();
        std::vector<double> g(blocked_.size(), inf);
        std::vector<int> parent(blocked_.size(), -1);
        std::vector<char> closed(blocked_.size(), 0);

        using Entry = std::tuple<double, int, int>;  // (f, x, y)
        std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
        g[index(start.first, start.second)] = 0.0;
        open.emplace(octile(start, goal), start.first, start.second);

        while (!open.empty()) {
            auto [f, x, y] = open.top();
            open.pop();
            int cur = index(x, y);
            if (Cell{x, y} == goal) {
                std::vector<Cell> out;
                for (int c = cur; c != -1; c = parent[c]) out.emplace_back(c % w_, c / w_);
                return {out.rbegin(), out.rend()};
            }
            if (closed[cur]) continue;
            closed[cur] = 1;
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    int nx = x + dx, ny = y + dy;
                    if ((dx == 0 && dy == 0) || !free(nx, ny)) continue;
                    if (dx && dy && !(free(x + dx, y) && free(x, y + dy))) continue;
                    double ng = g[cur] + ((dx && dy) ? DIAG : 1.0);
                    int nxt = index(nx, ny);
                    if (ng < g[nxt]) {
                        g[nxt] = ng;
                        parent[nxt] = cur;
                        open.emplace(ng + octile({nx, ny}, goal), nx, ny);
                    }
                }
            }
        }
        throw std::invalid_argument("no path between start and goal");
    }

    // Exact grid traversal (Amanatides-Woo) in integer arithmetic; see clear() in drone.py.
    bool clear(Cell a, Cell b) const {
        int x = a.first, y = a.second;
        long long ax = std::abs(b.first - x), ay = std::abs(b.second - y);
        int sx = (b.first > x) - (b.first < x), sy = (b.second > y) - (b.second < y);
        long long ix = 0, iy = 0;
        if (!free(x, y)) return false;
        while (ix != ax || iy != ay) {
            long long tx = (2 * ix + 1) * ay, ty = (2 * iy + 1) * ax;
            if (iy == ay || (ix < ax && tx < ty)) {
                x += sx; ++ix;
            } else if (ix == ax || ty < tx) {
                y += sy; ++iy;
            } else {  // exactly through a corner: both side cells must be free
                if (!(free(x + sx, y) && free(x, y + sy))) return false;
                x += sx; y += sy; ++ix; ++iy;
            }
            if (!free(x, y)) return false;
        }
        return true;
    }

    // From each kept waypoint, jump to the farthest following one it can see.
    std::vector<Cell> smooth(const std::vector<Cell>& p) const {
        std::vector<Cell> out{p.front()};
        size_t i = 0;
        while (i + 1 < p.size()) {
            size_t j = i + 1;
            while (j + 1 < p.size() && clear(p[i], p[j + 1])) ++j;
            out.push_back(p[j]);
            i = j;
        }
        return out;
    }

private:
    int w_, h_;
    std::vector<char> blocked_;

    bool inside(int x, int y) const { return x >= 0 && x < w_ && y >= 0 && y < h_; }
    size_t index(int x, int y) const { return static_cast<size_t>(y) * w_ + x; }
    static double octile(Cell a, Cell b) {
        int dx = std::abs(a.first - b.first), dy = std::abs(a.second - b.second);
        return std::max(dx, dy) + (DIAG - 1) * std::min(dx, dy);
    }
};

PYBIND11_MODULE(_fastgrid, m) {
    m.doc() = "C++ grid A* and path smoothing for planner.drone";
    py::class_<Grid>(m, "Grid")
        .def(py::init<int, int, const std::vector<Cell>&>(), py::arg("width"), py::arg("height"), py::arg("blocked"))
        .def("free", &Grid::free)
        .def("path", &Grid::path, py::arg("start"), py::arg("goal"))
        .def("clear", &Grid::clear)
        .def("smooth", &Grid::smooth);
}
