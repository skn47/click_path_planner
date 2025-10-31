#include "grid_map.h"

Cell::Cell(const Eigen::Vector2d& center, CellState state) : center(center), state(state) {}
Cell::Cell(CellState state) : state(state) {}
Cell::Cell(CellState state, size_t idx) : state(state), idx(idx) {}

GridMap::GridMap(int ny, int nx, const Eigen::Vector2d& min_pt, double resolution)
    : ny(ny), nx(nx), min_pt(min_pt), resolution(resolution)
{
    Cell cell_init{UNKNOWN};
    cells.resize(ny * nx, cell_init);
    for (int y = 0; y < ny; ++y) {
        for (int x = 0; x < nx; ++x) {
            Cell& c = (*this)(x, y);
            c.idx = y * nx + x;
            c.center.x() = min_pt.x() + (x + 0.5) * resolution;
            c.center.y() = min_pt.y() + (y + 0.5) * resolution;
        }
    }
}

Eigen::Vector2i GridMap::lin2grid(int lin_idx) const {
    //return {lin_idx % nx, static_cast<int>(lin_idx / nx)};
    return Eigen::Vector2i(lin_idx % nx, static_cast<int>(lin_idx / nx));
}

size_t GridMap::grid2lin(const Eigen::Vector2i& grid_idx) const {
    return grid_idx.y() * nx + grid_idx.x();
}

size_t GridMap::world2lin(const Eigen::Vector2d& sub) const {
    size_t y = static_cast<int>(std::floor((sub.y() - min_pt.y()) / resolution));
    size_t x = static_cast<int>(std::floor((sub.x() - min_pt.x()) / resolution));
    return y * nx + x;
}

Cell& GridMap::operator() (int x, int y) {
    return cells[y * nx + x];
}

const Cell* GridMap::operator() (int x, int y) const {
    return &cells[y * nx + x];
}

void GridMap::inflate_obstacles(double robot_radius) {
    const int r_cells = static_cast<int>(std::ceil(robot_radius / resolution));
    if (r_cells <= 0) return;

    const int r2 = r_cells * r_cells;

    std::vector<Eigen::Vector2i> occ;
    occ.reserve(nx * ny / 8); // heuristic
    for (int y = 0; y < ny; ++y) {
        for (int x = 0; x < nx; ++x) {
            if (cells[y * nx + x].state == OCCUPIED) {
                occ.emplace_back(x, y);
            }
        }
    }

    std::vector<Eigen::Vector2i> disk;
    disk.reserve((2 * r_cells + 1) * (2 * r_cells + 1));
    for (int dy = -r_cells; dy <= r_cells; ++dy) {
        for (int dx = -r_cells; dx <= r_cells; ++dx) {
            if (dx * dx + dy * dy <= r2) {
                disk.emplace_back(dx, dy);
            }
        }
    }

    for (const auto& p : occ) {
        for (const auto& d : disk) {
            const int x = p.x() + d.x();
            const int y = p.y() + d.y();
            if (x < 0 || x >= nx || y < 0 || y >= ny) continue;

            auto& cell = (*this)(x, y);
            if (cell.state == FREE || cell.state == UNKNOWN) {
                cell.state = INFLATED;
            }
        }
    }
}

double GridMap::A_star_search(int src_idx, int dst_idx, std::vector<const Cell*>& path) {
    const double root2 = std::sqrt(2);
    const std::array<int,8> dx = {0, 1, 0, -1, 1, 1, -1, -1};
    const std::array<int,8> dy = {1, 0, -1, 0, 1, -1, -1, 1};
    const std::array<double,8> cost = {1, 1, 1, 1, root2, root2, root2, root2};

    auto is_valid_move = [=](int idx, int dx, int dy) -> bool {
        if ((idx % nx == 0 && dx == -1) || (idx % nx == nx - 1 && dx == 1) ||
            (idx + nx * dy < 0) || (idx + nx * dy >= nx * ny) ||
            (cells[idx + dy * nx + dx].state == OCCUPIED) ||
            (cells[idx + dy * nx + dx].state == INFLATED))
        {
            return false;
        }
        return true;
    };

    const Eigen::Vector2i dst_grid = lin2grid(dst_idx);

    auto h = [=](int idx) {
        Eigen::Vector2i cur = lin2grid(idx);
        int delta_x = std::abs(cur.x() - dst_grid.x()), 
            delta_y = std::abs(cur.y() - dst_grid.y());
        return (delta_x + delta_y) + (root2 - 2) * std::min(delta_x, delta_y);
    };
    
    typedef std::pair<double,int> PairT;
    
    std::priority_queue<PairT, std::vector<PairT>, std::greater<PairT>> pq;
    std::vector<double> g(ny * nx, DBL_MAX);
    std::vector<double> f(ny * nx, DBL_MAX);
    std::vector<int> prev(ny * nx, -1);
    g[src_idx] = 0.0, f[src_idx] = h(src_idx);
    pq.emplace(std::make_pair(f[src_idx], src_idx));

    while (!pq.empty()) {
        double cur_f = pq.top().first;
        int u = pq.top().second;
        pq.pop();
        if (f[u] < cur_f) continue;
        else if (u == dst_idx) break;
        for (int i = 0; i < 8; ++i) {
            if (!is_valid_move(u, dx[i], dy[i])) continue;
            int v = u + dy[i] * nx + dx[i];
            if (g[u] + cost[i] < g[v]) {
                g[v] = g[u] + cost[i];
                f[v] = g[v] + h(v);
                prev[v] = u;
                pq.emplace(std::make_pair(f[v], v));
            }
        }
    }

    if (prev[dst_idx] == -1) return DBL_MAX;
    path.clear();
    int cur = dst_idx;
    while (prev[cur] != -1) {
        path.push_back(&cells[cur]);
        cur = prev[cur];
    }
    std::reverse(path.begin(),path.end());
    return g[dst_idx] * resolution;
}