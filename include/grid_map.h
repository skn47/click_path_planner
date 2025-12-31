#ifndef GRID_MAP_HPP
#define GRID_MAP_HPP

#include <Eigen/Core>
#include <cfloat>
#include <queue>
#include <vector>
#include <array>
#include <cmath>

enum CellState : unsigned char {
    FREE = 0,
    OCCUPIED = 1,
    INFLATED = 2,
    UNKNOWN = 3
};

class Cell {
public:
    Cell(const Eigen::Vector2d& center, CellState state);
    Cell(CellState state);
    Cell(CellState state, size_t idx);

    Eigen::Vector2d center = Eigen::Vector2d(0,0);
    CellState state = UNKNOWN;
    size_t idx = -1;
};

class GridMap {
public:
    GridMap(int ny, int nx, const Eigen::Vector2d& min_pt, double resolution);

    //Eigen::Vector2d to_grid_frame(const Eigen::Vector2d& sub) const;
    //Eigen::Vector3d GridMap::to_grid_frame(const Eigen::Vector3d& sub) const;
    

    Eigen::Vector2i lin2grid(int lin_idx) const;
    size_t grid2lin(const Eigen::Vector2i& grid_idx) const;
    size_t world2lin(const Eigen::Vector2d& sub) const;

    Cell& operator()(int x, int y);
    const Cell* operator()(int x, int y) const;

    void inflate_obstacles(double robot_radius = 0.25);

    double A_star_search(int src_idx, int dst_idx, std::vector<const Cell*>& path);

    int ny, nx;
    Eigen::Vector2d min_pt;
    double resolution;
    std::vector<Cell> cells;
    Cell* robot_loc;
};

#endif // GRID_MAP_HPP