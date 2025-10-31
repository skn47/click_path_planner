#include "point_cloud_handler.h"

PointCloudHandler::PointCloudHandler(const std::string& ground_pcd_path,
                      const std::string& obs_pcd_path,
                      double resolution)
        : resolution(resolution),
          ground_points(new Cloud),
          obs_points(new Cloud)
    {
        pcl::io::loadPCDFile(ground_pcd_path, *ground_points);
        pcl::io::loadPCDFile(obs_pcd_path, *obs_points);

        PointT gmin, gmax, omin, omax;
        pcl::getMinMax3D(*ground_points, gmin, gmax);
        pcl::getMinMax3D(*obs_points, omin, omax);

        const double pad = 2.0 * resolution;
        //const double pad = 0.25;
        min_xy = {
            std::min(gmin.x, omin.x) - pad,
            std::min(gmin.y, omin.y) - pad
        };
        max_xy = {
            std::max(gmax.x, omax.x) + pad,
            std::max(gmax.y, omax.y) + pad
        };

        nx = std::max(1, static_cast<int>(std::ceil((max_xy.x() - min_xy.x()) / resolution)));
        ny = std::max(1, static_cast<int>(std::ceil((max_xy.y() - min_xy.y()) / resolution)));
    }

GridMap PointCloudHandler::build_map(bool inflate, double inflate_radius) const {
    GridMap grid(ny, nx, min_xy, resolution);

    auto in_bounds = [&](double x, double y) -> bool {
        if (std::isnan(x) || std::isnan(y)) return false;
        int gx = static_cast<int>(std::floor((x - min_xy.x()) / resolution));
        int gy = static_cast<int>(std::floor((y - min_xy.y()) / resolution));
        return (gx >= 0 && gx < nx && gy >= 0 && gy < ny);
    };

    for (const auto& p : ground_points->points) {
        if (!in_bounds(p.x, p.y)) continue;
        size_t idx = grid.world2lin({p.x, p.y});
        if (grid.cells[idx].state == UNKNOWN) grid.cells[idx].state = FREE;
    }

    const double max_obs_z = 2.0;
    for (const auto& p : obs_points->points) {
        if (p.z > max_obs_z) continue;
        if (!in_bounds(p.x, p.y)) continue;
        size_t idx = grid.world2lin({p.x, p.y});
        grid.cells[idx].state = OCCUPIED;
    }

    if (inflate) grid.inflate_obstacles(inflate_radius);
    return grid;
}