#pragma once
#include <string>
#include <cmath>
#include <algorithm>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>

#include "grid_map.h"

class PointCloudHandler {
public:
    using PointT = pcl::PointXYZ;
    using Cloud = pcl::PointCloud<PointT>;

    PointCloudHandler(const std::string& ground_pcd_path,
                      const std::string& obs_pcd_path,
                      double resolution);

    GridMap build_map(bool inflate = true, double inflate_radius = 0.25) const;

private:
    Cloud::Ptr ground_points;
    Cloud::Ptr obs_points;
    double resolution = 0.05;
    int ny = 0, nx = 0;
    Eigen::Vector2d min_xy{0,0}, max_xy{0,0};
};