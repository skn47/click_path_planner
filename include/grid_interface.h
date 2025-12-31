#pragma once
//#include <ament_index_cpp/get_package_share_directory.hpp>

#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <filesystem>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>

#include <Eigen/Core>

#include "grid_map.h"

class GridInterface {
public:
    std::vector<int> clicked_cells_;
    std::vector<const Cell*> path_;
    //cv::Mat img_;
    GridMap& grid_;
    GridInterface(GridMap& grid, std::string win = "Occupancy Grid");
    cv::Mat color_grid();
    void attach();
    void refresh();
    void write_path(std::vector<Eigen::Vector2d>& ret);

    void save_path(const std::string& saves_dir);
    void load_path(const std::string& filepath);
    void load_path_from_map_frame(const std::string& filepath, double fiducial_yaw);
    bool map_to_grid_idx(double x_map, double y_map, double fiducial_yaw, int& idx_out) const;

    struct Waypoint {
        int cell_idx;
        double yaw_goal;
        Waypoint() : cell_idx(-1), yaw_goal(0.0) {}
        Waypoint(int cell_idx, double yaw_goal) : cell_idx(cell_idx), yaw_goal(yaw_goal) {}
    };

    std::deque<Waypoint> waypoints;

private:
    static void onMouseThunk(int event, int x, int y, int flags, void* userdata);
    void onMouse(int event, int x, int y, int /*flags*/);
    void drawPath();
    void drawRobot();
    //GridMap& grid_;
    std::string win_;
    cv::Mat img_;
    //std::vector<const Cell*> path_;
    double total_dist_ = 0.0;

    void drawClickedCells();
};