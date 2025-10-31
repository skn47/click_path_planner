#pragma once
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>

#include <Eigen/Core>

#include "grid_map.h"

class GridInterface {
public:
    GridInterface(GridMap& grid, std::string win = "Occupancy Grid");
    cv::Mat color_grid();
    void attach();
    void refresh();
    void write_path(std::vector<Eigen::Vector2d>& ret);

private:
    static void onMouseThunk(int event, int x, int y, int flags, void* userdata);
    void onMouse(int event, int x, int y, int /*flags*/);
    void drawPath();
    void drawRobot();
    GridMap& grid_;
    std::string win_;
    cv::Mat img_;
    std::vector<const Cell*> path_;
    double total_dist_ = 0.0;
};