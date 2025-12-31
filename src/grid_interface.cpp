#include "grid_interface.h"

GridInterface::GridInterface(GridMap& grid, std::string win)
    : grid_(grid), win_(win) {}

cv::Mat GridInterface::color_grid() {
    const cv::Vec3b COLOR_FREE     = cv::Vec3b(255, 255, 255); // FREE -> white
    const cv::Vec3b COLOR_OBSTACLE = cv::Vec3b(  0,   0,   0); // OCCUPIED -> black
    const cv::Vec3b COLOR_INFLATED = cv::Vec3b(128,   0,   0); // INFLATED -> dark blue (B high)
    const cv::Vec3b COLOR_UNKNOWN  = cv::Vec3b(220, 220, 220); // UNKNOWN -> gray

    const int H = grid_.ny, W = grid_.nx;
    cv::Mat img(H, W, CV_8UC3);

    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            const Cell& c = grid_.cells[y * W + x];
            cv::Vec3b color;
            switch (c.state) {
                case FREE:     color = COLOR_FREE;     break;
                case OCCUPIED: color = COLOR_OBSTACLE; break;
                case INFLATED: color = COLOR_INFLATED; break;
                case UNKNOWN:  color = COLOR_UNKNOWN;  break;
                default:       color = COLOR_UNKNOWN;  break;
            }
            const int draw_y = H - 1 - y; // flip vertically for display
            img.at<cv::Vec3b>(draw_y, x) = color;
        }
    }
    return img;
}

void GridInterface::attach() {
    cv::namedWindow(win_, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    cv::setMouseCallback(win_, &GridInterface::onMouseThunk, this);
    refresh();
}

void GridInterface::refresh() {
    img_ = color_grid();
    drawPath();
    drawRobot();

    drawClickedCells();

    cv::imshow(win_, img_);
}

void GridInterface::onMouseThunk(int event, int x, int y, int flags, void* userdata) {
    if (auto* self = static_cast<GridInterface*>(userdata)) {
        self->onMouse(event, x, y, flags);
    }
}

void GridInterface::onMouse(int event, int x, int y, int /*flags*/) {
    if (event != cv::EVENT_LBUTTONDOWN) return;
    if (x < 0 || x >= grid_.nx || y < 0 || y >= grid_.ny) return;

    const int gx = x;
    const int gy = grid_.ny - 1 - y;  // image -> grid

    if (!grid_.robot_loc) return;
    const Cell dst = grid_(gx, gy);

    // NEW PART

    if (clicked_cells_.empty() || clicked_cells_.back() != grid_(gx, gy).idx) {
        clicked_cells_.push_back(grid_(gx, gy).idx);
    }

    std::vector<const Cell*> route;
    double d = grid_.A_star_search(grid_.robot_loc->idx, dst.idx, route);
    if (d == DBL_MAX || route.empty()) return;

    path_.insert(path_.end(), route.begin(), route.end());
    total_dist_ += d;
    grid_.robot_loc = &grid_.cells[route.back()->idx];
    //refresh();
}

void GridInterface::drawPath() {
    for (const Cell* c : path_) {
      Eigen::Vector2i g = grid_.lin2grid(c->idx);
      int px = g.x(), py = grid_.ny - 1 - g.y();
      img_.at<cv::Vec3b>(py, px) = cv::Vec3b(0, 0, 255); // red
    }
}

void GridInterface::drawRobot() {
    if (!grid_.robot_loc) return;
    Eigen::Vector2i g = grid_.lin2grid(grid_.robot_loc->idx);
    int px = g.x(), py = grid_.ny - 1 - g.y();
    cv::circle(img_, {px, py}, 2, cv::Scalar(0, 255, 0), -1); // green dot
}

void GridInterface::write_path(std::vector<Eigen::Vector2d>& ret) {
    if ((int)ret.size() < path_.size()) ret.resize(path_.size());
    for (int i = 0; i < path_.size(); ++i) ret[i] = path_[i]->center;
}

void GridInterface::save_path(const std::string& saves_dir) {
    //std::string saves_dir = "/root/ros_overlay_ws/src/click_path_planner/saves/";
    //std::string pkg_path = ament_index_cpp::get_package_share_directory("click_path_planner");
    if (saves_dir.empty()) {
        std::cerr << "saves_dir parameter not set\n";
        return;
    }
    if (clicked_cells_.empty()) {
        //std::cerr << "No cells clicked\n";
        return;
    }

    using namespace std::chrono;

    std::string prefix = "save";
    std::string extension = ".txt";

    auto now = system_clock::now();
    std::time_t t = system_clock::to_time_t(now);

    std::tm tm;
    localtime_r(&t, &tm);  // thread-safe on Linux

    std::ostringstream oss;
    oss << prefix << "_"
        << std::put_time(&tm, "%Y-%m-%d_%H.%M.%S")
        << extension;
    
    std::string filename = oss.str();
    std::filesystem::path fullpath = std::filesystem::path(saves_dir) / filename;

    std::ofstream out(fullpath);
    if (!out) {
        std::cerr << "Failed to open: " << fullpath << "\n";
        return;
    }
    for (int idx : clicked_cells_) {
        out << idx << '\n';
    }
}

void GridInterface::load_path(const std::string& filepath) {
    std::ifstream in(filepath);
    if (!in) {
        std::cerr << "Failed to open saved path file: " << filepath << "\n";
        return;
    }

    std::vector<int> loaded_idxs;
    int idx;

    while (in >> idx) {
        loaded_idxs.push_back(idx);
    }

    if (loaded_idxs.empty()) {
        std::cerr << "Saved path file is empty: " << filepath << "\n";
        return;
    }

    if (!grid_.robot_loc) {
        std::cerr << "grid_.robot_loc is null; cannot reconstruct path.\n";
        return;
    }

    path_.clear();
    total_dist_ = 0.0;

    //clicked_cells_.insert(clicked_cells_.end(), loaded_idxs.begin(), loaded_idxs.end());

    for (int to_idx : loaded_idxs) {
        if (to_idx < 0 ||
            to_idx >= static_cast<int>(grid_.cells.size()))
        {
            std::cerr << "Invalid cell index in save file: " << to_idx << "\n";
            continue;
        }
        const int from_idx = grid_.robot_loc->idx;
        std::vector<const Cell*> route;
        double d = grid_.A_star_search(from_idx, to_idx, route);
        if (d == DBL_MAX || route.empty()) {
            std::cerr << "Invalid path segment from " << from_idx
                      << " to " << to_idx << "\n";
            continue;
        }
        path_.insert(path_.end(), route.begin(), route.end());
        total_dist_ += d;
        grid_.robot_loc = &grid_.cells[route.back()->idx];
    }
}

bool GridInterface::map_to_grid_idx(double x_map, double y_map, double fiducial_yaw, int& idx_out) const
{
    double x_grid = std::cos(fiducial_yaw) * x_map + std::sin(fiducial_yaw) * y_map;
    double y_grid = -std::sin(fiducial_yaw) * x_map + std::cos(fiducial_yaw) * y_map;

    idx_out = grid_.world2lin({x_grid, y_grid});
    std::cout << "x_grid: " << x_grid << ", y_grid: " << y_grid << std::endl; 
    return true;

    const double res = grid_.resolution;
    const double ox  = grid_.min_pt.x();
    const double oy  = grid_.min_pt.y();
    const int width  = grid_.nx;
    const int height = grid_.ny;

    int col = static_cast<int>(std::floor((x_map - ox) / res));
    int row = static_cast<int>(std::floor((oy - y_map) / res));

    if (col < 0 || col >= width || row < 0 || row >= height) {
        return false;
    }

    idx_out = row * width + col;
    return true;
}

void GridInterface::drawClickedCells() {
    for (int idx : clicked_cells_) {
        Eigen::Vector2i g = grid_.lin2grid(idx);
        int px = g.x(), py = grid_.ny - 1 - g.y();
        cv::circle(img_, {px, py}, 3, cv::Scalar(255, 0, 255), -1);
    }
}

void GridInterface::load_path_from_map_frame(const std::string& filepath, double fiducial_yaw) {
    std::ifstream in(filepath);
    if (!in) {
        std::cerr << "Failed to open saved path file: " << filepath << std::endl;
        return;
    }

    std::vector<int> loaded_idxs;
    double x_map, y_map, yaw_map;
    while (in >> x_map >> y_map >> yaw_map) {
        int cell_idx = -1;
        if (!map_to_grid_idx(x_map, y_map, fiducial_yaw, cell_idx)) {
            std::cerr << "Skipping out-of-bounds map point (" 
                      << x_map << ", " << y_map << ")" << std::endl;
            continue;
        }
        waypoints.push_back({cell_idx, std::atan2(std::sin(yaw_map + fiducial_yaw), std::cos(yaw_map + fiducial_yaw))});
        loaded_idxs.push_back(cell_idx);
    }

    if (loaded_idxs.empty()) {
        std::cerr << "Map-frame path file is empty or all points were invalid: "
                  << filepath << std::endl;
        return;
    }

    clicked_cells_.clear();
    clicked_cells_.insert(clicked_cells_.end(), loaded_idxs.begin(), loaded_idxs.end());

    std::cout << "Loaded " << clicked_cells_.size() 
              << " clicked cells from map-frame path file." << std::endl;

    if (loaded_idxs.empty()) {
        std::cerr << "Saved path file is empty: " << filepath << "\n";
        return;
    }

    if (!grid_.robot_loc) {
        std::cerr << "grid_.robot_loc is null; cannot reconstruct path.\n";
        return;
    }

    path_.clear();
    total_dist_ = 0.0;

    //clicked_cells_.insert(clicked_cells_.end(), loaded_idxs.begin(), loaded_idxs.end());

    for (int to_idx : loaded_idxs) {
        if (to_idx < 0 ||
            to_idx >= static_cast<int>(grid_.cells.size()))
        {
            std::cerr << "Invalid cell index in save file: " << to_idx << "\n";
            continue;
        }
        const int from_idx = grid_.robot_loc->idx;
        std::vector<const Cell*> route;
        double d = grid_.A_star_search(from_idx, to_idx, route);
        if (d == DBL_MAX || route.empty()) {
            std::cerr << "Invalid path segment from " << from_idx
                      << " to " << to_idx << "\n";
            continue;
        }
        path_.insert(path_.end(), route.begin(), route.end());
        total_dist_ += d;
        grid_.robot_loc = &grid_.cells[route.back()->idx];
    }
}