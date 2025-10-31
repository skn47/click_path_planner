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
    cv::namedWindow(win_);
    cv::setMouseCallback(win_, &GridInterface::onMouseThunk, this);
    refresh();
}

void GridInterface::refresh() {
    img_ = color_grid();
    drawPath();
    drawRobot();
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

    std::vector<const Cell*> route;
    double d = grid_.A_star_search(grid_.robot_loc->idx, dst.idx, route);
    if (d == DBL_MAX || route.empty()) return;

    path_.insert(path_.end(), route.begin(), route.end());
    total_dist_ += d;
    grid_.robot_loc = &grid_.cells[route.back()->idx];
    refresh();
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