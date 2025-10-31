#include <iostream>
#include <algorithm>
#include <memory>
#include <chrono>
#include <functional>
#include <opencv2/opencv.hpp>
#include <deque>

#include "point_cloud_handler.h"
#include "grid_interface.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "spot_action/action/move_relative_xy.hpp"
#include <Eigen/Core>

class MoveRelativeXYClient : public rclcpp::Node{
public:
  using MoveRelativeXY = spot_action::action::MoveRelativeXY;

  MoveRelativeXYClient() : Node("click_path_planner") {
    client_ = rclcpp_action::create_client<MoveRelativeXY>(this, "move_relative_xy");
  }

  void send_delta(float x, float y, float yaw) {
    using namespace std::chrono;
    if (!client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting.");
      return;
    }

    MoveRelativeXY::Goal goal_msg; goal_msg.x = x; goal_msg.y = y; goal_msg.yaw = yaw;

    RCLCPP_INFO(this->get_logger(), "Sending goal: x=%.3f y=%.3f yaw=%.3f", x, y, yaw);

    rclcpp_action::Client<MoveRelativeXY>::SendGoalOptions options;
    options.goal_response_callback =
      [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<MoveRelativeXY>> handle) {
        if (!handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server.");
        } else {
          RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result.");
        }
      };

    options.feedback_callback =
      [this](rclcpp_action::ClientGoalHandle<MoveRelativeXY>::SharedPtr /*handle*/,
             const std::shared_ptr<const MoveRelativeXY::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "Feedback: distance_to_goal=%.3f", feedback->distance_to_goal);
      };

    options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<MoveRelativeXY>::WrappedResult& result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Result: success=%s", result.result->success ? "true" : "false");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), "Goal aborted.");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Goal canceled.");
            break;
          default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
            break;
        }
    };
    client_->async_send_goal(goal_msg, options);
  }

  // Call this to start the sequence
void send_deltas(std::deque<Eigen::Vector2d> deltas) {
  auto queue = std::make_shared<std::deque<Eigen::Vector2d>>(std::move(deltas));
  send_next_delta(queue);
}

void send_next_delta(const std::shared_ptr<std::deque<Eigen::Vector2d>>& queue) {
  using Client     = rclcpp_action::Client<MoveRelativeXY>;
  using GoalHandle = Client::GoalHandle;
  using namespace std::chrono_literals;

  if (queue->empty()) {
    RCLCPP_INFO(this->get_logger(), "Path complete.");
    return;
  }

  if (!client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available.");
    return;
  }

  const Eigen::Vector2d tgt = queue->front();
  MoveRelativeXY::Goal goal;
  goal.x = tgt.x();
  goal.y = tgt.y();
  goal.yaw = 0.0;

  Client::SendGoalOptions options;
  std::weak_ptr<rclcpp::Node> weak = this->shared_from_this();

  options.goal_response_callback =
    [weak](std::shared_ptr<GoalHandle> gh) {
      if (auto node = weak.lock()) {
        if (!gh) RCLCPP_ERROR(node->get_logger(), "Goal rejected.");
        else     RCLCPP_INFO(node->get_logger(), "Goal accepted; waiting for result…");
      }
    };

  options.feedback_callback =
    [this](rclcpp_action::ClientGoalHandle<MoveRelativeXY>::SharedPtr /*handle*/,
            const std::shared_ptr<const MoveRelativeXY::Feedback> feedback) {
      RCLCPP_INFO(this->get_logger(), "Feedback: distance_to_goal=%.3f", feedback->distance_to_goal);
    };

  options.result_callback =
    [this, weak, queue](const GoalHandle::WrappedResult& res) {
      if (auto node = weak.lock()) {
        const bool ok = (res.code == rclcpp_action::ResultCode::SUCCEEDED) &&
                        (!res.result || res.result->success);
        if (!ok) {
          RCLCPP_WARN(node->get_logger(), "Step failed; stopping path.");
          return;
        }
        queue->pop_front();
        this->send_next_delta(queue);
      }
    };

  // options.feedback_callback = ...

  client_->async_send_goal(goal, options);
}


private:
    rclcpp_action::Client<MoveRelativeXY>::SharedPtr client_;
};

struct ResampleOptions {
    double step = 2.0;
    bool allow_short_last = true;
    double eps = 1e-9;
};

static inline double segLen(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    return (b - a).norm();
}

std::deque<Eigen::Vector2d>
make_translations_at_least_step(const std::vector<Eigen::Vector2d>& pts,
                                const ResampleOptions& opt = {}) {
    std::deque<Eigen::Vector2d> translations;
    // Resample points by arc length
    std::vector<Eigen::Vector2d> samples;
    samples.reserve(pts.size() * 2);

    Eigen::Vector2d cur = pts.front();  // trace polyline
    size_t seg_idx = 0;
    double remain = opt.step;

    while (seg_idx + 1 < pts.size()) {
        Eigen::Vector2d a = cur;
        Eigen::Vector2d b = pts[seg_idx + 1];
        Eigen::Vector2d ab = b - a;
        double L = ab.norm();

        if (L <= opt.eps) {
            cur = b;
            ++seg_idx;
            continue;
        }

        if (L + opt.eps >= remain) {
            Eigen::Vector2d dir = ab / L;
            Eigen::Vector2d newpt = a + dir * remain;
            samples.push_back(newpt);
            cur = newpt;
            remain = opt.step;
        } else {
            cur = b;
            ++seg_idx;
            remain -= L;
        }
    }
    if (opt.allow_short_last) {
        if ((samples.back() - pts.back()).norm() > opt.eps)
            samples.push_back(pts.back());
    } else {
    }
    for (size_t i = 1; i < samples.size(); ++i)
        translations.push_back(samples[i] - samples[i - 1]);
    return translations;
}

int main(int argc, char** argv) {
  try {
    /*PointCloudHandler pch("../data/hub_ground_points.pcd",
                          "../data/hub_off_ground_points_segmented3.pcd",
                          0.05);*/
    
    PointCloudHandler pch("../data/microgrid_ground.pcd",
                          "../data/microgroud_obs.pcd",
                          0.05);

    GridMap grid = pch.build_map(/*inflate=*/true, /*inflate_radius=*/0.25);

    auto it = std::find_if(grid.cells.begin(), grid.cells.end(),
                           [](const Cell& c){ return c.state == FREE; });
    if (it != grid.cells.end()) grid.robot_loc = &(*it);
    else grid.robot_loc = &grid.cells.front();  // fallback

    //grid.robot_loc = &grid.cells[grid.world2lin({0, 5})];
    grid.robot_loc = &grid.cells[grid.world2lin({0, -1})];

    std::cout << "Grid: " << grid.ny << " x " << grid.nx
              << "  res=" << grid.resolution << " m/cell\n";

    GridInterface ui(grid, "grid");
    ui.attach();

    for (;;) {
      int k = cv::waitKey(16);
      if (k == 27) break;        // ESC
    }
    cv::destroyAllWindows();

    std::vector<Eigen::Vector2d> path;
    ui.write_path(path);
    std::deque<Eigen::Vector2d> translations;

    Eigen::Vector2d cur = grid.cells[grid.world2lin({0, 5})].center;

    translations = make_translations_at_least_step(path);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveRelativeXYClient>();
    node->send_deltas(translations);
    rclcpp::spin(node);
    rclcpp::shutdown();

  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }
  return 0;
}