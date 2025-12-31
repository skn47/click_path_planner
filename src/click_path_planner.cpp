#include <iostream>
#include <algorithm>
#include <atomic>
#include <memory>
#include <chrono>
#include <future>
#include <functional>
#include <opencv2/opencv.hpp>
#include <deque>
#include <thread>

#include <fstream>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "point_cloud_handler.h"
#include "grid_interface.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "spot_action/action/move_relative_xy.hpp"
#include <Eigen/Core>

class MoveRelativeXYClient : public rclcpp::Node {
public:
  using MoveRelativeXY = spot_action::action::MoveRelativeXY;

  constexpr static double fiducial_yaw = 0.0;
  //constexpr static double fiducial_yaw = M_PI;

  MoveRelativeXYClient() : 
    Node("click_path_planner"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    this->declare_parameter<std::string>("ground_cloud_path", "");
    this->declare_parameter<std::string>("offground_cloud_path", "");
    this->declare_parameter<double>("resolution", 0.05);

    this->declare_parameter<std::string>("saves_dir", "");
    this->declare_parameter<std::string>("load_save_file", "");
    this->declare_parameter<std::string>("load_map_save_file", "");

    this->get_parameter("ground_cloud_path", ground_cloud_);
    this->get_parameter("offground_cloud_path", offground_cloud_);
    this->get_parameter("resolution", resolution_);

    this->get_parameter("saves_dir", saves_dir_);
    this->get_parameter("load_save_file", load_save_file_);
    this->get_parameter("load_map_save_file", load_map_save_file_);

    if (!load_save_file_.empty()) {
      RCLCPP_INFO(this->get_logger(), "Loading saved path from: %s",
                  load_save_file_.c_str());
    }

    if (ground_cloud_.empty() || offground_cloud_.empty()) {
      RCLCPP_FATAL(
        this->get_logger(),
        "Point cloud paths not set. Please provide 'ground_cloud_path' and 'offground_cloud_path' parameters."
      );
      throw std::runtime_error("Missing PCD paths");
    }

    RCLCPP_INFO(this->get_logger(), "Using ground cloud: %s", ground_cloud_.c_str());
    RCLCPP_INFO(this->get_logger(), "Using off-ground cloud: %s", offground_cloud_.c_str());
    client_ = rclcpp_action::create_client<MoveRelativeXY>(this, "move_relative_xy");
  }

  bool get_robot_pose_grid(Eigen::Vector3d& ret,
                          const std::string& map_frame = "world",
                          const std::string& base_frame = "base_link")
  {
    using namespace std::chrono_literals;
    geometry_msgs::msg::TransformStamped tf_grid_base;
    try {
      tf_grid_base = tf_buffer_.lookupTransform(map_frame, base_frame, tf2::TimePointZero, 200ms);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
      return false;
    }
    double x = tf_grid_base.transform.translation.x;
    double y = tf_grid_base.transform.translation.y;
    double grid_x = std::cos(fiducial_yaw) * x + std::sin(fiducial_yaw) * y;
    double grid_y = -std::sin(fiducial_yaw) * x + std::cos(fiducial_yaw) * y;
    tf2::Quaternion q(
      tf_grid_base.transform.rotation.x,
      tf_grid_base.transform.rotation.y,
      tf_grid_base.transform.rotation.z,
      tf_grid_base.transform.rotation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    double grid_yaw = std::atan2(std::sin(yaw + fiducial_yaw), std::cos(yaw + fiducial_yaw));
    ret = Eigen::Vector3d(grid_x, grid_y, grid_yaw);
    //std::cout << "(" << ret.x() << "," << ret.y() << ", yaw: " << ret.z() << "\n";
    return true;
  }

  bool update_robot_loc_from_tf(GridMap& grid,
                                const std::string& map_frame  = "world",
                                const std::string& base_frame = "base_link")
  {
    Eigen::Vector3d pos_grid;
    if (!get_robot_pose_grid(pos_grid, map_frame, base_frame)) {
      RCLCPP_WARN(this->get_logger(), "Could not get robot pose in grid frame from TF.");
      return false;
    }
    int idx = grid.world2lin(pos_grid.head<2>());
    if (idx < 0 || static_cast<std::size_t>(idx) >= grid.cells.size()) {
      RCLCPP_WARN(this->get_logger(), "Robot pose (%.3f, %.3f) is outside grid.", pos_grid.x(), pos_grid.y());
      return false;
    }
    grid.robot_loc = &grid.cells[idx];
    RCLCPP_INFO(this->get_logger(), "Updated robot cell from TF: x=%.3f y=%.3f idx=%d", pos_grid.x(), pos_grid.y(), idx);
    return true;
  }

  bool send_delta_sync(float x, float y, float yaw)
  {
    using Client     = rclcpp_action::Client<MoveRelativeXY>;
    using GoalHandle = Client::GoalHandle;
    using namespace std::chrono_literals;
    if (!client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting.");
      return false;
    }
    MoveRelativeXY::Goal goal_msg;
    goal_msg.x = x;
    goal_msg.y = y;
    goal_msg.yaw = yaw;
    RCLCPP_INFO(this->get_logger(),
                "Sending sync goal: x=%.3f y=%.3f yaw=%.3f", x, y, yaw);
    auto goal_options = Client::SendGoalOptions{};
    auto goal_future = client_->async_send_goal(goal_msg, goal_options);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_future)
        != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to send goal.");
      return false;
    }
    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server.");
      return false;
    }
    auto result_future = client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future)
        != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to get result.");
      return false;
    }
    auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_WARN(this->get_logger(), "Goal finished with code %d", (int)result.code);
      return false;
    }
    if (result.result && !result.result->success) {
      RCLCPP_WARN(this->get_logger(), "Action reported success=false.");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Step completed successfully.");
    return true;
  }

  /*
  bool follow_path_in_grid(
                        GridMap& grid,
                        GridInterface& ui,
                        const std::vector<Eigen::Vector2d>& path_grid,
                        double lookahead_dist = 2.0,
                        double max_step       = 3.0,
                        double goal_tol       = 0.10,
                        double yaw_tol        = 5 * M_PI / 180.0)
  {
    if (path_grid.empty()) {
      RCLCPP_WARN(this->get_logger(), "Empty path. Nothing to do.");
      return false;
    }

    rclcpp::Rate rate(10.0);
    const Eigen::Vector2d& goal = path_grid.back();
    auto it = path_grid.begin();
    const double max_yaw_step = 0.35;
    const double advance_tol  = std::min(lookahead_dist * 0.5, 0.20);

    auto wrap = [](double a){ return std::atan2(std::sin(a), std::cos(a)); };

    while (rclcpp::ok()) {
      Eigen::Vector3d pose_grid;
      if (!get_robot_pose_grid(pose_grid)) { rate.sleep(); continue; }
      //update_robot_loc_from_tf(grid);
      //ui.refresh();
      Eigen::Vector2d robot_xy = pose_grid.head<2>();
      double robot_yaw = pose_grid.z();

      double dist_to_goal = (goal - robot_xy).norm();
      if (dist_to_goal <= goal_tol) {
        RCLCPP_INFO(this->get_logger(), "Reached final goal (dist=%.3f).", dist_to_goal);
        return true;
      }

      while (it != path_grid.end() && ((*it - robot_xy).norm() <= advance_tol)) {
        ++it;
      }

      Eigen::Vector2d target = goal;
      if (it != path_grid.end()) {
        for (auto j = it; j != path_grid.end(); ++j) {
          double d = (*j - robot_xy).norm();
          if (d > lookahead_dist) { target = *j; it = j; break; }
        }
      }

      Eigen::Vector2d delta = target - robot_xy;
      double step_len = delta.norm();
      if (step_len < 1e-3) { rate.sleep(); continue; }
      if (step_len > max_step) { step_len = max_step; }

      double goal_yaw = std::atan2(delta.y(), delta.x());
      double yaw_err = wrap(goal_yaw - robot_yaw);

      const double k_yaw_p = 0.9;
      double yaw_cmd = std::clamp(k_yaw_p * yaw_err, -max_yaw_step, +max_yaw_step);

      if (std::abs(yaw_err) > yaw_tol) {
        RCLCPP_INFO(this->get_logger(),
                    "Heading align: curr=%.3f des=%.3f err=%.3f cmd=%.3f",
                    robot_yaw, goal_yaw, yaw_err, yaw_cmd);

        if (!send_delta_sync(0.0f, 0.0f, static_cast<float>(yaw_cmd))) {
          RCLCPP_WARN(this->get_logger(), "Yaw align step failed; stopping.");
          break;
        }
        rate.sleep();
        continue;
      }

      RCLCPP_INFO(this->get_logger(), "Forward: +x=%.3f (toward target %.3f, %.3f)",
                  step_len, target.x(), target.y());

      if (!send_delta_sync(static_cast<float>(step_len), 0.0f, 0.0f)) {
        RCLCPP_WARN(this->get_logger(), "Forward step failed; stopping.");
        break;
      }

      rate.sleep();
    }
    return false;
  }
  */

  bool follow_path_in_grid(
    GridMap& grid,
    GridInterface& ui,
    const std::vector<Eigen::Vector2d>& path_grid,
    double lookahead_dist = 2.0,
    double max_step       = 3.0,
    double goal_tol       = 0.10,
    double yaw_tol        = 5 * M_PI / 180.0)
  {
    if (path_grid.empty()) {
      RCLCPP_WARN(this->get_logger(), "Empty path. Nothing to do.");
      return false;
    }

    rclcpp::Rate rate(10.0);
    const Eigen::Vector2d& goal = path_grid.back();
    auto it = path_grid.begin();
    const double max_yaw_step = 0.35;
    const double advance_tol  = std::min(lookahead_dist * 0.5, 0.20);

    auto wrap = [](double a){ return std::atan2(std::sin(a), std::cos(a)); };

    const int    max_consecutive_failures = 10;
    const double max_stall_time           = 15.0;
    int consecutive_failures = 0;
    rclcpp::Time last_success = this->now();

    while (rclcpp::ok()) {
      Eigen::Vector3d pose_grid;
      if (!get_robot_pose_grid(pose_grid)) {
        rate.sleep();
        continue;
      }

      Eigen::Vector2d robot_xy = pose_grid.head<2>();
      double robot_yaw = pose_grid.z();

      double dist_to_goal = (goal - robot_xy).norm();
      if (dist_to_goal <= goal_tol) {
        RCLCPP_INFO(this->get_logger(),
                    "Reached final goal (dist=%.3f).", dist_to_goal);
        return true;
      }

      while (it != path_grid.end() && ((*it - robot_xy).norm() <= advance_tol)) {
        ++it;
      }

      Eigen::Vector2d target = goal;
      if (it != path_grid.end()) {
        for (auto j = it; j != path_grid.end(); ++j) {
          double d = (*j - robot_xy).norm();
          if (d > lookahead_dist) {
            target = *j;
            it = j;
            break;
          }
        }
      }

      Eigen::Vector2d delta = target - robot_xy;
      double step_len = delta.norm();
      if (step_len < 1e-3) {
        rate.sleep();
        continue;
      }
      if (step_len > max_step) {
        step_len = max_step;
      }

      double goal_yaw = std::atan2(delta.y(), delta.x());
      double yaw_err = wrap(goal_yaw - robot_yaw);

      const double k_yaw_p = 0.9;
      double yaw_cmd = std::clamp(k_yaw_p * yaw_err, -max_yaw_step, +max_yaw_step);

      auto check_abort = [&](const char* context) -> bool {
        auto now = this->now();
        double stalled = (now - last_success).seconds();

        if (consecutive_failures >= max_consecutive_failures ||
            stalled > max_stall_time)
        {
          RCLCPP_ERROR(this->get_logger(),
                      "%s: too many failures (%d) or stalled for %.1f s. Aborting.",
                      context, consecutive_failures, stalled);
          return true;
        }
        return false;
      };

      if (std::abs(yaw_err) > yaw_tol) {
        RCLCPP_INFO(this->get_logger(),
                    "Heading align: curr=%.3f des=%.3f err=%.3f cmd=%.3f",
                    robot_yaw, goal_yaw, yaw_err, yaw_cmd);

        if (!send_delta_sync(0.0f, 0.0f, static_cast<float>(yaw_cmd))) {
          ++consecutive_failures;
          RCLCPP_WARN(this->get_logger(),
                      "Yaw align step failed (consecutive=%d).",
                      consecutive_failures);

          if (check_abort("Yaw align")) {
            break;
          }

          rate.sleep();
          continue;
        } else {
          // success
          last_success = this->now();
          consecutive_failures = 0;
        }

        rate.sleep();
        continue;
      }

      RCLCPP_INFO(this->get_logger(),
                  "Forward: +x=%.3f (toward target %.3f, %.3f)",
                  step_len, target.x(), target.y());

      if (!send_delta_sync(static_cast<float>(step_len), 0.0f, 0.0f)) {
        ++consecutive_failures;
        RCLCPP_WARN(this->get_logger(),
                    "Forward step failed (consecutive=%d).",
                    consecutive_failures);

        if (check_abort("Forward")) {
          break;
        }

        rate.sleep();
        continue;
      } else {
        // success
        last_success = this->now();
        consecutive_failures = 0;
      }

      rate.sleep();
    }

    return false;
  }

  bool follow_waypoints(
    GridMap& grid,
    GridInterface& ui,
    const std::vector<Eigen::Vector2d>& path_grid,
    double lookahead_dist = 2.0,
    double max_step       = 3.0,
    double goal_tol       = 0.10,
    double yaw_tol        = 5 * M_PI / 180.0)
  {
    if (path_grid.empty()) {
      RCLCPP_WARN(this->get_logger(), "Empty path. Nothing to do.");
      return false;
    }

    rclcpp::Rate rate(10.0);
    const Eigen::Vector2d& goal = path_grid.back();
    auto it = path_grid.begin();
    const double max_yaw_step     = 0.35;
    const double advance_tol      = std::min(lookahead_dist * 0.5, 0.20);
    const double waypoint_pos_tol = 0.15;  // tune: "inside waypoint cell" tolerance

    auto wrap = [](double a){ return std::atan2(std::sin(a), std::cos(a)); };

    const int    max_consecutive_failures = 10;
    const double max_stall_time           = 15.0;
    int          consecutive_failures     = 0;
    rclcpp::Time last_success             = this->now();

    auto check_abort = [&](const char* context) -> bool {
      auto   now   = this->now();
      double stall = (now - last_success).seconds();

      if (consecutive_failures >= max_consecutive_failures ||
          stall > max_stall_time)
      {
        RCLCPP_ERROR(this->get_logger(),
                    "%s: too many failures (%d) or stalled for %.1f s. Aborting.",
                    context, consecutive_failures, stall);
        return true;
      }
      return false;
    };

    while (rclcpp::ok()) {
      // --- Get current pose ---
      Eigen::Vector3d pose_grid;
      if (!get_robot_pose_grid(pose_grid)) {
        rate.sleep();
        continue;
      }

      Eigen::Vector2d robot_xy = pose_grid.head<2>();
      double          robot_yaw = pose_grid.z();

      // --- Do NOT finish until all waypoints are consumed ---
      double dist_to_goal = (goal - robot_xy).norm();
      if (dist_to_goal <= goal_tol && ui.waypoints.empty()) {
        RCLCPP_INFO(this->get_logger(),
                    "Reached final goal (dist=%.3f) and all waypoints done.",
                    dist_to_goal);
        return true;
      }

      // ======================================================
      // 1) HANDLE CURRENT WAYPOINT (stop + yaw align)
      // ======================================================
      if (!ui.waypoints.empty()) {
        const auto& wp = ui.waypoints.front();

        // Get world/grid coordinates of waypoint cell center
        Eigen::Vector2d wp_xy = grid.cells[wp.cell_idx].center;  // <-- adapt to your API
        double dist_to_wp = (wp_xy - robot_xy).norm();

        // If we are inside/near the waypoint cell: STOP and align yaw
        if (dist_to_wp <= waypoint_pos_tol) {
          double desired_yaw = wp.yaw_goal; // absolute in grid frame
          double yaw_err     = wrap(desired_yaw - robot_yaw);

          if (std::abs(yaw_err) <= yaw_tol) {
            // Waypoint yaw satisfied → consume this waypoint
            RCLCPP_INFO(this->get_logger(),
                        "Waypoint %d reached & yaw aligned (err=%.3f). Popping.",
                        wp.cell_idx, yaw_err);
            ui.waypoints.pop_front();
            // Do NOT move forward this iteration; next loop will either
            // go to next waypoint or continue path.
            rate.sleep();
            continue;
          }

          // Need to rotate in place to reach waypoint's yaw goal
          const double k_yaw_p = 0.9;
          double yaw_cmd = std::clamp(k_yaw_p * yaw_err,
                                      -max_yaw_step, +max_yaw_step);

          RCLCPP_INFO(this->get_logger(),
                      "Waypoint yaw align: cell=%d curr=%.3f des=%.3f err=%.3f cmd=%.3f",
                      wp.cell_idx, robot_yaw, desired_yaw, yaw_err, yaw_cmd);

          // This send_delta_sync is effectively a MoveRelative command:
          // x=0, y=0, yaw=delta_yaw
          if (!send_delta_sync(0.0f, 0.0f, static_cast<float>(yaw_cmd))) {
            ++consecutive_failures;
            RCLCPP_WARN(this->get_logger(),
                        "Waypoint yaw align step failed (consecutive=%d).",
                        consecutive_failures);

            if (check_abort("Waypoint yaw align")) {
              break;
            }
          } else {
            last_success        = this->now();
            consecutive_failures = 0;
          }

          // Since we are in "stop at waypoint" mode, do not move forward
          rate.sleep();
          continue;
        }
        // else: we are not yet in the waypoint cell; fall through to normal path logic,
        // but we’ll clamp step_len so we don’t overshoot it.
      }

      // ======================================================
      // 2) NORMAL PATH-FOLLOWING (heading + forward move)
      // ======================================================

      // Advance path iterator when we’re close to current point
      while (it != path_grid.end() &&
            ((*it - robot_xy).norm() <= advance_tol))
      {
        ++it;
      }

      // Determine lookahead target
      Eigen::Vector2d target = goal;
      if (it != path_grid.end()) {
        for (auto j = it; j != path_grid.end(); ++j) {
          double d = (*j - robot_xy).norm();
          if (d > lookahead_dist) {
            target = *j;
            it     = j;
            break;
          }
        }
      }

      // Base step toward target
      Eigen::Vector2d delta = target - robot_xy;
      double          step_len = delta.norm();
      if (step_len < 1e-3) {
        rate.sleep();
        continue;
      }

      // If there is a waypoint, don't overshoot its cell center
      if (!ui.waypoints.empty()) {
        const auto& wp = ui.waypoints.front();
        Eigen::Vector2d wp_xy = grid.cells[wp.cell_idx].center;  // <-- adapt
        double dist_to_wp = (wp_xy - robot_xy).norm();
        step_len = std::min(step_len, dist_to_wp);
      }

      if (step_len > max_step) {
        step_len = max_step;
      }

      // Heading toward motion direction
      double goal_yaw = std::atan2(delta.y(), delta.x());
      double yaw_err  = wrap(goal_yaw - robot_yaw);

      const double k_yaw_p = 0.9;
      double yaw_cmd = std::clamp(k_yaw_p * yaw_err,
                                  -max_yaw_step, +max_yaw_step);

      // 2a) Align heading for forward motion
      if (std::abs(yaw_err) > yaw_tol) {
        RCLCPP_INFO(this->get_logger(),
                    "Heading align: curr=%.3f des=%.3f err=%.3f cmd=%.3f",
                    robot_yaw, goal_yaw, yaw_err, yaw_cmd);

        if (!send_delta_sync(0.0f, 0.0f, static_cast<float>(yaw_cmd))) {
          ++consecutive_failures;
          RCLCPP_WARN(this->get_logger(),
                      "Yaw align (path) step failed (consecutive=%d).",
                      consecutive_failures);

          if (check_abort("Yaw align (path)")) {
            break;
          }

          rate.sleep();
          continue;
        } else {
          last_success        = this->now();
          consecutive_failures = 0;
        }

        rate.sleep();
        continue;
      }

      // 2b) Move forward
      RCLCPP_INFO(this->get_logger(),
                  "Forward: +x=%.3f (toward target %.3f, %.3f)",
                  step_len, target.x(), target.y());

      if (!send_delta_sync(static_cast<float>(step_len), 0.0f, 0.0f)) {
        ++consecutive_failures;
        RCLCPP_WARN(this->get_logger(),
                    "Forward step failed (consecutive=%d).",
                    consecutive_failures);

        if (check_abort("Forward")) {
          break;
        }

        rate.sleep();
        continue;
      } else {
        last_success        = this->now();
        consecutive_failures = 0;
      }

      rate.sleep();
    }

    return false;
  }


  const std::string& ground_cloud_path() const { return ground_cloud_; }
  const std::string& offground_cloud_path() const { return offground_cloud_; }
  double resolution() const { return resolution_; }

  std::string saves_dir() const { return saves_dir_; }
  std::string load_save_file() const { return load_save_file_; }
  std::string load_map_save_file() const { return load_map_save_file_; }

private:
    rclcpp_action::Client<MoveRelativeXY>::SharedPtr client_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string ground_cloud_, offground_cloud_;
    double resolution_;
    
    std::string saves_dir_;
    std::string load_save_file_;
    std::string load_map_save_file_;
};

enum class UIState {
  DRAWING,
  RUNNING
};

int main(int argc, char** argv) {
  try {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveRelativeXYClient>();

    PointCloudHandler pch(
      node->ground_cloud_path(),
      node->offground_cloud_path(),
      node->resolution()
    );

    GridMap grid = pch.build_map(/*inflate=*/true, /*inflate_radius=*/0.25);

    std::cout << "Grid: " << grid.ny << " x " << grid.nx
              << "  res=" << grid.resolution << " m/cell\n";

    for (int i = 0; i < 10; ++i) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (!node->update_robot_loc_from_tf(grid, "world", "base_link")) {
      auto it = std::find_if(grid.cells.begin(), grid.cells.end(),
                             [](const Cell& c){ return c.state == FREE; });
      if (it != grid.cells.end()) grid.robot_loc = &(*it);
      else grid.robot_loc = &grid.cells.front();
      //grid.robot_loc = &grid.cells[grid.world2lin({5, 10})];
    }

    GridInterface ui(grid, "grid");
    ui.attach();

    std::vector<Eigen::Vector2d> path;
    UIState state = UIState::DRAWING;

    while (rclcpp::ok()) {
      rclcpp::spin_some(node);

      ui.refresh();

      int k = cv::waitKey(16);

      if (k == 's') {
        if (state == UIState::DRAWING) {
          ui.save_path(node->saves_dir());
          ui.clicked_cells_.clear();
          path.clear();
          ui.write_path(path);
          if (!path.empty()) {
            std::cout << "Path defined with " << path.size() << " points.\n";
            std::cout << "Starting path following (SYNC)...\n";

            bool ok = node->follow_path_in_grid(grid, ui, path);
            std::cout << "follow_path_in_grid returned: " << std::boolalpha << ok << "\n";

            if (ok) {
              std::cout << "Path following finished successfully.\n";
            }
            else {
              std::cout << "Path following failed.\n";
            }
            state = UIState::DRAWING;
            ui.path_.clear();
            path.clear();
            node->update_robot_loc_from_tf(grid, "world", "base_link");
          }
          else {
            std::cout << "No path defined.\n";
          }
        }
      }
      else if (k == 'l') {
        if (state == UIState::DRAWING) {
          ui.load_path(node->load_save_file());
        }
      }
      else if (k == 'd') {
        if (state == UIState::DRAWING) {
          ui.load_path_from_map_frame(node->load_map_save_file(), node->fiducial_yaw);
          //continue;
          ui.refresh();
          path.clear();
          ui.write_path(path);
          if (!path.empty()) {
            std::cout << "Path defined with " << path.size() << " points.\n";
            std::cout << "Starting path following (SYNC)...\n";

            bool ok = node->follow_waypoints(grid, ui, path);
            std::cout << "follow_waypoints returned: " << std::boolalpha << ok << "\n";

            if (ok) {
              std::cout << "Path following finished successfully.\n";
            }
            else {
              std::cout << "Path following failed.\n";
            }
            state = UIState::DRAWING;
            ui.path_.clear();
            path.clear();
            node->update_robot_loc_from_tf(grid, "world", "base_link");
          }
          else {
            std::cout << "No path defined.\n";
          }
        }
      }
      else if (k == 'c') {
        if (state == UIState::DRAWING) {
          ui.path_.clear();
          path.clear();
          node->update_robot_loc_from_tf(grid, "world", "base_link");
        }
      }
      else if (k == 27) {
        break;
      }
    }
    cv::destroyAllWindows();
    rclcpp::shutdown();
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }
  return 0;
}