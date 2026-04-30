#include "rclcpp/rclcpp.hpp"

#include <unitree_hg/msg/low_cmd.hpp>
#include <unitree_hg/msg/motor_cmd.hpp>

#include <unitree_hg/msg/low_state.hpp>
#include <unitree_hg/msg/motor_state.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <std_msgs/msg/bool.hpp>

#include <vector>
#include <chrono>
#include <map>
#include <string>
#include <urdf/model.h>

#include <atomic>
#include <csignal>

#include <g1_dex3_joint_defs.hpp>

using namespace std::chrono_literals;

// Set by SIGINT/SIGTERM handler so (a) main()'s polling loop can exit
// cleanly even from idle state, and (b) trajectoryCallback's waypoint
// loop can break out at the next iteration and fall through to the
// existing end-of-trajectory ramp for a smooth in-flight release. The
// originally-planned separate 3-second ramp from main() was removed in
// Plan 01-06 because it stole control authority back from the body
// controller and jerked the arm to q=0. See Plans 01-04 and 01-06.
static std::atomic<bool> g_shutdown_requested{false};
static void executor_signal_handler(int /*sig*/) {
  g_shutdown_requested.store(true);
}

// Struct to hold joint limits
struct JointLimits {
  double lower;
  double upper;
  double velocity;
  double effort;
};

class JointTrajectoryExecutor : public rclcpp::Node {
public:
  JointTrajectoryExecutor()
  : Node("joint_trajectory_executor")
  {
    RCLCPP_INFO(this->get_logger(), "Joint Trajectory Executor Node Initialized");

    rclcpp::QoS qos_profile(10);
    qos_profile.best_effort();

    cmd_pub_ = this->create_publisher<unitree_hg::msg::LowCmd>("/arm_sdk", qos_profile);
    left_hand_pub_ = this->create_publisher<std_msgs::msg::Bool>("/dex3/left/command", 10);
    right_hand_pub_ = this->create_publisher<std_msgs::msg::Bool>("/dex3/right/command", 10);

    traj_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/joint_trajectory_targets", 10,
      std::bind(&JointTrajectoryExecutor::trajectoryCallback, this, std::placeholders::_1));

    lowstate_sub_ = this->create_subscription<unitree_hg::msg::LowState>(
      "/lf/lowstate", 10,
      std::bind(&JointTrajectoryExecutor::lowstateCallback, this, std::placeholders::_1));

    // Load URDF and parse joint limits
    std::string urdf_xml;
        auto client = this->create_client<rcl_interfaces::srv::GetParameters>("/robot_state_publisher/get_parameters");
    while (!client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for /robot_state_publisher service...");
    }
    
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back("robot_description");

    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
      auto response = future.get();
      if (response->values.size() == 1 && response->values[0].type == rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
        urdf_xml = response->values[0].string_value;
      } else {
        RCLCPP_FATAL(this->get_logger(), "robot_description not found in /robot_state_publisher");
        rclcpp::shutdown();
        return;
      }
    } else {
      RCLCPP_FATAL(this->get_logger(), "Failed to connect to /robot_state_publisher/get_parameters service");
      rclcpp::shutdown();
      return;
    }

    if (!urdf_xml.empty()) {
      urdf::Model urdf_model;
      if (urdf_model.initString(urdf_xml)) {
        for (const auto& joint_pair : urdf_model.joints_) {
          const auto& joint = joint_pair.second;
          if (joint->type != urdf::Joint::REVOLUTE && joint->type != urdf::Joint::PRISMATIC) continue;
          if (!joint->limits) continue;
          JointLimits lim;
          lim.lower = joint->limits->lower;
          lim.upper = joint->limits->upper;
          lim.velocity = joint->limits->velocity;
          lim.effort = joint->limits->effort;
          joint_limits_[joint->name] = lim;
        }
        RCLCPP_INFO(this->get_logger(), "Loaded joint limits from URDF");
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to parse URDF for joint limits");
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "robot_description parameter is empty, joint limits not loaded");
    }
  }

  ~JointTrajectoryExecutor() override {
    RCLCPP_INFO(this->get_logger(), "Shutting down Joint Trajectory Executor Node");

    // Reaffirm body-controller authority on shutdown. In the normal
    // SIGINT/SIGTERM path, trajectoryCallback's own end-of-trajectory ramp
    // (or the new break-out path added in Plan 01-06) has already left
    // kNotUsedJoint.q at 0.0, so this is a redundant-but-harmless final
    // assertion. On a forced exit (SIGKILL, std::terminate) this destructor
    // may not run at all; arm_sdk's own timeout takes over in that case.
    unitree_hg::msg::LowCmd final_cmd;
    final_cmd.motor_cmd[JointIndex::kNotUsedJoint].q = 0.0f;
    cmd_pub_->publish(final_cmd);
  }

private:
  rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr left_hand_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr right_hand_pub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_sub_;
  std::map<std::string, JointLimits> joint_limits_;
  std::vector<float> latest_joint_positions_;

  void lowstateCallback(const unitree_hg::msg::LowState::SharedPtr msg) {
    // Store latest positions for all arm joints
    latest_joint_positions_.resize(joint_name_to_index.size(), 0.0f);
    for (const auto& pair : joint_name_to_index) {
      size_t idx = pair.second;
      if (idx < msg->motor_state.size()) {
        latest_joint_positions_[idx] = msg->motor_state[idx].q;
      }
    }
  }

  void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
    // Find out which hand to use based on the joint names
    if (msg->joint_names.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Received empty joint names in trajectory message");
      return;
    }
    auto hand_it = std::find_if(msg->joint_names.begin(), msg->joint_names.end(),
      [](const std::string& name) {
        return name.find("left") != std::string::npos || name.find("right") != std::string::npos;
      });
    if (hand_it == msg->joint_names.end()) {
      RCLCPP_ERROR(this->get_logger(), "No side found in trajectory message");
      return;
    }
    bool is_left_hand = hand_it->find("left") != std::string::npos;
    RCLCPP_INFO(this->get_logger(), "Executing trajectory for %s hand", is_left_hand ? "left" : "right");
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hand_cmd_pub = is_left_hand ? left_hand_pub_ : right_hand_pub_;

    // Plan 01-09: snapshot standing pose at callback entry. Arm is guaranteed
    // to be at standing here (either previous trajectory's ramp settled it
    // back, or robot just booted with body controller holding standing). This
    // snapshot is the target the end-of-trajectory ramp will drive toward.
    std::vector<float> standing_pose;
    if (!latest_joint_positions_.empty()) {
      standing_pose = latest_joint_positions_;
    }

    // Start with preparing the hand, open it fully
    std_msgs::msg::Bool hand_cmd;
    hand_cmd.data = false; // Open hand command
    hand_cmd_pub->publish(hand_cmd); // Open hand command
    // Wait for the hand to open
    rclcpp::sleep_for(1s);  // Wait for the hand to open
    RCLCPP_INFO(this->get_logger(), "Hand opened fully, starting trajectory execution");

    auto start_time = this->now();

    // Plan 01-06: honor SIGINT/SIGTERM mid-trajectory by breaking out of
    // the waypoint loop at the next iteration. The trailing hand-close +
    // 1s end-of-trajectory ramp then runs from the current trajectory
    // point, producing a smooth release without re-grabbing authority.
    for (size_t i = 0; i < msg->points.size() && !g_shutdown_requested.load(); ++i) {
      const auto& point = msg->points[i];
      unitree_hg::msg::LowCmd cmd_msg;

      cmd_msg.motor_cmd[JointIndex::kNotUsedJoint].q = 0.5f; // Full transition speed for trajectory following
      // Fill all joints with latest state first
      for (const auto& pair : joint_name_to_index) {
        size_t idx = pair.second;
        // Plan 01-10: enable PD control mode (was implicitly mode=0 before,
        // which caused arm_sdk to ignore q/kp/kd; reference: right_arm_mode.py).
        cmd_msg.motor_cmd[idx].mode = 1;
        if (latest_joint_positions_.size() > idx) {
          cmd_msg.motor_cmd[idx].q = latest_joint_positions_[idx];
        } else {
          cmd_msg.motor_cmd[idx].q = 0.0f;
        }
        cmd_msg.motor_cmd[idx].dq = 0.f;
        cmd_msg.motor_cmd[idx].kp = 60.0f;
        cmd_msg.motor_cmd[idx].kd = 1.5f;
        cmd_msg.motor_cmd[idx].tau = 0.f;
      }
      // Overwrite with trajectory values for joints present in this point
      for (size_t j = 0; j < point.positions.size(); ++j) {
        auto target_joint_name = msg->joint_names[j];
        auto target_index = joint_name_to_index.at(target_joint_name);
        auto target_position = point.positions[j];
        auto lim = joint_limits_.at(target_joint_name);
        target_position = std::min(std::max(target_position, lim.lower), lim.upper);
        cmd_msg.motor_cmd[target_index].q = target_position;
      }

      // Wait until the scheduled time for this point
      rclcpp::Time scheduled_time = start_time + point.time_from_start;
      rclcpp::Duration wait_time = scheduled_time - this->now();
      if (wait_time > rclcpp::Duration::from_seconds(0)) {
        rclcpp::sleep_for(std::chrono::nanoseconds(wait_time.nanoseconds()));
      }

      cmd_pub_->publish(cmd_msg);
    }

    RCLCPP_INFO(this->get_logger(), "Trajectory point %zu executed; holding stiff at end-point while hand closes.", msg->points.size());

    // Plan 01-12: capture trajectory end-point from the last waypoint.
    // latest_joint_positions_ is STALE here — single-threaded executor
    // blocks lowstateCallback for the entire trajectoryCallback, so it
    // still holds the standing pose from callback entry. Using it in
    // the hold loop or ramp was commanding the arm back to standing.
    std::vector<float> trajectory_endpoint;
    if (!standing_pose.empty()) {
      trajectory_endpoint = standing_pose;  // baseline for non-trajectory joints
    } else {
      trajectory_endpoint.resize(29, 0.0f);
    }
    if (!msg->points.empty()) {
      const auto& last_point = msg->points.back();
      for (size_t j = 0; j < last_point.positions.size() && j < msg->joint_names.size(); ++j) {
        auto it = joint_name_to_index.find(msg->joint_names[j]);
        if (it != joint_name_to_index.end()) {
          size_t idx = it->second;
          if (idx < trajectory_endpoint.size()) {
            auto lim = joint_limits_.at(msg->joint_names[j]);
            double pos = std::min(std::max(last_point.positions[j], lim.lower), lim.upper);
            trajectory_endpoint[idx] = static_cast<float>(pos);
          }
        }
      }
    }

    // Plan 01-11: close the publish gap. The two prior sleep_for(1s) calls
    // (one to settle the last waypoint, one to wait for hand close) left the
    // executor silent for 2 s right when smoothness mattered most -- firmware
    // had no master/q from us during that window and started dragging the arm
    // toward standing on its own. Replace with a single 1 s hold-publish loop
    // that keeps publishing master=0.5 + q=latest + mode=1 + kp/kd at 250 Hz.
    // Hand close runs in parallel (separate publisher) starting at frame 0.
    hand_cmd.data = true;
    hand_cmd_pub->publish(hand_cmd);

    {
      const int hold_steps = 250;                            // 1.0 s @ 250 Hz
      const auto hold_sleep_ns = std::chrono::nanoseconds(4'000'000); // 4 ms
      for (int s = 0; s < hold_steps; ++s) {
        unitree_hg::msg::LowCmd hold_cmd;
        hold_cmd.motor_cmd[JointIndex::kNotUsedJoint].q = 0.5f; // full planner authority
        for (const auto& pair : joint_name_to_index) {
          size_t idx = pair.second;
          hold_cmd.motor_cmd[idx].mode = 1;
          if (trajectory_endpoint.size() > idx) {
            hold_cmd.motor_cmd[idx].q = trajectory_endpoint[idx];
          } else {
            hold_cmd.motor_cmd[idx].q = 0.0f;
          }
          hold_cmd.motor_cmd[idx].dq = 0.f;
          hold_cmd.motor_cmd[idx].kp = 60.0f;
          hold_cmd.motor_cmd[idx].kd = 1.5f;
          hold_cmd.motor_cmd[idx].tau = 0.f;
        }
        cmd_pub_->publish(hold_cmd);
        rclcpp::sleep_for(hold_sleep_ns);
      }
    }
    RCLCPP_INFO(this->get_logger(), "Hand closed; trajectory execution complete, returning to default pose.");

    // Plan 01-12: use the trajectory end-point (computed from the last
    // waypoint above) as the ramp's starting pose, not the stale
    // latest_joint_positions_.
    std::vector<float> ramp_start_positions = trajectory_endpoint;

    // Smoothly interpolate final_cmd.motor_cmd[JointIndex::kNotUsedJoint].q from 1.0 to 0.0
    // Plan 01-08: 3 s ramp duration preserved. Plan 01-09 drives an explicit
    // q interpolation from trajectory end-point to standing snapshot.
    // Plan 01-11: bump steps 150 -> 750 so ramp publishes at 250 Hz (was 50 Hz),
    // matching reference smooth_exit cadence so firmware cannot wedge between frames.
    const double interp_duration = 3.0; // seconds
    const int interp_steps = 750;
    auto sleep_ns = std::chrono::nanoseconds(static_cast<int64_t>((interp_duration / interp_steps) * 1e9));
    for (int step = 0; step <= interp_steps; ++step) {
      double t = static_cast<double>(step) / interp_steps;
      double value = (1.0 - t) * 1.0 + t * 0.0; // Linear interpolation
      unitree_hg::msg::LowCmd final_cmd;
      final_cmd.motor_cmd[JointIndex::kNotUsedJoint].q = static_cast<float>(value);
      // Plan 01-09: drive the arm explicitly from trajectory end-point to
      // standing snapshot under stiff servoing (kp/kd back to 60/1.5).
      // Frame 0 (t=0): q = ramp start = trajectory end-point (matches actual,
      // no jerk). Frame 150 (t=1): q = standing snapshot (arm is at standing).
      // Master switch then hands off to body controller with arm already there.
      for (const auto& pair : joint_name_to_index) {
        size_t idx = pair.second;
        // Plan 01-10: enable PD control mode here too, so the q-interpolation
        // computed below is actually tracked stiffly by the motor controllers.
        final_cmd.motor_cmd[idx].mode = 1;
        if (ramp_start_positions.size() > idx && standing_pose.size() > idx) {
          final_cmd.motor_cmd[idx].q = static_cast<float>(
            (1.0 - t) * ramp_start_positions[idx] + t * standing_pose[idx]);
        } else if (latest_joint_positions_.size() > idx) {
          final_cmd.motor_cmd[idx].q = latest_joint_positions_[idx];
        } else {
          final_cmd.motor_cmd[idx].q = 0.0f;
        }
        final_cmd.motor_cmd[idx].dq = 0.f;
        final_cmd.motor_cmd[idx].kp = 60.0f;
        final_cmd.motor_cmd[idx].kd = 1.5f;
        final_cmd.motor_cmd[idx].tau = 0.f;
      }
      cmd_pub_->publish(final_cmd);
      rclcpp::sleep_for(sleep_ns);
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Replace rclcpp's default SIGINT handler so we can perform a graceful
  // arm-control release before the publisher is torn down. See Plan 01-04.
  rclcpp::uninstall_signal_handlers();
  std::signal(SIGINT, executor_signal_handler);
  std::signal(SIGTERM, executor_signal_handler);

  auto node = std::make_shared<JointTrajectoryExecutor>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  while (rclcpp::ok() && !g_shutdown_requested.load()) {
    exec.spin_some(std::chrono::milliseconds(50));
  }

  // SIGINT/SIGTERM received (or rclcpp::ok went false). The trajectory-end
  // ramp inside trajectoryCallback (line ~249) is the only graceful release
  // we need; if a callback was running, its ramp has already executed by
  // the time we get here. The destructor's instantaneous q=0.0 publish is
  // a harmless reaffirmation in idle state. See Plan 01-06.
  rclcpp::shutdown();
  return 0;
}
