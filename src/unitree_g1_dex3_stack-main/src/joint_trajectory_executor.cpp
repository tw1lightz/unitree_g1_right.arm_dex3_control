#include "rclcpp/rclcpp.hpp"

#include <unitree_hg/msg/low_cmd.hpp>
#include <unitree_hg/msg/motor_cmd.hpp>

#include <unitree_hg/msg/low_state.hpp>
#include <unitree_hg/msg/motor_state.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>


#include <array>
#include <vector>
#include <chrono>
#include <map>
#include <sstream>
#include <string>
#include <string_view>
#include <urdf/model.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>

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

namespace {

// Plan 04-01: consumed by Plan 04-02 validation and Plan 04-03 publish-loop reasoning; indices and names must stay in sync.
static constexpr std::array<JointIndex, 7> kRightArmJointIndices = {
  kRightShoulderPitch,
  kRightShoulderRoll,
  kRightShoulderYaw,
  kRightElbow,
  kRightWristRoll,
  kRightWristPitch,
  kRightWristYaw,
};

static constexpr std::array<std::string_view, 7> kRightArmJointNames = {
  "right_shoulder_pitch_joint",
  "right_shoulder_roll_joint",
  "right_shoulder_yaw_joint",
  "right_elbow_joint",
  "right_wrist_roll_joint",
  "right_wrist_pitch_joint",
  "right_wrist_yaw_joint",
};

inline bool isRightArmJoint(std::string_view name) {
  for (const auto& right_arm_name : kRightArmJointNames) {
    if (name == right_arm_name) {
      return true;
    }
  }
  return false;
}

}  // namespace

class JointTrajectoryExecutor : public rclcpp::Node {
public:
  JointTrajectoryExecutor()
  : Node("joint_trajectory_executor")
  {
    RCLCPP_INFO(this->get_logger(), "Joint Trajectory Executor Node Initialized");

    rclcpp::QoS qos_profile(10);
    qos_profile.best_effort();

    cmd_pub_ = this->create_publisher<unitree_hg::msg::LowCmd>("/arm_sdk", qos_profile);

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

        // Build KDL tree and right-arm chain for gravity compensation
        KDL::Tree kdl_tree;
        if (kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree)) {
          if (kdl_tree.getChain("torso_link", "right_wrist_yaw_link", kdl_chain_right_)) {
            // Gravity in torso_link frame: Z is up
            gravity_solver_ = std::make_unique<KDL::ChainDynParam>(
                kdl_chain_right_, KDL::Vector(0.0, 0.0, -9.81));
            gravity_enabled_ = true;
            RCLCPP_INFO(this->get_logger(),
                "KDL gravity compensation enabled: %u segments, %u joints",
                kdl_chain_right_.getNrOfSegments(), kdl_chain_right_.getNrOfJoints());
          } else {
            RCLCPP_WARN(this->get_logger(),
                "Failed to get KDL chain torso_link->right_wrist_yaw_link; gravity comp disabled");
          }
        } else {
          RCLCPP_WARN(this->get_logger(), "Failed to build KDL tree; gravity comp disabled");
        }
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
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_sub_;
  std::map<std::string, JointLimits> joint_limits_;
  std::vector<float> latest_joint_positions_;

  // KDL gravity compensation
  KDL::Chain kdl_chain_right_;
  std::unique_ptr<KDL::ChainDynParam> gravity_solver_;
  bool gravity_enabled_ = false;

  // KDL gravity torque correction (from calibrate_kdl_tau.py)
  // Wrist joints (4-6): scale=1 since KDL gives near-zero; only bias compensates
  const std::array<float, 7> gravity_scale_ = {1.5761f, 1.6540f, 2.1793f, 2.5543f, 1.0f, 1.0f, 1.0f};
  const std::array<float, 7> gravity_bias_  = {-0.0400f, 0.4672f, 0.1232f, 0.2478f, -0.0832f, 0.0251f, 0.0624f};

  // Compute gravity torques for 7 right-arm joints given their positions.
  std::vector<float> computeGravityTorques(const std::array<float, 7>& q_right_arm) {
    std::vector<float> torques(7, 0.0f);
    if (!gravity_enabled_) return torques;
    unsigned int nj = kdl_chain_right_.getNrOfJoints();
    KDL::JntArray q_kdl(nj);
    for (unsigned int i = 0; i < nj && i < 7; ++i) {
      q_kdl(i) = static_cast<double>(q_right_arm[i]);
    }
    KDL::JntArray gravity_torques(nj);
    if (gravity_solver_->JntToGravity(q_kdl, gravity_torques) == 0) {
      for (unsigned int i = 0; i < nj && i < 7; ++i) {
        torques[i] = gravity_scale_[i] * static_cast<float>(gravity_torques(i)) + gravity_bias_[i];
      }
    }
    return torques;
  }

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

    // Plan 04-02 D-03 stage 1: foreign-joint WARN + strip.
    std::vector<size_t> right_arm_columns_in_msg;
    right_arm_columns_in_msg.reserve(kRightArmJointNames.size());
    std::vector<std::string> filtered_joint_names;
    filtered_joint_names.reserve(kRightArmJointNames.size());
    std::vector<std::string> foreign_names;
    for (size_t k = 0; k < msg->joint_names.size(); ++k) {
      const auto& joint_name = msg->joint_names[k];
      if (isRightArmJoint(joint_name)) {
        right_arm_columns_in_msg.push_back(k);
        filtered_joint_names.push_back(joint_name);
      } else {
        foreign_names.push_back(joint_name);
      }
    }
    if (!foreign_names.empty()) {
      std::ostringstream foreign_oss;
      for (size_t k = 0; k < foreign_names.size(); ++k) {
        if (k > 0) foreign_oss << ", ";
        foreign_oss << foreign_names[k];
      }
      RCLCPP_WARN(this->get_logger(),
        "Trajectory contains %zu foreign (non-right-arm) joint(s): %s — stripping these columns and proceeding with the right-arm subset.",
        foreign_names.size(), foreign_oss.str().c_str());
    }

    // Plan 04-02 D-03 stage 2: completeness ERROR.
    std::vector<std::string> missing;
    for (const auto& right_arm_name : kRightArmJointNames) {
      bool found = false;
      for (const auto& filtered_name : filtered_joint_names) {
        if (filtered_name == right_arm_name) {
          found = true;
          break;
        }
      }
      if (!found) {
        missing.emplace_back(right_arm_name);
      }
    }
    if (!missing.empty()) {
      std::ostringstream missing_oss;
      for (size_t k = 0; k < missing.size(); ++k) {
        if (k > 0) missing_oss << ", ";
        missing_oss << missing[k];
      }
      RCLCPP_ERROR(this->get_logger(),
        "Trajectory missing %zu right-arm joint(s): %s — rejecting trajectory; no LowCmd published.",
        missing.size(), missing_oss.str().c_str());
      return;
    }


    // snapshot standing pose at callback entry — target for end-of-trajectory ramp
    std::vector<float> standing_pose;
    if (!latest_joint_positions_.empty()) {
      standing_pose = latest_joint_positions_;
    }

    auto start_time = this->now();

    auto publish_command_for_positions = [&](const std::vector<double>& positions) {
      unitree_hg::msg::LowCmd cmd_msg;

      cmd_msg.motor_cmd[JointIndex::kNotUsedJoint].q = 1.0f; // Full motion-sdk authority (matches free_arm_demo.py)
      // Fill all joints with latest state first
      // Plan 04-03 D-06 Option A: all 28 body joints locked with kp=60; non-right-arm joints held at latest_joint_positions_. Trajectory column override below drives right-arm q. Matches free_arm_demo.py coexistence pattern.
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
        bool is_wrist = (pair.first.find("wrist") != std::string::npos);
        cmd_msg.motor_cmd[idx].kp = is_wrist ? 40.0f : 100.0f;
        cmd_msg.motor_cmd[idx].kd = 5.0f;
        cmd_msg.motor_cmd[idx].tau = 0.f;
      }
      // Plan 04-02 D-05: walk only the right-arm columns from the original msg; map lookups are guarded by stage 2 above.
      for (size_t k = 0; k < right_arm_columns_in_msg.size(); ++k) {
        const size_t j = right_arm_columns_in_msg[k];
        auto target_joint_name = msg->joint_names[j];
        auto target_index = joint_name_to_index.at(target_joint_name);
        auto target_position = positions[j];
        auto lim = joint_limits_.at(target_joint_name);
        target_position = std::min(std::max(target_position, lim.lower), lim.upper);
        cmd_msg.motor_cmd[target_index].q = target_position;
      }

      // Gravity compensation: KDL feedforward torques for right-arm joints
      if (gravity_enabled_) {
        std::array<float, 7> q_ra;
        for (size_t k = 0; k < 7; ++k) {
          q_ra[k] = cmd_msg.motor_cmd[kRightArmJointIndices[k]].q;
        }
        auto grav_tau = computeGravityTorques(q_ra);
        for (size_t k = 0; k < 7; ++k) {
          cmd_msg.motor_cmd[kRightArmJointIndices[k]].tau = grav_tau[k];
        }
      }

      cmd_pub_->publish(cmd_msg);
    };

    auto sleep_until = [&](const rclcpp::Time& scheduled_time) {
      rclcpp::Duration wait_time = scheduled_time - this->now();
      if (wait_time > rclcpp::Duration::from_seconds(0)) {
        rclcpp::sleep_for(std::chrono::nanoseconds(wait_time.nanoseconds()));
      }
    };

    const int64_t trajectory_publish_period_ns = 4'000'000;

    if (!msg->points.empty() && !g_shutdown_requested.load()) {
      const auto& first_point = msg->points.front();
      sleep_until(start_time + first_point.time_from_start);
      publish_command_for_positions(first_point.positions);
    }

    // Plan 01-06: honor SIGINT/SIGTERM mid-trajectory by breaking out of
    // the waypoint loop at the next iteration. The end-of-trajectory ramp
    // then runs from the current trajectory point, producing a smooth release.
    for (size_t i = 1; i < msg->points.size() && !g_shutdown_requested.load(); ++i) {
      const auto& previous_point = msg->points[i - 1];
      const auto& point = msg->points[i];
      rclcpp::Time segment_start_time = start_time + previous_point.time_from_start;
      rclcpp::Time segment_end_time = start_time + point.time_from_start;
      const int64_t segment_duration_ns = (segment_end_time - segment_start_time).nanoseconds();

      if (segment_duration_ns > 0) {
        for (int64_t elapsed_ns = trajectory_publish_period_ns;
             elapsed_ns < segment_duration_ns && !g_shutdown_requested.load();
             elapsed_ns += trajectory_publish_period_ns)
        {
          const double t = static_cast<double>(elapsed_ns) / static_cast<double>(segment_duration_ns);
          std::vector<double> interpolated_positions = point.positions;
          for (size_t j = 0; j < interpolated_positions.size() && j < previous_point.positions.size(); ++j) {
            interpolated_positions[j] =
              (1.0 - t) * previous_point.positions[j] + t * point.positions[j];
          }
          sleep_until(segment_start_time + rclcpp::Duration(std::chrono::nanoseconds(elapsed_ns)));
          publish_command_for_positions(interpolated_positions);
        }
      }

      if (!g_shutdown_requested.load()) {
        sleep_until(segment_end_time);
        publish_command_for_positions(point.positions);
      }
    }

    RCLCPP_INFO(this->get_logger(), "Trajectory %zu waypoints executed; holding stiff at end-point.", msg->points.size());

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

    // 1 s hold-publish loop at 250 Hz — keeps publishing master=0.5 + endpoint q
    // so the arm has time to settle at the trajectory end-point.

    {
      const int hold_steps = 250;                            // 1.0 s @ 250 Hz
      const auto hold_sleep_ns = std::chrono::nanoseconds(4'000'000); // 4 ms
      for (int s = 0; s < hold_steps; ++s) {
        unitree_hg::msg::LowCmd hold_cmd;
        hold_cmd.motor_cmd[JointIndex::kNotUsedJoint].q = 1.0f; // full motion-sdk authority
        // Plan 04-03 D-06 Option A: all 28 body joints locked with kp=60 at trajectory end-point.
        for (const auto& pair : joint_name_to_index) {
          size_t idx = pair.second;
          hold_cmd.motor_cmd[idx].mode = 1;
          if (trajectory_endpoint.size() > idx) {
            hold_cmd.motor_cmd[idx].q = trajectory_endpoint[idx];
          } else {
            hold_cmd.motor_cmd[idx].q = 0.0f;
          }
          hold_cmd.motor_cmd[idx].dq = 0.f;
          bool is_wrist = (pair.first.find("wrist") != std::string::npos);
          hold_cmd.motor_cmd[idx].kp = is_wrist ? 40.0f : 100.0f;
          hold_cmd.motor_cmd[idx].kd = 5.0f;
          hold_cmd.motor_cmd[idx].tau = 0.f;
        }
        // Gravity compensation for hold
        if (gravity_enabled_) {
          std::array<float, 7> q_ra;
          for (size_t k = 0; k < 7; ++k) {
            q_ra[k] = hold_cmd.motor_cmd[kRightArmJointIndices[k]].q;
          }
          auto grav_tau = computeGravityTorques(q_ra);
          for (size_t k = 0; k < 7; ++k) {
            hold_cmd.motor_cmd[kRightArmJointIndices[k]].tau = grav_tau[k];
          }
        }
        cmd_pub_->publish(hold_cmd);
        rclcpp::sleep_for(hold_sleep_ns);
      }
    }
    RCLCPP_INFO(this->get_logger(), "Trajectory execution complete, ramping back to standing pose.");

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
      double value = (1.0 - t) * 1.0 + t * 0.0; // Linear interpolation from 1.0 (matching trajectory/hold) to 0.0
      unitree_hg::msg::LowCmd final_cmd;
      final_cmd.motor_cmd[JointIndex::kNotUsedJoint].q = static_cast<float>(value);
      // Plan 01-09: drive the arm explicitly from trajectory end-point to
      // standing snapshot under stiff servoing (kp/kd back to 60/1.5).
      // Frame 0 (t=0): q = ramp start = trajectory end-point (matches actual,
      // no jerk). Frame 150 (t=1): q = standing snapshot (arm is at standing).
      // Master switch then hands off to body controller with arm already there.
      // Plan 04-03 D-06 Option A: all 28 body joints locked with kp=60; q interpolates to standing_pose over 3 s.
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
        bool is_wrist = (pair.first.find("wrist") != std::string::npos);
        final_cmd.motor_cmd[idx].kp = is_wrist ? 40.0f : 100.0f;
        final_cmd.motor_cmd[idx].kd = 5.0f;
        final_cmd.motor_cmd[idx].tau = 0.f;
      }
      // Gravity compensation for ramp
      if (gravity_enabled_) {
        std::array<float, 7> q_ra;
        for (size_t k = 0; k < 7; ++k) {
          q_ra[k] = final_cmd.motor_cmd[kRightArmJointIndices[k]].q;
        }
        auto grav_tau = computeGravityTorques(q_ra);
        for (size_t k = 0; k < 7; ++k) {
          final_cmd.motor_cmd[kRightArmJointIndices[k]].tau = grav_tau[k];
        }
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
