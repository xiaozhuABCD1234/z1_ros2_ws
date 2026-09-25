// Copyright 2026 z1_ros2_ws contributors
//
// Official Unitree control law for the Z1 arm as a ros2_control controller:
//
//   tau = Kp * (q_desired - q) + Kd * (qdot_desired - qdot)
//         clamped to the joint's effort limit
//
// This mirrors `unitree_legged_control/src/unitree_joint_control_tool.cpp`
// (calcTorque = posStiffness*(targetPos-currentPos) +
//               velStiffness*(targetVel-currentVel) + targetTorque)
// including its torque clamping, and drives an effort command interface instead
// of a position one. The FollowJointTrajectory action interface is the same as
// joint_trajectory_controller, so MoveIt's controller mapping does not change.

#ifndef Z1_CONTROLLERS__Z1_JOINT_TRAJECTORY_CONTROLLER_HPP_
#define Z1_CONTROLLERS__Z1_JOINT_TRAJECTORY_CONTROLLER_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace z1_controllers
{

class Z1JointTrajectoryController : public controller_interface::ControllerInterface
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  Z1JointTrajectoryController() = default;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Waypoint
  {
    double t;                  // seconds from trajectory start
    std::vector<double> q;     // positions [rad]
    std::vector<double> v;     // velocities [rad/s], may be empty
  };

  struct Trajectory
  {
    std::vector<Waypoint> waypoints;
    double duration{0.0};
  };

  /// What update() wants the (non-realtime) action monitor to report.
  enum class Outcome
  {
    NONE,
    SUCCEEDED,
    CANCELED,
    PATH_TOLERANCE,
    GOAL_TIME,
  };

  // action callbacks
  rclcpp_action::GoalResponse on_goal(const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal);
  rclcpp_action::CancelResponse on_cancel(const std::shared_ptr<GoalHandle> goal_handle);
  void on_accepted(const std::shared_ptr<GoalHandle> goal_handle);

  // helpers
  bool build_trajectory(
    const trajectory_msgs::msg::JointTrajectory & msg,
    std::shared_ptr<const Trajectory> & trajectory, std::string & error);
  void interpolate(
    const Trajectory & trajectory, double t, std::vector<double> & q,
    std::vector<double> & v) const;
  /// Apply the torque law to every joint; false if a command interface refused
  /// the value (already latched in command_write_failed_).
  bool write_effort(const std::vector<double> & q, const std::vector<double> & v,
    const std::vector<double> & q_des, const std::vector<double> & v_des);
  void clamp_to_limits(std::vector<double> & q_des) const;
  /// Zero the integral terms (new trajectory / deactivate).
  void reset_integral();
  /// Stop tracking the active trajectory, hold `hold` and queue `outcome`.
  void finish_trajectory(Outcome outcome, const std::vector<double> & hold);
  void action_monitor();

  // configuration
  std::vector<std::string> joints_;
  std::vector<double> kp_;
  std::vector<double> kd_;
  std::vector<double> effort_limits_;
  std::vector<double> path_tolerance_;
  std::vector<double> goal_tolerance_;
  std::vector<double> lower_limits_;
  std::vector<double> upper_limits_;
  std::vector<double> integral_;   // integral of the position error (i_gain > 0 only)
  double boundary_margin_{0.02};   // rad, keep the reference off the hard stops
  double i_gain_{0.0};             // 1/s, integral action (0 = off, like the official yaml)
  double i_limit_{5.0};            // N*m, clamp of the integral state
  double i_deadband_{0.002};       // rad, do not integrate inside this band
  double i_velocity_gate_{0.05};   // rad/s, integrate only when nearly stopped
  double period_{0.0};             // s, last update() period
  bool path_diag_logged_{false};   // one-shot diagnostic for path tolerance
  static constexpr double windup_threshold_{0.15};  // rad, integrate only when close
  double goal_time_{0.0};
  double monitor_rate_{20.0};
  // Optional parking pose driven to on activation (slew-limited).
  std::vector<double> initial_position_;
  double initial_position_speed_{0.5};  // rad/s
  bool startup_active_{false};


  // runtime state
  size_t dof_{0};
  std::vector<double> hold_position_;
  // Scratch buffers for update(); sized once so the realtime loop never allocates.
  std::vector<double> q_;
  std::vector<double> qdot_;
  std::vector<double> q_des_;
  std::vector<double> v_des_;
  rclcpp::Time trajectory_start_;
  std::atomic<bool> has_trajectory_{false};
  std::atomic<bool> cancel_requested_{false};
  std::atomic<bool> command_write_failed_{false};
  std::atomic<Outcome> outcome_{Outcome::NONE};
  std::atomic<size_t> active_goal_id_{0};
  size_t finished_goal_id_{0};

  realtime_tools::RealtimeBuffer<std::shared_ptr<const Trajectory>> trajectory_buffer_;

  std::mutex goal_mutex_;
  std::shared_ptr<GoalHandle> active_goal_;

  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;
};

}  // namespace z1_controllers

#endif  // Z1_CONTROLLERS__Z1_JOINT_TRAJECTORY_CONTROLLER_HPP_
