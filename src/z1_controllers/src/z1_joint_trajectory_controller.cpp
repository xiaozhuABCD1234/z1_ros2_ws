// Copyright 2026 z1_ros2_ws contributors
//
// See include/z1_controllers/z1_joint_trajectory_controller.hpp for what this
// controller replicates (the official Unitree torque law).

#include "z1_controllers/z1_joint_trajectory_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"

namespace z1_controllers
{

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

controller_interface::CallbackReturn Z1JointTrajectoryController::on_init()
{
  auto node = get_node();
  const auto logger = node->get_logger();

  if (!node->has_parameter("joints"))
  {
    node->declare_parameter("joints", std::vector<std::string>{});
  }
  joints_ = node->get_parameter("joints").get_value<std::vector<std::string>>();
  if (joints_.empty())
  {
    RCLCPP_ERROR(logger, "Parameter 'joints' must list the controlled joints.");
    return controller_interface::CallbackReturn::ERROR;
  }
  dof_ = joints_.size();

  kp_.resize(dof_, 300.0);
  kd_.resize(dof_, 5.0);
  effort_limits_.resize(dof_, 30.0);
  path_tolerance_.resize(dof_, 0.2);
  goal_tolerance_.resize(dof_, 0.05);
  lower_limits_.assign(dof_, kNaN);
  upper_limits_.assign(dof_, kNaN);
  q_.assign(dof_, kNaN);
  qdot_.assign(dof_, kNaN);
  q_des_.assign(dof_, kNaN);
  v_des_.assign(dof_, 0.0);
  // Official Z1 joint limits (const.xacro), used as clamp defaults.
  const std::vector<std::pair<double, double>> default_limits = {
    {-2.61799, 2.61799},  // joint1
    {0.0, 2.96706},       // joint2
    {-2.87979, 0.0},      // joint3
    {-1.51844, 1.51844},  // joint4
    {-1.34390, 1.34390},  // joint5
    {-2.79253, 2.79253},  // joint6
  };
  for (size_t i = 0; i < dof_ && i < default_limits.size(); ++i)
  {
    lower_limits_[i] = default_limits[i].first;
    upper_limits_[i] = default_limits[i].second;
  }

  if (!node->has_parameter("boundary_margin"))
  {
    node->declare_parameter("boundary_margin", boundary_margin_);
  }
  boundary_margin_ = std::max(0.0, node->get_parameter("boundary_margin").get_value<double>());

  // Integral action. The effort interface leaves a steady-state offset: gravity
  // plus the URDF joint friction (damping/friction = 1.0) hold the arm a few
  // 1e-2 rad short of the reference, i.e. the simulation stops visibly off the
  // planned pose. The integral term removes exactly that constant disturbance.
  // Keep the gain below ~200 for a 250 Hz loop, and freeze it while the error is
  // large so a big move cannot wind it up.
  if (!node->has_parameter("i_gain"))
  {
    node->declare_parameter("i_gain", i_gain_);
  }
  if (!node->has_parameter("i_limit"))
  {
    node->declare_parameter("i_limit", i_limit_);
  }
  if (!node->has_parameter("i_deadband"))
  {
    node->declare_parameter("i_deadband", i_deadband_);
  }
  if (!node->has_parameter("i_velocity_gate"))
  {
    node->declare_parameter("i_velocity_gate", i_velocity_gate_);
  }
  i_gain_ = std::max(0.0, node->get_parameter("i_gain").get_value<double>());
  i_limit_ = std::max(0.0, node->get_parameter("i_limit").get_value<double>());
  i_deadband_ = std::max(0.0, node->get_parameter("i_deadband").get_value<double>());
  i_velocity_gate_ = std::max(0.0, node->get_parameter("i_velocity_gate").get_value<double>());
  integral_.assign(dof_, 0.0);

  for (size_t i = 0; i < dof_; ++i)
  {
    const std::string & joint = joints_[i];
    const std::string prefix = "gains." + joint + ".";
    if (!node->has_parameter(prefix + "p"))
    {
      node->declare_parameter(prefix + "p", kp_[i]);
    }
    if (!node->has_parameter(prefix + "d"))
    {
      node->declare_parameter(prefix + "d", kd_[i]);
    }
    kp_[i] = node->get_parameter(prefix + "p").get_value<double>();
    kd_[i] = node->get_parameter(prefix + "d").get_value<double>();

    // Torque clamp: the joint's URDF effort limit.
    const std::string limit_name = "effort_limits." + joint;
    if (!node->has_parameter(limit_name))
    {
      node->declare_parameter(limit_name, effort_limits_[i]);
    }
    effort_limits_[i] = node->get_parameter(limit_name).get_value<double>();

    const std::string constraint_prefix = "constraints." + joint + ".";
    if (!node->has_parameter(constraint_prefix + "trajectory"))
    {
      node->declare_parameter(constraint_prefix + "trajectory", path_tolerance_[i]);
    }
    if (!node->has_parameter(constraint_prefix + "goal"))
    {
      node->declare_parameter(constraint_prefix + "goal", goal_tolerance_[i]);
    }
    path_tolerance_[i] =
      node->get_parameter(constraint_prefix + "trajectory").get_value<double>();
    goal_tolerance_[i] = node->get_parameter(constraint_prefix + "goal").get_value<double>();

    // Joint position limits: the reference is clamped to stay `boundary_margin`
    // away from them (see clamp_to_limits). Defaults are the official Z1 limits
    // (z1_description/urdf/const.xacro); override with
    // position_limits.<joint>.{lower,upper} if needed.
    const std::string limits_prefix = "position_limits." + joint + ".";
    if (!node->has_parameter(limits_prefix + "lower"))
    {
      node->declare_parameter(limits_prefix + "lower", lower_limits_[i]);
    }
    if (!node->has_parameter(limits_prefix + "upper"))
    {
      node->declare_parameter(limits_prefix + "upper", upper_limits_[i]);
    }
    lower_limits_[i] = node->get_parameter(limits_prefix + "lower").get_value<double>();
    upper_limits_[i] = node->get_parameter(limits_prefix + "upper").get_value<double>();
  }

  if (!node->has_parameter("constraints.goal_time"))
  {
    node->declare_parameter("constraints.goal_time", goal_time_);
  }
  goal_time_ = node->get_parameter("constraints.goal_time").get_value<double>();

  if (!node->has_parameter("action_monitor_rate"))
  {
    node->declare_parameter("action_monitor_rate", monitor_rate_);
  }
  monitor_rate_ = node->get_parameter("action_monitor_rate").get_value<double>();

  if (!node->has_parameter("initial_position"))
  {
    node->declare_parameter("initial_position", std::vector<double>{});
  }
  initial_position_ = node->get_parameter("initial_position").get_value<std::vector<double>>();
  if (!initial_position_.empty() && initial_position_.size() != dof_)
  {
    RCLCPP_ERROR(
      logger, "initial_position has %zu entries, expected %zu; ignoring it.",
      initial_position_.size(), dof_);
    initial_position_.clear();
  }
  if (!node->has_parameter("initial_position_speed"))
  {
    node->declare_parameter("initial_position_speed", initial_position_speed_);
  }
  initial_position_speed_ = node->get_parameter("initial_position_speed").get_value<double>();

  RCLCPP_INFO(
    logger, "Z1 joint trajectory controller: %zu joints, Kp=%.1f Kd=%.1f (joint 0)",
    dof_, kp_[0], kd_[0]);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
Z1JointTrajectoryController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_)
  {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

controller_interface::InterfaceConfiguration
Z1JointTrajectoryController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_)
  {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  for (const auto & joint : joints_)
  {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return config;
}

controller_interface::CallbackReturn Z1JointTrajectoryController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto node = get_node();
  using namespace std::placeholders;

  trajectory_buffer_.writeFromNonRT(std::shared_ptr<const Trajectory>());

  action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
    node, node->get_name() + std::string("/follow_joint_trajectory"),
    std::bind(&Z1JointTrajectoryController::on_goal, this, _1, _2),
    std::bind(&Z1JointTrajectoryController::on_cancel, this, _1),
    std::bind(&Z1JointTrajectoryController::on_accepted, this, _1));

  if (monitor_rate_ <= 0.0)
  {
    monitor_rate_ = 20.0;
  }
  monitor_timer_ = node->create_wall_timer(
    std::chrono::duration<double>(1.0 / monitor_rate_),
    std::bind(&Z1JointTrajectoryController::action_monitor, this));

  hold_position_.assign(dof_, kNaN);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Z1JointTrajectoryController::on_activate(
  const rclcpp_lifecycle::State &)
{
  // Start from wherever the arm is and hold it there.
  hold_position_.assign(dof_, kNaN);
  for (size_t i = 0; i < dof_ && i < state_interfaces_.size(); ++i)
  {
    hold_position_[i] = state_interfaces_[i].get_optional().value_or(kNaN);
  }
  for (size_t i = 0; i < dof_ && i < command_interfaces_.size(); ++i)
  {
    if (!command_interfaces_[i].set_value(0.0))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Cannot write command interface '%s'.",
        command_interfaces_[i].get_name().c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  has_trajectory_ = false;
  cancel_requested_ = false;
  outcome_ = Outcome::NONE;
  // Drive to the parking pose if one is configured (see the "startup slew" in
  // update()); it keeps the arm strictly inside its joint limits, which MoveIt
  // requires for a start state (joint2's lower and joint3's upper limit are
  // exactly 0 rad in this arm, i.e. the zero pose sits on the boundary).
  startup_active_ = !initial_position_.empty();
  RCLCPP_INFO(get_node()->get_logger(), "Z1 joint trajectory controller active (torque PD).");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Z1JointTrajectoryController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  // Do not keep pushing torque into the hardware once we stop controlling it.
  for (size_t i = 0; i < dof_ && i < command_interfaces_.size(); ++i)
  {
    if (!command_interfaces_[i].set_value(0.0))
    {
      RCLCPP_WARN(
        get_node()->get_logger(), "Cannot reset command interface '%s'.",
        command_interfaces_[i].get_name().c_str());
      break;
    }
  }
  has_trajectory_ = false;
  reset_integral();
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    active_goal_.reset();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse Z1JointTrajectoryController::on_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowJointTrajectory::Goal> goal)
{
  auto logger = get_node()->get_logger();
  if (get_lifecycle_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    RCLCPP_ERROR(logger, "Rejecting goal: controller is not active.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  const auto & trajectory = goal->trajectory;
  if (trajectory.joint_names.size() != dof_)
  {
    RCLCPP_ERROR(
      logger, "Rejecting goal: trajectory has %zu joints, controller has %zu.",
      trajectory.joint_names.size(), dof_);
    return rclcpp_action::GoalResponse::REJECT;
  }
  for (const auto & name : joints_)
  {
    if (
      std::find(trajectory.joint_names.begin(), trajectory.joint_names.end(), name) ==
      trajectory.joint_names.end())
    {
      RCLCPP_ERROR(logger, "Rejecting goal: joint '%s' is missing.", name.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  if (trajectory.points.empty())
  {
    RCLCPP_ERROR(logger, "Rejecting goal: trajectory has no points.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Z1JointTrajectoryController::on_cancel(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  std::lock_guard<std::mutex> lock(goal_mutex_);
  if (active_goal_ && active_goal_ == goal_handle)
  {
    cancel_requested_ = true;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  return rclcpp_action::CancelResponse::REJECT;
}

void Z1JointTrajectoryController::on_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  auto logger = get_node()->get_logger();
  std::shared_ptr<const Trajectory> trajectory;
  std::string error;
  if (!build_trajectory(goal_handle->get_goal()->trajectory, trajectory, error))
  {
    RCLCPP_ERROR(logger, "Rejecting goal after acceptance: %s", error.c_str());
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    result->error_code = FollowJointTrajectory::Result::INVALID_JOINTS;
    result->error_string = error;
    goal_handle->abort(result);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (active_goal_)
    {
      auto preempted = std::make_shared<FollowJointTrajectory::Result>();
      preempted->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
      preempted->error_string = "Goal preempted by a newer goal.";
      active_goal_->abort(preempted);
    }
    active_goal_ = goal_handle;
  }

  // Command the position we are at right now as the trajectory's start, so the
  // first sample does not see a jump.
  trajectory_start_ = get_node()->now();
  trajectory_buffer_.writeFromNonRT(trajectory);
  cancel_requested_ = false;
  outcome_ = Outcome::NONE;
  has_trajectory_ = true;
  path_diag_logged_ = false;
  reset_integral();
  ++active_goal_id_;
  RCLCPP_INFO(
    logger, "Accepted trajectory: %zu waypoints, %.3f s.", trajectory->waypoints.size(),
    trajectory->duration);
}

// ---------------------------------------------------------------- trajectory

bool Z1JointTrajectoryController::build_trajectory(
  const trajectory_msgs::msg::JointTrajectory & msg,
  std::shared_ptr<const Trajectory> & out, std::string & error)
{
  // Map the incoming joint order onto our own.
  std::vector<size_t> incoming_to_local(msg.joint_names.size(), 0);
  for (size_t i = 0; i < msg.joint_names.size(); ++i)
  {
    auto it = std::find(joints_.begin(), joints_.end(), msg.joint_names[i]);
    if (it == joints_.end())
    {
      error = "unknown joint '" + msg.joint_names[i] + "'";
      return false;
    }
    incoming_to_local[i] = static_cast<size_t>(std::distance(joints_.begin(), it));
  }

  auto trajectory = std::make_shared<Trajectory>();
  trajectory->waypoints.reserve(msg.points.size());

  double previous_time = -1.0;
  for (const auto & point : msg.points)
  {
    if (point.positions.size() != msg.joint_names.size())
    {
      error = "waypoint without one position per joint";
      return false;
    }
    const bool has_velocities = point.velocities.size() == msg.joint_names.size();

    Waypoint waypoint;
    waypoint.t = static_cast<double>(point.time_from_start.sec) +
                 static_cast<double>(point.time_from_start.nanosec) * 1e-9;
    if (waypoint.t < previous_time)
    {
      error = "waypoint times are not monotonically increasing";
      return false;
    }
    previous_time = waypoint.t;
    waypoint.q.assign(dof_, 0.0);
    if (has_velocities)
    {
      waypoint.v.assign(dof_, 0.0);
    }
    for (size_t i = 0; i < incoming_to_local.size(); ++i)
    {
      const size_t local = incoming_to_local[i];
      waypoint.q[local] = point.positions[i];
      if (has_velocities)
      {
        waypoint.v[local] = point.velocities[i];
      }
    }
    trajectory->waypoints.push_back(std::move(waypoint));
  }

  if (trajectory->waypoints.empty())
  {
    error = "empty trajectory";
    return false;
  }
  // Trajectories that do not start at t = 0 must ramp from where we are now;
  // otherwise the very first sample would be a step, which trips the path
  // tolerance. (MoveIt always starts at t = 0, hand-written goals often do
  // not.)
  if (trajectory->waypoints.front().t > 0.0)
  {
    Waypoint start = trajectory->waypoints.front();
    start.t = 0.0;
    const bool has_velocities = !start.v.empty();
    start.q.assign(dof_, 0.0);
    if (has_velocities)
    {
      std::fill(start.v.begin(), start.v.end(), 0.0);
    }
    for (size_t i = 0; i < dof_; ++i)
    {
      const double measured =
        (i < state_interfaces_.size()) ? state_interfaces_[i].get_optional().value_or(kNaN) : kNaN;
      start.q[i] = std::isfinite(measured) ? measured : trajectory->waypoints.front().q[i];
    }
    trajectory->waypoints.insert(trajectory->waypoints.begin(), start);
  }
  trajectory->duration = trajectory->waypoints.back().t;
  out = trajectory;
  return true;
}

void Z1JointTrajectoryController::interpolate(
  const Trajectory & trajectory, double t, std::vector<double> & q,
  std::vector<double> & v) const
{
  const auto & points = trajectory.waypoints;

  // Before the start / after the end: hold the first / last waypoint.
  if (t <= points.front().t)
  {
    q = points.front().q;
    v.assign(dof_, 0.0);
    return;
  }
  if (t >= points.back().t)
  {
    q = points.back().q;
    v.assign(dof_, 0.0);
    return;
  }

  size_t i = 0;
  while (i + 1 < points.size() && points[i + 1].t < t)
  {
    ++i;
  }
  const auto & p0 = points[i];
  const auto & p1 = points[i + 1];
  const double h = p1.t - p0.t;
  if (h <= 0.0)
  {
    q = p1.q;
    v.assign(dof_, 0.0);
    return;
  }
  const double s = (t - p0.t) / h;

  // Cubic Hermite: zero velocities where the sender did not provide any.
  const double h00 = 2.0 * s * s * s - 3.0 * s * s + 1.0;
  const double h10 = s * s * s - 2.0 * s * s + s;
  const double h01 = -2.0 * s * s * s + 3.0 * s * s;
  const double h11 = s * s * s - s * s;
  const double dh00 = 6.0 * s * s - 6.0 * s;
  const double dh10 = 3.0 * s * s - 4.0 * s + 1.0;
  const double dh01 = -6.0 * s * s + 6.0 * s;
  const double dh11 = 3.0 * s * s - 2.0 * s;

  q.assign(dof_, 0.0);
  v.assign(dof_, 0.0);
  for (size_t j = 0; j < dof_; ++j)
  {
    const double v0 = p0.v.empty() ? 0.0 : p0.v[j];
    const double v1 = p1.v.empty() ? 0.0 : p1.v[j];
    q[j] = h00 * p0.q[j] + h10 * h * v0 + h01 * p1.q[j] + h11 * h * v1;
    v[j] = (dh00 * p0.q[j] + dh10 * h * v0 + dh01 * p1.q[j] + dh11 * h * v1) / h;
  }
}

// -------------------------------------------------------------------- update

controller_interface::return_type Z1JointTrajectoryController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  period_ = period.seconds();
  // The scratch buffers are members so that the realtime loop does not allocate.
  for (size_t i = 0; i < dof_; ++i)
  {
    q_[i] = (i < state_interfaces_.size())
              ? state_interfaces_[i].get_optional().value_or(kNaN)
              : kNaN;
    qdot_[i] = ((i + dof_) < state_interfaces_.size())
                 ? state_interfaces_[i + dof_].get_optional().value_or(kNaN)
                 : kNaN;
    q_des_[i] = kNaN;
    v_des_[i] = 0.0;
  }

  const auto trajectory = *trajectory_buffer_.readFromRT();

  if (cancel_requested_)
  {
    cancel_requested_ = false;
    finish_trajectory(Outcome::CANCELED, q_);
  }
  else if (has_trajectory_ && trajectory)
  {
    const double t = (time - trajectory_start_).seconds();
    if (t < trajectory->waypoints.front().t)
    {
      // Before the first waypoint: stay where we are instead of jumping to it.
      for (size_t i = 0; i < dof_; ++i)
      {
        if (!std::isfinite(hold_position_[i]))
        {
          hold_position_[i] = q_[i];
        }
        q_des_[i] = hold_position_[i];
      }
      return write_effort(q_, qdot_, q_des_, v_des_)
               ? controller_interface::return_type::OK
               : controller_interface::return_type::ERROR;
    }
    interpolate(*trajectory, t, q_des_, v_des_);
    clamp_to_limits(q_des_);

    // Path tolerance while moving, goal tolerance once the trajectory is over.
    bool path_violated = false;
    bool goal_violated = false;
    for (size_t i = 0; i < dof_; ++i)
    {
      const double error = std::abs(q_[i] - q_des_[i]);
      if (error > path_tolerance_[i])
      {
        path_violated = true;
        if (!path_diag_logged_)
        {
          RCLCPP_WARN(get_node()->get_logger(),
                      "path tolerance: %s q=%.4f q_des=%.4f err=%.4f tol=%.3f t=%.3f dur=%.3f",
                      joints_[i].c_str(), q_[i], q_des_[i], error, path_tolerance_[i], t,
                      trajectory->duration);
        }
      }
      if (t >= trajectory->duration && error > goal_tolerance_[i])
      {
        goal_violated = true;
      }
    }

    if (t >= trajectory->duration && !goal_violated)
    {
      finish_trajectory(Outcome::SUCCEEDED, q_des_);
    }
    else if (t >= trajectory->duration + std::max(goal_time_, 0.0) && goal_violated)
    {
      finish_trajectory(Outcome::GOAL_TIME, q_des_);
    }
    else if (path_violated && t < trajectory->duration)
    {
      finish_trajectory(Outcome::PATH_TOLERANCE, q_des_);
    }
  }
  else
  {
    // Holding: PD to the last commanded position, i.e. "stand still".
    for (size_t i = 0; i < dof_; ++i)
    {
      if (!std::isfinite(hold_position_[i]))
      {
        hold_position_[i] = q_[i];
      }
      q_des_[i] = hold_position_[i];
    }

    // Startup: slew the hold reference to the configured parking pose so the
    // arm does not slam into it at full torque.
    if (startup_active_)
    {
      const double max_step = std::max(initial_position_speed_, 0.0) * period.seconds();
      bool reached = true;
      for (size_t i = 0; i < dof_; ++i)
      {
        const double error = initial_position_[i] - hold_position_[i];
        if (std::abs(error) > max_step)
        {
          hold_position_[i] += std::copysign(max_step, error);
          reached = false;
        }
        else
        {
          hold_position_[i] = initial_position_[i];
        }
        q_des_[i] = hold_position_[i];
      }
      if (reached)
      {
        startup_active_ = false;
        RCLCPP_INFO(get_node()->get_logger(), "Parking pose reached; holding it.");
      }
    }
    clamp_to_limits(q_des_);
  }

  return write_effort(q_, qdot_, q_des_, v_des_)
           ? controller_interface::return_type::OK
           : controller_interface::return_type::ERROR;
}

void Z1JointTrajectoryController::finish_trajectory(
  Outcome outcome, const std::vector<double> & hold)
{
  has_trajectory_ = false;
  trajectory_buffer_.writeFromNonRT(std::shared_ptr<const Trajectory>());
  hold_position_ = hold;
  outcome_ = outcome;
}

void Z1JointTrajectoryController::clamp_to_limits(std::vector<double> & q_des) const
{
  // Keep the position reference off the hard stops. Without this, a joint that
  // settles exactly on its limit (joint2's lower / joint3's upper limit are 0
  // rad and the parking pose sits on them) ends up a few 1e-14 rad *outside*
  // the limits, and MoveIt's CheckStartStateBounds refuses to plan from such a
  // state at all - the arm becomes uncontrollable.
  for (size_t i = 0; i < dof_ && i < q_des.size(); ++i)
  {
    if (!std::isfinite(lower_limits_[i]) || !std::isfinite(upper_limits_[i]))
    {
      continue;
    }
    const double lower = lower_limits_[i] + boundary_margin_;
    const double upper = upper_limits_[i] - boundary_margin_;
    if (lower > upper)
    {
      continue;  // range shorter than the margin, do not fight it
    }
    q_des[i] = std::clamp(q_des[i], lower, upper);
  }
}

void Z1JointTrajectoryController::reset_integral()
{
  std::fill(integral_.begin(), integral_.end(), 0.0);
}

bool Z1JointTrajectoryController::write_effort(
  const std::vector<double> & q, const std::vector<double> & qdot,
  const std::vector<double> & q_des, const std::vector<double> & v_des)
{
  for (size_t i = 0; i < dof_ && i < command_interfaces_.size(); ++i)
  {
    if (!std::isfinite(q[i]) || !std::isfinite(q_des[i]))
    {
      continue;
    }
    const double position_error = q_des[i] - q[i];
    const double velocity_error = v_des[i] - (std::isfinite(qdot[i]) ? qdot[i] : 0.0);
    double torque = kp_[i] * position_error + kd_[i] * velocity_error;

    if (i_gain_ > 0.0 && period_ > 0.0 && std::abs(position_error) < windup_threshold_ &&
        std::abs(position_error) > i_deadband_ && std::abs(velocity_error) < i_velocity_gate_)
    {
      // Integrate only when the joint is close, not already inside the deadband,
      // and nearly stopped: otherwise the integral fights dry friction or a
      // contact and turns into a limit cycle (visible jitter).
      integral_[i] = std::clamp(integral_[i] + position_error * period_, -i_limit_, i_limit_);
      torque += i_gain_ * integral_[i];
    }
    else if (i_gain_ > 0.0)
    {
      torque += i_gain_ * integral_[i];
    }

    // Official law clamps the torque to the joint's effort limit.
    torque = std::clamp(torque, -effort_limits_[i], effort_limits_[i]);
    RCLCPP_INFO_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "DIAG %s: q=%.4f q_des=%.4f e=%.4f tau=%.2f i=%.3f",
      joints_[i].c_str(), q[i], q_des[i], position_error, torque,
      i_gain_ > 0.0 ? integral_[i] : 0.0);
    if (!command_interfaces_[i].set_value(torque))
    {
      command_write_failed_ = true;
      return false;
    }
  }
  return true;
}

// ------------------------------------------------------------- action monitor

void Z1JointTrajectoryController::action_monitor()
{
  if (command_write_failed_.exchange(false))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to write an effort command interface; is the hardware still active?");
  }

  const auto outcome = outcome_.load();
  if (outcome == Outcome::NONE)
  {
    return;
  }
  if (active_goal_id_.load() == finished_goal_id_)
  {
    return;  // already reported
  }

  std::shared_ptr<GoalHandle> goal;
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    goal = active_goal_;
    active_goal_.reset();
  }
  if (!goal)
  {
    outcome_ = Outcome::NONE;
    return;
  }
  finished_goal_id_ = active_goal_id_.load();

  auto result = std::make_shared<FollowJointTrajectory::Result>();
  switch (outcome)
  {
    case Outcome::SUCCEEDED:
      result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
      result->error_string = "Trajectory executed within tolerances.";
      goal->succeed(result);
      break;
    case Outcome::CANCELED:
      result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
      result->error_string = "Goal canceled; holding position.";
      goal->canceled(result);
      break;
    case Outcome::PATH_TOLERANCE:
      result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
      result->error_string = "Path tolerance violated.";
      goal->abort(result);
      break;
    case Outcome::GOAL_TIME:
      result->error_code = FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
      result->error_string = "Goal not reached within goal_time.";
      goal->abort(result);
      break;
    default:
      break;
  }
  outcome_ = Outcome::NONE;
}

}  // namespace z1_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  z1_controllers::Z1JointTrajectoryController, controller_interface::ControllerInterface)
