// Copyright 2026 the z1_ros2_ws authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "z1_ros2_control/z1_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace z1_ros2_control
{

namespace
{
/// Cap on datagrams consumed per pump, so a flooded socket cannot stall the
/// worker thread.
constexpr int kMaxDrainPerCycle = 16;

/// How far the commanded joint position may sit from the measured one before
/// `set_fsm_state JOINTCTRL` refuses to enter joint control: past this, entering
/// would be a step input rather than a resume. 0.05 rad is the same "close
/// enough" the trajectory controller's goal tolerance uses.
constexpr double kJointCtrlRejoinTolerance = 0.05;

/// How long the lifecycle callbacks sleep between polls of the async thread.
constexpr auto kPollInterval = std::chrono::milliseconds(2);

/// The label handshake between the service callbacks and the I/O path must stay
/// lock-free: it runs on the async worker thread. On x86-64 it does, but a
/// standard library that decided otherwise would silently put a mutex in the
/// realtime path, so check it at build time.
static_assert(
  std::atomic<char>::is_always_lock_free,
  "the label handshake must not be able to take a lock on the worker thread");

int64_t steady_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::steady_clock::now().time_since_epoch())
    .count();
}

/// `std::stoi` that reports failure instead of throwing into ros2_control.
bool parse_int(const std::string & text, int & out)
{
  try {
    size_t consumed = 0;
    const int value = std::stoi(text, &consumed);
    if (consumed != text.size()) {
      return false;
    }
    out = value;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

std::string trim(const std::string & text)
{
  const auto begin = text.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = text.find_last_not_of(" \t\r\n");
  return text.substr(begin, end - begin + 1);
}

/// Keeps the socket and `cmd_` single-owner for the duration of one pump.
///
/// read() runs on the async worker, while a service callback runs on the
/// controller_manager's executor and a lifecycle callback runs on whatever thread
/// drove the transition - so two of them really can be inside pump_once() at the
/// same time. That is not merely a duplicate-send problem: `assemble_frame()`
/// writes `cmd_` field by field, so a datagram built while the other thread is
/// rewriting it would carry a *different command*, not a stale one.
///
/// Skipping instead of waiting is deliberate: blocking here would put a lock in
/// the I/O path, and every caller either retries (the wait loops) or simply
/// gets the other thread's frame (read()/write()).
class IoGuard
{
public:
  explicit IoGuard(std::atomic<bool> & busy)
  : busy_(busy), owned_(!busy.exchange(true, std::memory_order_acq_rel))
  {
  }

  ~IoGuard()
  {
    if (owned_) {
      busy_.store(false, std::memory_order_release);
    }
  }

  IoGuard(const IoGuard &) = delete;
  IoGuard & operator=(const IoGuard &) = delete;

  bool owned() const { return owned_; }

private:
  std::atomic<bool> & busy_;
  bool owned_;
};
}  // namespace

int Z1System::motor_index_for_joint(const std::string & joint_name)
{
  if (joint_name.size() == 6 && joint_name.compare(0, 5, "joint") == 0 &&
      joint_name[5] >= '1' && joint_name[5] <= '6') {
    return joint_name[5] - '1';
  }
  // Unitree's own ROS 1 driver calls this joint "jointGripper" and the SDK calls
  // the motor "the 7th one"; it is motor slot 6 in both cases.
  if (joint_name == "jointGripper") {
    return static_cast<int>(kGripperIndex);
  }
  return -1;
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

hardware_interface::CallbackReturn Z1System::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  const auto & info = get_hardware_info();
  std::string error;

  if (info.type != "system") {
    RCLCPP_ERROR(
      get_logger(), "'%s' must be declared as type=\"system\", got \"%s\".", info.name.c_str(),
      info.type.c_str());
    return CallbackReturn::ERROR;
  }

  if (!read_parameters(error)) {
    RCLCPP_ERROR(get_logger(), "%s", error.c_str());
    return CallbackReturn::ERROR;
  }

  // Validate the joint set. Every arm motor slot we command has to be wired to a
  // command interface: `command_position_` for a joint that the URDF forgot would
  // stay at 0 forever and we would command the arm to its zero pose.
  std::array<bool, kMotorCount> slots_covered{};
  has_gripper_ = false;
  arm_joint_names_.clear();

  for (const auto & joint : info.joints) {
    const int index = motor_index_for_joint(joint.name);
    if (index < 0) {
      RCLCPP_ERROR(
        get_logger(), "'%s': unknown joint '%s'; expected joint1..joint6 or jointGripper.",
        info.name.c_str(), joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    if (slots_covered[static_cast<size_t>(index)]) {
      RCLCPP_ERROR(get_logger(), "'%s': joint '%s' declared twice.", info.name.c_str(),
                   joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    slots_covered[static_cast<size_t>(index)] = true;
    if (index == static_cast<int>(kGripperIndex)) {
      has_gripper_ = true;
    } else {
      arm_joint_names_.push_back(joint.name);
    }
  }

  for (size_t i = 0; i < kArmJointCount; ++i) {
    if (!slots_covered[i]) {
      RCLCPP_ERROR(
        get_logger(), "'%s': motor slot %zu has no joint; joint%zu is required.", info.name.c_str(),
        i, i + 1);
      return CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(
    get_logger(), "'%s': %zu arm joints%s, controller at %s:%u, own port %u.", info.name.c_str(),
    arm_joint_names_.size(),
    has_gripper_ ? " + gripper" : " (no gripper; motor slot 6 is held at its measured angle)",
    ctrl_ip_.c_str(), ctrl_port_, own_port_);

  // No check on info.rw_rate: for an async component ros2_control replaces it with
  // the controller_manager's update_rate before on_init() ever runs, so it is
  // never the URDF's value and never 0. The rate that matters - the SendCmd rate
  // z1_ctrl sees - is measured and logged by report_read_rate() instead.

  return CallbackReturn::SUCCESS;
}

bool Z1System::read_parameters(std::string & error)
{
  const auto & parameters = get_hardware_info().hardware_parameters;

  const auto get = [&parameters](const std::string & key) -> const std::string * {
    const auto it = parameters.find(key);
    return it == parameters.end() ? nullptr : &it->second;
  };

  if (const auto * value = get("ctrl_ip")) {
    ctrl_ip_ = *value;
  }

  const auto parse_port = [&](const char * key, uint16_t & out) -> bool {
    const auto * value = get(key);
    if (value == nullptr) {
      return true;
    }
    int parsed = 0;
    if (!parse_int(*value, parsed) || parsed <= 0 || parsed > 65535) {
      error = std::string("parameter '") + key + "' is not a valid port: '" + *value + "'";
      return false;
    }
    out = static_cast<uint16_t>(parsed);
    return true;
  };
  if (!parse_port("ctrl_port", ctrl_port_) || !parse_port("own_port", own_port_)) {
    return false;
  }

  const auto parse_positive = [&](const char * key, int & out) -> bool {
    const auto * value = get(key);
    if (value == nullptr) {
      return true;
    }
    int parsed = 0;
    if (!parse_int(*value, parsed) || parsed <= 0) {
      error = std::string("parameter '") + key + "' must be a positive integer: '" + *value + "'";
      return false;
    }
    out = parsed;
    return true;
  };
  if (!parse_positive("disconnect_timeout_ms", disconnect_timeout_ms_) ||
      !parse_positive("temperature_limit", temperature_limit_) ||
      !parse_positive("fsm_timeout_ms", fsm_timeout_ms_)) {
    return false;
  }

  if (const auto * value = get("activate_fsm_sequence")) {
    activate_sequence_.clear();
    size_t begin = 0;
    for (;;) {
      const auto comma = value->find(',', begin);
      const std::string token =
        trim(value->substr(begin, comma == std::string::npos ? std::string::npos : comma - begin));
      if (!token.empty()) {
        UNITREE_ARM::ArmFSMState state{};
        if (!parseFsmStateName(token, state)) {
          error = "activate_fsm_sequence contains an unknown state '" + token + "'";
          return false;
        }
        if (!isSafeFsmState(state)) {
          // TOSTATE/SAVESTATE/TEACH/TEACHREPEAT need a label and CARTESIAN needs
          // posture deltas; neither is available before the controllers are up.
          error = "activate_fsm_sequence may only contain " + std::string(safeFsmStates()) +
                  "; '" + token + "' is reached through the set_fsm_state service instead.";
          return false;
        }
        activate_sequence_.push_back(state);
      }
      if (comma == std::string::npos) {
        break;
      }
      begin = comma + 1;
    }
    if (activate_sequence_.empty()) {
      error = "activate_fsm_sequence is empty; expected " + std::string(safeFsmStates());
      return false;
    }
  }

  if (const auto * value = get("deactivate_fsm")) {
    if (*value == "passive") {
      deactivate_to_passive_ = true;
    } else if (*value == "hold") {
      deactivate_to_passive_ = false;
    } else {
      error = "parameter 'deactivate_fsm' must be 'passive' or 'hold', got '" + *value + "'";
      return false;
    }
  }

  return true;
}

// ---------------------------------------------------------------------------
// Interfaces
// ---------------------------------------------------------------------------

// The InterfaceDescription-based replacements for this pair cannot wire up a
// component's own value pointers (there is no public way to assign them), and
// upstream's own reference component still overrides these two, so we do too.
// See z1_ros2_control/README.md.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

std::vector<hardware_interface::StateInterface> Z1System::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(get_hardware_info().joints.size() * 3);

  // Note: built with the pre-InterfaceDescription constructor. Upstream's own
  // test component (hardware_interface_testing/test/test_components/test_system.cpp
  // at 4.48.0) still uses this form; the replacement
  // `on_export_state_interfaces()` hands back interfaces whose value pointer
  // cannot be assigned from the outside, so it is not usable for wiring up a
  // component's own buffers yet.
  for (const auto & joint : get_hardware_info().joints) {
    const auto index = static_cast<size_t>(motor_index_for_joint(joint.name));
    interfaces.emplace_back(
      joint.name, hardware_interface::HW_IF_POSITION, &motor_position_[index]);
    interfaces.emplace_back(
      joint.name, hardware_interface::HW_IF_VELOCITY, &motor_velocity_[index]);
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &motor_effort_[index]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> Z1System::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(get_hardware_info().joints.size());

  for (const auto & joint : get_hardware_info().joints) {
    const auto index = static_cast<size_t>(motor_index_for_joint(joint.name));
    interfaces.emplace_back(
      joint.name, hardware_interface::HW_IF_POSITION, &command_position_[index]);
  }
  return interfaces;
}

#pragma GCC diagnostic pop

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

hardware_interface::CallbackReturn Z1System::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::string error;
  if (!udp_.open(ctrl_ip_, ctrl_port_, own_port_, error)) {
    RCLCPP_ERROR(get_logger(), "cannot open the link to z1_ctrl: %s", error.c_str());
    return CallbackReturn::ERROR;
  }

  cycles_ = 0;
  cycles_without_packet_ = 0;
  last_fault_message_.clear();
  last_rx_steady_ns_.store(0);
  reported_state_.store(static_cast<int32_t>(UNITREE_ARM::ArmFSMState::INVALID));

  // The services exist as soon as the component is configured, which happens at
  // ros2_control_node startup - they can therefore report why activation is
  // refusing to happen.
  create_fsm_services();

  RCLCPP_INFO(
    get_logger(), "bound UDP port %u, sending to z1_ctrl at %s:%u", own_port_, ctrl_ip_.c_str(),
    ctrl_port_);
  return CallbackReturn::SUCCESS;
}

void Z1System::create_fsm_services()
{
  const auto node = get_node();
  if (!node) {
    // get_node() is the component's own node, which ros2_control creates when it
    // loads the plugin and adds to the controller_manager's executor. Without it
    // there is nowhere to serve the FSM from, and no other process can: z1_ctrl
    // only ever answers the one client that binds 8072.
    RCLCPP_WARN(
      get_logger(),
      "this hardware component has no node, so set_fsm_state/get_fsm_state are unavailable; "
      "the FSM can then only be driven through activate_fsm_sequence and on_deactivate.");
    return;
  }

  set_state_service_ = node->create_service<z1_ros2_control::srv::SetFSMState>(
    "~/set_fsm_state",
    [this](
      const std::shared_ptr<z1_ros2_control::srv::SetFSMState::Request> request,
      std::shared_ptr<z1_ros2_control::srv::SetFSMState::Response> response) {
      handle_set_fsm_state(request, response);
    });
  get_state_service_ = node->create_service<z1_ros2_control::srv::GetFSMState>(
    "~/get_fsm_state",
    [this](
      const std::shared_ptr<z1_ros2_control::srv::GetFSMState::Request> request,
      std::shared_ptr<z1_ros2_control::srv::GetFSMState::Response> response) {
      handle_get_fsm_state(request, response);
    });

  RCLCPP_INFO(
    get_logger(), "%s/set_fsm_state and %s/get_fsm_state serve %s",
    node->get_fully_qualified_name(), node->get_fully_qualified_name(), serviceFsmStates());
}

hardware_interface::CallbackReturn Z1System::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::string error;

  if (!udp_.is_open()) {
    RCLCPP_ERROR(get_logger(), "cannot activate: the link to z1_ctrl is not open.");
    return CallbackReturn::ERROR;
  }

  // We cannot switch the FSM before we know where the arm currently is, and we
  // must not send a single frame before that either: a zeroed SendCmd would be a
  // command to the zero pose.
  if (!wait_for_first_packet(error)) {
    RCLCPP_ERROR(get_logger(), "cannot activate: %s", error.c_str());
    return CallbackReturn::ERROR;
  }

  // Start from the measured pose, so entering JOINTCTRL is not a step input.
  // This mirrors what the official SDK does in unitreeArm::startTrack().
  command_position_ = motor_position_;

  // From here on pump_once() transmits a frame on every call, and it is the
  // request_state_and_wait() loops below that provide those calls: read() is not
  // running yet (see the note on pump_once in the header).
  link_armed_.store(true);

  for (const auto state : activate_sequence_) {
    if (!request_state_and_wait(state, "", error)) {
      RCLCPP_ERROR(get_logger(), "%s", error.c_str());
      // Do not leave a half-claimed link behind: dropping link_armed_ stops the
      // frame train that carried the request, and z1_ctrl falls back to PASSIVE
      // on its own when it stops hearing from us.
      link_armed_.store(false);
      return CallbackReturn::ERROR;
    }
  }

  activated_.store(true);
  RCLCPP_INFO(
    get_logger(),
    "active: z1_ctrl reports %s, commanding the measured pose. If this process dies, z1_ctrl's "
    "ARMSDK falls back to PASSIVE when the frames stop arriving and the arm loses its torque.",
    fsmStateName(static_cast<UNITREE_ARM::ArmFSMState>(reported_state_.load())));
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Z1System::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  activated_.store(false);

  if (link_armed_.load() && deactivate_to_passive_) {
    std::string error;
    if (!request_state_and_wait(UNITREE_ARM::ArmFSMState::PASSIVE, "", error)) {
      RCLCPP_WARN(get_logger(), "%s", error.c_str());
    }
  }

  RCLCPP_INFO(
    get_logger(), "deactivated after %llu cycles (%llu without a packet from z1_ctrl).",
    static_cast<unsigned long long>(cycles_),
    static_cast<unsigned long long>(cycles_without_packet_));

  if (!deactivate_to_passive_) {
    RCLCPP_WARN(
      get_logger(),
      "deactivate_fsm:=hold - the arm is still being held in JOINTCTRL. It will only go limp once "
      "this process stops sending.");
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Z1System::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Dropping the link is what makes z1_ctrl fall back to PASSIVE by itself
  // (ARMSDK::_sendRecv does that whenever it stops receiving), so there is
  // nothing to send here.
  link_armed_.store(false);
  activated_.store(false);
  set_state_service_.reset();
  get_state_service_.reset();
  udp_.close();
  RCLCPP_INFO(get_logger(), "link closed; z1_ctrl will fall back to PASSIVE on its own.");
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Realtime cycle
// ---------------------------------------------------------------------------

void Z1System::refresh_published_label()
{
  // An epoch-checked copy, not a lock: `label_` is written by the service thread,
  // so a store can land in the middle of the byte loop below. Equal epochs mean
  // no store happened between the first and the last byte, so `candidate` is one
  // label and not a mixture of two. A failed check is simply retried on the next
  // pump, and the service callback waits for the publish before it arms a state
  // request, so a torn label never reaches the wire.
  if (label_epoch_.load(std::memory_order_acquire) == published_label_epoch_) {
    return;
  }
  const uint32_t before = label_epoch_.load(std::memory_order_acquire);
  std::array<char, kLabelCapacity> candidate{};
  for (size_t i = 0; i < kLabelCapacity; ++i) {
    candidate[i] = label_[i].load(std::memory_order_relaxed);
  }
  if (label_epoch_.load(std::memory_order_acquire) == before) {
    published_label_ = candidate;
    published_label_epoch_.store(before, std::memory_order_release);
  }
}

hardware_interface::return_type Z1System::pump_once(std::string & error)
{
  IoGuard guard(io_busy_);
  if (!guard.owned()) {
    // The async worker is already pumping, or a lifecycle transition is. Its
    // frame is as good as ours, and every caller retries.
    return hardware_interface::return_type::OK;
  }

  refresh_published_label();

  // Drain the socket and keep the newest well-formed packet. Reading only one
  // datagram per call would make the reported state fall progressively further
  // behind the arm.
  UNITREE_ARM::RecvState newest{};
  bool got_packet = false;
  bool hard_error = false;

  for (int i = 0; i < kMaxDrainPerCycle; ++i) {
    const auto result = udp_.recv(newest, error);
    if (result == Z1UdpClient::RecvResult::Received) {
      got_packet = true;
      continue;
    }
    if (result == Z1UdpClient::RecvResult::NoData) {
      break;
    }
    if (result == Z1UdpClient::RecvResult::Malformed) {
      report_fault("protocol: " + error);
      continue;
    }
    // A genuine socket failure, e.g. ECONNREFUSED while z1_ctrl is not running.
    report_fault("link: " + error);
    hard_error = true;
    break;
  }

  if (got_packet) {
    apply_state(newest);
    last_rx_steady_ns_.store(steady_ns());
  } else {
    ++cycles_without_packet_;
  }

  // Transmit. This is why the handshake has to be pumped: the frame carrying a
  // state request is the only way to ask z1_ctrl for something, and ros2_control
  // will not call read()/write() until this component is already ACTIVE.
  if (link_armed_.load()) {
    assemble_frame();
    if (!udp_.send(cmd_, error)) {
      report_fault("send: " + error);
    } else if (got_packet) {
      clear_fault();
    }
  }

  return hard_error ? hardware_interface::return_type::ERROR : hardware_interface::return_type::OK;
}

hardware_interface::return_type Z1System::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  ++cycles_;

  // read() only runs while the component is ACTIVE, so this guard is about the
  // window around on_cleanup(), which closes the socket before ros2_control has
  // necessarily stopped calling us.
  if (!udp_.is_open()) {
    return hardware_interface::return_type::OK;
  }

  report_read_rate(steady_ns());

  // The error text is only ever used by pump_once's own report_fault() calls, so a
  // local is enough here.
  std::string error;
  const bool hard_error = pump_once(error) == hardware_interface::return_type::ERROR;

  if (!activated_.load()) {
    // While INACTIVE we still receive (and report) state, but a quiet arm is not
    // an error yet.
    return hardware_interface::return_type::OK;
  }

  if (hard_error) {
    return hardware_interface::return_type::ERROR;
  }

  if (!link_is_fresh()) {
    report_fault(
      "link: no RecvState from z1_ctrl for more than " + std::to_string(disconnect_timeout_ms_) +
      " ms. Has z1_ctrl exited, or lost the arm? The arm is limp while this lasts.");
    return hardware_interface::return_type::ERROR;
  }

  const std::string fault = first_motor_fault();
  if (!fault.empty()) {
    report_fault("motor: " + fault);
    return hardware_interface::return_type::ERROR;
  }

  clear_fault();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Z1System::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // The frame is assembled and transmitted by read(); see the comment there.
  // write() assembles it too so that the buffer a controller just wrote into is
  // reflected immediately rather than one cycle late.
  IoGuard guard(io_busy_);
  if (!guard.owned()) {
    // A lifecycle or service callback is mid-pump and will assemble a frame from
    // the same command buffer; doing it twice at once is what the guard prevents.
    return hardware_interface::return_type::OK;
  }
  assemble_frame();
  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void Z1System::assemble_frame()
{
  const auto requested = static_cast<UNITREE_ARM::ArmFSMState>(requested_state_.load());
  const bool joint_control = requested == UNITREE_ARM::ArmFSMState::JOINTCTRL;

  cmd_.head[0] = kHead0;
  cmd_.head[1] = kHead1;
  cmd_.state = requested;
  // "track" tells z1_ctrl to follow jointCmd in JOINTCTRL. It is also what makes
  // State_Cartesian follow the posture deltas it reads out of the same bytes, and
  // CARTESIAN is not reachable from here - so leaving it true for anything else
  // would only be misleading.
  cmd_.track = joint_control;

  if (joint_control) {
    for (size_t i = 0; i < kMotorCount; ++i) {
      auto & joint_cmd = cmd_.valueUnion.jointCmd[i];
      // z1_ctrl's joint servo law is fixed inside the controller, so gains are
      // deliberately left at zero (upstream's own LOWCMD state reads only Pos and W
      // out of this struct). Torque feed-forward is a LOWCMD feature and this
      // interface does not expose it.
      joint_cmd.T = 0.0F;
      // Velocity feed-forward stays zero: the trajectory controller drives a
      // position interface, and the arm's own loop differentiates it.
      joint_cmd.W = 0.0F;
      joint_cmd.K_P = 0.0F;
      joint_cmd.K_W = 0.0F;

      if (i == kGripperIndex && !has_gripper_) {
        // No gripper joint in the URDF: track the measured angle instead of
        // commanding 0, so that attaching a gripper later needs only
        // use_gripper:=true and never sends it a surprise goal.
        joint_cmd.Pos = static_cast<float>(motor_position_[i]);
      } else {
        joint_cmd.Pos = static_cast<float>(command_position_[i]);
      }
    }
    return;
  }

  // Every other reachable state either ignores valueUnion (PASSIVE, BACKTOSTART,
  // CALIBRATION) or reads it as `name`. Zeroing it keeps a stale jointCmd from
  // being reinterpreted as something else - `name`, `jointCmd[0..2]` and
  // `trajCmd` all start at the same byte.
  std::memset(&cmd_.valueUnion, 0, sizeof(cmd_.valueUnion));

  if (fsmStateTakesLabel(requested)) {
    // published_label_ is always NUL-terminated: publish_label() rejects anything
    // longer than kMaxLabelLength, so copying the whole field is safe and is what
    // gives z1_ctrl a terminated std::string.
    std::memcpy(cmd_.valueUnion.name, published_label_.data(), kLabelCapacity);
  }
}

void Z1System::apply_state(const UNITREE_ARM::RecvState & state)
{
  for (size_t i = 0; i < kMotorCount; ++i) {
    const auto & joint_state = state.jointState[i];
    motor_position_[i] = joint_state.Pos;
    motor_velocity_[i] = joint_state.W;
    motor_effort_[i] = joint_state.T;

    // A Z1 joint can carry one or two motors (JointMotorType::SINGLE_MOTOR /
    // DOUBLE_MOTOR), hence state[2]. Slot 0 is always populated. Slot 1 is only
    // consulted when it looks populated, so that a joint with a single motor
    // cannot raise a phantom fault from an untouched slot.
    int8_t temperature = joint_state.state[0].temperature;
    uint8_t error = joint_state.state[0].error;
    uint8_t connected = joint_state.state[0].isConnected.state;
    if (joint_state.state[1].temperature != 0) {
      temperature = std::max(temperature, joint_state.state[1].temperature);
      error |= joint_state.state[1].error;
      if (joint_state.state[1].isConnected.state != 0) {
        connected = joint_state.state[1].isConnected.state;
      }
    }
    motor_temperature_[i] = temperature;
    motor_error_[i] = error;
    motor_connected_[i] = connected;
  }

  const auto reported = static_cast<UNITREE_ARM::ArmFSMState>(state.state);
  const auto previous = static_cast<UNITREE_ARM::ArmFSMState>(
    reported_state_.exchange(static_cast<int32_t>(state.state)));
  if (reported != previous && link_armed_.load()) {
    RCLCPP_INFO(
      get_logger(), "z1_ctrl state: %s -> %s", fsmStateName(previous), fsmStateName(reported));
  }
}

std::string Z1System::first_motor_fault() const
{
  for (size_t i = 0; i < kMotorCount; ++i) {
    // Motor slot 6 only exists when the URDF declares jointGripper. When it does
    // not, the slot is not ours: z1_ctrl keeps reporting whatever the arm's
    // seventh motor connector does, and an unconnected connector reads as
    // "motor disconnected". Treating that as a fault would make read() fail on
    // every cycle of a perfectly healthy 6-axis arm, so the gripper slot is
    // skipped entirely - disconnected, hot or otherwise - unless the URDF asks
    // for it. This is the one fault that can only be found on a real arm.
    if (i == kGripperIndex && !has_gripper_) {
      continue;
    }

    const char * name = (i == kGripperIndex) ? "gripper" : arm_joint_names_[i].c_str();

    if ((motor_error_[i] & kErrorMask) != 0) {
      return std::string(name) + ": " + motorErrorToString(motor_error_[i]);
    }
    if (motor_connected_[i] != 0) {
      // From Unitree's Motor_Connected comment: 0 ok, 1 disconnected, 2 CRC error.
      return std::string(name) + (motor_connected_[i] == 1 ? ": motor disconnected"
                                                           : ": motor CRC error");
    }
    if (motor_temperature_[i] > temperature_limit_) {
      return std::string(name) + ": over-temperature, " + std::to_string(motor_temperature_[i]) +
             " C (limit " + std::to_string(temperature_limit_) + " C)";
    }
  }
  return "";
}

void Z1System::report_read_rate(int64_t now_ns)
{
  ++rate_window_cycles_;
  if (rate_window_start_ns_ == 0) {
    rate_window_start_ns_ = now_ns;
    rate_window_cycles_ = 0;
    return;
  }

  const int64_t elapsed_ns = now_ns - rate_window_start_ns_;
  if (elapsed_ns < 1000000000LL) {
    return;
  }

  const double hz =
    static_cast<double>(rate_window_cycles_) * 1e9 / static_cast<double>(elapsed_ns);
  // First measurement always, then only when the rate leaves a wide band: the odd
  // dropped cycle is normal, a halving of the rate is not.
  if (rate_reference_hz_ == 0.0 || hz < rate_reference_hz_ * 0.8 ||
      hz > rate_reference_hz_ * 1.25) {
    RCLCPP_INFO(
      get_logger(),
      "read() is running at %.0f Hz - that is the SendCmd rate z1_ctrl sees. ros2_control "
      "paced this component with the controller_manager's update_rate (%u Hz).",
      hz, get_hardware_info().rw_rate);
    rate_reference_hz_ = hz;
  }

  rate_window_start_ns_ = now_ns;
  rate_window_cycles_ = 0;
}

bool Z1System::link_is_fresh() const
{
  const int64_t last = last_rx_steady_ns_.load();
  if (last == 0) {
    return false;
  }
  return (steady_ns() - last) < (static_cast<int64_t>(disconnect_timeout_ms_) * 1000000LL);
}

bool Z1System::wait_for_first_packet(std::string & error)
{
  // Nothing transmits yet - link_armed_ is still false, because a zeroed SendCmd
  // would be a command to the zero pose - but z1_ctrl sends RecvState whether or
  // not anyone is its client (verified against the real controller: it keeps
  // streaming at ~10 Hz with no SendCmd ever arriving), so listening is enough to
  // learn where the arm is.
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(fsm_timeout_ms_);
  while (std::chrono::steady_clock::now() < deadline) {
    std::string pump_error;
    pump_once(pump_error);
    if (link_is_fresh()) {
      return true;
    }
    std::this_thread::sleep_for(kPollInterval);
  }
  error = "no RecvState from z1_ctrl within " + std::to_string(fsm_timeout_ms_) +
          " ms. Is z1_ctrl running, started from its build directory, and connected to the arm?";
  return false;
}

bool Z1System::publish_label(const std::string & label, std::string & error)
{
  if (label.size() > kMaxLabelLength) {
    error = "label '" + label + "' is longer than " + std::to_string(kMaxLabelLength) +
            " characters; z1_ctrl reads it out of a char[10] and builds a std::string from it";
    return false;
  }

  const uint32_t epoch = label_epoch_.load(std::memory_order_relaxed) + 1;
  for (size_t i = 0; i < kLabelCapacity; ++i) {
    label_[i].store(i < label.size() ? label[i] : '\0', std::memory_order_relaxed);
  }
  // Release: the bytes above happen-before any load that observes this epoch.
  label_epoch_.store(epoch, std::memory_order_release);

  // Do not point z1_ctrl at the state before the label has been taken up: z1_ctrl
  // reads `name` when it *enters* the state, so the first frame that carries the
  // state also has to carry a correct label. pump_once() is what performs that
  // copy. While this loop runs, read() may be pumping in parallel (this callback
  // is on the controller_manager's executor, the worker thread is separate), so
  // both may make progress - IoGuard keeps them from assembling a frame at the
  // same time. All we need is to see the epoch move, which happens within a cycle
  // or two of either of them.
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(fsm_timeout_ms_);
  while (std::chrono::steady_clock::now() < deadline) {
    if (published_label_epoch_.load(std::memory_order_acquire) == epoch) {
      return true;
    }
    if (!udp_.is_open()) {
      error = "cannot hand the label to the I/O path: the link to z1_ctrl is closed";
      return false;
    }
    std::string pump_error;
    pump_once(pump_error);
    std::this_thread::sleep_for(kPollInterval);
  }
  error = "the label was not taken up by the I/O path within " + std::to_string(fsm_timeout_ms_) +
          " ms";
  return false;
}

bool Z1System::request_state_and_wait(
  UNITREE_ARM::ArmFSMState state, const std::string & label, std::string & error)
{
  // The label has to be published before the state is requested; see
  // publish_label().
  if (fsmStateTakesLabel(state) && !publish_label(label, error)) {
    return false;
  }

  // Stored even when z1_ctrl already reports `state`: read() transmits whatever
  // is in here, so leaving it at its previous value would let the next frame
  // undo a request that we reported as done.
  requested_state_.store(static_cast<int32_t>(state), std::memory_order_release);

  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(fsm_timeout_ms_);
  while (std::chrono::steady_clock::now() < deadline) {
    if (reported_state_.load() == static_cast<int32_t>(state)) {
      return true;
    }
    if (!udp_.is_open()) {
      error = std::string("asked z1_ctrl for ") + fsmStateName(state) + " but the link is closed";
      return false;
    }
    // pump_once() is what puts the request on the wire: read() is not running
    // during on_activate(), and during a service call it is running in parallel
    // (both ends are safe to pump at once, see the README).
    std::string pump_error;
    pump_once(pump_error);
    std::this_thread::sleep_for(kPollInterval);
  }

  error = std::string("timed out after ") + std::to_string(fsm_timeout_ms_) + " ms asking z1_ctrl "
          "for " + fsmStateName(state) + "; it still reports " +
          fsmStateName(static_cast<UNITREE_ARM::ArmFSMState>(reported_state_.load())) +
          ". Not every FSM transition is legal from every state - see "
          "z1_ros2_control/README.md for the known-good sequences.";
  return false;
}

// ---------------------------------------------------------------------------
// FSM services
// ---------------------------------------------------------------------------

void Z1System::handle_set_fsm_state(
  const std::shared_ptr<z1_ros2_control::srv::SetFSMState::Request> request,
  std::shared_ptr<z1_ros2_control::srv::SetFSMState::Response> response)
{
  // The controller_manager's executor may be multithreaded, and two overlapping
  // transitions would interleave their `requested_state_` stores. read() and
  // write() never take this lock, so the worker thread cannot be blocked by it.
  std::lock_guard<std::mutex> guard(service_mutex_);

  const auto reported = static_cast<UNITREE_ARM::ArmFSMState>(reported_state_.load());
  response->success = false;
  response->current_state = fsmStateName(reported);

  UNITREE_ARM::ArmFSMState state{};
  if (!parseFsmStateName(request->state, state)) {
    response->message = "unknown state '" + request->state + "'; this component accepts " +
                        serviceFsmStates();
    return;
  }
  if (!isServiceFsmState(state)) {
    response->message =
      std::string(fsmStateName(state)) +
      " is not reachable through this interface: it reads a Cartesian posture, a trajectory or raw "
      "motor commands out of SendCmd, and this component only ever fills in joint positions. It "
      "accepts " + serviceFsmStates() + ".";
    return;
  }
  if (fsmStateTakesLabel(state)) {
    if (request->label.empty()) {
      response->message =
        std::string(fsmStateName(state)) +
        " is addressed by a label, so `label` has to name an entry of z1_ctrl's "
        "config/savedArmStates.csv (e.g. forward, startFlat, show_left, show_mid, show_right).";
      return;
    }
    if (request->label.size() > kMaxLabelLength) {
      response->message = "label '" + request->label + "' is longer than " +
                          std::to_string(kMaxLabelLength) + " characters";
      return;
    }
  } else if (!request->label.empty()) {
    response->message = std::string(fsmStateName(state)) +
                        " does not read a label (it would overlay jointCmd[0] in the union), so "
                        "`label` has to be empty for it";
    return;
  }

  if (!link_armed_.load()) {
    response->message =
      "the link to z1_ctrl is not armed, so no request can be delivered. Activate the hardware "
      "component first, e.g. ros2 control set_hardware_component_state z1 active";
    return;
  }

  if (state == UNITREE_ARM::ArmFSMState::JOINTCTRL) {
    // Entering JOINTCTRL makes z1_ctrl track `valueUnion.jointCmd`, which comes
    // straight from `command_position_` - whatever the trajectory controller last
    // wrote. If the arm has been limp (PASSIVE), hand-guided (TEACH) or
    // calibrated since then, that reference is stale and entering JOINTCTRL is a
    // step input. Only on_activate() re-seeds it from the measured pose, so this
    // refuses instead of yanking the arm and points at the lifecycle.
    //
    // read() only runs while the component is ACTIVE, so pump once first: while
    // INACTIVE the state arrays would otherwise be a stale sample and the
    // comparison below meaningless.
    std::string pump_error;
    pump_once(pump_error);

    double worst = 0.0;
    size_t worst_index = 0;
    for (size_t i = 0; i < kArmJointCount; ++i) {
      const double offset = std::abs(command_position_[i] - motor_position_[i]);
      if (offset > worst) {
        worst = offset;
        worst_index = i;
      }
    }
    if (worst > kJointCtrlRejoinTolerance) {
      std::ostringstream detail;
      detail << "cannot enter JOINTCTRL: the commanded position is " << std::fixed
             << std::setprecision(3) << worst
             << " rad away from where the arm actually is (joint" << (worst_index + 1)
             << "), so entering it would be a step input. This component only re-seeds the joint "
                "command from the measured pose during activation - cycle the hardware component "
                "instead: ros2 control set_hardware_component_state z1 inactive, then ... active.";
      response->message = detail.str();
      return;
    }
  }

  if (reported == state) {
    // Keep the outgoing frames consistent with what we report, but say plainly
    // that nothing was requested: z1_ctrl's checkChange() ignores a request for
    // the state it is already in, so a second TOSTATE while already in TOSTATE is
    // a no-op rather than a new target.
    requested_state_.store(static_cast<int32_t>(state), std::memory_order_release);
    response->success = true;
    response->current_state = fsmStateName(state);
    response->message = std::string("z1_ctrl already reports ") + fsmStateName(state) +
                        "; nothing was requested" +
                        (fsmStateTakesLabel(state)
                           ? " - the FSM does not re-enter the state it is already in, so a new "
                             "label only takes effect after leaving it (e.g. via JOINTCTRL)"
                           : "");
    return;
  }

  std::string error;
  if (!request_state_and_wait(state, request->label, error)) {
    response->message = error;
    response->current_state =
      fsmStateName(static_cast<UNITREE_ARM::ArmFSMState>(reported_state_.load()));
    return;
  }

  response->success = true;
  response->current_state = fsmStateName(state);
  response->message = std::string(fsmStateName(state)) + " acknowledged by z1_ctrl";
  if (request->label.empty()) {
    RCLCPP_INFO(
      get_logger(), "set_fsm_state: %s -> %s", fsmStateName(reported), fsmStateName(state));
  } else {
    RCLCPP_INFO(
      get_logger(), "set_fsm_state: %s -> %s (label '%s')", fsmStateName(reported),
      fsmStateName(state), request->label.c_str());
  }
}

void Z1System::handle_get_fsm_state(
  const std::shared_ptr<z1_ros2_control::srv::GetFSMState::Request> /*request*/,
  std::shared_ptr<z1_ros2_control::srv::GetFSMState::Response> response)
{
  response->state = fsmStateName(static_cast<UNITREE_ARM::ArmFSMState>(reported_state_.load()));
  response->active = activated_.load();
  response->link_armed = link_armed_.load();
  response->link_fresh = link_is_fresh();
}

void Z1System::report_fault(const std::string & message)
{
  if (message != last_fault_message_) {
    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
    last_fault_message_ = message;
  }
}

void Z1System::clear_fault()
{
  if (!last_fault_message_.empty()) {
    RCLCPP_INFO(get_logger(), "recovered from: %s", last_fault_message_.c_str());
    last_fault_message_.clear();
  }
}

}  // namespace z1_ros2_control

PLUGINLIB_EXPORT_CLASS(z1_ros2_control::Z1System, hardware_interface::SystemInterface)
