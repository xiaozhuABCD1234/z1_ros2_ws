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
#include <cstdlib>
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
/// Cap on datagrams consumed per read(), so a flooded socket cannot stall the
/// 500 Hz worker thread.
constexpr int kMaxDrainPerCycle = 16;

/// How long the lifecycle callbacks sleep between polls of the async thread.
constexpr auto kPollInterval = std::chrono::milliseconds(2);

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
    get_logger(), "'%s': %zu arm joints%s, controller at %s:%u, own port %u, rw_rate %u Hz.",
    info.name.c_str(), arm_joint_names_.size(),
    has_gripper_ ? " + gripper" : " (no gripper; motor slot 6 is held at its measured angle)",
    ctrl_ip_.c_str(), ctrl_port_, own_port_, info.rw_rate);

  if (info.rw_rate == 0u) {
    RCLCPP_WARN(
      get_logger(),
      "'%s': rw_rate is 0. This component must be declared with rw_rate=\"500\" - z1_ctrl's "
      "ARMSDK loop runs at 500 Hz and treats a cycle without an incoming SendCmd as a timeout.",
      info.name.c_str());
  }

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
          error = "activate_fsm_sequence contains an unknown state '" + token + "'; expected " +
                  supportedFsmStates();
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
      error = "activate_fsm_sequence is empty; expected " + std::string(supportedFsmStates());
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

  RCLCPP_INFO(
    get_logger(), "bound UDP port %u, sending to z1_ctrl at %s:%u", own_port_, ctrl_ip_.c_str(),
    ctrl_port_);
  return CallbackReturn::SUCCESS;
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

  // From here on read() transmits a frame every cycle. Doing this before the FSM
  // handshake is deliberate: it is what carries the requested state to z1_ctrl.
  link_armed_.store(true);
  assemble_frame();

  for (const auto state : activate_sequence_) {
    if (!request_state_and_wait(state, error)) {
      RCLCPP_ERROR(get_logger(), "%s", error.c_str());
      // Do not leave a half-claimed link behind; read() keeps sending, so the
      // PASSIVE request will go out even though we failed forward.
      link_armed_.store(false);
      return CallbackReturn::ERROR;
    }
  }

  activated_.store(true);
  RCLCPP_INFO(
    get_logger(),
    "active: z1_ctrl reports %s, commanding the measured pose. If this process dies, z1_ctrl "
    "forces PASSIVE after ~20 ms and the arm loses its torque.",
    fsmStateName(static_cast<UNITREE_ARM::ArmFSMState>(reported_state_.load())));
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Z1System::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  activated_.store(false);

  if (link_armed_.load() && deactivate_to_passive_) {
    std::string error;
    if (!request_state_and_wait(UNITREE_ARM::ArmFSMState::PASSIVE, error)) {
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
  udp_.close();
  RCLCPP_INFO(get_logger(), "link closed; z1_ctrl will fall back to PASSIVE on its own.");
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Realtime cycle
// ---------------------------------------------------------------------------

hardware_interface::return_type Z1System::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  ++cycles_;

  // The async worker thread is started before on_init, so we are called while the
  // component is still UNCONFIGURED and there is no socket yet.
  if (!udp_.is_open()) {
    return hardware_interface::return_type::OK;
  }

  // Drain the socket and keep the newest well-formed packet. Reading only one
  // datagram per cycle would make the reported state fall progressively further
  // behind the arm.
  UNITREE_ARM::RecvState newest{};
  bool got_packet = false;
  std::string error;

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
    if (activated_.load()) {
      return hardware_interface::return_type::ERROR;
    }
    break;
  }

  if (got_packet) {
    apply_state(newest);
    last_rx_steady_ns_.store(steady_ns());
  } else {
    ++cycles_without_packet_;
  }

  // Transmit. Assembling here (rather than only in write()) is what makes the
  // FSM handshake work: write() is not called until the component is ACTIVE,
  // which only happens after on_activate() has already returned.
  if (link_armed_.load()) {
    assemble_frame();
    if (!udp_.send(cmd_, error)) {
      report_fault("send: " + error);
    } else if (got_packet) {
      clear_fault();
    }
  }

  if (!activated_.load()) {
    // While INACTIVE we still receive (and report) state, but a quiet arm is not
    // an error yet.
    return hardware_interface::return_type::OK;
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
  assemble_frame();
  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void Z1System::assemble_frame()
{
  cmd_.head[0] = kHead0;
  cmd_.head[1] = kHead1;

  const auto requested = static_cast<UNITREE_ARM::ArmFSMState>(requested_state_.load());
  cmd_.state = requested;
  // "track" tells z1_ctrl to follow jointCmd (JOINTCTRL) or posture[0]
  // (CARTESIAN). For every other state the joint commands are ignored, so
  // leaving it true would be harmless but misleading.
  cmd_.track = (requested == UNITREE_ARM::ArmFSMState::JOINTCTRL);

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
    const char * name =
      (i == kGripperIndex || i >= arm_joint_names_.size()) ? "gripper" : arm_joint_names_[i].c_str();

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
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(fsm_timeout_ms_);
  while (std::chrono::steady_clock::now() < deadline) {
    if (link_is_fresh()) {
      return true;
    }
    std::this_thread::sleep_for(kPollInterval);
  }
  error = "no RecvState from z1_ctrl within " + std::to_string(fsm_timeout_ms_) +
          " ms. Is z1_ctrl running, started from its build directory, and connected to the arm?";
  return false;
}

bool Z1System::request_state_and_wait(UNITREE_ARM::ArmFSMState state, std::string & error)
{
  requested_state_.store(static_cast<int32_t>(state));

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
    // read() on the worker thread is what transmits the request.
    std::this_thread::sleep_for(kPollInterval);
  }

  error = std::string("timed out after ") + std::to_string(fsm_timeout_ms_) + " ms asking z1_ctrl "
          "for " + fsmStateName(state) + "; it still reports " +
          fsmStateName(static_cast<UNITREE_ARM::ArmFSMState>(reported_state_.load())) +
          ". Not every FSM transition is legal from every state - see "
          "z1_ros2_control/README.md for the known-good sequences.";
  return false;
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
