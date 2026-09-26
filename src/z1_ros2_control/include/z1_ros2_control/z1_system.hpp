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

#ifndef Z1_ROS2_CONTROL__Z1_SYSTEM_HPP_
#define Z1_ROS2_CONTROL__Z1_SYSTEM_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "z1_ros2_control/z1_protocol.hpp"
#include "z1_ros2_control/z1_udp_client.hpp"

namespace z1_ros2_control
{

/// ros2_control backend for a real Z1 driven by the official `z1_controller`.
///
/// This component is a **client**, not a controller. The official `z1_ctrl`
/// process owns the 250 Hz loop that talks to the arm (`192.168.123.110:8881`),
/// plus the finite state machine, the kinematics and the collision guard. We
/// speak the same loopback protocol that the official `z1_sdk` speaks,
/// so `z1_ctrl` keeps doing all the dangerous work and this class only moves
/// joint positions in and joint state out.
///
/// ### Threading, and why there are no locks anywhere
///
/// The component is declared `is_async="true"` in the URDF, so ros2_control
/// runs `read()` and `write()` on its own worker thread, each cycle calling
/// `read()` and then `write()` **on that one thread**. Consequences:
///
///  * `read()` and `write()` are never concurrent with each other, so the
///    command buffer and the link statistics need no synchronisation at all.
///  * `write()` only publishes the command into `cmd_`; `read()` is what
///    actually transmits it. A command therefore reaches the arm on the cycle
///    *after* the controller produced it - one 2 ms frame, the same pipeline
///    delay the official SDK has.
///  * Nothing in the I/O path may take a mutex, because the controller_manager
///    thread reads the state arrays concurrently. They are plain `double`s (see
///    `export_state_interfaces()`); a torn read would at worst mix joints from
///    two adjacent frames, which controllers tolerate, whereas a lock here would
///    risk priority inversion against the 500 Hz thread.
///
/// `rw_rate` must be 500, not 250: `z1_ctrl`'s `ARMSDK` loop runs at
/// `dt = 0.002` s and considers a cycle without an incoming `SendCmd` to be a
/// timeout. The controller_manager's own `update_rate` stays 250 to match the
/// arm's servo loop.
class Z1System : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  // Overriding the deprecated `export_*_interfaces()` pair on purpose: the
  // InterfaceDescription-based replacements cannot wire a component's own value
  // pointers yet. See the definitions in src/z1_system.cpp.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
#pragma GCC diagnostic pop

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// Build the outgoing frame from `command_position_` / `requested_state_`.
  /// Idempotent, and called from both read() and write(); see the .cpp.
  void assemble_frame();

  /// Log a fault, but only when it differs from the one already being reported,
  /// so a 500 Hz loop cannot flood the log.
  void report_fault(const std::string & message);
  void clear_fault();

  /// Read every `<param>` we understand out of the URDF `<hardware>` block.
  bool read_parameters(std::string & error);

  /// Map a URDF joint name onto its motor slot in the packet.
  /// `joint1..joint6` -> 0..5, `jointGripper` -> 6, anything else -> -1.
  static int motor_index_for_joint(const std::string & joint_name);

  /// Fold one received packet into the exported state arrays.
  void apply_state(const UNITREE_ARM::RecvState & state);

  /// Request an FSM state and wait until `z1_ctrl` reports it. Only called from
  /// the non-realtime lifecycle callbacks.
  bool request_state_and_wait(UNITREE_ARM::ArmFSMState state, std::string & error);

  /// Block until a packet has arrived within `disconnect_timeout_ms_`.
  bool wait_for_first_packet(std::string & error);

  /// True when a packet has arrived recently enough to trust the arm's state.
  bool link_is_fresh() const;

  /// Returns a description of the first motor fault found, or an empty string.
  std::string first_motor_fault() const;

  // ---- configuration, from the URDF <hardware><param> block ----------------
  std::string ctrl_ip_ = kDefaultCtrlIp;
  uint16_t ctrl_port_ = kDefaultCtrlPort;
  uint16_t own_port_ = kDefaultOwnPort;
  int disconnect_timeout_ms_ = 200;
  int temperature_limit_ = kDefaultTemperatureLimit;
  int fsm_timeout_ms_ = 2000;
  std::vector<UNITREE_ARM::ArmFSMState> activate_sequence_;
  /// What to ask for when ros2_control deactivates the component.
  bool deactivate_to_passive_ = true;

  // ---- joint bookkeeping ---------------------------------------------------
  /// URDF joint names, index-aligned with `info_.joints`. Only used for logs and
  /// to validate that every motor slot we command is actually wired up.
  std::vector<std::string> arm_joint_names_;
  bool has_gripper_ = false;

  // ---- shared with the controller_manager's read loop ----------------------
  // Written by read() on the async thread, read by controllers. Plain doubles on
  // purpose: see the class comment about locks.
  std::array<double, kMotorCount> motor_position_{};
  std::array<double, kMotorCount> motor_velocity_{};
  std::array<double, kMotorCount> motor_effort_{};
  // Written by controllers through the command interfaces, consumed by write().
  std::array<double, kMotorCount> command_position_{};

  // ---- diagnostics, only ever touched from the async thread -----------------
  std::array<int8_t, kMotorCount> motor_temperature_{};
  std::array<uint8_t, kMotorCount> motor_error_{};
  std::array<uint8_t, kMotorCount> motor_connected_{};
  std::string last_fault_message_;
  uint64_t cycles_ = 0;
  uint64_t cycles_without_packet_ = 0;

  // ---- link ----------------------------------------------------------------
  Z1UdpClient udp_;
  UNITREE_ARM::SendCmd cmd_{};
  /// Requested FSM state; written by on_activate/on_deactivate, read by write().
  std::atomic<int32_t> requested_state_{static_cast<int32_t>(UNITREE_ARM::ArmFSMState::PASSIVE)};
  /// Last FSM state reported by z1_ctrl; written by read(), polled by
  /// on_activate while it waits for a transition.
  std::atomic<int32_t> reported_state_{static_cast<int32_t>(UNITREE_ARM::ArmFSMState::INVALID)};
  std::atomic<bool> activated_{false};
  /// True once read() should be putting frames on the wire. Unlike
  /// `activated_`, this is set *before* the FSM handshake in on_activate(),
  /// because the handshake itself is carried by those frames.
  std::atomic<bool> link_armed_{false};
  /// Set by read(), polled by wait_for_first_packet(). steady_clock, not ROS time:
  /// this is a real-time link budget and must not be affected by sim time.
  std::atomic<int64_t> last_rx_steady_ns_{0};
};

}  // namespace z1_ros2_control

#endif  // Z1_ROS2_CONTROL__Z1_SYSTEM_HPP_
