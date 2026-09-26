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
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "z1_ros2_control/srv/get_fsm_state.hpp"
#include "z1_ros2_control/srv/set_fsm_state.hpp"
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
/// ### Threading, and why there are no locks in the I/O path
///
/// The component is declared `is_async="true"` in the URDF, so ros2_control
/// runs `read()` and `write()` on its own worker thread, each cycle calling
/// `write()` and then `read()` **on that one thread** - but *only while the
/// component is ACTIVE*, and paced by the controller_manager's `update_rate`
/// rather than by the URDF's `rw_rate` (see below). ros2_control's
/// `AsyncComponentThread` gates every
/// call on the lifecycle state, which is why on_activate() has to drive the
/// socket itself; see `pump_once()`. Consequences:
///
///  * `pump_once()` is the only code that touches the socket or `cmd_`, and it is
///    single-owner through `IoGuard`. Two threads can want to pump at once (the
///    async worker and a service callback); the loser skips, it never waits.
///  * `pump_once()` both receives and transmits. A command therefore reaches the
///    arm on the cycle *after* the controller produced it - one frame, the same
///    pipeline delay the official SDK has.
///  * Nothing in the I/O path may take a mutex, because the controller_manager
///    thread reads the state arrays concurrently. They are plain `double`s (see
///    `export_state_interfaces()`); a torn read would at worst mix joints from
///    two adjacent frames, which controllers tolerate, whereas a lock here would
///    risk priority inversion against the worker thread.
///
/// The one mutex in the class, `service_mutex_`, is taken by the `set_fsm_state`
/// callback only. read()/write() never touch it.
///
/// `rw_rate` should be 500: `z1_ctrl`'s `ARMSDK` loop runs at `dt = 0.002` s, so
/// 500 Hz is one `SendCmd` per tick. The rate is not something this class gets to
/// choose, though: ros2_control 4.48 paces an async component with the
/// controller_manager's `update_rate` and ignores the URDF's `rw_rate` attribute
/// (measured both ways - 250 -> 250 Hz, 500 -> 500 Hz, with the attribute saying
/// 500 in both runs). `report_read_rate()` therefore logs what actually happens,
/// and `z1_bringup/config/z1_controllers.yaml` is where the number is set.
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
  /// so a fast I/O loop cannot flood the log.
  void report_fault(const std::string & message);
  void clear_fault();

  /// Read every `<param>` we understand out of the URDF `<hardware>` block.
  bool read_parameters(std::string & error);

  /// Create `~/set_fsm_state` and `~/get_fsm_state` on the component's own node
  /// (`get_node()`, which ros2_control adds to the controller_manager
  /// executor), and tear them down again on cleanup.
  void create_fsm_services();
  /// `~/set_fsm_state`: validate, hand the label to read(), then wait for
  /// z1_ctrl to report the transition. Non-realtime: it blocks this executor
  /// thread for at most `fsm_timeout_ms_`.
  void handle_set_fsm_state(
    const std::shared_ptr<z1_ros2_control::srv::SetFSMState::Request> request,
    std::shared_ptr<z1_ros2_control::srv::SetFSMState::Response> response);
  /// `~/get_fsm_state`.
  void handle_get_fsm_state(
    const std::shared_ptr<z1_ros2_control::srv::GetFSMState::Request> request,
    std::shared_ptr<z1_ros2_control::srv::GetFSMState::Response> response);

  /// Publish `label` for read() to pick up, and wait until it has. See the
  /// long comment on the label handshake in the .cpp.
  bool publish_label(const std::string & label, std::string & error);

  /// Map a URDF joint name onto its motor slot in the packet.
  /// `joint1..joint6` -> 0..5, `jointGripper` -> 6, anything else -> -1.
  static int motor_index_for_joint(const std::string & joint_name);

  /// Fold one received packet into the exported state arrays.
  void apply_state(const UNITREE_ARM::RecvState & state);

  /// Request an FSM state and wait until `z1_ctrl` reports it. Only called from
  /// the non-realtime lifecycle and service callbacks.
  bool request_state_and_wait(
    UNITREE_ARM::ArmFSMState state, const std::string & label, std::string & error);

  /// Block until a packet has arrived within `disconnect_timeout_ms_`.
  bool wait_for_first_packet(std::string & error);

  /// True when a packet has arrived recently enough to trust the arm's state.
  bool link_is_fresh() const;

  /// Report how fast ros2_control is actually calling read(), because that - not
  /// `rw_rate` in the URDF - is what z1_ctrl's 2 ms ARMSDK loop sees. Logs once
  /// when the first second is up, then only when the rate leaves a +/-20% band
  /// around the last reported value, so a healthy run stays quiet.
  /// Never returns an error: a slow cycle must not drop an arm that is standing
  /// still and healthy (see the disconnect check in read()).
  void report_read_rate(int64_t now_ns);

  /// Pick up a label the set_fsm_state callback has staged (see the comment on
  /// label_ in this header). Called at the top of pump_once(), so it works both
  /// on the async thread and while a lifecycle callback is pumping.
  void refresh_published_label();

  /// One synchronous exchange with z1_ctrl: pick up a pending label, drain the
  /// socket, fold the newest datagram into the state arrays, and - when the link
  /// is armed - put the current frame on the wire. **This is the only code that
  /// touches the socket.**
  ///
  /// read() calls it on the async worker thread, but only while the component is
  /// ACTIVE: ros2_control's AsyncComponentThread checks
  /// `get_lifecycle_state().id() == PRIMARY_STATE_ACTIVE` before every
  /// read()/write(), so *nothing* calls read() during on_activate(). The
  /// lifecycle and service callbacks therefore pump the socket themselves, which
  /// is what makes the "listen for one RecvState, then ask for JOINTCTRL"
  /// handshake in on_activate() work at all. See z1_ros2_control/README.md.
  ///
  /// \returns ERROR only for a hard socket failure; a quiet arm is `OK`.
  hardware_interface::return_type pump_once(std::string & error);

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

  // ---- the label handshake, and the one lock in this class ------------------
  //
  // A label is 10 bytes and only four states want one, but the same four states
  // give z1_ctrl somewhere to jump to, so a torn label is not a cosmetic bug: it
  // would address a different entry of savedArmStates.csv. `label_`/`label_epoch_`
  // are written by the service thread and read by the async worker, so the writer
  // bumps the epoch after the bytes and read() re-checks it after copying - a
  // two-epoch read that fails is simply retried on the next 2 ms cycle, and the
  // service callback waits for the publish before it arms the state request. No
  // lock is involved, and read() never blocks on the service thread.
  std::array<std::atomic<char>, kLabelCapacity> label_{};
  std::atomic<uint32_t> label_epoch_{0};
  /// Owned by the async worker: the last label read() managed to verify.
  std::array<char, kLabelCapacity> published_label_{};
  std::atomic<uint32_t> published_label_epoch_{0};

  /// Set while one thread is inside pump_once()/write(), so that the async
  /// worker and a lifecycle or service callback cannot assemble and send at the
  /// same time. Lock-free on purpose; see IoGuard in the .cpp.
  std::atomic<bool> io_busy_{false};

  /// Serialises the service callbacks against each other (the executor may be
  /// multithreaded). Never taken by read()/write().
  std::mutex service_mutex_;
  rclcpp::Service<z1_ros2_control::srv::SetFSMState>::SharedPtr set_state_service_;
  rclcpp::Service<z1_ros2_control::srv::GetFSMState>::SharedPtr get_state_service_;

  // ---- diagnostics, only ever touched from the async thread -----------------
  std::array<int8_t, kMotorCount> motor_temperature_{};
  std::array<uint8_t, kMotorCount> motor_error_{};
  std::array<uint8_t, kMotorCount> motor_connected_{};
  std::string last_fault_message_;
  uint64_t cycles_ = 0;
  uint64_t cycles_without_packet_ = 0;

  // ---- measured transport rate (diagnostics only), async thread only --------
  int64_t rate_window_start_ns_ = 0;
  uint32_t rate_window_cycles_ = 0;
  /// 0 until the first measurement is logged.
  double rate_reference_hz_ = 0.0;

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
