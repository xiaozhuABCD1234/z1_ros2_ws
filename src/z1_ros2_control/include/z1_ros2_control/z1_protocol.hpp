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

#ifndef Z1_ROS2_CONTROL__Z1_PROTOCOL_HPP_
#define Z1_ROS2_CONTROL__Z1_PROTOCOL_HPP_

#include <cctype>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>

// The Unitree Z1 wire protocol, vendored verbatim. `thirdparty/` is on the
// include path (see CMakeLists.txt). See thirdparty/unitree/z1/PROVENANCE.md for
// the upstream commit, the licence and why these layouts are trusted.
#include "unitree/z1/arm_common.h"

namespace z1_ros2_control
{

// ---------------------------------------------------------------------------
// Link parameters.
//
// z1_ctrl's client-facing ports are hardcoded in its main.cpp:
//
//   ctrlComp->cmdPanel = new ARMSDK(events, emptyAction, "127.0.0.1", 8072, 8071, 0.002);
//                                                                   ^sdkPort ^ownPort
//
// so z1_ctrl BINDS 8071, SENDS its RecvState to 8072, and the client must bind
// 8072. The IP is loopback: z1_ctrl and this hardware interface have to run on
// the same machine. For a second arm upstream's own instructions are to edit
// that line (8074/8073) plus config/config.xml.
// ---------------------------------------------------------------------------
constexpr const char * kDefaultCtrlIp = "127.0.0.1";
constexpr uint16_t kDefaultCtrlPort = 8071;   // z1_ctrl's ARMSDK binds this
constexpr uint16_t kDefaultOwnPort = 8072;    // we bind this; z1_ctrl sends here

// ARMSDK::_sendRecv() rejects anything whose length is not exactly
// SENDCMD_LENGTH and whose first two bytes are not this magic.
constexpr uint8_t kHead0 = 0xFE;
constexpr uint8_t kHead1 = 0xFF;

// ---------------------------------------------------------------------------
// Layout assertions.
//
// These are the whole point of this header. Both ends of this link are raw byte
// layouts on a UDP socket, so a compiler that inserts one byte of padding - or
// an upstream header change - would otherwise corrupt traffic silently at
// runtime. Asserting here turns all of that into a build failure.
// ---------------------------------------------------------------------------
static_assert(sizeof(UNITREE_ARM::ArmFSMState) == 4, "ArmFSMState must be 4 bytes (enum class int)");
static_assert(sizeof(UNITREE_ARM::JointCmd) == 20, "JointCmd must be 20 bytes");
static_assert(sizeof(UNITREE_ARM::Motor_State) == 3, "Motor_State must be 3 bytes");
static_assert(sizeof(UNITREE_ARM::JointState) == 22, "JointState must be 22 bytes");
static_assert(sizeof(UNITREE_ARM::Posture) == 48, "Posture must be 48 bytes");
static_assert(sizeof(UNITREE_ARM::TrajCmd) == 128, "TrajCmd must be 128 bytes");
static_assert(sizeof(UNITREE_ARM::ValueUnion) == 140, "ValueUnion must be 140 bytes");

static_assert(sizeof(UNITREE_ARM::SendCmd) == 147, "SendCmd must be 147 bytes");
static_assert(sizeof(UNITREE_ARM::RecvState) == 208, "RecvState must be 208 bytes");

// The upstream constants have to agree with the sizes above.
static_assert(UNITREE_ARM::SENDCMD_LENGTH == 147, "upstream SENDCMD_LENGTH disagrees");
static_assert(UNITREE_ARM::RECVSTATE_LENGTH == 208, "upstream RECVSTATE_LENGTH disagrees");

// Field offsets that the UDP client and the mock both depend on.
static_assert(offsetof(UNITREE_ARM::SendCmd, head) == 0, "SendCmd.head offset");
static_assert(offsetof(UNITREE_ARM::SendCmd, state) == 2, "SendCmd.state offset");
static_assert(offsetof(UNITREE_ARM::SendCmd, track) == 6, "SendCmd.track offset");
static_assert(offsetof(UNITREE_ARM::SendCmd, valueUnion) == 7, "SendCmd.valueUnion offset");

// 7 * 20 = 140 bytes of JointCmd, so joint i sits at 7 + 20*i and its Pos at
// 15 + 20*i. The gripper is motor index 6, at offset 127..146.
static_assert(offsetof(UNITREE_ARM::SendCmd, valueUnion) + 6 * sizeof(UNITREE_ARM::JointCmd) +
                  offsetof(UNITREE_ARM::JointCmd, Pos) ==
                135,
              "jointCmd[6].Pos (the gripper) offset");

static_assert(offsetof(UNITREE_ARM::RecvState, state) == 2, "RecvState.state offset");
static_assert(offsetof(UNITREE_ARM::RecvState, jointState) == 6, "RecvState.jointState offset");
static_assert(offsetof(UNITREE_ARM::RecvState, cartesianState) == 160,
              "RecvState.cartesianState offset");

// ---------------------------------------------------------------------------
// Number of motors in a packet. This is fixed by the protocol - the array in
// SendCmd/RecvState is always 7 wide - regardless of how many joints the URDF
// happens to describe. index 0..5 are the arm, index 6 is the gripper.
// ---------------------------------------------------------------------------
constexpr size_t kMotorCount = 7;
constexpr size_t kArmJointCount = 6;
constexpr size_t kGripperIndex = 6;

// Upstream's over-temperature limit (LowlevelState::temporatureLimit).
constexpr int8_t kDefaultTemperatureLimit = 80;

// ---------------------------------------------------------------------------
// Labels.
//
// Four states are addressed not by joint angles but by a *name*: z1_ctrl reads
// `ValueUnion::name`, looks the name up in its config/savedArmStates.csv and
// drives the arm to (or saves the current pose as) that entry. The official SDK
// calls them labelRun / labelSave / teach / teachRepeat.
//
// The field is a plain `char[10]` and z1_ctrl builds a std::string out of it, so
// the label has to be NUL-terminated inside those bytes: at most
// kMaxLabelLength characters.
// ---------------------------------------------------------------------------
constexpr size_t kLabelCapacity = 10;
static_assert(
  sizeof(UNITREE_ARM::ValueUnion{}.name) == kLabelCapacity, "ValueUnion::name must be 10 bytes");

/// Longest label accepted on the wire: what fits in `name` plus a NUL.
constexpr size_t kMaxLabelLength = kLabelCapacity - 1;

// Motor_State::error bits, from the vendored header's comment block.
constexpr uint8_t kErrorPhaseCurrent = 0x01;
constexpr uint8_t kErrorPhaseLeakage = 0x02;
constexpr uint8_t kErrorOverheat = 0x04;
constexpr uint8_t kErrorJumped = 0x20;
constexpr uint8_t kErrorMask =
  kErrorPhaseCurrent | kErrorPhaseLeakage | kErrorOverheat | kErrorJumped;

/// Human-readable name for a motor error byte, for diagnostics.
inline const char * motorErrorToString(uint8_t error)
{
  switch (error & kErrorMask) {
    case 0:
      return "ok";
    case kErrorPhaseCurrent:
      return "phase current too large";
    case kErrorPhaseLeakage:
      return "phase leakage";
    case kErrorOverheat:
      return "overheat";
    case kErrorJumped:
      return "jumped";
    default:
      return "multiple faults";
  }
}

/// `ArmFSMState` -> the string upstream uses on the wire/keyboard, for logs.
inline const char * fsmStateName(UNITREE_ARM::ArmFSMState state)
{
  switch (state) {
    case UNITREE_ARM::ArmFSMState::INVALID:
      return "INVALID";
    case UNITREE_ARM::ArmFSMState::PASSIVE:
      return "PASSIVE";
    case UNITREE_ARM::ArmFSMState::JOINTCTRL:
      return "JOINTCTRL";
    case UNITREE_ARM::ArmFSMState::CARTESIAN:
      return "CARTESIAN";
    case UNITREE_ARM::ArmFSMState::MOVEJ:
      return "MOVEJ";
    case UNITREE_ARM::ArmFSMState::MOVEL:
      return "MOVEL";
    case UNITREE_ARM::ArmFSMState::MOVEC:
      return "MOVEC";
    case UNITREE_ARM::ArmFSMState::TRAJECTORY:
      return "TRAJECTORY";
    case UNITREE_ARM::ArmFSMState::TOSTATE:
      return "TOSTATE";
    case UNITREE_ARM::ArmFSMState::SAVESTATE:
      return "SAVESTATE";
    case UNITREE_ARM::ArmFSMState::TEACH:
      return "TEACH";
    case UNITREE_ARM::ArmFSMState::TEACHREPEAT:
      return "TEACHREPEAT";
    case UNITREE_ARM::ArmFSMState::CALIBRATION:
      return "CALIBRATION";
    case UNITREE_ARM::ArmFSMState::SETTRAJ:
      return "SETTRAJ";
    case UNITREE_ARM::ArmFSMState::BACKTOSTART:
      return "BACKTOSTART";
    case UNITREE_ARM::ArmFSMState::NEXT:
      return "NEXT";
    case UNITREE_ARM::ArmFSMState::LOWCMD:
      return "LOWCMD";
  }
  return "UNKNOWN";
}

/// Parse one of the names printed by `fsmStateName` (case-insensitive).
/// Returns false for an unknown name; `out` is untouched then.
///
/// `LOWCMD` is deliberately absent from the table below: it hands raw motor
/// commands to the arm and bypasses z1_ctrl's joint servo law, so nothing in
/// this workspace may ask for it. `INVALID`, `NEXT` and the retired `SETTRAJ`
/// are internal markers rather than states one can request.
inline bool parseFsmStateName(const std::string & name, UNITREE_ARM::ArmFSMState & out)
{
  static const std::pair<const char *, UNITREE_ARM::ArmFSMState> kTable[] = {
    {"PASSIVE", UNITREE_ARM::ArmFSMState::PASSIVE},
    {"JOINTCTRL", UNITREE_ARM::ArmFSMState::JOINTCTRL},
    {"CARTESIAN", UNITREE_ARM::ArmFSMState::CARTESIAN},
    {"MOVEJ", UNITREE_ARM::ArmFSMState::MOVEJ},
    {"MOVEL", UNITREE_ARM::ArmFSMState::MOVEL},
    {"MOVEC", UNITREE_ARM::ArmFSMState::MOVEC},
    {"TRAJECTORY", UNITREE_ARM::ArmFSMState::TRAJECTORY},
    {"TOSTATE", UNITREE_ARM::ArmFSMState::TOSTATE},
    {"SAVESTATE", UNITREE_ARM::ArmFSMState::SAVESTATE},
    {"TEACH", UNITREE_ARM::ArmFSMState::TEACH},
    {"TEACHREPEAT", UNITREE_ARM::ArmFSMState::TEACHREPEAT},
    {"CALIBRATION", UNITREE_ARM::ArmFSMState::CALIBRATION},
    {"BACKTOSTART", UNITREE_ARM::ArmFSMState::BACKTOSTART},
  };
  for (const auto & [text, state] : kTable) {
    if (name.size() == std::string(text).size()) {
      bool equal = true;
      for (size_t i = 0; i < name.size(); ++i) {
        const auto a = static_cast<char>(::toupper(static_cast<unsigned char>(name[i])));
        if (a != text[i]) {
          equal = false;
          break;
        }
      }
      if (equal) {
        out = state;
        return true;
      }
    }
  }
  return false;
}

/// Names accepted in the `activate_fsm_sequence` parameter, and by
/// `set_fsm_state`.
///
/// These are the states this component can drive with nothing but a joint
/// position command: exactly the ones that read neither `valueUnion.name` nor a
/// posture / trajectory payload out of `SendCmd`. The states that are missing,
/// and why (checked against `libZ1_x86_64.so`'s FSM; see
/// thirdparty/unitree/z1/PROVENANCE.md):
///
///  * `CARTESIAN` reads `valueUnion` as six *doubles* (posture deltas), not as
///    `jointCmd[]`: `movsd 0x7b(%rax)` in `State_Cartesian::run()` is
///    `jointCmd[1].Pos`'s byte reinterpreted as a double. Since we only ever
///    write floats there, the deltas would be garbage.
///  * `MOVEJ` / `MOVEL` / `MOVEC` need a Cartesian target, and `TRAJECTORY`
///    reads `valueUnion.trajCmd`; we send neither.
///  * `LOWCMD` bypasses z1_ctrl's joint servo law entirely.
inline const char * safeFsmStates()
{
  return "PASSIVE|JOINTCTRL|BACKTOSTART|CALIBRATION";
}

/// States addressed by a label rather than by joint angles: `TOSTATE` and
/// `TEACH` / `TEACHREPEAT` / `SAVESTATE` build a std::string from
/// `valueUnion.name` (`lea 0x5f(%rcx)` in each of their enter() functions,
/// i.e. `SendCmd.valueUnion`), so a request for one of them has to carry one.
///
/// They are deliberately *not* part of `safeFsmStates()`: `activate_fsm_sequence`
/// runs before any controller is alive to supply a label, so the union would
/// still be empty and z1_ctrl would be asked to move to the entry whose name is
/// the empty string. They are reachable through the `set_fsm_state` service
/// instead, which always sends a label first.
inline const char * labelledFsmStates()
{
  return "TOSTATE|SAVESTATE|TEACH|TEACHREPEAT";
}

/// States the `set_fsm_state` service accepts: `safeFsmStates()` plus
/// `labelledFsmStates()`.
inline const char * serviceFsmStates()
{
  return "PASSIVE|JOINTCTRL|BACKTOSTART|CALIBRATION|TOSTATE|SAVESTATE|TEACH|TEACHREPEAT";
}

/// True for the states that read `SendCmd::valueUnion.name`.
inline bool fsmStateTakesLabel(UNITREE_ARM::ArmFSMState state)
{
  switch (state) {
    case UNITREE_ARM::ArmFSMState::TOSTATE:
    case UNITREE_ARM::ArmFSMState::SAVESTATE:
    case UNITREE_ARM::ArmFSMState::TEACH:
    case UNITREE_ARM::ArmFSMState::TEACHREPEAT:
      return true;
    default:
      return false;
  }
}

/// Member of `safeFsmStates()`, i.e. legal in `activate_fsm_sequence`.
inline bool isSafeFsmState(UNITREE_ARM::ArmFSMState state)
{
  switch (state) {
    case UNITREE_ARM::ArmFSMState::PASSIVE:
    case UNITREE_ARM::ArmFSMState::JOINTCTRL:
    case UNITREE_ARM::ArmFSMState::BACKTOSTART:
    case UNITREE_ARM::ArmFSMState::CALIBRATION:
      return true;
    default:
      return false;
  }
}

/// Member of `serviceFsmStates()`, i.e. legal in a `set_fsm_state` request.
inline bool isServiceFsmState(UNITREE_ARM::ArmFSMState state)
{
  return isSafeFsmState(state) || fsmStateTakesLabel(state);
}

}  // namespace z1_ros2_control

#endif  // Z1_ROS2_CONTROL__Z1_PROTOCOL_HPP_
