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

#ifndef Z1_ROS2_CONTROL__Z1_UDP_CLIENT_HPP_
#define Z1_ROS2_CONTROL__Z1_UDP_CLIENT_HPP_

#include <cstdint>
#include <string>

#include "z1_ros2_control/z1_protocol.hpp"

namespace z1_ros2_control
{

/// One end of the loopback link to `z1_ctrl`'s `ARMSDK`.
///
/// Deliberately dumb: a bound, connected, **non-blocking** UDP socket plus the
/// handful of counters needed to tell "the arm is quiet" apart from "we are
/// sending malformed packets". All policy (timeouts, FSM transitions, error
/// reporting) lives in `Z1System`.
///
/// Why this is hand-written instead of reusing Unitree's `UDPPort`
/// (`z1_controller/include/message/udp.h`): that class's constructor, `send`,
/// `recv` and `resetIO` are defined inside the prebuilt `libZ1_x86_64.so`, so
/// using it would mean linking Unitree's whole FSM into the ros2_control
/// process. The datagram handling we actually need is about 40 lines.
class Z1UdpClient
{
public:
  Z1UdpClient() = default;
  ~Z1UdpClient();

  Z1UdpClient(const Z1UdpClient &) = delete;
  Z1UdpClient & operator=(const Z1UdpClient &) = delete;

  /// Create the socket, bind `own_port`, connect to `to_ip:to_port` and switch
  /// to non-blocking. Returns false and fills `error` on failure.
  bool open(const std::string & to_ip, uint16_t to_port, uint16_t own_port, std::string & error);

  void close();

  bool is_open() const { return fd_ >= 0; }

  /// Send one SendCmd. The header bytes and the length are checked here so a
  /// bug in the caller cannot put a malformed datagram on the wire.
  /// Returns false and fills `error` on failure (including EAGAIN).
  bool send(const UNITREE_ARM::SendCmd & cmd, std::string & error);

  /// Try to receive one RecvState.
  ///
  /// \returns `Received`  a well-formed 208-byte packet with the right header
  ///          `NoData`    nothing pending on the socket (the normal case for
  ///                      most calls - the arm answers at its own cadence)
  ///          `Malformed` a datagram arrived but was dropped, see `bad_length()`
  ///                      / `bad_head()` to tell which
  ///          `Failed`    a real socket error, `error` is filled
  enum class RecvResult { Received, NoData, Malformed, Failed };
  RecvResult recv(UNITREE_ARM::RecvState & state, std::string & error);

  // Counters, for diagnostics and for the mock's self-check.
  uint64_t sent() const { return sent_; }
  uint64_t received() const { return received_; }
  uint64_t bad_length() const { return bad_length_; }
  uint64_t bad_head() const { return bad_head_; }

private:
  int fd_ = -1;
  uint64_t sent_ = 0;
  uint64_t received_ = 0;
  uint64_t bad_length_ = 0;
  uint64_t bad_head_ = 0;
};

}  // namespace z1_ros2_control

#endif  // Z1_ROS2_CONTROL__Z1_UDP_CLIENT_HPP_
