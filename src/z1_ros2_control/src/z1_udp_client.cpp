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

#include "z1_ros2_control/z1_udp_client.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <string>

namespace z1_ros2_control
{

namespace
{
/// A datagram we are willing to read. Anything longer is dropped, but we still
/// have to read it off the socket so it does not pile up in the receive queue.
constexpr size_t kRecvScratch = 512;

std::string errnoString(const char * what)
{
  return std::string(what) + ": " + std::strerror(errno);
}
}  // namespace

Z1UdpClient::~Z1UdpClient() { close(); }

void Z1UdpClient::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool Z1UdpClient::open(
  const std::string & to_ip, uint16_t to_port, uint16_t own_port, std::string & error)
{
  close();

  fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ < 0) {
    error = errnoString("socket");
    return false;
  }

  // z1_ctrl restarts during bring-up and rebinds 8071; without SO_REUSEADDR our
  // bind of 8072 can fail with EADDRINUSE while the old socket is still around.
  int on = 1;
  if (::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) < 0) {
    error = errnoString("setsockopt(SO_REUSEADDR)");
    close();
    return false;
  }

  sockaddr_in own{};
  own.sin_family = AF_INET;
  own.sin_port = htons(own_port);
  // z1_ctrl sends its RecvState to 127.0.0.1:<own_port>. Binding loopback when
  // we are talking to loopback keeps the port off every other interface; a
  // non-loopback controller IP (multi-host experiments) needs INADDR_ANY.
  own.sin_addr.s_addr =
    (to_ip == "127.0.0.1") ? htonl(INADDR_LOOPBACK) : htonl(INADDR_ANY);

  if (::bind(fd_, reinterpret_cast<sockaddr *>(&own), sizeof(own)) < 0) {
    error = errnoString("bind") + " (own_port=" + std::to_string(own_port) + ")";
    close();
    return false;
  }

  sockaddr_in to{};
  to.sin_family = AF_INET;
  to.sin_port = htons(to_port);
  if (::inet_pton(AF_INET, to_ip.c_str(), &to.sin_addr) != 1) {
    error = "ctrl_ip is not a valid IPv4 address: " + to_ip;
    close();
    return false;
  }

  // connect() on a UDP socket does not hand-shake; it fixes the peer so that
  // plain send()/recv() work and so that the kernel drops datagrams that did
  // not come from z1_ctrl.
  if (::connect(fd_, reinterpret_cast<sockaddr *>(&to), sizeof(to)) < 0) {
    error = errnoString("connect");
    close();
    return false;
  }

  // Non-blocking: read() runs inside the async worker thread at 500 Hz and must
  // never wait for the arm.
  const int flags = ::fcntl(fd_, F_GETFL, 0);
  if (flags < 0 || ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
    error = errnoString("fcntl(O_NONBLOCK)");
    close();
    return false;
  }

  sent_ = received_ = bad_length_ = bad_head_ = 0;
  return true;
}

bool Z1UdpClient::send(const UNITREE_ARM::SendCmd & cmd, std::string & error)
{
  if (fd_ < 0) {
    error = "socket is not open";
    return false;
  }

  // Fail loudly on our side rather than have z1_ctrl silently discard us:
  // ARMSDK::_sendRecv() requires exactly this length and header.
  if (cmd.head[0] != kHead0 || cmd.head[1] != kHead1) {
    error = "refusing to send a command with a wrong header";
    return false;
  }

  const ssize_t n = ::send(fd_, &cmd, sizeof(cmd), 0);
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      error = "send would block";
    } else if (errno == ECONNREFUSED) {
      // Connected UDP surfaces ICMP port-unreachable here: nobody is bound to
      // the controller port, i.e. z1_ctrl is not running.
      error = "connection refused - is z1_ctrl running and bound to the controller port?";
    } else {
      error = errnoString("send");
    }
    return false;
  }
  if (static_cast<size_t>(n) != sizeof(cmd)) {
    error = "short send: " + std::to_string(n) + " of " + std::to_string(sizeof(cmd));
    return false;
  }

  ++sent_;
  return true;
}

Z1UdpClient::RecvResult Z1UdpClient::recv(UNITREE_ARM::RecvState & state, std::string & error)
{
  if (fd_ < 0) {
    error = "socket is not open";
    return RecvResult::Failed;
  }

  // Peek first so that we can tell "nothing yet" from "wrong size". MSG_TRUNC
  // makes the kernel report the datagram's true length even though the peek
  // buffer is one byte, and MSG_PEEK leaves the datagram queued for the real
  // read below.
  uint8_t peek = 0;
  const ssize_t available = ::recv(fd_, &peek, sizeof(peek), MSG_PEEK | MSG_TRUNC);
  if (available < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return RecvResult::NoData;
    }
    if (errno == ECONNREFUSED) {
      error = "connection refused - is z1_ctrl running and bound to the controller port?";
    } else {
      error = errnoString("recv(MSG_PEEK)");
    }
    return RecvResult::Failed;
  }

  if (static_cast<size_t>(available) != sizeof(UNITREE_ARM::RecvState)) {
    // Drain it, otherwise the malformed datagram is returned by every
    // subsequent call and we spin on the same packet forever.
    uint8_t scratch[kRecvScratch];
    const ssize_t dropped = ::recv(fd_, scratch, sizeof(scratch), 0);
    if (dropped < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      error = errnoString("recv(drop)");
      return RecvResult::Failed;
    }
    ++bad_length_;
    error = "dropped a " + std::to_string(available) + "-byte datagram, expected " +
            std::to_string(sizeof(UNITREE_ARM::RecvState));
    return RecvResult::Malformed;
  }

  const ssize_t n = ::recv(fd_, &state, sizeof(state), 0);
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      // Another reader won the race; treat as "nothing".
      return RecvResult::NoData;
    }
    error = errnoString("recv");
    return RecvResult::Failed;
  }
  if (static_cast<size_t>(n) != sizeof(state)) {
    error = "short recv: " + std::to_string(n);
    return RecvResult::Failed;
  }

  // Mirror ARMSDK::_sendRecv(), which rejects commands without this header. If
  // z1_ctrl changes its magic we want to notice, not to drive the arm blind.
  if (state.head[0] != kHead0 || state.head[1] != kHead1) {
    ++bad_head_;
    error = "dropped a datagram with a wrong header";
    return RecvResult::Malformed;
  }

  ++received_;
  return RecvResult::Received;
}

}  // namespace z1_ros2_control
