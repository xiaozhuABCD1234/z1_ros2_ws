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

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <type_traits>

#include "z1_ros2_control/z1_protocol.hpp"
#include "z1_ros2_control/z1_udp_client.hpp"

namespace
{

using z1_ros2_control::kHead0;
using z1_ros2_control::kHead1;

/// Bind a socket to an ephemeral loopback port and report which one we got.
/// The socket is closed immediately, so this is only a hint - good enough for a
/// unit test, and far more robust than hardcoding 8071/8072.
uint16_t free_udp_port()
{
  const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    return 0;
  }
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  address.sin_port = 0;
  if (::bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
    ::close(fd);
    return 0;
  }
  socklen_t length = sizeof(address);
  if (::getsockname(fd, reinterpret_cast<sockaddr *>(&address), &length) < 0) {
    ::close(fd);
    return 0;
  }
  const uint16_t port = ntohs(address.sin_port);
  ::close(fd);
  return port;
}

/// The controller side of the link, for tests: bound, non-blocking loopback.
class FakeZ1Ctrl
{
public:
  explicit FakeZ1Ctrl(uint16_t port)
  {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
      return;
    }
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(port);
    if (::bind(fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
      ::close(fd_);
      fd_ = -1;
      return;
    }
    timeval timeout{};
    timeout.tv_usec = 200000;
    ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  }

  ~FakeZ1Ctrl()
  {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

  bool valid() const { return fd_ >= 0; }

  ssize_t receive(void * buffer, size_t length, sockaddr_in & from)
  {
    socklen_t from_length = sizeof(from);
    return ::recvfrom(fd_, buffer, length, 0, reinterpret_cast<sockaddr *>(&from), &from_length);
  }

  ssize_t reply(const void * buffer, size_t length, const sockaddr_in & to)
  {
    return ::sendto(
      fd_, buffer, length, 0, reinterpret_cast<const sockaddr *>(&to), sizeof(to));
  }

private:
  int fd_ = -1;
};

/// The definitions have to be copyable byte-for-byte: both ends of the link
/// memcpy them straight onto a socket.
TEST(Z1Protocol, StructuresAreTriviallyCopyable)
{
  EXPECT_TRUE(std::is_trivially_copyable<UNITREE_ARM::JointCmd>::value);
  EXPECT_TRUE(std::is_trivially_copyable<UNITREE_ARM::SendCmd>::value);
  EXPECT_TRUE(std::is_trivially_copyable<UNITREE_ARM::RecvState>::value);
}

/// The static_asserts in z1_protocol.hpp already fail the build if these move;
/// repeating them here makes the numbers visible in the test log and catches a
/// build that somehow lost the header.
TEST(Z1Protocol, SizesMatchTheWireProtocol)
{
  EXPECT_EQ(sizeof(UNITREE_ARM::JointCmd), 20u);
  EXPECT_EQ(sizeof(UNITREE_ARM::JointState), 22u);
  EXPECT_EQ(sizeof(UNITREE_ARM::Posture), 48u);
  EXPECT_EQ(sizeof(UNITREE_ARM::ValueUnion), 140u);
  EXPECT_EQ(sizeof(UNITREE_ARM::SendCmd), 147u);
  EXPECT_EQ(sizeof(UNITREE_ARM::RecvState), 208u);

  // ARMSDK::_sendRecv() compares the received length against exactly this.
  EXPECT_EQ(UNITREE_ARM::SENDCMD_LENGTH, 147);
  EXPECT_EQ(UNITREE_ARM::RECVSTATE_LENGTH, 208);
}

TEST(Z1Protocol, FieldOffsets)
{
  EXPECT_EQ(offsetof(UNITREE_ARM::SendCmd, head), 0u);
  EXPECT_EQ(offsetof(UNITREE_ARM::SendCmd, state), 2u);
  EXPECT_EQ(offsetof(UNITREE_ARM::SendCmd, track), 6u);
  EXPECT_EQ(offsetof(UNITREE_ARM::SendCmd, valueUnion), 7u);

  EXPECT_EQ(offsetof(UNITREE_ARM::RecvState, state), 2u);
  EXPECT_EQ(offsetof(UNITREE_ARM::RecvState, jointState), 6u);
  EXPECT_EQ(offsetof(UNITREE_ARM::RecvState, cartesianState), 160u);

  // joint i's Pos sits at 7 + 20*i + 8. The gripper is motor 6.
  EXPECT_EQ(offsetof(UNITREE_ARM::SendCmd, valueUnion) + 20u * 0u + 8u, 15u);
  EXPECT_EQ(offsetof(UNITREE_ARM::SendCmd, valueUnion) + 20u * 5u + 8u, 115u);
  EXPECT_EQ(offsetof(UNITREE_ARM::SendCmd, valueUnion) + 20u * 6u + 8u, 135u);
}

TEST(Z1Protocol, FsmStateNameRoundTrip)
{
  UNITREE_ARM::ArmFSMState parsed{};
  ASSERT_TRUE(z1_ros2_control::parseFsmStateName("JOINTCTRL", parsed));
  EXPECT_EQ(parsed, UNITREE_ARM::ArmFSMState::JOINTCTRL);
  // Case-insensitive, so a parameter file in lower case still works.
  ASSERT_TRUE(z1_ros2_control::parseFsmStateName("backtostart", parsed));
  EXPECT_EQ(parsed, UNITREE_ARM::ArmFSMState::BACKTOSTART);
  EXPECT_FALSE(z1_ros2_control::parseFsmStateName("NONSENSE", parsed));
  EXPECT_EQ(z1_ros2_control::fsmStateName(UNITREE_ARM::ArmFSMState::LOWCMD), std::string("LOWCMD"));
}

TEST(Z1Protocol, MotorErrorDecoding)
{
  EXPECT_STREQ(z1_ros2_control::motorErrorToString(0x00), "ok");
  EXPECT_STREQ(z1_ros2_control::motorErrorToString(0x01), "phase current too large");
  EXPECT_STREQ(z1_ros2_control::motorErrorToString(0x04), "overheat");
  // 0x40 is documented as "nothing" and must not be treated as a fault.
  EXPECT_EQ(z1_ros2_control::kErrorMask & 0x40, 0);
}

// ---------------------------------------------------------------------------
// The interesting part: what actually goes on the wire.
// ---------------------------------------------------------------------------

class Z1UdpClientTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ctrl_port_ = free_udp_port();
    own_port_ = free_udp_port();
    ASSERT_NE(ctrl_port_, 0);
    ASSERT_NE(own_port_, 0);
    ASSERT_NE(ctrl_port_, own_port_);
    server_ = std::make_unique<FakeZ1Ctrl>(ctrl_port_);
    ASSERT_TRUE(server_->valid());

    std::string error;
    ASSERT_TRUE(client_.open("127.0.0.1", ctrl_port_, own_port_, error)) << error;
  }

  uint16_t ctrl_port_ = 0;
  uint16_t own_port_ = 0;
  std::unique_ptr<FakeZ1Ctrl> server_;
  z1_ros2_control::Z1UdpClient client_;
};

TEST_F(Z1UdpClientTest, SendsA147ByteFrameAtTheDocumentedOffsets)
{
  UNITREE_ARM::SendCmd cmd{};
  cmd.head[0] = kHead0;
  cmd.head[1] = kHead1;
  cmd.state = UNITREE_ARM::ArmFSMState::JOINTCTRL;
  cmd.track = true;
  cmd.valueUnion.jointCmd[3].Pos = 0.75F;
  cmd.valueUnion.jointCmd[6].Pos = -1.5F;

  std::string error;
  ASSERT_TRUE(client_.send(cmd, error)) << error;
  EXPECT_EQ(client_.sent(), 1u);

  uint8_t buffer[512];
  sockaddr_in from{};
  const ssize_t received = server_->receive(buffer, sizeof(buffer), from);
  ASSERT_EQ(received, 147);
  EXPECT_EQ(ntohs(from.sin_port), own_port_) << "the client must send from the port it bound";

  EXPECT_EQ(buffer[0], kHead0);
  EXPECT_EQ(buffer[1], kHead1);

  int32_t state = 0;
  std::memcpy(&state, buffer + 2, sizeof(state));
  EXPECT_EQ(state, static_cast<int32_t>(UNITREE_ARM::ArmFSMState::JOINTCTRL));
  EXPECT_EQ(buffer[6], 1) << "track";

  float position = 0.0F;
  std::memcpy(&position, buffer + 75, sizeof(position));  // 7 + 3*20 + 8
  EXPECT_FLOAT_EQ(position, 0.75F);

  std::memcpy(&position, buffer + 135, sizeof(position));  // 7 + 6*20 + 8
  EXPECT_FLOAT_EQ(position, -1.5F);
}

TEST_F(Z1UdpClientTest, ReceivesA208ByteState)
{
  UNITREE_ARM::SendCmd cmd{};
  cmd.head[0] = kHead0;
  cmd.head[1] = kHead1;
  std::string error;
  ASSERT_TRUE(client_.send(cmd, error)) << error;

  uint8_t buffer[512];
  sockaddr_in from{};
  ASSERT_EQ(server_->receive(buffer, sizeof(buffer), from), 147);

  UNITREE_ARM::RecvState reply{};
  reply.head[0] = kHead0;
  reply.head[1] = kHead1;
  reply.state = UNITREE_ARM::ArmFSMState::JOINTCTRL;
  reply.jointState[2].Pos = 1.25F;
  reply.jointState[2].W = -0.5F;
  reply.jointState[2].T = 3.5F;
  reply.jointState[6].Pos = -0.25F;
  ASSERT_EQ(server_->reply(&reply, sizeof(reply), from), static_cast<ssize_t>(sizeof(reply)));

  UNITREE_ARM::RecvState got{};
  ASSERT_EQ(client_.recv(got, error), z1_ros2_control::Z1UdpClient::RecvResult::Received)
    << error;
  EXPECT_EQ(client_.received(), 1u);
  EXPECT_EQ(got.state, UNITREE_ARM::ArmFSMState::JOINTCTRL);
  EXPECT_FLOAT_EQ(got.jointState[2].Pos, 1.25F);
  EXPECT_FLOAT_EQ(got.jointState[2].W, -0.5F);
  EXPECT_FLOAT_EQ(got.jointState[2].T, 3.5F);
  EXPECT_FLOAT_EQ(got.jointState[6].Pos, -0.25F);
}

TEST_F(Z1UdpClientTest, ReportsNoDataWhenTheArmIsQuiet)
{
  std::string error;
  UNITREE_ARM::RecvState got{};
  EXPECT_EQ(
    client_.recv(got, error), z1_ros2_control::Z1UdpClient::RecvResult::NoData);
  EXPECT_TRUE(error.empty());
}

TEST_F(Z1UdpClientTest, DropsADatagramOfTheWrongLength)
{
  UNITREE_ARM::SendCmd cmd{};
  cmd.head[0] = kHead0;
  cmd.head[1] = kHead1;
  std::string error;
  ASSERT_TRUE(client_.send(cmd, error)) << error;

  uint8_t buffer[512];
  sockaddr_in from{};
  ASSERT_EQ(server_->receive(buffer, sizeof(buffer), from), 147);
  const uint8_t short_reply[100] = {};
  ASSERT_EQ(server_->reply(short_reply, sizeof(short_reply), from), 100);

  UNITREE_ARM::RecvState got{};
  EXPECT_EQ(client_.recv(got, error), z1_ros2_control::Z1UdpClient::RecvResult::Malformed);
  EXPECT_EQ(client_.bad_length(), 1u);
  // And the socket is drained, so we do not report the same packet forever.
  EXPECT_EQ(client_.recv(got, error), z1_ros2_control::Z1UdpClient::RecvResult::NoData);
  EXPECT_EQ(client_.bad_length(), 1u);
}

TEST_F(Z1UdpClientTest, DropsADatagramWithTheWrongHeader)
{
  UNITREE_ARM::SendCmd cmd{};
  cmd.head[0] = kHead0;
  cmd.head[1] = kHead1;
  std::string error;
  ASSERT_TRUE(client_.send(cmd, error)) << error;

  uint8_t buffer[512];
  sockaddr_in from{};
  ASSERT_EQ(server_->receive(buffer, sizeof(buffer), from), 147);

  UNITREE_ARM::RecvState reply{};  // head left at {0, 0}
  reply.state = UNITREE_ARM::ArmFSMState::PASSIVE;
  ASSERT_EQ(server_->reply(&reply, sizeof(reply), from), static_cast<ssize_t>(sizeof(reply)));

  UNITREE_ARM::RecvState got{};
  EXPECT_EQ(client_.recv(got, error), z1_ros2_control::Z1UdpClient::RecvResult::Malformed);
  EXPECT_EQ(client_.bad_head(), 1u);
  EXPECT_EQ(client_.received(), 0u);
}

TEST_F(Z1UdpClientTest, RefusesToSendAFrameWithAWrongHeader)
{
  UNITREE_ARM::SendCmd cmd{};  // head {0, 0}
  std::string error;
  EXPECT_FALSE(client_.send(cmd, error));
  EXPECT_FALSE(error.empty());
  EXPECT_EQ(client_.sent(), 0u);
}

TEST_F(Z1UdpClientTest, FailsWhenNobodyIsListening)
{
  // Nothing is bound to this port, so the kernel answers with ICMP
  // port-unreachable and a connected UDP socket surfaces ECONNREFUSED.
  const uint16_t dead_port = free_udp_port();
  z1_ros2_control::Z1UdpClient client;
  std::string error;
  ASSERT_TRUE(client.open("127.0.0.1", dead_port, free_udp_port(), error)) << error;

  UNITREE_ARM::SendCmd cmd{};
  cmd.head[0] = kHead0;
  cmd.head[1] = kHead1;
  // The first send usually succeeds; the error is reported on a later call.
  client.send(cmd, error);
  UNITREE_ARM::RecvState got{};
  bool saw_failure = false;
  for (int attempt = 0; attempt < 20 && !saw_failure; ++attempt) {
    if (client.recv(got, error) == z1_ros2_control::Z1UdpClient::RecvResult::Failed) {
      saw_failure = true;
    }
    usleep(5000);
  }
  EXPECT_TRUE(saw_failure) << "a closed controller port should be reported as a link failure";
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
