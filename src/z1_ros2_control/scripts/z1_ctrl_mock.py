#!/usr/bin/env python3
# Copyright 2026 the z1_ros2_ws authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Stand-in for the official `z1_ctrl`, so the ROS 2 stack can be tested with no arm.

It plays the server half of the loopback link:

  * binds 127.0.0.1:8071 (that is where `ARMSDK` in z1_ctrl binds)
  * every `--rate` Hz sends a 208-byte `RecvState` to 127.0.0.1:8072
  * receives 147-byte `SendCmd` datagrams and checks their header and length
  * integrates a trivial first-order joint model towards the commanded
    positions, so trajectories actually converge and
    `joint_trajectory_controller` does not abort on a goal tolerance

It also reproduces the two official behaviours that matter for safety testing:

  * the outgoing `RecvState.state` is whatever this script currently believes the
    FSM is, so a client can be tested against a controller that refuses a
    transition (`--reject-states`);
  * if no `SendCmd` arrives for `--disconnect-ms`, the state is forced back to
    PASSIVE. `ARMSDK::_sendRecv()` does exactly this, which is why a crashing ROS
    node makes a real Z1 go limp.

Examples
--------
    # plain 6-axis arm, quiet
    ros2 run z1_ros2_control z1_ctrl_mock.py

    # with a gripper joint present, and a 5% packet loss rate to exercise the
    # disconnect detection
    ros2 run z1_ros2_control z1_ctrl_mock.py --gripper --drop-rate 0.05

    # refuse PASSIVE->JOINTCTRL, to see the activation handshake fail cleanly
    ros2 run z1_ros2_control z1_ctrl_mock.py --reject-states JOINTCTRL
"""

import argparse
import random
import socket
import struct
import sys
import time

# ---------------------------------------------------------------------------
# Wire format. These must stay byte-for-byte identical to
# thirdparty/unitree/z1/arm_common.h - the C++ side pins the same numbers down
# with static_assert, and test/test_z1_protocol.cpp checks them at runtime.
#
# SendCmd:   2 head + 4 state + 1 track + 140 valueUnion            = 147
# RecvState: 2 head + 4 state + 7*22 jointState + 48 posture        = 208
# JointCmd:  5 float                                                 = 20
# JointState: 4 float + 2 * Motor_State(int8, uint8, uint8)          = 22
# ---------------------------------------------------------------------------
HEAD = bytes([0xFE, 0xFF])
SEND_CMD = struct.Struct('<2B i ? 140s')
RECV_STATE = struct.Struct('<2B i ' + '4f6B' * 7 + '6d')
JOINT_CMD = struct.Struct('<5f')

MOTOR_COUNT = 7
ARM_JOINTS = 6
GRIPPER_INDEX = 6

# How many values RECV_STATE.pack() wants: two head bytes, the state, one
# JointState per motor (4 floats plus *two* Motor_State blocks of three), then the
# six Cartesian doubles. build_reply() checks itself against this, because getting
# it wrong is a struct.error on the very first send - the mock then exits without
# having sent a single datagram.
RECV_FIELD_COUNT = 2 + 1 + MOTOR_COUNT * (4 + 2 * 3) + 6

assert SEND_CMD.size == 147, SEND_CMD.size
assert RECV_STATE.size == 208, RECV_STATE.size
assert JOINT_CMD.size == 20, JOINT_CMD.size
assert RECV_FIELD_COUNT == 79, RECV_FIELD_COUNT

# ArmFSMState, from the vendored header.
FSM_STATES = {
    'INVALID': 0,
    'PASSIVE': 1,
    'JOINTCTRL': 2,
    'CARTESIAN': 3,
    'MOVEJ': 4,
    'MOVEL': 5,
    'MOVEC': 6,
    'TRAJECTORY': 7,
    'TOSTATE': 8,
    'SAVESTATE': 9,
    'TEACH': 10,
    'TEACHREPEAT': 11,
    'CALIBRATION': 12,
    'SETTRAJ': 13,
    'BACKTOSTART': 14,
    'NEXT': 15,
    'LOWCMD': 16,
}
STATE_NAMES = {value: name for name, value in FSM_STATES.items()}

PASSIVE = FSM_STATES['PASSIVE']
JOINTCTRL = FSM_STATES['JOINTCTRL']

# Motor_Connected.state, packed into the top two bits of a uint8. 0 means the
# motor is connected; nothing here ever sets a fault, that is what
# --motor-fault is for.
CONNECTED_OK = 0


def state_name(value):
    return STATE_NAMES.get(value, 'UNKNOWN({})'.format(value))


class MockZ1:
    def __init__(self, args):
        self.args = args
        self.joint_count = ARM_JOINTS + (1 if args.gripper else 0)

        # Trivial plant state: the arm starts wherever --initial says and then
        # chases the commanded position at no more than --max-vel rad/s.
        self.position = [0.0] * MOTOR_COUNT
        for index in range(min(self.joint_count, len(args.initial))):
            self.position[index] = args.initial[index]
        # The holder for an absent gripper tracks the measured angle, so the mock
        # starts it wherever it is and never moves it.
        self.velocity = [0.0] * MOTOR_COUNT
        self.target = list(self.position)

        self.fsm = PASSIVE
        self.track = False
        # Diagnostic counters.
        self.rx_packets = 0
        self.rx_bad_size = 0
        self.rx_bad_head = 0
        self.rx_bad_state = 0
        self.tx_packets = 0
        self.rx_dropped_for_test = 0
        self.last_rx_time = None
        self.forced_passive_count = 0
        self.states_seen = set()

    # -- socket -------------------------------------------------------------

    def open(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.args.bind_ip, self.args.bind_port))
        self.sock.setblocking(False)
        self.client = (self.args.client_ip, self.args.client_port)

    # -- protocol -----------------------------------------------------------

    def read_commands(self):
        """Drain every pending SendCmd. Keeps the newest, like the real client."""
        while True:
            try:
                data, _ = self.sock.recvfrom(2048)
            except BlockingIOError:
                return
            self.rx_packets += 1
            if len(data) != SEND_CMD.size:
                self.rx_bad_size += 1
                continue
            head, state, track, blob = SEND_CMD.unpack(data)
            if bytes([head[0], head[1]]) != HEAD:
                self.rx_bad_head += 1
                continue
            if state not in STATE_NAMES:
                self.rx_bad_state += 1
                continue

            previous = self.fsm
            if state in self.args.reject_states:
                # Refuse the transition but keep reporting the old state, which is
                # how a real illegal FSM transition looks from the client side.
                pass
            elif state != self.fsm:
                self.fsm = state
                if self.args.verbose:
                    print('[fsm] {} -> {}'.format(state_name(previous), state_name(self.fsm)))
            self.states_seen.add(state)
            self.track = track

            for index in range(self.joint_count):
                _t, _w, pos, _kp, _kw = JOINT_CMD.unpack_from(blob, JOINT_CMD.size * index)
                self.target[index] = pos

            self.last_rx_time = time.monotonic()

    def build_reply(self):
        """Pack one RecvState: the current FSM state plus every motor's telemetry."""
        fields = [HEAD[0], HEAD[1], self.fsm]
        for index in range(MOTOR_COUNT):
            torque = self.args.stiffness * (self.target[index] - self.position[index])
            torque = max(-self.args.torque_limit, min(self.args.torque_limit, torque))
            fields.extend([
                torque,
                self.velocity[index],
                0.0,  # acceleration, unused by the hardware interface
                self.position[index],
                # A JointState carries *two* Motor_State blocks, because a Z1 joint
                # can be driven by two motors. This mock models one motor per joint,
                # so the first block describes it and the second one stays zeroed.
                # That is the faithful choice, not a shortcut: `apply_state()` only
                # consults the second block when its temperature is non-zero, so a
                # zeroed block can never raise a phantom fault.
                (35 & 0xFF),  # temperature, Celsius
                (0x01 if self.args.motor_fault == index else 0) & 0xFF,
                CONNECTED_OK,
                0,  # second motor: temperature
                0,  # second motor: error
                0,  # second motor: connected
            ])
        # Cartesian posture is not used by the hardware interface; report zeros.
        fields.extend([0.0] * 6)
        if len(fields) != RECV_FIELD_COUNT:
            raise AssertionError('build_reply produced {} values, RecvState needs {}'.format(
                len(fields), RECV_FIELD_COUNT))
        return RECV_STATE.pack(*fields)

    # -- plant --------------------------------------------------------------

    def step(self, dt):
        """Move the joints towards their targets, with a velocity limit."""
        if self.fsm != JOINTCTRL or not self.track:
            for index in range(MOTOR_COUNT):
                self.velocity[index] = 0.0
            return

        for index in range(self.joint_count):
            error = self.target[index] - self.position[index]
            step = max(-self.args.max_vel * dt, min(self.args.max_vel * dt, error))
            self.position[index] += step
            self.velocity[index] = step / dt if dt > 0 else 0.0

    # -- main loop ----------------------------------------------------------

    def run(self):
        self.open()
        period = 1.0 / self.args.rate
        next_deadline = time.monotonic()
        disconnect_s = self.args.disconnect_ms / 1000.0
        report_next = time.monotonic() + 1.0
        verbose_next = time.monotonic() + self.args.verbose_period

        print('z1_ctrl mock: {} joints{} on {}:{} -> {}:{} at {} Hz'.format(
            self.joint_count, ' (with gripper)' if self.args.gripper else ' (no gripper)',
            self.args.bind_ip, self.args.bind_port, self.client[0], self.client[1],
            self.args.rate))
        print('Sending RecvState to {}:{}; waiting for z1_ros2_control/Z1System to show up ...'
              .format(self.client[0], self.client[1]))

        try:
            while True:
                next_deadline += period
                self.read_commands()

                # Mirror ARMSDK::_sendRecv(): no client traffic for long enough
                # and the controller drops to PASSIVE on its own.
                if self.last_rx_time is not None and self.fsm != PASSIVE:
                    if time.monotonic() - self.last_rx_time > disconnect_s:
                        print('[fsm] no SendCmd for {:.0f} ms -> forcing PASSIVE '
                              '(the real z1_ctrl does this too)'.format(self.args.disconnect_ms))
                        self.fsm = PASSIVE
                        self.track = False
                        self.forced_passive_count += 1

                self.step(period)

                if (self.args.drop_rate <= 0.0 or
                        random.random() >= self.args.drop_rate):
                    self.sock.sendto(self.build_reply(), self.client)
                    self.tx_packets += 1
                else:
                    self.rx_dropped_for_test += 1

                now = time.monotonic()
                if self.args.verbose and now >= verbose_next:
                    verbose_next = now + self.args.verbose_period
                    print('[state] fsm={} track={} q={} target={}'.format(
                        state_name(self.fsm), self.track,
                        ' '.join('{:+.3f}'.format(v) for v in self.position[:self.joint_count]),
                        ' '.join('{:+.3f}'.format(v) for v in self.target[:self.joint_count])))

                if now >= report_next:
                    report_next = now + 1.0
                    print('[stats] tx={} rx={} bad_size={} bad_head={} bad_state={} '
                          'drop_sim={} forced_passive={}'.format(
                              self.tx_packets, self.rx_packets, self.rx_bad_size,
                              self.rx_bad_head, self.rx_bad_state,
                              self.rx_dropped_for_test, self.forced_passive_count))
                    if self.rx_bad_header_seen():
                        print('  !! the client sent a datagram with a wrong header or length')

                sleep_for = next_deadline - time.monotonic()
                if sleep_for > 0:
                    time.sleep(sleep_for)
                elif sleep_for < -period:
                    # We fell behind; resynchronise instead of free-running.
                    next_deadline = time.monotonic()
        except KeyboardInterrupt:
            pass
        finally:
            self.sock.close()
            print('\nz1_ctrl mock stopped. tx={} rx={} bad_size={} bad_head={} bad_state={} '
                  'forced_passive={}'.format(
                      self.tx_packets, self.rx_packets, self.rx_bad_size, self.rx_bad_head,
                      self.rx_bad_state, self.forced_passive_count))
            print('states requested by the client: {}'.format(
                ', '.join(state_name(s) for s in sorted(self.states_seen)) or 'none'))
            if self.rx_packets > 0 and JOINTCTRL not in self.states_seen:
                print('NOTE: the client never asked for JOINTCTRL.')
        # Deliberately outside the finally block: a `return` in there would swallow
        # whatever the loop raised, so a crash on the very first packet would look
        # like a clean exit with status 0.
        return 0

    def rx_bad_header_seen(self):
        return self.rx_bad_size or self.rx_bad_head or self.rx_bad_state


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--bind-ip', default='127.0.0.1',
                        help='address to bind; z1_ctrl binds 8071 on loopback (default: %(default)s)')
    parser.add_argument('--bind-port', type=int, default=8071,
                        help='port z1_ctrl binds and the client sends to (default: %(default)s)')
    parser.add_argument('--client-ip', default='127.0.0.1',
                        help='where RecvState is sent, i.e. the client bind address '
                             '(default: %(default)s)')
    parser.add_argument('--client-port', type=int, default=8072,
                        help='port the client binds and z1_ctrl sends to (default: %(default)s)')
    parser.add_argument('--rate', type=float, default=500.0,
                        help='RecvState rate in Hz; z1_ctrl uses 500 (default: %(default)s)')
    parser.add_argument('--gripper', action='store_true',
                        help='pretend a gripper is fitted, so the 7th motor is commanded by the '
                             'client instead of being held')
    parser.add_argument('--max-vel', type=float, default=2.0,
                        help='joint velocity limit of the fake plant, rad/s (default: %(default)s)')
    parser.add_argument('--stiffness', type=float, default=30.0,
                        help='reported torque per rad of tracking error (default: %(default)s)')
    parser.add_argument('--torque-limit', type=float, default=30.0,
                        help='reported torque clamp, N*m (default: %(default)s)')
    parser.add_argument('--initial', type=float, nargs='*', default=[],
                        help='initial joint angles in radians (default: all zero)')
    parser.add_argument('--disconnect-ms', type=float, default=100.0,
                        help='force PASSIVE after this long without a SendCmd, mirroring '
                             'ARMSDK::_sendRecv (default: %(default)s)')
    parser.add_argument('--drop-rate', type=float, default=0.0,
                        help='fraction of RecvState datagrams to withhold, to exercise the '
                             'disconnect detection (default: %(default)s)')
    parser.add_argument('--reject-states', nargs='*', default=[], metavar='STATE',
                        choices=sorted(FSM_STATES),
                        help='FSM states to refuse, to simulate an illegal transition')
    parser.add_argument('--motor-fault', type=int, default=-1, metavar='INDEX',
                        help='raise motor error bit 0x01 on this motor index')
    parser.add_argument('--verbose', action='store_true', help='log every FSM change')
    parser.add_argument('--verbose-period', type=float, default=0.5,
                        help='seconds between state dumps with --verbose (default: %(default)s)')
    args = parser.parse_args(argv)

    if args.rate <= 0:
        parser.error('--rate must be positive')
    if not 0.0 <= args.drop_rate < 1.0:
        parser.error('--drop-rate must be in [0, 1)')
    if not 0 <= args.motor_fault < MOTOR_COUNT and args.motor_fault != -1:
        parser.error('--motor-fault must be in [0, {}]'.format(MOTOR_COUNT - 1))
    args.reject_states = [FSM_STATES[name] for name in args.reject_states]
    return args


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        return MockZ1(args).run()
    except OSError as error:
        print('cannot bind {}:{} - is z1_ctrl (or another mock) already running? ({})'.format(
            args.bind_ip, args.bind_port, error), file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
