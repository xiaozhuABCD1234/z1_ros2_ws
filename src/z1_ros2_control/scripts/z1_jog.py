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
"""Move exactly one joint of the Z1 a small, slow amount, on the real arm.

Why this exists
---------------
A `FollowJointTrajectory` point lists *absolute* positions for every joint it
names. Sending one point for `joint1` alone, or sending a point whose other
entries are 0, does not "move joint1 and leave the rest alone" - it commands the
other joints to whatever those entries say. That is an easy way to move five
joints by accident while believing you are moving one.

This script therefore always sends a full six-joint point: the selected joint
gets its current position plus the requested change, and every other joint
explicitly gets its current position.

It is meant for bring-up and for checking a signal chain, not for real work;
use MoveIt for that. The guards are deliberately conservative:

  * the change is refused if it exceeds --max-delta (default 0.2 rad),
  * the speed is refused if it exceeds --max-velocity (default 0.2 rad/s),
  * the plan is printed before anything is sent, and --dry-run stops there,
  * --force relaxes both guards, and says so.

Before anything moves, the arm's current pose has to be known, so this does not
send a goal until /joint_states has produced a complete sample.
"""

import argparse
import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINTS = ["joint{}".format(index) for index in range(1, 7)]


def parse_joint(text):
    """Accept '3', 'joint3' or 'jointGripper'."""
    if text in ARM_JOINTS or text == "jointGripper":
        return text
    try:
        index = int(text)
    except ValueError:
        return None
    if 1 <= index <= 6:
        return "joint{}".format(index)
    return None


class Jogger(Node):
    def __init__(self, args):
        super().__init__("z1_jog")
        self.args = args
        self.positions = None
        self._subscription = self.create_subscription(
            JointState, args.joint_states_topic, self._on_joint_state, 10
        )
        self._client = ActionClient(self, FollowJointTrajectory, args.action_name)

    def _on_joint_state(self, message):
        by_name = dict(zip(message.name, message.position))
        if all(joint in by_name for joint in ARM_JOINTS):
            self.positions = [float(by_name[joint]) for joint in ARM_JOINTS]

    def wait_for_positions(self):
        deadline = time.monotonic() + self.args.goal_timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.positions is not None:
                return self.positions
        return None


def build_goal(joints, positions, target_index, target):
    """One point, all joints explicit: the point is absolute, not a delta."""
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(joints)
    point = JointTrajectoryPoint()
    point.positions = list(positions)
    point.positions[target_index] = target
    goal.trajectory.points = [point]
    return goal, point


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("joint", help="which joint to move: 1..6")
    change = parser.add_mutually_exclusive_group(required=True)
    change.add_argument("--delta", type=float, help="change in rad, signed")
    change.add_argument("--position", type=float, help="absolute target angle in rad")
    parser.add_argument(
        "--velocity", type=float, default=0.05,
        help="rad/s for the move; the duration is derived from it (default: %(default)s)",
    )
    parser.add_argument(
        "--max-delta", type=float, default=0.2,
        help="refuse a larger change unless --force (default: %(default)s rad)",
    )
    parser.add_argument(
        "--max-velocity", type=float, default=0.2,
        help="refuse a faster move unless --force (default: %(default)s rad/s)",
    )
    parser.add_argument("--force", action="store_true", help="relax both guards above")
    parser.add_argument("--dry-run", action="store_true", help="print the plan, send nothing")
    parser.add_argument(
        "--action-name", default="/joint_trajectory_controller/follow_joint_trajectory"
    )
    parser.add_argument("--joint-states-topic", default="/joint_states")
    parser.add_argument(
        "--goal-timeout", type=float, default=10.0,
        help="seconds to wait for /joint_states and for the action server (default: %(default)s)",
    )
    parser.add_argument(
        "--result-timeout", type=float, default=60.0,
        help="seconds to wait for the trajectory result (default: %(default)s)",
    )
    args = parser.parse_args(argv)

    joint = parse_joint(args.joint)
    if joint is None:
        parser.error("joint must be 1..6 (got '{}')".format(args.joint))
    if joint == "jointGripper":
        parser.error("the gripper is not an arm joint; use gripper_controller instead")
    if joint not in ARM_JOINTS:
        parser.error("'{}' is not one of {}".format(joint, ", ".join(ARM_JOINTS)))

    rclpy.init()
    node = Jogger(args)
    try:
        positions = node.wait_for_positions()
        if positions is None:
            print(
                "no complete sample on {} within {:.0f} s. Is the stack up and the hardware "
                "active? (ros2 launch z1_bringup control.launch.py)".format(
                    args.joint_states_topic, args.goal_timeout
                ),
                file=sys.stderr,
            )
            return 1

        index = ARM_JOINTS.index(joint)
        before = positions[index]
        target = before + args.delta if args.delta is not None else args.position
        change = target - before

        if abs(change) < 1e-4:
            print("{} is already at {:+.4f} rad; nothing to do".format(joint, before))
            return 0

        relaxed = []
        if abs(change) > args.max_delta and not args.force:
            print(
                "refusing to move {} by {:.4f} rad: more than --max-delta {:.4f}. "
                "Pass --force if that is really what you want.".format(
                    joint, change, args.max_delta
                ),
                file=sys.stderr,
            )
            return 1
        if abs(change) > args.max_delta:
            relaxed.append("delta")
        if args.velocity > args.max_velocity and not args.force:
            print(
                "refusing --velocity {:.3f} rad/s: more than --max-velocity {:.3f}. "
                "Pass --force if that is really what you want.".format(
                    args.velocity, args.max_velocity
                ),
                file=sys.stderr,
            )
            return 1
        if args.velocity > args.max_velocity:
            relaxed.append("velocity")

        duration = max(1.0, abs(change) / args.velocity)
        goal, point = build_goal(ARM_JOINTS, positions, index, target)
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)

        print("plan:")
        print("  joint      {} (index {})".format(joint, index))
        print("  now        {:+.4f} rad".format(before))
        print("  target     {:+.4f} rad ({:+.4f} rad, i.e. {:+.2f} deg)".format(
            target, change, change * 57.29577951308232))
        print("  duration   {:.2f} s -> {:.4f} rad/s".format(duration, abs(change) / duration))
        print("  others     held at their current angle (the trajectory point is absolute)")
        if relaxed:
            print("  --force    ignored the {} guard(s)".format(" and ".join(relaxed)))
        if args.dry_run:
            print("\n--dry-run: nothing sent.")
            return 0

        if not node._client.wait_for_server(timeout_sec=args.goal_timeout):
            print(
                "no action server on {} within {:.0f} s".format(
                    args.action_name, args.goal_timeout
                ),
                file=sys.stderr,
            )
            return 1

        print("\nsending...")
        send_future = node._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=args.goal_timeout)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            print("goal was rejected", file=sys.stderr)
            return 1

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=args.result_timeout)
        result = result_future.result()
        if result is None:
            print("no result within {:.0f} s".format(args.result_timeout), file=sys.stderr)
            return 1

        outcome = result.result
        print("result: error_code={} '{}'".format(outcome.error_code, outcome.error_string))

        # Give the arm a moment, then report where it actually ended up.
        settle = time.monotonic() + 1.0
        while time.monotonic() < settle:
            rclpy.spin_once(node, timeout_sec=0.1)
        after = node.positions[index] if node.positions else float("nan")
        print("{}: {:+.4f} -> {:+.4f} rad (target {:+.4f}, off by {:+.4f})".format(
            joint, before, after, target, after - target))
        return 0 if outcome.error_code == 0 else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
