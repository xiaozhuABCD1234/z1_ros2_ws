# z1_controllers

ros2_control controller for the Unitree Z1 arm that applies the **official torque
law** and exposes the same `FollowJointTrajectory` action interface as
`joint_trajectory_controller`.

```
tau_i = Kp_i * (q_des_i - q_i) + Kd_i * (qdot_des_i - qdot_i)
tau_i = clamp(tau_i, -effort_limit_i, +effort_limit_i)
```

这对应官方 `unitree_legged_control` 的控制律
（`src/unitree_joint_control_tool.cpp`：

```c
calcTorque = posStiffness*(targetPos-currentPos)
           + velStiffness*(targetVel-currentVel) + targetTorque;
```

再按 URDF 的 effort 限幅）。官方那组增益写在
`robots/z1_description/config/robot_control.yaml`（`pid: {p: 300, i: 0, d: 5}`），
注意它在官方 C++ 里其实被 `#ifdef rqtTune` 编译掉了，真正的 Kp/Kd 由上层通过
`MotorCmd` 传入 —— 但律和数值就是这套。

## 为什么需要它

Gazebo 后端上，`gz_ros2_control` 的位置接口只有 `v = P·e`（单一 P 增益，没有 D），
配上 URDF 里 1.0 N·m 的关节摩擦就是个极限环：关节在指令值附近持续晃动
（在停靠位姿上实测 std 0.006–0.015 rad、峰峰最大 0.076 rad），把 P 从 50 提到 300
毫无改善。换成本控制器（`effort` 接口 + PD）后实测 std ≤ 0.006、峰峰 ≤ 0.013 rad，
MoveGroup 的 plan+execute 正常。

代价是官方律里 i = 0，所以有重力静差（实测 0.001–0.021 rad，Kp=300 时约
`τ_grav / Kp`）——这是官方行为的原样复现，不是 bug。

## 用法

`z1_description` 在 Gazebo 后端会把机械臂的 `command_interface` 从 `position` 切成
`effort`（`z1_ros2_control.xacro`），所以控制器必须走 effort 接口：

```yaml
# z1_bringup/config/z1_controllers_gz_effort.yaml
controller_manager:
  ros__parameters:
    joint_trajectory_controller:
      type: z1_controllers/Z1JointTrajectoryController

joint_trajectory_controller:
  ros__parameters:
    joints: [joint1, joint2, joint3, joint4, joint5, joint6]
    gains:
      joint1: {p: 300.0, d: 5.0}
      ...
    effort_limits:      # = URDF effort 限幅
      joint1: 30.0
      joint2: 60.0
      ...
    constraints:
      goal_time: 0.5
      joint1: {trajectory: 0.25, goal: 0.05}
      ...
    # 激活后限速滑到这个位姿再停住（见下），值 = 官方 forward 位姿
    initial_position: [0.0, 1.5, -1.0, -0.54, 0.0, 0.0]
    initial_position_speed: 0.8
```

`z1_bringup/launch/gazebo_ros2_control.launch.py` 默认就用这个文件；MoveIt 侧不用改，
因为控制器名（`joint_trajectory_controller`）和动作接口都没变。

## 参数

| 参数 | 含义 |
|---|---|
| `joints` | 受控关节，顺序即命令顺序 |
| `gains.<joint>.{p,d}` | PD 增益（默认 300 / 5） |
| `effort_limits.<joint>` | 力矩限幅（代码默认 30；`z1_controllers_gz_effort.yaml` 里 joint2 = 60，与 URDF 一致） |
| `constraints.<joint>.{trajectory,goal}` | 路径/目标容差（默认 0.2 / 0.05 rad） |
| `constraints.goal_time` | 到点后允许的额外时间（默认 0 s） |
| `position_limits.<joint>.{lower,upper}` | 关节限位（默认 = 官方 URDF 限位），位置参考会被 clamp 到其中 |
| `boundary_margin` | 参考位置与限位的安全边距（默认 0.02 rad，见下） |
| `initial_position`, `initial_position_speed` | 激活后的停靠位姿与限速（后者默认 0.5 rad/s，见下） |
| `action_monitor_rate` | 动作结果上报频率（默认 20 Hz） |

## 关于 `initial_position`

零位/停放位姿**正好**在 joint2 下限（0）和 joint3 上限（0）上：PD 的重力静差会把实测值
推到边界外大约 `1e-14` rad，而 MoveIt 2.12 的 `CheckStartStateBounds` 对越界零容忍
（`Start state out of bounds` → `START_STATE_INVALID`，planning 直接失败）。
所以控制器激活后会把“保持点”按 `initial_position_speed` 限速滑到 SRDF 的 `forward`
位姿（官方值：`[0, 1.5, -1, -0.54, 0, 0]`，来自 `z1_controller/config/`
`savedArmStates.csv` 与 `z1_sdk` 的 `q_FORWARD`），该位姿最小余量 0.98 rad（joint4），
joint5 = joint6 = 0。留空则仅保持上电瞬间的位置。

## 实现要点

* 命令接口：`effort`；状态接口：`position` + `velocity`。
* 轨迹插值：逐段三次 Hermite（发送方没给速度的航点按零速度处理，等价于 smoothstep）；
  单航点轨迹会在前面补一个“当前位置”航点，避免直接阶跃。
* 容差：运行中越界报 `PATH_TOLERANCE_VIOLATED`；跑完后 `goal_time` 内没进入目标容差报
  `GOAL_TOLERANCE_VIOLATED`；取消后回到保持点并报 `canceled`。
* 实时循环内只做插值 + PD + 写命令；动作结果由非实时的 wall timer 上报。
