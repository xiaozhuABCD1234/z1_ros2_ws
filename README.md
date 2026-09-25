# z1_ros2_ws — Unitree Z1 的 ROS 2（Jazzy）工作空间

Unitree Z1 机械臂的 ROS 2 移植：URDF、Gazebo Sim、ros2_control 和 MoveIt 2 均已跑通。
剩余部分是硬件接口，用于通过官方的 `z1_controller` 与真机通信。

## 功能包

| 功能包 | 状态 | 内容 |
|---|---|---|
| [`z1_description`](src/z1_description) | ✅ | URDF/xacro、网格模型、RViz 配置、`display.launch.py` |
| [`z1_bringup`](src/z1_bringup) | ✅ | ros2_control / Gazebo 启动、控制器配置 |
| [`z1_moveit_config`](src/z1_moveit_config) | ✅ | SRDF、运动学、关节限位、MoveIt 2 演示 |
| [`z1_controllers`](src/z1_controllers) | ✅ | 官方力矩律控制器 `Z1JointTrajectoryController`（τ = Kp·e + Kd·ė，限幅），给 Gazebo 的 effort 接口用 |
| `z1_ros2_control` | ⏳ 下一步 | 对接 `z1_controller` 的 `hardware_interface::SystemInterface` |

参数、接口、设计原因和已知限制都写在**各包自己的 README** 里，本文件只做入口。

## 编译

```bash
cd ~/Projects/z1_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

依赖均已安装；可选 `ros-jazzy-trac-ik-kinematics-plugin`（比 KDL 更好的 IK）。

## 运行

| 用途 | 命令 |
|---|---|
| 仅 URDF（RViz + 关节滑块） | `ros2 launch z1_description display.launch.py` |
| ros2_control，虚拟硬件 | `ros2 launch z1_bringup mock_hardware.launch.py` |
| ros2_control，虚拟硬件，不带 RViz | `ros2 launch z1_bringup control.launch.py` |
| Gazebo Sim（gz 位置伺服） | `ros2 launch z1_bringup gazebo.launch.py` |
| Gazebo Sim + ros2_control | `ros2 launch z1_bringup gazebo_ros2_control.launch.py` |
| MoveIt 2 演示（虚拟硬件 + RViz） | `ros2 launch z1_moveit_config demo.launch.py` |
| MoveIt 2 + RViz 控制 Gazebo 仿真 | `ros2 launch z1_moveit_config demo.launch.py use_gazebo:=true` |

常用参数：`use_gripper:=true|false`、`use_rviz:=true|false`、`headless:=true`（Gazebo）、
`world:=<file>`、`gz_verbosity:=0..4`。

Gazebo + ros2_control 默认按**官方力矩律**驱动机械臂（[`z1_controllers`](src/z1_controllers)
走 effort 接口，增益 300/5，限幅 = URDF effort）；mock/真机路径仍是位置接口 +
`joint_trajectory_controller`。要回到位置伺服：
`controllers_file:=$(ros2 pkg prefix z1_bringup)/share/z1_bringup/config/z1_controllers.yaml`。

**本机机械臂没有末端执行器时**，所有涉及模型的命令都加 `use_gripper:=false`——它会把
URDF、SRDF（[`z1_no_gripper.srdf`](src/z1_moveit_config/config/z1_no_gripper.srdf)）
和控制器映射一起切到 6 轴无夹爪版本（两种配置的默认值都是 `true`）：

```bash
ros2 launch z1_moveit_config demo.launch.py use_gripper:=false use_gazebo:=true   # 仿真 + RViz 控制
ros2 launch z1_bringup gazebo_ros2_control.launch.py use_gripper:=false          # 只起仿真
ros2 launch z1_description display.launch.py use_gripper:=false                  # 只看 URDF
```

快速验证：

```bash
# 通过 joint_trajectory_controller 下发轨迹（mock / gz_ros2_control 两条路径均可）
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint1, joint2, joint3, joint4, joint5, joint6],
    points: [{positions: [0.5, 0.5, -0.4, 0.3, 0.2, 0.1],
              time_from_start: {sec: 2, nanosec: 0}}]}}" --feedback

# 夹爪动作；0 = 闭合，负值 = 张开
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand "{command: {position: -0.8, max_effort: 10.0}}"

# 单关节，仅 Gazebo gz-servo 路径可用
ros2 topic pub --once /joint1/cmd_pos std_msgs/msg/Float64 "{data: 0.6}"
```

## 效果

| Gazebo Sim（`gazebo.launch.py`） | MoveIt 2（`z1_moveit_config demo.launch.py`） |
|---|---|
| ![Gazebo Sim](docs/images/gazebo_sim.png) | ![MoveIt 2](docs/images/moveit_rviz.png) |

两者都通过不同的软件栈被驱动到同一类关节目标：一条是经 `/jointN/cmd_pos` 的 gz 位置伺服，
另一条是 OMPL → `joint_trajectory_controller` → ros2_control。

## 设计要点

* **一份 URDF，三种 ros2_control 后端**：`hardware_plugin:=` 选
  `mock_components/GenericSystem`（默认）、`z1_ros2_control/Z1System`（真机，下一步）或
  `gz_ros2_control/GazeboSimSystem`；控制器集合与 MoveIt 配置完全一致。
* **Gazebo 后端用 `effort` 接口，其余用 `position`**（`z1_ros2_control.xacro` 自动切）。
  gz_ros2_control 的位置接口只有单一 P 增益、没有 D，会跟关节摩擦形成持续抖动；
  改用 effort 后由 [`z1_controllers`](src/z1_controllers) 施加官方力矩律。
* **`use_gripper` 贯穿三层**：URDF、MoveIt（SRDF + 控制器映射）、`gripper_controller`
  spawner，三处必须同值，否则 MoveIt 会引用模型里不存在的 link。
* **[`control.launch.py`](src/z1_bringup/launch/control.launch.py) 是共用核心**，
  mock、Gazebo、MoveIt 的启动文件都组合它；两条 Gazebo 路径（gz 伺服 + bridge、
  gz_ros2_control）相互独立，不能同时运行。
* **真机上关节级伺服律留在 `z1_controller` 内部**，ros2_control 只下发位置指令。

## 踩坑记录

完整叙述（含实测数据）见各包 README 与代码注释，这里只列结论：

1. `gripperStator` 的 joint 与 link 同名会让 `gz sim` 拒绝加载模型 → 固定关节改名
   `gripperStator_joint`。
2. Blender 导出的 DAE 材质在 gz/assimp 路径上丢失，link 渲染成过曝白色 → 每个 visual
   显式写 `<material>`。
3. 零位/停放位姿与官方圆柱碰撞体互穿 8 mm → SRDF 中禁用 `link02`–`link06` 碰撞对。
4. `gz_ros2_control` 的 `position_proportional_gain` 写在 URDF `<plugin>` 里传不到插件，
   必须写进参数文件。
5. `IncludeLaunchDescription` 会把 `launch_arguments` 泄漏到父作用域，父启动文件不能复用
   同名但不同值的参数（见 `demo.launch.py` 注释）。
6. MoveIt 的起始状态检查在 Gazebo 路径上很苛刻：`allowed_start_tolerance` 默认 0.01 rad
   会拒掉每次执行（设为 0.05），且零位正好落在 joint2 下限 / joint3 上限上、重力静差会
   越界 1e-14 rad（控制器激活后先限速滑到 SRDF 的 `ready` 位姿）。
7. 官方的律是力矩 PD（`unitree_legged_control`），不是 gz 位置伺服那条 `v = P·e`；
   两条路径分别用 `z1_gazebo.xacro` 和 `z1_controllers` 复现后仿真抖动归零。

## 来源 / 许可证

网格模型、惯量和关节限位来自
[`unitreerobotics/unitree_ros`](https://github.com/unitreerobotics/unitree_ros)
（`robots/z1_description`，**BSD-3-Clause**，© Unitree Robotics），
详见 `src/z1_description/LICENSE`。与 ROS 1 原版的差异列在
`src/z1_description/README.md` 中。
