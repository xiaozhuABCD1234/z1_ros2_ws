# z1_moveit_config

Unitree Z1 机械臂的 MoveIt 2 配置包。

## 目录结构

```
config/z1.srdf                 # 带夹爪：规划组（arm、gripper、arm_with_gripper）、
                               # 命名状态（zero、ready、open、closed）、碰撞矩阵
config/z1_no_gripper.srdf      # 无末端执行器：只有 arm 组 + zero/ready + 臂内碰撞矩阵
config/kinematics.yaml         # KDL（本机安装的唯一 IK 插件）
config/joint_limits.yaml       # 来自 URDF 的速度限位 + 加速度限位
config/moveit_controllers.yaml # JTC（机械臂）+ GripperActionController（夹爪）
config/moveit_controllers_no_gripper.yaml  # 同上，但只有 JTC
rviz/moveit.rviz               # MotionPlanning 显示与面板（规划组 arm）
launch/demo.launch.py          # ros2_control（虚拟硬件或 Gazebo）+ move_group + RViz2
launch/move_group.launch.py    # move_group only
z1_moveit_config/moveit_config.py   # 共用的 MoveItConfigsBuilder 配置
```

## 运行

```bash
# 虚拟硬件（理想伺服）+ RViz，拖动交互式标记即可 Plan/Execute
ros2 launch z1_moveit_config demo.launch.py

# 不带 RViz（无界面跑规划/执行）
ros2 launch z1_moveit_config demo.launch.py use_rviz:=false

# Gazebo Sim 物理（gz_ros2_control）+ MoveIt + RViz（move_group/RViz 走 /clock）
ros2 launch z1_moveit_config demo.launch.py use_gazebo:=true

# 无末端执行器（本机情况）：URDF、SRDF、控制器映射一起切成 6 轴
ros2 launch z1_moveit_config demo.launch.py use_gripper:=false use_gazebo:=true

# 在已运行的机器人之上只起 MoveIt（虚拟硬件或真机）
ros2 launch z1_bringup control.launch.py use_gripper:=false &
ros2 launch z1_moveit_config move_group.launch.py use_gripper:=false

# 在 Gazebo + ros2_control 之上只起 MoveIt
ros2 launch z1_bringup gazebo_ros2_control.launch.py use_gripper:=false use_rviz:=false &
ros2 launch z1_moveit_config move_group.launch.py use_gripper:=false use_sim_time:=true
```

无界面（headless）规划与执行验证（无需 RViz）：

```bash
ros2 action send_goal /move_action moveit_msgs/action/MoveGroup \
"{request: {group_name: arm, num_planning_attempts: 10, allowed_planning_time: 5.0,
  max_velocity_scaling_factor: 0.3, max_acceleration_scaling_factor: 0.3,
  goal_constraints: [{name: target, joint_constraints: [
    {joint_name: joint1, position: 0.4, tolerance_above: 0.01, tolerance_below: 0.01, weight: 1.0},
    {joint_name: joint2, position: 1.1, tolerance_above: 0.01, tolerance_below: 0.01, weight: 1.0},
    {joint_name: joint3, position: -1.0, tolerance_above: 0.01, tolerance_below: 0.01, weight: 1.0},
    {joint_name: joint4, position: 0.7, tolerance_above: 0.01, tolerance_below: 0.01, weight: 1.0},
    {joint_name: joint5, position: 0.5, tolerance_above: 0.01, tolerance_below: 0.01, weight: 1.0},
    {joint_name: joint6, position: 0.3, tolerance_above: 0.01, tolerance_below: 0.01, weight: 1.0}]}]},
 planning_options: {plan_only: false}}"
```

## 说明

* `demo.launch.py` 的参数：`use_gripper`、`use_gazebo`、`use_rviz`、`headless`；
  `move_group.launch.py` 的：`use_gripper`、`use_sim_time`。
* `use_gripper` 必须和 URDF 一致：`false` 时换成 arm-only 的
  `config/z1_no_gripper.srdf` + `config/moveit_controllers_no_gripper.yaml`，
  并从关节限位里去掉 `jointGripper`；`moveit_config.py` 负责这个切换。
* RViz 用的 `rviz/moveit.rviz` 里默认规划组就是 `arm`，交互式标记直接可用。
* 本配置描述的是**带夹爪的机械臂**。如果不用夹爪，用 `use_gripper:=false`，
  不要手改 `z1.srdf`。
* `trajectory_execution.allowed_start_tolerance` 在控制器映射文件里设为 **0.05**：
  Gazebo 路径下机械臂用官方力矩律（i = 0），到点后会有约 0.001–0.021 rad 的重力静差，
  默认的 0.01 会把它当成 `start point deviates from current robot state` 而拒绝执行。
* MoveIt 只接受**严格在限位内**的起始状态（`CheckStartStateBounds`，越界 1e-14 rad 也拒）。
  零位正好落在 joint2 下限与 joint3 上限上，所以 `z1_controllers` 激活后会先停到 SRDF 的
  `ready` 位姿（`initial_position`），从那里开始规划。
* SRDF 中禁用了 `link02`/`link06` 的自碰撞：使用官方的圆柱碰撞基元时，腕部在
  零位/停放位姿会与肩部电机互穿 8 mm，这会导致 MoveIt 拒绝从该位姿开始规划。
  参见 `config/z1.srdf` 中的注释（arm-only 版本里有同样的说明）。
* 官方 URDF 中 `link01` 和 `link05` 没有碰撞几何；MoveIt 会给出警告，
  并按空几何处理。
* IK 使用 KDL。若要用 `trac_ik`/`pick_ik`，需要
  `sudo apt install ros-jazzy-trac-ik-kinematics-plugin`，
  并修改 `config/kinematics.yaml` 中的一行。
* 未配置任何 3D 传感器，因此 move_group 会打印
  `No 3D sensor plugin(s) defined for octomap updates`——这是预期行为。
