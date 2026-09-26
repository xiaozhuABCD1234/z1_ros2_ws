# z1_moveit_config

Unitree Z1 机械臂的 MoveIt 2 配置包。

## 目录结构

```
config/z1.srdf                 # 带夹爪：规划组（arm、gripper、arm_with_gripper）、
                               # 命名状态（官方全套：zero、home、forward、stow、
                               # pounce、show_left/mid/right、open、closed）、碰撞矩阵
config/z1_no_gripper.srdf      # 无末端执行器：只有 arm 组 + 8 个臂状态 + 臂内碰撞矩阵
                               # （取值与出处见“命名状态：全部来自官方”）
config/kinematics.yaml         # KDL（本机安装的唯一 IK 插件）
config/joint_limits.yaml       # 来自 URDF 的速度限位 + 加速度限位
config/moveit_controllers.yaml # JTC（机械臂）+ GripperActionController（夹爪）
config/moveit_controllers_no_gripper.yaml  # 同上，但只有 JTC
rviz/moveit.rviz               # MotionPlanning 显示与面板（规划组 arm）
launch/demo.launch.py          # ros2_control（Gazebo 或真机）+ move_group + RViz2
launch/move_group.launch.py    # move_group only
z1_moveit_config/moveit_config.py   # 共用的 MoveItConfigsBuilder 配置
```

## 运行

```bash
# Gazebo Sim 物理（gz_ros2_control）+ MoveIt + RViz（move_group/RViz 走 /clock），默认
ros2 launch z1_moveit_config demo.launch.py

# 不带 RViz（无界面跑规划/执行）
ros2 launch z1_moveit_config demo.launch.py use_rviz:=false

# 真机（z1_ros2_control/Z1System）+ RViz；需要先在 ~/Projects/z1_controller/build
# 里起 ./z1_ctrl，且启动时机械臂会先失力到 PASSIVE
ros2 launch z1_moveit_config demo.launch.py use_gazebo:=false use_gripper:=false

# 无末端执行器（本机情况）：URDF、SRDF、控制器映射一起切成 6 轴
ros2 launch z1_moveit_config demo.launch.py use_gripper:=false use_gazebo:=true

# 在已运行的机器人之上只起 MoveIt（Gazebo 或真机）
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
* `use_gripper` 必须和 URDF 一致，`moveit_config.py` 负责切换：`false` 时换成 arm-only 的
  `config/z1_no_gripper.srdf` + `config/moveit_controllers_no_gripper.yaml`，并从关节限位
  里去掉 `jointGripper`。本配置描述的是**带夹爪的机械臂**，不用夹爪就用参数切，
  不要手改 `z1.srdf`。
* RViz 用的 `rviz/moveit.rviz` 里默认规划组就是 `arm`，交互式标记直接可用。
* 起始状态相关的两条硬约束（详见根 README 的踩坑结论）：`allowed_start_tolerance` 在
  控制器映射文件里设为 **0.05**（官方力矩律 i = 0，到点后有 0.001–0.021 rad 重力静差，
  默认 0.01 会报 `start point deviates from current robot state`）；零位正好落在 joint2
  下限与 joint3 上限上，所以 `z1_controllers` 激活后先停到 SRDF 的 `forward` 位姿。
* SRDF 中禁用了 `link02`/`link06` 的自碰撞（官方圆柱碰撞基元在零位互穿 8 mm），
  参见 `config/z1.srdf` 注释。
* 官方 URDF 中 `link01` 和 `link05` 没有碰撞几何；MoveIt 会给出警告，
  并按空几何处理。
* IK 使用 KDL。若要用 `trac_ik`/`pick_ik`，需要
  `sudo apt install ros-jazzy-trac-ik-kinematics-plugin`，
  并修改 `config/kinematics.yaml` 中的一行。
* 未配置任何 3D 传感器，因此 move_group 会打印
  `No 3D sensor plugin(s) defined for octomap updates`——这是预期行为。

## 命名状态：全部来自官方

SRDF 里的 8 个臂状态就是 Unitree 官方的位姿，没有本地自造的名字（`open`/`closed` 是夹爪的）。
“限位余量”一列是用 URDF 限位算出来的（`0` 表示该关节正好压在限位上）。

| 状态 | j1 | j2 | j3 | j4 | j5 | j6 | 官方出处 | 限位余量 |
|---|---|---|---|---|---|---|---|---|
| `zero` | 0 | 0 | 0 | 0 | 0 | 0 | 用户手册 §3.1 零位（J1/J6 缝隙刻线对齐） | joint2/joint3 正好在界上 |
| `home` | 0 | 0 | -0.005 | -0.074 | 0 | 0 | `z1_ros`（官方 ROS1 MoveIt）SRDF 的 `home` = `savedArmStates.csv` 的 `startFlat`（`backToStart()` 的目标） | joint2 正好在界上，joint3 余 0.005 |
| `forward` | 0 | 1.5 | -1.0 | -0.54 | 0 | 0 | `savedArmStates.csv` 的 `forward`；`z1_sdk` 的 `q_FORWARD` / `targetPos` | **0.98**（joint4） |
| `pounce` | 0 | 1.57 | -0.707 | -0.9 | 0 | 0 | `z1_ros` SRDF 的 `pounce` | 0.62（joint4） |
| `show_left` | -1.0 | 0.9 | -1.0 | -0.3 | 0 | 0 | `savedArmStates.csv` | 0.90（joint2） |
| `show_mid` | 0 | 0.9 | -1.2 | 0.25 | 0 | 0 | `savedArmStates.csv` | 0.90（joint2） |
| `show_right` | 1.0 | 0.9 | -1.0 | -0.3 | 0 | 0 | `savedArmStates.csv` | 0.90（joint2） |
| `stow` | 0 | 1.1 | -1.1 | 1.5707 | 0 | 0 | `z1_ros` SRDF 的 `stow`（同时是官方 `gazebo.launch` 的生成位姿、`fake_controllers.yaml` 的初始位姿） | ❌ joint4 超限 0.052 rad |

`z1_controllers` 的停靠位姿用的是 `forward`——官方位姿里唯一完整落在限位内、又远离 joint2/joint3
边界的那个。

实测（`demo.launch.py use_rviz:=false`，`plan_only`；该记录取自已移除的 mock 硬件后端，
起点是否越限由 URDF 限位决定，与后端无关）：

* `forward` / `zero` 作起始状态 → `SUCCEEDED`；`stow` → `ABORTED`：
  `Joint 'joint4' from the starting state is outside bounds by: [1.5707 ] should be in the range [-1.51844 ], [1.51844 ].`
* `check_state_validity` 里 `forward`/`home`/`stow` 都是 `valid=True, contacts=[]`（无自碰撞）——
  但这个服务**只查碰撞、不查限位**（`stow` 在它那里也是 True），别拿它当起始状态检查用。
* `home` / `pounce` / `show_*` 的限位数值已核对，MoveIt 实测还没跑。

结论：**官方从来不把 joint5 取非零值**（joint5 限位 ±77°，0 在区间内部，而 q5 = ±90° 才是
腕部奇异、已在限位之外），所以本仓库里没有任何 joint5 ≠ 0 的命名状态。
