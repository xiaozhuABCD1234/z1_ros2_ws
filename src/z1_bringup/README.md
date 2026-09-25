# z1_bringup

Z1 机械臂的启动文件与控制器配置。

## 启动文件

| 启动文件 | 启动内容 |
|---|---|
| `control.launch.py` | **共用核心**：`robot_state_publisher`、`ros2_control_node`、`joint_state_broadcaster`、`joint_trajectory_controller`、`gripper_controller`。不含可视化。 |
| `mock_hardware.launch.py` | `control.launch.py` + RViz2（虚拟硬件，即理想伺服） |
| `gazebo.launch.py` | gz-sim 8 + `ros_gz_sim create` + `ros_gz_bridge`，关节由 gz 的 `JointPositionController` 伺服从动（不使用 ros2_control） |
| `gazebo_ros2_control.launch.py` | gz-sim 8 + `gz_ros2_control`（controller_manager 跑在 Gazebo server 内）+ `/clock` 桥接；默认用**官方力矩律**驱动机械臂（`z1_controllers/Z1JointTrajectoryController` 走 effort 接口） |
| `../z1_description/launch/display.launch.py` | 仅 URDF：`robot_state_publisher` + `joint_state_publisher_gui` + RViz2 |

参数：`use_gripper`、`use_rviz`，以及各启动文件专属的 `rviz_config`（mock）、
`world` / `headless` / `gz_verbosity`（Gazebo）、`hardware_plugin`（control）、
`controllers_file`（gazebo_ros2_control，默认官方力矩律那一份）。

共用部分（启动参数、`gz sim` 启动、`robot_state_publisher`、模型 spawn、RViz，
以及**各路径要 spawn 的控制器列表**）集中在 `z1_bringup/launch_parts.py`
（安装为 `z1_bringup.launch_parts`），启动文件不各自复制，以免两边漂移。

## 硬件后端

`control.launch.py hardware_plugin:=...` 用于选择 ros2_control 后端
（即 `z1_description` 中的 `<ros2_control>`）：

| 插件 | 含义 |
|---|---|
| `mock_components/GenericSystem` | 理想伺服，无硬件（默认） |
| `z1_ros2_control/Z1System` | 经 `z1_controller` 连接真机（待实现） |
| `gz_ros2_control/GazeboSimSystem` | Gazebo Sim 物理，供 `gazebo_ros2_control.launch.py` 使用 |

三种情况下控制器、接口和 MoveIt 配置完全相同。

## 控制器配置

两份文件，控制器名一样，只是机械臂那个的**类型**不同：

* `config/z1_controllers.yaml` —— 位置接口 + `joint_trajectory_controller/JointTrajectoryController`，
  用于 mock 硬件与真机（真机的关节伺服律在 `z1_controller` 里）。
* `config/z1_controllers_gz_effort.yaml` —— effort 接口 + `z1_controllers/Z1JointTrajectoryController`
  （官方力矩律 `τ = Kp·e + Kd·ė`，增益 300/5，限幅 = URDF effort），
  `gazebo_ros2_control.launch.py` 的默认值。两者都继承同一个 `joint_state_broadcaster`
  与 `gripper_controller`。

共有部分：

* `controller_manager.update_rate = 250 Hz`，即官方 `z1_controller` 的循环频率
  （`CtrlComponents::dt = 1/250 s`）。
* `joint_trajectory_controller`：逐关节的轨迹/目标容差。
* `gripper_controller`（`position_controllers/GripperActionController`）作用于
  `jointGripper`，仅在 `use_gripper:=true` 时启动（夹爪在所有后端都保持 position 接口）。
* `gz_ros_control:` 段 —— 仅当关节还有 position/velocity 接口时才会用到（即
  `controllers_file` 指向旧的位置配置时）；力矩律那份不需要它。

## 接口

**ros2_control 路径**（`control.launch.py`、`mock_hardware`、`gazebo_ros2_control`）

| 接口 | 类型 |
|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` |
| `/joint_trajectory_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` |
| `/gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` |

**gz-servo 路径**（`gazebo.launch.py`，不使用 ros2_control）

| 接口 | 类型 |
|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState`（来自 `gz.msgs.Model`） |
| `/joint<N>/cmd_pos`、`/jointGripper/cmd_pos` | `std_msgs/msg/Float64`（桥接到 `gz.msgs.Double`） |
| `/clock` | `rosgraph_msgs/msg/Clock` |

gz 伺服的增益位于 `z1_description/urdf/z1_gazebo.xacro`。

## 路线图

1. ✅ 虚拟硬件、Gazebo（伺服从动与 ros2_control 两条路径）、MoveIt 2。
2. ⏳ `z1_ros2_control` 硬件接口：链接 `z1_controller` 中的 `libZ1_<arch>.so`
   （FSM、`Z1Model`、`IOUDP` 都在其中，且不依赖 ROS），并以 `SystemInterface`
   的形式暴露出来；之后 `hardware_plugin:=z1_ros2_control/Z1System` 即可把整套栈
   切到真机上。
   * 把 250 Hz 的 UDP 收发放在独立线程里，绝不要放进 `write()`
   * **不要**照搬 ROS 1 `main.cpp` 把整个进程设为 `SCHED_FIFO` 最高优先级的做法——
     那会把 executor 饿死；应给 I/O 线程一个有上限的优先级
   * 把控制器的 FSM 映射为服务（`~/clear_error`、`~/calibrate`、`~/teach`、
     `~/back_to_start`），并把断连/过温上报为硬件错误

## 已知限制

* gz-servo 路径（`gazebo.launch.py`）与 ros2_control 路径不能同时运行——
  同一个关节只能有一个“所有者”。
* dartsim 会忽略网格碰撞几何，因此夹爪的 STL 碰撞体不会被仿真
  （机械臂各 link 使用圆柱基元，不受影响）。
* 力矩律的 i = 0，所以有重力静差（实测 0.001–0.021 rad）——这是官方行为的原样复现；
  容差留了 0.05 rad 的余量。
* 真机尚未验证——硬件插件目前还不存在。
