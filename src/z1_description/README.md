# z1_description

Unitree Z1 机械臂（6 自由度 + 可选夹爪）的 ROS 2 描述包。

## 目录结构

```
urdf/
  z1.urdf.xacro             # 入口：参数 + 各 include
  const.xacro               # 质量 / 惯量 / 限位 / 碰撞体尺寸 / 材质
  z1_body.xacro             # link 与 joint（宏 z1_body）
  z1_ros2_control.xacro     # <ros2_control> 块（宏 z1_ros2_control）
  z1_gz_ros2_control.xacro  # gz_ros2_control 插件，仅该后端使用
  z1_gazebo.xacro           # gz-sim 伺服系统（宏 z1_gazebo）
meshes/{visual,collision}/
rviz/z1_display.rviz
launch/display.launch.py
```

解析方式：

```bash
ros2 run xacro xacro $(ros2 pkg prefix z1_description)/share/z1_description/urdf/z1.urdf.xacro
```

## xacro 参数

| 参数 | 默认值 | 作用 |
|---|---|---|
| `use_gripper` | `true` | 附加 `gripperStator` / `gripperMover` 与 `jointGripper` |
| `hardware_plugin` | `z1_ros2_control/Z1System` | ros2_control 后端：`z1_ros2_control/Z1System`（真机）或 `gz_ros2_control/GazeboSimSystem`（Gazebo） |
| `use_gazebo` | `false` | 加入 gz-sim 的 `JointStatePublisher`，并为每个关节加一个 `JointPositionController`（独立 Gazebo 路径，不使用 ros2_control） |
| `use_world_link` | `true` | 加入 `world` link，并把 `link00` 固定到它上面（固定基座） |

## 与 ROS 1 原版的差异

1. **固定法兰关节由 `gripperStator` 改名为 `gripperStator_joint`。**
   在 SDF 中 link 和 joint 共用一个坐标系命名空间，原版命名（joint 与 link 同名）
   会让 `gz sim` 报错 `frame with name[gripperStator] already exists`。
2. **每个 visual 上都显式写了 `<material>`。** Blender 导出的 COLLADA 材质在
   gz-sim/assimp 路径上不会生效，link 会渲染成过曝的白色——在 Gazebo 背景里几乎不可见。
   现在所有 link 使用显式定义的灰色材质。
3. **移除 transmissions**：ROS 2 的 URDF 中没有 `transmission_interface`，
   该职责由 `<ros2_control>` 接管。
4. **移除 `gazebo_ros_control` 插件**：改由 gz-sim 系统（`z1_gazebo.xacro`）
   和 ros2_control（`z1_ros2_control.xacro`）承担。
5. `<xacro:arg UnitreeGripper>` → `use_gripper`；基于 catkin/`$(find)` 的 include
   保留（xacro 会通过 ament index 解析 `$(find <pkg>)`）。
6. **原版中 `link01` / `link05` 没有碰撞几何**，只有 visual（MoveIt 会给出警告）。
   这里保持原样，没有自行编造几何体。

## 暴露给 ros2_control 的接口

每个关节（`joint1`…`joint6`、`jointGripper`）：

* 命令：`position`；但 `hardware_plugin:=gz_ros2_control/GazeboSimSystem` 时，六个机械臂
  关节改成 `effort`（gz_ros2_control 的 position 接口只有单一 P、无 D，会抖；
  换 effort 后由 `z1_controllers` 施加官方力矩律）。夹爪在所有后端都是 `position`。
* 状态：`position`、`velocity`、`effort`

声明 `effort` 是为了便于查看；真机控制器的 `tau` 反馈即为 effort 状态，Gazebo 后端由 dartsim 上报。

## 许可证

网格模型与数值：BSD-3-Clause，© 2016-2022 HangZhou YuShu TECHNOLOGY CO.,LTD.
（"Unitree Robotics"）——详见 `LICENSE`。
