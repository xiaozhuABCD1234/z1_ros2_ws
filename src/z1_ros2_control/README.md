# z1_ros2_control

真机 Unitree Z1 的 ros2_control 后端：一个 `hardware_interface::SystemInterface`，
通过**官方 `z1_controller`**（进程名 `z1_ctrl`）驱动机械臂。

已在实机上跑通：激活 → 保持姿态 → 走轨迹 → 回位 → 切 PASSIVE，见
[实机验证结果](#实机验证结果)。

## 为什么是"客户端"而不是"驱动"

`z1_ctrl` 才是唯一的实时主控：它拥有 250 Hz 的电机环、状态机（FSM）、逆运动学、
碰撞检测和整定好的关节伺服律，并通过 `192.168.123.110:8881` 直接对机械臂说话。
本包只做一件事：**在 loopback 上扮演官方 `z1_sdk` 的角色**。

```
ros2_control / MoveIt
        │  position 命令接口（ros2_control 内部）
        ▼
   Z1System（本包，插件）  ──UDP 147 B SendCmd──▶  z1_ctrl ──UDP──▶ 机械臂 :8881
        ▲                  ◀─UDP 208 B RecvState── (127.0.0.1:8071)     192.168.123.110
        └── 位置/速度/力矩 状态接口
```

这样划分的三个理由：

1. **不重复实时的活**。关节伺服律、限速、重力补偿都在 `z1_ctrl` 里，本包只送目标位置，
   不产生力矩。真机上的实测轨迹跟踪误差 ~0.002 rad。
2. **不链接 Unitree 的预编译库**。`libZ1_x86_64.so` 里是整套 FSM 和它自己的线程；
   把它塞进 ros2_control 进程会得到两套状态机。本包只 vendor 了 BSD-3-Clause 的
   `arm_common.h`（见 `thirdparty/unitree/z1/PROVENANCE.md`）。
3. **一条链路只有一个客户端**。`z1_ctrl` 的 `ARMSDK` 把 `RecvState` 固定发到 8072，
   所以本进程必须独占它——FSM 服务因此挂在**硬件组件自己的节点**上，而不是另外起一个
   节点（另一个进程抢 8072 只会让两边都收不到）。

## 依赖：官方 z1_controller

本包**不能**单独工作。先准备 `z1_ctrl`：

```bash
# 1. 取源码（本工作空间用的是 master @ 639bb773bd11b0e8495faf40eadc9e8eb8bc8400）
cd ~/Projects
git clone https://github.com/unitreerobotics/z1_controller.git   # 或下载该 commit 的 tarball

# 2. 编译（需要 Boost 与 Eigen3，Ubuntu 上是 libboost-all-dev / libeigen3-dev）
cd z1_controller && mkdir -p build && cd build
cmake .. && make -j$(nproc)      # 产出 build/z1_ctrl，并链到 lib/libZ1_x86_64.so

# 3. 网卡必须和机械臂同网段。机械臂固定 192.168.123.110，
#    所以本机要有 192.168.123.x/24：
ip -brief addr | grep 192.168.123
ping -c1 192.168.123.110
```

`z1_controller/config/config.xml` 里的 IP/端口（默认 `192.168.123.110` / `8881`）
必须和实际机械臂一致；`unitreeArmTools.py` 可以改机械臂自己的 IP。

**启动顺序不能颠倒**，而且 `z1_ctrl` 必须从它的 `build/` 目录启动（源码里写死了
`armConfigPath = "../config/"`）：

```bash
# 终端 1：先起控制器。它一启动就把机械臂置为 PASSIVE（无力矩），
#         所以机械臂会失去支撑——确认它在安全位置或有人扶住。
cd ~/Projects/z1_controller/build && ./z1_ctrl

# 终端 2：再起 ROS 2 侧
cd ~/Projects/z1_ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch z1_bringup control.launch.py use_gripper:=false
```

顺序错了会怎样：本组件在 `on_activate()` 里必须先收到一帧 `RecvState` 才敢发包
（否则第一帧就是"零位指令"），所以 `z1_ctrl` 不在就会激活超时；而 ros2_control 在
硬件激活失败时会直接 `std::terminate` 整个 `ros2_control_node`。

## 接口

| 方向 | 名称 | 说明 |
|---|---|---|
| 命令 | `joint1..joint6`（+`jointGripper`） `/position` | 唯一被转发的命令接口 |
| 状态 | `joint1..joint6`（+`jointGripper`） `/position` `/velocity` `/effort` | 来自每个 `RecvState` |
| 服务 | `~/set_fsm_state`（`z1_ros2_control/srv/SetFSMState`） | 请求 FSM 状态，可带 label |
| 服务 | `~/get_fsm_state`（`z1_ros2_control/srv/GetFSMState`） | 只读查询 |

节点名就是 URDF 里 `<ros2_control name="z1">` 的 `z1`，所以默认是
`/z1/set_fsm_state`、`/z1/get_fsm_state`。

```bash
ros2 service call /z1/get_fsm_state z1_ros2_control/srv/GetFSMState "{}"
ros2 service call /z1/set_fsm_state z1_ros2_control/srv/SetFSMState "{state: JOINTCTRL, label: ''}"
ros2 service call /z1/set_fsm_state z1_ros2_control/srv/SetFSMState "{state: TOSTATE, label: show_mid}"
ros2 service call /z1/set_fsm_state z1_ros2_control/srv/SetFSMState "{state: PASSIVE, label: ''}"
```

只暴露位置命令接口是刻意的：`z1_ctrl` 的关节伺服律是固定的，送增益或力矩没有意义。

## 参数（URDF `<hardware><param>`）

`z1_bringup/launch/control.launch.py` 的同名 launch 参数会传到这些 `<param>`，
默认值一致，**改一处要改两处**。

| 参数 | 默认 | 含义 |
|---|---|---|
| `ctrl_ip` | `127.0.0.1` | `z1_ctrl` 所在主机。协议只支持同机（loopback） |
| `ctrl_port` | `8071` | `z1_ctrl` 的 `ARMSDK` 绑定端口 |
| `own_port` | `8072` | 本组件绑定；`z1_ctrl` 把 `RecvState` 发到这里 |
| `activate_fsm_sequence` | `JOINTCTRL` | 激活时依次请求的状态（逗号分隔） |
| `disconnect_timeout_ms` | `200` | 超过这么久没有 `RecvState` 就报硬件错误 |
| `temperature_limit` | `80` | 过温阈值（℃） |
| `fsm_timeout_ms` | 默认 `2000`（未暴露成 launch 参数） | 等一次 FSM 转移的上限 |

## 生命周期与线程模型

这是本包最容易踩坑的地方，也是实机调试出来的结论。

### 只有 ACTIVE 才会调用 read()/write()

ros2_control 的 `AsyncComponentThread`（Jazzy 4.48）逐字是：

```cpp
if (component->get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
  component->write(...); component->read(...);
}
```

也就是说**激活过程中 `read()` 一次都不会被调用**。而"先收到一帧 `RecvState`，
把命令锁成实测位姿，再请求 `JOINTCTRL`"这套握手的收发都在 `read()` 里——最初的版本
因此必然激活超时（`no RecvState from z1_ctrl within 2000 ms`）。

解法：socket I/O 全部收进 `Z1System::pump_once()`，它是**唯一**碰 socket 和 `cmd_` 的
代码，由三处驱动：

* `read()`——异步工作线程，只在 ACTIVE 期间；
* `on_activate()` / `on_deactivate()`——生命周期回调自己泵（`wait_for_first_packet()`、
  `request_state_and_wait()` 的循环里各调一次）；
* `set_fsm_state` 回调——组件 INACTIVE 但链路还 armed 时（例如只想把机械臂切到
  `BACKTOSTART` 而不激活控制器），此时 `read()` 不跑，必须自己泵。

### 并发：无锁的 I/O 占用标志

上面三处可能同时想泵：异步线程在跑 `read()`，而 service 回调在 controller_manager 的
执行器上。`pump_once()` 里的 `assemble_frame()` 是逐字段写 `cmd_` 的，**并发组装会发出
一条"混合"的指令**——那不是过期指令，是另一条指令。所以用 `IoGuard`（一个
`std::atomic<bool>` 的 try-lock）保证同一时刻只有一个泵：

* 抢不到的直接**跳过**，绝不等待——等待就是在 I/O 路径上放锁，会把 500 Hz 线程拖进
  优先级反转；
* 跳过是安全的：另一处正在做同样的事，而所有调用点要么重试（等待循环），要么并不
  在意（`read()`/`write()` 下一拍就有新帧）。

`read()`/`write()` 里刻意只用普通 `double` 暴露状态，容忍撕裂读（最多把相邻两帧的
关节混在一起，控制器能吃下）；唯一的互斥量 `service_mutex_` 只被 `set_fsm_state`
回调持有。

### 传输率

`rw_rate="500"` 是设计目标（`z1_ctrl` 的 `ARMSDK` 循环 `dt = 0.002 s`），但
**ros2_control 4.48 不认它**：异步工作线程是按 controller_manager 的 `update_rate`
跑的。实机实测：

```
[controller_manager.hardware_component.system.z1]: read() is being called at 250 Hz
                                                   (rw_rate in the URDF says 250).
```

`z1_ctrl` 接受 250 Hz（它进入 joint space control 并稳定保持），所以两者保持解耦；
`read()` 会在速率首次测出、以及偏离上次报告值 ±20% 时打印实测值。想让它严格 500 Hz，
把 `z1_bringup/config/z1_controllers.yaml` 的 `controller_manager.update_rate` 改成
500（代价是控制器也按 500 Hz 更新）。

## FSM

### 可达的状态

只有"能用位置指令驱动"的状态才开放。依据是对 `libZ1_x86_64.so` 的反汇编
（地址见 `z1_protocol.hpp` 的注释）：

| 类别 | 状态 | 依据 |
|---|---|---|
| 忽略 `valueUnion` | `PASSIVE`、`JOINTCTRL`、`BACKTOSTART`、`CALIBRATION` | 它们的 enter/run 里没有任何对 `sendCmd` 载荷的读取 |
| 读 `valueUnion.name` | `TOSTATE`、`SAVESTATE`、`TEACH`、`TEACHREPEAT` | 各自 `enter()` 里的 `lea 0x5f(%rcx)`，即 `SendCmd.valueUnion` |
| **不开放** | `CARTESIAN` | 把 `valueUnion` 当 6 个 **double**（姿态增量）读（`State_Cartesian::run()` 里的 `movsd 0x7b(%rax)`），而本包只往里写 float |
| **不开放** | `MOVEJ`/`MOVEL`/`MOVEC`/`TRAJECTORY` | 需要笛卡尔目标或 `valueUnion.trajCmd`，本包不产生 |
| **不开放** | `LOWCMD` | 绕过 `z1_ctrl` 的关节伺服律；`parseFsmStateName()` 故意不认这个名字 |

* `activate_fsm_sequence` 只接受**第一类**：那串状态在任何控制器起来之前就要生效，
  那时没人能提供 label。
* `set_fsm_state` 接受第一类 + 第二类；第二类必须带 `label`，第一类必须不带（label 会
  覆盖 `jointCmd[0]`，两者在 union 里同址）。
* 已经在目标状态时服务返回成功但**什么也不做**，并在 `message` 里说明——`z1_ctrl` 的
  `checkChange()` 不会重入当前状态，所以 `TOSTATE` → 另一个 `TOSTATE` 的新 label 要先
  离开该状态（例如先切 `JOINTCTRL`）才生效。

### label 取自 z1_ctrl 的 CSV

`label` 只能是 `z1_controller/config/savedArmStates.csv` 里的条目名（最多 9 字符，
字段是 `char[10]` 且 `z1_ctrl` 会拿它构造 `std::string`）。本工作空间的
`z1_moveit_config` 用其中两个：

```
forward,    0.0, 1.5,  -1.0,   -0.54,  0.0, 0.0
startFlat,  0.0, 0.0,  -0.005, -0.074, 0.0, 0.0
show_left, show_mid, show_right, ...
```

label 从 service 线程交给 I/O 路径用的是 epoch 校验的双缓冲（`label_` 是
`std::atomic<char>` 数组 + `label_epoch_`）：写者写完字节再递增 epoch，`read()` 在复制
前后各读一次 epoch，相等才采纳。所以撕裂的 label 永远不会上线，而服务回调会等到
"已被采纳"才真正请求状态——`z1_ctrl` 是在**进入**状态时读 `name` 的。

## 安全设计

* **激活不是阶跃**。`on_activate()` 先把命令缓冲设成实测关节角（对应官方 SDK 的
  `startTrack()`），再请求 `JOINTCTRL`。
* **不知道位置就不许发包**。`wait_for_first_packet()` 之前 `link_armed_` 为 false，
  一个字节都不发——零填充的 `SendCmd` 就是"去零位"。
* **无夹爪时第 7 个电机跟随实测角**，且被排除在故障判定之外。实机上 z1_ctrl 会打印
  `[GRIPPER] The arm does not have gripper`，第 7 槽读数为温度 0、错误 0。若把它当故障
  检查，一台健康的 6 轴机械臂会每周期返回 ERROR（这个坑只有真机能暴露）。
* **故障一律返回硬件 ERROR**：链路 200 ms 无包、电机 error 位、未连接/CRC、过温。
  返回 ERROR 会让 ros2_control 停掉控制器，随后本进程退出 → `z1_ctrl` 因为收不到
  `SendCmd` 而自己回到 PASSIVE。
* **`0x40` 不是故障**。真机六个关节全部常驻 `error=0x40`，而 vendored 头里该位注释为
  "nothing"，`kErrorMask` 已排除它——实机确认这个判断是必须的。
* 不暴露 `LOWCMD`、不链接 Unitree 的 `.so`、不用 `SCHED_FIFO` 占满整个进程。

## 实机验证结果

硬件：本机 `enp89s0 = 192.168.123.99/24`，机械臂 `192.168.123.110`（ping 0.4 ms），
无末端夹爪，`use_gripper:=false`。

已确认：

| 项 | 结果 |
|---|---|
| `z1_ctrl` 无客户端时的输出 | 仍以 ~10 Hz 发 `RecvState`（6.01 s 收到 61 帧，0 错长度/错包头） |
| 只读遥测 | `PASSIVE`，六关节温度 33–35 ℃，全部 `connected=ok`、`error=0x40` |
| 双电机关节 | `joint2` 报两组 `Motor_State`（另一组温度 35 ℃、`error=0x40`），其余只有第一组 |
| 激活 | `active: z1_ctrl reports JOINTCTRL, commanding the measured pose`，`z1_ctrl` 打印 `Switched from passive to joint space control` |
| 角度一致性 | `z1_ctrl` 自己的 `joint space q` 与 `/joint_states` 一致（-0.00913 -0.00092 0.08303 -0.07593 0.02260 -0.03599） |
| 传输率 | `read()` 实测 250 Hz（`rw_rate` 被框架覆盖成 CM 的 `update_rate`） |
| 轨迹 | 单点轨迹 `joint1 → +0.05 rad`：`Goal successfully reached!`，实测到位 +0.04794 |
| 回位 | 回原姿态同样 `Goal successfully reached!` |
| FSM 服务 | `set_fsm_state PASSIVE` → `success=True, message='PASSIVE acknowledged by z1_ctrl'` |
| 故障上报 | 全程无电机/温度/链路故障 |

**尚未在实机上验证**：`BACKTOSTART`、`CALIBRATION`、`TEACH`/`TEACHREPEAT`、
`TOSTATE`/`SAVESTATE` 的真实动作；进程被杀后 `z1_ctrl` 自动回 PASSIVE 的时延；
夹爪（本机没有）。

### 单点轨迹是绝对位置

上面那次"单关节"验证实际上动了 6 个关节：`joint_trajectory_controller` 的单点轨迹里
`positions` 是**每个列出关节的绝对目标**，没被轨迹点覆盖的关节会被拉到该点里的值。
只给了 `[0.05, 0, 0, 0, 0, 0]`，于是 joint3/4/5/6 也从 (0.080, -0.076, 0.023, -0.036)
被拉到 ~0。幅度都在 0.2 rad 以内所以无害，但**要动一个关节就必须显式写出其余关节的
当前位置**（或用 MoveIt 规划）。

## 无机械臂时

`scripts/z1_ctrl_mock.py` 扮演 `z1_ctrl`（不是扮演机械臂）：绑定 8071、按速率回
`RecvState`、校验 `SendCmd` 的头和长度、用一阶模型让轨迹真能收敛，并复刻
"客户端停发 → 强制 PASSIVE" 和 `--reject-states` 非法转移。它用来验证本包而不必接
真机：

```bash
ros2 run z1_ros2_control z1_ctrl_mock.py --gripper --drop-rate 0.05
```

注意它只是控制器替身。真机路径上真正的握手、`read()` 的调用时机、`0x40` 错误位、
双电机关节这类问题它都盖不住。

## 已知问题

* 启动日志里的三条噪声都无害：
  `AsyncFunctionHandler is configured with DETACHED scheduling policy`、
  `ResourceManager has already loaded a urdf`（`/robot_description` 被投递了两次）、
  以及偶发的 `spawner_joint_state_broadcaster ... can not be configured from 'active' state`
  （`joint_state_broadcaster` 最终是 active 的）。
* 没有 RT 优先级：日志会提示
  `Could not enable FIFO RT scheduling policy`。需要的话按 ros2_control 文档配
  `limits.conf`/`rtprio`。
* 只能跑一个客户端（协议本身如此），所以 `set_fsm_state` 必须在同一个进程里；
  另外 `ros2 control` 的其它进程不能同时占 8072。
* `ctrl_ip` 只支持 loopback：`z1_ctrl` 与本组件必须同机。
