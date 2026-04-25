# UR5 Rehab Workspace 使用说明

本仓库是一个基于 ROS Noetic + Gazebo 的 UR5 康复与导纳控制工作空间，核心目标包括：

- 运行 UR5 三种康复笛卡尔轨迹（直线、弧线、圆轨迹）
- 运行上肢前伸直线稳定性验证，并统计末端平均跟踪误差
- 运行定点循环干扰力下的导纳控制（被动顺应）

本文面向“克隆仓库后直接上手”的用户，尽量给出可直接复制的命令。

---

## 1. 工作空间结构梳理

仓库主目录结构（与功能相关部分）：

```text
ur5_rehab_ws/
├── src/
│   ├── ur5_gazebo/                 # 核心康复与传感处理包
│   │   ├── launch/
│   │   ├── scripts/
│   │   └── config/ur5_controllers.yaml
│   ├── ur5_gazebo_advance/         # 导纳控制与扰动力发布
│   │   ├── launch/
│   │   └── scripts/
│   ├── universal_robot/            # UR 官方模型、ur_gazebo、MoveIt 配置
│   ├── ros_control/
│   └── ros_controllers/
├── build/
└── devel/
```

功能重点集中在两个自定义包：

- `src/ur5_gazebo`
- `src/ur5_gazebo_advance`

---

## 2. 环境准备与编译

## 2.1 系统要求

建议环境：

- Ubuntu 20.04
- ROS Noetic
- Gazebo（Noetic 默认 Gazebo11）

## 2.2 克隆后编译

在工作空间根目录执行：

```bash
cd ~/ur5_rehab_ws
catkin_make
source devel/setup.bash
```

建议把 source 命令加入 `~/.bashrc`：

```bash
echo "source ~/ur5_rehab_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 3. 自定义 Launch 文件总览（重点）

以下是你最常用、最需要了解的 launch：

## 3.1 `ur5_gazebo` 包

1. `ur5_gazebo.launch`
- 作用：启动 UR5 Gazebo + 康复 IK + 康复轨迹控制
- 支持三种轨迹：
  - `line` 直线
  - `arc` 弧线
  - `circle` 圆
  - `all` 顺序执行三种

2. `ur5_gazebo_test.launch`
- 作用：启动 UR5 Gazebo + 简单演示关节轨迹（`ur5_demo_motion.py`）
- 用途：快速验证控制器/仿真链路是否正常

3. `ur5_rehab_stability_validation.launch`
- 作用：
  - 运行“上肢前伸直线（5s）”稳定性验证
  - 计算并输出末端位置平均跟踪误差
- 直线轨迹：
  - 起点 `(0.4, 0.0, 0.6)`
  - 终点 `(0.6, 0.2, 0.5)`
  - 时长 `5s`

4. `ur5_ft_wrench_corrector.launch`
- 作用：力传感器数据去偏置/缩放，发布修正后的 wrench
- 典型输出：`/ur5/ft_sensor/wrench_corrected`

5. `ur5_external_wrench_applier.launch`
- 作用：将命令 wrench 施加到 Gazebo 中 UR5 指定连杆（默认 wrist_3_link）
- 典型输入：`/ur5/ft_sensor/command_wrench`

## 3.2 `ur5_gazebo_advance` 包

1. `ur5_admittance_control.launch`
- 作用：一键启动“定点导纳控制 + 循环扰动力”完整链路
- 可选启动内容：
  - 机器人仿真
  - 力数据修正
  - 外力施加
  - 周期扰动力发布
  - 导纳控制主节点

## 3.3 依赖的上游 Launch（自动 include）

1. `universal_robot/ur_gazebo/launch/ur5_bringup.launch`
- 作用：官方 UR5 Gazebo 启动入口，加载模型和 ros_control 控制器
- 你的自定义 launch 基本都通过 include 这个文件来启动机器人

---

## 4. 如何运行：三种康复轨迹

## 4.1 一次运行完三种轨迹

```bash
roslaunch ur5_gazebo ur5_gazebo.launch trajectory_mode:=all loop:=false
```

## 4.2 只运行某一种轨迹

```bash
# 直线轨迹
roslaunch ur5_gazebo ur5_gazebo.launch trajectory_mode:=line

# 弧线轨迹
roslaunch ur5_gazebo ur5_gazebo.launch trajectory_mode:=arc

# 圆轨迹
roslaunch ur5_gazebo ur5_gazebo.launch trajectory_mode:=circle
```

## 4.3 循环运行（长时间观察）

```bash
roslaunch ur5_gazebo ur5_gazebo.launch trajectory_mode:=all loop:=true
```

常用可调参数：

- `sample_dt`：采样周期（默认 0.01）
- `start_delay`：轨迹下发后延时启动
- `pause_between`：不同轨迹之间的间隔

---

## 5. 如何运行：上肢前伸稳定性验证 + 平均误差

启动命令：

```bash
roslaunch ur5_gazebo ur5_rehab_stability_validation.launch
```

该流程会自动做三件事：

1. 生成并执行前伸直线轨迹
2. 由关节角做正运动学得到实际末端位姿
3. 对齐参考轨迹与实际轨迹，输出平均/最大位置误差（mm）

终端可关注日志关键词：

- `UR5 end-effector tracking mean error`
- `UR5 end-effector tracking max  error`

避免“机械臂初始躺地抬起阶段”误差污染：

- launch 中已提供：
  - `eval_start_delay`（默认 1.5s）
  - `eval_start_min_z`（默认 0.45m）
- 仅当满足时间和高度门限后才开始统计误差

---

## 6. 如何运行：定点循环干扰力导纳控制

## 6.1 一键运行完整链路（推荐）

```bash
roslaunch ur5_gazebo_advance ur5_admittance_control.launch
```

默认行为：

- UR5 固定目标位姿附近导纳控制
- 周期扰动力沿 x 轴施加
- 力经 corrector 修正后驱动导纳器

## 6.2 常用参数示例

```bash
roslaunch ur5_gazebo_advance ur5_admittance_control.launch \
  wrench_axis:=x \
  wrench_amplitude:=8.0 \
  wrench_frequency:=0.8 \
  fixed_position:='[0.45, 0.0, 0.55]' \
  md_diag:='[1.0, 1.0, 1.0]' \
  bd_diag:='[60.0, 60.0, 60.0]' \
  kd_diag:='[220.0, 220.0, 220.0]'
```

参数含义简述：

- `fixed_position`：导纳平衡点
- `md_diag`：虚拟质量（大一些响应更“钝”）
- `bd_diag`：虚拟阻尼（大一些振荡更小）
- `kd_diag`：虚拟刚度（大一些回位更强）
- `wrench_amplitude/frequency`：干扰力幅值和频率

## 6.3 分模块调试（可选）

只开导纳控制，不开周期扰动力：

```bash
roslaunch ur5_gazebo_advance ur5_admittance_control.launch start_periodic_wrench:=false
```

只测试外力施加节点：

```bash
roslaunch ur5_gazebo ur5_external_wrench_applier.launch
```

只测试力修正节点：

```bash
roslaunch ur5_gazebo ur5_ft_wrench_corrector.launch
```

---

## 7. 主动康复模式新增内容（Mode1 / Mode2）

这一部分对应 `src/ur5_gazebo_advance` 中新增的主动康复控制链路，目标是补充两种模式：

- `mode1`：无预设轨迹，参考位姿取当前末端位姿，用户通过外力推动末端，观察导纳顺应效果
- `mode2`：有预设圆轨迹参考，机械臂沿名义轨迹运动，同时允许导纳偏移，便于观察“轨迹 + 顺应”的组合效果

## 7.1 新增/修改文件概览

本次与主动康复直接相关的新增或修改如下：

- 新增脚本：
  - `src/ur5_gazebo_advance/scripts/ur5_active_rehab_admittance_controller.py`
  - `src/ur5_gazebo_advance/scripts/ur5_circular_reference_generator.py`
  - `src/ur5_gazebo_advance/scripts/ur5_delayed_tare_trigger.py`
- 新增 launch：
  - `src/ur5_gazebo_advance/launch/ur5_active_rehab_common.launch`
  - `src/ur5_gazebo_advance/launch/ur5_active_rehab_mode1.launch`
  - `src/ur5_gazebo_advance/launch/ur5_active_rehab_mode2.launch`
- 新增参数文件：
  - `src/ur5_gazebo_advance/config/mode1_params.yaml`
  - `src/ur5_gazebo_advance/config/mode2_controller_params.yaml`
  - `src/ur5_gazebo_advance/config/mode2_trajectory.yaml`
- 修改上游 bringup：
  - `src/universal_robot/ur_gazebo/launch/ur5_bringup.launch`

`ur5_bringup.launch` 的修改只有一项核心作用：新增 `gazebo_world` 参数并继续向下传递。这样上层的主动康复 launch 就能决定 Gazebo 启动时加载哪个 world，而不必硬编码在官方 bringup 内部。

## 7.2 新增 Python 脚本说明

1. `ur5_active_rehab_admittance_controller.py`

- 功能：主动康复主控制器，统一支持 `mode1` 和 `mode2`
- 输入：
  - `/joint_states`
  - `/ur5/ft_sensor/wrench_corrected`
  - `/ur5/end_effector_pose`
  - `mode2` 下还会订阅 `/ur5/active_rehab/trajectory_reference`
- 输出：
  - `/ur5_arm_controller/command`
  - `/ur5/active_rehab/reference_pose`
  - `/ur5/active_rehab/nominal_reference_pose`
- 主要实现：
  - 在笛卡尔空间做三维导纳积分
  - 用数值 IK 把参考末端位姿转成 6 关节轨迹点
  - `mode1` 首次收到末端位姿后，将该位姿作为初始参考位姿
  - `mode2` 将轨迹生成器给出的圆轨迹作为名义参考，再叠加导纳偏移

2. `ur5_circular_reference_generator.py`

- 功能：为 `mode2` 按固定频率发布圆轨迹参考位姿
- 输入参数：圆心、半径、持续时间、平面、姿态四元数
- 输出：`/ur5/active_rehab/trajectory_reference`
- 主要实现：
  - 根据 `mode2_trajectory.yaml` 的配置生成 `xy`、`yz` 或 `xz` 平面圆轨迹
  - 支持 `loop`，默认循环发布，便于持续观察轨迹跟踪效果

3. `ur5_delayed_tare_trigger.py`

- 功能：启动后延时触发 FT 传感器去皮（tare）
- 默认行为：等待 `3.0s` 后，向 `/ur5/ft_sensor/tare` 连续发布 2 次 `std_msgs/Empty`
- 设计原因：
  - Gazebo、控制器、重力项和传感器链路在启动的前几秒通常还未稳定
  - 若过早采集零点，容易把启动瞬态误当成外力偏置
  - 延时 tare 能让 `/ur5/ft_sensor/wrench_corrected` 更接近真正的零力状态

## 7.3 新增 Launch 文件说明

1. `ur5_active_rehab_common.launch`

- 功能：主动康复模式的公共启动骨架
- 负责统一拉起以下共用链路：
  - UR5 Gazebo bringup
  - FT wrench corrector
  - delayed tare
  - forward kinematics
  - 可选 external wrench applier
  - 可选 periodic wrench publisher
- 设计目的：把 `mode1` 和 `mode2` 共用的底层启动步骤集中到一个文件，避免重复维护两套相同的启动逻辑

2. `ur5_active_rehab_mode1.launch`

- 功能：启动主动康复 `mode1`
- 当前默认行为：
  - 启动 `ur5_active_rehab_common.launch`
  - 启动 `ur5_active_rehab_admittance_controller.py`
  - 默认启用 `start_external_wrench_applier:=true`
  - 默认 world 为 `worlds/empty.world`
- 适用场景：通过 `rostopic pub` 向 `/ur5/ft_sensor/command_wrench` 发外力命令，观察末端顺应

3. `ur5_active_rehab_mode2.launch`

- 功能：启动主动康复 `mode2`
- 当前默认行为：
  - 启动 `ur5_active_rehab_common.launch`
  - 启动 `ur5_circular_reference_generator.py`
  - 启动 `ur5_active_rehab_admittance_controller.py`
  - 启动 `ur5_rehab_tracking_error_monitor.py`
- 适用场景：运行圆轨迹主动康复演示，并同时评估名义参考轨迹与实际末端轨迹误差

## 7.4 参数文件说明

1. `mode1_params.yaml`

- 当前参数：`Md=diag(1,1,1)`、`Bd=diag(10,10,10)`、`Kd=diag(20,20,20)`
- 作用：控制 `mode1` 的导纳刚柔程度和位移修正范围
- 当前还设置了：
  - `force_deadband: 0.0`
  - `max_correction: [0.10, 0.10, 0.10]`

2. `mode2_controller_params.yaml`

- 当前参数：`Md=diag(1,1,1)`、`Bd=diag(50,50,50)`、`Kd=diag(200,200,200)`
- 作用：使 `mode2` 在跟踪预设圆轨迹时回位更强、阻尼更大

3. `mode2_trajectory.yaml`

- 当前默认轨迹：
  - 平面：`yz`
  - 圆心：`[0.0, 0.5, 0.5]`
  - 半径：`0.1`
  - 周期：`8.0s`
  - 循环：`true`

## 7.5 使用说明：主动康复 Mode1

启动方式：

```bash
roslaunch ur5_gazebo_advance ur5_active_rehab_mode1.launch
```

该命令默认会自动启动：

- UR5 Gazebo 仿真
- wrench corrector
- delayed tare
- forward kinematics
- external wrench applier
- `mode1` 导纳控制器

如果要通过 `pub` 的方式施加外力，可向 `/ur5/ft_sensor/command_wrench` 发布 `geometry_msgs/WrenchStamped`。

示例 1：沿世界坐标系 `+x` 方向持续施加 8N 外力

```bash
rostopic pub -r 50 /ur5/ft_sensor/command_wrench geometry_msgs/WrenchStamped \
'{header: {frame_id: "world"}, wrench: {force: {x: 8.0, y: 0.0, z: 0.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}}'
```

示例 2：沿世界坐标系 `+z` 方向持续施加 8N 外力

```bash
rostopic pub -r 50 /ur5/ft_sensor/command_wrench geometry_msgs/WrenchStamped \
'{header: {frame_id: "world"}, wrench: {force: {x: 0.0, y: 0.0, z: 8.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}}'
```

停止施力的方法：

- 直接 `Ctrl-C` 停止 `rostopic pub`，外力注入节点会在超时后自动清零
- 或手动发一条零力消息：

```bash
rostopic pub -1 /ur5/ft_sensor/command_wrench geometry_msgs/WrenchStamped \
'{header: {frame_id: "world"}, wrench: {force: {x: 0.0, y: 0.0, z: 0.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}}'
```

## 7.6 使用说明：主动康复 Mode2

启动方式：

```bash
roslaunch ur5_gazebo_advance ur5_active_rehab_mode2.launch
```

默认行为：

- 先完成 Gazebo、FK、wrench corrector、delayed tare 等公共链路启动
- 圆轨迹参考生成器持续发布圆轨迹位姿
- 导纳控制器跟踪名义圆轨迹，并根据修正后的外力做顺应偏移
- 误差监视器输出轨迹执行误差，便于验收 `mode2`

常见可调参数示例：

```bash
roslaunch ur5_gazebo_advance ur5_active_rehab_mode2.launch \
  start_error_monitor:=true
```

如果要修改圆轨迹尺寸或平面，直接调整 `src/ur5_gazebo_advance/config/mode2_trajectory.yaml` 中的 `center`、`radius`、`plane`、`duration` 即可。

## 7.7 相关改动说明

除了新增脚本和 launch，本次主动康复链路还依赖以下结构性调整：

- `src/universal_robot/ur_gazebo/launch/ur5_bringup.launch`
  - 新增 `gazebo_world` 参数
  - 允许上层 launch 决定 Gazebo 加载的 world
- `src/ur5_gazebo_advance/config/*.yaml`
  - 把 `mode1` 与 `mode2` 的导纳参数、轨迹参数独立出来，便于单独调参
- `src/ur5_gazebo/launch/ur5_external_wrench_applier.launch`
  - 在 `mode1` 下作为 `pub` 施力链路的下游执行器使用

---

## 8. 关键话题链路速览

## 8.1 康复轨迹链路

- 目标笛卡尔位姿：`/ur5/rehab_target_pose` 或 `/ur5/stability_target_pose`
- IK 结果关节角：`/ur5/rehab_ik_solved_joints` 或 `/ur5/stability_ik_solved_joints`
- 控制命令：`/ur5_arm_controller/command`
- 实际关节状态：`/joint_states`

## 8.2 稳定性误差评估链路

- 参考位姿：`/ur5/stability_reference_pose`
- FK 实际位姿：`/ur5/stability_actual_pose`
- 误差结果：终端日志输出平均/最大误差

## 8.3 导纳控制链路

- 原始力：`/ur5/ft_sensor/wrench`
- 修正力：`/ur5/ft_sensor/wrench_corrected`
- 外力命令：`/ur5/ft_sensor/command_wrench`
- 主动康复参考位姿：`/ur5/active_rehab/reference_pose`
- 名义参考位姿：`/ur5/active_rehab/nominal_reference_pose`
- `mode2` 轨迹参考：`/ur5/active_rehab/trajectory_reference`
- 关节命令：`/ur5_arm_controller/command`

---

## 9. 常见问题与排查

1. 启动后机械臂不动
- 检查是否已 `source devel/setup.bash`
- 检查控制话题是否有发布：
  ```bash
  rostopic hz /ur5_arm_controller/command
  ```

2. 误差特别大
- 先看是否在抬臂阶段就开始统计
- 调大 `eval_start_delay`，或调低/调高 `eval_start_min_z` 到合适值

3. 导纳控制对外力没反应
- 确认修正力话题是否在更新：
  ```bash
  rostopic echo -n 1 /ur5/ft_sensor/wrench_corrected
  ```
- 确认外力命令话题是否在更新：
  ```bash
  rostopic echo -n 1 /ur5/ft_sensor/command_wrench
  ```

4. 第一次轨迹偶尔丢失
- 通常是控制器订阅尚未建立，可稍增 `start_delay`

---

## 10. 推荐最小复现实验顺序

1. 编译并 source
2. 运行三轨迹：
   ```bash
   roslaunch ur5_gazebo ur5_gazebo.launch trajectory_mode:=all
   ```
3. 运行稳定性验证：
   ```bash
   roslaunch ur5_gazebo ur5_rehab_stability_validation.launch
   ```
4. 运行导纳控制：
   ```bash
   roslaunch ur5_gazebo_advance ur5_admittance_control.launch
   ```
5. 运行主动康复 `mode1`：
   ```bash
   roslaunch ur5_gazebo_advance ur5_active_rehab_mode1.launch
   ```
6. 运行主动康复 `mode2`：
   ```bash
   roslaunch ur5_gazebo_advance ur5_active_rehab_mode2.launch
   ```
