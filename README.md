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
  - `src/ur5_gazebo_advance/scripts/ur5_pulse_wrench_publisher.py`
  - `src/ur5_gazebo_advance/scripts/ur5_force_marker_publisher.py`
  - `src/ur5_gazebo_advance/scripts/ur5_plot_active_rehab_bag.py`
- 新增 launch：
  - `src/ur5_gazebo_advance/launch/ur5_active_rehab_common.launch`
  - `src/ur5_gazebo_advance/launch/ur5_active_rehab_mode1.launch`
  - `src/ur5_gazebo_advance/launch/ur5_active_rehab_mode2.launch`
- 新增参数文件：
  - `src/ur5_gazebo_advance/config/mode1_params.yaml`
  - `src/ur5_gazebo_advance/config/mode2_controller_params.yaml`
  - `src/ur5_gazebo_advance/config/mode2_trajectory.yaml`
  - `src/ur5_gazebo_advance/config/ur5_force_only_visualization.rviz`
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
  - `mode1` 启动后可先移动到安全关节位姿，再将当前末端位姿作为初始参考位姿
  - `mode1` 会缓慢吸收导纳偏移到名义参考位姿中，便于表现“患者主动推动、机械臂柔顺跟随”
  - `mode2` 将轨迹生成器给出的圆轨迹作为名义参考，再叠加导纳偏移
  - 可将 wrench 从消息坐标系通过 TF 变换到控制参考坐标系，避免施力方向和控制方向不一致

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

4. `ur5_pulse_wrench_publisher.py`

- 功能：向 `/ur5/ft_sensor/command_wrench` 自动发布测试用外力，不需要手动拖动或手动 `rostopic pub`
- 支持两类模式：
  - `single_axis`：沿指定轴发布分段力，可选 `square` 或 `sine` 波形
  - `mode2_cycle`：按照 mode2 圆轨迹位置，自动生成 4 个阶段的力循环
- `mode1` 测试中常用 `single_axis + sine`：
  - 每一段 active window 内是正弦力
  - 每段之间有停顿，便于观察“施力时跟随、撤力后稳定”的效果
- `mode2` 测试中常用 `mode2_cycle`：
  - 第 1 阶段：沿圆轨迹平面法向正方向施力
  - 第 2 阶段：沿圆轨迹切向正方向施力
  - 第 3 阶段：沿圆轨迹平面法向负方向施力
  - 第 4 阶段：沿圆轨迹切向负方向施力
  - 这样可以同时验证“偏离轨迹时被柔顺拉回”和“沿轨迹方向主动用力时阻力较小”

5. `ur5_force_marker_publisher.py`

- 功能：把 wrench 显示成 RViz 中的箭头和数值文字
- 输入：默认读取 `/ur5/ft_sensor/command_wrench`
- 输出：`/ur5/active_rehab/force_markers`
- 主要用途：
  - 在 RViz 中直观看到当前施力方向和大小
  - 同时显示 X/Y/Z 坐标轴，避免误判施力坐标系
  - 配合 `ur5_force_only_visualization.rviz` 使用，启动后只关注力箭头，不显示复杂模型

6. `ur5_plot_active_rehab_bag.py`

- 功能：读取 rosbag，自动生成 mode1/mode2 的验证图片和 CSV 汇总
- `mode1` 输出重点：
  - 三个方向的力-位置响应图
  - 力-位移灵敏度图
  - 灵敏度时间序列和 CSV 汇总
- `mode2` 输出重点：
  - 指令力与修正力对比
  - 末端实际轨迹、导纳参考轨迹、名义轨迹和预设圆轨迹对比
  - 轨迹误差、导纳偏移和撤力后的恢复情况
  - 每段外力对应的峰值误差/恢复误差 CSV 汇总

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
  - 可选 pulse wrench publisher
- 设计目的：把 `mode1` 和 `mode2` 共用的底层启动步骤集中到一个文件，避免重复维护两套相同的启动逻辑

2. `ur5_active_rehab_mode1.launch`

- 功能：启动主动康复 `mode1`
- 当前默认行为：
  - 启动 `ur5_active_rehab_common.launch`
  - 启动 `ur5_active_rehab_admittance_controller.py`
  - 默认启用 `start_external_wrench_applier:=true`
  - 默认不启动 pulse wrench 和 RViz，可通过参数打开
  - 默认 world 为 `worlds/empty.world`
- 适用场景：通过正弦脉冲外力测试零力跟随效果，观察末端是否随外力柔顺移动、撤力后是否稳定

3. `ur5_active_rehab_mode2.launch`

- 功能：启动主动康复 `mode2`
- 当前默认行为：
  - 启动 `ur5_active_rehab_common.launch`
  - 启动 `ur5_circular_reference_generator.py`
  - 启动 `ur5_active_rehab_admittance_controller.py`
  - 启动 `ur5_rehab_tracking_error_monitor.py`
  - 默认启动 pulse wrench、external wrench applier 和 force RViz
- 适用场景：运行圆轨迹主动康复演示，并通过 4 阶段外力评估轨迹偏离、柔顺引导和回归能力

## 7.4 参数文件说明

1. `mode1_params.yaml`

- 当前参数：`Md=diag(1,1,1)`、`Bd=diag(10,10,10)`、`Kd=diag(20,20,20)`
- 作用：控制 `mode1` 的导纳刚柔程度和位移修正范围
- 当前还设置了：
  - `force_deadband: 0.1`
  - `max_correction: [0.06, 0.06, 0.06]`
  - `mode1_safe_start_enabled: true`
  - `mode1_reference_drift_rate`、`mode1_reference_drift_max_step`、`mode1_reference_max_offset`

2. `mode2_controller_params.yaml`

- 当前参数：`Md=diag(1,1,1)`、`Bd=diag(50,50,50)`、`Kd=diag(200,200,200)`
- 作用：使 `mode2` 在跟踪预设圆轨迹时回位更强、阻尼更大
- 当前 IK 使用位置 + 姿态约束，`ik_orientation_weight` 用于调节姿态误差权重

3. `mode2_trajectory.yaml`

- 当前默认轨迹：
  - 平面：`yz`
  - 圆心：`[0.0, 0.5, 0.5]`
  - 半径：`0.1`
  - 周期：`8.0s`
  - 循环：`true`

## 7.5 使用说明：主动康复 Mode1

### 7.5.1 Mode1 要验证什么

`mode1` 对应临床中的零力跟随/减重康复训练。它没有预设轨迹，控制器把启动后的当前末端位姿作为名义参考。患者或测试脚本施加外力时，导纳控制器根据力实时修正末端参考位姿，让机械臂顺着外力方向移动；撤去外力后，机械臂应保持稳定、没有明显震荡或卡顿。

本仓库推荐用 `ur5_pulse_wrench_publisher.py` 自动生成“周期正弦力 + 停顿”的测试：

- 正弦力阶段：观察末端是否平滑跟随
- 停顿阶段：观察撤力后是否稳定，是否有过冲或持续振荡
- 多个周期重复：用于计算力-位移灵敏度，目标是跟随灵敏度不低于 `0.5 cm/N`

### 7.5.2 启动 Mode1 正弦力测试

```bash
roslaunch ur5_gazebo_advance ur5_active_rehab_mode1.launch \
  start_pulse_wrench:=true \
  start_force_rviz:=true \
  wrench_axis:=x \
  wrench_frame_id:=base_link \
  wrench_waveform:=sine \
  wrench_amplitude:=1.0 \
  wrench_frequency:=0.5 \
  wrench_periods_per_burst:=2 \
  wrench_interval_duration:=2.0 \
  wrench_cycles:=3
```

该命令默认会自动启动：

- UR5 Gazebo 仿真
- wrench corrector
- delayed tare
- forward kinematics
- external wrench applier
- `mode1` 导纳控制器
- pulse wrench publisher
- RViz force marker 可视化

RViz 中的红色箭头表示当前命令外力方向和大小，旁边的数字是力的模长。Gazebo 中应能看到末端随正弦外力平滑偏移；正弦力结束后的停顿阶段，末端应稳定在新的柔顺位置附近，不应出现明显抖动。

### 7.5.3 使用 timeout 延时记录 rosbag

先启动上面的 launch。等 Gazebo、RViz 和控制器稳定后，在另一个终端记录 rosbag。也可以在命令前加 `sleep`，让录包自动延时开始：

```bash
mkdir -p demo_bags
sleep 8
timeout 35s rosbag record -O demo_bags/mode1_sine_follow.bag \
  /ur5/ft_sensor/command_wrench \
  /ur5/ft_sensor/wrench_corrected \
  /ur5/end_effector_pose \
  /ur5/active_rehab/reference_pose \
  /ur5/active_rehab/nominal_reference_pose \
  /joint_states
```

这里的 `sleep 8` 表示延时 8 秒再开始录制，`timeout 35s` 表示录制 35 秒后自动停止，避免把启动瞬态或过长空白段录进 bag。录制时长需要覆盖：

- Gazebo 启动后的稳定时间
- `wrench_start_delay`
- 所有正弦力 burst
- 每个 burst 之间的停顿
- 最后撤力后的稳定阶段

### 7.5.4 解析 rosbag 并绘图

```bash
mkdir -p demo_plots/mode1
rosrun ur5_gazebo_advance ur5_plot_active_rehab_bag.py \
  --mode mode1 \
  --bag demo_bags/mode1_sine_follow.bag \
  --output-dir demo_plots/mode1
```

主要输出：

- `mode1_force_position_x.png`、`mode1_force_position_y.png`、`mode1_force_position_z.png`
  - 上半图：对应轴上的命令外力
  - 下半图：实际末端位置、导纳参考位置、名义参考位置
  - 看法：外力出现时，实际末端位置应跟随参考位置同向变化；外力撤去后，曲线应趋于稳定，不应持续振荡
- `mode1_force_displacement_sensitivity.png`
  - 横轴是力的大小，纵轴是末端位移，按 X/Y/Z 分别拟合
  - 看法：拟合斜率就是灵敏度，单位是 `cm/N`；课程要求可按 `>= 0.5 cm/N` 判断是否足够柔顺
- `mode1_sensitivity_summary.png`
  - 三个方向灵敏度的柱状汇总图
  - 看法：红色虚线是 `0.5 cm/N` 要求线，柱子高于该线表示该方向满足灵敏度要求
- `mode1_force_displacement_ratio_timeseries.png`
  - 显示每个施力片段内的力、位移和瞬时位移/力比值
  - 看法：正弦力阶段曲线应随力变化平滑变化；停顿阶段不应出现异常尖峰
- `mode1_sensitivity_summary.csv`
  - 给出每个轴的拟合灵敏度、`R^2`、样本数、最大位移和平均力，适合写实验报告

## 7.6 使用说明：主动康复 Mode2

### 7.6.1 Mode2 要验证什么

`mode2` 对应临床中的柔顺轨迹跟踪/主动引导康复训练。它有一条预设圆轨迹：无外力时，UR5 应稳定跟踪圆轨迹；有外力时，导纳控制器允许末端偏离名义轨迹，但通过虚拟刚度和阻尼把末端柔顺地引导回轨迹。

本仓库推荐用 `ur5_pulse_wrench_publisher.py` 的 `mode2_cycle` 测试。每个循环包含 4 个阶段：

```bash
normal_positive -> tangent_forward -> normal_negative -> tangent_backward
```

含义：

- 法向力：把末端推离圆轨迹平面，用于观察控制器是否柔顺偏移并回到轨迹
- 切向力：沿圆轨迹方向推动，用于观察“患者主动为主、机械引导为辅”时是否阻力较小
- 正负两个方向：验证控制器对不同方向扰动都能稳定响应

### 7.6.2 启动 Mode2 四阶段外力测试

```bash
roslaunch ur5_gazebo_advance ur5_active_rehab_mode2.launch \
  start_pulse_wrench:=true \
  start_force_rviz:=true \
  wrench_pattern:=mode2_cycle \
  wrench_frame_id:=base_link \
  wrench_amplitude:=8.0 \
  wrench_on_duration:=2.0 \
  wrench_interval_duration:=2.0 \
  wrench_cycles:=2
```

该命令会同时启动：

- Gazebo
- RViz force marker 可视化
- 圆轨迹参考生成器
- 主动康复导纳控制器
- 四阶段外力发布器
- 轨迹误差监视器

Gazebo 中应能看到机械臂持续沿圆轨迹运动。RViz 中红色箭头会按 4 个阶段改变方向。外力阶段末端允许偏离名义圆轨迹；撤力停顿阶段，末端应平滑回到名义轨迹附近。

### 7.6.3 使用 timeout 记录 rosbag

先启动上面的 launch。等第一圈轨迹稳定后，在另一个终端记录。也可以用 `sleep` 自动跳过启动初期：

```bash
mkdir -p demo_bags
sleep 10
timeout 45s rosbag record -O demo_bags/mode2_force_cycle.bag \
  /ur5/ft_sensor/command_wrench \
  /ur5/ft_sensor/wrench_corrected \
  /ur5/end_effector_pose \
  /ur5/active_rehab/reference_pose \
  /ur5/active_rehab/nominal_reference_pose \
  /ur5/active_rehab/trajectory_reference \
  /joint_states
```

如果 `wrench_cycles:=0`，表示四阶段循环无限重复，录包时更应该使用 `timeout` 限制时长。

### 7.6.4 解析 rosbag 并绘图

```bash
mkdir -p demo_plots/mode2
rosrun ur5_gazebo_advance ur5_plot_active_rehab_bag.py \
  --mode mode2 \
  --bag demo_bags/mode2_force_cycle.bag \
  --output-dir demo_plots/mode2
```

主要输出：

- `mode2_command_force_xyz.png`
  - 分别显示 X/Y/Z 方向上的命令外力和修正后交互力，以及合力大小
  - 看法：命令外力应呈 4 阶段脉冲；修正力应能反映实际施加到末端的力，方向和大小不应明显错位
- `mode2_force_position_x.png`、`mode2_force_position_y.png`、`mode2_force_position_z.png`
  - 上半图：对应轴上的命令力和修正力
  - 下半图：实际末端位置、导纳参考位置、名义参考位置、预设轨迹位置
  - 看法：外力阶段实际末端允许偏离名义轨迹；停顿阶段应逐步回到名义轨迹和预设轨迹附近
- `mode2_yz_trajectory.png`
  - 从 YZ 平面看预设圆轨迹、导纳参考轨迹和实际末端轨迹
  - 看法：无外力时三条曲线应接近；外力阶段实际轨迹会向外偏移，但整体仍应围绕圆轨迹平滑运动
- `mode2_tracking_error.png`
  - 上半图显示交互力大小，下半图显示 `actual - nominal`、`actual - compliant_reference`、`actual - trajectory` 等误差
  - 看法：橙色阴影是命令外力 active segment；阴影内误差允许上升，阴影结束后误差应下降并恢复稳定
- `mode2_position_offset_xyz.png`
  - 分 X/Y/Z 显示相对名义轨迹、导纳参考和预设轨迹的偏移
  - 看法：用于判断偏移主要发生在哪个方向，以及撤力后各方向是否都能回到 0 附近
- `mode2_tracking_error_summary.csv`
  - 汇总平均/最大跟踪误差，适合用于判断无外力或撤力后的跟踪指标
- `mode2_pulse_response_summary.csv`
  - 按每个外力片段汇总峰值误差和撤力后恢复误差
  - 看法：峰值误差越大表示顺应越明显；撤力后恢复误差越小说明轨迹引导和稳定性越好

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
   roslaunch ur5_gazebo_advance ur5_active_rehab_mode1.launch \
     start_pulse_wrench:=true start_force_rviz:=true wrench_waveform:=sine
   ```
6. 运行主动康复 `mode2`：
   ```bash
   roslaunch ur5_gazebo_advance ur5_active_rehab_mode2.launch
   ```
