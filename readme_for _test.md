# 欢迎来到我们的ArmAssistArm代码仓库！

---

首先，先来介绍一下我们的代码包：打开工作空间后，你会看到6个ROS包：
- ros_control
- ros_controllers
- universal_robot
- ur5_gazebo
- ur5_gazebo_advance
- ur5_kinematics

我们的工作主要存放在后3个包：
- ur5_gazebo
（python包【编写者：贾东睿（正逆解节点、基础任务的python实现、传感器安装）】）
- ur5_gazebo_advance
（python包【编写者：贾东睿（导纳控制实现）/周昭宏（两种康复模式实现）】）
- ur5_kinematics
（cpp包【编写者：罗斯民（正逆解节点、基础任务的cpp实现）】）

因此，我们组同时完成了基础任务的python实现和cpp实现，但是由【贾东睿】负责的基础部分主要服务于后续的导纳控制，但是精度要求均满足（可视化有所欠缺），由【罗斯民】负责的cpp版本的基础任务的实现则专门服务于基础部分的考核，可视化较好。

---

接下来，我们会针对测试的指标给出相应的命令行：

## 注意：如果出现文件权限不足，请先在工作空间编译！

- 如果你想查看基础任务的python实现，请运行：
```bash
roslaunch ur5_gazebo ur5_gazebo.launch
```
> 机械臂的位姿切换需要时间，请耐心等待；运动位姿精度可以从bash中查看滚动消息

- 如果你想查看附加任务中的恒力模式，且想要查看六维传感器的测量精度，请运行：
```bash
roslaunch ur5_gazebo_advance ur5_admittance_control.launch   wrench_mode:=constant   wrench_axis:=x   wrench_constant_value:=8.0
```
如果你想更清楚的观测数据，请安装：
```bash
sudo apt update
sudo apt install ros-noetic-plotjuggler-ros
```
然后操作如下：
首先点击页面中的“start”
![alt text](./img/image.png)

选中这两项：
![alt text](./img/image-1.png)

在这个窗口中把想要观测的数据拖动到右侧大窗口
![alt text](./img/image-2.png)

- 如果你想查看我们的导纳控制在正弦力下的表现，请运行：
```bash
 roslaunch ur5_gazebo_advance ur5_admittance_control.launch
```
如果你想查看详细的数据，方法同上。

- 如果你想查看导纳控制在0力模式下的直线跟踪，请运行：
```bash
 roslaunch ur5_gazebo_advance ur5_rehab_stability_validation.launch

```
> 命令行窗口会在机械臂停止时，打印轨迹的跟踪误差

---

- 如果你想查看我们的两种主动康复柔顺控制模式（mode1 / mode2），请继续阅读：

> mode1（零力跟随模式）和 mode2（柔顺轨迹跟踪模式）由【周昭宏】负责实现，涉及 `ur5_active_rehab_common.launch`、`ur5_active_rehab_mode1.launch`、`ur5_active_rehab_mode2.launch` 以及配套的 Python 脚本和参数配置文件。

### 模式1：零力跟随（对应临床减重康复训练）

参数设定：调低刚度与阻尼参数（Kd=diag(20,20,20)，Bd=diag(10,10,10)），无预设参考轨迹，参考位姿初始值为机械臂当前末端位姿。机械臂根据末端交互力实时修正参考位姿，模拟患者主动施力时的减重跟随效果。验证指标：跟随灵敏度 >= 0.5 cm/N，撤力后稳定无震荡。

启动命令（推荐使用正弦脉冲力测试）：
```bash
roslaunch ur5_gazebo_advance ur5_active_rehab_mode1.launch \
  start_pulse_wrench:=true \
  start_force_rviz:=true \
  wrench_axis:=x \
  wrench_waveform:=sine \
  wrench_amplitude:=1.0 \
  wrench_frequency:=0.5
```

常用参数说明：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `start_pulse_wrench` | false | 是否启动脉冲力发布器，设为 true 会自动施加周期性外力 |
| `start_force_rviz` | false | 是否启动 RViz 力可视化窗口 |
| `wrench_axis` | x | 施力方向轴，可选 x / y / z |
| `wrench_frame_id` | base_link | 力的参考坐标系 |
| `wrench_waveform` | square | 波形，可选 sine（正弦）或 square（方波） |
| `wrench_amplitude` | 1.0 | 力幅值，单位 N |
| `wrench_frequency` | 0.5 | 力频率，单位 Hz |
| `wrench_periods_per_burst` | 2 | 每段脉冲包含的周期数 |
| `wrench_start_delay` | 5.0 | 启动后延时开始施力，单位 s |
| `wrench_on_duration` | 2.0 | 每段施力时长，单位 s（方波模式使用） |
| `wrench_off_duration` | 2.0 | 每段停顿时长，单位 s（方波模式使用） |
| `wrench_interval_duration` | 同 off_duration | 脉冲段之间的间隔时长 |
| `wrench_cycles` | 3 | 脉冲段数，设为 0 则不限制（**推荐**） |

导纳参数可通过 `config/mode1_params.yaml` 调整（Md / Bd / Kd、force_deadband、max_correction 等）。

### 模式2：柔顺轨迹跟踪（对应临床主动引导康复训练）

参数设定：采用固定导纳参数（Kd=diag(200,200,200)，Bd=diag(50,50,50)），以肩关节环转画圆轨迹为预设参考。无外力时稳定跟踪圆轨迹；受外力时平滑修正，撤力后回到轨迹附近。验证指标：无外力时平均跟踪误差 <= 5 mm。

启动命令（推荐使用四阶段 cycle 测试）：
```bash
roslaunch ur5_gazebo_advance ur5_active_rehab_mode2.launch \
  start_pulse_wrench:=true \
  start_force_rviz:=true \
  wrench_pattern:=mode2_cycle \
  wrench_amplitude:=8.0 \
  wrench_on_duration:=2.0 \
  wrench_interval_duration:=2.0
```

四阶段 cycle 含义：法向正力 → 切向正力 → 法向负力 → 切向负力，分别验证法向偏移引导和切向主动推动效果。

常用参数说明：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `start_pulse_wrench` | true | 是否启动脉冲力发布器 |
| `start_force_rviz` | true | 是否启动 RViz 力可视化 |
| `start_error_monitor` | true | 是否启动轨迹误差监视节点 |
| `wrench_pattern` | single_axis | 脉冲模式，mode2 测试请设为 `mode2_cycle` |
| `wrench_axis` | x | 施力方向轴（single_axis 模式使用） |
| `wrench_amplitude` | 8.0 | 力幅值，单位 N |
| `wrench_start_delay` | 4.0 | 启动后延时开始施力，单位 s |
| `wrench_on_duration` | 2.0 | 每阶段施力时长，单位 s |
| `wrench_interval_duration` | 同 off_duration | 阶段间停顿时长，单位 s |
| `wrench_cycles` | 0 | 阶段循环次数，默认 0 即不限制（无需修改） |
| `wrench_circle_center` | [0.0, 0.5, 0.5] | 圆轨迹圆心位置 |
| `wrench_circle_plane` | yz | 圆轨迹平面，可选 xy / yz / xz |

轨迹参数可通过 `config/mode2_trajectory.yaml` 调整（圆心、半径、周期等），导纳参数通过 `config/mode2_controller_params.yaml` 调整。

### 录制 rosbag 并绘图分析

启动上述任一模式后，在另一个终端录制 bag：

**mode1 录包：**
```bash
mkdir -p demo_bags
sleep 8
timeout 60s rosbag record -O demo_bags/mode1_test.bag \
  /ur5/ft_sensor/command_wrench \
  /ur5/ft_sensor/wrench_corrected \
  /ur5/end_effector_pose \
  /ur5/active_rehab/reference_pose \
  /ur5/active_rehab/nominal_reference_pose \
  /joint_states
```

**mode2 录包：**
```bash
mkdir -p demo_bags
sleep 10
timeout 60s rosbag record -O demo_bags/mode2_test.bag \
  /ur5/ft_sensor/command_wrench \
  /ur5/ft_sensor/wrench_corrected \
  /ur5/end_effector_pose \
  /ur5/active_rehab/reference_pose \
  /ur5/active_rehab/nominal_reference_pose \
  /ur5/active_rehab/trajectory_reference \
  /joint_states
```

> `sleep` 跳过启动初期不稳定阶段，`timeout 60s` 控制录制时长，避免 bag 文件过大。

录制完成后，用绘图脚本解析并生成验证图片和 CSV 汇总：

**mode1 绘图：**
```bash
mkdir -p demo_plots/mode1
rosrun ur5_gazebo_advance ur5_plot_active_rehab_bag.py \
  --mode mode1 \
  --bag demo_bags/mode1_test.bag \
  --output-dir demo_plots/mode1
```

输出包括：
- `mode1_force_position_x/y/z.png` — 各轴力-位置响应曲线
- `mode1_force_displacement_sensitivity.png` — 力-位移灵敏度拟合图（指标：>= 0.5 cm/N）
- `mode1_sensitivity_summary.png` — 三轴灵敏度柱状汇总
- `mode1_sensitivity_summary.csv` — 数值汇总，适合写报告

**mode2 绘图：**
```bash
mkdir -p demo_plots/mode2
rosrun ur5_gazebo_advance ur5_plot_active_rehab_bag.py \
  --mode mode2 \
  --bag demo_bags/mode2_test.bag \
  --output-dir demo_plots/mode2
```

输出包括：
- `mode2_command_force_xyz.png` — 命令力与修正力对比
- `mode2_force_position_x/y/z.png` — 各轴力-位置响应
- `mode2_yz_trajectory.png` — YZ 平面圆轨迹对比
- `mode2_tracking_error.png` — 跟踪误差时间序列
- `mode2_tracking_error_summary.csv` — 平均/最大误差汇总
- `mode2_pulse_response_summary.csv` — 各阶段峰值误差与恢复误差