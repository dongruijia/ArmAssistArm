# UR5 Rehab Workspace 使用说明（精简版）

面向老师验收，按下面命令可直接运行并检查指标。

## 1. 环境准备

```bash
cd ~/ur5_rehab_ws
catkin_make
source devel/setup.bash
```

每个新终端建议先执行：

```bash
cd ~/ur5_rehab_ws
source devel/setup.bash
```

## 2. 3.2.2 正逆运动学验收

### 2.1 正运动学（自定义 FK vs MoveIt FK）

终端 A：
```bash
roslaunch ur_gazebo ur5_bringup.launch \
  transmission_hw_interface:=hardware_interface/PositionJointInterface \
  controller_config_file:=$(rospack find ur5_gazebo)/config/ur5_controllers.yaml \
  controllers:="joint_state_controller ur5_arm_controller"
```

终端 B：
```bash
roslaunch ur5_moveit_config moveit_planning_execution.launch sim:=true
```

终端 C：
```bash
rosrun ur5_gazebo ur5_forward_kinematics.py
```

终端 D：
```bash
rosrun ur5_gazebo ur5_moveit_fk.py
```

终端 E：
```bash
rosrun ur5_gazebo fk_error_calculator.py
```

验收指标：
- 位置误差 <= 1 mm
- 姿态误差 <= 0.5 deg

### 2.2 逆运动学（牛顿-拉夫逊）

终端 F：
```bash
rosrun ur5_gazebo ur5_ik_solver.py _target_topic:=/ur5/end_effector_pose
```

终端 G：
```bash
rosrun ur5_gazebo ur5_ik_verifier.py _match_tolerance:=0.05
```

验收指标：
- 角度误差 <= 0.1 deg
- 无奇异位形导致的报错/崩溃

## 3. 3.2.3 康复轨迹规划与跟踪

三种轨迹一键运行：

```bash
roslaunch ur5_gazebo ur5_gazebo.launch trajectory_mode:=all loop:=false sample_dt:=0.01
```

单独运行：

```bash
roslaunch ur5_gazebo ur5_gazebo.launch trajectory_mode:=line
roslaunch ur5_gazebo ur5_gazebo.launch trajectory_mode:=arc
roslaunch ur5_gazebo ur5_gazebo.launch trajectory_mode:=circle
```

对应关系：
- line：前伸直线（5s）
- arc：肘关节屈伸圆弧（6s）
- circle：肩关节环转画圆（8s）

## 4. 4.2.1 六维力传感器验收

URDF 插件位置：
- src/universal_robot/ur_gazebo/urdf/ur_macro.xacro

检查命令：

```bash
rostopic hz /ur5/ft_sensor/wrench
rostopic echo -n 5 /ur5/ft_sensor/wrench
```

关键配置：
- updateRate = 1000
- topic = /ur5/ft_sensor/wrench
- plugin = libgazebo_ros_ft_sensor.so

## 5. 4.2.2 基础导纳控制器验收

启动导纳控制：

```bash
roslaunch ur5_gazebo_advance ur5_admittance_control.launch \
  control_rate_hz:=100.0 \
  force_lpf_cutoff_hz:=10.0 \
  md_diag:='[1.0, 1.0, 1.0]' \
  bd_diag:='[50.0, 50.0, 50.0]' \
  kd_diag:='[200.0, 200.0, 200.0]'
```

稳定性验证（直线轨迹）：

```bash
roslaunch ur5_gazebo_advance ur5_rehab_stability_validation.launch
```

查看日志关键字：
- UR5 end-effector tracking mean error
- UR5 end-effector tracking max  error

验收指标：
- 无外力时稳定无震荡
- 平均误差 <= 3 mm

## 6. 4.2.3 两种主动康复模式验收

### 6.1 模式1（零力跟随）

```bash
roslaunch ur5_gazebo_advance ur5_active_rehab_mode1.launch \
  start_pulse_wrench:=true \
  start_force_rviz:=true \
  wrench_waveform:=sine \
  wrench_axis:=x \
  wrench_amplitude:=1.0 \
  wrench_frequency:=0.5
```

指标：
- 跟随无明显滞后/卡顿
- 撤力后稳定
- 灵敏度 >= 0.5 cm/N

### 6.2 模式2（柔顺轨迹跟踪）

```bash
roslaunch ur5_gazebo_advance ur5_active_rehab_mode2.launch \
  start_pulse_wrench:=true \
  start_force_rviz:=true \
  wrench_pattern:=mode2_cycle \
  wrench_amplitude:=8.0 \
  wrench_on_duration:=2.0 \
  wrench_interval_duration:=2.0
```

指标：
- 无外力时稳定跟踪圆轨迹
- 受外力时平滑修正，无硬冲击
- 无外力阶段平均误差 <= 5 mm

## 7. 3.3 / 4.3 可视化与量化

```bash
rviz
rqt_plot
```

常用曲线：

```bash
rqt_plot /ur5/ft_sensor/wrench/wrench/force/x /ur5/ft_sensor/wrench_corrected/wrench/force/x
rqt_plot /ur5/admittance/debug/raw_force_norm /ur5/admittance/debug/filtered_force_norm
rqt_plot /ur5/admittance/debug/delta_pos_norm
```

离线绘图：

```bash
rosrun ur5_gazebo_advance ur5_plot_active_rehab_bag.py --help
```

## 8. 最小复现实验顺序

1. 三轨迹
```bash
roslaunch ur5_gazebo ur5_gazebo.launch trajectory_mode:=all
```

2. 稳定性验证
```bash
roslaunch ur5_gazebo_advance ur5_rehab_stability_validation.launch
```

3. 基础导纳
```bash
roslaunch ur5_gazebo_advance ur5_admittance_control.launch
```

4. 主动模式1
```bash
roslaunch ur5_gazebo_advance ur5_active_rehab_mode1.launch start_pulse_wrench:=true
```

5. 主动模式2
```bash
roslaunch ur5_gazebo_advance ur5_active_rehab_mode2.launch
```
