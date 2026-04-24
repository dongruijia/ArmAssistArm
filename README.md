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

## 4.1 一次跑完三种轨迹

```bash
roslaunch ur5_gazebo ur5_gazebo.launch trajectory_mode:=all loop:=false
```

## 4.2 只跑某一种轨迹

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

## 7. 关键话题链路速览

## 7.1 康复轨迹链路

- 目标笛卡尔位姿：`/ur5/rehab_target_pose` 或 `/ur5/stability_target_pose`
- IK 结果关节角：`/ur5/rehab_ik_solved_joints` 或 `/ur5/stability_ik_solved_joints`
- 控制命令：`/ur5_arm_controller/command`
- 实际关节状态：`/joint_states`

## 7.2 稳定性误差评估链路

- 参考位姿：`/ur5/stability_reference_pose`
- FK 实际位姿：`/ur5/stability_actual_pose`
- 误差结果：终端日志输出平均/最大误差

## 7.3 导纳控制链路

- 原始力：`/ur5/ft_sensor/wrench`
- 修正力：`/ur5/ft_sensor/wrench_corrected`
- 外力命令：`/ur5/ft_sensor/command_wrench`
- 导纳参考位姿：`/ur5/admittance/reference_pose`
- 关节命令：`/ur5_arm_controller/command`

---

## 8. 常见问题与排查

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

## 9. 推荐最小复现实验顺序

1. 编译并 source
2. 跑三轨迹：
   ```bash
   roslaunch ur5_gazebo ur5_gazebo.launch trajectory_mode:=all
   ```
3. 跑稳定性验证：
   ```bash
   roslaunch ur5_gazebo ur5_rehab_stability_validation.launch
   ```
4. 跑导纳控制：
   ```bash
   roslaunch ur5_gazebo_advance ur5_admittance_control.launch
   ```

---

如果你希望，我还可以继续补一份“英文版 README”或“论文实验复现实验脚本（bash 一键跑三组实验）”。
