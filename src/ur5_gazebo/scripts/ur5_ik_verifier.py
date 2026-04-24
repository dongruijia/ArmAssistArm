#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import matplotlib.animation as animation

DEG2RAD = np.pi / 180.0
RAD2DEG = 180.0 / np.pi

class IKAccuracyVerifier:
    def __init__(self):
        rospy.init_node("ik_accuracy_verifier", anonymous=True)

        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

        self.original_joints = None
        self.ik_solved_joints = None

        # 绘图缓存
        self.max_err_history = []
        self.limit_line = [0.1] * 50  # 0.1° 红线
        self.max_points = 50

        # 订阅话题
        rospy.Subscriber("/joint_states", JointState, self.original_cb)
        rospy.Subscriber("/ur5/ik_solved_joints", JointState, self.ik_cb)

        # 初始化画布
        self.fig, self.ax = plt.subplots(figsize=(10, 4))
        self.ax.set_ylim(0, 0.3)
        self.ax.set_title("IK 角度误差实时波形 (最大误差)")
        self.ax.set_ylabel("误差 (°)")
        self.ax.set_xlabel("时间点")
        self.ax.grid(True)

        # 画 0.1° 合格线
        self.line_limit, = self.ax.plot(range(50), self.limit_line, 'r--', label="0.1° 合格线", linewidth=2)
        self.line_err, = self.ax.plot([], [], 'b-', label="实时最大误差", linewidth=2)
        self.ax.legend()

        rospy.loginfo("✅ IK 精度验证节点（带实时波形图）已启动")

    def original_cb(self, msg):
        try:
            joint_map = dict(zip(msg.name, msg.position))
            self.original_joints = np.array([joint_map[name] for name in self.joint_names])
            self.compute_error()
        except:
            return

    def ik_cb(self, msg):
        try:
            joint_map = dict(zip(msg.name, msg.position))
            self.ik_solved_joints = np.array([joint_map[name] for name in self.joint_names])
            self.compute_error()
        except:
            return

    def compute_error(self):
        if self.original_joints is None or self.ik_solved_joints is None:
            return

        err_rad = np.abs(self.original_joints - self.ik_solved_joints)
        err_deg = err_rad * RAD2DEG
        max_err = np.max(err_deg)
        mean_err = np.mean(err_deg)

        # 保存历史，画波形
        self.max_err_history.append(max_err)
        if len(self.max_err_history) > self.max_points:
            self.max_err_history.pop(0)

        # 打印
        rospy.loginfo(f"📊 最大误差={max_err:.3f}° | 平均={mean_err:.3f}°")
        if max_err <= 0.1:
            rospy.loginfo(f"✅ 精度合格 ≤0.1°")
        else:
            rospy.logwarn(f"⚠️ 精度不合格")

    def update_plot(self, frame):
        x = list(range(len(self.max_err_history)))
        y = self.max_err_history
        self.line_err.set_data(x, y)
        return self.line_err, self.line_limit

    def start_plot(self):
        ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=200, blit=True, cache_frame_data=False
        )
        plt.show()

if __name__ == "__main__":
    node = IKAccuracyVerifier()
    node.start_plot()