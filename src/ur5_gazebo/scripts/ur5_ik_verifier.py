#!/usr/bin/env python3
"""验证逆运动学求解结果与实际关节状态的偏差。"""

import rospy
import numpy as np
from collections import deque
from sensor_msgs.msg import JointState

DEG2RAD = np.pi / 180.0
RAD2DEG = 180.0 / np.pi

class IKAccuracyVerifier:
    """比较 IK 输出与原始关节状态，检查解算精度。"""

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

        self.original_buffer = deque(maxlen=200)
        self.match_tolerance = rospy.get_param("~match_tolerance", 0.05)

        # 订阅话题
        rospy.Subscriber("/joint_states", JointState, self.original_cb)
        rospy.Subscriber("/ur5/ik_solved_joints", JointState, self.ik_cb)

        rospy.loginfo("✅ IK 精度验证节点 | 实时监测角度误差 ≤0.1°")
        rospy.loginfo("================================================")

    def original_cb(self, msg):
        try:
            joint_map = dict(zip(msg.name, msg.position))
            original_joints = np.array([joint_map[name] for name in self.joint_names])
            self.original_buffer.append((msg.header.stamp, original_joints))
        except Exception:
            return

    def ik_cb(self, msg):
        try:
            joint_map = dict(zip(msg.name, msg.position))
            ik_solved_joints = np.array([joint_map[name] for name in self.joint_names])
            self.compute_error(msg.header.stamp, ik_solved_joints)
        except Exception:
            return

    def _find_matching_original(self, ik_stamp):
        """在缓冲区中寻找时间上最接近的原始关节状态。"""

        if not self.original_buffer:
            return None

        best_joints = None
        best_idx = None
        best_dt = None

        for idx, (stamp, joints) in enumerate(self.original_buffer):
            dt = abs((stamp - ik_stamp).to_sec())
            if best_dt is None or dt < best_dt:
                best_dt = dt
                best_idx = idx
                best_joints = joints

        if best_dt is None or best_dt > self.match_tolerance:
            return None

        while len(self.original_buffer) > best_idx + 1:
            self.original_buffer.popleft()

        self.original_buffer.popleft()
        return best_joints

    def compute_error(self, ik_stamp, ik_solved_joints):
        """计算角度误差并给出是否达标的提示。"""

        original_joints = self._find_matching_original(ik_stamp)
        if original_joints is None:
            rospy.logwarn_throttle(1.0, "[监测] 等待匹配的关节数据...")
            return

        err_rad = np.abs(original_joints - ik_solved_joints)
        err_deg = err_rad * RAD2DEG
        max_err = np.max(err_deg)
        mean_err = np.mean(err_deg)

        # 实时打印结果
        rospy.loginfo(f"📊 最大误差={max_err:.3f}° | 平均误差={mean_err:.3f}°")
        if max_err <= 0.1:
            rospy.loginfo(f"✅ 精度合格：角度误差 ≤ 0.1°")
        else:
            rospy.logwarn(f"⚠️ 精度不合格")
        
        rospy.loginfo("================================================")

if __name__ == "__main__":
    try:
        node = IKAccuracyVerifier()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass