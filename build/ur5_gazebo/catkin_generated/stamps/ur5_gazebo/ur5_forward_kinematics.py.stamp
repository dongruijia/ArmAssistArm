#!/usr/bin/env python3
"""根据关节状态实时计算并发布 UR5 末端位姿。"""

import rospy
import numpy as np
import tf.transformations as tf_t
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

class UR5ForwardKinematicsNode:
    """基于 DH 参数计算 UR5 当前末端位姿。"""

    def __init__(self):
        rospy.init_node("ur5_fk_solver", anonymous=True)

        # 1. 经过校准的 UR5 标准 DH 参数
        self.dh_params = [
            {'alpha':  np.pi/2, 'a':  0,        'd':  0.089159},
            {'alpha':  0,       'a': -0.425,    'd':  0},
            {'alpha':  0,       'a': -0.39225,  'd':  0},
            {'alpha':  np.pi/2, 'a':  0,        'd':  0.10915},
            {'alpha': -np.pi/2, 'a':  0,        'd':  0.09465},
            {'alpha':  0,       'a':  0,        'd':  0.0823}
        ]

        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

        # Align DH frame with URDF/MoveIt frames.
        # For most UR5 descriptions: base_link -> DH base is a yaw = pi rotation.
        self.base_yaw_correction = rospy.get_param("~base_yaw_correction", np.pi)
        # For wrist_3_link comparison this should usually be 0.0.
        self.tool_yaw_correction = rospy.get_param("~tool_yaw_correction", 0.0)

        self.pose_pub = rospy.Publisher("/ur5/end_effector_pose", PoseStamped, queue_size=10)
        self.joint_sub = rospy.Subscriber("/joint_states", JointState, self.joint_callback)

        rospy.loginfo("UR5 FK node started")
        rospy.loginfo("base_yaw_correction=%.6f, tool_yaw_correction=%.6f",
                      self.base_yaw_correction, self.tool_yaw_correction)

    def get_transform(self, alpha, a, d, q):
        """Compute one standard DH transform: Rz(q) * Tz(d) * Tx(a) * Rx(alpha)."""
        T_rz = tf_t.rotation_matrix(q, (0, 0, 1))
        T_tz = tf_t.translation_matrix((0, 0, d))
        T_tx = tf_t.translation_matrix((a, 0, 0))
        T_rx = tf_t.rotation_matrix(alpha, (1, 0, 0))
        
        return tf_t.concatenate_matrices(T_rz, T_tz, T_tx, T_rx)

    def joint_callback(self, msg):
        """将 joint_states 按固定关节顺序重排后执行正运动学。"""

        try:
            # 确保消息包含所有需要的关节
            if not all(name in msg.name for name in self.joint_names):
                return

            joint_map = dict(zip(msg.name, msg.position))
            q_values = [joint_map[name] for name in self.joint_names]

            # 注意：这里的固定旋转需要与 IK 节点保持一致，否则两者参考系会错位。
            T_base = tf_t.rotation_matrix(self.base_yaw_correction, (0, 0, 1))
            T_total = T_base

            for i in range(6):
                p = self.dh_params[i]
                T_i = self.get_transform(p['alpha'], p['a'], p['d'], q_values[i])
                T_total = np.dot(T_total, T_i)

            # Optional fixed correction from DH tip frame to selected URDF tip frame.
            if abs(self.tool_yaw_correction) > 1e-12:
                T_tool = tf_t.rotation_matrix(self.tool_yaw_correction, (0, 0, 1))
                T_total = np.dot(T_total, T_tool)

            self.publish_pose(T_total)

        except Exception as e:
            rospy.logerr("FK failed: %s", str(e))

    def publish_pose(self, T):
        """将齐次变换矩阵转换为 PoseStamped 发布。"""

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"

        pos = tf_t.translation_from_matrix(T)
        msg.pose.position.x = pos[0]
        msg.pose.position.y = pos[1]
        msg.pose.position.z = pos[2]

        quat = tf_t.quaternion_from_matrix(T)
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]

        self.pose_pub.publish(msg)

if __name__ == "__main__":
    try:
        node = UR5ForwardKinematicsNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass