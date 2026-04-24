#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

# ====================== UR5 官方标准 DH 参数 ======================
# 单位：米(m)，弧度(rad)
# d: 沿 z 轴偏移
# a: 沿 x 轴偏移
# alpha: 绕 x 轴扭转
# 完全匹配 UR5 官方模型，保证精度
DH_PARAMS = [
    # d,      a,    alpha
    [0.089159, 0.0,  np.pi/2],   # joint 1
    [0.0,     -0.425, 0.0],       # joint 2
    [0.0,     -0.39225, 0.0],     # joint 3
    [0.10915, 0.0,  np.pi/2],    # joint 4
    [0.09465, 0.0, -np.pi/2],    # joint 5
    [0.0823,  0.0, 0.0],          # joint 6
]

# 关节顺序（必须和官方一致）
UR5_JOINT_ORDER = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
]

def dh_transform(d, a, alpha, theta):
    """
    标准 DH 变换矩阵
    输入：DH 参数 + 关节角
    输出：4x4 齐次变换矩阵
    """
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0.0, sa,     ca,     d],
        [0.0, 0.0,    0.0,    1.0]
    ])

def forward_kinematics(joint_positions):
    """
    正运动学：输入6个关节角 → 输出末端 4x4 位姿矩阵
    """
    T = np.eye(4)
    for i, q in enumerate(joint_positions):
        d, a, alpha = DH_PARAMS[i]
        T_i = dh_transform(d, a, alpha, q)
        T = np.dot(T, T_i)
    return T

def matrix_to_pose(T):
    """
    4x4 齐次矩阵 → ROS PoseStamped 消息
    自动计算四元数
    """
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "base_link"

    # 位置
    pose.pose.position.x = T[0, 3]
    pose.pose.position.y = T[1, 3]
    pose.pose.position.z = T[2, 3]

    # 旋转矩阵 → 四元数
    r11, r12, r13 = T[0, 0], T[0, 1], T[0, 2]
    r21, r22, r23 = T[1, 0], T[1, 1], T[1, 2]
    r31, r32, r33 = T[2, 0], T[2, 1], T[2, 2]

    qw = np.sqrt(1.0 + r11 + r22 + r33) / 2.0
    qx = (r32 - r23) / (4.0 * qw) if qw > 1e-6 else 0.0
    qy = (r13 - r31) / (4.0 * qw) if qw > 1e-6 else 0.0
    qz = (r21 - r12) / (4.0 * qw) if qw > 1e-6 else 0.0

    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw

    return pose

class UR5FKNode:
    def __init__(self):
        rospy.init_node("ur5_forward_kinematics")
        self.pub = rospy.Publisher("/ur5/end_effector_pose", PoseStamped, queue_size=10)
        self.sub = rospy.Subscriber("/joint_states", JointState, self.joint_cb)
        rospy.loginfo("✅ UR5 正运动学节点已启动，发布话题: /ur5/end_effector_pose")

    def joint_cb(self, msg):
        """接收关节状态 → 计算正解 → 发布位姿"""
        try:
            # 按官方顺序提取6个关节角
            q = [msg.position[msg.name.index(j)] for j in UR5_JOINT_ORDER]
            T = forward_kinematics(q)
            pose = matrix_to_pose(T)
            self.pub.publish(pose)
        except Exception as e:
            rospy.logwarn(f"FK 计算异常: {e}")

if __name__ == "__main__":
    try:
        node = UR5FKNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass