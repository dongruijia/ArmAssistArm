#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest
from std_msgs.msg import Header

class UR5MoveItFKNode:
    def __init__(self):
        rospy.init_node("ur5_moveit_fk")
        
        # 等待 MoveIt FK 服务
        rospy.loginfo("等待 MoveIt FK 服务...")
        rospy.wait_for_service("/compute_fk")
        self.fk_service = rospy.ServiceProxy("/compute_fk", GetPositionFK)
        rospy.loginfo("✅ MoveIt FK 服务已连接")

        # 发布官方正解位姿
        self.pub = rospy.Publisher("/ur5/moveit_end_effector_pose", PoseStamped, queue_size=10)
        self.sub = rospy.Subscriber("/joint_states", JointState, self.joint_cb)

        # UR5 配置
        self.ROOT_LINK = "base_link"
        self.TIP_LINK = "ee_link"
        self.JOINT_NAMES = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

    def joint_cb(self, msg):
        try:
            # 1. 按顺序提取关节角
            q = [msg.position[msg.name.index(j)] for j in self.JOINT_NAMES]

            # 2. 构造 FK 请求
            req = GetPositionFKRequest()
            req.header = Header()
            req.header.stamp = rospy.Time.now()
            req.header.frame_id = self.ROOT_LINK

            req.fk_link_names = [self.TIP_LINK]
            req.robot_state.joint_state.name = self.JOINT_NAMES
            req.robot_state.joint_state.position = q

            # 3. 调用 MoveIt 官方正解
            resp = self.fk_service(req)

            # 4. 发布结果
            if len(resp.pose_stamped) > 0:
                pose = resp.pose_stamped[0]
                pose.header.frame_id = self.ROOT_LINK
                self.pub.publish(pose)

        except Exception as e:
            rospy.logwarn(f"MoveIt FK 调用失败: {e}")

if __name__ == "__main__":
    try:
        node = UR5MoveItFKNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass