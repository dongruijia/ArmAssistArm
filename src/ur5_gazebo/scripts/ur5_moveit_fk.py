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
        
        rospy.loginfo("等待 MoveIt FK 服务...")
        rospy.wait_for_service("/compute_fk")
        self.fk_service = rospy.ServiceProxy("/compute_fk", GetPositionFK)
        rospy.loginfo("✅ MoveIt FK 服务已连接")

        self.pub = rospy.Publisher("/ur5/moveit_end_effector_pose", PoseStamped, queue_size=10)
        self.sub = rospy.Subscriber("/joint_states", JointState, self.joint_cb)

        self.ROOT_LINK = "base_link"
        self.TIP_LINK = "wrist_3_link"  # 这里改了！ee_link 不存在

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
            q = [msg.position[msg.name.index(j)] for j in self.JOINT_NAMES]

            req = GetPositionFKRequest()
            req.header.frame_id = self.ROOT_LINK

            # ====================== 修复 2：使用消息时间戳，适配仿真 ======================
            req.header.stamp = msg.header.stamp  

            req.fk_link_names = [self.TIP_LINK]
            req.robot_state.joint_state.name = self.JOINT_NAMES
            req.robot_state.joint_state.position = q

            resp = self.fk_service(req)

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