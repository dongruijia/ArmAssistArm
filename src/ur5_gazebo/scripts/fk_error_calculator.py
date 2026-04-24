#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped

class FKErrorCalculator:
    def __init__(self):
        rospy.init_node("fk_error_calculator")
        
        self.pose_custom = None   # 你的正解
        self.pose_moveit = None   # 官方正解
        
        # 订阅两个话题
        rospy.Subscriber("/ur5/end_effector_pose", PoseStamped, self.custom_pose_cb)
        rospy.Subscriber("/ur5/moveit_end_effector_pose", PoseStamped, self.moveit_pose_cb)
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("✅ 正运动学误差对比节点已启动！")
        rospy.loginfo("📌 位置误差单位：mm")
        rospy.loginfo("📌 姿态误差单位：°")
        rospy.loginfo("=" * 60)

    def custom_pose_cb(self, msg):
        self.pose_custom = msg.pose

    def moveit_pose_cb(self, msg):
        self.pose_moveit = msg.pose
        # 两个数据都收到才计算误差
        if self.pose_custom is not None and self.pose_moveit is not None:
            self.calculate_and_print_error()

    def calculate_and_print_error(self):
        # --------------------- 1. 计算位置误差（毫米） ---------------------
        dx = self.pose_custom.position.x - self.pose_moveit.position.x
        dy = self.pose_custom.position.y - self.pose_moveit.position.y
        dz = self.pose_custom.position.z - self.pose_moveit.position.z
        
        position_error_mm = np.sqrt(dx**2 + dy**2 + dz**2) * 1000.0

        # --------------------- 2. 计算姿态误差（角度） ---------------------
        qc = self.pose_custom.orientation
        qm = self.pose_moveit.orientation
        
        # 四元数点积
        dot_product = qc.x*qm.x + qc.y*qm.y + qc.z*qm.z + qc.w*qm.w
        dot_product = np.clip(dot_product, -1.0, 1.0)
        
        # 转换为角度误差
        angle_error_deg = np.degrees(2 * np.arccos(abs(dot_product)))

        # --------------------- 3. 打印结果（高亮绿色） ---------------------
        rospy.loginfo("\033[1;32m"
                      f"位置误差：{position_error_mm:>6.2f} mm   |   "
                      f"姿态误差：{angle_error_deg:>6.2f} °"
                      "\033[0m")

if __name__ == "__main__":
    try:
        node = FKErrorCalculator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass