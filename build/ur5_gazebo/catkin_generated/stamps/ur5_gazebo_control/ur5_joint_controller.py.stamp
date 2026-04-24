#!/usr/bin/env python3
import math

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def make_point(positions, seconds):
    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start = rospy.Duration.from_sec(seconds)
    return point


def publish_demo_trajectory(pub):
    traj = JointTrajectory()
    traj.joint_names = JOINT_NAMES
    traj.header.stamp = rospy.Time.now() + rospy.Duration(0.2)

    # 选取一组安全且明显的动作点，机械臂会在两端姿态之间循环。
    pose_a = [0.0, -1.3, 1.4, -1.6, -1.57, 0.0]
    pose_b = [0.8, -1.0, 1.1, -1.8, -1.57, 0.8]
    pose_c = [-0.8, -1.0, 1.1, -1.8, -1.57, -0.8]

    traj.points = [
        make_point(pose_a, 2.0),
        make_point(pose_b, 5.0),
        make_point(pose_c, 8.0),
        make_point(pose_a, 11.0),
    ]

    pub.publish(traj)


if __name__ == "__main__":
    rospy.init_node("ur5_demo_motion")
    cmd_topic = rospy.get_param("~command_topic", "/ur5_arm_controller/command")
    cycle_sec = rospy.get_param("~cycle_sec", 12.0)

    publisher = rospy.Publisher(cmd_topic, JointTrajectory, queue_size=1)

    # 等待控制器订阅命令话题，避免第一条轨迹丢失。
    timeout_t = rospy.Time.now() + rospy.Duration(10.0)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and publisher.get_num_connections() == 0:
        if rospy.Time.now() > timeout_t:
            rospy.logwarn("No subscriber on %s yet, continue anyway.", cmd_topic)
            break
        rate.sleep()

    rospy.loginfo("UR5 demo motion node started, publishing to %s", cmd_topic)

    loop_rate = rospy.Rate(1.0 / max(cycle_sec, 1.0))
    while not rospy.is_shutdown():
        publish_demo_trajectory(publisher)
        loop_rate.sleep()