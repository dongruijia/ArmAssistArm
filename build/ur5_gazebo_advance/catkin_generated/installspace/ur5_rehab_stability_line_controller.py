#!/usr/bin/env python3
"""生成稳定性测试直线轨迹并驱动机械臂执行。"""

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class UR5RehabStabilityLineController:
    """生成固定直线参考轨迹，并同步请求 IK 后下发执行。"""

    def __init__(self):
        rospy.init_node("ur5_rehab_stability_line_controller")

        self.command_topic = rospy.get_param("~command_topic", "/ur5_arm_controller/command")
        self.target_topic = rospy.get_param("~target_topic", "/ur5/stability_target_pose")
        self.reference_topic = rospy.get_param("~reference_topic", "/ur5/stability_reference_pose")
        self.ik_output_topic = rospy.get_param("~ik_output_topic", "/ur5/stability_ik_solved_joints")

        self.dt = float(rospy.get_param("~sample_dt", 0.01))
        self.start_delay = float(rospy.get_param("~start_delay", 0.5))
        self.ik_timeout = float(rospy.get_param("~ik_timeout", 0.5))

        self.duration = 5.0
        self.start_point = np.array([0.4, 0.0, 0.6], dtype=float)
        self.end_point = np.array([0.6, 0.2, 0.5], dtype=float)

        self.orientation_quat = rospy.get_param("~orientation_quat", [0.0, 1.0, 0.0, 0.0])
        self.orientation_quat = np.array(self.orientation_quat, dtype=float)
        if self.orientation_quat.shape != (4,):
            raise ValueError("~orientation_quat must have 4 elements")
        norm = np.linalg.norm(self.orientation_quat)
        if norm < 1e-12:
            raise ValueError("~orientation_quat norm is zero")
        self.orientation_quat /= norm

        self.current_joints = np.zeros(6)
        self.has_joint_state = False
        self.latest_ik_solution = None
        self.latest_ik_stamp = rospy.Time(0)

        self.command_pub = rospy.Publisher(self.command_topic, JointTrajectory, queue_size=1)
        self.target_pub = rospy.Publisher(self.target_topic, PoseStamped, queue_size=1)
        self.reference_pub = rospy.Publisher(self.reference_topic, PoseStamped, queue_size=1)

        rospy.Subscriber("/joint_states", JointState, self.joint_state_cb)
        rospy.Subscriber(self.ik_output_topic, JointState, self.ik_result_cb)

    def joint_state_cb(self, msg):
        try:
            joint_map = dict(zip(msg.name, msg.position))
            self.current_joints = np.array([joint_map[name] for name in JOINT_NAMES], dtype=float)
            self.has_joint_state = True
        except Exception:
            return

    def ik_result_cb(self, msg):
        try:
            joint_map = dict(zip(msg.name, msg.position))
            self.latest_ik_solution = np.array([joint_map[name] for name in JOINT_NAMES], dtype=float)
            self.latest_ik_stamp = msg.header.stamp
        except Exception:
            return

    def wait_for_joint_state(self):
        timeout_t = rospy.Time.now() + rospy.Duration(10.0)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and not self.has_joint_state:
            if rospy.Time.now() > timeout_t:
                raise RuntimeError("Timeout waiting for /joint_states")
            rate.sleep()

    def wait_for_controller_connection(self):
        timeout_t = rospy.Time.now() + rospy.Duration(10.0)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and self.command_pub.get_num_connections() == 0:
            if rospy.Time.now() > timeout_t:
                rospy.logwarn("No subscriber on %s yet, continue anyway.", self.command_topic)
                return
            rate.sleep()

    def wait_for_ik_connection(self):
        timeout_t = rospy.Time.now() + rospy.Duration(10.0)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and self.target_pub.get_num_connections() == 0:
            if rospy.Time.now() > timeout_t:
                raise RuntimeError("Timeout waiting for IK subscriber on %s" % self.target_topic)
            rate.sleep()

    def quintic_scalars(self, total_time):
        point_count = int(round(total_time / self.dt))
        times = np.linspace(0.0, total_time, point_count + 1)
        tau = np.clip(times / total_time, 0.0, 1.0)
        scalars = 10.0 * tau ** 3 - 15.0 * tau ** 4 + 6.0 * tau ** 5
        return times, scalars

    def build_line_trajectory(self):
        """根据五次时间缩放生成笛卡尔直线轨迹。"""

        times, scalars = self.quintic_scalars(self.duration)
        points = self.start_point[None, :] + scalars[:, None] * (self.end_point - self.start_point)[None, :]
        return times, points

    def request_ik_solution(self, point, orientation):
        """发布单个目标位姿并等待对应时间戳的 IK 返回。"""

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = float(point[0])
        pose.pose.position.y = float(point[1])
        pose.pose.position.z = float(point[2])
        pose.pose.orientation.x = float(orientation[0])
        pose.pose.orientation.y = float(orientation[1])
        pose.pose.orientation.z = float(orientation[2])
        pose.pose.orientation.w = float(orientation[3])

        # 注意：这里依赖时间戳一一对应，请保持 IK 节点透传消息时间戳。
        expected_stamp = pose.header.stamp
        self.target_pub.publish(pose)

        deadline = rospy.Time.now() + rospy.Duration.from_sec(self.ik_timeout)
        rate = rospy.Rate(500)
        while not rospy.is_shutdown():
            if self.latest_ik_solution is not None and self.latest_ik_stamp == expected_stamp:
                return self.latest_ik_solution.copy()
            if rospy.Time.now() > deadline:
                raise RuntimeError("Timeout waiting IK response for pose stamp=%s" % str(expected_stamp.to_sec()))
            rate.sleep()

    def solve_joint_trajectory(self, times, cart_points):
        joint_points = []

        for index, (time_sec, point) in enumerate(zip(times, cart_points)):
            q_sol = self.request_ik_solution(point, self.orientation_quat)

            joint_point = JointTrajectoryPoint()
            joint_point.positions = q_sol.tolist()
            joint_point.time_from_start = rospy.Duration.from_sec(float(time_sec))
            joint_points.append(joint_point)

            if index in (0, len(times) - 1) or (index + 1) % 100 == 0:
                rospy.loginfo("[stability_line] IK solved %d/%d", index + 1, len(times))

        return joint_points

    def publish_reference_in_real_time(self, exec_start, times, cart_points):
        """按计划执行时刻实时发布参考位姿，供误差监测节点对齐。"""

        for time_sec, point in zip(times, cart_points):
            target_stamp = exec_start + rospy.Duration.from_sec(float(time_sec))

            while not rospy.is_shutdown() and rospy.Time.now() < target_stamp:
                rospy.sleep(0.0005)

            ref = PoseStamped()
            ref.header.stamp = target_stamp
            ref.header.frame_id = "base_link"
            ref.pose.position.x = float(point[0])
            ref.pose.position.y = float(point[1])
            ref.pose.position.z = float(point[2])
            ref.pose.orientation.x = float(self.orientation_quat[0])
            ref.pose.orientation.y = float(self.orientation_quat[1])
            ref.pose.orientation.z = float(self.orientation_quat[2])
            ref.pose.orientation.w = float(self.orientation_quat[3])
            self.reference_pub.publish(ref)

    def run(self):
        self.wait_for_joint_state()
        self.wait_for_controller_connection()
        self.wait_for_ik_connection()

        times, cart_points = self.build_line_trajectory()
        joint_points = self.solve_joint_trajectory(times, cart_points)

        trajectory = JointTrajectory()
        trajectory.joint_names = JOINT_NAMES
        trajectory.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(self.start_delay)
        trajectory.points = joint_points
        self.command_pub.publish(trajectory)

        rospy.loginfo("Published stability line trajectory: %d points, duration=%.2fs", len(joint_points), self.duration)

        exec_start = trajectory.header.stamp
        self.publish_reference_in_real_time(exec_start, times, cart_points)
        rospy.loginfo("Stability line trajectory reference publishing complete")


if __name__ == "__main__":
    try:
        node = UR5RehabStabilityLineController()
        node.run()
    except rospy.ROSInterruptException:
        pass
