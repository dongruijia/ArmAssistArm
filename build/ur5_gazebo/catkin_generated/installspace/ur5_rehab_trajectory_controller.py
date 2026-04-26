#!/usr/bin/env python3
"""生成康复训练轨迹，并通过 IK 结果下发关节轨迹。"""

import numpy as np
import rospy
import tf.transformations as tf_t
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


class UR5RehabTrajectoryController:
    """生成多段康复轨迹，并将笛卡尔轨迹转换为关节轨迹执行。"""

    def __init__(self):
        rospy.init_node("ur5_rehab_trajectory_controller")

        self.command_topic = rospy.get_param("~command_topic", "/ur5_arm_controller/command")
        self.target_topic = rospy.get_param("~target_topic", "/ur5/rehab_target_pose")
        self.ik_output_topic = rospy.get_param("~ik_output_topic", "/ur5/rehab_ik_solved_joints")
        self.dt = float(rospy.get_param("~sample_dt", 0.01))
        self.start_delay = float(rospy.get_param("~start_delay", 0.5))
        self.pause_between = float(rospy.get_param("~pause_between", 0.2))
        self.run_mode = rospy.get_param("~trajectory_mode", "all")
        self.loop = bool(rospy.get_param("~loop", False))
        self.ik_timeout = float(rospy.get_param("~ik_timeout", 0.5))

        self.current_joints = np.zeros(6)
        self.has_joint_state = False
        self.latest_ik_solution = None
        self.latest_ik_stamp = rospy.Time(0)

        # 发布目标位姿触发 IK，并将求解后的关节轨迹发送给控制器。
        self.command_pub = rospy.Publisher(self.command_topic, JointTrajectory, queue_size=1)
        self.target_pub = rospy.Publisher(self.target_topic, PoseStamped, queue_size=1)
        self.joint_sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_cb)
        self.ik_sub = rospy.Subscriber(self.ik_output_topic, JointState, self.ik_result_cb)

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
                raise RuntimeError("Timeout waiting for rehab IK node subscriber on %s" % self.target_topic)
            rate.sleep()

    def resolve_orientation(self):
        """优先使用参数给定姿态，否则沿用当前末端姿态。"""

        orientation_quat = rospy.get_param("~orientation_quat", [])
        if len(orientation_quat) == 4:
            quat = np.array(orientation_quat, dtype=float)
            norm = np.linalg.norm(quat)
            if norm > 1e-12:
                quat /= norm
                return quat

        T = self.compute_fk(self.current_joints)
        quat = np.array(tf_t.quaternion_from_matrix(T), dtype=float)
        quat /= np.linalg.norm(quat)
        return quat

    def compute_fk(self, q):
        dh = [
            (np.pi / 2.0, 0.0, 0.089159),
            (0.0, -0.425, 0.0),
            (0.0, -0.39225, 0.0),
            (np.pi / 2.0, 0.0, 0.10915),
            (-np.pi / 2.0, 0.0, 0.09465),
            (0.0, 0.0, 0.0823),
        ]
        base_yaw_correction = rospy.get_param("~base_yaw_correction", np.pi)
        tool_yaw_correction = rospy.get_param("~tool_yaw_correction", 0.0)

        T = tf_t.rotation_matrix(base_yaw_correction, (0, 0, 1))
        for joint_value, (alpha, a, d) in zip(q, dh):
            T_rz = tf_t.rotation_matrix(joint_value, (0, 0, 1))
            T_tz = tf_t.translation_matrix((0, 0, d))
            T_tx = tf_t.translation_matrix((a, 0, 0))
            T_rx = tf_t.rotation_matrix(alpha, (1, 0, 0))
            T = T @ tf_t.concatenate_matrices(T_rz, T_tz, T_tx, T_rx)

        if abs(tool_yaw_correction) > 1e-12:
            T = T @ tf_t.rotation_matrix(tool_yaw_correction, (0, 0, 1))
        return T

    def quintic_scalars(self, total_time):
        """生成五次多项式时间缩放，保证起止速度和加速度平滑。"""

        point_count = int(round(total_time / self.dt))
        times = np.linspace(0.0, total_time, point_count + 1)
        tau = np.clip(times / total_time, 0.0, 1.0)
        scalars = 10.0 * tau ** 3 - 15.0 * tau ** 4 + 6.0 * tau ** 5
        return times, scalars

    def build_line_trajectory(self):
        start = np.array([0.4, 0.0, 0.6], dtype=float)
        end = np.array([0.6, 0.2, 0.5], dtype=float)
        times, scalars = self.quintic_scalars(5.0)
        points = start[None, :] + scalars[:, None] * (end - start)[None, :]
        return "trajectory_1_line", times, points

    def build_arc_trajectory(self):
        center = np.array([0.5, 0.0, 0.5], dtype=float)
        radius = 0.15
        times, scalars = self.quintic_scalars(6.0)
        theta = np.pi * scalars

        points = np.zeros((len(times), 3), dtype=float)
        points[:, 0] = center[0] + radius * np.cos(theta)
        points[:, 1] = center[1]
        points[:, 2] = center[2] + radius * np.sin(theta)
        return "trajectory_2_arc", times, points

    def build_circle_trajectory(self):
        center = np.array([0.0, 0.5, 0.5], dtype=float)
        radius = 0.1
        times, scalars = self.quintic_scalars(8.0)
        theta = 2.0 * np.pi * scalars

        points = np.zeros((len(times), 3), dtype=float)
        points[:, 0] = center[0]
        points[:, 1] = center[1] + radius * np.cos(theta)
        points[:, 2] = center[2] + radius * np.sin(theta)
        return "trajectory_3_circle", times, points

    def selected_trajectories(self):
        """根据参数选择单条或全部预设轨迹。"""

        builders = {
            "1": self.build_line_trajectory,
            "2": self.build_arc_trajectory,
            "3": self.build_circle_trajectory,
            "line": self.build_line_trajectory,
            "arc": self.build_arc_trajectory,
            "circle": self.build_circle_trajectory,
        }
        if self.run_mode == "all":
            return [
                self.build_line_trajectory(),
                self.build_arc_trajectory(),
                self.build_circle_trajectory(),
            ]
        if self.run_mode not in builders:
            raise ValueError("~trajectory_mode must be one of all/1/2/3/line/arc/circle")
        return [builders[self.run_mode]()]

    def request_ik_solution(self, point, orientation):
        """为单个笛卡尔点请求对应的关节解。"""

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

        expected_stamp = pose.header.stamp
        # 通过时间戳匹配本次目标位姿对应的 IK 结果。
        self.target_pub.publish(pose)

        deadline = rospy.Time.now() + rospy.Duration.from_sec(self.ik_timeout)
        rate = rospy.Rate(500)
        while not rospy.is_shutdown():
            if self.latest_ik_solution is not None:
                if self.latest_ik_stamp == expected_stamp:
                    return self.latest_ik_solution.copy()
            if rospy.Time.now() > deadline:
                raise RuntimeError("Timeout waiting IK response for pose stamp=%s" % str(expected_stamp.to_sec()))
            rate.sleep()

    def solve_joint_trajectory(self, name, times, cart_points, orientation):
        """逐点调用 IK，将笛卡尔轨迹离散成关节轨迹。"""

        joint_points = []

        for index, (time_sec, point) in enumerate(zip(times, cart_points)):
            q_sol = self.request_ik_solution(point, orientation)

            joint_point = JointTrajectoryPoint()
            joint_point.positions = q_sol.tolist()
            joint_point.time_from_start = rospy.Duration.from_sec(float(time_sec))
            joint_points.append(joint_point)

            if index in (0, len(times) - 1) or (index + 1) % 100 == 0:
                rospy.loginfo(
                    "[%s] rehab IK %d/%d solved",
                    name,
                    index + 1,
                    len(times),
                )

        return joint_points

    def publish_trajectory(self, name, joint_points):
        """发布整条关节轨迹，并等待其基本执行完成。"""

        trajectory = JointTrajectory()
        trajectory.joint_names = JOINT_NAMES
        trajectory.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(self.start_delay)
        trajectory.points = joint_points
        self.command_pub.publish(trajectory)

        total_time = joint_points[-1].time_from_start.to_sec()
        rospy.loginfo("Published %s with %d points to %s", name, len(joint_points), self.command_topic)
        rospy.sleep(self.start_delay + total_time + self.pause_between)

    def run_once(self):
        """完成一次轨迹求解与执行流程。"""

        self.wait_for_joint_state()
        self.wait_for_controller_connection()
        self.wait_for_ik_connection()
        orientation = self.resolve_orientation()
        rospy.loginfo(
            "Using end-effector orientation quat=[%.4f, %.4f, %.4f, %.4f]",
            orientation[0],
            orientation[1],
            orientation[2],
            orientation[3],
        )

        for name, times, cart_points in self.selected_trajectories():
            joint_points = self.solve_joint_trajectory(name, times, cart_points, orientation)
            self.publish_trajectory(name, joint_points)
            if joint_points:
                self.current_joints = np.array(joint_points[-1].positions, dtype=float)

    def run(self):
        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            self.run_once()
            if not self.loop:
                break
            rate.sleep()


if __name__ == "__main__":
    try:
        controller = UR5RehabTrajectoryController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("UR5 rehab trajectory controller failed: %s", exc)