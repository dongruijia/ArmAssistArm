#!/usr/bin/env python3
"""固定点导纳控制节点，根据外力修正末端参考位置。"""

import numpy as np
import rospy
import tf.transformations as tf_t
from geometry_msgs.msg import PoseStamped, WrenchStamped
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


class UR5NumericIK:
    """供导纳控制器内部调用的轻量数值 IK 求解器。"""

    def __init__(self):
        self.dh = [
            (np.pi / 2.0, 0.0, 0.089159),
            (0.0, -0.425, 0.0),
            (0.0, -0.39225, 0.0),
            (np.pi / 2.0, 0.0, 0.10915),
            (-np.pi / 2.0, 0.0, 0.09465),
            (0.0, 0.0, 0.0823),
        ]
        self.base_yaw_correction = float(rospy.get_param("~base_yaw_correction", np.pi))
        self.tool_yaw_correction = float(rospy.get_param("~tool_yaw_correction", 0.0))

        default_joint_limits_deg = np.array([
            [-360.0, 360.0],
            [-360.0, 360.0],
            [-360.0, 360.0],
            [-360.0, 360.0],
            [-360.0, 360.0],
            [-360.0, 360.0],
        ])
        joint_limits_deg = np.array(
            rospy.get_param("~joint_limits_deg", default_joint_limits_deg.tolist()), dtype=float
        )
        if joint_limits_deg.shape != (6, 2):
            raise ValueError("~joint_limits_deg must be a 6x2 array")
        self.joint_limits = np.deg2rad(joint_limits_deg).T

        self.max_iter = int(rospy.get_param("~ik_max_iter", 120))
        self.tol_pos = float(rospy.get_param("~ik_tol_pos", 1e-3))
        self.tol_ori = float(rospy.get_param("~ik_tol_ori", 5e-3))
        self.step = float(rospy.get_param("~ik_step", 0.8))
        self.max_step_norm = float(rospy.get_param("~ik_max_step_norm", 0.2))
        self.damping = float(rospy.get_param("~ik_damping", 0.03))
        self.position_only = bool(rospy.get_param("~ik_position_only", False))
        self.orientation_weight = float(rospy.get_param("~ik_orientation_weight", 0.20))

    @staticmethod
    def _dh_A(alpha, a, d, q):
        T_rz = tf_t.rotation_matrix(q, (0, 0, 1))
        T_tz = tf_t.translation_matrix((0, 0, d))
        T_tx = tf_t.translation_matrix((a, 0, 0))
        T_rx = tf_t.rotation_matrix(alpha, (1, 0, 0))
        return tf_t.concatenate_matrices(T_rz, T_tz, T_tx, T_rx)

    def fk(self, q):
        T = tf_t.rotation_matrix(self.base_yaw_correction, (0, 0, 1))
        for i in range(6):
            alpha, a, d = self.dh[i]
            T = T @ self._dh_A(alpha, a, d, q[i])
        if abs(self.tool_yaw_correction) > 1e-12:
            T = T @ tf_t.rotation_matrix(self.tool_yaw_correction, (0, 0, 1))
        return T

    def jacobian(self, q):
        J = np.zeros((6, 6))
        T = tf_t.rotation_matrix(self.base_yaw_correction, (0, 0, 1))
        joint_axes = []
        joint_pos = []

        for i in range(6):
            joint_axes.append(T[:3, 2].copy())
            joint_pos.append(T[:3, 3].copy())
            alpha, a, d = self.dh[i]
            T = T @ self._dh_A(alpha, a, d, q[i])

        if abs(self.tool_yaw_correction) > 1e-12:
            T = T @ tf_t.rotation_matrix(self.tool_yaw_correction, (0, 0, 1))

        pe = T[:3, 3]
        for i in range(6):
            J[:3, i] = np.cross(joint_axes[i], pe - joint_pos[i])
            J[3:, i] = joint_axes[i]
        return J

    @staticmethod
    def orientation_error(Rc, Rd):
        return 0.5 * (
            np.cross(Rc[:, 0], Rd[:, 0])
            + np.cross(Rc[:, 1], Rd[:, 1])
            + np.cross(Rc[:, 2], Rd[:, 2])
        )

    def solve(self, target_pos, target_quat, q0):
        """对当前参考位姿执行一次数值 IK 求解。"""

        Td = tf_t.quaternion_matrix(target_quat)
        Td[:3, 3] = target_pos

        q = np.clip(np.array(q0, dtype=float), self.joint_limits[0], self.joint_limits[1])

        for _ in range(self.max_iter):
            Tc = self.fk(q)
            ep = Td[:3, 3] - Tc[:3, 3]
            eo = self.orientation_error(Tc[:3, :3], Td[:3, :3])

            if np.linalg.norm(ep) < self.tol_pos:
                if self.position_only or np.linalg.norm(eo) < self.tol_ori:
                    return q.copy(), True

            J = self.jacobian(q)
            if self.position_only:
                J_task = J[:3, :]
                e_task = ep
            else:
                J_task = np.vstack([J[:3, :], self.orientation_weight * J[3:, :]])
                e_task = np.hstack([ep, self.orientation_weight * eo])

            A = J_task @ J_task.T + (self.damping ** 2) * np.eye(J_task.shape[0])
            try:
                y = np.linalg.solve(A, e_task)
            except np.linalg.LinAlgError:
                y = np.linalg.pinv(A) @ e_task
            dq = J_task.T @ y

            dq_norm = np.linalg.norm(dq)
            if dq_norm > self.max_step_norm:
                dq = dq * (self.max_step_norm / dq_norm)

            q = np.clip(q + self.step * dq, self.joint_limits[0], self.joint_limits[1])

        return q.copy(), False


class UR5AdmittanceFixedPointController:
    """根据测得外力在线修正固定点附近的末端参考位姿。"""

    @staticmethod
    def _param_vec3(name, default):
        value = rospy.get_param(name, default)
        if isinstance(value, (int, float)):
            return np.array([float(value), float(value), float(value)], dtype=float)
        arr = np.array(value, dtype=float).reshape(-1)
        if arr.size != 3:
            raise ValueError(f"{name} must be a scalar or a 3-element array")
        return arr

    def __init__(self):
        rospy.init_node("ur5_admittance_fixed_point_controller")

        self.control_rate_hz = float(rospy.get_param("~control_rate_hz", 100.0))
        self.command_horizon = float(rospy.get_param("~command_horizon", 0.05))
        self.command_topic = rospy.get_param("~command_topic", "/ur5_arm_controller/command")
        self.wrench_topic = rospy.get_param("~wrench_topic", "/ur5/ft_sensor/wrench_corrected")
        self.ee_pose_topic = rospy.get_param("~ee_pose_topic", "/ur5/ee_pose")
        self.reference_pose_topic = rospy.get_param("~reference_pose_topic", "/ur5/admittance/reference_pose")
        self.filtered_wrench_topic = rospy.get_param(
            "~filtered_wrench_topic", "/ur5/admittance/filtered_wrench"
        )

        self.fixed_position = np.array(rospy.get_param("~fixed_position", [0.65, 0.0, 0.55]), dtype=float)
        self.fixed_orientation = np.array(
            rospy.get_param("~fixed_orientation", [0.0, 0.0, 0.0, 1.0]), dtype=float
        )
        self.use_current_pose_as_fixed = bool(rospy.get_param("~use_current_pose_as_fixed", False))

        self.max_correction = np.array(rospy.get_param("~max_correction", [0.1, 0.1, 0.1]), dtype=float)
        self.force_deadband = float(rospy.get_param("~force_deadband", 0.0))
        self.force_filter_type = str(rospy.get_param("~force_filter_type", "lpf")).strip().lower()
        if self.force_filter_type not in ("lpf", "kalman"):
            rospy.logwarn(
                "Unknown force_filter_type=%s, fallback to lpf", self.force_filter_type
            )
            self.force_filter_type = "lpf"

        self.Md = np.array(rospy.get_param("~Md_diag", [1.0, 1.0, 1.0]), dtype=float)
        self.Bd = np.array(rospy.get_param("~Bd_diag", [50.0, 50.0, 50.0]), dtype=float)
        self.Kd = np.array(rospy.get_param("~Kd_diag", [200.0, 200.0, 200.0]), dtype=float)

        self.force_lpf_cutoff_hz = float(rospy.get_param("~force_lpf_cutoff_hz", 10.0))
        self.alpha = np.exp(-2.0 * np.pi * self.force_lpf_cutoff_hz / self.control_rate_hz)
        self.kalman_process_var = self._param_vec3("~kalman_process_var", [1e-3, 1e-3, 1e-3])
        self.kalman_measurement_var = self._param_vec3("~kalman_measurement_var", [5e-1, 5e-1, 5e-1])

        q_norm = np.linalg.norm(self.fixed_orientation)
        if q_norm < 1e-12:
            self.fixed_orientation = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        else:
            self.fixed_orientation = self.fixed_orientation / q_norm

        self.current_joints = np.zeros(6)
        self.has_joint_state = False
        self.current_ee_pose = None
        self.has_ee_pose = False
        self.nominal_set = not self.use_current_pose_as_fixed

        self.raw_force = np.zeros(3)
        self.filtered_force = np.zeros(3)
        self.delta_x = np.zeros(3)
        self.delta_v = np.zeros(3)
        self.last_wrench_frame_id = "base_link"
        self.kalman_x = np.zeros(3)
        self.kalman_P = np.ones(3)

        self.ik_solver = UR5NumericIK()

        # 订阅力和位姿反馈，输出经导纳修正后的关节轨迹与参考位姿。
        self.command_pub = rospy.Publisher(self.command_topic, JointTrajectory, queue_size=1)
        self.reference_pose_pub = rospy.Publisher(self.reference_pose_topic, PoseStamped, queue_size=10)
        self.filtered_wrench_pub = rospy.Publisher(
            self.filtered_wrench_topic, WrenchStamped, queue_size=10
        )

        self.joint_sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_cb, queue_size=50)
        self.wrench_sub = rospy.Subscriber(self.wrench_topic, WrenchStamped, self.wrench_cb, queue_size=50)
        self.ee_pose_sub = rospy.Subscriber(self.ee_pose_topic, PoseStamped, self.ee_pose_cb, queue_size=50)

        rospy.loginfo("admittance fixed-point controller started")
        rospy.loginfo("  control_rate_hz: %.1f", self.control_rate_hz)
        rospy.loginfo("  wrench_topic: %s", self.wrench_topic)
        rospy.loginfo("  ee_pose_topic: %s", self.ee_pose_topic)
        rospy.loginfo("  command_topic: %s", self.command_topic)
        rospy.loginfo("  filtered_wrench_topic: %s", self.filtered_wrench_topic)
        rospy.loginfo("  force_filter_type: %s", self.force_filter_type)
        rospy.loginfo("  force_lpf_cutoff_hz: %.3f", self.force_lpf_cutoff_hz)
        rospy.loginfo("  kalman_process_var: %s", self.kalman_process_var.tolist())
        rospy.loginfo("  kalman_measurement_var: %s", self.kalman_measurement_var.tolist())
        rospy.loginfo("  Md: %s", self.Md.tolist())
        rospy.loginfo("  Bd: %s", self.Bd.tolist())
        rospy.loginfo("  Kd: %s", self.Kd.tolist())

    def joint_state_cb(self, msg):
        try:
            joint_map = dict(zip(msg.name, msg.position))
            self.current_joints = np.array([joint_map[name] for name in JOINT_NAMES], dtype=float)
            self.has_joint_state = True
        except Exception:
            return

    def ee_pose_cb(self, msg):
        """需要时用当前末端位姿初始化固定参考点。"""

        self.current_ee_pose = msg
        self.has_ee_pose = True
        # 注意：固定点只初始化一次，避免控制过程中参考点持续漂移。
        if self.use_current_pose_as_fixed and not self.nominal_set:
            self.fixed_position = np.array(
                [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float
            )
            self.fixed_orientation = np.array(
                [
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w,
                ],
                dtype=float,
            )
            q_norm = np.linalg.norm(self.fixed_orientation)
            if q_norm > 1e-12:
                self.fixed_orientation /= q_norm
            else:
                self.fixed_orientation = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
            self.nominal_set = True
            rospy.loginfo("Initialized fixed pose from ee_pose topic")

    def wrench_cb(self, msg):
        self.raw_force = np.array(
            [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z], dtype=float
        )
        if msg.header.frame_id:
            self.last_wrench_frame_id = msg.header.frame_id

    def _kalman_update(self, measurement):
        """三轴独立的一阶离散卡尔曼更新。"""

        P_pred = self.kalman_P + self.kalman_process_var
        K = P_pred / (P_pred + self.kalman_measurement_var)
        self.kalman_x = self.kalman_x + K * (measurement - self.kalman_x)
        self.kalman_P = (1.0 - K) * P_pred
        return self.kalman_x.copy()

    def _update_admittance(self, dt):
        # 以质量-阻尼-刚度模型积分得到末端位置修正量。
        if self.force_filter_type == "kalman":
            self.filtered_force = self._kalman_update(self.raw_force)
        else:
            self.filtered_force = self.alpha * self.filtered_force + (1.0 - self.alpha) * self.raw_force

        if self.force_deadband > 0.0:
            mask = np.abs(self.filtered_force) < self.force_deadband
            self.filtered_force[mask] = 0.0

        accel = (self.filtered_force - self.Bd * self.delta_v - self.Kd * self.delta_x) / self.Md
        self.delta_v += accel * dt
        self.delta_x += self.delta_v * dt
        self.delta_x = np.clip(self.delta_x, -self.max_correction, self.max_correction)

    def _publish_reference_pose(self, position):
        """发布导纳修正后的参考位姿，便于可视化和调试。"""

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])
        pose.pose.orientation.x = float(self.fixed_orientation[0])
        pose.pose.orientation.y = float(self.fixed_orientation[1])
        pose.pose.orientation.z = float(self.fixed_orientation[2])
        pose.pose.orientation.w = float(self.fixed_orientation[3])
        self.reference_pose_pub.publish(pose)

    def _publish_filtered_wrench(self):
        """发布一阶低通后的外力，便于监控滤波效果。"""

        msg = WrenchStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.last_wrench_frame_id
        msg.wrench.force.x = float(self.filtered_force[0])
        msg.wrench.force.y = float(self.filtered_force[1])
        msg.wrench.force.z = float(self.filtered_force[2])
        msg.wrench.torque.x = 0.0
        msg.wrench.torque.y = 0.0
        msg.wrench.torque.z = 0.0
        self.filtered_wrench_pub.publish(msg)

    def _publish_joint_command(self, q_cmd):
        """将当前 IK 解打包成短时域关节指令。"""

        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = JOINT_NAMES

        pt = JointTrajectoryPoint()
        pt.positions = q_cmd.tolist()
        pt.time_from_start = rospy.Duration.from_sec(self.command_horizon)
        traj.points = [pt]

        self.command_pub.publish(traj)

    def run(self):
        """主循环：更新导纳状态、求解 IK、发布参考与控制命令。"""

        rate = rospy.Rate(self.control_rate_hz)
        dt = 1.0 / self.control_rate_hz

        while not rospy.is_shutdown() and not self.has_joint_state:
            rospy.logwarn_throttle(2.0, "Waiting for /joint_states")
            rate.sleep()

        while not rospy.is_shutdown() and not self.nominal_set:
            rospy.logwarn_throttle(2.0, "Waiting for ee_pose to initialize fixed pose")
            rate.sleep()

        while not rospy.is_shutdown():
            self._update_admittance(dt)
            self._publish_filtered_wrench()

            target_position = self.fixed_position + self.delta_x
            self._publish_reference_pose(target_position)

            q_sol, ok = self.ik_solver.solve(target_position, self.fixed_orientation, self.current_joints)
            if ok:
                self._publish_joint_command(q_sol)
            else:
                rospy.logwarn_throttle(1.0, "IK did not converge for current admittance reference")

            rate.sleep()


if __name__ == "__main__":
    try:
        node = UR5AdmittanceFixedPointController()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("admittance fixed-point controller failed: %s", str(exc))
