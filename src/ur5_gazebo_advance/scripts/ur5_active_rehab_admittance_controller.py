#!/usr/bin/env python3

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
        self.position_only = bool(rospy.get_param("~ik_position_only", True))
        self.orientation_weight = float(rospy.get_param("~ik_orientation_weight", 0.05))

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


class UR5ActiveRehabAdmittanceController:
    def __init__(self):
        rospy.init_node("ur5_active_rehab_admittance_controller")

        self.control_mode = rospy.get_param("~control_mode", "mode1")
        if self.control_mode not in ("mode1", "mode2"):
            raise ValueError("~control_mode must be mode1 or mode2")

        self.control_rate_hz = float(rospy.get_param("~control_rate_hz", 100.0))
        self.command_horizon = float(rospy.get_param("~command_horizon", 0.05))
        self.reference_frame_id = rospy.get_param("~reference_frame_id", "base_link")

        self.command_topic = rospy.get_param("~command_topic", "/ur5_arm_controller/command")
        self.wrench_topic = rospy.get_param("~wrench_topic", "/ur5/ft_sensor/wrench_corrected")
        self.ee_pose_topic = rospy.get_param("~ee_pose_topic", "/ur5/end_effector_pose")
        self.reference_pose_topic = rospy.get_param("~reference_pose_topic", "/ur5/active_rehab/reference_pose")
        self.nominal_reference_topic = rospy.get_param(
            "~nominal_reference_topic", "/ur5/active_rehab/nominal_reference_pose"
        )
        self.trajectory_reference_topic = rospy.get_param(
            "~trajectory_reference_topic", "/ur5/active_rehab/trajectory_reference"
        )
        self.trajectory_reference_timeout = float(rospy.get_param("~trajectory_reference_timeout", 0.2))

        self.Md = np.array(rospy.get_param("~Md_diag", [1.0, 1.0, 1.0]), dtype=float)
        self.Bd = np.array(rospy.get_param("~Bd_diag", [50.0, 50.0, 50.0]), dtype=float)
        self.Kd = np.array(rospy.get_param("~Kd_diag", [200.0, 200.0, 200.0]), dtype=float)
        self.max_correction = np.array(rospy.get_param("~max_correction", [0.1, 0.1, 0.1]), dtype=float)
        self.force_deadband = float(rospy.get_param("~force_deadband", 0.0))

        self.force_lpf_cutoff_hz = float(rospy.get_param("~force_lpf_cutoff_hz", 10.0))
        self.alpha = np.exp(-2.0 * np.pi * self.force_lpf_cutoff_hz / self.control_rate_hz)

        self.current_joints = np.zeros(6)
        self.has_joint_state = False
        self.current_ee_pose = None
        self.has_ee_pose = False

        self.nominal_position = np.zeros(3)
        self.nominal_orientation = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        self.reference_initialized = False

        self.latest_trajectory_pose = None
        self.latest_trajectory_stamp = rospy.Time(0)

        self.raw_force = np.zeros(3)
        self.filtered_force = np.zeros(3)
        self.delta_x = np.zeros(3)
        self.delta_v = np.zeros(3)

        self.ik_solver = UR5NumericIK()

        self.command_pub = rospy.Publisher(self.command_topic, JointTrajectory, queue_size=1)
        self.reference_pose_pub = rospy.Publisher(self.reference_pose_topic, PoseStamped, queue_size=10)
        self.nominal_reference_pub = rospy.Publisher(self.nominal_reference_topic, PoseStamped, queue_size=10)

        rospy.Subscriber("/joint_states", JointState, self.joint_state_cb, queue_size=50)
        rospy.Subscriber(self.wrench_topic, WrenchStamped, self.wrench_cb, queue_size=50)
        rospy.Subscriber(self.ee_pose_topic, PoseStamped, self.ee_pose_cb, queue_size=50)
        rospy.Subscriber(
            self.trajectory_reference_topic,
            PoseStamped,
            self.trajectory_reference_cb,
            queue_size=50,
        )

        rospy.loginfo("active rehab admittance controller started")
        rospy.loginfo("  control_mode: %s", self.control_mode)
        rospy.loginfo("  wrench_topic: %s", self.wrench_topic)
        rospy.loginfo("  ee_pose_topic: %s", self.ee_pose_topic)
        rospy.loginfo("  trajectory_reference_topic: %s", self.trajectory_reference_topic)
        rospy.loginfo("  Md: %s", self.Md.tolist())
        rospy.loginfo("  Bd: %s", self.Bd.tolist())
        rospy.loginfo("  Kd: %s", self.Kd.tolist())

    @staticmethod
    def _normalize_quaternion(quat):
        quat = np.array(quat, dtype=float)
        norm = np.linalg.norm(quat)
        if norm < 1e-12:
            return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        return quat / norm

    @staticmethod
    def _pose_to_arrays(msg):
        position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ], dtype=float)
        orientation = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ], dtype=float)
        return position, orientation

    def joint_state_cb(self, msg):
        try:
            joint_map = dict(zip(msg.name, msg.position))
            self.current_joints = np.array([joint_map[name] for name in JOINT_NAMES], dtype=float)
            self.has_joint_state = True
        except Exception:
            return

    def ee_pose_cb(self, msg):
        self.current_ee_pose = msg
        self.has_ee_pose = True

        if self.control_mode == "mode1" and not self.reference_initialized:
            position, orientation = self._pose_to_arrays(msg)
            self.nominal_position = position
            self.nominal_orientation = self._normalize_quaternion(orientation)
            self.reference_initialized = True
            rospy.loginfo("Initialized mode1 reference from current end-effector pose")

    def wrench_cb(self, msg):
        self.raw_force = np.array(
            [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z], dtype=float
        )

    def trajectory_reference_cb(self, msg):
        self.latest_trajectory_pose = msg
        self.latest_trajectory_stamp = rospy.Time.now()
        if self.control_mode == "mode2":
            position, orientation = self._pose_to_arrays(msg)
            self.nominal_position = position
            self.nominal_orientation = self._normalize_quaternion(orientation)
            self.reference_initialized = True

    def _has_fresh_trajectory_reference(self):
        if self.latest_trajectory_pose is None:
            return False
        age = (rospy.Time.now() - self.latest_trajectory_stamp).to_sec()
        return age <= self.trajectory_reference_timeout

    def _update_admittance(self, dt):
        self.filtered_force = self.alpha * self.filtered_force + (1.0 - self.alpha) * self.raw_force

        if self.force_deadband > 0.0:
            mask = np.abs(self.filtered_force) < self.force_deadband
            self.filtered_force[mask] = 0.0

        accel = (self.filtered_force - self.Bd * self.delta_v - self.Kd * self.delta_x) / self.Md
        self.delta_v += accel * dt
        self.delta_x += self.delta_v * dt
        self.delta_x = np.clip(self.delta_x, -self.max_correction, self.max_correction)

    def _get_nominal_reference(self):
        if self.control_mode == "mode1":
            if not self.reference_initialized:
                raise RuntimeError("mode1 reference is not initialized")
            return self.nominal_position.copy(), self.nominal_orientation.copy()

        if not self.reference_initialized:
            raise RuntimeError("mode2 trajectory reference is not initialized")

        if not self._has_fresh_trajectory_reference():
            rospy.logwarn_throttle(1.0, "Trajectory reference is stale, holding last nominal pose")

        return self.nominal_position.copy(), self.nominal_orientation.copy()

    def _publish_pose(self, publisher, position, orientation):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.reference_frame_id
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])
        pose.pose.orientation.x = float(orientation[0])
        pose.pose.orientation.y = float(orientation[1])
        pose.pose.orientation.z = float(orientation[2])
        pose.pose.orientation.w = float(orientation[3])
        publisher.publish(pose)

    def _publish_joint_command(self, q_cmd):
        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = q_cmd.tolist()
        point.time_from_start = rospy.Duration.from_sec(self.command_horizon)
        traj.points = [point]

        self.command_pub.publish(traj)

    def run(self):
        rate = rospy.Rate(self.control_rate_hz)
        dt = 1.0 / self.control_rate_hz

        while not rospy.is_shutdown() and not self.has_joint_state:
            rospy.logwarn_throttle(2.0, "Waiting for /joint_states")
            rate.sleep()

        while not rospy.is_shutdown() and not self.has_ee_pose:
            rospy.logwarn_throttle(2.0, "Waiting for end-effector pose")
            rate.sleep()

        while not rospy.is_shutdown() and not self.reference_initialized:
            if self.control_mode == "mode2":
                rospy.logwarn_throttle(2.0, "Waiting for trajectory reference pose")
            else:
                rospy.logwarn_throttle(2.0, "Waiting for mode1 reference initialization")
            rate.sleep()

        ik_fail_count = 0

        while not rospy.is_shutdown():
            self._update_admittance(dt)

            nominal_position, nominal_orientation = self._get_nominal_reference()
            target_position = nominal_position + self.delta_x

            self._publish_pose(self.nominal_reference_pub, nominal_position, nominal_orientation)
            self._publish_pose(self.reference_pose_pub, target_position, nominal_orientation)

            q_sol, ok = self.ik_solver.solve(target_position, nominal_orientation, self.current_joints)
            if ok:
                self._publish_joint_command(q_sol)
                if ik_fail_count > 0:
                    rospy.loginfo("IK recovered after %d consecutive failures", ik_fail_count)
                    ik_fail_count = 0
            else:
                ik_fail_count += 1
                rospy.logwarn_throttle(1.0, "IK did not converge for current admittance target")
                self._publish_joint_command(self.current_joints)
                self.delta_x = np.zeros(3)
                self.delta_v = np.zeros(3)

            rate.sleep()


if __name__ == "__main__":
    try:
        node = UR5ActiveRehabAdmittanceController()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("active rehab admittance controller failed: %s", str(exc))
