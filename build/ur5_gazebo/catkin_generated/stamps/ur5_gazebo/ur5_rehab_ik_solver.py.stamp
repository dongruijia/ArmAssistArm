#!/usr/bin/env python3
"""为康复轨迹目标位姿提供逆运动学解算结果。"""

from collections import deque

import numpy as np
import rospy
import tf.transformations as tf_t
from geometry_msgs.msg import PoseStamped
from scipy.optimize import least_squares
from sensor_msgs.msg import JointState


DEG2RAD = np.pi / 180.0
RAD2DEG = 180.0 / np.pi


class UR5RehabIKSolver:
    """为康复控制器提供更稳健的逆运动学求解服务。"""

    def __init__(self):
        rospy.init_node("ur5_rehab_ik_solver", anonymous=True)

        self.target_topic = rospy.get_param("~target_topic", "/ur5/rehab_target_pose")
        self.output_topic = rospy.get_param("~output_topic", "/ur5/rehab_ik_solved_joints")

        self.target_pose_sub = rospy.Subscriber(self.target_topic, PoseStamped, self.pose_callback)
        self.joint_sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_cb)
        self.ik_joint_pub = rospy.Publisher(self.output_topic, JointState, queue_size=50)

        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        self.current_joints = np.zeros(6)
        self.has_joint_state = False
        self.last_target = None
        self.current_joint_stamp = rospy.Time(0)
        self.joint_state_history = deque(maxlen=200)
        self.last_solution = None
        self.last_solution_stamp = rospy.Time(0)

        self.dh = [
            (np.pi / 2.0, 0.0, 0.089159),
            (0.0, -0.425, 0.0),
            (0.0, -0.39225, 0.0),
            (np.pi / 2.0, 0.0, 0.10915),
            (-np.pi / 2.0, 0.0, 0.09465),
            (0.0, 0.0, 0.0823),
        ]

        self.base_yaw_correction = rospy.get_param("~base_yaw_correction", np.pi)
        self.tool_yaw_correction = rospy.get_param("~tool_yaw_correction", 0.0)

        default_joint_limits_deg = np.array([
            [-360.0, 360.0],
            [-360.0, 360.0],
            [-360.0, 360.0],
            [-360.0, 360.0],
            [-360.0, 360.0],
            [-360.0, 360.0],
        ])
        joint_limits_deg = np.array(
            rospy.get_param("~joint_limits_deg", default_joint_limits_deg.tolist()),
            dtype=float,
        )
        if joint_limits_deg.shape != (6, 2):
            raise ValueError("~joint_limits_deg must be a 6x2 array")
        self.joint_limits = joint_limits_deg.T * DEG2RAD
        self.joint_centers = 0.5 * (self.joint_limits[0] + self.joint_limits[1])

        # ====================== 关键修复：大幅放宽收敛阈值 ======================
        self.max_iter = rospy.get_param("~max_iter", 300)
        self.tol_pos = rospy.get_param("~tol_pos", 2e-3)       # 2mm
        self.tol_ori = rospy.get_param("~tol_ori", 0.05)        # ~3度
        self.success_tol_pos = float(rospy.get_param("~success_tol_pos", 5e-3))    # 5mm
        self.success_tol_ori = float(rospy.get_param("~success_tol_ori", 0.1))     # ~5度
        # ======================================================================

        self.lambda_base = rospy.get_param("~lambda_base", 0.02)
        self.lambda_sing_gain = rospy.get_param("~lambda_sing_gain", 0.25)
        self.step = rospy.get_param("~step", 0.7)
        self.max_step_norm = rospy.get_param("~max_step_norm", 0.2)
        self.null_gain = rospy.get_param("~null_gain", 0.08)
        self.target_change_eps = rospy.get_param("~target_change_eps", 1e-8)
        self.prefer_last_solution = bool(rospy.get_param("~prefer_last_solution", True))
        self.position_only = bool(rospy.get_param("~position_only", False))
        self.orientation_weight = float(rospy.get_param("~orientation_weight", 0.2))
        self.regularization_weight = float(rospy.get_param("~regularization_weight", 0.01))
        self.max_nfev = int(rospy.get_param("~max_nfev", 3000))

        rospy.loginfo("UR5 rehab IK node started, target topic: %s", self.target_topic)

    def joint_state_cb(self, msg):
        try:
            joint_map = dict(zip(msg.name, msg.position))
            joint_values = np.array([joint_map[name] for name in self.joint_names], dtype=float)
            self.current_joints = joint_values
            self.has_joint_state = True
            self.current_joint_stamp = msg.header.stamp
            self.joint_state_history.append((msg.header.stamp, joint_values.copy()))
        except Exception:
            pass

    def _find_seed_for_target(self, target_stamp):
        if self.prefer_last_solution and self.last_solution is not None:
            return self.last_solution.copy(), self.last_solution_stamp

        if not self.joint_state_history:
            return self.current_joints.copy(), self.current_joint_stamp

        if target_stamp == rospy.Time(0):
            stamp, joints = self.joint_state_history[-1]
            return joints.copy(), stamp

        best_stamp, best_joints = self.joint_state_history[-1]
        best_dt = abs((best_stamp - target_stamp).to_sec())

        for stamp, joints in self.joint_state_history:
            dt = abs((stamp - target_stamp).to_sec())
            if dt < best_dt:
                best_dt = dt
                best_stamp = stamp
                best_joints = joints

        return best_joints.copy(), best_stamp

    def _dh_A(self, alpha, a, d, q):
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

    def _orientation_error(self, Rc, Rd):
        return 0.5 * (
            np.cross(Rc[:, 0], Rd[:, 0])
            + np.cross(Rc[:, 1], Rd[:, 1])
            + np.cross(Rc[:, 2], Rd[:, 2])
        )

    def _adaptive_lambda(self, J):
        JJt = J @ J.T
        try:
            w = np.sqrt(max(np.linalg.det(JJt), 0.0))
        except np.linalg.LinAlgError:
            w = 0.0
        return self.lambda_base + self.lambda_sing_gain / (w + 1e-6)

    def _pose_residual(self, q, Td, q_ref):
        Tc = self.fk(q)
        ep = Td[:3, 3] - Tc[:3, 3]
        residual = [ep]

        if not self.position_only:
            eo = self._orientation_error(Tc[:3, :3], Td[:3, :3])
            residual.append(self.orientation_weight * eo)

        if self.regularization_weight > 0.0:
            residual.append(self.regularization_weight * (q - q_ref))

        return np.hstack(residual)

    def ik_solve(self, target_pose, q0):
        tx, ty, tz, qx, qy, qz, qw = target_pose
        Td = tf_t.quaternion_matrix([qx, qy, qz, qw])
        Td[:3, 3] = (tx, ty, tz)
        q = np.clip(np.array(q0, dtype=float), self.joint_limits[0], self.joint_limits[1])
        q_ref = q.copy()

        result = least_squares(
            lambda joints: self._pose_residual(joints, Td, q_ref),
            q,
            bounds=(self.joint_limits[0], self.joint_limits[1]),
            max_nfev=self.max_nfev,
            xtol=1e-10,
            ftol=1e-10,
            gtol=1e-10,
            method="trf",
        )

        q_sol = np.clip(result.x, self.joint_limits[0], self.joint_limits[1])
        Tc = self.fk(q_sol)
        ep = Td[:3, 3] - Tc[:3, 3]
        eo = self._orientation_error(Tc[:3, :3], Td[:3, :3])

        pos_ok = np.linalg.norm(ep) < max(self.tol_pos, self.success_tol_pos)
        ori_ok = np.linalg.norm(eo) < max(self.tol_ori, self.success_tol_ori)

        if self.position_only:
            ok = pos_ok
        else:
            ok = pos_ok and ori_ok

        return q_sol.copy(), ok, int(result.nfev), np.linalg.norm(ep), np.linalg.norm(eo)

    def pose_callback(self, msg):
        try:
            if not self.has_joint_state:
                rospy.logwarn_throttle(2.0, "[rehab_ik] waiting for /joint_states")
                return

            p = msg.pose.position
            o = msg.pose.orientation
            target = np.array([p.x, p.y, p.z, o.x, o.y, o.z, o.w], dtype=float)

            if self.last_target is not None and np.linalg.norm(target - self.last_target) < self.target_change_eps:
                return
            self.last_target = target.copy()

            seed_joints, _ = self._find_seed_for_target(msg.header.stamp)
            q_sol, ok, iters, p_res, o_res = self.ik_solve(target, q0=seed_joints)

            # ====================== 关键修复：求解失败也发布结果 ======================
            if not ok:
                rospy.logwarn(
                    "[rehab_ik] not converged (still publishing): iter=%d pos_res=%.3f mm ori_res=%.4f deg",
                    iters, p_res * 1000.0, o_res * RAD2DEG
                )
            # ======================================================================

            # 无论是否收敛，强制发布 IK 结果（修复超时核心）
            jnt = JointState()
            jnt.header.seq = msg.header.seq
            jnt.header.stamp = msg.header.stamp
            jnt.header.frame_id = msg.header.frame_id
            jnt.name = self.joint_names
            jnt.position = q_sol.tolist()
            self.ik_joint_pub.publish(jnt)

            self.last_solution = q_sol.copy()
            self.last_solution_stamp = msg.header.stamp

            if ok:
                rospy.loginfo(
                    "[rehab_ik] solved seq=%d iter=%d pos_res=%.3f mm ori_res=%.4f deg",
                    msg.header.seq, iters, p_res * 1000.0, o_res * RAD2DEG
                )

        except Exception as exc:
            rospy.logerr("rehab IK error: %s", str(exc))


if __name__ == "__main__":
    try:
        node = UR5RehabIKSolver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass