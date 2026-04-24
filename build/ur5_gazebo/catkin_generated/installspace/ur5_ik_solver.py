#!/usr/bin/env python3
import rospy
import numpy as np
import tf.transformations as tf_t
from collections import deque
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

DEG2RAD = np.pi / 180.0
RAD2DEG = 180.0 / np.pi

class UR5IKNewtonRaphson:
    def __init__(self):
        rospy.init_node("ur5_ik_newton_raphson_node", anonymous=True)

        self.target_topic = rospy.get_param("~target_topic", "/ur5/end_effector_pose")
        self.target_pose_sub = rospy.Subscriber(self.target_topic, PoseStamped, self.pose_callback)
        self.joint_sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_cb)
        self.ik_joint_pub = rospy.Publisher("/ur5/ik_solved_joints", JointState, queue_size=10)

        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

        self.current_joints = np.zeros(6)
        self.has_joint_state = False
        self.last_target = None
        self.current_joint_stamp = rospy.Time(0)
        self.joint_state_history = deque(maxlen=200)

        self.dh = [
            (np.pi/2, 0, 0.089159),
            (0, -0.425, 0),
            (0, -0.39225, 0),
            (np.pi/2, 0, 0.10915),
            (-np.pi/2, 0, 0.09465),
            (0, 0, 0.0823)
        ]

        # Keep IK FK convention exactly consistent with ur5_forward_kinematics.py.
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
        joint_limits_deg = np.array(rospy.get_param("~joint_limits_deg", default_joint_limits_deg.tolist()), dtype=float)
        if joint_limits_deg.shape != (6, 2):
            raise ValueError("~joint_limits_deg must be a 6x2 array")
        self.joint_limits = joint_limits_deg.T * DEG2RAD
        self.joint_centers = 0.5 * (self.joint_limits[0] + self.joint_limits[1])

        self.max_iter = rospy.get_param("~max_iter", 250)
        self.tol_pos = rospy.get_param("~tol_pos", 1e-4)
        self.tol_ori = rospy.get_param("~tol_ori", 1.7e-3)
        self.lambda_base = rospy.get_param("~lambda_base", 0.02)
        self.lambda_sing_gain = rospy.get_param("~lambda_sing_gain", 0.25)
        self.step = rospy.get_param("~step", 0.7)
        self.max_step_norm = rospy.get_param("~max_step_norm", 0.2)
        self.null_gain = rospy.get_param("~null_gain", 0.08)
        self.target_change_eps = rospy.get_param("~target_change_eps", 1e-8)

        rospy.loginfo("UR5 IK node started, target topic: %s", self.target_topic)

    def joint_state_cb(self, msg):
        try:
            joint_map = dict(zip(msg.name, msg.position))
            joint_values = np.array([joint_map[name] for name in self.joint_names])
            self.current_joints = joint_values
            self.has_joint_state = True
            self.current_joint_stamp = msg.header.stamp
            self.joint_state_history.append((msg.header.stamp, joint_values.copy()))
        except Exception:
            pass

    def _find_seed_for_target(self, target_stamp):
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

    def ik_solve(self, target_pose, q0):
        tx, ty, tz, qx, qy, qz, qw = target_pose
        Td = tf_t.quaternion_matrix([qx, qy, qz, qw])
        Td[:3, 3] = (tx, ty, tz)
        q = np.clip(q0, self.joint_limits[0], self.joint_limits[1])
        q_ref = q0.copy()

        pos_weight = 1.0
        ori_weight = 0.7

        for it in range(self.max_iter):
            Tc = self.fk(q)
            ep = Td[:3, 3] - Tc[:3, 3]
            Rc = Tc[:3, :3]
            Rd = Td[:3, :3]
            eo = self._orientation_error(Rc, Rd)
            e = np.hstack((pos_weight * ep, ori_weight * eo))

            if np.linalg.norm(ep) < self.tol_pos and np.linalg.norm(eo) < self.tol_ori:
                return q, True, it + 1, np.linalg.norm(ep), np.linalg.norm(eo)

            J = self.jacobian(q)
            I = np.eye(6)

            Jw = J.copy()
            Jw[:3, :] *= pos_weight
            Jw[3:, :] *= ori_weight

            lam = self._adaptive_lambda(Jw)
            Jd = Jw.T @ np.linalg.inv(Jw @ Jw.T + (lam ** 2) * I)

            # Null-space term: prefer solution near seed and away from joint limits.
            grad = 0.7 * (q_ref - q) + 0.3 * (self.joint_centers - q)
            N = I - Jd @ Jw

            dq = self.step * (Jd @ e + self.null_gain * (N @ grad))
            dq_norm = np.linalg.norm(dq)
            if dq_norm > self.max_step_norm:
                dq *= (self.max_step_norm / (dq_norm + 1e-12))

            q += dq
            q = np.clip(q, self.joint_limits[0], self.joint_limits[1])

        Tc = self.fk(q)
        ep = Td[:3, 3] - Tc[:3, 3]
        eo = self._orientation_error(Tc[:3, :3], Td[:3, :3])
        return q, False, self.max_iter, np.linalg.norm(ep), np.linalg.norm(eo)

    def pose_callback(self, msg):
        try:
            if not self.has_joint_state:
                rospy.logwarn_throttle(2.0, "[IK] waiting for /joint_states")
                return

            p = msg.pose.position
            o = msg.pose.orientation
            target = np.array([p.x, p.y, p.z, o.x, o.y, o.z, o.w], dtype=float)

            if self.last_target is not None:
                if np.linalg.norm(target - self.last_target) < self.target_change_eps:
                    return
            self.last_target = target.copy()

            seed_joints, seed_stamp = self._find_seed_for_target(msg.header.stamp)
            q_sol, ok, iters, p_res, o_res = self.ik_solve(target, q0=seed_joints)

            if not ok:
                rospy.logwarn(
                    "[IK] not converged: iter=%d pos_res=%.3f mm ori_res=%.4f deg",
                    iters,
                    p_res * 1000.0,
                    o_res * RAD2DEG,
                )
                return

            jnt = JointState()
            jnt.header.stamp = seed_stamp
            jnt.name = self.joint_names
            jnt.position = q_sol.tolist()
            self.ik_joint_pub.publish(jnt)

            T_check = self.fk(q_sol)
            p_err = np.linalg.norm(np.array([p.x, p.y, p.z]) - T_check[:3, 3])
            q_err_deg = np.abs(q_sol - seed_joints) * RAD2DEG
            max_joint_err_deg = float(np.max(q_err_deg))

            rospy.loginfo(
                "[IK] solved: iter=%d pos_err=%.3f mm ori_res=%.4f deg max_joint_delta=%.4f deg",
                iters,
                p_err * 1000.0,
                o_res * RAD2DEG,
                max_joint_err_deg,
            )

        except Exception as e:
            rospy.logerr("IK error: %s", str(e))

if __name__ == "__main__":
    try:
        node = UR5IKNewtonRaphson()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass