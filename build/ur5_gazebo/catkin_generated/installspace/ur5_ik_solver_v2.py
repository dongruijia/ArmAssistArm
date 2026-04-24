#!/usr/bin/env python3
import rospy
import numpy as np
import tf.transformations as tf_t
from collections import deque
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

DEG2RAD = np.pi / 180.0
RAD2DEG = 180.0 / np.pi


class UR5IKNewtonRaphsonV2:
    def __init__(self):
        rospy.init_node("ur5_ik_newton_raphson_v2_node", anonymous=True)

        self.target_topic = rospy.get_param("~target_topic", "/ur5/ik_target_pose")
        self.target_pose_sub = rospy.Subscriber(self.target_topic, PoseStamped, self.pose_callback)
        self.joint_sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_cb)
        self.ik_joint_pub = rospy.Publisher("/ur5/ik_solved_joints", JointState, queue_size=10)

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

        self.dh = [
            (np.pi / 2, 0, 0.089159),
            (0, -0.425, 0),
            (0, -0.39225, 0),
            (np.pi / 2, 0, 0.10915),
            (-np.pi / 2, 0, 0.09465),
            (0, 0, 0.0823),
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
            rospy.get_param("~joint_limits_deg", default_joint_limits_deg.tolist()), dtype=float
        )
        if joint_limits_deg.shape != (6, 2):
            raise ValueError("~joint_limits_deg must be a 6x2 array")
        self.joint_limits = joint_limits_deg.T * DEG2RAD
        self.joint_centers = 0.5 * (self.joint_limits[0] + self.joint_limits[1])

        self.max_iter = rospy.get_param("~max_iter", 600)
        self.tol_pos = rospy.get_param("~tol_pos", 1e-3)
        self.tol_ori = rospy.get_param("~tol_ori", 1.7e-3)
        self.position_only = rospy.get_param("~position_only", True)
        self.ori_weight = rospy.get_param("~ori_weight", 0.2)
        self.lambda_base = rospy.get_param("~lambda_base", 0.02)
        self.lambda_sing_gain = rospy.get_param("~lambda_sing_gain", 0.25)
        self.step = rospy.get_param("~step", 0.5)
        self.max_step_norm = rospy.get_param("~max_step_norm", 0.2)
        self.null_gain = rospy.get_param("~null_gain", 0.08)
        self.target_change_eps = rospy.get_param("~target_change_eps", 1e-9)

        rospy.loginfo(
            "UR5 IK v2 node started, target topic: %s, position_only=%s",
            self.target_topic,
            str(self.position_only),
        )

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
        t_rz = tf_t.rotation_matrix(q, (0, 0, 1))
        t_tz = tf_t.translation_matrix((0, 0, d))
        t_tx = tf_t.translation_matrix((a, 0, 0))
        t_rx = tf_t.rotation_matrix(alpha, (1, 0, 0))
        return tf_t.concatenate_matrices(t_rz, t_tz, t_tx, t_rx)

    def fk(self, q):
        t = tf_t.rotation_matrix(self.base_yaw_correction, (0, 0, 1))
        for i in range(6):
            alpha, a, d = self.dh[i]
            t = t @ self._dh_A(alpha, a, d, q[i])
        if abs(self.tool_yaw_correction) > 1e-12:
            t = t @ tf_t.rotation_matrix(self.tool_yaw_correction, (0, 0, 1))
        return t

    def jacobian(self, q):
        j = np.zeros((6, 6))

        t = tf_t.rotation_matrix(self.base_yaw_correction, (0, 0, 1))
        joint_axes = []
        joint_pos = []

        for i in range(6):
            joint_axes.append(t[:3, 2].copy())
            joint_pos.append(t[:3, 3].copy())

            alpha, a, d = self.dh[i]
            t = t @ self._dh_A(alpha, a, d, q[i])

        if abs(self.tool_yaw_correction) > 1e-12:
            t = t @ tf_t.rotation_matrix(self.tool_yaw_correction, (0, 0, 1))

        p_e = t[:3, 3]
        for i in range(6):
            j[:3, i] = np.cross(joint_axes[i], p_e - joint_pos[i])
            j[3:, i] = joint_axes[i]

        return j

    def _orientation_error(self, r_c, r_d):
        return 0.5 * (
            np.cross(r_c[:, 0], r_d[:, 0])
            + np.cross(r_c[:, 1], r_d[:, 1])
            + np.cross(r_c[:, 2], r_d[:, 2])
        )

    def _adaptive_lambda(self, j):
        jjt = j @ j.T
        try:
            w = np.sqrt(max(np.linalg.det(jjt), 0.0))
        except np.linalg.LinAlgError:
            w = 0.0
        return self.lambda_base + self.lambda_sing_gain / (w + 1e-6)

    def ik_solve(self, target_pose, q0):
        tx, ty, tz, qx, qy, qz, qw = target_pose
        t_d = tf_t.quaternion_matrix([qx, qy, qz, qw])
        t_d[:3, 3] = (tx, ty, tz)

        q = np.clip(q0, self.joint_limits[0], self.joint_limits[1])
        q_ref = q0.copy()

        pos_weight = 1.0
        ori_weight = 0.0 if self.position_only else self.ori_weight

        for it in range(self.max_iter):
            t_c = self.fk(q)
            e_p = t_d[:3, 3] - t_c[:3, 3]
            if self.position_only:
                e_o = np.zeros(3)
            else:
                e_o = self._orientation_error(t_c[:3, :3], t_d[:3, :3])
            e = np.hstack((pos_weight * e_p, ori_weight * e_o))

            if self.position_only:
                converged = np.linalg.norm(e_p) < self.tol_pos
            else:
                converged = np.linalg.norm(e_p) < self.tol_pos and np.linalg.norm(e_o) < self.tol_ori
            if converged:
                return q, True, it + 1, np.linalg.norm(e_p), np.linalg.norm(e_o)

            j = self.jacobian(q)
            i6 = np.eye(6)

            j_w = j.copy()
            j_w[:3, :] *= pos_weight
            j_w[3:, :] *= ori_weight

            lam = self._adaptive_lambda(j_w)
            j_d = j_w.T @ np.linalg.inv(j_w @ j_w.T + (lam ** 2) * i6)

            grad = 0.7 * (q_ref - q) + 0.3 * (self.joint_centers - q)
            n_mat = i6 - j_d @ j_w

            dq = self.step * (j_d @ e + self.null_gain * (n_mat @ grad))
            dq_norm = np.linalg.norm(dq)
            if dq_norm > self.max_step_norm:
                dq *= self.max_step_norm / (dq_norm + 1e-12)

            q += dq
            q = np.clip(q, self.joint_limits[0], self.joint_limits[1])

        t_c = self.fk(q)
        e_p = t_d[:3, 3] - t_c[:3, 3]
        if self.position_only:
            e_o = np.zeros(3)
        else:
            e_o = self._orientation_error(t_c[:3, :3], t_d[:3, :3])
        return q, False, self.max_iter, np.linalg.norm(e_p), np.linalg.norm(e_o)

    def pose_callback(self, msg):
        try:
            if not self.has_joint_state:
                rospy.logwarn_throttle(2.0, "[IK-v2] waiting for /joint_states")
                return

            p = msg.pose.position
            o = msg.pose.orientation
            target = np.array([p.x, p.y, p.z, o.x, o.y, o.z, o.w], dtype=float)

            if self.last_target is not None and np.linalg.norm(target - self.last_target) < self.target_change_eps:
                return
            self.last_target = target.copy()

            seed_joints, seed_stamp = self._find_seed_for_target(msg.header.stamp)
            q_sol, ok, iters, p_res, o_res = self.ik_solve(target, q0=seed_joints)

            if not ok:
                rospy.logwarn(
                    "[IK-v2] not converged: iter=%d pos_res=%.3f mm ori_res=%.4f deg",
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

            rospy.loginfo(
                "[IK-v2] solved: iter=%d pos_res=%.3f mm ori_res=%.4f deg",
                iters,
                p_res * 1000.0,
                o_res * RAD2DEG,
            )

        except Exception as e:
            rospy.logerr("IK-v2 error: %s", str(e))


if __name__ == "__main__":
    try:
        node = UR5IKNewtonRaphsonV2()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
