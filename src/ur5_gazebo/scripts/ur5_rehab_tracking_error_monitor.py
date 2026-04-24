#!/usr/bin/env python3

from collections import deque

import numpy as np
import rospy
import tf.transformations as tf_t
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState


class UR5RehabTrackingErrorMonitor:
    def __init__(self):
        rospy.init_node("ur5_rehab_tracking_error_monitor")

        self.reference_topic = rospy.get_param("~reference_topic", "/ur5/stability_reference_pose")
        self.joint_topic = rospy.get_param("~joint_topic", "/joint_states")
        self.actual_pose_topic = rospy.get_param("~actual_pose_topic", "/ur5/stability_actual_pose")
        self.match_tolerance = float(rospy.get_param("~match_tolerance", 0.03))
        self.expected_duration = float(rospy.get_param("~expected_duration", 5.0))
        self.sample_dt = float(rospy.get_param("~sample_dt", 0.01))
        self.eval_start_delay = float(rospy.get_param("~eval_start_delay", 1.5))
        self.eval_start_min_z = float(rospy.get_param("~eval_start_min_z", 0.45))

        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

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

        self.reference_buffer = deque(maxlen=2000)
        self.error_samples = []
        self.match_count = 0
        self.expected_samples = int(round(self.expected_duration / self.sample_dt)) + 1
        self.reported_final = False
        self.last_ref_stamp = rospy.Time(0)
        self.first_ref_stamp = rospy.Time(0)
        self.first_ref_point = None
        self.eval_started = False

        self.actual_pose_pub = rospy.Publisher(self.actual_pose_topic, PoseStamped, queue_size=10)
        rospy.Subscriber(self.reference_topic, PoseStamped, self.reference_cb)
        rospy.Subscriber(self.joint_topic, JointState, self.joint_cb)

        rospy.loginfo(
            "Tracking error monitor started, expected samples: %d, eval_start_delay=%.2fs, eval_start_min_z=%.3fm",
            self.expected_samples,
            self.eval_start_delay,
            self.eval_start_min_z,
        )

    def reference_cb(self, msg):
        point = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ], dtype=float)
        self.reference_buffer.append((msg.header.stamp, point))
        self.last_ref_stamp = msg.header.stamp
        if self.first_ref_point is None:
            self.first_ref_stamp = msg.header.stamp
            self.first_ref_point = point.copy()

    def should_start_evaluation(self, stamp, actual_pos):
        if self.first_ref_point is None:
            return False

        delay_ok = stamp >= self.first_ref_stamp + rospy.Duration.from_sec(self.eval_start_delay)
        z_ok = actual_pos[2] >= self.eval_start_min_z
        return delay_ok and z_ok

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

    def pop_matching_reference(self, stamp):
        if not self.reference_buffer:
            return None

        best_idx = None
        best_dt = None
        best_point = None

        for idx, (ref_stamp, ref_point) in enumerate(self.reference_buffer):
            dt = abs((ref_stamp - stamp).to_sec())
            if best_dt is None or dt < best_dt:
                best_dt = dt
                best_idx = idx
                best_point = ref_point

        if best_dt is None or best_dt > self.match_tolerance:
            return None

        while len(self.reference_buffer) > best_idx + 1:
            self.reference_buffer.popleft()
        self.reference_buffer.popleft()
        return best_point

    def publish_actual_pose(self, stamp, T):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = "base_link"

        pos = tf_t.translation_from_matrix(T)
        quat = tf_t.quaternion_from_matrix(T)

        pose_msg.pose.position.x = float(pos[0])
        pose_msg.pose.position.y = float(pos[1])
        pose_msg.pose.position.z = float(pos[2])
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])

        self.actual_pose_pub.publish(pose_msg)

    def maybe_report_final(self):
        if not self.eval_started:
            return

        if self.reported_final:
            return

        enough_samples = self.match_count >= self.expected_samples
        timeout_after_last_ref = (
            self.last_ref_stamp != rospy.Time(0)
            and rospy.Time.now() > self.last_ref_stamp + rospy.Duration(0.5)
            and self.match_count > 0
        )

        if enough_samples or timeout_after_last_ref:
            mean_error = float(np.mean(self.error_samples))
            max_error = float(np.max(self.error_samples))
            rospy.loginfo("=" * 60)
            rospy.loginfo("UR5 end-effector tracking mean error: %.3f mm", mean_error * 1000.0)
            rospy.loginfo("UR5 end-effector tracking max  error: %.3f mm", max_error * 1000.0)
            rospy.loginfo("matched samples: %d", self.match_count)
            rospy.loginfo("=" * 60)
            self.reported_final = True

    def joint_cb(self, msg):
        try:
            joint_map = dict(zip(msg.name, msg.position))
            q = np.array([joint_map[name] for name in self.joint_names], dtype=float)
        except Exception:
            return

        T = self.fk(q)
        self.publish_actual_pose(msg.header.stamp, T)

        actual_pos = T[:3, 3]

        if not self.eval_started:
            if self.should_start_evaluation(msg.header.stamp, actual_pos):
                self.eval_started = True
                self.reference_buffer.clear()
                self.error_samples = []
                self.match_count = 0
                rospy.loginfo("Start tracking error evaluation after arm raised")
            return

        ref_point = self.pop_matching_reference(msg.header.stamp)
        if ref_point is None:
            self.maybe_report_final()
            return

        error = float(np.linalg.norm(actual_pos - ref_point))
        self.error_samples.append(error)
        self.match_count += 1

        if self.match_count % 100 == 0:
            mean_error = float(np.mean(self.error_samples))
            rospy.loginfo("tracking samples=%d, mean error=%.3f mm", self.match_count, mean_error * 1000.0)

        self.maybe_report_final()


if __name__ == "__main__":
    try:
        node = UR5RehabTrackingErrorMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
