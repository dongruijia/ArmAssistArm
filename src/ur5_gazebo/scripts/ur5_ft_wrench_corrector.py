#!/usr/bin/env python3
"""对力传感器数据进行去皮、缩放、重力和偏置修正。"""

import numpy as np
import rospy
import tf.transformations as tf_t
import tf2_ros
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Empty


def _vec_sub(a, b):
    """三维向量逐元素相减。"""

    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def _vec_mul(a, b):
    """三维向量逐元素相乘。"""

    return [a[0] * b[0], a[1] * b[1], a[2] * b[2]]


class FTWrenchCorrector:
    """对原始六维力数据做去皮和标定修正后再发布。"""

    def __init__(self):
        rospy.init_node("ur5_ft_wrench_corrector")

        self.raw_topic = rospy.get_param("~raw_topic", "/ur5/ft_sensor/wrench")
        self.corrected_topic = rospy.get_param("~corrected_topic", "/ur5/ft_sensor/wrench_corrected")
        self.tare_topic = rospy.get_param("~tare_topic", "/ur5/ft_sensor/tare")

        self.use_initial_tare = bool(rospy.get_param("~use_initial_tare", True))
        self.tare_sample_count = int(rospy.get_param("~tare_sample_count", 500))
        self.require_tare_before_publish = bool(rospy.get_param("~require_tare_before_publish", True))

        self.static_force_bias = self._read_vec_param("~static_force_bias", [0.0, 0.0, 0.0])
        self.static_torque_bias = self._read_vec_param("~static_torque_bias", [0.0, 0.0, 0.0])
        self.force_scale = self._read_vec_param("~force_scale", [1.0, 1.0, 1.0])
        self.torque_scale = self._read_vec_param("~torque_scale", [1.0, 1.0, 1.0])
        self.enable_gravity_compensation = bool(
            rospy.get_param("~enable_gravity_compensation", False)
        )
        self.distal_mass_kg = float(rospy.get_param("~distal_mass_kg", 0.0))
        self.gravity_reference_frame = rospy.get_param("~gravity_reference_frame", "base_link")
        self.sensor_frame_fallback = rospy.get_param("~sensor_frame_fallback", "")
        self.gravity_tf_lookup_timeout_sec = float(
            rospy.get_param("~gravity_tf_lookup_timeout_sec", 0.02)
        )
        self.gravity_vector = np.array(
            self._read_vec_param("~gravity_vector", [0.0, 0.0, -9.81]), dtype=float
        )
        self.gravity_force_sign = np.array(
            self._read_vec_param("~gravity_force_sign", [1.0, 1.0, 1.0]), dtype=float
        )

        self.dynamic_force_bias = [0.0, 0.0, 0.0]
        self.dynamic_torque_bias = [0.0, 0.0, 0.0]

        self._tare_active = self.use_initial_tare
        self._tare_remaining = self.tare_sample_count
        self._tare_completed = not self.require_tare_before_publish and not self.use_initial_tare
        self._force_acc = [0.0, 0.0, 0.0]
        self._torque_acc = [0.0, 0.0, 0.0]
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.corrected_pub = rospy.Publisher(self.corrected_topic, WrenchStamped, queue_size=50)
        self.raw_sub = rospy.Subscriber(self.raw_topic, WrenchStamped, self._raw_cb, queue_size=200)
        self.tare_sub = rospy.Subscriber(self.tare_topic, Empty, self._tare_cb, queue_size=1)

        rospy.loginfo("ft wrench corrector started")
        rospy.loginfo("  raw_topic: %s", self.raw_topic)
        rospy.loginfo("  corrected_topic: %s", self.corrected_topic)
        rospy.loginfo("  tare_topic: %s", self.tare_topic)
        rospy.loginfo("  tare_sample_count: %d", self.tare_sample_count)
        rospy.loginfo("  require_tare_before_publish: %s", self.require_tare_before_publish)
        rospy.loginfo("  static_force_bias: %s", self.static_force_bias)
        rospy.loginfo("  static_torque_bias: %s", self.static_torque_bias)
        rospy.loginfo("  force_scale: %s", self.force_scale)
        rospy.loginfo("  torque_scale: %s", self.torque_scale)
        rospy.loginfo("  enable_gravity_compensation: %s", self.enable_gravity_compensation)
        if self.enable_gravity_compensation:
            rospy.loginfo("  distal_mass_kg: %.6f", self.distal_mass_kg)
            rospy.loginfo("  gravity_reference_frame: %s", self.gravity_reference_frame)
            rospy.loginfo("  sensor_frame_fallback: %s", self.sensor_frame_fallback)
            rospy.loginfo("  gravity_vector: %s", self.gravity_vector.tolist())
            rospy.loginfo("  gravity_force_sign: %s", self.gravity_force_sign.tolist())

    @staticmethod
    def _read_vec_param(name, default):
        val = rospy.get_param(name, default)
        if not isinstance(val, list) or len(val) != 3:
            rospy.logwarn("%s should be a 3-element list, fallback to %s", name, default)
            return list(default)
        return [float(val[0]), float(val[1]), float(val[2])]

    def _tare_cb(self, _msg):
        self._tare_active = True
        self._tare_remaining = self.tare_sample_count
        self._tare_completed = False
        self._force_acc = [0.0, 0.0, 0.0]
        self._torque_acc = [0.0, 0.0, 0.0]
        rospy.loginfo("tare requested: collecting %d samples", self.tare_sample_count)

    def _update_tare(self, force_raw, torque_raw):
        """在去皮阶段累积样本并更新动态偏置。"""

        if not self._tare_active:
            return

        self._force_acc[0] += force_raw[0]
        self._force_acc[1] += force_raw[1]
        self._force_acc[2] += force_raw[2]
        self._torque_acc[0] += torque_raw[0]
        self._torque_acc[1] += torque_raw[1]
        self._torque_acc[2] += torque_raw[2]
        self._tare_remaining -= 1

        if self._tare_remaining <= 0:
            n = float(max(self.tare_sample_count, 1))
            self.dynamic_force_bias = [self._force_acc[0] / n, self._force_acc[1] / n, self._force_acc[2] / n]
            self.dynamic_torque_bias = [self._torque_acc[0] / n, self._torque_acc[1] / n, self._torque_acc[2] / n]
            self._tare_active = False
            self._tare_completed = True
            rospy.loginfo("tare completed")
            rospy.loginfo("  dynamic_force_bias: %s", self.dynamic_force_bias)
            rospy.loginfo("  dynamic_torque_bias: %s", self.dynamic_torque_bias)

    def _gravity_force_bias(self, source_frame, stamp):
        if not self.enable_gravity_compensation or self.distal_mass_kg <= 0.0:
            return np.zeros(3, dtype=float)

        if not source_frame:
            source_frame = self.sensor_frame_fallback
        if not source_frame:
            return np.zeros(3, dtype=float)

        lookup_stamp = stamp if stamp != rospy.Time() else rospy.Time(0)
        timeout = rospy.Duration.from_sec(max(self.gravity_tf_lookup_timeout_sec, 0.0))
        try:
            transform = self.tf_buffer.lookup_transform(
                source_frame,
                self.gravity_reference_frame,
                lookup_stamp,
                timeout,
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            rospy.logwarn_throttle(
                1.0,
                "gravity compensation transform lookup failed: %s -> %s (%s)",
                self.gravity_reference_frame,
                source_frame,
                str(exc),
            )
            return np.zeros(3, dtype=float)

        quat = transform.transform.rotation
        rotation = tf_t.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])[:3, :3]
        gravity_force_ref = self.distal_mass_kg * self.gravity_vector
        gravity_force_sensor = rotation.dot(gravity_force_ref)
        return gravity_force_sensor * self.gravity_force_sign

    def _raw_cb(self, msg):
        force_raw = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z], dtype=float)
        torque_raw = [msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
        force_gravity_bias = self._gravity_force_bias(msg.header.frame_id, msg.header.stamp)
        force_input = (force_raw - force_gravity_bias).tolist()

        self._update_tare(force_input, torque_raw)

        # 注意：若要求先去皮，完成前不会输出修正后的力数据。
        if self.require_tare_before_publish and not self._tare_completed:
            return

        total_force_bias = [
            self.static_force_bias[0] + self.dynamic_force_bias[0],
            self.static_force_bias[1] + self.dynamic_force_bias[1],
            self.static_force_bias[2] + self.dynamic_force_bias[2],
        ]
        total_torque_bias = [
            self.static_torque_bias[0] + self.dynamic_torque_bias[0],
            self.static_torque_bias[1] + self.dynamic_torque_bias[1],
            self.static_torque_bias[2] + self.dynamic_torque_bias[2],
        ]

        force_corrected = _vec_mul(_vec_sub(force_input, total_force_bias), self.force_scale)
        torque_corrected = _vec_mul(_vec_sub(torque_raw, total_torque_bias), self.torque_scale)

        out = WrenchStamped()
        out.header = msg.header
        out.wrench.force.x = force_corrected[0]
        out.wrench.force.y = force_corrected[1]
        out.wrench.force.z = force_corrected[2]
        out.wrench.torque.x = torque_corrected[0]
        out.wrench.torque.y = torque_corrected[1]
        out.wrench.torque.z = torque_corrected[2]

        self.corrected_pub.publish(out)


if __name__ == "__main__":
    try:
        node = FTWrenchCorrector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
