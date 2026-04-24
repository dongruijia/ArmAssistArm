#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Empty


def _vec_sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def _vec_mul(a, b):
    return [a[0] * b[0], a[1] * b[1], a[2] * b[2]]


class FTWrenchCorrector:
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

        self.dynamic_force_bias = [0.0, 0.0, 0.0]
        self.dynamic_torque_bias = [0.0, 0.0, 0.0]

        self._tare_active = self.use_initial_tare
        self._tare_remaining = self.tare_sample_count
        self._tare_completed = not self.require_tare_before_publish and not self.use_initial_tare
        self._force_acc = [0.0, 0.0, 0.0]
        self._torque_acc = [0.0, 0.0, 0.0]

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

    def _raw_cb(self, msg):
        force_raw = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]
        torque_raw = [msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]

        self._update_tare(force_raw, torque_raw)

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

        force_corrected = _vec_mul(_vec_sub(force_raw, total_force_bias), self.force_scale)
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
