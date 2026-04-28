#!/usr/bin/env python3
"""发布外力信号（周期/恒力），用于导纳控制测试。"""

import math

import rospy
from geometry_msgs.msg import WrenchStamped


class PeriodicWrenchPublisher:
    """生成正弦或恒定形式的力/力矩激励信号。"""

    def __init__(self):
        rospy.init_node("ur5_periodic_wrench_publisher")

        self.topic = rospy.get_param("~topic", "/ur5/ft_sensor/command_wrench")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.rate_hz = float(rospy.get_param("~rate_hz", 100.0))
        self.mode = rospy.get_param("~mode", "periodic").lower()

        self.axis = rospy.get_param("~axis", "x").lower()
        self.amplitude = float(rospy.get_param("~amplitude", 10.0))
        self.frequency = float(rospy.get_param("~frequency", 0.5))
        self.bias = float(rospy.get_param("~bias", 0.0))
        self.constant_value = float(rospy.get_param("~constant_value", self.bias + self.amplitude))

        self.torque_axis = rospy.get_param("~torque_axis", "")
        self.torque_amplitude = float(rospy.get_param("~torque_amplitude", 0.0))
        self.torque_frequency = float(rospy.get_param("~torque_frequency", self.frequency))
        self.torque_bias = float(rospy.get_param("~torque_bias", 0.0))
        self.torque_constant_value = float(
            rospy.get_param("~torque_constant_value", self.torque_bias + self.torque_amplitude)
        )
        self.latch = bool(rospy.get_param("~latch", self.mode == "constant"))

        if self.mode not in ("periodic", "constant"):
            raise ValueError("~mode must be one of periodic/constant")

        if self.axis not in ("x", "y", "z"):
            raise ValueError("~axis must be one of x/y/z")
        if self.torque_axis and self.torque_axis not in ("x", "y", "z"):
            raise ValueError("~torque_axis must be empty or one of x/y/z")

        self.pub = rospy.Publisher(self.topic, WrenchStamped, queue_size=20, latch=self.latch)

        rospy.loginfo("periodic wrench publisher started")
        rospy.loginfo("  topic: %s", self.topic)
        rospy.loginfo("  mode: %s axis: %s", self.mode, self.axis)
        rospy.loginfo("  latch: %s", self.latch)
        if self.mode == "periodic":
            rospy.loginfo("  periodic force: amplitude=%.3f frequency=%.3f bias=%.3f", self.amplitude, self.frequency, self.bias)
        else:
            rospy.loginfo("  constant force: value=%.3f", self.constant_value)

    @staticmethod
    def _set_axis(vec, axis, value):
        """按轴名称将标量写入三维向量。"""

        if axis == "x":
            vec[0] = value
        elif axis == "y":
            vec[1] = value
        elif axis == "z":
            vec[2] = value

    def _compute_force_value(self, t):
        """根据当前模式计算力值。"""

        if self.mode == "constant":
            return self.constant_value
        return self.bias + self.amplitude * math.sin(2.0 * math.pi * self.frequency * t)

    def _compute_torque_value(self, t):
        """根据当前模式计算力矩值。"""

        if self.mode == "constant":
            return self.torque_constant_value
        return self.torque_bias + self.torque_amplitude * math.sin(2.0 * math.pi * self.torque_frequency * t)

    def _build_wrench_msg(self, force_value, torque_value=None):
        """构造一帧扳手消息，避免常量模式下反复拼装相同载荷。"""

        msg = WrenchStamped()
        msg.header.frame_id = self.frame_id

        force = [0.0, 0.0, 0.0]
        self._set_axis(force, self.axis, force_value)
        msg.wrench.force.x = force[0]
        msg.wrench.force.y = force[1]
        msg.wrench.force.z = force[2]

        if self.torque_axis and torque_value is not None:
            torque = [0.0, 0.0, 0.0]
            self._set_axis(torque, self.torque_axis, torque_value)
            msg.wrench.torque.x = torque[0]
            msg.wrench.torque.y = torque[1]
            msg.wrench.torque.z = torque[2]

        return msg

    def run(self):
        """按设定频率持续发布外力/力矩扳手信号。"""

        rate = rospy.Rate(self.rate_hz)
        t0 = rospy.Time.now().to_sec()
        constant_msg = None

        if self.mode == "constant":
            torque_value = self._compute_torque_value(0.0) if self.torque_axis else None
            constant_msg = self._build_wrench_msg(self.constant_value, torque_value)

        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec() - t0
            if self.mode == "constant":
                msg = constant_msg
            else:
                force_value = self._compute_force_value(t)
                torque_value = self._compute_torque_value(t) if self.torque_axis else None
                msg = self._build_wrench_msg(force_value, torque_value)

            msg.header.stamp = rospy.Time.now()

            self.pub.publish(msg)
            rate.sleep()


if __name__ == "__main__":
    try:
        node = PeriodicWrenchPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("periodic wrench publisher failed: %s", str(exc))
