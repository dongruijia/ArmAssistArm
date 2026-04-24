#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import WrenchStamped


class PeriodicWrenchPublisher:
    def __init__(self):
        rospy.init_node("ur5_periodic_wrench_publisher")

        self.topic = rospy.get_param("~topic", "/ur5/ft_sensor/command_wrench")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.rate_hz = float(rospy.get_param("~rate_hz", 100.0))

        self.axis = rospy.get_param("~axis", "x").lower()
        self.amplitude = float(rospy.get_param("~amplitude", 10.0))
        self.frequency = float(rospy.get_param("~frequency", 0.5))
        self.bias = float(rospy.get_param("~bias", 0.0))

        self.torque_axis = rospy.get_param("~torque_axis", "")
        self.torque_amplitude = float(rospy.get_param("~torque_amplitude", 0.0))
        self.torque_frequency = float(rospy.get_param("~torque_frequency", self.frequency))
        self.torque_bias = float(rospy.get_param("~torque_bias", 0.0))

        if self.axis not in ("x", "y", "z"):
            raise ValueError("~axis must be one of x/y/z")
        if self.torque_axis and self.torque_axis not in ("x", "y", "z"):
            raise ValueError("~torque_axis must be empty or one of x/y/z")

        self.pub = rospy.Publisher(self.topic, WrenchStamped, queue_size=20)

        rospy.loginfo("periodic wrench publisher started")
        rospy.loginfo("  topic: %s", self.topic)
        rospy.loginfo("  axis: %s amplitude: %.3f frequency: %.3f", self.axis, self.amplitude, self.frequency)

    @staticmethod
    def _set_axis(vec, axis, value):
        if axis == "x":
            vec[0] = value
        elif axis == "y":
            vec[1] = value
        elif axis == "z":
            vec[2] = value

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        t0 = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec() - t0
            force_value = self.bias + self.amplitude * math.sin(2.0 * math.pi * self.frequency * t)

            msg = WrenchStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame_id

            force = [0.0, 0.0, 0.0]
            self._set_axis(force, self.axis, force_value)
            msg.wrench.force.x = force[0]
            msg.wrench.force.y = force[1]
            msg.wrench.force.z = force[2]

            if self.torque_axis:
                torque_value = self.torque_bias + self.torque_amplitude * math.sin(
                    2.0 * math.pi * self.torque_frequency * t
                )
                torque = [0.0, 0.0, 0.0]
                self._set_axis(torque, self.torque_axis, torque_value)
                msg.wrench.torque.x = torque[0]
                msg.wrench.torque.y = torque[1]
                msg.wrench.torque.z = torque[2]

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
