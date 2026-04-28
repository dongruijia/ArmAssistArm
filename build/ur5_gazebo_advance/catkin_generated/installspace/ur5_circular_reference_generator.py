#!/usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped


class UR5CircularReferenceGenerator:
    def __init__(self):
        rospy.init_node("ur5_circular_reference_generator")

        self.rate_hz = float(rospy.get_param("~rate_hz", 100.0))
        self.topic = rospy.get_param("~topic", "/ur5/active_rehab/trajectory_reference")
        self.frame_id = rospy.get_param("~frame_id", "base_link")
        self.start_delay = float(rospy.get_param("~start_delay", 0.5))

        self.center = np.array(rospy.get_param("~center", [0.0, 0.5, 0.5]), dtype=float)
        self.radius = float(rospy.get_param("~radius", 0.1))
        self.duration = float(rospy.get_param("~duration", 8.0))
        self.loop = bool(rospy.get_param("~loop", True))
        self.plane = rospy.get_param("~plane", "yz").lower()

        orientation_quat = rospy.get_param("~orientation_quat", [0.0, 1.0, 0.0, 0.0])
        self.orientation_quat = np.array(orientation_quat, dtype=float)
        norm = np.linalg.norm(self.orientation_quat)
        if norm < 1e-12:
            self.orientation_quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        else:
            self.orientation_quat /= norm

        if self.plane not in ("xy", "yz", "xz"):
            raise ValueError("~plane must be xy, yz, or xz")

        self.pub = rospy.Publisher(self.topic, PoseStamped, queue_size=10)

        rospy.loginfo("circular reference generator started")
        rospy.loginfo("  topic: %s", self.topic)
        rospy.loginfo("  plane: %s", self.plane)
        rospy.loginfo("  center: %s", self.center.tolist())
        rospy.loginfo("  radius: %.3f", self.radius)
        rospy.loginfo("  duration: %.2f s", self.duration)

    def _point_on_circle(self, elapsed):
        if self.duration <= 0.0:
            theta = 0.0
        else:
            theta = 2.0 * np.pi * (elapsed / self.duration)

        point = self.center.copy()
        if self.plane == "xy":
            point[0] += self.radius * np.cos(theta)
            point[1] += self.radius * np.sin(theta)
        elif self.plane == "yz":
            point[1] += self.radius * np.cos(theta)
            point[2] += self.radius * np.sin(theta)
        else:
            point[0] += self.radius * np.cos(theta)
            point[2] += self.radius * np.sin(theta)
        return point

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        rospy.sleep(self.start_delay)
        start_time = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            elapsed = rospy.Time.now().to_sec() - start_time
            if elapsed > self.duration:
                if self.loop and self.duration > 0.0:
                    elapsed = elapsed % self.duration
                else:
                    elapsed = self.duration

            point = self._point_on_circle(elapsed)

            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = float(point[2])
            pose.pose.orientation.x = float(self.orientation_quat[0])
            pose.pose.orientation.y = float(self.orientation_quat[1])
            pose.pose.orientation.z = float(self.orientation_quat[2])
            pose.pose.orientation.w = float(self.orientation_quat[3])

            self.pub.publish(pose)
            rate.sleep()


if __name__ == "__main__":
    try:
        node = UR5CircularReferenceGenerator()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("circular reference generator failed: %s", str(exc))
