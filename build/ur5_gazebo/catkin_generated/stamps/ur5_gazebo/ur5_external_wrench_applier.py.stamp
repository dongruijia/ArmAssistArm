#!/usr/bin/env python3

import threading

import rospy
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest
from geometry_msgs.msg import WrenchStamped


class ExternalWrenchApplier:
    def __init__(self):
        rospy.init_node("ur5_external_wrench_applier")

        self.body_name = rospy.get_param("~body_name", "robot::wrist_3_link")
        self.command_topic = rospy.get_param("~command_topic", "/ur5/ft_sensor/command_wrench")
        self.reference_frame = rospy.get_param("~reference_frame", "world")
        self.use_msg_frame = bool(rospy.get_param("~use_msg_frame", False))
        self.apply_rate_hz = float(rospy.get_param("~apply_rate_hz", 100.0))
        self.command_timeout = float(rospy.get_param("~command_timeout", 0.3))

        self._lock = threading.Lock()
        self._last_cmd = WrenchStamped()
        self._last_cmd_time = rospy.Time(0)

        rospy.wait_for_service("/gazebo/apply_body_wrench")
        self.apply_wrench = rospy.ServiceProxy("/gazebo/apply_body_wrench", ApplyBodyWrench)

        self.command_sub = rospy.Subscriber(
            self.command_topic, WrenchStamped, self._command_cb, queue_size=10
        )

        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.apply_rate_hz, 1.0)), self._on_timer)

        rospy.loginfo("external wrench applier started")
        rospy.loginfo("  body_name: %s", self.body_name)
        rospy.loginfo("  command_topic: %s", self.command_topic)
        rospy.loginfo("  reference_frame: %s", self.reference_frame)
        rospy.loginfo("  apply_rate_hz: %.1f", self.apply_rate_hz)

    def _command_cb(self, msg):
        with self._lock:
            self._last_cmd = msg
            self._last_cmd_time = rospy.Time.now()

    def _on_timer(self, _event):
        with self._lock:
            now = rospy.Time.now()
            is_active = (now - self._last_cmd_time).to_sec() <= self.command_timeout
            if is_active:
                cmd = self._last_cmd
            else:
                cmd = WrenchStamped()

        request = ApplyBodyWrenchRequest()
        request.body_name = self.body_name

        if self.use_msg_frame and cmd.header.frame_id:
            request.reference_frame = cmd.header.frame_id
        else:
            request.reference_frame = self.reference_frame

        request.wrench = cmd.wrench
        request.start_time = rospy.Time(0)
        request.duration = rospy.Duration(1.0 / max(self.apply_rate_hz, 1.0))

        try:
            response = self.apply_wrench(request)
            if not response.success:
                rospy.logwarn_throttle(1.0, "apply_body_wrench rejected: %s", response.status_message)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(1.0, "apply_body_wrench call failed: %s", str(exc))


if __name__ == "__main__":
    try:
        node = ExternalWrenchApplier()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
