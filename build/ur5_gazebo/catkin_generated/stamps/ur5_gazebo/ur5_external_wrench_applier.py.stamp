#!/usr/bin/env python3
"""通过 Gazebo 服务向机械臂指定连杆持续施加外力。"""

import threading

import rospy
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest
from geometry_msgs.msg import WrenchStamped


class ExternalWrenchApplier:
    """把输入力指令转换成 Gazebo 连续施力请求。"""

    def __init__(self):
        rospy.init_node("ur5_external_wrench_applier")

        self.body_name = rospy.get_param("~body_name", "robot::wrist_3_link")
        self.command_topic = rospy.get_param("~command_topic", "/ur5/ft_sensor/command_wrench")
        self.reference_frame = rospy.get_param("~reference_frame", "world")
        self.use_msg_frame = bool(rospy.get_param("~use_msg_frame", False))
        self.apply_rate_hz = float(rospy.get_param("~apply_rate_hz", 100.0))
        self.command_timeout = float(rospy.get_param("~command_timeout", 0.3))
        self.timeout_hold_cycles = int(rospy.get_param("~timeout_hold_cycles", 2))
        self.wrench_duration = float(
            rospy.get_param("~wrench_duration", 1.5 / max(self.apply_rate_hz, 1.0))
        )

        if self.wrench_duration <= 0.0:
            raise ValueError("~wrench_duration must be positive")
        if self.command_timeout < self.wrench_duration:
            rospy.logwarn(
                "command_timeout %.3f is shorter than wrench_duration %.3f; raising timeout to avoid premature zeroing",
                self.command_timeout,
                self.wrench_duration,
            )
            self.command_timeout = self.wrench_duration

        self._lock = threading.Lock()
        self._last_cmd = WrenchStamped()
        self._last_cmd_time = rospy.Time(0)
        self._timeout_miss_count = 0

        rospy.wait_for_service("/gazebo/apply_body_wrench")
        self.apply_wrench = rospy.ServiceProxy(
            "/gazebo/apply_body_wrench", ApplyBodyWrench, persistent=True
        )

        self.command_sub = rospy.Subscriber(
            self.command_topic, WrenchStamped, self._command_cb, queue_size=10
        )

        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.apply_rate_hz, 1.0)), self._on_timer)

        rospy.loginfo("external wrench applier started")
        rospy.loginfo("  body_name: %s", self.body_name)
        rospy.loginfo("  command_topic: %s", self.command_topic)
        rospy.loginfo("  reference_frame: %s", self.reference_frame)
        rospy.loginfo("  apply_rate_hz: %.1f", self.apply_rate_hz)
        rospy.loginfo("  command_timeout: %.3f", self.command_timeout)
        rospy.loginfo("  wrench_duration: %.3f", self.wrench_duration)
        rospy.loginfo("  timeout_hold_cycles: %d", self.timeout_hold_cycles)

    def _command_cb(self, msg):
        with self._lock:
            self._last_cmd = msg
            self._last_cmd_time = rospy.Time.now()
            self._timeout_miss_count = 0

    def _on_timer(self, _event):
        """按固定频率重复施力，超时后自动回落为零外力。"""

        with self._lock:
            now = rospy.Time.now()
            is_active = (now - self._last_cmd_time).to_sec() <= self.command_timeout
            # 注意：Gazebo 的施力持续时间较短，因此这里用定时器重复刷新。
            if is_active:
                cmd = self._last_cmd
                self._timeout_miss_count = 0
            elif (
                self.timeout_hold_cycles > 0
                and self._last_cmd_time.to_sec() > 0.0
                and self._timeout_miss_count < self.timeout_hold_cycles
            ):
                # 对短时通信抖动做零阶保持，避免在正弦激励下出现非物理归零尖峰。
                cmd = self._last_cmd
                self._timeout_miss_count += 1
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
        request.duration = rospy.Duration(self.wrench_duration)

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
