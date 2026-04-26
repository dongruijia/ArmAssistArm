#!/usr/bin/env python3
"""发布分段式外力，用于演示施力与撤力过程。"""

import ast
import math

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, WrenchStamped


class PulseWrenchPublisher:
    """按固定节拍发布单轴脉冲，或按 mode2 轨迹方向发布四段循环脉冲。"""

    def __init__(self):
        rospy.init_node("ur5_pulse_wrench_publisher")

        self.topic = rospy.get_param("~topic", "/ur5/ft_sensor/command_wrench")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.rate_hz = float(rospy.get_param("~rate_hz", 100.0))
        self.pattern = rospy.get_param("~pattern", "single_axis").lower()

        self.axis = rospy.get_param("~axis", "x").lower()
        self.amplitude = float(rospy.get_param("~amplitude", 1.0))
        self.bias = float(rospy.get_param("~bias", 0.0))
        self.waveform = rospy.get_param("~waveform", "square").lower()
        self.frequency = float(rospy.get_param("~frequency", 0.5))
        self.periods_per_burst = float(rospy.get_param("~periods_per_burst", 2.0))

        self.start_delay = float(rospy.get_param("~start_delay", 2.0))
        self.on_duration = float(rospy.get_param("~on_duration", 2.0))
        self.off_duration = float(rospy.get_param("~off_duration", 2.0))
        self.interval_duration = float(rospy.get_param("~interval_duration", self.off_duration))
        self.cycles = int(rospy.get_param("~cycles", 3))
        self.publish_zero_at_end = bool(rospy.get_param("~publish_zero_at_end", True))

        self.trajectory_topic = rospy.get_param("~trajectory_topic", "/ur5/active_rehab/trajectory_reference")
        self.circle_center = self._parse_vector_param(
            rospy.get_param("~circle_center", [0.0, 0.5, 0.5]),
            "circle_center",
        )
        self.circle_plane = rospy.get_param("~circle_plane", "yz").lower()
        self.reference_timeout = float(rospy.get_param("~reference_timeout", 0.5))
        self.latest_reference_position = None
        self.latest_reference_stamp = rospy.Time(0)

        if self.axis not in ("x", "y", "z"):
            raise ValueError("~axis must be one of x/y/z")
        if self.pattern not in ("single_axis", "mode2_cycle"):
            raise ValueError("~pattern must be one of single_axis/mode2_cycle")
        if self.waveform not in ("square", "sine"):
            raise ValueError("~waveform must be one of square/sine")
        if self.on_duration <= 0.0:
            raise ValueError("~on_duration must be > 0")
        if self.off_duration < 0.0:
            raise ValueError("~off_duration must be >= 0")
        if self.interval_duration < 0.0:
            raise ValueError("~interval_duration must be >= 0")
        if self.frequency <= 0.0:
            raise ValueError("~frequency must be > 0")
        if self.periods_per_burst <= 0.0:
            raise ValueError("~periods_per_burst must be > 0")
        if self.circle_center.shape != (3,):
            raise ValueError("~circle_center must be a 3-element list")
        if self.circle_plane not in ("xy", "yz", "xz"):
            raise ValueError("~circle_plane must be one of xy/yz/xz")

        self.pub = rospy.Publisher(self.topic, WrenchStamped, queue_size=20)
        if self.pattern == "mode2_cycle":
            rospy.Subscriber(self.trajectory_topic, PoseStamped, self.trajectory_cb, queue_size=20)

        rospy.loginfo("pulse wrench publisher started")
        rospy.loginfo("  topic: %s", self.topic)
        rospy.loginfo("  pattern: %s", self.pattern)
        if self.pattern == "single_axis":
            rospy.loginfo(
                "  axis: %s amplitude: %.3f waveform: %s",
                self.axis,
                self.amplitude,
                self.waveform,
            )
        else:
            rospy.loginfo("  mode2 trajectory_topic: %s", self.trajectory_topic)
            rospy.loginfo("  mode2 circle_plane: %s", self.circle_plane)
            rospy.loginfo("  mode2 circle_center: %s", self.circle_center.tolist())
        if self.pattern == "single_axis" and self.waveform == "sine":
            rospy.loginfo(
                "  frequency: %.3f periods_per_burst: %.3f burst_duration: %.3fs first_direction_sign: %.1f",
                self.frequency,
                self.periods_per_burst,
                self._active_duration(),
                self._base_direction_sign(),
            )
        rospy.loginfo(
            "  start_delay: %.3f active_duration: %.3f off_duration: %.3f interval_duration: %.3f cycles: %s",
            self.start_delay,
            self._active_duration(),
            self.off_duration,
            self.interval_duration,
            self._cycles_label(),
        )

    @staticmethod
    def _parse_vector_param(value, name):
        if isinstance(value, str):
            text = value.strip()
            try:
                value = ast.literal_eval(text)
            except (SyntaxError, ValueError):
                value = [part.strip() for part in text.strip("[]").split(",") if part.strip()]

        vector = np.array(value, dtype=float).reshape(-1)
        if vector.shape != (3,):
            raise ValueError("~{} must be a 3-element list".format(name))
        return vector

    def _cycles_label(self):
        if self.cycles <= 0:
            return "unlimited"
        return str(self.cycles)

    @staticmethod
    def _set_axis(msg, axis, value):
        if axis == "x":
            msg.wrench.force.x = value
        elif axis == "y":
            msg.wrench.force.y = value
        else:
            msg.wrench.force.z = value

    @staticmethod
    def _set_force_vector(msg, force):
        msg.wrench.force.x = float(force[0])
        msg.wrench.force.y = float(force[1])
        msg.wrench.force.z = float(force[2])

    def _publish_force(self, value):
        msg = WrenchStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        if np.isscalar(value):
            self._set_axis(msg, self.axis, float(value))
        else:
            force = np.array(value, dtype=float).reshape(-1)
            if force.size == 1:
                self._set_axis(msg, self.axis, float(force[0]))
            elif force.size == 3:
                self._set_force_vector(msg, force)
            else:
                raise ValueError("force vector must be scalar or length 3")
        self.pub.publish(msg)

    def trajectory_cb(self, msg):
        self.latest_reference_position = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            dtype=float,
        )
        if msg.header.stamp.to_sec() > 0.0:
            self.latest_reference_stamp = msg.header.stamp
        else:
            self.latest_reference_stamp = rospy.Time.now()

    def _active_duration(self):
        if self.pattern == "single_axis" and self.waveform == "sine":
            return self.periods_per_burst / self.frequency
        return self.on_duration

    def _base_direction_sign(self):
        return -1.0 if self.amplitude < 0.0 else 1.0

    def _burst_direction_sign(self, cycle_index):
        if self.waveform != "sine":
            return self._base_direction_sign()

        phase = (cycle_index - 1) % 4
        if phase in (0, 3):
            return self._base_direction_sign()
        return -self._base_direction_sign()

    def _sine_burst_force(self, phase_time, cycle_index):
        burst_direction = self._burst_direction_sign(cycle_index)
        magnitude = abs(self.amplitude) * 0.5 * (
            1.0 - math.cos(2.0 * math.pi * self.frequency * phase_time)
        )
        return self.bias + burst_direction * magnitude

    def _has_fresh_reference(self):
        if self.latest_reference_position is None:
            return False
        age = (rospy.Time.now() - self.latest_reference_stamp).to_sec()
        return age <= self.reference_timeout

    def _compute_mode2_directions(self):
        if not self._has_fresh_reference():
            return None, None

        radial = self.latest_reference_position - self.circle_center
        if self.circle_plane == "xy":
            radial[2] = 0.0
        elif self.circle_plane == "yz":
            radial[0] = 0.0
        else:
            radial[1] = 0.0

        radial_norm = np.linalg.norm(radial)
        if radial_norm < 1e-9:
            return None, None

        radial_unit = radial / radial_norm
        if self.circle_plane == "xy":
            tangent_unit = np.array([-radial_unit[1], radial_unit[0], 0.0], dtype=float)
        elif self.circle_plane == "yz":
            tangent_unit = np.array([0.0, -radial_unit[2], radial_unit[1]], dtype=float)
        else:
            tangent_unit = np.array([-radial_unit[2], 0.0, radial_unit[0]], dtype=float)
        return radial_unit, tangent_unit

    def _mode2_plane_normal(self):
        if self.circle_plane == "xy":
            return np.array([0.0, 0.0, 1.0], dtype=float)
        if self.circle_plane == "yz":
            return np.array([1.0, 0.0, 0.0], dtype=float)
        return np.array([0.0, 1.0, 0.0], dtype=float)

    @staticmethod
    def _mode2_step_name(step_index):
        step_names = (
            "normal_positive",
            "tangent_forward",
            "normal_negative",
            "tangent_backward",
        )
        return step_names[step_index - 1]

    def _mode2_step_force(self, step_index):
        _radial_unit, tangent_unit = self._compute_mode2_directions()
        if tangent_unit is None:
            return np.zeros(3), "waiting_reference"

        normal_unit = self._mode2_plane_normal()
        directions = (
            normal_unit,
            tangent_unit,
            -normal_unit,
            -tangent_unit,
        )
        return abs(self.amplitude) * directions[step_index - 1], self._mode2_step_name(step_index)

    def _phase_value(self, elapsed):
        if elapsed < self.start_delay:
            return np.zeros(3), "delay", 0, 0, "delay"

        if self.pattern == "mode2_cycle":
            active_time = elapsed - self.start_delay
            active_duration = self._active_duration()
            step_period = active_duration + self.interval_duration
            if step_period <= 0.0:
                return np.zeros(3), "finished", self.cycles, 0, "finished"

            steps_per_cycle = 4
            cycle_period = steps_per_cycle * step_period
            if self.cycles > 0 and active_time >= self.cycles * cycle_period:
                return np.zeros(3), "finished", self.cycles, 0, "finished"

            cycle_index = int(active_time / cycle_period) if cycle_period > 0.0 else 0
            cycle_time = active_time - cycle_index * cycle_period
            step_index = int(cycle_time / step_period) + 1
            phase_time = cycle_time - (step_index - 1) * step_period

            if phase_time < active_duration:
                force_vector, step_name = self._mode2_step_force(step_index)
                if step_name == "waiting_reference":
                    return force_vector, "waiting_reference", cycle_index + 1, step_index, step_name
                return force_vector, "on", cycle_index + 1, step_index, step_name
            return np.zeros(3), "off", cycle_index + 1, step_index, self._mode2_step_name(step_index)

        active_time = elapsed - self.start_delay
        active_duration = self._active_duration()
        period = active_duration + self.interval_duration
        if period <= 0.0:
            return np.zeros(3), "finished", self.cycles, 0, "finished"

        if self.cycles > 0 and active_time >= self.cycles * period:
            return np.zeros(3), "finished", self.cycles, 0, "finished"

        cycle_index = int(active_time / period) if period > 0.0 else 0
        phase_time = active_time - cycle_index * period
        if phase_time < active_duration:
            if self.waveform == "sine":
                force_value = self._sine_burst_force(phase_time, cycle_index + 1)
                return force_value, "on", cycle_index + 1, 1, self.axis
            return self.bias + self.amplitude, "on", cycle_index + 1, 1, self.axis
        return 0.0, "off", cycle_index + 1, 1, self.axis

    def _wait_for_mode2_reference(self):
        if self.pattern != "mode2_cycle":
            return

        rate = rospy.Rate(max(self.rate_hz, 1.0))
        while not rospy.is_shutdown() and not self._has_fresh_reference():
            rospy.logwarn_throttle(
                2.0,
                "Waiting for fresh trajectory reference on %s before starting mode2 pulse cycle",
                self.trajectory_topic,
            )
            self._publish_force(np.zeros(3))
            rate.sleep()

    def run(self):
        rate = rospy.Rate(max(self.rate_hz, 1.0))
        self._wait_for_mode2_reference()
        t0 = rospy.Time.now().to_sec()
        last_phase = None
        last_cycle = -1
        last_step = -1

        while not rospy.is_shutdown():
            elapsed = rospy.Time.now().to_sec() - t0
            force_value, phase, cycle_index, step_index, step_name = self._phase_value(elapsed)

            if phase != last_phase or cycle_index != last_cycle or step_index != last_step:
                if phase == "delay":
                    rospy.loginfo("pulse phase: waiting %.2fs before first force pulse", self.start_delay)
                elif phase == "on":
                    if self.pattern == "mode2_cycle":
                        force_vector = np.array(force_value, dtype=float).reshape(3)
                        rospy.loginfo(
                            "pulse phase: cycle %d step %d ON (%s), force=(%.3f, %.3f, %.3f) N",
                            cycle_index,
                            step_index,
                            step_name,
                            force_vector[0],
                            force_vector[1],
                            force_vector[2],
                        )
                    else:
                        rospy.loginfo(
                            "pulse phase: cycle %d ON, force=%.3f N",
                            cycle_index,
                            float(force_value),
                        )
                elif phase == "off":
                    if self.pattern == "mode2_cycle":
                        rospy.loginfo(
                            "pulse phase: cycle %d step %d OFF (%s)",
                            cycle_index,
                            step_index,
                            step_name,
                        )
                    else:
                        rospy.loginfo("pulse phase: cycle %d OFF", cycle_index)
                elif phase == "waiting_reference":
                    rospy.logwarn(
                        "pulse phase: cycle %d step %d waiting for fresh trajectory reference",
                        cycle_index,
                        step_index,
                    )
                else:
                    rospy.loginfo("pulse phase: finished")
                last_phase = phase
                last_cycle = cycle_index
                last_step = step_index

            if phase == "finished":
                if self.publish_zero_at_end:
                    self._publish_force(np.zeros(3))
                break

            self._publish_force(force_value)
            rate.sleep()


if __name__ == "__main__":
    try:
        node = PulseWrenchPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("pulse wrench publisher failed: %s", str(exc))
