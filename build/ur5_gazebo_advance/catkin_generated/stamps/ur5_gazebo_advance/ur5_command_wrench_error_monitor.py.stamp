#!/usr/bin/env python3
"""监测六维力控制发布与修正接收之间的相对误差并打印到终端。"""

import numpy as np
import rospy
import tf.transformations as tf_t
import tf2_ros
from geometry_msgs.msg import WrenchStamped


class CommandWrenchErrorMonitor:
    """比较发送与接收的力三维模长并输出相对误差。"""

    def __init__(self):
        rospy.init_node("ur5_command_wrench_error_monitor")

        self.command_topic = rospy.get_param("~command_topic", "/ur5/ft_sensor/command_wrench")
        self.wrench_topic = rospy.get_param("~wrench_topic", "/ur5/ft_sensor/wrench_corrected")
        self.compare_frame = str(rospy.get_param("~compare_frame", "base_link"))
        self.transform_timeout = float(rospy.get_param("~transform_timeout", 0.02))
        self.print_rate_hz = float(rospy.get_param("~print_rate_hz", 5.0))
        self.epsilon = float(rospy.get_param("~epsilon", 1e-6))
        self.active_threshold = float(rospy.get_param("~active_threshold", 1e-3))
        self.latest_command_vec = None
        self.latest_command_frame = ""
        self.latest_command_stamp = rospy.Time(0)
        self.latest_wrench_vec = None
        self.latest_wrench_frame = ""
        self.latest_wrench_stamp = rospy.Time(0)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.command_sub = rospy.Subscriber(self.command_topic, WrenchStamped, self.command_cb, queue_size=50)
        self.wrench_sub = rospy.Subscriber(self.wrench_topic, WrenchStamped, self.wrench_cb, queue_size=50)

        rospy.loginfo("command-wrench error monitor started")
        rospy.loginfo("  command_topic: %s", self.command_topic)
        rospy.loginfo("  wrench_topic: %s", self.wrench_topic)
        rospy.loginfo("  compare_frame: %s", self.compare_frame)
        rospy.loginfo("  print_rate_hz: %.2f", self.print_rate_hz)
        rospy.loginfo("  active_threshold: %.6f", self.active_threshold)

    @staticmethod
    def wrench_to_vector(msg):
        return np.array(
            [
                msg.wrench.force.x,
                msg.wrench.force.y,
                msg.wrench.force.z,
                msg.wrench.torque.x,
                msg.wrench.torque.y,
                msg.wrench.torque.z,
            ],
            dtype=float,
        )

    def command_cb(self, msg):
        self.latest_command_vec = self.wrench_to_vector(msg)
        self.latest_command_frame = msg.header.frame_id.strip() if msg.header.frame_id else ""
        self.latest_command_stamp = msg.header.stamp

    def wrench_cb(self, msg):
        self.latest_wrench_vec = self.wrench_to_vector(msg)
        self.latest_wrench_frame = msg.header.frame_id.strip() if msg.header.frame_id else ""
        self.latest_wrench_stamp = msg.header.stamp

    def _transform_force_to_compare_frame(self, force_vec, source_frame, stamp):
        if not source_frame:
            return force_vec.copy(), False
        if source_frame == self.compare_frame:
            return force_vec.copy(), True

        query_time = stamp
        if query_time.to_sec() == 0.0:
            query_time = rospy.Time(0)

        try:
            transform = self.tf_buffer.lookup_transform(
                self.compare_frame,
                source_frame,
                query_time,
                rospy.Duration.from_sec(self.transform_timeout),
            )
            q = transform.transform.rotation
            R = tf_t.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
            out = force_vec.copy()
            out[:3] = R @ force_vec[:3]
            out[3:] = R @ force_vec[3:]
            return out, True
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
            tf2_ros.TransformException,
        ) as exc:
            rospy.logwarn_throttle(
                1.0,
                "monitor tf failed from %s to %s: %s",
                source_frame,
                self.compare_frame,
                str(exc),
            )
            return force_vec.copy(), False

    def run(self):
        rate = rospy.Rate(self.print_rate_hz)
        while not rospy.is_shutdown():
            if self.latest_command_vec is None or self.latest_wrench_vec is None:
                rospy.logwarn_throttle(2.0, "Waiting for both command and wrench messages")
                rate.sleep()
                continue

            cmd = self.latest_command_vec
            wr = self.latest_wrench_vec
            cmd_frame = self.latest_command_frame
            wr_frame = self.latest_wrench_frame
            cmd_stamp = self.latest_command_stamp
            wr_stamp = self.latest_wrench_stamp

            cmd_cmp, cmd_ok = self._transform_force_to_compare_frame(cmd, cmd_frame, cmd_stamp)
            wr_cmp, wr_ok = self._transform_force_to_compare_frame(wr, wr_frame, wr_stamp)

            if cmd_ok and wr_ok:
                cmd_eval = cmd_cmp
                wr_eval = wr_cmp
                active_frame = self.compare_frame
            else:
                cmd_eval = cmd
                wr_eval = wr
                active_frame = "raw"

            cmd_force_norm = float(np.sqrt(np.sum(np.square(cmd_eval[:3]))))
            recv_force_norm = float(np.sqrt(np.sum(np.square(wr_eval[:3]))))

            signed_rel_error = float("nan")
            abs_rel_error = float("nan")
            recv_over_cmd = float("nan")
            if cmd_force_norm >= self.active_threshold:
                signed_rel_error = (recv_force_norm - cmd_force_norm) / max(cmd_force_norm, self.epsilon)
                abs_rel_error = abs(recv_force_norm - cmd_force_norm) / max(cmd_force_norm, self.epsilon)
                recv_over_cmd = recv_force_norm / max(cmd_force_norm, self.epsilon)

            x_err = float(wr_eval[0] - cmd_eval[0])
            yz_residual = float(np.sqrt(wr_eval[1] ** 2 + wr_eval[2] ** 2))

            rospy.loginfo(
                "[cmd-wrench-error] frame=%s cmd_src=%s recv_src=%s cmd_force_xyz=%s recv_force_xyz=%s x_err=%.4f yz_residual=%.4f cmd_norm=%.6f recv_norm=%.6f recv/cmd=%s signed_rel=%s abs_rel=%s",
                active_frame,
                cmd_frame if cmd_frame else "<empty>",
                wr_frame if wr_frame else "<empty>",
                np.array2string(cmd_eval, precision=4, suppress_small=True),
                np.array2string(wr_eval, precision=4, suppress_small=True),
                x_err,
                yz_residual,
                cmd_force_norm,
                recv_force_norm,
                f"{recv_over_cmd:.6f}" if np.isfinite(recv_over_cmd) else "nan",
                f"{signed_rel_error:.6f}" if np.isfinite(signed_rel_error) else "nan",
                f"{abs_rel_error:.6f}" if np.isfinite(abs_rel_error) else "nan",
            )
            rate.sleep()


if __name__ == "__main__":
    try:
        node = CommandWrenchErrorMonitor()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("command-wrench error monitor failed: %s", str(exc))
