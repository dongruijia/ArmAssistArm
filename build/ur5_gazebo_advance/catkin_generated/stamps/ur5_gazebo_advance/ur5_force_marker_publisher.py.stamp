#!/usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import Point, WrenchStamped
import tf2_ros
import tf.transformations as tf_t
from visualization_msgs.msg import Marker, MarkerArray


class UR5ForceMarkerPublisher:
    def __init__(self):
        rospy.init_node("ur5_force_marker_publisher")

        self.wrench_topic = rospy.get_param("~wrench_topic", "/ur5/ft_sensor/wrench_corrected")
        self.marker_topic = rospy.get_param("~marker_topic", "/ur5/active_rehab/force_markers")
        self.marker_frame_id = rospy.get_param("~marker_frame_id", "world")
        self.fallback_frame_id = rospy.get_param("~fallback_frame_id", "wrist_3_link")
        self.anchor_frame_id = rospy.get_param("~anchor_frame_id", "wrist_3_link")
        self.publish_axes_without_wrench = bool(rospy.get_param("~publish_axes_without_wrench", True))
        self.tf_lookup_timeout_sec = float(rospy.get_param("~tf_lookup_timeout_sec", 0.05))
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 30.0))

        self.force_scale_m_per_n = float(rospy.get_param("~force_scale_m_per_n", 0.35))
        self.min_arrow_length_m = float(rospy.get_param("~min_arrow_length_m", 0.06))
        self.max_arrow_length_m = float(rospy.get_param("~max_arrow_length_m", 0.45))
        self.zero_threshold_n = float(rospy.get_param("~zero_threshold_n", 0.03))

        self.arrow_shaft_diameter_m = float(rospy.get_param("~arrow_shaft_diameter_m", 0.018))
        self.arrow_head_diameter_m = float(rospy.get_param("~arrow_head_diameter_m", 0.035))
        self.arrow_head_length_m = float(rospy.get_param("~arrow_head_length_m", 0.05))
        self.text_height_m = float(rospy.get_param("~text_height_m", 0.055))
        self.text_offset_m = float(rospy.get_param("~text_offset_m", 0.07))
        self.axis_enabled = bool(rospy.get_param("~axis_enabled", True))
        self.axis_length_m = float(rospy.get_param("~axis_length_m", 0.20))
        self.axis_text_offset_m = float(rospy.get_param("~axis_text_offset_m", 0.035))
        self.axis_shaft_diameter_m = float(rospy.get_param("~axis_shaft_diameter_m", 0.010))
        self.axis_head_diameter_m = float(rospy.get_param("~axis_head_diameter_m", 0.020))
        self.axis_head_length_m = float(rospy.get_param("~axis_head_length_m", 0.030))
        self.axis_text_height_m = float(rospy.get_param("~axis_text_height_m", 0.045))

        self.last_msg = None
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=10)
        rospy.Subscriber(self.wrench_topic, WrenchStamped, self.wrench_cb, queue_size=10)
        rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate_hz, 1.0)), self.timer_cb)

        rospy.loginfo("force marker publisher started")
        rospy.loginfo("  wrench_topic: %s", self.wrench_topic)
        rospy.loginfo("  marker_topic: %s", self.marker_topic)
        rospy.loginfo("  marker_frame_id: %s", self.marker_frame_id)
        rospy.loginfo("  anchor_frame_id: %s", self.anchor_frame_id)
        rospy.loginfo("  publish_axes_without_wrench: %s", self.publish_axes_without_wrench)

    def wrench_cb(self, msg):
        self.last_msg = msg

    def lookup_transform(self, target_frame, source_frame, stamp):
        lookup_stamp = stamp if stamp != rospy.Time() else rospy.Time(0)
        timeout = rospy.Duration.from_sec(max(self.tf_lookup_timeout_sec, 0.0))
        try:
            return self.tf_buffer.lookup_transform(target_frame, source_frame, lookup_stamp, timeout)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            rospy.logwarn_throttle(
                1.0,
                "force marker transform lookup failed: %s -> %s (%s)",
                source_frame,
                target_frame,
                str(exc),
            )
            return None

    def transform_force_to_marker_frame(self, force_vec, source_frame, stamp):
        target_frame = self.marker_frame_id or source_frame
        if source_frame == target_frame:
            return force_vec, target_frame

        transform = self.lookup_transform(target_frame, source_frame, stamp)
        if transform is None:
            return None, target_frame

        quat = transform.transform.rotation
        rotation = tf_t.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])[:3, :3]
        force_target = rotation.dot(force_vec)
        return force_target, target_frame

    def lookup_anchor_in_marker_frame(self, target_frame, stamp):
        anchor_frame = self.anchor_frame_id or self.fallback_frame_id
        if not anchor_frame or anchor_frame == target_frame:
            return np.zeros(3)

        transform = self.lookup_transform(target_frame, anchor_frame, stamp)
        if transform is None:
            return None

        trans = transform.transform.translation
        anchor = np.array([trans.x, trans.y, trans.z], dtype=float)
        return anchor

    def timer_cb(self, _event):
        if self.last_msg is None and not self.publish_axes_without_wrench:
            return

        if self.last_msg is None:
            stamp = rospy.Time.now()
            frame_id = self.marker_frame_id or self.fallback_frame_id
            force_target = np.zeros(3)
        else:
            msg = self.last_msg
            force = msg.wrench.force
            force_source = np.array([float(force.x), float(force.y), float(force.z)], dtype=float)

            source_frame = msg.header.frame_id or self.fallback_frame_id
            stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
            force_target, frame_id = self.transform_force_to_marker_frame(
                force_source, source_frame, stamp
            )
            if force_target is None:
                return

        anchor = self.lookup_anchor_in_marker_frame(frame_id, stamp)
        if anchor is None:
            return
        norm = float(np.linalg.norm(force_target))

        markers = MarkerArray()
        if self.axis_enabled:
            self.append_axis_markers(markers, frame_id, stamp, anchor)

        arrow = Marker()
        arrow.header.frame_id = frame_id
        arrow.header.stamp = stamp
        arrow.ns = "force"
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.frame_locked = False
        arrow.pose.orientation.w = 1.0
        arrow.scale.x = self.arrow_shaft_diameter_m
        arrow.scale.y = self.arrow_head_diameter_m
        arrow.scale.z = self.arrow_head_length_m
        arrow.color.r = 0.95
        arrow.color.g = 0.30
        arrow.color.b = 0.10
        arrow.color.a = 1.0 if norm >= self.zero_threshold_n else 0.0

        start = Point()
        start.x = float(anchor[0])
        start.y = float(anchor[1])
        start.z = float(anchor[2])
        end = Point()
        end.x = float(anchor[0])
        end.y = float(anchor[1])
        end.z = float(anchor[2])

        if norm >= self.zero_threshold_n:
            length = max(self.min_arrow_length_m, norm * self.force_scale_m_per_n)
            length = min(length, self.max_arrow_length_m)
            unit_vec = force_target / norm
            end.x = float(anchor[0] + unit_vec[0] * length)
            end.y = float(anchor[1] + unit_vec[1] * length)
            end.z = float(anchor[2] + unit_vec[2] * length)

        arrow.points = [start, end]
        markers.markers.append(arrow)

        text = Marker()
        text.header.frame_id = frame_id
        text.header.stamp = stamp
        text.ns = "force"
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.frame_locked = False
        text.pose.orientation.w = 1.0
        text.pose.position.x = float(end.x)
        text.pose.position.y = float(end.y)
        text.pose.position.z = float(end.z + self.text_offset_m)
        text.scale.z = self.text_height_m
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 0.95
        text.text = "{:.2f} N".format(norm)
        markers.markers.append(text)

        self.marker_pub.publish(markers)

    def append_axis_markers(self, markers, frame_id, stamp, origin):
        axis_specs = [
            ("X", 10, (1.0, 0.0, 0.0), (self.axis_length_m, 0.0, 0.0)),
            ("Y", 20, (0.0, 1.0, 0.0), (0.0, self.axis_length_m, 0.0)),
            ("Z", 30, (0.0, 0.4, 1.0), (0.0, 0.0, self.axis_length_m)),
        ]

        for label, base_id, color, endpoint in axis_specs:
            arrow = Marker()
            arrow.header.frame_id = frame_id
            arrow.header.stamp = stamp
            arrow.ns = "axes"
            arrow.id = base_id
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.frame_locked = False
            arrow.pose.orientation.w = 1.0
            arrow.scale.x = self.axis_shaft_diameter_m
            arrow.scale.y = self.axis_head_diameter_m
            arrow.scale.z = self.axis_head_length_m
            arrow.color.r = color[0]
            arrow.color.g = color[1]
            arrow.color.b = color[2]
            arrow.color.a = 0.95
            start = Point()
            start.x = float(origin[0])
            start.y = float(origin[1])
            start.z = float(origin[2])
            end = Point()
            end.x = float(origin[0] + endpoint[0])
            end.y = float(origin[1] + endpoint[1])
            end.z = float(origin[2] + endpoint[2])
            arrow.points = [start, end]
            markers.markers.append(arrow)

            text = Marker()
            text.header.frame_id = frame_id
            text.header.stamp = stamp
            text.ns = "axes"
            text.id = base_id + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.frame_locked = False
            text.pose.orientation.w = 1.0
            text.pose.position.x = float(origin[0] + endpoint[0])
            text.pose.position.y = float(origin[1] + endpoint[1])
            text.pose.position.z = float(origin[2] + endpoint[2] + self.axis_text_offset_m)
            text.scale.z = self.axis_text_height_m
            text.color.r = color[0]
            text.color.g = color[1]
            text.color.b = color[2]
            text.color.a = 1.0
            text.text = label
            markers.markers.append(text)


if __name__ == "__main__":
    try:
        UR5ForceMarkerPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("force marker publisher failed: %s", str(exc))
