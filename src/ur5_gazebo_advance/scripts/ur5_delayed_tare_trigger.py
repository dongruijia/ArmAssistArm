#!/usr/bin/env python3

import rospy
from std_msgs.msg import Empty


class UR5DelayedTareTrigger:
    def __init__(self):
        rospy.init_node("ur5_delayed_tare_trigger")

        self.topic = rospy.get_param("~topic", "/ur5/ft_sensor/tare")
        self.delay_sec = float(rospy.get_param("~delay_sec", 3.0))
        self.repeat_count = int(rospy.get_param("~repeat_count", 2))
        self.repeat_interval = float(rospy.get_param("~repeat_interval", 0.2))

        self.pub = rospy.Publisher(self.topic, Empty, queue_size=1, latch=True)

        rospy.loginfo("delayed tare trigger started")
        rospy.loginfo("  topic: %s", self.topic)
        rospy.loginfo("  delay_sec: %.2f", self.delay_sec)

    def run(self):
        rospy.sleep(self.delay_sec)

        for index in range(max(self.repeat_count, 1)):
            if rospy.is_shutdown():
                return
            self.pub.publish(Empty())
            rospy.loginfo("published tare trigger %d/%d", index + 1, max(self.repeat_count, 1))
            if index + 1 < max(self.repeat_count, 1):
                rospy.sleep(self.repeat_interval)


if __name__ == "__main__":
    try:
        node = UR5DelayedTareTrigger()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("delayed tare trigger failed: %s", str(exc))
