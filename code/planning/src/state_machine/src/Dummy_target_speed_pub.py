#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float32
import time


def talker():
    pub = rospy.Publisher('target_speed', Float32, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        time.sleep(20)
        data = 10.0
        pub.publish(data)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
