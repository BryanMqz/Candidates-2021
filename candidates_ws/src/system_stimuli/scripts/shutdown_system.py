#!/usr/bin/env python3
# license removed for brevity
import rospy
import time
from std_msgs.msg import UInt16

def system_health():
    #Definition of data
    # 0 - 'System is stable'
    # 1 - 'System encountered an error'
    rospy.init_node('shutdown_system', anonymous=True)
    data = rospy.Publisher('system_health', UInt16, queue_size=10)
    rospy.loginfo('Checking robot state...')
    data.publish(int(input("\n0 for stable or 1 for error:\n")))
    rospy.loginfo("\nSystem status published.")
    rate = rospy.Rate(1)# 1hz


if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            system_health()
    except rospy.ROSInterruptException:
        pass