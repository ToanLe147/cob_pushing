#!/usr/bin/env python
# This script is used to test communcation between ontology and ROS
import rospy
from cob_hand_bridge.msg import JointValues


def talker():
    data = JointValues()
    data.position_cdeg = (3611, -610)
    # data.velocity_cdeg_s = (1000, 1000)
    # data.current_100uA = (-2120, 1400)
    # pub = rospy.Publisher('chatter', String, queue_size=10)
    pub = rospy.Publisher('/gripper_left/internal/command', JointValues, queue_size=10)
    rospy.init_node('simple_command', anonymous=True)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        hello_str = data
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
