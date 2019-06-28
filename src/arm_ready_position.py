#!/usr/bin/python

import rospy
from std_msgs.msg import String
from simple_script_server import *
sss = simple_script_server()


def callback(msg):
    if msg.data == "Closed":
        sss.sleep(1.0)
        sss.move("gripper_left", "close")
        sss.sleep(1.0)
        sss.move("gripper_right", "close")


if __name__ == '__main__':
    # Init node and ROS parameters will be used
    rospy.init_node('arm_ready_position')
    sss.init('arm_left')
    sss.init('arm_right')
    sss.init('gripper_right')
    sss.init('gripper_left')

    # Go to home position
    # sss.move("arm_left", "side")
    # sss.sleep(2.0)
    # sss.move("arm_right", "side")
    # sss.sleep(1.0)
    # sss.move("gripper_left", "close")
    # sss.sleep(1.0)
    # sss.move("gripper_right", "close")

    # Go to pushing cart position
    # Using launch and custom yaml for arm positions

    # sss.move("arm_right", "ready")
    # sss.sleep(2.0)
    # sss.move("arm_left", "ready")
    # sss.sleep(1.0)
    # sss.move("gripper_left", "open")
    # sss.sleep(1.0)
    # sss.move("gripper_right", "open")

    # Raw code (Worked)
    # sss.move("arm_left", [[0.33, 1.69, -1.41, 0.83, 0.00, -1.88, 1.39]])
    # sss.sleep(2.0)
    # sss.move("arm_right", [[-0.32, -1.69, 1.41, -0.83, 0.00, 1.70, -1.42]])

    # Test
    # sss.move("arm_left", [[0.33, 1.69, -1.41, 0.83, 0.00, -1.00, 1.39]], False)
    # sss.sleep(2.0)
    # sss.move("arm_right", [[-0.32, -1.69, 1.41, -0.83, 0.00, 0.95, -1.42]], False)

    # sss.move("gripper_left", [[0.85, 0.0]])
    # sss.sleep(1.0)
    # sss.move("gripper_right", [[0.85, 0.0]])

    sss.move("gripper_left", [[-0.85, 0.0]])
    sss.sleep(1.0)
    sss.move("gripper_right", [[-0.85, 0.0]])

    # rospy.Subscriber("/gripper_command", String, callback)
    # rospy.spin()
