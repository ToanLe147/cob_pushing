#!/usr/bin/python

import rospy
from simple_script_server import *
sss = simple_script_server()


if __name__ == '__main__':
    # Init node and ROS parameters will be used
    rospy.init_node('arm_right_ready_position')
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
    sss.move("arm_right", "ready")
    sss.sleep(2.0)
    sss.move("arm_left", "ready")
    sss.sleep(1.0)
    sss.move("gripper_left", "open")
    sss.sleep(1.0)
    sss.move("gripper_right", "open")

    # Raw code
    # sss.move("arm_left", [[0,0,0,0,0,0,0], [0.75, 1.63, -1.41, 1.19, 0.00, -0.63, 1.59]])
    # sss.sleep(2.0)
    # sss.move("arm_right", [[0,0,0,0,0,0,0], [-0.75, -1.63, 1.41, -1.19, 0.00, 0.35, -1.53]])
