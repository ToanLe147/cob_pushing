#!/usr/bin/python

import rospy
from simple_script_server import *
sss = simple_script_server()


if __name__ == '__main__':
    # Init node and ROS parameters will be used
    rospy.init_node('arm_ready_position')
    sss.init('arm_left')
    sss.init('arm_right')
    # sss.init('gripper_right')
    # sss.init('gripper_left')

    #=================== TEST ==================
    # sss.move("gripper_right", [[0.68, -0.11]])
    #===========================================

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
    # sss.sleep(1.0)
    # sss.move("gripper_left", "open")
    # sss.sleep(1.0)
    # sss.move("gripper_right", "open")

    # Test (go down)
    # sss.move("arm_left", [[0.315, 1.69, -1.41, 0.83, 0.00, -1.88, 1.39]], True)
    # sss.sleep(1.0)
    # sss.move("arm_right", [[-0.3, -1.69, 1.41, -0.83, 0.00, 1.70, -1.42]], True)
    #
    # sss.sleep(2.0)
    #
    # close
    # sss.move("gripper_left", [[0.85, 0.0]])
    # sss.sleep(1.0)
    # sss.move("gripper_right", [[0.85, 0.0]])

    # sss.sleep(5.0)

    # open
    # sss.move("gripper_left", [[-0.85, 0.0]])
    # sss.sleep(1.0)
    # sss.move("gripper_right", [[-0.85, 0.0]])

    # sss.sleep(2.0)

    # Raw code (Go up)
    # sss.move("arm_left", [[0.24, 1.69, -1.41, 0.83, 0.00, -1.88, 1.39]], False)
    # sss.sleep(1.0)
    # sss.move("arm_right", [[-0.23, -1.69, 1.41, -0.83, 0.00, 1.73, -1.42]], False)
