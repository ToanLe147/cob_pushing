#!/usr/bin/python
# Copyright 2017 Mattia Racca

import rospy
from simple_script_server import *
sss = simple_script_server()
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger, TriggerResponse
from cob_light.srv import *
from cob_light.msg import *


def setLightRedSweep_cb(req):
    #rospy.wait_for_service('/light_torso/set_light')
    sss.set_light("light_base","red")

    try:
        set_light_torso = rospy.ServiceProxy("/light_torso/set_light",SetLightMode)
        light_mode = LightMode()
        cyan_color = ColorRGBA()
        cyan_color.r = 1.0
        cyan_color.g = 0.0
        cyan_color.b = 0.0
        cyan_color.a = 0.4
        light_mode.colors.append(cyan_color)
        light_mode.mode = 8
        light_mode.frequency = 30
        resp = set_light_torso(light_mode)
        print resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return TriggerResponse(False, "Calling light service failed.")
    return TriggerResponse(True, "")
