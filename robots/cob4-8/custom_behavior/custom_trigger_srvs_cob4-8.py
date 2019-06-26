#!/usr/bin/python

import rospy
import custom_behavior
from std_srvs.srv import Trigger, TriggerResponse

def trigger_srvs():
    rospy.init_node('custom_trigger_srvs')
    s = rospy.Service('/custom_behavior/setLightRedSweep',Trigger,custom_behavior.setLightRedSweep_cb)

    rospy.spin()

if __name__ == "__main__":
    trigger_srvs()
