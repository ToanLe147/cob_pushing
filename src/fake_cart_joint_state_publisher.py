#! /usr/bin/env python
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#         * Redistributions of source code must retain the above copyright
#             notice, this list of conditions and the following disclaimer.
#         * Redistributions in binary form must reproduce the above copyright
#             notice, this list of conditions and the following disclaimer in the
#             documentation and/or other materials provided with the distribution.
#         * Neither the name of the Willow Garage, Inc. nor the names of its
#             contributors may be used to endorse or promote products derived from
#             this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Jon Scholz

PKG="pr2_gazebo_cartworld"
import roslib; roslib.load_manifest(PKG)
import rospy
import time
import sys
import threading

import tf
from sensor_msgs.msg import *
from geometry_msgs.msg import *

class cart_joint_state_publisher:
    def __init__(self):
        """Fake publisher for cart joint states

        Arguments:
        - `self`:
        """
        self._joint_state_pub = rospy.Publisher("cart_joint_states", JointState)

        self.tb = tf.TransformBroadcaster()

        # Make the message just once since nothing is changing...
        self.jointstate = JointState()
        self.jointstate.header.stamp = rospy.Time.now()
        self.jointstate.name = ["base_top_joint", "handle_joint", "front_right_caster", "front_left_caster", "front_right_wheel", "front_left_wheel", "back_right_caster", "back_left_caster", "back_right_wheel", "back_left_wheel"]
        self.jointstate.position = [0.0] * len(self.jointstate.name)
        self.jointstate.velocity = [0.0] * len(self.jointstate.name)
        self.jointstate.effort = [0.0] * len(self.jointstate.name)

        # Set a transform from the "base" link in the cart urdf to the "cart" frame
        self.base_pose = PoseStamped(pose=Pose(position=Point(0.0, 0.0, 0.0),
                                               orientation=Quaternion(0.0, 0.0, 0.0, 1.0)))
        self.base_pose.header.stamp = rospy.Time.now()
        self.base_pose_tup = self.pose_msg_to_tuple(self.base_pose.pose)

        # set up thread
        self.thread = threading.Thread(target=self._pub_thread, args = (10,))
        self.thread.setDaemon(1)
        self.alive = threading.Event()
        self.alive.set()
        self.thread.start()

    def pose_msg_to_tuple(self, Pose):
      """Returns a list representation of Pose [[vector],[euler angles]]"""
      return ((Pose.position.x,
               Pose.position.y,
               Pose.position.z),
               (Pose.orientation.x,
                Pose.orientation.y,
                Pose.orientation.z,
                Pose.orientation.w))

    def _pub_thread(self, rate):
        """
        """
        r=rospy.Rate(rate)
        while not rospy.is_shutdown() and self.alive.isSet():
            self.jointstate.header.stamp = rospy.Time.now()
            self.publish_cart_joint_states()
            self.tb.sendTransform(self.base_pose_tup[0],
                                  self.base_pose_tup[1],
                                  rospy.get_rostime(),
                                  "base", "cart")
            r.sleep()

    def publish_cart_joint_states(self):
        """Publish dummy values
        """
        self._joint_state_pub.publish(self.jointstate)

if __name__ == '__main__':
    #from optparse import OptionParser
    #parser = OptionParser(usage="")
    #parser.add_option("-x", dest="cart_pose_x", default=0.0,
    #                  help="x displacement of cart model from \"cart\" frame in tf")
    #(options, args) = parser.parse_args()
    rospy.init_node("cart_joint_state_publisher")
    c=cart_joint_state_publisher()
    rospy.spin()
