#!/usr/bin/python
import rospy
import threading

import tf
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion


class robot_joint_state_publisher:
    def __init__(self):
        """Fake publisher for armt joint states

        Arguments:
        - `self`:
        """
        self._joint_state_pub = rospy.Publisher("joint_states", JointState)
        self.arm_left = rospy.get_param('/arm_left/joint_names')
        self.arm_left_states = rospy.get_param('/arm_left/joint_states')

        self.arm_right = rospy.get_param('/arm_right/joint_names')
        self.arm_right_states = rospy.get_param('/arm_right/joint_states')

        self.other = rospy.get_param('/other_joint/joint_names')
        self.other_states = rospy.get_param('/other_joint/joint_states')

        self.jointstate = JointState()
        self.jointstate.header.stamp = rospy.Time.now()
        self.jointstate.name = self.arm_left + self.arm_right + self.other
        self.jointstate.position = self.arm_left_states + self.arm_right_states + self.other_states

        self.tb = tf.TransformBroadcaster()

        # Set a transform from the "base" link in the arm urdf to the "arm" frame
        self.base_pose = PoseStamped(pose=Pose(position=Point(0.0, 0.0, 0.0),
                                               orientation=Quaternion(0.0, 0.0, 0.0, 1.0)))
        self.base_pose.header.stamp = rospy.Time.now()
        self.base_pose_tup = self.pose_msg_to_tuple(self.base_pose.pose)

        # set up thread
        self.thread = threading.Thread(target=self._pub_thread, args=(10,))
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
            self.publish_arm_joint_states()
            # self.tb.sendTransform(self.base_pose_tup[0],
            #                       self.base_pose_tup[1],
            #                       rospy.get_rostime(),
            #                       "arm_base_link", "base_link")
            r.sleep()

    def publish_arm_joint_states(self):
        """Publish dummy values
        """
        self.arm_left_states = rospy.get_param('/arm_left/joint_states')

        self.arm_right_states = rospy.get_param('/arm_right/joint_states')

        self.other_states = rospy.get_param('/other_joint/joint_states')

        # Make the message just once since nothing is changing...
        self.jointstate.position = self.arm_left_states + self.arm_right_states + self.other_states

        self._joint_state_pub.publish(self.jointstate)


if __name__ == '__main__':
    rospy.init_node("robot_joint_state_publisher")
    c=robot_joint_state_publisher()
    rospy.spin()
