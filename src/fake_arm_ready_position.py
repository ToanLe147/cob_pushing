#!/usr/bin/python
import rospy
import threading

import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion


class arm_joint_state_publisher:
    def __init__(self):
        """Fake publisher for armt joint states

        Arguments:
        - `self`:
        """
        self._joint_state_pub = rospy.Publisher("joint_states", JointState)

        self.tb = tf.TransformBroadcaster()

        # Make the message just once since nothing is changing...
        self.jointstate = JointState()
        self.jointstate.header.stamp = rospy.Time.now()
        self.jointstate.name = ['arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint', 'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint', 'arm_left_7_joint', 'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint', 'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint', 'arm_right_7_joint', 'fl_caster_rotation_joint', 'fl_caster_r_wheel_joint', 'b_caster_rotation_joint', 'b_caster_r_wheel_joint', 'fr_caster_rotation_joint', 'fr_caster_r_wheel_joint', 'head_1_joint', 'head_2_joint', 'head_3_joint', 'sensorring_joint', 'gripper_right_finger_1_joint', 'gripper_right_finger_2_joint', 'gripper_left_finger_1_joint', 'gripper_left_finger_2_joint']
        self.jointstate.position = [0.75, 1.63, -1.41, 1.19, 0.00, -0.63, 1.59, -0.75, -1.63, 1.41, -1.19, 0.00, 0.45, -1.53, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

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
        self._joint_state_pub.publish(self.jointstate)


if __name__ == '__main__':
    #from optparse import OptionParser
    #parser = OptionParser(usage="")
    #parser.add_option("-x", dest="arm_pose_x", default=0.0,
    #                  help="x displacement of arm model from \"arm\" frame in tf")
    #(options, args) = parser.parse_args()
    rospy.init_node("arm_joint_state_publisher")
    c=arm_joint_state_publisher()
    rospy.spin()
