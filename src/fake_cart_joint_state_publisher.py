#!/usr/bin/python
import rospy
import threading

import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, PolygonStamped


class cart_joint_state_publisher:
    def __init__(self):
        """Fake publisher for cart joint states

        Arguments:
        - `self`:
        """
        self._joint_state_pub = rospy.Publisher("cart_joint_states", JointState, queue_size=10)
        self.cart_footprint_pub = rospy.Publisher("/cart_footprint", PolygonStamped, queue_size=10)

        self.tb = tf.TransformBroadcaster()

        # Make the message just once since nothing is changing...
        self.jointstate = JointState()
        self.jointstate.header.stamp = rospy.Time.now()
        self.jointstate.name = ["lf_caster_joint", "lf_wheel_joint", "rf_caster_joint", "rf_wheel_joint", "lb_caster_joint", "lb_wheel_joint", "rb_caster_joint", "rb_wheel_joint"]
        self.jointstate.position = [0.0] * len(self.jointstate.name)

        # Cart footprint message
        self.cart_footprint = PolygonStamped()
        self.cart_footprint.header.stamp = rospy.Time.now()
        self.cart_footprint.header.frame_id = "cart_base_link"
        self.cart_footprint.polygon.points = [Point(0.335, 0.335, 0.0), Point(0.335, -0.335, 0.0), Point(-0.335, -0.335, 0.0), Point(-0.335, 0.335, 0.0)]

        # Set a transform from the "base" link in the cart urdf to the "cart" frame
        self.base_pose = PoseStamped(pose=Pose(position=Point(1.15, 0.0, 0.0),
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
            self.publish_cart_joint_states()
            self.tb.sendTransform(self.base_pose_tup[0],
                                  self.base_pose_tup[1],
                                  rospy.get_rostime(),
                                  "cart_base_link", "base_link")
            r.sleep()

    def publish_cart_joint_states(self):
        """Publish dummy values
        """
        self._joint_state_pub.publish(self.jointstate)
        self.cart_footprint_pub.publish(self.cart_footprint)


if __name__ == '__main__':
    #from optparse import OptionParser
    #parser = OptionParser(usage="")
    #parser.add_option("-x", dest="cart_pose_x", default=0.0,
    #                  help="x displacement of cart model from \"cart\" frame in tf")
    #(options, args) = parser.parse_args()
    rospy.init_node("cart_joint_state_publisher")
    c=cart_joint_state_publisher()
    rospy.spin()
