#!/usr/bin/python
# Import ROS environments and message types
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
# Import calculating and operating packages
from simple_script_server import *
import laser_geometry.laser_geometry as lg
import numpy as np
from sklearn.cluster import KMeans


class cob_ready_state():
    def __init__(self):
        # Visualization topics:
        self.visual_points = rospy.Publisher("/visual_points", MarkerArray, queue_size=10)
        self.visual_laser_msg = rospy.Publisher("/visual_laser_msg", LaserScan, queue_size=1)

        # The cart status:
        self.the_cart = False
        self.cart_legs=[]
        self.cart_area = 0.2

        # Robot status:
        self.current_position = Odometry()
        self.standing_position = []

        # Handlers and feedback data:
        self.filtered_laser_msg = LaserScan()
        self.laser_projection = lg.LaserProjection()
        self.sss = simple_script_server()

    def drive_arms_ready(self):
        self.sss.init('arm_left')
        self.sss.init('arm_right')
        self.sss.sleep(1.0)
        self.sss.move("arm_right", "ready")
        self.sss.sleep(2.0)
        self.sss.move("arm_left", "ready")

    def open_grippers(self):
        self.sss.init('gripper_right')
        self.sss.init('gripper_left')
        self.sss.sleep(1.0)
        self.sss.move("gripper_left", "open")
        self.sss.sleep(1.0)
        self.sss.move("gripper_right", "open")

    def close_grippers(self):
        self.sss.init('gripper_right')
        self.sss.init('gripper_left')
        self.sss.sleep(1.0)
        self.sss.move("gripper_left", "close")
        self.sss.sleep(1.0)
        self.sss.move("gripper_right", "close")

    def handle_laser_msg(self, msg):
        self.filtered_laser_msg = msg
        # Only get data inside scnning radius of 1.1 meter
        ranges_list = np.array([])
        for i in self.filtered_laser_msg.ranges:
            range = np.nan
            if 234 < self.filtered_laser_msg.ranges.index(i) < 318:
                if str(i) != "nan":
                    if i < 1.3:
                        range = i
            ranges_list = np.append(ranges_list, range)
        # Update laser msg and visualize in Rviz
        self.filtered_laser_msg.ranges = ranges_list
        self.visualization(self.filtered_laser_msg, "laser")
        self.detect_cart_legs()

    def detect_cart_legs(self):
        # convert the message of type LaserScan to a PointCloud2
        pc2_msg = self.laser_projection.projectLaser(self.filtered_laser_msg)
        point_list = pc2.read_points_list(pc2_msg)
        dataset = []
        legs = [[np.nan, np.nan],
                [np.nan, np.nan],
                [np.nan, np.nan],
                [np.nan, np.nan]]
        for i in point_list:
            dataset.append([i.x, i.y])
        # K-mean clustering the dataset to get center of each leg:
        if len(dataset) >= 4:
            kmeans = KMeans(n_clusters=4, random_state=0).fit(dataset)
            legs = kmeans.cluster_centers_
        # Check if the legs is the cart legs then define the position the robot should be at
        height = abs(legs[1][0] - legs[0][0])
        width = abs(legs[2][1] - legs[0][1])
        area = height * width
        print(height, width, area)
        ##  Check if detected legs form same area as real cart legs (agree with threshhold less than 0.02 squared meter)
        if self.cart_area - area <= 0.03:
            if len(self.cart_legs)==0:
                self.cart_legs = legs
                self.standing_position =[[
                    (self.cart_legs[0][0] + self.cart_legs[2][0])/2,
                    (self.cart_legs[0][1] + self.cart_legs[2][1])/2]]
                self.the_cart = True
                # Visualize the cart in RVIZ
                self.visualization(self.standing_position, "points")
                self.visualization(self.cart_legs, "points")
        else:
            self.the_cart = False

    def visualization(self, data, type):
        markerArray = MarkerArray()
        id = 0

        def marker_generator():
            marker = Marker()
            marker.header.frame_id = "/base_laser_front_link"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.025
            marker.scale.y = 0.025
            marker.scale.z = 0.025
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.z = 0.0
            return marker

        if type == "points":
            if len(data) > 1:
                for i in data:
                    marker = marker_generator()
                    marker.pose.position.x = i[0]
                    marker.pose.position.y = i[1]
                    markerArray.markers.append(marker)

                    # Renumber the marker IDs
                    for m in markerArray.markers:
                        m.id = id
                        id += 1
            else:
                marker = marker_generator()
                marker.pose.position.x = data[0][0]
                marker.pose.position.y = data[0][1]
                markerArray.markers.append(marker)
            self.visual_points.publish(markerArray)

        if type == "laser":
            self.visual_laser_msg.publish(data)


if __name__ == '__main__':
    rospy.init_node("laser_handler_testing")
    cob = cob_ready_state()

    # if not cob.the_cart:
    #     cob.drive_arms_ready()
    #     cob.open_grippers()

    rospy.Subscriber("/base_laser_front/scan", LaserScan, cob.handle_laser_msg)
    rospy.spin()
