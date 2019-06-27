#!/usr/bin/python

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import laser_geometry.laser_geometry as lg
import numpy as np
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sklearn.cluster import KMeans


class laser_handler():
    def __init__(self):
        self.lp = lg.LaserProjection()
        self.pub_position = rospy.Publisher("/standing_position", Twist, queue_size=10)
        self.pub_visualization = rospy.Publisher("/legs_position", MarkerArray, queue_size=10)
        self.pub_gripper_command = rospy.Publisher("/gripper_command", String, queue_size=1)
        self.raw_laser_data = LaserScan()
        self.filtered_laser_data = LaserScan()
        self.ref_cart_leg_position = [[0.25, 0.67], [-0.2, 0.65]]
        self.legs = []
        self.position = Twist()
        self.position_thresh_hold = 0.05

    def get_dataset(self):
        # convert scanned distance data to coordinate x,y data list
        self.dataset = []
        if self.filtered_laser_data:
            self.filtered_pc_msg = self.lp.projectLaser(self.filtered_laser_data)
            point_list = pc2.read_points_list(self.filtered_pc_msg)
            # add x, y coordinates in operating data due to z equals to Zero
            for i in point_list:
                self.dataset.append([i.x, i.y])
        # print(self.dataset)

    def standing_position(self):
        # return standing position of robot for grasping the cart
        position = []
        self.legs = []
        # K-mean clustering self.dataset
        if len(self.dataset) >= 4:
            kmeans = KMeans(n_clusters=4, random_state=0).fit(self.dataset)
            self.legs = kmeans.cluster_centers_
            self.legs.sort()
            position.append((self.legs[0][1]+self.legs[1][1])/2)
            position.append((self.legs[0][0]+self.legs[1][0])/2)
        # Trick with legs data
        if self.legs[0][0] < self.ref_cart_leg_position[0][0] and self.legs[0][1] < self.ref_cart_leg_position[0][1]:

            if self.legs[2][0] > self.ref_cart_leg_position[1][0] and self.legs[2][1] < self.ref_cart_leg_position[1][1]:

                print("============222222")
                self.pub_gripper_command.publish("Closed")

        print(self.legs[0][1])
        print("=========")
        print(self.ref_cart_leg_position[0][1])
        print("//////")

        # Visualize legs
        self.visual(self.legs, 1)
        # self.visual(self.legs[3], 2)
        # Publish filter scan data
        self.pub_position.publish(self.position)

    def filter_cart(self, laser_ranges):
        # Filter the cart
        ranges_list = np.array([])
        for i in laser_ranges:
            range = i
            if str(i) != "nan":
                if i > 1.1:
                    range = np.nan
            ranges_list = np.append(ranges_list, range)
        return ranges_list

    def callback(self, msg):
        # Initial output message same structure as input message
        self.raw_laser_data = msg
        self.filtered_laser_data = msg
        # Update output msg with filtered ranges
        self.filtered_laser_data.ranges = self.filter_cart(self.raw_laser_data.ranges)
        # Update dataset
        self.get_dataset()
        # Desire standing position of robot
        self.standing_position()

    def visual(self, legs, type):
        #Visualize legs for testing
        if type == 1:
            markerArray = MarkerArray()
            id = 0

            for i in legs:
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
                marker.pose.position.x = np.round(i[1], 2)
                marker.pose.position.y = np.round(i[0], 2)
                marker.pose.position.z = 0.0

                markerArray.markers.append(marker)
                # Renumber the marker IDs
                for m in markerArray.markers:
                    m.id = id
                    id += 1
            self.pub_visualization.publish(markerArray)
        else:
            markerArray = MarkerArray()
            marker = Marker()
            marker.header.frame_id = "/base_laser_front_link"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = legs[1]
            marker.pose.position.y = legs[0]
            marker.pose.position.z = 0.0
            markerArray.markers.append(marker)
            self.pub_visualization.publish(markerArray)


if __name__ == '__main__':
    rospy.init_node("laser_handler")
    handler = laser_handler()
    rospy.Subscriber("/base_laser_front/scan", LaserScan, handler.callback)
    rospy.spin()
