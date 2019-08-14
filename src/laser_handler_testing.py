#!/usr/bin/python
# Import ROS environments and message types
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import PolygonStamped, Point
# Import calculating and operating packages
from simple_script_server import *
import laser_geometry.laser_geometry as lg
import numpy as np
from sklearn.cluster import KMeans
from shapely.geometry import Point as shape_point
from shapely.geometry.polygon import Polygon as shape_polygon


class cob_ready_state():
    def __init__(self):
        # Visualization topics:
        self.visual_cart_footprint_estimator = rospy.Publisher("/visual_cart_footprint_estimator", PolygonStamped, queue_size=1)
        self.visual_laser_msg = rospy.Publisher("/cart_filtered_scan", LaserScan, queue_size=1)
        self.visual_cart = rospy.Publisher("/isolated_cart_scan", LaserScan, queue_size=1)

        # The cart status:
        self.the_cart_pub = rospy.Publisher("/cart_is_holded", String, queue_size=1)
        self.ref_cart_legs=[(1.375, 0.225),
                            (0.705, 0.225),
                            (0.705, -0.225),
                            (1.375, -0.225)]
        self.the_cart = False

        # Cart footprint estimator message
        self.cart_footprint_estimator = PolygonStamped()
        self.cart_footprint_estimator.header.frame_id = "base_link"

        # Handlers and feedback data:
        self.filtered_laser_msg = LaserScan()
        self.cart_isolated_laser_msg = LaserScan()
        self.laser_projection = lg.LaserProjection()
        self.sss = simple_script_server()

    def drive_arms_ready(self):
        self.sss.init('arm_left')
        self.sss.init('arm_right')
        self.sss.sleep(1.0)
        self.sss.move("arm_right", "ready")
        self.sss.sleep(1.0)
        self.sss.move("arm_left", "ready")

    def handle_laser_msg(self, msg):
        self.filtered_laser_msg = msg
        self.cart_isolated_laser_msg = msg
        filtered_ranges_list = np.array([])
        cart_ranges_list = np.array([])
        # self.laser_length = len(msg.ranges)

        # Only get data outside scnning radius of 1.3 meter
        for i in self.filtered_laser_msg.ranges:
            range_filter = i
            if 300 < self.filtered_laser_msg.ranges.index(i) < 420:
                if str(i) != "nan":
                    if i < 1.485:
                        range_filter = np.nan
            filtered_ranges_list = np.append(filtered_ranges_list, range_filter)

        # Only get data inside scnning radius of 1.3 meter
        for i in self.cart_isolated_laser_msg.ranges:
            range_temp = np.nan
            if 320 < self.cart_isolated_laser_msg.ranges.index(i) < 400:
                if str(i) != "nan":
                    if i < 1.485:
                        range_temp = i
            # if 0 < self.cart_isolated_laser_msg.ranges.index(i) < 10:  #Test
            #     range_temp = i
            cart_ranges_list = np.append(cart_ranges_list, range_temp)

        # Update laser msg and visualize in Rviz
        self.filtered_laser_msg.ranges = filtered_ranges_list
        self.visualization(self.filtered_laser_msg, "filtered_laser")
        self.cart_isolated_laser_msg.ranges = cart_ranges_list
        self.visualization(self.cart_isolated_laser_msg, "isolated_cart")
        self.detect_cart_legs()

    def detect_cart_legs(self):
        # convert the message of type LaserScan to a PointCloud2
        pc2_msg = self.laser_projection.projectLaser(self.cart_isolated_laser_msg)
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
        # print(legs)

        def check_leg_position(ref, source):
            check = 0
            polygon = shape_polygon(ref)
            for i in source:
                point = shape_point(i[0], i[1])
                if polygon.contains(point):
                    check += 1
            if check >= len(ref):
                return True
            else:
                return False

        # Manually check position of the cart:
        if check_leg_position(self.ref_cart_legs, legs) and check_leg_position(self.ref_cart_legs, dataset):
            self.the_cart = True
            self.cart_footprint_estimator.header.stamp = rospy.Time.now()
            leg1 = Point(legs[0][0], legs[0][1], 0.0)
            leg2 = Point(legs[3][0], legs[3][1], 0.0)
            leg3 = Point(legs[1][0], legs[1][1], 0.0)
            leg4 = Point(legs[2][0], legs[2][1], 0.0)
            self.cart_footprint_estimator.polygon.points = [leg1, leg2, leg3, leg4]
            self.visual_cart_footprint_estimator.publish(self.cart_footprint_estimator)
            # self.close_grippers()
        else:
            self.the_cart = False
        rospy.loginfo("Cart detected: " + str(self.the_cart))
        self.the_cart_pub.publish(str(self.the_cart))

    def visualization(self, data, type):

        if type == "filtered_laser":
            self.visual_laser_msg.publish(data)

        if type == "isolated_cart":
            self.visual_cart.publish(data)


if __name__ == '__main__':
    rospy.init_node("grasping_cart")
    cob = cob_ready_state()

    # if not cob.the_cart:
    #     cob.drive_arms_ready()

    rospy.Subscriber("/scan_unified", LaserScan, cob.handle_laser_msg)
    rospy.spin()
