#! /usr/bin/env python3



# !/usr/bin/python3

from enum import Enum
import time

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Quaternion, PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import Spin, NavigateToPose
from turtle_tf2_py.turtle_tf2_broadcaster import quaternion_from_euler
from nav_msgs.msg import OccupancyGrid

from irobot_create_msgs.action import Dock, Undock
from irobot_create_msgs.msg import DockStatus

import tf2_geometry_msgs as tfg
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration as rclpyDuration
from rclpy.node import Node

from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data


from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker

from sensor_msgs.msg import Image
from std_msgs.msg import String

import tf_transformations

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

from playsound import playsound
from pydub import AudioSegment
from pydub.playback import play

from gtts import gTTS
from io import BytesIO
import pygame

import concurrent.futures
import threading


from rclpy.executors import SingleThreadedExecutor
from threading import Thread


class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)


qos_profile = amcl_pose_qos




class ImageGatherer(Node):


    def __init__(self, node_name='image_gatherer', namespace=''):
        super().__init__(node_name=node_name)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        # # Neded so it even acts.
        # self.spin_client = ActionClient(self, Spin, 'spin')

        # for parking
        self.bridge = CvBridge()
        self.rgb_image_sub = self.create_subscription(Image, "/top_camera/rgb/preview/image_raw", self.rgb_callback, qos_profile_sensor_data)
        
        self.parking = False

        self.stats_pub = self.create_publisher(Twist, "/top_img_stats", QoSReliabilityPolicy.BEST_EFFORT)

        self.get_logger().info(f"ImageGatherer has been initialized!")
    

    def destroyNode(self):
        self.nav_to_pose_client.destroy()
        super().destroy_node()

    def set_parking(self, truth_val):
        self.parking = truth_val
    
    def get_latest_img(self):
        return self.latest_top_img

    def rgb_callback(self, data):
        
        # print("Here 1!")

        # if self.parking:
        if True:       

            # print("Here!")

            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

                cv2.imshow("Top image", cv_image)
                key = cv2.waitKey(1)
                if key==27:
                    print("exiting")
                    exit()
                
                colour_tresholded = self.get_white_pixels_threshold(cv_image)
                area, centroid = self.get_area_and_centroid(colour_tresholded)
                img_shape = colour_tresholded.shape

                self.publish_stats(centroid, area, img_shape)
                
            except CvBridgeError as e:
                print(e)
    



    def publish_stats(self, centroid, area, img_shape):
        
        twist_msg = Twist()

        twist_msg.linear.x = centroid[0]
        twist_msg.linear.y = centroid[1]
        twist_msg.linear.z = area
        
        twist_msg.angular.x = img_shape[0]
        twist_msg.angular.y = img_shape[1]

        self.stats_pub.publish(twist_msg)


    def get_white_pixels_threshold(self, img):
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        sat_img = hsv_img[:,:,1]
        val_img = hsv_img[:,:,2]

        saturation_treshold = 50
        sat_img = sat_img < saturation_treshold

        value_treshold = 230
        val_img = val_img > value_treshold


        colour_tresholded = np.logical_and(sat_img, val_img)
        
        colour_tresholded = colour_tresholded.astype(np.uint8)*255


        cv2.imshow("White tresholded", colour_tresholded)
        key = cv2.waitKey(1)

        return colour_tresholded



    def get_area_and_centroid(self, img):



        img = img.copy().astype('uint8')
        returned_data = cv2.connectedComponentsWithStats(img)
        num_of_components = returned_data[0]
        labels = returned_data[1]
        stats = returned_data[2]
        centroids = returned_data[3]


        if num_of_components == 1:
            return (0, (0, 0))
        

        # To prevent problems with tiny specs of white in the image messing things up,
        # we choose the component with the largest area
        max_area_ix = 0
        max_area = 0

        for i in range(1, num_of_components):
            if(stats[(i, cv2.CC_STAT_AREA)] > max_area):
                max_area = stats[(i, cv2.CC_STAT_AREA)]
                max_area_ix = i

        # print("centroids")
        # print(centroids)

        centroid = centroids[max_area_ix]
        area = stats[(max_area_ix, cv2.CC_STAT_AREA)]

        return (area, centroid)


        """
        for i in range (1, num_of_components):

            
            if(stats[(i, cv2.CC_STAT_AREA)] > 700):
                ix_left = stats[(i, cv2.CC_STAT_LEFT)]
                ix_right = stats[(i, cv2.CC_STAT_LEFT)] + stats[(i, cv2.CC_STAT_WIDTH)]
                ix_top = stats[(i, cv2.CC_STAT_TOP)]
                ix_bottom = stats[(i, cv2.CC_STAT_TOP)] + stats[(i, cv2.CC_STAT_HEIGHT)]
                
                # tole pa ne dela for some reason:
                # img[ix_top:ix_bottom][ix_left:ix_right] = 0
                img[ix_top:ix_bottom, ix_left:ix_right] = 0
        """

        """
        stats so oblike (label, COLUMN)
        Column vrednosti so:
        CC_STAT_LEFT Python: cv.CC_STAT_LEFT
        The leftmost (x) coordinate which is the inclusive start of the bounding box in the horizontal direction.
        CC_STAT_TOP Python: cv.CC_STAT_TOP
        CC_STAT_WIDTH 
        Python: cv.CC_STAT_WIDTH
        The horizontal size of the bounding box.
        CC_STAT_HEIGHT 
        Python: cv.CC_STAT_HEIGHT
        The vertical size of the bounding box.
        CC_STAT_AREA The total area (in pixels)
        """








def main(args=None):

    rclpy.init(args=args)

    ig = ImageGatherer()


    rclpy.spin(ig)

    cv2.destroyAllWindows()


    ig.destroyNode()

    # And a simple example
if __name__=="__main__":
    main()