#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import tf2_ros

import random

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy










from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy
from rclpy.duration import Duration

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

import tf2_geometry_msgs as tfg
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

import math



from sensor_msgs.msg import CameraInfo

















qos_profile = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)

class RingDetector(Node):
    def __init__(self):
        super().__init__('transform_point')

        # Basic ROS stuff
        timer_frequency = 2
        timer_period = 1/timer_frequency

        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # Marker array object used for visualizations
        self.marker_array = MarkerArray()
        self.marker_num = 1

        # Subscribe to the image and/or depth topic
        self.image_sub = self.create_subscription(Image, "/oakd/rgb/preview/image_raw", self.image_callback, 1)
        self.depth_sub = self.create_subscription(Image, "/oakd/rgb/preview/depth", self.depth_callback, 1)


        # For publishing the markers
        self.marker_topic = "/detected_rings"
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, QoSReliabilityPolicy.BEST_EFFORT)

        # Publiser for the visualization markers
        # self.marker_pub = self.create_publisher(Marker, "/ring", QoSReliabilityPolicy.BEST_EFFORT)

        self.latest_depth_image = None


        # Object we use for transforming between coordinate frames
        # self.tf_buf = tf2_ros.Buffer()
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        # cv2.namedWindow("Binary Image", cv2.WINDOW_NORMAL)
        # cv2.namedWindow("Detected contours", cv2.WINDOW_NORMAL)
        # cv2.namedWindow("Detected rings", cv2.WINDOW_NORMAL)
        # cv2.namedWindow("Depth window", cv2.WINDOW_NORMAL)








        # Marker handling

        # Get camera calibration to be able to get the 3D coordinates of the points in the image
        self.camera_info = None
        self.subscription = self.create_subscription(
            CameraInfo,
            #'/oakd_rgb_camera_optical_frame/camera_info',
            '/oakd/rgb/preview/camera_info',
            self.camera_info_callback,
            10)


        # For listening and loading the 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # self.pointcloud_sub = self.create_subscription(PointCloud2, "/oakd/rgb/preview/depth/points", self.pointcloud_callback, qos_profile_sensor_data)

        self.curr_markers = []


        # False positive checking, by making sure we detect 
        # the marker repetedly enough times.
        self.CHECK_FALSE_POSITIVES = False
        self.buffer_size = 10
        self.false_pos_marker_buffer = []

        # for unique markers
        self.marker_id = 0
        self.min_distance_between_markers = 2.
        self.detected_markers = []


        self.color_diff_treshold = 30




    










    def camera_info_callback(self, msg):
            fx = msg.k[0]
            fy = msg.k[4]
            cx = msg.k[2]
            cy = msg.k[5]
            self.camera_info = (fx, fy, cx, cy)
            # print(f"Camera Intrinsics: fx={fx}, fy={fy}, cx={cx}, cy={cy}")
    
    def camera_pixels_to_3d(self, x, y, depth):

        if depth == float('inf'):
            print("Depth is inf")
            return None
        
        if depth < 0:
            print("Depth is negative")
            return None

        if self.camera_info is None:
            return None

        fx, fy, cx, cy = self.camera_info
        """
        Convert depth and pixel coordinates to 3D point.
        :param x: Pixel x coordinate
        :param y: Pixel y coordinate
        :param depth: Depth value at (x, y)
        :param fx: Focal length in x
        :param fy: Focal length in y
        :param cx: Principal point x
        :param cy: Principal point y
        :return: (X, Y, Z) in camera coordinates
        """

        # I think this conversion wasn't necessary so it messed things up.
        # Z = depth / 1000.0  # Convert mm to meters
        Z = depth
        X = (x - cx) * Z / fx
        Y = (y - cy) * Z / fy
        return (X, Y, Z)

    def point_from_coordinates(self, point_tuple, frame_id, time_stamp):

        x, y, z = point_tuple

        # Create a PointStamped in the /base_link frame of the robot
        # "Stamped" means that the message type contains a Header
        
        point_in_robot_frame = PointStamped()
        point_in_robot_frame.header.frame_id = frame_id #"/base_link"
        
        # point_in_robot_frame.header.stamp = self.get_clock().now().to_msg()   # This was suggested by copilot. Caution.
        point_in_robot_frame.header.stamp = time_stamp

        point_in_robot_frame.point.x = float(x)
        point_in_robot_frame.point.y = float(y)
        point_in_robot_frame.point.z = float(z)

        return point_in_robot_frame


    """
    Checking if markers are new and not false positives.
    """

            
    def distance(self, marker1, marker2):
        # calculate distance between 2 markers in a 3D space
        x1 = marker1.pose.position.x
        x2 = marker2.pose.position.x
        y1 = marker1.pose.position.y
        y2 = marker2.pose.position.y
        z1 = marker1.pose.position.z
        z2 = marker2.pose.position.z

        dist = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)

        return dist

    def new(self, marker):
        # check if marker is far enough from the already detected markers
        for previously_detected_marker in self.detected_markers:
            if self.distance(marker, previously_detected_marker) < self.min_distance_between_markers:
                return False
        
        return True
    

    def notFalsePositive(self, marker):
        # check if enough markers have been detected to prevent a false positive

        self.false_pos_marker_buffer.append(marker)
        if len(self.false_pos_marker_buffer) < self.buffer_size + 1:
            return False

        marker_counter = 0
        for curr_marker in self.false_pos_marker_buffer:
            if self.distance(marker, curr_marker) < self.min_distance_between_markers / 2:
                marker_counter += 1

        self.false_pos_marker_buffer.pop(0)

        if marker_counter < self.buffer_size // 3:
            return False
        return True





















    def create_marker(self, point_stamped, marker_id, color=(0.0, 0.0, 0.0)):
        """You can see the description of the Marker message here: https://docs.ros2.org/galactic/api/visualization_msgs/msg/Marker.html"""

        # create marker from PointStamped

        marker = Marker()

        marker.header = point_stamped.header

        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.id = marker_id

        # Set the scale of the marker
        scale = 0.15
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        # Set the color
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0

        # Set the pose of the marker
        marker.pose.position.x = point_stamped.point.x
        marker.pose.position.y = point_stamped.point.y
        marker.pose.position.z = point_stamped.point.z

        return marker



    def publish_marker_from_point(self, point_in_robot_frame, transform, color=(0.0, 0.0, 0.0)):
        

        # # Now we look up the transform between the base_link and the map frames
        # # and then we apply it to our PointStamped
        # time_now = rclpy.time.Time()
        # timeout = Duration(seconds=0.1)
        # try:
        #     # An example of how you can get a transform from /base_link frame to the /map frame
        #     # as it is at time_now, wait for timeout for it to become available
        #     trans = self.tf_buffer.lookup_transform("map", "base_link", time_now, timeout)
        #     #self.get_logger().info(f"Looks like the transform is available.")
        
        # except TransformException as te:
        #     self.get_logger().info(f"Could not get the transform: {te}")
        #     return
        
        trans = transform




        # Now we apply the transform to transform the point_in_robot_frame to the map frame
        # The header in the result will be copied from the Header of the transform
        point_in_map_frame = tfg.do_transform_point(point_in_robot_frame, trans)
        #self.get_logger().info(f"We transformed a PointStamped!")

        # If the transformation exists, create a marker from the point, in order to visualize it in Rviz
        marker_in_map_frame = self.create_marker(point_in_map_frame, self.marker_id, color)

        # if not math.isnan(marker_in_map_frame.pose.position.x) and not math.isnan(marker_in_map_frame.pose.position.y) and self.new(marker_in_map_frame):

        if math.isnan(marker_in_map_frame.pose.position.x) or math.isnan(marker_in_map_frame.pose.position.y):
            return
        
        if not self.new(marker_in_map_frame):
            return
        

        if self.CHECK_FALSE_POSITIVES:
            # check if marker already detected
            if self.notFalsePositive(marker_in_map_frame):
                return



        # if it's new, append it to detected markers
        self.detected_markers.append(marker_in_map_frame)

        # Publish the marker
        self.marker_pub.publish(marker_in_map_frame)
        self.get_logger().info(f"The marker has been published to {self.marker_topic}. You are able to visualize it in Rviz")
        #self.get_logger().info(f"x: {marker_in_map_frame.pose.position.x}, y: {marker_in_map_frame.pose.position.y}, z: {marker_in_map_frame.pose.position.z}")
        
        for curr_marker in self.detected_markers:
            self.get_logger().info(f"x: {curr_marker.pose.position.x}, y: {curr_marker.pose.position.y}, z: {curr_marker.pose.position.z}")

        # Increase the marker_id, so we dont overwrite the same marker.
        self.marker_id += 1

        #else:
            #self.get_logger().info(f"marker has already been detected")



    def publish_marker(self, x, y, z, time_stamp, transform, color=(0.0, 0.0, 0.0)):

        # Create a PointStamped in the /base_link frame of the robot
        # "Stamped" means that the message type contains a Header
        
        point_in_robot_frame = PointStamped()
        point_in_robot_frame.header.frame_id = "/base_link"
        
        # point_in_robot_frame.header.stamp = self.get_clock().now().to_msg()   # This was suggested by copilot. Caution.
        point_in_robot_frame.header.stamp = time_stamp

        point_in_robot_frame.point.x = float(x)
        point_in_robot_frame.point.y = float(y)
        point_in_robot_frame.point.z = float(z)

        self.publish_marker_from_point(point_in_robot_frame, transform, color)




    """
    THIS ISNT FINISHED YET.
    I DIDNT THIN IT WOULD WORK.
    BUT SOME WORK HAS BEEN DONE AND IT COULD HELP IN THE FUTURE."""


    """
    
    def marker_append(self, marker_points_list):
        self.curr_markers = []

        for x, y in marker_points_list:
            self.curr_markers.append((x,y))
    """


    """
    # def pointcloud_callback(self, data):

    #     # print("pointcloud_callback!")

    #     # get point cloud attributes
    #     height = data.height
    #     width = data.width
    #     point_step = data.point_step
    #     row_step = data.row_step		

    #     # iterate over marker coordinates
    #     for x,y in self.curr_markers:

    #         # get 3-channel representation of the poitn cloud in numpy format
    #         a = pc2.read_points_numpy(data, field_names= ("x", "y", "z"))
    #         a = a.reshape((height,width,3))

    #         # read center coordinates
    #         d = a[y,x,:]


    #         # Create a PointStamped in the /base_link frame of the robot
    #         # "Stamped" means that the message type contains a Header
    #         point_in_robot_frame = PointStamped()
    #         point_in_robot_frame.header.frame_id = "/base_link"
    #         point_in_robot_frame.header.stamp = data.header.stamp

    #         point_in_robot_frame.point.x = float(d[0])
    #         point_in_robot_frame.point.y = float(d[1])
    #         point_in_robot_frame.point.z = float(d[2])

    #         self.publish_marker(point_in_robot_frame)





    # Managing a pointcloud

    def pointcloud_callback(self, data):
            
        return
            
        self.latest_point_cloud = data


    def manage_pointcloud(self, pointcloud_at_that_time, curr_markers):
        
        data = pointcloud_at_that_time
        # print("pointcloud_callback!")

        # get point cloud attributes
        height = data.height
        width = data.width
        point_step = data.point_step
        row_step = data.row_step		


        # get 3-channel representation of the poitn cloud in numpy format
        a = pc2.read_points_numpy(data, field_names= ("x", "y", "z"))
        a = a.reshape((height,width,3))

        # iterate over marker coordinates
        # for x,y in self.curr_markers:
        for x,y in curr_markers:


            # read center coordinates
            d = a[y,x,:]


            # Create a PointStamped in the /base_link frame of the robot
            # "Stamped" means that the message type contains a Header
            point_in_robot_frame = PointStamped()
            point_in_robot_frame.header.frame_id = "/base_link"
            point_in_robot_frame.header.stamp = data.header.stamp

            point_in_robot_frame.point.x = float(d[0])
            point_in_robot_frame.point.y = float(d[1])
            point_in_robot_frame.point.z = float(d[2])

            self.publish_marker(point_in_robot_frame)


    """














    def get_decimal_rgb_from_bounding_boxes_of_BGR(self, cv_image, rough_bounding_boxes, thresh):

        # print("np.max(thresh)")
        # print(np.max(thresh))

        img = cv_image.copy()




        # cv2.imshow("Color before tresh", img)
        # cv2.waitKey(1)

        for i in range(3):
            img[:,:,i] = img[:,:,i] * thresh
        


        # cv2.imshow("Color after tresh", img)
        # cv2.waitKey(1)


        diff_sum = np.zeros((img.shape[0], img.shape[1]))

        # They were all green markers, just different shades.
        # Probably because grey values were in there too much.

        for i in range(3):
            for j in range(i, 3):
                diff_sum += np.abs(img[:,:,i] - img[:,:,j])

        # cv2.imshow("Diff_sum", diff_sum)
        # cv2.waitKey(1)
        

        enough_diff = diff_sum > self.color_diff_treshold

        for i in range(3):
            img[:,:,i] = img[:,:,i] * enough_diff
        

        # cv2.imshow("Color after diff_sum", img)
        # cv2.waitKey(1)

        
        



        rgb_values = []

        for box in rough_bounding_boxes:
            

            x, y, major_ax = box

            # print("x, y, major_ax")
            # print(x, y, major_ax)

            curr_img = img[y-major_ax:y+major_ax, x-major_ax:x+major_ax, :]

            cv2.imshow("Color after bbox", curr_img)
            cv2.waitKey(1)

            colors = []

            for i in range(3):
                color_vec = curr_img[:,:,i]
                color_vec = color_vec.reshape(-1)
                # print("color_vec")
                # print(color_vec)
                color_vec = color_vec[color_vec > 10]
                # print("color_vec")
                # print(color_vec)
                mean = np.mean(color_vec) / 255
                if np.isnan(mean) or np.isinf(mean):
                    mean = 0.0
                colors.append(mean)

            blue_mean = colors[0]
            green_mean = colors[1]
            red_mean = colors[2]

            rgb_values.append((red_mean, green_mean, blue_mean))

        return rgb_values


    def get_axis_tips_from_elipse(self, elipse):



        angle = elipse[2]
        angle_rad = np.deg2rad(angle)


        """
        axis lengths returned by the function are actually the full lengths of the major and minor axes of the ellipse
        
        The values returned by cv2.fitEllipse() for the axes are the width and height of the 
        bounding rectangle of the ellipse, not directly labeled as 'major' and 'minor'. 

        angle helps determine how the width and height relate to the major and minor axes:
        If the angle is 0 degrees, the width is the major axis and the height is the minor axis.
        If the angle is 90 degrees, the height is the major axis and the width is the minor axis.

        Just think of them as the first and second axis.
        """


        first_axis_tip_x = elipse[0][0] + elipse[1][0]/2 * np.cos(angle_rad)
        first_axis_tip_y = elipse[0][1] + elipse[1][0]/2 * np.sin(angle_rad)

        second_axis_tip_x = elipse[0][0] + elipse[1][1]/2 * np.cos(angle_rad)
        second_axis_tip_y = elipse[0][1] + elipse[1][1]/2 * np.sin(angle_rad)
        

        return np.floor(np.array([first_axis_tip_x, first_axis_tip_y])).astype(int), np.floor(np.array([second_axis_tip_x, second_axis_tip_y])).astype(int)


    def get_depth_from_point(self, depth_img, point):

        
        
        x = point[0] if point[0] < depth_img.shape[1] else depth_img.shape[1]-1
        y = point[1] if point[1] < depth_img.shape[0] else depth_img.shape[0]-1
        
        x = x if x > 0 else 0
        y = y if y > 0 else 0

        # print("depth_img.shape")
        # print(depth_img.shape)

        return depth_img[y, x]


    def get_mean_depths_from_bounding_boxes(self, current_depth_image, rough_bounding_boxes, thresh=None):

        # We are skipping this optimizetion for now
        if thresh is not None:
            depth_img = current_depth_image * thresh
        else:
            depth_img = current_depth_image


        depths = []

        for box in rough_bounding_boxes:
            x, y, major_ax = box
    
            depth_img_vec = depth_img[y-major_ax:y+major_ax, x-major_ax:x+major_ax].reshape((-1))

            finite_arr = depth_img_vec[np.isfinite(depth_img_vec)]

            if finite_arr.size == 0:
                depths.append(None)
                continue
            mean = np.mean(finite_arr)

            depths.append(mean)
        
        return depths


    def elipse_check(self, thresholded_img, gray_img, current_depth_image, cv_image):
        
        thresh = thresholded_img
        gray = gray_img

        # cv2.imshow("Binary Image", thresh)
        # cv2.waitKey(1)

        # Extract contours
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        

        # Example of how to draw the contours, only for visualization purposes
        cv2.drawContours(gray, contours, -1, (255, 0, 0), 3)
        cv2.imshow("Detected contours", gray)
        cv2.waitKey(1)


        # Fit elipses to all extracted contours
        elps = []
        for cnt in contours:
            #     print cnt
            #     print cnt.shape
            if cnt.shape[0] >= 20:
                ellipse = cv2.fitEllipse(cnt)
                elps.append(ellipse)



        MAX_CENTER_DISTANCE_TRESHOLD = 5
        MAX_ANGLE_DIFF_TRESHOLD = 20
        MAX_MAJOR_MINOR_BORDER_DIFF_TRESHOLD = 4

        USE_DEPTH_DIFF = True
        MIN_DEPTH_DIFF_RELATIVE_TRESHOLD = 0.4

        # Find two elipses with same centers
        candidates = []
        for n in range(len(elps)):
            for m in range(n + 1, len(elps)):

                # e[0] is the center of the ellipse (x,y), e[1] are the lengths of major and minor axis (major, minor), e[2] is the rotation in degrees

                """
                e[0][0] is the x-coordinate in the image.
                e[0][1] is the y-coordinate in the image.
                They are both floats (more precise estimates.

                I don't know why, but x comes first and y second. It's different than images.
                """

                e1 = elps[n]
                e2 = elps[m]

                # The centers of the two elipses should be within 5 pixels of each other (is there a better treshold?)
                dist = np.sqrt(((e1[0][0] - e2[0][0]) ** 2 + (e1[0][1] - e2[0][1]) ** 2))
                if dist >= MAX_CENTER_DISTANCE_TRESHOLD:
                    continue

                # The rotation of the elipses should be whitin 4 degrees of eachother
                angle_diff = np.abs(e1[2] - e2[2])
                if angle_diff > MAX_ANGLE_DIFF_TRESHOLD:
                    continue




                # These aren't actually minor and major, but rather width and height of the angled bounding box around the elipse.
                # Which one is major and which minor (the 0th or 1st one) depends on the angle.

                # So just think of them as the first and second axis.

                e1_minor_axis = e1[1][0]
                e1_major_axis = e1[1][1]

                e2_minor_axis = e2[1][0]
                e2_major_axis = e2[1][1]

                if e1_major_axis>=e2_major_axis and e1_minor_axis>=e2_minor_axis: # the larger ellipse should have both axis larger
                    le = e1 # e1 is larger ellipse
                    se = e2 # e2 is smaller ellipse
                elif e2_major_axis>=e1_major_axis and e2_minor_axis>=e1_minor_axis:
                    le = e2 # e2 is larger ellipse
                    se = e1 # e1 is smaller ellipse
                else:
                    continue # if one ellipse does not contain the other, it is not a ring





                # The widths of the ring along the major and minor axis should be roughly the same
                border_major = (le[1][1]-se[1][1])/2
                border_minor = (le[1][0]-se[1][0])/2
                border_diff = np.abs(border_major - border_minor)

                if border_diff > MAX_MAJOR_MINOR_BORDER_DIFF_TRESHOLD:
                    continue






                if USE_DEPTH_DIFF:
                    
                    if current_depth_image is None:
                        continue
                    

                    x = int(le[0][0])
                    y = int(le[0][1])


                    dim1 = int(le[1][0])
                    dim2 = int(le[1][1])

                    larger_dim = max(dim1, dim2)

                    bbox_envelop = (x, y, larger_dim)
                    bbox_centre = (x, y, 1) # 3x3 in the centre
                    envelop_mean_depth, centre_mean_depth = self.get_mean_depths_from_bounding_boxes(current_depth_image, [bbox_envelop, bbox_centre])

                    if envelop_mean_depth is None:
                        continue

                    if centre_mean_depth is None:
                        pass
                    else:
                        if (np.abs(envelop_mean_depth - centre_mean_depth) / envelop_mean_depth) < MIN_DEPTH_DIFF_RELATIVE_TRESHOLD:
                            continue

                    





                """
                # Old way of doing it.
                # Not tested.

                if USE_DEPTH_DIFF:

                    
                    # Test if the centre of a ring is at aprox the same distance as the ring.
                    # If it is, it's not a ring, but a picture of it.
                    # Watch out for infinite values, which might happen in a ring.
                    

                    # If we haven't gotten the depth image yet, we simply skip eliminate this as a possibility.
                    if current_depth_image is None:
                        continue

                    le_center = np.floor(np.array(le[0])).astype(int)
                    le_first_axis_tip, le_second_axis_tip = self.get_axis_tips_from_elipse(le)

                    le_sample_points = [le_first_axis_tip, le_second_axis_tip, -le_first_axis_tip, -le_second_axis_tip]


                    se_center = np.floor(np.array(se[0])).astype(int)
                    se_first_axis_tip, se_second_axis_tip = self.get_axis_tips_from_elipse(se)

                    se_sample_points = [se_first_axis_tip, se_second_axis_tip, -se_first_axis_tip, -se_second_axis_tip]

                    # avg_center = np.floor(    (le_center + se_center) / 2    ).astype(int)
                    avg_sample_points = [ np.floor(    (le_sample_points[i] + se_sample_points[i]) / 2    ).astype(int) for i in range(4) ]

                    centre_depths = [ self.get_depth_from_point(current_depth_image, le_center), self.get_depth_from_point(current_depth_image, se_center) ]
                    avg_centre_depth = np.mean(centre_depths)

                    sample_depths = [ self.get_depth_from_point(current_depth_image, avg_sample_points[i]) for i in range(4)]
                    avg_sample_depth = np.mean(sample_depths)
                    
                    if np.isinf(avg_sample_depth):
                        print(10*"PROBLEM!!!\n" + "avg_sample_depth is inf - so ring points are inf in depth")
                    
                    if np.isinf(avg_centre_depth):
                        # This means we won and should add it to the candidates.
                        pass
                    else:
                        if np.abs(avg_sample_depth - avg_centre_depth) < MIN_DEPTH_DIFF_TRESHOLD:
                            continue
                """

                    







                candidates.append([le,se])

                # print("gray.shape")
                # print(gray.shape)
                # print("e1")
                # print(e1)
                # print("e2")
                # print(e2)

                # if USE_DEPTH_DIFF:
                #     print("current_depth_image.shape")
                #     print(current_depth_image.shape)
                #     print("avg_sample_points")
                #     print(avg_sample_points)
                #     print("avg_sample_depth")
                #     print(avg_sample_depth)
                #     print("avg_centre_depth")
                #     print(avg_centre_depth)












        # Plot the rings on the image
        for c in candidates:

            # the centers of the ellipses
            e1 = c[0]
            e2 = c[1]

            # drawing the ellipses on the image
            cv2.ellipse(cv_image, e1, (0, 255, 0), 2)
            cv2.ellipse(cv_image, e2, (0, 255, 0), 2)

            # Get a bounding box, around the first ellipse ('average' of both elipsis)
            size = (e1[1][0]+e1[1][1])/2
            center = (e1[0][1], e1[0][0])

            x1 = int(center[0] - size / 2)
            x2 = int(center[0] + size / 2)
            x_min = x1 if x1>0 else 0
            x_max = x2 if x2<cv_image.shape[0] else cv_image.shape[0]

            y1 = int(center[1] - size / 2)
            y2 = int(center[1] + size / 2)
            y_min = y1 if y1 > 0 else 0
            y_max = y2 if y2 < cv_image.shape[1] else cv_image.shape[1]


        if len(candidates) > 0:
                cv2.imshow("Detected rings", cv_image)
                cv2.waitKey(1)
                # input("Waiting")
                return candidates


        return []







    def image_callback(self, data):
        # self.get_logger().info(f"I got a new image! Will try to find rings...")
        


        current_depth_image = self.latest_depth_image


        # Now we look up the transform between the base_link and the map frames
        # and then we apply it to our PointStamped

        time_stamp = data.header.stamp
        
        time_now = rclpy.time.Time()
        timeout = Duration(seconds=0.1)
        try:
            # An example of how you can get a transform from /base_link frame to the /map frame
            # as it is at time_now, wait for timeout for it to become available
            robot_frame_to_map_transform = self.tf_buffer.lookup_transform("map", "base_link", time_now, timeout)
            #self.get_logger().info(f"Looks like the transform is available.")
        
        except TransformException as te:
            self.get_logger().info(f"Could not get the transform: {te}")
            return



        # from ros2 run tf2_tools view_frames
        # oakd_rgb_camera_optical_frame: \n  parent: 'oakd_rgb_camera_frame


        # I think oakd_rgb_camera_optical_frame doesn't take pixels.
        # I think it is just set in the optical center of the camera - the 0,0 of the lens.
        # While oakd_rgb_camera_frame is in the center of the camera or sth.


        timeout = Duration(seconds=0.1)
        try:
            # An example of how you can get a transform from /base_link frame to the /map frame
            # as it is at time_now, wait for timeout for it to become available
            camera_to_robot_frame_transform = self.tf_buffer.lookup_transform("base_link", "oakd_rgb_camera_optical_frame", time_now, timeout)
            #self.get_logger().info(f"Looks like the transform is available.")
        
        except TransformException as te:
            self.get_logger().info(f"Could not get the transform: {te}")
            return







        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)



        # blue = cv_image[:,:,0]
        # green = cv_image[:,:,1]
        # red = cv_image[:,:,2]



        # Otherwise the green elipses added for imshow are added to the image and it messes up color detection.
        curr_color_img = cv_image.copy()



        # Tranform image to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # gray = red




        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        sat_img = hsv_img[:,:,1]
        val_img = hsv_img[:,:,2]

        saturation_treshold = 50
        sat_img = sat_img > saturation_treshold

        value_treshold = 10 # 0 was tried to include black rings, but it only caught insufficiently few pixels on the ring 
        val_img = val_img > value_treshold


        colour_tresholded = np.logical_and(sat_img, val_img)

        # Doesn't work. Everythig is white then.
        # # For black ring detection
        # black_treshold = 15
        # black_img = val_img < black_treshold
        # colour_tresholded = np.logical_or(colour_tresholded, black_img)
        
        colour_tresholded = colour_tresholded.astype(np.uint8)*255


        # cv2.imshow("Colour tresholded", colour_tresholded)
        # cv2.waitKey(1)

        thresholded = colour_tresholded[:colour_tresholded.shape[0]*1//3,:]

        thresholded = cv2.GaussianBlur(thresholded,(3,3),0)


        thresh = colour_tresholded.copy()
        thresh[:,:] = 0
        thresh[:colour_tresholded.shape[0]*1//3,:] = thresholded

        elipse_candidates = self.elipse_check(thresh, gray, current_depth_image, cv_image)
        
        
        
        
        
        # Try the approach with grey otsu tresholding.
        if len(elipse_candidates) == 0:

            # lower 2/3 is all 0s, because rings can't be there
            # So only upper 1/3 is left alone
            # gray[gray.shape[0]*1//3:,:] = 0

            thresholder = gray[:gray.shape[0]*1//3,:]

            # Apply Gaussian Blur
            thresholder = cv2.GaussianBlur(thresholder,(3,3),0)

            # Do histogram equalization
            thresholder = cv2.equalizeHist(thresholder)


            # Binarize the image, there are different ways to do it
            #ret, thresholded = cv2.threshold(thresholder, 50, 255, 0)
            #ret, thresholded = cv2.threshold(thresholder, 70, 255, cv2.THRESH_BINARY)

            ret, thresholded = cv2.threshold(thresholder, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)



            # The background and foreground are switched in the thresholded image
            # Switch them back - only whis way morphologcal operations do what you expect.
            thresholded = cv2.bitwise_not(thresholded)



            # Local tresholding
            # thresholded = cv2.adaptiveThreshold(thresholder, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 15, 30)
            
            # thresh = np.vstack((thresh, np.zeros((gray.shape[0]-thresh.shape[0], gray.shape[1]))))
            
            thresh = gray.copy()
            thresh[:,:] = 0
            thresh[:gray.shape[0]*1//3,:] = thresholded







            # Morphological operations on the image to
            # - get rid of the handle of the ring
            # - make the inner and outer contour of the ring more pronounced
            

            # The holder has to be removed for a ring to be detected.
            # At a large distance, making the size big enough for the holder to be removed, makes the ring not whole and it doesn't get detected anyway.
            # So we need to prioritize closer detections with a larger size.
            # So make the upper bound larger, because it small sizes are only useful for distant rings, where the detection doesn't happen anyway.
            # And increse the maximum size so that the holder is eroded even when close.
            # Although, the rings and holders seem to vary in size and so some rings require smaller kernels.

            size = random.randint(4,12)
            # size = 5
            

            kernel_size = (size, size)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, kernel_size) # cv2.MORPH_RECT

            # New idea: it seems only the ring-end of the holder remains after closing.
            # Solution: just close again. Or maybe add an opening. Or just an erosion.
            
            # thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
            

            thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
            thresh = cv2.erode(thresh, kernel, iterations=1)

            # # Probably needs a smaller kernel.
            # thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
            # thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
            # thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)



            # # Doesn't work, because it malforms the ring too much to stay an elipse:
            # thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
            # thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
            # thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
            # thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
            # thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

            elipse_candidates = self.elipse_check(thresh, gray, current_depth_image, cv_image)
        

        # An elipse candidate is of the form:
        # (((first_elipse_center_x, first_elipse_center_y), (bbox_width?, bbox_height?), first_elipse_angle),
        # ((second_elipse_center_x, second_elipse_center_y), (bbox_width?, bbox_height?), second_elipse_angle))

        # Where the first elipse is the larger one.

        # So we get the centres of the  elipse and pass that tuple, as is needed.
        # centres = [e[0][0] for e in elipse_candidates]
        # But the coordinates are floats, but we need to floor them to integers
        # so we know what pixel to look at in the depth image.
        # centres = [(int(a), int(b)) for a,b in centres]


        # We do all of this just for the larger elipses.
        centres = []
        rough_bounding_boxes = []
        for candidate in elipse_candidates:
            
            x = int(candidate[0][0][0])
            y = int(candidate[0][0][1])


            dim1 = int(candidate[0][1][0])
            dim2 = int(candidate[0][1][1])

            larger_dim = max(dim1, dim2)
            
            centres.append((x, y))
            # centres.append(candidate[])
            rough_bounding_boxes.append((x, y, larger_dim))

        mean_depths = self.get_mean_depths_from_bounding_boxes(current_depth_image, rough_bounding_boxes)

        # print("mean_depths")
        # print(mean_depths)

        binary_thresh = thresh.copy()
        binary_thresh[binary_thresh>0] = 1

        # watch out for BGR
        colors = self.get_decimal_rgb_from_bounding_boxes_of_BGR(curr_color_img, rough_bounding_boxes, binary_thresh)
        

        
        for ix, depth in enumerate(mean_depths):

            if depth == None:
                continue

            coordinates_3d = self.camera_pixels_to_3d(centres[ix][0], centres[ix][1], depth)
            if coordinates_3d is None:
                continue

            point_in_camera_frame = self.point_from_coordinates(coordinates_3d, "oakd_rgb_camera_optical_frame", time_stamp)
            point_in_robot_frame = tfg.do_transform_point(point_in_camera_frame, camera_to_robot_frame_transform)

            # print(f"Depth at centre: {depth}")

            self.publish_marker_from_point(point_in_robot_frame, robot_frame_to_map_transform, colors[ix])









    def depth_callback(self,data):

        # print("data")
        # print(data.header)
        # print(data.encoding)
        # print(data.height)
        # print(data.width)
        # print(data.step)
        # print(data.is_bigendian)

        # input("wait")
        
        # print(data)

        # input("wait")

        # print(data.data)

        # print(5*"\n")

        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)

        
        # print(depth_image.shape)
        # input("wait")


        self.latest_depth_image = depth_image.copy()

        # Visualization in OpenCV:
        depth_image[depth_image==np.inf] = 0
        
        # Do the necessairy conversion so we can visuzalize it in OpenCV
        image_1 = depth_image / 65536.0 * 255
        image_1 = image_1/np.max(image_1)*255

        image_viz = np.array(image_1, dtype= np.uint8)

        # cv2.imshow("Depth window", image_viz)
        # cv2.waitKey(1)


def main():

    rclpy.init(args=None)
    rd_node = RingDetector()

    rclpy.spin(rd_node)

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()