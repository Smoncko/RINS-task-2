#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
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

from ultralytics import YOLO

# from rclpy.parameter import Parameter
# from rcl_interfaces.msg import SetParametersResult

class detect_faces(Node):

    def __init__(self):
        super().__init__('detect_faces')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('device', ''),
        ])

        self.marker_topic = "/detected_faces"

        self.detection_color = (0,0,255)
        self.device = self.get_parameter('device').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.scan = None

        # For listening and loading the 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # For publishing the markers
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, QoSReliabilityPolicy.BEST_EFFORT)

        # For face detection
        self.rgb_image_sub = self.create_subscription(Image, "/oakd/rgb/preview/image_raw", self.rgb_callback, qos_profile_sensor_data)
        self.pointcloud_sub = self.create_subscription(PointCloud2, "/oakd/rgb/preview/depth/points", self.pointcloud_callback, qos_profile_sensor_data)
        self.model = YOLO("yolov8n.pt")

        # all detected faces by YOLO
        self.faces = []

        self.buffer_size = 10
        self.face_buffer = []

        # for unique faces
        self.marker_id = 0
        self.min_distance_between_faces = 1.
        self.detected_faces = []

        self.get_logger().info(f"Node has been initialized! Will publish face markers to {self.marker_topic}.")

    def rgb_callback(self, data):

        self.faces = []

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            #self.get_logger().info(f"Running inference on image...")

            # run inference
            res = self.model.predict(cv_image, imgsz=(256, 320), show=False, verbose=False, classes=[0], device=self.device)

            # iterate over results
            for x in res:
                bbox = x.boxes.xyxy
                if bbox.nelement() == 0: # skip if empty
                    continue

                #self.get_logger().info(f"Person has been detected!")

                bbox = bbox[0]

                # draw rectangle
                cv_image = cv2.rectangle(cv_image, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), self.detection_color, 3)

                cx = int((bbox[0]+bbox[2])/2)
                cy = int((bbox[1]+bbox[3])/2)

                # draw the center of bounding box
                cv_image = cv2.circle(cv_image, (cx,cy), 5, self.detection_color, -1)

                self.faces.append((cx,cy))

            cv2.imshow("image", cv_image)
            key = cv2.waitKey(1)
            if key==27:
                print("exiting")
                exit()
            
        except CvBridgeError as e:
            print(e)
            
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
        # check if marker is far enough from the already detected faces
        for face in self.detected_faces:
            if self.distance(marker, face) < self.min_distance_between_faces:
                return False
        
        return True
    
    def notFalsePositive(self, marker):
        # check if enough markers have been detected to prevent a false positive

        self.face_buffer.append(marker)
        if len(self.face_buffer) < self.buffer_size + 1:
            return False

        face_counter = 0
        for face in self.face_buffer:
            if self.distance(marker, face) < self.min_distance_between_faces / 2:
                face_counter += 1

        self.face_buffer.pop(0)

        if face_counter < self.buffer_size // 3:
            return False
        return True
    
    def publish_face(self, point_in_robot_frame):

        # Now we look up the transform between the base_link and the map frames
        # and then we apply it to our PointStamped
        time_now = rclpy.time.Time()
        timeout = Duration(seconds=0.1)
        try:
            # An example of how you can get a transform from /base_link frame to the /map frame
            # as it is at time_now, wait for timeout for it to become available
            trans = self.tf_buffer.lookup_transform("map", "base_link", time_now, timeout)
            #self.get_logger().info(f"Looks like the transform is available.")

            # Now we apply the transform to transform the point_in_robot_frame to the map frame
            # The header in the result will be copied from the Header of the transform
            point_in_map_frame = tfg.do_transform_point(point_in_robot_frame, trans)
            #self.get_logger().info(f"We transformed a PointStamped!")

            # If the transformation exists, create a marker from the point, in order to visualize it in Rviz
            marker_in_map_frame = self.create_marker(point_in_map_frame, self.marker_id)

            if not math.isnan(marker_in_map_frame.pose.position.x) and not math.isnan(marker_in_map_frame.pose.position.y) and self.new(marker_in_map_frame):

                # check if face already detected
                if self.notFalsePositive(marker_in_map_frame):

                    # if it's new, append it to detected faces
                    self.detected_faces.append(marker_in_map_frame)

                    # Publish the marker
                    self.marker_pub.publish(marker_in_map_frame)
                    self.get_logger().info(f"The marker has been published to {self.marker_topic}. You are able to visualize it in Rviz")
                    #self.get_logger().info(f"x: {marker_in_map_frame.pose.position.x}, y: {marker_in_map_frame.pose.position.y}, z: {marker_in_map_frame.pose.position.z}")
                    
                    for face in self.detected_faces:
                        self.get_logger().info(f"x: {face.pose.position.x}, y: {face.pose.position.y}, z: {face.pose.position.z}")

                    # Increase the marker_id, so we dont overwrite the same marker.
                    self.marker_id += 1

                #else:
                    #self.get_logger().info(f"Face has already been detected")

        except TransformException as te:
            self.get_logger().info(f"Could not get the transform: {te}")

    def create_marker(self, point_stamped, marker_id):
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
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Set the pose of the marker
        marker.pose.position.x = point_stamped.point.x
        marker.pose.position.y = point_stamped.point.y
        marker.pose.position.z = point_stamped.point.z

        return marker
    

    def pointcloud_callback(self, data):

        # get point cloud attributes
        height = data.height
        width = data.width
        point_step = data.point_step
        row_step = data.row_step		

        # iterate over face coordinates
        for x,y in self.faces:

            # get 3-channel representation of the poitn cloud in numpy format
            a = pc2.read_points_numpy(data, field_names= ("x", "y", "z"))
            a = a.reshape((height,width,3))

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

            self.publish_face(point_in_robot_frame)

            
def main():
    print('Face detection node starting.')

    rclpy.init(args=None)
    node = detect_faces()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()