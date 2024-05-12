#! /usr/bin/env python3
# Mofidied from Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


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






class RobotCommander(Node):

    def __init__(self, node_name='robot_commander', namespace=''):
        super().__init__(node_name=node_name, namespace=namespace)


        self.pose_frame_id = 'map'
        
        # Flags and helper variables
        self.goal_handle = None
        self.cancel_goal = False
        self.result_future = None
        self.feedback = None
        self.status = None
        self.initial_pose_received = False
        self.is_docked = None

        self.last_destination_goal = ("go", (0.0, 0.0, 0.57))
        self.hello_dist = 0.5
        self.navigation_list = []

        self.faces_greeted = 0


        self.map_np = None
        self.map_data = {"map_load_time":None,
                         "resolution":None,
                         "width":None,
                         "height":None,
                         "origin":None} # origin will be in the format [x,y,theta]
        

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        # # rc.point_hash(point) to point_in_map_frame. This way we don't duplicate points.
        # self.detected_faces = {}
        
        # self.face_marker_sub = self.create_subscription(Marker, "/people_marker", self.face_detect_callback, QoSReliabilityPolicy.BEST_EFFORT)


        # ROS2 subscribers
        self.occupancy_grid_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, qos_profile)

        self.create_subscription(DockStatus,
                                 'dock_status',
                                 self._dockCallback,
                                 qos_profile_sensor_data)
        
        self.localization_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                              'amcl_pose',
                                                              self._amclPoseCallback,
                                                              amcl_pose_qos)
        

        self.face_sub = self.create_subscription(Marker, "/detected_faces", self.face_detected_callback, QoSReliabilityPolicy.BEST_EFFORT)
        self.cylinder_sub = self.create_subscription(Marker, "/detected_cylinder", self.cylinder_detected_callback, QoSReliabilityPolicy.BEST_EFFORT)
        self.ring_sub = self.create_subscription(Marker, "/detected_rings", self.ring_detected_callback, QoSReliabilityPolicy.BEST_EFFORT)
        
        # Here we abuse the Twist message because we don't want to go making our own one.
        self.top_img_stats_sub = self.create_subscription(Twist, "/top_img_stats", self.top_img_stats_callback, QoSReliabilityPolicy.BEST_EFFORT)
        self.latest_top_img_stats = (None, None, None) # (centroid, area, shape)
        self.top_img_changed = False


        

        # ROS2 publishers
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      'initialpose',
                                                      10)

        # ROS2 Action clients
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, 'spin')
        self.undock_action_client = ActionClient(self, Undock, 'undock')
        self.dock_action_client = ActionClient(self, Dock, 'dock')


        # for parking
        self.arm_pub = self.create_publisher(String, "/arm_command", QoSReliabilityPolicy.BEST_EFFORT)
        self.parking_pub = self.create_publisher(Twist, "cmd_vel", QoSReliabilityPolicy.BEST_EFFORT)

        # For waiting by spinning (this allows subscriptions to call callbacks, as opposed to time.sleep(0))
        self.waiting_spin_dir = 1 # vals 1 and -1


        self.get_logger().info(f"Robot commander has been initialized!")
        
    def destroyNode(self):
        self.nav_to_pose_client.destroy()
        super().destroy_node()     


    def map_pixel_to_world(self, x, y, theta=0):
        ### Convert a pixel in an numpy image, to a real world location
        ### Works only for theta=0
        assert not self.map_data["resolution"] is None

        # Apply resolution, change of origin, and translation
        # 
        world_x = x*self.map_data["resolution"] + self.map_data["origin"][0]
        world_y = (self.map_data["height"]-y)*self.map_data["resolution"] + self.map_data["origin"][1]

        # Apply rotation
        return world_x, world_y

    def world_to_map_pixel(self, world_x, world_y, world_theta=0.2):
        ### Convert a real world location to a pixel in a numpy image
        ### Works only for theta=0
        assert self.map_data["resolution"] is not None

        # Apply resolution, change of origin, and translation
        # x is the first coordinate, which in opencv (numpy) that is the matrix row - vertical
        x = int((world_x - self.map_data["origin"][0])/self.map_data["resolution"])
        y = int(self.map_data["height"] - (world_y - self.map_data["origin"][1])/self.map_data["resolution"] )
        
        # Apply rotation
        return x, y
    
    def parking_camera(self):
        msg = String()
        msg.data = "look_for_parking"
        self.arm_pub.publish(msg)
    
    def normal_camera(self):
        msg = String()
        msg.data = "garage"
        self.arm_pub.publish(msg)


    def face_detected_callback(self, msg):
        
        self.info("Face detected!")

        face_location = np.array([msg.pose.position.x, msg.pose.position.y])

        curr_pos = self.get_curr_pos()  
        curr_pos_location = np.array([curr_pos.point.x, curr_pos.point.y])

        vec_to_face_normed = face_location - curr_pos_location
        vec_to_face_normed /= np.linalg.norm(vec_to_face_normed)

        face_goal_location = face_location - self.hello_dist * vec_to_face_normed

        fi = np.arctan2(vec_to_face_normed[1], vec_to_face_normed[0])

        add_to_navigation = [
            ("go", (face_goal_location[0], face_goal_location[1], fi)),
            ("say_hi", 0),
            ("spin", 3.14),
            self.last_destination_goal
        ]

        self.prepend_to_nav_list(add_to_navigation, spin_full_after_go=False)

        self.cancel_goal = True
        # self.cancelTask()

    def ring_detected_callback(self, msg):
        self.info("Ring detected!")

        r = int(msg.color.r * 255)
        g = int(msg.color.g * 255)
        b = int(msg.color.b * 255)

        h, s, v = self.rgb2hsv(r, g, b)

        color = ""
        if(v < 0.1):
            color = "black"
        elif h < 15 or h > 350:
            color = "red"
        elif h > 20 and h < 65:
            color = "yellow"
        elif h > 65 and h < 150:
            color = "green"
        elif h > 180 and h < 265:
            color = "blue"

        string_to_say = color + " ring."

        add_to_navigation = [
            ("say_color", string_to_say),
        ]


        if color == "green":

            ring_location = np.array([msg.pose.position.x, msg.pose.position.y])

            curr_pos = self.get_curr_pos()
            while curr_pos is None:
                print("Waiting for point...")
                time.sleep(0)
                curr_pos = self.get_curr_pos()

            curr_pos_location = np.array([curr_pos.point.x, curr_pos.point.y])

            vec_to_face_normed = ring_location - curr_pos_location
            vec_to_face_normed /= np.linalg.norm(vec_to_face_normed)

            fi = np.arctan2(vec_to_face_normed[1], vec_to_face_normed[0])

            add_to_navigation.append(    ("go", (ring_location[0], ring_location[1], fi))    )
            add_to_navigation.append(("park", None))

            self.parking_camera()
            
        add_to_navigation.append(self.last_destination_goal)


        self.prepend_to_nav_list(add_to_navigation, spin_full_after_go=False)

        self.cancel_goal = True
        # self.cancelTask()


    def cylinder_detected_callback(self, msg):
        
        self.info("Cylinder detected!")

        r = int(msg.color.r * 255)
        g = int(msg.color.g * 255)
        b = int(msg.color.b * 255)

        h, s, v = self.rgb2hsv(r, g, b)

        color = ""
        if(v < 0.1):
            color = "black"
        elif h < 15 or h > 350:
            color = "red"
        elif h > 20 and h < 65:
            color = "yellow"
        elif h > 65 and h < 150:
            color = "green"
        elif h > 180 and h < 265:
            color = "blue"

        string_to_say = color + " cylinder"

        add_to_navigation = [
            ("say_color", string_to_say),
            self.last_destination_goal
        ]

        self.prepend_to_nav_list(add_to_navigation, spin_full_after_go=False)

        self.cancel_goal = True
        # self.cancelTask()


    def top_img_stats_callback(self, msg):

        # We decided to abuse the twist message so we don't have to make our own one.

        # The ordering is the same that we get from get_area_and_centroid() and img.shape in image_gatherer.py
        # Names msg.linear.x don't necessarily mean the x coodrinate. It is simply the first of the two components.

        centroid = (msg.linear.x, msg.linear.y)
        area = msg.linear.z
        shape = (int(msg.angular.x), int(msg.angular.y))

        self.latest_top_img_stats = (centroid, area, shape)
        self.top_img_changed = True
    
    
    def get_latest_top_img_stats(self):
        top_img_has_changed = self.top_img_changed
        self.top_img_changed = False
        return self.latest_top_img_stats[0], self.latest_top_img_stats[1], self.latest_top_img_stats[2], top_img_has_changed
    

    def get_top_img_stats_with_waiting_for_change(self):

        centroid, area, shape, top_img_has_changed = self.get_latest_top_img_stats()

        is_none_present = area is None or centroid is None or shape is None

        cycle_duration = 1000    # miliseconds
        cycle_start_time = time.time()
        while is_none_present or not top_img_has_changed:
            
            if (time.time() - cycle_start_time) >= cycle_duration:
                cycle_start_time = time.time()
                print("Waiting for new image stats...")
            
            self.wait_by_spinning()
            centroid, area, shape, top_img_has_changed = self.get_latest_top_img_stats()
        
        
        
        return centroid, area, shape
    

    def rgb2hsv(self, r, g, b):

        c_high = max(r, max(g, b))
        c_low = min(r, min(g, b))
        c_rng = c_high - c_low

        s = 0
        if c_high > 0:
            s = c_rng / c_high

        v = c_high / 255

        r1 = (c_high - r) / c_rng
        g1 = (c_high - g) / c_rng
        b1 = (c_high - b) / c_rng

        h1 = 0
        if r == c_high:
            h1 = b1 - g1
        elif g == c_high:
            h1 = r1 - b1 + 2
        elif b == c_high:
            h1 = g1 - r1 + 4

        h = 0
        if c_rng == 0:
            h = 0
        elif h1 < 0:
            h = (h1 + 6) / 6
        else:
            h = h1 / 6

        h = h * 360

        return h, s, v

    def map_callback(self, msg):
            self.get_logger().info(f"Read a new Map (Occupancy grid) from the topic.")
            # reshape the message vector back into a map
            self.map_np = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
            # fix the direction of Y (origin at top for OpenCV, origin at bottom for ROS2)
            self.map_np = np.flipud(self.map_np)
            # change the colors so they match with the .pgm image
            self.map_np[self.map_np==0] = 127
            self.map_np[self.map_np==100] = 0
            # load the map parameters
            self.map_data["map_load_time"]=msg.info.map_load_time
            self.map_data["resolution"]=msg.info.resolution
            self.map_data["width"]=msg.info.width
            self.map_data["height"]=msg.info.height
            quat_list = [msg.info.origin.orientation.x,
                        msg.info.origin.orientation.y,
                        msg.info.origin.orientation.z,
                        msg.info.origin.orientation.w]
            self.map_data["origin"]=[msg.info.origin.position.x,
                                    msg.info.origin.position.y,
                                    tf_transformations.euler_from_quaternion(quat_list)[-1]]
            #self.get_logger().info(f"Read a new Map (Occupancy grid) from the topic.")


    def get_curr_pos(self):
            # Create a PointStamped in the /base_link frame of the robot
            # The point is located 0.5m in from of the robot
            # "Stamped" means that the message type contains a Header
            point_in_robot_frame = PointStamped()
            point_in_robot_frame.header.frame_id = "/base_link"
            point_in_robot_frame.header.stamp = self.get_clock().now().to_msg()

            point_in_robot_frame.point.x = 0.
            point_in_robot_frame.point.y = 0.
            point_in_robot_frame.point.z = 0.

            # Now we look up the transform between the base_link and the map frames
            # and then we apply it to our PointStamped


            time_now = rclpy.time.Time()
            timeout =rclpyDuration(seconds=0.1)
            try:
                # An example of how you can get a transform from /base_link frame to the /map frame
                # as it is at time_now, wait for timeout for it to become available
                trans = self.tf_buffer.lookup_transform("map", "base_link", time_now, timeout)
                self.get_logger().info(f"Looks like the transform is available.")

                # Now we apply the transform to transform the point_in_robot_frame to the map frame
                # The header in the result will be copied from the Header of the transform
                point_in_map_frame = tfg.do_transform_point(point_in_robot_frame, trans)
                self.get_logger().info(f"We transformed a PointStamped!")

                return point_in_map_frame


            except TransformException as te:
                self.get_logger().info(f"Cound not get the transform: {te}")
    

    def create_marker(self, point_stamped, marker_id):
        """You can see the description of the Marker message here: https://docs.ros2.org/galactic/api/visualization_msgs/msg/Marker.html"""
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

    def make_cv2_window(self):
        cv2.namedWindow("Just for showing what is in rc.map_np", cv2.WINDOW_NORMAL)

    def show_map(self):
        cv2.imshow("Just for showing what is in rc.map_np", self.map_np)
        cv2.waitKey(1000)

    def goToPose(self, pose, behavior_tree=''):
        """Send a `NavToPose` action request."""
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = behavior_tree

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                  str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                   self._feedbackCallback)
        
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
                       str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True


    def cancelTask(self):
        """Cancel pending task request of any type."""
        self.info('Canceling current task.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return


    def spin(self, spin_dist=1.57, time_allowance=10, print_info=True):
        self.debug("Waiting for 'Spin' action server")
        while not self.spin_client.wait_for_server(timeout_sec=1.0):
            self.info("'Spin' action server not available, waiting...")
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = spin_dist
        goal_msg.time_allowance = Duration(sec=time_allowance)

        if print_info:
            self.info(f'Spinning to angle {goal_msg.target_yaw}....')
        send_goal_future = self.spin_client.send_goal_async(goal_msg, self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Spin request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True


    def wait_by_spinning(self, spin_dist=2*3.14 * 1e-7, spin_seconds=1):
        # Keep spin seconds low so that we remain in almost the same place.
        # Spin seconds has to be int.
        spin_dir = self.waiting_spin_dir
        self.waiting_spin_dir = -self.waiting_spin_dir

        self.spin(spin_dir * spin_dist, spin_seconds, print_info=False)
        



    def undock(self):
        """Perform Undock action."""
        self.info('Undocking...')
        self.undock_send_goal()

        while not self.isUndockComplete():
            time.sleep(0.1)

    def undock_send_goal(self):
        goal_msg = Undock.Goal()
        self.undock_action_client.wait_for_server()
        goal_future = self.undock_action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, goal_future)

        self.undock_goal_handle = goal_future.result()

        if not self.undock_goal_handle.accepted:
            self.error('Undock goal rejected')
            return

        self.undock_result_future = self.undock_goal_handle.get_result_async()

    def isUndockComplete(self):
        """
        Get status of Undock action.

        :return: ``True`` if undocked, ``False`` otherwise.
        """
        if self.undock_result_future is None or not self.undock_result_future:
            return True

        rclpy.spin_until_future_complete(self, self.undock_result_future, timeout_sec=0.1)

        if self.undock_result_future.result():
            self.undock_status = self.undock_result_future.result().status
            if self.undock_status != GoalStatus.STATUS_SUCCEEDED:
                self.info(f'Goal with failed with status code: {self.status}')
                return True
        else:
            return False

        self.info('Undock succeeded')
        return True

    def isTaskComplete(self):
        """Check if the task request of any type is complete yet."""

        # self.info('Checking if task is complete')
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.debug(f'Task with failed with status code: {self.status}')
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.debug('Task succeeded!')
        return True

    def getFeedback(self):
        """Get the pending action feedback message."""
        return self.feedback

    def getResult(self):
        """Get the pending action result message."""
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return TaskResult.SUCCEEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            return TaskResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            return TaskResult.CANCELED
        else:
            return TaskResult.UNKNOWN

    def waitUntilNav2Active(self, navigator='bt_navigator', localizer='amcl'):
        """Block until the full navigation system is up and running."""
        self._waitForNodeToActivate(localizer)
        if not self.initial_pose_received:
            time.sleep(1)
        self._waitForNodeToActivate(navigator)
        self.info('Nav2 is ready for use!')
        return

    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.debug(f'Waiting for {node_name} to become active..')
        node_service = f'{node_name}/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(f'{node_service} service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while state != 'active':
            self.debug(f'Getting {node_name} state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug(f'Result of get_state: {state}')
            time.sleep(2)
        return
    
    def YawToQuaternion(self, angle_z = 0.):
        quat_tf = quaternion_from_euler(0, 0, angle_z)

        # Convert a list to geometry_msgs.msg.Quaternion
        quat_msg = Quaternion(x=quat_tf[0], y=quat_tf[1], z=quat_tf[2], w=quat_tf[3])
        return quat_msg

    def _amclPoseCallback(self, msg):
        self.debug('Received amcl pose')
        self.initial_pose_received = True
        self.current_pose = msg.pose
        return

    def _feedbackCallback(self, msg):
        self.debug('Received action feedback message')
        self.feedback = msg.feedback
        return
    
    def _dockCallback(self, msg: DockStatus):
        self.is_docked = msg.is_docked

    def setInitialPose(self, pose):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = pose
        msg.header.frame_id = self.pose_frame_id
        msg.header.stamp = 0
        self.info('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)
        return




    def cmd_vel(self, direction, miliseconds, linear_velocity=20.0):
        # Directions: "forward", "right", "left", "backward"
        # miliseconds: how long to move

        velocity = Twist()

        if direction == "forward":
            velocity.linear.x = linear_velocity
        elif direction == "backward":
            velocity.linear.x = -linear_velocity
        elif direction == "left":
            velocity.angular.z = -0.5
        elif direction == "right":
            velocity.angular.z = 0.5
        
        
        self.parking_pub.publish(velocity)


        # I think the velocity persists if you don't reset it.

        duration = miliseconds/1000 # to get seconds

        start_time = time.time()
        while (time.time() - start_time) < duration:
            time.sleep(0)

        velocity = Twist()
        self.parking_pub.publish(velocity)

    def park(self):

        print("Here 1")
        
        centroid, area, shape = self.get_top_img_stats_with_waiting_for_change()

        angles = []
        areas_at_angles = []

        print("Here 2")

        point_in_map_frame = self.get_curr_pos()
        while point_in_map_frame is None:
            print("Waiting for point...")
            time.sleep(0)
            point_in_map_frame = self.get_curr_pos()

        x = point_in_map_frame.point.x
        y = point_in_map_frame.point.y

        num_of_angles = 8
        for i in range(num_of_angles):
            fi = i * 2*3.14 / num_of_angles
            pose = self.get_pose_obj(x, y, fi)
            self.goToPose(pose)

            while not self.isTaskComplete():
                time.sleep(0)
            
            centroid, area, shape = self.get_top_img_stats_with_waiting_for_change()
            
            angles.append(fi)
            areas_at_angles.append(area)
        
        print("Here 3")

        max_area_ix = areas_at_angles.index(max(areas_at_angles))
        chosen_fi = angles[max_area_ix]

        pose = self.get_pose_obj(x, y, chosen_fi)
        self.goToPose(pose)

        while not self.isTaskComplete():
            time.sleep(0)



        print("Here 4")

        acceptable_errors = [5, 3]
        acceptable_areas = [5000, 1000]

        for i in range(len(acceptable_errors)):
            self.top_camera_centre_robot_to_blob_centre(acceptable_error=acceptable_errors[i], milliseconds=100, printout=True)
            print("Here 5")
            self.top_camera_reduce_blob_area(acceptable_area=acceptable_areas[i], milliseconds=100, printout=True)



        # img_max_y = curr_img.shape[0]

        # while not(np.abs(centroid[1] - img_max_y) < 5):
            
        #     self.cmd_vel("forward", 100)
            
        #     curr_img = self.get_white_pixels_treshold(self.image_gatherer.get_latest_img())
        #     centroid, area = self.get_area_and_centroid(curr_img) 



        

        print("Here 6")

        print("Parking complete!")
        



        

    def top_camera_centre_robot_to_blob_centre(self, acceptable_error=10, milliseconds=15, printout=False):

        centroid, _, shape = self.get_top_img_stats_with_waiting_for_change()        
        
        img_middle_x = shape[1] / 2

        while not(np.abs(centroid[0] - img_middle_x) < acceptable_error):
            
            if(printout):
                print("Centroid[0]: ", centroid[0])
                print("img_middle_x: ", img_middle_x)
                print("np.abs(centroid[0] - img_middle_x):")
                print(np.abs(centroid[0] - img_middle_x))

            if centroid[0] < img_middle_x:
                self.cmd_vel("right", milliseconds)
            else:
                self.cmd_vel("left", milliseconds)

            centroid, _, shape = self.get_top_img_stats_with_waiting_for_change()


    def top_camera_reduce_blob_area(self, acceptable_area=100, milliseconds=1, printout=False):

        _, area, _ = self.get_top_img_stats_with_waiting_for_change()

        while area > acceptable_area:
            if(printout):
                print("Area: ", area)

            self.cmd_vel("forward", milliseconds)
            _, area, _ = self.get_top_img_stats_with_waiting_for_change()
        
        






    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return


    def get_pose_obj(self, x, y, fi):

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation = self.YawToQuaternion(fi)

        return goal_pose
    

    def add_to_nav_list(self, to_add_list, spin_full_after_go=False):

        for tup in to_add_list:
            if tup[0] == "go":
                self.navigation_list.append(("go", self.get_pose_obj(*tup[1]), tup[1]))
                if spin_full_after_go:
                    self.navigation_list.append(("spin", 6.28, None))
            elif tup[0] == "spin":
                self.navigation_list.append(("spin", tup[1], None))
            elif tup[0] == "say_hi":
                self.navigation_list.append(("say_hi", None, None))
            elif tup[0] == "say_color":
                self.navigation_list.append(("say_color", tup[1], None))
            elif tup[0] == "park":
                self.navigation_list.append(("park", None, None))


    def prepend_to_nav_list(self, to_add_list, spin_full_after_go=False):

        for tup in reversed(to_add_list):
            if tup[0] == "go":
                self.navigation_list.insert(0, ("go", self.get_pose_obj(*tup[1]), tup[1]))
                if spin_full_after_go:
                    self.navigation_list.insert(0, ("spin", 6.28, None))
            elif tup[0] == "spin":
                self.navigation_list.insert(0, ("spin", tup[1], None))
            elif tup[0] == "say_hi":
                self.navigation_list.insert(0, ("say_hi", None, None))
            elif tup[0] == "say_color":
                self.navigation_list.insert(0, ("say_color", tup[1], None))
            elif tup[0] == "park":
                self.navigation_list.insert(0, ("park", None, None))

    def say_hi(self):
        playsound("src/RINS-task-2/voice/zivjo.mp3")
        self.faces_greeted += 1
    
    def say_color(self, color: str):
        
        self.info(color)

        mp3_fp = BytesIO()
        tts = gTTS(color, lang="en")
        tts.write_to_fp(mp3_fp)
        mp3_fp.seek(0)

        pygame.init()
        pygame.mixer.init()
        pygame.mixer.music.load(mp3_fp)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            pygame.time.Clock().tick(10)
        
        #Če Pygame ne dela:
        #tts.save("color.mp3")
        #playsound("/color.mp3")

        #Če Playsound ne dela:
        #v = vlc.MediaPlayer("/color.mp3")
        #v.play()





def main(args=None):

    rclpy.init(args=args)

    rc = RobotCommander()

    # Wait until Nav2 and Localizer are available
    rc.waitUntilNav2Active()


    rc.parking_camera()

    # Check if the robot is docked, only continue when a message is recieved
    while rc.is_docked is None:
        rclpy.spin_once(rc, timeout_sec=0.5)

    # If it is docked, undock it first
    if rc.is_docked:
        rc.undock()
    










    # The mesh in rviz is coordinates.
    # The docking station is 0,0
    # Use Publish Point to hover and see the coordinates.
    # x: -2 to 4
    # y: -2.5 to 5
        

    # contains tuples of three types:
    # ("go", <PoseStamped object>), ("spin", angle_to_spin_by), ("say_hi", None)
    

    # UP = 0.0
    # RIGHT = 1.57
    # DOWN = 3.14
    # LEFT = 4.71

    UP = 0.0
    LEFT = 1.57
    DOWN = 3.14
    RIGHT = 4.71

    add_to_navigation = [
        
        # For testing
        ("go", (-1.06, 0.24, DOWN)),
        ("park", None),

        ("go", (-0.65, 0., DOWN)),

        ("go", (1.1, -2., UP)),

        ("go", (3.5, -1.3, DOWN)),

        ("go", (2.8, -0.8, LEFT)),

        ("go", (2.35, 1.35, RIGHT)),

        ("go", (1.6, 0., DOWN)),

        ("go", (0.95, 1.75, DOWN)),

        ("go", (-1.08, 0.92, DOWN)),

        ("go", (-1.35, 3.35, RIGHT)),

        ("go", (0.5, 3.2, UP)),

        ("go", (2.15, 1.8, RIGHT)),

    ]

    rc.add_to_nav_list(add_to_navigation, spin_full_after_go=False)



    while len(rc.navigation_list) > 0:


        print("\n\n")
        print(rc.navigation_list[0][0])
        print(rc.navigation_list[0][2])
        print("Goals following it:")

        desired_num_of_following_goals = 2
        for i in range(1, desired_num_of_following_goals+1):
            if i < len(rc.navigation_list):
                print(rc.navigation_list[i][0])
                print(rc.navigation_list[i][2])
                if i == desired_num_of_following_goals:
                    print("And more...")

        print("\n\n")






        curr_type, curr_goal, curr_goal_coordinates = rc.navigation_list[0]
        
        if curr_type == "go":

            rc.last_destination_goal = (curr_type, curr_goal_coordinates)
            rc.goToPose(curr_goal)

        elif curr_type == "spin":
            rc.spin(curr_goal)

        elif curr_type == "say_hi":
            rc.say_hi()
            if rc.faces_greeted == 3:
                break
        
        elif curr_type == "say_color":
            rc.say_color(curr_goal)
        
        elif curr_type == "park":
            rc.parking_camera()
            rc.park()
        

        del rc.navigation_list[0]



        printout_counter = 0
        while not rc.isTaskComplete():

            # if mg.clicked or rc.stop_spin:
            #     rc.cancel_goal()
            if rc.cancel_goal:
                rc.cancel_goal = False
                rc.cancelTask()

            if printout_counter % 3 == 0:
                rc.info("Waiting for the task to complete...")
            printout_counter += 1

            time.sleep(1)

        
        
    
        # input("Enter sth to continue.")
        
    
    input("Navigation list completed, waiting to terminate. Enter anything.")
        

    """# Example
    if False:    
        # Finally send it a goal to reach
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = rc.get_clock().now().to_msg()

        goal_pose.pose.position.x = 2.6
        goal_pose.pose.position.y = -1.3
        goal_pose.pose.orientation = rc.YawToQuaternion(0.57)

        rc.goToPose(goal_pose)

        while not rc.isTaskComplete():
            rc.info("Waiting for the task to complete...")
            time.sleep(1)

        rc.spin(-0.57)"""


    rc.destroyNode()

    # And a simple example
if __name__=="__main__":
    main()