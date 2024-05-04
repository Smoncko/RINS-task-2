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

        cv2.namedWindow("Binary Image", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Detected contours", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Detected rings", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Depth window", cv2.WINDOW_NORMAL)        




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




    def image_callback(self, data):
        # self.get_logger().info(f"I got a new image! Will try to find rings...")

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)


        current_depth_image = self.latest_depth_image

        blue = cv_image[:,:,0]
        green = cv_image[:,:,1]
        red = cv_image[:,:,2]



        # Tranform image to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # gray = red

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






        cv2.imshow("Binary Image", thresh)
        cv2.waitKey(1)

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

        USE_DEPTH_DIFF = False
        MIN_DEPTH_DIFF_TRESHOLD = 5

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

                    """"
                    Test if the centre of a ring is at aprox the same distance as the ring.
                    If it is, it's not a ring, but a picture of it.
                    Watch out for infinite values, which might happen in a ring.
                    """

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


                    







                candidates.append((e1,e2))

                print("gray.shape")
                print(gray.shape)
                print("e1")
                print(e1)
                print("e2")
                print(e2)

                if USE_DEPTH_DIFF:
                    print("current_depth_image.shape")
                    print(current_depth_image.shape)
                    print("avg_sample_points")
                    print(avg_sample_points)
                    print("avg_sample_depth")
                    print(avg_sample_depth)
                    print("avg_centre_depth")
                    print(avg_centre_depth)












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

        if len(candidates)>0:
                cv2.imshow("Detected rings", cv_image)
                cv2.waitKey(1)
                # input("Waiting")









    def depth_callback(self,data):

        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)


        self.latest_depth_image = depth_image

        # Visualization in OpenCV:

        depth_image[depth_image==np.inf] = 0
        
        # Do the necessairy conversion so we can visuzalize it in OpenCV
        image_1 = depth_image / 65536.0 * 255
        image_1 = image_1/np.max(image_1)*255

        image_viz = np.array(image_1, dtype= np.uint8)

        cv2.imshow("Depth window", image_viz)
        cv2.waitKey(1)


def main():

    rclpy.init(args=None)
    rd_node = RingDetector()

    rclpy.spin(rd_node)

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()