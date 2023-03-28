#!/usr/bin/env python3
import numpy as np
import cv2
import os
import cv_bridge
import rospy

from std_msgs.msg import String
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker

from task_node import TaskNode


class WaterGunTaskNode(TaskNode):

    def __init__(self, detection_method='combined', bag_bounds=(250, 300), flip_point=(360, 640)):

        #########################################
        ######## Execution Debug Mode ###########
        #########################################

        self.debug = True
        super().__init__('water_gun_task')
        self.bridge = cv_bridge.CvBridge()

        ########################################
        ############# PUBLISHERS ###############
        ########################################

        self.center_pub = rospy.Publisher(
            '/pilot_suite/water_gun_task/target_center_pose', Point, queue_size=1)
        self.marker_pub = rospy.Publisher(
            '/pilot_suite/water_gun_task/debug/marker_pub', Marker, queue_size=1)

        if(self.debug):
            self.image_segmented_pub = rospy.Publisher(
                '/pilot_suite/water_gun_task/debug/segmented_img', Image, queue_size=1)
            self.image_filtered_pub = rospy.Publisher(
                '/pilot_suite/water_gun_task/debug/filtered_img', Image, queue_size=1)
            self.image_outliers_pub = rospy.Publisher(
                '/pilot_suite/water_gun_task/debug/outliers_img', Image, queue_size=1)

        # self.velocity_pub = rospy.Publisher('pilot_suite/velocity_command',VelocityCommand, queue_size=1 )
        self.detection_method = detection_method
        self.bag_bounds = bag_bounds
        self.flip_point = flip_point
        self.flip_index = -1

        # Subscribe to approximatley synchornized depth and rgb images
        depth_sub = Subscriber('/zed2i/zed_node/depth/depth_registered', Image)
        rgb_sub = Subscriber('/zed2i/zed_node/rgb/image_rect_color', Image)

        self.ats = ApproximateTimeSynchronizer(
            [depth_sub, rgb_sub], queue_size=1, slop=.1)

        self.ats.registerCallback(self.callback)
        self.bag_positions = []

        self.SHRINK_FACTOR = 4
        self.BLUE_DEVIATION_THRESHOLD = 30
        self.DEPTH_THRESHOLD = 10
        self.BLUE_CONSTANT = 120
        self.SV_THRESHOLD = 100
        self.VAR_THRESHOLD = 0.7

        self.COUNT_THRESHOLD = 100

        self.MIN_DEPTH = 1
        self.MAX_DEPTH = 15.0

        #everything in terms of meters
        self.pixel_width = 0
        self.s = 0.3 # x distance between camera and water gun (water gun to the left of camera, so smaller pixel numbers)
        self.y = 0.4 # y distance between camera and water gun (water gun below camera, so higher pixel numbers)
        self.D = 0.5
        self.fov = 2.0944 # this is 120 degrees as the zed 2i advertises
        self.center_history = []

    def depth_and_sv_mask(self, original_img, depth_img):

        res_img = np.copy(original_img)

        res_img[depth_img > self.DEPTH_THRESHOLD] = np.zeros(3)
        res_img[res_img[:, :, 0] < self.BLUE_CONSTANT - 30] = np.zeros(3)
        res_img[res_img[:, :, 0] > self.BLUE_CONSTANT + 30] = np.zeros(3)
        res_img[res_img[:, :, 1] < self.SV_THRESHOLD] = np.zeros(3)
        res_img[res_img[:, :, 2] < self.SV_THRESHOLD] = np.zeros(3)

        return res_img

    def identify_center(self, input_img):

        not_detected = (None, None)

        if self.debug:
            not_detected = ((None, None), None, None)

        input_img = np.array(input_img)

        min_diff = np.min(
            np.abs(input_img[:, :, 0] - 120 * np.ones(input_img.shape[:2])))

        blue_color = self.BLUE_CONSTANT + (min_diff if (120 + min_diff)
                                           in input_img[:, :, 0] else -min_diff)

        if abs(blue_color - 120) > self.BLUE_DEVIATION_THRESHOLD:
            return not_detected

        blues = np.argwhere(np.abs(input_img - blue_color) < 1)
        if len(blues) <= self.COUNT_THRESHOLD:
            return not_detected

        x_blues, y_blues = blues[:, 0], blues[:, 1]
        if self.debug:
            filtered_img = input_img.copy()

            for i in range(input_img.shape[0]):
                for j in range(input_img.shape[1]):
                    if (abs(input_img[i][j][0] - blue_color) < 1):
                        continue
                    else:
                        filtered_img[i, j, :] = [0, 0, 0]

        def remove_outliers(x_blues, y_blues):

            x_mean = np.mean(x_blues)
            y_mean = np.mean(y_blues)

            variances = np.square(x_blues - x_mean) + np.square(y_blues - y_mean)

            mean_var = np.mean(variances)
            if mean_var == 0:
                return x_blues, y_blues

            return x_blues[variances / mean_var <= self.VAR_THRESHOLD], y_blues[variances / mean_var <= self.VAR_THRESHOLD]

        x_blues, y_blues = remove_outliers(x_blues, y_blues)
        if len(x_blues) <= self.COUNT_THRESHOLD or len(y_blues) <=self.COUNT_THRESHOLD:
            return not_detected

        # convert location to location relative to center of the frame

        if self.debug:
            postoutliers_img = filtered_img.copy()
            for i in range(len(postoutliers_img)):
                for j in range(len(postoutliers_img[0])):
                    postoutliers_img[i, j, :] = np.zeros(3)
            for i in range(len(x_blues)):
                postoutliers_img[x_blues[i], y_blues[i],
                                 :] = np.array([120, 100, 100])

            return (int(self.SHRINK_FACTOR * np.median(x_blues)),
                    int(self.SHRINK_FACTOR * np.median(y_blues))), filtered_img, postoutliers_img

        else:

            return (int(self.SHRINK_FACTOR * np.median(x_blues)),
                    int(self.SHRINK_FACTOR * np.median(y_blues)), )

    def segment_image(self, input_img):

        scaled_dim = (input_img.shape[1] // self.SHRINK_FACTOR,
                      input_img.shape[0] // self.SHRINK_FACTOR)

        img = cv2.resize(input_img, scaled_dim, interpolation=cv2.INTER_AREA)

        twoDimage = np.float32(img.reshape((-1, 3)))

        criteria = (cv2.TERM_CRITERIA_EPS +
                    cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)

        K = 7
        attempts = 1

        _, label, center = cv2.kmeans(
            twoDimage, K, None, criteria, attempts, cv2.KMEANS_PP_CENTERS)
        center = np.uint8(center)
        res = center[label.flatten()]

        result_image = res.reshape((img.shape))

        return result_image

    def callback(self, depth_img, img):
        print("in call back now")
        if not self.active:
            return
        # Convert depth and rgb image using cvBridge
        try:
            depth_img = self.bridge.imgmsg_to_cv2(depth_img, "32FC1")
            img = cv2.cvtColor(self.bridge.imgmsg_to_cv2(
                img, "bgr8"), cv2.COLOR_BGR2HSV)
        except cv_bridge.CvBridgeError as e:
            rospy.loginfo(e)

        H, W, _ = img.shape
        mid_x = W // 2
        mid_y = H // 2

        segmented_img = self.segment_image(
            self.depth_and_sv_mask(img, depth_img))

        if self.debug:

            target_center, filtered_img, postoutliers_img = self.identify_center(segmented_img)

            if filtered_img is None:
                filtered_img = np.zeros((H, W, 3), dtype=np.uint8)
                print("No filtered image exists!")
            if postoutliers_img is None:
                postoutliers_img = np.zeros((H, W, 3), dtype=np.uint8)
                print("No post-outliers image exists!")

            if target_center != (None, None):

                for i in range(-20, 20):
                    for j in range(-20, 20):
                        try:
                            segmented_img[(i+target_center[0])//self.SHRINK_FACTOR,
                                          (j+target_center[1])//self.SHRINK_FACTOR] = np.array([0, 100, 75])
                        except:
                            pass

            self.image_segmented_pub.publish(self.bridge.cv2_to_imgmsg(
                cv2.cvtColor(segmented_img, cv2.COLOR_HSV2BGR), "bgr8"))
            self.image_filtered_pub.publish(self.bridge.cv2_to_imgmsg(
                cv2.cvtColor(filtered_img, cv2.COLOR_HSV2BGR), "bgr8"))
            self.image_outliers_pub.publish(self.bridge.cv2_to_imgmsg(
                cv2.cvtColor(postoutliers_img, cv2.COLOR_HSV2BGR), "bgr8"))

        else:
            target_center = self.identify_center(segmented_img)

        self.center_history.append(
            (target_center[1], target_center[0], depth_img[target_center[0], target_center[1]]))
        if len(self.center_history) >= 15:
            ch = np.array(self.center_history)
            means = np.mean(ch, axis=0)
            vars = np.square(ch[:, 0] - means[0]) + np.square(
                ch[:, 1] - means[1]) + np.square(ch[:, 2] - means[2])
            vars_avg = np.mean(vars)
            self.center_history = ch[vars / vars_avg <= 0.5].tolist()
        if len(self.center_history) > 15:
            self.center_history.pop(0)
        elif len(self.center_history) < 15:
            return

        ch = np.array(self.center_history)
        means = np.mean(ch, axis=0)
        point = Point()

        # convert pixels to meters
        meters_over_pixels = np.tan(self.fov/2)*means[2] / W

        # convert x and y to meters
        x_m = meters_over_pixels * (means[0] - mid_x)
        y_m = meters_over_pixels * (means[1] - mid_y)


        # x value is the column, y value is the row. these are relative to the center of the frame, so (0,0) means the target center at frame center
        point.x = x_m + self.s
        point.y = - y_m + self.y
        point.z = means[2] # maybe it should just be self.D?

        print("stablizied center (last two should equal):", x_m, y_m, means[2], self.D)
        self.center_pub.publish(point)


    def run(self):
        while not rospy.is_shutdown():
            if self.active:
                pass
            self.rate.sleep()


if __name__ == '__main__':

    print("Python Script Running!")
    rospy.init_node('water_gun_node')
    task_node = WaterGunTaskNode()
    task_node.active = True
    task_node.run()
