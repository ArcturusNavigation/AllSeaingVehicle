#!/usr/bin/env python3
import numpy as np
import cv2
import os
import cv_bridge
import statistics
import rospy

from std_msgs.msg import Int8
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Point, Vector3

from task_node import TaskNode


class TurtleNestTaskNode(TaskNode):

    def __init__(self, color):

        #########################################
        ######## Execution Debug Mode ###########
        #########################################

        self.debug = True
        super().__init__('turtle_nest_task')
        self.bridge = cv_bridge.CvBridge()

        ########################################
        ############# PUBLISHERS ###############
        ########################################

        self.num_circles = rospy.Publisher(
            '/pilot_suite/turtle_nest_task/num_circles_pub', Int8, queue_size=1)
        
        self.velocity_pub = rospy.Publisher(
            '/pilot_suite/turtle_nest_task/vel_pub', Twist, queue_size=1)


        if(self.debug):
            self.image_segmented_pub = rospy.Publisher(
                '/pilot_suite/turtle_nest_task/debug/segmented_img', Image, queue_size=1)
            self.image_filtered_pub = rospy.Publisher(
                '/pilot_suite/turtle_nest_task/debug/filtered_img', Image, queue_size=1)
            self.image_outliers_pub = rospy.Publisher(
                '/pilot_suite/turtle_nest_task/debug/outliers_img', Image, queue_size=1)

        # Subscribe to approximatley synchornized depth and rgb images
        depth_sub = Subscriber('/zed2i/zed_node/depth/depth_registered', Image)
        rgb_sub = Subscriber('/zed2i/zed_node/rgb/image_rect_color', Image)

        self.ats = ApproximateTimeSynchronizer(
            [depth_sub, rgb_sub], queue_size=1, slop=.1)

        self.ats.registerCallback(self.callback)
        self.bag_positions = []

        self.SHRINK_FACTOR = 2
        self.TARGET_COLOR_DEVIATION_THRESHOLD = 15
        self.DEPTH_THRESHOLD = 15
        if color == "blue":
            self.TARGET_COLOR_CONSTANT = 120
        elif color == "green":
            self.TARGET_COLOR_CONSTANT = 60
        elif color == "red":
            self.TARGET_COLOR_CONSTANT = 0

        self.SV_THRESHOLD = 50
        self.SV_THRESHOLD = 50
        self.VAR_THRESHOLD = 0.7

        self.MIN_DEPTH = 0.5
        self.MAX_DEPTH = 15.0

        self.VELOCITY_SCALE = 1.5

        #everything in terms of meters
        self.pixel_width = 0
        self.fov = 2.0944 # this is 120 degrees as the zed 2i advertises
        self.num_history = []

    def depth_and_sv_mask(self, original_img, depth_img):

        res_img = np.copy(original_img)

        res_img[depth_img > self.DEPTH_THRESHOLD] = np.zeros(3)

        if self.TARGET_COLOR_CONSTANT != 0:
            res_img[res_img[:, :, 0] < self.TARGET_COLOR_CONSTANT - self.TARGET_COLOR_DEVIATION_THRESHOLD] = np.zeros(3)
            res_img[res_img[:, :, 0] > self.TARGET_COLOR_CONSTANT + self.TARGET_COLOR_DEVIATION_THRESHOLD] = np.zeros(3)
        else:
            res_img[np.abs(90 - res_img[:, :, 0]) > (90 - self.TARGET_COLOR_DEVIATION_THRESHOLD)] = np.zeros(3)
            

        res_img[res_img[:, :, 1] < self.SV_THRESHOLD] = np.zeros(3)
        res_img[res_img[:, :, 2] < self.SV_THRESHOLD] = np.zeros(3)

        return res_img

    def remove_outliers(self, x_target_colors, y_target_colors):

        x_mean = np.mean(x_target_colors)
        y_mean = np.mean(y_target_colors)

        variances = np.square(x_target_colors - x_mean) + np.square(y_target_colors - y_mean)

        mean_var = np.mean(variances)
        if mean_var == 0:
            return x_target_colors, y_target_colors

        return x_target_colors[variances / mean_var <= self.VAR_THRESHOLD], y_target_colors[variances / mean_var <= self.VAR_THRESHOLD]


    def filter_circles(self, input_img):

        not_detected = None

        if self.debug:
            not_detected = (None, None)

        input_img = np.array(input_img)

        min_diff = np.min(
            np.abs(input_img[:, :, 0] - self.TARGET_COLOR_CONSTANT * np.ones(input_img.shape[:2])))

        target_color_color = self.TARGET_COLOR_CONSTANT + (min_diff if (self.TARGET_COLOR_CONSTANT + min_diff)
                                           in input_img[:, :, 0] else -min_diff)
    

        if abs(target_color_color - self.TARGET_COLOR_CONSTANT) > self.TARGET_COLOR_DEVIATION_THRESHOLD:
            return not_detected

        target_colors = np.argwhere(np.abs(input_img - target_color_color) < 1)
        if len(target_colors) == 0:
            return not_detected

        x_target_colors, y_target_colors = target_colors[:, 0], target_colors[:, 1]
        # x_target_colors, y_target_colors = self.remove_outliers(x_target_colors, y_target_colors)
        if len(x_target_colors) == 0 or len(y_target_colors) == 0:
            return not_detected


        filtered_img = input_img.copy()

        for i in range(input_img.shape[0]):
            for j in range(input_img.shape[1]):
                if (abs(input_img[i][j][0] - target_color_color) > self.TARGET_COLOR_DEVIATION_THRESHOLD):
                    filtered_img[i, j, :] = [0, 0, 0]


        # convert location to location relative to center of the frame
        """
        postoutliers_img = filtered_img.copy()
        for i in range(len(postoutliers_img)):
            for j in range(len(postoutliers_img[0])):
                postoutliers_img[i, j, :] = np.zeros(3)
        for i in range(len(x_target_colors)):
            postoutliers_img[x_target_colors[i], y_target_colors[i],
                             :] = np.array([0, 100, 255])
        """

        if self.debug:
            return (filtered_img, filtered_img)
        
        return filtered_img


    def segment_image(self, input_img):

        scaled_dim = (input_img.shape[1] // self.SHRINK_FACTOR,
                      input_img.shape[0] // self.SHRINK_FACTOR)

        img = cv2.resize(input_img, scaled_dim, interpolation=cv2.INTER_AREA)

        twoDimage = np.float32(img.reshape((-1, 3)))

        criteria = (cv2.TERM_CRITERIA_EPS +
                    cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)

        K = 15
        attempts = 1

        _, label, center = cv2.kmeans(
            twoDimage, K, None, criteria, attempts, cv2.KMEANS_PP_CENTERS)
        center = np.uint8(center)
        res = center[label.flatten()]

        result_image = res.reshape((img.shape))

        return result_image

    def count_cicles(self, filtered_img):

        if (filtered_img is None):
            return 0
        
        center = np.array([0, 0])
        
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(filtered_img[:,:,2])

        num_large_cc = 0

        for i in range(1, num_labels):
            if stats[i, cv2.CC_STAT_AREA] >= 15:
                num_large_cc += 1
                center += centroids[i]
        
        if num_large_cc == 0:
            return 0, -1

        return num_large_cc, center // num_large_cc


    def callback(self, depth_img, img):

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

        segmented_img = self.depth_and_sv_mask(img, depth_img)

        if self.TARGET_COLOR_CONSTANT == 120:
            segmented_img = self.segment_image(segmented_img)

        if self.debug:
            filtered_img, postoutliers_img = self.filter_circles(segmented_img)
        else:
            postoutliers_img = self.filter_circles(segmented_img)

        if postoutliers_img is None:
            return

        if self.debug:

            self.image_segmented_pub.publish(self.bridge.cv2_to_imgmsg(
                cv2.cvtColor(segmented_img, cv2.COLOR_HSV2BGR), "bgr8"))
            self.image_filtered_pub.publish(self.bridge.cv2_to_imgmsg(
                cv2.cvtColor(filtered_img, cv2.COLOR_HSV2BGR), "bgr8"))
            self.image_outliers_pub.publish(self.bridge.cv2_to_imgmsg(
                cv2.cvtColor(postoutliers_img, cv2.COLOR_HSV2BGR), "bgr8"))

        num_circles, center = self.count_cicles(postoutliers_img)

        print("Detected", num_circles, " dots")

        if num_circles == 0:
            return

        self.num_history.append(num_circles)
        self.center_history.append(center)

        if len(self.num_history) >= 10:
            ch = np.array(self.num_history)
            mode_dots = statistics.mode(ch)
            variances = np.square(ch - mode_dots)
            variances_avg = np.mean(variances)
            self.num_history = ch[variances / (variances_avg + 1e-6) <= 15].tolist()
        if len(self.num_history) > 10:
            self.num_history.pop(0)
        elif len(self.num_history) < 10:
            return

        ch = np.array(self.num_history)

        num_dots = Int8()
        num_dots.data = mode_dots

        #print("Number of dots detected", num_dots.data)
        self.num_circles.publish(num_dots)

        self.center_history.append(
            (center[1], center[0], depth_img[center[0], center[1]]))
        if len(self.center_history) >= 15:
            ch = np.array(self.center_history)
            means = np.mean(ch, axis=0)
            vars = np.square(ch[:, 0] - means[0]) + np.square(
                ch[:, 1] - means[1]) + np.square(ch[:, 2] - means[2])
            vars_avg = np.mean(vars)
            self.center_history = ch[vars / vars_avg <= 0.5].tolist()
        if len(self.center_history) > 10:
            self.center_history.pop(0)
        elif len(self.center_history) < 15:
            return

        ch = np.array(self.center_history)
        means = np.mean(ch, axis=0)
        point = Point()

        # convert pixels to meters
        meters_over_pixels = np.tan(self.fov/2)*means[2] / W

        # convert x and y to meters
        center_x = meters_over_pixels * (means[0] - mid_x)
        center_y = meters_over_pixels * (means[1] - mid_y)
        center_z = means[2]

        print("Point", center_x, center_z)

        linear_velocity, angular_velocity = Vector3(), Vector3()

        linear_velocity.x = self.VELOCITY_SCALE * center_z
        linear_velocity.y = self.VELOCITY_SCALE * center_x
        linear_velocity.z = 0

        angular_velocity.x = 0
        angular_velocity.y = 0
        angular_velocity.z = 0

        velocity_msg = Twist()

        velocity_msg.linear = linear_velocity
        velocity_msg.angular = angular_velocity

        self.velocity_pub.publish(velocity_msg)



    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == '__main__':

    print("Python Script Running!")
    rospy.init_node('turtle_nest_node')
    task_node = TurtleNestTaskNode("red")
    task_node.active = True
    task_node.run()
