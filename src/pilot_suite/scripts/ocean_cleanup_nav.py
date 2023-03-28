#!/usr/bin/env python3
import numpy as np
import cv2
import os
import cv_bridge
import rospy

from std_msgs.msg import String
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Point, Vector3
from visualization_msgs.msg import Marker

from task_node import TaskNode

# cant just set waypoint and need to get velocities and continue updating

class OceanCleanupTaskNode(TaskNode):

    def __init__(self, detection_method = 'combined', bag_bounds = (250, 300), flip_point = (260, 640)):
        
        #########################################
        ######## Execution Debug Mode ###########
        #########################################

        self.debug = True
        super().__init__('ocean_cleanup_task')
        self.bridge = cv_bridge.CvBridge()

        ########################################
        ############# PUBLISHERS ###############
        ########################################

        self.center_pub = rospy.Publisher(
            '/pilot_suite/ocean_cleanup_task/', Point, queue_size=1)
        self.velocity_pub = rospy.Publisher(
            '/pilot_suite/ocean_cleanup_task/vel_pub', Twist, queue_size=1)
        
        if(self.debug):
            self.image_segmented_pub = rospy.Publisher(
                '/pilot_suite/ocean_cleanup_task/debug/segmented_img', Image, queue_size=1)
            self.image_filtered_pub = rospy.Publisher(
                '/pilot_suite/ocean_cleanup_task/debug/filtered_img', Image, queue_size=1)
            self.image_outliers_pub = rospy.Publisher(
                '/pilot_suite/ocean_cleanup_task/debug/outliers_img', Image, queue_size=1)
            
        # do we need this
        self.detection_method = detection_method
        self_bag_bounds = bag_bounds
        self.flip_point = flip_point
        self.flip_index = -1

        # Subscribe to approximately synchronized depth and rgb images
        depth_sub = Subscriber('/zed2i/zed_node/depth/depth_registered', Image)
        rgb_sub = Subscriber('/zed2i/zed_node/rgb/image_rect_color', Image)

        self.ats = ApproximateTimeSynchronizer(
            [depth_sub, rgb_sub], queue_size = 1, slop = .1
        )

        self.ats.registerCallback(self.callback)
        self.bag_positions = []

        self.SHRINK_FACTOR = 4
        self.ORANGE_DEVIATION_THRESHOLD = 15
        self.DEPTH_THRESHOLD = 10
        self.ORANGE_CONSTANT = 15
        self.SV_THRESHOLD = 100
        self.VAR_THRESHOLD = 0.7

        self.VELOCITY_SCALE = 0.5

        # THIS MIGHT CHANGE BECAUSE POOL NOODLE
        self.COUNT_THRESHOLD = 2

        self.MIN_DEPTH = 1
        self.MAX_DEPTH = 15.0

        # Everything is in meters
        self.pixel_width = 0

        # NEED CHECK IF THIS IS NEEDED
        self.D = 0.5
        self.fov = 2.0944 # this is 120 degrees as the zed 2i advertises
        self.center_history = []

    def depth_and_sv_mask(self, original_img, depth_img):
        res_img = np.copy(original_img)

        res_img[depth_img > self.DEPTH_THRESHOLD] = np.zeros(3)
        if self.ORANGE_CONSTANT != 0:
            res_img[res_img[:, :, 0] < self.ORANGE_CONSTANT - self.ORANGE_DEVIATION_THRESHOLD] = np.zeros(3)
            res_img[res_img[:, :, 0] > self.ORANGE_CONSTANT + self.ORANGE_DEVIATION_THRESHOLD] = np.zeros(3)
        else:
            res_img[np.abs(90 - res_img[:, :, 0]) < (90 - self.ORANGE_DEVIATION_THRESHOLD)] = np.zeros(3)
            

        res_img[res_img[:, :, 1] < self.SV_THRESHOLD] = np.zeros(3)
        res_img[res_img[:, :, 2] < self.SV_THRESHOLD] = np.zeros(3)

        return res_img

    def identify_left(self, input_img):
        
        not_detected = (None, None)

        if self.debug:
            not_detected = ((None, None), None, None)

        input_img = np.array(input_img)
        old_detection = False

        if old_detection:
            # CHECK TO SEE IF THIS IS CORRECT
            min_diff = np.min(
                np.abs(input_img[:, :, 0] - self.ORANGE_CONSTANT * np.ones(input_img.shape[:2]))
            )

            orange_color = self.ORANGE_CONSTANT + (min_diff if (self.ORANGE_CONSTANT + min_diff) 
                                                in input_img[:, :, 0] else -min_diff)
            print(orange_color)
            print(min_diff)
            if abs(orange_color - self.ORANGE_CONSTANT) > 30:
                return not_detected

            oranges = np.argwhere(np.abs(input_img - orange_color) < 1)

        else:
            oranges = np.argwhere(np.abs(input_img - self.ORANGE_CONSTANT) < self.ORANGE_DEVIATION_THRESHOLD)

        print("we have ", len(oranges), " oranges")
        # if len(oranges) <= self.COUNT_THRESHOLD or len(oranges) >= self.COUNT_THRESHOLD * 100:
        #     return not_detected
        if len(oranges) >= self.COUNT_THRESHOLD * 10000:
            return not_detected
        x_oranges, y_oranges = oranges[:, 0], oranges[:, 1]
        if self.debug:
            filtered_img = input_img.copy()

            for i in range(input_img.shape[0]):
                for j in range(input_img.shape[1]):
                    if (abs(input_img[i][j][0] - self.ORANGE_DEVIATION_THRESHOLD) < self.ORANGE_DEVIATION_THRESHOLD): # orange_color and 1 if using segmentation
                        continue
                    else:
                        filtered_img[i, j, :] = [0, 0, 0]
        
        def remove_outliers(x_oranges, y_oranges):

            x_mean = np.mean(x_oranges)
            y_mean = np.mean(y_oranges)

            variances = np.square(x_oranges - x_mean) + np.square(y_oranges - y_mean)

            mean_var = np.mean(variances)
            if mean_var == 0:
                return x_oranges, y_oranges

            return x_oranges[variances / mean_var <= self.VAR_THRESHOLD], y_oranges[variances / mean_var <- self.VAR_THRESHOLD]

        # x_oranges, y_oranges = remove_outliers(x_oranges, y_oranges) don't need to remove outliers
        if len(x_oranges) <= self.COUNT_THRESHOLD or len(y_oranges) <= self.COUNT_THRESHOLD:
            return not_detected

        # Convert location to location relative to center of the frame
        self.SHRINK_FACTOR = 1 # set to 1 because we dont run segmentation
        if self.debug:
            postoutliers_img = np.zeros(filtered_img.shape)
            for i in range(len(x_oranges)):
                postoutliers_img[x_oranges[i], y_oranges[i], :] = np.array([120, 100, 100])

            # Trying to find the leftmost 
            return (int(self.SHRINK_FACTOR * np.amin(x_oranges)),
                    int(self.SHRINK_FACTOR * np.amin(y_oranges))), filtered_img, postoutliers_img

        else:

            return (int(self.SHRINK_FACTOR * np.amin(x_oranges)),
                    int(self.SHRINK_FACTOR * np.amin(y_oranges)), )

    def segment_image(self, input_img):
        scaled_dim = (input_img.shape[1] // self.SHRINK_FACTOR,
                    input_img.shape[0] // self.SHRINK_FACTOR)

        img = cv2.resize(input_img, scaled_dim, interpolation=cv2.INTER_AREA)

        twoDimage = np.float32(img.reshape((-1, 3)))

        criteria = (cv2.TERM_CRITERIA_EPS +
                    cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)

        K = 2
        attempts = 1

        _, label, center = cv2.kmeans(
            twoDimage, K, None, criteria, attempts, cv2.KMEANS_PP_CENTERS)
        center = np.uint8(center)
        res = center[label.flatten()]

        result_image = res.reshape((img.shape))

        return result_image

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

        # THIS WILL HAVE TO CHANGE DEPENDING ON FUNCTION
        if False:
            segmented_img = self.segment_image(
                self.depth_and_sv_mask(img, depth_img)
            )
        else:
            segmented_img =  self.depth_and_sv_mask(img, depth_img)
        if self.debug:

            target_left, filtered_img, postoutliers_img = self.identify_left(segmented_img)

            if filtered_img is None:
                filtered_img = np.zeros((H, W, 3), dtype = np.uint8)
                print("No filtered image exists!")
            if postoutliers_img is None:
                postoutliers_img = np.zeros((H, W, 3), dtype = np.uint8)
                print("No post-outliers image exists!")

            if target_left != (None, None):

                for i in range(-20, 20):
                    for j in range(-20, 20):
                        try:
                            segmented_img[(i+target_left[0])//self.SHRINK_FACTOR,
                            (j+target_left[1])//self.SHRINK_FACTOR] = np.array([0, 100, 75])
                        except:
                            pass

            self.image_segmented_pub.publish(self.bridge.cv2_to_imgmsg(
                cv2.cvtColor(segmented_img, cv2.COLOR_HSV2BGR), "bgr8"
            ))
            self.image_filtered_pub.publish(self.bridge.cv2_to_imgmsg(
                cv2.cvtColor(filtered_img, cv2.COLOR_HSV2BGR), "bgr8"
            ))
            #self.image_outliers_pub.publish(self.bridge.cv2_to_imgmsg(cv2.cvtColor(postoutliers_img, cv2.COLOR_HSV2BGR), "bgr8"))

        else:
            target_left = self.identify_left(segmented_img)

        if target_left == (None, None):
            return

        self.center_history.append(
            (target_left[1], target_left[0], depth_img[target_left[0], target_left[1]])
        )
        using_history = False
        if using_history:
            if len(self.center_history) >= 15:
                ch = np.array(self.center_history)
                means = np.mean(ch, axis = 0)
                variances = np.square(ch[:, 0] - means[0]) + np.square(
                ch[:, 1] - means[1])
                variances_avg = np.mean(variances)
                self.center_history = ch[variances / variances_avg <= 15].tolist()
            if len(self.center_history) > 15:
                self.center_history.pop(0)
            elif len(self.center_history) < 15:
                return

        ch = np.array(self.center_history)
        if not using_history:
            self.center_history.pop()
        print("the shape is", ch.shape)
        means = np.mean(ch, axis=0)
        point = Point()
        print("the depth is: ", means[2])
        # Convert pixels to meters
        meters_over_pixels = np.tan(self.fov/2)*means[2] / W

        # Convert x and y to meters
        x_m = meters_over_pixels * (means[0] - mid_x)
        y_m = meters_over_pixels * (means[1] - mid_y)

        # x value is in the column, y value is the row. These are relative to the center of the frame, so (0,0) means the target center at frame center
        # THIS IS FOR WATERGUN SO I DONT NEED RIGHT?
        center_x = x_m
        center_y = y_m 
        center_z = means[2] # maybe it should just be self.D?

        print("stablizied center (last two should equal):", x_m, y_m, means[2])
        linear_velocity, angular_velocity = Vector3(), Vector3()

        linear_velocity.x = self.VELOCITY_SCALE * np.max(0, center_z - 1.0)
        linear_velocity.y = self.VELOCITY_SCALE * center_x
        linear_velocity.z = 0

        angular_velocity.x = 0
        angular_velocity.y = 0
        angular_velocity.z = 0

        velocity_msg = Twist()

        velocity_msg.linear = linear_velocity
        velocity_msg.angular = angular_velocity
        print("linearly velocity", linear_velocity.x, linear_velocity.y)
        self.velocity_pub.publish(velocity_msg)

    def run(self):
        while not rospy.is_shutdown():
            if self.active:
                pass
            self.rate.sleep()
    
if __name__ == '__main__':

    print("Python Script Running!")
    rospy.init_node('ocean_cleanup_node')
    task_node = OceanCleanupTaskNode()
    task_node.active = True
    task_node.run()