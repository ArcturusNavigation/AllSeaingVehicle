#!/usr/bin/env python3
import numpy as np
import cv2
import os
import cv_bridge  
import rospy 

from std_msgs.msg import String
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image 
from geometry_msgs.msg import Twist

from task_node import TaskNode

class WaterGunTaskNode(TaskNode):
    
    def __init__(self, detection_method= 'combined',bag_bounds=(250,300),flip_point= (360, 640)):

        super().__init__('water_gun_task')
        self.bridge = cv_bridge.CvBridge()
        self.target_pub = rospy.Publisher('/pilot_suite/water_gun_task/debug/target_img', Image, queue_size=1)
        self.img_pub = rospy.Publisher('/pilot_suite/water_gun_task/debug/rgb_img', Image, queue_size=1)
        # self.velocity_pub = rospy.Publisher('pilot_suite/velocity_command',VelocityCommand, queue_size=1 )
        self.detection_method = detection_method
        self.bag_bounds = bag_bounds
        self.flip_point = flip_point
        self.flip_index = -1
        # Subscribe to approximatley synchornized depth and rgb images
        depth_sub = Subscriber('/zed2i/zed_node/depth/depth_registered', Image)
        rgb_sub = Subscriber('/zed2i/zed_node/rgb/image_rect_color', Image)
        self.ats = ApproximateTimeSynchronizer([depth_sub, rgb_sub], queue_size=1, slop=.1)
        self.ats.registerCallback(self.callback)
        self.bag_positions = []
 
        self.SHRINK_FACTOR = 4
        self.BLUE_DEVIATION_THRESHOLD = 100
        self.DEPTH_THRESHOLD = 10


    def depth_mask(self, original_img, depth_img):

        res_img = np.copy(original_img)

        res_img[depth_img > self.DEPTH_THRESHOLD] = np.zeros(3)

        return res_img


    def identify_center(self, input_img):

        input_img = np.array(input_img)

        min_diff = np.min(
            np.abs(input_img[:, :, 0] - 120 * np.ones(input_img.shape[:2])))

        blue_color = 120 + (min_diff if (120 + min_diff)
                            in input_img[:, :, 0] else -min_diff)

        if abs(blue_color - 120) > self.BLUE_DEVIATION_THRESHOLD:
            return (-1, -1)

        blues = np.argwhere(np.abs(input_img - blue_color) < 1)

        x_blues, y_blues = blues[:, 0], blues[:, 1]

        def remove_outliers(x_blues, y_blues):

            x_mean = np.mean(x_blues)
            y_mean = np.mean(y_blues)

            vars = np.square(x_blues - x_mean) + np.square(y_blues - y_mean)
            mean_var = np.mean(vars)

            return x_blues[vars / mean_var <= 2], y_blues[vars / mean_var <= 2]

        x_blues, y_blues = remove_outliers(x_blues, y_blues)

        return (int(self.SHRINK_FACTOR * np.median(x_blues)),
                int(self.SHRINK_FACTOR * np.median(y_blues)))


    def segment_image(self, input_img):

        scaled_dim = (input_img.shape[1] // self.SHRINK_FACTOR,
                    input_img.shape[0] // self.SHRINK_FACTOR)

        img = cv2.resize(input_img, scaled_dim, interpolation=cv2.INTER_AREA)

        twoDimage = np.float32(img.reshape((-1, 3)))

        criteria = (cv2.TERM_CRITERIA_EPS +
                    cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)

        K = 5
        attempts = 2

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
            img = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except cv_bridge.CvBridgeError as e:
            rospy.loginfo(e)

        print("Callback is running!")

        print(depth_img)
        cv2.imwrite("~/test.jpg", self.depth_mask(img, depth_img))

        print("center at", self.identify_center(self.segment_image(self.depth_mask(img, depth_img))))

    def run(self):
        while not rospy.is_shutdown():
            if self.active:
                if len(self.bag_positions) > 0:
                    if self.flip_index >= len(self.bag_positions):
                        continue 
                    bag_position = self.bag_positions[self.flip_index]
                    dist = np.sqrt((bag_position[0] - self.flip_point[0])**2 + (bag_position[1] - self.flip_point[1])**2)
                    if dist < 50:
                        self.stop()
                        self.flip_bag()
                        self.flip_index += 1 
                    else:
                        angle = np.arctan2(bag_position[1] - self.flip_point[1], bag_position[0] - self.flip_point[0])
                        self.update_velocity(dist, angle)
            self.rate.sleep()
  #    vidcap = cv2.VideoCapture('competition_video.mp4')

#    success, image = vidcap.read()
#    scale = 2
#   NEW_IMG_SIZE = (image.shape[1] // scale, image.shape[0] // scale)
#   count = 1
#   for i in range(1, count):
#       cv.imwrite(f"centered/frame{i}.jpg", identify_center(cv2.imread(f"processed/frame{i}.jpg")))
#   os.system("ffmpeg -framerate 30 -i processed/frame%d.jpg -c:v libx264 -r 30 new_output_3.mp4")

if __name__ == '__main__':

    print("Python Script Running!")
    rospy.init_node('bag_detector')
    task_node = WaterGunTaskNode()
    task_node.active = True 
    task_node.run()
