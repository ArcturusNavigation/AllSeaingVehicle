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
        self.target_pub = rospy.Publisher('/pilot_suite/oyster_task/debug/target_img', Image, queue_size=1)
        self.img_pub = rospy.Publisher('/pilot_suite/oyster_task/debug/rgb_img', Image, queue_size=1)
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
 
    def depth_mask(self, original_img, depth_img, threshold):

        print("Depth masking right now")

        res_img = np.copy(original_img)

        for i in range(original_img.shape[0]):
            for j in range(original_img.shape[1]):
                if depth_img[i, j] > threshold:
                    res_img[i, j, :] = 0

        print("Done depth masking")

        return res_img

    def reject_outliers(self, data, m=0.7):
        return data
        return data[abs(data - np.mean(data)) < m * np.std(data)]
    
    def identify_center(self, input_img):

        print("Identifying Center...")

        input_img = cv2.cvtColor(input_img, cv2.COLOR_RGB2HSV)

        min_diff = np.min(np.abs(input_img[:,:,0] - 240 * np.ones(input_img.shape[:-1])))
        blue_color = 240 + (min_diff if (240 + min_diff) in input_img else -min_diff)

        h_range = 30 # should be 100
        
        print("Center Identified")

    # blue_color = 240

        x_blues, y_blues = [], []

        result_img = input_img

        for i in range(input_img.shape[0]):
            for j in range(input_img.shape[1]):
                if (input_img[i][j][0] >= (blue_color - h_range) and input_img[i][j][0] <= (blue_color + h_range)):
                    x_blues.append(i)
                    y_blues.append(j)
                    result_img[i, j, :] = np.array([0, 100, 100])
        
        x_blues, y_blues = np.array(x_blues), np.array(y_blues)
        x_blues, y_blues = x_blues[~np.isnan(x_blues)], y_blues[~np.isnan(y_blues)]

        #print(x_blues)
        #print(y_blues)
        if (len(x_blues) == 0 or len(y_blues) == 0):
            print("no center found")
            center_index = (-1,-1)
        else:
            center_index = (int(np.median(x_blues)), int(np.median(y_blues)))

        # is_blue = (input_img >= (blue_color - h_range) and input_img <= (blue_color + h_range))
        # is_blue = is_blue[:, :, 0]

        # print(np.where(is_blue))

        # center_index = np.int32(np.mean(np.where(is_blue), axis=-1))

        return center_index

        for deltax in range(-3, 3):
            for deltay in range(-3, 3):
                try:
                    red = np.array([120, 100, 100])
                    result_img[center_index[0] + deltax, center_index[1] + deltay, :] = red
                except:
                    print("On Edge")

        return cv2.cvtColor(result_img, cv2.COLOR_HSV2RGB)

        return res_img

    def image_segmentation_algorithm(self, input_img):

        print("Segmenting Image...")
        
        # Creating kernel
        kernel = np.ones((2, 2), np.uint8)

        # Using cv2.erode() method 
        # img = cv2.erode(img, kernel, iterations = 5) 
        # img = cv2.dilate(img, kernel, iterations = 5)

        twoDimage = input_img.reshape((-1,3))
        twoDimage = np.float32(twoDimage)

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        K = 5
        attempts=10

        ret,label,center=cv2.kmeans(twoDimage,K,None,criteria,attempts,cv2.KMEANS_PP_CENTERS)
        center = np.uint8(center)
        res = center[label.flatten()]
        result_image = res.reshape((input_img.shape))

        print("Image Segmented")
        
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

        img = np.array(img)
        depth_img = np.array(depth_img)

        NEW_IMG_SIZE = (img.shape[0] // 4, img.shape[1] // 4)
        depth_img = cv2.resize(depth_img, dsize=NEW_IMG_SIZE)
        img = cv2.resize(img, dsize=NEW_IMG_SIZE)

        print("center at", self.identify_center(self.image_segmentation_algorithm(self.depth_mask(img, depth_img, 10))))

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