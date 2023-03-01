import numpy as np
import cv2 as cv
import os
import cv_bridge  
import rospy 

from std_msgs.msg import String
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image 
from geometry_msgs.msg import Twist
from pilot_suite.msg import VelocityCommand

from pilot_suite.task_node import TaskNode

class WaterGunTaskNode(TaskNode):
    def __init__(self, detection_method= 'combined',bag_bounds=(250,300),flip_point= (360, 640)):
        super().__init__('water_gun_task')
        self.bridge = cv_bridge.CvBridge()
        self.target_pub = rospy.Publisher('/pilot_suite/oyster_task/debug/target_img', Image, queue_size=1)
        self.img_pub = rospy.Publisher('/pilot_suite/oyster_task/debug/rgb_img', Image, queue_size=1)
        self.velocity_pub = rospy.Publisher('pilot_suite/velocity_command',VelocityCommand, queue_size=1 )
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
        self.prev_bag_positions = []

    def callback(self, depth_img, img):
        if not self.active:
            return
        # Convert depth and rgb image using cvBridge
        try: 
            depth_img = self.bridge.imgmsg_to_cv2(depth_img, "32FC1")
            img = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except cv_bridge.CvBridgeError as e:
            rospy.loginfo(e)
        if self.detection_method == 'depth':
            raise NotImplementedError 
        elif self.detection_method == 'color':
            target_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
            target_img = cv.inRange(target_img, (0, 0 ,0), (255, 255, 40))
            cv.dilate(target_img, np.ones((5,5), np.uint8), target_img, iterations=4)
        elif self.detection_method == 'combined':
            target_img = depth_img
            target_img[np.isnan(target_img)] = 0
            cv.dilate(target_img, np.ones((5,5), np.uint8), target_img, iterations=1)
            target_img = cv.inRange(target_img, .7, 1.25)
            color_mask = cv.inRange(cv.cvtColor(img, cv.COLOR_BGR2HSV), (0, 0 ,0), (255, 255, 40))
            cv.dilate(color_mask, np.ones((5,5), np.uint8), color_mask, iterations=3)
            target_img = cv.bitwise_and(target_img,color_mask)
        self.target_pub.publish(self.bridge.cv2_to_imgmsg(target_img, "mono8")) 
        # Find contours
        contours, _ = cv.findContours(target_img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        rects = [cv.minAreaRect(contour) for contour in contours]
        floats = []
        bag_positions = []
        for rect in rects: 
            matched = False 
            (x,y), (w,h), angle = rect
            box = np.int0(cv.boxPoints(rect))
            if w < 100 and h < 100:
                continue
            if w < 10 or h< 10:
                continue
            for i in range(len(floats)):
                (x1,y1), (w1,h1), angle1 = floats[i]
                angle_diff = min(np.abs(angle-angle1), np.abs(min(angle, angle1) - 90 - max(angle, angle1)))
                dist = np.sqrt((x-x1)**2 + (y-y1)**2)
                if angle_diff < 10 and (dist > self.bag_bounds[0] and dist < self.bag_bounds[1]):
                    cv.drawContours(img, [box], 0, (0,0,255), 2)
                    box1 = np.int0(cv.boxPoints(floats[i]))
                    cv.drawContours(img, [box1], 0, (0,0,255), 2)
                    bag_position = (int((x+x1)/2), int((y+y1)/2))
                    bag_positions.append(bag_position)
                    cv.circle(img, bag_position, 10, (0,0,255), -1)
                    floats.remove(floats[i])
                    matched = True
                    break 
            if not matched:
                floats.append(rect)
        if len(bag_positions) > 0:
            self.flip_index += len(bag_positions) - len(self.bag_positions)
            bag_positions.sort(key= lambda x: x[1])
            self.prev_bag_positions = self.bag_positions
            self.bag_positions = bag_positions
            if self.flip_index >= len(self.bag_positions):
                self.flip_index = -1
            cv.line(img, self.flip_point, bag_positions[self.flip_index], (0,255,0), 2)
            cv.circle(img, self.flip_point, 10, (0,255,0), -1)
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))

    # TODO: Add PID controller
    def update_velocity(self, dist, angle):
        vel_command = VelocityCommand()
        vel_command.twist = Twist()
        vel_command.twist.linear.x = 0.25
        vel_command.twist.angular.z = angle * 2
    
    def stop(self):
        vel_command = VelocityCommand()
        vel_command.twist = Twist()
        self.velocity_pub.publish(vel_command)

    def flip_bag(self):
        pass 

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
    def image_segmentation_algorithm(input_img):
        
        # Creating kernel
        kernel = np.ones((2, 2), np.uint8)

        # Using cv2.erode() method 
        img = cv2.resize(input_img, dsize=NEW_IMG_SIZE)
        # img = cv2.erode(img, kernel, iterations = 5) 
        # img = cv2.dilate(img, kernel, iterations = 5)

        twoDimage = img.reshape((-1,3))
        twoDimage = np.float32(twoDimage)

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        K = 5
        attempts=10

        ret,label,center=cv2.kmeans(twoDimage,K,None,criteria,attempts,cv2.KMEANS_PP_CENTERS)
        center = np.uint8(center)
        res = center[label.flatten()]
        result_image = res.reshape((img.shape))
        
        return result_image
            
    def reject_outliers(data, m=0.7):
        return data
        return data[abs(data - np.mean(data)) < m * np.std(data)]
    
    def identify_center(input_img):

        input_img = cv2.cvtColor(input_img, cv2.COLOR_RGB2HSV)

        min_diff = np.min(np.abs(input_img[:,:,0] - 240 * np.ones(input_img.shape[:-1])))
        blue_color = 240 + (min_diff if (240 + min_diff) in input_img else -min_diff)

        h_range = 100

    # blue_color = 240

        x_blues, y_blues = [], []

        result_img = input_img

        for i in range(input_img.shape[0]):
            for j in range(input_img.shape[1]):
                print(i, j, input_img[i][j][0], blue_color)
                if  not (input_img[i][j][0] >= (blue_color - h_range) and input_img[i][j][0] <= (blue_color + h_range)):
                    x_blues.append(i)
                    y_blues.append(j)
                    result_img[i, j, :] = np.array([0, 100, 100])
        
        center_index = (int(np.median(reject_outliers(np.array(x_blues)))), int(np.median(reject_outliers(np.array(y_blues)))))

        # is_blue = (input_img >= (blue_color - h_range) and input_img <= (blue_color + h_range))
        # is_blue = is_blue[:, :, 0]

        # print(np.where(is_blue))

        # center_index = np.int32(np.mean(np.where(is_blue), axis=-1))

        for deltax in range(-3, 3):
            for deltay in range(-3, 3):
                try:
                    red = np.array([120, 100, 100])
                    result_img[center_index[0] + deltax, center_index[1] + deltay, :] = red
                except:
                    print("On Edge")

        return cv2.cvtColor(result_img, cv2.COLOR_HSV2RGB)

    vidcap = cv2.VideoCapture('competition_video.mp4')

    success, image = vidcap.read()
    scale = 2
    NEW_IMG_SIZE = (image.shape[1] // scale, image.shape[0] // scale)
    count = 1
    for i in range(1, count):
        cv.imwrite(f"centered/frame{i}.jpg", identify_center(cv2.imread(f"processed/frame{i}.jpg")))
    os.system("ffmpeg -framerate 30 -i processed/frame%d.jpg -c:v libx264 -r 30 new_output_3.mp4")

if __name__ == '__main__':
    rospy.init_node('bag_detector')
    task_node = WaterGunTaskNode()
    task_node.active = True 
    task_node.run()
