#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import tf
import message_filters
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from std_msgs.msg import Float32


class WaterGunController:

    def __init__(self):
        self.image_sub = message_filters.Subscriber(
            "/zed2i/zed_node/rgb/image_rect_color", Image)
        self.info_sub = message_filters.Subscriber(
            "/zed2i/zed_node/rgb/camera_info", Image)
		self.yaw_pub = rospy.Publisher("/watergun/yaw", Float32, queue_size=1)
		self.pitch_pub = rospy.Publisher("/watergun/pitch", Float32, queue_size=1)
		self.img_pub = rospy.Publisher("/watergun/debug", Image, queue_size=1)
        ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.info_sub], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback)
		self.listener = tf.TransformListener()
		self.watergun_tf = self.listener.lookupTransform(
		    '/watergun', '/camera', rospy.Time(0))

    def callback(self, image_msg, info_msg):
		try:
			image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
		except CvBridgeError as e:
            center_2d,frame = self.find_center(image)
		if center_2d is not None: 
			center_3d = self.project_to_3d(center_2d, info_msg)
			watergun_pos = self.watergun_tf.transform.translation
			yaw, pitch = self.calc_angles(center_3d, watergun_pos)
			self.yaw_pub.publish(yaw)
			self.pitch_pub.publish(pitch)
			self.img_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

	def calc_angles(self, center_3d, watergun_pos):
		x = center_3d.x - watergun_pos.x
		y = center_3d.y - watergun_pos.y
		z = center_3d.z - watergun_pos.z
		angle_1 = np.arctan2(y, x)
		angle_2 = np.arctan2(z, np.sqrt(x**2 + y**2))
		return angle_1, angle_2

	def find_center(self, img):
		kernel = np.ones((5, 5), np.uint8)
		img_erosion = cv2.erode(img, kernel, iterations=1)
		img_dilation = cv2.dilate(img_erosion, kernel, iterations=2)
		frame = img_dilation
		# Convert BGR to HSV
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		# Blue color boundaries
		lower_bound = np.array([108, 150, 0])
		upper_bound = np.array([140, 255, 255])

		mask = cv2.inRange(hsv, lower_bound, upper_bound)
		contours, _ = cv2.findContours(
			mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		bounding_boxes = []
		for contour in contours:
			# c = max(contours, key = cv2.contourArea)
			x, y, w, h = cv2.boundingRect(contour)
			if w > 40 and h > 40:
				bounding_boxes.append(((x, y), (x+w, y+h)))
		# cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)
		min_x = np.inf
		max_x = -np.inf
		min_y = np.inf
		max_y = -np.inf
		for bounding_box in bounding_boxes:
			x, y = bounding_box[0]
			w, h = bounding_box[1][0] - x, bounding_box[1][1] - y
			min_x = min(min_x, x)
			max_x = max(max_x, x+w)
			min_y = min(min_y, y)
			max_y = max(max_y, y+h)
			# cv2.rectangle(frame, bounding_box[0], bounding_box[1], (0,0,255), 2)
		cv2.rectangle(frame, (min_x, min_y),
						(max_x, max_y), (0, 0, 255), 5)
		return ((min_x+max_x)/2, (min_y+max_y)/2), frame


if __name__ == '__main__':
    rospy.init_node('water_gun_controller')
    water_gun_controller = WaterGunController()
    rospy.spin()
