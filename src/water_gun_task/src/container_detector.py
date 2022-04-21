#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

class ContainerDetection():
	"Class that detects container with ball in it using color segmentation"


	def __init__(self):
		#self.container_pub = # TODO  rospy.Publisher()
		self.debug_pub = rospy.Publisher("/debug_img", Image, queue_size = 10)
		#self.image_sub = # TODO rospy.Subscriber(,,image_callback)
		self.bridge = CvBridge()

	
	def image_callback(self, image_msg):
		# Apply your imported color segmentation function (cd_color_segmentation) to the image msg here
		# From your bounding box, take the center pixel on the bottom
		# (We know this pixel corresponds to a point on the ground plane)
		# publish this pixel (u, v) to the /relative_container_px topic; the homography transformer will
		# convert it to the boat frame.

		#################################
		# detect the container and publish its
		# pixel location in the image.
		#################################

		image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

		#Change these bounds to get correct color
		lower_bound = (0, 100, 10)
		upper_bound = (50, 255, 255)
		template = [lower_bound, upper_bound]

		#Get bounding box using color segmentation
		bb = cd_color_segmentation(image, template)
		#cone_base = ((bb[0,0]+bb[1,0])/2, bb[1][1])

		# Used for cones, will change for container for boat
		# cone_msg = ConeLocationPixel()
		# cone_msg.u=(bb[0][0]+bb[1][0])/2
		# cone_msg.v = bb[1][1]
		debug_img = cv2.rectangle(image, bb[0], bb[1], (0, 255, 0), 2) #adds box onto image
		debug_msg = self.bridge.cv2_to_imgmsg(debug_img, "bgr8")
		self.debug_pub.publish(debug_msg)
		#self.cone_pub.publish(cone_msg)

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def container_color_segmentation(img, template=None):
	"""
	Implement the cone detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected. BGR.
		template_file_path; List of two tuples: (low values, high values) in where each value is (hue, sat, val)
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
	"""
	########## YOUR CODE STARTS HERE ##########
	x = y = w = h = 0
	# image_print(img) #prints image
	# bounding values: change for different lighting conditions in HSV
	if template is not None:
		lower_bounds = template[0]
		upper_bounds = template[1]
	else:
		lower_bounds = (5, 200, 100)
		upper_bounds = (35, 255, 255)

	#Convert bgr to hsv    
	hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) #converts bgr to hsv
	#Creates binary mask image
	mask = cv2.inRange(hsv_img, lower_bounds, upper_bounds) 
	image_print(mask)
	
	#Erode and clean up image
	element = np.ones((5, 5), np.uint8) #kernel for erosion/dilation
	erosion_dst = cv2.erode(mask, element, iterations=1) #shrinks mask area down
	mask = cv2.dilate(erosion_dst, element, iterations=1) #increases mask area
	

	hsv, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) #finding areas were masks exist
	cnt = None
	max_area = -1
	# finding largest contour in mask
	for i in contours:
		area = i.shape[0]
		if area > max_area:
			cnt = i
			max_area = area
	if cnt is not None:
		x, y, w, h = cv2.boundingRect(cnt) # creates rectangle around largest contour
		cv2.rectangle(img, (x, y), (x+w,y+h), (255, 0, 0), 2) #adds box onto image
	#image_print(img)
	bounding_box = ((x,y),(x+w,y+h))

	########### YOUR CODE ENDS HERE ###########

	# Return bounding box
	return bounding_box


if __name__ == '__main__':
	img = cv2.imread("roboatContainer.png") # insert image path
	container_color_segmentation(img)