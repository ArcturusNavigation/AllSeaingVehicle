#!/usr/bin/env python3
"""
References:
https://pytorch.org/hub/ultralytics_yolov5/
https://pypi.org/project/yolo5/
"""

#TODO: Update requirements.txt so that after installation you can run this segmentaiton node
import torch
import cv_bridge 

import rospy
from sensor_msgs.msg import Image, CompressedImage
from perception_suite.msg import LabeledBoundingBox2D, LabeledBoundingBox2DArray
ZED_TOPIC = '/zed2i/zed_node/rgb/image_rect_color'
class Yolov5Detector():
    def __init__(self, img_topic=ZED_TOPIC):
        self.img_sub = rospy.Subscriber('/zed2i/zed_node/rgb/image_rect_color', Image, self.img_callback, queue_size=1, buff_size=2**24)
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True) # TODO: Instead of pretrained weights, load our weights. 
        self.pub = rospy.Publisher('/perception_suite/bounding_boxes', LabeledBoundingBox2DArray)
    
    def img_callback(self, img):
        try:
            img = self.bridge.imgmsg_to_cv2(img, "bgr8") # TODO: Match to model training(bgr8 or rgb8)
        except cv_bridge.CvBridgeError as e: 
            rospy.loginfo(e)
        results = self.model(img) # May need to resize image
        preds = results.pandas().xyxy[0]  # img1 predictions (pandas)
        bboxes = LabeledBoundingBox2DArray()
        bboxes.header = img.header
        for _, row in preds.iterrows():
            bbox = LabeledBoundingBox2D()
            bbox.min_x = row['xmin']
            bbox.min_y = row['ymin']
            bbox.max_x = row['max_x']
            bbox.max_y = row['max_y']
            bbox.label = row['class']
            bbox.probability = row['confidence']
            bboxes.boxes.append(bbox)
        self.pub.publish(bboxes)

if __name__ == "__main__":
    rospy.init_node("yolov5", anonymous=True)
    detector = Yolov5Detector()
    rospy.spin()