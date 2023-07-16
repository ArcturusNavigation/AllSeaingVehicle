"""
References:
https://pytorch.org/hub/ultralytics_yolov5/
https://pypi.org/project/yolo5/
"""

import rospy
import cv_bridge 
import cv2
from perception_suite.msg import LabeledBoundingBox2D, LabeledBoundingBox2DArray
from sensor_msgs.msg import Image
from utility.geometry import Vec2D
from utility.constants import BUOY_CLASSES

class BuoyDetector():
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()

        self.bbox_mins = {}
        self.bbox_maxes = {}

        self.bbox_sub = rospy.Subscriber(
            '/perception_suite/bounding_boxes',
            LabeledBoundingBox2DArray,
            self.bbox_callback,
            queue_size=1
        )
        self.img_sub = rospy.Subscriber(
            '/perception_suite/segmented_image', 
            Image,
            self.img_callback, 
            queue_size=1, 
            buff_size=2**24,
        )
        self.bbox_pub = rospy.Publisher('/perception_suite/buoy_boxes', LabeledBoundingBox2DArray, queue_size=1)
        self.img_pub = rospy.Publisher('/perception_suite/buoy_image', Image, queue_size=1)

    def bbox_callback(self, bboxes):
        
        self.bbox_mins = {} 
        self.bbox_maxes = {}
        for bbox in bboxes.boxes:
            if bbox.label in BUOY_CLASSES.values():
                if bbox.label not in self.bbox_mins:
                    self.bbox_mins[bbox.label] = Vec2D(bbox.min_x, bbox.min_y)
                    self.bbox_maxes[bbox.label] = Vec2D(bbox.max_x, bbox.max_y)
                    continue
                
                self.bbox_mins[bbox.label].x = min(bbox.min_x, self.bbox_mins[bbox.label].x)
                self.bbox_mins[bbox.label].y = min(bbox.min_y, self.bbox_mins[bbox.label].y)
                self.bbox_maxes[bbox.label].x = max(bbox.max_x, self.bbox_maxes[bbox.label].x)
                self.bbox_maxes[bbox.label].y = max(bbox.max_y, self.bbox_maxes[bbox.label].y)
                
        print("bbox_mins:", self.bbox_mins)
        print("bbox_maxes:", self.bbox_maxes)

        new_bboxes = LabeledBoundingBox2DArray()
        new_bboxes.header.stamp = rospy.Time.now()
        bboxes.header.frame_id = "zed2i_camera_center"
        for label in self.bbox_mins:
            labeled_bbox = LabeledBoundingBox2D()
            labeled_bbox.label = label
            labeled_bbox.min_x = self.bbox_mins[label].x
            labeled_bbox.min_y = self.bbox_mins[label].y
            labeled_bbox.max_x = self.bbox_maxes[label].x
            labeled_bbox.max_y = self.bbox_maxes[label].y
            new_bboxes.boxes.append(labeled_bbox)

        self.bbox_pub.publish(new_bboxes)

    def img_callback(self, img):
        try:
            img = self.bridge.imgmsg_to_cv2(img, "rgb8")
        except cv_bridge.CvBridgeError as e:
            rospy.loginfo(e)

        for label in self.bbox_mins:
            cv2.rectangle(
                img, 
                (self.bbox_mins[label].x,  self.bbox_mins[label].y), 
                (self.bbox_maxes[label].x, self.bbox_maxes[label].y), 
                (0, 0, 255), 
                4
            )
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(img, "rgb8"))

if __name__ == "__main__":
    rospy.init_node("buoy_detector")
    detector = BuoyDetector()
    rospy.spin()

