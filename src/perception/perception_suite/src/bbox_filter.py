"""
References:
https://pdfs.semanticscholar.org/12d2/267145496c1c8608e973a0e4a3a148404185.pdf

This script makes the bounding boxes of the objects continuous by remembering previous bounding boxes and combining them with current bounding boxes, checking whether they are the same object by comparing IoU. Also filters bboxes based on y value.
"""

import rospy
import cv_bridge 
import cv2
import numpy as np
from perception_suite.msg import LabeledBoundingBox2D, LabeledBoundingBox2DArray
from sensor_msgs.msg import Image
from utility.geometry import Vec2D
from utility.constants import IMG_WIDTH, IMG_HEIGHT
from perception_suite.utils import *

class BBoxFilter():
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()

        self.bbox_multiplier = 1.5
        self.filter_y_threshold = 0
        self.forget_threshold = rospy.Duration.from_sec(0.5)
        self.remembered_bboxes = []

        self.bbox_sub = rospy.Subscriber(
            '/perception_suite/bounding_boxes',
            LabeledBoundingBox2DArray,
            self.bbox_callback,
            queue_size=1
        )
        self.img_sub = rospy.Subscriber(
            '/zed2i/zed_node/rgb/image_rect_color', 
            Image,
            self.img_callback, 
            queue_size=1, 
            buff_size=2**24,
        )
        self.bbox_pub = rospy.Publisher('/perception_suite/filtered_boxes', LabeledBoundingBox2DArray, queue_size=1)
        self.img_pub = rospy.Publisher('/perception_suite/filtered_image', Image, queue_size=1)

    def filter_remembered_bboxes(self):
        self.remembered_bboxes = list(filter(lambda x: rospy.Time.now() - x[1] < self.forget_threshold, self.remembered_bboxes))
#        self.remembered_bboxes = list(filter(lambda x: x[0].min_y > self.filter_y_threshold, self.remembered_bboxes))
#        self.remembered_bboxes = list(filter(lambda x: x[0].min_x > 0, self.remembered_bboxes))
#        self.remembered_bboxes = list(filter(lambda x: x[0].min_y > 0, self.remembered_bboxes))
#        self.remembered_bboxes = list(filter(lambda x: x[0].max_x < IMG_WIDTH, self.remembered_bboxes))
#        self.remembered_bboxes = list(filter(lambda x: x[0].max_y < IMG_HEIGHT, self.remembered_bboxes))

    def bbox_matching(self, curr_bboxes):
        
        # Calculate distance between two objects based on IoU
        distance_matrix = np.empty((len(curr_bboxes), len(self.remembered_bboxes)))
        for i, curr_bbox in enumerate(curr_bboxes):
            for j, (prev_bbox, _) in enumerate(self.remembered_bboxes):
                iou = calculate_iou(curr_bbox, prev_bbox)
                if iou == 0:
                    distance = float("inf")
                else:
                    distance = 1 / iou
                distance_matrix[i, j] = distance

         # Calculate pairs that are closest to each other
        pairs = {}
        while distance_matrix.size != 0 and np.min(distance_matrix) != float("inf"):
            min_row, min_col = divmod(distance_matrix.argmin(), distance_matrix.shape[1]) 
            pairs[min_row] = min_col
            distance_matrix[min_row, :] = float("inf")
            distance_matrix[:, min_col] = float("inf")

        # For current bboxes in pairs, modify remembered bboxes. For others add them to remembered bboxes.
        for i, curr_bbox in enumerate(curr_bboxes):
            if i in pairs:
                self.remembered_bboxes[pairs[i]][0] = curr_bbox
                self.remembered_bboxes[pairs[i]][1] = rospy.Time.now()
            else:
                self.remembered_bboxes.append([curr_bbox, rospy.Time.now()])

    def bbox_callback(self, bboxes):

        bigger_bboxes = [] 
        for bbox in bboxes.boxes:
            diff = Vec2D(
                bbox.max_x - bbox.min_x,
                bbox.max_y - bbox.min_y
            )
            center = Vec2D(
                (bbox.max_x + bbox.min_x) / 2,
                (bbox.max_y + bbox.min_y) / 2
            )
            min_vec = center - (diff / 2) * self.bbox_multiplier
            max_vec = center + (diff / 2) * self.bbox_multiplier

            new_bbox = LabeledBoundingBox2D()
            new_bbox.label = bbox.label
            new_bbox.min_x = int(min_vec.x)
            new_bbox.max_x = int(max_vec.x)
            new_bbox.min_y = int(min_vec.y)
            new_bbox.max_y = int(max_vec.y)

            bigger_bboxes.append(new_bbox)

        self.bbox_matching(bigger_bboxes)
        self.filter_remembered_bboxes() 

        new_bboxes = LabeledBoundingBox2DArray()
        new_bboxes.header.stamp = rospy.Time.now()
        for bbox, _ in self.remembered_bboxes:
            new_bboxes.boxes.append(bbox)
        self.bbox_pub.publish(new_bboxes)

    def img_callback(self, img):
        try:
            img = self.bridge.imgmsg_to_cv2(img, "rgb8")
        except cv_bridge.CvBridgeError as e:
            rospy.loginfo(e)

        for bbox, _ in self.remembered_bboxes:
            cv2.rectangle(
                img,
                (bbox.min_x, bbox.min_y),
                (bbox.max_x, bbox.max_y),
                (0, 255, 0),
                4
            )

        self.img_pub.publish(self.bridge.cv2_to_imgmsg(img, "rgb8"))

if __name__ == "__main__":
    rospy.init_node("bbox_filter")
    detector = BBoxFilter()
    rospy.spin()

