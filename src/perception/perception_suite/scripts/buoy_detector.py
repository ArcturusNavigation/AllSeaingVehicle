"""
References:
https://pytorch.org/hub/ultralytics_yolov5/
https://pypi.org/project/yolo5/
"""

import rospy
from perception_suite.msg import LabeledBoundingBox2D, LabeledBoundingBox2DArray
from utility.geometry import Vec2D

BUOY_CLASSES = {
    "RED": -1,
    "YELLOW": 6,
    "GREEN": -2
}

class BuoyDetector():
    def __init__(self):
        self.sub = rospy.Subscriber(
            '/perception_suite/bounding_boxes',
            LabeledBoundingBox2DArray,
            self.bbox_callback,
            queue_size=1
        )
    
    def bbox_callback(self, bboxes):
        bbox_min_sums = {} 
        bbox_max_sums = {}
        for bbox in bboxes.boxes:
            if bbox.label in BUOY_CLASSES.values():
                if bbox.label in bbox_min_sums:
                    bbox_min_sums[bbox.label] += Vec2D(bbox.min_x, bbox.min_y)
                    bbox_max_sums[bbox.label] += Vec2D(bbox.max_x, bbox.max_y)
                else:
                    bbox_min_sums[bbox.label] = Vec2D(bbox.min_x, bbox.min_y)
                    bbox_max_sums[bbox.label] = Vec2D(bbox.max_x, bbox.max_y)
                
        print(bbox_min_sums)

if __name__ == "__main__":
    rospy.init_node("buoy_detector")
    detector = BuoyDetector()
    rospy.spin()

