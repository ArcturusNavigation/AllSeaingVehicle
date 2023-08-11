"""
References:
https://pytorch.org/hub/ultralytics_yolov5/
https://pypi.org/project/yolo5/
"""

import getpass
import numpy as np
import torch
import cv_bridge 
import cv2
import PIL
import rospy
import rospkg
import sensor_msgs
from perception_suite.msg import LabeledBoundingBox2D, LabeledBoundingBox2DArray

class Yolov5Image():
    def __init__(self):
        bridge = cv_bridge.CvBridge()
        rospack = rospkg.RosPack()
        
        # Get pretrained yolov5 models for colored buoys and cardinal markers
        path_hubconfig = f"/home/{getpass.getuser()}/yolov5"
        path_color_model = rospack.get_path("perception_suite") + "/model/buoy_detection_best_weights_v7.pt"
        self.color_model = torch.hub.load(path_hubconfig, 'custom', path=path_color_model, source='local')
        path_cardinal_model = rospack.get_path("perception_suite") + "/model/best_cardinal_marks.pt"
        self.cardinal_model = torch.hub.load(path_hubconfig, 'custom', path=path_cardinal_model, source='local')

        # Subscribers and publishers
        bbox_pub = rospy.Publisher('/perception_suite/bounding_boxes', LabeledBoundingBox2DArray, queue_size=1)
        img_pub = rospy.Publisher('/perception_suite/segmented_image', sensor_msgs.msg.Image, queue_size=1)

        # Publisher loop
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            # Get image
            img = PIL.Image.open(rospack.get_path("perception_suite") + "/images/test_cardinal_west.jpg")
            img = np.array(img.convert("RGB"))

            # Get image predictions for colored buoys and cardinal markers
            color_results = self.color_model(img)
            color_preds = color_results.pandas().xyxy[0]
            cardinal_results = self.cardinal_model(img)        
            cardinal_preds = cardinal_results.pandas().xyxy[0]

            rospy.loginfo(color_preds)
            rospy.loginfo(cardinal_preds)

            # Set header of bboxes
            bboxes = LabeledBoundingBox2DArray()
            bboxes.header.stamp = rospy.Time.now()
            bboxes.header.frame_id = "zed2i_camera_center"

            # Set bounding boxes around colored buoys
            for _, row in color_preds.iterrows():
                bbox = LabeledBoundingBox2D()
                
                bbox.min_x = int(row['xmin'])
                bbox.min_y = int(row['ymin'])
                
                bbox.max_x = int(row['xmax'])
                bbox.max_y = int(row['ymax'])
                
                bbox.label = row['class']
                bbox.probability = row['confidence']
                bboxes.boxes.append(bbox)
                cv2.rectangle(img, (bbox.min_x, bbox.min_y), (bbox.max_x, bbox.max_y), (255, 0, 0), 4)

            # Set bounding boxes around cardinal markers
            for _, row in cardinal_preds.iterrows():
                bbox = LabeledBoundingBox2D()
                
                bbox.min_x = int(row['xmin'])
                bbox.min_y = int(row['ymin'])
                
                bbox.max_x = int(row['xmax'])
                bbox.max_y = int(row['ymax'])
                
                bbox.label = row['class']
                bbox.probability = row['confidence']
                bboxes.boxes.append(bbox)
                cv2.rectangle(img, (bbox.min_x, bbox.min_y), (bbox.max_x, bbox.max_y), (0, 0, 255), 4)

            # Publish image and boudnign box
            img_pub.publish(bridge.cv2_to_imgmsg(img, "rgb8"))
            bbox_pub.publish(bboxes)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("yolov5_image")
    detector = Yolov5Image()
    rospy.spin()
