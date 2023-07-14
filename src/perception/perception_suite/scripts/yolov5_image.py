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

class Yolov5Detector():
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        rospack = rospkg.RosPack()
        
        # Get pretrained yolov5 model from local
        path_hubconfig = f"/home/{getpass.getuser()}/yolov5" # download yolov5 on the home directory
        path_trained_model = rospack.get_path("perception_suite") + "/model/buoy_detection_best_weights_v2.pt"
        self.model = torch.hub.load(path_hubconfig, 'custom', path=path_trained_model, source='local')

        # Model inference settings
        # self.model.classes = [32] # Only detect sports ball
        # self.model.conf = 0.05    # Confidence threshold
        # self.model.iou = 0.25     # NMS IoU threshold (how close it is to an actual sports ball)

        # Subscribers and publishers
        self.pub = rospy.Publisher('/perception_suite/bounding_boxes', LabeledBoundingBox2DArray, queue_size=1)
        self.img_pub = rospy.Publisher('/perception_suite/segmented_image', sensor_msgs.msg.Image, queue_size=1)

        # Get image
        self.img = PIL.Image.open(rospack.get_path("perception_suite") + "/images/test.jpg")
        self.img = np.array(self.img.convert("RGB"))

        # Publisher loop
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            # Detect buoys using yolov5
            results = self.model(self.img)
            preds = results.pandas().xyxy[0]  # Image 1 predictoins
            rospy.loginfo(preds)
            bboxes = LabeledBoundingBox2DArray()
            # set header of bboxes
            bboxes.header.stamp = rospy.Time.now()
            bboxes.header.frame_id = "zed2i_camera_center"

            for _, row in preds.iterrows():
                bbox = LabeledBoundingBox2D()
                
                bbox.min_x = int(row['xmin'])
                bbox.min_y = int(row['ymin'])
                
                bbox.max_x = int(row['xmax'])
                bbox.max_y = int(row['ymax'])
                
                bbox.label = row['class']
                bbox.probability = row['confidence']
                bboxes.boxes.append(bbox)
                cv2.rectangle(self.img, (bbox.min_x, bbox.min_y), (bbox.max_x, bbox.max_y), (255, 0, 0), 4)

            # Publish image and boudnign box
            self.img_pub.publish(self.bridge.cv2_to_imgmsg(self.img, "rgb8"))
            self.pub.publish(bboxes)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("yolov5_image")
    detector = Yolov5Detector()
    rospy.spin()
