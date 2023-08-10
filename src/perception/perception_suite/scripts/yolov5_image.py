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
        
        # Get pretrained yolov5 model from local
        path_hubconfig = f"/home/{getpass.getuser()}/yolov5" # download yolov5 on the home directory
        path_trained_model = rospack.get_path("perception_suite") + "/model/buoy_detection_best_weights_v7.pt"
        model = torch.hub.load(path_hubconfig, 'custom', path=path_trained_model, source='local')

        # Model inference settings
        # model.classes = [32] # Only detect sports ball
        # model.conf = 0.05    # Confidence threshold
        # model.iou = 0.25     # NMS IoU threshold (how close it is to an actual sports ball)

        # Subscribers and publishers
        bbox_pub = rospy.Publisher('/perception_suite/bounding_boxes', LabeledBoundingBox2DArray, queue_size=1)
        img_pub = rospy.Publisher('/perception_suite/segmented_image', sensor_msgs.msg.Image, queue_size=1)

        # Publisher loop
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            # Get image
            img = PIL.Image.open(rospack.get_path("perception_suite") + "/images/straight_test.jpg")
            img = np.array(img.convert("RGB"))

            # Detect buoys using yolov5
            results = model(img)
            preds = results.pandas().xyxy[0]  # Image predictions
            rospy.loginfo(preds)
            bboxes = LabeledBoundingBox2DArray()

            # Set header of bboxes
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
                cv2.rectangle(img, (bbox.min_x, bbox.min_y), (bbox.max_x, bbox.max_y), (255, 0, 0), 4)

            # Publish image and boudnign box
            img_pub.publish(bridge.cv2_to_imgmsg(img, "rgb8"))
            bbox_pub.publish(bboxes)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("yolov5_image")
    detector = Yolov5Image()
    rospy.spin()