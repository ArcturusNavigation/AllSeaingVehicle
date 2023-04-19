"""
References:
https://pytorch.org/hub/ultralytics_yolov5/
https://pypi.org/project/yolo5/
"""

#TODO: Update requirements.txt so that after installation you can run this segmentaiton node
import torch
import cv_bridge 
import cv2

import rospy
from sensor_msgs.msg import Image, CompressedImage
from perception_suite.msg import LabeledBoundingBox2D, LabeledBoundingBox2DArray

ZED_TOPIC = '/zed2i/zed_node/rgb/image_rect_color'
KEYS_TO_REMOVE = ["epoch", "best_fitness", "ema", "updates", "optimizer", "opt", "git", "date"]
class Yolov5Detector():
    def __init__(self, img_topic=ZED_TOPIC):
        self.bridge = cv_bridge.CvBridge()
        self.model = torch.hub.load('/home/toyat/.local/lib/python3.8/site-packages/yolov5', 'custom', path='model/yolov5s.pt', source='local')
        self.pub = rospy.Publisher('/perception_suite/bounding_boxes', LabeledBoundingBox2DArray, queue_size=1)
        self.img_pub = rospy.Publisher('/perception_suite/segmented_image', Image, queue_size=1)
        self.img_sub = rospy.Subscriber('/zed2i/zed_node/rgb/image_rect_color', Image, self.img_callback, queue_size=1, buff_size=2**24)
    
    def img_callback(self, img):
        try:
            img = self.bridge.imgmsg_to_cv2(img, "rgb8") # TODO: Match to model training(bgr8 or rgb8)
        except cv_bridge.CvBridgeError as e: 
            rospy.loginfo(e)
        results = self.model(img) # May need to resize image
        preds = results.pandas().xyxy[0]  # img1 predictions (pandas)
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
            cv2.rectangle(img, (bbox.min_x, bbox.min_y), (bbox.max_x, bbox.max_y), (255,0,0), 4)
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(img, "rgb8"))
        self.pub.publish(bboxes)

if __name__ == "__main__":
    rospy.init_node("yolov5", anonymous=True)
    detector = Yolov5Detector()
    rospy.spin()
