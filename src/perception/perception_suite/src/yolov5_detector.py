"""
References:
https://pytorch.org/hub/ultralytics_yolov5/
https://pypi.org/project/yolo5/
"""

import getpass
import torch
import cv_bridge 
import cv2
import rospy
import rospkg
from sensor_msgs.msg import Image
from perception_suite.msg import LabeledBoundingBox2D, LabeledBoundingBox2DArray

class Yolov5Detector():
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        rospack = rospkg.RosPack()

        # Get pretrained yolov5 models for colored buoys and cardinal markers
        path_hubconfig = f"/home/{getpass.getuser()}/yolov5"
        #path_color_model = rospack.get_path("perception_suite") + "/model/njord_weights_color1.pt"
        path_color_model = rospack.get_path("perception_suite") + "/model/buoy_detection_best_weights_v7.pt"
        self.color_model = torch.hub.load(path_hubconfig, 'custom', path=path_color_model, source='local')
        path_cardinal_model = rospack.get_path("perception_suite") + "/model/njord_weights_cardinal1.pt"
        self.cardinal_model = torch.hub.load(path_hubconfig, 'custom', path=path_cardinal_model, source='local')

        # Model options
        self.color_model.classes = [2, 4]

        # Subscribers and publishers
        self.bbox_pub = rospy.Publisher('/perception_suite/bounding_boxes', LabeledBoundingBox2DArray, queue_size=1)
        self.img_pub = rospy.Publisher('/perception_suite/segmented_image', Image, queue_size=1)
        self.img_sub = rospy.Subscriber(
            '/zed2i/zed_node/rgb/image_rect_color', 
            Image, 
            self.img_callback, 
            queue_size=1, 
            buff_size=2**24,
        )
    
    def img_callback(self, img):
        try:
            img = self.bridge.imgmsg_to_cv2(img, "rgb8")
        except cv_bridge.CvBridgeError as e:
            rospy.loginfo(e)

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

        self.img_pub.publish(self.bridge.cv2_to_imgmsg(img, "rgb8"))
        self.bbox_pub.publish(bboxes)

if __name__ == "__main__":
    rospy.init_node("yolov5_detector")
    detector = Yolov5Detector()
    rospy.spin()
