import torch
import cv_bridge
import cv2

import rospy
from sensor_msgs.msg import Image, CompressedImage
from perception_suite.msg import LabeledBoundingBox2D, LabeledBoundingBox2DArray

class BuoySegmentation:
    def __init__(self):

        self.bridge = cv_bridge.CvBridge()

        # Subscribers and publishers
        self.pub = rospy.Publisher(
            "/perception_suite/bounding_boxes", LabeledBoundingBox2DArray, queue_size=1
        )
        self.img_pub = rospy.Publisher(
            "/perception_suite/segmented_image", Image, queue_size=1
        )
        self.img_sub = rospy.Subscriber(
            "/zed2i/zed_node/rgb/image_rect_color",
            Image,
            self.img_callback,
            queue_size=1,
            buff_size=2 ** 24,
        )

    def img_callback(self, img):
        try:
            img = self.bridge.imgmsg_to_cv2(img, "rgb8")
        except cv_bridge.CvBridgeError as e:
            rospy.loginfo(e)

        results = self.model(img)  # May need to resize image
        preds = results.pandas().xyxy[0]  # img1 predictions (pandas)
        rospy.loginfo(preds)
        bboxes = LabeledBoundingBox2DArray()
        # set header of bboxes
        bboxes.header.stamp = rospy.Time.now()
        bboxes.header.frame_id = "zed2i_camera_center"

        for _, row in preds.iterrows():
            bbox = LabeledBoundingBox2D()

            bbox.min_x = int(row["xmin"])
            bbox.min_y = int(row["ymin"])

            bbox.max_x = int(row["xmax"])
            bbox.max_y = int(row["ymax"])

            bbox.label = row["class"]
            bbox.probability = row["confidence"]
            bboxes.boxes.append(bbox)
            cv2.rectangle(
                img, (bbox.min_x, bbox.min_y), (bbox.max_x, bbox.max_y), (255, 0, 0), 4
            )
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(img, "rgb8"))
        self.pub.publish(bboxes)


if __name__ == "__main__":
    rospy.init_node("buoy_segmentation", anonymous=True)
    detector = BuoySegmentation()
    rospy.spin()
