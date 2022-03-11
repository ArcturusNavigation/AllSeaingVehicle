#include "ros/ros.h"
#include "vision_msgs/BoundingBox2DArray.msg"

class SegmentationNode {
 protected:
  ros::NodeHandle nh_;
  ros::Subscriber img_sub_;
  ros::Publisher box_pub_;

 private:
  BoundBox2DArray box_array_;
  cv::Mat img_;

 public:
  SegmentationNode::SegmentationNode(ros::NodeHandle n) : nh_(n) {
    img_sub_ = nh_.subscribe("/sensor_suite/image", 1,
                             &SegmentationNode::imgCallback, this);
    box_pub_ = nh_.advertise<vision_msgs::BoundingBox2DArray>(
        "/sensor_suite/bounding_boxes", 1);
  }
  SegmentationNode::imgCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    img_ = cv_ptr->image;
    // Convert from BGR8 to HSV
    cv::cvtColor(img_, img_, cv::COLOR_BGR2HSV);
    // Use erosion on Image:
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::erode(img_, img_, element);
    // Use dilation on Image:
    cv::dilate(img_, img_, element);
    // Colour segmentation with each color
    // Use red thresholding on Image:
    cv::Mat thresh_red;
    cv::inRange(img_, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255),
                thresh_red);
    // Use green thresholding on Image:
    cv::Mat thresh_green;
    cv::inRange(img_, cv::Scalar(40, 100, 100), cv::Scalar(70, 255, 255),
                thresh_green);
    // Use yellow thresholding on image:
    cv::Mat thresh_yellow;
    cv::inRange(img_, cv::Scalar(20, 100, 100), cv::Scalar(30, 255, 255),
                thresh_yellow);
    // Get multiple contours on Image:
    std::vector<std::vector<cv::Point>> contours_red;
    std::vector<std::vector<cv::Point>> contours_green;
    std::vector<std::vector<cv::Point>> contours_yellow;
    cv::findContours(thresh_red, contours_red, CV_RETR_EXTERNAL,
                     CV_CHAIN_APPROX_SIMPLE);
    cv::findContours(thresh_green, contours_green, CV_RETR_EXTERNAL,
                     CV_CHAIN_APPROX_SIMPLE);
    cv::findContours(thresh_yellow, contours_yellow, CV_RETR_EXTERNAL,
                     CV_CHAIN_APPROX_SIMPLE);
    // Create bounding boxes for each contour
    for (int i = 0; i < contours_red.size(); i++) {
      cv::Rect rect = cv::boundingRect(contours_red[i]);
      cv::rectangle(img_, rect, cv::Scalar(0, 0, 255), 2);
      vision_msgs::BoundingBox2D box;
      box.x = rect.x;
      box.y = rect.y;
      box.width = rect.width;
      box.height = rect.height;
      box.label = "red";
      box_array_.boxes.push_back(box);
    }
    for (int i = 0; i < contours_green.size(); i++) {
      cv::Rect rect = cv::boundingRect(contours_green[i]);
      cv::rectangle(img_, rect, cv::Scalar(0, 255, 0), 2);
      vision_msgs::BoundingBox2D box;
      box.x = rect.x;
      box.y = rect.y;
      box.width = rect.width;
      box.height = rect.height;
      box.label = "green";
      box_array_.boxes.push_back(box);
    }
    for (int i = 0; i < contours_yellow.size(); i++) {
      cv::Rect rect = cv::boundingRect(contours_yellow[i]);
      cv::rectangle(img_, rect, cv::Scalar(0, 255, 255), 2);
      vision_msgs::BoundingBox2D box;
      box.x = rect.x;
      box.y = rect.y;
      box.width = rect.width;
      box.height = rect.height;
      box.label = "yellow";
      box_array_.boxes.push_back(box);
    }
    box_pub_.publish(box_array_);
  }
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "segmentation_node");
  ros::NodeHandle nh;
  SegmentationNode segmentation_node(nh);
  ros::spin();
  return 0;
}