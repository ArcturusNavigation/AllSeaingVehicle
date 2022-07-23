#include "math.h"

#include "cv_bridge/cv_bridge.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_suite/LabeledBoundingBox2D.h"
#include "sensor_suite/LabeledBoundingBox2DArray.h"
#include <sensor_suite/Object.h>
#include "sensor_suite/ObjectArray.h"
#include <image_geometry/pinhole_camera_model.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

// TODO
// Refactor so each new segmentation node extends base class 
// Add enums for colors for more readable array access


class SegmentationNode {
 protected:
  ros::NodeHandle nh_;
  ros::Subscriber img_sub_;
  ros::Publisher img_pub_;
  ros::Publisher box_pub_;

 private:
  cv::Mat img_; 

 public:
  SegmentationNode(ros::NodeHandle n) : nh_(n) {
    img_sub_ = nh_.subscribe("/zed2i/zed_node/rgb/image_rect_color", 1,
                             &SegmentationNode::imgCallback, this);
    img_pub_ = nh_.advertise<sensor_msgs::Image>("/sensor_suite/image", 1);
    box_pub_ = nh_.advertise<sensor_suite::LabeledBoundingBox2DArray>(
        "/sensor_suite/bounding_boxes", 1);
  }
  void imgCallback(const sensor_msgs::ImageConstPtr& msg) {
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
    cv::drawContours(img_, contours_red, -1, cv::Scalar(0, 0, 255), 2);
    cv::drawContours(img_, contours_green, -1, cv::Scalar(0, 255, 0), 2);
    cv::drawContours(img_, contours_yellow, -1, cv::Scalar(255, 255, 0), 2);
    // Create bounding boxes for each contour
    sensor_suite::LabeledBoundingBox2DArray box_array;
    for (int i = 0; i < contours_red.size(); i++) {
      cv::Rect rect = cv::boundingRect(contours_red[i]);
      cv::rectangle(img_, rect, cv::Scalar(0, 0, 255), 2);
      sensor_suite::LabeledBoundingBox2D box;
      box.x = rect.x;
      box.y = rect.y;
      box.width = rect.width;
      box.height = rect.height;
      box.label = 1;  // TODO: Change to Enum
      box_array.boxes.push_back(box);
    }
    for (int i = 0; i < contours_green.size(); i++) {
      cv::Rect rect = cv::boundingRect(contours_green[i]);
      cv::rectangle(img_, rect, cv::Scalar(0, 255, 0), 2);
      sensor_suite::LabeledBoundingBox2D box;
      box.x = rect.x;
      box.y = rect.y;
      box.width = rect.width;
      box.height = rect.height;
      box.label = 2;
      box_array.boxes.push_back(box);
    }
    for (int i = 0; i < contours_yellow.size(); i++) {
      cv::Rect rect = cv::boundingRect(contours_yellow[i]);
      cv::rectangle(img_, rect, cv::Scalar(0, 255, 255), 2);
      sensor_suite::LabeledBoundingBox2D box;
      box.x = rect.x;
      box.y = rect.y;
      box.width = rect.width;
      box.height = rect.height;
      box.label = 3;
      box_array.boxes.push_back(box);
    }
    box_array.header.stamp = msg->header.stamp;
    box_pub_.publish(box_array);
    img_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "segmentation_node");
  ros::NodeHandle nh;
  SegmentationNode segmentation_node(nh);
  ros::spin();
  return 0;
}