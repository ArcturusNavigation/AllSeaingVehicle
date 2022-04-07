#include "math.h"

#include "cv_bridge/cv_bridge.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
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
  sensor_suite::LabeledBoundingBox2DArray box_array_;
  cv::Mat img_; 

 public:
  SegmentationNode(ros::NodeHandle n) : nh_(n) {
    img_sub_ = nh_.subscribe("/zed2i/rgb/image_rect_color", 1,
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
    for (int i = 0; i < contours_red.size(); i++) {
      cv::Rect rect = cv::boundingRect(contours_red[i]);
      cv::rectangle(img_, rect, cv::Scalar(0, 0, 255), 2);
      sensor_suite::LabeledBoundingBox2D box;
      box.x = rect.x;
      box.y = rect.y;
      box.width = rect.width;
      box.height = rect.height;
      box.label = 1;  // TODO: Change to Enum
      box_array_.boxes.push_back(box);
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
      box_array_.boxes.push_back(box);
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
      box_array_.boxes.push_back(box);
    }
    box_pub_.publish(box_array_);
    img_pub_.publish(cv_ptr->toImageMsg());
  }
};

class SegmentationClusterNode {
 protected:
  ros::NodeHandle nh_;
  message_filters::Subscriber<sensor_msgs::Image> img_sub_;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;
  ros::Publisher object_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher box_pub_;
  ros::Publisher img_pub_;
  image_geometry::PinholeCameraModel cam_model_; 

 private:
  cv::Mat img_;
  cv::Mat depth_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,sensor_msgs::CameraInfo>
      SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;

 public:
  SegmentationClusterNode(ros::NodeHandle n) : nh_(n), img_sub_(nh_, "/zed2i/zed_node/rgb/image_rect_color", 1),
                                               depth_sub_(nh_, "/zed2i/zed_node/depth/depth_registered", 1),
                                               info_sub_(nh_, "/zed2i/zed_node/rgb_raw/camera_info", 1)
                                               {
    object_pub_ = nh_.advertise<sensor_suite::ObjectArray>("/sensor_suite/objects", 1);
    box_pub_ = nh_.advertise<sensor_suite::LabeledBoundingBox2DArray>("/sensor_suite/bounding_boxes", 1);
    img_pub_ = nh_.advertise<sensor_msgs::Image>("/sensor_suite/segmented_image", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/sensor_suite/object_markers", 1);
    sync_.reset(new Sync(SyncPolicy(10), img_sub_, depth_sub_,info_sub_));
    sync_->registerCallback(boost::bind(&SegmentationClusterNode::imgCallback, this, _1, _2,_3));
  }

  void imgCallback(const sensor_msgs::ImageConstPtr& img_msg, const sensor_msgs::ImageConstPtr& depth_msg,const sensor_msgs::CameraInfoConstPtr& info_msg) {
    cam_model_.fromCameraInfo(info_msg);
    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImagePtr depth_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
      depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    img_ = cv_ptr->image;
    depth_ = depth_ptr->image;
    // cam_model_.fromCameraInfo(info_msg);
    // Convert from BGR8 to HSV
    cv::cvtColor(img_, img_, cv::COLOR_BGR2HSV);
    // Use erosion on Image:
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::erode(img_, img_, element);
    // Use dilation on Image:
    cv::dilate(img_, img_, element);
    // Colour thresholding with each color
    cv::Mat thresholds[3];
    cv::Scalar color_ranges[3][2] = {{cv::Scalar(0, 100, 100),  cv::Scalar(10, 255, 255)}, { cv::Scalar(40, 100, 100), cv::Scalar(70, 255, 255)}, { cv::Scalar(20, 100, 100),cv::Scalar(30, 255, 255)}};
    for(int i = 0; i < 3; i++){
      cv::inRange(img_, color_ranges[i][0], color_ranges[i][1],thresholds[i]);
    }
    // Get multiple contours on Image:
    std::vector<std::vector<cv::Point>> colored_contours[3]; 
    cv::Scalar colors[3] = {cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0), cv::Scalar(255, 0, 0)};
    for(int i = 0; i < 3; i++){
      cv::findContours(thresholds[i], colored_contours[i], CV_RETR_EXTERNAL,
                      CV_CHAIN_APPROX_SIMPLE);
    }
    // Create bounding boxes and objects for each contour
    sensor_suite::LabeledBoundingBox2DArray box_array_;
    sensor_suite::ObjectArray object_array_;
    visualization_msgs::MarkerArray marker_array_;

    for(int i = 0; i < 3; i++){
      for(int j = 0; j < colored_contours[i].size(); j++){
      cv::Rect rect = cv::boundingRect(colored_contours[i][j]);
        if(isValidBox(&rect)){
          cv::rectangle(img_, rect, colors[i], 4);
          sensor_suite::LabeledBoundingBox2D box;
          box.x = rect.x;
          box.y = rect.y;
          box.width = rect.width;
          box.height = rect.height;
          box.label = i + 1;  // TODO: Change to Enum
          box_array_.boxes.push_back(box);
          sensor_suite::Object object;
          cv::Point3d point = cam_model_.projectPixelTo3dRay(cv::Point2d(rect.x + rect.width / 2, rect.y + 3*rect.height / 4));
          object.point.x = point.x;
          object.point.y = point.y;
          object.point.z = point.z;
          object.label = i + 1;
          object.header = img_msg->header;
          object_array_.objects.push_back(object);
          // Visualization
          visualization_msgs::Marker marker;
          marker.header.frame_id = "zed2i_base_link";
          marker.header.stamp = ros::Time();
          marker.type = visualization_msgs::Marker::SPHERE;
          marker.action = visualization_msgs::Marker::ADD; 
          marker.pose.position = object.point; 
          marker.pose.orientation.x = 0.0;
          marker.pose.orientation.y = 0.0;
          marker.pose.orientation.z = 0.0;
          marker.pose.orientation.w = 1.0;
          marker.scale.x = .1;
          marker.scale.y = 0.1;
          marker.scale.z = 0.1; 
          marker.color.a = 1.0;
          marker.color.r = 1.0;
          marker.color.g = 1.0;
          marker.color.b = 1.0; 
          marker_array_.markers.push_back(marker);
        }else{
          // colored_contours[i].erase(colored_contours[i].begin() + j);
          // j--;
        }
      }
      // cv::drawContours(img_, colored_contours[i], -1, colors[i], 4);
    }
    box_pub_.publish(box_array_);
    object_pub_.publish(object_array_);
    img_pub_.publish(cv_ptr->toImageMsg());
    marker_pub_.publish(marker_array_);
  }

  bool isValidBox(cv::Rect* rect){
    if(rect->width*rect->height < 50){
      return false;
    }
    if(rect->width > rect->height){
      if (rect->width/rect->height > 2){
        return false; 
      }
    }
    else{
      if(rect->height/rect->width > 2){
        return false; 
      }
    }
    return true;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "segmentation_node");
  ros::NodeHandle nh;
  SegmentationClusterNode segmentation_node(nh);
  ros::spin();
  return 0;
}