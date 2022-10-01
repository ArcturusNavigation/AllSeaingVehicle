#include <bits/stdc++.h>
#include <iostream>
#include <sstream>
#include <vector>

#include "cv_bridge/cv_bridge.h"

#include "geometry.h"
#include "ros/ros.h"
#include "tf/transform_listener.h"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include "pcl_ros/transforms.h"

#include "image_geometry/pinhole_camera_model.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "sensor_msgs/PointCloud2.h"
#
#include "sensor_suite/LabeledBoundingBox2D.h"
#include "sensor_suite/LabeledBoundingBox2DArray.h"
#include <sensor_suite/Object.h>
#include "sensor_suite/ObjectArray.h"


// Projection code based on personal work for 6.172
// Clustering based on following tutorial:
// https://pcl.readthedocs.io/en/latest/region_growing_segmentation.html#region-growing-segmentation
// How to sync messages: https://pkok.github.io/2020/08/02/
// Extract indices:
// https://pointclouds.org/documentation/tutorials/extract_indices.html

class ClusterNode {
 protected:
  ros::NodeHandle nh_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub_;
  message_filters::Subscriber<sensor_suite::LabeledBoundingBox2DArray>
      bbox_sub_;
  ros::Subscriber cam_sub_;
  ros::Publisher pcl_pub_;
  ros::Publisher object_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher debug_pub_;
  tf::TransformListener listener_;

  image_geometry::PinholeCameraModel cam_model_;
  sensor_msgs::CameraInfo cam_info_;
  cv_bridge::CvImage debug_img_;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr buffer_;
  pcl::PointCloud<pcl::PointXYZ>::iterator iter;
  
  pcl::IndicesPtr indices_;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  
  pcl::search::Search<pcl::PointXYZ>::Ptr tree_;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec_;

 private:
  vector e, viewDirection;
  vector w, u, v;
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2, sensor_suite::LabeledBoundingBox2DArray>
      SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync_;

 public:
  ClusterNode(ros::NodeHandle n)
      : nh_(n),
        indices_(new std::vector<int>),
        pcl_sub_(nh_, "/velodyne_points2", 1),
        bbox_sub_(nh_, "/sensor_suite/bounding_boxes", 1),
        sync_(SyncPolicy(10), pcl_sub_,bbox_sub_),
        buffer_(new pcl::PointCloud<pcl::PointXYZ>) {
    cam_sub_ = nh_.subscribe("/zed_node/camera_info", 1,
                             &ClusterNode::setupCamera, this);
    // Calculate basis vectors for projections TODO: Fix
    // 1080 = image height
    // float f = (float)1080 / 1000.;
    // vector up = newVector(0, 0, 1);
    // w = scale(1 / qsize(viewDirection), viewDirection);
    // u = scale(1 / qsize(qcross(up, w)), qcross(up, w));
    // v = qcross(w, u);
    // e = newVector(800 * f, 100 * f, 0);
    // viewDirection = newVector(-1, 0, 0);
    tree_.reset(new pcl::search::KdTree<pcl::PointXYZ>);
    ec_.setClusterTolerance(.5); // 50 cm 
    ec_.setMinClusterSize(3);
    ec_.setMaxClusterSize(100);
    ec_.setSearchMethod(tree_);


    // Initiate subscribers and publishers
    // sync_.reset(new Sync(SyncPolicy(10), pcl_sub_, bbox_sub_));
    sync_.registerCallback(
        boost::bind(&ClusterNode::pcCallback,this, _1, _2));
    object_pub_ =
        nh_.advertise<sensor_suite::ObjectArray>("/sensor_suite/objects", 1);
    marker_pub_ =
        nh_.advertise<visualization_msgs::MarkerArray>("/sensor_suite/markers",1);
    debug_pub_ = nh_.advertise<sensor_msgs::Image>("/sensor_suite/projection_img", 1);
    // Clear point_cloud pointers
    // TODO: Move to initialization List
    // normals_.reset(new pcl::PointCloud<pcl::Normal>);
    // buffer_->header.frame_id = "base_link";
    std::cout << "Initialized!" << std::endl;
  }
  void setupCamera(const sensor_msgs::CameraInfoConstPtr& cam_info) {
    cam_info_ = *cam_info;
    debug_img_.header.frame_id = "camera";
    debug_img_.encoding = "bgr8";
    debug_img_.image = cv::Mat::zeros(cam_info->height, cam_info->width, CV_8UC3);
    cam_model_.fromCameraInfo(cam_info);
    cam_sub_.shutdown();
  }

  void pcCallback(
      const sensor_msgs::PointCloud2ConstPtr& pcl_msg,
      const sensor_suite::LabeledBoundingBox2DArrayConstPtr& bbox_msg) {
    // std::cout << "Callback!" << std::endl;
    pcl::fromROSMsg(*pcl_msg, *buffer_);
    debug_img_.image.setTo(0);
    tf::StampedTransform pc_tf;
    listener_.lookupTransform("map","base_link",ros::Time(0),pc_tf);
    pcl_ros::transformPointCloud(*buffer_,*buffer_,pc_tf);
    // pcl::removeNaNFromPointCloud(buffer_);  TODO // Source:
    // https://github.com/daviddoria/Examples/blob/master/c%2B%2B/PCL/Filters/RemoveNaNFromPointCloud/RemoveNaNFromPointCloud.cpp
    ec_.setInputCloud(buffer_);
    extract.setInputCloud(buffer_);
    // ec_.setIndices(indices_);

    // normal_estimator.compute(*normals_);
    std::vector<pcl::PointIndices> clusters;
    ec_.extract(clusters);
    std::cout << "Found " << clusters.size() << " clusters." << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    sensor_suite::ObjectArray objects;
    visualization_msgs::MarkerArray markers; 
    // Loop through every point in each cluster and project to bounding box
    int min_count = 0;  // minimum points in cluster labelled as one object to
                        // avoid nosie TODO: Use
    int id = 0;
    for (auto cluster : clusters) {
      pcl::PointIndices::Ptr cluster_indices(new pcl::PointIndices);
      cluster_indices->indices = cluster.indices;
      extract.setIndices(cluster_indices);
      extract.filter(*cloud);
      pcl::CentroidPoint<pcl::PointXYZ> centroid;
      pcl::PointXYZ centroid_point;
      // TODO(hturner08): Optimize
      for (int i = 0; i < cloud->points.size(); i++) {
        vector p;
        p.x = cloud->points[i].x;
        p.y = cloud->points[i].y;
        p.z = cloud->points[i].z;
        centroid.add(cloud->points[i]);
      }
      centroid.get(centroid_point);
      centroid_point.z = 0;
      vector p;
      p.x = centroid_point.x;
      p.y = centroid_point.y;
      p.z = centroid_point.z;
      geometry_msgs::PointStamped point;
      point.point.x = p.x;
      point.point.y = p.y;
      point.point.z = p.z;
      point.header.frame_id = "map";
      point.header.stamp = ros::Time(0);
      geometry_msgs::PointStamped transformed_point;
      // Transform point with cam_Tf to the 3d position with respect to the camera
      listener_.transformPoint("camera", point, transformed_point);
      cv::Point3d cam_point(transformed_point.point.x, transformed_point.point.y, transformed_point.point.z);
      cv::Point2d px = cam_model_.project3dToPixel(cam_point);
      std::cout << "Projected point: " << px.x << " " << px.y << std::endl;
      markPixel(px, debug_img_.image);
      // pixel px = projectPointToImage(
      //       p, 1080,
      //       1920);  // Image sized based on
                    // https://support.stereolabs.com/hc/en-us/articles/360007395634-What-is-the-camera-focal-length-and-field-of-view-  
      sensor_suite::Object new_object;
      new_object.pos.x = centroid_point.x; //TODO: Finish
      new_object.pos.y = centroid_point.y;
      new_object.pos.z = centroid_point.z; 
      new_object.label = label; 
      new_object.num_points = cloud->points.size();
      new_object.task_label = new_object.num_points > 3 ? 1 : 0; // TODO: Replace with getRegion call
      objects.objects.push_back(new_object);
      markers.markers.push_back(createMarker(centroid_point,id++));
    }
    object_pub_.publish(objects);
    marker_pub_.publish(markers);
    debug_img_.header.stamp = ros::Time::now();
    debug_pub_.publish(debug_img_);
    return;
  }

  visualization_msgs::Marker createMarker(pcl::CentroidPoint<pcl::PointXYZ> centroid, int id){
    visualization_msgs::Marker marker; 
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "course_objects"; // TODO: Refactor
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = centroid_point.x;
    marker.pose.position.y = centroid_point.y;
    marker.pose.position.z = centroid_point.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = .2;
    marker.scale.y = .2;
    marker.scale.z = .2;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    marker.lifetime = ros::Duration(0.5);
    return marker;
  }

  // TODO: Finish this function
  void markPixel(cv::Point2d px, cv::Mat& img) {
    for(int i = std::max((int)(px.x-25),0); i < std::min((int)(px.x + 25),(int)(cam_info_.width)); i++){
      for(int j = std::max((int)(px.y-25),0); j < std::min((int)(px.y + 25),(int)(cam_info_.height)); j++){
        img.at<uchar>(j,i) = 255;
      }
    }
  }
  // Loops through bounding boxes and returns the label of the bounding box
  int getRegion(
      cv::Point2d px,
      const sensor_suite::LabeledBoundingBox2DArrayConstPtr& bbox_msg) {
    for (int i = 0; i < bbox_msg->boxes.size(); i++) {
      float min_x = bbox_msg->boxes[i].x - bbox_msg->boxes[i].width / 2;
      float max_x = bbox_msg->boxes[i].x + bbox_msg->boxes[i].width / 2;
      float min_y = bbox_msg->boxes[i].y - bbox_msg->boxes[i].height / 2;
      float max_y = bbox_msg->boxes[i].y + bbox_msg->boxes[i].height / 2;
      if (px.x > min_x && px.x < max_x && px.y > min_y && px.y < max_y) {
        return bbox_msg->boxes[i].label;
      }
    }
    return 0;
  }

  // Based of this piazza post: https://piazza.com/class/kt40btbwj942gr?cid=528
  vector projectPointToPlane(vector point) {
    // center image frame
    vector normalVector = qcross(u, v);

    float scalar =
        qdot(normalVector, e) / qdot(qsubtract(point, e), normalVector);
    vector projectedPoint = qsubtract(e, scale(scalar, qsubtract(point, e)));
    return projectedPoint;
  }
  pixel projectPointToImage(vector point, int image_height, int image_width) {
    vector projection = projectPointToPlane(point);
    double index_i = -(double)projection.y + (double)(image_width / 2.0);
    double index_j = (double)projection.z + (double)(image_height / 2.0f);
    if (index_i < 0) {
      index_i = -1;
    } else if (index_i > image_width) {
      index_i = image_width + 1;
    }
    if (index_j < 0) {
      index_j = -1;
    } else if (index_j > image_height) {
      index_j = image_height + 1;
    }
    pixel projection_pixel;
    projection_pixel.u = index_i;
    projection_pixel.v = index_j;
    return projection_pixel;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "cluster_node");
  ros::NodeHandle n;
  ClusterNode cluster_node(n);
  ros::spin();
  return 0;
}