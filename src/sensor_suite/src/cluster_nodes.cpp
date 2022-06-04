#include <bits/stdc++.h>
#include <iostream>
#include <sstream>
#include <vector>

#include "geometry.h"
#include "ros/ros.h"
#include "tf/transform_listener.h"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include "pcl_ros/point_cloud.h"

#include "sensor_msgs/PointCloud2.h"
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
  ros::Publisher pcl_pub_;
  ros::Publisher object_pub_;
  tf::TransformListener listener_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr buffer_;
  pcl::IndicesPtr indices_;
  pcl::search::Search<pcl::PointXYZ>::Ptr tree_;
  pcl::PointCloud<pcl::Normal>::Ptr normals_;

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  pcl::PointCloud<pcl::PointXYZ>::iterator iter;

 private:
  vector e, viewDirection;
  vector w, u, v;
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2, sensor_suite::LabeledBoundingBox2DArray>
      SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;

 public:
  ClusterNode(ros::NodeHandle n)
      : nh_(n),
        indices_(new std::vector<int>),
        pcl_sub_(nh_, "/sensor_suite/raw_cloud", 1),
        bbox_sub_(nh_, "/sensor_suite/bounding_boxes", 1),
        buffer_(new pcl::PointCloud<pcl::PointXYZ>) {
    // calculate basis vectors for projections

    // 1080 = image height
    float f = (float)1080 / 1000.;
    vector up = newVector(0, 0, 1);
    w = scale(1 / qsize(viewDirection), viewDirection);
    u = scale(1 / qsize(qcross(up, w)), qcross(up, w));
    v = qcross(w, u);
    e = newVector(800 * f, 100 * f, 0);
    viewDirection = newVector(-1, 0, 0);
    normal_estimator.setSearchMethod(tree_);
    normal_estimator.setKSearch(50);
    reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(1.0);

    // Initiate subscribers and publishers
    sync_.reset(new Sync(SyncPolicy(10), pcl_sub_, bbox_sub_));
    sync_->registerCallback(
        boost::bind(&ClusterNode::pcCallback, this, _1, _2));
    object_pub_ =
        nh_.advertise<sensor_suite::Object>("/sensor_suite/object", 1);

    // Clear point_cloud pointers
    // TODO: Move to initialization List
    normals_.reset(new pcl::PointCloud<pcl::Normal>);
    tree_.reset(new pcl::search::KdTree<pcl::PointXYZ>);
    buffer_->header.frame_id = "world";
  }

  void pcCallback(
      const sensor_msgs::PointCloud2ConstPtr& pcl_msg,
      const sensor_suite::LabeledBoundingBox2DArrayConstPtr& bbox_msg) {
    pcl::fromROSMsg(*pcl_msg, *buffer_);
    // pcl::removeNaNFromPointCloud(buffer_);  TODO // Source:
    // https://github.com/daviddoria/Examples/blob/master/c%2B%2B/PCL/Filters/RemoveNaNFromPointCloud/RemoveNaNFromPointCloud.cpp
    normal_estimator.setInputCloud(buffer_);
    reg.setInputCloud(buffer_);
    extract.setInputCloud(buffer_);
    reg.setIndices(indices_);

    normal_estimator.compute(*normals_);
    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);

    // Loop through every point in each cluster and project to bounding box
    int min_count = 0;  // minimum points in cluster labelled as one object to
                        // avoid nosie TODO: Use
    for (auto cluster : clusters) {
      pcl::PointIndices::Ptr cluster_indices(new pcl::PointIndices);
      cluster_indices->indices = cluster.indices;
      extract.setIndices(cluster_indices);
      extract.filter(*cloud);
      pcl::CentroidPoint<pcl::PointXYZ> centroid;
      pcl::PointXYZ centroid_point;
      for (int i = 0; i < cloud->points.size(); i++) {
        vector p;
        p.x = cloud->points[i].x;
        p.y = cloud->points[i].y;
        p.z = cloud->points[i].z;
        centroid.add(cloud->points[i]);
        }
      centroid.get(centroid_point);
      vector p;
      p.x = centroid_point.x;
      p.y = centroid_point.y;
      p.z = centroid_point.z;
      pixel px = projectPointToImage(
            p, 1080,
            1920);  // Image sized based on
                    // https://support.stereolabs.com/hc/en-us/articles/360007395634-What-is-the-camera-focal-length-and-field-of-view-  
      int label = getRegion(px, bbox_msg);
      sensor_suite::Object new_object;
      // new_object.point = centroid_point; TODO: Finish
      new_object.label = label;
      new_object.num_points = cloud->points.size();
      object_pub_.publish(new_object);
    }
    return;
  }
  // TODO: Finish this function

  // Loops through bounding boxes and returns the label of the bounding box
  int getRegion(
      pixel px,
      const sensor_suite::LabeledBoundingBox2DArrayConstPtr& bbox_msg) {
    for (int i = 0; i < bbox_msg->boxes.size(); i++) {
      float min_x = bbox_msg->boxes[i].x - bbox_msg->boxes[i].width / 2;
      float max_x = bbox_msg->boxes[i].x + bbox_msg->boxes[i].width / 2;
      float min_y = bbox_msg->boxes[i].y - bbox_msg->boxes[i].height / 2;
      float max_y = bbox_msg->boxes[i].y + bbox_msg->boxes[i].height / 2;
      if (px.u > min_x && px.u < max_x && px.v > min_y && px.v < max_y) {
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

// using namespace std::chrono_literals;

// class ColorClusterNode {
//  protected:
//   ros::NodeHandle nh_;
//   message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub_;
//   ros::Publisher pcl_pub_;
//   ros::Publisher object_pub_;
//   tf::TransformListener listener_;

//   pcl::PointCloud<pcl::PointXYZRGB>::Ptr buffer_;
//   pcl::IndicesPtr indices_;
//   pcl::search::Search<pcl::PointXYZRGB>::Ptr tree_;
//   pcl::PointCloud<pcl::Normal>::Ptr normals_;

//   pcl::ExtractIndices<pcl::PointXYZRGB> extract;
//   pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
//   pcl::SACSegmentation<pcl::PointXYZRGB> seg;
//   pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
//   pcl::PointCloud<pcl::PointXYZRGB>::iterator iter;

//  public:
//   ClusterNode(ros::NodeHandle n)
//       : nh_(n),
//         indices_(new std::vector<int>),
//         pcl_sub_(nh_, "/zed/point_cloud/cloud_registered", 1),
//         buffer_(new pcl::PointCloud<pcl::PointXYZRGB>) {
//     // Initiate subscribers and publishers
//     pcl_sub_.registerCallback(boost::bind(&ClusterNode::cloud_cb, this, _1));
//     object_pub_ = nh_.advertise<sensor_suite::ObjectArray>("/sensor_suite/new_objects", 1);

//     // Clear point_cloud pointers
//     // TODO: Move to initialization List
//     tree_.reset(new pcl::search::KdTree<pcl::PointXYZRGB>);
//     buffer_->header.frame_id = "map";
//   }

//   void cloud_cb(
//       const sensor_msgs::PointCloud2ConstPtr& pcl_msg) {
//     pcl::fromROSMsg(*pcl_msg, *buffer_);
//     // pcl::removeNaNFromPointCloud(buffer_);  TODO // Source:
//     // https://github.com/daviddoria/Examples/blob/master/c%2B%2B/PCL/Filters/RemoveNaNFromPointCloud/RemoveNaNFromPointCloud.cpp
//     normal_estimator.setInputCloud(buffer_);
//     reg.setInputCloud(buffer_);
//     reg.setIndices(indices_);
//     reg.setSearchMethod(tree);
//     reg.setDistanceThreshold(10);
//     reg.setPointColorThreshold(6);
//     reg.setREgionColorThreshold(5);
//     reg.setMinClusterSize(600);
//     extract.setInputCloud(buffer_);

//     std::vector<pcl::PointIndices> clusters;
//     reg.extract(clusters);
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
//         new pcl::PointCloud<pcl::PointXYZRGB>);

//     // Loop through every point in each cluster and project to bounding box
//     int min_count = 0;  // minimum points in cluster labelled as one object to
//                         // avoid nosie TODO: Use
//     for (auto cluster : clusters) {
//       pcl::PointIndices::Ptr cluster_indices(new pcl::PointIndices);
//       cluster_indices->indices = cluster.indices;
//       extract.setIndices(cluster_indices);
//       extract.filter(*cloud);
//       pcl::CentroidPoint<pcl::PointXYZRGB> centroid;
//       pcl::PointXYZRGB centroid_point;
//       for (int i = 0; i < cloud->points.size(); i++) {
//         centroid.add(cloud->points[i]);       
//         int label = getLabel();
//         if (label != 0) {
//           centroid.get(centroid_point);
//           sensor_suite::Object new_object;
//           // new_object.point = centroid_point; TODO: Finish
//           new_object.label = label;
//           new_object.num_points = cloud->points.size();
//           object_pub_.publish(new_object);
//           break;
//         }
//       }
//     }
//     return;
//   }
//   int getLabel(pcl::PointXYZRGB p){

//   }
// };

int main(int argc, char** argv) {
  ros::init(argc, argv, "cluster_node");
  ros::NodeHandle n;
  ClusterNode cluster_node(n);
  ros::spin();
  return 0;
}