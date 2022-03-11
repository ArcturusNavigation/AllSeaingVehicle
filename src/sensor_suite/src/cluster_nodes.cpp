#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/region_growing.h>

#include <iostream>
#include <vector>

#include "geometry.h"
#include "pcl_ros/point_cloud.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_listener.h"

// Projection code based on personal work for 6.172
// Clustering based on following tutorial:
// https://pcl.readthedocs.io/en/latest/region_growing_segmentation.html#region-growing-segmentation
class ClusterNode {
 protected:
  ros::NodeHandle nh_;
  ros::Subscriber pcl_sub_;
  ros::Publisher pcl_pub_;
  tf::TransformListener listener_;

  pcl::PCLPointCloud<pcl::PointXYZ> buffer_;
  pcl::IndicesPtr indices;
  pcl::search::Search<pcl::PointXYZ>::Ptr tree_;
  pcl::PointCloud<pcl::Normal>::Ptr normals_;

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  pcl::SACSEgmentation<pcl::PointXYZ> seg;
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  pcl::PointCloud<pcl::PointXYZ>::iterator iter;

 private:
  vector e, viewDirection;
  vector w, u, v;

 public:
  ClusterNode(ros::NodeHandle n) : nh_(n), indices(new std::vector<int>) {
    // calculate basis vectors for projections
    vector up = newVector(0, 0, 1);
    w = scale(1 / qsize(viewDirection), viewDirection);
    u = scale(1 / qsize(qcross(up, w)), qcross(up, w));
    v = qcross(w, u);
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud);
    normal_estimator.setKSearch(50);
    reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
    reg.setCuravtureThreshold(1.0);

    // Initiate subscribers and publishers
    pcl_sub_ = nh_.subscribe("/sensor_suite/raw_cloud", 1,
                             &ClusterNode::pcCallback, this);
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/sensor_suite/clustered_cloud", 1);
    e = newVector(800 * f, 100 * f, 0);
    viewDirection = newVector(-1, 0, 0);

    // Clear point_cloud pointers
    // TODO: Move to initialization List
    buffer_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    normals_.reset(new pcl::PointCloud<pcl::Normal>);
    tree_.reset(new pcl::search::KdTree<pcl::PointXYZ>);
    buffer_->header.frame_id = "world";
  }

  void ClusterNode::pcCallback(
      const sensor_msgs::PointCloud2ConstPtr& pcl_msg) {
    pcl::fromROSMsg(*input, *buffer);
    pcl::removeNaNFromPointCloud(*buffer, *indices);

    reg.setInputCloud(buffer_);
    reg.SetIndices(indices_);

    normal_estimator.compute(*normals);
    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);
    for (int i = 0; i < clusters.size(); i++) {
      for (int j = 0; j < clusters[i].indices.size(); j++) {
        // vector point = {point.x, point.y, point.z};
        // pixel px = ClusterNode::projectPointToImage(
        //     point, 1080,
        //     1920, )  // Image sized based on
        // https://support.stereolabs.com/hc/en-us/articles/360007395634-What-is-the-camera-focal-length-and-field-of-view-
        // TO-DO Checkbbounding box to determine how to assign cluster
      }
    }
    return;
  }

  // Based of this piazza post: https://piazza.com/class/kt40btbwj942gr?cid=528
  vector ClusterNode::projectPointToPlane(vector point) {
    // center image frame
    vector normalVector = qcross(u, v);

    float scalar =
        qdot(normalVector, e) / qdot(qsubtract(point, e), normalVector);
    vector projectedPoint = qsubtract(e, scale(scalar, qsubtract(point, e)));
    return projectedPoint;
  }
  pixel ClusterNode::projectPointToImage(vector point, int image_height,
                                         int image_width) {
    vector projection = projectPointToPlane(point);
    double index_i = -(double)projection.y + (double)(width / 2.0);
    double inde_j = (double)projection.z + (double)(height / 2.0f);
    if (index_i < 0) {
      index_i = -1;
    } else if (index_i > width) {
      index_i = width + 1;
    }
    if (index_j < 0) {
      index_j = -1;
    } else if (index_j > height) {
      index_j = height + 1;
    }
    Pixel projection_pixel;
    projectionPixel.u = index_i;
    projectionPixel.v = index_j;
    return projectionPixel
  }
}