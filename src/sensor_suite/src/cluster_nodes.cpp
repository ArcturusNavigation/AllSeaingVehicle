#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/region_growing.h>
#include <sensor_suite/Object.h>

#include <iostream>
#include <vector>

#include "geometry.h"
#include "pcl_ros/point_cloud.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_listener.h"
#include "vision_msgs/BoundingBox2DArray.msg"

// Projection code based on personal work for 6.172
// Clustering based on following tutorial:
// https://pcl.readthedocs.io/en/latest/region_growing_segmentation.html#region-growing-segmentation
// How to sync messages: https://pkok.github.io/2020/08/02/
// Extract indices:
// https://pointclouds.org/documentation/tutorials/extract_indices.html

class ClusterNode {
 protected:
  ros::NodeHandle nh_;
  message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZ>> pcl_sub_;
  message_filters::Subscriber<vision_msgs::BoundingBox2DArray> bbox_sub_;
  ros::Publisher pcl_pub_;
  ros::Publisher object_pub_;
  tf::TransformListener listener_;

  pcl::PointCloud<pcl::PointXYZ> buffer_;
  pcl::IndicesPtr indices;
  pcl::search::Search<pcl::PointXYZ>::Ptr tree_;
  pcl::PointCloud<pcl::Normal>::Ptr normals_;

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  pcl::SACSEgmentation<pcl::PointXYZ> seg;
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  pcl::PointCloud<pcl::PointXYZ>::iterator iter;

 private:
  vector e, viewDirection;
  vector w, u, v;

 public:
  ClusterNode(ros::NodeHandle n)
      : nh_(n),
        indices(new std::vector<int>),
        pcl_sub_(nh_, "/sensor_suite/raw_cloud", 1),
        bbox_sub_(nh_, "/sensor_suite/bounding_boxes", 1),
  {
    // calculate basis vectors for projections
    vector up = newVector(0, 0, 1);
    w = scale(1 / qsize(viewDirection), viewDirection);
    u = scale(1 / qsize(qcross(up, w)), qcross(up, w));
    v = qcross(w, u);
    e = newVector(800 * f, 100 * f, 0);
    viewDirection = newVector(-1, 0, 0);
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud);
    normal_estimator.setKSearch(50);
    reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
    reg.setCuravtureThreshold(1.0);

    // Initiate subscribers and publishers
    using sync_pol = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::PointCloud2, vision_msgs::BoundingBox2DArray>;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), pcl_sub_,
                                                 bbox_sub_);

    sync.registerCallback(&ClusterNode::pcCallback);
    object_pub_ =
        nh_.advertise<sensor_suite::Object>("/sensor_suite/object", 1);

    // Clear point_cloud pointers
    // TODO: Move to initialization List
    buffer_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    normals_.reset(new pcl::PointCloud<pcl::Normal>);
    tree_.reset(new pcl::search::KdTree<pcl::PointXYZ>);
    buffer_->header.frame_id = "world";
  }

  void ClusterNode::pcCallback(const sensor_msgs::PointCloud2ConstPtr& pcl_msg,
                               const BoundingBox2DArrayConstPtr& bbox_msg) {
    pcl::fromROSMsg(*pcl_msg, *buffer);
    pcl::removeNaNFromPointCloud(*buffer, *indices);

    reg.setInputCloud(buffer_);
    extract.setInputCloud(buffer_);
    reg.SetIndices(indices_);

    normal_estimator.compute(*normals);
    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);

    // Loop through every point in each cluster and project to bounding box
    int min_count = 0;  // minimum points in cluster labelled as one object to
                        // avoid nosie TODO: Use
    for (auto cluster : clusters) {
      extract.setIndices(cluster);
      extract.filter(*cloud);
      pcl::CentroidPoint<pcl::PointXYZ> centroid;
      pcl::PointXYZ centroid_point;
      for (int i = 0; i < cloud->points.size(); i++) {
        vector p = cloud->points[i];
        centroid.add(cloud->points[i]);
        pixel px = ClusterNode::projectPointToImage(
            point, 1080,
            1920, )  // Image sized based on
                     // https://support.stereolabs.com/hc/en-us/articles/360007395634-What-is-the-camera-focal-length-and-field-of-view-
            // TO-DO Checkbbounding box to determine how to assign cluster
            int label = ClusterNode::getRegion(px, bbox_msg);
        if (label != 0) {
          centroid.get(centroid_point);
          object_pub_.publish(sensor_suite::Object(centroid_point, label,
                                                   cloud->points.size()));
          break;
        }
      }
    }
    return;
  }
  // TODO: Finish this function

  // Loops through bounding boxes and returns the label of the bounding box
  int ClusterNode::getRegion(pixel px,
                             const BoundingBox2DArrayConstPtr& bbox_msg) {
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
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "cluster_node");
  ros::NodeHandle n;
  ClusterNode cluster_node(n);
  ros::spin();
  return 0;
}