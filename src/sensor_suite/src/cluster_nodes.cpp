#include "geometry.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
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
  sensor_msgs::PointCloud2::Ptr buffer_;

 private:
  vector e, viewDirection;
  vector w, u, v;

 public:
  ClusterNode(ros::NodeHandle n) : nh_(n), {
    pcl_sub_ = nh_.subscribe("/sensor_suite/raw_cloud", 1,
                             &ClusterNode::pcCallback, this);
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/sensor_suite/clustered_cloud", 1);
    e = newVector(800 * f, 100 * f, 0);
    viewDirection = newVector(-1, 0, 0);

    // calculate basis vectors
    vector up = newVector(0, 0, 1);
    w = scale(1 / qsize(viewDirection), viewDirection);
    u = scale(1 / qsize(qcross(up, w)), qcross(up, w));
    v = qcross(w, u);
    buffer_.reset(new sensor_msgs::PointCloud2);
    buffer_->header.frame_id = "world";
  }

  void ClusterNode::pcCallback(
      const sensor_msgs::PointCloud2ConstPtr& pcl_msg) {
    pcl::PCLPointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*input, cloud);
    pcl::PCLPointCloud2 clustered_cloud;

    pcl::SACSEgmentation<pcl::PointXYZ> seg;

    for (auto& point : pcl_msg->points) {
      vector point = {point.x, point.y, point.z};
      pixel px = ClusterNode::projectPointToImage(
          point, 1080,
          1920, )  // Image sized based on
                   // https://support.stereolabs.com/hc/en-us/articles/360007395634-What-is-the-camera-focal-length-and-field-of-view-
    }
    return;
  }

  // Based of this piazza post: https://piazza.com/class/kt40btbwj942gr?cid=528
  vector ClusterNode::projectPointToPlane(vector point) {
    // center image frame
    vector normalVector = qcross(u, v);  // Todo: Move to main render function

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