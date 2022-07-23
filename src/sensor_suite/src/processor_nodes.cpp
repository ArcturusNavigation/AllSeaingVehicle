#include <string.h>

#include "laser_geometry/laser_geometry.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "tf/transform_listener.h"

class LaserScanProcessorNode {
 protected:
  ros::NodeHandle nh_;  // Node access point
  laser_geometry::LaserProjection projector_;
  tf::TransformListener tf_;  // Not in use for now but will transfrom from
                              // lidar frame to robot frame
  sensor_msgs::LaserScan msg_;
  ros::Subscriber laser_sub_;
  ros::Publisher pc_pub_;  // Point cloud Publisher

 public:
  LaserScanProcessorNode(ros::NodeHandle n) : nh_(n) {
    laser_sub_ = nh_.subscribe("/commercial/raw_scan", 10,
                               &LaserScanProcessorNode::scanCallback, this);
    pc_pub_ = nh_.advertise<sensor_msgs::PointCloud>("/sensor_suite/raw_cloud",
                                                     1, true);
  }
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in) {
    sensor_msgs::PointCloud cloud;
    // It is best practice to still wrap transformation in try catch statement
    // even with messagefilter
    try {
      projector_.transformLaserScanToPointCloud("base_link", *scan_in, cloud,
                                                tf_);
    } catch (tf::TransformException& e) {
      std::cout << e.what();
      return;
    }
    // TODO: Combine with segmentation map to get objects

    pc_pub_.publish(cloud);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "laser_scan_processor_node");
  ros::NodeHandle n;
  LaserScanProcessorNode node(n);
  ros::spin();

  return 0;
}