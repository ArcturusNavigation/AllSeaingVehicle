#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "laser_geometry/laser_geometry.h"
#include "tf/transform_listener.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include <string.h>

class LaserScanProcessorNode{
protected:
    ros::NodeHandle nh_; // Node access point
    laser_geometry::LaserProjection projector_;
    tf::TransformListener tf_; // Not in use for now but will transfrom from lidar frame to robot frame
    sensor_msgs::LaserScan msg_; 
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
    ros::Publisher pc_pub_; // Point cloud Publisher

public:
    LaserScanProcessorNode::LaserScanProcessorNode(std::string publish_topic): 
    laser_sub_(n_, 'commercial/raw_scan', 10),
    laser_notifier_(laser_sub_, listener_, "base_link", 10)
    {
        laser_notifier_.registerCallback(boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
        laser_notifier_.setTolerance(ros::Duration(0.01)); // TODO: Explain
        pc_pub = n_.advertise(sensor_msgs::PointCloud>("/sensor_suite/raw_cloud", 1);

    }
    LaserScanProcessorNode::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in){
        sensor_msgs::PointCloud cloud;
        // It is best practice to still wrap transformation in try catch statement even with messagefilter
        try{
        projector.transformLaserScanToPointCloud("base_link",*scan_in,cloud, listener);
        }catch(tf::TransformException& e){
            std::cout << e.what();
            return;
        }
        // TODO: Combine with segmentation map to get objects

        pc_pub_.publish(cloud);
    }
    
}

int main(int argc, char **argv){
    ros::init(argc, argv, "laser_scan_processor_node")
    LaserScanProcessorNode t; 
    ros::spin();

    return 0;
}