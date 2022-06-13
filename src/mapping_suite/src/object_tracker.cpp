#include "math.h"

#include "cv_bridge/cv_bridge.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <sensor_suite/Object.h>
#include "sensor_suite/ObjectArray.h"


class SegmentationNode {
 protected:
  ros::NodeHandle nh_;
  ros::Subscriber object_sub_;
  ros::Publisher object_pub_;
  ros::Publisher box_pub_;
};