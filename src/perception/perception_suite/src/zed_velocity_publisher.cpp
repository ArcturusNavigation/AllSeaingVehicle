#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <string>

class ZedVelocityPublisher {
    private:
        ros::Publisher pubVel;
        ros::Subscriber subPose;

        ros::Time prevTime;
        geometry_msgs::Twist prevPose;

    public:
        ZedVelocityPublisher(ros::NodeHandle *nh);
        void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
};

ZedVelocityPublisher::ZedVelocityPublisher(ros::NodeHandle *nh) {
    pubVel = nh->advertise<geometry_msgs::Twist>("/perception_suite/zed_velocity", 1, this);
    subPose = nh->subscribe("/zed2i/zed_node/pose", 10, &ZedVelocityPublisher::poseCallback, this);
    prevTime = ros::Time();
    prevPose = geometry_msgs::Twist();
}

void ZedVelocityPublisher::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {

    // Camera position in map frame
    ros::Time currTime = msg->header.stamp;
    double tx = msg->pose.position.x;
    double ty = msg->pose.position.y;
    double tz = msg->pose.position.z;

    // Orientation quaternion
    tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);

    // 3x3 Rotation matrix from quaternion
    tf2::Matrix3x3 m(q);

    // Roll Pitch and Yaw from rotation matrix
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Calculate velocity
    if (currTime > prevTime) {
        double dt = (currTime - prevTime).toSec();

        geometry_msgs::Twist vel;
        vel.linear.x = (tx - prevPose.linear.x) / dt;
        vel.linear.y = (ty - prevPose.linear.y) / dt;
        vel.linear.z = (tz - prevPose.linear.z) / dt;
        vel.angular.x = (roll - prevPose.angular.x) / dt;
        vel.angular.y = (pitch - prevPose.angular.y) / dt;
        vel.angular.z = (yaw - prevPose.angular.z) / dt;

        // Publish the velocity
        pubVel.publish(vel);

        // Output the measure
        ROS_INFO("Pose: X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f",
                tx, ty, tz,
                roll, pitch, yaw);

        ROS_INFO("Velocity: X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f", 
                vel.linear.x,  vel.linear.y,  vel.linear.z,
                vel.angular.x, vel.angular.y, vel.angular.z);
    }

    // Update previous values
    prevTime = currTime;
    prevPose.linear.x = tx;
    prevPose.linear.y = ty;
    prevPose.linear.z = tz;
    prevPose.angular.x = roll;
    prevPose.angular.y = pitch;
    prevPose.angular.z = yaw;
        
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "zed_velocity_publisher");
    ros::NodeHandle nh;
    ZedVelocityPublisher zedVelPub = ZedVelocityPublisher(&nh);
    ros::spin();
    return 0;

}