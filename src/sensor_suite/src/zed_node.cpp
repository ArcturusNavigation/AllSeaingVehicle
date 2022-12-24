#include <sl/Camera.hpp>

#include "cv_bridge/cv_bridge.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "geometry_msgs/Pose.h"

using namespace std;
using namespace sl;

cv::Mat slMat2cvMat(Mat& input);
geometry_msgs::Pose slPose2rosPose(Pose& input);

class ZedNode{
    protected:
        ros::NodeHandle nh_;
        ros::Publisher img_pub_;
        ros::Publisher pose_pub_;
        Camera zed; // ZED camera object
        InitParameters init_params; // ZED parameters
        PositionalTrackingParameters positional_tracking_param; // Camera tracking parameters
    private:
        sl::Mat img_;
    public:
        ZedNode(ros::NodeHandle n) : nh_(n) {
            img_pub_ = nh_.advertise<sensor_msgs::Image>("/zed2i/zed_node/rgb/image_rect_color", 1);
            pose_pub_ = nh_.advertise<sensor_msgs::Image>("/zed2i/zed_node/pose",1);
            init_params.sdk_verbose = true;
            init_params.camera_resolution= sl::RESOLUTION::HD720;
            init_params.depth_mode = sl::DEPTH_MODE::NONE; // no depth computation required here
            init_params.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP; // ROS's standard coordinate system is right handed z-up
            init_params.coordinate_units = UNIT::METER; // By default, coordinate values are in millimeters
            positional_tracking_param.enable_area_memory = true;
            sl::Transform initial_position;
            // Set the initial positon of the Camera Frame at 1m80 above the World Frame
            initial_position.setTranslation(sl::Translation(0,0,1)); //CONFIG: Set to camera's initial position
            positional_tracking_param.initial_world_transform = initial_position;
            auto returned_state = zed.open(init_params);
            if (returned_state != ERROR_CODE::SUCCESS) {
                return;
            }
            returned_state = zed.enablePositionalTracking(positional_tracking_param);
            if (returned_state != ERROR_CODE::SUCCESS) {
                // print("Enabling positionnal tracking failed: ", returned_state);
                zed.close();
                return;
            }
        }
        
        int run(int rate){
            ros::Rate loop_rate(rate);
            Pose camera_path;
            POSITIONAL_TRACKING_STATE tracking_state;
            while(ros::ok()){
                auto returned_state = zed.grab();
                if (returned_state == ERROR_CODE::SUCCESS) {
                    // Retrieve left image
                    zed.retrieveImage(img_, VIEW::LEFT);
                    /* 
                    Retrieve camera pose in world frame
                    Note: To get camera position in real world space, use REFERENCE_FRAME::WORLD, otherwise use 
                    REFERENCE_FRAME::CAMERA to get the change in pose relative to the last position (odometry).                  
                    */
                    tracking_state = zed.getPosition(camera_path, REFERENCE_FRAME::WORLD);
                    // Convert sl::Mat to cv::Mat (share buffer)
                    cv::Mat cvImage = slMat2cvMat(img_);
                    //Convert sl::Pose to ros::geometry_msgs/Pose 
                    geometry_msgs::Pose cameraPose = slPose2rosPose(camera_path);
                    // Publish to topics
                    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cvImage).toImageMsg();
                    img_pub_.publish(msg);
                    pose_pub_.publish(cameraPose);
                }
                else{
                    break;
                }
            ros::spinOnce();
            loop_rate.sleep();
            }
            zed.disablePositionalTracking();
            zed.close();
            return EXIT_SUCCESS;
        }

        ~ZedNode(){
            zed.disablePositionalTracking();
            zed.close();
        }

};
int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle nh;
    ZedNode zed_node(nh);
    zed_node.run(30);

}

// Mapping between MAT_TYPE and CV_TYPE
int getOCVtype(sl::MAT_TYPE type) {
    int cv_type = -1;
    switch (type) {
        case MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
        case MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
        case MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
        case MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
        default: break;
    }
    return cv_type;
}

/**
* Conversion function between sl::Mat and cv::Mat
**/
cv::Mat slMat2cvMat(Mat& input) {
    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), getOCVtype(input.getDataType()), input.getPtr<sl::uchar1>(MEM::CPU), input.getStepBytes(sl::MEM::CPU));
}
/**
* Conversion function between sl::Pose and ros geometry_msgs::Pose
**/
geometry_msgs::Pose slPose2rosPose(Pose& input){
    geometry_msgs::Pose output;
    output.position.x = input.getTranslation().tx;
    output.position.y = input.getTranslation().ty;
    output.position.z = input.getTranslation().tz;
    output.orientation.x = input.getOrientation().ox;
    output.orientation.y = input.getOrientation().oy;
    output.orientation.z = input.getOrientation().oz;
    output.orientation.w = input.getOrientation().ow;
    return output;
}