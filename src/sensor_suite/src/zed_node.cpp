#include <sl/Camera.hpp>

#include "cv_bridge/cv_bridge.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

using namespace std;
using namespace sl;

cv::Mat slMat2cvMat(Mat& input);

class ZedNode{
    protected:
        ros::NodeHandle nh_;
        ros::Publisher img_pub_;
        Camera zed; // ZED camera object
        InitParameters init_params; // ZED parameters
    private:
        sl::Mat img_;
    public:
        ZedNode(ros::NodeHandle n) : nh_(n) {
            img_pub_ = nh_.advertise<sensor_msgs::Image>("/zed2i/zed_node/rgb/image_rect_color", 1);
            init_params.sdk_verbose = true;
            init_params.camera_resolution= sl::RESOLUTION::HD720;
            init_params.depth_mode = sl::DEPTH_MODE::NONE; // no depth computation required here
            auto returned_state = zed.open(init_params);
            if (returned_state != ERROR_CODE::SUCCESS) {
                return;
            }
        }
        
        int run(int rate){
            ros::Rate loop_rate(rate);
            while(ros::ok()){
                auto returned_state = zed.grab();
                if (returned_state == ERROR_CODE::SUCCESS) {
                    // Retrieve left image
                    zed.retrieveImage(img_, VIEW::LEFT);
                    // Convert sl::Mat to cv::Mat (share buffer)
                    cv::Mat cvImage = slMat2cvMat(img_);
                    // Publish image
                    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cvImage).toImageMsg();
                    img_pub_.publish(msg);
                }
                else{
                    break;
                }
            ros::spinOnce();
            loop_rate.sleep();
            }
            zed.close();
            return EXIT_SUCCESS;
        }

        ~ZedNode(){
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