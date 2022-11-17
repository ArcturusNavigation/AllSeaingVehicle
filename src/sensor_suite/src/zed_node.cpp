#include <sl/Camera.hpp>

class ZedNode{
    protected:
        ros::NodeHandle nh_;
        ros::Publisher rgb_pub_;
        ros::Publisher depth_pub_;
        Camera zed; // ZED camera object
        InitParameters init_params; // ZED parameters
        bool publish_depth_data;
    private:
        cv::Mat img_;
    public:
        ZedNode(ros::NodeHandle n, bool publish_depth_data) : nh_(n), publish_depth_data_(publish_depth_data) {
            img_pub_ = nh_.advertise<sensor_msgs::Image>("/zed2i/zed_node/rgb/image_rect_color", 1);
            init_parameters.sdk_verbose = true;
            init_parameters.camera_resolution= sl::RESOLUTION::HD720;
            init_parameters.depth_mode = sl::DEPTH_MODE::NONE; // no depth computation required here
            auto returned_state = zed.open(init_parameters);
            if (returned_state != ERROR_CODE::SUCCESS) {
                print("Camera Open", returned_state, "Exit program.");
                return EXIT_FAILURE;
            }
        }
        
        run(int rate){
            ros::Rate loop_rate(rate);
            while(ros::ok()){
                returned_state = zed.grab();
                if (returned_state == ERROR_CODE::SUCCESS) {
                    // Retrieve left image
                    zed.retrieveImage(img_, VIEW::LEFT);
                    // Convert sl::Mat to cv::Mat (share buffer)
                    cv::Mat cvImage = cv::Mat((int) img_.getHeight(), (int) img_.getWidth(), CV_8UC4, img_.getPtr<sl::uchar1>(sl::MEM::CPU));
                    // Publish image
                    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_).toImageMsg();
                    rgb_pub_.publish(msg);
                }
                else{
                    print("Error during capture : ", returned_state);
                    break;
                }
            ros::spinOnce();
            loop_rate.sleep();
            }
        }

        ~ZedNode(){
            zed.close();
            return EXIT_SUCCESS;
        }

};
int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle nh;
    ZedNode zed_node(nh, false);
    zed_node.run(30);

}