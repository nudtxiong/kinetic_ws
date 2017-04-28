#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <std_msgs/Header.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>


#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "image_process/FileFunctions.h"
#include <std_msgs/Header.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>
using namespace std;
#define AVI_FILE_CONST
std::string m_imagedir="/home/nubot9/uav_images";
void color_selection(std::string  & encodings,cv::Mat image);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_publish");
    ros::NodeHandle n;
    image_transport::ImageTransport it_(n);
    image_transport::CameraPublisher streaming_pub_;
    streaming_pub_ = it_.advertiseCamera("camera/image_raw", 1);
    sensor_msgs::Image img_;
    sensor_msgs::CameraInfo cam_info_;
    int count = 0;
    cv_bridge::CvImage bridge_;
    /*char * environment;
    int agent;
    if((environment = getenv("AGENT"))==NULL)
    {
        ROS_ERROR("this agent number is not read by robot");
        return 0;
    }
    agent = atoi(environment);
    std::stringstream ss;
    ss<<agent;
    std::string calibration_path="/home/nubot"+ss.str()+"/nubot_ws/src/nubot/omni_vision/calib_results/"+ss.str()+"/";*/
#  ifdef AVI_FILE_CONST
    cv::VideoCapture capture(m_imagedir+"/cartracking.avi");
    if(!capture.isOpened())
        cout<<"fail to open!"<<endl;
    long totalFrameNumber = capture.get(CV_CAP_PROP_FRAME_COUNT); //获取帧数
    std::cout<< totalFrameNumber<<std::endl;
    //capture.set( CV_CAP_PROP_POS_FRAMES,0);     //设置起始帧
    int fps = capture.get(CV_CAP_PROP_FPS); //获取帧率
    ros::Rate loop_rate(10);
#else
    std::vector<string> filenames = FileFunctions::Dir(m_imagedir.c_str(), ".jpg", true);
    ros::Rate loop_rate(30);
#endif


    while (ros::ok())
    {
#  ifdef AVI_FILE_CONST
        if(!capture.read(bridge_.image))
        {
            cout<<"读取视频失败"<<endl;
            //ros::shutdown();
        }
        bridge_.header.stamp = ros::Time::now();
#else
        bridge_.image = cv::imread(filenames[count],1);
        bridge_.header.stamp = ros::Time::now();
#endif
        color_selection(bridge_.encoding,bridge_.image);
        bridge_.toImageMsg(img_);
        img_.header.stamp = cam_info_.header.stamp = bridge_.header.stamp;
        streaming_pub_.publish(img_,cam_info_);
        static ros::Time publish_time=ros::Time::now();
        ros::Duration duration  = ros::Time::now()-publish_time;
        publish_time=ros::Time::now();
        ROS_INFO("the duration is %f ms",duration.toSec()*1000);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    std::cout<< count<<std::endl;
    return 0;
}
void color_selection(std::string  & encodings,cv::Mat image)
{
    switch(image.type()) {
    case CV_8SC1:  encodings = sensor_msgs::image_encodings::MONO8; break;
    case CV_8UC1:  encodings = sensor_msgs::image_encodings::MONO8; break;
    case CV_16SC1: encodings = sensor_msgs::image_encodings::MONO16; break;
    case CV_16UC1: encodings = sensor_msgs::image_encodings::MONO16; break;
    case CV_8UC3:  encodings = sensor_msgs::image_encodings::BGR8; break;
        // case CV_8UC3:  encodings = sensor_msgs::image_encodings::RGB8; break;
    case CV_8UC4:  encodings = sensor_msgs::image_encodings::BGRA8; break;
        //  case CV_8UC4:  encodings = sensor_msgs::image_encodings::RGBA8; break;
    }
}



