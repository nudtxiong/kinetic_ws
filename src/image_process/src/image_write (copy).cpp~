#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <std_msgs/Header.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>

using namespace std;

void imageCallback(const sensor_msgs::ImageConstPtr& _image_msg)
{

    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr=cv_bridge::toCvShare(_image_msg,sensor_msgs::image_encodings::BGR8);

    static int count = 0;
    char * environment;
    int agent;
    if((environment = getenv("AGENT"))==NULL)
        ROS_ERROR("this agent number is not read by robot");
    agent = atoi(environment);
    std::stringstream ss;
    ss<<agent;

    std::stringstream ss_img_label;
    ss_img_label << count;
    std::string calibration_path="/home/nubot"+ss.str()+"/workspace/src/image_process/images/"+ss_img_label.str()+".jpg";
    cv::imwrite(calibration_path,cv_ptr->image);
    count++;

    static ros::Time publish_time=ros::Time::now();
    ros::Duration duration  = ros::Time::now()-publish_time;
    publish_time=ros::Time::now();
    ROS_INFO("the duration is %f ms",duration.toSec()*1000);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "image_write");
    ros::NodeHandle n;
    image_transport::ImageTransport it_(n);
    image_transport::Subscriber img_sub_;
    img_sub_= it_.subscribe("/camera/image_raw", 500, &imageCallback);
    ros::spin();
    return 0;
 }
