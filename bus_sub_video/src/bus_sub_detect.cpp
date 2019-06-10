#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
     {
       cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
       cv::waitKey(30);
     }
     catch (cv_bridge::Exception& e)
     {
       ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
     }
}
void imageCallback1(const sensor_msgs::ImageConstPtr& msg)
{
    try
     {
       cv::imshow("view1", cv_bridge::toCvShare(msg, "bgr8")->image);
       cv::waitKey(30);
     }
     catch (cv_bridge::Exception& e)
     {
       ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
     }
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "bus_sub");
   ros::NodeHandle nh;

   cv::namedWindow("view");
   cv::namedWindow("view1");
   cv::startWindowThread();
   image_transport::ImageTransport it(nh);
   image_transport::Subscriber sub = it.subscribe("camera/detect", 1, imageCallback);
   image_transport::Subscriber sub1= it.subscribe("camera/detect1", 1, imageCallback1);
   ros::spin();
   cv::destroyWindow("view");
   cv::destroyWindow("view1");
}
