#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

cv::Mat frame;

bool flag = false;

void msgCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){

    if(!frame.empty()){
        for(int i=0;i<msg->bounding_boxes.size();i++){
            if(msg->bounding_boxes[i].Class != "person") continue;
            cv::rectangle(frame,cv::Point(msg->bounding_boxes[i].xmin,msg->bounding_boxes[i].ymin),cv::Point(msg->bounding_boxes[i].xmax,msg->bounding_boxes[i].ymax),cv::Scalar(255,0,255),3,8);
        }

        cv::imshow("Yolo V3", frame);
        cv::waitKey(30);
    }


}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "bus_pub_video");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/camera/rgb/image_raw", 1);

    ros::Subscriber yolo_boundingBox = nh.subscribe("/darknet_ros/bounding_boxes",100,msgCallback);

    std::istringstream video_sourceCmd(argv[1]);
    int video_source;
    // Check if it is indeed a number
    if(!(video_sourceCmd >> video_source)) return 1;
    cv::VideoCapture cap(video_source);
    // Check if video device can be opened with the given index
    if(!cap.isOpened()) return 1;

    sensor_msgs::ImagePtr msg;

    ros::Rate loop_rate(5);
    cv::namedWindow("Yolo V3");

    while (nh.ok()) {
        cap >> frame;
        // Check if grabbed frame is actually full with some content
        if(!frame.empty()){
             msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
             pub.publish(msg);

             std::printf("send image \n");
             cv::waitKey(1);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    cv::destroyWindow("view");
}
