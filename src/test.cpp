#include "object_detector.h"
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

void callback(const sensor_msgs::CompressedImage image_msg)
{

}


class ImageGrabber
{
public:
    ImageGrabber(){};
    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat image;
};

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    this->image = cv_ptr->image;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "talker");
    ros::NodeHandle nodeHandler;
    ImageGrabber igb = ImageGrabber();

    image_transport::ImageTransport it_(nodeHandler);
    image_transport::Subscriber sub = it_.subscribe("/phone1/camera/image", 1, &ImageGrabber::GrabImage,&igb);
    // cv::Mat image = cv::imread("/home/liam/catkin_ws/src/mobile_robot_project/src/man.jpg");
    // namedWindow( "Display window1" );// Create a window for display.
    // imshow( "Display window1", image );                   // Show our image inside it.

    //     waitKey(0);
    ObjectDetector object_detector = ObjectDetector(nodeHandler, "/home/liam/catkin_ws/src/mobile_robot_project/src/yolov3.weights",
                                                            "/home/liam/catkin_ws/src/mobile_robot_project/src/yolov3.cfg", "/home/liam/catkin_ws/src/mobile_robot_project/src/yolov3.txt", true);
    while(ros::ok()){
        object_detector.run(igb.image);
    }
    

}