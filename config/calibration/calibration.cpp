#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/stereo.hpp>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <sstream>
#include <iostream>
#include <fstream>

const float calibrationSquareDImension = .0256; //meters
// const float arucoSquareDimension = 
const cv::Size chessboardDimensions = cv::Size(7,7);

void createKnownBoardPositions(cv::Size boardSize, float squareEdgeLength, std::vector<cv::Point3f>& corners)
{
    for(int i = 0; i< boardSize.height; i++)
    {
        for(int j = 0; j<boardSize.width; j++)
        {
            corners.push_back(cv::Point3f(j*squareEdgeLength, i*squareEdgeLength, 0.0f));
        }
    }
}

void getChessboardCorners(std::vector<cv::Mat> images, std::vector<std::vector<cv::Point2f>> allFoundCorners, bool showResults)
{
    for(std::vector<cv::Mat>::iterator iter = images.begin(); iter!=images.end(); iter++)
    {
        std::vector<cv::Point2f> pointBuf;
        bool found = findChessboardCorners(*iter, cv::Size(7,7), pointBuf, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (found)
        {
            allFoundCorners.push_back(pointBuf);
        }

        if(showResults)
        {
           cv::drawChessboardCorners(*iter, cv::Size(7,7), pointBuf, found);
           cv::imshow("Looking For Corners", *iter);
           cv::waitKey(0); 
        }
    }
}

class ImageGrabber
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
public:
    ImageGrabber(ros::NodeHandle nh) : nh_(nh), it_(nh_)
    {
        image_sub_ = it_.subscribe("/phone1/camera/image", 1, &ImageGrabber::GrabImage, this);
        ROS_INFO_STREAM("IMAGE GRABBER INITIALIZED");
    }
    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat image;
};

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        ROS_ERROR_STREAM("in grab image");
        cv_ptr = cv_bridge::toCvShare(msg);
        this->image = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calibration_node");
    cv::Mat frame;
    cv::Mat drawToFrame;
    ros::NodeHandle nh;
    ImageGrabber igb = ImageGrabber(nh);

    cv::Mat cameraMatrix = cv::Mat::eye(3,3, CV_64F);

    std::vector<cv::Mat> savedImages;

    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

    int framesPerSecond = 20;
    ros::Duration(10).sleep();

    cv::namedWindow("Phone", cv::WINDOW_AUTOSIZE);
    cv::imshow("phone", igb.image);
    cv::waitKey(0);
    ROS_ERROR_STREAM("HERE ");
    while (ros::ok())
    {
        frame = igb.image;
        std::vector<cv::Vec2f> foundPoints;
        bool found = false;

        found = cv::findChessboardCorners(frame, chessboardDimensions, foundPoints, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        frame.copyTo(drawToFrame);
        cv::drawChessboardCorners(drawToFrame, chessboardDimensions, foundPoints, found);
        if(found)
        {
           cv::imshow("Phone", drawToFrame);
        }
        else cv::imshow("phone", frame);
        char character = cv::waitKey(1000 / framesPerSecond);
    }
    
    return 0;

}