#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

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
        bool found = findChessboardCorners(*iter, cv::Size(7,7), pointBuf, cv::CV_CALIB_CB_ADAPTIVE_THRESH | cv::CV_CALIB_CB_NORMALIZE_IMAGE);

        if (found)
        {
            allFoundCorners.push_back(pointBuf);
        }

        if(showResults)
        {
           drawChessboardCorners(*iter, cv::Size(7,7), pointBuf, found);
           cv::imshow("Looking For Corners", *iter);
           cv::waitKey(0); 
        }
    }
}

class ImageGrabber
{
public:
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

int main(int argv, char** argc)
{
    cv::Mat frame;
    cv::Mat drawToFrame;
    ros::NodeHandle nodeHandler;
    ImageGrabber igb();

    image_transport::ImageTransport it_(nodeHandler);
    image_transport::Subscriber sub = it_.subscribe("/phone1/camera/image", 1, &ImageGrabber::GrabImage,&igb);
    cv::Mat cameraMatrix = cv::Mat::eye(3,3, CV_64F);

    std::vector<cv::Mat> savedImages;

    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

    return 0;

}