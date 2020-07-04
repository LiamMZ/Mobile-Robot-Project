#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <ros/ros.h>

using namespace cv;
using namespace dnn;
class ObjectDetector{
public:
    ObjectDetector(ros::NodeHandle nh, std::string model_file,
                   std::string config_file, std::string classes_file, bool show_image);
    void run(Mat &image);
private:
    void preprocess(const Mat& frame, Size inpSize,
                       const Scalar& mean, bool swapRB);
    void postprocess(Mat& frame, const std::vector<Mat>& outs);
    void drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame);

    ros::NodeHandle nh_;
    std::vector<std::string> classes_;
    cv::dnn::Net net_;
    float scale_;
    float confThreshold_;
    std::vector<String> outNames_;
    bool show_image_;

};