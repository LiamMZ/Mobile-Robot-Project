#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <ros/ros.h>

namespace mobile_robot{
class ObjectDetector{
public:
    ObjectDetector(ros::nodeHandle nh, std::string model_file,
                   std::string config_file, std::string classes_file);
    void run();
private:
    inline void preprocess(const Mat& frame, Net& net, Size inpSize, float scale,
                       const Scalar& mean, bool swapRB);
    void postprocess(Mat& frame, const std::vector<Mat>& outs, Net& net, int backend);
    void drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame);

    ros::NodeHandle nh_;
    std::vector<std::string> classes_;
    cv::dnn::Net net_;

};
}