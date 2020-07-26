
#include "object_detector.h"
#include <fstream>
#include <sstream>

#include <opencv2/dnn.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <algorithm>
#include <mutex>
#include <thread>
#include <queue>

using namespace cv;
using namespace dnn;

ObjectDetector::ObjectDetector(ros::NodeHandle nh, std::string model_file,
                   std::string config_file, std::string classes_file, bool show_image) : nh_(nh)
{
    std::ifstream ifs(classes_file.c_str());
    if (!ifs.is_open())
        CV_Error(Error::StsError, "File " + classes_file + " not found");
    std::string line;
    while (std::getline(ifs, line))
    {
        classes_.push_back(line);
    }

    net_ = readNet(model_file, config_file);
    scale_ = 0.00392;
    confThreshold_ = 0.5;
    outNames_ = net_.getUnconnectedOutLayersNames();
    show_image_ = show_image;

}

void ObjectDetector::preprocess(const Mat& frame, Size inpSize,
                       const Scalar& mean, bool swapRB)
{
    static Mat blob;
    // Create a 4D blob from a frame.
    if (inpSize.width <= 0) inpSize.width = frame.cols;
    if (inpSize.height <= 0) inpSize.height = frame.rows;
    blobFromImage(frame, blob, 1, cv::Size(416, 416));

    // Run a model.
    net_.setInput(blob, "", scale_, mean);
    if (net_.getLayer(0)->outputNameToIndex("im_info") != -1)  // Faster-RCNN or R-FCN
    {
        resize(frame, frame, inpSize);
        Mat imInfo = (Mat_<float>(1, 3) << inpSize.height, inpSize.width, 1.6f);
        net_.setInput(imInfo, "im_info");
    }
}

void ObjectDetector::postprocess(Mat& frame, const std::vector<Mat>& outs)
{
    static std::vector<int> outLayers = net_.getUnconnectedOutLayers();
    static std::string outLayerType = net_.getLayer(outLayers[0])->type;

    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<Rect> boxes;
    // if (outLayerType == "DetectionOutput")
    // {
    //     // Network produces output blob with a shape 1x1xNx7 where N is a number of
    //     // detections and an every detection is a vector of values
    //     // [batchId, classId, confidence, left, top, right, bottom]
    //     CV_Assert(outs.size() > 0);
    //     for (size_t k = 0; k < outs.size(); k++)
    //     {
    //         float* data = (float*)outs[k].data;
    //         for (size_t i = 0; i < outs[k].total(); i += 7)
    //         {
    //             float confidence = data[i + 2];
    //             if (confidence > confThreshold_)
    //             {
    //                 int left   = (int)data[i + 3];
    //                 int top    = (int)data[i + 4];
    //                 int right  = (int)data[i + 5];
    //                 int bottom = (int)data[i + 6];
    //                 int width  = right - left + 1;
    //                 int height = bottom - top + 1;
    //                 if (width <= 2 || height <= 2)
    //                 {
    //                     left   = (int)(data[i + 3] * frame.cols);
    //                     top    = (int)(data[i + 4] * frame.rows);
    //                     right  = (int)(data[i + 5] * frame.cols);
    //                     bottom = (int)(data[i + 6] * frame.rows);
    //                     width  = right - left + 1;
    //                     height = bottom - top + 1;
    //                 }
    //                 classIds.push_back((int)(data[i + 1]) - 1);  // Skip 0th background class id.
    //                 boxes.push_back(Rect(left, top, width, height));
    //                 confidences.push_back(confidence);
    //             }
    //         }
    //     }
    // }
    if (outLayerType == "Region")
    {
        for (size_t i = 0; i < outs.size(); ++i)
        {
            // Network produces output blob with a shape NxC where N is a number of
            // detected objects and C is a number of classes + 4 where the first 4
            // numbers are [center_x, center_y, width, height]
            float* data = (float*)outs[i].data;
            for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
            {
                Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
                Point classIdPoint;
                double confidence;
                minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
                if (confidence > confThreshold_ && std::find(classIds.begin(),classIds.end(), classIdPoint.x)==classIds.end())
                {
                    int centerX = (int)(data[0] * frame.cols);
                    int centerY = (int)(data[1] * frame.rows);
                    int width = (int)(data[2] * frame.cols);
                    int height = (int)(data[3] * frame.rows);
                    int left = centerX - width / 2;
                    int top = centerY - height / 2;

                    classIds.push_back(classIdPoint.x);
                    confidences.push_back((float)confidence);
                    boxes.push_back(Rect(left, top, width, height));
                }
            }
        }
    }
    else
        CV_Error(Error::StsNotImplemented, "Unknown output layer type: " + outLayerType);

    // NMS is used inside Region layer only on DNN_BACKEND_OPENCV for another backends we need NMS in sample
    // or NMS is required if number of outputs > 1
    // if (outLayers.size() > 1 || (outLayerType == "Region" && backend != DNN_BACKEND_OPENCV))
    // {
    //     std::map<int, std::vector<size_t> > class2indices;
    //     for (size_t i = 0; i < classIds.size(); i++)
    //     {
    //         if (confidences[i] >= confThreshold)
    //         {
    //             class2indices[classIds[i]].push_back(i);
    //         }
    //     }
    //     std::vector<Rect> nmsBoxes;
    //     std::vector<float> nmsConfidences;
    //     std::vector<int> nmsClassIds;
    //     for (std::map<int, std::vector<size_t> >::iterator it = class2indices.begin(); it != class2indices.end(); ++it)
    //     {
    //         std::vector<Rect> localBoxes;
    //         std::vector<float> localConfidences;
    //         std::vector<size_t> classIndices = it->second;
    //         for (size_t i = 0; i < classIndices.size(); i++)
    //         {
    //             localBoxes.push_back(boxes[classIndices[i]]);
    //             localConfidences.push_back(confidences[classIndices[i]]);
    //         }
    //         std::vector<int> nmsIndices;
    //         NMSBoxes(localBoxes, localConfidences, confThreshold, nmsThreshold, nmsIndices);
    //         for (size_t i = 0; i < nmsIndices.size(); i++)
    //         {
    //             size_t idx = nmsIndices[i];
    //             nmsBoxes.push_back(localBoxes[idx]);
    //             nmsConfidences.push_back(localConfidences[idx]);
    //             nmsClassIds.push_back(it->first);
    //         }
    //     }
    //     boxes = nmsBoxes;
    //     classIds = nmsClassIds;
    //     confidences = nmsConfidences;
    // }

    for (size_t idx = 0; idx < boxes.size(); ++idx)
    {
        Rect box = boxes[idx];
        drawPred(classIds[idx], confidences[idx], box.x, box.y,
                 box.x + box.width, box.y + box.height, frame);
    }
}

void ObjectDetector::drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame)
{
    rectangle(frame, Point(left, top), Point(right, bottom), Scalar(0, 255, 0));

    std::string label = format("%.2f", conf);
    if (!classes_.empty())
    {
        CV_Assert(classId < (int)classes_.size());
        label = classes_[classId] + ": " + label;
    }

    int baseLine;
    Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

    top = max(top, labelSize.height);
    rectangle(frame, Point(left, top - labelSize.height),
              Point(left + labelSize.width, top + baseLine), Scalar::all(255), FILLED);
    putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.5, Scalar());
}


void ObjectDetector::run(Mat &image)
{
    int inpWidth = image.cols;
    int inpHeight = image.rows;
    preprocess(image, Size(image.cols, image.rows), cv::mean(image), false);
    std::vector<Mat> outs;
    net_.forward(outs, outNames_);
    postprocess(image, outs);
    if (show_image_){
        namedWindow( "Display window" );// Create a window for display.
        imshow( "Display window", image );                   // Show our image inside it.

        waitKey(0);
    }
}
