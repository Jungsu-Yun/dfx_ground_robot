#ifndef ORB_FEATURE_CONTROLLER_FRAME_H
#define ORB_FEATURE_CONTROLLER_FRAME_H

#include <opencv2/opencv.hpp>

#include "ORBConfig.h"

class Frame
{
private:
    cv::Mat color_image;
    cv::Mat gray_image;
    cv::Mat depth_image;

    cv::Mat orb_descriptor;
    std::vector<cv::KeyPoint> keypoints;
    std::vector<cv::Point2f> points;

    std::vector<cv::DMatch> good_matches;

    cv::Ptr<cv::Feature2D> orb_feature;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    ORBConfig config;

    void compute_feature();
public:
    Frame(cv::Mat color_image, cv::Mat depth_image, ORBConfig config);
    void ORBMathcing(Frame previous_frame);
};


#endif //ORB_FEATURE_CONTROLLER_FRAME_H
