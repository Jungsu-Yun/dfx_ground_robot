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

    ORBConfig* config;

public:
    Frame(cv::Mat color_image, cv::Mat depth_image, ORBConfig* config);
    void compute_feature();

    cv::Mat get_color_image();
    cv::Mat get_gray_image();
    cv::Mat get_orb_descriptor();
    std::vector<cv::KeyPoint> get_keypoint();
    cv::Mat get_depth_image();
};


#endif //ORB_FEATURE_CONTROLLER_FRAME_H
