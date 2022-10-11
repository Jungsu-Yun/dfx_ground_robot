#ifndef ORB_FEATURE_CONTROLLER_TRACKER_H
#define ORB_FEATURE_CONTROLLER_TRACKER_H

#include "Frame.h"
#include "ORBConfig.h"
#include <opencv2/opencv.hpp>

class Tracker
{
private:
    Frame* init_frame;
    Frame* previous_frame;
    ORBConfig* config;
    cv::Ptr<cv::DescriptorMatcher> ORBMather;

    std::vector<cv::DMatch> matches;
    std::vector<cv::Point2f> ORBPoints;
    std::vector<cv::Point2f> GoodPoints;

    cv::Mat overlayFrame;
    cv::Mat mask;

public:
    Tracker(Frame* init_frame, Frame* previous_frame, ORBConfig* config);
    Tracker(ORBConfig* config);
    bool ORBMatching();
    bool isDistanceLimit(cv::DMatch match);
    bool OpticalFlow(Frame* now_frame);
    void setInitFrame(Frame* init_frame);
    void setPreviousFrame(Frame* previous_frame);
};


#endif //ORB_FEATURE_CONTROLLER_TRACKER_H
