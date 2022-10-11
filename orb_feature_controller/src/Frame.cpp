#include "Frame.h"

Frame::Frame(cv::Mat color_image, cv::Mat depth_image, ORBConfig config)
{
    this->color_image = color_image;
    this->depth_image = depth_image;
    this->config = config;
    cv::cvtColor(color_image, this->gray_image, cv::COLOR_RGB2GRAY);

    this->orb_feature = cv::ORB::create(this->config.nFeatures);
    this->matcher = cv::BFMatcher::create(this->config.normType);

    compute_feature();
}

void Frame::compute_feature()
{
    this->orb_feature->detectAndCompute(this->gray_image, cv::Mat(), this->keypoints, this->orb_descriptor);
}

void Frame::ORBMathcing(Frame)
{
    
}
