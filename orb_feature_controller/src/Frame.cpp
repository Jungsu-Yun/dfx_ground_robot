#include "Frame.h"

Frame::Frame(cv::Mat color_image, cv::Mat depth_image, ORBConfig* config)
{
    this->color_image = color_image;
    this->depth_image = depth_image;
    this->config = config;
    cv::cvtColor(color_image, this->gray_image, cv::COLOR_RGB2GRAY);

    this->orb_feature = cv::ORB::create(this->config->nFeatures);
    compute_feature();
}

void Frame::compute_feature()
{
    this->orb_feature->detectAndCompute(this->gray_image, cv::Mat(), this->keypoints, this->orb_descriptor);
}

cv::Mat Frame::get_color_image()
{
    return this->color_image;
}

cv::Mat Frame::get_gray_image()
{
    return this->gray_image;
}

cv::Mat Frame::get_orb_descriptor()
{
    return this->orb_descriptor;
}

cv::Mat Frame::get_depth_image()
{
    return this->depth_image;
}

std::vector<cv::KeyPoint> Frame::get_keypoint()
{
    return this->keypoints;
}
