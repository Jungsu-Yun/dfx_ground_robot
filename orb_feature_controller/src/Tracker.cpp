#include "Tracker.h"

Tracker::Tracker(Frame *init_frame, Frame *previous_frame, ORBConfig *config)
{
    this->previous_frame = previous_frame;
    this->init_frame = init_frame;
    this->config = config;

    this->ORBMather = cv::BFMatcher::create(config->normType);

    ORBMatching();
}

Tracker::Tracker(ORBConfig *config)
{
    this->config = config;
    this->ORBMather = cv::BFMatcher::create(config->normType);
}

bool Tracker::isDistanceLimit(cv::DMatch match)
{
    if(match.distance < this->config->distanceLimit)
        return true;
    return false;
}

bool Tracker::ORBMatching()
{
    this->ORBMather->match(init_frame->get_orb_descriptor(), previous_frame->get_orb_descriptor(), this->matches);

    if(!this->matches.empty())
    {
        matches.erase(std::remove_if(this->matches.begin(), this->matches.end(), isDistanceLimit), this->matches.end());

        std::sort(this->matches.begin(), this->matches.end());
        if(this->matches.size() < config->goodMatchNum)
            for(auto iter = matches.begin() ; iter != matches.end() ; iter++)
                this->ORBPoints.push_back(previous_frame->get_keypoint().at(iter->trainIdx).pt);
        else
            for(auto iter = matches.begin() ; iter != matches.begin() + config->goodMatchNum ; iter++)
                this->ORBPoints.push_back(previous_frame->get_keypoint().at(iter->trainIdx).pt);

        return true;
    }
    return false;
}

bool Tracker::OpticalFlow(Frame *now_frame)
{
    cv::Mat target_gray = now_frame->get_gray_image();
    this->mask = cv::Mat::zeros(now_frame->get_color_image().size(), now_frame->get_color_image().type());

    std::vector<cv::Point2f> tracked_point;
    std::vector<uchar> status;
    std::vector<float> error;

    try
    {
        cv::calcOpticalFlowPyrLK(this->previous_frame->get_gray_image(), target_gray, this->ORBPoints, tracked_point, status, error);
        for(int i = 0 ; i < this->ORBPoints.size() ; i++)
        {
            if(status[i] == 1 && error[i] < config->error_limit)
                this->GoodPoints.push_back(tracked_point[i]);

            cv::line(this->mask, this->ORBPoints[i], tracked_point[i], cv::Scalar(255,255,255), 1);
            cv::circle(this->mask, this->ORBPoints[i], 2, cv::Scalar(255, 0, 0), -1);
            cv::circle(this->mask, tracked_point[i], 2, cv::Scalar(255, 0, 0), -1);
        }

        cv::Mat tmp;
        cv::addWeighted(now_frame->get_color_image(), 0.7, this->previous_frame->get_color_image(), 0.3, 0.0, tmp);
        cv::add(tmp, mask, this->overlayFrame);
        return true;
    }
    catch(cv::Exception &e)
    {
        return false;
    }
}

void Tracker::setInitFrame(Frame *init_frame)
{
    this->init_frame = init_frame;
}

void Tracker::setPreviousFrame(Frame *previous_frame)
{
    this->previous_frame = previous_frame;
    ORBMatching();
}