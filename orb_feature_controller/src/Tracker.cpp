#include "Tracker.h"

Tracker::Tracker(Frame *init_frame, Frame *previous_frame, ORBConfig *config)
{
    this->previous_frame = previous_frame;
    this->init_frame = init_frame;
    this->config = config;

    this->ORBMather = cv::BFMatcher::create(config->normType);

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
    this->matches.clear();
    this->matches.shrink_to_fit();
    std::vector<cv::DMatch> matches_temp;
    try{
        this->ORBMather->match(init_frame->get_orb_descriptor(), previous_frame->get_orb_descriptor(), matches_temp);

        if(!matches_temp.empty())
        {
            for(auto iter = matches_temp.begin() ; iter != matches_temp.end() ; iter++)
            {
                if(iter->distance < this->config->distanceLimit)
                {
                    cv::DMatch tmp;
                    tmp.distance = iter->distance;
                    tmp.trainIdx = iter->trainIdx;
                    tmp.imgIdx = iter->imgIdx;
                    tmp.queryIdx = iter->queryIdx;
                    this->matches.push_back(tmp);
                }
            }
            if(this->matches.empty())
                return false;

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
    catch (cv::Exception e)
    {
        std::cout << e.what() << std::endl;
        return false;
    }
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
        if(tracked_point.empty())
            return false;
        for(int i = 0 ; i < tracked_point.size() ; i++)
        {
            if(status[i] == 1 && (error[i] < config->error_limit))
            {
                this->GoodPreviousPoints.push_back(this->ORBPoints[i]);
                this->GoodCurrentPoints.push_back(tracked_point[i]);

                cv::line(this->mask, this->ORBPoints[i], tracked_point[i], cv::Scalar(255,255,255), 1);
                cv::circle(this->mask, this->ORBPoints[i], 2, cv::Scalar(255, 0, 0), -1);
                cv::circle(this->mask, tracked_point[i], 2, cv::Scalar(255, 0, 0), -1);
            }
        }
        std::cout << this->GoodCurrentPoints.size() << std::endl;
        if(this->GoodCurrentPoints.empty() || this->GoodPreviousPoints.empty() || this->GoodCurrentPoints.size() < config->nGoodFollowed)
            return false;

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
}

double Tracker::CalculateRadian(Frame *now_frame)
{
    int result = 0;
    double previous_x, current_x, previous_y, current_y = 0.0;
    double previous_depth = this->previous_frame->get_gray_image().at<unsigned short>((int)this->GoodCurrentPoints.at(0).x, (int)this->GoodCurrentPoints.at(0).y);
    double current_depth = now_frame->get_depth_image().at<unsigned short>((int)this->GoodCurrentPoints.at(0).x, (int)this->GoodCurrentPoints.at(0).y);

    for(int i = 0 ; i < this->GoodCurrentPoints.size() ; i++)
    {
        //포인트가 오른쪽으로 이동 -> -1
        if(((int)this->GoodCurrentPoints.at(i).x - (int)this->GoodPreviousPoints.at(i).x) > 0)
            result -= 1;
        //포인트가 왼쪽으로 이동 -> +1
        else if(((int)this->GoodCurrentPoints.at(i).x - (int)this->GoodPreviousPoints.at(i).x) < 0)
            result += 1;

        //depth 최소값 찾기
//        int depth_data = now_frame->get_depth_image().at<unsigned short>(this->GoodCurrentPoints.at(i).x, this->GoodCurrentPoints.at(i).y) * 0.001;
        int depth_data = now_frame->get_depth_image().at<unsigned short>(this->GoodCurrentPoints.at(i).x, this->GoodCurrentPoints.at(i).y);
        if((depth_data < current_depth) && depth_data != 0)
        {
            previous_x = this->GoodPreviousPoints[i].x;
            current_x = this->GoodCurrentPoints[i].x;
            previous_y = this->GoodPreviousPoints[i].y;
            current_y = this->GoodCurrentPoints[i].y;
//            previous_depth = this->previous_frame->get_gray_image().at<unsigned short>((int)this->GoodPreviousPoints.at(i).x, (int)this->GoodPreviousPoints.at(i).y) * 0.001;
//            current_depth = now_frame->get_depth_image().at<unsigned short>((int)this->GoodCurrentPoints.at(i).x, (int)this->GoodCurrentPoints.at(i).y) * 0.001;
            previous_depth = this->previous_frame->get_gray_image().at<unsigned short>((int)this->GoodPreviousPoints.at(i).x, (int)this->GoodPreviousPoints.at(i).y);
            current_depth = now_frame->get_depth_image().at<unsigned short>((int)this->GoodCurrentPoints.at(i).x, (int)this->GoodCurrentPoints.at(i).y);
        }
    }

    this->GoodCurrentPoints.clear();
    this->GoodPreviousPoints.clear();
    this->GoodCurrentPoints.shrink_to_fit();
    this->GoodPreviousPoints.shrink_to_fit();

    //각도 계산하기
    double radian = ((M_PI / 2.0) - atan2(abs(current_depth-previous_depth), abs(current_x-previous_x)));
    std::cout << previous_depth << "\t" << current_depth << "\t" << current_x << "\t" << previous_x << "\t" << radian << std::endl;
//    double radian = atan2(abs(current_depth-previous_depth), abs(current_x-previous_x));
//    double radian = atan2(abs(current_y - previous_y), abs(current_x - previous_x));
//    std::cout << "Radian : " << radian << "\tDegree : " << (radian * 180) / 3.14159265 << "\tresult : " << result / abs(result) << std::endl;
//    std::cout << "Output : " << radian * (double)(result / abs(result)) << std::endl;

    return radian * (double)(result / abs(result))*100;
}

cv::Mat Tracker::getOverlayFrame()
{
    return this->overlayFrame;
}

bool Tracker::OpticalFlow(Frame* now_frame, double radi)
{
    cv::Mat target_gray = now_frame->get_gray_image();
    this->mask = cv::Mat::zeros(now_frame->get_color_image().size(), now_frame->get_color_image().type());

    std::vector<cv::Point2f> tracked_point;
    std::vector<uchar> status;
    std::vector<float> error;

    int result = 0;
    double previous_x, current_x, previous_y, current_y = 0.0;
    double previous_depth = 0.0;
    double current_depth = 0.0;

    int cnt;

    try
    {
        cv::calcOpticalFlowPyrLK(this->previous_frame->get_gray_image(), target_gray, this->ORBPoints, tracked_point, status, error);
        if(tracked_point.empty())
            return false;
        for(int i = 0 ; i < tracked_point.size() ; i++)
        {
            if(status[i] == 1 && (error[i] < config->error_limit))
            {
                if(tracked_point[i].x - this->ORBPoints[i].x > 0)
                    result -= 1;
                else if(tracked_point[i].x - this->ORBPoints[i].x < 0)
                    result += 1;

                if (cnt == 0)
                {
                    current_x = tracked_point[i].x;
                    current_depth = now_frame->get_depth_image().at<float>((int)tracked_point[i].x, (int)tracked_point[i].y) * 0.001;

                    previous_x = this->ORBPoints[i].x;
                    previous_depth = previous_frame->get_depth_image().at<float>((int)this->ORBPoints[i].x, (int)this->ORBPoints[i].y) * 0.001;
                }
                else if (now_frame->get_depth_image().at<unsigned short>((int)tracked_point[i].x, (int)tracked_point[i].y) < current_depth && now_frame->get_depth_image().at<float>((int)tracked_point[i].x, (int)tracked_point[i].y) != 0 && previous_frame->get_depth_image().at<float>((int)this->ORBPoints[i].x, (int)this->ORBPoints[i].y))
                {
                    current_x = tracked_point[i].x;
                    current_depth = now_frame->get_depth_image().at<unsigned short>((int)tracked_point[i].x, (int)tracked_point[i].y) * 0.001;
                    previous_x = this->ORBPoints[i].x;
                    previous_depth = previous_frame->get_depth_image().at<unsigned short>((int)this->ORBPoints[i].x, (int)this->ORBPoints[i].y) * 0.001;
                }
                cv::line(this->mask, this->ORBPoints[i], tracked_point[i], cv::Scalar(255,255,255), 1);
                cv::circle(this->mask, this->ORBPoints[i], 2, cv::Scalar(255, 0, 0), -1);
                cv::circle(this->mask, tracked_point[i], 2, cv::Scalar(255, 0, 0), -1);
                cnt++;
            }
        }
        if(cnt == 0 || cnt < config->nGoodFollowed)
            return false;

        cv::Mat tmp;
        cv::addWeighted(now_frame->get_color_image(), 0.7, this->previous_frame->get_color_image(), 0.3, 0.0, tmp);
        cv::add(tmp, mask, this->overlayFrame);

//        this->radian = ((M_PI / 2.0) - atan2(abs(current_depth-previous_depth), abs(current_x-previous_x)));
        this->radian = atan2(abs(current_depth-previous_depth), abs(current_x-previous_x)) * (double)(result / abs(result));
//        std::cout << previous_depth << "\t" << current_depth << "\t" << current_x << "\t" << previous_x << "\t" << radian << std::endl;
        return true;
    }
    catch(cv::Exception &e)
    {
        return false;
    }
}
