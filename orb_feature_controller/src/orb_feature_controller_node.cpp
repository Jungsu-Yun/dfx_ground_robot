#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "Frame.h"
#include "Tracker.h"
#include "ORBConfig.h"

Tracker* tracker;
ORBConfig* config;
image_transport::Publisher image_pub;
bool isInit = false;
bool isFirst = false;

Frame* initFrame;
Frame* firstFrame;

void ImageCallback(const sensor_msgs::Image::ConstPtr &RGB, const sensor_msgs::Image::ConstPtr &Depth) {
    try {
        cv_bridge::CvImageConstPtr RGBPtr;
        cv_bridge::CvImageConstPtr DepthPtr;
        RGBPtr = cv_bridge::toCvCopy(RGB);
        DepthPtr = cv_bridge::toCvCopy(Depth);

        if (!isInit) {
            ROS_INFO("INIT!");
            initFrame = new Frame(RGBPtr->image, DepthPtr->image, config);
        tracker->setInitFrame(initFrame);
            isInit = true;
        } else if (!isFirst) {
            ROS_INFO("First!");
            firstFrame = new Frame(RGBPtr->image, DepthPtr->image, config);
            tracker->setPreviousFrame(firstFrame);
            isFirst = true;
        } else {
            ROS_INFO("Tracking!!");
            Frame currentFrame(RGBPtr->image, DepthPtr->image, config);

            tracker->OpticalFlow(&currentFrame);
            double radian = tracker->CalculateRadian(&currentFrame);
            if(radian < 0)
                ROS_INFO("Left!! : %f", radian);
            else if(radian > 0)
                ROS_INFO("Right! : %f", radian);

            cv_bridge::CvImage output;
            output.encoding = sensor_msgs::image_encodings::RGB8;
            output.image = tracker->getOverlayFrame();
            image_pub.publish(output.toImageMsg());
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("%s", e.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "orb_feature_tracking_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(1), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageCallback,_1,_2));

    image_pub = it.advertise("overlay_image", 1);

    config = new ORBConfig(3000, cv::NORM_HAMMING, 50, 100, 15.0);
    tracker = new Tracker(config);

    ros::spin();

    return 0;
}