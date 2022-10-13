#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/Twist.h>

#include <unistd.h>
#include <termios.h>
#include <iostream>
#include <chrono>

#include "Frame.h"
#include "Tracker.h"
#include "ORBConfig.h"

Tracker* tracker;
ORBConfig* config;

image_transport::Publisher image_pub;
ros::Publisher cmd_pub;

bool isInit = false;
bool isFirst = false;

Frame* initFrame;
Frame* firstFrame;

geometry_msgs::Twist twist;

static struct termios initial_settings, new_settings;
static int peek_character = -1;

int key;

void init_keyboard()
{
    tcgetattr(0,&initial_settings);
    new_settings = initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_cc[VMIN] = 1;
    new_settings.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &new_settings);
}

void close_keyboard()
{
    tcsetattr(0, TCSANOW, &initial_settings);
}

int _kbhit()
{
    unsigned char ch;
    int nread;

    if (peek_character != -1) return 1;
    new_settings.c_cc[VMIN]=0;
    tcsetattr(0, TCSANOW, &new_settings);
    nread = read(0,&ch,1);
    new_settings.c_cc[VMIN]=1;
    tcsetattr(0, TCSANOW, &new_settings);
    if(nread == 1)
    {
        peek_character = ch;
        return 1;
    }
    return 0;
}

int _getch()
{
    char ch;

    if(peek_character != -1)
    {
        ch = peek_character;
        peek_character = -1;
        return ch;
    }
    read(0,&ch,1);
    return ch;
}

int get_Key()
{
    auto start = std::chrono::system_clock::now();
    while(std::chrono::duration<double>(std::chrono::system_clock::now() - start).count() < 1.0)
    {
        if(_kbhit())
        {
            int ch = _getch();
            return ch;
        }
        else
        {
            return key;
        }
    }
}

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
            if(tracker->ORBMatching())
            {
                isFirst = true;
                return;
            }
        } else {
//            ROS_INFO("Tracking!!");
            Frame currentFrame(RGBPtr->image, DepthPtr->image, config);

            if(!tracker->OpticalFlow(&currentFrame))
            {
                isInit = false;
                isFirst = false;
            }
            else
            {
                double radian = tracker->CalculateRadian(&currentFrame);
                ROS_INFO("radian: %f", radian);
                twist.angular.z = radian;

                cv_bridge::CvImage output;
                output.encoding = sensor_msgs::image_encodings::RGB8;
                output.image = tracker->getOverlayFrame();
                image_pub.publish(output.toImageMsg());
            }
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("%s", e.what());
    }
}

int main(int argc, char** argv)
{
    init_keyboard();

    ros::init(argc, argv, "orb_feature_tracking_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(1), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageCallback,_1,_2));

    cmd_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    image_pub = it.advertise("overlay_image", 1);

    config = new ORBConfig(3000, cv::NORM_HAMMING, 50, 100, 15.0);
    tracker = new Tracker(config);

    while(ros::ok())
    {
        ros::spinOnce();
        key = get_Key();
        if(key == 'w')
        {
            twist.linear.x = 0.1;
        }
        else if(key == 's')
        {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
        }
        else if(key == 'x')
        {
            ros::spinOnce();
            twist.linear.x = -0.1;
        }
        else if(key == 'a')
        {

        }
        else if(key == 'd')
        {

        }
        else if(key == 'q')
            break;

        cmd_pub.publish(twist);
    }

    close_keyboard();

    return 0;
}