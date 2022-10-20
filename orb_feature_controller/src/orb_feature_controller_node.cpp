#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <unistd.h>
#include <termios.h>
#include <iostream>
#include <chrono>

#include "Frame.h"
#include "Tracker.h"
#include "ORBConfig.h"
#include "depth_conversion.h"

Tracker* tracker;
ORBConfig* config;

image_transport::Publisher image_pub;
//image_transport::Publisher depth_pub;
//ros::Publisher pointcloud_pub;
ros::Publisher cmd_pub;

bool isInit = false;
bool isFirst = false;

Frame* initFrame;
Frame* firstFrame;

geometry_msgs::Twist twist;

static struct termios initial_settings, new_settings;
static int peek_character = -1;

double max_linear_velocity = 0.26;
double max_angular_velocity = 1.82;
double linear_velocity_step_size = 0.01;
double angular_velocity_step_size = 0.1;

double current_angular_velocity = 0.0;
double straight_angular_velocity = 0.0;

bool isStraight = false;

double roi = 0.5;

double check_max_linear_speed(double linear)
{
    if(abs(linear) <= max_linear_velocity)
        return linear;
    else
        return max_linear_velocity * (linear / abs(linear));
//        return max_linear_velocity;
}

double check_max_angular_speed(double angular)
{
    if(abs(angular) <= max_angular_velocity)
        return angular;
    else
        return max_angular_velocity * (angular / abs(angular));
//        return max_angular_velocity;
}

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
    while(std::chrono::duration<double>(std::chrono::system_clock::now() - start).count() < 0.1)
    {
        if(_kbhit())
        {
            int ch = _getch();
            return ch;
        }
        else
        {
            return 0;
        }
    }
}

void ImageCallback(const sensor_msgs::Image::ConstPtr &RGB, const sensor_msgs::Image::ConstPtr &Depth, const sensor_msgs::CameraInfo::ConstPtr &info) {
    try {
        cv_bridge::CvImageConstPtr RGBPtr;
        cv_bridge::CvImageConstPtr DepthPtr;

        sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
        image_geometry::PinholeCameraModel model;

        cloud_msg->header = Depth->header;
        cloud_msg->height = Depth->height;
        cloud_msg->width = Depth->width;
        cloud_msg->is_dense = false;
        cloud_msg->is_bigendian = false;

        sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
        pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

        model.fromCameraInfo(info);

        if (Depth->encoding == sensor_msgs::image_encodings::TYPE_16UC1 || Depth->encoding == sensor_msgs::image_encodings::MONO16)
            depth_image_proc::convert<uint16_t>(Depth, cloud_msg, model);
        else if (Depth->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
            depth_image_proc::convert<float>(Depth, cloud_msg, model);

        pcl::PCLPointCloud2 pcl_pc;
        pcl_conversions::toPCL(*cloud_msg, pcl_pc);

        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc, *temp_cloud);

        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(temp_cloud);
        sor.setMeanK(10);
        sor.filter(*filtered_cloud);

        sensor_msgs::PointCloud2 tmp_pcl_msg;
        sensor_msgs::PointCloud pcl_msg;
        pcl::toROSMsg(*filtered_cloud, tmp_pcl_msg);
//        pointcloud_pub.publish(tmp_pcl_msg);

        sensor_msgs::convertPointCloud2ToPointCloud(tmp_pcl_msg, pcl_msg);

        float center_x = info->K[2];
        float center_y = info->K[5];
        float focal_x = info->K[0];
        float focal_y = info->K[4];

        cv::Mat depth_filtered = cv::Mat(Depth->height, Depth->width, CV_32FC1, cv::Scalar(std::numeric_limits<float>::max()));

        for(int i = 0 ; i < pcl_msg.points.size() ; i++)
        {
            if(pcl_msg.points[i].z == pcl_msg.points[i].z)
            {
                float z = pcl_msg.points[i].z * 1000.0;
                float u = (pcl_msg.points[i].x * 1000.0 * focal_x) / z;
                float v = (pcl_msg.points[i].y *1000.0 * focal_y) / z;
                int pixel_pos_x = (int)(u + center_x);
                int pixel_pos_y = (int)(v + center_y);

                if(pixel_pos_x > (info->width - 1))
                    pixel_pos_x = info->width - 1;
                if(pixel_pos_y > (info->height - 1))
                    pixel_pos_y = info->height - 1;
                depth_filtered.at<float>(pixel_pos_y, pixel_pos_x) = z;
            }
        }
        depth_filtered.convertTo(depth_filtered, CV_16UC1);

//        cv_bridge::CvImage depth_output;
//        depth_output.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
//        depth_output.image = depth_filtered;
//        depth_pub.publish(depth_output.toImageMsg());

        RGBPtr = cv_bridge::toCvCopy(RGB);
        DepthPtr = cv_bridge::toCvCopy(Depth);

        cv::Rect rect((RGB->width / 2) - ((RGB->width / 2 )* roi), (RGB->height / 2) - ((RGB->height / 2)*roi), RGB->width * roi, RGB->height*roi);
        if (!isInit) {
            initFrame = new Frame(RGBPtr->image(rect), DepthPtr->image(rect), config);
            initFrame = new Frame(RGBPtr->image(rect), depth_filtered(rect), config);
            tracker->setInitFrame(initFrame);
            isInit = true;
            twist.angular.z = 0.0;
        } else if (!isFirst) {
//            firstFrame = new Frame(RGBPtr->image(rect), DepthPtr->image(rect), config);
            firstFrame = new Frame(RGBPtr->image(rect), depth_filtered(rect), config);
            tracker->setPreviousFrame(firstFrame);
            if(tracker->ORBMatching())
                isFirst = true;
            else
                isInit = false;
            twist.angular.z = 0.0;
        } else {
//            Frame currentFrame(RGBPtr->image(rect), DepthPtr->image(rect), config);
            Frame currentFrame(RGBPtr->image(rect), depth_filtered(rect), config);

            if(!tracker->OpticalFlow(&currentFrame))
            {
                isInit = false;
                isFirst = false;
                twist.angular.z = 0.0;
            }
            else
            {
                double radian = tracker->CalculateRadian(&currentFrame);
                if(isStraight)
                {
                    radian = check_max_angular_speed(radian);
                    twist.angular.z = radian;
                }
                std::cout << "Raidan : " << (float)radian << "\t" << twist.angular.z << std::endl;
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
    int nFeatures = 1000;
    int distanceLimit = 50;
    int goodMatchNum = 100;
    double error_limit = 15.0;
    int nGoodFollowed = 10;
    std::string cmd_topic = "/cmd_vel";
    std::string rgb_topic = "/camera/color/image_raw";
    std::string depth_topic = "/camera/aligned_depth_to_color/image_raw";
    ros::init(argc, argv, "orb_feature_tracking_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    std::string name_of_node = ros::this_node::getName();
    nh.param(name_of_node+"/max_linear_velocity", max_linear_velocity, 0.22);
    nh.param(name_of_node+"/max_angular_velocity", max_angular_velocity, 2.84);
    nh.param(name_of_node+"/linear_step_size", linear_velocity_step_size, 0.01);
    nh.param(name_of_node+"/angular_step_size", angular_velocity_step_size, 0.1);
    nh.param(name_of_node+"/nFeatures", nFeatures, 1000);
    nh.param(name_of_node+"/distance_limit", distanceLimit, 50);
    nh.param(name_of_node+"good_match_num", goodMatchNum, 100);
    nh.param(name_of_node+"good_followed_num", nGoodFollowed, 10);
    nh.param(name_of_node+"/error_limit", error_limit, 5.0);
    nh.param(name_of_node+"/RoI", roi, 0.5);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nh, "/camera/aligned_depth_to_color/camera_info", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub, info_sub);
    sync.registerCallback(boost::bind(&ImageCallback,_1, _2, _3));

    cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    image_pub = it.advertise("/overlay_image", 1);
//    depth_pub = it.advertise("/debug_image", 1);
//    pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/test_pt", 10);
    ros::Rate rate(30);

    config = new ORBConfig(nFeatures, cv::NORM_HAMMING, distanceLimit, goodMatchNum, error_limit, nGoodFollowed);
    tracker = new Tracker(config);

    while(ros::ok())
    {
        ros::spinOnce();
        int key = get_Key();
        if(key == 'w')
        {
            double linear_speed = twist.linear.x + linear_velocity_step_size;
            linear_speed = check_max_linear_speed(linear_speed);
            twist.linear.x = linear_speed;

            if(isInit && isFirst)
                isStraight = true;
            else
                twist.angular.z = 0.0;
        }
        else if(key == 's')
        {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            current_angular_velocity = 0.0;
            isStraight = false;
        }
        else if(key == 'x')
        {
            double linear_speed = twist.linear.x - linear_velocity_step_size;
            linear_speed = check_max_linear_speed(linear_speed);
            twist.linear.x = linear_speed;
            if(isInit && isFirst)
                isStraight = true;
            else
                twist.angular.z = 0.0;
        }
        else if(key == 'a')
        {
            current_angular_velocity += angular_velocity_step_size;
            current_angular_velocity = check_max_angular_speed(current_angular_velocity);

            twist.angular.z = current_angular_velocity;
            isStraight = false;
        }
        else if(key == 'd')
        {
            current_angular_velocity -= angular_velocity_step_size;
            current_angular_velocity = check_max_angular_speed(current_angular_velocity);

            twist.angular.z = current_angular_velocity;
            isStraight = false;
        }
        else if(key == 'q')
            break;

        cmd_pub.publish(twist);
        rate.sleep();
    }

    close_keyboard();

    return 0;
}