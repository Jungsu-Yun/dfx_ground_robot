#include <ros/ros.h>
#include <dfx_ground_robot_fake/DfxGroundRobotFake.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dfx_ground_robot_fake_node");
    DfxGroundRobotFake robotFake;

    ros::Rate loop_rate(30);

    while(ros::ok())
    {
        robotFake.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}