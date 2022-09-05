#ifndef DFX_GROUND_ROBOT_FAKE_DFXGROUNDROBOTFAKE_H
#define DFX_GROUND_ROBOT_FAKE_DFXGROUNDROBOTFAKE_H

#include <ros/ros.h>
#include <ros/time.h>

#include <std_msgs/Int32.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <dfx_ground_robot_msgs/SensorState.h>

#define WHEEL_RADIUS                    0.04535     // meter

#define LEFT_FRONT                      0
#define LEFT_REAR                       1
#define RIGHT_FRONT                     2
#define RIGHT_REAR                      3

#define MAX_LINEAR_VELOCITY             0.22   // m/s
#define MAX_ANGULAR_VELOCITY            2.84   // rad/s
#define VELOCITY_STEP                   0.01   // m/s
#define VELOCITY_LINEAR_X               0.01   // m/s
#define VELOCITY_ANGULAR_Z              0.1    // rad/s
#define SCALE_VELOCITY_LINEAR_X         1
#define SCALE_VELOCITY_ANGULAR_Z        1

#define DEG2RAD(x)                      (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                      (x * 57.2957795131)  // *180/PI

#define TORQUE_ENABLE                   1       // Value for enabling the torque of motor
#define TORQUE_DISABLE                  0       // Value for disabling the torque of motor

class DfxGroundRobotFake
{
private:
    ros::NodeHandle node_handle;
    ros::NodeHandle prev_node_handle;

    ros::Time las_cmd_vel_time;
    ros::Time prev_update_time;

    ros::Publisher joint_state_publisher;
    ros::Publisher odometry_publisher;

    ros::Subscriber cmd_vel_subscriber;

    sensor_msgs::JointState jointState;
    nav_msgs::Odometry odometry;
    tf::TransformBroadcaster tf_broadcaster;

    double wheel_speed_cmd[4];
    double goal_linear_velocity;
    double goal_angular_velocity;
    double cmd_vel_timeout;

    float odom_pose[3];
    float odom_velocity[3];
    double pose_conv[36];

    std::string joint_states_name[4];

    double last_position[4];
    double last_velocity[4];

    double wheel_seperation;
    double turning_radius;
    double robot_radius;

    void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg);
    bool updateOdometry(ros::Duration diff_time);
    void updateJoint();
    void updateTF(geometry_msgs::TransformStamped& odom_tf);

public:
    DfxGroundRobotFake();
    ~DfxGroundRobotFake();

    bool init();
    bool update();
};


#endif //DFX_GROUND_ROBOT_FAKE_DFXGROUNDROBOTFAKE_H
