#include <dfx_ground_robot_fake/DfxGroundRobotFake.h>

DfxGroundRobotFake::DfxGroundRobotFake() : prev_node_handle("~")
{
    bool init_result = init();
    ROS_ASSERT(init_result);
}

bool DfxGroundRobotFake::init()
{
    this->wheel_seperation = 0.160;
    this->turning_radius = 0.080;
    this->robot_radius = 0.105;

    node_handle.param("wheel_left_joint_name", joint_states_name[LEFT_FRONT],  std::string("left front joint"));
    node_handle.param("wheel_right_joint_name", joint_states_name[LEFT_REAR],  std::string("left rear joint"));
    node_handle.param("wheel_right_joint_name", joint_states_name[RIGHT_FRONT],  std::string("right front joint"));
    node_handle.param("wheel_right_joint_name", joint_states_name[RIGHT_REAR],  std::string("right rear joint"));
    node_handle.param("joint_states_frame", jointState.header.frame_id, std::string("base_footprint"));
    node_handle.param("odom_frame", odometry.header.frame_id, std::string("odom"));
    node_handle.param("base_frame", odometry.child_frame_id, std::string("base_footprint"));

    wheel_speed_cmd[LEFT_FRONT] = 0.0;
    wheel_speed_cmd[LEFT_REAR] = 0.0;
    wheel_speed_cmd[RIGHT_FRONT] = 0.0;
    wheel_speed_cmd[RIGHT_REAR] = 0.0;

    goal_linear_velocity = 0.0;
    goal_angular_velocity = 0.0;
    cmd_vel_timeout = 1.0;

    last_position[LEFT_FRONT] = 0.0;
    last_position[LEFT_REAR] = 0.0;
    last_position[RIGHT_FRONT] = 0.0;
    last_position[RIGHT_REAR] = 0.0;

    last_velocity[LEFT_FRONT] = 0.0;
    last_velocity[LEFT_REAR] = 0.0;
    last_velocity[RIGHT_FRONT] = 0.0;
    last_velocity[RIGHT_REAR] = 0.0;

    double pcov[36] = { 0.1,   0,   0,   0,   0, 0,
                        0, 0.1,   0,   0,   0, 0,
                        0,   0, 1e6,   0,   0, 0,
                        0,   0,   0, 1e6,   0, 0,
                        0,   0,   0,   0, 1e6, 0,
                        0,   0,   0,   0,   0, 0.2};
    memcpy(&(odom_.pose.covariance),pcov,sizeof(double)*36);
    memcpy(&(odom_.twist.covariance),pcov,sizeof(double)*36);

    odom_pose[0] = 0.0;
    odom_pose[1] = 0.0;
    odom_pose[2] = 0.0;

    odom_velocity[0] = 0.0;
    odom_velocity[1] = 0.0;
    odom_velocity[2] = 0.0;

    jointState.name.push_back(joint_states_name[LEFT_FRONT]);
    jointState.name.push_back(joint_states_name[LEFT_REAR]);
    jointState.name.push_back(joint_states_name[RIGHT_FRONT]);
    jointState.name.push_back(joint_states_name[RIGHT_REAR]);

    jointState.position.resize(2,0.0);
    jointState.velocity.resize(2,0.0);
    jointState.effort.resize(2,0.0);

    joint_state_publisher = node_handle.advertise<sensor_msgs::JointState>("joint_states", 100);
    odometry_publisher = node_handle.advertise<nav_msgs::Odometry>("odom", 100);

    cmd_vel_subscriber = node_handle.subscribe("/cmd_vel", 100, &DfxGroundRobotFake::cmdCallback, this);

    prev_update_time = ros::Time::now();
    return true;
}

void DfxGroundRobotFake::cmdCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    las_cmd_vel_time = ros::Time::now();

    goal_linear_velocity = msg->linear.x;
    goal_angular_velocity = msg->angular.z;

    wheel_speed_cmd[LEFT_FRONT] = wheel_speed_cmd[LEFT_REAR] = goal_linear_velocity - (goal_angular_velocity * wheel_seperation / 2);
    wheel_speed_cmd[RIGHT_FRONT] = wheel_speed_cmd[RIGHT_REAR] = goal_linear_velocity + (goal_angular_velocity * wheel_seperation / 2);
}
