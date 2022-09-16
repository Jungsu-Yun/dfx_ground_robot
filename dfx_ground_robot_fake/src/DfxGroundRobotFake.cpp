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

    node_handle.param("wheel_left_joint_name", joint_states_name[LEFT_FRONT],  std::string("left_front_joint"));
    node_handle.param("wheel_right_joint_name", joint_states_name[LEFT_REAR],  std::string("left_rear_joint"));
    node_handle.param("wheel_right_joint_name", joint_states_name[RIGHT_FRONT],  std::string("right_front_joint"));
    node_handle.param("wheel_right_joint_name", joint_states_name[RIGHT_REAR],  std::string("right_rear_joint"));
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
    memcpy(&(odometry.pose.covariance),pcov,sizeof(double)*36);
    memcpy(&(odometry.twist.covariance),pcov,sizeof(double)*36);

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
    last_cmd_vel_time = ros::Time::now();

    goal_linear_velocity = msg->linear.x;
    goal_angular_velocity = msg->angular.z;

    wheel_speed_cmd[LEFT_FRONT] = wheel_speed_cmd[LEFT_REAR] = goal_linear_velocity - (goal_angular_velocity * wheel_seperation / 2);
    wheel_speed_cmd[RIGHT_FRONT] = wheel_speed_cmd[RIGHT_REAR] = goal_linear_velocity + (goal_angular_velocity * wheel_seperation / 2);
}

bool DfxGroundRobotFake::updateOdometry(ros::Duration diff_time)
{
    double wheel_lf, wheel_lr, wheel_rf, wheel_rr;
    double delta_s, delta_theta;

    double v[4], w[4];

    wheel_lf = wheel_lr = wheel_rf = wheel_rr = 0.0;
    delta_s = delta_theta = 0.0;

    v[LEFT_FRONT] = wheel_speed_cmd[LEFT_FRONT];
    v[LEFT_REAR] = wheel_speed_cmd[LEFT_REAR];
    w[LEFT_FRONT] = v[LEFT_FRONT] / WHEEL_RADIUS;
    w[LEFT_REAR] = v[LEFT_REAR] / WHEEL_RADIUS;

    v[RIGHT_FRONT] = wheel_speed_cmd[RIGHT_FRONT];
    v[RIGHT_REAR] = wheel_speed_cmd[RIGHT_REAR];
    w[RIGHT_FRONT] = v[RIGHT_FRONT] / WHEEL_RADIUS;
    w[RIGHT_REAR] = v[RIGHT_REAR] / WHEEL_RADIUS;

    last_velocity[LEFT_FRONT] = w[LEFT_FRONT];
    last_velocity[LEFT_REAR] = w[LEFT_REAR];
    last_velocity[RIGHT_FRONT] = w[RIGHT_FRONT];
    last_velocity[RIGHT_REAR] = w[RIGHT_REAR];

    wheel_lf = w[LEFT_FRONT] * diff_time.toSec();
    wheel_lr = w[LEFT_REAR] * diff_time.toSec();
    wheel_rf = w[RIGHT_FRONT] * diff_time.toSec();
    wheel_rr = w[RIGHT_REAR] * diff_time.toSec();

    if(isnan(wheel_lf))
        wheel_lf = 0.0;
    if(isnan(wheel_lr))
        wheel_lr = 0.0;
    if(isnan(wheel_rf))
        wheel_rf = 0.0;
    if(isnan(wheel_rr))
        wheel_rr = 0.0;

    last_position[LEFT_FRONT] += wheel_lf;
    last_position[LEFT_REAR] += wheel_lr;
    last_position[RIGHT_FRONT] += wheel_rf;
    last_position[RIGHT_REAR] += wheel_rr;

    delta_s = WHEEL_RADIUS * (wheel_rf + wheel_lf) / 2.0;
    delta_theta = WHEEL_RADIUS * (wheel_rf - wheel_lf) / wheel_seperation;

    odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
    odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
    odom_pose[2] += delta_theta;

    odom_velocity[0] = delta_s / diff_time.toSec();
    odom_velocity[1] = 0.0;
    odom_velocity[2] = delta_theta / diff_time.toSec();

    odometry.pose.pose.position.x = odom_pose[0];
    odometry.pose.pose.position.y = odom_pose[1];
    odometry.pose.pose.position.z = 0;
    odometry.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_pose[2]);

    odometry.twist.twist.linear.x = odom_velocity[0];
    odometry.twist.twist.angular.z = odom_velocity[2];

    return true;
}

void DfxGroundRobotFake::updateJoint()
{
    jointState.position[LEFT_FRONT] = last_position[LEFT_FRONT];
    jointState.position[LEFT_REAR] = last_position[LEFT_REAR];
    jointState.position[RIGHT_FRONT] = last_position[RIGHT_FRONT];
    jointState.position[RIGHT_REAR] = last_position[RIGHT_REAR];

    jointState.velocity[LEFT_FRONT] = last_velocity[LEFT_FRONT];
    jointState.velocity[LEFT_REAR] = last_velocity[LEFT_REAR];
    jointState.velocity[RIGHT_FRONT] = last_velocity[RIGHT_FRONT];
    jointState.velocity[RIGHT_REAR] = last_velocity[RIGHT_REAR];
}

void DfxGroundRobotFake::updateTF(geometry_msgs::TransformStamped &odom_tf)
{
    odom_tf.header = odometry.header;
    odom_tf.child_frame_id = odometry.child_frame_id;
    odom_tf.transform.translation.x = odometry.pose.pose.position.x;
    odom_tf.transform.translation.y = odometry.pose.pose.position.y;
    odom_tf.transform.translation.z = odometry.pose.pose.position.z;
    odom_tf.transform.rotation = odometry.pose.pose.orientation;
}

bool DfxGroundRobotFake::update()
{
    ros::Time time_now = ros::Time::now();
    ros::Duration step_time = time_now - prev_update_time;
    prev_update_time = time_now;

    if((time_now - last_cmd_vel_time).toSec() > cmd_vel_timeout)
        wheel_speed_cmd[LEFT_FRONT] = wheel_speed_cmd[LEFT_REAR] = wheel_speed_cmd[RIGHT_FRONT] = wheel_speed_cmd[RIGHT_REAR] = 0.0;

    updateOdometry(step_time);
    odometry.header.stamp = time_now;
    odometry_publisher.publish(odometry);

    updateJoint();
    jointState.header.stamp = time_now;
    joint_state_publisher.publish(jointState);

    geometry_msgs::TransformStamped odom_tf;
    updateTF(odom_tf);
    tf_broadcaster.sendTransform(odom_tf);

    return true;
}
