
// ROS2 Legged Robot Locomotion Node
// Copyright (c) 2020, Robert M. Dome
// Based upon Kevin Ochs ROS Hexapd Locomotion Node, Copyright (c) 2014, Kevin M. Ochs.
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of the <organization> nor the
//     names of its contributors may be used to endorse or promote products
//     derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Author: Robert M. Dome


#ifndef CONTROL_HPP_
#define CONTROL_HPP_

#include <cmath>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <legged_robot_msgs/msg/pose.hpp>
#include <legged_robot_msgs/msg/rpy.hpp>
#include <legged_robot_msgs/msg/legs_joints.hpp>
#include <legged_robot_msgs/msg/feet_positions.hpp>
#include <legged_robot_msgs/msg/sounds.hpp>

//==============================================================================
// Define class Control: This is the main structure of data that manipulates
// the legged_robot.
//==============================================================================

class Control : public rclcpp::Node {
public:

    Control();

    void callbackControlParam();

    void setLeggedRobotActiveState(bool state);

    bool getLeggedRobotActiveState(void);

    void setPrevLeggedRobotActiveState(bool state);

    bool getPrevLeggedRobotActiveState(void);

    void publishJointStates(const legged_robot_msgs::msg::LegsJoints &legs, const legged_robot_msgs::msg::RPY &head,
                            sensor_msgs::msg::JointState *joint_state);

    void publishOdometry(const geometry_msgs::msg::Twist &gait_vel);

    void publishTwist(const geometry_msgs::msg::Twist &gait_vel);

    void partitionCmd_vel(geometry_msgs::msg::Twist *cmd_vel);

    int MASTER_LOOP_RATE;  // Master loop rate
    sensor_msgs::msg::JointState joint_state_;
    legged_robot_msgs::msg::Pose body_;    // Body link rotation
    legged_robot_msgs::msg::RPY head_;
    legged_robot_msgs::msg::LegsJoints legs_;
    legged_robot_msgs::msg::FeetPositions feet_;
    double STANDING_BODY_HEIGHT;
    geometry_msgs::msg::Twist gait_vel_;
    geometry_msgs::msg::Twist cmd_vel_;

private:
    std::shared_ptr <rclcpp::AsyncParametersClient> parameters_client;
    legged_robot_msgs::msg::Sounds sounds_; // Sound bool array
    std_msgs::msg::Bool imu_override_; // Override body levelling for body manipulation
    bool imu_init_stored_; // Auto-levelling
    double imu_roll_lowpass_, imu_pitch_lowpass_, imu_yaw_lowpass_, imu_roll_init_, imu_pitch_init_; // Auto-levelling
    double MAX_BODY_ROLL_COMP, MAX_BODY_PITCH_COMP, COMPENSATE_INCREMENT, COMPENSATE_TO_WITHIN; // Auto-levelling yaml config
    double BODY_MAX_ROLL, BODY_MAX_PITCH, BODY_MAX_YAW, HEAD_MAX_YAW, HEAD_MAX_PITCH; // Mechanical limits
    double VELOCITY_DIVISION;
    double pose_x_ = 0.0;  // pose odometry
    double pose_y_ = 0.0;  // pose odometry
    double pose_th_ = 0.0; // pose odometry
    int NUMBER_OF_LEGS;        // Number of legs
    int NUMBER_OF_HEAD_JOINTS; // Number of head segments
    int NUMBER_OF_LEG_JOINTS;  // Number of leg segments
    int SERVO_COUNT;
    std::vector <std::map<std::string, auto>> SERVOS;
    std::vector <std::string> servo_names_;
    std::vector<int> servo_orientation_;
    bool legged_robot_state_;      // Current loop state
    bool prev_legged_robot_state_; // Previous loop state
    rclcpp::Time current_time_odometry_, last_time_odometry_, current_time_cmd_vel_, last_time_cmd_vel_;
    tf2_ros::TransformBroadcaster odom_broadcaster;
    geometry_msgs::msg::Twist cmd_vel_incoming_;

    // Topics we are subscribing
    rclcpp::Subscriber cmd_vel_sub_;

    void cmd_velCallback(const geometry_msgs::msg::TwistConstPtr &cmd_vel_msg);

    rclcpp::Subscriber body_scalar_sub_;

    void bodyCallback(const geometry_msgs::msg::AccelStampedConstPtr &body_scalar_msg);

    rclcpp::Subscriber head_scalar_sub_;

    void headCallback(const geometry_msgs::msg::AccelStampedConstPtr &head_scalar_msg);

    rclcpp::Subscriber state_sub_;

    void stateCallback(const std_msgs::msg::BoolConstPtr &state_msg);

    rclcpp::Subscriber imu_override_sub_;

    void imuOverrideCallback(const std_msgs::msg::BoolConstPtr &imu_override_msg);

    rclcpp::Subscriber imu_sub_;

    void imuCallback(const sensor_msgs::msg::ImuConstPtr &imu_msg);

    // Topics we are publishing
    rclcpp::Publisher sounds_pub_;
    rclcpp::Publisher joint_state_pub_;
    rclcpp::Publisher odom_pub_;
    rclcpp::Publisher twist_pub_;

    // Services we call
    rclcpp::ServiceClient imu_calibrate_;
    std_srvs::Empty calibrate_;

    // Node Handle
    auto node = rclcpp::Node::make_shared("nh");
};

#endif // CONTROL_H_
