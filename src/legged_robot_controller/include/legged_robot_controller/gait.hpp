
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


#ifndef GAIT_HPP_
#define GAIT_HPP_

#include <cmath> // std::abs
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <legged_robot_msgs/msg/feet_positions.hpp>

//=============================================================================
// Define structs and classes for gait system
//=============================================================================

class GetGaitParams : public rclcpp::Node {
public:
    GetGaitParams();

    void callbackGaitParam(std::shared_future <std::vector<rclcpp::Parameter>> future);

private:
    std::shared_ptr <rclcpp::AsyncParametersClient> parameters_client;
};

class Gait {
public:
    Gait(void);

    void gaitCycle(const geometry_msgs::msg::Twist &cmd_vel, legged_robot_msgs::msg::FeetPositions *feet,
                   geometry_msgs::msg::Twist *gait_vel);

private:
    void cyclePeriod(const geometry_msgs::msg::Pose2D &base, legged_robot_msgs::msg::FeetPositions *feet,
                     geometry_msgs::msg::Twist *gait_vel);

    void sequence_change(std::vector<int> &vec);

    geometry_msgs::msg::Pose2D smooth_base_;
    rclcpp::Time current_time_, last_time_;
    bool is_travelling_;      // True if the robot is moving, not just in a cycle
    bool in_cycle_;           // True if the robot is in a gait cycle
    int CYCLE_LENGTH;         // Number of steps in cycle
    int NUMBER_OF_LEGS;       // Leg order in cycle of the leg
    double LEG_LIFT_HEIGHT;   // Height of a leg cycle
    std::string GAIT_STYLE;    // gait style Tripod or Ripple
    int cycle_period_;        // Current period in cycle
    int extra_gait_cycle_;    // Forcing some extra timed cycles
    double period_distance;
    double period_height;
    double gait_factor;
    std::vector<int> cycle_leg_number_; // Leg gait order (grouping) ['RR', 'RM', 'RF', 'LR', 'LM', 'LF']

};

#endif // GAIT_H_
