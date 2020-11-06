
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


#ifndef IK_HPP_
#define IK_HPP_

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <FABRIK2D.hpp>
#include <legged_robot_msgs/msg/pose.hpp>
#include <legged_robot_msgs/msg/legs_joints.hpp>
#include <legged_robot_msgs/msg/feet_positions.hpp>

//=============================================================================
// Define structs and classes for gait system
//=============================================================================

struct Trig
{
    double sine;
    double cosine;
};

class IKParams : public rclcpp::Node {
public:
    GetIKParams();

    void callbackIKParam(std::shared_future <std::vector<rclcpp::Parameter>> future);

private:
    std::shared_ptr <rclcpp::AsyncParametersClient> parameters_client;
};

class Ik {
public:
    Ik(void);

    void calculateIK(const legged_robot_msgs::msg::FeetPositions &feet, const legged_robot_msgs::msg::Pose &body,
                     legged_robot_msgs::msg::LegsJoints *legs);

private:
    Trig getSinCos( double angle_rad );
    std::vector<double> COXA_TO_CENTER_X, COXA_TO_CENTER_Y; // Distance from coxa joint to the center pivot
    std::vector<double> INIT_COXA_ANGLE; // Initial coxa offsets in radians
    std::vector<double> INIT_FOOT_POS_X, INIT_FOOT_POS_Y, INIT_FOOT_POS_Z; // Start position Of feet
    std::vector<double> LENGTHS; // Leg segment measurements
    double COXA_LENGTH, FEMUR_LENGTH, TIBIA_LENGTH, TARSUS_LENGTH;
    int NUMBER_OF_LEGS; // Number of legs
    int NUMBER_OF_LEG_SEGMENTS;
    double STANDING_BODY_HEIGHT;
    std::string ALGORITHM;
};

#endif // IK_H_
