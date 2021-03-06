
// ROS2 Legged Robot Controller Node
// Copyright (c) 2020, Robert M. Dome
// Based upon Kevin Ochs ROS Hexapd Controller Node, Copyright (c) 2014, Kevin M. Ochs.
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


#include <gait.hpp>

static const double PI = atan(1.0) * 4.0;

//==============================================================================
//  Constructor: Initialize gait variables
//==============================================================================

Gait::Gait() : Node("get_gait_params") {
    parameters_client =
            std::make_shared<rclcpp::AsyncParametersClient>(this, "/legged_robot_parameter_server");

    parameters_client->wait_for_service();
    auto parameters_future = parameters_client->get_parameters(
            {"CYCLE_LENGTH",
             "LEG_LIFT_HEIGHT",
             "NUMBER_OF_LEGS",
             "GAIT_STYLE"
            },
            std::bind(&Gait::callbackGaitParam, this, std::placeholders::_1));
}

cycle_period_ = 25;
is_travelling_ = false;
in_cycle_ = false;
extra_gait_cycle_ = 1;
current_time_ = rclcpp::Time::now();
last_time_ = rclcpp::Time::now();
if (GAIT_STYLE == "TETRAPOD") {
gait_factor = 1.0;
cycle_leg_number_ = {1, 0, 0, 1, 1, 0, 0, 1};
}
if (GAIT_STYLE == "TRIPOD") {
gait_factor = 1.0;
cycle_leg_number_ = {0, 1, 1, 0, 0, 1};
}
if (GAIT_STYLE == "RIPPLE") {
gait_factor = 0.5;
cycle_leg_number_ = {1, 2, 2, 0, 0, 1};
}
period_distance = 0;
period_height = 0;
}

void Gait::callbackGaitParam(std::shared_future <std::vector<rclcpp::Parameter>> future) {
    auto param = future.get();

    CYCLE_LENGTH = param.at(0);
    LEG_LIFT_HEIGHT = param.at(1);
    NUMBER_OF_LEGS = param.at(2);
    GAIT_STYLE = param.at(3);
}
//=============================================================================
// step calculation
//=============================================================================

void Gait::cyclePeriod(const geometry_msgs::msg::Pose2D &base, legged_robot_msgs::msg::FeetPositions *feet,
                       geometry_msgs::msg::Twist *gait_vel) {
    period_height = sin(cycle_period_ * PI / CYCLE_LENGTH);

    // Calculate current velocities for this period of the gait
    // This factors in the sinusoid of the step for accurate odometry
    current_time_ = rclcpp::Time::now();
    double dt = (current_time_ - last_time_).toSec();
    gait_vel->linear.x = ((PI * base.x) / CYCLE_LENGTH) * period_height * (1.0 / dt);
    gait_vel->linear.y = ((-PI * base.y) / CYCLE_LENGTH) * period_height * (1.0 / dt);
    gait_vel->angular.z = ((PI * base.theta) / CYCLE_LENGTH) * period_height * (1.0 / dt);
    last_time_ = current_time_;

    for (int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++) {
        // Lifts the leg and move it forward
        if (cycle_leg_number_[leg_index] == 0 && is_travelling_ == true) {
            period_distance = cos(cycle_period_ * PI / CYCLE_LENGTH);
            feet->foot[leg_index].position.x = base.x * period_distance;
            feet->foot[leg_index].position.y = base.y * period_distance;
            feet->foot[leg_index].position.z = LEG_LIFT_HEIGHT * period_height;
            feet->foot[leg_index].orientation.yaw = base.theta * period_distance;
        }
        // Moves legs backward pushing the body forward
        if (cycle_leg_number_[leg_index] == 1) {
            period_distance = cos(cycle_period_ * PI * gait_factor / CYCLE_LENGTH);
            feet->foot[leg_index].position.x = -base.x * period_distance;
            feet->foot[leg_index].position.y = -base.y * period_distance;
            feet->foot[leg_index].position.z = 0;
            feet->foot[leg_index].orientation.yaw = -base.theta * period_distance;
        }
        if (cycle_leg_number_[leg_index] == 2) {
            period_distance = cos((CYCLE_LENGTH + cycle_period_) * PI * gait_factor / CYCLE_LENGTH);
            feet->foot[leg_index].position.x = -base.x * period_distance;
            feet->foot[leg_index].position.y = -base.y * period_distance;
            feet->foot[leg_index].position.z = 0;
            feet->foot[leg_index].orientation.yaw = -base.theta * period_distance;
        }
    }
}

//=============================================================================
// Gait Sequencing
//=============================================================================

void Gait::gaitCycle(const geometry_msgs::msg::Twist &cmd_vel, legged_robot_msgs::msg::FeetPositions *feet,
                     geometry_msgs::msg::Twist *gait_vel) {
    // Convert velocities into actual distance for gait/foot positions
    geometry_msgs::msg::Pose2D base;
    base.x = cmd_vel.linear.x / PI * CYCLE_LENGTH;
    base.y = cmd_vel.linear.y / PI * CYCLE_LENGTH;
    base.theta = cmd_vel.angular.z / PI * CYCLE_LENGTH;

    // Low pass filter on the values to avoid jerky movements due to rapid value changes
    smooth_base_.x = base.x * 0.05 + (smooth_base_.x * (1.0 - 0.05));
    smooth_base_.y = base.y * 0.05 + (smooth_base_.y * (1.0 - 0.05));
    smooth_base_.theta = base.theta * 0.05 + (smooth_base_.theta * (1.0 - 0.05));

    // Check to see if we are actually travelling
    if ((std::abs(smooth_base_.y) > 0.001) || // 1 mm
        (std::abs(smooth_base_.x) > 0.001) || // 1 mm
        (std::abs(smooth_base_.theta) > 0.00436332313)) // 0.25 degree
    {
        is_travelling_ = true;
    } else {
        is_travelling_ = false;
        // Check to see if the legs are in a non rest state
        for (int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++) {
            if ((std::abs(feet->foot[leg_index].position.x) > 0.001) || // 1 mm
                (std::abs(feet->foot[leg_index].position.y) > 0.001) || // 1 mm
                (std::abs(feet->foot[leg_index].orientation.yaw) > 0.034906585) || // 2 degrees
                std::abs(feet->foot[leg_index].position.z) > 0.001) // 1 mm
            {
                // If so calculate the rest of the cycle and add another complete cycle
                // This forces another cycle to allow all legs to set down after travel is stopped
                extra_gait_cycle_ = CYCLE_LENGTH - cycle_period_ + CYCLE_LENGTH;
                break;
            } else {
                extra_gait_cycle_ = 1;
            }
        }

        // countdown for in_cycle state
        if (extra_gait_cycle_ > 1) {
            extra_gait_cycle_--;
            in_cycle_ = !(extra_gait_cycle_ == 1);
        }
    }

    // If either is true we consider the gait active
    if (is_travelling_ == true || in_cycle_ == true) {
        cyclePeriod(smooth_base_, feet, gait_vel);
        cycle_period_++;
    } else {
        // Reset period to start just to be sure. ( It should be here anyway )
        cycle_period_ = 0;
    }

    // Loop cycle and switch the leg groupings for cycle
    if (cycle_period_ == CYCLE_LENGTH) {
        cycle_period_ = 0;
        sequence_change(cycle_leg_number_); //sequence change
    }
}

//=============================================================================
// Gait Sequence Change
//=============================================================================

void Gait::sequence_change(std::vector<int> &vec) {
    for (int i = 0; i < vec.size(); i++) {
        if (vec[i] == 0) vec[i] = 1;
        else if (vec[i] == 1 && GAIT_STYLE == "RIPPLE") vec[i] = 2;
        else vec[i] = 0;
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Gait>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}