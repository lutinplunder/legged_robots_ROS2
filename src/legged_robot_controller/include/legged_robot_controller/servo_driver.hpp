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

#ifndef SERVO_DRIVER_HPP_
#define SERVO_DRIVER_HPP_

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>

// Default setting
#define BAUDRATE    1000000
#define DEVICENAME  "/dev/ttyUSB0"      // Check which port is being used on your controller
#define PROTOCOL_VERSION   2.0          // See which protocol version is used in the Dynamixel
#define TORQUE_ON   1
#define TORQUE_OFF  0
#define LEN_GOAL_POSITION  2
//==============================================================================
// Define the class(s) for Servo Drivers.
//==============================================================================

class ServoDriver : public rclcpp::Node {
public:
    ServoDriver();

    ~ServoDriver(void);

    void transmitServoPositions(const sensor_msgs::msg::JointState &joint_state);

    void makeSureServosAreOn(const sensor_msgs::msg::JointState &joint_state);

    void freeServos(void);

    void callbackServoDriverParam();

private:
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(
            DEVICENAME); // Initialize PacketHandler instance
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(
            PROTOCOL_VERSION); // Set the protocol version
    uint8_t dxl_error = 0;                          // Dynamixel error
    uint16_t dxl_present_position = 0;              // Present position
    uint16_t currentPos;
    uint8_t param_goal_position[2];

    void convertAngles(const sensor_msgs::msg::JointState &joint_state);

    std::shared_ptr <rclcpp::AsyncParametersClient> parameters_client;
    std::vector<int> cur_pos_; // Current position of servos
    std::vector<int> goal_pos_; // Goal position of servos
    std::vector<int> pose_steps_; // Increment to use going from current position to goal position
    std::vector<int> write_pos_; // Position of each servo for sync_write packet
    std::vector<double> OFFSET; // Physical hardware offset of servo horn
    std::vector<int> ID; // Servo IDs
    std::vector<int> TICKS; // Total number of ticks, meaning resolution of dynamixel servo
    std::vector<int> CENTER; // Center value of dynamixel servo
    std::vector<double> RAD_TO_SERVO_RESOLUTION; // Radians to servo conversion
    std::vector<double> MAX_RADIANS; // Max rotation your servo is manufactured to do. i.e. 360 degrees for MX etc.
    std::vector<int> servo_orientation_; // If the servo is physically mounted backwards this sign is flipped
    bool portOpenSuccess = false;
    bool torque_on = true;
    bool torque_off = true;
    bool writeParamSuccess = true;
    bool servos_free_;
    std::vector<std::map<std::string, auto>> SERVOS;
    int SERVO_COUNT;
    int NUMBER_OF_LEGS;        // Number of legs
    int NUMBER_OF_HEAD_JOINTS; // Number of head segments
    int NUMBER_OF_LEG_JOINTS;  // Number of leg segments
    int TORQUE_ENABLE, PRESENT_POSITION_L, GOAL_POSITION_L, INTERPOLATION_LOOP_RATE;
};

#endif // SERVO_DRIVER_H_
