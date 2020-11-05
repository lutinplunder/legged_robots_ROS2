
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


#include <rclcpp/rclcpp.hpp>
#include <control.hpp>
#include <gait.hpp>
#include <ik.hpp>
#include <servo_driver.hpp>

//=============================================================================
// Main
//=============================================================================

class LeggedRobotControllerNode : public rclcpp::Node
{
public:
    LeggedRobotControllerNode() : Node("legged_robot_controller_node") {}
private:
};

int main( int argc, char **argv )
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LeggedRobotControllerNode>;

    // Create class objects
    Control control;
    Gait gait;
    Ik ik;
    ServoDriver servoDriver;

    // Establish initial leg positions for default pose in robot publisher
    gait.gaitCycle( control.cmd_vel_, &control.feet_, &control.gait_vel_ );
    ik.calculateIK( control.feet_, control.body_, &control.legs_ );
    control.publishJointStates( control.legs_, control.head_, &control.joint_state_ );
    control.publishOdometry( control.gait_vel_ );
    control.publishTwist( control.gait_vel_ );


//    rclcpp::Clock (RCL_SYSTEM_TIME);
    rclcpp::Clock current_time_, last_time_;
    current_time_ = LeggedRobotControllerNode::get_clock();
    last_time_ = rclcpp::Clock.now();

    rclcpp::executors::MultiThreadedExecutor::spin	()
    rclcpp::executors::MultiThreadedExecutor::run	( 2 )
//    ros::AsyncSpinner spinner( 2 ); // Using 2 threads
//    spinner.start();
    rclcpp::Rate loop_rate( control.MASTER_LOOP_RATE );  // Speed limit of loop ( Will go slower than this )
    while( rclcpp::ok() )
    {
        current_time_ = rclcpp::Time::now();
        double dt = ( current_time_ - last_time_ ).toSec();

        // Divide cmd_vel by the loop rate to get appropriate velocities for gait period
        control.partitionCmd_vel( &control.cmd_vel_ );

        // Start button on controller has been pressed stand up
        if( control.getLeggedRobotActiveState() == true && control.getPrevLeggedRobotActiveState() == false )
        {
            RCLCPP_INFO(rclcpp::get_logger(), "Robot is standing up.";
        while( control.body_.position.z < control.STANDING_BODY_HEIGHT )
            {
                control.body_.position.z = control.body_.position.z + 0.001; // 1 mm increment

                // IK solver for legs and body orientation
                ik.calculateIK( control.feet_, control.body_, &control.legs_ );

                // Commit new positions and broadcast over USB2AX as well as jointStates
                control.publishJointStates( control.legs_, control.head_, &control.joint_state_ );
                servoDriver.transmitServoPositions( control.joint_state_ );
                control.publishOdometry( control.gait_vel_ );
                control.publishTwist( control.gait_vel_ );
            }
            control.setPrevLeggedRobotActiveState( true );
        }

        // We are live and standing up
        if( control.getLeggedRobotActiveState() == true && control.getPrevLeggedRobotActiveState() == true )
        {
            // Gait Sequencer
            gait.gaitCycle( control.cmd_vel_, &control.feet_, &control.gait_vel_ );
            control.publishTwist( control.gait_vel_ );

            // IK solver for legs and body orientation
            ik.calculateIK( control.feet_, control.body_, &control.legs_ );

            // Commit new positions and broadcast over USB2AX as well as jointStates
            control.publishJointStates( control.legs_, control.head_, &control.joint_state_ );
            servoDriver.transmitServoPositions( control.joint_state_ );
            control.publishOdometry( control.gait_vel_ );
            control.publishTwist( control.gait_vel_ );

            // Set previous LeggedRobot state of last loop so we know if we are shutting down on the next loop
            control.setPrevLeggedRobotActiveState( true );
        }

        // Shutting down LeggedRobot so let us do a gradual sit down and turn off torque
        if( control.getLeggedRobotActiveState() == false && control.getPrevLeggedRobotActiveState() == true )
        {
            RCLCPP_INFO("Robot sitting down.");
            while( control.body_.position.z > 0 )
            {
                control.body_.position.z = control.body_.position.z - 0.001; // 1 mm increment

                // Gait Sequencer called to make sure we are on all six feet
                gait.gaitCycle( control.cmd_vel_, &control.feet_, &control.gait_vel_ );

                // IK solver for legs and body orientation
                ik.calculateIK( control.feet_, control.body_, &control.legs_ );

                // Commit new positions and broadcast over USB2AX as well as jointStates
                control.publishJointStates( control.legs_, control.head_, &control.joint_state_ );
                servoDriver.transmitServoPositions( control.joint_state_ );
                control.publishOdometry( control.gait_vel_ );
                control.publishTwist( control.gait_vel_ );
            }

            // Release torque
            std::chrono::duration<500, std::milli>.sleep();
            servoDriver.freeServos();
            RCLCPP_INFO("Robot servos torque is now off.");

            // Locomotion is now shut off
            control.setPrevLeggedRobotActiveState( false );
        }
        // Sitting down with servo torque off. Publish jointState message every half second
        if( control.getLeggedRobotActiveState() == false && control.getPrevLeggedRobotActiveState() == false )
        {
            std::chrono::duration<500, std::milli>.sleep();
            control.publishJointStates( control.legs_, control.head_, &control.joint_state_ );
            control.publishOdometry( control.gait_vel_ );
            control.publishTwist( control.gait_vel_ );
        }
        loop_rate.sleep();
        last_time_ = current_time_;
    }
    return 0;
}