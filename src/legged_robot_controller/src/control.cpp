
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

#include <control.hpp>

static const double PI = atan(1.0)*4.0;

//==============================================================================
// Constructor
//==============================================================================

Control::Control( void )
{
    rclcpp::param::get( "NUMBER_OF_LEGS", NUMBER_OF_LEGS );
    rclcpp::param::get( "NUMBER_OF_LEG_SEGMENTS", NUMBER_OF_LEG_JOINTS );
    rclcpp::param::get( "NUMBER_OF_HEAD_SEGMENTS", NUMBER_OF_HEAD_JOINTS );
    rclcpp::param::get( "BODY_MAX_ROLL", BODY_MAX_ROLL );
    rclcpp::param::get( "BODY_MAX_PITCH", BODY_MAX_PITCH );
    rclcpp::param::get( "BODY_MAX_YAW", BODY_MAX_YAW );
    rclcpp::param::get( "HEAD_MAX_YAW", HEAD_MAX_YAW );
    rclcpp::param::get( "HEAD_MAX_PITCH", HEAD_MAX_PITCH );
    rclcpp::param::get( "STANDING_BODY_HEIGHT", STANDING_BODY_HEIGHT );
    rclcpp::param::get( "SERVOS", SERVOS );
    rclcpp::param::get( "MAX_BODY_ROLL_COMP", MAX_BODY_ROLL_COMP );
    rclcpp::param::get( "MAX_BODY_PITCH_COMP", MAX_BODY_PITCH_COMP );
    rclcpp::param::get( "COMPENSATE_INCREMENT", COMPENSATE_INCREMENT );
    rclcpp::param::get( "COMPENSATE_TO_WITHIN", COMPENSATE_TO_WITHIN );
    rclcpp::param::get( "MASTER_LOOP_RATE", MASTER_LOOP_RATE );
    rclcpp::param::get( "VELOCITY_DIVISION", VELOCITY_DIVISION );
    current_time_odometry_ = rclcpp::Time::now();
    last_time_odometry_ = rclcpp::Time::now();
    current_time_cmd_vel_ = rclcpp::Time::now();
    last_time_cmd_vel_ = rclcpp::Time::now();
    // Find out how many servos/joints we have
    SERVO_COUNT = (NUMBER_OF_LEGS * NUMBER_OF_LEG_JOINTS) + NUMBER_OF_HEAD_JOINTS
    joint_state_.name.resize( SERVO_COUNT );
    joint_state_.position.resize( SERVO_COUNT );
    servo_names_.resize( SERVO_COUNT );
    servo_orientation_.resize( SERVO_COUNT );
    for( int i = 0; i < SERVO_COUNT; i++ )
    {
        int j = i+1;
        std::string(k)  = std::to_string(j);
        rclcpp::param::get( ("/SERVOS/" + static_cast<std::string>( k ) + "/name"), servo_names_[i] );
        rclcpp::param::get( ("/SERVOS/" + static_cast<std::string>( k ) + "/sign"), servo_orientation_[i] );
    }
    prev_legged_robot_state_ = false;
    legged_robot_state_ = false;
    imu_init_stored_ = false;
    imu_override_.data = false;
    imu_roll_lowpass_ = 0.0;
    imu_pitch_lowpass_ = 0.0;
    imu_yaw_lowpass_ = 0.0;
    imu_roll_init_ = 0.0;
    imu_pitch_init_ = 0.0;

    // Topics we are subscribing
    cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>( "/cmd_vel", 1, &Control::cmd_velCallback, this );
    body_scalar_sub_ = nh_.subscribe<geometry_msgs::AccelStamped>( "/body_scalar", 1, &Control::bodyCallback, this );
    head_scalar_sub_ = nh_.subscribe<geometry_msgs::AccelStamped>( "/head_scalar", 1, &Control::headCallback, this );
    state_sub_ = nh_.subscribe<std_msgs::Bool>( "/state", 1, &Control::stateCallback, this );
    imu_override_sub_ = nh_.subscribe<std_msgs::Bool>( "/imu/imu_override", 1, &Control::imuOverrideCallback, this );
    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>( "/imu/data", 1, &Control::imuCallback, this );

    // Topics we are publishing
    sounds_pub_ = nh_.advertise<legged_robot_msgs::Sounds>( "/sounds", 10 );
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>( "/joint_states", 10 );
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>( "/odometry/calculated", 50 );
    twist_pub_ = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>( "/twist", 50 );

    // Send service request to the imu to re-calibrate
    imu_calibrate_ = nh_.serviceClient<std_srvs::Empty>("/imu/calibrate");
    imu_calibrate_.call( calibrate_ );
}

//==============================================================================
// Getter and Setters
//==============================================================================

void Control::setLeggedRobotActiveState( bool state )
{
    legged_robot_state_ = state;
}

bool Control::getLeggedRobotActiveState( void )
{
    return legged_robot_state_;
}

void Control::setPrevLeggedRobotActiveState( bool state )
{
    prev_legged_robot_state_ = state;
}

bool Control::getPrevLeggedRobotActiveState( void )
{
    return prev_legged_robot_state_;
}

//==============================================================================
// Odometry Publisher
//==============================================================================
void Control::publishOdometry( const geometry_msgs::msg::Twist &gait_vel )
{
    // compute odometry in a typical way given the velocities of the robot

    // calculate time elapsed
    current_time_odometry_ = rclcpp::Time::now();
    double dt = ( current_time_odometry_ - last_time_odometry_ ).toSec();

    double vth = gait_vel.angular.z;
    double delta_th = vth * dt;
    pose_th_ += delta_th;

    double vx = gait_vel.linear.x;
    double vy = gait_vel.linear.y;
    double delta_x = ( vx * cos( pose_th_ ) - vy * sin( pose_th_ ) ) * dt;
    double delta_y = ( vx * sin( pose_th_ ) + vy * cos( pose_th_ ) ) * dt;
    pose_x_ += delta_x;
    pose_y_ += delta_y;

    // since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::msg::Quaternion odom_quat = tf::createQuaternionMsgFromYaw( pose_th_ );

    // first, we'll publish the transform over tf
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_odometry_;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = pose_x_;
    odom_trans.transform.translation.y = pose_y_;
    odom_trans.transform.translation.z = body_.position.z;
    odom_trans.transform.rotation = odom_quat;

    // Uncomment odom_broadcaster to send the transform. Only used if debugging calculated odometry.
    // odom_broadcaster.sendTransform( odom_trans );

    // next, we'll publish the odometry message over ROS
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time_odometry_;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    // set the position
    odom.pose.pose.position.x = pose_x_;
    odom.pose.pose.position.y = pose_y_;
    odom.pose.pose.position.z = body_.position.z;
    odom.pose.pose.orientation = odom_quat;

    odom.pose.covariance[0] = 0.00001;  // x
    odom.pose.covariance[7] = 0.00001;  // y
    odom.pose.covariance[14] = 0.00001; // z
    odom.pose.covariance[21] = 1000000000000.0; // rot x
    odom.pose.covariance[28] = 1000000000000.0; // rot y
    odom.pose.covariance[35] = 0.001; // rot z

    // set the velocity
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;
    odom.twist.covariance = odom.pose.covariance; // needed?

    odom_pub_.publish( odom );
    last_time_odometry_ = current_time_odometry_;
}

//==============================================================================
// Twist Publisher
//==============================================================================
void Control::publishTwist( const geometry_msgs::msg::Twist &gait_vel )
{
    geometry_msgs::msg::TwistWithCovarianceStamped twistStamped;
    twistStamped.header.stamp = rclcpp::Time::now();
    twistStamped.header.frame_id = "odom";

    twistStamped.twist.twist.linear.x = gait_vel.linear.x;
    twistStamped.twist.twist.linear.y = gait_vel.linear.y;
    twistStamped.twist.twist.angular.z = gait_vel.angular.z;

    twistStamped.twist.covariance[0] = 0.00001;  // x
    twistStamped.twist.covariance[7] = 0.00001;  // y
    twistStamped.twist.covariance[14] = 0.00001; // z
    twistStamped.twist.covariance[21] = 1000000000000.0; // rot x
    twistStamped.twist.covariance[28] = 1000000000000.0; // rot y
    twistStamped.twist.covariance[35] = 0.001; // rot z

    twist_pub_.publish( twistStamped );
}

//==============================================================================
// Joint State Publisher
//==============================================================================
void Control::publishJointStates( const legged_robot_msgs::msg::LegsJoints &legs, const legged_robot_msgs::msg::RPY &head, sensor_msgs::msg::JointState *joint_state )
{
    joint_state->header.stamp = rclcpp::Time::now();
    int i = 0;
    for( int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++ )
    {
        joint_state->name[i] = servo_names_[i];
        joint_state->position[i] = servo_orientation_[i] * legs.leg[leg_index].coxa;
        i++;
        joint_state->name[i] = servo_names_[i];
        joint_state->position[i] = servo_orientation_[i] * legs.leg[leg_index].femur;
        i++;
        joint_state->name[i] = servo_names_[i];
        joint_state->position[i] = servo_orientation_[i] * legs.leg[leg_index].tibia;
        i++;
        if ( NUMBER_OF_LEG_JOINTS == 4 )
        {
            joint_state->name[i] = servo_names_[i];
            joint_state->position[i] = servo_orientation_[i] * legs.leg[leg_index].tarsus;
            i++;
        }
    }
    switch( NUMBER_OF_HEAD_JOINTS )
    {
        case 1:
            joint_state->name[i] = servo_names_[i];
            joint_state->position[i] = head_.yaw;
            i++;
            break;
        case 2:
            joint_state->name[i] = servo_names_[i];
            joint_state->position[i] = head_.yaw;
            i++;
            joint_state->name[i] = servo_names_[i];
            joint_state->position[i] = head_.pitch;
            i++;
            break;
        default:
            break;
    }
    joint_state_pub_.publish( *joint_state );
}

//==============================================================================
// Topics we subscribe to
//==============================================================================
//==============================================================================
// cmd_vel callback
//==============================================================================

void Control::cmd_velCallback( const geometry_msgs::msg::TwistConstPtr &cmd_vel_msg )
{
    cmd_vel_incoming_.linear.x = cmd_vel_msg->linear.x;
    cmd_vel_incoming_.linear.y = cmd_vel_msg->linear.y;
    cmd_vel_incoming_.angular.z = cmd_vel_msg->angular.z;
}

//==============================================================================
// Override IMU and manipulate body orientation callback
//==============================================================================

void Control::bodyCallback( const geometry_msgs::msg::AccelStampedConstPtr &body_scalar_msg )
{
    rclcpp::Time current_time = rclcpp::Time::now();
    double time_delta = current_time.toSec() - body_scalar_msg->header.stamp.toSec();
    if ( time_delta < 1.0 ) // Don't move if timestamp is stale over a second
    {
        if( imu_override_.data == true )
        {
            // To prevent violent motion changes the values are ran through a low pass filter
            body_.orientation.roll = ( body_scalar_msg->accel.angular.x * BODY_MAX_ROLL )* 0.01 + ( body_.orientation.roll * ( 1.0 - 0.01 ) );
            body_.orientation.pitch = ( body_scalar_msg->accel.angular.y * BODY_MAX_PITCH ) * 0.01 + ( body_.orientation.pitch * ( 1.0 - 0.01 ) );
            body_.orientation.yaw = ( body_scalar_msg->accel.angular.z * BODY_MAX_YAW ) * 0.01 + ( body_.orientation.yaw * ( 1.0 - 0.01 ) );
        }
    }
}

//==============================================================================
// Pan head callback
//==============================================================================

void Control::headCallback( const geometry_msgs::msg::AccelStampedConstPtr &head_scalar_msg )
{
    rclcpp::Time current_time = rclcpp::Time::now();
    double time_delta = current_time.toSec() - head_scalar_msg->header.stamp.toSec();
    if ( time_delta < 1.0 ) // Don't move if timestamp is stale over a second
    {
        head_.yaw = head_scalar_msg->accel.angular.z * HEAD_MAX_YAW;
        head_.pitch = head_scalar_msg->accel.angular.y * HEAD_MAX_PITCH;
    }
}

//==============================================================================
// Active state callback - currently simple on/off - stand/sit
//==============================================================================

void Control::stateCallback( const std_msgs::msg::BoolConstPtr &state_msg )
{
    if(state_msg->data == true )
    {
        if( getLeggedRobotActiveState() == false )
        {
            // Activating legged_robot
            body_.position.y = 0.0;
            body_.position.z = 0.0;
            body_.position.x = 0.0;
            body_.orientation.pitch = 0.0;
            body_.orientation.yaw = 0.0;
            body_.orientation.roll = 0.0;
            setLeggedRobotActiveState( true );
            sounds_.stand = true;
            sounds_pub_.publish( sounds_ );
            sounds_.stand = false;
        }
    }

    if( state_msg->data == false )
    {
        if( getLeggedRobotActiveState() == true )
        {
            // Sit down legged_robot
            body_.position.y = 0.0;
            body_.position.x = 0.0;
            body_.orientation.pitch = 0.0;
            body_.orientation.yaw = 0.0;
            body_.orientation.roll = 0.0;
            setLeggedRobotActiveState( false );
            sounds_.shut_down = true;
            sounds_pub_.publish( sounds_ );
            sounds_.shut_down = false;
        }
    }
}

//==============================================================================
// IMU override callback
//==============================================================================

void Control::imuOverrideCallback( const std_msgs::msg::BoolConstPtr &imu_override_msg )
{
    imu_override_.data = imu_override_msg->data;
}

//==============================================================================
// IMU callback to auto-level body if on non level ground
//==============================================================================

void Control::imuCallback( const sensor_msgs::msg::ImuConstPtr &imu_msg )
{
    if( imu_override_.data == false )
    {
        const geometry_msgs::msg::Vector3 &lin_acc = imu_msg->linear_acceleration;

        if( imu_init_stored_ == false )
        {
            imu_roll_init_ = atan2( lin_acc.x, sqrt( lin_acc.y * lin_acc.y + lin_acc.z * lin_acc.z ) );
            imu_pitch_init_ = -atan2( lin_acc.y, lin_acc.z );
            imu_pitch_init_ = ( imu_pitch_init_ >= 0.0 ) ? ( PI - imu_pitch_init_ ) : ( -imu_pitch_init_ - PI );
            imu_init_stored_ = true;
        }

        // low-pass filter to smooth out noise
        imu_roll_lowpass_ = lin_acc.x * 0.01 + ( imu_roll_lowpass_ * ( 1.0 - 0.01 ) );
        imu_pitch_lowpass_ = lin_acc.y * 0.01 + ( imu_pitch_lowpass_ * ( 1.0 - 0.01 ) );
        imu_yaw_lowpass_ = lin_acc.z * 0.01 + ( imu_yaw_lowpass_ * ( 1.0 - 0.01 ) );

        double imu_roll = atan2( imu_roll_lowpass_, sqrt( imu_pitch_lowpass_ * imu_pitch_lowpass_ + imu_yaw_lowpass_ * imu_yaw_lowpass_ ) );
        double imu_pitch = -atan2( imu_pitch_lowpass_, imu_yaw_lowpass_ );
        imu_pitch = ( imu_pitch >= 0.0 ) ? ( PI - imu_pitch ) : ( -imu_pitch - PI );

        double imu_roll_delta = imu_roll_init_ - imu_roll;
        double imu_pitch_delta = imu_pitch_init_ - imu_pitch;

        if( ( std::abs( imu_roll_delta ) > MAX_BODY_ROLL_COMP ) || ( std::abs( imu_pitch_delta ) > MAX_BODY_PITCH_COMP ) )
        {
            sounds_.auto_level = true;
            sounds_pub_.publish( sounds_ );
            sounds_.auto_level = false;
        }

        if( imu_roll_delta < -COMPENSATE_TO_WITHIN )
        {
            if( body_.orientation.roll < MAX_BODY_ROLL_COMP )
            {
                //body_.orientation.roll = body_.orientation.roll + COMPENSATE_INCREMENT;
            }
        }

        if( imu_roll_delta > COMPENSATE_TO_WITHIN )
        {
            if( body_.orientation.roll > -MAX_BODY_ROLL_COMP )
            {
                //body_.orientation.roll = body_.orientation.roll - COMPENSATE_INCREMENT;
            }
        }

        if( imu_pitch_delta < -COMPENSATE_TO_WITHIN )
        {
            if( body_.orientation.pitch < MAX_BODY_PITCH_COMP )
            {
                //body_.orientation.pitch = body_.orientation.pitch + COMPENSATE_INCREMENT;
            }
        }

        if( imu_pitch_delta > COMPENSATE_TO_WITHIN )
        {
            if( body_.orientation.pitch > -MAX_BODY_PITCH_COMP )
            {
                //body_.orientation.pitch = body_.orientation.pitch - COMPENSATE_INCREMENT;
            }
        }
    }
}

//==============================================================================
// Partitions up the cmd_vel to the speed of the loop rate
//==============================================================================

void Control::partitionCmd_vel( geometry_msgs::msg::Twist *cmd_vel )
{
    // Instead of getting delta time we are calculating with a static division
    double dt = VELOCITY_DIVISION;

    double delta_th = cmd_vel_incoming_.angular.z * dt;
    double delta_x = ( cmd_vel_incoming_.linear.x * cos( delta_th ) - cmd_vel_incoming_.linear.y * sin( delta_th ) ) * dt;
    double delta_y = ( cmd_vel_incoming_.linear.x * sin( delta_th ) + cmd_vel_incoming_.linear.y * cos( delta_th ) ) * dt;
    cmd_vel->linear.x = delta_x;
    cmd_vel->linear.y = delta_y;
    cmd_vel->angular.z = delta_th;
}

