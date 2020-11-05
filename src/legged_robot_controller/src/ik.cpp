
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

#include <ik.hpp>

//==============================================================================
//  Constructor: Initialize ik variables
//==============================================================================

Ik::Ik(void) {
    //=============================================================================
    // Define Physical Measurements in m <config file>
    //=============================================================================

    rclcpp::param::get("COXA_TO_CENTER_X", COXA_TO_CENTER_X);
    rclcpp::param::get("COXA_TO_CENTER_Y", COXA_TO_CENTER_Y);
    rclcpp::param::get("INIT_COXA_ANGLE", INIT_COXA_ANGLE);
    rclcpp::param::get("INIT_FOOT_POS_X", INIT_FOOT_POS_X);
    rclcpp::param::get("INIT_FOOT_POS_Y", INIT_FOOT_POS_Y);
    rclcpp::param::get("INIT_FOOT_POS_Z", INIT_FOOT_POS_Z);
    rclcpp::param::get("LENGTHS", LENGTHS);
    rclcpp::param::get("NUMBER_OF_LEGS", NUMBER_OF_LEGS);
    rclcpp::param::get("NUMBER_OF_LEG_SEGMENTS", NUMBER_OF_LEG_SEGMENTS);
    rclcpp::param::get("ALGORITHM", ALGORITHM);
}

//=============================================================================
// getSinCos:  Get the sinus and cosinus from the angle
//=============================================================================

Trig Ik::getSinCos( double angle_rad )
{
    Trig body_trig;

    body_trig.sine = sin( angle_rad );
    body_trig.cosine = cos( angle_rad );

    return body_trig;
}

//=============================================================================
// Inverse Kinematics
//=============================================================================


void Ik::calculateIK(const legged_robot_msgs::msg::FeetPositions &feet, const legged_robot_msgs::msg::Pose &body,
                     legged_robot_msgs::msg::LegsJoints *legs) {
    double bodyIKx[8]{0, 0, 0, 0, 0, 0, 0, 0};
    double bodyIKy[8]{0, 0, 0, 0, 0, 0, 0, 0};
    double bodyIKz[8]{0, 0, 0, 0, 0, 0, 0, 0};
    double coxaIKx[8]{0, 0, 0, 0, 0, 0, 0, 0};
    double coxaIKy[8]{0, 0, 0, 0, 0, 0, 0, 0};
    double coxaIKz[8]{0, 0, 0, 0, 0, 0, 0, 0};
    double legIKx[8]{0, 0, 0, 0, 0, 0, 0, 0};
    double legIKz[8]{0, 0, 0, 0, 0, 0, 0, 0};

    if (ALGORITHM == "REACHING") {
        for (int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++) {

            //rpy&t of bodyX
            bodyIKx[leg_index] = ((COXA_TO_CENTER_X[leg_index] * cos(body.orientation.pitch * (M_PI / 180)) *
                                   cos(body.orientation.yaw * (M_PI / 180)) + COXA_TO_CENTER_Y[leg_index] *
                                                                              -cos(body.orientation.pitch *
                                                                                   (M_PI / 180)) *
                                                                              sin(body.orientation.yaw * (M_PI / 180)) +
                                   0 + sin(body.orientation.pitch * (M_PI / 180))) + bodyX);
            //rpy&t of bodyY
            bodyIKy[leg_index] = ((COXA_TO_CENTER_X[leg_index] * (sin(body.orientation.roll * (M_PI / 180)) *
                                                                  sin(body.orientation.pitch * (M_PI / 180)) *
                                                                  cos(body.orientation.yaw * (M_PI / 180)) +
                                                                  cos(body.orientation.roll * (M_PI / 180)) *
                                                                  sin(body.orientation.yaw * (M_PI / 180))) +
                                   COXA_TO_CENTER_Y[leg_index] * (sin(body.orientation.roll * (M_PI / 180)) *
                                                                  sin(body.orientation.pitch * (M_PI / 180)) *
                                                                  sin(body.orientation.yaw * (M_PI / 180)) +
                                                                  cos(body.orientation.roll * (M_PI / 180)) *
                                                                  cos(body.orientation.yaw * (M_PI / 180))) +
                                   0 * -sin(body.orientation.roll * (M_PI / 180)) * cos(body.orientation.pitch *
                                                                                        (M_PI / 180))) + bodyY);
            //rpy&t of bodyZ
            bodyIKz[leg_index] = ((COXA_TO_CENTER_X[leg_index] * (-cos(body.orientation.roll * (M_PI / 180)) *
                                                                  sin(body.orientation.pitch * (M_PI / 180)) *
                                                                  cos(body.orientation.yaw * (M_PI / 180)) +
                                                                  sin(body.orientation.roll * (M_PI / 180)) *
                                                                  sin(body.orientation.yaw * (M_PI / 180))) +
                                   COXA_TO_CENTER_Y[leg_index] * (cos(body.orientation.roll * (M_PI / 180)) *
                                                                  sin(body.orientation.pitch * (M_PI / 180)) *
                                                                  sin(body.orientation.yaw * (M_PI / 180)) +
                                                                  sin(body.orientation.roll * (M_PI / 180)) *
                                                                  cos(body.orientation.yaw * (M_PI / 180))) +
                                   0 * cos(body.orientation.roll * (M_PI / 180)) *
                                   cos(body.orientation.pitch * (M_PI / 180))));

            if (COXA_TO_CENTER_X[leg_index] < 0) {
                //rpy of CoxaX
                coxaIKx[leg_index] = (-(LENGTHS[0] * cos(atan((feet.foot[leg_index].position.y - bodyIKy[leg_index]) /
                                                              (feet.foot[leg_index].position.x -
                                                               bodyIKx[leg_index])))) - bodyIKx[leg_index]) *
                                     cos(body.orientation.pitch * (M_PI / 180)) *
                                     cos(body.orientation.yaw * (M_PI / 180)) +
                                     (-(LENGTHS[0] * sin(atan((feet.foot[leg_index].position.y - bodyIKy[leg_index]) /
                                                              (feet.foot[leg_index].position.x -
                                                               bodyIKx[leg_index])))) - bodyIKy[leg_index]) *
                                     -cos(body.orientation.pitch * (M_PI / 180)) *
                                     sin(body.orientation.yaw * (M_PI / 180)) +
                                     (bodyIKz[leg_index]) * sin(body.orientation.pitch * (M_PI / 180));
                //rpy of CoxaY
                coxaIKy[leg_index] = (-(LENGTHS[0] * cos(atan((feet.foot[leg_index].position.y - bodyIKy[leg_index]) /
                                                              (feet.foot[leg_index].position.x -
                                                               bodyIKx[leg_index])))) - bodyIKx[leg_index]) *
                                     (sin(body.orientation.roll * (M_PI / 180)) *
                                      sin(body.orientation.pitch * (M_PI / 180)) *
                                      cos(body.orientation.yaw * (M_PI / 180)) +
                                      cos(body.orientation.roll * (M_PI / 180)) *
                                      sin(body.orientation.yaw * (M_PI / 180))) + (-(LENGTHS[0] * sin(atan(
                        (feet.foot[leg_index].position.y - bodyIKy[leg_index]) /
                        (feet.foot[leg_index].position.x - bodyIKx[leg_index])))) - bodyIKy[leg_index]) *
                                                                                  (-sin(body.orientation.roll *
                                                                                        (M_PI / 180)) *
                                                                                   sin(body.orientation.pitch *
                                                                                       (M_PI / 180)) *
                                                                                   sin(body.orientation.yaw *
                                                                                       (M_PI / 180)) +
                                                                                   cos(body.orientation.roll *
                                                                                       (M_PI / 180)) *
                                                                                   cos(body.orientation.yaw *
                                                                                       (M_PI / 180))) +
                                     (bodyIKz[leg_index]) * -sin(body.orientation.roll * (M_PI / 180)) *
                                     cos(body.orientation.pitch * (M_PI / 180));
            } else {
                //rpy of CoxaX
                coxaIKx[leg_index] = ((LENGTHS[0] * cos(atan((feet.foot[leg_index].position.y - bodyIKy[leg_index]) /
                                                             (feet.foot[leg_index].position.x - bodyIKx[leg_index])))) +
                                      bodyIKx[leg_index]) *
                                     cos(body.orientation.pitch * (M_PI / 180)) *
                                     cos(body.orientation.yaw * (M_PI / 180)) +
                                     ((LENGTHS[0] * sin(atan((feet.foot[leg_index].position.y - bodyIKy[leg_index]) /
                                                             (feet.foot[leg_index].position.x - bodyIKx[leg_index])))) +
                                      bodyIKy[leg_index]) *
                                     -cos(body.orientation.pitch * (M_PI / 180)) *
                                     sin(body.orientation.yaw * (M_PI / 180)) +
                                     (bodyIKz[leg_index]) * sin(body.orientation.pitch * (M_PI / 180));
                //rpy of CoxaY
                coxaIKy[leg_index] = ((LENGTHS[0] * cos(atan((feet.foot[leg_index].position.y - bodyIKy[leg_index]) /
                                                             (feet.foot[leg_index].position.x - bodyIKx[leg_index])))) +
                                      bodyIKx[leg_index]) *
                                     (sin(body.orientation.roll * (M_PI / 180)) *
                                      sin(body.orientation.pitch * (M_PI / 180)) *
                                      cos(body.orientation.yaw * (M_PI / 180)) +
                                      cos(body.orientation.roll * (M_PI / 180)) *
                                      sin(body.orientation.yaw * (M_PI / 180))) + ((LENGTHS[0] *
                                                                                    sin(atan(
                                                                                            (feet.foot[leg_index].position.y -
                                                                                             bodyIKy[leg_index]) /
                                                                                            (feet.foot[leg_index].position.x -
                                                                                             bodyIKx[leg_index])))) +
                                                                                   bodyIKy[leg_index]) *
                                                                                  (-sin(body.orientation.roll *
                                                                                        (M_PI / 180)) *
                                                                                   sin(body.orientation.pitch *
                                                                                       (M_PI / 180)) *
                                                                                   sin(body.orientation.yaw *
                                                                                       (M_PI / 180)) +
                                                                                   cos(body.orientation.roll *
                                                                                       (M_PI / 180)) *
                                                                                   cos(body.orientation.yaw *
                                                                                       (M_PI / 180))) +
                                     (bodyIKz[leg_index]) *
                                     -sin(body.orientation.roll * (M_PI / 180)) *
                                     cos(body.orientation.pitch * (M_PI / 180));
            }

            //rpy of CoxaZ
            coxaIKz[leg_index] = (((LENGTHS[0] * cos(atan((feet.foot[leg_index].position.y - bodyIKy[leg_index]) /
                                                          (feet.foot[leg_index].position.x - bodyIKx[leg_index])))) +
                                   bodyIKx[leg_index]) *
                                  (-cos(body.orientation.roll * (M_PI / 180)) *
                                   sin(body.orientation.pitch * (M_PI / 180)) *
                                   cos(body.orientation.yaw * (M_PI / 180)) +
                                   sin(body.orientation.roll * (M_PI / 180)) *
                                   sin(body.orientation.yaw * (M_PI / 180))) + ((LENGTHS[0] *
                                                                                 sin(atan(
                                                                                         (feet.foot[leg_index].position.y -
                                                                                          bodyIKy[leg_index]) /
                                                                                         (feet.foot[leg_index].position.x -
                                                                                          bodyIKx[leg_index])))) +
                                                                                bodyIKy[leg_index]) *
                                                                               (cos(body.orientation.roll *
                                                                                    (M_PI / 180)) *
                                                                                sin(body.orientation.pitch *
                                                                                    (M_PI / 180)) *
                                                                                sin(body.orientation.yaw *
                                                                                    (M_PI / 180)) +
                                                                                sin(body.orientation.roll *
                                                                                    (M_PI / 180)) *
                                                                                cos(body.orientation.yaw *
                                                                                    (M_PI / 180))) +
                                  (bodyIKz[leg_index]) *
                                  cos(body.orientation.roll * (M_PI / 180)) *
                                  cos(body.orientation.pitch * (M_PI / 180))) -
                                 STANDING_BODY_HEIGHT;

            //output for leg IK solver. Leg IK is decoupled from y so we need to convert x,y to hypotenuse to use as x in leg frame.
            legIKx[leg_index] = sqrt(pow(feet.foot[leg_index].position.x - bodyIKx[leg_index], 2) +
                                     pow(feet.foot[leg_index].position.y - bodyIKy[leg_index], 2)) - LENGTHS[1];
            legIKz[leg_index] = coxaIKz[leg_index];

            // For a 3DOF arm, we have 2 links and 3+1 joints,
            // where the end effector counts as one joint in this case.
            Fabrik2D Fabrik2D(NUMBER_OF_LEG_SEGMENTS, LENGTHS);

            // Solve leg 1 IK,
            Fabrik2D.solve(legIKx[leg_index], legIKz[leg_index], LENGTHS);
            // Get the angles (in radians [-pi,pi])
            legs->leg[leg_index].coxa = atan((feet.foot[leg_index].position.y - bodyIKy[leg_index]) /
                                             (feet.foot[leg_index].position.x - bodyIKx[leg_index]));
            legs->leg[leg_index].femur = Fabrik2D.getAngle(0);
            legs->leg[leg_index].tibia = Fabrik2D.getAngle(1);
            legs->leg[leg_index].tarsus = Fabrik2D.getAngle(2);

        }
    } else if (ALGORITHM == "STANDARD") {
        double sign = -1.0;
        for (int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++) {
            if (leg_index <= 2) {
                sign = -1.0;
            } else {
                sign = 1.0;
            }

            // First calculate sinus and co-sinus for each angular axis
            Trig A = getSinCos(body.orientation.yaw + feet.foot[leg_index].orientation.yaw);
            Trig B = getSinCos(body.orientation.pitch);
            Trig G = getSinCos(body.orientation.roll);

            // Calculating totals from the feet to center of the body
            double cpr_x = feet.foot[leg_index].position.x + body.position.x - INIT_FOOT_POS_X[leg_index] -
                           COXA_TO_CENTER_X[leg_index];

            double cpr_y = feet.foot[leg_index].position.y +
                           sign * (body.position.y + INIT_FOOT_POS_Y[leg_index] + COXA_TO_CENTER_Y[leg_index]);

            double cpr_z =
                    feet.foot[leg_index].position.z + body.position.z + TARSUS_LENGTH - INIT_FOOT_POS_Z[leg_index];


            // Calculation of angular matrix of body (Tait-Bryan angles Z, Y, X)
            // http://en.wikipedia.org/wiki/Euler_angles
            double body_pos_x = cpr_x - ((cpr_x * A.cosine * B.cosine) +
                                         (cpr_y * A.cosine * B.sine * G.sine - cpr_y * G.cosine * A.sine) +
                                         (cpr_z * A.sine * G.sine + cpr_z * A.cosine * G.cosine * B.sine)
            );

            double body_pos_y = cpr_y - ((cpr_x * B.cosine * A.sine) +
                                         (cpr_y * A.cosine * G.cosine + cpr_y * A.sine * B.sine * G.sine) +
                                         (cpr_z * G.cosine * A.sine * B.sine - cpr_z * A.cosine * G.sine)
            );

            double body_pos_z =
                    cpr_z - ((-cpr_x * B.sine) + (cpr_y * B.cosine * G.sine) + (cpr_z * B.cosine * G.cosine));

            // Calculate foot position
            double feet_pos_x =
                    -INIT_FOOT_POS_X[leg_index] + body.position.x - body_pos_x + feet.foot[leg_index].position.x;
            double feet_pos_y = INIT_FOOT_POS_Y[leg_index] +
                                sign * (body.position.y - body_pos_y + feet.foot[leg_index].position.y);
            double feet_pos_z = INIT_FOOT_POS_Z[leg_index] - TARSUS_LENGTH + body.position.z - body_pos_z -
                                feet.foot[leg_index].position.z;

            // Length between the Root and Foot Position ...Pythagorean theorem
            double femur_to_tarsus = sqrt(pow(feet_pos_x, 2) + pow(feet_pos_y, 2)) - COXA_LENGTH;

            if (std::abs(femur_to_tarsus) > (FEMUR_LENGTH + TIBIA_LENGTH)) {
                RCLCPP_FATAL(node->get_logger(), "IK Solver cannot solve a foot position that is not within leg reach!!!");
                RCLCPP_FATAL(node->get_logger(), "Shutting down so configuration can be fixed!!!");
                RCLCPP::shutdown();
                break;
            }

            // Length of the sides of the triangle formed by the femur, tibia and tarsus joints.
            double side_a = FEMUR_LENGTH;
            double side_a_sqr = pow(FEMUR_LENGTH, 2);

            double side_b = TIBIA_LENGTH;
            double side_b_sqr = pow(TIBIA_LENGTH, 2);

            double side_c = sqrt(pow(femur_to_tarsus, 2) + pow(feet_pos_z, 2));
            double side_c_sqr = pow(side_c, 2);

            // We are using the law of cosines on the triangle formed by the femur, tibia and tarsus joints.
            double angle_b = acos((side_a_sqr - side_b_sqr + side_c_sqr) / (2.0 * side_a * side_c));
            double angle_c = acos((side_a_sqr + side_b_sqr - side_c_sqr) / (2.0 * side_a * side_b));

            // Angle of line between the femur and Tarsus joints with respect to feet_pos_z.
            double theta = atan2(femur_to_tarsus, feet_pos_z);

            // Resulting joint angles in radians.
            legs->leg[leg_index].coxa = atan2(feet_pos_x, feet_pos_y) + INIT_COXA_ANGLE[leg_index];
            legs->leg[leg_index].femur = (PI / 2) - (theta + angle_b);
            legs->leg[leg_index].tibia = (PI / 2) - angle_c;
            legs->leg[leg_index].tarsus = legs->leg[leg_index].femur + legs->leg[leg_index].tibia;
        }
    }
}