/**********************************************************************************************
 * FABRIK 2D inverse kinematics solver - Version 1.0.3
 * by Henrik Söderlund <henrik.a.soderlund@gmail.com>
 *
 * Copyright (c) 2018 Henrik Söderlund
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 **********************************************************************************************/

#include "../include/legged_robot_controller/FABRIK2D.hpp"
#include <cmath>
#include <cstdlib>

/* Fabrik2D(numJoints, lengths)
 * inputs: numJoints, lengths
 *
 * creates the chain to be used for the inverse kinematics solver
 */
Fabrik2D::Fabrik2D(int numJoints, double *lengths) {

    this->numJoints = numJoints;
    createChain(lengths);

    this->tolerance = 0.0005; // 0.5mm tolerance default
}

/* createChain(lengths)
 * inputs: lengths
 *
 * length size should always be one lesser than the number of joints
 */
void Fabrik2D::createChain(double *lengths) {

    auto *chain = (Chain *) malloc(sizeof(Chain));
    chain->joints = (Joint *) malloc(sizeof(Joint) * this->numJoints);

    chain->joints[0].x = 0;
    chain->joints[0].y = 0;
    chain->joints[0].angle = 0;

    double sumLengths = 0;
    for (int i = 1; i < this->numJoints; i++) {
        sumLengths = sumLengths + lengths[i - 1];
        chain->joints[i].x = 0;
        chain->joints[i].y = sumLengths;
        chain->joints[i].angle = 0;
    }

    this->chain = chain;
}

/* solve(x, y, lengths)
 * inputs: x and y positions of target, lengths between each joint
 *
 * solves the inverse kinematics of the stored chain to reach the target
 */
bool Fabrik2D::solve(double x, double y, double *lengths) {

    // Distance between root and target (root is always 0,0)
    double dist = sqrt(x * x + y * y);

    // Total length of chain
    double totalLength = 0;
    for (int i = 0; i < this->numJoints - 1; i++) {
        totalLength = totalLength + lengths[i];
    }

    // Check whether the target is within reach
    if (dist > totalLength) {
        // The target is unreachable

        for (int i = 0; i < this->numJoints - 1; i++) {
            // Find the distance r_i between the target (x,y) and the joint i position (jx,jy)
            double jx = this->chain->joints[i].x;
            double jy = this->chain->joints[i].y;
            double r_i = distance(jx, jy, x, y);
            double lambda_i = ((double) lengths[i]) / r_i;

            // Find the new joint positions
            this->chain->joints[i + 1].x = (double) ((1 - lambda_i) * jx + lambda_i * x);
            this->chain->joints[i + 1].y = (double) ((1 - lambda_i) * jy + lambda_i * y);
        }

        return false;
    } else {
        // The target is reachable; this, set as (bx,by) the initial position of the joint i
        double bx = this->chain->joints[0].x;
        double by = this->chain->joints[0].y;

        // Check whether the distance between the end effector joint n (ex,ey) and the target is
        // greater than a tolerance
        double ex = this->chain->joints[this->numJoints - 1].x;
        double ey = this->chain->joints[this->numJoints - 1].y;
        double dif = distance(ex, ey, x, y);

        double prevDif = 0;
        double tolerance = this->tolerance;
        while (dif > tolerance) {

            if (prevDif == dif)
                tolerance *= 2;

            prevDif = dif;

            // STAGE 1: FORWARD REACHING
            // Set the end effector as target
            this->chain->joints[this->numJoints - 1].x = x;
            this->chain->joints[this->numJoints - 1].y = y;

            for (int i = this->numJoints - 2; i >= 0; i--) {

                // Find the distance r_i between the new joint position i+1 (nx,ny)
                // and the joint i (jx,jy)
                double jx = this->chain->joints[i].x;
                double jy = this->chain->joints[i].y;
                double nx = this->chain->joints[i + 1].x;
                double ny = this->chain->joints[i + 1].y;
                double r_i = distance(jx, jy, nx, ny);
                double lambda_i = ((double) lengths[i]) / r_i;

                // Find the new joint positions
                this->chain->joints[i].x = (double) ((1 - lambda_i) * nx + lambda_i * jx);
                this->chain->joints[i].y = (double) ((1 - lambda_i) * ny + lambda_i * jy);
            }

            // STAGE 2: BACKWARD REACHING
            // Set the root at its initial position
            this->chain->joints[0].x = bx;
            this->chain->joints[0].y = by;

            for (int i = 0; i < this->numJoints - 1; i++) {

                // Find the distance r_i between the new joint position i (nx,ny)
                // and the joint i+1 (jx,jy)
                double jx = this->chain->joints[i + 1].x;
                double jy = this->chain->joints[i + 1].y;
                double nx = this->chain->joints[i].x;
                double ny = this->chain->joints[i].y;
                double r_i = distance(jx, jy, nx, ny);
                double lambda_i = ((double) lengths[i]) / r_i;

                // Find the new joint positions
                this->chain->joints[i + 1].x = (double) ((1 - lambda_i) * nx + lambda_i * jx);
                this->chain->joints[i + 1].y = (double) ((1 - lambda_i) * ny + lambda_i * jy);
            }

            // Update distance between end effector and target
            ex = this->chain->joints[this->numJoints - 1].x;
            ey = this->chain->joints[this->numJoints - 1].y;
            dif = distance(ex, ey, x, y);
        }
    }


    this->chain->joints[0].angle = atan2(this->chain->joints[1].y, this->chain->joints[1].x);

    double prevAngle = this->chain->joints[0].angle;
    for (int i = 2; i <= this->numJoints - 1; i++) {
        double ax = this->chain->joints[i - 1].x;
        double ay = this->chain->joints[i - 1].y;
        double bx = this->chain->joints[i].x;
        double by = this->chain->joints[i].y;

        double aAngle = atan2(by - ay, bx - ax);

        this->chain->joints[i - 1].angle = aAngle - prevAngle;

        prevAngle = aAngle;
    }

    return true;
}

/* solve2(x, y, z, angle, offset, lengths)
 * inputs: x, y and z positions of target, desired tool angle, gripping offset and lengths between each joint
 * outputs: True if solvable, false if not solvable
 *
 * !!! tool angle is in radians !!!
 *
 * solves the inverse kinematics of the stored chain to reach the target with tool angle and gripping offset
 * introducing the z-axis, which allows a rotational base of the manipulator
 *
 * angle of the chain defines the base rotation
 *
 * the x- and y-axes define the plane and the z-axis defines the offset from the plane
 *
 * will only work for 4DOF, i.e. 4 joints or more and a rotational base
 */
bool Fabrik2D::solve2(double x, double y, double z, double toolAngle, double grippingOffset, double *lengths) {
    if (this->numJoints >= 4) {

        // Find wrist center by moving from the desired position with tool angle and link length
        double oc_x = x - (lengths[this->numJoints - 2] + grippingOffset) * cos(toolAngle);
        double oc_y = y - (lengths[this->numJoints - 2] + grippingOffset) * sin(toolAngle);

        // We solve IK from first joint to wrist center
        int tmp = this->numJoints;
        this->numJoints = this->numJoints - 1;

        bool solvable = solve(oc_x, oc_y, lengths);

        this->numJoints = tmp;

        if (solvable == true) {

            // Update the end effector position to preserve tool angle
            this->chain->joints[this->numJoints - 1].x =
                    this->chain->joints[this->numJoints - 2].x + lengths[this->numJoints - 2] * cos(toolAngle);
            this->chain->joints[this->numJoints - 1].y =
                    this->chain->joints[this->numJoints - 2].y + lengths[this->numJoints - 2] * sin(toolAngle);

            // Update angle of last joint
            this->chain->joints[0].angle = atan2(this->chain->joints[1].y, this->chain->joints[1].x);

            double prevAngle = this->chain->joints[0].angle;
            for (int i = 2; i <= this->numJoints - 1; i++) {
                double ax = this->chain->joints[i - 1].x;
                double ay = this->chain->joints[i - 1].y;
                double bx = this->chain->joints[i].x;
                double by = this->chain->joints[i].y;

                double aAngle = atan2(by - ay, bx - ax);

                this->chain->joints[i - 1].angle = aAngle - prevAngle;

                prevAngle = aAngle;
            }

            // Save tool angle
            this->chain->joints[this->numJoints - 1].angle = toolAngle;

            // Save base angle (if z different from zero)
            if (z != 0) {
                this->chain->z = z;
                this->chain->angle = atan2(z, x);
            }

        }

    }
}

/* solve(x, y, angle, lengths)
 * inputs: x and y positions of target, desired tool angle and lengths between each joint
 * outputs: True if solvable, false if not solvable
 *
 * !!! tool angle is in radians !!!
 *
 * solves the inverse kinematics of the stored chain to reach the target with tool angle
 *
 * will only work for 3DOF
 */
bool Fabrik2D::solve(double x, double y, double toolAngle, double *lengths) {
    return solve2(x, y, 0, toolAngle, 0, lengths);
}

/* solve(x, y, angle, offset, lengths)
 * inputs: x and y positions of target, desired tool angle and lengths between each joint
 * outputs: True if solvable, false if not solvable
 *
 * !!! tool angle is in radians !!!
 *
 * solves the inverse kinematics of the stored chain to reach the target with tool angle
 * and gripping offset
 *
 * will only work for 3DOF
 */
bool Fabrik2D::solve(double x, double y, double toolAngle, double grippingOffset, double *lengths) {
    return solve2(x, y, 0, toolAngle, grippingOffset, lengths);
}

/* solve2(x, y, z, lengths)
 * inputs: x, y and z positions of target, desired tool angle and lengths between each joint
 * outputs: True if solvable, false if not solvable
 *
 * !!! tool angle is in radians !!!
 *
 * solves the inverse kinematics of the stored chain to reach the target
 * introducing the z-axis, which allows a rotational base of the manipulator
 *
 * angle of the chain defines the base rotation
 *
 * the x- and y-axes define the plane and the z-axis defines the offset from the plane
 *
 * will only work for 4DOF, i.e. 4 joints or more and a rotational base
 */
bool Fabrik2D::solve2(double x, double y, double z, double *lengths) {
    double r = distance(0, 0, x, z);

    bool solvable = solve(r, y, lengths);
    if (solvable == true) {
        this->chain->z = z;
        this->chain->angle = atan2(z, x);
    }

    return solvable;
}

/* solve(x, y, z, toolAngle, lengths)
 * inputs: x, y and z positions of target, desired tool angle and lengths between each joint
 * outputs: True if solvable, false if not solvable
 *
 * !!! tool angle is in radians !!!
 *
 * solves the inverse kinematics of the stored chain to reach the target with tool angle
 * introducing the z-axis, which allows a rotational base of the manipulator
 *
 * angle of the chain defines the base rotation
 *
 * the x- and y-axes define the plane and the z-axis defines the offset from the plane
 *
 * will only work for 4DOF, i.e. 4 joints or more and a rotational base
 */
bool Fabrik2D::solve2(double x, double y, double z, double toolAngle, double *lengths) {
    return solve2(x, y, z, toolAngle, 0, lengths);
}

/* getX(joint)
 * inputs: joint number
 * outputs: x position of joint
 */
double Fabrik2D::getX(int joint) {
    if (joint >= 0 && joint < numJoints) {

        return this->chain->joints[joint].x;

    }
    return 0;
}

/* getY(joint)
 * inputs: joint number
 * outputs: y position of joint
 */
double Fabrik2D::getY(int joint) {
    if (joint >= 0 && joint < numJoints) {

        return this->chain->joints[joint].y;

    }
    return 0;
}

/* getAngle(joint)
 * inputs: joint number
 * outputs: angle (radians) of joint
 */
double Fabrik2D::getAngle(int joint) {
    if (joint >= 0 && joint < numJoints) {

        return this->chain->joints[joint].angle;

    }
    return 0;
}

/* getZ()
 * outputs: z offset of the chain from the plane
 */
double Fabrik2D::getZ() {
    return this->chain->z;
}

/* getBaseAngle()
 * outputs: base angle (radians) of chain
 */
double Fabrik2D::getBaseAngle() {
    return this->chain->angle;
}

/* setBaseAngle()
 * inputs: base angle (radians) of chain to set
 */
void Fabrik2D::setBaseAngle(double baseAngle) {
    this->chain->angle = baseAngle;
}

/* setTolerance(tolerance)
 * inputs: tolerance value
 *
 * sets the tolerance of the distance between the end effector and the target
 */
void Fabrik2D::setTolerance(double tolerance) {
    this->tolerance = tolerance;
}

/* distance(x1,y1,x2,y2)
 * inputs: coordinates
 * outputs: distance between points
 *
 * Uses euclidean distance
 */
double Fabrik2D::distance(double x1, double y1, double x2, double y2) {
    double xDiff = x2 - x1;
    double yDiff = y2 - y1;
    return sqrt(xDiff * xDiff + yDiff * yDiff);
}

/* setJoints(angles, lengths)
 * inputs: New joint angles (in radians) and list of lengths between each joint
 *
 * manually sets the joint angles and updates their position using forward kinematics
 */
void Fabrik2D::setJoints(double *angles, double *lengths) {
    double angListLen = sizeof(angles) / sizeof(double);
    double lenListLen = sizeof(lengths) / sizeof(double);
    if (angListLen == numJoints && lenListLen == numJoints - 1) {
        double accAng = angles[0];
        double accX = 0;
        double accY = 0;
        this->chain->joints[0].angle = angles[0];

        for (int i = 1; i < this->numJoints; i++) {
            accAng += angles[i];
            this->chain->joints[i].x = accX + lengths[i - 1] * cos(accAng);
            this->chain->joints[i].y = accY + lengths[i - 1] * sin(accAng);
            this->chain->joints[i].angle = angles[i];
            accX = this->chain->joints[i].x;
            accY = this->chain->joints[i].y;
        }
    }
}