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

#ifndef FABRIK2D_hpp
#define FABRIK2D_hpp


class Fabrik2D {
public:
    /* Fabrik2D(numJoints, lengths)
     * inputs: numJoints, lengths
     *
     * creates the chain to be used for the inverse kinematics solver
     */
    Fabrik2D(int numJoints, double *lengths);

    /* solve(x, y, lengths)
     * inputs: x and y positions of target, lengths between each joint
     * outputs: True if solvable, false if not solvable
     *
     * solves the inverse kinematics of the stored chain to reach the target
     */
    bool solve(double x, double y, double *lengths);

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
    bool solve(double x, double y, double toolAngle, double *lengths);

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
    bool solve(double x, double y, double toolAngle, double grippingOffset, double *lengths);

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
    bool solve2(double x, double y, double z, double *lengths);

    /* solve2(x, y, z, toolAngle, lengths)
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
    bool solve2(double x, double y, double z, double toolAngle, double *lengths);

    /* solve2(x, y, z, angle, offset, lengths)
     * inputs: x, y and z positions of target, desired tool angle, gripping offset and lengths
               between each joint
     * outputs: True if solvable, false if not solvable
     *
     * !!! tool angle is in radians !!!
     *
     * solves the inverse kinematics of the stored chain to reach the target with tool angle
     * and gripping offset
     * introducing the z-axis, which allows a rotational base of the manipulator
     *
     * angle of the chain defines the base rotation
     *
     * the x- and y-axes define the plane and the z-axis defines the offset from the plane
     *
     * will only work for 4DOF, i.e. 4 joints or more and a rotational base
     */
    bool solve2(double x, double y, double z, double toolAngle, double grippingOffset, double *lengths);

    /* getX(joint)
     * inputs: joint number
     * outputs: x position of joint
     */
    double getX(int joint);

    /* getY(joint)
     * inputs: joint number
     * outputs: y position of joint
     */
    double getY(int joint);

    /* getZ()
     * outputs: z offset of the chain from the plane
     */
    double getZ();

    /* getAngle(joint)
     * inputs: joint number
     * outputs: angle (radians) of joint
     */
    double getAngle(int joint);

    /* getBaseAngle()
     * outputs: base angle (radians) of chain
     */
    double getBaseAngle();

    /* setBaseAngle()
     * inputs: base angle (radians) of chain to set
     */
    void setBaseAngle(double baseAngle);

    /* setTolerance(tolerance)
     * inputs: tolerance value
     *
     * sets the tolerance of the distance between the end effector and the target
     */
    void setTolerance(double tolerance);

    /* setJoints(angles, lengths)
     * inputs: New joint angles(in radians) and list of lengths between each joint
     *
     * manually sets the joint angles and updates their position using forward kinematics
     */
    void setJoints(double *angles, double *lengths);

private:

    // Joint struct
    typedef struct {
        double x; // x position of joint relative to origin
        double y; // y position of joint relative to origin
        double angle; // angle of joint (if the joint has adjacent joints or origin)
    } Joint;

    // Chain struct
    typedef struct {
        Joint *joints; // list of joints
        double z;  // z position defining the offset of the chain from the plane
        double angle; // base (plane) rotation
    } Chain;

    // Number of joints in the chain
    int numJoints;
    // Tolerance of distance between end effector and target
    double tolerance;
    // The chain containing joints
    Chain *chain{};

    /* createChain(lengths)
     * inputs: lengths
     *
     * length size should always be one lesser than the number of joints
     */
    void createChain(double *lengths);

    /* distance(x1,y1,x2,y2)
     * inputs: coordinates
     * outputs: distance between points
     *
     * Uses euclidean distance
     */
    double distance(double x1, double y1, double x2, double y2);
};

#endif