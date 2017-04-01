/*******************************************************************************
* Copyright 2017 Mathieu Rondonneau
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/


#ifndef BIOLOID_MOTION_UTILS_H_
#define BIOLOID_MOTION_UTILS_H_

#include <cmath>
#include <vector>
#include <iostream>
#include <map>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define NUMBER_OF_JOINTS 18
#define VELOCITY_VECTOR_SIZE 3
#define VELOCITY_WALKING 0.02
#define ANGLE_DELAY 2

#define WHEEL_RADIUS                    0.033     // meter
#define WHEEL_SEPARATION                0.16      // meter (0.16 / 0.287)
#define ROBOT_RADIUS                    0.078     // meter (0.078 / 0.294)

#define LEFT                            0
#define RIGHT                           1

#define MAX_LINEAR_VELOCITY             0.22   // m/s
#define MAX_ANGULAR_VELOCITY            2.84   // rad/s
#define VELOCITY_STEP                   0.01   // m/s
#define VELOCITY_LINEAR_X               0.01   // m/s
#define VELOCITY_ANGULAR_Z              0.1    // rad/s
#define SCALE_VELOCITY_LINEAR_X         1
#define SCALE_VELOCITY_ANGULAR_Z        1

#define DEG2RAD(x)                      (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                      (x * 57.2957795131)  // *180/PI

#define TORQUE_ENABLE                   1       // Value for enabling the torque of motor
#define TORQUE_DISABLE                  0       // Value for disabling the torque of motor

typedef std::map<std::string, double> joints_t;

joints_t interpolate(
		const joints_t anglesa,
		joints_t anglesb,
		double coefa);

double get_distance(
		joints_t anglesa,
		joints_t anglesb);

// Generated at compile time
constexpr double pi()
{
	return std::atan(1) * 4;
}

#endif // BIOLOID_MOTION_UTILS_H_
