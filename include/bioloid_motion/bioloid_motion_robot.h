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


#ifndef BIOLOID_MOTION_H_
#define BIOLOID_MOTION_H_

#include <math.h>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

#include <bioloid_motion/bioloid_motion_utils.h>

class bioloid_robot
{
 public:
  bioloid_robot();
  ~bioloid_robot();

  void set_walk_velocity(double x, double y, double t);
  std::string namespace_get(void);
  void set_angles(joints_t angles);
  void set_angles_slow(joints_t stop_angles, unsigned int delay);
  joints_t get_angles(void);

 private:

  // ROS Time
  ros::Time start_time;
  ros::Time stop_time;

  bool init(void);

  // ROS NodeHandle
  ros::NodeHandle nodeh_;
  ros::NodeHandle nodeh_priv_;

  // ROS Topic Publishers
  ros::Publisher cmd_vel_pub_;
  std::map<std::string, ros::Publisher> joint_position_cmd_pub_;

  // ROS Topic Subscribers
  ros::Subscriber joint_states_sub_;

  // ROS Messages
  sensor_msgs::JointState joint_states_msg;
  std_msgs::Float64 float64_msg;
  geometry_msgs::Twist twist_msg;

  std::string ns;
  joints_t joints;

  // Function prototypes
  void joint_callback(const sensor_msgs::JointStateConstPtr msg);
};

#endif // BIOLOID_MOTION_H_
