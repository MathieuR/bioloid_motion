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

#include <bioloid_motion/bioloid_motion_robot.h>


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "walker_demo");


  ROS_INFO_STREAM("Instantiating Bioloid Robot");
  bioloid_robot robot;
  ros::Duration(1).sleep();

  ROS_INFO_STREAM("Walker Demo Starting");

  ROS_INFO_STREAM("Velocity: " << 0.2);
  robot.set_walk_velocity(0.2, 0, 0);
  ros::Duration(10).sleep();

  ROS_INFO_STREAM("Velocity: " << 1);
  robot.set_walk_velocity(1, 0, 0);
  ros::Duration(10).sleep();

  robot.set_walk_velocity(0, 0, 0);

  ROS_INFO_STREAM("Walker Demo Finished");
  return 0;
}
