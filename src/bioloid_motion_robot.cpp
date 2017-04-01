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

bioloid_robot::bioloid_robot():
	nodeh_priv_("~"),
	ns("/typea/")
{
  ROS_ASSERT(init());
}

bioloid_robot::~bioloid_robot()
{
}

std::string bioloid_robot::namespace_get(void)
{
	return ns;
}

bool bioloid_robot::init(void)
{
	ROS_ASSERT(joints.empty());

	// initialize subscribers
	joint_states_sub_  = nodeh_.subscribe(ns+"joint_states",
		1,
		&bioloid_robot::joint_callback,
		this);

	ROS_INFO_STREAM("+Waiting for joints to be populated...");

	while (ros::ok()) {
		ros::spinOnce();
		ros::Duration(1).sleep();
		if (joints.size())
			break;
	}

	if (joints.size() != NUMBER_OF_JOINTS)
	{
		ROS_INFO("%d motors connected.", (unsigned int)joints.size());
		ROS_ERROR("Number of actuator required %d", NUMBER_OF_JOINTS);
		return false;
	}

	// initialize publishers

	ROS_INFO_STREAM("+Creating joint command publishers...");
	joints_t::iterator it;

	for (it=joints.begin(); it!=joints.end(); ++it) {
	  joint_position_cmd_pub_.insert(std::pair<std::string, ros::Publisher>(
		it->first,
		nodeh_.advertise<std_msgs::Float64>(ns+it->first+"_position_controller/command", 10)));
	  ROS_INFO_STREAM(" -joint: " << it->first);
	}

	cmd_vel_pub_ = nodeh_.advertise<geometry_msgs::Twist>(ns+"cmd_vel", 10);

  return true;
}

void bioloid_robot::joint_callback(const sensor_msgs::JointStateConstPtr msg)
{
	joints_t::iterator it;

	for (uint32_t i = 0; i < msg->name.size(); i++) {

		it = joints.find(msg->name[i]);

		if (it == joints.end()) {
			joints.insert(std::pair<std::string,double>(msg->name[i], msg->position[i]));
		} else {
			it->second = msg->position[i];
		}
	}
}

void bioloid_robot::set_walk_velocity(double x, double y, double t)
{
	twist_msg.linear.x = x;
	twist_msg.linear.y = y;
	twist_msg.angular.z = t;
	cmd_vel_pub_.publish(twist_msg);
}

joints_t bioloid_robot::get_angles(void)
{
	ROS_ASSERT(joints.size() == NUMBER_OF_JOINTS);
	return joints;
}


void bioloid_robot::set_angles(joints_t angles)
{
	joints_t::iterator it;

	for (it=angles.begin(); it!=angles.end(); ++it) {
		float64_msg.data = it->second;
		//ROS_INFO_STREAM("set_angle " << it->first << ": " << it->second);
		joint_position_cmd_pub_[it->first].publish(float64_msg);
	}
}

void bioloid_robot::set_angles_slow(joints_t stop_angles, unsigned int delay)
{
	ros::Time current_time;
	start_time = ros::Time::now();
	stop_time.sec = start_time.sec + delay;
	joints_t new_joints;

	double ratio;

	while (ros::ok()) {
		current_time = ros::Time::now();
		if (current_time.sec > stop_time.sec)
			break;

		ratio = (current_time.sec - start_time.sec) / delay;
		new_joints = interpolate(stop_angles, joints, ratio);
		set_angles(new_joints);

		ros::Rate(100).sleep();
	}
}

