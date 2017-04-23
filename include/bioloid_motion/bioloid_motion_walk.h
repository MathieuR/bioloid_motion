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


#ifndef BIOLOID_WALK_H_
#define BIOLOID_WALK_H_

#include <bioloid_motion/bioloid_motion_utils.h>
#include <bioloid_motion/bioloid_motion_robot.h>

class walk_joint_function
{
 public:

  walk_joint_function();
  walk_joint_function(const walk_joint_function& function);
  ~walk_joint_function();

  double get_y(double x);
  void mirror(void);
  void clone(const walk_joint_function* function);
  void print(void);

  void offset_set(double new_offset);
  double offset_get(void);
  void in_offset_set(double new_in_offset);
  void scale_set(double new_scale);
  double scale_get(void);
  void in_scale_set(double new_in_scale);
  double in_scale_get(void);
  void fout_below_set(double new_fout);
  walk_joint_function& operator= (const walk_joint_function& clone);
 private:

  double offset;
  double scale;
  double in_offset;
  double in_scale;

  double fout_above;
  double fout_below;
};

class walk_function
{
 public:

  walk_function();
  walk_function(double swing_scale, double step_scale, double step_offset,
  		double ankle_offset, double vx_scale, double vy_scale, double vt_scale);
  ~walk_function();

  void generate(void);
  joints_t angles_get(bool phase, double x, std::vector<double> velocity);
  void apply_velocity(joints_t &angles,
		  std::vector<double> velocity, bool phase, double x);;

 private:

  void generate_right(void);
  void print(void);

  double swing_scale;
  double step_scale;
  double step_offset;
  double ankle_offset;
  double vx_scale;
  double vy_scale;
  double vt_scale;

  // phase
  std::map<std::string, walk_joint_function> pfn;
  // anti-phase
  std::map<std::string, walk_joint_function> afn;
};

class walker
{
 public:

	walker(bioloid_robot *robot);
	~walker();
	void update(void);

 private:
	ros::NodeHandle nodeh_;
	ros::NodeHandle nodeh_priv_;
	bioloid_robot *robotp;
	bool running;
	std::vector<double> velocity_target;
	std::vector<double> velocity_current;
	walk_function function;
	joints_t ready_pos;
	ros::Subscriber cmd_vel_sub_;
	void cmd_vel_callback(const geometry_msgs::TwistConstPtr msg);
	void walk_stop(void);
	void walk_init(void);
	void walk_start(void);
	void velocity_set(double x, double y, double t);
	bool walking_get(void);
	void velocity_update(std::vector<double> target, double n);
	double get_distance_to_ready(void);

	// Name of this class
	std::string name_;

	// Frequency
	int loop_hz = 0;
};

#endif // BIOLOID_WALK_H_
