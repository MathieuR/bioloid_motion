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

#include <cmath>
#include <iostream>
#include <bioloid_motion/bioloid_motion_walk.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

walk_joint_function::walk_joint_function():
	offset(0),
	scale(1),
	in_offset(0),
	in_scale(1),
	fout_above(0),
	fout_below(0)
{

}

walk_joint_function::walk_joint_function(const walk_joint_function& function)
{
	offset = function.offset;
	scale = function.scale;
	in_offset = function.in_offset;
	in_scale = function.in_scale;
	fout_below = function.fout_below;
	fout_above = function.fout_above;
}

walk_joint_function::~walk_joint_function()
{

}

double walk_joint_function::get_y(double x)
{
	double f = sin(in_offset + in_scale * x);
	f = offset + scale * f;
	if (fout_above)
		if (f > fout_above)
			f = fout_above;
	if (fout_below)
		if (f < fout_below)
			f = fout_below;

	return f;
}

void walk_joint_function::clone(const walk_joint_function* function)
{
	offset = function->offset;
	scale = function->scale;
	in_offset = function->in_offset;
	in_scale = function->in_scale;
	fout_below = function->fout_below;
	fout_above = function->fout_above;
}

void walk_joint_function::mirror(void)
{
	offset *= -1;
	scale *= -1;
	double tmp = fout_below * -1;
	fout_below = fout_above * -1;
	fout_above = tmp;
}

void walk_joint_function::print(void)
{
	ROS_INFO_STREAM(
	"y=" << offset << "+" << scale << "* sin(" << in_offset << "+" << in_scale << ")");
}

void walk_joint_function::offset_set(double new_offset)
{
	offset = new_offset;
}

double walk_joint_function::offset_get(void)
{
	return offset;
}

void walk_joint_function::in_offset_set(double new_in_offset)
{
	in_offset = new_in_offset;
}

void walk_joint_function::scale_set(double new_scale)
{
	scale = new_scale;
}

double walk_joint_function::scale_get(void)
{
	return scale;
}

void walk_joint_function::in_scale_set(double new_in_scale)
{
	in_scale = new_in_scale;
}

double walk_joint_function::in_scale_get(void)
{
	return in_scale;
}

void walk_joint_function::fout_below_set(double new_fout)
{
	fout_below = new_fout;
}

walk_joint_function& walk_joint_function::operator= (const walk_joint_function& clone)
{
	scale = clone.scale;
	in_scale = clone.in_scale;
	offset = clone.offset;
	in_offset = clone.in_offset;
	fout_below = clone.fout_below;
	fout_above = clone.fout_above;

	return *this;
}


walk_function::walk_function():
	swing_scale(0.4),
	step_scale(0.3),
	step_offset(0.55),
	ankle_offset(0),
	vx_scale(0.5),
	vy_scale(0.5),
	vt_scale(0.4)
{
	generate();
}

walk_function::walk_function(
		double swing_scale, double step_scale, double step_offset,
		double ankle_offset, double vx_scale, double vy_scale, double vt_scale):
	swing_scale(swing_scale),
	step_scale(step_scale),
	step_offset(step_offset),
	ankle_offset(ankle_offset),
	vx_scale(vx_scale),
	vy_scale(vy_scale),
	vt_scale(vt_scale)
{
	generate();
}

walk_function::~walk_function()
{

}

void walk_function::generate(void)
{
	walk_joint_function f1;
	walk_joint_function f2;
	walk_joint_function f3;
	walk_joint_function f33;
	walk_joint_function f4;
	walk_joint_function f5;
	walk_joint_function f6;
	walk_joint_function f7;


	// f1 = thigh1=ankle1=L=R in phase
	f1.in_scale_set(pi());
	f1.scale_set(-1 * swing_scale);
	f1.fout_below_set(f1.scale_get()/2);
	pfn.insert(std::pair<std::string, walk_joint_function>("l_ankle_lateral_joint", f1));
	pfn.insert(std::pair<std::string, walk_joint_function>("l_hip_lateral_joint", f1));

	ROS_INFO_STREAM("f1.scale: " << f1.scale_get());
	ROS_INFO_STREAM("pfn[l_ankle_lateal_joint].scale: " << pfn["l_ankle_lateral_joint"].scale_get());

	// f2 = mirror f1 in antiphase
	f2.clone(&f1);
	f2.mirror();
	afn.insert(std::pair<std::string,walk_joint_function>("l_ankle_lateral_joint", f2));
	afn.insert(std::pair<std::string,walk_joint_function>("l_hip_lateral_joint", f2));

	ROS_INFO_STREAM("pfn[l_ankle_lateral_joint].scale: " << pfn["l_ankle_lateral_joint"].scale_get());
	ROS_INFO_STREAM("afn[l_ankle_lateral_joint].scale: " << afn["l_ankle_lateral_joint"].scale_get());

	ROS_ASSERT(pfn["l_ankle_lateral_joint"].scale_get() == -1*afn["l_ankle_lateral_joint"].scale_get());

	// f3
	f3.in_scale_set(pi());
	f3.scale_set(step_scale);
	f3.offset_set(step_offset);
	pfn.insert(std::pair<std::string,walk_joint_function>("l_hip_swing_joint", f3));
	f33.clone(&f3);
	f33.mirror();
	f33.offset_set(f33.offset_get() + ankle_offset);
	pfn.insert(std::pair<std::string,walk_joint_function>("l_ankle_swing_joint", f33));

	// f4
	f4.clone(&f3);
	f4.mirror();
	f4.offset_set(f4.offset_get() * 2);
	f4.scale_set(f4.scale_get() * 2);
	pfn.insert(std::pair<std::string,walk_joint_function>("l_knee_joint", f4));

	// f5
	f5.clone(&f3);
	f5.in_scale_set(f5.in_scale_get() * 2);
	f5.scale_set(0);
	afn.insert(std::pair<std::string,walk_joint_function>("l_hip_swing_joint", f5));

	// f6
	f6.clone(&f3);
	f6.mirror();
	f6.in_scale_set(f6.in_scale_get() * 2);
	f6.scale_set(f5.scale_get());
	f6.offset_set(f6.offset_get() + ankle_offset);
	afn.insert(std::pair<std::string,walk_joint_function>("l_ankle_swing_joint", f6));

	// f7
	f7.clone(&f4);
	f7.scale_set(0);
	afn.insert(std::pair<std::string,walk_joint_function>("l_knee_joint", f7));

	generate_right();

	print();
}

// Obtain the joint angles for a given phase, position in cycle (x 0,1)) and velocity parameters
joints_t walk_function::angles_get(bool phase, double x, std::vector<double> velocity)
{

	ROS_ASSERT(pfn.size() == afn.size());

    joints_t angles;

    std::map<std::string, walk_joint_function>::iterator itp;

    if (phase) {
    	for (itp = pfn.begin(); itp != pfn.end(); ++itp)
    		angles.insert(std::pair<std::string, double>(itp->first, itp->second.get_y(x)));
    } else {
    	for (itp = afn.begin(); itp != afn.end(); ++itp)
    		angles.insert(std::pair<std::string, double>(itp->first, itp->second.get_y(x)));
	}

    apply_velocity(angles, velocity, phase, x);

    return angles;
}

// Modify on the walk-on-spot joint angles to apply the velocity vector
void walk_function::apply_velocity(joints_t &angles,
		std::vector<double> velocity, bool phase, double x)
{
	ROS_ASSERT(velocity.size() == VELOCITY_VECTOR_SIZE);
	ROS_ASSERT(angles.size() == pfn.size());

    // VX
    double v = velocity[0] * vx_scale;
    double d = (x * 2 - 1) * v;
    if (phase) {
        angles["l_hip_swing_joint"] += d;
        angles["l_ankle_swing_joint"] += d;
        angles["r_hip_swing_joint"] += d;
        angles["r_ankle_swing_joint"] += d;
    } else {
        angles["l_hip_swing_joint"] -= d;
        angles["l_ankle_swing_joint"] -= d;
        angles["r_hip_swing_joint"] -= d;
        angles["r_ankle_swing_joint"] -= d;
    }

    //ROS_INFO_STREAM("vel: " << v << "delta: " << d);
    //ROS_INFO_STREAM("angles[l_ankle_swing_joint]" << angles["l_ankle_swing_joint"]);

    // VY
    v = velocity[1] * vy_scale;
    d = x * v;
    double d2 = (1 - x) * v;
    if (v >= 0) {
        if (phase) {
            angles["l_hip_lateral_joint"] -= d;
            angles["l_ankle_lateral_joint"] -= d;
            angles["r_hip_lateral_joint"] += d;
            angles["r_ankle_lateral_joint"] += d;
        } else {
            angles["l_hip_lateral_joint"] -= d2;
            angles["l_ankle_lateral_joint"] -= d2;
            angles["r_hip_lateral_joint"] += d2;
            angles["r_ankle_lateral_joint"] += d2;
        }
    } else {
        if (phase) {
            angles["l_hip_lateral_joint"] += d2;
            angles["l_ankle_lateral_joint"] += d2;
            angles["r_hip_lateral_joint"] -= d2;
            angles["r_ankle_lateral_joint"] -= d2;
        } else {
            angles["l_hip_lateral_joint"] += d;
            angles["l_ankle_lateral_joint"] += d;
            angles["r_hip_lateral_joint"] -= d;
            angles["r_ankle_lateral_joint"] -= d;
        }
    }

    // VT
    v = velocity[2] * vt_scale;
    d = x * v;
    d2 = (1 - x) * v;
    if (v >= 0) {
        if (phase) {
            angles["l_hip_twist_joint"] = -d;
            angles["r_hip_twist_joint"] = d;
        } else {
            angles["l_hip_twist_joint"] = -d2;
            angles["r_hip_twist_joint"] = d2;
        }
    } else {
        if (phase) {
            angles["l_hip_twist_joint"] = d2;
            angles["r_hip_twist_joint"] = -d2;
        } else {
            angles["l_hip_twist_joint"] = d;
            angles["r_hip_twist_joint"] = -d;
        }
    }
}

// Mirror CPG functions from left to right and antiphase right
void walk_function::generate_right(void)
{
	std::map<std::string, walk_joint_function>::iterator itp;

	for (itp=pfn.begin(); itp!=pfn.end(); ++itp) {
		std::string j(itp->first);
		ROS_INFO_STREAM("generate right: " << j);
		j.erase(0, 2);
		pfn["r_"+j].clone(&afn["l_"+j]);
		pfn["r_"+j].mirror();
		afn["r_"+j].clone(&pfn["l_"+j]);
		afn["r_"+j].mirror();
	}
}

// Display CPG function
void walk_function::print(void)
{
	std::map<std::string, walk_joint_function>::iterator itp;
	std::map<std::string, walk_joint_function>::iterator ita = afn.begin();

	ROS_ASSERT(pfn.size() == afn.size());

	for (itp=pfn.begin(); itp!=pfn.end(); ++itp, ++ita ) {
		ROS_INFO_STREAM(itp->first << " p ");
		itp->second.print();
		ROS_INFO_STREAM(ita->first << " a ");
		ita->second.print();
	}
}

walker::walker(bioloid_robot *robot):
		nodeh_priv_("walker"),
		robotp(robot),
		running(false),
		name_("walker")
{
	// Load rosparams
	std::size_t error = 0;
	error += !rosparam_shortcuts::get(name_, nodeh_priv_, "loop_hz", loop_hz);
	if (error)
	{
		ROS_WARN_STREAM_NAMED(name_, "Walker requires the following config in the yaml:");
		ROS_WARN_STREAM_NAMED(name_, "   loop_hz: <x> $  10 < x < 100");
	}
	rosparam_shortcuts::shutdownIfError(name_, error);

	velocity_target.resize(3,0.0);
	velocity_current.resize(3,0.0);

	// initialize subscribers
	cmd_vel_sub_  = nodeh_.subscribe(robotp->namespace_get()+"cmd_vel",
	1,
	&walker::cmd_vel_callback,
	this);
}

walker::~walker()
{

}

// If not there yet, go to initial walk position
void walker::walk_init(void)
{
    joints_t::iterator it;

	ready_pos = function.angles_get(true, (double)0.0, velocity_current);

    if (get_distance_to_ready() > VELOCITY_WALKING) {
    	ROS_INFO_STREAM("Going to walk position...");
        //for (it = ready_pos.begin(); it != ready_pos.end(); ++it) {
        	//ROS_INFO_STREAM("ready_pos " << it->first << ": " << it->second);
        //}
        robotp->set_angles_slow(ready_pos, ANGLE_DELAY);
		ROS_INFO_STREAM("Done.");
    }
}

void walker::walk_start(void)
{
	if (!running) {
		ROS_INFO_STREAM("Starting...");
		// TODO: Too long for callback, move in thread.
		walk_init();
        running = true;
	}
}

void walker::walk_stop(void)
{
    if (running) {
        ROS_INFO_STREAM("Stopping...");
		running = false;
    }
}

void walker::velocity_set(double x, double y, double t)
{
	velocity_target = {x, y, t};

    if (x == 0 && y == 0 && t == 0)
    	walk_stop();
    else
    	walk_start();
}

void walker::cmd_vel_callback(const geometry_msgs::TwistConstPtr msg)
{
	velocity_set(msg->linear.x, msg->linear.y, msg->angular.z);
}

bool walker::walking_get(void)
{
	ROS_ASSERT(velocity_current.size() == VELOCITY_VECTOR_SIZE);

	for (unsigned int i = 0; i < VELOCITY_VECTOR_SIZE; i++) {
	    if (abs(velocity_current[i]) > VELOCITY_WALKING)
	    	return true;
	}
	return false;
}

void walker::velocity_update(std::vector<double> target, double n)
{
	joints_t::iterator it;
	double a = (double)3 / n;
	double b = (double)1 - a;

	ROS_ASSERT(target.size() == VELOCITY_VECTOR_SIZE);

	for (unsigned int i = 0; i < VELOCITY_VECTOR_SIZE; i++) {
		velocity_current[i] = a * target[i] + b * velocity_current[i];
	}
}

double walker::get_distance_to_ready(void)
{
	joints_t angles = robotp->get_angles();

	return get_distance(ready_pos, angles);
}

// Main walking loop, smoothly update velocity vectors and apply corresponding angles
void walker::update(void)
{

	// Global walk loop
	uint32_t n = 50;
	uint32_t p = true;
	uint32_t i = 0;
	double x = 0.0;
	joints_t angles;

	ros::Rate r(loop_hz);

	ROS_INFO_STREAM_NAMED(name_, "Loop rate is set to: " << loop_hz);

	velocity_current = { 0.0, 0.0, 0.0};

	while (ros::ok()) {

		// Do not move if nothing to do and already at 0
		if (!running && (i == 0)) {

			velocity_update(velocity_target, n);

		} else {

			// Process next 1/50th
			x = (double)i++ / n;
			//ROS_INFO_STREAM("|i=" << i - 1 << " |p=" << p << " |x=" << x << " |vc=" << velocity_current[0] << " |vt=" << velocity_target[0]);
			angles = function.angles_get(p, x, velocity_current);
			velocity_update(velocity_target, n);
			robotp->set_angles(angles);

			// Next phase
			if (i > n) {
			 i = 0;
			 p = !p;
			}
		}

		ros::spinOnce();
		r.sleep();
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "walker");

	ROS_INFO_STREAM("Instantiating Bioloid Robot");
	bioloid_robot robot;
	ros::Duration(1).sleep();

	walker walk(&robot);
	ROS_INFO_STREAM("Bioloid Robot ready");

	ROS_INFO_STREAM("Started walking thread");
	walk.update();
	ROS_INFO_STREAM("Finished walking thread");

	return 0;
}
