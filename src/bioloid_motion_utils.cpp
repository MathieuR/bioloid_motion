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
#include <bioloid_motion/bioloid_motion_utils.h>

joints_t interpolate(
		joints_t anglesa,
		joints_t anglesb,
		double coefa)
{
	joints_t::iterator it;
	joints_t new_joints;

	ROS_ASSERT_MSG(anglesa.size() <= anglesb.size(), "%d > %d", anglesa.size(), anglesb.size());

	for (it = anglesa.begin(); it != anglesa.end(); ++it) {
		new_joints[it->first] = it->second * coefa + anglesb[it->first] * (1 - coefa);
    }
	return new_joints;
}

double get_distance(
		joints_t anglesa,
		joints_t anglesb)
{
	joints_t::iterator it;
	double d = 0;

	ROS_ASSERT_MSG(anglesa.size() <= anglesb.size(), "%d > %d", anglesa.size(), anglesb.size());

	for (it = anglesa.begin(); it != anglesa.end(); ++it) {
        d += std::abs(anglesb[it->first] - it->second);
    }

    d /= anglesb.size();

    return d;
}


