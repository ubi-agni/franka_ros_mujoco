/*********************************************************************
 * Copyright 2017 Franka Emika GmbH
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
 *
 *
 * Changes made for franka_mujoco:
 *  - namespace
 *  - Interaction with Gazebo has been fully replaced with MuJoCo interaction, but the core functionality stayed the
 *same.
 *********************************************************************/
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: David P. Leins*/

#include <urdf/model.h>

#include <franka_mujoco/joint.h>

namespace franka_mujoco {

void Joint::update(const ros::Duration &dt)
{
	double pos = d_ptr->qpos[m_ptr->jnt_dofadr[id]];

	switch (type) {
		case urdf::Joint::PRISMATIC:
			position = pos;
			break;

		case urdf::Joint::REVOLUTE:
		case urdf::Joint::CONTINUOUS:
			position += angles::shortest_angular_distance(position, pos);
			break;
	}

	velocity     = d_ptr->qvel[m_ptr->jnt_dofadr[id]];
	acceleration = d_ptr->qacc[m_ptr->jnt_dofadr[id]];
	effort       = d_ptr->qfrc_applied[m_ptr->jnt_dofadr[id]];

	if (std::isnan(lastVelocity))
		lastVelocity = velocity;

	jerk             = (acceleration - lastAcceleration) / dt.toSec();
	lastAcceleration = acceleration;
}

double Joint::getLinkMass() const
{
	return m_ptr->body_mass[id];
}

bool Joint::isInCollision() const
{
	return std::abs(effort - command) > collision_threshold;
}

bool Joint::isInContact() const
{
	return std::abs(effort - command) > contact_threshold;
}
} // namespace franka_mujoco
