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
 *  Copyright (c) 2023, Bielefeld University
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

#pragma once

#include <mujoco_ros/mujoco_env.h>
#include <mujoco_ros/common_types.h>

#include <angles/angles.h>
#include <control_toolbox/pid.h>
#include <franka/robot_state.h>
#include <joint_limits_interface/joint_limits.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <boost/optional.hpp>

namespace franka_mujoco {

/**
 * Specifies the current control method of the joint
 */
enum ControlMethod
{
	EFFORT,
	POSITION,
	VELOCITY
};

/**
 * A data container holding all relevant information about a robotic joint.
 *
 * Calling @ref update on this object will compute its internal state based on the all currenlty
 * supplied information such as position, efforts etc.
 */
struct Joint
{
public:
	Joint()              = default;
	Joint(Joint &&)      = default;
	Joint(const Joint &) = delete;

	/**
	 * Calculate all members such as accelerations, jerks velocities by differentiation
	 * @param[in] dt the current time step since last time this method was called
	 * @param[in] position_noise optional noise to add to the joint position (only if type is continuous or revolute)
	 * which defaults to 0.0 (Addition of UBI)
	 */
	void update(const ros::Duration &dt, double position_noise = 0.0);

	/// Handles to MuJoCo model and data
	const mjModel *m_ptr;
	mjData *d_ptr;

	/// Name of this joint. Should be unique in whole simulation
	std::string name;

	/// MuJoCo object id of this joint
	int id;

	/// The type of joint, i.e. revolute, prismatic, ... @see
	/// http://docs.ros.org/en/diamondback/api/urdf/html/classurdf_1_1Joint.html
	int type;

	/// Joint limits @see
	/// http://docs.ros.org/en/diamondback/api/urdf/html/classurdf_1_1JointLimits.html
	joint_limits_interface::JointLimits limits;

	/// The axis of rotation/translation of this joint in local coordinates
	Eigen::Vector3d axis;

	/// The currently applied command from a controller acting on this joint either in \f$N\f$ or
	/// \f$Nm\f$ without gravity
	double command = 0;

	/// The current desired position that is used for the PID controller when the joints control
	/// method is "POSITION". When the control method is not "POSITION", this value will only be
	/// updated once at the start of the controller and stay the same until a new controller is
	/// started.
	double desired_position = 0;

	/// The current desired velocity that is used for the PID controller when the joints control
	/// method is "VELOCITY". When the control method is not "VELOCITY", this value will be set to
	/// zero.
	double desired_velocity = 0;

	/// Decides whether the joint is doing torque control or if the position or velocity should
	/// be controlled, or if the joint is entirely uncontrolled
	boost::optional<ControlMethod> control_method = boost::none;

	/// The currently acting gravity force or torque acting on this joint in \f$N\f$ or \f$Nm\f$
	double gravity = 0;

	/// The current position of this joint either in \f$m\f$ or \f$rad\f$
	double position = 0;

	/// The current velocity of this joint either in \f$\frac{m}{s}\f$ or \f$\frac{rad}{s}\f$
	double velocity = 0;

	/// The current total force or torque acting on this joint in either \f$N\f$ or \f$Nm\f$
	double effort = 0;

	/// The currently acting jerk acting on this this joint in either \f$\frac{m}{s^3}\f$ or
	/// \f$\frac{rad}{s^3}\f$
	double jerk = 0;

	/// The currenlty acting acceleration on this joint in either \f$\frac{m}{s^2}\f$ or
	/// \f$\frac{rad}{s^2}\f$
	double acceleration = 0;

	/// Above which threshold forces or torques will be interpreted as "contacts" by @ref isInContact
	double contact_threshold = std::numeric_limits<double>::infinity();

	/// Above which threshold forces or torques will be interpreted as "collisions" by @ref
	/// isInCollision
	double collision_threshold = std::numeric_limits<double>::infinity();

	/// Position used as desired position if `control_method` is none
	double stop_position = 0;

	/**
	 * Decide what the desired position of this joint is based on:
	 * 1. If a reflex is present, return `position`
	 * 2. ...otherwise if the control method is POSITION, return `desired_position`
	 * 3. ...otherwise if the control method is EFFORT, return `desired_position`
	 * 4. ...otherwise return `position`
	 *
	 * @param[in] mode - the current mode the robot is in
	 * @return either `position` or `desired_position`
	 */
	double getDesiredPosition(const franka::RobotMode &mode) const;

	/**
	 * Decide what the desired velocity of this joint is based on:
	 * 1. If a reflex is present, return `acceleration`
	 * 2. ...otherwise if the control method is EFFORT, return `0`
	 * 4. ...otherwise return `acceleration`
	 *
	 * @param[in] mode - the current mode the robot is in
	 * @return either `acceleration` or `0`
	 */
	double getDesiredVelocity(const franka::RobotMode &mode) const;

	/**
	 * Decide what the desired acceleration of this joint is based on:
	 * 1. If a reflex is present, return `acceleration`
	 * 2. ...otherwise if the control method is EFFORT, return `0`
	 * 3. ...otherwise return `acceleration`
	 * @param[in] mode - the current mode the robot is in
	 * @return either `acceleration` or `0`
	 */
	double getDesiredAcceleration(const franka::RobotMode &mode) const;

	/**
	 * Decide what the desired torque of this joint is based on:
	 * 1. If a reflex is present, return `0`
	 * 2. ...otherwise if the control method is not EFFORT, return `0`
	 * 4. ...otherwise return `command`
	 *
	 * @param[in] mode - the current mode the robot is in
	 * @return either `command` or `0`
	 */
	double getDesiredTorque(const franka::RobotMode &mode) const;

	/**
	 * Get the total link mass of this joint's child
	 * @return the mass in \f$kg\f$
	 */
	double getLinkMass() const;

	/**
	 * Is the joint currently in contact with something?
	 * @return `true` if @ref effort > @ref contact_threshold
	 */
	bool isInContact() const;

	/**
	 * Is the joint currently in contact with something?
	 * @return `true` if @ref effort > @ref collision_threshold
	 */
	bool isInCollision() const;

	/// The PID used for the controller, when in "position" control mode. In other modes these gains are ignored
	control_toolbox::Pid position_controller;

	/// The PID used for the controller, when in "velocity" control mode. In other modes these gains are ignored
	control_toolbox::Pid velocity_controller;

private:
	double lastVelocity     = std::numeric_limits<double>::quiet_NaN();
	double lastAcceleration = std::numeric_limits<double>::quiet_NaN();
};

} // namespace franka_mujoco
