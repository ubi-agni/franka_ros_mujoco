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
 *  - class name FrankaGripperSim -> FrankaGripperMujoco
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

#include <cmath>
#include <memory>

#include <pluginlib/class_list_macros.h>

#include <franka_mujoco/franka_gripper_mujoco.h>

namespace franka_mujoco {

using actionlib::SimpleActionServer;
using control_msgs::GripperCommandAction;
using franka_gripper::GraspAction;
using franka_gripper::HomingAction;
using franka_gripper::MoveAction;
using franka_gripper::StopAction;

bool FrankaGripperMujoco::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &nh)
{
	std::string ns = nh.getNamespace();
	std::string arm_id;

	if (not nh.getParam("arm_id", arm_id)) {
		ROS_ERROR_STREAM_NAMED("FrankaGripperMujoco", "Could not find required parameter '" << ns << "/arm_id'");
		return false;
	}
	std::string finger1 = arm_id + "_finger_joint1";
	std::string finger2 = arm_id + "_finger_joint2";

	nh.param<double>("move/width_tolerance", tolerance_move_, kDefaultMoveWidthTolerance);
	nh.param<double>("gripper_action/width_tolerance", tolerance_gripper_action_, kDefaultGripperActionWidthTolerance);
	nh.param<double>("gripper_action/speed", speed_default_, kDefaultGripperActionSpeed);
	nh.param<double>("grasp/resting_threshold", speed_threshold_, kGraspRestingThreshold);
	nh.param<int>("grasp/consecutive_samples", speed_samples_, kGraspConsecutiveSamples);

	try {
		finger1_ = hw->getHandle(finger1);
		finger2_ = hw->getHandle(finger2);
	} catch (const hardware_interface::HardwareInterfaceException &ex) {
		ROS_ERROR_STREAM_NAMED("FrankaGripperMujoco", "Could not get joint handle(s): " << ex.what());
		return false;
	}

	if (not pid1_.initParam(ns + "/finger1/gains")) {
		return false;
	}
	if (not pid2_.initParam(ns + "/finger2/gains")) {
		return false;
	}

	pub_.init(nh, "joint_states", 1);
	pub_.lock();
	pub_.msg_.name = { finger1, finger2 };
	pub_.unlock();

	action_stop_ = std::make_unique<SimpleActionServer<StopAction>>(
	    nh, "stop", boost::bind(&FrankaGripperMujoco::onStopGoal, this, _1), false);
	action_stop_->start();

	action_homing_ = std::make_unique<SimpleActionServer<HomingAction>>(
	    nh, "homing", boost::bind(&FrankaGripperMujoco::onHomingGoal, this, _1), false);
	action_homing_->registerPreemptCallback([&]() {
		ROS_INFO_STREAM_NAMED("FrankaGripperMujoco", "Homing Action cancelled");
		setState(State::IDLE);
	});
	action_homing_->start();

	action_move_ = std::make_unique<SimpleActionServer<MoveAction>>(
	    nh, "move", boost::bind(&FrankaGripperMujoco::onMoveGoal, this, _1), false);
	action_move_->registerPreemptCallback([&]() {
		ROS_INFO_STREAM_NAMED("FrankaGripperMujoco", "Moving Action cancelled");
		setState(State::IDLE);
	});
	action_move_->start();

	action_grasp_ = std::make_unique<SimpleActionServer<GraspAction>>(
	    nh, "grasp", boost::bind(&FrankaGripperMujoco::onGraspGoal, this, _1), false);
	action_grasp_->registerPreemptCallback([&]() {
		ROS_INFO_STREAM_NAMED("FrankaGripperMujoco", "Grasping Action cancelled");
		setState(State::IDLE);
	});
	action_grasp_->start();

	action_gc_ = std::make_unique<SimpleActionServer<GripperCommandAction>>(
	    nh, "gripper_action", boost::bind(&FrankaGripperMujoco::onGripperActionGoal, this, _1), false);
	action_gc_->registerPreemptCallback([&]() {
		ROS_INFO_STREAM_NAMED("FrankaGripperMujoco", "Gripper Command Action cancelled");
		setState(State::IDLE);
	});
	action_gc_->start();

	ROS_INFO_STREAM_NAMED("FrankaGripperMujoco", "Successfully initialized Franka Gripper Controller for joints '"
	                                                 << finger1 << "' and '" << finger2 << "'");
	return true;
}

void FrankaGripperMujoco::starting(const ros::Time & /*unused*/)
{
	transition(State::IDLE, Config());
	pid1_.reset();
	pid2_.reset();
}

void FrankaGripperMujoco::update(const ros::Time &now, const ros::Duration &period)
{
	if (rate_trigger_() and pub_.trylock()) {
		pub_.msg_.header.stamp = now;
		pub_.msg_.position     = { finger1_.getPosition(), this->finger2_.getPosition() };
		pub_.msg_.velocity     = { finger1_.getVelocity(), this->finger2_.getVelocity() };
		pub_.msg_.effort       = { finger1_.getEffort(), this->finger2_.getEffort() };
		pub_.unlockAndPublish();
	}

	// Read state threadsafe
	double width = finger1_.getPosition() + this->finger2_.getPosition();
	mutex_.lock();
	State state    = state_;
	auto tolerance = config_.tolerance;
	double w_d     = config_.width_desired;
	double dw_d    = config_.speed_desired * std::copysign(1.0, w_d - width);
	mutex_.unlock();
	if (state == State::IDLE) {
		// Track position of other finger to simulate mimicked joints + high damping
		control(finger1_, this->pid1_, this->finger2_.getPosition(), 0, 0, period);
		control(finger2_, this->pid2_, this->finger1_.getPosition(), 0, 0, period);
		return;
	}

	// Compute control signal and send to joints
	double w1_d = finger1_.getPosition() + 0.5 * dw_d * period.toSec();
	double w2_d = finger2_.getPosition() + 0.5 * dw_d * period.toSec();

	// Only in case when we hold we want to add the desired force, in any other state don't add
	// anything extra to the command
	double f_d = 0;

	if (state == State::HOLDING) {
		// When an object is grasped, next to the force to apply, also track the other finger
		// to not make both fingers drift away from middle simultaneously
		w1_d = finger2_.getPosition();
		w2_d = finger1_.getPosition();
		std::lock_guard<std::mutex> lock(mutex_);
		f_d = config_.force_desired / 2.0;
	}

	control(finger1_, this->pid1_, w1_d, 0.5 * dw_d, f_d, period);
	control(finger2_, this->pid2_, w2_d, 0.5 * dw_d, f_d, period);

	if (w_d - tolerance.inner < width and width < w_d + tolerance.outer) {
		// Goal reached, update statemachine
		if (state == State::MOVING) {
			// Done with move motion, switch to idle again
			transition(State::IDLE, Config{ .width_desired = config_.width_desired,
			                                .speed_desired = 0,
			                                .force_desired = 0,
			                                .tolerance     = config_.tolerance });
			return;
		}
	}

	if (state == State::GRASPING or state == State::MOVING) {
		// Since the velocity signal is noisy it can easily happen that one sample is below the
		// threshold To avoid abortion because of noise, we have to read at least N consecutive number
		// of samples before interpreting something was grasped (or not)
		static int speed_threshold_counter = 0;
		double speed                       = finger1_.getVelocity() + this->finger2_.getVelocity();
		if (std::abs(speed) <= speed_threshold_) {
			speed_threshold_counter++;
		} else {
			speed_threshold_counter = 0;
		}

		if (speed_threshold_counter >= speed_samples_) {
			if (state == State::GRASPING) {
				// Done with grasp motion, switch to holding, i.e. keep position & force
				transition(State::HOLDING, Config{ .width_desired = width,
				                                   .speed_desired = 0,
				                                   .force_desired = config_.force_desired,
				                                   .tolerance     = config_.tolerance });
			} else {
				// Moving failed due to object between fingers. Switch to idle.
				ROS_DEBUG_STREAM_NAMED("FrankaGripperMujoco", "Moving failed due to an object between fingers");
				transition(State::IDLE, Config{ .width_desired = width,
				                                .speed_desired = 0,
				                                .force_desired = 0,
				                                .tolerance     = config_.tolerance });
			}
			speed_threshold_counter = 0;
		}
	}
}

void FrankaGripperMujoco::control(hardware_interface::JointHandle &joint, control_toolbox::Pid &pid, double q_d,
                                  double dq_d, double f_d, const ros::Duration &period)
{
	double error  = q_d - joint.getPosition();
	double derror = dq_d - joint.getVelocity();
	joint.setCommand(pid.computeCommand(error, derror, period) + f_d);
}

void FrankaGripperMujoco::setState(const State &&state)
{
	std::lock_guard<std::mutex> lock(mutex_);
	state_ = state;
}

void FrankaGripperMujoco::setConfig(const Config &&config)
{
	std::lock_guard<std::mutex> lock(mutex_);
	config_ = config;
}

void FrankaGripperMujoco::transition(const State &&state, const Config &&config)
{
	std::lock_guard<std::mutex> lock(mutex_);
	state_  = state;
	config_ = config;
}

void FrankaGripperMujoco::interrupt(const std::string &message, const State &except)
{
	if (except != State::MOVING and action_move_ != nullptr and this->action_move_->isActive()) {
		franka_gripper::MoveResult result;
		result.success = static_cast<decltype(result.success)>(false);
		result.error   = message;
		action_move_->setAborted(result, result.error);
	}
	if (except != State::GRASPING and action_grasp_ != nullptr and this->action_grasp_->isActive()) {
		franka_gripper::GraspResult result;
		result.success = static_cast<decltype(result.success)>(false);
		result.error   = message;
		action_grasp_->setAborted(result, result.error);
	}
	if (except != State::MOVING and action_homing_ != nullptr and this->action_homing_->isActive()) {
		franka_gripper::HomingResult result;
		result.success = static_cast<decltype(result.success)>(false);
		result.error   = message;
		action_homing_->setAborted(result, result.error);
	}
}

void FrankaGripperMujoco::waitUntilStateChange()
{
	State original = state_; // copy

	ros::Rate rate(30);
	while (ros::ok()) {
		{
			std::lock_guard<std::mutex> lock(mutex_);
			if (state_ != original) {
				return;
			}
		}
		rate.sleep();
	}
}

void FrankaGripperMujoco::onStopGoal(const franka_gripper::StopGoalConstPtr & /*goal*/)
{
	ROS_INFO_STREAM_NAMED("FrankaGripperMujoco", "Stop Action goal received");

	interrupt("Command interrupted, because stop action was called", State::IDLE);

	transition(State::IDLE, Config{ .width_desired = config_.width_desired,
	                                .speed_desired = 0,
	                                .force_desired = 0,
	                                .tolerance     = config_.tolerance });

	franka_gripper::StopResult result;
	result.success = static_cast<decltype(result.success)>(true);
	action_stop_->setSucceeded(result);
}

void FrankaGripperMujoco::onHomingGoal(const franka_gripper::HomingGoalConstPtr & /*goal*/)
{
	ROS_INFO_STREAM_NAMED("FrankaGripperMujoco", "New Homing Action goal received");

	if (state_ != State::IDLE) {
		interrupt("Command interrupted, because new homing action called", State::MOVING);
	}

	franka_gripper::GraspEpsilon eps;
	eps.inner = tolerance_move_;
	eps.outer = tolerance_move_;
	transition(State::MOVING,
	           Config{ .width_desired = kMaxFingerWidth, .speed_desired = 0.02, .force_desired = 0, .tolerance = eps });

	waitUntilStateChange();
	if (!action_homing_->isActive()) {
		// Homing Action was interrupted from another action goal callback and already preempted.
		// Don't try to resend result now
		return;
	}

	franka_gripper::HomingResult result;
	if (state_ != State::IDLE) {
		result.success = static_cast<decltype(result.success)>(false);
		result.error   = "Unexpected state transistion: The gripper not in IDLE as expected";
		action_homing_->setAborted(result, result.error);
		return;
	}

	result.success = static_cast<decltype(result.success)>(true);
	action_homing_->setSucceeded(result);
}

void FrankaGripperMujoco::onMoveGoal(const franka_gripper::MoveGoalConstPtr &goal)
{
	ROS_INFO_STREAM_NAMED("FrankaGripperMujoco", "New Move Action Goal received: " << goal->width << " m");
	if (goal->speed < 0) {
		franka_gripper::MoveResult result;
		result.success = static_cast<decltype(result.success)>(false);
		result.error   = "Only positive speeds allowed";
		action_move_->setAborted(result, result.error);
		return;
	}
	ROS_INFO_STREAM_NAMED("FrankaGripperMujoco", "Speed ok");

	if (goal->width < 0 or goal->width > kMaxFingerWidth or not std::isfinite(goal->width)) {
		franka_gripper::MoveResult result;
		result.success = static_cast<decltype(result.success)>(false);
		result.error   = "Target width has to lie between 0 .. " + std::to_string(kMaxFingerWidth);
		action_move_->setAborted(result, result.error);
		return;
	}
	ROS_INFO_STREAM_NAMED("FrankaGripperMujoco", "Width ok");

	if (state_ != State::IDLE) {
		interrupt("Command interrupted, because new move action called", State::MOVING);
		ROS_INFO_STREAM_NAMED("FrankaGripperMujoco", "Move Action Interrupted");
	}

	bool move_succeeded = move(goal->width, goal->speed);

	if (!action_move_->isActive()) {
		// Move Action was interrupted from another action goal callback and already preempted.
		// Don't try to resend result now
		ROS_INFO_STREAM_NAMED("FrankaGripperMujoco", "Move Action Interrupted after move call");
		return;
	}

	franka_gripper::MoveResult result;
	if (!move_succeeded) {
		result.success = static_cast<decltype(result.success)>(false);
		result.error   = "Unexpected state transistion: The gripper not in IDLE as expected";
		action_move_->setAborted(result, result.error);
		ROS_INFO_STREAM_NAMED("FrankaGripperMujoco", "Unexpected state transition");
		return;
	}
	franka_gripper::GraspEpsilon eps;
	eps.inner = tolerance_move_;
	eps.outer = tolerance_move_;

	double width   = finger1_.getPosition() + this->finger2_.getPosition(); // recalculate
	bool ok        = goal->width - eps.inner < width and width < goal->width + eps.outer;
	result.success = static_cast<decltype(result.success)>(ok);
	action_move_->setSucceeded(result);
}

void FrankaGripperMujoco::onGraspGoal(const franka_gripper::GraspGoalConstPtr &goal)
{
	ROS_INFO_STREAM_NAMED("FrankaGripperMujoco", "New Grasp Action Goal received: " << goal->force << "N");

	if (goal->width >= kMaxFingerWidth or goal->width < 0) {
		franka_gripper::GraspResult result;
		result.success = static_cast<decltype(result.success)>(false);
		result.error   = "Can only grasp inside finger width from [0 .. " + std::to_string(kMaxFingerWidth) + "[";
		action_grasp_->setAborted(result, result.error);
		return;
	}
	if (goal->speed < 0) {
		franka_gripper::GraspResult result;
		result.success = static_cast<decltype(result.success)>(false);
		result.error   = "Only positive speeds allowed";
		action_grasp_->setAborted(result, result.error);
		return;
	}

	if (state_ != State::IDLE) {
		interrupt("Command interrupted, because new grasp action called", State::GRASPING);
	}

	bool grasp_succeeded = grasp(goal->width, goal->speed, goal->force, goal->epsilon);

	if (!action_grasp_->isActive()) {
		// Grasping Action was interrupted from another action goal callback and already preempted.
		// Don't try to resend result now
		return;
	}

	franka_gripper::GraspResult result;
	if (state_ != State::HOLDING) {
		result.success = static_cast<decltype(result.success)>(false);
		result.error   = "Unexpected state transistion: The gripper is not in HOLDING as expected";
		action_grasp_->setAborted(result, result.error);
		return;
	}

	result.success = static_cast<decltype(result.success)>(grasp_succeeded);
	if (!grasp_succeeded) {
		double current_width = finger1_.getPosition() + finger2_.getPosition();
		result.error         = "When the gripper stopped (below speed of " + std::to_string(speed_threshold_) +
		               " m/s the width between the fingers was not at " + std::to_string(goal->width) + "m (-" +
		               std::to_string(goal->epsilon.inner) + "m/+" + std::to_string(goal->epsilon.outer) + "m) but at " +
		               std::to_string(current_width) + "m";
		setState(State::IDLE);
	}

	action_grasp_->setSucceeded(result);
}

void FrankaGripperMujoco::onGripperActionGoal(const control_msgs::GripperCommandGoalConstPtr &goal)
{
	control_msgs::GripperCommandResult result;
	// HACK: As one gripper finger is <mimic>, MoveIt!'s trajectory execution manager
	// only sends us the width of one finger. Multiply by 2 to get the intended width.
	double width_d = goal->command.position * 2;

	ROS_INFO_STREAM_NAMED("FrankaGripperMujoco",
	                      "New Gripper Command Action Goal received: " << goal->command.position << "m, "
	                                                                   << goal->command.max_effort << "N");

	if (width_d > kMaxFingerWidth || width_d < 0.0) {
		std::string error = "Commanding out of range position! max_position = " + std::to_string(kMaxFingerWidth / 2) +
		                    ", commanded position = " + std::to_string(goal->command.position) +
		                    ". Be aware that you command the position of"
		                    " each finger which is half of the total opening width!";
		ROS_ERROR_STREAM_NAMED("FrankaGripperSim", error);
		result.reached_goal = static_cast<decltype(result.reached_goal)>(false);
		action_gc_->setAborted(result, error);
		return;
	}

	franka_gripper::GraspEpsilon eps;
	eps.inner = tolerance_gripper_action_;
	eps.outer = tolerance_gripper_action_;

	double current_width                = finger1_.getPosition() + finger2_.getPosition();
	constexpr double kMinimumGraspForce = 1e-4;
	bool succeeded                      = false;

	if (std::abs(goal->command.max_effort) < kMinimumGraspForce || width_d > current_width) {
		succeeded = move(width_d, speed_default_);
		if (!action_gc_->isActive()) {
			// Gripper Action was interrupted from another action goal callback and already preempted.
			// Don't try to resend result now
			return;
		}
	} else {
		succeeded = grasp(width_d, speed_default_, goal->command.max_effort, eps);
		if (!action_gc_->isActive()) {
			// Gripper Action was interrupted from another action goal callback and already preempted.
			// Don't try to resend result now
			return;
		}
		if (state_ != State::HOLDING) {
			result.reached_goal = static_cast<decltype(result.reached_goal)>(false);
			std::string error   = "Unexpected state transition: The gripper is not in HOLDING as expected";
			action_gc_->setAborted(result, error);
			return;
		}
	}

	result.position     = finger1_.getPosition() + finger2_.getPosition();
	result.effort       = 0;
	result.stalled      = static_cast<decltype(result.stalled)>(false);
	result.reached_goal = static_cast<decltype(result.reached_goal)>(succeeded);
	if (!succeeded) {
		setState(State::IDLE);
	}
	action_gc_->setSucceeded(result);
}

bool FrankaGripperMujoco::move(double width, double speed)
{
	franka_gripper::GraspEpsilon eps;
	eps.inner = tolerance_move_;
	eps.outer = tolerance_move_;
	transition(State::MOVING,
	           Config{ .width_desired = width, .speed_desired = speed, .force_desired = 0, .tolerance = eps });
	waitUntilStateChange();
	return state_ == State::IDLE;
}

bool FrankaGripperMujoco::grasp(double width, double speed, double force, const franka_gripper::GraspEpsilon &epsilon)
{
	double current_width = finger1_.getPosition() + finger2_.getPosition();
	double direction     = std::copysign(1.0, width - current_width);
	transition(State::GRASPING, Config{ .width_desired = width < current_width ? 0 : kMaxFingerWidth,
	                                    .speed_desired = speed,
	                                    .force_desired = direction * force,
	                                    .tolerance     = epsilon });

	waitUntilStateChange();
	current_width = finger1_.getPosition() + finger2_.getPosition();
	return width - epsilon.inner < current_width && current_width < width + epsilon.outer;
}

} // namespace franka_mujoco

PLUGINLIB_EXPORT_CLASS(franka_mujoco::FrankaGripperMujoco, controller_interface::ControllerBase);
