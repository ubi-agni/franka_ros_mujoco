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
 *  - FrankaHWSim extends mujoco_ros_control::RobotHWSim instead of gazebo_ros_control::RobotHWSim
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

#include <joint_limits_interface/joint_limits_urdf.h>
#include <boost/algorithm/clamp.hpp>

#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/SwitchController.h>

#include <franka/duration.h>
#include <franka_hw/franka_hw.h>
#include <franka_example_controllers/pseudo_inversion.h>

#include <franka_mujoco/franka_hw_mujoco.h>
#include <franka_mujoco/model_kdl.h>

#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>

#include <pluginlib/class_list_macros.hpp>

namespace franka_mujoco {

using actionlib::SimpleActionServer;
using boost::sml::state;

FrankaHWSim::FrankaHWSim() : sm_(this->robot_state_, this->joints_) {}

bool FrankaHWSim::initSim(MujocoSim::mjModelPtr m, MujocoSim::mjDataPtr d, const std::string &robot_namespace,
                          ros::NodeHandle model_nh, const urdf::Model *const urdf,
                          std::vector<transmission_interface::TransmissionInfo> transmissions)
{
	m_ptr_ = m;
	d_ptr_ = d;

	robot_initialized_ = false;

	robot_initialized_pub_ = model_nh.advertise<std_msgs::Bool>("initialized", 1);
	std_msgs::Bool msg;
	msg.data = static_cast<decltype(msg.data)>(false);
	robot_initialized_pub_.publish(msg);

	action_recovery_ = std::make_unique<SimpleActionServer<franka_msgs::ErrorRecoveryAction>>(
	    model_nh, "franka_control/error_recovery",
	    [&](const franka_msgs::ErrorRecoveryGoalConstPtr &goal) {
		    if (robot_state_.robot_mode == franka::RobotMode::kUserStopped) {
			    ROS_WARN_STREAM_NAMED("franka_hw_sim", "Cannot recover errors since the user stop seems still pressed");
			    action_recovery_->setSucceeded();
			    return;
		    }
		    try {
			    restartControllers();
			    ROS_INFO_NAMED("franka_hw_sim", "Recovered from error");
			    sm_.process_event(ErrorRecovery());
			    action_recovery_->setSucceeded();
		    } catch (const std::runtime_error &e) {
			    ROS_WARN_STREAM_NAMED("franka_hw_sim", "Error recovery failed: " << e.what());
			    action_recovery_->setAborted();
		    }
	    },
	    false);
	action_recovery_->start();

	// Default to 'panda' as arm_id
	model_nh.param<std::string>("arm_id", arm_id_, "panda");
	ROS_DEBUG_STREAM_NAMED("franka_hw_sim", "arm_id is '" << arm_id_ << "'");
	if (arm_id_ != robot_namespace) {
		ROS_WARN_STREAM_NAMED("franka_hw_sim",
		                      "Caution: Robot names differ! Read 'arm_id: "
		                          << arm_id_ << "' from parameter server but URDF defines '<robotNamespace>"
		                          << robot_namespace << "</robotNamespace>. Will use '" << arm_id_ << "!");
	}

	model_nh.param<double>("tau_ext_lowpass_filter", tau_ext_lowpass_filter_, kDefaultTauExtLowpassFilter);
	ROS_DEBUG_NAMED("franka_hw_sim", "tau is: %f", tau_ext_lowpass_filter_);

	std::array<double, 3> gravity = { m_ptr_->opt.gravity[0], m_ptr_->opt.gravity[1], m_ptr_->opt.gravity[2] };
	ROS_DEBUG_NAMED("franka_hw_sim", "Sim Gravity is: %.2f %.2f %.2f", gravity[0], gravity[1], gravity[2]);

	// Generate list of franka_mujoco::Joint to store all relevant information
	for (const auto &transmission : transmissions) {
		if (transmission.type_ != "transmission_interface/SimpleTransmission") {
			continue;
		}
		if (transmission.joints_.empty()) {
			ROS_WARN_STREAM_NAMED("franka_hw_sim", "Transmission " << transmission.name_ << " has no associated joints.");
			return false;
		}
		if (transmission.joints_.size() > 1) {
			ROS_WARN_STREAM_NAMED("franka_hw_sim",
			                      "Transmission "
			                          << transmission.name_ << " has more than one joint."
			                          << " Currently the franka robot hardware simulation interface only supports one.");
			return false;
		}

		// Fill a 'joint' struct which holds all nevessary data
		auto joint   = std::make_shared<franka_mujoco::Joint>();
		joint->name  = transmission.joints_[0].name_;
		joint->m_ptr = m;
		joint->d_ptr = d;

		if (urdf == NULL) {
			ROS_ERROR_STREAM_NAMED("franka_hw_sim",
			                       "Could not find any URDF model. Was it loaded on the parameter server?");
			return false;
		}

		auto urdf_joint = urdf->getJoint(joint->name);
		if (!urdf_joint) {
			ROS_ERROR_STREAM_NAMED("franka_hw_sim", "Could not get joint '"
			                                            << joint->name
			                                            << "' from URDF. Make sure naming inside the URDF and between "
			                                               "URDF and MuJoCo XML is consistent!");
			return false;
		}
		joint->type = urdf_joint->type;
		joint_limits_interface::getJointLimits(urdf_joint, joint->limits);
		ROS_DEBUG_STREAM_NAMED("franka_hw_sim",
		                       "Creating joint " << joint->name << " of transmission type " << transmission.type_);
		joint->axis = Eigen::Vector3d(urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z);

		int id = MujocoSim::jointName2id(m_ptr_.get(), joint->name, robot_namespace);
		if (id == -1) {
			ROS_ERROR_STREAM_NAMED("franka_hw_sim", "Could not get joint '"
			                                            << joint->name << "' from MuJoCo model."
			                                            << " Make sure naming between URDF and MuJoCo XML is consistent!");
			return false;
		}
		joint->id = id;
		// set the control method for finger joints to effort
		if (joint->name.find(arm_id_ + "_finger_joint") != std::string::npos) {
			joint->control_method = EFFORT;
		}
		joints_.emplace(joint->name, joint);
	}

	// After the joint data containers have been fully initialized and their memory addresses don't
	// change anymore, get the respective addresses to pass them to the handles

	for (auto &pair : joints_) {
		initJointStateHandle(pair.second);
	}

	// Register all supported command interfaces
	for (const auto &transmission : transmissions) {
		for (const auto &k_interface : transmission.joints_[0].hardware_interfaces_) {
			auto joint = joints_[transmission.joints_[0].name_];
			if (transmission.type_ == "transmission_interface/SimpleTransmission") {
				ROS_INFO_STREAM_NAMED("franka_hw_sim",
				                      "Found transmission interface of joint " << joint->name << " : " << k_interface);
				if (k_interface == "hardware_interface/EffortJointInterface") {
					ROS_DEBUG_STREAM_NAMED("franka_hw_sim", "Initializing effort command handle for joint " << joint->name);
					initEffortCommandHandle(joint);
					continue;
				}

				if (k_interface == "hardware_interface/PositionJointInterface") {
					// Initiate position motion generator (PID controller)
					joint->position_controller.initParam(robot_namespace + "/motion_generators/position/gains/" +
					                                     joint->name);

					initPositionCommandHandle(joint);
					continue;
				}

				if (k_interface == "hardware_interface/VelocityJointInterface") {
					// Initiate velocity motion generator (PID controller)
					joint->velocity_controller.initParam(robot_namespace + "/motion_generators/velocity/gains/" +
					                                     joint->name);

					initVelocityCommandHandle(joint);
					continue;
				}
			}

			if (transmission.type_ == "franka_hw/FrankaStateInterface") {
				ROS_INFO_STREAM_NAMED("franka_hw_sim", "Found transmission interface '" << transmission.type_ << "'");
				try {
					initFrankaStateHandle(arm_id_, *urdf, transmission);
					continue;
				} catch (const std::invalid_argument &e) {
					ROS_ERROR_STREAM_NAMED("franka_hw_sim", e.what());
					return false;
				}
			}

			if (transmission.type_ == "franka_hw/FrankaModelInterface") {
				ROS_INFO_STREAM_NAMED("franka_hw_sim", "Found transmission interface '" << transmission.type_ << "'");
				double singularity_threshold;
				model_nh.param<double>("singularity_warning_threshold", singularity_threshold, -1);
				try {
					initFrankaModelHandle(arm_id_, *urdf, transmission, singularity_threshold);
					continue;
				} catch (const std::invalid_argument &e) {
					ROS_ERROR_STREAM_NAMED("franka_hw_sim", e.what());
					return false;
				}
				ROS_WARN_STREAM_NAMED("franka_hw_sim", "Unsupported transmission interface of joint '"
				                                           << joint->name << " : " << k_interface);
			}
		}
	}

	// After all handles have been assigned to interfaces, register them
	assert(this->eji_.getNames().size() >= 7);
	assert(this->pji_.getNames().size() >= 7);
	assert(this->vji_.getNames().size() >= 7);
	assert(this->jsi_.getNames().size() >= 7);
	assert(this->fsi_.getNames().size() == 1);
	assert(this->fmi_.getNames().size() == 1);

	registerInterface(&this->eji_);
	registerInterface(&this->pji_);
	registerInterface(&this->vji_);
	registerInterface(&this->jsi_);
	registerInterface(&this->fsi_);
	registerInterface(&this->fmi_);

	serviceServers.push_back(model_nh.advertiseService("franka_control/set_EE_frame", &FrankaHWSim::setEEFrameCB, this));
	serviceServers.push_back(model_nh.advertiseService("franka_control/set_k_frame", &FrankaHWSim::setKFrameCB, this));
	serviceServers.push_back(model_nh.advertiseService("franka_control/set_load", &FrankaHWSim::setLoadCB, this));
	serviceServers.push_back(model_nh.advertiseService("franka_control/set_force_torque_collision_behavior",
	                                                   &FrankaHWSim::setCollisionBehaviorCB, this));
	serviceServers.push_back(model_nh.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(
	    "franka_control/set_user_stop", [&](auto &request, auto &response) {
		    sm_.process_event(UserStop{ static_cast<bool>(request.data) });
		    response.success = true;
		    return true;
	    }));
	serviceServers.push_back(
	    model_nh.advertiseService<mujoco_ros_msgs::SetFloat::Request, mujoco_ros_msgs::SetFloat::Response>(
	        "franka_control/set_position_noise_sigma", [&](auto &request, auto &response) {
		        if (MujocoSim::detail::settings_.eval_mode) {
			        ROS_DEBUG_NAMED("franka_hw_sim", "Evaluation mode is active. Checking hash validity");
			        if (MujocoSim::detail::settings_.admin_hash != request.admin_hash) {
				        ROS_ERROR_NAMED("franka_hw_sim",
				                        "Hash mismatch, no permission to change joint position noise distribution!");
				        response.success = false;
				        return true;
			        }
			        ROS_DEBUG_NAMED("franka_hw_sim", "Hash valid, change authorized.");
		        }
		        noise_dist.reset(new std::normal_distribution<double>(0.0, request.value));
		        response.success = true;
		        return true;
	        }));

	service_controller_list_ =
	    model_nh.serviceClient<controller_manager_msgs::ListControllers>("controller_manager/list_controllers");
	service_controller_switch_ =
	    model_nh.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller");

	verifier_ = std::make_unique<ControllerVerifier>(joints_, arm_id_);
	return readParameters(model_nh, *urdf);
}

void FrankaHWSim::restartControllers()
{
	// Restart controllers by stopping and starting all running ones
	auto name = service_controller_list_.getService();
	if (not service_controller_list_.waitForExistence(ros::Duration(3))) {
		throw std::runtime_error("Cannot find service '" + name + "'. Is the controller_manager running?");
	}

	controller_manager_msgs::ListControllers list;
	if (not service_controller_list_.call(list)) {
		throw std::runtime_error("Service call '" + name + "' failed");
	}

	controller_manager_msgs::SwitchController swtch;
	for (const auto &controller : list.response.controller) {
		if (controller.state != "running") {
			continue;
		}
		swtch.request.stop_controllers.push_back(controller.name);
		swtch.request.start_controllers.push_back(controller.name);
	}

	swtch.request.start_asap = static_cast<decltype(swtch.request.start_asap)>(true);
	swtch.request.strictness = controller_manager_msgs::SwitchControllerRequest::STRICT;
	if (not service_controller_switch_.call(swtch) or not static_cast<bool>(swtch.response.ok)) {
		throw std::runtime_error("Service call '" + service_controller_switch_.getService() + "' failed");
	}
}

void FrankaHWSim::initJointStateHandle(const std::shared_ptr<franka_mujoco::Joint> &joint)
{
	jsi_.registerHandle(
	    hardware_interface::JointStateHandle(joint->name, &joint->position, &joint->velocity, &joint->effort));
}

void FrankaHWSim::initEffortCommandHandle(const std::shared_ptr<franka_mujoco::Joint> &joint)
{
	eji_.registerHandle(hardware_interface::JointHandle(jsi_.getHandle(joint->name), &joint->command));
}

void FrankaHWSim::initPositionCommandHandle(const std::shared_ptr<franka_mujoco::Joint> &joint)
{
	pji_.registerHandle(hardware_interface::JointHandle(jsi_.getHandle(joint->name), &joint->desired_position));
}

void FrankaHWSim::initVelocityCommandHandle(const std::shared_ptr<franka_mujoco::Joint> &joint)
{
	vji_.registerHandle(hardware_interface::JointHandle(jsi_.getHandle(joint->name), &joint->desired_velocity));
}

void FrankaHWSim::initFrankaStateHandle(const std::string &robot, const urdf::Model &urdf,
                                        const transmission_interface::TransmissionInfo &transmission)
{
	if (transmission.joints_.size() != 7) {
		throw std::invalid_argument("Cannot create franka_hw/FrankaStateInterface for robot '" + robot +
		                            "_robot' because " + std::to_string(transmission.joints_.size()) +
		                            " joints were found beneath the <transmission> tag, but 7 are required.");
	}

	// Initialize robot_mode to "Idle". Once a controller is started, we will switch to "Move"
	robot_state_.robot_mode = franka::RobotMode::kIdle;

	// Check if all joints defined in the <trasmission> actually exists in the URDF
	for (const auto &joint : transmission.joints_) {
		if (!urdf.getJoint(joint.name_)) {
			throw std::invalid_argument("Cannot create franka_hw/FrankaStateInterface for robot '" + robot +
			                            "_robot' because" + " the specified joint '" + joint.name_ +
			                            "' in the <transmission> tag cannot be found in the URDF");
		}
		ROS_DEBUG_STREAM_NAMED("franka_hw_sim", "Found joint " << joint.name_ << " to belong to a Panda robot");
	}
	fsi_.registerHandle(franka_hw::FrankaStateHandle(robot + "_robot", robot_state_));
}

void FrankaHWSim::initFrankaModelHandle(const std::string &robot, const urdf::Model &urdf,
                                        const transmission_interface::TransmissionInfo &transmission,
                                        double singularity_threshold)
{
	if (transmission.joints_.size() != 2) {
		throw std::invalid_argument("Cannot create franka_hw/FrankaModelInterface for robot '" + robot +
		                            "_model' because " + std::to_string(transmission.joints_.size()) +
		                            " joints were found beneath the <transmission> tag, but 2 are required.");
	}

	for (const auto &joint : transmission.joints_) {
		if (!urdf.getJoint(joint.name_)) {
			throw std::invalid_argument("Cannot create franka_hw/FrankaModelInterface for robot '" + robot +
			                            "_model' because the specified joint '" + joint.name_ +
			                            "' in the <transmission> tag cannot be found in the URDF");
		}
	}

	auto root = std::find_if(transmission.joints_.begin(), transmission.joints_.end(),
	                         [&](const transmission_interface::JointInfo &i) { return i.role_ == "root"; });
	if (root == transmission.joints_.end()) {
		throw std::invalid_argument("Cannot create franka_hw/FrankaModelInterface for robot '" + robot +
		                            "_model' because no <joint> with <role>root</root> can be found "
		                            "in the <transmission>");
	}

	auto tip = std::find_if(transmission.joints_.begin(), transmission.joints_.end(),
	                        [&](const transmission_interface::JointInfo &i) { return i.role_ == "tip"; });
	if (tip == transmission.joints_.end()) {
		throw std::invalid_argument("Cannot create franka_hw/FrankaModelInterface for robot '" + robot +
		                            "_model' because no <joint> with <role>tip</role> can be found "
		                            "in the <transmission>");
	}

	try {
		auto root_link = urdf.getJoint(root->name_)->parent_link_name;
		auto tip_link  = urdf.getJoint(tip->name_)->child_link_name;

		model_ = std::make_unique<franka_mujoco::ModelKDL>(urdf, root_link, tip_link, singularity_threshold);
	} catch (const std::invalid_argument &e) {
		throw std::invalid_argument("Cannot create franka_hw/FrankaModelInterface for robot '" + robot + "_model'. " +
		                            e.what());
	}
	fmi_.registerHandle(franka_hw::FrankaModelHandle(robot + "_model", *this->model_, robot_state_));
}

bool FrankaHWSim::setLoadCB(franka_msgs::SetLoad::Request &req, franka_msgs::SetLoad::Response &res)
{
	ROS_INFO_STREAM_NAMED("franka_hw_sim", arm_id_ << ": Setting Load");
	robot_state_.m_load = req.mass;
	std::copy(req.F_x_center_load.cbegin(), req.F_x_center_load.cend(), robot_state_.F_x_Cload.begin());
	std::copy(req.load_inertia.cbegin(), req.load_inertia.cend(), robot_state_.I_load.begin());
	updateRobotStateDynamics();
	res.success = true;
	return true;
}

bool FrankaHWSim::setEEFrameCB(franka_msgs::SetEEFrame::Request &req, franka_msgs::SetEEFrame::Response &res)
{
	ROS_INFO_STREAM_NAMED("franka_hw_sim", arm_id_ << ": Setting NE_T_EE transformation");
	std::copy(req.NE_T_EE.cbegin(), req.NE_T_EE.cend(), robot_state_.NE_T_EE.begin());
	updateRobotStateDynamics();
	res.success = true;
	return true;
}

bool FrankaHWSim::setKFrameCB(franka_msgs::SetKFrame::Request &req, franka_msgs::SetKFrame::Response &res)
{
	ROS_INFO_STREAM_NAMED("franka_hw_sim", arm_id_ << ": Setting EE_T_K transformation");
	std::copy(req.EE_T_K.cbegin(), req.EE_T_K.cend(), robot_state_.EE_T_K.begin());
	updateRobotStateDynamics();
	res.success = true;
	return true;
}

bool FrankaHWSim::setCollisionBehaviorCB(franka_msgs::SetForceTorqueCollisionBehavior::Request &req,
                                         franka_msgs::SetForceTorqueCollisionBehavior::Response &res)
{
	ROS_INFO_STREAM_NAMED("franka_hw_sim", arm_id_ << ": Setting Collision Behavior");

	for (int i = 0; i < 7; i++) {
		std::string name                   = arm_id_ + "_joint" + std::to_string(i + 1);
		joints_[name]->contact_threshold   = req.lower_torque_thresholds_nominal.at(i);
		joints_[name]->collision_threshold = req.upper_torque_thresholds_nominal.at(i);
	}

	std::move(req.lower_force_thresholds_nominal.begin(), req.lower_force_thresholds_nominal.end(),
	          lower_force_thresholds_nominal_.begin());

	std::move(req.upper_force_thresholds_nominal.begin(), req.upper_force_thresholds_nominal.end(),
	          upper_force_thresholds_nominal_.begin());

	res.success = true;
	return true;
}

void FrankaHWSim::readSim(ros::Time time, ros::Duration period)
{
	for (const auto &pair : joints_) {
		auto joint = pair.second;

		if (noise_dist) {
			joint->update(period, (*noise_dist)(rand_generator));
		} else {
			joint->update(period);
		}
	}
	updateRobotState(time);
}

double FrankaHWSim::positionControl(Joint &joint, double setpoint, const ros::Duration &period)
{
	double error;
	const double kJointLowerLimit = joint.limits.min_position;
	const double kJointUpperLimit = joint.limits.max_position;
	switch (joint.type) {
		case urdf::Joint::REVOLUTE:
			angles::shortest_angular_distance_with_limits(joint.position, setpoint, kJointLowerLimit, kJointUpperLimit,
			                                              error);
			break;
		case urdf::Joint::PRISMATIC:
			error = boost::algorithm::clamp(setpoint - joint.position, kJointLowerLimit, kJointUpperLimit);
			break;
		default:
			std::string error_message = "Only revolute or prismatic joints are allowed for position control right now";
			ROS_FATAL("%s", error_message.c_str());
			throw std::invalid_argument(error_message);
	}

	return boost::algorithm::clamp(joint.position_controller.computeCommand(error, period), -joint.limits.max_effort,
	                               joint.limits.max_effort);
}

double FrankaHWSim::velocityControl(Joint &joint, double setpoint, const ros::Duration &period)
{
	return boost::algorithm::clamp(joint.velocity_controller.computeCommand(setpoint - joint.velocity, period),
	                               -joint.limits.max_effort, joint.limits.max_effort);
}

void FrankaHWSim::writeSim(ros::Time time, ros::Duration period)
{
	// Update gravity, since it currently can be changed at runtime in mujoco_ros
	auto g = model_->gravity(robot_state_, { m_ptr_->opt.gravity[0], m_ptr_->opt.gravity[1], m_ptr_->opt.gravity[2] });

	for (auto &pair : joints_) {
		auto joint = pair.second;

		// Retrieve effort control command
		double effort = 0;

		// Finger joints must still be controllable from franka_gripper_mujoco controller
		if (not sm_.is(state<Move>) and not contains(pair.first, "finger_joint")) {
			effort = positionControl(*joint, joint->stop_position, period);
		} else if (joint->control_method == POSITION) {
			effort = positionControl(*joint, joint->desired_position, period);
		} else if (joint->control_method == VELOCITY) {
			velocityControl(*joint, joint->desired_velocity, period);
		} else if (joint->control_method == EFFORT) {
			// Feed-forward commands in effort control
			effort = joint->command;
		}

		// Check if this joint is affected by gravity compensation
		std::string prefix = this->arm_id_ + "_joint";
		if (pair.first.rfind(prefix, 0) != std::string::npos) {
			int i          = std::stoi(pair.first.substr(prefix.size())) - 1;
			joint->gravity = g.at(i);
		}

		effort += joint->gravity;

		// send effort control command
		if (not std::isfinite(effort)) {
			ROS_WARN_STREAM_NAMED("franka_hw_sim", "Command for " << joint->name << " is not finite, won't send to robot");
			continue;
		}
		d_ptr_->qfrc_applied[m_ptr_->jnt_dofadr[joint->id]] = effort;
	}
}

bool FrankaHWSim::readParameters(const ros::NodeHandle &nh, const urdf::Model &urdf)
{
	try {
		guessEndEffector(nh, urdf);

		nh.param<double>("m_load", robot_state_.m_load, 0);

		std::string I_load; // NOLINT [readability-identifier-naming]
		nh.param<std::string>("I_load", I_load, "0 0 0 0 0 0 0 0 0");
		robot_state_.I_load = readArray<9>(I_load, "I_load");

		std::string F_x_Cload; // NOLINT [readability-identifier-naming]
		nh.param<std::string>("F_x_Cload", F_x_Cload, "0 0 0");
		robot_state_.F_x_Cload = readArray<3>(F_x_Cload, "F_x_Cload");

		std::string NE_T_EE; // NOLINT [readability-identifier-naming]
		nh.param<std::string>("NE_T_EE", NE_T_EE, "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1");

		robot_state_.NE_T_EE = readArray<16>(NE_T_EE, "NE_T_EE");

		std::string EE_T_K; // NOLINT [readability-identifier-naming]
		nh.param<std::string>("EE_T_K", EE_T_K, "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1");
		robot_state_.EE_T_K = readArray<16>(EE_T_K, "EE_T_K");

		// TODO: Currently doesn't do anything. Gravity is taken directly from MuJoCo
		// The gravity can be changed on the fly, how should it be updated?
		std::string gravity_vector;
		if (nh.getParam("gravity_vector", gravity_vector)) {
			gravity_earth_ = readArray<3>(gravity_vector, "gravity_vector");
		}

		// Only nominal cases supported for now
		std::vector<double> lower_torque_thresholds = franka_hw::FrankaHW::getCollisionThresholds(
		    "lower_torque_thresholds_nominal", nh, { 20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0 });

		std::vector<double> upper_torque_thresholds = franka_hw::FrankaHW::getCollisionThresholds(
		    "upper_torque_thresholds_nominal", nh, { 20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0 });

		lower_force_thresholds_nominal_ = franka_hw::FrankaHW::getCollisionThresholds(
		    "lower_force_thresholds_nominal", nh, { 20.0, 20.0, 20.0, 25.0, 25.0, 25.0 });
		upper_force_thresholds_nominal_ = franka_hw::FrankaHW::getCollisionThresholds(
		    "upper_force_thresholds_nominal", nh, { 20.0, 20.0, 20.0, 25.0, 25.0, 25.0 });

		for (int i = 0; i < 7; i++) {
			std::string name                   = arm_id_ + "_joint" + std::to_string(i + 1);
			joints_[name]->contact_threshold   = lower_torque_thresholds.at(i);
			joints_[name]->collision_threshold = upper_torque_thresholds.at(i);
		}
	} catch (const std::invalid_argument &e) {
		ROS_ERROR_STREAM_NAMED("franka_hw_sim", e.what());
		return false;
	}

	updateRobotStateDynamics();
	return true;
}

void FrankaHWSim::guessEndEffector(const ros::NodeHandle &nh, const urdf::Model &urdf)
{
	auto hand_link = arm_id_ + "_hand";
	auto hand      = urdf.getLink(hand_link);

	if (hand != nullptr) {
		ROS_INFO_STREAM_NAMED("franka_hw_sim", "Found link '" << hand_link
		                                                      << "' in URDF. Assuming it is defining the kinematics & "
		                                                         "inertias of the Franka hand gripper");
	}

	// By absolute default unless URDF or ROS params say otherwise, assume no end-effector
	double def_m_ee         = 0;
	std::string def_i_ee    = "0.0 0 0 0 0.0 0 0 0 0.0";
	std::string def_f_x_cee = "0 0 0";
	std::string def_f_t_ne  = "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1";
	if (!nh.hasParam("F_T_NE") && hand != nullptr) {
		// NOTE: We cannot interpret the Joint pose from the URDF directly, because its <arm_id>_link is mounted at the
		// flange directly and not at NE
		def_f_t_ne = "0.7071 -0.7071 0 0 0.7071 0.7071 0 0 0 0 1 0 0 0 0.1034 1";
	}
	std::string F_T_NE; // NOLINT [readability-identifier-naming]
	nh.param<std::string>("F_T_NE", F_T_NE, def_f_t_ne);
	robot_state_.F_T_NE = readArray<16>(F_T_NE, "F_T_NE");

	if (!nh.hasParam("m_ee") && hand != nullptr) {
		if (hand->inertial == nullptr) {
			throw std::invalid_argument("Trying to use inertia of " + hand_link +
			                            " but this link has no <intertial> tag defined in it.");
		}
		def_m_ee = hand->inertial->mass;
	}
	nh.param<double>("m_ee", robot_state_.m_ee, def_m_ee);

	if (!nh.hasParam("I_ee") && hand != nullptr) {
		if (hand->inertial == nullptr) {
			throw std::invalid_argument("Trying to use inertia of " + hand_link +
			                            " but this link has no <intertial> tag defined in it.");
		}
		//clang-format off
		def_i_ee = std::to_string(hand->inertial->ixx) + " " + std::to_string(hand->inertial->ixy) + " " +
		           std::to_string(hand->inertial->ixz) + " " + std::to_string(hand->inertial->ixy) + " " +
		           std::to_string(hand->inertial->iyy) + " " + std::to_string(hand->inertial->iyz) + " " +
		           std::to_string(hand->inertial->ixz) + " " + std::to_string(hand->inertial->iyz) + " " +
		           std::to_string(hand->inertial->izz);
		// clang-format on
	}
	std::string I_ee; // NOLINT [readability-identifier-naming]
	nh.param<std::string>("I_ee", I_ee, def_i_ee);
	robot_state_.I_ee = readArray<9>(I_ee, "I_ee");

	if (!nh.hasParam("F_x_Cee") && hand != nullptr) {
		if (hand->inertial == nullptr) {
			throw std::invalid_argument("Trying to use inertia of " + hand_link +
			                            " but this link has no <intertial> tag defined in it.");
		}
		def_f_x_cee = std::to_string(hand->inertial->origin.position.x) + " " +
		              std::to_string(hand->inertial->origin.position.y) + " " +
		              std::to_string(hand->inertial->origin.position.z);
	}
	std::string F_x_Cee; // NOLINT [readability-identifier-naming]
	nh.param<std::string>("F_x_Cee", F_x_Cee, def_f_x_cee);
	robot_state_.F_x_Cee = readArray<3>(F_x_Cee, "F_x_Cee");
}

void FrankaHWSim::updateRobotStateDynamics()
{
	robot_state_.m_total = robot_state_.m_ee + robot_state_.m_load;

	Eigen::Map<Eigen::Matrix4d>(robot_state_.F_T_EE.data()) =
	    Eigen::Matrix4d(robot_state_.F_T_NE.data()) * Eigen::Matrix4d(robot_state_.NE_T_EE.data());

	Eigen::Map<Eigen::Matrix3d>(robot_state_.I_total.data()) = shiftInertiaTensor(
	    Eigen::Matrix3d(robot_state_.I_ee.data()), robot_state_.m_ee, Eigen::Vector3d(robot_state_.F_x_Cload.data()));
}

void FrankaHWSim::updateRobotState(ros::Time time)
{
	// This is ensured, because a FrankaStateInterface checks for at least 7 joints in the URDF
	assert(joints_.size() >= 7);

	auto mode = robot_state_.robot_mode;
	for (int i = 0; i < 7; i++) {
		std::string name       = arm_id_ + "_joint" + std::to_string(i + 1);
		const auto &joint      = joints_.at(name);
		robot_state_.q[i]      = joint->position;
		robot_state_.dq[i]     = joint->velocity;
		robot_state_.tau_J[i]  = joint->effort;
		robot_state_.dtau_J[i] = joint->jerk;

		robot_state_.q_d[i]     = joint->getDesiredPosition(mode);
		robot_state_.dq_d[i]    = joint->getDesiredVelocity(mode);
		robot_state_.ddq_d[i]   = joint->getDesiredAcceleration(mode);
		robot_state_.tau_J_d[i] = joint->getDesiredTorque(mode);

		// For now we assume flexible joints
		robot_state_.theta[i]  = joint->position;
		robot_state_.dtheta[i] = joint->velocity;

		// first time initialization of desired position
		if (not robot_initialized_) {
			joint->desired_position = joint->position;
			joint->stop_position    = joint->position;
		}

		if (robot_initialized_) {
			double tau_ext = joint->effort - joint->command + joint->gravity;

			// Exponential moving average filter from tau_ext -> tau_ext_hat_filtered
			robot_state_.tau_ext_hat_filtered[i] =
			    tau_ext_lowpass_filter_ * tau_ext + (1 - tau_ext_lowpass_filter_) * robot_state_.tau_ext_hat_filtered[i];
		}

		robot_state_.joint_contact[i]   = static_cast<double>(joint->isInContact());
		robot_state_.joint_collision[i] = static_cast<double>(joint->isInCollision());
	}

	// Calculate estimated wrenches in Task frame from external joint torques with jacobians
	Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_ext(robot_state_.tau_ext_hat_filtered.data());
	Eigen::MatrixXd j0_transpose_pinv;
	Eigen::MatrixXd jk_transpose_pinv;
	Eigen::Matrix<double, 6, 7> j0(model_->zeroJacobian(franka::Frame::kStiffness, robot_state_).data());
	Eigen::Matrix<double, 6, 7> jk(model_->bodyJacobian(franka::Frame::kStiffness, robot_state_).data());

	franka_example_controllers::pseudoInverse(j0.transpose(), j0_transpose_pinv);
	franka_example_controllers::pseudoInverse(jk.transpose(), jk_transpose_pinv);

	Eigen::VectorXd f_ext_0                                 = j0_transpose_pinv * tau_ext;
	Eigen::VectorXd f_ext_k                                 = jk_transpose_pinv * tau_ext;
	Eigen::VectorXd::Map(&robot_state_.O_F_ext_hat_K[0], 6) = f_ext_0;
	Eigen::VectorXd::Map(&robot_state_.K_F_ext_hat_K[0], 6) = f_ext_k;

	for (int i = 0; i < robot_state_.cartesian_contact.size(); i++) {
		// Evaluate the cartesian contact/collisions in K frame
		double fi                           = std::abs(f_ext_k(i));
		robot_state_.cartesian_contact[i]   = static_cast<double>(fi > lower_force_thresholds_nominal_.at(i));
		robot_state_.cartesian_collision[i] = static_cast<double>(fi > upper_force_thresholds_nominal_.at(i));
	}

	robot_state_.control_command_success_rate = 1.0;
	robot_state_.time                         = franka::Duration(time.toNSec() / 1e6 /*ms*/);
	robot_state_.O_T_EE                       = model_->pose(franka::Frame::kEndEffector, robot_state_);

#ifdef ENABLE_BASE_ACCELERATION
	// This will always be {0,0,-9.81} on the real robot as it cannot be mounted differently for now
	robot_state_.O_ddP_O = gravity_earth_;
#endif

	std_msgs::Bool msg;
	msg.data           = static_cast<decltype(msg.data)>(true);
	robot_initialized_ = true;
	robot_initialized_pub_.publish(msg);
}

bool FrankaHWSim::prepareSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                                const std::list<hardware_interface::ControllerInfo> & /*stop_list*/)
{
	return std::all_of(start_list.cbegin(), start_list.cend(),
	                   [this](const auto &controller) { return verifier_->isValidController(controller); });
}

void FrankaHWSim::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                           const std::list<hardware_interface::ControllerInfo> &stop_list)
{
	forControlledJoint(stop_list, [](franka_mujoco::Joint &joint, const ControlMethod /*&method*/) {
		joint.control_method   = boost::none;
		joint.stop_position    = joint.position;
		joint.desired_position = joint.position;
		joint.desired_velocity = 0;
	});
	forControlledJoint(start_list, [](franka_mujoco::Joint &joint, const ControlMethod &method) {
		joint.control_method = method;
		// sets the desired joint position once for the effort interface
		joint.desired_position = joint.position;
		joint.desired_velocity = 0;
	});

	sm_.process_event(SwitchControl());
}

void FrankaHWSim::forControlledJoint(const std::list<hardware_interface::ControllerInfo> &controllers,
                                     const std::function<void(franka_mujoco::Joint &joint, const ControlMethod &)> &f)
{
	for (const auto &controller : controllers) {
		if (not verifier_->isClaimingArmController(controller)) {
			continue;
		}
		for (const auto &resource : controller.claimed_resources) {
			auto control_method = ControllerVerifier::determineControlMethod(resource.hardware_interface);
			if (not control_method) {
				continue;
			}
			for (const auto &joint_name : resource.resources) {
				auto &joint = joints_.at(joint_name);
				f(*joint, control_method.value());
			}
		}
	}
}

void FrankaHWSim::eStopActive(bool /* active */) {}

} // namespace franka_mujoco

PLUGINLIB_EXPORT_CLASS(franka_mujoco::FrankaHWSim, mujoco_ros_control::RobotHWSim)
