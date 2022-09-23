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

#pragma once

#include <ros/ros.h>

#include <mujoco_ros_control/robot_hw_sim.h>

#include <actionlib/server/simple_action_server.h>

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include <franka/robot_state.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/model_base.h>

#include <franka_hw/services.h>

#include <urdf/model.h>

#include <franka_mujoco/controller_verifier.h>
#include <franka_mujoco/joint.h>
#include <franka_mujoco/statemachine.h>

#include <franka_msgs/ErrorRecoveryAction.h>
#include <franka_msgs/SetEEFrame.h>
#include <franka_msgs/SetForceTorqueCollisionBehavior.h>
#include <franka_msgs/SetKFrame.h>
#include <franka_msgs/SetLoad.h>

#include <controller_manager/controller_manager.h>
#include <transmission_interface/transmission_parser.h>

#include <Eigen/Dense>
#include <boost/optional.hpp>
#include <boost_sml/sml.hpp>

#include <mutex>

namespace franka_mujoco {

/**
 * A custom implementation of a robot hardware interface,
 * which is able to simulate franka interfaces in MuJoCo.
 *
 * Specifically it supports the following hardware transmission types
 *
 * ### transmission_interface/SimpleTransmission
 * - hardware_interface/JointStateInterface
 * - hardware_interface/EffortJointInterface
 * - hardware_interface/PositionJointInterface
 * - hardware_interface/VelocityJointInterface
 *
 * ### franka_hw/FrankaStateInterface
 * ### franka_hw/FrankaModelInterface
 *
 */
class FrankaHWSim : public mujoco_ros_control::RobotHWSim
{
public:
	/**
	 * Create a new FrankaHWSim instance
	 */
	FrankaHWSim();

	/**
	 * Initialize the simulated robot hardware and parse all supported transmissions.
	 *
	 * @param[in] m MuJoCo model handle.
	 * @param[in] d MuJoCo data handle.
	 * @param[in] robot_namespace the name of the robot passed inside the rosparam config. Should match the
	 * `<robotNamespace>` tag from the URDF
	 * @param[in] model_nh root node handle of the node into which this plugin is loaded (usually
	 * the mujoco server node)
	 * @param[in] urdf the parsed URDF which should be added
	 * @param[in] transmissions a list of transmissions of the model which should be simulated
	 * @return `true` if initialization succeeds, `false` otherwise
	 */
	bool initSim(MujocoSim::mjModelPtr m, MujocoSim::mjDataPtr d, const std::string &robot_namespace,
	             ros::NodeHandle model_nh, const urdf::Model *const urdf,
	             std::vector<transmission_interface::TransmissionInfo> transmissions) override;

	/**
	 * Fetch data from the MuJoCo simulation and pass it on to the hardware interfaces.
	 *
	 * This will e.g. read the joint positions, velocities and efforts and write them out to
	 * controllers via the
	 [JointStateInterface](http://docs.ros.org/en/jade/api/hardware_interface/html/c++/classhardware__interface_1_1JointStateInterface.html)
	and/or `franka_hw::FrankaStateInterface`

	*
	* @param[in] time   the current (simulated) ROS time
	* @param[in] period the time step at which the simulation is running
	*/
	void readSim(ros::Time time, ros::Duration period) override;

	/**
	 * Pass the data send from controllers via the hardware interfaces onto the simulation.
	 *
	 * This will e.g. write the joint commands (torques or forces) to the corresponding joint in
	 * MuJoCo in each timestep. These commands are usually sent via an
	 * [EffortJointInterface](http://docs.ros.org/en/jade/api/hardware_interface/html/c++/classhardware__interface_1_1EffortJointInterface.html)
	 *
	 * @param[in] time   the current (simulated) ROS time
	 * @param[in] period the time step at which the simulation is running
	 */
	void writeSim(ros::Time time, ros::Duration period) override;

	/**
	 * Set the emergency stop state (not yet implemented)
	 *
	 * @param[in] active does currently nothing.
	 */
	void eStopActive(const bool active) override;

	/**
	 * Switches the control mode of the robot arm
	 *
	 * @param start_list list of controllers to start
	 * @param stop_list list of controllers to stop
	 */
	void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
	              const std::list<hardware_interface::ControllerInfo> &stop_list) override;

	/**
	 * Check (in non-realtime) if given controllers could be started and stopped from the current state of the RobotHW
	 * with regard to necessary hardware interface switches and prepare the switching. Start and stop list are disjoint.
	 * This handles the check and preparation, the actual switch is commited in doSwitch().
	 * @param start_list list of controllers to start
	 * @param stop_list list of controllers to stop
	 */
	bool prepareSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
	                   const std::list<hardware_interface::ControllerInfo> & /*stop_list*/) override;

	// Service callbacks
	bool setEEFrameCB(franka_msgs::SetEEFrame::Request &req, franka_msgs::SetEEFrame::Response &rep);
	bool setKFrameCB(franka_msgs::SetKFrame::Request &req, franka_msgs::SetKFrame::Response &rep);
	bool setLoadCB(franka_msgs::SetLoad::Request &req, franka_msgs::SetLoad::Response &rep);
	bool setCollisionBehaviorCB(franka_msgs::SetForceTorqueCollisionBehavior::Request &req,
	                            franka_msgs::SetForceTorqueCollisionBehavior::Response &rep);

private:
	MujocoSim::mjModelPtr m_ptr_;
	MujocoSim::mjDataPtr d_ptr_;

	bool robot_initialized_;

	std::unique_ptr<ControllerVerifier> verifier_;

	std::array<double, 3> gravity_earth_;

	std::string arm_id_;

	std::map<std::string, std::shared_ptr<franka_mujoco::Joint>> joints_;

	hardware_interface::JointStateInterface jsi_;
	hardware_interface::EffortJointInterface eji_;
	hardware_interface::PositionJointInterface pji_;
	hardware_interface::VelocityJointInterface vji_;
	franka_hw::FrankaStateInterface fsi_;
	franka_hw::FrankaModelInterface fmi_;

	boost::sml::sm<franka_mujoco::StateMachine, boost::sml::thread_safe<std::mutex>> sm_;
	franka::RobotState robot_state_;
	std::unique_ptr<franka_hw::ModelBase> model_;

	const double kDefaultTauExtLowpassFilter = 1.0; // no filtering per default of tau_ext_hat_filtered
	double tau_ext_lowpass_filter_;

	std::vector<double> lower_force_thresholds_nominal_;
	std::vector<double> upper_force_thresholds_nominal_;

	ros::Publisher robot_initialized_pub_;
	std::vector<ros::ServiceServer> serviceServers;
	ros::ServiceClient service_controller_list_;
	ros::ServiceClient service_controller_switch_;
	std::unique_ptr<actionlib::SimpleActionServer<franka_msgs::ErrorRecoveryAction>> action_recovery_;

	void initFrankaStateHandle(const std::string &robot, const urdf::Model &urdf,
	                           const transmission_interface::TransmissionInfo &transmission);
	void initFrankaModelHandle(const std::string &robot, const urdf::Model &urdf,
	                           const transmission_interface::TransmissionInfo &transmission,
	                           double singularity_threshold);

	void initJointStateHandle(const std::shared_ptr<franka_mujoco::Joint> &joint);
	void initEffortCommandHandle(const std::shared_ptr<franka_mujoco::Joint> &joint);
	void initPositionCommandHandle(const std::shared_ptr<franka_mujoco::Joint> &joint);
	void initVelocityCommandHandle(const std::shared_ptr<franka_mujoco::Joint> &joint);

	void initServices();

	void restartControllers();

	void updateRobotState(ros::Time time);
	void updateRobotStateDynamics();

	bool readParameters(const ros::NodeHandle &nh, const urdf::Model &urdf);
	void guessEndEffector(const ros::NodeHandle &nh, const urdf::Model &urdf);

	static double positionControl(Joint &joint, double setpoint, const ros::Duration &period);
	static double velocityControl(Joint &joint, double setpoint, const ros::Duration &period);

	template <int N>
	std::array<double, N> readArray(std::string param, std::string name = "")
	{
		std::array<double, N> x;

		std::istringstream iss(param);
		std::vector<std::string> values{ std::istream_iterator<std::string>{ iss },
			                              std::istream_iterator<std::string>{} };
		if (values.size() != N) {
			throw std::invalid_argument("Expected parameter '" + name + "' to have exactely " + std::to_string(N) +
			                            " numbers separated by spaces, but found " + std::to_string(values.size()));
		}
		std::transform(values.begin(), values.end(), x.begin(), [](std::string v) -> double { return std::stod(v); });
		return x;
	}

	/**
	 * Helper function for generating a skew symmetric matrix for a given input vector such  that:
	 * \f$\mathbf{0} = \mathbf{M} \cdot \mathrm{vec}\f$
	 *
	 * @param[in] vec the 3D input vector for which to generate the matrix for
	 * @return\f$\mathbf{M}\f$ i.e. a skew symmetric matrix for `vec`
	 */
	static Eigen::Matrix3d skewMatrix(const Eigen::Vector3d &vec)
	{
		Eigen::Matrix3d vec_hat;
		// clang-format off
                vec_hat <<
                          0, -vec(2),  vec(1),
                     vec(2),       0, -vec(0),
                    -vec(1),  vec(0),       0;
		// clang-format on
		return vec_hat;
	}

	/**
	 * Shift the moment of inertia tensor by a given offset.
	 *
	 * This method is based on Steiner's [Parallel Axis
	 * Theorem](https://de.wikipedia.org/wiki/Steinerscher_Satz#Verallgemeinerung_auf_Tr%C3%A4gheitstensoren)
	 *
	 * \f$\mathbf{I^{(p)}} = \mathbf{I} + m \tilde{p}^\top \tilde{p}\f$
	 *
	 * where \f$\tilde{p}\f$ is the @ref skewMatrix of `p`
	 *
	 * @param[in] I the inertia tensor defined in the original frame or center or mass of `m`
	 * @param[in] m the mass of the body in \f$kg\f$
	 * @param[in] p the offset vector to move the inertia tensor along starting from center of mass
	 * @return the shifted inertia tensor \f$\mathbf{I^{\left( p \right)}}\f$
	 */
	static Eigen::Matrix3d shiftInertiaTensor(Eigen::Matrix3d I, double m, Eigen::Vector3d p)
	{
		Eigen::Matrix3d P  = skewMatrix(p);
		Eigen::Matrix3d Ip = I + m * P.transpose() * P;
		return Ip;
	}

	void forControlledJoint(const std::list<hardware_interface::ControllerInfo> &controllers,
	                        const std::function<void(franka_mujoco::Joint &joint, const ControlMethod &)> &f);
};

} // namespace franka_mujoco
