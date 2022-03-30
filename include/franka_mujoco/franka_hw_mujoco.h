#pragma once

#include <ros/ros.h>

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include <franka/robot_state.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/model_base.h>

#include <franka_hw/services.h>
#include <franka_msgs/SetEEFrame.h>
#include <franka_msgs/SetForceTorqueCollisionBehavior.h>
#include <franka_msgs/SetKFrame.h>
#include <franka_msgs/SetLoad.h>

#include <urdf/model.h>

#include <franka_mujoco/joint.h>

#include <controller_manager/controller_manager.h>
#include <transmission_interface/transmission_parser.h>

#include <Eigen/Dense>

namespace franka_mujoco {

class FrankaHWSim : public hardware_interface::RobotHW
{
public:
	FrankaHWSim(ros::NodeHandle &hn);
	void readSim(ros::Time time, ros::Duration period);
	void writeSim(ros::Time time, ros::Duration period);

	// Service callbacks
	bool setEEFrameCB(franka_msgs::SetEEFrame::Request &req, franka_msgs::SetEEFrame::Response &rep);
	bool setKFrameCB(franka_msgs::SetKFrame::Request &req, franka_msgs::SetKFrame::Response &rep);
	bool setLoadCB(franka_msgs::SetLoad::Request &req, franka_msgs::SetLoad::Response &rep);
	bool setCollisionBehaviorCB(franka_msgs::SetForceTorqueCollisionBehavior::Request &req,
	                            franka_msgs::SetForceTorqueCollisionBehavior::Response &rep);

	bool setBodyPoseCB(franka_msgs::SetLoad::Request &req, franka_msgs::SetLoad::Response &rep);

private:
	/// If gazebo::Joint::GetForceTorque() yielded already a non-zero value
	bool efforts_initialized_;
	std::array<double, 3> gravity_earth_;

	// For now this is constant because nothing else is supported
	const std::string arm_id_ = "panda";

	std::map<std::string, std::shared_ptr<franka_mujoco::Joint>> joints_;

	hardware_interface::JointStateInterface jsi_;
	hardware_interface::EffortJointInterface eji_;
	franka_hw::FrankaStateInterface fsi_;
	franka_hw::FrankaModelInterface fmi_;

	franka::RobotState robot_state_;
	std::unique_ptr<franka_hw::ModelBase> model_;

	urdf::Model *urdf_model_ptr_;

	boost::shared_ptr<controller_manager::ControllerManager> cm_;

	std::vector<transmission_interface::TransmissionInfo> transmissions_;
	std::string robot_namespace_;
	std::string robot_description_;

	const double kDefaultTauExtLowpassFilter = 1.0; // no filtering per default of tau_ext_hat_filtered
	double tau_ext_lowpass_filter_;

	std::vector<double> lower_force_thresholds_nominal_;
	std::vector<double> upper_force_thresholds_nominal_;

	std::vector<ros::ServiceServer> serviceServers;

	void initFrankaStateHandle(const std::string &robot, const urdf::Model &urdf,
	                           const transmission_interface::TransmissionInfo &transmission);
	void initFrankaModelHandle(const std::string &robot, const urdf::Model &urdf,
	                           const transmission_interface::TransmissionInfo &transmission,
	                           double singularity_threshold);

	void initJointStateHandle(const std::shared_ptr<franka_mujoco::Joint> &joint);
	void initEffortCommandHandle(const std::shared_ptr<franka_mujoco::Joint> &joint);

	void initServices();

	void updateRobotState(ros::Time time);
	void updateRobotStateDynamics();

	bool readParameters(const ros::NodeHandle &nh, const urdf::Model &urdf);
	void guessEndEffector(const ros::NodeHandle &nh, const urdf::Model &urdf);

	std::string getURDF(const ros::NodeHandle &nh, std::string robot_description);
	void queueThread();
	bool initRobot(ros::NodeHandle &nh);

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
};

} // namespace franka_mujoco
