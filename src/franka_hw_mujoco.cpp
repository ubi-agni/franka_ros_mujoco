#include <franka_hw/franka_hw.h>
#include <franka_example_controllers/pseudo_inversion.h>

#include <franka_mujoco/franka_hw_mujoco.h>
#include <franka_mujoco/mujoco_sim_proxy.h>
#include <franka_mujoco/model_kdl.h>

namespace franka_mujoco {
FrankaHWSim::FrankaHWSim(ros::NodeHandle &nh)
{
	robot_description_ = "robot_description";

	const std::string urdf_string = getURDF(nh, robot_description_);
	ROS_DEBUG_NAMED("franka_hw_sim", "Parsing URDF for transmissions...");
	transmission_interface::TransmissionParser::parse(urdf_string, transmissions_);

	urdf::Model urdf_model;
	urdf_model_ptr_ = urdf_model.initString(urdf_string) ? &urdf_model : NULL;

	if (!initRobot(nh)) {
		ROS_FATAL_NAMED("franka_hw_sim", "Could not initialize robot simulation interface");
		return;
	}
}

std::string FrankaHWSim::getURDF(const ros::NodeHandle &nh, std::string robot_description)
{
	std::string urdf_string;

	// Search on and wait for param server
	while (urdf_string.empty()) {
		std::string search_param_name;
		if (nh.searchParam(robot_description, search_param_name)) {
			ROS_INFO_ONCE_NAMED("franka_hw_sim", "waiting for URDF in parameter [%s] on parameter server",
			                    search_param_name.c_str());

			nh.getParam(search_param_name, urdf_string);
		} else {
			ROS_INFO_ONCE_NAMED("franka_hw_sim", "waiting for URDF in parameter [%s] on parameter server",
			                    robot_description.c_str());

			nh.getParam(robot_description, urdf_string);
		}

		usleep(100000);
	}
	ROS_DEBUG_STREAM_NAMED("franka_hw_sim", "Received URDF from parameter server");
	return urdf_string;
}

bool FrankaHWSim::initRobot(ros::NodeHandle &nh)
{
	efforts_initialized_ = false;

	// Default to 'panda' as arm_id
	nh.param<std::string>("arm_id", arm_id_, "panda");
	ROS_DEBUG_STREAM_NAMED("franka_hw_sim", "arm_id is '" << arm_id_ << "'");

	nh.param<double>("tau_ext_lowpass_filter", tau_ext_lowpass_filter_, kDefaultTauExtLowpassFilter);
	ROS_DEBUG_NAMED("franka_hw_sim", "tau is: %f", tau_ext_lowpass_filter_);

	// Wait for sim to be ready since we need to fetch information from it
	while (!MujocoSimProxy::isSimReady()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	std::array<double, 3> gravity = MujocoSimProxy::getGravity();
	ROS_DEBUG_NAMED("franka_hw_sim", "Sim Gravity is: %.2f %.2f %.2f", gravity[0], gravity[1], gravity[2]);

	// Generate list of franka_mujoco::Joint to store all relevant information
	for (const auto &transmission : transmissions_) {
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
		auto joint  = std::make_shared<franka_mujoco::Joint>();
		joint->name = transmission.joints_[0].name_;

		if (urdf_model_ptr_ == NULL) {
			ROS_ERROR_STREAM_NAMED("franka_hw_sim",
			                       "Could not find any URDF model. Was it loaded on the parameter server?");
			return false;
		}

		auto urdf_joint = urdf_model_ptr_->getJoint(joint->name);
		if (!urdf_joint) {
			ROS_ERROR_STREAM_NAMED("franka_hw_sim", "Could not get joint '"
			                                            << joint->name
			                                            << "' from URDF. Make sure naming inside the URDF and between "
			                                               "URDF and MuJoCo XML is consistent!");
			return false;
		}
		joint->type = urdf_joint->type;
		ROS_DEBUG_STREAM_NAMED("franka_hw_sim",
		                       "Creating joint " << joint->name << " of transmission type " << transmission.type_);
		joint->axis = Eigen::Vector3d(urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z);

		int id = MujocoSimProxy::jointName2id(joint->name);
		if (id == -1) {
			ROS_ERROR_STREAM_NAMED("franka_hw_sim", "Could not get joint '"
			                                            << joint->name << "' from MuJoCo model."
			                                            << " Make sure naming between URDF and MuJoCo XML is consistent!");
			return false;
		}
		joint->id = id;
		joints_.emplace(joint->name, joint);
	}

	// After the joint data containers have been fully initialized and their memory addresses don't
	// change anymore, get the respective addresses to pass them to the handles

	for (auto &pair : joints_) {
		initJointStateHandle(pair.second);
	}

	// Register all supported command interfaces
	for (auto &transmission : transmissions_) {
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
			}

			if (transmission.type_ == "franka_hw/FrankaStateInterface") {
				ROS_INFO_STREAM_NAMED("franka_hw_sim", "Found transmission interface '" << transmission.type_ << "'");
				try {
					initFrankaStateHandle(arm_id_, *urdf_model_ptr_, transmission);
					continue;
				} catch (const std::invalid_argument &e) {
					ROS_ERROR_STREAM_NAMED("franka_hw_sim", e.what());
					return false;
				}
			}

			if (transmission.type_ == "franka_hw/FrankaModelInterface") {
				ROS_INFO_STREAM_NAMED("franka_hw_sim", "Found transmission interface '" << transmission.type_ << "'");
				double singularity_threshold;
				nh.param<double>("singularity_warning_threshold", singularity_threshold, -1);
				try {
					initFrankaModelHandle(arm_id_, *urdf_model_ptr_, transmission, singularity_threshold);
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
	registerInterface(&this->eji_);
	registerInterface(&this->jsi_);
	registerInterface(&this->fsi_);
	registerInterface(&this->fmi_);

	// serviceServers.push_back(nh.advertiseService("set_load", setBodyPoseCB, &this));

	serviceServers.push_back(nh.advertiseService("set_EE_frame", &FrankaHWSim::setEEFrameCB, this));
	serviceServers.push_back(nh.advertiseService("set_k_frame", &FrankaHWSim::setKFrameCB, this));
	serviceServers.push_back(nh.advertiseService("set_load", &FrankaHWSim::setLoadCB, this));
	serviceServers.push_back(
	    nh.advertiseService("set_force_torque_collision_behavior", &FrankaHWSim::setCollisionBehaviorCB, this));

	return readParameters(nh, *urdf_model_ptr_);
}

// bool FrankaHWSim::setBodyPoseCB(franka_msgs::SetLoad::Request &req, franka_msgs::SetLoad::Response &rep) {

// }

void FrankaHWSim::initJointStateHandle(const std::shared_ptr<franka_mujoco::Joint> &joint)
{
	jsi_.registerHandle(
	    hardware_interface::JointStateHandle(joint->name, &joint->position, &joint->velocity, &joint->effort));
}

void FrankaHWSim::initEffortCommandHandle(const std::shared_ptr<franka_mujoco::Joint> &joint)
{
	eji_.registerHandle(hardware_interface::JointHandle(jsi_.getHandle(joint->name), &joint->command));
}

void FrankaHWSim::initFrankaStateHandle(const std::string &robot, const urdf::Model &urdf,
                                        const transmission_interface::TransmissionInfo &transmission)
{
	if (transmission.joints_.size() != 7) {
		throw std::invalid_argument("Cannot create franka_hw/FrankaStateInterface for robot '" + robot +
		                            "_robot' because " + std::to_string(transmission.joints_.size()) +
		                            " joints were found beneath the <transmission> tag, but 7 are required.");
	}

	// Check if all joints defined in the <trasmission> actually exists in the URDF
	for (const auto &joint : transmission.joints_) {
		if (!urdf.getJoint(joint.name_)) {
			throw std::invalid_argument("Cannot create franka_hw/FrankaStateInterface for robot '" + robot +
			                            "_robot' because" + " the specified joint '" + joint.name_ +
			                            "' in the <transmission> tag cannot be found in the URDF");
		}
		ROS_DEBUG_STREAM_NAMED("franka_hw_sim", "Found joint " << joint.name_ << " to belog to a Panda robot");
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
	// ROS_INFO_NAMED("mujoco_sim_hw", "Running read sim update");
	for (const auto &pair : joints_) {
		auto joint = pair.second;
		joint->update(period);
	}
	updateRobotState(time);
}

void FrankaHWSim::writeSim(ros::Time time, ros::Duration period)
{
	MujocoSimProxy::sim_mtx.lock();

	auto g = model_->gravity(robot_state_, gravity_earth_);

	for (auto &pair : joints_) {
		auto joint = pair.second;
		// Check if this joint is affected by gravity compensation
		std::string prefix = this->arm_id_ + "_joint";
		if (pair.first.rfind(prefix, 0) != std::string::npos) {
			int i          = std::stoi(pair.first.substr(prefix.size())) - 1;
			joint->gravity = g.at(i);
		}

		auto command = joint->command + joint->gravity;

		if (std::isnan(command)) {
			ROS_WARN_STREAM_NAMED("franka_hw_sim", "Command for " << joint->name << "is NaN, won't send to robot");
			continue;
		}
		MujocoSimProxy::setJointEffort(command, joint->id);
	}
	MujocoSimProxy::sim_mtx.unlock();
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
		robot_state_.EE_T_K = readArray<16>(NE_T_EE, "NE_T_EE");

		std::string EE_T_K; // NOLINT [readability-identifier-naming]
		nh.param<std::string>("EE_T_K", EE_T_K, "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1");
		robot_state_.EE_T_K = readArray<16>(EE_T_K, "EE_T_K");

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
		    "lower_torque_thresholds_nominal", nh, { 20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0 });
		upper_force_thresholds_nominal_ = franka_hw::FrankaHW::getCollisionThresholds(
		    "upper_torque_thresholds_nominal", nh, { 20.0, 20.0, 20.0, 25.0, 25.0, 25.0 });

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

	for (int i = 0; i < 7; i++) {
		std::string name       = arm_id_ + "_joint" + std::to_string(i + 1);
		const auto &joint      = joints_.at(name);
		robot_state_.q[i]      = joint->position;
		robot_state_.dq[i]     = joint->velocity;
		robot_state_.tau_J[i]  = joint->effort;
		robot_state_.dtau_J[i] = joint->jerk;

		// Since we don't support position or velocity interface yet, we set the deisred joint trajectory to zero
		// indicating we are in torque control mode
		robot_state_.q_d[i]     = joint->position; // special case for resetting motion generators
		robot_state_.dq_d[i]    = 0;
		robot_state_.ddq_d[i]   = 0;
		robot_state_.tau_J_d[i] = joint->command;

		// For now we assume flexible joints
		robot_state_.theta[i]  = joint->position;
		robot_state_.dtheta[i] = joint->velocity;

		if (efforts_initialized_) {
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

	efforts_initialized_ = true;
}

} // namespace franka_mujoco
