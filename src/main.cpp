#include <controller_manager/controller_manager.h>

#include <franka_mujoco/mujoco_sim_proxy.h>
#include <franka_mujoco/franka_hw_mujoco.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "FrankaMujoco");

	ros::AsyncSpinner spinner(4);
	spinner.start();

	ros::NodeHandle nh = ros::NodeHandle("~");

	bool vis = true;

	std::string filename;

	nh.getParam("modelfile", filename);
	ROS_INFO_STREAM("Using modelfile " << filename);

	if (filename.empty()) {
		ROS_FATAL("No modelfile was provided, simulation cannot start!");
		return -1;
	}

	std::thread sim_thread(MujocoSimProxy::init, filename);

	franka_mujoco::FrankaHWSim hw_sim(nh);
	controller_manager::ControllerManager cm(&hw_sim);

	double control_rate = 0.001;
	ros::Rate loop_rate(1 / control_rate);
	ros::Duration d;
	ros::Time last_time(ros::Time::now());

	while (ros::ok() && MujocoSimProxy::isUp()) {
		// d Should always be 0.001 because of the rate which depends on sim time,
		// but we compute the difference just to double check.
		ros::Time t(ros::Time::now());
		d = t - last_time;

		hw_sim.readSim(t, d);
		cm.update(t, d);
		hw_sim.writeSim(t, d);

		last_time = t;
		loop_rate.sleep();
	}

	MujocoSimProxy::requestExternalShutdow();
	sim_thread.join();

	ROS_INFO("Franka MuJoCo node is terminating");
}
