#include <urdf/model.h>

#include <franka_mujoco/joint.h>
#include <franka_mujoco/mujoco_sim_proxy.h>

namespace franka_mujoco {

void Joint::update(const ros::Duration &dt)
{
	std::array<double, 4> sim_vals = MujocoSimProxy::getJointData(id);

	position     = sim_vals[0];
	velocity     = sim_vals[1];
	acceleration = sim_vals[2];
	effort       = sim_vals[3];

	if (std::isnan(lastVelocity))
		lastVelocity = velocity;

	jerk             = (acceleration - lastAcceleration) / dt.toSec();
	lastAcceleration = acceleration;
}

double Joint::getLinkMass() const
{
	return MujocoSimProxy::getBodyMass(id);
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
