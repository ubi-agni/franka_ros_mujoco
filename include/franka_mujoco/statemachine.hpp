#pragma once

#include <franka/robot_state.h>
#include <franka_mujoco/joint.hpp>

#include <ros/ros.h>
#include <boost_sml/sml.hpp>
#include <map>
#include <memory>
#include <string>

namespace franka_mujoco {

using JointMap = std::map<std::string, std::shared_ptr<Joint>>;

// States
struct Idle
{};
struct Move
{};
struct UserStopped
{};

// Events
struct ErrorRecovery
{};
struct SwitchControl
{};
struct UserStop
{
	bool pressed;
};

// Guards
inline const auto contains = [](const auto &haystack, const auto &needle) {
	return haystack.find(needle) != std::string::npos;
};
inline const auto isPressed  = [](const UserStop &event) { return event.pressed; };
inline const auto isReleased = [](const UserStop &event, const JointMap &joints) { return not event.pressed; };
inline const auto isStarting = [](const SwitchControl &event, const JointMap &joints) {
	for (auto &joint : joints) {
		if (contains(joint.first, "_finger_joint")) {
			continue;
		}
		if (joint.second->control_method) {
			return true;
		}
	}
	return false;
};
inline const auto isStopping = [](const SwitchControl &event, const JointMap &joints) { return not isStarting(event, joints); };

// Actions
inline const auto start = [](franka::RobotState &state) { state.robot_mode = franka::RobotMode::kMove; };
inline const auto idle  = [](franka::RobotState &state) { state.robot_mode = franka::RobotMode::kIdle; };
inline const auto stop  = [](franka::RobotState &state, JointMap &joints) {
   ROS_WARN("User stop pressed, stopping robot");
   state.robot_mode = franka::RobotMode::kUserStopped;
   state.q_d        = state.q;
   state.dq_d       = { 0 };
   state.ddq_d      = { 0 };

   for (auto &joint : joints) {
      if (contains(joint.first, "_finger_joint")) {
         continue;
      }
      joint.second->stop_position    = joint.second->position;
      joint.second->desired_position = joint.second->position;
      joint.second->desired_velocity = 0;
   }
};

struct StateMachine
{
	auto operator()() const
	{
		using namespace boost::sml;
		return make_transition_table(
		    // clang-format off
       *state<Idle>        + event<SwitchControl>[isStarting] / start = state<Move>,
        state<Idle>        + event<UserStop>[isPressed]       / stop  = state<UserStopped>,
        state<Idle>        + event<ErrorRecovery>             / start = state<Move>,
        state<Move>        + event<SwitchControl>[isStopping] / idle  = state<Idle>,
        state<Move>        + event<UserStop>[isPressed]       / stop  = state<UserStopped>,
        state<UserStopped> + event<UserStop>[isReleased]      / idle  = state<Idle>
		    // clang-format on
		);
	}
};
} // namespace franka_mujoco
