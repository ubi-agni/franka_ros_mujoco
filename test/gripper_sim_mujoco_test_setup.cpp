#include "gripper_sim_mujoco_test_setup.h"
#include <mujoco_ros_msgs/SetBodyState.h>
#include <mujoco_ros_msgs/GetStateUint.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <thread>

void GripperSimTestSetup::SetUp()
{
	ros::ServiceClient service =
	    n.serviceClient<mujoco_ros_msgs::GetStateUint>("/mujoco_server/get_loading_request_state");
	mujoco_ros_msgs::GetStateUint srv;
	ASSERT_TRUE(service.call(srv)) << "Could not get loading request state";
	while (srv.response.state.value > 0) {
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		ASSERT_TRUE(service.call(srv));
	}
	homing_client =
	    std::make_unique<actionlib::SimpleActionClient<franka_gripper::HomingAction>>("franka_gripper/homing", true);
	move_client =
	    std::make_unique<actionlib::SimpleActionClient<franka_gripper::MoveAction>>("franka_gripper/move", true);
	grasp_client =
	    std::make_unique<actionlib::SimpleActionClient<franka_gripper::GraspAction>>("franka_gripper/grasp", true);
	homing_client->waitForServer();
	move_client->waitForServer();
	grasp_client->waitForServer();
	resetStone();
	updateFingerState();
}

void GripperSimTestSetup::updateFingerState()
{
	auto msg       = ros::topic::waitForMessage<sensor_msgs::JointState>("/franka_gripper/joint_states", n);
	finger_1_pos   = msg->position.at(0);
	finger_2_pos   = msg->position.at(1);
	finger_1_force = msg->effort.at(0);
	finger_2_force = msg->effort.at(1);
}

void GripperSimTestSetup::resetStone()
{
	ros::ServiceClient service = n.serviceClient<mujoco_ros_msgs::SetBodyState>("/mujoco_server/set_body_state");
	service.waitForExistence(ros::Duration(5.0));
	mujoco_ros_msgs::SetBodyState srv;
	srv.request.state.name                    = "stone";
	srv.request.state.pose.pose.position.x    = 0.56428;
	srv.request.state.pose.pose.position.y    = -0.221972;
	srv.request.state.pose.pose.position.z    = 0.475121;
	srv.request.state.pose.pose.orientation.w = 1.0;
	srv.request.state.pose.header.frame_id    = "world";
	service.call(srv);
	if (not srv.response.success) {
		ROS_ERROR("resetting stone failed");
	}
}
