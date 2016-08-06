#ifndef GAIT_HPP
#define GAIT_HPP

#include <ros/ros.h>

#include <vector>

#include <quadruped_msgs/GetLegIKSolver.h>
#include <quadruped_msgs/LegTrajectoryCommand.h>
#include <quadruped_msgs/LegTrajectoryAddPoint.h>
#include <quadruped_msgs/LegJointState.h>
//#include <visualization_msgs/Marker.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTolerance.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "trajectory.hpp"

class Gait {
	public:
		bool init(void);
		bool ClearTrajectories(void);
		void StraightCrawl(double step, double total_distance);
		bool IK_Trajectories(void);
		bool RunTrajectories(void);

	private:
		ros::NodeHandle node, private_node;
		ros::ServiceClient ik_client;
		std::vector<geometry_msgs::Point> middle_points;
		std::vector<Trajectory> trajectories;
		std::vector<std::vector<quadruped_msgs::LegJointState> > trajectory_joints;
		actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *move;

		size_t leg_number;


};

#endif // GAIT_HPP
