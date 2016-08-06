#include "gait.hpp"


bool Gait::init(void)
{
	ik_client = node.serviceClient<quadruped_msgs::GetLegIKSolver>("/get_ik");
	ROS_INFO("Waiting for /get_ik server");
	ros::service::waitForService("/get_ik");
	ROS_INFO("Connected to server");


	move = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/quadruped/quadruped_joint_controller/follow_joint_trajectory", true);
	ROS_INFO("Waiting for follow_joint_trajectory server");
	move->waitForServer();
	ROS_INFO("Connected to server");

	middle_points.resize(4);
	middle_points[0].x = -150.0/1000;
	middle_points[0].y = 100.0/1000;
	middle_points[0].z = -130.0/1000;
	middle_points[1].x = -150.0/1000;
	middle_points[1].y = -100.0/1000;
	middle_points[1].z = -130.0/1000;
	middle_points[2].x = 150.0/1000;
	middle_points[2].y = -100.0/1000;
	middle_points[2].z = -130.0/1000;
	middle_points[3].x = 150.0/1000;
	middle_points[3].y = 100.0/1000;
	middle_points[3].z = -130.0/1000;

	trajectory_joints.resize(4);

	leg_number = 4;

	return true;
}

bool Gait::ClearTrajectories(void)
{
	trajectories.clear();
}


void Gait::StraightCrawl(double step, double total_distance)
{
	std::vector<geometry_msgs::Point> start_points, control_points, end_points;
	std::vector<std::vector<geometry_msgs::Point> > bspline_trajs;
	start_points.resize(4);
	control_points.resize(4);
	end_points.resize(4);
	bspline_trajs.resize(4);

	trajectories.resize(4);

	for (size_t i = 0; i < 4; i++) {
		start_points[i].x = middle_points[i].x;
		start_points[i].y = middle_points[i].y - 40.0/1000.0;
		start_points[i].z = middle_points[i].z;

		control_points[i].x = middle_points[i].x;
		control_points[i].y = middle_points[i].y;
		control_points[i].z = middle_points[i].z + 50.0/1000.0;

		end_points[i].x = middle_points[i].x;
		end_points[i].y = middle_points[i].y + 40.0/1000.0;
		end_points[i].z = middle_points[i].z;

		bspline_trajs[i].push_back(start_points[i]);
		bspline_trajs[i].push_back(control_points[i]);
		bspline_trajs[i].push_back(end_points[i]);
	}
	//const double p_update = 1.0/20.0;
	//const double body_speed = 50.0/1000.0;
	//double total_time = total_distance/body_speed;
	//double step_time = total_time * step / total_distance;
	//double step_split_num = step_time/p_update;
	//double remainder = total_distance - step * (double)(int)(total_distance/step);

	//std::vector<std::vector<geometry_msgs::Point> > tmp_trajs;
	//tmp_trajs.resize(4);
	trajectories[0].AddBSpline(bspline_trajs[0], 30, false);
	trajectories[2].AddBSpline(bspline_trajs[2], 30, false);
	trajectories[1].AddStraightLine(end_points[1], start_points[1], 30);
	trajectories[3].AddStraightLine(end_points[3], start_points[3], 30);

	trajectories[1].AddBSpline(bspline_trajs[1], 30, false);
	trajectories[3].AddBSpline(bspline_trajs[3], 30, false);
	trajectories[0].AddStraightLine(end_points[0], start_points[0], 30);
	trajectories[2].AddStraightLine(end_points[2], start_points[2], 30);

	//for (size_t leg_i = 0; leg_i < 4; leg_i++) {
	//	std::vector<int32_t> leg_number;
	//	for (size_t i = 0; i < trajectories[leg_i].trajectory.size(); i++)
	//		leg_number.push_back(leg_i);
	//	trajectories[leg_i].AddPoints(leg_number, tmp_trajs[leg_i]);
	//	//std::cout << "leg_i " << leg_i << " leg_trajs.size " << leg_trajs[leg_i].size() << std::endl;
	//}
}

bool Gait::IK_Trajectories(void)
{
	trajectory_joints.clear();
	//response.error_codes = response.IK_FOUND;

	quadruped_msgs::GetLegIKSolver srv;
	srv.request.leg_number.clear();
	srv.request.current_joints.clear();
	srv.request.target_points.clear();

	quadruped_msgs::LegJointState cur_leg_joint;
	cur_leg_joint.joint[0] = 90;
	cur_leg_joint.joint[1] = 90;
	cur_leg_joint.joint[2] = 90;
	for (size_t leg_i = 0; leg_i < leg_number; leg_i++) {
		for (size_t j = 0; j < trajectories[leg_i].trajectory.size(); j++) {
			srv.request.leg_number.push_back(leg_i);
			srv.request.current_joints.push_back(cur_leg_joint);
			srv.request.target_points.push_back(trajectories[leg_i].trajectory[j]);
		}
	}

	if (ik_client.call(srv) == false) {
		ROS_ERROR("Failed to call /get_ik");
		return false;
	} else {
		if (srv.response.error_codes == srv.response.IK_FOUND) {
			ROS_INFO("IK solution was found");
			for (size_t i = 0; i < srv.response.target_joints.size(); i++) {
				trajectory_joints[srv.request.leg_number[i]].push_back(srv.response.target_joints[i]);
				//ROS_INFO_STREAM("[!!From Leg_Trajectory] Target Joints" << srv.response.target_joints[i]);
			}
			//trajectory_joints.push_back(srv.response.target_joints[0]);
		} else if (srv.response.error_codes == srv.response.IK_NOT_FOUND) {
			ROS_ERROR("An IK solution could not be found");
			return false;
			//response.error_codes = response.IK_NOT_FOUND;
		}
	}

	return true;
}

bool Gait::RunTrajectories(void)
{
	trajectory_msgs::JointTrajectory traj;
	traj.joint_names.push_back("base_trochanter_0");
	traj.joint_names.push_back("base_trochanter_1");
	traj.joint_names.push_back("base_trochanter_2");
	traj.joint_names.push_back("base_trochanter_3");
	traj.joint_names.push_back("trochanter_femur_0");
	traj.joint_names.push_back("trochanter_femur_1");
	traj.joint_names.push_back("trochanter_femur_2");
	traj.joint_names.push_back("trochanter_femur_3");
	traj.joint_names.push_back("femur_tibia_0");
	traj.joint_names.push_back("femur_tibia_1");
	traj.joint_names.push_back("femur_tibia_2");
	traj.joint_names.push_back("femur_tibia_3");

	for (size_t loop = 0; loop < 100; loop++) {
	for (size_t i = 0; i < trajectory_joints[0].size(); i++) {
		trajectory_msgs::JointTrajectoryPoint pt;
		for (size_t j = 0; j < 4; j++) {
			//pt.positions.push_back(3.14*((double)trajectory_joints[j][i].joint[0]-90.0)/180.0);
			//pt.positions.push_back(3.14*((double)trajectory_joints[j][i].joint[1]-90.0)/180.0);
			//pt.positions.push_back(3.14*((double)trajectory_joints[j][i].joint[2]-90.0)/180.0);
			pt.positions.push_back((trajectory_joints[j][i].joint[0]/180l - 0.5l) * M_PI);
		}
		for (size_t j = 0; j < 4; j++) {
			switch (j) {
				case 0:
					pt.positions.push_back(trajectory_joints[j][i].joint[1]/180*M_PI - M_PI/2);
					break;
				case 1:
					pt.positions.push_back(trajectory_joints[j][i].joint[1]/180*M_PI - M_PI/2);
					break;
				case 2:
					pt.positions.push_back(M_PI/2 - trajectory_joints[j][i].joint[1]/180*M_PI);
					break;
				case 3:
					pt.positions.push_back(M_PI/2 - trajectory_joints[j][i].joint[1]/180*M_PI);
					break;
			}
		}
		for (size_t j = 0; j < 4; j++) {
			switch (j) {
				case 0:
					pt.positions.push_back(trajectory_joints[j][i].joint[2]/180*M_PI - M_PI/2);
					break;
				case 1:
					pt.positions.push_back(trajectory_joints[j][i].joint[2]/180*M_PI - M_PI/2);
					break;
				case 2:
					pt.positions.push_back(M_PI/2 - trajectory_joints[j][i].joint[2]/180*M_PI);
					break;
				case 3:
					pt.positions.push_back(M_PI/2 - trajectory_joints[j][i].joint[2]/180*M_PI);
					break;
			}
		}
		for (size_t j = 0; j < 12; j++) {
			//pt.positions.push_back(1.0 - (double)(i+1) * 1.0 / 10.0);
			pt.velocities.push_back(0.0);
			pt.accelerations.push_back(0.0);
			ROS_INFO_STREAM("posisions " << i << " " << j << " " << pt.positions[j]);
		}
		pt.time_from_start = ros::Duration(0.005*(i+1+loop*trajectory_joints[0].size()));
		traj.points.push_back(pt);
	}
	}
	traj.header.stamp = ros::Time::now() + ros::Duration(1.0);

	control_msgs::FollowJointTrajectoryGoal goal;
	goal.trajectory = traj;

	for (size_t i = 0; i < 12; i++) {
		control_msgs::JointTolerance jt;
		jt.name=traj.joint_names[i];
		jt.position = 0.0;
		jt.velocity = -1;
		jt.acceleration = -1;
		goal.goal_tolerance.push_back(jt);
	}

	ROS_INFO("Send goal");
	move->sendGoal(goal);

	bool finished_within_time = move->waitForResult(ros::Duration(15.0));
	if (!finished_within_time) {
		move->cancelGoal();
		ROS_INFO("Timed out achieving goal A");
	} else {
		actionlib::SimpleClientGoalState state = move->getState();
		if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
			ROS_INFO("Action finished: %s", state.toString().c_str());
		} else {
			control_msgs::FollowJointTrajectoryResult result;
			result = *move->getResult();
			ROS_INFO("Action failed: %s", state.toString().c_str());
			ROS_WARN("Addition Information: %s", state.text_.c_str());
			ROS_WARN("Result Error Code: %d", result.error_code);
			ROS_WARN_STREAM("Result Error String: " << result.error_string);
		}
	}

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "quadruped_gait");
	Gait gait;
	if (gait.init() == false) {
		ROS_ERROR("Failed to init gait");
		return 0;
	}
	ros::Duration(1).sleep();
	gait.ClearTrajectories();
	std::vector<geometry_msgs::Point> trajectory;
	trajectory.resize(2);

	trajectory[0].x = 0.0;
	trajectory[0].y = 0.0;
	trajectory[0].z = 0.0;

	trajectory[1].x = 000.0/1000.0;
	trajectory[1].y = 600.0/1000.0;
	trajectory[1].z = 0.0/1000.0;

	ROS_INFO("StraightCrawl");
	gait.StraightCrawl(10, 10);
	ROS_INFO("IK_Trajectories");
	gait.IK_Trajectories();
	ROS_INFO("RunTrajectories");
	while (1) {
		gait.RunTrajectories();
	}

	//ros::spin();
	return 0;
}
