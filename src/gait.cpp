#include "gait.hpp"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>


bool Gait::init(void)
{
	trajectory_command_client = node.serviceClient<quadruped_msgs::LegTrajectoryCommand>("leg_trajectory/command", 1000);
	trajectory_add_point_client = node.serviceClient<quadruped_msgs::LegTrajectoryAddPoint>("leg_trajectory/add_point", 1000);
	ros::service::waitForService("leg_trajectory/command");
	ros::service::waitForService("leg_trajectory/add_point");

	middle_points.resize(4);
	//middle_points[0].x = -120.0/1000;
	//middle_points[0].y = 120.0/1000;
	//middle_points[0].z = 90.0/1000;
	//middle_points[1].x = -120.0/1000;
	//middle_points[1].y = -120.0/1000;
	//middle_points[1].z = 90.0/1000;
	//middle_points[2].x = 120.0/1000;
	//middle_points[2].y = -120.0/1000;
	//middle_points[2].z = 90.0/1000;
	//middle_points[3].x = 120.0/1000;
	//middle_points[3].y = 120.0/1000;
	//middle_points[3].z = 90.0/1000;

	//middle_points[0].x = -120.0/1000;
	//middle_points[0].y = 60.0/1000;
	//middle_points[0].z = -90.0/1000;
	//middle_points[1].x = -120.0/1000;
	//middle_points[1].y = -60.0/1000;
	//middle_points[1].z = -90.0/1000;
	//middle_points[2].x = 120.0/1000;
	//middle_points[2].y = -60.0/1000;
	//middle_points[2].z = -90.0/1000;
	//middle_points[3].x = 120.0/1000;
	//middle_points[3].y = 60.0/1000;
	//middle_points[3].z = -90.0/1000;

	middle_points[0].x = -100.0/1000;
	middle_points[0].y = 100.0/1000;
	middle_points[0].z = -130.0/1000;
	middle_points[1].x = -100.0/1000;
	middle_points[1].y = -100.0/1000;
	middle_points[1].z = -130.0/1000;
	middle_points[2].x = 100.0/1000;
	middle_points[2].y = -100.0/1000;
	middle_points[2].z = -130.0/1000;
	middle_points[3].x = 100.0/1000;
	middle_points[3].y = 100.0/1000;
	middle_points[3].z = -130.0/1000;


	//while (leg_trajectory_point_pub[i].getNumSubscribers() == 0 || leg_trajectory_command_pub[i].getNumSubscribers() == 0);
	marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	arrow.header.frame_id = "base_link";
	arrow.header.stamp = ros::Time::now();
	arrow.ns = "arrow";
	arrow.action = visualization_msgs::Marker::ADD;
	//arrow.pose.orientation.x = 0.0;
	//arrow.pose.orientation.y = 0.0;
	//arrow.pose.orientation.z = 0.0;
	//arrow.pose.orientation.w = 1.0;
	arrow.type = visualization_msgs::Marker::ARROW;
	arrow.scale.x = 0.002;
	arrow.scale.y = 0.004;
	//arrow.scale.z = 0.003;
	arrow.color.b = 1.0;
	arrow.color.a = 1.0;
	arrow.points.clear();

	return true;
}



bool Gait::CallLegTrajectory(quadruped_msgs::LegTrajectoryCommand& srv)
{
	ROS_INFO_STREAM("[!!From Gait] Call LegTrajectoryCommand " << srv.request.command);
	if (trajectory_command_client.call(srv) == false) {
		ROS_ERROR("Failed to call trajectory");
		return false;
	} else {
		ROS_INFO("Succeeded in callin server");
		if (srv.response.error_codes == srv.response.SUCCEEDED) {
			ROS_INFO("Succeeded in Calling LegTrajectory");
		} else {
			ROS_ERROR("Failed to Call LegTrajectory");
			return false;
		}
	}
	return true;
}



bool Gait::CalcLegTrajectory(void)
{
	quadruped_msgs::LegTrajectoryCommand srv;
	srv.request.command = srv.request.CALC;
	CallLegTrajectory(srv);
	if (srv.response.error_codes == srv.response.FAILED) {
		ROS_INFO("Failed to calc");
		return false;
	}

	return true;
}



bool Gait::RunLegTrajectory(void)
{
	quadruped_msgs::LegTrajectoryCommand srv;
	srv.request.command = srv.request.RUN;

	return CallLegTrajectory(srv);
	//quadruped_msgs::LegTrajectoryCommand command;
	//command.command = command.RUN;
	//command.exec_time = 2.0;
	//command.loop = true;
	//trajectory_command_pub.publish(command);
}

void Gait::ConcatinateDividedBodyTrajWithCrawl(size_t leg_i, size_t total_period_num, size_t update_per_one_period_num, const std::vector<geometry_msgs::Point>& splited_body_traj, std::vector<std::vector<geometry_msgs::Point> >& result_array_of_leg_traj)
{
	std::vector<geometry_msgs::Point> copied_splited_body_traj;
	std::copy(splited_body_traj.begin(), splited_body_traj.end(), std::back_inserter(copied_splited_body_traj));
	//for (size_t i = 0; i < total_period_num; i++) {
	//	std::vector<geometry_msgs::Point> tmp_array_of_trajectory;
	//	for (size_t j = 0; j < update_per_one_period_num*3/4; j++) {
	//		tmp_array_of_trajectory.push_back(splited_body_traj[i*update_per_one_period_num + j]);
	//	}

	//	result_array_of_leg_traj.push_back(tmp_array_of_trajectory);
	//}

	std::vector<geometry_msgs::Point> tmp_array_of_trajectory;
	switch (leg_i) {
		case 0:
			std::copy(splited_body_traj.begin(), splited_body_traj.begin() + update_per_one_period_num*(4-1)/4, std::back_inserter(tmp_array_of_trajectory));
			copied_splited_body_traj.erase(copied_splited_body_traj.begin(), copied_splited_body_traj.begin() + update_per_one_period_num*4/4);
			break;
		case 1:
			std::copy(splited_body_traj.begin(), splited_body_traj.begin() + update_per_one_period_num*(3-1)/4, std::back_inserter(tmp_array_of_trajectory));
			copied_splited_body_traj.erase(copied_splited_body_traj.begin(), copied_splited_body_traj.begin() + update_per_one_period_num*3/4);
			break;
		case 2:
			std::copy(splited_body_traj.begin(), splited_body_traj.begin() + update_per_one_period_num*(2-1)/4, std::back_inserter(tmp_array_of_trajectory));
			copied_splited_body_traj.erase(copied_splited_body_traj.begin(), copied_splited_body_traj.begin() + update_per_one_period_num*2/4);
			break;
		case 3:
			//std::copy(splited_body_traj.begin(), splited_body_traj.begin() + update_per_one_period_num*(1-1)/4, std::back_inserter(tmp_array_of_trajectory));
			//for (size_t i = 0; i < (update_per_one_period_num/4); i++) {
			//	tmp_array_of_trajectory.push_back(middle_points[3]);
			//}
			copied_splited_body_traj.erase(copied_splited_body_traj.begin(), copied_splited_body_traj.begin() + update_per_one_period_num*1/4);
			break;
	}
	result_array_of_leg_traj.push_back(tmp_array_of_trajectory);

	for (size_t i = 0; i < total_period_num; i++) {
		tmp_array_of_trajectory.clear();
		std::copy(copied_splited_body_traj.begin(), copied_splited_body_traj.begin() + update_per_one_period_num*3/4, std::back_inserter(tmp_array_of_trajectory));
		copied_splited_body_traj.erase(copied_splited_body_traj.begin(), copied_splited_body_traj.begin() + update_per_one_period_num);
		result_array_of_leg_traj.push_back(tmp_array_of_trajectory);
	}
}


void Gait::ConvertBodyTrajToLegTraj(size_t leg_i, std::vector<std::vector<geometry_msgs::Point> >& array_of_traj)
{
	for (size_t i = 0; i < array_of_traj.size(); i++) {
		if (array_of_traj[i].size() == 0)
			continue;
		geometry_msgs::Point middle_point_of_trajectory;
		middle_point_of_trajectory.x = (array_of_traj[i][0].x + array_of_traj[i][array_of_traj[i].size()-1].x)/2.0;
		middle_point_of_trajectory.y = (array_of_traj[i][0].y + array_of_traj[i][array_of_traj[i].size()-1].y)/2.0;
		middle_point_of_trajectory.z = (array_of_traj[i][0].z + array_of_traj[i][array_of_traj[i].size()-1].z)/2.0;
		for (size_t j = 0; j < array_of_traj[i].size(); j++) {
			array_of_traj[i][j].x -= (middle_point_of_trajectory.x - middle_points[leg_i].x);
			array_of_traj[i][j].y -= (middle_point_of_trajectory.y - middle_points[leg_i].y);
			array_of_traj[i][j].z -= (middle_point_of_trajectory.z - middle_points[leg_i].z);
		}
	}
}



void Gait::GenerateCrawlGait(double body_speed, double freq, const std::vector<geometry_msgs::Point>& body_traj)
{
	const double p_update = 1.0/20.0;
	const double duty_ratio = 0.75;

	const double total_period_num = 10;


	std::vector<std::vector<geometry_msgs::Point> > leg_trajs;
	leg_trajs.resize(4);

	double total_distance = 0.0;
	for (size_t i = 0; (i + 1) < body_traj.size(); i++) {
		total_distance += sqrt(pow(body_traj[i].x - body_traj[i+1].x, 2) +
				pow(body_traj[i].y - body_traj[i+1].y, 2));
	}
	double total_time = p_update * total_period_num * 4.0 * (double)((int)(total_distance/body_speed/p_update/total_period_num/4.0));
	double real_total_time = total_distance / body_speed;
	//double total_period_num = freq * total_time;
	double distance_by_period = total_distance / total_period_num;
	double time_by_period = total_time / total_period_num;
	size_t total_update_num = (size_t)(total_time / p_update);
	size_t update_per_one_period_num = total_update_num / total_period_num;

	ROS_INFO_STREAM("total_distance " << total_distance << " total_time " << total_time << " total_update_num " << total_update_num << " real_total_time " << real_total_time);

	// Divide body traj by update time;
	std::vector<geometry_msgs::Point> divided_body_traj;
	AddBSpline(body_traj, total_update_num+1, divided_body_traj, true);
	std::reverse(divided_body_traj.begin(), divided_body_traj.end());

	std::vector<geometry_msgs::Vector3> vectors;
	for (size_t i = 0; (i+1) < divided_body_traj.size(); i++) {
		geometry_msgs::Vector3 tmp_vector;
		tmp_vector.x = divided_body_traj[i+1].x - divided_body_traj[i].x;
		tmp_vector.y = divided_body_traj[i+1].y - divided_body_traj[i].y;
		tmp_vector.z = divided_body_traj[i+1].z - divided_body_traj[i].z;
		vectors.push_back(tmp_vector);

		arrow.id = i+1000;
		arrow.points.clear();
		arrow.points.push_back(divided_body_traj[i]);
		arrow.points.push_back(divided_body_traj[i+1]);
		//marker_pub.publish(arrow);
		//std::cout << "published " << i << " / " << divided_body_traj.size() << std::endl;
		//ros::Duration(0.01).sleep();
	}

	std::vector<std::vector<std::vector<geometry_msgs::Point> > > array_of_supporting_leg_traj;
	array_of_supporting_leg_traj.resize(4);

	for (size_t i = 0; i < 4; i++) {
		// 分割した胴体軌道を歩容に合わせてそれぞれの脚の支持時の軌道に変換
		ConcatinateDividedBodyTrajWithCrawl(i, total_period_num, update_per_one_period_num, divided_body_traj, array_of_supporting_leg_traj[i]);
		ConvertBodyTrajToLegTraj(i, array_of_supporting_leg_traj[i]);
		AddBSplineIdlingLegTraj(array_of_supporting_leg_traj[i], update_per_one_period_num/4.0, leg_trajs[i]);
	}

	for (size_t leg_i = 0; leg_i < 4; leg_i++) {
		std::vector<int32_t> leg_number;
		for (size_t i = 0; i < leg_trajs[leg_i].size(); i++)
			leg_number.push_back(leg_i);
		AddPointToTrajectory(leg_number, leg_trajs[leg_i]);
		//std::cout << "leg_i " << leg_i << " leg_trajs.size " << leg_trajs[leg_i].size() << std::endl;
	}

	CalcLegTrajectory();
}



void Gait::AddBSplineIdlingLegTraj(const std::vector<std::vector<geometry_msgs::Point> >& array_of_supporting_leg_traj, size_t num_of_spline, std::vector<geometry_msgs::Point>& result_array_of_supporting_leg_traj)
{

	for (size_t i = 0; i < array_of_supporting_leg_traj.size(); i++) {
		if (i != 0) {
			geometry_msgs::Point start_point, control_point, end_point;
			if (array_of_supporting_leg_traj[i-1].size() != 0)
				start_point = array_of_supporting_leg_traj[i-1][array_of_supporting_leg_traj[i-1].size() - 1];
			else
				start_point = middle_points[3];
			end_point = array_of_supporting_leg_traj[i][0];
			control_point.x = (start_point.x + end_point.x)/2;
			control_point.y = (start_point.y + end_point.y)/2;
			control_point.z = (start_point.z + end_point.z)/2 + 20.0/1000.0;

			std::vector<geometry_msgs::Point> spline_array_of_supporting_leg_traj;
			spline_array_of_supporting_leg_traj.push_back(start_point);
			spline_array_of_supporting_leg_traj.push_back(control_point);
			spline_array_of_supporting_leg_traj.push_back(end_point);

			AddBSpline(spline_array_of_supporting_leg_traj, num_of_spline, result_array_of_supporting_leg_traj, false);
		}
		for (size_t j = 0; j < array_of_supporting_leg_traj[i].size(); j++) {
			result_array_of_supporting_leg_traj.push_back(array_of_supporting_leg_traj[i][j]);
			//std::cout << array_of_supporting_leg_traj[i][j] << std::endl;
		}
	}
}



//void Gait::CalcMiddlePoint(const std::vector<geometry_msgs::Vector3> vectors)
//{
//}



//void Gait::SplitTrajWithLinearInterpolation(const std::vector<geometry_msgs::Point> trajectory, size_t number_of_split, std::vector<geometry_msgs::Point>& result_points)
//{
//}



void Gait::MoveLegDefault(void)
{
	std::vector<int32_t> leg_numbers = {0, 1, 2, 3};
	//ROS_INFO_STREAM("[!!From Gait] Call AddPoint " << srv.request.command);
	AddPointToTrajectory(leg_numbers, middle_points);
	ros::Duration wait(1.0);
	CalcLegTrajectory();
	RunLegTrajectory();
}

void Gait::InternalDivision(double t, const geometry_msgs::Point &start_point, const geometry_msgs::Point &end_point, geometry_msgs::Point &result_point)
{
	result_point.x = (1.0 - t) * start_point.x + end_point.x * t;
	result_point.y = (1.0 - t) * start_point.y + end_point.y * t;
	result_point.z = (1.0 - t) * start_point.z + end_point.z * t;
}



void Gait::AddStraightLine(const geometry_msgs::Point &start_position, const geometry_msgs::Point &end_position, size_t num, std::vector<geometry_msgs::Point> &result_positions)
{
	for (size_t i = 0; i < num; i++) {
		geometry_msgs::Point result_position;
		InternalDivision((double)i/(double)num, start_position, end_position, result_position);
		result_positions.push_back(result_position);
	}
}


void Gait::AddStraightLines(const std::vector<geometry_msgs::Point>& positions, const std::vector<size_t>& num_of_points, std::vector<geometry_msgs::Point>& result_positions)
{
	for (size_t i = 0; i < num_of_points.size(); i++) {
		AddStraightLine(positions[i], positions[i+1], num_of_points[i], result_positions);
	}
}



void Gait::CreateGait(const geometry_msgs::Point& A, const geometry_msgs::Point& B, const geometry_msgs::Point& C, const geometry_msgs::Point& D, size_t num_of_AB, size_t num_of_BC, size_t num_of_CD, size_t num_of_DA, std::vector<geometry_msgs::Point>& result_positions)
{
	AddStraightLine(A, B, num_of_AB, result_positions);
	AddStraightLine(B, C, num_of_BC, result_positions);
	AddStraightLine(C, D, num_of_CD, result_positions);
	AddStraightLine(D, A, num_of_DA, result_positions);
}


int fractorial(int n)
{
	int fractorial_of_n = 1;
	while (n) {
		fractorial_of_n *= n;
		n--;
	}

	return fractorial_of_n;
}


int comb(int n, int r)
{
	return fractorial(n) / (fractorial(n - r) * fractorial(r));
}



void Gait::CalcBSpline(const std::vector<double>& t, const std::vector<geometry_msgs::Point>& points, std::vector<geometry_msgs::Point>& result_points)
{
	for (size_t i = 0; i < t.size(); i++) {
		geometry_msgs::Point tmp_pos;
		tmp_pos.x = 0;
		tmp_pos.y = 0;
		tmp_pos.z = 0;
		for (size_t j = 0; j < points.size(); j++) {
			tmp_pos.x += comb(points.size() - 1, j) * pow((1-t[i]), points.size() - 1 - j) * pow(t[i], j) * points[j].x;
			tmp_pos.y += comb(points.size() - 1, j) * pow((1-t[i]), points.size() - 1 - j) * pow(t[i], j) * points[j].y;
			tmp_pos.z += comb(points.size() - 1, j) * pow((1-t[i]), points.size() - 1 - j) * pow(t[i], j) * points[j].z;
		}
		//std::cout << tmp_pos << std::endl;
		result_points.push_back(tmp_pos);
	}
}



void Gait::AddBSpline(const std::vector<geometry_msgs::Point>& positions, size_t num_of_spline, std::vector<geometry_msgs::Point>& result_positions, bool constant_speed)
{
	size_t i = 0;
	std::vector<double> ts;
	if (constant_speed == false) {
		i = 0;
		for (double t = 0; i < num_of_spline; t = (double)i*1.0/(double)num_of_spline) {
			ts.push_back(t);
			i++;
		}
		CalcBSpline(ts, positions, result_positions);
		return;
	} else {
		std::vector<geometry_msgs::Point> parametric_entries;
		std::vector<double> distances;
		i = 0;
		for (double t = 0; i <= 100; t = (double)i*1.0/(double)100) {
			ts.push_back(t);
			i++;
		}
		CalcBSpline(ts, positions, parametric_entries);

		distances.push_back(0.0);
		for (size_t j = 1; j <= 100; j++) {
			double dist = sqrt(pow(parametric_entries[j].x - parametric_entries[j - 1].x, 2) +
					pow(parametric_entries[j].y - parametric_entries[j - 1].y, 2) +
					pow(parametric_entries[j].z - parametric_entries[j - 1].z, 2));
			distances.push_back(distances[j - 1] + dist);
		}
		for (size_t j = 0; j <= 100; j++) {
			distances[j] /= distances[100];
		}
		distances[100] = 1.00;


		std::vector<double> u;
		i = 0;
		for (double t = 0; i < num_of_spline; t = (double)i * 1.0/(double)num_of_spline) {
			for (size_t j = 100; j >= 0; j--) {
				if (t >= distances[j]) {
					double ratio = (t - distances[j])/(distances[j+1] - distances[j]);
					double real_u = 1.0/100.0*(j+ratio);
					u.push_back(real_u);
					break;
				}
			}
			i++;
		}
		CalcBSpline(u, positions, result_positions);
		return;
	}
}



void Gait::CreateBSplineGait(const std::vector<geometry_msgs::Point>& positions, size_t num_of_spline, size_t num_of_straight, std::vector<geometry_msgs::Point>& result_positions)
{
	AddStraightLine(positions[positions.size() - 1], positions[0], num_of_straight, result_positions);
	AddBSpline(positions, num_of_spline, result_positions, false);
}

void Gait::AddPointToTrajectory(const std::vector<int32_t>& leg_number, const std::vector<geometry_msgs::Point>& points)
{
	quadruped_msgs::LegTrajectoryAddPoint srv;
	srv.request.leg_number = leg_number;
	srv.request.points = points;
	if (trajectory_add_point_client.call(srv) == false) {
		ROS_ERROR("Failed to add points to trajectory");
		//return false;
	} else {
		ROS_INFO("Succeeded in callin server");
		if (srv.response.error_codes == srv.response.SUCCEEDED) {
			ROS_INFO("Succeeded in AddPointToTrajectory");
		} else {
			ROS_ERROR("Failed to AddPointToTrajectory");
			//return false;
		}
	}
}


void Gait::AddPoints(void)
{
	std::vector<int32_t> leg_numbers;
	std::vector<geometry_msgs::Point> trajectory, control_points;
	control_points.resize(7);


	control_points[0].x = -180/1000.0;
	control_points[0].y = 10/1000.0;
	control_points[0].z = -80/1000.0;

	control_points[1].x = -180/1000.0;
	control_points[1].y = -30/1000.0;
	control_points[1].z = -50/1000.0;

	control_points[2].x = -180/1000.0;
	control_points[2].y = 30/1000.0;
	control_points[2].z = -20/1000.0;

	control_points[3].x = -180/1000.0;
	control_points[3].y = 60/1000.0;
	control_points[3].z = 10/1000.0;

	control_points[4].x = -180/1000.0;
	control_points[4].y = 90/1000.0;
	control_points[4].z = -20/1000.0;

	control_points[5].x = -180/1000.0;
	control_points[5].y = 150/1000.0;
	control_points[5].z = -50/1000.0;

	control_points[6].x = -180/1000.0;
	control_points[6].y = 110/1000.0;
	control_points[6].z = -80/1000.0;

	CreateBSplineGait(control_points, 10, 20, trajectory);

	for (size_t i = 0; i < trajectory.size(); i++) {
		leg_numbers.push_back(0);
	}

	AddPointToTrajectory(leg_numbers, trajectory);



	//trajectory_point_pub.publish(points);
	//for (size_t i = 0; i < points.points.size(); i++) {
	//	ROS_INFO("leg_number: %d %f,%f,%f", 0, points.points[i].x, points.points[i].y, points.points[i].z);
	//}

	CalcLegTrajectory();

}




bool Gait::Clear(void)
{
	quadruped_msgs::LegTrajectoryCommand srv;
	srv.request.command = srv.request.CLEAR;

	return CallLegTrajectory(srv);
	//quadruped_msgs::LegTrajectoryCommand command;
	//command.command = command.CLEAR;
	//command.exec_time = 2.0;
	//trajectory_command_pub.publish(command);
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
	gait.Clear();
	std::vector<geometry_msgs::Point> trajectory;
	trajectory.resize(2);

	trajectory[0].x = 0.0;
	trajectory[0].y = 0.0;
	trajectory[0].z = 0.0;

	trajectory[1].x = 000.0/1000.0;
	trajectory[1].y = 600.0/1000.0;
	trajectory[1].z = 0.0/1000.0;

	gait.GenerateCrawlGait(30.0/1000.0, 0, trajectory);
	//gait.MoveLegDefault();
	//gait.AddPoints();
	gait.RunLegTrajectory();

	//ros::spin();
	return 0;
}
