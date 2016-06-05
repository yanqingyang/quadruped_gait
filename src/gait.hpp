#ifndef GAIT_HPP
#define GAIT_HPP

#include <ros/ros.h>

#include <vector>

#include <quadruped_msgs/GetLegIKSolver.h>
#include <quadruped_msgs/LegTrajectoryCommand.h>
#include <quadruped_msgs/LegTrajectoryAddPoint.h>
#include <quadruped_msgs/LegJointState.h>
#include <visualization_msgs/Marker.h>

//     A       D
//     *-------*
//    /         \
//   /           \
//  *-------------*
//  B             C

class Gait {
	public:
		bool init(void);
		void AddPoints(void);
		void AddPointToTrajectory(const std::vector<int32_t>& leg_number, const std::vector<geometry_msgs::Point>& points);
		bool Clear(void);
		void MoveLegDefault(void);
		bool RunLegTrajectory(void);
		void GenerateCrawlGait(double body_speed, double freq, const std::vector<geometry_msgs::Point>& body_traj);


	private:
		bool CallLegTrajectory(quadruped_msgs::LegTrajectoryCommand& srv);
		bool CalcLegTrajectory(void);

		void ConcatinateDividedBodyTrajWithCrawl(size_t leg_i, size_t total_period_num, size_t update_per_one_period_num, const std::vector<geometry_msgs::Point>& splited_body_traj, std::vector<std::vector<geometry_msgs::Point> >& result_array_of_leg_traj);
		void ConvertBodyTrajToLegTraj(size_t leg_i, std::vector<std::vector<geometry_msgs::Point> >& array_of_body_traj);

		void InternalDivision(double t, const geometry_msgs::Point &start_point, const geometry_msgs::Point &end_point, geometry_msgs::Point &result_point);
		void AddStraightLine(const geometry_msgs::Point &start_position, const geometry_msgs::Point &end_position, size_t num, std::vector<geometry_msgs::Point> &result_positions);
		//void CreateGait(const geometry_msgs::Point& A, const geometry_msgs::Point& B, const geometry_msgs::Point& C, const geometry_msgs::Point& D, std::vector<geometry_msgs::Point>& points, size_t num_of_partition);
		void AddStraightLines(const std::vector<geometry_msgs::Point>& positions, const std::vector<size_t>& num_of_points, std::vector<geometry_msgs::Point>& result_positions);

		void CreateGait(const geometry_msgs::Point& A, const geometry_msgs::Point& B, const geometry_msgs::Point& C, const geometry_msgs::Point& D, size_t num_of_AB, size_t num_of_BC, size_t num_of_CD, size_t num_of_DA, std::vector<geometry_msgs::Point>& result_positions);

		void CalcBSpline(const std::vector<double>& t, const std::vector<geometry_msgs::Point>& points, std::vector<geometry_msgs::Point>& result_points);
		void AddBSpline(const std::vector<geometry_msgs::Point>& positions, size_t num_of_spline, std::vector<geometry_msgs::Point>& result_positions, bool constant_speed);
		void CreateBSplineGait(const std::vector<geometry_msgs::Point>& positions, size_t num_of_spline, size_t num_of_straight, std::vector<geometry_msgs::Point>& result_positions);
		void AddBSplineIdlingLegTraj(const std::vector<std::vector<geometry_msgs::Point> >& array_of_supporing_leg_traj, size_t num_of_spline, std::vector<geometry_msgs::Point>& result_points);

		ros::NodeHandle node, private_node;
		//ros::Publisher trajectory_command_pub;
		ros::ServiceClient trajectory_command_client;
		ros::ServiceClient trajectory_add_point_client;
		ros::Publisher marker_pub;
		visualization_msgs::Marker arrow;

		std::vector<geometry_msgs::Point> middle_points;
};

#endif // GAIT_HPP
