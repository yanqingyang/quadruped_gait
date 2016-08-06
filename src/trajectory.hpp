#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/Point.h>

class Trajectory {
	public:
		std::vector<geometry_msgs::Point> trajectory;
		void AddPoints(const std::vector<geometry_msgs::Point>& points);
		void AddStraightLine(const geometry_msgs::Point &start_position, const geometry_msgs::Point &end_position, size_t num);
		void AddBSpline(const std::vector<geometry_msgs::Point>& positions, size_t num_of_spline, bool constant_speed);
	private:
		//void CalcBSpline(const std::vector<double>& t, const std::vector<geometry_msgs::Point>& points);
		void CalcBSpline(const std::vector<double>& t, const std::vector<geometry_msgs::Point>& points, std::vector<geometry_msgs::Point>& result_points);
		void InternalDivision(double t, const geometry_msgs::Point &start_point, const geometry_msgs::Point &end_point, geometry_msgs::Point &result_point);
};

#endif // TRAJECTORY_HPP
