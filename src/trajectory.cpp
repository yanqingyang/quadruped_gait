#include "trajectory.hpp"

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

void Trajectory::AddPoints(const std::vector<geometry_msgs::Point>& points)
{
	std::copy(points.begin(), points.end(), std::back_inserter(trajectory));
}


void Trajectory::InternalDivision(double t, const geometry_msgs::Point &start_point, const geometry_msgs::Point &end_point, geometry_msgs::Point &result_point)
{
	result_point.x = (1.0 - t) * start_point.x + end_point.x * t;
	result_point.y = (1.0 - t) * start_point.y + end_point.y * t;
	result_point.z = (1.0 - t) * start_point.z + end_point.z * t;
}


void Trajectory::AddStraightLine(const geometry_msgs::Point &start_position, const geometry_msgs::Point &end_position, size_t num)
{
	for (size_t i = 0; i < num; i++) {
		geometry_msgs::Point result_position;
		InternalDivision((double)i/(double)num, start_position, end_position, result_position);
		trajectory.push_back(result_position);
	}
}



//void Trajectory::CalcBSpline(const std::vector<double>& t, const std::vector<geometry_msgs::Point>& points)
void Trajectory::CalcBSpline(const std::vector<double>& t, const std::vector<geometry_msgs::Point>& points, std::vector<geometry_msgs::Point>& result_points)
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
		//trajectory.push_back(tmp_pos);
		result_points.push_back(tmp_pos);
	}
}



void Trajectory::AddBSpline(const std::vector<geometry_msgs::Point>& positions, size_t num_of_spline, bool constant_speed)
{
	size_t i = 0;
	std::vector<double> ts;
	if (constant_speed == false) {
		i = 0;
		for (double t = 0; i < num_of_spline; t = (double)i*1.0/(double)num_of_spline) {
			ts.push_back(t);
			i++;
		}
		CalcBSpline(ts, positions, trajectory);
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
		CalcBSpline(u, positions, trajectory);
		return;
	}
}

