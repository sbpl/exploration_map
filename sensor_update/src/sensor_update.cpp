/*
 * sensor_update.cpp
 *
 *  Created on: Sep 23, 2014
 *      Author: bmacallister
 */

#include <sensor_update/sensor_update.h>

namespace sensor_update
{

int discretize(double s, double res)
{
	double v = s / res;
	v >= 0 ? v = floor(v) : v = ceil(v - 1);
	int d = static_cast<int>(v);
	return d;
}

sensor_update::sensor_update()
{
}

sensor_update::sensor_update(pose pose, sensor_reading sensor_reading)
{
	pose_ = pose;
	reading_ = sensor_reading;
}

sensor_update::~sensor_update()
{
}

bool sensor_update::get_transformed_sensor_ends(std::vector<position>& trans_sensor_ends) const
{

	//sanity check
	if (reading_.rays.size() == 0)
	{
		return false;
	}

	//create position for each distance/angle pair given pose
	std::vector<position> sensor_end_pos;
	for (size_t i = 0; i < reading_.rays.size(); i++)
	{
		double yaw = reading_.rays[i].angle.yaw;
		double roll = reading_.rays[i].angle.roll;
		double distance = reading_.rays[i].distance;
		double base_quat[4];
		generic_transform::convert_eular_to_quaterion(roll, 0, yaw, base_quat);
		orientation base_orient(base_quat);
		position base_pos(distance, 0, 0);
		position pos = generic_transform::rotate_position(base_pos, base_orient);
		sensor_end_pos.push_back(pos);
	}

	//transform ends to pose taken with scan
	for (auto a : sensor_end_pos)
	{
		a = generic_transform::transform_position_from_frame(a, pose_);
		trans_sensor_ends.push_back(a);
	}

	return true;
}

bool sensor_update::get_discrete_ray_trace_cells(double resolution, std::vector<discrete_cell>& cells) const
{
	std::vector<position> sensor_ends;
	if (!get_transformed_sensor_ends(sensor_ends))
	{
		return false;
	}

	std::vector<discrete_cell> intersecting_cells;

	//for each ray
	for (auto & ray_end : sensor_ends)
	{

		//shift to local coordinates
		ray_end.x -= pose_.pos.x;
		ray_end.y -= pose_.pos.y;
		ray_end.z -= pose_.pos.z;

		//for each cell ray crosses over
		double max_distance = sqrt(ray_end.x * ray_end.x + ray_end.y * ray_end.y + ray_end.z * ray_end.z);
		double current_dist = 0;
		double step_size = resolution;
		double dx = ray_end.x / max_distance;
		double dy = ray_end.y / max_distance;
		double dz = ray_end.z / max_distance;

		while (true)
		{
			//Add new cell
			int cost = get_ray_cost(current_dist, max_distance);
			double x = dx * current_dist;
			double y = dy * current_dist;
			double z = dz * current_dist;

			//shift back to world coordinates
			x += pose_.pos.x;
			y += pose_.pos.y;
			z += pose_.pos.z;

			int X = discretize(x, resolution);
			int Y = discretize(y, resolution);
			int Z = discretize(z, resolution);
			discrete_cell d(X, Y, Z, cost);
			intersecting_cells.push_back(d);

			//leave when max distance reached
			if (current_dist >= max_distance)
			{
				break;
			}

			//increment distance
			current_dist += step_size;
			if (current_dist > max_distance)
			{
				current_dist = max_distance;
			}
		}
		cells = intersecting_cells;
	}

	return true;
}

int lidar_update::get_ray_cost(double distance, double end_distance) const
{
	if (distance < end_distance)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

lidar_update::lidar_update() :
		sensor_update()
{

}

lidar_update::lidar_update(pose pose, sensor_reading reading) :
		sensor_update(pose, reading)
{

}

sensor_update_type sensor_update::get_type() const
{
	return sensor_update_type::generic;
}

sensor_update_type lidar_update::get_type() const
{
	return sensor_update_type::lidar;
}

camera_update::camera_update()
{
}

camera_update::camera_update(pose pose, sensor_reading reading) :
		sensor_update(pose, reading)
{
}

sensor_update_type camera_update::get_type() const
{
	return sensor_update_type::camera;
}

int camera_update::get_ray_cost(double distance, double end_distance) const
{
	return 1;
}

pose sensor_update::get_pose() const
{
	return pose_;
}

bool camera_update::get_transformed_sensor_ends(std::vector<position>& trans_sensor_ends) const
{

	//sanity check
	if (reading_.rays.size() == 0)
	{
		return false;
	}

	//create position for each distance/angle pair given pose
	std::vector<position> sensor_end_pos;
	for (size_t i = 0; i < reading_.rays.size(); i++)
	{
		double yaw = reading_.rays[i].angle.yaw;
		double roll = reading_.rays[i].angle.roll;
		double distance = reading_.rays[i].distance;
		double base_quat[4];

		//rotate on yaw
		generic_transform::convert_eular_to_quaterion(0, 0, yaw, base_quat);
		orientation base_orient1(base_quat);
		position base_pos(distance, 0, 0);
		position pos = generic_transform::rotate_position(base_pos, base_orient1);

		//rotate on roll
		generic_transform::convert_eular_to_quaterion(roll, 0, 0, base_quat);
		orientation base_orient2(base_quat);
		pos = generic_transform::rotate_position(pos, base_orient2);

		//add
		sensor_end_pos.push_back(pos);
	}

	//transform ends to pose taken with scan
	for (auto a : sensor_end_pos)
	{
		a = generic_transform::transform_position_from_frame(a, pose_);
		trans_sensor_ends.push_back(a);
	}

	return true;
}

robot_volume_update::robot_volume_update() :
		sensor_update()
{
	height_ = 0;
	radius_ = 0;
}

robot_volume_update::robot_volume_update(pose pose, double height, double radius) :
		sensor_update(pose, sensor_reading())
{
	height_ = height;
	radius_ = radius;
	reading_ = generate_reading(radius_);
}

bool robot_volume_update::get_transformed_sensor_ends(std::vector<position>& trans_sensor_ends) const
{

	//sanity check
	if (reading_.rays.size() == 0)
	{
		return false;
	}

	double min_height = -height_ / 2;
	double max_height = height_ / 2;
	double height_res = 0.05;
	double current_height = min_height;

	//create position for each distance/angle/height triplet given pose
	std::vector<position> sensor_end_pos;
	while (true)
	{
		for (size_t i = 0; i < reading_.rays.size(); i++)
		{
			double yaw = reading_.rays[i].angle.yaw;
			double roll = reading_.rays[i].angle.roll;
			double distance = reading_.rays[i].distance;
			double base_quat[4];
			generic_transform::convert_eular_to_quaterion(roll, 0, yaw, base_quat);
			orientation base_orient(base_quat);

			//if height at bottom or top, perform for distances up to max
			double start_distance = distance;
			double end_distance = distance;
			double dist_res = 0.05;
			if (current_height == max_height || current_height == min_height)
			{
				start_distance = dist_res;
			}
			double current_distance = start_distance;
			while (true)
			{
				position base_pos(current_distance, 0, current_height);
				position pos = generic_transform::rotate_position(base_pos, base_orient);
				sensor_end_pos.push_back(pos);

				if (current_distance == end_distance)
				{
					break;
				}

				current_distance += dist_res;

				if (current_distance > end_distance)
				{
					current_distance = end_distance;
				}
			}
		}

		if (current_height == max_height)
		{
			break;
		}

		current_height += height_res;
		if (current_height - max_height > 0)
		{
			current_height = max_height;
		}

	}

	//transform ends to pose taken with scan
	for (auto a : sensor_end_pos)
	{
		a = generic_transform::transform_position_from_frame(a, pose_);
		trans_sensor_ends.push_back(a);
	}

	return true;
}

sensor_update_type robot_volume_update::get_type() const
{
	return sensor_update_type::robot_volume;
}

sensor_reading robot_volume_update::generate_reading(double radius) const
{
	sensor_reading reading;
	double pi = 3.14159;
	double res = 5.0 * (pi / 180);
	size_t count = static_cast<size_t>(2 * pi / res);
	reading.rays.reserve(count);
	double ang = 0;
	while (true)
	{
		sensor_ray ray;
		ray.angle.yaw = ang;
		ray.angle.roll = 0;
		ray.distance = radius;
		reading.rays.push_back(ray);

		if (ang == 2 * pi)
		{
			break;
		}

		ang += res;

		if (ang > 2 * pi)
		{
			ang = 2 * pi;
		}
	}
	return reading;
}

int robot_volume_update::get_ray_cost(double distance, double end_distance) const
{
	return 0;
}

}
