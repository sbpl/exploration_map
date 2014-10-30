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

orientation normalize(orientation ori)
{
	double mag = sqrt(ori.x * ori.x + ori.y * ori.y + ori.z * ori.z + ori.w * ori.w);
	ori.x = ori.x / mag;
	ori.y = ori.y / mag;
	ori.z = ori.z / mag;
	ori.w = ori.w / mag;
	return ori;
}

void normalize(double q[4])
{
	double q_mag = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] /= q_mag;
	q[1] /= q_mag;
	q[2] /= q_mag;
	q[3] /= q_mag;
	q_mag = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

void quaternion_multiply(const double r[4], const double q[4], double f[4])
{
	f[0] = (r[0] * q[0]) - (r[1] * q[1]) - (r[2] * q[2]) - (r[3] * q[3]);
	f[1] = (r[0] * q[1]) + (r[1] * q[0]) + (r[2] * q[3]) - (r[3] * q[2]);
	f[2] = (r[0] * q[2]) - (r[1] * q[3]) + (r[2] * q[0]) + (r[3] * q[1]);
	f[3] = (r[0] * q[3]) + (r[1] * q[2]) - (r[2] * q[1]) + (r[3] * q[0]);
}

orientation rotate_orientation(orientation one, orientation two)
{
	double r[4] =
	{ one.w, one.x, one.y, one.z };
	double q[4] =
	{ one.w, one.x, one.y, one.z };
	double f[4] =
	{ 0, 0, 0, 0 };
	quaternion_multiply(r, q, f);
	orientation three(f);
	return three;
}

void quaternion_conjugate(const double r[4], double q[4])
{
	q[0] = r[0];
	q[1] = -r[1];
	q[2] = -r[2];
	q[3] = -r[3];
}

void quaternion_inverse(const double q[4], double q_inv[4])
{
	double q_con[4];
	quaternion_conjugate(q, q_con);
	double q_mag = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q_inv[0] = q_con[0] / q_mag;
	q_inv[1] = q_con[1] / q_mag;
	q_inv[2] = q_con[2] / q_mag;
	q_inv[3] = q_con[3] / q_mag;
}

position rotate_position(position pos, orientation ori)
{
	double p[4] =
	{ 0, pos.x, pos.y, pos.z };
	double q[4] =
	{ ori.w, ori.x, ori.y, ori.z };
	double qi[4];
	quaternion_inverse(q, qi);
	double p1[4];
	double p2[4];

	quaternion_multiply(q, p, p1);
	quaternion_multiply(p1, qi, p2);

	pos.x = p2[1];
	pos.y = p2[2];
	pos.z = p2[3];
	return pos;
}

position transform_position(position pos, pose p)
{
	//rotate
	pos = rotate_position(pos, p.ori);

	//translate
	pos.x += p.pos.x;
	pos.y += p.pos.y;
	pos.z += p.pos.z;

	return pos;
}

void convert_eular_to_quaterion(double roll, double pitch, double yaw, double q[4])
{
	double eps = 0.00000000001;
	double q_den = sqrt(cos(pitch) * cos(roll) + cos(pitch) * cos(yaw) + cos(roll) * cos(yaw) + sin(pitch) * sin(roll) * sin(yaw) + 1) + eps;
	q[0] = q_den / 2;
	q[1] = (cos(pitch) * sin(roll) + cos(yaw) * sin(roll) - cos(roll) * sin(pitch) * sin(yaw)) / (2 * q_den);
	q[2] = (sin(pitch) + sin(roll) * sin(yaw) + cos(roll) * cos(yaw) * sin(pitch)) / (2 * q_den);
	q[3] = (cos(pitch) * sin(yaw) + cos(roll) * sin(yaw) - cos(yaw) * sin(pitch) * sin(roll)) / (2 * q_den);
	normalize(q);
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
		convert_eular_to_quaterion(roll, 0, yaw, base_quat);
		orientation base_orient(base_quat);
		position base_pos(distance, 0, 0);
		position pos = rotate_position(base_pos, base_orient);
		sensor_end_pos.push_back(pos);
	}

	//transform ends to pose taken with scan
	for (auto a : sensor_end_pos)
	{
		a = transform_position(a, pose_);
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

}

bool sensor_update::camera_update::get_transformed_sensor_ends(std::vector<position>& trans_sensor_ends) const
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
		convert_eular_to_quaterion(0, 0, yaw, base_quat);
		orientation base_orient1(base_quat);
		position base_pos(distance, 0, 0);
		position pos = rotate_position(base_pos, base_orient1);

		//rotate on roll
		convert_eular_to_quaterion(roll, 0, 0, base_quat);
		orientation base_orient2(base_quat);
		pos = rotate_position(pos, base_orient2);

		//add
		sensor_end_pos.push_back(pos);
	}

	//transform ends to pose taken with scan
	for (auto a : sensor_end_pos)
	{
		a = transform_position(a, pose_);
		trans_sensor_ends.push_back(a);
	}

	return true;
}
