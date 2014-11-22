/*
 * generic_transform.h
 *
 *  Created on: Nov 17, 2014
 *      Author: bmacallister
 */

#ifndef GENERIC_TRANSFORM_H_
#define GENERIC_TRANSFORM_H_

#include <string>
#include <iostream>
#include <algorithm>

namespace generic_transform
{

template <typename orientation>
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

template <typename orientation>
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

template<typename position, typename orientation>
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

template<typename position, typename pose>
position transform_position_from_frame(position pos, pose p)
{
	//rotate
	pos = rotate_position(pos, p.ori);

	//translate
	pos.x += p.pos.x;
	pos.y += p.pos.y;
	pos.z += p.pos.z;

	return pos;
}

template<typename position, typename pose>
position transform_position_to_frame(position pos, pose p)
{
	//translate
	pos.x -= p.pos.x;
	pos.y -= p.pos.y;
	pos.z -= p.pos.z;

	//reverse orientation
	p.ori.x = -p.ori.x;
	p.ori.y = -p.ori.y;
	p.ori.z = -p.ori.z;

	//rotate
	pos = rotate_position(pos, p.ori);

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

template<typename orientation>
void convert_quaternion_to_orientation(double q[4], orientation & orient)
{
	orient.w = q[0];
	orient.x = q[1];
	orient.y = q[2];
	orient.z = q[3];
}

}




#endif /* GENERIC_TRANSFORM_H_ */
