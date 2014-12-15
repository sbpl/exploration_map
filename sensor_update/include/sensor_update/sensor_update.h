/*
 * sensor_update.h
 *
 *  Created on: Sep 23, 2014
 *      Author: bmacallister
 */

#ifndef SENSOR_UPDATE_H_
#define SENSOR_UPDATE_H_

#include <tuple>
#include <string>
#include <iostream>
#include <generic_transform/generic_transform.h>
#include <type_traits>

/**
 *
 */
namespace sensor_update
{

/**
 *
 */
class position
{
public:
	position()
{
		x = 0;
		y = 0;
		z = 0;
}

	position(double _x, double _y, double _z)
	{
		x = _x;
		y = _y;
		z = _z;
	}
	double x;
	double y;
	double z;
};

/**
 *
 */
class orientation
{
public:

	orientation()
{
		w = 1;
		x = 0;
		y = 0;
		z = 0;
}

	orientation(double quat[4])
	{
		w = quat[0];
		x = quat[1];
		y = quat[2];
		z = quat[3];
	}

	double x;
	double y;
	double z;
	double w;
};

/**
 *
 */
class discrete_cell
{
public:

	discrete_cell()
{
		X = 0;
		Y = 0;
		Z = 0;
		value = 0;
}
	discrete_cell(int _x, int _y, int _z, int _v)
	{
		X = _x;
		Y = _y;
		Z = _z;
		value = _v;
	}

	int X;
	int Y;
	int Z;
	int value;
};

/**
 *
 */
class pose
{
public:
	position pos;
	orientation ori;
};

class sensor_angle
{
public:
	double yaw;
	double roll;
};

/**
 *
 */
class sensor_ray
{
public:
	sensor_angle angle;
	double distance;
};

/**
 *
 */
class sensor_reading
{
public:
	std::vector<sensor_ray> rays;

};

enum class sensor_update_type
{
	generic, lidar, camera, robot_volume
};

/**
 *
 */
class sensor_update
{
public:

	/**
	 *
	 */
	sensor_update();

	/**
	 *
	 * @param
	 * @param
	 */
	sensor_update(pose pose, sensor_reading sensor_reading);

	/**
	 *
	 */
	virtual ~sensor_update();

	/**
	 *
	 * @return
	 */
	virtual sensor_update_type get_type() const;

	/**
	 *
	 * @param trans_sensor_ends
	 */
	virtual bool get_transformed_sensor_ends(std::vector<position> & trans_sensor_ends) const;

	/**
	 *
	 * @param resolution
	 * @param cells
	 */
	bool get_discrete_ray_trace_cells(double resolution, std::vector<discrete_cell> & cells) const;

	pose get_pose() const;

private:

	/**
	 *
	 * @param distance
	 * @param end_distance
	 * @return
	 */
	virtual int get_ray_cost(double distance, double end_distance) const = 0;

protected:

	pose pose_;
	sensor_reading reading_;

};

/**
 *
 */
class lidar_update: public sensor_update
{
public:
	lidar_update();

	lidar_update(pose pose, sensor_reading reading) ;

	sensor_update_type get_type() const;

private:

	/**
	 *
	 * @param distance
	 * @param end_distance
	 * @return
	 */
	int get_ray_cost(double distance, double end_distance) const;
};

/**
 *
 */
class camera_update: public sensor_update
{
public:
	camera_update();

	camera_update(pose pose, sensor_reading reading);

	bool get_transformed_sensor_ends(std::vector<position> & trans_sensor_ends) const;


	sensor_update_type get_type() const;

private:

	/**
	 *
	 * @param distance
	 * @param end_distance
	 * @return
	 */
	int get_ray_cost(double distance, double end_distance) const;

};

/**
 *
 */
class robot_volume_update: public sensor_update
{
	public:

	robot_volume_update();

	robot_volume_update(pose pose, double height, double radius);

	bool get_transformed_sensor_ends(std::vector<position> & trans_sensor_ends) const;

	sensor_update_type get_type() const;

	private:

	double radius_;
	double height_;

	sensor_reading generate_reading(double radius) const;

	/**
	 *
	 * @param distance
	 * @param end_distance
	 * @return
	 */
	int get_ray_cost(double distance, double end_distance) const;

};

}

#endif /* SENSOR_UPDATE_H_ */
