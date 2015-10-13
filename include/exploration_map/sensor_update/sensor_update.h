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
 * \brief namespace for sensor updates and related structures
 */
namespace sensor_update
{

/**
 * \brief standard 3d position
 */
class position
{
public:

	/**
	 * default constructor
	 */
	position()
	{
		x = 0;
		y = 0;
		z = 0;
	}

	/**
	 * initialization constructor
	 * @param _x
	 * @param _y
	 * @param _z
	 */
	position(double _x, double _y, double _z)
	{
		x = _x;
		y = _y;
		z = _z;
	}

	//member variables
	double x;
	double y;
	double z;
};

/**
 * \brief contains 4d orientation (quaternion)
 */
class orientation
{
public:

	/**
	 * default constructor
	 */
	orientation()
	{
		w = 1;
		x = 0;
		y = 0;
		z = 0;
	}

	/**
	 * initialization constructor
	 * @param quat a 4d array containing quaternion components in order of w,x,y,z
	 */
	orientation(double quat[4])
	{
		w = quat[0];
		x = quat[1];
		y = quat[2];
		z = quat[3];
	}

	//member variables
	double x;
	double y;
	double z;
	double w;
};

/**
 * \brief holds 3d discretized location and value
 */
class discrete_cell
{
public:

	/**
	 * default constructor
	 */
	discrete_cell()
	{
		X = 0;
		Y = 0;
		Z = 0;
		value = 0;
	}

	/**
	 * initialization constructor
	 * @param _x
	 * @param _y
	 * @param _z
	 * @param _v
	 */
	discrete_cell(int _x, int _y, int _z, int _v)
	{
		X = _x;
		Y = _y;
		Z = _z;
		value = _v;
	}

	//member variables
	int X;
	int Y;
	int Z;
	int value;
};

/**
 * \brief contains a position and orientation as defined in same namespace
 */
class pose
{
public:

	//members
	position pos;
	orientation ori;
};

/**
 * \brief contains angle of sensor in terms of eular yaw and roll angles
 */
class sensor_angle
{
public:
	double yaw;
	double roll;
};

/**
 * \brief contains a sensor angle and distance
 */
class sensor_ray
{
public:
	sensor_angle angle;
	double distance;
};

/**
 * \brief contains a list of sensor_rays
 */
class sensor_reading
{
public:
	std::vector<sensor_ray> rays;

};

/**
 * \brief enum used to disambiguation between types of sensor updates
 */
enum class sensor_update_type
{
	generic /** default */,
	lidar /** used for lidar scans */,
	camera /** used for camera scans */,
	robot_volume /** used for projection of robot footprint */
};

/**
 * \brief couples a sensor reading with pose to return transformed ray ends and ray traces
 */
class sensor_update
{
public:

	/**
	 * \brief default constructor
	 */
	sensor_update();


	/**
	 * \brief initialization constructor
	 * @param pose
	 * @param sensor_reading
	 */
	sensor_update(pose pose, sensor_reading sensor_reading);

	/**
	 * \brief destructor
	 */
	virtual ~sensor_update();

	/**
	 * \brief returns the type of sensor update
	 * subclasses overload this
	 * @return sensor type
	 */
	virtual sensor_update_type get_type() const;

	/**
	 * \brief returns the continuous positions of the ray ends from this sensor update
	 * @param trans_sensor_ends output parameter for storing ray ends
	 * @return is true if procedure succeeded
	 */
	virtual bool get_transformed_sensor_ends(std::vector<position> & trans_sensor_ends) const;

	/**
	 * \brief returns ray trace of sensor for the given resolution
	 * @param resolution desired resolution
	 * @param cells output parameter holding the cells and their costs
	 * @return true if function succeeded
	 */
	bool get_discrete_ray_trace_cells(double resolution, std::vector<discrete_cell> & cells) const;

	/**
	 * \brief returns the pose for this update
	 */
	pose get_pose() const;

private:

	/**
	 * \brief returns the cost of a point along the ray compared by comparison to the ray's end.
	 * @param distance
	 * @param end_distance
	 * @return cost
	 */
	virtual int get_ray_cost(double distance, double end_distance) const = 0;

protected:

	//member variables
	pose pose_;
	sensor_reading reading_;

};

/**
 * \brief sensor update for lidar scans
 */
class lidar_update: public sensor_update
{
public:

	/**
	 * default constructor
	 */
	lidar_update();

	/**
	 * initialization constructor
	 * @param pose
	 * @param reading
	 */
	lidar_update(pose pose, sensor_reading reading);

	/**
	 * \brief returns lidar type
	 * @return lidar type
	 */
	sensor_update_type get_type() const;

private:

	/**
	 * \brief returns ray cost of point for lidar scan
	 * @param distance
	 * @param end_distance
	 * @return cost
	 */
	int get_ray_cost(double distance, double end_distance) const;
};

/**
 * \brief sensor_update for camera scan
 */
class camera_update: public sensor_update
{
public:

	/**
	 * \brief default constructor
	 */
	camera_update();

	/**
	 * \brief Initialization constructor
	 * @param pose
	 * @param reading
	 */
	camera_update(pose pose, sensor_reading reading);

	/**
	 * \brief Specialization of parent classe's get transformed ends function for camera
	 * @param trans_sensor_ends output parameter containing ends
	 * @return true if succeeded
	 */
	bool get_transformed_sensor_ends(std::vector<position> & trans_sensor_ends) const;

	/**
	 * \brief returns camera type
	 * @return camera type
	 */
	sensor_update_type get_type() const;

private:

	/**
	 * \brief returns cost of point for camera
	 * @param distance
	 * @param end_distance
	 * @return ray cost
	 */
	int get_ray_cost(double distance, double end_distance) const;

};

/**
 * \brief sensor update for robot footprint
 * in shape of cylinder
 */
class robot_volume_update: public sensor_update
{
public:

	/**
	 * \brief default constructor
	 */
	robot_volume_update();

	/**
	 * \brief initialization constructor
	 * @param pose
	 * @param height of robot
	 * @param radius of robot
	 */
	robot_volume_update(pose pose, double height, double radius);


	/**
	 * \brief returns transformed sensor ends for robot footprint
	 * @param trans_sensor_ends output containing ends of footprint
	 * @return success
	 */
	bool get_transformed_sensor_ends(std::vector<position> & trans_sensor_ends) const;

	/**
	 * \brief robot volume type
	 * @return robot volue type
	 */
	sensor_update_type get_type() const;

private:

	//member variables
	double radius_;
	double height_;

	/**
	 * \brief generates a scan slice for the footprint
	 * @param radius
	 * @return the scan slice
	 */
	sensor_reading generate_reading(double radius) const;

	/**
	 * \brief returns cost point in ray for robot footprint
	 * @param distance
	 * @param end_distance
	 * @return cost
	 */
	int get_ray_cost(double distance, double end_distance) const;

};

}

#endif /* SENSOR_UPDATE_H_ */
