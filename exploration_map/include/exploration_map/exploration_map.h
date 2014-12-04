/*
 * exploration_map.h
 *
 *  Created on: Oct 1, 2014
 *      Author: bmacallister
 */

#ifndef EXPLORATION_MAP_H_
#define EXPLORATION_MAP_H_

#include <vector>
#include <string>
#include <iostream>
#include <algorithm>
#include <sensor_update/sensor_update.h>
#include <generic_transform/generic_transform.h>
#include <algorithm>
#include <sstream>
#include <iomanip>

/**
 *
 */
namespace exploration
{

/**
 *
 */
enum class exploration_type
{
	unknown, explored, occupied, unoccupied
};

/**
 * todo
 */
class point
{
public:

	/**
	 *
	 */
	point();

	/**
	 *
	 * @param _p
	 */
	point(const point & _p);

	/**
	 *
	 * @param _x
	 * @param _y
	 * @param _z
	 */
	point(double _x, double _y, double _z);

	/**
	 *
	 * @return
	 */
	std::string to_string() const;

	friend std::ostream& operator<<(std::ostream &out, const point & p);

	double x;
	double y;
	double z;

};

/**
 * todo
 */
class orientation
{
public:

	/**
	 *
	 */
	orientation();

	/**
	 *
	 * @param _c
	 */
	orientation(const orientation & _c);

	/**
	 *
	 * @param _x
	 * @param _y
	 * @param _z
	 * @param _w
	 */
	orientation(double _x, double _y, double _z, double _w);

	/**
	 *
	 * @return
	 */
	std::string to_string() const;

	friend std::ostream& operator<<(std::ostream &out, const orientation & ori);

	double x;
	double y;
	double z;
	double w;

};

/**
 * todo
 */
class pose
{
public:

	/**
	 *
	 */
	pose();

	/**
	 *
	 * @param _pos
	 * @param _ori
	 */
	pose(point _pos, orientation _ori);

	/**
	 *
	 * @return
	 */
	std::string to_string() const;

	friend std::ostream& operator<<(std::ostream &out, const pose & p);

	point pos;
	orientation ori;
};

/**
 * todo
 */
class cell
{
public:
	int X;
	int Y;
	int Z;
};

/**
 * todo
 */
class cell_list
{
public:

	/**
	 *
	 */
	void push_back(cell c);

	size_t size();

	std::vector<cell> list;
};

/**
 * todo
 */
template<typename T>
class generic_map
{
public:

	/**
	 *
	 */
	generic_map();

	/**
	 *
	 * @param _resolution
	 * @param _size_x
	 * @param _size_y
	 * @param _size_z
	 */
	generic_map(double _resolution, int _size_x, int _size_y, int _size_z);

	/**
	 *
	 */
	~generic_map();

	void initialize(double _resolution, int _size_x, int _size_y, int _size_z);

	void initialize(double _resolution, int _size_x, int _size_y, int _size_z, T _default_value);

	std::vector<std::vector<T> > & operator[](int x);

	const std::vector<std::vector<T> > & operator[](int x) const;

	T at(int x, int y, int z) const;

	template<typename cell>
	bool is_in_bounds(cell c);

	std::vector<std::vector<std::vector<T> > > map_;
	double resolution;
	int size_x;
	int size_y;
	int size_z;
};

/**
 * todo
 */
class map_config
{
public:

	map_config()
	{
		resolution = 0;
		size_x = 0;
		size_y = 0;
		size_z = 0;
		base_height_min = 0;
		base_height_max = 0;
	}

	double resolution;
	int size_x;
	int size_y;
	int size_z;
	double base_height_min;
	double base_height_max;
	pose origin;

};

/**
 * todo
 */
class occupancy_map_config
{
public:

	occupancy_map_config()
	{
		update_increment_value = 0;
		update_decrement_value = 0;
		occ_threshold = 0;
		unnoc_threshold = 0;
	}

	double update_increment_value;
	double update_decrement_value;
	double occ_threshold;
	double unnoc_threshold;
};

/**
 * todo
 */
class exploration_map_config
{
public:
	map_config map_config_;
	occupancy_map_config occ_map_config_;
};

/**
 * todo
 */
class exploration_map
{
public:

	/**
	 *
	 */
	exploration_map();

	/**
	 *
	 * @param _config
	 */
	exploration_map(exploration_map_config _config);

	/**
	 *
	 */
	~exploration_map();

	/**
	 *
	 * @param _config
	 * @return
	 */
	bool initialize(exploration_map_config _config);

	/**
	 *
	 * @param update
	 * @return
	 */
	bool update_map(const sensor_update::sensor_update & update, cell_list & updated_cells);

	/**
	 *
	 * @return
	 */
	const generic_map<exploration_type> * get_exploration_map();

	const exploration_map_config * get_configuration() const;

	cell map_frame(const cell & c);

	bool is_in_bounds(const cell & c);

	static int discretize(double s, double res);

	static double continuous(int d, double res);

private:

	bool update_occupancy_map(const sensor_update::lidar_update & update, cell_list & updated_cells);

	bool update_observation_map(const sensor_update::camera_update & update, cell_list & updated_cells);

	bool update_exploration_map(const cell_list& sensor_update_cells, cell_list & updated_cells);

	exploration_map_config config_;
	pose disc_origin_;
	generic_map<exploration_type> exp_map;
	generic_map<double> occupancy_map;
	generic_map<int> observation_map;

};

}

#endif /* EXPLORATION_MAP_H_ */
