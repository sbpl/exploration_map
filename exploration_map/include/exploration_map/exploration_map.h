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
#include <algorithm>


/**
 *
 */
namespace exploration_map
{

/**
 *
 */
enum class exploration_type
{
	unknown, explored, occupied
};

class point
{
public:
	double x;
	double y;
	double z;
};

class cell
{
public:
	int X;
	int Y;
	int Z;
};

class cell_list
{
public:

	void push_back(cell c)
	{
		list.push_back(c);
	}

	std::vector<cell> list;
};

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

	std::vector< std::vector<T> > & operator[](int x)
	{
		return map_[x];
	}

	const std::vector< std::vector<T> > & operator[](int x) const
	{
		return map_[x];
	}

	T at(int x, int y, int z) const
	{
		return map_.at(x).at(y).at(z);
	}

	std::vector<std::vector< std::vector<T> > > map_;
	double resolution;
	int size_x;
	int size_y;
	int size_z;
};

/**
 *
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
	}

	double resolution;
	int size_x;
	int size_y;
	int size_z;
	point origin;

};

/**
 *
 */
class occupancy_map_config
{
public:

	occupancy_map_config()
	{
		update_increment_value = 0;
		update_decrement_value = 0;
		occ_threshold = 0;
	}

	double update_increment_value;
	double update_decrement_value;
	double occ_threshold;
};

/**
 *
 */
class config
{
public:
	map_config map_config_;
	occupancy_map_config occ_map_config_;
};

/**
 *
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
	exploration_map(config _config);

	/**
	 *
	 */
	~exploration_map();

	/**
	 *
	 * @param _config
	 * @return
	 */
	bool initialize(config _config);

	/**
	 *
	 * @param update
	 * @return
	 */
	bool update_map(const sensor_update::sensor_update & update);

	/**
	 *
	 * @return
	 */
	const generic_map<exploration_type> * get_exploration_map();

	const config * get_configuration() const;

	cell map_frame(const cell & c);

	bool is_in_bounds(const cell & c);



private:

	bool update_occupancy_map(const sensor_update::lidar_update & update, cell_list & updated_cells);

	bool update_observation_map(const sensor_update::camera_update & update, cell_list & updated_cells);

	bool update_exploration_map(const cell_list & updated_cells);



	config config_;
	generic_map<exploration_type> exp_map;
	generic_map<double> occupancy_map;
	generic_map<int> observation_map;
	cell origin_cell_;

};

}


#endif /* EXPLORATION_MAP_H_ */
