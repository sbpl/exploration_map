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
 * \brief namespace for exploration map stuff
 */
namespace exploration
{

/**
 * \brief enum used to disambiguate between types of exploration values
 */
enum class exploration_type
{
	unknown /**value not known*/,
	explored /**value is free and has also been seen by camera*/,
	occupied /**value is an obstacles */,
	unoccupied /**value is a free space */
};

/**
 * \brief a construct for a 3d point in space
 */
class point
{
public:

	/**
	 * \brief default constructor
	 */
	point();

	/**
	 * \brief copy constructor
	 * @param _p
	 */
	point(const point & _p);

	/**
	 * \brief initialization constructor
	 * @param _x
	 * @param _y
	 * @param _z
	 */
	point(double _x, double _y, double _z);

	/**
	 * \brief returns formatted string representation of this construct
	 * @return a string
	 */
	std::string to_string() const;

	/**
	 * \brief stream operator using to_string method
	 * @param out
	 * @param p
	 * @return reference to output stream
	 */
	friend std::ostream& operator<<(std::ostream &out, const point & p);

	//member variables
	double x;
	double y;
	double z;

};

/**
 * \brief a construct for a 4d orientation (quaternion)
 */
class orientation
{
public:

	/**
	 * \brief default constructor
	 */
	orientation();

	/**
	 * \brief copy constructor
	 * @param _c
	 */
	orientation(const orientation & _c);

	/**
	 * \brief initialization constructor
	 * @param _x
	 * @param _y
	 * @param _z
	 * @param _w
	 */
	orientation(double _x, double _y, double _z, double _w);

	/**
	 * \brief returns string representation of orientation
	 * @return string
	 */
	std::string to_string() const;

	/**
	 * \brief stream operator that returns string representation of orientation
	 * @param out
	 * @param ori
	 * @return reference to output stream
	 */
	friend std::ostream& operator<<(std::ostream &out, const orientation & ori);

	//member variables
	double x;
	double y;
	double z;
	double w;

};

/**
 * \brief construct containing a position and orientation
 */
class pose
{
public:

	/**
	 * \brief default constructor
	 */
	pose();

	/**
	 * \brief initialization constructor
	 * @param _pos
	 * @param _ori
	 */
	pose(point _pos, orientation _ori);

	/**
	 * \brief returns string representation of pose
	 * @return string
	 */
	std::string to_string() const;

	/**
	 * stream operator that modifies output stream to include string representation of pose
	 * @param out
	 * @param p
	 * @return reference to output stream
	 */
	friend std::ostream& operator<<(std::ostream &out, const pose & p);

	//member variables
	point pos;
	orientation ori;
};

/**
 * \brief construct for a 3d discretized cell
 */
class cell
{
public:

	//member variables
	int X;
	int Y;
	int Z;
};

/**
 * \brief contains a list of cells
 */
class cell_list
{
public:


	/**
	 * \brief appends cell to list
	 * @param c
	 */
	void push_back(cell c);

	/**
	 * returns size of list
	 * @return list size
	 */
	size_t size();

	//member variables
	std::vector<cell> list;
};

/**
 * \brief construct that represents a discretized map of arbitrary type
 */
template<typename T>
class generic_map
{
public:

	/**
	 * \brief default constructor
	 */
	generic_map();

	/**
	 * \brief initialization constructor
	 * @param _resolution
	 * @param _size_x
	 * @param _size_y
	 * @param _size_z
	 */
	generic_map(double _resolution, int _size_x, int _size_y, int _size_z);

	/**
	 * \brief destructor
	 */
	~generic_map();

	/**
	 * \brief initializes map with specified params
	 * @param _resolution
	 * @param _size_x
	 * @param _size_y
	 * @param _size_z
	 */
	void initialize(double _resolution, int _size_x, int _size_y, int _size_z);

	/**
	 * \brief similar to method with same name with a specified default value for all cells in map
	 * @param _resolution
	 * @param _size_x
	 * @param _size_y
	 * @param _size_z
	 * @param _default_value
	 */
	void initialize(double _resolution, int _size_x, int _size_y, int _size_z, T _default_value);

	/**
	 * \brief indexer function for unwrapping internal vector of vectors
	 * @param x
	 * @return reference to row of map
	 */
	std::vector<std::vector<T> > & operator[](int x);

	/**
	 * \brief indexer function similar to above but with const
	 * @param x
	 * @return const reference to row of map
	 */
	const std::vector<std::vector<T> > & operator[](int x) const;

	/**
	 * \brief indexer function to return value of cell in map
	 * @param x
	 * @param y
	 * @param z
	 * @return cell value
	 */
	T at(int x, int y, int z) const;

	/**
	 * \brief bounds checking function
	 * @param c
	 * @return true if cell provided is within bounds
	 */
	template<typename cell>
	bool is_in_bounds(cell c);

	//member variables
	std::vector<std::vector<std::vector<T> > > map_;
	double resolution;
	int size_x;
	int size_y;
	int size_z;
};

/**
 * \brief contains configuration parameters for a map
 */
class map_config
{
public:

	/**
	 *\brief default constructor
	 */
	map_config()
	{
		resolution = 0;
		size_x = 0;
		size_y = 0;
		size_z = 0;
		ground_plane_height_threshold = 0;
	}


	double resolution; /** map resolution */
	int size_x; /** length of map in cells */
	int size_y; /** depth of map in cells */
	int size_z; /** height of map in cells */
	double ground_plane_height_threshold; /** specifies how high an obstacle has to be (metric wise) for it be considered and obstacle */
	pose origin; /**specifies the origin of the map */

};

/**
 * \brief contains configuration parameters for an occupancy grid
 */
class occupancy_map_config
{
public:

	/**
	 * \brief default constructor
	 */
	occupancy_map_config()
	{
		lidar_update_increment_value = 0;
		lidar_update_decrement_value = 0;
		occ_threshold = 0;
		unnoc_threshold = 0;
	}

	double lidar_update_increment_value; /** specifies how much a cell's value increases when scan votes that it is occupied */
	double lidar_update_decrement_value; /** specifies how much a cell's value decrease when scan votes that it is clear */
	double occ_threshold; /** specifies the min value for an occupied cell */
	double unnoc_threshold; /** specifies the max value for a cleared cell */
};

/**
 * \brief contains a set of configuration sets
 */
class exploration_map_config
{
public:
	map_config map_config_;
	occupancy_map_config occ_map_config_;
};

/**
 * \brief used to merge sensor data of different types to generate and update an exploration map
 */
class exploration_map
{
public:

	/**
	 * \brief default constructor
	 */
	exploration_map();

	/**
	 * \brief initialization constructor
	 * @param _config
	 */
	exploration_map(exploration_map_config _config);

	/**
	 * \brief destructor
	 */
	~exploration_map();

	/**
	 * \brief initializes the exploration map given a set of configuration parameters
	 * @param _config
	 * @return
	 */
	bool initialize(exploration_map_config _config);

	/**
	 * \brief updates the map given a sensor update
	 * @param update
	 * @return success of update
	 */
	bool update_map(const sensor_update::sensor_update & update, cell_list & updated_cells);

	/**
	 * \brief returns reference to the exploration map
	 * @return reference to map
	 */
	const generic_map<exploration_type> * get_exploration_map();

	/**
	 * \brief returns reference to the exploration map's configuration
	 * @return reference to current config
	 */
	const exploration_map_config * get_configuration() const;

	/**
	 * \brief returns map frame version of cell in world frame
	 * @param c
	 * @return map frame version of cell
	 */
	cell map_frame(const cell & c);

	/**
	 * \brief checks bounds of cell (assumed to be in map frame)
	 * @param c
	 * @return true if cell in map bounds
	 */
	bool is_in_bounds(const cell & c);

	/**
	 * \brief returns discrete representation of continuous value
	 * @param s the continuous value
	 * @param res the discretization resolution
	 * @return discrete version of continuous value
	 */
	static int discretize(double s, double res);

	/**
	 * \brief returns continuous representation of discrete value
	 * @param d the discrete value
	 * @param res the discretization resolution
	 * @return continuous value
	 */
	static double continuous(int d, double res);

private:

	/**
	 * \brief updates underlying occupancy grid
	 * @param update
	 * @param updated_cells
	 * @param increment_value
	 * @param decrement_value
	 * @return true if success
	 */
	bool update_occupancy_map(const sensor_update::sensor_update & update, cell_list & updated_cells, double increment_value, double decrement_value);

	/**
	 * \brief updates underlying observation map
	 * @param update
	 * @param updated_cells
	 * @return true if success
	 */
	bool update_observation_map(const sensor_update::sensor_update & update, cell_list & updated_cells);

	/**
	 * \brief updates underlying exploration map
	 * @param sensor_update_cells
	 * @param updated_cells
	 * @return true if success
	 */
	bool update_exploration_map(const cell_list& sensor_update_cells, cell_list & updated_cells);

	//member variables
	exploration_map_config config_;
	pose disc_origin_;
	generic_map<exploration_type> exp_map;
	generic_map<double> occupancy_map;
	generic_map<int> observation_map;

};

}

#endif /* EXPLORATION_MAP_H_ */
