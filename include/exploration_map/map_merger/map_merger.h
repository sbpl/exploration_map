/*
 * map_merger.h
 *
 *  Created on: Nov 14, 2014
 *      Author: bmacallister
 */

#include <algorithm>
#include <iostream>
#include <string>

#include <exploration_map/exploration_map/exploration_map.h>
#include <exploration_map/exploration_map/exploration_map.hpp>
#include <exploration_map/generic_transform/generic_transform.h>

#ifndef MAP_MERGER_H_
#define MAP_MERGER_H_

namespace exploration
{

/**
 * \brief a construct for a 3d point with an arbitrary value
 */
template<typename value_type>
class point_valued: public point
{
public:

	/**
	 * \brief default constructor
	 */
	point_valued() :
			point(), value()
	{
	}

	value_type value; /**< the value */
};

/**
 * \brief a construct for a 3d cell with an arbitrary value
 */
template<typename value_type>
class cell_valued: public cell
{
public:

	/**
	 * \brief default constructor
	 */
	cell_valued() :
			cell(), value()
	{
	}

	//member variables
	value_type value; /**< the value */
};

/**
 * \brief construct for an update of a map
 */
class map_update
{
public:
	int map_id; /**< the inner map id for which this update is for */
	std::vector<point_valued<exploration_type> > points; /**< the actual update itself */
};

/**
 * \brief holds configuration information for scan matching
 */
class scan_match_config
{
public:
	int dx; /**< max change in x */
	int dy; /**< max change in y */
	int dz; /**< max change in z */
	int dyaw; /**< max change in yaw */
	double metric_res; /**< spatial resolution */
	double angular_res; /**< angular resolution */
};

/**
 * \brief holds configuration information for merging merging
 */
class map_merge_config
{
public:

	/**
	 * \brief default constructor
	 */
	map_merge_config() :
			map_config_()
	{
		number_of_maps = 0;
	}


	/** @name config member variables
	 *
	 */
	/// @{
	map_config map_config_; /**< configuration for map */
	scan_match_config scan_match_config_; /**< configuration for scan matching */
	/// @}

	/** @name other member variables
	 *
	 */
	/// @{
	int number_of_maps; /**< number of maps the map merger will think about */
	/// @}
};

/**
 * \brief mergers some arbitrary number of maps together into one.
 * Updates for individual map sources will likewise update the merged map.
 */
class map_merger
{

public:

	/**
	 * \brief default constructor
	 */
	map_merger();

	/**
	 * \brief destructor
	 */
	~map_merger();

	/**
	 * \brief initialization constructor
	 * @param _config
	 */
	map_merger(map_merge_config _config);

	/**
	 * \brief initializes map merger
	 * @param _config
	 */
	void initialize(map_merge_config _config);

	/**
	 * \brief takes in a map update and updates map mergers map along with its copy of the map the update was for
	 * @param update
	 * @param updated_cells the cells that were updated in the merged map
	 * @return success of procedure
	 */
	int receive_map_update(const map_update & update, cell_list & updated_cells);

	/**
	 * \brief returns a reference to the inner map in question
	 * @param map_id the index of the map
	 * @param map the reference to this map
	 * @return success of call
	 */
	int get_map(int map_id, const generic_map<exploration_type>*& map);

	/**
	 * \brief returns a reference to the origin for the map in question
	 * used for transforming points with respect to this map to merged map's frame
	 * @param map_id index of the map
	 * @param origin reference to this map's origin
	 * @return success of call
	 */
	int get_origin(int map_id, const pose *& origin);

	/**
	 *
	 * @param map
	 * @return
	 */
	int get_master_map(const generic_map<exploration_type>*& map);

	/**
	 * \brief returns to when the origins for all maps have been set.
	 * This happens during the scan matching process.
	 * @return when origins have been set.
	 */
	bool origins_are_initialized();

private:

	/**
	 * \brief updates the specified map with the specified map update
	 * @param update
	 * @param destination_frame the origin map_merger's frame with respect to the given map
	 * @param map
	 * @param upated_cells
	 * @return success of operation
	 */
	int update_map(const map_update& update, const pose& destination_frame, generic_map<exploration_type> & map, cell_list * upated_cells);

	/**
	 * \brief performs scan matching to generate origins for all internal maps
	 */
	void generate_initial_transforms();

	/** @name member variables
	 *
	 */
	/// @{
	map_merge_config config_;
	std::vector<generic_map<exploration_type> > maps_;
	std::vector<int> map_counter_;
	std::vector<pose> map_origins_;
	generic_map<exploration_type> master_map_;
	bool origins_initialized_;
	/// @}
};

}

#endif /* MAP_MERGER_H_ */
