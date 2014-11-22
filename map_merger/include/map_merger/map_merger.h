/*
 * map_merger.h
 *
 *  Created on: Nov 14, 2014
 *      Author: bmacallister
 */

#include <string>
#include <iostream>
#include <algorithm>
#include <exploration_map/exploration_map.h>
#include <generic_transform/generic_transform.h>

#ifndef MAP_MERGER_H_
#define MAP_MERGER_H_

namespace exploration
{

template<typename value_type>
class point_valued : public point
{
public:
	point_valued() : point(), value()
	{
	}
	value_type value;
};
template<typename value_type>
class cell_valued: public cell
{
public:
	cell_valued() : cell(), value()
{
}
	value_type value;
};

class map_update
{
public:
	int map_id;
	std::vector<point_valued<exploration_type> > points;
};

class scan_match_config
{
public:
	int dx;
	int dy;
	int dz;
	int dyaw;
	double metric_res;
	double angular_res;
};

class map_merge_config
{
public:

	map_merge_config() : map_config_()
	{
		number_of_maps = 0;
	}

	map_config map_config_;
	scan_match_config scan_match_config_;
	int number_of_maps;
};


class map_merger
{

public:

	map_merger();

	~map_merger();

	map_merger(map_merge_config _config);

	void initialize(map_merge_config _config);

	int receive_map_update(const map_update & update);

	int get_map(int map_id, const generic_map<exploration_type>*& map );

	int get_master_map( const generic_map<exploration_type>*& map);

private:


	int update_map(const map_update& update, const pose& origin_frame, const pose& destination_frame,  generic_map <exploration_type> & map);

	void generate_initial_transforms();


	map_merge_config config_;
	std::vector< generic_map <exploration_type> > maps_;
	std::vector<int> map_counter_;
	std::vector<pose> map_origins_;
	generic_map <exploration_type> master_map_;
	bool origins_initialized_;
};

}

#endif /* MAP_MERGER_H_ */

/*
 *  frame: /map
 map_resolution: 0.05
 map_origin_x: -35.025
 map_origin_y: -12.025
 map_size_x: 1200
 map_size_y: 700

 robot_0_map_name: /Segbot/exploration_map_node/exploration_map
 robot_1_map_name: /Hexarotor/exploration_map/exploration_map

 map_publish_rate: 1.0
 */
