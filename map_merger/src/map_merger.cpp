/*
 * map_merger.cpp
 *
 *  Created on: Nov 14, 2014
 *      Author: bmacallister
 */

#include <map_merger/map_merger.h>

exploration::map_merger::map_merger()
{
	origins_initialized_ = false;
}

exploration::map_merger::~map_merger()
{
}

exploration::map_merger::map_merger(map_merge_config _config)
{
	origins_initialized_ = false;
	initialize(_config);
}

void exploration::map_merger::initialize(map_merge_config _config)
{
	config_ = _config;
	int map_num = config_.number_of_maps;
	double resolution = config_.map_config_.resolution;
	int size_x = config_.map_config_.size_x;
	int size_y = config_.map_config_.size_y;
	int size_z = config_.map_config_.size_z;

	printf("scan_match dx %d\n", config_.scan_match_config_.dx);

	//initialize maps and origins
	for (int i = 0; i < map_num; i++)
	{
		generic_map<exploration_type> m;
		m.initialize(resolution, size_x, size_y, size_z);
		maps_.push_back(std::move(m)); //move constructor

		pose empty_pose;
		printf("%f %f %f , %f %f %f %f\n", empty_pose.pos.x, empty_pose.pos.y, empty_pose.pos.z, empty_pose.ori.x, empty_pose.ori.y, empty_pose.ori.z,
				empty_pose.ori.w);
		map_origins_.push_back(empty_pose);
	}

	//initialize map counters
	for (int i = 0; i < map_num; i++)
	{
		map_counter_.push_back(0);
	}

	//initialize master map
	master_map_.initialize(resolution, size_x, size_y, size_z);
}

int exploration::map_merger::receive_map_update(const map_update& update, cell_list & updated_cells)
{
	int map_id = update.map_id;
	if (map_id >= config_.number_of_maps)
	{
		return 0;
	}

	//update specified map: no transform required
	printf("update map %d\n", update.map_id);
	cell_list locally_updated_cells;
	update_map(update, pose(), maps_[map_id], &locally_updated_cells);

	//increment counter
	map_counter_[map_id]++;

	//check map counter for all maps to see if updating master map is appropriate
	bool update_master = true;
	for (auto c : map_counter_)
	{
		//printf("map counter value %d\n", c);
		if (c < 10)
		{
			update_master = false;
			break;
		}
	}

	printf("size of map origins is %d\n", (int) map_origins_.size());
	for (auto empty_pose : map_origins_)
	{
		//empty_pose
		printf("%f %f %f , %f %f %f %f\n", empty_pose.pos.x, empty_pose.pos.y, empty_pose.pos.z, empty_pose.ori.x, empty_pose.ori.y, empty_pose.ori.z,
				empty_pose.ori.w);
	}

	if (update_master)
	{
		// initialize origins (transforms to master frame)
		if (!origins_initialized_)
		{
			generate_initial_transforms();
			origins_initialized_ = true;
		}

		//update master map frame using its transform from child frame
		printf("update master map with map id %d\n", (int) map_id);
		update_map(update, map_origins_[map_id], master_map_, &updated_cells);
	}
	else
	{
		//if not time to update master return list of updated cells as those that were updated on the local map
		updated_cells.list.insert(updated_cells.list.end(), locally_updated_cells.list.begin(), locally_updated_cells.list.end());
	}

	return 1;

}

int exploration::map_merger::get_map(int map_id, const generic_map<exploration_type>*& map)
{
	if (map_id >= config_.number_of_maps)
	{
		return 0;
	}

	map = &maps_[map_id];

	return 1;
}

int exploration::map_merger::get_master_map(const generic_map<exploration_type>*& map)
{
	map = &master_map_;
	return 1;
}

int exploration::map_merger::update_map(const map_update& update, const pose& destination_frame, generic_map<exploration_type>& map,
		cell_list * updated_cells)
{
	double res = config_.map_config_.resolution;
	auto destination = destination_frame;
	int update_count = 0;

	printf("update map using pos %f %f %f\n", destination_frame.pos.x, destination_frame.pos.y, destination_frame.pos.z);

	for (auto p : update.points)
	{
		//transform point to local frame
		p = generic_transform::transform_position_to_frame(p, config_.map_config_.origin);

		//transform point to map frame
		p = generic_transform::transform_position_to_frame(p, destination);

		//discretize
		cell current_cell;
		current_cell.X = exploration_map::discretize(p.x, res);
		current_cell.Y = exploration_map::discretize(p.y, res);
		current_cell.Z = exploration_map::discretize(p.z, res);
		auto val = p.value;

		//if out of bounds, continue
		if (!map.is_in_bounds(current_cell))
		{
			continue;
		}

		//set cell value
		auto prev_val = map[current_cell.X][current_cell.Y][current_cell.Z];
		map[current_cell.X][current_cell.Y][current_cell.Z] = val;

		if(prev_val != map[current_cell.X][current_cell.Y][current_cell.Z])
		{
			update_count++;
			if(updated_cells != NULL)
			{
				updated_cells->push_back(current_cell);
			}
		}

	}
	//printf("number of obstacles is %d\n", obs_count);
	return update_count;
}

void exploration::map_merger::generate_initial_transforms()
{
	//get scan match params
	int max_dx = config_.scan_match_config_.dx;
	int max_dy = config_.scan_match_config_.dy;
	int max_dz = config_.scan_match_config_.dz;
	int max_dyaw = config_.scan_match_config_.dyaw;
	double m_res = config_.scan_match_config_.metric_res;
	double a_res = config_.scan_match_config_.angular_res;

	printf("generate initial transforms\n");

	//set master map values to first map
	master_map_.map_ = maps_[0].map_;

	//get base scans for all maps besides first
	std::vector<std::vector<point_valued<exploration_type> > > map_scans;
	int occ_cells = 0;
	for (int i = 1; i < config_.number_of_maps; i++)
	{
		auto & current_map = maps_[i];
		map_scans.push_back(std::vector<point_valued<exploration_type> >());

		//search map for occupied cells
		for (int x = 0; x < current_map.size_x; x++)
		{
			for (int y = 0; y < current_map.size_y; y++)
			{
				for (int z = 0; z < current_map.size_z; z++)
				{
					//grab continuous position
					if (current_map[x][y][z] == exploration_type::occupied)
					{
						point_valued<exploration_type> p;
						p.x = exploration_map::continuous(x, current_map.resolution);
						p.y = exploration_map::continuous(y, current_map.resolution);
						p.z = exploration_map::continuous(z, current_map.resolution);
						p.value = exploration_type::occupied;

						//transform back to world
						p = generic_transform::transform_position_from_frame(p, config_.map_config_.origin);

						map_scans[i - 1].push_back(p);
						occ_cells++;
					}
				}
			}
		}
		//printf("number of occupied cells in scan %d \n", occ_cells);
	}

//	std::string blah2;
//	std::cin >> blah2;

	std::vector<pose> best_origin_list;

	//for each base scan
	for (auto scan : map_scans)
	{
		//max_affinity = 0;
		int max_affinity = 0;

		//set best origin = 0;
		pose best_origin;

		//for each dx increment
		for (int dx = -max_dx; dx <= max_dx; dx++)
		{
			//for each dy increment
			for (int dy = -max_dy; dy <= max_dy; dy++)
			{
				//for each dz increment
				for (int dz = -max_dz; dz <= max_dz; dz++)
				{
					//for each dyaw increment
					for (int dyaw = -max_dyaw; dyaw <= max_dyaw; dyaw++)
					{
						//make scan copy
						auto current_scan = scan;

						//transform points by xc,yc,zc,yaw
						double xc = static_cast<double>(dx) * m_res;
						double yc = static_cast<double>(dy) * m_res;
						double zc = static_cast<double>(dz) * m_res;
						double yawc = static_cast<double>(dyaw) * a_res;
						//printf("dyaw %d yawc%f\n", dyaw, yawc);
						pose trans;
						trans.pos.x = xc;
						trans.pos.y = yc;
						trans.pos.z = zc;
						double quat[4];
						generic_transform::convert_eular_to_quaterion(0, 0, yawc, quat);
						generic_transform::convert_quaternion_to_orientation(quat, trans.ori);
						for (auto & p : current_scan)
						{
							//transform by offset
							p = generic_transform::transform_position_to_frame(p, trans);

							//transform back to frame of map
							p = generic_transform::transform_position_to_frame(p, config_.map_config_.origin);
						}

						//discretize points into cells
						std::vector<cell_valued<exploration_type> > cell_list;
						cell_list.reserve(current_scan.size());
						for (auto & p : current_scan)
						{
							cell_valued<exploration_type> c;
							c.X = exploration_map::discretize(p.x, config_.map_config_.resolution);
							c.Y = exploration_map::discretize(p.y, config_.map_config_.resolution);
							c.Z = exploration_map::discretize(p.z, config_.map_config_.resolution);
							c.value = p.value;
							cell_list.push_back(c);
						}

						//affinity = 0;
						int affinity = 0;

						//for each cell
						for (auto & c : cell_list)
						{

							// bounds check cell
							if (!master_map_.is_in_bounds(c))
							{
								continue;
							}

							// if cell has same value  increment affinity
							if (master_map_[c.X][c.Y][c.Z] == c.value)
							{
								affinity++;
							}
						}

						//printf("origin position %f %f %f\n", trans.pos.x, trans.pos.y, trans.pos.z);
						//printf("origin orientation %f %f %f %f\n", trans.ori.w, trans.ori.x, trans.ori.y, trans.ori.z);
						//printf("affinity %d\n", affinity);

						//if affinity > max_affinity
						if (affinity > max_affinity)
						{

							//max affinity = affinity
							max_affinity = affinity;

							//set origin for this map to best origin
							best_origin = trans;
						}
					}
				}
			}
		}
		printf("best affinity %d\n", max_affinity);
		printf("best origin position %f %f %f\n", best_origin.pos.x, best_origin.pos.y, best_origin.pos.z);
		printf("best origin orientation %f %f %f %f\n", best_origin.ori.w, best_origin.ori.x, best_origin.ori.y, best_origin.ori.z);
		best_origin_list.push_back(best_origin);
	}

	//now re-assign origins to all maps but first
	for (size_t i = 1; i < map_origins_.size(); i++)
	{
		map_origins_[i] = best_origin_list[i - 1];
		printf("map_origins %d is %f %f %f\n", (int) i, map_origins_[i].pos.x, map_origins_[i].pos.y, map_origins_[i].pos.z);
	}

}

int exploration::map_merger::get_origin(int map_id, const pose*& origin)
{
	if (map_id >= config_.number_of_maps)
	{
		return 0;
	}

	origin = &map_origins_[map_id];

	return 1;
}

bool exploration::map_merger::origins_are_initialized()
{
	return origins_initialized_;
}
