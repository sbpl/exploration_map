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
	point origin = config_.map_config_.origin;
	int size_x = config_.map_config_.size_x;
	int size_y = config_.map_config_.size_y;
	int size_z = config_.map_config_.size_z;

	//initialize maps and origins
	for (int i = 0; i < map_num; i++)
	{
		generic_map<exploration_type> m;
		m.initialize(resolution, size_x, size_y, size_z);
		maps_.push_back(std::move(m)); //move constructor
		map_origins_.push_back(pose());
	}

	//initialize map counters
	for (int i = 0; i < map_num; i++)
	{
		map_counter_.push_back(0);
	}

	//initialize master map
	master_map_.initialize(resolution, size_x, size_y, size_z);
}

int exploration::map_merger::receive_map_update(const map_update& update)
{
	int map_id = update.map_id;
	if (map_id >= config_.number_of_maps)
	{
		return 0;
	}

	//update specified map: no transform required
	update_map(update, pose(), config_.map_config_.origin, maps_[map_id]);

	//increment counter
	map_counter_[map_id]++;

	//check map counter for all maps to see if updating master map is appropriate
	bool update_master = true;
	for (auto c : map_counter_)
	{
		if (c < 1)
		{
			update_master = false;
			break;
		}
	}

	// initialize origins (transforms to master frame)
	if (!origins_initialized_)
	{
		generate_initial_transforms();
	}

	//update master map frame using its transform from child frame
	update_map(update, pose(), map_origins_[map_id], maps_[map_id]);

	return 1;

}

int exploration::map_merger::get_map(int map_id, const generic_map<exploration_type>* map)
{
	if (map_id >= config_.number_of_maps)
	{
		return 0;
	}

	map = &maps_[map_id];

	return 1;
}

int exploration::map_merger::get_master_map(const generic_map<exploration_type>* map)
{
	map = &master_map_;
	return 1;
}

int exploration::map_merger::update_map(const map_update& update, const pose& origin_frame, const pose& destination_frame,
		generic_map<exploration_type>& map)
{
	double res = config_.map_config_.resolution;
	auto origin = origin_frame;
	auto destination = destination_frame;
	int update_count = 0;
	for (auto p : update.points)
	{
		//transform from point to global frame
		p = generic_transform::transform_position_from_frame(p, origin);

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
		map[current_cell.X][current_cell.Y][current_cell.Z] = val;

		update_count++;

	}
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

	//set origin map (first map)
	const auto & origin_map = maps_[0];

	//get base scans for all maps besides first
	std::vector<std::vector<point_valued<exploration_type> > > map_scans;
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
						map_scans[i - 1].push_back(p);
					}
				}
			}
		}
	}

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
						double yawc = static_cast<double>(dyaw) * m_res;
						pose trans;
						trans.pos.x = xc;
						trans.pos.y = yc;
						trans.pos.z = zc;
						double quat[4];
						generic_transform::convert_eular_to_quaterion(0, 0, yawc, quat);
						generic_transform::convert_quaternion_to_orientation(quat, trans.ori);
						for (auto auto & p : current_scan)
						{
							p = generic_transform::transform_position_to_frame(p, trans);
						}

						//discretize points into cells
						std::vector<cell_valued<exploration_type> > cell_list;
						cell_list.reserve(current_scan.size());
						for (auto & p : current_scan)
						{
							cell c;
							c.X = exploration_map::discretize(p.x, config_.map_config_.resolution);
							c.Y = exploration_map::discretize(p.y, config_.map_config_.resolution);
							c.Z = exploration_map::discretize(p.z, config_.map_config_.resolution);
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

						//if affinity > max_affinity
						if (affinity > max_affinity)
						{

							//max affinity = affinity
							max_affinity = affinity;

							// origin = orign from x,y,z,yaw
							point pos(xc, yc, zc);
							orientation ori;
							double quat[4];
							generic_transform::convert_eular_to_quaterion(0, 0, yawc, quat);
							generic_transform::convert_quaternion_to_orientation(quat, ori);
							pose origin(pos, ori);

							//set origin for this map to best origin
							best_origin = origin;

						}
					}
				}
			}
		}
		best_origin_list.push_back(best_origin);
	}

	//now re-assign origins to all maps but first
	for(size_t i =1; i < map_origins_.size(); i++)
	{
		map_origins_[i] = best_origin_list[i-1];
	}

}

