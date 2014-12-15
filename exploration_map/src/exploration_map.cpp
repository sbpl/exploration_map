/*
 * exploration_map.cpp
 *
 *  Created on: Oct 1, 2014
 *      Author: bmacallister
 */

#include <exploration_map/exploration_map.h>
#include <exploration_map/exploration_map.hpp>

template class generic_map<exploration_type> ;
template class generic_map<int> ;
template class generic_map<double> ;

int exploration::exploration_map::discretize(double s, double res)
{
	double v = s / res;
	v >= 0 ? v = floor(v) : v = ceil(v - 1);
	int d = static_cast<int>(v);
	return d;
}

exploration::exploration_map::exploration_map(exploration_map_config _config)
{
	initialize(_config);
}

exploration::exploration_map::~exploration_map()
{
}

bool exploration::exploration_map::update_map(const sensor_update::sensor_update& update, cell_list & updated_cells)
{
	//apply update procedure depending on update type
	cell_list sensor_updated_cells;
	auto t = update.get_type();
	switch (t)
	{
	case sensor_update::sensor_update_type::lidar:
	{
		update_occupancy_map(update, sensor_updated_cells, config_.occ_map_config_.lidar_update_increment_value, config_.occ_map_config_.lidar_update_decrement_value);
		break;
	}
	case sensor_update::sensor_update_type::camera:
	{
		update_observation_map(update, sensor_updated_cells);
		break;
	}
	case sensor_update::sensor_update_type::robot_volume:
	{
		update_occupancy_map(update, sensor_updated_cells,1.0,1.0);
		update_exploration_map(sensor_updated_cells, updated_cells);
		update_observation_map(update, sensor_updated_cells);
		break;
	}
	default:
		printf("exploration_map: warning do not know of sensor update type given\n");
		break;
	}

	//update exploration map from updated cells
	return update_exploration_map(sensor_updated_cells, updated_cells);
}

const exploration::generic_map<exploration::exploration_type>* exploration::exploration_map::get_exploration_map()
{
	return &exp_map;
}

bool exploration::exploration_map::initialize(exploration_map_config _config)
{
	config_ = _config;
	disc_origin_ = config_.map_config_.origin;
	disc_origin_.pos.x = discretize(config_.map_config_.origin.pos.x, config_.map_config_.resolution);
	disc_origin_.pos.y = discretize(config_.map_config_.origin.pos.y, config_.map_config_.resolution);
	disc_origin_.pos.z = discretize(config_.map_config_.origin.pos.z, config_.map_config_.resolution);

	//initialize maps
	double resolution = config_.map_config_.resolution;
	int size_x = config_.map_config_.size_x;
	int size_y = config_.map_config_.size_y;
	int size_z = config_.map_config_.size_z;

	exp_map.initialize(resolution, size_x, size_y, size_z);
	occupancy_map.initialize(resolution, size_x, size_y, size_z, 0.5);
	observation_map.initialize(resolution, size_x, size_y, size_z, 0);
	return true;
}

bool exploration::exploration_map::update_occupancy_map(const sensor_update::sensor_update& update, cell_list& updated_cells, double increment_value, double decrement_value)
{
	std::vector<sensor_update::discrete_cell> cells;
	//todo: have this function return cells that line up with frame
	update.get_discrete_ray_trace_cells(config_.map_config_.resolution, cells);

	for (auto a : cells)
	{

		//convert to exploration_map::cell
		cell current_cell;
		current_cell.X = a.X;
		current_cell.Y = a.Y;
		current_cell.Z = a.Z;

		//get cell in map frame
		current_cell = map_frame(current_cell);

		//if out of bounds, continue
		if (!occupancy_map.is_in_bounds(current_cell))
		{
			continue;
		}

		double prev_val = occupancy_map[current_cell.X][current_cell.Y][current_cell.Z];

		//update
		double new_value = prev_val;
		if (a.value > 0)
		{
			new_value += increment_value;
		}
		else
		{
			new_value -= decrement_value;
		}
		new_value = std::max(std::min(new_value, 1.0), 0.0);
		occupancy_map[current_cell.X][current_cell.Y][current_cell.Z] = new_value;

		//if cell update add to updated list
		if (new_value != prev_val)
		{
			updated_cells.push_back(current_cell);
		}
	}
	return true;

}

bool exploration::exploration_map::is_in_bounds(const cell& c)
{
	if (c.X < 0 || c.X >= config_.map_config_.size_x)
	{
		return false;
	}
	if (c.Y < 0 || c.Y >= config_.map_config_.size_y)
	{
		return false;
	}
	if (c.Z < 0 || c.Z >= config_.map_config_.size_z)
	{
		return false;
	}

	return true;
}

exploration::cell exploration::exploration_map::map_frame(const cell& c)
{
	point kp;
	kp.x = static_cast<double>(c.X);
	kp.y = static_cast<double>(c.Y);
	kp.z = static_cast<double>(c.Z);

	//transform to map frame
	kp = generic_transform::transform_position_to_frame(kp, disc_origin_);

	//set to cell
	cell k;
	k.X = static_cast<int>(round(kp.x));
	k.Y = static_cast<int>(round(kp.y));
	k.Z = static_cast<int>(round(kp.z));

	return k;
}

bool exploration::exploration_map::update_exploration_map(const cell_list& sensor_update_cells, cell_list & updated_cells)
{
	//for each updated cell
	for (auto a : sensor_update_cells.list)
	{
		bool occupied = false;
		bool explored = false;
		bool unoccupied = false;

		//check occupancy
		if (occupancy_map[a.X][a.Y][a.Z] > config_.occ_map_config_.occ_threshold)
		{
			occupied = true;
		}

		if (occupancy_map[a.X][a.Y][a.Z] < config_.occ_map_config_.unnoc_threshold)
		{
			unoccupied = true;
		}

		//check if explored
		if (observation_map[a.X][a.Y][a.Z] > 0)
		{
			explored = true;
		}

		auto prev_value = exp_map[a.X][a.Y][a.Z];

		//update value
		exp_map[a.X][a.Y][a.Z] = exploration_type::unknown;
		if (unoccupied)
		{
			exp_map[a.X][a.Y][a.Z] = exploration_type::unoccupied;
		}
		if (explored)
		{
			exp_map[a.X][a.Y][a.Z] = exploration_type::explored;
		}
		if (occupied)
		{
			exp_map[a.X][a.Y][a.Z] = exploration_type::occupied;
		}

		//if exploration cell changed, add to list of updated cells
		if (prev_value != exp_map[a.X][a.Y][a.Z])
		{
			updated_cells.push_back(a);
		}

	}
	return true;
}

exploration::exploration_map::exploration_map()
{
}

const exploration::exploration_map_config* exploration::exploration_map::get_configuration() const
{
	return &config_;
}

bool exploration::exploration_map::update_observation_map(const sensor_update::sensor_update& update, cell_list& updated_cells)
{
	std::vector<sensor_update::discrete_cell> cells;
	update.get_discrete_ray_trace_cells(config_.map_config_.resolution, cells);

	bool skip_through_points = false;

	sensor_update::pose sensor_pose = update.get_pose();
	cell root_cell;
	root_cell.X = discretize(sensor_pose.pos.x, config_.map_config_.resolution);
	root_cell.Y = discretize(sensor_pose.pos.y, config_.map_config_.resolution);
	root_cell.Z = discretize(sensor_pose.pos.z, config_.map_config_.resolution);
	root_cell = map_frame(root_cell);

	//assuming base level is the only level that matters for determining traversability

	for (auto a : cells)
	{

		//convert to exploration_map::cell
		cell current_cell;
		current_cell.X = a.X;
		current_cell.Y = a.Y;
		current_cell.Z = a.Z;

		//get cell in map frame
		current_cell = map_frame(current_cell);

		//if out of bounds, continue
		if (!observation_map.is_in_bounds(current_cell))
		{
			continue;
		}

		//Store previous value
		double prev_val = observation_map[current_cell.X][current_cell.Y][current_cell.Z];

		//Reset through point flag if current cell near origin
		if (skip_through_points)
		{
			if (abs(root_cell.X - current_cell.X) < 2 && abs(root_cell.Y - current_cell.Y) < 2 && abs(root_cell.Z - current_cell.Z) < 2)
			{
				skip_through_points = false;
			}
		}

		//Skip this value if it was considered a through point
		if (skip_through_points)
		{
			continue;
		}

		bool occ = false;
		bool unk = false;
		//Start skipping points if current point goes over impassable terrain
		if (exp_map[current_cell.X][current_cell.Y][current_cell.Z] == exploration_type::occupied)
		{
			occ = true;
		}

		//Start skipping points if current point goes over unknown terrain
		if (exp_map[current_cell.X][current_cell.Y][current_cell.Z] == exploration_type::unknown)
		{
			unk = true;
		}

		if ((occ || unk))
		{
			skip_through_points = true;
			continue;
		}

		//update value of observation
		double new_value = 1;
		observation_map[current_cell.X][current_cell.Y][current_cell.Z] = new_value;

		//if cell update add to updated list
		if (new_value != prev_val)
		{
			updated_cells.push_back(current_cell);
		}
	}
	return true;

}

double exploration::exploration_map::continuous(int d, double res)
{
	double s = (static_cast<double>(d) * res) + (res / 2.0);
	return s;
}

