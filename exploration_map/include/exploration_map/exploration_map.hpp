/*
 * exploration_map.hpp
 *
 *  Created on: Oct 1, 2014
 *      Author: bmacallister
 */

#ifndef EXPLORATION_MAP_HPP_
#define EXPLORATION_MAP_HPP_

#include <exploration_map/exploration_map.h>

template<typename T>
inline exploration::generic_map<T>::generic_map()
{
	resolution = 0;
	size_x = 0;
	size_y = 0;
	size_z = 0;
}

template<typename T>
inline exploration::generic_map<T>::~generic_map()
{
}

template<typename T>
inline exploration::generic_map<T>::generic_map(double _resolution, int _size_x, int _size_y, int _size_z)
{
	initialize(_resolution, _size_x, _size_y, _size_z);
}

template<typename T>
inline void exploration::generic_map<T>::initialize(double _resolution, int _size_x, int _size_y, int _size_z)
{
	resolution = _resolution;
	size_x = _size_x;
	size_y = _size_y;
	size_z = _size_z;
	map_.resize(size_x);
	for (int i = 0; i < size_x; i++)
	{
		map_[i].resize(size_y);
		for(int j = 0; j < size_y; j++)
		{
			map_[i][j].resize(size_z);
		}
	}
}

template<typename T>
inline void exploration::generic_map<T>::initialize(double _resolution, int _size_x, int _size_y, int _size_z, T _default_value)
{
	initialize(_resolution, _size_x, _size_y, _size_z);
	for (int i = 0; i < size_x; i++)
	{
		for (int j = 0; j < size_y; j++)
		{
			for (int k = 0; k < size_z; k++)
			{
				map_[i][j][k] = _default_value;
			}
		}
	}
}
template<typename T> template<typename cell>
inline bool exploration::generic_map<T>::is_in_bounds(cell c)
{
	if (c.X < 0 || c.X >= size_x)
	{
		return false;
	}
	if (c.Y < 0 || c.Y >= size_y)
	{
		return false;
	}
	if (c.Z < 0 || c.Z >= size_z)
	{
		return false;
	}

	return true;
}




#endif /* EXPLORATION_MAP_HPP_ */
