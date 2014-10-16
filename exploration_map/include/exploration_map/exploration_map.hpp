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
inline exploration_map::generic_map<T>::generic_map()
{
	resolution = 0;
	size_x = 0;
	size_y = 0;
}

template<typename T>
inline exploration_map::generic_map<T>::~generic_map()
{
}

template<typename T>
inline exploration_map::generic_map<T>::generic_map(double _resolution, int _size_x, int _size_y)
{
	initialize(_resolution, _size_x, _size_y);
}

template<typename T>
inline void exploration_map::generic_map<T>::initialize(double _resolution, int _size_x, int _size_y)
{
	resolution = _resolution;
	size_x = _size_x;
	size_y = _size_y;
	map_.resize(size_x);
	for (int i = 0; i < size_x; i++)
	{
		map_[i].resize(size_y);
	}
}

template<typename T>
inline void exploration_map::generic_map<T>::initialize(double _resolution, int _size_x, int _size_y, T _default_value)
{
	initialize(_resolution, _size_x, _size_y);
	for (int i = 0; i < size_x; i++)
	{
		for (int j = 0; j < size_y; j++)
		{
			map_[i][j] = _default_value;
		}
	}
}

#endif /* EXPLORATION_MAP_HPP_ */
