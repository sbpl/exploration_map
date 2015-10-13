/*
 * exploration_map_types.cpp
 *
 *  Created on: Nov 26, 2014
 *      Author: bmacallister
 */

#include <exploration_map/exploration_map/exploration_map.h>

using namespace exploration;

/*
 * point implementation
 */

point::point()
{
	x = 0;
	y = 0;
	z = 0;
}

point::point(const point & _p)
{
	x = _p.x;
	y = _p.y;
	z = _p.z;
}

point::point(double _x, double _y, double _z)
{
	x = _x;
	y = _y;
	z = _z;
}

std::string point::to_string() const
{
	std::stringstream f;
	int prec = 3;
	f << std::fixed << std::setprecision(prec);
	std::stringstream stream;
	stream << "x: " << f.str() << x;
	stream << " y: " << f.str() << y;
	stream << " z: " << f.str() << z;
	return stream.str();
}

std::ostream& operator<< (std::ostream &out, const point & p)
{
	out << p.to_string();
	return out;
}

/*
 * orientation implementation
 */

orientation::orientation()
{
	x = 0;
	y = 0;
	z = 0;
	w = 1.0;
}

orientation::orientation(const orientation & _c)
{
	x = _c.x;
	y = _c.y;
	z = _c.z;
	w = _c.w;
}

orientation::orientation(double _x, double _y, double _z, double _w)
{
	x = _x;
	y = _y;
	z = _z;
	w = _w;
}

std::string orientation::to_string() const
{
	std::stringstream f;
	int prec = 3;
	f << std::fixed << std::setprecision(prec);
	std::stringstream stream;
	stream << "w: " << f.str() << w;
	stream << " x: " << f.str() << x;
	stream << " y: " << f.str() << y;
	stream << " z: " << f.str() << z;
	return stream.str();
}

std::ostream& operator<< (std::ostream &out, const orientation & ori)
{
	out << ori.to_string();
	return out;
}

/*
 * pose implementation
 */

pose::pose() :
		pos(), ori()
{

}

pose::pose(point _pos, orientation _ori) :
		pos(_pos), ori(_ori)
{

}

std::string pose::to_string() const
{
	std::stringstream stream;
	stream << "pos: " << pos.to_string() ;
	stream << " ori: " << ori.to_string();
	return stream.str();
}

std::ostream& operator<< (std::ostream &out, const pose & p)
{
	out << p.to_string();
	return out;
}

/*
 * cell list implementation
 */

void cell_list::push_back(exploration::cell c)
{
	list.push_back(c);
}

size_t cell_list::size()
{
	return list.size();
}
