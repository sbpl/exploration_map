/*
 * map_merger_node.cpp
 *
 *  Created on: Oct 22, 2014
 *      Author: bmacallister
 */

#include <map_merger_node/map_merger_node.h>

map_merger_node::map_merger_node() :
		nh(), ph("~")
{
	initialize();
}

map_merger_node::~map_merger_node()
{
}

void map_merger_node::initialize()
{
	setup_ros();
}

void map_merger_node::setup_ros()
{
	ph.getParam("frame", frame_name);
	ph.getParam("map_resolution", map_resoultion);
	ph.getParam("map_origin_x", map_origin_x);
	ph.getParam("map_origin_y", map_origin_y);
	ph.getParam("map_size_x", map_size_x);
	ph.getParam("map_size_y", map_size_y);
	ph.getParam("robot_0_map_name", robot_0_map_name);
	ph.getParam("robot_1_map_name", robot_1_map_name);
	ph.getParam("map_publish_rate", map_publish_rate);

	//timers
	map_publish_timer = nh.createTimer(ros::Duration(map_publish_rate), &map_merger_node::map_publish_callback, this);

	//subscribers
	robot_0_map_subscriber = nh.subscribe(robot_0_map_name.c_str(), 1, &map_merger_node::robot_0_map_callback, this);
	robot_1_map_subscriber = nh.subscribe(robot_1_map_name.c_str(), 1, &map_merger_node::robot_1_map_callback, this);

	//publishers
	map_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZI> >("master_map", 1);

}

int main(int argc, char ** argv)
{

	ros::init(argc,argv,"map_merger_node");
	map_merger_node map_merger;
	ros::spin();

}


void map_merger_node::map_publish_callback(const ros::TimerEvent& event)
{
	ROS_INFO("publish master map");
	publish_master_map();
}

void map_merger_node::robot_0_map_callback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& msg)
{
	ROS_INFO("got map 0");
	map_0_point_cloud = *msg;
}

void map_merger_node::robot_1_map_callback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& msg)
{
	ROS_INFO("got map 1");
	map_1_point_cloud = *msg;
}

void map_merger_node::publish_master_map()
{
	int size_x = map_size_x;
	int size_y = map_size_y;
	double res = map_resoultion;
	double origin_x = map_origin_x;
	double origin_y = map_origin_y;
	double origin_z = 0;

	//todo: replace this with what map merger returns
	std::vector<pcl::PointXYZI> points;
	for(auto & p : map_0_point_cloud.points)
	{
		points.push_back(p);
	}
	for(auto & p : map_1_point_cloud.points)
	{
		points.push_back(p);
	}

	//done

//	std::vector<pcl::PointXYZI> points;
//	for (int x = 0; x < size_x; x++)
//	{
//		for (int y = 0; y < size_y; y++)
//		{
//
//			pcl::PointXYZI p;
//			p.x = x * res + res / 2 + origin_x;
//			p.y = y * res + res / 2 + origin_y;
//			p.z = 0 + origin_z;
//
//			exploration_map::exploration_type val = map->at(x, y);
//
//			switch (val)
//			{
//			case exploration_map::exploration_type::occupied:
//				p.intensity = 100;
//				break;
//			case exploration_map::exploration_type::explored:
//				p.intensity = 50;
//				break;
//			default:
//				p.intensity = 0;
//				break;
//			}
//
//			points.push_back(p);
//		}
//	}
	publish_point_cloud(points, map_publisher);
}

void map_merger_node::publish_point_cloud(const std::vector<pcl::PointXYZI> & points, const ros::Publisher & pub)
{
	pcl::PointCloud<pcl::PointXYZI> cloud;
	cloud.points.resize(points.size());
	std::copy(points.begin(), points.end(), cloud.points.begin());
	cloud.height = 1;
	cloud.width = cloud.points.size();
	cloud.header.frame_id = frame_name;
	cloud.header.stamp = ros::Time::now();
	pub.publish(cloud);
}
