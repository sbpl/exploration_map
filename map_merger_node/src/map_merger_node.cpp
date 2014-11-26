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
	number_of_maps = 2;
	setup_ros();
	initialize_map_merger();
}

void map_merger_node::setup_ros()
{
	ph.getParam("frame", frame_name);
	ph.getParam("map_resolution", map_resoultion);
	ph.getParam("map_origin_x", map_origin_x);
	ph.getParam("map_origin_y", map_origin_y);
	ph.getParam("map_origin_z", map_origin_z);
	ph.getParam("map_size_x", map_size_x);
	ph.getParam("map_size_y", map_size_y);
	ph.getParam("map_size_z", map_size_z);
	ph.getParam("robot_0_map_name", robot_0_map_name);
	ph.getParam("robot_1_map_name", robot_1_map_name);
	ph.getParam("map_publish_rate", map_publish_rate);
	ph.getParam("scan_match_ang_res", scan_match_angular_res);
	ph.getParam("scan_match_met_res", scan_match_metric_res);
	ph.getParam("scan_match_dx", scan_match_dx);
	ph.getParam("scan_match_dy", scan_match_dy);
	ph.getParam("scan_match_dz", scan_match_dz);
	ph.getParam("scan_match_dyaw", scan_match_dyaw);

	ROS_INFO("frame %s\n", frame_name.c_str());
	ROS_INFO("map_resolution %f\n", map_resoultion);
	ROS_ERROR("scan_match_dx %d", scan_match_dx);
	ROS_ERROR("scan_match_dyaw %d", scan_match_dyaw);
	ROS_ERROR("map publish rate %f\n", map_publish_rate);

	//timers
	map_publish_timer = ph.createTimer(ros::Duration(map_publish_rate), &map_merger_node::map_publish_callback, this);

	//subscribers
	robot_0_map_subscriber = nh.subscribe(robot_0_map_name.c_str(), 1, &map_merger_node::robot_0_map_callback, this);
	robot_1_map_subscriber = nh.subscribe(robot_1_map_name.c_str(), 1, &map_merger_node::robot_1_map_callback, this);

//	//publishers
	map_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZI> >("master_map", 1);
	map_0_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZI> >("map0", 1);
	map_1_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZI> >("map1", 1);

}

int main(int argc, char ** argv)
{

	ros::init(argc, argv, "map_merger_node");
	map_merger_node map_merger;
	ROS_ERROR("map merger made\n");
	ros::spin();

}

void map_merger_node::map_publish_callback(const ros::TimerEvent& event)
{
	ROS_ERROR("publish master map");
	publish_master_map();
}

void map_merger_node::robot_0_map_callback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& msg)
{
	ROS_INFO("got map 0");
	map_0_point_cloud = *msg;

	//get map update
	exploration::map_update update;
	update.map_id = 0;
	convert_point_cloud_to_map_update(map_0_point_cloud, update);

	//update map merger
	merger.receive_map_update(update);

	ROS_INFO("map 0 update finished\n");
}

void map_merger_node::robot_1_map_callback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& msg)
{
	ROS_INFO("got map 1");
	map_1_point_cloud = *msg;

	//get map update
	exploration::map_update update;
	update.map_id = 1;
	convert_point_cloud_to_map_update(map_1_point_cloud, update);

	//update map merger
	merger.receive_map_update(update);

	ROS_INFO("map 1 update finished\n");
}

void map_merger_node::publish_master_map()
{

	//publish map 0 and 1 for debugging purposes
	const exploration::generic_map<exploration::exploration_type> * map0;
	const exploration::generic_map<exploration::exploration_type> * map1;
	merger.get_map(0, map0);
	merger.get_map(1, map1);
	std::vector<pcl::PointXYZI> map0_cloud, map1_cloud;
	get_point_cloud_from_map(map0, map0_cloud);
	get_point_cloud_from_map(map1, map1_cloud);
	publish_point_cloud(map0_cloud, map_0_publisher);
	publish_point_cloud(map1_cloud, map_1_publisher);

	//todo: publish master map
	const exploration::generic_map<exploration::exploration_type> * master_map;
	merger.get_master_map(master_map);
	std::vector<pcl::PointXYZI> master_map_cloud;
	get_point_cloud_from_map(master_map,master_map_cloud);
	publish_point_cloud(master_map_cloud, map_publisher);
}

void map_merger_node::publish_point_cloud(const std::vector<pcl::PointXYZI> & points, const ros::Publisher & pub)
{
	ROS_ERROR("publish point cloud with %d points\n", (int) points.size());
	pcl::PointCloud<pcl::PointXYZI> cloud;
	cloud.points.resize(points.size());
	std::copy(points.begin(), points.end(), cloud.points.begin());
	cloud.height = 1;
	cloud.width = cloud.points.size();
	cloud.header.frame_id = frame_name;
	cloud.header.stamp = ros::Time::now();
	pub.publish(cloud);
}

void map_merger_node::initialize_map_merger()
{
	exploration::map_merge_config con;

	//map size/resolution
	con.map_config_.size_x = map_size_x;
	con.map_config_.size_y = map_size_y;
	con.map_config_.size_z = map_size_z;
	con.map_config_.resolution = map_resoultion;

	//map origin
	exploration::pose origin;
	origin.pos.x = map_origin_x;
	origin.pos.y = map_origin_y;
	origin.pos.z = map_origin_z;
	con.map_config_.origin = origin;

	//number of maps
	con.number_of_maps = number_of_maps;

	//scan matching
	con.scan_match_config_.angular_res = scan_match_angular_res;
	con.scan_match_config_.dx = scan_match_dx;
	con.scan_match_config_.dy = scan_match_dy;
	con.scan_match_config_.dz = scan_match_dz;
	con.scan_match_config_.dyaw = scan_match_dyaw;
	con.scan_match_config_.metric_res = scan_match_metric_res;

	merger.initialize(con);

}

void map_merger_node::convert_point_cloud_to_map_update(pcl::PointCloud<pcl::PointXYZI>& point_cloud, exploration::map_update& update)
{
	update.points.reserve(point_cloud.size());
	for (auto & p : point_cloud)
	{
		exploration::point_valued<exploration::exploration_type> point;
		point.x = p.x;
		point.y = p.y;
		point.z = p.z;

		switch (static_cast<int>(p.intensity))
		{
		case 100:
			point.value = exploration::exploration_type::occupied;
			break;
		case 50:
			point.value = exploration::exploration_type::explored;
			break;
		default:
			point.value = exploration::exploration_type::unknown;
			break;
		}
		update.points.push_back(point);
	}
}

void map_merger_node::get_point_cloud_from_map(const exploration::generic_map<exploration::exploration_type>* map,
		std::vector<pcl::PointXYZI>& points)
{
	//map dimensions
	int size_x = map->size_x;
	int size_y = map->size_y;
	int size_z = map->size_z;
	double res = map->resolution;

	for (int x = 0; x < size_x; x++)
	{
		for (int y = 0; y < size_y; y++)
		{
			for (int z = 0; z < size_z; z++)
			{
				pcl::PointXYZI p;
				//todo call cont function
				p.x = x * res + res / 2 + map_origin_x;
				p.y = y * res + res / 2 + map_origin_y;
				p.z = z * res + res / 2 + map_origin_z;

				exploration::exploration_type val = map->at(x, y, z);

				switch (val)
				{
				case exploration::exploration_type::occupied:
					p.intensity = 100;
					break;
				case exploration::exploration_type::explored:
					p.intensity = 50;
					break;
				default:
					continue;
					break;
				}

				points.push_back(p);
			}
		}
	}

//	exploration::exploration_map::continuous(
}
