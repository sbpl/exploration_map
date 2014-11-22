/*
 * map_merger_node.h
 *
 *  Created on: Oct 22, 2014
 *      Author: bmacallister
 */

#ifndef MAP_MERGER_NODE_H_
#define MAP_MERGER_NODE_H_

#include <iostream>
#include <string>
#include <ros/ros.h>
#include "ros/time.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <exploration_map/exploration_map.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <algorithm>

class map_merger_node
{

public:
	map_merger_node();

	~map_merger_node();

	void initialize();

	void setup_ros();

private:

	void robot_0_map_callback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & msg);

	void robot_1_map_callback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & msg);

	void map_publish_callback(const ros::TimerEvent & event);

	void publish_point_cloud(const std::vector<pcl::PointXYZI> & points, const ros::Publisher & pub);

	void publish_master_map();

	//config variables
	std::string frame_name;
	double map_resoultion;
	double map_origin_x;
	double map_origin_y;
	int map_size_x;
	int map_size_y;
	double map_publish_rate;
	std::string robot_0_map_name;
	std::string robot_1_map_name;

	//ros node handlers
	ros::NodeHandle nh;
	ros::NodeHandle ph;

	//ros timer
	ros::Timer map_publish_timer;

	//ros subscribers
	ros::Subscriber robot_0_map_subscriber;
	ros::Subscriber robot_1_map_subscriber;

	//ros publishers
	ros::Publisher map_publisher;

	//variables
	pcl::PointCloud<pcl::PointXYZI> map_0_point_cloud;
	pcl::PointCloud<pcl::PointXYZI> map_1_point_cloud;



};

#endif /* MAP_MERGER_NODE_H_ */


