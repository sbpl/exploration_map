/*
 * map_merger_node.h
 *
 *  Created on: Oct 22, 2014
 *      Author: bmacallister
 */

#ifndef MAP_MERGER_NODE_H_
#define MAP_MERGER_NODE_H_

#include <algorithm>
#include <iostream>
#include <string>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include <exploration_map/exploration_map.h>
#include <exploration_map/map_merger/map_merger.h>

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

	void robot_0_pose_callback(const geometry_msgs::PoseStamped::ConstPtr & msg);

	void robot_1_pose_callback(const geometry_msgs::PoseStamped::ConstPtr & msg);

	void map_publish_callback(const ros::TimerEvent & event);

	void publish_point_cloud(const std::vector<pcl::PointXYZI> & points, const ros::Publisher & pub);

	void publish_pose(const geometry_msgs::PoseStamped & pose, const ros::Publisher & pub);

	void publish_robot_poses();

	void publish_inner_maps();

	void publish_master_map();

	void publish_map_update();

	void initialize_map_merger();

	void convert_point_cloud_to_map_update(pcl::PointCloud<pcl::PointXYZI> & point_cloud, exploration::map_update & update);

	void get_point_cloud_from_map(const exploration::generic_map<exploration::exploration_type> * map, std::vector<pcl::PointXYZI> & points);

	void get_point_cloud_from_map_update(const exploration::cell_list & cells, std::vector<pcl::PointXYZI> & points);

	void transform_pose_stamped_to_master_map_frame(geometry_msgs::PoseStamped & pose, const exploration::pose * origin);

	void transform_pose_stamped_to_local_map_frame(geometry_msgs::PoseStamped & pose, const exploration::pose * origin);

	void transform_pose_stamped_with_origin(geometry_msgs::PoseStamped & pose, const exploration::pose * origin, bool inverse);

	void goal_list_callback(const nav_msgs::PathConstPtr & msg);

	//config variables
	std::string frame_name;
	double map_resolution;
	double map_origin_x;
	double map_origin_y;
	double map_origin_z;
	int map_size_x;
	int map_size_y;
	int map_size_z;
	int number_of_maps;
	double map_publish_rate;
	std::string robot_0_map_name;
	std::string robot_1_map_name;
	std::string robot_0_pose_name;
	std::string robot_1_pose_name;
	std::string goal_list_name;
	double scan_match_angular_res;
	double scan_match_metric_res;
	int scan_match_dx;
	int scan_match_dy;
	int scan_match_dz;
	int scan_match_dyaw;
	std::string goal_0_name;
	std::string goal_1_name;

	bool m_publish_inner_maps;

	//ros node handlers
	ros::NodeHandle nh;
	ros::NodeHandle ph;

	//ros timer
	ros::Timer map_publish_timer;

	//ros subscribers
	ros::Subscriber robot_0_map_subscriber;
	ros::Subscriber robot_1_map_subscriber;
	ros::Subscriber robot_0_pose_subscriber;
	ros::Subscriber robot_1_pose_subscriber;
	ros::Subscriber goal_list_subscriber;

	//ros publishers
	ros::Publisher map_publisher;
	ros::Publisher map_update_publisher;
	ros::Publisher map_0_publisher;
	ros::Publisher map_1_publisher;
	ros::Publisher robot_1_pose_publisher;
	ros::Publisher robot_0_pose_publisher;
	ros::Publisher robot_poses_publisher;
	ros::Publisher robot_0_goal_publisher;
	ros::Publisher robot_1_goal_publisher;

	//variables
	pcl::PointCloud<pcl::PointXYZI> map_0_point_cloud;
	pcl::PointCloud<pcl::PointXYZI> map_1_point_cloud;
	exploration::map_merger merger;
	exploration::cell_list updated_cell_list;
	std::vector<geometry_msgs::PoseStamped> robot_poses;

};

#endif /* MAP_MERGER_NODE_H_ */

