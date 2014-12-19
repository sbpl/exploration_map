/*
 * exploration_map_node.h
 *
 *  Created on: Sep 23, 2014
 *      Author: bmacallister
 */

#ifndef EXPLORATION_MAP_NODE_H_
#define EXPLORATION_MAP_NODE_H_

#include <iostream>
#include <string>
#include <ros/ros.h>
#include "ros/time.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_update/sensor_update.h>
#include <exploration_map/exploration_map.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <algorithm>
#include <camera_node/camera_scan.h>

/**
 * Interface between ros environment and exploration map
 */
class exploration_map_node
{
public:

	/**
	 * Constructor
	 */
	exploration_map_node();

	/**
	 * Destructor
	 */
	~exploration_map_node();

	/**
	 * sets up ROS related infrastructure
	 */
	void ros_configure();

private:

	/**
	 *
	 * @param msg
	 */
	void horizontal_lidar_callback(const sensor_msgs::LaserScanConstPtr & msg);

	void vertical_lidar_callback(const sensor_msgs::LaserScanConstPtr & msg);

	void camera_scan_callback(const camera_node::camera_scanConstPtr & msg);

	void map_publish_timer_callback(const ros::TimerEvent & event);

	void convert_pose_to_sensor_update_pose(const geometry_msgs::PoseStamped pose, sensor_update::pose & sensor_pose);

	void convert_laser_scan_to_sensor_update_ray(const sensor_msgs::LaserScan & scan, sensor_update::sensor_reading & reading);

	void convert_camera_scan_to_sensor_update_ray(const camera_node::camera_scan & scan, sensor_update::sensor_reading & reading);

	void publish_sensor_update(const sensor_update::sensor_update & update, const ros::Publisher & pub);

	void publish_sensor_update_ray_trace(const sensor_update::sensor_update & update, const ros::Publisher & pub);

	void publish_point_cloud(const std::vector<pcl::PointXYZI> & points, const ros::Publisher & pub);

	bool get_lateset_pose_from_tf(geometry_msgs::PoseStamped & pose, const std::string & pose_name, const ros::Time & time);

	bool initialize_exploration_map();

	bool update_exploration_map(const sensor_update::sensor_update & update);

	void publish_exploration_map();

	void publish_exploration_map_update();

	void update_robot_pose(const ros::Time & time);

	void publish_robot_pose();

	void update_exploration_map_from_robot_pose(const geometry_msgs::PoseStamped & pose);

	double map_resolution;
	geometry_msgs::Point map_origin;
	int map_size_x;
	int map_size_y;
	int map_size_z;
	std::string frame_name;
	std::string horizontal_lidar_pose_tf_name;
	std::string vertical_lidar_pose_tf_name;
	std::string camera_pose_tf_name;
	std::string robot_pose_tf_name;
	std::string horizontal_lidar_topic_name;
	std::string vertical_lidar_topic_name;
	std::string camera_scan_topic_name;
	double map_publish_rate;

	double occupancy_prob_thresh;
	double unnocupied_prob_thresh;
	double lidar_update_increment;
	double lidar_update_decrement;
	int number_of_scans_to_skip;
	int camera_scan_counter;
	int horizontal_lidar_scan_counter;
	int vertical_lidar_scan_counter;
	double ground_plane_height_threshold;
	bool publish_debug_messages;
	double min_sensor_distance_threshold;
	double robot_radius;
	double robot_height;

	//node handles
	ros::NodeHandle n;
	ros::NodeHandle pn;
	tf::TransformListener listener;

	//ros subscriber
	ros::Subscriber horiz_lidar_sub;
	ros::Subscriber verti_lidar_sub;
	ros::Subscriber camera_scan_sub;

	//ros timer
	ros::Timer map_publish_timer;

	//ros publishers
	ros::Publisher horizontal_lidar_update_pub;
	ros::Publisher horizontal_lidar_update_ray_trace_pub;
	ros::Publisher vertical_lidar_update_pub;
	ros::Publisher vertical_lidar_update_ray_trace_pub;
	ros::Publisher camera_update_pub;
	ros::Publisher camera_update_ray_trace_pub;
	ros::Publisher exploration_map_pub;
	ros::Publisher exploration_map_update_pub;
	ros::Publisher robot_pose_pub;
	ros::Publisher robot_vol_update_pub;
	ros::Publisher robot_vol_update_ray_trace_pub;

	exploration::exploration_map exp_map;
	exploration::cell_list updated_cell_list;
	geometry_msgs::PoseStamped current_robot_pose;

};

#endif /* EXPLORATION_MAP_NODE_H_ */
