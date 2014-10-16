/*
 * camera_node.h
 *
 *  Created on: Oct 14, 2014
 *      Author: bmacallister
 */

#ifndef CAMERA_NODE_H_
#define CAMERA_NODE_H_

#include <string>
#include <iostream>
#include <ros/ros.h>
#include <camera_node/camera_scan.h>
#include <sensor_msgs/Image.h>


class camera_scan_node
{

public:

	/**
	 *
	 */
	camera_scan_node();


	/**
	 *
	 */
	~camera_scan_node();

	/**
	 *
	 */
	void ros_configure();


private:

	void camera_image_callback(const sensor_msgs::ImageConstPtr msg);

	void publish_camera_scan(const camera_node::camera_scan & scan);

	//ros related
	ros::NodeHandle nh;
	ros::NodeHandle ph;
	ros::Subscriber camera_subscriber;
	ros::Publisher camera_scan_publisher;

	std::string frame_id;
	std::string image_topic_name;

	//scan related
	double sensor_field_of_view_rad;
	double sensor_distance;
	double sensor_angular_resolution;

};


#endif /* CAMERA_NODE_H_ */
