/*
 * camera_node.h
 *
 *  Created on: Oct 14, 2014
 *      Author: bmacallister
 */

#ifndef CAMERA_NODE_H_
#define CAMERA_NODE_H_

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <exploration_map/camera_scan.h>

/**
 * \brief ros node for retrieving camera images and publishing corresponding camera scans
 */
class camera_scan_node
{
public:

    /**
     * \brief constructor
     */
    camera_scan_node();

    /**
     * \brief destructor
     */
    ~camera_scan_node();

    /**
     * \brief configures ros related entities including params and callback functions
     */
    void ros_configure();

private:

    /**
     * \brief callback function when image of camera received
     * @param msg the msg the image is in
     */
    void camera_image_callback(const sensor_msgs::ImageConstPtr msg);

    /**
     * \brief publishes the camera scan with same timestamp as it coresponding image
     * @param scan the camera scan
     */
    void publish_camera_scan(const exploration_map::camera_scan & scan);

    //ros related
    ros::NodeHandle nh;
    ros::NodeHandle ph;
    ros::Subscriber camera_subscriber;
    ros::Publisher camera_scan_publisher;

    std::string frame_id;
    std::string image_topic_name;

    //scan related
    double sensor_field_of_view_rad; /** angle (in radians) of sensor field of view */
    double sensor_distance; /** max distance of scan */
    double sensor_angular_resolution; /**angular distance between rays cast is scan */
};

#endif /* CAMERA_NODE_H_ */
