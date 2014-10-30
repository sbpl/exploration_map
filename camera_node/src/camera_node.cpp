
/*
 * camera_node.cpp
 *
 *  Created on: Oct 14, 2014
 *      Author: bmacallister
 */

#include <camera_node/camera_node.h>

camera_scan_node::camera_scan_node():ph("~")
{
	frame_id = "";
	sensor_field_of_view_rad = 0;
	sensor_distance = 0;

	ros_configure();
}

camera_scan_node::~camera_scan_node()
{
}

void camera_scan_node::ros_configure()
{
	ph.getParam("image_topic_name", image_topic_name);
	ph.getParam("frame", frame_id);
	ph.getParam("sensor_field_of_view", sensor_field_of_view_rad);
	ph.getParam("senor_distance", sensor_distance);
	ph.getParam("sensor_angular_res", sensor_angular_resolution);

	camera_subscriber = nh.subscribe(image_topic_name, 1, &camera_scan_node::camera_image_callback, this);
	camera_scan_publisher = ph.advertise<camera_node::camera_scan>("camera_scan",1);

}

void camera_scan_node::camera_image_callback(const sensor_msgs::ImageConstPtr msg)
{

	sensor_msgs::Image im = *msg;

	//generate camera scan with image
	camera_node::camera_scan cam_scan;
	cam_scan.header = im.header;
	cam_scan.image_data = im;
	cam_scan.yaw_angle_increment = sensor_angular_resolution;
	cam_scan.yaw_angle_max = sensor_field_of_view_rad / 2;
	cam_scan.yaw_angle_min = -sensor_field_of_view_rad /2 ;
	cam_scan.roll_angle_increment = sensor_angular_resolution;
	cam_scan.roll_angle_min = 0;
	cam_scan.roll_angle_max = M_PI;
	cam_scan.range_max = sensor_distance;
	cam_scan.range_min = sensor_distance;

	//generate ranges
	double current_ang = cam_scan.yaw_angle_min;
	double current_roll_ang = cam_scan.roll_angle_min;
	while(true)
	{
		double d = sensor_distance;
		cam_scan.ranges.push_back(d);

		if(current_ang >=  cam_scan.yaw_angle_max)
		{
			current_ang = cam_scan.yaw_angle_min;
			current_roll_ang += cam_scan.roll_angle_increment;
			if(current_roll_ang >= cam_scan.roll_angle_max)
			{
				break;
			}
		}

		current_ang += cam_scan.yaw_angle_increment;
	}
	//publish cam scan
	publish_camera_scan(cam_scan);
}

void camera_scan_node::publish_camera_scan(const camera_node::camera_scan& scan)
{
	camera_scan_publisher.publish(scan);
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"camera_scan_node");
	camera_scan_node n;

	ros::spin();
}
