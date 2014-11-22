/*
 * exploration_map_node.cpp
 *
 *  Created on: Sep 23, 2014
 *      Author: bmacallister
 */

#include <exploration_map_node/exploration_map_node.h>

exploration_map_node::exploration_map_node() :
		n(), pn("~")
{
	camera_scan_counter = 0;
	horizontal_lidar_scan_counter = 0;
	vertical_lidar_scan_counter = 0;
	ros_configure();
	initialize_exploration_map();
}

exploration_map_node::~exploration_map_node()
{
}

void exploration_map_node::ros_configure()
{
	//get params
	pn.getParam("frame", frame_name);
	pn.getParam("map_resolution", map_resolution);
	pn.getParam("map_origin_x", map_origin.x);
	pn.getParam("map_origin_y", map_origin.y);
	pn.getParam("map_origin_z", map_origin.z);
	pn.getParam("map_size_x", map_size_x);
	pn.getParam("map_size_y", map_size_y);
	pn.getParam("map_size_z", map_size_z);
	pn.getParam("horizontal_lidar_pose_tf_name", horizontal_lidar_pose_tf_name);
	pn.getParam("vertical_lidar_pose_tf_name", vertical_lidar_pose_tf_name);
	pn.getParam("camera_pose_tf_name", camera_pose_tf_name);
	pn.getParam("horizontal_lidar_topic_name", horizontal_lidar_topic_name);
	pn.getParam("vertical_lidar_topic_name", vertical_lidar_topic_name);
	pn.getParam("camera_scan_topic_name", camera_scan_topic_name);
	pn.getParam("lidar_update_increment", lidar_update_increment);
	pn.getParam("lidar_update_decrement", lidar_update_decrement);
	pn.getParam("occupancy_prob_threshold", occupancy_prob_thresh);
	pn.getParam("camera_scan_topic_name", camera_scan_topic_name);
	pn.getParam("map_publish_rate", map_publish_rate);
	pn.getParam("number_of_scans_to_skip", number_of_scans_to_skip);
	pn.getParam("base_height_min", base_height_min);
	pn.getParam("base_height_max", base_height_max);
	pn.getParam("unnocupied_prob_threshold", unnocupied_prob_thresh);
	pn.getParam("publish_debug_messages", publish_debug_messages);
	pn.getParam("min_sensor_distance_threshold", min_sensor_distance_threshold);

	ROS_INFO("subscribed to %s and %s\n", horizontal_lidar_pose_tf_name.c_str(), horizontal_lidar_topic_name.c_str());
	ROS_INFO("subscribed to %s and %s\n", camera_pose_tf_name.c_str(), camera_scan_topic_name.c_str());
	ROS_INFO("subscribed to %s and %s\n", vertical_lidar_pose_tf_name.c_str(), vertical_lidar_topic_name.c_str());
	ROS_INFO("min distance threshold for sensors %f\n", min_sensor_distance_threshold);
	ROS_INFO("publish debugging messages (%d) \n", publish_debug_messages);

	//subscribe
	horiz_lidar_sub = n.subscribe(horizontal_lidar_topic_name, 1, &exploration_map_node::horizontal_lidar_callback, this);
	camera_scan_sub = n.subscribe(camera_scan_topic_name, 1, &exploration_map_node::camera_scan_callback, this);
	verti_lidar_sub = n.subscribe(vertical_lidar_topic_name, 1, &exploration_map_node::vertical_lidar_callback, this);

	//timer
	map_publish_timer = n.createTimer(ros::Duration(map_publish_rate), &exploration_map_node::map_publish_timer_callback, this);

	//advertise published topics
	horizontal_lidar_update_pub = pn.advertise<pcl::PointCloud<pcl::PointXYZI> >("horizontal_lidar_update", 1);
	horizontal_lidar_update_ray_trace_pub = pn.advertise<pcl::PointCloud<pcl::PointXYZI> >("horizontal_lidar_update_ray_trace", 1);
	vertical_lidar_update_pub = pn.advertise<pcl::PointCloud<pcl::PointXYZI> >("vertical_lidar_update", 1);
	vertical_lidar_update_ray_trace_pub = pn.advertise<pcl::PointCloud<pcl::PointXYZI> >("vertical_lidar_update_ray_trace", 1);
	camera_update_pub = pn.advertise<pcl::PointCloud<pcl::PointXYZI> >("camera_update", 1);
	camera_update_ray_trace_pub = pn.advertise<pcl::PointCloud<pcl::PointXYZI> >("camera_update_ray_trace", 1);
	exploration_map_pub = pn.advertise<pcl::PointCloud<pcl::PointXYZI> >("exploration_map", 1);
}

void exploration_map_node::horizontal_lidar_callback(const sensor_msgs::LaserScanConstPtr& msg)
{
	//skip scan if required
	if (horizontal_lidar_scan_counter < number_of_scans_to_skip)
	{
		horizontal_lidar_scan_counter++;
		return;
	}
	else
	{
		horizontal_lidar_scan_counter = 0;
	}

	geometry_msgs::PoseStamped pose;
	if (!get_lateset_pose_from_tf(pose, horizontal_lidar_pose_tf_name))
	{
		ROS_ERROR("Error: could not process horizontal lidar because latest pose could not be retrieved");
		return;
	}

	//create lidar update
	sensor_update::pose sense_pose;
	sensor_update::sensor_reading reading;
	convert_pose_to_sensor_update_pose(pose, sense_pose);
	sensor_msgs::LaserScan scan = *msg;
	convert_laser_scan_to_sensor_update_ray(scan, reading);
	sensor_update::lidar_update update(sense_pose, reading);

	if (publish_debug_messages)
	{
		// publish transformed points
		publish_sensor_update(update, horizontal_lidar_update_pub);

		// publish transformed cells
		publish_sensor_update_ray_trace(update, horizontal_lidar_update_ray_trace_pub);
	}

	//update exploration map
	update_exploration_map(update);

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "exploration_map_node");
	exploration_map_node en;
	ros::spin();
}

void exploration_map_node::convert_pose_to_sensor_update_pose(const geometry_msgs::PoseStamped pose, sensor_update::pose& sensor_pose)
{
	sensor_pose.pos.x = pose.pose.position.x;
	sensor_pose.pos.y = pose.pose.position.y;
	sensor_pose.pos.z = pose.pose.position.z;
	sensor_pose.ori.w = pose.pose.orientation.w;
	sensor_pose.ori.x = pose.pose.orientation.x;
	sensor_pose.ori.y = pose.pose.orientation.y;
	sensor_pose.ori.z = pose.pose.orientation.z;
}

void exploration_map_node::convert_laser_scan_to_sensor_update_ray(const sensor_msgs::LaserScan & scan, sensor_update::sensor_reading & reading)
{
	float range_min = min_sensor_distance_threshold;
	float range_max = scan.range_max;
	float angle_min = scan.angle_min;
	float angle_increment = scan.angle_increment;
	double current_angle = angle_min;
	for (auto & r : scan.ranges)
	{
		double angle = current_angle;
		double distance = r;
		if (distance >= range_min && distance <= range_max)
		{
			sensor_update::sensor_ray ray;
			ray.angle.yaw = angle;
			ray.angle.roll = 0;
			ray.distance = distance;
			reading.rays.push_back(ray);
		}
		current_angle += angle_increment;
	}
}

void exploration_map_node::publish_sensor_update(const sensor_update::sensor_update& update, const ros::Publisher & pub)
{

	std::vector<sensor_update::position> lidar_points;
	update.get_transformed_sensor_ends(lidar_points);

	//convert to 3d points
	std::vector<pcl::PointXYZI> points;
	for (auto & lp : lidar_points)
	{
		pcl::PointXYZI p;
		p.x = static_cast<float>(lp.x);
		p.y = static_cast<float>(lp.y);
		p.z = static_cast<float>(lp.z);
		points.push_back(p);
	}

	publish_point_cloud(points, pub);
}

void exploration_map_node::publish_sensor_update_ray_trace(const sensor_update::sensor_update& update, const ros::Publisher & pub)
{
	std::vector<sensor_update::discrete_cell> ray_trace_cells;
	update.get_discrete_ray_trace_cells(map_resolution, ray_trace_cells);

	//convert to 3d points
	std::vector<pcl::PointXYZI> points;
	for (auto & cell : ray_trace_cells)
	{
		pcl::PointXYZI p;
		p.x = cell.X * map_resolution + map_resolution / 2;
		p.y = cell.Y * map_resolution + map_resolution / 2;
		p.z = cell.Z * map_resolution + map_resolution / 2;
		p.intensity = static_cast<float>(cell.value);
		points.push_back(p);
	}

	publish_point_cloud(points, pub);
}

void exploration_map_node::publish_point_cloud(const std::vector<pcl::PointXYZI> & points, const ros::Publisher & pub)
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

bool exploration_map_node::get_lateset_pose_from_tf(geometry_msgs::PoseStamped& pose, const std::string & pose_name)
{
	auto input_pose = pose;
	input_pose.header.frame_id = pose_name;
	input_pose.header.stamp = ros::Time(0);
	input_pose.pose.position.x = 0;
	input_pose.pose.position.y = 0;
	input_pose.pose.position.z = 0;
	input_pose.pose.orientation.w = 1;
	input_pose.pose.orientation.x = 0;
	input_pose.pose.orientation.y = 0;
	input_pose.pose.orientation.z = 0;

	try
	{
		listener.transformPose("map", input_pose, pose);
	}

	catch (tf::TransformException &e)
	{
		ROS_ERROR("ERROR: could not get pose from tf\n, %s", e.what());
		return false;
	}
	tf::Quaternion bt;
	tf::quaternionMsgToTF(pose.pose.orientation, bt);

	auto pos = pose.pose.position;
	auto ori = pose.pose.orientation;
	return true;
}

bool exploration_map_node::initialize_exploration_map()
{
	exploration::exploration_map_config con;

	// map related config
	con.map_config_.resolution = map_resolution;
	con.map_config_.origin.pos.x = map_origin.x;
	con.map_config_.origin.pos.y = map_origin.y;
	con.map_config_.origin.pos.z = map_origin.z;
	con.map_config_.size_x = map_size_x;
	con.map_config_.size_y = map_size_y;
	con.map_config_.size_z = map_size_z;
	con.map_config_.base_height_min = base_height_min;
	con.map_config_.base_height_max = base_height_max;

	//occupancy related config
	con.occ_map_config_.occ_threshold = occupancy_prob_thresh;
	con.occ_map_config_.update_decrement_value = lidar_update_decrement;
	con.occ_map_config_.update_increment_value = lidar_update_increment;
	con.occ_map_config_.unnoc_threshold = unnocupied_prob_thresh;

	exp_map.initialize(con);

	return true;
}

bool exploration_map_node::update_exploration_map(const sensor_update::sensor_update& update)
{
	exp_map.update_map(update);
	return true;
}

void exploration_map_node::camera_scan_callback(const camera_node::camera_scanConstPtr& msg)
{
	//skip scan if required
	if (camera_scan_counter < number_of_scans_to_skip)
	{
		camera_scan_counter++;
		return;
	}
	else
	{
		camera_scan_counter = 0;
	}

	//get pose
	geometry_msgs::PoseStamped pose;
	if (!get_lateset_pose_from_tf(pose, camera_pose_tf_name))
	{
		ROS_ERROR("could not retrieve pose for camera scan");
		return;
	}

	//create camera update
	sensor_update::pose sense_pose;
	sensor_update::sensor_reading reading;
	convert_pose_to_sensor_update_pose(pose, sense_pose);
	camera_node::camera_scan scan = *msg;
	convert_camera_scan_to_sensor_update_ray(scan, reading);
	sensor_update::camera_update update(sense_pose, reading);

	if (publish_debug_messages)
	{
		// publish transformed points
		publish_sensor_update(update, camera_update_pub);

		// publish transformed cells
		publish_sensor_update_ray_trace(update, camera_update_ray_trace_pub);
	}

	//update exploration map
	update_exploration_map(update);
}

void exploration_map_node::convert_camera_scan_to_sensor_update_ray(const camera_node::camera_scan& scan, sensor_update::sensor_reading& reading)
{
	float range_min = scan.range_min;
	float range_max = scan.range_max;
	float yaw_angle_min = scan.yaw_angle_min;
	float roll_angle_min = scan.roll_angle_min;

	float yaw_angle_max = scan.yaw_angle_max;

	float yaw_angle_increment = scan.yaw_angle_increment;
	float roll_angle_increment = scan.roll_angle_increment;

	double current_yaw_angle = yaw_angle_min;
	double current_roll_angle = roll_angle_min;

	for (auto & r : scan.ranges)
	{

		double distance = r;
		if (distance >= range_min && distance <= range_max)
		{
			sensor_update::sensor_ray ray;
			ray.angle.yaw = current_yaw_angle;
			ray.angle.roll = current_roll_angle;
			ray.distance = distance;
			reading.rays.push_back(ray);
		}

		current_yaw_angle += yaw_angle_increment;

		if (current_yaw_angle > yaw_angle_max)
		{
			current_yaw_angle = yaw_angle_min;
			current_roll_angle += roll_angle_increment;
		}
	}
}

void exploration_map_node::map_publish_timer_callback(const ros::TimerEvent& event)
{
	publish_exploration_map();
}

void exploration_map_node::vertical_lidar_callback(const sensor_msgs::LaserScanConstPtr& msg)
{
	//skip scan if required
	if (vertical_lidar_scan_counter < number_of_scans_to_skip)
	{
		vertical_lidar_scan_counter++;
		return;
	}
	else
	{
		vertical_lidar_scan_counter = 0;
	}

	geometry_msgs::PoseStamped pose;
	if (!get_lateset_pose_from_tf(pose, vertical_lidar_pose_tf_name))
	{
		ROS_ERROR("Error: could not process vertical lidar because latest pose could not be retrieved");
		return;
	}

	//create lidar update
	sensor_update::pose sense_pose;
	sensor_update::sensor_reading reading;
	convert_pose_to_sensor_update_pose(pose, sense_pose);
	sensor_msgs::LaserScan scan = *msg;
	convert_laser_scan_to_sensor_update_ray(scan, reading);
	sensor_update::lidar_update update(sense_pose, reading);

	if (publish_debug_messages)
	{
		// publish transformed points
		publish_sensor_update(update, vertical_lidar_update_pub);

		// publish transformed cells
		publish_sensor_update_ray_trace(update, vertical_lidar_update_ray_trace_pub);
	}

	//update exploration map
	update_exploration_map(update);
}

void exploration_map_node::publish_exploration_map()
{
	auto map = exp_map.get_exploration_map();
	auto con = exp_map.get_configuration();
	int size_x = con->map_config_.size_x;
	int size_y = con->map_config_.size_y;
	int size_z = con->map_config_.size_z;
	double res = con->map_config_.resolution;
	double origin_x = con->map_config_.origin.x;
	double origin_y = con->map_config_.origin.y;
	double origin_z = con->map_config_.origin.z;

	std::vector<pcl::PointXYZI> points;
	for (int x = 0; x < size_x; x++)
	{
		for (int y = 0; y < size_y; y++)
		{

			for (int z = 0; z < size_z; z++)
			{

				pcl::PointXYZI p;
				p.x = x * res + res / 2 + origin_x;
				p.y = y * res + res / 2 + origin_y;
				p.z = z * res + res / 2 + origin_z;

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
	publish_point_cloud(points, exploration_map_pub);
}

