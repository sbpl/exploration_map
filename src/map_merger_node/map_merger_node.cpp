/*
 * map_merger_node.cpp
 *
 *  Created on: Oct 22, 2014
 *      Author: bmacallister
 */

#include <exploration_map/map_merger_node/map_merger_node.h>

#include <pcl_conversions/pcl_conversions.h>

map_merger_node::map_merger_node() :
    nh(),
    ph("~")
{
    initialize();
}

map_merger_node::~map_merger_node()
{
}

void map_merger_node::initialize()
{
    number_of_maps = 2;
    robot_poses.resize(number_of_maps);
    setup_ros();
    initialize_map_merger();
}

void map_merger_node::setup_ros()
{
    if (!ph.getParam("frame", frame_name) ||
        !ph.getParam("map_resolution", map_resolution) ||
        !ph.getParam("map_origin_x", map_origin_x) ||
        !ph.getParam("map_origin_y", map_origin_y) ||
        !ph.getParam("map_origin_z", map_origin_z) ||
        !ph.getParam("map_size_x", map_size_x) ||
        !ph.getParam("map_size_y", map_size_y) ||
        !ph.getParam("map_size_z", map_size_z) ||
        !ph.getParam("robot_0_map_name", robot_0_map_name) ||
        !ph.getParam("robot_1_map_name", robot_1_map_name) ||
        !ph.getParam("robot_0_pose_name", robot_0_pose_name) ||
        !ph.getParam("robot_1_pose_name", robot_1_pose_name) ||
        !ph.getParam("goal_list_name", goal_list_name) ||
        !ph.getParam("map_publish_rate", map_publish_rate) ||
        !ph.getParam("scan_match_ang_res", scan_match_angular_res) ||
        !ph.getParam("scan_match_met_res", scan_match_metric_res) ||
        !ph.getParam("scan_match_dx", scan_match_dx) ||
        !ph.getParam("scan_match_dy", scan_match_dy) ||
        !ph.getParam("scan_match_dz", scan_match_dz) ||
        !ph.getParam("scan_match_dyaw", scan_match_dyaw) ||
        !ph.getParam("goal_0_name", goal_0_name) ||
        !ph.getParam("goal_1_name", goal_1_name))
    {
        throw std::runtime_error("Failed to retrieve some parameters from the param server");
    }

    ph.param("publish_inner_maps", m_publish_inner_maps, false);

    ROS_INFO("frame %s\n", frame_name.c_str());
    ROS_INFO("map_resolution %f\n", map_resolution);
    ROS_ERROR("scan_match_dx %d", scan_match_dx);
    ROS_ERROR("scan_match_dyaw %d", scan_match_dyaw);
    ROS_ERROR("map publish rate %f\n", map_publish_rate);

    //timers
    map_publish_timer = ph.createTimer(ros::Duration(map_publish_rate), &map_merger_node::map_publish_callback, this);

    //subscribers
    robot_0_map_subscriber = nh.subscribe(robot_0_map_name.c_str(), 100, &map_merger_node::robot_0_map_callback, this);
    robot_1_map_subscriber = nh.subscribe(robot_1_map_name.c_str(), 100, &map_merger_node::robot_1_map_callback, this);
    robot_0_pose_subscriber = nh.subscribe(robot_0_pose_name.c_str(), 1, &map_merger_node::robot_0_pose_callback, this);
    robot_1_pose_subscriber = nh.subscribe(robot_1_pose_name.c_str(), 1, &map_merger_node::robot_1_pose_callback, this);
    goal_list_subscriber = nh.subscribe(goal_list_name.c_str(), 1, &map_merger_node::goal_list_callback, this);

    //publishers
    map_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZI> >("merged_map", 1);
    map_update_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZI> >("merged_map_update", 1);
    map_0_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZI> >("map0", 1);
    map_1_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZI> >("map1", 1);
    robot_0_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("pose0", 1);
    robot_1_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("pose1", 1);
    robot_poses_publisher = nh.advertise<nav_msgs::Path>("robot_poses", 1);
    robot_0_goal_publisher = nh.advertise<geometry_msgs::PoseStamped>(goal_0_name.c_str(), 1);
    robot_1_goal_publisher = nh.advertise<geometry_msgs::PoseStamped>(goal_1_name.c_str(), 1);
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
    //publish map 0 and 1 for debugging purposes
    if (m_publish_inner_maps) {
        this->publish_inner_maps();
    }

    if (merger.origins_are_initialized()) {
        ROS_ERROR("publish map");
        publish_master_map();

        //publish map update
        publish_map_update();
        updated_cell_list.list.clear();

        //publish robot poses
        publish_robot_poses();
    }
}

void map_merger_node::robot_0_map_callback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& msg)
{
    static unsigned int exp_seq = 0;
    map_0_point_cloud = *msg;
    ROS_INFO("got map 0 with seq number %d", map_0_point_cloud.header.seq);
    if (exp_seq != 0 and exp_seq != map_0_point_cloud.header.seq) {
        ROS_ERROR("sequence number for map 0 does not match expected (%d vs %d)", map_0_point_cloud.header.seq, exp_seq);
    }
    exp_seq = map_0_point_cloud.header.seq + 1;

    //get map update
    exploration::map_update update;
    update.map_id = 0;
    convert_point_cloud_to_map_update(map_0_point_cloud, update);

    //update map merger
    merger.receive_map_update(update, updated_cell_list);

    ROS_INFO("map 0 update finished\n");
}

void map_merger_node::robot_1_map_callback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& msg)
{
    static unsigned int exp_seq = 0;
    map_1_point_cloud = *msg;
    ROS_INFO("got map 1 with seq number %d", map_1_point_cloud.header.seq);
    if (exp_seq != 0 and exp_seq != map_1_point_cloud.header.seq) {
        ROS_ERROR("sequence number for map 1 does not match expected (%d vs %d)", map_1_point_cloud.header.seq, exp_seq);
    }
    exp_seq = map_1_point_cloud.header.seq + 1;

    //get map update
    exploration::map_update update;
    update.map_id = 1;
    convert_point_cloud_to_map_update(map_1_point_cloud, update);

    //update map merger
    merger.receive_map_update(update, updated_cell_list);

    ROS_INFO("map 1 update finished\n");
}

void map_merger_node::publish_master_map()
{
    const exploration::generic_map<exploration::exploration_type> * master_map;
    merger.get_master_map(master_map);
    std::vector<pcl::PointXYZI> master_map_cloud;
    get_point_cloud_from_map(master_map, master_map_cloud);
    publish_point_cloud(master_map_cloud, map_publisher);
}

void map_merger_node::publish_point_cloud(const std::vector<pcl::PointXYZI> & points, const ros::Publisher & pub)
{
    ROS_ERROR("publish point cloud with %d points\n", (int ) points.size());
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.points.resize(points.size());
    std::copy(points.begin(), points.end(), cloud.points.begin());
    cloud.height = 1;
    cloud.width = cloud.points.size();
    cloud.header.frame_id = frame_name;
    cloud.header.stamp = pcl_conversions::toPCL(ros::Time::now());
    pub.publish(cloud);
}

void map_merger_node::initialize_map_merger()
{
    exploration::map_merge_config con;

    //map size/resolution
    con.map_config_.size_x = map_size_x;
    con.map_config_.size_y = map_size_y;
    con.map_config_.size_z = map_size_z;
    con.map_config_.resolution = map_resolution;

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

void map_merger_node::convert_point_cloud_to_map_update(
    pcl::PointCloud<pcl::PointXYZI>& point_cloud,
    exploration::map_update& update)
{
    update.points.reserve(point_cloud.size());
    for (auto & p : point_cloud) {
        exploration::point_valued<exploration::exploration_type> point;
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;

        switch (static_cast<int>(p.intensity)) {
        case 100:
            point.value = exploration::exploration_type::occupied;
            break;
        case 50:
            point.value = exploration::exploration_type::explored;
            break;
        default:
            point.value = exploration::exploration_type::unoccupied;
            break;
        }
        update.points.push_back(point);
    }
}

void map_merger_node::robot_0_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ROS_INFO("got robot 0 pose");
    geometry_msgs::PoseStamped p = *(msg);

    auto pos = p.pose.position;
    auto ori = p.pose.orientation;
    ROS_INFO("pose is %f %f %f, %f %f %f %f", pos.x, pos.y, pos.z, ori.w, ori.x, ori.y, ori.z);

    //get map merger origin
    const exploration::pose * origin;
    merger.get_origin(0, origin);

    //transform pose according to its respective map merger origin
    transform_pose_stamped_to_master_map_frame(p, origin);

    pos = p.pose.position;
    ori = p.pose.orientation;
    ROS_INFO("transformed to %f %f %f, %f %f %f %f", pos.x, pos.y, pos.z, ori.w, ori.x, ori.y, ori.z);

    //assign to list of poses
    robot_poses[0] = p;

    //publish pose
    publish_pose(p, robot_0_pose_publisher);
}

void map_merger_node::robot_1_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ROS_INFO("got robot 1 pose");
    geometry_msgs::PoseStamped p = *(msg);

    auto pos = p.pose.position;
    auto ori = p.pose.orientation;
    ROS_INFO("pose is %f %f %f, %f %f %f %f", pos.x, pos.y, pos.z, ori.w, ori.x, ori.y, ori.z);

    //get map merger origin
    const exploration::pose * origin;
    merger.get_origin(1, origin);

    //transform pose according to its respective map merger origin
    transform_pose_stamped_to_master_map_frame(p, origin);

    pos = p.pose.position;
    ori = p.pose.orientation;
    ROS_INFO("transformed to %f %f %f, %f %f %f %f", pos.x, pos.y, pos.z, ori.w, ori.x, ori.y, ori.z);

    //assign to list of poses
    robot_poses[1] = p;

    //publish pose
    publish_pose(p, robot_1_pose_publisher);
}

void map_merger_node::publish_pose(const geometry_msgs::PoseStamped& pose, const ros::Publisher& pub)
{
    auto p = pose;
    p.header.frame_id = frame_name;
    p.header.stamp = ros::Time::now();
    if (merger.origins_are_initialized()) {
        pub.publish(p);
    }
}

void map_merger_node::get_point_cloud_from_map(
    const exploration::generic_map<exploration::exploration_type>* map,
    std::vector<pcl::PointXYZI>& points)
{
    //map dimensions
    int size_x = map->size_x;
    int size_y = map->size_y;
    int size_z = map->size_z;
    double res = map->resolution;

    for (int x = 0; x < size_x; x++) {
        for (int y = 0; y < size_y; y++) {
            for (int z = 0; z < size_z; z++) {
                pcl::PointXYZI p;
                p.x = exploration::exploration_map::continuous(x, res) + map_origin_x;
                p.y = exploration::exploration_map::continuous(y, res) + map_origin_y;
                p.z = exploration::exploration_map::continuous(z, res) + map_origin_z;

                exploration::exploration_type val = map->at(x, y, z);

                switch (val) {
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
}

void map_merger_node::get_point_cloud_from_map_update(
    const exploration::cell_list& cells,
    std::vector<pcl::PointXYZI>& points)
{
    const exploration::generic_map<exploration::exploration_type> * map;
    merger.get_master_map(map);

    //map dimensions
    double res = map->resolution;

    points.reserve(updated_cell_list.size());
    for (auto c : updated_cell_list.list) {
        pcl::PointXYZI p;
        p.x = exploration::exploration_map::continuous(c.X, res) + map_origin_x;
        p.y = exploration::exploration_map::continuous(c.Y, res) + map_origin_y;
        p.z = exploration::exploration_map::continuous(c.Z, res) + map_origin_z;

        exploration::exploration_type val = map->at(c.X, c.Y, c.Z);

        switch (val) {
        case exploration::exploration_type::occupied:
            p.intensity = 100;
            break;
        case exploration::exploration_type::explored:
            p.intensity = 50;
            break;
        default:
            p.intensity = 0;
            break;
        }
        points.push_back(p);
    }
}

void map_merger_node::publish_map_update()
{
    std::vector<pcl::PointXYZI> points;
    get_point_cloud_from_map_update(updated_cell_list, points);
    publish_point_cloud(points, map_update_publisher);
}

void map_merger_node::publish_inner_maps()
{
    const exploration::generic_map<exploration::exploration_type> * map0;
    const exploration::generic_map<exploration::exploration_type> * map1;
    merger.get_map(0, map0);
    merger.get_map(1, map1);
    std::vector<pcl::PointXYZI> map0_cloud, map1_cloud;
    get_point_cloud_from_map(map0, map0_cloud);
    get_point_cloud_from_map(map1, map1_cloud);
    publish_point_cloud(map0_cloud, map_0_publisher);
    publish_point_cloud(map1_cloud, map_1_publisher);
}

void map_merger_node::publish_robot_poses()
{
    nav_msgs::Path path_poses;
    path_poses.header.frame_id = frame_name;
    path_poses.header.seq = 0;
    path_poses.header.stamp = ros::Time::now();
    for (auto & p : robot_poses) {
        path_poses.poses.push_back(p);
    }
    robot_poses_publisher.publish(path_poses);
}

void map_merger_node::transform_pose_stamped_with_origin(
    geometry_msgs::PoseStamped& pose,
    const exploration::pose* origin,
    bool inverse)
{
    tf::Stamped<tf::Pose> tf_stamped_pose;
    tf::poseStampedMsgToTF(pose, tf_stamped_pose);
    tf::Quaternion oq(origin->ori.x, origin->ori.y, origin->ori.z, origin->ori.w);
    tf::Vector3 ov(origin->pos.x, origin->pos.y, origin->pos.z);
    tf::Transform trans(oq, ov);
    if (inverse) {
        trans = trans.inverse();
    }
    auto a = trans * tf_stamped_pose;
    tf_stamped_pose.setOrigin(a.getOrigin());
    tf_stamped_pose.setRotation(a.getRotation());
    tf::poseStampedTFToMsg(tf_stamped_pose, pose);
}

void map_merger_node::transform_pose_stamped_to_master_map_frame(
    geometry_msgs::PoseStamped& pose,
    const exploration::pose* origin)
{
    transform_pose_stamped_with_origin(pose, origin, false);
}

void map_merger_node::transform_pose_stamped_to_local_map_frame(
    geometry_msgs::PoseStamped& pose,
    const exploration::pose* origin)
{
    transform_pose_stamped_with_origin(pose, origin, true);
}

void map_merger_node::goal_list_callback(const nav_msgs::PathConstPtr& msg)
{
    nav_msgs::Path goal_list = *(msg);

    for (size_t i = 0; i < goal_list.poses.size(); i++) {
        auto p = goal_list.poses[i];

        //get map merger origin
        const exploration::pose * origin;
        if (!merger.get_origin(i, origin)) {
            continue;
        }

        //transform pose according to its respective map merger origin
        transform_pose_stamped_to_local_map_frame(p, origin);

        for (size_t a = 0; a < 5; a++) {
            //publish goal
            switch (i) {
            case 0:
                publish_pose(p, robot_0_goal_publisher);
                break;
            case 1:
                publish_pose(p, robot_1_goal_publisher);
            default:
                break;
            }

            ros::Duration(1.0).sleep();
        }
    }
}
