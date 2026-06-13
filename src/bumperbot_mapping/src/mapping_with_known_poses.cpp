#include <tf2/utils.h> //for tf2::getYaw to get the yaw angle from a quaternion 
#include <geometry_msgs/msg/transform_stamped.hpp> //for geometry_msgs::msg::TransformStamped to store the transform from the "odom" frame to the "base_link" frame
#include <bumperbot_mapping/mapping_with_known_poses.hpp>
#include <geometry_msgs/msg/point.hpp> //for geometry_msgs::msg::Point to represent the start and end points of the ray
#include <visualization_msgs/msg/marker.hpp> //for visualization_msgs::msg::Marker to visualize the robot's pose and the scan points in RViz
#include <visualization_msgs/msg/marker_array.hpp> //for visualization_msgs::msg::MarkerArray to visualize the rays in RViz
using std::placeholders::_1;

namespace bumperbot_mapping
{
    //utility function to convert a pose to a cell index. Need to divide the pose by the resolution of the map to get the cell index, and also adjust for the origin of the map.
    Pose coordinateToPose(const double x, const double y, const nav_msgs::msg::MapMetaData &map_info)
    {
        Pose pose;
        pose.x = std::round((x - map_info.origin.position.x) / map_info.resolution); //convert the x coordinate from meters to cells and adjust for the origin of the map
        pose.y = std::round((y - map_info.origin.position.y) / map_info.resolution); //convert the y coordinate from meters to cells and adjust for the origin of the map
        return pose;
    }

    bool poseOnMap(const Pose &pose, const nav_msgs::msg::MapMetaData &map_info)
    {
        return (pose.x >= 0 && pose.x < static_cast<int>(map_info.width) && 
                pose.y >= 0 && pose.y < static_cast<int>(map_info.height)); //check if the pose is within the bounds of the map
    }

    unsigned int poseToCell(const Pose &pose, const nav_msgs::msg::MapMetaData &map_info)
    {
        return pose.y * map_info.width + pose.x; //convert the pose to a cell index (row-major order)
    }
    
    void MappingWithKnownPoses::publishMap(nav_msgs::msg::OccupancyGrid & map_)
    {
        map_.header.stamp = get_clock()->now(); //update the timestamp of the map   
        map_publisher_->publish(map_); //publish the map
    }

    MappingWithKnownPoses::MappingWithKnownPoses(const std::string &node_name) : Node(node_name)
    {
        declare_parameter<double>("width", 50); //declare a parameter for the width of the map (in meters)
        declare_parameter<double>("height", 50); //declare a parameter for the height of the map (in meters)
        declare_parameter<double>("resolution", 0.1); //declare a parameter for the resolution of the map (in meters/cell)

        double width = get_parameter("width").as_double(); //get the width of the map from the parameter server
        double height = get_parameter("height").as_double(); //get the height of the map from the parameter server
        map_.info.resolution = get_parameter("resolution").as_double(); //get the resolution of the map from the parameter server
        map_.info.width = std::round(width / map_.info.resolution); //calculate the width of the map in cells (because the map is represented as a grid of cells, we need to convert the width from meters to cells)
        map_.info.height = std::round(height / map_.info.resolution); //calculate the height of the map in cells (because the map is represented as a grid of cells, we need to convert the height from meters to cells)
        map_.info.origin.position.x = -std::round(width / 2.0); //set the origin of the map to be at the center of the map (in meters)
        map_.info.origin.position.y = -std::round(height / 2.0); //set the origin of the map to be at the center of the map (in meters)

        map_.header.frame_id = "odom"; //set the frame id of the map to be "odom"
        map_.data = std::vector<int8_t>(map_.info.width * map_.info.height, -1); //initialize the map data with -1 (unknown). It is a vector of int8, where each cell can be -1 (unknown), 0 (free), or 100 (occupied)
    
        map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", 1); //create a publisher for the map
        ray_array_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("ray_markers", 10);
        
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&MappingWithKnownPoses::scanCallback, this, std::placeholders::_1)); //create a subscription for the laser scan data
        timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&MappingWithKnownPoses::timerCallback , this)); //create a timer to publish the map regularly (every 1 second)
    
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock()); //create a tf2 buffer to store the transforms
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }


    void MappingWithKnownPoses::scanCallback(const sensor_msgs::msg::LaserScan & scan)
{
    visualization_msgs::msg::MarkerArray marker_array;


    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    geometry_msgs::msg::TransformStamped t;
    try
    {
        t = tf_buffer_->lookupTransform(map_.header.frame_id, scan.header.frame_id, tf2::TimePointZero);

    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_ERROR(get_logger(), "unable to get transform from %s to %s: %s", 
                     map_.header.frame_id.c_str(), scan.header.frame_id.c_str(), ex.what());
        return;
    }

    // posizione del robot nella mappa (2D, solo x/y)
    Pose robot_pose = coordinateToPose(t.transform.translation.x, t.transform.translation.y, map_.info);

    //get rpy
    double roll, pitch, yaw;
    tf2::Quaternion q(
        t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    

    if (!poseOnMap(robot_pose, map_.info))
    {
        RCLCPP_ERROR(get_logger(), "robot pose is out of the map bounds, cannot update the map");
        return;
    }

    for(size_t i = 0; i < scan.ranges.size(); ++i)
    {
        if (!std::isfinite(scan.ranges.at(i)) || 
            scan.ranges.at(i) < scan.range_min || 
            scan.ranges.at(i) >= scan.range_max - 6)
        {
            continue;
        }


        // calcola il punto finale del beam nel frame del laser (coordinate polari → cartesiane)
        // l'angolo di elevazione è 0 per un LiDAR 2D, ma se il laser è inclinato (pitch != 0)
        // tf2::doTransform gestirà automaticamente la rotazione corretta in 3D
        double laser_angle = scan.angle_min + (i * scan.angle_increment);
        
        geometry_msgs::msg::PointStamped beam_in_laser;
        beam_in_laser.header.frame_id = scan.header.frame_id;
        beam_in_laser.header.stamp = scan.header.stamp;
        beam_in_laser.point.x = scan.ranges.at(i) * std::cos(laser_angle); // x nel frame laser
        beam_in_laser.point.y = scan.ranges.at(i) * std::sin(laser_angle); // y nel frame laser
        beam_in_laser.point.z = scan.ranges.at(i)  * std::sin(pitch); // 2D, nessuna elevazione nel frame laser

        // trasforma il punto dal frame laser al frame della mappa (odom)
        // tf2::doTransform applica rotazione + traslazione completa in 3D,
        // quindi se il laser ha pitch != 0, beam_in_odom.point.z sarà != start_point.z
        geometry_msgs::msg::PointStamped beam_in_odom;
        tf2::doTransform(beam_in_laser, beam_in_odom, t);

        double px = beam_in_odom.point.x;
        double py = beam_in_odom.point.y;
        // beam_in_odom.point.z contiene la z corretta tenendo conto del pitch del laser

        // aggiorna la mappa 2D (proiezione sul piano XY)
        Pose beam_pose = coordinateToPose(px, py, map_.info);
        
        if (!poseOnMap(beam_pose, map_.info))
        {
            continue;
        }

        unsigned int beam_cell = poseToCell(beam_pose, map_.info);
        map_.data.at(beam_cell) = 100;

        // marker per visualizzare il raggio in RViz
        visualization_msgs::msg::Marker ray;
        ray.header.frame_id = map_.header.frame_id;
        ray.header.stamp = get_clock()->now();
        ray.ns = "laser_scan_rays";
        ray.id = i;
        ray.type = visualization_msgs::msg::Marker::LINE_STRIP;
        ray.action = visualization_msgs::msg::Marker::ADD;
        ray.scale.x = 0.005; // sottile
        ray.color.a = 1.0;
        ray.color.r = 1.0;
        ray.color.g = 0.0;
        ray.color.b = 0.0;

        // start: posizione del laser nel frame odom
        geometry_msgs::msg::Point start_point;
        start_point.x = t.transform.translation.x;
        start_point.y = t.transform.translation.y;
        start_point.z = t.transform.translation.z;

        // end: posizione del beam nel frame odom, z dipende dal pitch del laser
        geometry_msgs::msg::Point end_point;
        end_point.x = px;
        end_point.y = py;
        end_point.z = beam_in_odom.point.z; // z corretta in 3D

        ray.points.push_back(start_point);
        ray.points.push_back(end_point);
        marker_array.markers.push_back(ray);
    }

    ray_array_publisher_->publish(marker_array);
}

        void MappingWithKnownPoses::timerCallback()
        {
            map_.header.stamp = get_clock()->now(); //update the timestamp of the map
            publishMap(map_); //publish the map regularly (every 1 second)
        }
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bumperbot_mapping::MappingWithKnownPoses>("mapping_with_known_poses_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}