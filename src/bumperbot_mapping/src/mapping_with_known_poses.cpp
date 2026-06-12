#include <tf2/utils.h> //for tf2::getYaw to get the yaw angle from a quaternion 
#include <geometry_msgs/msg/transform_stamped.hpp> //for geometry_msgs::msg::TransformStamped to store the transform from the "odom" frame to the "base_link" frame
#include <bumperbot_mapping/mapping_with_known_poses.hpp>

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
    
    void MappingWithKnownPoses::publishMap(const nav_msgs::msg::OccupancyGrid &map)
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
        map_.data.resize(map_.info.width * map_.info.height, -1); //initialize the map data with -1 (unknown). It is a vector of int8, where each cell can be -1 (unknown), 0 (free), or 100 (occupied)
    
        map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", 1); //create a publisher for the map
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&MappingWithKnownPoses::scanCallback, this, std::placeholders::_1)); //create a subscription for the laser scan data

        timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&MappingWithKnownPoses::timerCallback, this)); //create a timer to publish the map regularly (every 1 second)
    
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock()); //create a tf2 buffer to store the transforms
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }


    void MappingWithKnownPoses::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        //get the transform from the "odom" frame to the "base_link" frame at the time of the scan
        geometry_msgs::msg::TransformStamped t;
        try
        {
            t = tf_buffer_->lookupTransform(map_.header.frame_id, scan->header.frame_id, tf2::TimePointZero); //lookup the transform from the "odom" frame to the "base_link" frame at the time of the scan. We use tf2::TimePointZero to get the latest available transform, which is usually good enough for mapping purposes. If we want to get the exact transform at the time of the scan, we can use scan->header.stamp instead, but it may cause some delay if the transform is not available yet.
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(get_logger(), "unable to get transform from %s to %s: %s", map_.header.frame_id.c_str(), scan->header.frame_id.c_str(), ex.what());
            return; //if we cannot get the transform, we cannot update the map, so we return
        }

        //convert the transform to a pose
        Pose robot_pose = coordinateToPose(t.transform.translation.x, t.transform.translation.y, map_.info); //convert the robot's position from coordinates to pose (in cells)

        if (!poseOnMap(robot_pose, map_.info))
        {
            RCLCPP_ERROR(get_logger(), "robot pose is out of the map bounds, cannot update the map");
        }
        
        unsigned int robot_cell = poseToCell(robot_pose, map_.info); //convert the robot's pose to a cell index (not used in this example, but can be useful for future extensions)
        map_.data[robot_cell] = 100; //mark the robot's cell as occupied (for visualization purposes, we can mark the robot's cell as occupied, but in a real mapping application, we may want to keep it as free or unknown)
        
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