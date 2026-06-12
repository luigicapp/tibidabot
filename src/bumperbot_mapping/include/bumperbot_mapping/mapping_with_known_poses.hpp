#ifndef MAPPING_WITH_KNOWN_POSES_HPP
#define MAPPING_WITH_KNOWN_POSES_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_listener.h> //in order to listen to the tf2 transforms
#include <tf2_ros/buffer.h>

namespace bumperbot_mapping
{
    struct Pose
    {
        Pose() = default;  //we can give constructors to a struct as well
        Pose(const int x, const int y) : x(x), y(y) {}//we can give constructors to a struct as well
        int x;
        int y;
    };

    //some utility functions
    unsigned int poseToCell(const Pose &pose, const nav_msgs::msg::MapMetaData &map_info); //map_info contains the resolution and the origin of the map, which are needed to convert from pose to cell index (return unsiged int because the cell index is always positive and is a vector index)
    Pose coordinateToPose(const double x, const double y, const nav_msgs::msg::MapMetaData &map_info); //map_info contains the resolution and the origin of the map, which are needed to convert from coordinate to pose
    bool poseOnMap(const Pose &pose, const nav_msgs::msg::MapMetaData &map_info); //check if the pose is on the map or not
    
    class MappingWithKnownPoses : public rclcpp::Node
    {
    public:

        MappingWithKnownPoses(const std::string &node_name);
        void publishMap(const nav_msgs::msg::OccupancyGrid &map);
    private:

        void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
        void timerCallback(); //will publish regularly the map

        nav_msgs::msg::OccupancyGrid map_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;

        rclcpp::TimerBase::SharedPtr timer_; //will call the timerCallback 

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    };
}

#endif // MAPPING_WITH_KNOWN_POSES_HPP
