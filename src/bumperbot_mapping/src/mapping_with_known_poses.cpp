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
            return;
        }
        
        //needed in order to express laser scan in cartesian coordinates in the map frame, because the laser scan is in polar coordinates in the "base_link" frame, we need to convert it to cartesian coordinates in the "odom" frame using the robot's pose and the transform's rotation
        tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w); //convert the transform's rotation from a quaternion to a tf2::Quaternion
        tf2::Matrix3x3 m(q); //convert the quaternion to a rotation matrix
        double roll, pitch, yaw; //variables to store the roll, pitch, and yaw (we just need yaw for 2D mapping, but we can also get roll and pitch if needed for future extensions)
        m.getRPY(roll, pitch, yaw); //get the roll, pitch, and yaw from the rotation matrix

        for(size_t i = 0; i < scan->ranges.size(); ++i) //iterate through each scan point in the laser scan
        {
            //to draw obstacles
            double angle = scan->angle_min + (i * scan->angle_increment) + yaw; //calculate the angle of the current scan point in the "base_link" frame (in radians)
            double px = scan->ranges[i] * std::cos(angle); //calculate the x coordinate of the scan point in the map frame (in meters)
            double py = scan->ranges[i] * std::sin(angle); //calculate the y coordinate of the scan point in the map frame (in meters)
            px += t.transform.translation.x; //adjust the x coordinate of the scan point by the robot's x position in the map frame (in meters)
            py += t.transform.translation.y; //adjust the y coordinate of the scan point by the robot's y position in the map frame (in meters)
            
            Pose beam_pose = coordinateToPose(px, py, map_.info); //calculate the pose of the scan point in the map frame (in cells)
            
            if (!poseOnMap(beam_pose, map_.info) )
            {
                continue;
            }
            unsigned int beam_cell = poseToCell(beam_pose, map_.info); //convert the scan point's pose to a cell index
            map_.data[beam_cell] = 100; //mark the cell of the scan
        }
                    
            /*float min_range = scan->range_min; //get the minimum range of the scan
            float dx = range * std::cos(tf2::getYaw(t.transform.rotation)); //calculate the x component of the scan point in the map frame
            float dy = range * std::sin(tf2::getYaw(t.transform.rotation)); //calculate the y component of the scan point in the map frame
            float distance = std::sqrt(dx * dx + dy * dy); //calculate the distance from the robot to the scan point
            while(distance > min_range) //we will mark the cells along the ray from the robot to the scan point as free until we reach the scan point or the minimum range
            {
                Pose freePose = coordinateToPose(robot_pose.x + dx * (distance - min_range) / distance, 
                                                robot_pose.y + dy * (distance - min_range) / distance, 
                                                map_.info); //calculate the pose of the free cell along the ray (in cells)

                if (poseOnMap(freePose, map_.info))
                {
                    unsigned int free_cell = poseToCell(freePose, map_.info); //convert the free cell's pose to a cell index
                    if (map_.data[free_cell] != 100) //only mark the cell as free if it is not already marked as occupied by another scan point
                    {
                        map_.data[free_cell] = 0; //mark the cell as free (because we know that there is no obstacle at that cell)
                    }
                }

                distance -= min_range; //move closer to the robot by the minimum range for the next iteration
            }*/
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