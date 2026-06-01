#ifndef ODOMETRY_MOTION_MODEL_HPP
#define ODOMETRY_MOTION_MODEL_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>



class OdometryMotionModel : public rclcpp::Node
{
public:
    OdometryMotionModel(const std::string& name);


private:
    void odomCallback(const nav_msgs::msg::Odometry &);


    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_; //will just publish an array of poeses

    geometry_msgs::msg::PoseArray samples_; //will contain the particles
    
    double alpha1_; //will describe how the noise affect the 3 elementary component of the motion (rot1,trasl,rot2)
    double alpha2_;
    double alpha3_;
    double alpha4_;

    int nr_samples_;

    double last_odom_x_;
    double last_odom_y_;
    double last_odom_theta_;

    bool is_first_odom_;
};

#endif // ODOMETRY_MOTION_MODEL_HPP