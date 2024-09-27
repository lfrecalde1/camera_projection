#ifndef CAMERA_PROJECTION_NODE_HPP
#define CAMERA_PROJECTION_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <eigen3/Eigen/Geometry>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/point_stamped.hpp>

namespace camera_projection_node
{
class CameraProjectionNode : public rclcpp::Node
{
public:
    explicit CameraProjectionNode(const rclcpp::NodeOptions &options);

private:
    // Callbacks
    void quadrotorOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void payloadOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    
    // Helper functions
    void publishCamera();
    void tfBroadcast(const nav_msgs::msg::Odometry &odom_msg);
    void QuadrotorPayloadToCameraOdomMsg(const Eigen::Vector3d &camera_pos, const Eigen::Quaterniond &camera_ori, const nav_msgs::msg::Odometry &quad_odom, nav_msgs::msg::Odometry &odom) const;
    void vectorToCameraVectorMsg(const Eigen::Vector3d &camera_pos, const Eigen::Quaterniond &camera_ori, const nav_msgs::msg::Odometry &quad_odom, const nav_msgs::msg::Odometry &payload_odom, geometry_msgs::msg::Vector3Stamped &vector) const;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr quadrotor_odom_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr payload_odom_subscriber_;
    
    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr camera_odom_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr vector_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_publisher_;

    // Transform broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Data
    nav_msgs::msg::Odometry quadrotor_odometry_;  
    nav_msgs::msg::Odometry payload_odometry_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Camera position and orientation
    Eigen::Vector3d camera_pos_;
    Eigen::Quaterniond camera_ori_;

    std::string camera_name_;
    std::string world_frame_id_;
    std::string quad_name_;
};
}  // namespace camera_projection_node

#endif  // CAMERA_PROJECTION_NODE_HPP