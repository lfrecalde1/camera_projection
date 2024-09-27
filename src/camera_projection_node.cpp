#include "camera_projection/camera_projection_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace camera_projection_node
{
CameraProjectionNode::CameraProjectionNode(const rclcpp::NodeOptions &options)
    : Node("camera_projection_node", options),
      camera_pos_(0.0, 0.0, -0.075), // Initialize camera position
      camera_ori_(-0.70, 0.70, 0.0, 0.0) // Initialize camera orientation
{
    // Declare parameters with default values
    this->declare_parameter("world_frame_id", std::string("world"));
    this->declare_parameter("quadrotor_name", std::string("quadrotor"));
    this->declare_parameter("camera_frame", std::string("camera"));
    this->declare_parameter("camera_position.x", 0.0);
    this->declare_parameter("camera_position.y", 0.0);
    this->declare_parameter("camera_position.z", -0.075);
    this->declare_parameter("camera_ori.x", -0.70);
    this->declare_parameter("camera_ori.y", 0.70);
    this->declare_parameter("camera_ori.z", 0.0);
    this->declare_parameter("camera_ori.w", 0.0);

    // Get parameters
    this->get_parameter("world_frame_id", world_frame_id_);
    this->get_parameter("quadrotor_name", quad_name_);
    this->get_parameter("camera_frame", camera_name_);
    this->get_parameter("camera_position.x", camera_pos_(0));
    this->get_parameter("camera_position.y", camera_pos_(1));
    this->get_parameter("camera_position.z", camera_pos_(2));
    this->get_parameter("camera_ori.x", camera_ori_.x());
    this->get_parameter("camera_ori.y", camera_ori_.y());
    this->get_parameter("camera_ori.z", camera_ori_.z());
    this->get_parameter("camera_ori.w", camera_ori_.w());

    // Normalize the quaternion to avoid invalid rotations
    camera_ori_.normalize();

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Subscriber to /quadrotor/odom
    quadrotor_odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/" + quad_name_ + "/odom", 10, 
        std::bind(&CameraProjectionNode::quadrotorOdomCallback, this, std::placeholders::_1));

    // Subscriber to /quadrotor/payload/odom
    payload_odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/" + quad_name_ + "/payload/odom", 10, 
        std::bind(&CameraProjectionNode::payloadOdomCallback, this, std::placeholders::_1));
    
    // Publisher for quadrotor/camera/odom
    std::string camera_odom_topic = "/" + quad_name_ + "/" + camera_name_ + "/odom";
    camera_odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(camera_odom_topic, 10);

    // Publisher for vector3_stamped
    vector_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/" + quad_name_ + "/" + "payload" + "/vector", 10);
    point_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/" + quad_name_ + "/" + "payload" + "/point", 10);

    // Timer to publish at 100 Hz (every 5 milliseconds)
    timer_ = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&CameraProjectionNode::publishCamera, this));
}

void CameraProjectionNode::quadrotorOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    quadrotor_odometry_ = *msg;
}

void CameraProjectionNode::payloadOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    payload_odometry_ = *msg;
}

void CameraProjectionNode::publishCamera()
{
    nav_msgs::msg::Odometry camera_odom_msg;
    camera_odom_msg.header.frame_id = world_frame_id_;
    camera_odom_msg.child_frame_id = camera_name_;
    camera_odom_msg.header.stamp = quadrotor_odometry_.header.stamp;

    // Convert quadrotor state to camera odometry
    QuadrotorPayloadToCameraOdomMsg(camera_pos_, camera_ori_, quadrotor_odometry_, camera_odom_msg);

    // Publish camera odometry
    camera_odom_publisher_->publish(camera_odom_msg);

    // Broadcast transform
    tfBroadcast(camera_odom_msg);

    // Create and publish vector message
    geometry_msgs::msg::Vector3Stamped vector_msg;
    vector_msg.header.frame_id = camera_name_;
    vector_msg.header.stamp = this->get_clock()->now();
    vectorToCameraVectorMsg(camera_pos_, camera_ori_, quadrotor_odometry_, payload_odometry_, vector_msg);
    vector_publisher_->publish(vector_msg);

    // Publish vector as a point 
    geometry_msgs::msg::PointStamped payload_position_camera_point_msg;
    payload_position_camera_point_msg.header.frame_id = camera_name_;
    payload_position_camera_point_msg.header.stamp = vector_msg.header.stamp;
    payload_position_camera_point_msg.point.x = vector_msg.vector.x;
    payload_position_camera_point_msg.point.y = vector_msg.vector.y;
    payload_position_camera_point_msg.point.z = vector_msg.vector.z;
    point_publisher_->publish(payload_position_camera_point_msg);
}

void CameraProjectionNode::tfBroadcast(const nav_msgs::msg::Odometry &odom_msg)
{
    geometry_msgs::msg::TransformStamped ts;

    ts.header.stamp = odom_msg.header.stamp;
    ts.header.frame_id = odom_msg.header.frame_id;
    ts.child_frame_id = odom_msg.child_frame_id;

    ts.transform.translation.x = odom_msg.pose.pose.position.x;
    ts.transform.translation.y = odom_msg.pose.pose.position.y;
    ts.transform.translation.z = odom_msg.pose.pose.position.z;

    ts.transform.rotation = odom_msg.pose.pose.orientation;

    tf_broadcaster_->sendTransform(ts);
}

void CameraProjectionNode::QuadrotorPayloadToCameraOdomMsg(const Eigen::Vector3d &camera_pos, const Eigen::Quaterniond &camera_ori, const nav_msgs::msg::Odometry &quad_odom, nav_msgs::msg::Odometry &odom) const {
    // Quadrotor pose variables
    Eigen::Matrix<double, 4, 1> td_w;
    Eigen::Matrix<double, 4, 1> tc_b;
    Eigen::Matrix<double, 4, 1> t;

    Eigen::Matrix<double, 4, 1> q;
    Eigen::Matrix<double, 4, 1> quat_c;
    Eigen::Matrix<double, 4, 1> quat;
    Eigen::Matrix<double, 4, 1> quat_camera;

    // Translation as quat
    td_w << 0.0, quad_odom.pose.pose.position.x, quad_odom.pose.pose.position.y, quad_odom.pose.pose.position.z;
    tc_b << 0.0, camera_pos(0), camera_pos(1), camera_pos(2);

    // Quaternion drone
    q << quad_odom.pose.pose.orientation.w , quad_odom.pose.pose.orientation.x, quad_odom.pose.pose.orientation.y, quad_odom.pose.pose.orientation.z;

    // Quaternion Camera
    quat_camera << camera_ori.w(), camera_ori.x(), camera_ori.y(), camera_ori.z();

    // Aux variables quaternion and conjugate quatenrion
    quat_c << q(0), -q(1), -q(2), -q(3);
    quat << q(0), q(1), q(2), q(3);

    Eigen::Matrix<double, 4, 4> H_plus_q;
    H_plus_q << quat(0), -quat(1), -quat(2), -quat(3),
                        quat(1),  quat(0), -quat(3),  quat(2),
                        quat(2),  quat(3),  quat(0), -quat(1),
                        quat(3), -quat(2),  quat(1),  quat(0);

    // Perform the first multiplication
    Eigen::Matrix<double, 4, 1> aux_value = H_plus_q * tc_b;
    Eigen::Matrix<double, 4, 4> H_plus_aux;
    H_plus_aux << aux_value(0), -aux_value(1), -aux_value(2), -aux_value(3),
                  aux_value(1),  aux_value(0), -aux_value(3),  aux_value(2),
                  aux_value(2),  aux_value(3),  aux_value(0), -aux_value(1),
                  aux_value(3), -aux_value(2),  aux_value(1),  aux_value(0);
    // Perform the second multiplication
    Eigen::Matrix<double, 4, 1> tc_w = H_plus_aux * quat_c;
    t = tc_w + td_w;

    // Compute complete rotation
    Eigen::Matrix<double, 4, 1> quat_wc = H_plus_q * quat_camera;

    // Transform to quaternion object to use normalize
    Eigen::Quaterniond q_wc;
    q_wc.w() = quat_wc(0);
    q_wc.vec() << quat_wc(1), quat_wc(2), quat_wc(3);
    q_wc.normalize();

    odom.pose.pose.position.x = t(1);
    odom.pose.pose.position.y = t(2);
    odom.pose.pose.position.z = t(3);

    odom.pose.pose.orientation.w = q_wc.w();
    odom.pose.pose.orientation.x = q_wc.x();
    odom.pose.pose.orientation.y = q_wc.y();
    odom.pose.pose.orientation.z = q_wc.z();
}

void CameraProjectionNode::vectorToCameraVectorMsg(const Eigen::Vector3d &camera_pos, const Eigen::Quaterniond &camera_ori, const nav_msgs::msg::Odometry &quad_odom, const nav_msgs::msg::Odometry &payload_odom, geometry_msgs::msg::Vector3Stamped &vector) const
    {
        // Quadrotor pose variables
        Eigen::Matrix<double, 4, 1> tb_w;
        Eigen::Matrix<double, 4, 1> tc_b;
        Eigen::Matrix<double, 4, 1> t_w;
        Eigen::Matrix<double, 4, 1> t;
        Eigen::Matrix<double, 4, 1> t_c;

        Eigen::Matrix<double, 4, 1> q;
        Eigen::Matrix<double, 4, 1> q_wb;
        Eigen::Matrix<double, 4, 1> q_bc;
        Eigen::Matrix<double, 4, 1> q_bc_c;
        Eigen::Matrix<double, 4, 1> q_wc_c;

        // Translation as quat
        tb_w << 0.0, quad_odom.pose.pose.position.x, quad_odom.pose.pose.position.y, quad_odom.pose.pose.position.z;
        tc_b << 0.0, camera_pos(0), camera_pos(1), camera_pos(2);
        t_w << 0.0, payload_odom.pose.pose.position.x, payload_odom.pose.pose.position.y, payload_odom.pose.pose.position.z;

        // Quaternion drone
        q << quad_odom.pose.pose.orientation.w, quad_odom.pose.pose.orientation.x, quad_odom.pose.pose.orientation.y, quad_odom.pose.pose.orientation.z;
        q_wb << q(0), q(1), q(2), q(3);

        // Quaternion Camera
        q_bc << camera_ori.w(), camera_ori.x(), camera_ori.y(), camera_ori.z();
        q_bc_c << camera_ori.w(), -camera_ori.x(), -camera_ori.y(), -camera_ori.z();

        // Aux matrix
        Eigen::Matrix<double, 4, 4> H_plus_q_wb;
        H_plus_q_wb << q_wb(0), -q_wb(1), -q_wb(2), -q_wb(3),
                       q_wb(1),  q_wb(0), -q_wb(3),  q_wb(2),
                       q_wb(2),  q_wb(3),  q_wb(0), -q_wb(1),
                       q_wb(3), -q_wb(2),  q_wb(1),  q_wb(0);

        Eigen::Matrix<double, 4, 4> H_plus_q_bc_c;
        H_plus_q_bc_c << q_bc_c(0), -q_bc_c(1), -q_bc_c(2), -q_bc_c(3),
                         q_bc_c(1),  q_bc_c(0), -q_bc_c(3),  q_bc_c(2),
                         q_bc_c(2),  q_bc_c(3),  q_bc_c(0), -q_bc_c(1),
                         q_bc_c(3), -q_bc_c(2),  q_bc_c(1),  q_bc_c(0);

        // Compute complete rotation
        Eigen::Matrix<double, 4, 1> q_wc = H_plus_q_wb * q_bc;
        float q_wc_norm = std::sqrt(q_wc(0) * q_wc(0) + q_wc(1) * q_wc(1) + q_wc(2) * q_wc(2) + q_wc(3) * q_wc(3));
        q_wc << q_wc(0) / q_wc_norm, q_wc(1) / q_wc_norm, q_wc(2) / q_wc_norm, q_wc(3) / q_wc_norm;
        q_wc_c << q_wc(0), -q_wc(1), -q_wc(2), -q_wc(3);

        // Aux matrix
        Eigen::Matrix<double, 4, 4> H_plus_q_wc_c;
        H_plus_q_wc_c << q_wc_c(0), -q_wc_c(1), -q_wc_c(2), -q_wc_c(3),
                         q_wc_c(1),  q_wc_c(0), -q_wc_c(3),  q_wc_c(2),
                         q_wc_c(2),  q_wc_c(3),  q_wc_c(0), -q_wc_c(1),
                         q_wc_c(3), -q_wc_c(2),  q_wc_c(1),  q_wc_c(0);

        // Difference between payload and drone location
        t = t_w - tb_w;

        Eigen::Matrix<double, 4, 1> aux_value_difference = H_plus_q_wc_c * t;
        Eigen::Matrix<double, 4, 1> aux_value_camera = H_plus_q_bc_c * tc_b;

        // Aux matrix
        Eigen::Matrix<double, 4, 4> H_plus_difference;
        H_plus_difference << aux_value_difference(0), -aux_value_difference(1), -aux_value_difference(2), -aux_value_difference(3),
                             aux_value_difference(1),  aux_value_difference(0), -aux_value_difference(3),  aux_value_difference(2),
                             aux_value_difference(2),  aux_value_difference(3),  aux_value_difference(0), -aux_value_difference(1),
                             aux_value_difference(3), -aux_value_difference(2),  aux_value_difference(1),  aux_value_difference(0);

        Eigen::Matrix<double, 4, 4> H_plus_camera;
        H_plus_camera << aux_value_camera(0), -aux_value_camera(1), -aux_value_camera(2), -aux_value_camera(3),
                         aux_value_camera(1),  aux_value_camera(0), -aux_value_camera(3),  aux_value_camera(2),
                         aux_value_camera(2),  aux_value_camera(3),  aux_value_camera(0), -aux_value_camera(1),
                         aux_value_camera(3), -aux_value_camera(2),  aux_value_camera(1),  aux_value_camera(0);

        t_c = H_plus_difference * q_wc - H_plus_camera * q_bc;

        vector.vector.x = t_c(1);
        vector.vector.y = t_c(2);
        vector.vector.z = t_c(3);
    }

// Rest of the function implementations remain the same...

// Register the component
RCLCPP_COMPONENTS_REGISTER_NODE(camera_projection_node::CameraProjectionNode)
}  // namespace camera_projection_node