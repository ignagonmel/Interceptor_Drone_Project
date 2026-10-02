#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"

class EKFNode : public rclcpp::Node {
public:
    EKFNode();

private:
    void measurementCallback(const geometry_msgs::msg::Point::SharedPtr msg);

    //Kalman Filter Variables
    bool initialized_;
    double dt_;
    Eigen::VectorXd x_;
    Eigen::MatrixXd F_, H_, P_, Q_, R_;

    //ROS2 Interfaces
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_meas_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    nav_msgs::msg::Path path_msg_;
};

EKFNode::EKFNode() : Node("ekf_node_cpp"), initialized_(false), dt_(0.02) {
    //Subscription to noise measurements topic
    sub_meas_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/target/measurement",
        10, std::bind(&EKFNode::measurementCallback, this, std::placeholders::_1));

    //Result Publishers
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/target/estimated_pose", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/target/estimated_path", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/target/estimated_marker", 10);

    //Initial State x (6x1) [x, y, z, vx, vy, vz]^T
    x_ = Eigen::VectorXd::Zero(6);

    //State Transition Matrix F (6x6)
    F_ = Eigen::MatrixXd::Identity(6, 6);
    F_(0, 3) = dt_;
    F_(1, 4) = dt_;
    F_(2, 5) = dt_;

    //Obserbation Matrix H (3x6)
    H_ = Eigen::MatrixXd::Zero(3, 6);
    H_(0, 0) = 1.0;
    H_(1, 1) = 1.0;
    H_(2, 2) = 1.0;

    //Error Covariance Matrix P (6x6)
    P_ = Eigen::MatrixXd::Identity(6, 6) * 1.0;

    //Process Noise Matrix Q (6x6)
    double q = 0.5;
    double dt2 = std::pow(dt_, 2);
    double dt3 = std::pow(dt_, 3);

    Q_ = Eigen::MatrixXd::Zero(6, 6);

    
    Q_(0, 0) = q * (dt3 / 3.0);
    Q_(0, 3) = q * (dt2 / 2.0);
    Q_(3, 0) = q * (dt2 / 2.0);
    Q_(3, 3) = q * dt_;

    
    Q_(1, 1) = q * (dt3 / 3.0);
    Q_(1, 4) = q * (dt2 / 2.0);
    Q_(4, 1) = q * (dt2 / 2.0);
    Q_(4, 4) = q * dt_;

    
    Q_(2, 2) = q * (dt3 / 3.0);
    Q_(2, 5) = q * (dt2 / 2.0);
    Q_(5, 2) = q * (dt2 / 2.0);
    Q_(5, 5) = q * dt_;

    //Measurement Noise Matrix R (3x3)
    R_ = Eigen::MatrixXd::Identity(3, 3) * (0.35 * 0.35);

    path_msg_.header.frame_id = "map";

    RCLCPP_INFO(this->get_logger(), "EKF Node Initialized Correctly!");

}

void EKFNode::measurementCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
       //Convert ROS2 Message into Eigen vector (3x1)
       Eigen::Vector3d z(msg->x, msg->y, msg->z);

       //State Initialization with first received measurement
       if (!initialized_) {
        x_(0) = z(0);
        x_(1) = z(1);
        x_(2) = z(2);
        x_(3) = 0.0; // Initial Vx assumed to be 0
        x_(4) = 0.0; // Initial Vy assumed to be 0
        x_(5) = 0.0; // Initial Vz assumed to be 0

        initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "EKF Initialized with Position: [%.2f, %.2f, %.2f]", z(0), z(1), z(2));
        return;
       }

       //   --- PREDICTION PHASE ---
       Eigen::VectorXd x_pred = F_ * x_;
       Eigen::MatrixXd P_pred = (F_ * P_ * F_.transpose()) + Q_;

       //   --- CORRECTION PHASE (UPDATE) ---
       Eigen::Vector3d y = z - (H_ * x_pred);   //  INNOVATION OR WASTE (3x1)
       Eigen::Matrix3d S = (H_ * P_pred * H_.transpose()) + R_; //  WASTE COVARIANCE (3x3)
       Eigen::MatrixXd K = P_pred * H_.transpose() * S.inverse();   //  KALMAN GAIN (6x3)

       x_ = x_pred + (K * y);   //  UPDATED STATE (6x1)
       Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);   
       P_ = (I - (K * H_)) * P_pred;    //  UPDATED COVARIANCE (6x6)

       // --- PUBLISH ROS2 RESULTS ---
       auto now = this->get_clock()->now();

       // 1. Publish PoseStamped
       geometry_msgs::msg::PoseStamped pose;
       pose.header.stamp = now;
       pose.header.frame_id = "map";
       pose.pose.position.x = x_(0);
       pose.pose.position.y = x_(1);
       pose.pose.position.z = x_(2);
       pose.pose.orientation.w = 1.0;
       pose_pub_->publish(pose);
    
       // 2. Publish Trajectory (Path)
       path_msg_.header.stamp = now;
       path_msg_.poses.push_back(pose);
       if (path_msg_.poses.size() > 500) {
           path_msg_.poses.erase(path_msg_.poses.begin());
       }
       path_pub_->publish(path_msg_);

       // 3. Publish Visual Marker for RViz2
       visualization_msgs::msg::Marker marker;
       marker.header.stamp = now;
       marker.header.frame_id = "map";
       marker.ns = "ekf_target_cpp";
       marker.id = 1;
       marker.type = visualization_msgs::msg::Marker::SPHERE;
       marker.action = visualization_msgs::msg::Marker::ADD;
       marker.pose.position = pose.pose.position;
       marker.scale.x = 0.4;
       marker.scale.y = 0.4;
       marker.scale.z = 0.4;
       marker.color.r = 0.0;
       marker.color.g = 1.0;
       marker.color.b = 0.0;
       marker.color.a = 1.0;
       marker_pub_->publish(marker);
}
int main(int argc, char ** argv) {
    // 1. Initiate ROS2 Communications
    rclcpp::init(argc, argv);

    // 2. Create EKF node in C++ Instance
    auto node = std::make_shared<EKFNode>();

    // 3. Keep Node Answering to incoming Callbacks
    rclcpp::spin(node);

    // 4. Free Resources when closing (Ctrl + C)
    rclcpp::shutdown();
    return 0;
}