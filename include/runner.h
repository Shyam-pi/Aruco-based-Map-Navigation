#ifndef RUNNER_H
#define RUNNER_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <map>
#include <string>
#include <vector>
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "mage_msgs/msg/advanced_logical_camera_image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

/**
 * @brief The Runner class represents a node that performs various tasks.
 */
class Runner : public rclcpp::Node {
public:
    /**
     * @brief Constructs a new Runner object.
     * @param tf_buffer The tf2_ros::Buffer object for transforming coordinates.
     */
    Runner(tf2_ros::Buffer& tf_buffer);

    std::map<std::string, geometry_msgs::msg::PoseStamped> myMap; /**< A map to store pose stamped messages. */
    int markerID; /**< The ID of the marker. */

    /**
     * @brief Generates a list of pose stamped messages.
     * @return A vector of pose stamped messages.
     */
    std::vector<geometry_msgs::msg::PoseStamped> generatePoseList();

    /**
     * @brief Prints the contents of myMap.
     */
    void printMyMapContents();

    /**
     * @brief Stops the camera subscription.
     */
    void stopCameraSubscription();

private:
    const std::string target_frame = "map"; /**< The target frame for coordinate transformation. */
    const std::string source_frame1 = "camera1_frame"; /**< The source frame for camera 1. */
    const std::string source_frame2 = "camera2_frame"; /**< The source frame for camera 2. */
    const std::string source_frame3 = "camera3_frame"; /**< The source frame for camera 3. */
    const std::string source_frame4 = "camera4_frame"; /**< The source frame for camera 4. */
    const std::string source_frame5 = "camera5_frame"; /**< The source frame for camera 5. */

    /**
     * @brief Callback function for aruco marker messages.
     * @param msg The received aruco marker message.
     */
    void arucoMarkerCallback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

    /**
     * @brief Callback function for camera 1 messages.
     * @param msg The received camera 1 message.
     */
    void c1_Callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Callback function for camera 2 messages.
     * @param msg The received camera 2 message.
     */
    void c2_Callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Callback function for camera 3 messages.
     * @param msg The received camera 3 message.
     */
    void c3_Callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Callback function for camera 4 messages.
     * @param msg The received camera 4 message.
     */
    void c4_Callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Callback function for camera 5 messages.
     * @param msg The received camera 5 message.
     */
    void c5_Callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Gets the parameters based on the marker ID.
     * @param markerID The ID of the marker.
     * @return A vector of strings containing the parameters.
     */
    std::vector<std::string> getParam(int markerID);

    std::map<int, std::string> colorMap; /**< A map to store color mappings. */
    tf2_ros::Buffer& tf_buffer_; /**< The tf2_ros::Buffer object for transforming coordinates. */
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_; /**< The tf2_ros::TransformListener object. */
    geometry_msgs::msg::TransformStamped transformStamped1, transformStamped2, transformStamped3, transformStamped4, transformStamped5; /**< TransformStamped objects for camera frames. */
    std::map<std::string, geometry_msgs::msg::TransformStamped> transforms; /**< A map to store transform stamped messages. */

    // Subscription members
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr subscription_aruco; /**< Subscription for aruco marker messages. */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr subscription_c1; /**< Subscription for camera 1 messages. */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr subscription_c2; /**< Subscription for camera 2 messages. */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr subscription_c3; /**< Subscription for camera 3 messages. */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr subscription_c4; /**< Subscription for camera 4 messages. */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr subscription_c5; /**< Subscription for camera 5 messages. */
};

/**
 * @brief The InitialPoseSetter class represents a node that sets the initial pose.
 */
class InitialPoseSetter : public rclcpp::Node {
public:
    /**
     * @brief Constructs a new InitialPoseSetter object.
     */
    InitialPoseSetter();

private:
    /**
     * @brief Callback function for odometry messages.
     * @param msg The received odometry message.
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_; /**< Subscription for odometry messages. */
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_publisher_; /**< Publisher for initial pose messages. */
};

#endif // RUNNER_H
