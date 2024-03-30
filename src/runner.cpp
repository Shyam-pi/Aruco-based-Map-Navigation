#include "runner.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rclcpp_action/rclcpp_action.hpp>

// Constructor for the Runner class
Runner::Runner(tf2_ros::Buffer &tf_buffer) : Node("driver_node", rclcpp::NodeOptions()
                                                                     .allow_undeclared_parameters(true)
                                                                     .automatically_declare_parameters_from_overrides(true)),
                                             tf_buffer_(tf_buffer)
{

    // Create a TransformListener object
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

    // Lookup and store the transform between target_frame and source_frame1
    transformStamped1 = tf_buffer.lookupTransform(target_frame, source_frame1, tf2::TimePointZero, tf2::durationFromSec(4.0));

    // Lookup and store the transform between target_frame and source_frame2
    transformStamped2 = tf_buffer.lookupTransform(target_frame, source_frame2, tf2::TimePointZero, tf2::durationFromSec(4.0));

    // Lookup and store the transform between target_frame and source_frame3
    transformStamped3 = tf_buffer.lookupTransform(target_frame, source_frame3, tf2::TimePointZero, tf2::durationFromSec(4.0));

    // Lookup and store the transform between target_frame and source_frame4
    transformStamped4 = tf_buffer.lookupTransform(target_frame, source_frame4, tf2::TimePointZero, tf2::durationFromSec(4.0));

    // Lookup and store the transform between target_frame and source_frame5
    transformStamped5 = tf_buffer.lookupTransform(target_frame, source_frame5, tf2::TimePointZero, tf2::durationFromSec(4.0));

    // Store the transforms in a map
    transforms["camera1_frame"] = transformStamped1;
    transforms["camera2_frame"] = transformStamped2;
    transforms["camera3_frame"] = transformStamped3;
    transforms["camera4_frame"] = transformStamped4;
    transforms["camera5_frame"] = transformStamped5;

    // Initialize markerID to -1
    markerID = -1;

    // Initialize the colorMap
    colorMap = {{0, "red"}, {1, "green"}, {2, "blue"}, {3, "orange"}, {4, "purple"}};

    // Create a QoS object with reliability set to best effort
    rclcpp::QoS qos(10);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    // Create subscriptions to aruco_markers and  all mage camera image topics
    subscription_aruco = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
        "aruco_markers", qos, std::bind(&Runner::arucoMarkerCallback, this, std::placeholders::_1));

    subscription_c1 = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
        "mage/camera1/image", qos, std::bind(&Runner::c1_Callback, this, std::placeholders::_1));

    subscription_c2 = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
        "mage/camera2/image", qos, std::bind(&Runner::c2_Callback, this, std::placeholders::_1));

    subscription_c3 = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
        "mage/camera3/image", qos, std::bind(&Runner::c3_Callback, this, std::placeholders::_1));

    subscription_c4 = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
        "mage/camera4/image", qos, std::bind(&Runner::c4_Callback, this, std::placeholders::_1));

    subscription_c5 = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
        "mage/camera5/image", qos, std::bind(&Runner::c5_Callback, this, std::placeholders::_1));
}

// Callback function for aruco_markers topic
void Runner::arucoMarkerCallback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{
    // Check if the marker_ids vector is not empty and assign the first element to markerID
    if (!msg->marker_ids.empty())
    {
        markerID = msg->marker_ids[0];
    }

    if (markerID != -1)
    {
        // Unsubscribe by resetting the subscription object
        subscription_aruco.reset();
    }
}

// Callback function for mage/camera1/image topic
void Runner::c1_Callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    // Create variables for camera and map positions
    geometry_msgs::msg::PoseStamped pos_camera, pos_map;
    // Store the camera position in pos_camera
    pos_camera.pose = msg->part_poses[0].pose;
    pos_camera.header.frame_id = "camera1_frame";
    // Transform camera position to map position
    tf2::doTransform(pos_camera, pos_map, transformStamped1);
    pos_map.header.frame_id = "map";
    pos_map.pose.position.z = 0.007846;
    // Update the map with the color and corresponding map position
    myMap[colorMap[int(msg->part_poses[0].part.color)]] = pos_map;
}

// Callback function for mage/camera2/image topic
void Runner::c2_Callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    // Create variables for camera and map positions
    geometry_msgs::msg::PoseStamped pos_camera, pos_map;
    // Store the camera position in pos_camera
    pos_camera.pose = msg->part_poses[0].pose;
    pos_camera.header.frame_id = "camera2_frame";
    // Transform camera position to map position
    tf2::doTransform(pos_camera, pos_map, transformStamped2);
    pos_map.header.frame_id = "map";
    pos_map.pose.position.z = 0.007846;
    // Update the map with the color and corresponding map position
    myMap[colorMap[int(msg->part_poses[0].part.color)]] = pos_map;
}

// Callback function for mage/camera3/image topic
void Runner::c3_Callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    // Create variables for camera and map positions
    geometry_msgs::msg::PoseStamped pos_camera, pos_map;
    // Store the camera position in pos_camera
    pos_camera.pose = msg->part_poses[0].pose;
    pos_camera.header.frame_id = "camera3_frame";
    // Transform camera position to map position
    tf2::doTransform(pos_camera, pos_map, transformStamped3);
    pos_map.header.frame_id = "map";
    pos_map.pose.position.z = 0.007846;
    // Update the map with the color and corresponding map position
    myMap[colorMap[int(msg->part_poses[0].part.color)]] = pos_map;
}

// Callback function for mage/camera4/image topic
void Runner::c4_Callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    // Create variables for camera and map positions
    geometry_msgs::msg::PoseStamped pos_camera, pos_map;
    // Store the camera position in pos_camera
    pos_camera.pose = msg->part_poses[0].pose;
    pos_camera.header.frame_id = "camera4_frame";
    // Transform camera position to map position
    tf2::doTransform(pos_camera, pos_map, transformStamped4);
    pos_map.header.frame_id = "map";
    pos_map.pose.position.z = 0.007846;
    // Update the map with the color and corresponding map position
    myMap[colorMap[int(msg->part_poses[0].part.color)]] = pos_map;
}


    // Callback function for mage/camera5/image topic
    void Runner::c5_Callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
    {
        // Create variables for camera and map positions
        geometry_msgs::msg::PoseStamped pos_camera, pos_map;
        // Store the camera position in pos_camera
        pos_camera.pose = msg->part_poses[0].pose;
        pos_camera.header.frame_id = "camera5_frame";

        // Transform camera position to map position
        tf2::doTransform(pos_camera, pos_map, transformStamped5);
        pos_map.header.frame_id = "map";
        pos_map.pose.position.z = 0.007846;

        // Update the map with the color and corresponding map position
        myMap[colorMap[int(msg->part_poses[0].part.color)]] = pos_map;
    }

    // Get parameters based on markerID
    std::vector<std::string> Runner::getParam(int markerID)
    {
        std::vector<std::string> out;
        for (int i = 1; i <= 5; ++i)
        {
            out.push_back(this->get_parameter("aruco_" + std::to_string(markerID) + ".wp" + std::to_string(i) + ".color").value_to_string().c_str());
        }
        return out;
    }

    // Generate a list of poses based on color names
    std::vector<geometry_msgs::msg::PoseStamped> Runner::generatePoseList()
    {
        std::vector<std::string> colorNames = getParam(markerID);

        // Create an empty vector to store pose information
        std::vector<geometry_msgs::msg::PoseStamped> pose_list;

        // Check if the size of colorNames vector is greater than or equal to 5
        if (colorNames.size() >= 5)
        {
            // Iterate over each colorName in the colorNames vector
            for (const auto &colorName : colorNames)
            {
                // Print the marker ID and colorName using RCLCPP_INFO
                RCLCPP_INFO(this->get_logger(), "Color %d: %s", markerID, colorName.c_str());

                // Add the corresponding pose from myMap to the pose_list vector
                pose_list.push_back(myMap[colorName]);
            }
        }

        return pose_list;
    }

    // Print the contents of myMap
    void Runner::printMyMapContents()
    {
        for (const auto &pair : myMap)
        {
            const auto &key = pair.first;
            const auto &pose = pair.second.pose;
            RCLCPP_INFO(this->get_logger(), "Key: %s, Position: x=%f, y=%f, z=%f", key.c_str(), pose.position.x, pose.position.y, pose.position.z);
        }
    }

    // Stop camera subscriptions
    void Runner::stopCameraSubscription()
    {
        subscription_c1.reset();
        subscription_c2.reset();
        subscription_c3.reset();
        subscription_c4.reset();
        subscription_c5.reset();
    }

    // Constructor for the InitialPoseSetter class
    InitialPoseSetter::InitialPoseSetter() : Node("initial_pose_setter")
    {
        // Create a subscription to the odom topic
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&InitialPoseSetter::odomCallback, this, std::placeholders::_1));

        // Create a publisher for the initialpose topic
        initialpose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10);

        RCLCPP_INFO(this->get_logger(), "Initial Pose Setter Node has started.");
    }

    // Callback function for the odom topic
    void InitialPoseSetter::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Create an initial_pose message
        geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
        initial_pose.header = msg->header;
        initial_pose.header.frame_id = "map";
        initial_pose.pose = msg->pose;

        // Publish the initial_pose message
        initialpose_publisher_->publish(initial_pose);
        RCLCPP_INFO(this->get_logger(), "Published initial pose from odometry data.");

        // Reset the odom subscriber
        odom_subscriber_.reset();
    }

    // Main function
    int main(int argc, char *argv[])
    {

        rclcpp::init(argc, argv);

        // Setting the initial pose

        // Create an InitialPoseSetter object
        auto init_node = std::make_shared<InitialPoseSetter>();

        // Start time for publishing
        auto start_time = init_node->now();

        // Publish for 2 seconds
        while (init_node->now() - start_time <= std::chrono::seconds(2))
        {
            // Perform spinning for a short duration to process any callbacks
            rclcpp::spin_some(init_node);
        }

        // Reset the initialization node
        init_node.reset();

        // Create a tf_buffer object
        tf2_ros::Buffer tf_buffer(std::make_shared<rclcpp::Clock>());

        // Create a Runner object
        auto node = std::make_shared<Runner>(tf_buffer);

        // Action Goal
        auto action_client = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(node, "/follow_waypoints");
        while (!action_client->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_INFO(node->get_logger(), "Waiting for the action server to be available...");
        }
        // Create goal_msg object of type FollowWaypoints::Goal for the FollowWaypoints action
        auto goal_msg = nav2_msgs::action::FollowWaypoints::Goal();
        // Create send_goal_options object of type SendGoalOptions for the FollowWaypoints action
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();

        // Set the goal_response_callback to a lambda function that takes a shared_future as input
        send_goal_options.goal_response_callback =
            [&node](std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr> future)
        {
            auto goal_handle = future.get();
            // Get the goal_handle from the future
            if (!goal_handle)
            {
                // If the goal_handle is null, log an error message
                RCLCPP_ERROR(node->get_logger(), "Goal was rejected by the action server");
            }
            else
            {
                // If the goal_handle is not null, log an info message
                RCLCPP_INFO(node->get_logger(), "Goal accepted by the action server");
            }
        };
        // Set the feedback_callback FOR send goal to a lambda function that takes a GoalHandle as input
        send_goal_options.result_callback =
            [&node](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::WrappedResult &result)
        {
            // Logging result code after robot finishes navigation to all waypoints
            switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(node->get_logger(), "Goal succeeded");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(node->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO(node->get_logger(), "Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(node->get_logger(), "Unknown result code");
                break;
            }
        };

        while (true)
        {
            // makes sure map has atleast 5 batteries  and a marker has been detected by the robot before sending the goal
            if (node->myMap.size() >= 5 && node->markerID != -1)
            {
                // Generate a list of poses
                goal_msg.poses = node->generatePoseList();
                // stopping the camera subscription after getting list of poses
                node->stopCameraSubscription();
                // Send the goal to the action server
                auto future_goal_handle = action_client->async_send_goal(goal_msg, send_goal_options);
                break;
            }
            // if the map has less than 5 batteries or no marker has been detected
            else
            {
                // Print the size of myMap and the markerID
                // std::cout << "myMap size is " << node->myMap.size() << " and markerid is " << node->markerID << ". \n";
            }
            // Perform spinning for a short duration to process any callbacks
            rclcpp::spin_some(node);
        }
        // Perform spinning for a short duration to process any callbacks
        rclcpp::spin(node);
        // Shutdown the node
        rclcpp::shutdown();
        return 0;
    }
